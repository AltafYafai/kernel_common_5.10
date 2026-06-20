// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/vindicator/tsuki.c
 * Tsuki — Kernel-level Android property manipulation via ksud
 *
 * Provides a workqueue-based interface to set Android system properties
 * at the kernel level using KernelSU's ksud resetprop, bypassing SELinux
 * and property_service restrictions. Uses Kiryuu for root execution.
 *
 * Author: GrayRavens
 * Co-authored-by: Kanagawa Yamada <albert.wesley.dion@gmail.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/string.h>
#include <linux/fs.h>

extern int kiryuu_exec(const char *cmd);

/*
 * Path to the ksud binary. Override via module param if KernelSU
 * is installed at a non-default location.
 */
static char ksud_path[256] = "/data/adb/ksud";
module_param_string(ksud_path, ksud_path, sizeof(ksud_path), 0644);
MODULE_PARM_DESC(ksud_path, "Path to the ksud binary");

/*
 * Lazy KSU availability check: cached after first probe.
 */
static int ksu_available;
static DEFINE_SPINLOCK(ksu_check_lock);

static bool ksu_is_available(void)
{
	struct file *f;
	unsigned long flags;
	int cached;

	spin_lock_irqsave(&ksu_check_lock, flags);
	cached = ksu_available;
	spin_unlock_irqrestore(&ksu_check_lock, flags);

	if (cached)
		return cached > 0;

	f = filp_open(ksud_path, O_RDONLY, 0);
	if (IS_ERR(f)) {
		spin_lock_irqsave(&ksu_check_lock, flags);
		ksu_available = -1;
		spin_unlock_irqrestore(&ksu_check_lock, flags);
		return false;
	}
	filp_close(f, NULL);

	spin_lock_irqsave(&ksu_check_lock, flags);
	ksu_available = 1;
	spin_unlock_irqrestore(&ksu_check_lock, flags);
	return true;
}

#define TSUKI_QUEUE_MAX	64

struct tsuki_job {
	char *prop;
	char *val;
	struct list_head list;
};

static LIST_HEAD(tsuki_queue);
static DEFINE_SPINLOCK(tsuki_lock);
static struct work_struct tsuki_work;
static atomic_t tsuki_queue_count = ATOMIC_INIT(0);

/**
 * tsuki_setprop - Queue an Android property change
 * @prop: Property name (e.g. "persist.sys.power_profile")
 * @val:  New value for the property
 *
 * Returns: 0 on success, negative errno on error
 *
 * The property change is applied asynchronously by a workqueue that
 * calls ksud resetprop. If the queue is full, -ENOSPC is returned.
 */
int tsuki_setprop(const char *prop, const char *val)
{
	struct tsuki_job *job;
	unsigned long flags;

	if (!prop || !val)
		return -EINVAL;

	if (atomic_read(&tsuki_queue_count) >= TSUKI_QUEUE_MAX)
		return -ENOSPC;

	job = kmalloc(sizeof(*job), GFP_KERNEL);
	if (!job)
		return -ENOMEM;

	job->prop = kstrdup(prop, GFP_KERNEL);
	job->val = kstrdup(val, GFP_KERNEL);
	if (!job->prop || !job->val) {
		kfree(job->prop);
		kfree(job->val);
		kfree(job);
		return -ENOMEM;
	}

	spin_lock_irqsave(&tsuki_lock, flags);
	list_add_tail(&job->list, &tsuki_queue);
	atomic_inc(&tsuki_queue_count);
	spin_unlock_irqrestore(&tsuki_lock, flags);

	schedule_work(&tsuki_work);

	pr_debug("tsuki: queued '%s' = '%s' (queue_depth=%d)\n",
		 prop, val, atomic_read(&tsuki_queue_count));
	return 0;
}
EXPORT_SYMBOL_GPL(tsuki_setprop);

static void tsuki_process_queue(struct work_struct *work)
{
	struct tsuki_job *job, *tmp;
	unsigned long flags;
	LIST_HEAD(batch);

	spin_lock_irqsave(&tsuki_lock, flags);
	list_splice_init(&tsuki_queue, &batch);
	atomic_set(&tsuki_queue_count, 0);
	spin_unlock_irqrestore(&tsuki_lock, flags);

	list_for_each_entry_safe(job, tmp, &batch, list) {
		char cmd[512];
		int ret;	if (!ksu_is_available())
			goto skip;

		snprintf(cmd, sizeof(cmd),
			 "%s resetprop -n '%s' '%s'",
			 ksud_path, job->prop, job->val);

		pr_debug("tsuki: applying '%s' = '%s'\n", job->prop, job->val);
		ret = kiryuu_exec(cmd);
		if (ret < 0)
			pr_warn("tsuki: failed to set '%s' (err=%d)\n",
				job->prop, ret);

		list_del(&job->list);
		kfree(job->prop);
		kfree(job->val);
		kfree(job);
		continue;

	skip:
		list_del(&job->list);
		kfree(job->prop);
		kfree(job->val);
		kfree(job);
	}
}

/**
 * tsuki_setprop_sync - Set a property synchronously (blocking)
 * @prop: Property name
 * @val:  New value
 *
 * Returns: 0 on success, negative errno on error
 *
 * Unlike tsuki_setprop(), this blocks until ksud completes.
 * Use for boot-time property init where ordering matters.
 */
int tsuki_setprop_sync(const char *prop, const char *val)
{
	char cmd[512];

	if (!prop || !val)
		return -EINVAL;

	if (!ksu_is_available())
		return -ENOENT;

	snprintf(cmd, sizeof(cmd),
		 "%s resetprop -n '%s' '%s'", ksud_path, prop, val);

	pr_info("tsuki: sync setprop '%s' = '%s'\n", prop, val);
	return kiryuu_exec(cmd);

}
EXPORT_SYMBOL_GPL(tsuki_setprop_sync);

static int __init tsuki_init(void)
{
	INIT_WORK(&tsuki_work, tsuki_process_queue);
	pr_info("tsuki: Kernel property relay initialized (max_queue=%d)\n",
		TSUKI_QUEUE_MAX);
	return 0;
}
late_initcall(tsuki_init);

static void __exit tsuki_exit(void)
{
	struct tsuki_job *job, *tmp;
	unsigned long flags;

	cancel_work_sync(&tsuki_work);

	spin_lock_irqsave(&tsuki_lock, flags);
	list_for_each_entry_safe(job, tmp, &tsuki_queue, list) {
		list_del(&job->list);
		kfree(job->prop);
		kfree(job->val);
		kfree(job);
	}
	atomic_set(&tsuki_queue_count, 0);
	spin_unlock_irqrestore(&tsuki_lock, flags);

	pr_info("tsuki: unloaded\n");
}
module_exit(tsuki_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GrayRavens");
MODULE_DESCRIPTION("Tsuki — Kernel-level Android property manipulation via ksud");
