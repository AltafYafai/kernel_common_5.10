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

extern int kiryuu_exec(const char *cmd);

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
		int ret;

		snprintf(cmd, sizeof(cmd),
			 "/data/adb/ksud resetprop -n '%s' '%s'",
			 job->prop, job->val);

		pr_debug("tsuki: applying '%s' = '%s'\n", job->prop, job->val);
		ret = kiryuu_exec(cmd);
		if (ret < 0)
			pr_warn("tsuki: failed to set '%s' (err=%d)\n",
				job->prop, ret);

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

	snprintf(cmd, sizeof(cmd),
		 "/data/adb/ksud resetprop -n '%s' '%s'", prop, val);

	pr_info("tsuki: sync setprop '%s' = '%s'\n", prop, val);
	return kiryuu_exec(cmd, 0);
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
