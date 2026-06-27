// SPDX-License-Identifier: GPL-2.0-only
/*
 * zenith_stune_boost - Automatic top-app schedtune.boost
 *
 * Periodically checks whether any task is running in the top-app cgroup.
 * If so, writes the configured boost value to /dev/stune/top-app/schedtune.boost
 * so EAS prefers higher frequencies for foreground tasks. When no top-app
 * tasks are detected, the boost is cleared.
 *
 * Fully automatic — no userspace interaction required.
 *
 * Tunables (/sys/module/zenith_stune_boost/parameters/):
 *   enabled       — master switch (1/0, default 1)
 *   boost_value   — boost value 0-100 (default 30)
 *   interval_ms   — polling interval in ms (default 1000)
 */
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/cgroup.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/printk.h>

static bool stune_enabled __read_mostly = true;
static unsigned int stune_value __read_mostly = 30;
static unsigned int stune_interval_ms __read_mostly = 1000;
static char stune_boost_file[256] __read_mostly = "/dev/stune/top-app/schedtune.boost";

module_param_named(enabled, stune_enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable stune boost (default: true)");
module_param_named(boost_value, stune_value, uint, 0644);
MODULE_PARM_DESC(boost_value, "Boost value 0-100 (default: 30)");
module_param_named(interval_ms, stune_interval_ms, uint, 0644);
MODULE_PARM_DESC(interval_ms, "Polling interval in ms (default: 1000)");

static struct task_struct *stune_thread;
static unsigned int stune_last_written = UINT_MAX;

/* Write a value to the schedtune.boost control file */
static void stune_do_write(unsigned int value)
{
	struct file *file;
	char buf[8];
	int len;
	loff_t pos = 0;

	if (value == stune_last_written)
		return;

	file = filp_open(stune_boost_file, O_WRONLY, 0);
	if (IS_ERR(file))
		return;

	len = snprintf(buf, sizeof(buf), "%u\n", value);
	kernel_write(file, buf, len, &pos);
	filp_close(file, NULL);

	stune_last_written = value;
}

/* Check if any task is currently in the top-app cgroup */
static bool stune_has_top_app_tasks(void)
{
	struct task_struct *p;
	bool found = false;

	rcu_read_lock();
	for_each_process(p) {
		struct cgroup_subsys_state *css;

		if (p->flags & PF_EXITING || p->flags & PF_KTHREAD)
			continue;

		css = task_css(p, cpu_cgrp_id);
		if (!css || !css->cgroup || !css->cgroup->kn)
			continue;

		if (strcmp(css->cgroup->kn->name, "top-app") == 0) {
			found = true;
			break;
		}
	}
	rcu_read_unlock();

	return found;
}

/* Main worker thread: poll for top-app presence and write boost */
static int stune_worker(void *data)
{
	while (!kthread_should_stop()) {
		if (READ_ONCE(stune_enabled)) {
			bool active = stune_has_top_app_tasks();

			if (active)
				stune_do_write(READ_ONCE(stune_value));
			else
				stune_do_write(0);
		}

		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(READ_ONCE(stune_interval_ms)));
		__set_current_state(TASK_RUNNING);
	}
	return 0;
}

static int __init stune_init(void)
{
	stune_thread = kthread_run(stune_worker, NULL, "zenith_stune");
	if (IS_ERR(stune_thread)) {
		pr_err("zenith_stune_boost: failed to start worker thread\n");
		return PTR_ERR(stune_thread);
	}

	pr_info("zenith_stune_boost: auto mode, value=%u interval=%ums\n",
		READ_ONCE(stune_value), READ_ONCE(stune_interval_ms));
	return 0;
}

static void __exit stune_exit(void)
{
	if (stune_thread && !IS_ERR(stune_thread)) {
		kthread_stop(stune_thread);
		stune_do_write(0);
	}
}

module_init(stune_init);
module_exit(stune_exit);

MODULE_DESCRIPTION("Zenith Stune Boost — automatic top-app schedtune boost");
MODULE_AUTHOR("XTENSEI");
MODULE_LICENSE("GPL v2");
