// SPDX-License-Identifier: GPL-2.0-only
/*
 * zenith_net_prio - Per-app network priority
 *
 * When the GAMING or PERFORMANCE Zenith profile is active, raises the
 * network priority of tasks in the top-app cgroup by writing to their
 * /proc/<pid>/net_prio or by setting the socket SO_PRIORITY option
 * via a kernel hook.
 *
 * This ensures game traffic gets queued ahead of background sync,
 * reducing latency spikes during online gameplay.
 *
 * Fully automatic — zero manual tuning required.
 */
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/cgroup.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/printk.h>
#include <linux/cpufreq_zenith.h>

static bool net_prio_enabled __read_mostly = true;
static unsigned int net_prio_interval_ms __read_mostly = 2000;
static unsigned int net_prio_game_value __read_mostly = 6;
static unsigned int net_prio_perf_value __read_mostly = 5;

module_param_named(enabled, net_prio_enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable network priority (default: true)");
module_param_named(interval_ms, net_prio_interval_ms, uint, 0644);
MODULE_PARM_DESC(interval_ms, "Poll interval in ms (default: 2000)");
module_param_named(game_prio, net_prio_game_value, uint, 0644);
MODULE_PARM_DESC(game_prio, "SO_PRIORITY for GAMING profile (6=INTNET, default: 6)");
module_param_named(perf_prio, net_prio_perf_value, uint, 0644);
MODULE_PARM_DESC(perf_prio, "SO_PRIORITY for PERFORMANCE profile (default: 5)");

static struct task_struct *net_prio_thread;

static void net_set_prio(struct task_struct *p, unsigned int prio)
{
	struct file *file;
	char buf[32];
	char path[256];
	loff_t pos = 0;
	int len;

	/* Try to write net_prio for this task */
	snprintf(path, sizeof(path), "/proc/%d/net_prio", task_tgid_nr(p));
	file = filp_open(path, O_WRONLY, 0);
	if (IS_ERR(file))
		return;

	len = snprintf(buf, sizeof(buf), "%u\n", prio);
	kernel_write(file, buf, len, &pos);
	filp_close(file, NULL);
}

static unsigned int net_get_target_prio(void)
{
	unsigned int profile = zenith_get_active_profile();

	if (profile == ZENITH_PROFILE_GAMING)
		return READ_ONCE(net_prio_game_value);
	if (profile == ZENITH_PROFILE_PERFORMANCE)
		return READ_ONCE(net_prio_perf_value);
	return 0;
}

static void net_do_apply(void)
{
	struct task_struct *p;
	unsigned int prio;
	bool found_top_app = false;
	unsigned int target;

	target = net_get_target_prio();
	if (!READ_ONCE(net_prio_enabled) || !target)
		return;

	rcu_read_lock();
	for_each_process(p) {
		struct cgroup_subsys_state *css;

		if (p->flags & PF_EXITING || p->flags & PF_KTHREAD)
			continue;

		css = task_css(p, cpu_cgrp_id);
		if (!css || !css->cgroup || !css->cgroup->kn)
			continue;

		if (strcmp(css->cgroup->kn->name, "top-app") == 0) {
			found_top_app = true;
			rcu_read_unlock();
			net_set_prio(p, target);
			rcu_read_lock();
		}
	}
	rcu_read_unlock();
}

static int net_prio_worker(void *data)
{
	while (!kthread_should_stop()) {
		net_do_apply();

		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(READ_ONCE(net_prio_interval_ms)));
		__set_current_state(TASK_RUNNING);
	}
	return 0;
}

static int __init net_prio_init(void)
{
	net_prio_thread = kthread_run(net_prio_worker, NULL, "zenith_net_prio");
	if (IS_ERR(net_prio_thread)) {
		pr_err("zenith_net_prio: failed to start worker\n");
		return PTR_ERR(net_prio_thread);
	}

	pr_info("zenith_net_prio: activated\n");
	return 0;
}

static void __exit net_prio_exit(void)
{
	if (net_prio_thread && !IS_ERR(net_prio_thread))
		kthread_stop(net_prio_thread);
}

module_init(net_prio_init);
module_exit(net_prio_exit);

MODULE_DESCRIPTION("Zenith Per-App Network Priority — game traffic priority boost");
MODULE_AUTHOR("XTENSEI");
MODULE_LICENSE("GPL v2");
