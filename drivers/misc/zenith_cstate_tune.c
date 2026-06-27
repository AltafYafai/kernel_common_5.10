// SPDX-License-Identifier: GPL-2.0-only
/*
 * zenith_cstate_tune - Automatic C-state latency tuning
 *
 * When the top-app cgroup is active (foreground app running), raises the
 * PM QoS CPU_DMA_LATENCY requirement so cpuidle avoids deep C-states that
 * have high wake-up latency.  When no top-app tasks are active, releases
 * the requirement so the CPU can enter deep idle for power saving.
 *
 * Fully automatic — zero manual tuning required.
 *
 * Tunables (/sys/module/zenith_cstate_tune/parameters/):
 *   enabled       — master switch (1/0, default 1)
 *   latency_us    — PM QoS latency requirement in µs when active (1-500, default 50)
 *   interval_ms   — polling interval (default 1000)
 */
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/cgroup.h>
#include <linux/pm_qos.h>
#include <linux/printk.h>

static bool cstate_enabled __read_mostly = true;
static s32 cstate_latency_us __read_mostly = 50;
static unsigned int cstate_interval_ms __read_mostly = 1000;

module_param_named(enabled, cstate_enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable C-state tuning (default: true)");
module_param_named(latency_us, cstate_latency_us, int, 0644);
MODULE_PARM_DESC(latency_us, "PM QoS latency in us when active (1-500, default 50)");
module_param_named(interval_ms, cstate_interval_ms, uint, 0644);
MODULE_PARM_DESC(interval_ms, "Polling interval in ms (default: 1000)");

static struct task_struct *cstate_thread;
static struct pm_qos_request cstate_qos_req;
static bool cstate_boost_active;

/* Check if any task is in the top-app cgroup */
static bool cstate_has_top_app(void)
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

static int cstate_worker(void *data)
{
	while (!kthread_should_stop()) {
		if (READ_ONCE(cstate_enabled)) {
			bool active = cstate_has_top_app();

			if (active && !cstate_boost_active) {
				cpu_latency_qos_add_request(&cstate_qos_req,
							    READ_ONCE(cstate_latency_us));
				cstate_boost_active = true;
			} else if (!active && cstate_boost_active) {
				cpu_latency_qos_remove_request(&cstate_qos_req);
				cstate_boost_active = false;
			}
		}

		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(READ_ONCE(cstate_interval_ms)));
		__set_current_state(TASK_RUNNING);
	}
	return 0;
}

static int __init cstate_init(void)
{
	cstate_thread = kthread_run(cstate_worker, NULL, "zenith_cstate");
	if (IS_ERR(cstate_thread)) {
		pr_err("zenith_cstate_tune: failed to start worker\n");
		return PTR_ERR(cstate_thread);
	}

	pr_info("zenith_cstate_tune: latency=%dus interval=%ums\n",
		READ_ONCE(cstate_latency_us), READ_ONCE(cstate_interval_ms));
	return 0;
}

static void __exit cstate_exit(void)
{
	if (cstate_thread && !IS_ERR(cstate_thread)) {
		kthread_stop(cstate_thread);
		if (cstate_boost_active)
			cpu_latency_qos_remove_request(&cstate_qos_req);
	}
}

module_init(cstate_init);
module_exit(cstate_exit);

MODULE_DESCRIPTION("Zenith C-State Tuning — automatic PM QoS for lower latency");
MODULE_AUTHOR("XTENSEI");
MODULE_LICENSE("GPL v2");
