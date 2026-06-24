// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/vindicator/sen.c
 * Sen — Touch-based gaming frequency boost via PM QoS
 *
 * Intercepts touch input events and temporarily raises the minimum CPU
 * frequency floor via PM QoS (freq_qos).  Unlike the old per-tick
 * vendor-hook approach (which ran on every cpufreq update and pinned
 * max freq the entire boost window), this applies a one-shot QoS
 * constraint that cpufreq enforces natively — zero per-tick overhead.
 *
 * Author: GrayRavens
 * Co-authored-by: Kanagawa Yamada <albert.wesley.dion@gmail.com>
 */

#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/pm_qos.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/spinlock.h>
#include <linux/input.h>

#define SEN_BOOST_DURATION_MS	500
#define SEN_BOOST_PCT		100	/* percent of cpuinfo.max_freq */

static bool sen_enabled = true;
module_param(sen_enabled, bool, 0644);
MODULE_PARM_DESC(sen_enabled, "Enable Sen gaming boost (default: true)");

static unsigned int sen_boost_ms = SEN_BOOST_DURATION_MS;
module_param(sen_boost_ms, uint, 0644);
MODULE_PARM_DESC(sen_boost_ms, "Touch boost duration in ms (default: 500)");

static unsigned int sen_boost_pct = SEN_BOOST_PCT;
module_param(sen_boost_pct, uint, 0644);
MODULE_PARM_DESC(sen_boost_pct,
		 "Boost target as percent of cpuinfo.max_freq (default: 100)");

static bool sen_is_boosted;
static DEFINE_SPINLOCK(sen_lock);
static struct delayed_work sen_boost_off_work;
static struct work_struct sen_boost_on_work;

/* Hook pointer exported by the input subsystem */
extern void (*sen_touch_hook)(unsigned int type, unsigned int code);

/* ------------------------------------------------------------------ */
/* PM QoS per-CPU state — fixed array, no krealloc (see Shun)         */
/* ------------------------------------------------------------------ */
struct sen_qos_entry {
	struct freq_qos_request min_req;
	bool			init;
	unsigned int		max_khz;
};

static struct sen_qos_entry sen_entries[NR_CPUS];

/* ------------------------------------------------------------------ */
/* Boost helpers                                                      */
/* ------------------------------------------------------------------ */

/*
 * Apply the boost: raise min freq on every online CPU's policy.
 * Lazily initialises the freq_qos request on first touch for each
 * policy-head CPU, so cpufreq does not need to be ready at init.
 *
 * Must be called from process context (workqueue) because
 * freq_qos_add/update_request may call blocking_notifier_call_chain.
 */
static void sen_apply_boost(void)
{
	unsigned int cpu;

	for_each_online_cpu(cpu) {
		struct cpufreq_policy *policy;
		struct sen_qos_entry *e = &sen_entries[cpu];

		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;

		/*
		 * Only act once per policy (cpumask_first matching
		 * policy->cpu).  Sibling CPUs share the same policy.
		 */
		if (policy->cpu == cpu) {
			unsigned int target_khz;
			bool was_init = e->init;

			/*
			 * Cache max freq on first touch so future
			 * touches skip the freq table lookup.  The
			 * policy pointer is still needed for the
			 * constraints field on the first alloc.
			 */
			if (!was_init)
				e->max_khz = policy->cpuinfo.max_freq;

			target_khz = e->max_khz * sen_boost_pct / 100U;
			if (target_khz < policy->cpuinfo.min_freq)
				target_khz = policy->cpuinfo.min_freq;

			if (!was_init) {
				freq_qos_add_request(&policy->constraints,
						     &e->min_req,
						     FREQ_QOS_MIN,
						     target_khz);
				e->init = true;
			} else {
				freq_qos_update_request(&e->min_req,
							target_khz);
			}
		}
		cpufreq_cpu_put(policy);
	}
}

/*
 * Release the boost: restore min freq to each policy's natural floor.
 * Does NOT remove the QoS request — that is done by sen_qos_cleanup().
 */
static void sen_release_boost(void)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu) {
		struct cpufreq_policy *policy;
		struct sen_qos_entry *e = &sen_entries[cpu];

		if (!e->init)
			continue;

		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;

		freq_qos_update_request(&e->min_req,
					policy->cpuinfo.min_freq);
		cpufreq_cpu_put(policy);
	}
}

/* ------------------------------------------------------------------ */
/* Workqueue — runs the potential-sleeping freq_qos calls              */
/* ------------------------------------------------------------------ */
static void sen_boost_on_work_fn(struct work_struct *work)
{
	sen_apply_boost();
}

/* ------------------------------------------------------------------ */
/* Touch hook — called from the input subsystem on every touch event   */
/* ------------------------------------------------------------------ */
static void sen_trigger_boost(unsigned int type, unsigned int code)
{
	unsigned long flags;

	if (!sen_enabled)
		return;

	/*
	 * Only fire on actual touch events, not on keyboard,
	 * gamepad, power button, or other non-touch inputs.
	 * Filter for absolute multi-touch position events.
	 */
	if (type != EV_ABS)
		return;
	if (code != ABS_MT_POSITION_X && code != ABS_MT_POSITION_Y &&
	    code != ABS_MT_TRACKING_ID && code != ABS_MT_TOUCH_MAJOR &&
	    code != ABS_MT_WIDTH_MAJOR && code != ABS_PRESSURE)
		return;

	spin_lock_irqsave(&sen_lock, flags);
	if (!sen_is_boosted)
		pr_debug("sen: touch boost ON\n");
	sen_is_boosted = true;
	spin_unlock_irqrestore(&sen_lock, flags);

	/*
	 * Defer QoS work to process context: the touch hook may fire
	 * from atomic context (input subsystem event pipeline), but
	 * freq_qos_add/update_request can call into blocking notifiers.
	 */
	schedule_work(&sen_boost_on_work);

	mod_delayed_work(system_wq, &sen_boost_off_work,
			 msecs_to_jiffies(sen_boost_ms));
}

static void sen_boost_off(struct work_struct *work)
{
	unsigned long flags;

	spin_lock_irqsave(&sen_lock, flags);
	sen_is_boosted = false;
	spin_unlock_irqrestore(&sen_lock, flags);

	sen_release_boost();

	pr_debug("sen: touch boost OFF\n");
}

/* ------------------------------------------------------------------ */
/* QoS cleanup — remove all requests from the PM QoS framework         */
/* ------------------------------------------------------------------ */
static void sen_qos_cleanup(void)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu) {
		struct sen_qos_entry *e = &sen_entries[cpu];

		if (!e->init)
			continue;
		freq_qos_remove_request(&e->min_req);
		e->init = false;
		e->max_khz = 0;
	}
}

/* ------------------------------------------------------------------ */
/* Init / Exit                                                        */
/* ------------------------------------------------------------------ */
static int __init sen_init(void)
{
	memset(sen_entries, 0, sizeof(sen_entries));

	INIT_DELAYED_WORK(&sen_boost_off_work, sen_boost_off);
	INIT_WORK(&sen_boost_on_work, sen_boost_on_work_fn);

	/*
	 * Hook into the input subsystem via the exported
	 * sen_touch_hook pointer.  No per-tick vendor hook.
	 */
	sen_touch_hook = sen_trigger_boost;

	pr_info("sen: active (boost=%ums, pct=%u%%)\n",
		sen_boost_ms, sen_boost_pct);
	return 0;
}
late_initcall(sen_init);

static void __exit sen_exit(void)
{
	sen_touch_hook = NULL;

	/* Cancel pending QoS work and the boost-off timer */
	cancel_work_sync(&sen_boost_on_work);
	cancel_delayed_work_sync(&sen_boost_off_work);

	/* Release any active QoS constraint */
	sen_release_boost();

	/* Remove all QoS requests from the PM QoS framework */
	sen_qos_cleanup();

	pr_info("sen: unloaded\n");
}
module_exit(sen_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GrayRavens");
MODULE_DESCRIPTION("Sen — Touch-based gaming frequency boost via PM QoS");
