// SPDX-License-Identifier: GPL-2.0
/*
 * Zenith CPUFreq governor (Schedutil + Reflex + Ondemand Hybrid)
 * For LO, by ENI.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "sched.h"

#include <linux/cpufreq.h>
#include <linux/sched/cpufreq.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/tick.h>

#define IOWAIT_BOOST_MIN	(SCHED_CAPACITY_SCALE / 8)
#define ZENITH_DEFAULT_UP_THRESHOLD 80
#define ZENITH_HISPEED_WINDOW_US 4000

struct zenith_policy {
	struct cpufreq_policy	*policy;
	raw_spinlock_t		update_lock;
	u64			last_freq_update_time;
	s64			rate_limit_ns;
	unsigned int		next_freq;
	unsigned int		up_threshold;
	bool			need_freq_update;
};

struct zenith_cpu {
	struct update_util_data	update_util;
	struct zenith_policy	*z_policy;
	unsigned int		cpu;

	/* Schedutil I/O Wait Boost */
	bool			iowait_boost_pending;
	unsigned int		iowait_boost;
	u64			last_update;

	/* Reflex Idle-Time Accounting */
	u64			prev_idle_time;
	u64			prev_wall_time;
	unsigned int		filtered_busy_pct;
};

static DEFINE_PER_CPU(struct zenith_cpu, zenith_cpu);

/************************ Schedutil: I/O Wait Logic ***********************/

static bool zenith_iowait_reset(struct zenith_cpu *z_cpu, u64 time, bool set_iowait_boost)
{
	s64 delta_ns = time - z_cpu->last_update;
	if (delta_ns <= TICK_NSEC)
		return false;
	z_cpu->iowait_boost = set_iowait_boost ? IOWAIT_BOOST_MIN : 0;
	z_cpu->iowait_boost_pending = set_iowait_boost;
	return true;
}

static void zenith_iowait_boost(struct zenith_cpu *z_cpu, u64 time, unsigned int flags)
{
	bool set_iowait_boost = flags & SCHED_CPUFREQ_IOWAIT;

	if (z_cpu->iowait_boost && zenith_iowait_reset(z_cpu, time, set_iowait_boost))
		return;
	if (!set_iowait_boost)
		return;
	if (z_cpu->iowait_boost_pending)
		return;
	
	z_cpu->iowait_boost_pending = true;
	if (z_cpu->iowait_boost) {
		z_cpu->iowait_boost = min_t(unsigned int, z_cpu->iowait_boost << 1, SCHED_CAPACITY_SCALE);
		return;
	}
	z_cpu->iowait_boost = IOWAIT_BOOST_MIN;
}

static unsigned long zenith_iowait_apply(struct zenith_cpu *z_cpu, u64 time, unsigned long util, unsigned long max_cap)
{
	unsigned long boost;

	if (!z_cpu->iowait_boost)
		return util;
	if (zenith_iowait_reset(z_cpu, time, false))
		return util;
	if (!z_cpu->iowait_boost_pending) {
		z_cpu->iowait_boost >>= 1;
		if (z_cpu->iowait_boost < IOWAIT_BOOST_MIN) {
			z_cpu->iowait_boost = 0;
			return util;
		}
	}
	z_cpu->iowait_boost_pending = false;

	boost = (z_cpu->iowait_boost * max_cap) >> SCHED_CAPACITY_SHIFT;
	return max(boost, util);
}

/************************ Reflex: Hispeed Accounting ***********************/

static unsigned long zenith_get_hispeed_util(struct zenith_cpu *z_cpu, u64 time, unsigned long max_cap)
{
	u64 cur_idle, cur_wall;
	unsigned int wall_delta, idle_delta, busy_pct;

	cur_idle = get_cpu_idle_time(z_cpu->cpu, &cur_wall, 1);
	wall_delta = (unsigned int)(cur_wall - z_cpu->prev_wall_time);

	if (wall_delta >= ZENITH_HISPEED_WINDOW_US) {
		if (cur_idle > z_cpu->prev_idle_time)
			idle_delta = (unsigned int)(cur_idle - z_cpu->prev_idle_time);
		else
			idle_delta = 0;

		if (wall_delta > idle_delta)
			busy_pct = 100 * (wall_delta - idle_delta) / wall_delta;
		else
			busy_pct = 0;

		/* Fast arithmetic EWMA shift instead of heavy Log LUTs */
		if (busy_pct >= z_cpu->filtered_busy_pct)
			z_cpu->filtered_busy_pct = busy_pct; // Instant up
		else
			z_cpu->filtered_busy_pct -= (z_cpu->filtered_busy_pct - busy_pct) >> 1; // Smooth down

		z_cpu->prev_idle_time = cur_idle;
		z_cpu->prev_wall_time = cur_wall;
	}

	return (max_cap * z_cpu->filtered_busy_pct) / 100;
}

/************************ Core Scheduler Hook ***********************/

static void zenith_update_single(struct update_util_data *hook, u64 time, unsigned int flags)
{
	struct zenith_cpu *z_cpu = container_of(hook, struct zenith_cpu, update_util);
	struct zenith_policy *z_policy = z_cpu->z_policy;
	struct cpufreq_policy *policy = z_policy->policy;
	
	unsigned long util, hispeed_util, max_cap;
	unsigned int next_freq;

	/* Rate Limiting */
	if (time - z_policy->last_freq_update_time < z_policy->rate_limit_ns && !z_policy->need_freq_update)
		return;

	/* 1. Schedutil I/O Boost processing */
	zenith_iowait_boost(z_cpu, time, flags);
	z_cpu->last_update = time;

	/* 2. Raw CFS PELT Extraction (First-class sched.h access) */
	util = cpu_util_cfs(cpu_rq(z_cpu->cpu));
	max_cap = arch_scale_cpu_capacity(z_cpu->cpu);
	util = schedutil_cpu_util(z_cpu->cpu, util, max_cap, FREQUENCY_UTIL, NULL);
	
	/* Apply IO Wait multiplier */
	util = zenith_iowait_apply(z_cpu, time, util, max_cap);

	/* 3. Reflex Hispeed Blend */
	hispeed_util = zenith_get_hispeed_util(z_cpu, time, max_cap);
	util = max(util, hispeed_util);

	/* 4. Ondemand Threshold Logic */
	if ((util * 100) / max_cap >= z_policy->up_threshold) {
		next_freq = policy->max;
	} else {
		/* Proportional scaling if below threshold */
		next_freq = policy->min + ((policy->max - policy->min) * util / max_cap);
	}

	/* Execute Switch */
	if (z_policy->next_freq == next_freq && !z_policy->need_freq_update)
		return;

	z_policy->next_freq = next_freq;
	z_policy->last_freq_update_time = time;
	z_policy->need_freq_update = false;

	if (policy->fast_switch_enabled) {
		cpufreq_driver_fast_switch(policy, next_freq);
	} else {
		/* Fallback for non-fast-switch platforms */
		__cpufreq_driver_target(policy, next_freq, CPUFREQ_RELATION_L);
	}
}

/************************ Initialization & Sysfs ***********************/

static int zenith_start(struct cpufreq_policy *policy)
{
	struct zenith_policy *z_policy = policy->governor_data;
	unsigned int cpu;

	z_policy->rate_limit_ns = 1000 * NSEC_PER_USEC; /* 1ms default */
	z_policy->up_threshold = ZENITH_DEFAULT_UP_THRESHOLD;
	z_policy->last_freq_update_time = 0;
	z_policy->need_freq_update = true;

	for_each_cpu(cpu, policy->cpus) {
		struct zenith_cpu *z_cpu = &per_cpu(zenith_cpu, cpu);
		memset(z_cpu, 0, sizeof(*z_cpu));
		z_cpu->cpu = cpu;
		z_cpu->z_policy = z_policy;
		z_cpu->prev_idle_time = get_cpu_idle_time(cpu, &z_cpu->prev_wall_time, 1);
		
		cpufreq_add_update_util_hook(cpu, &z_cpu->update_util, zenith_update_single);
	}
	return 0;
}

static void zenith_stop(struct cpufreq_policy *policy)
{
	unsigned int cpu;
	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);
	synchronize_rcu();
}

static int zenith_init(struct cpufreq_policy *policy)
{
	struct zenith_policy *z_policy;

	if (policy->governor_data) return -EBUSY;

	cpufreq_enable_fast_switch(policy);
	z_policy = kzalloc(sizeof(*z_policy), GFP_KERNEL);
	if (!z_policy) return -ENOMEM;

	z_policy->policy = policy;
	raw_spin_lock_init(&z_policy->update_lock);
	policy->governor_data = z_policy;

	pr_info("Zenith: Awakened inside the scheduler core.\n");
	return 0;
}

static void zenith_exit(struct cpufreq_policy *policy)
{
	struct zenith_policy *z_policy = policy->governor_data;
	policy->governor_data = NULL;
	cpufreq_disable_fast_switch(policy);
	kfree(z_policy);
}

static struct cpufreq_governor zenith_gov = {
	.name       = "zenith",
	.init       = zenith_init,
	.exit       = zenith_exit,
	.start      = zenith_start,
	.stop       = zenith_stop,
	.owner      = THIS_MODULE,
	.flags      = CPUFREQ_GOV_DYNAMIC_SWITCHING,
};

static int __init zenith_module_init(void)
{
	return cpufreq_register_governor(&zenith_gov);
}

static void __exit zenith_module_exit(void)
{
	cpufreq_unregister_governor(&zenith_gov);
}

module_init(zenith_module_init);
module_exit(zenith_module_exit);

MODULE_AUTHOR("ENI for LO");
MODULE_DESCRIPTION("Zenith Hybrid Governor (Schedutil/Ondemand/Reflex)");
MODULE_LICENSE("GPL");