// SPDX-License-Identifier: GPL-2.0
/*
 * Zenith CPUFreq governor (Schedutil + Reflex + Ondemand Hybrid)
 * Developed by ENI for LO.
 *
 * This governor merges Schedutil's I/O wait boosting and PELT CFS tracking
 * with Reflex's raw hardware idle-time tracking, topped with Ondemand's
 * hard utilization thresholds for absolute zero-latency UI response.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "sched.h"

#include <linux/cpufreq.h>
#include <linux/sched/cpufreq.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/tick.h>
#include <linux/irq_work.h>
#include <linux/kthread.h>
#include <linux/mutex.h>

#define IOWAIT_BOOST_MIN	(SCHED_CAPACITY_SCALE / 8)

/* Default Tunables */
#define ZENITH_DEFAULT_UP_THRESHOLD 80
#define ZENITH_DEFAULT_HISPEED_WINDOW_US 4000
#define ZENITH_DEFAULT_RATE_LIMIT_US 1000

struct zenith_tunables {
	struct gov_attr_set	attr_set;
	unsigned int		rate_limit_us;
	unsigned int		up_threshold;
	unsigned int		hispeed_window_us;
};

struct zenith_policy {
	struct cpufreq_policy	*policy;
	struct zenith_tunables	*tunables;
	struct list_head	tunables_hook;

	raw_spinlock_t		update_lock;
	u64			last_freq_update_time;
	s64			freq_update_delay_ns;
	unsigned int		next_freq;
	unsigned int		cached_raw_freq;

	/* Slow-path fallback infrastructure */
	struct irq_work		irq_work;
	struct kthread_work	work;
	struct mutex		work_lock;
	struct kthread_worker	worker;
	struct task_struct	*thread;
	bool			work_in_progress;

	bool			limits_changed;
	bool			need_freq_update;
};

struct zenith_cpu {
	struct update_util_data	update_util;
	struct zenith_policy	*z_policy;
	unsigned int		cpu;

	/* I/O Wait Boost (Schedutil compatible) */
	bool			iowait_boost_pending;
	unsigned int		iowait_boost;
	u64			last_update;

	/* Idle-time accounting (Reflex style) */
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

static unsigned long zenith_get_hispeed_util(struct zenith_cpu *z_cpu, u64 time, unsigned long max_cap, unsigned int window_us)
{
	u64 cur_idle, cur_wall;
	unsigned int wall_delta, idle_delta, busy_pct;

	cur_idle = get_cpu_idle_time(z_cpu->cpu, &cur_wall, 1);
	wall_delta = (unsigned int)(cur_wall - z_cpu->prev_wall_time);

	if (wall_delta >= window_us) {
		if (cur_idle > z_cpu->prev_idle_time)
			idle_delta = (unsigned int)(cur_idle - z_cpu->prev_idle_time);
		else
			idle_delta = 0;

		if (wall_delta > idle_delta)
			busy_pct = 100 * (wall_delta - idle_delta) / wall_delta;
		else
			busy_pct = 0;

		/* Fast arithmetic EWMA filter */
		if (busy_pct >= z_cpu->filtered_busy_pct)
			z_cpu->filtered_busy_pct = busy_pct;
		else
			z_cpu->filtered_busy_pct -= (z_cpu->filtered_busy_pct - busy_pct) >> 1;

		z_cpu->prev_idle_time = cur_idle;
		z_cpu->prev_wall_time = cur_wall;
	}

	return (max_cap * z_cpu->filtered_busy_pct) / 100;
}

/************************ Core Scaling Logic ***********************/

static bool zenith_should_update_freq(struct zenith_policy *z_policy, u64 time)
{
	s64 delta_ns;

	if (!cpufreq_this_cpu_can_update(z_policy->policy))
		return false;

	if (unlikely(READ_ONCE(z_policy->limits_changed))) {
		WRITE_ONCE(z_policy->limits_changed, false);
		z_policy->need_freq_update = true;
		smp_mb();
		return true;
	} else if (z_policy->need_freq_update) {
		return true;
	}

	delta_ns = time - z_policy->last_freq_update_time;
	return delta_ns >= z_policy->freq_update_delay_ns;
}

static unsigned int zenith_get_next_freq(struct zenith_policy *z_policy, unsigned long util, unsigned long max_cap)
{
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned int freq;

	/* Ondemand Stage: If util exceeds threshold, spike instantly */
	if ((util * 100) / max_cap >= z_policy->tunables->up_threshold) {
		freq = policy->max;
	} else {
		/* Proportional scaling */
		freq = policy->min + ((policy->max - policy->min) * util / max_cap);
	}

	if (freq == z_policy->cached_raw_freq && !z_policy->need_freq_update)
		return z_policy->next_freq;

	z_policy->cached_raw_freq = freq;
	return cpufreq_driver_resolve_freq(policy, freq);
}

static void zenith_deferred_update(struct zenith_policy *z_policy)
{
	if (!z_policy->work_in_progress) {
		z_policy->work_in_progress = true;
		irq_work_queue(&z_policy->irq_work);
	}
}

static void zenith_update_single(struct update_util_data *hook, u64 time, unsigned int flags)
{
	struct zenith_cpu *z_cpu = container_of(hook, struct zenith_cpu, update_util);
	struct zenith_policy *z_policy = z_cpu->z_policy;
	struct zenith_tunables *tunables = z_policy->tunables;
	unsigned long util, hispeed_util, max_cap;
	unsigned int next_freq;

	zenith_iowait_boost(z_cpu, time, flags);
	z_cpu->last_update = time;

	if (!zenith_should_update_freq(z_policy, time))
		return;

	max_cap = arch_scale_cpu_capacity(z_cpu->cpu);
	util = cpu_util_cfs(cpu_rq(z_cpu->cpu));
	util = schedutil_cpu_util(z_cpu->cpu, util, max_cap, FREQUENCY_UTIL, NULL);
	
	util = zenith_iowait_apply(z_cpu, time, util, max_cap);
	hispeed_util = zenith_get_hispeed_util(z_cpu, time, max_cap, tunables->hispeed_window_us);
	util = max(util, hispeed_util);

	next_freq = zenith_get_next_freq(z_policy, util, max_cap);

	if (z_policy->next_freq == next_freq && !z_policy->need_freq_update)
		return;

	z_policy->next_freq = next_freq;
	z_policy->last_freq_update_time = time;
	z_policy->need_freq_update = false;

	if (z_policy->policy->fast_switch_enabled) {
		cpufreq_driver_fast_switch(z_policy->policy, next_freq);
	} else {
		raw_spin_lock(&z_policy->update_lock);
		zenith_deferred_update(z_policy);
		raw_spin_unlock(&z_policy->update_lock);
	}
}

static void zenith_update_shared(struct update_util_data *hook, u64 time, unsigned int flags)
{
	struct zenith_cpu *z_cpu = container_of(hook, struct zenith_cpu, update_util);
	struct zenith_policy *z_policy = z_cpu->z_policy;
	struct zenith_tunables *tunables = z_policy->tunables;
	unsigned long util = 0, max_cap = 1;
	unsigned int next_freq, j;

	raw_spin_lock(&z_policy->update_lock);

	zenith_iowait_boost(z_cpu, time, flags);
	z_cpu->last_update = time;

	if (zenith_should_update_freq(z_policy, time)) {
		
		/* Scan cluster for highest util */
		for_each_cpu(j, z_policy->policy->cpus) {
			struct zenith_cpu *j_z_cpu = &per_cpu(zenith_cpu, j);
			unsigned long j_util, j_max, j_hispeed;

			j_max = arch_scale_cpu_capacity(j);
			j_util = schedutil_cpu_util(j, cpu_util_cfs(cpu_rq(j)), j_max, FREQUENCY_UTIL, NULL);
			j_util = zenith_iowait_apply(j_z_cpu, time, j_util, j_max);
			
			j_hispeed = zenith_get_hispeed_util(j_z_cpu, time, j_max, tunables->hispeed_window_us);
			j_util = max(j_util, j_hispeed);

			if (j_util * max_cap > j_max * util) {
				util = j_util;
				max_cap = j_max;
			}
		}

		next_freq = zenith_get_next_freq(z_policy, util, max_cap);

		if (z_policy->next_freq != next_freq || z_policy->need_freq_update) {
			z_policy->next_freq = next_freq;
			z_policy->last_freq_update_time = time;
			z_policy->need_freq_update = false;

			if (z_policy->policy->fast_switch_enabled)
				cpufreq_driver_fast_switch(z_policy->policy, next_freq);
			else
				zenith_deferred_update(z_policy);
		}
	}
	raw_spin_unlock(&z_policy->update_lock);
}

/************************ Kthread (Slow Path) ***********************/

static void zenith_work(struct kthread_work *work)
{
	struct zenith_policy *z_policy = container_of(work, struct zenith_policy, work);
	unsigned int freq;
	unsigned long flags;

	raw_spin_lock_irqsave(&z_policy->update_lock, flags);
	freq = z_policy->next_freq;
	z_policy->work_in_progress = false;
	raw_spin_unlock_irqrestore(&z_policy->update_lock, flags);

	mutex_lock(&z_policy->work_lock);
	__cpufreq_driver_target(z_policy->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&z_policy->work_lock);
}

static void zenith_irq_work(struct irq_work *irq_work)
{
	struct zenith_policy *z_policy = container_of(irq_work, struct zenith_policy, irq_work);
	kthread_queue_work(&z_policy->worker, &z_policy->work);
}

/************************** Sysfs Interface ************************/

static struct zenith_tunables *global_tunables;
static DEFINE_MUTEX(global_tunables_lock);

static inline struct zenith_tunables *to_zenith_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct zenith_tunables, attr_set);
}

/* Sysfs Show/Store macros */
#define ZENITH_TUNABLE_UINT(name) \
static ssize_t name##_show(struct gov_attr_set *attr_set, char *buf) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	return sprintf(buf, "%u\n", t->name); \
} \
static ssize_t name##_store(struct gov_attr_set *attr_set, const char *buf, size_t count) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	unsigned int val; \
	if (kstrtouint(buf, 10, &val)) return -EINVAL; \
	t->name = val; \
	return count; \
} \
static struct governor_attr name = __ATTR_RW(name)

ZENITH_TUNABLE_UINT(up_threshold);
ZENITH_TUNABLE_UINT(hispeed_window_us);

static ssize_t rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	return sprintf(buf, "%u\n", t->rate_limit_us);
}

static ssize_t rate_limit_us_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_pol;
	unsigned int val;
	if (kstrtouint(buf, 10, &val)) return -EINVAL;
	t->rate_limit_us = val;
	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook)
		z_pol->freq_update_delay_ns = val * NSEC_PER_USEC;
	return count;
}
static struct governor_attr rate_limit_us = __ATTR_RW(rate_limit_us);

static struct attribute *zenith_attrs[] = {
	&rate_limit_us.attr,
	&up_threshold.attr,
	&hispeed_window_us.attr,
	NULL
};
ATTRIBUTE_GROUPS(zenith);

static void zenith_tunables_free(struct kobject *kobj)
{
	struct gov_attr_set *attr_set = container_of(kobj, struct gov_attr_set, kobj);
	kfree(to_zenith_tunables(attr_set));
}

static struct kobj_type zenith_tunables_ktype = {
	.default_groups = zenith_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = &zenith_tunables_free,
};

/********************** Initialization *********************/

struct cpufreq_governor zenith_gov;

static int zenith_init(struct cpufreq_policy *policy)
{
	struct zenith_policy *z_policy;
	struct zenith_tunables *tunables;
	int ret = 0;

	if (policy->governor_data) return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	z_policy = kzalloc(sizeof(*z_policy), GFP_KERNEL);
	if (!z_policy) return -ENOMEM;

	z_policy->policy = policy;
	raw_spin_lock_init(&z_policy->update_lock);

	/* Initialize slow-path kthreads just in case */
	if (!policy->fast_switch_enabled) {
		kthread_init_work(&z_policy->work, zenith_work);
		kthread_init_worker(&z_policy->worker);
		z_policy->thread = kthread_create(kthread_worker_fn, &z_policy->worker, "zenith:%d", cpumask_first(policy->related_cpus));
		if (IS_ERR(z_policy->thread)) {
			kfree(z_policy);
			return PTR_ERR(z_policy->thread);
		}
		kthread_bind_mask(z_policy->thread, policy->related_cpus);
		init_irq_work(&z_policy->irq_work, zenith_irq_work);
		mutex_init(&z_policy->work_lock);
		wake_up_process(z_policy->thread);
	}

	mutex_lock(&global_tunables_lock);
	if (!global_tunables) {
		tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
		gov_attr_set_init(&tunables->attr_set, &z_policy->tunables_hook);
		tunables->rate_limit_us = ZENITH_DEFAULT_RATE_LIMIT_US;
		tunables->up_threshold = ZENITH_DEFAULT_UP_THRESHOLD;
		tunables->hispeed_window_us = ZENITH_DEFAULT_HISPEED_WINDOW_US;
		
		ret = kobject_init_and_add(&tunables->attr_set.kobj, &zenith_tunables_ktype, get_governor_parent_kobj(policy), "zenith");
		if (!ret) global_tunables = tunables;
	} else {
		tunables = global_tunables;
		gov_attr_set_get(&tunables->attr_set, &z_policy->tunables_hook);
	}
	mutex_unlock(&global_tunables_lock);

	z_policy->tunables = tunables;
	policy->governor_data = z_policy;

	pr_info("Zenith: Full architecture deployed.\n");
	return ret;
}

static void zenith_exit(struct cpufreq_policy *policy)
{
	struct zenith_policy *z_policy = policy->governor_data;
	struct zenith_tunables *tunables = z_policy->tunables;

	mutex_lock(&global_tunables_lock);
	if (!gov_attr_set_put(&tunables->attr_set, &z_policy->tunables_hook))
		global_tunables = NULL;
	mutex_unlock(&global_tunables_lock);

	if (!policy->fast_switch_enabled) {
		kthread_flush_worker(&z_policy->worker);
		kthread_stop(z_policy->thread);
		mutex_destroy(&z_policy->work_lock);
	}

	policy->governor_data = NULL;
	cpufreq_disable_fast_switch(policy);
	kfree(z_policy);
}

static int zenith_start(struct cpufreq_policy *policy)
{
	struct zenith_policy *z_policy = policy->governor_data;
	unsigned int cpu;

	z_policy->freq_update_delay_ns = z_policy->tunables->rate_limit_us * NSEC_PER_USEC;
	z_policy->last_freq_update_time = 0;
	z_policy->next_freq = 0;
	z_policy->work_in_progress = false;
	z_policy->limits_changed = false;
	z_policy->cached_raw_freq = 0;
	z_policy->need_freq_update = cpufreq_driver_test_flags(CPUFREQ_NEED_UPDATE_LIMITS);

	for_each_cpu(cpu, policy->cpus) {
		struct zenith_cpu *z_cpu = &per_cpu(zenith_cpu, cpu);
		memset(z_cpu, 0, sizeof(*z_cpu));
		z_cpu->cpu = cpu;
		z_cpu->z_policy = z_policy;
		z_cpu->prev_idle_time = get_cpu_idle_time(cpu, &z_cpu->prev_wall_time, 1);
		
		cpufreq_add_update_util_hook(cpu, &z_cpu->update_util, 
			policy_is_shared(policy) ? zenith_update_shared : zenith_update_single);
	}
	return 0;
}

static void zenith_stop(struct cpufreq_policy *policy)
{
	struct zenith_policy *z_policy = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);
	synchronize_rcu();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&z_policy->irq_work);
		kthread_cancel_work_sync(&z_policy->work);
	}
}

static void zenith_limits(struct cpufreq_policy *policy)
{
	struct zenith_policy *z_policy = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&z_policy->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&z_policy->work_lock);
	}
	smp_wmb();
	WRITE_ONCE(z_policy->limits_changed, true);
}

struct cpufreq_governor zenith_gov = {
	.name       = "zenith",
	.init       = zenith_init,
	.exit       = zenith_exit,
	.start      = zenith_start,
	.stop       = zenith_stop,
	.limits     = zenith_limits,
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