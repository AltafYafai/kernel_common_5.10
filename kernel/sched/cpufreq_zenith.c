// SPDX-License-Identifier: GPL-2.0
/*
 * Zenith CPUFreq Governor V2 (EAS/EM/Thermal/Display Hybrid)
 * Developed by ENI exclusively for LO.
 *
 * Architecture Additions:
 * 1. Energy Model (EM) Awareness: Reads mW costs from the device tree to prevent 
 * inefficient frequency spikes during thermal throttling.
 * 2. Display-State Awareness: `screen_state` sysfs hook kills Reflex tracking 
 * and forces deep-sleep biases when the display is off.
 * 3. Dynamic Thermal Thresholding: `thermal_state` sysfs hook dynamically 
 * relaxes up_thresholds to let silicon breathe.
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
#include <linux/energy_model.h>
#include <trace/events/power.h>

/* Constants & Defaults */
#define IOWAIT_BOOST_MIN			(SCHED_CAPACITY_SCALE / 8)
#define ZENITH_DEFAULT_UP_THRESHOLD		80
#define ZENITH_DEFAULT_HISPEED_WINDOW_US	4000
#define ZENITH_DEFAULT_HISPEED_FILTER_SHIFT	1
#define ZENITH_DEFAULT_UP_RATE_LIMIT_US		500
#define ZENITH_DEFAULT_DOWN_RATE_LIMIT_US	2000
#define ZENITH_DEFAULT_POWERSAVE_BIAS		0
#define ZENITH_DEFAULT_IO_IS_BUSY		1

/*
 * Zenith Tunables & State API
 */
struct zenith_tunables {
	struct gov_attr_set	attr_set;
	unsigned int		up_rate_limit_us;
	unsigned int		down_rate_limit_us;
	unsigned int		up_threshold;
	unsigned int		hispeed_window_us;
	unsigned int		hispeed_filter_shift;
	unsigned int		powersave_bias;
	unsigned int		io_is_busy;
	
	/* Zenith Environment API */
	unsigned int		screen_state;   /* 1 = ON, 0 = OFF */
	unsigned int		thermal_state;  /* 0 = COOL, 1 = THROTTLING */
};

struct zenith_policy {
	struct cpufreq_policy	*policy;
	struct zenith_tunables	*tunables;
	struct list_head	tunables_hook;

	raw_spinlock_t		update_lock;
	u64			last_freq_update_time;
	s64			min_rate_limit_ns;
	s64			up_rate_delay_ns;
	s64			down_rate_delay_ns;
	unsigned int		next_freq;
	unsigned int		cached_raw_freq;

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

	bool			iowait_boost_pending;
	unsigned int		iowait_boost;
	u64			last_update;
	unsigned long		bw_dl;
	unsigned long		max_capacity;

	u64			prev_idle_time;
	u64			prev_wall_time;
	unsigned int		busy_pct;
	unsigned int		filtered_busy_pct;
	bool			hispeed_active;
};

static DEFINE_PER_CPU(struct zenith_cpu, zenith_cpu);

/************************ Schedutil: I/O Wait & DL Logic ***********************/

static bool zenith_iowait_reset(struct zenith_cpu *z_cpu, u64 time, bool set_iowait_boost)
{
	s64 delta_ns = time - z_cpu->last_update;
	if (delta_ns <= TICK_NSEC)
		return false;

	z_cpu->iowait_boost = set_iowait_boost ? IOWAIT_BOOST_MIN : 0;
	z_cpu->iowait_boost_pending = set_iowait_boost;
	return true;
}

static void zenith_iowait_boost(struct zenith_cpu *z_cpu, u64 time, unsigned int flags, unsigned int io_is_busy)
{
	bool set_iowait_boost = (flags & SCHED_CPUFREQ_IOWAIT) && io_is_busy;

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
	
	boost = max(boost, util);
	boost = uclamp_rq_util_with(cpu_rq(z_cpu->cpu), boost, NULL);
	return boost;
}

static inline void zenith_ignore_dl_rate_limit(struct zenith_cpu *z_cpu, struct zenith_policy *z_policy)
{
	if (cpu_bw_dl(cpu_rq(z_cpu->cpu)) > z_cpu->bw_dl)
		WRITE_ONCE(z_policy->limits_changed, true);
}

/************************ Reflex: Asymmetric EWMA Idle Tracking ***********************/

static void zenith_update_busy_pct(struct zenith_cpu *z_cpu, unsigned int window_us, unsigned int filter_shift)
{
	u64 cur_idle, cur_wall;
	unsigned int wall_delta, idle_delta;

	cur_idle = get_cpu_idle_time(z_cpu->cpu, &cur_wall, 1);
	wall_delta = (unsigned int)(cur_wall - z_cpu->prev_wall_time);

	if (wall_delta >= window_us) {
		z_cpu->busy_pct = 0;
		z_cpu->hispeed_active = true;
		z_cpu->prev_idle_time = cur_idle;
		z_cpu->prev_wall_time = cur_wall;
		return;
	}

	if (!z_cpu->hispeed_active)
		return;

	z_cpu->hispeed_active = false;

	if (cur_idle > z_cpu->prev_idle_time)
		idle_delta = (unsigned int)(cur_idle - z_cpu->prev_idle_time);
	else
		idle_delta = 0;

	if (wall_delta > idle_delta)
		z_cpu->busy_pct = 100 * (wall_delta - idle_delta) / wall_delta;
	else
		z_cpu->busy_pct = 0;

	z_cpu->prev_idle_time = cur_idle;
	z_cpu->prev_wall_time = cur_wall;

	if (!filter_shift || z_cpu->busy_pct >= z_cpu->filtered_busy_pct) {
		z_cpu->filtered_busy_pct = z_cpu->busy_pct;
	} else {
		unsigned int step = (z_cpu->filtered_busy_pct - z_cpu->busy_pct) >> filter_shift;
		if (step)
			z_cpu->filtered_busy_pct -= step;
		else
			z_cpu->filtered_busy_pct = z_cpu->busy_pct;
	}
}

static unsigned long zenith_blend_util(struct zenith_cpu *z_cpu, unsigned long pelt_util, unsigned long max_cap, unsigned int screen_state)
{
	unsigned long hispeed_util;

	/* If display is off, kill Reflex vision to save power */
	if (!screen_state || !z_cpu->filtered_busy_pct)
		return pelt_util;

	hispeed_util = (max_cap * z_cpu->filtered_busy_pct) / 100;
	return max(pelt_util, hispeed_util);
}

/************************ Core EAS Utilization Extraction ***********************/

static unsigned long zenith_get_util(struct zenith_cpu *z_cpu)
{
	struct rq *rq = cpu_rq(z_cpu->cpu);
	unsigned long util = cpu_util_cfs(rq);
	unsigned long max = arch_scale_cpu_capacity(z_cpu->cpu);

	z_cpu->max_capacity = max;
	z_cpu->bw_dl = cpu_bw_dl(rq);

	return schedutil_cpu_util(z_cpu->cpu, util, max, FREQUENCY_UTIL, NULL);
}

/************************ Energy Model (EM) Evaluation ***********************/

static unsigned int zenith_em_cap_freq(struct zenith_policy *z_policy, unsigned int target_freq)
{
	struct cpufreq_policy *policy = z_policy->policy;
	struct em_perf_domain *pd = em_cpu_get(policy->cpu);
	struct em_perf_state *ps;
	int i;

	/* If no Energy Model is registered or we aren't thermal throttling, skip */
	if (!pd || !z_policy->tunables->thermal_state)
		return target_freq;

	/* Scan EM array to find the mW cost of the target frequency */
	for (i = 0; i < pd->nr_perf_states; i++) {
		ps = &pd->table[i];
		if (ps->frequency >= target_freq) {
			/* * If this state consumes disproportionately high power (heuristic: 
			 * if it's the absolute highest state and we are throttling), cap it 
			 * to the previous state to save mW.
			 */
			if (i == pd->nr_perf_states - 1 && i > 0) {
				return pd->table[i - 1].frequency;
			}
			break;
		}
	}
	return target_freq;
}

/************************ Zenith Scaling Math ***********************/

static bool zenith_up_down_rate_limit(struct zenith_policy *z_policy, u64 time, unsigned int next_freq)
{
	s64 delta_ns = time - z_policy->last_freq_update_time;

	if (next_freq > z_policy->next_freq && delta_ns < z_policy->up_rate_delay_ns)
		return true;

	if (next_freq < z_policy->next_freq && delta_ns < z_policy->down_rate_delay_ns)
		return true;

	return false;
}

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
	}

	if (z_policy->work_in_progress)
		return true;

	delta_ns = time - z_policy->last_freq_update_time;
	return delta_ns >= z_policy->min_rate_limit_ns;
}

static unsigned int zenith_get_next_freq(struct zenith_policy *z_policy, unsigned long util, unsigned long max_cap)
{
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned int freq, target_freq;
	unsigned int margin;
	
	/* Dynamic Environment Overrides */
	unsigned int dynamic_up_thresh = z_policy->tunables->up_threshold;
	unsigned int dynamic_bias = z_policy->tunables->powersave_bias;

	if (z_policy->tunables->screen_state == 0) {
		dynamic_up_thresh = 95; /* Hard to wake up */
		dynamic_bias = 500;     /* 50% penalty */
	} else if (z_policy->tunables->thermal_state == 1) {
		dynamic_up_thresh = 90; /* Relaxed for thermals */
	}

	/* 1. Ondemand Brutality */
	if ((util * 100) / max_cap >= dynamic_up_thresh) {
		freq = policy->max;
		goto resolve;
	}

	/* 2. Schedutil EAS Proportional Math with Headroom */
	if (arch_scale_freq_invariant())
		freq = policy->cpuinfo.max_freq;
	else
		freq = policy->cur + (policy->cur >> 2); 

	freq = map_util_freq(util, freq, max_cap);

	/* 3. Powersave Bias */
	if (dynamic_bias) {
		margin = freq * dynamic_bias / 1000;
		freq = freq - margin;
	}

resolve:
	if (freq == z_policy->cached_raw_freq && !z_policy->need_freq_update)
		return z_policy->next_freq;

	z_policy->cached_raw_freq = freq;
	target_freq = cpufreq_driver_resolve_freq(policy, freq);

	/* 4. Energy Model Validation */
	target_freq = zenith_em_cap_freq(z_policy, target_freq);

	return target_freq;
}

static void zenith_execute_switch(struct zenith_policy *z_policy, u64 time, unsigned int next_freq)
{
	if (z_policy->need_freq_update) {
		z_policy->need_freq_update = false;
		if (z_policy->next_freq == next_freq && !cpufreq_driver_test_flags(CPUFREQ_NEED_UPDATE_LIMITS))
			return;
	} else if (z_policy->next_freq == next_freq) {
		return;
	}

	if (zenith_up_down_rate_limit(z_policy, time, next_freq))
		return;

	z_policy->next_freq = next_freq;
	z_policy->last_freq_update_time = time;

	if (z_policy->policy->fast_switch_enabled) {
		cpufreq_driver_fast_switch(z_policy->policy, next_freq);
	} else if (!z_policy->work_in_progress) {
		z_policy->work_in_progress = true;
		irq_work_queue(&z_policy->irq_work);
	}
}

/************************ Scheduler Hooks ***********************/

static void zenith_update_single(struct update_util_data *hook, u64 time, unsigned int flags)
{
	struct zenith_cpu *z_cpu = container_of(hook, struct zenith_cpu, update_util);
	struct zenith_policy *z_policy = z_cpu->z_policy;
	struct zenith_tunables *tunables = z_policy->tunables;
	unsigned long util, max_cap;
	unsigned int next_f;

	zenith_iowait_boost(z_cpu, time, flags, tunables->io_is_busy);
	z_cpu->last_update = time;

	zenith_ignore_dl_rate_limit(z_cpu, z_policy);

	if (!zenith_should_update_freq(z_policy, time))
		return;

	util = zenith_get_util(z_cpu);
	max_cap = z_cpu->max_capacity;
	
	util = zenith_iowait_apply(z_cpu, time, util, max_cap);
	
	zenith_update_busy_pct(z_cpu, tunables->hispeed_window_us, tunables->hispeed_filter_shift);
	util = zenith_blend_util(z_cpu, util, max_cap, tunables->screen_state);

	next_f = zenith_get_next_freq(z_policy, util, max_cap);

	if (z_policy->policy->fast_switch_enabled) {
		zenith_execute_switch(z_policy, time, next_f);
	} else {
		raw_spin_lock(&z_policy->update_lock);
		zenith_execute_switch(z_policy, time, next_f);
		raw_spin_unlock(&z_policy->update_lock);
	}
}

static void zenith_update_shared(struct update_util_data *hook, u64 time, unsigned int flags)
{
	struct zenith_cpu *z_cpu = container_of(hook, struct zenith_cpu, update_util);
	struct zenith_policy *z_policy = z_cpu->z_policy;
	struct zenith_tunables *tunables = z_policy->tunables;
	unsigned long util = 0, max_cap = 1;
	unsigned int next_f, j;

	raw_spin_lock(&z_policy->update_lock);

	zenith_iowait_boost(z_cpu, time, flags, tunables->io_is_busy);
	z_cpu->last_update = time;

	zenith_ignore_dl_rate_limit(z_cpu, z_policy);

	if (zenith_should_update_freq(z_policy, time)) {
		
		for_each_cpu(j, z_policy->policy->cpus) {
			struct zenith_cpu *j_z_cpu = &per_cpu(zenith_cpu, j);
			unsigned long j_util, j_max;

			j_util = zenith_get_util(j_z_cpu);
			j_max = j_z_cpu->max_capacity;
			j_util = zenith_iowait_apply(j_z_cpu, time, j_util, j_max);
			
			zenith_update_busy_pct(j_z_cpu, tunables->hispeed_window_us, tunables->hispeed_filter_shift);
			j_util = zenith_blend_util(j_z_cpu, j_util, j_max, tunables->screen_state);

			if (j_util * max_cap > j_max * util) {
				util = j_util;
				max_cap = j_max;
			}
		}

		next_f = zenith_get_next_freq(z_policy, util, max_cap);
		zenith_execute_switch(z_policy, time, next_f);
	}

	raw_spin_unlock(&z_policy->update_lock);
}

/************************ Kthread Slow Path ***********************/

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

/************************** Sysfs Interface & Tunables ************************/

static struct zenith_tunables *global_tunables;
static DEFINE_MUTEX(global_tunables_lock);
static DEFINE_MUTEX(min_rate_lock);

static inline struct zenith_tunables *to_zenith_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct zenith_tunables, attr_set);
}

static void update_min_rate_limit_ns(struct zenith_policy *z_policy)
{
	mutex_lock(&min_rate_lock);
	z_policy->min_rate_limit_ns = min(z_policy->up_rate_delay_ns, z_policy->down_rate_delay_ns);
	mutex_unlock(&min_rate_lock);
}

#define ZENITH_TUNABLE_UINT(_name) \
static ssize_t _name##_show(struct gov_attr_set *attr_set, char *buf) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	return sprintf(buf, "%u\n", t->_name); \
} \
static ssize_t _name##_store(struct gov_attr_set *attr_set, const char *buf, size_t count) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	unsigned int val; \
	if (kstrtouint(buf, 10, &val)) return -EINVAL; \
	t->_name = val; \
	return count; \
} \
static struct governor_attr _name = __ATTR_RW(_name)

ZENITH_TUNABLE_UINT(hispeed_window_us);
ZENITH_TUNABLE_UINT(io_is_busy);
ZENITH_TUNABLE_UINT(screen_state);
ZENITH_TUNABLE_UINT(thermal_state);

static ssize_t up_threshold_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->up_threshold);
}

static ssize_t up_threshold_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val == 0 || val > 100)
		return -EINVAL;
	t->up_threshold = val;
	return count;
}
static struct governor_attr up_threshold = __ATTR_RW(up_threshold);

static ssize_t hispeed_filter_shift_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->hispeed_filter_shift);
}

static ssize_t hispeed_filter_shift_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val >= 32)
		return -EINVAL;
	t->hispeed_filter_shift = val;
	return count;
}
static struct governor_attr hispeed_filter_shift =
	__ATTR_RW(hispeed_filter_shift);

static ssize_t powersave_bias_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->powersave_bias);
}

static ssize_t powersave_bias_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1000)
		return -EINVAL;
	t->powersave_bias = val;
	return count;
}
static struct governor_attr powersave_bias = __ATTR_RW(powersave_bias);

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->up_rate_limit_us);
}

static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_pol;
	unsigned int val;

	if (kstrtouint(buf, 10, &val)) return -EINVAL;
	t->up_rate_limit_us = val;

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		z_pol->up_rate_delay_ns = val * NSEC_PER_USEC;
		update_min_rate_limit_ns(z_pol);
	}
	return count;
}
static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->down_rate_limit_us);
}

static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_pol;
	unsigned int val;

	if (kstrtouint(buf, 10, &val)) return -EINVAL;
	t->down_rate_limit_us = val;

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		z_pol->down_rate_delay_ns = val * NSEC_PER_USEC;
		update_min_rate_limit_ns(z_pol);
	}
	return count;
}
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);

static struct attribute *zenith_attrs[] = {
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&up_threshold.attr,
	&hispeed_window_us.attr,
	&hispeed_filter_shift.attr,
	&powersave_bias.attr,
	&io_is_busy.attr,
	&screen_state.attr,
	&thermal_state.attr,
	NULL
};
ATTRIBUTE_GROUPS(zenith);

static void zenith_tunables_free(struct kobject *kobj)
{
	kfree(to_zenith_tunables(container_of(kobj, struct gov_attr_set, kobj)));
}

static struct kobj_type zenith_tunables_ktype = {
	.default_groups = zenith_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = &zenith_tunables_free,
};

/********************** Lifecycle & Registration *********************/

static int zenith_kthread_create(struct zenith_policy *z_policy)
{
	struct task_struct *thread;
	struct sched_attr attr = {
		.size		= sizeof(struct sched_attr),
		.sched_policy	= SCHED_DEADLINE,
		.sched_flags	= SCHED_FLAG_SUGOV,
		.sched_nice	= 0,
		.sched_priority	= 0,
		.sched_runtime	=  1000000,
		.sched_deadline = 10000000,
		.sched_period	= 10000000,
	};
	int ret;

	if (z_policy->policy->fast_switch_enabled)
		return 0;

	kthread_init_work(&z_policy->work, zenith_work);
	kthread_init_worker(&z_policy->worker);
	thread = kthread_create(kthread_worker_fn, &z_policy->worker, "zenith:%d", cpumask_first(z_policy->policy->related_cpus));
	if (IS_ERR(thread)) return PTR_ERR(thread);

	ret = sched_setattr_nocheck(thread, &attr);
	if (ret) {
		kthread_stop(thread);
		return ret;
	}

	z_policy->thread = thread;
	if (!z_policy->policy->dvfs_possible_from_any_cpu)
		kthread_bind_mask(thread, z_policy->policy->related_cpus);

	init_irq_work(&z_policy->irq_work, zenith_irq_work);
	mutex_init(&z_policy->work_lock);
	wake_up_process(thread);

	return 0;
}

static int zenith_init(struct cpufreq_policy *policy)
{
	struct zenith_policy *z_policy;
	struct zenith_tunables *tunables;
	int ret;

	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	z_policy = kzalloc(sizeof(*z_policy), GFP_KERNEL);
	if (!z_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	z_policy->policy = policy;
	raw_spin_lock_init(&z_policy->update_lock);

	ret = zenith_kthread_create(z_policy);
	if (ret)
		goto free_z_policy;

	mutex_lock(&global_tunables_lock);

	if (global_tunables) {
		tunables = global_tunables;
		gov_attr_set_get(&tunables->attr_set, &z_policy->tunables_hook);
		goto out;
	}

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (!tunables) {
		ret = -ENOMEM;
		goto unlock;
	}

	gov_attr_set_init(&tunables->attr_set, &z_policy->tunables_hook);

	tunables->up_rate_limit_us	= ZENITH_DEFAULT_UP_RATE_LIMIT_US;
	tunables->down_rate_limit_us	= ZENITH_DEFAULT_DOWN_RATE_LIMIT_US;
	tunables->up_threshold		= ZENITH_DEFAULT_UP_THRESHOLD;
	tunables->hispeed_window_us	= ZENITH_DEFAULT_HISPEED_WINDOW_US;
	tunables->hispeed_filter_shift	= ZENITH_DEFAULT_HISPEED_FILTER_SHIFT;
	tunables->powersave_bias	= ZENITH_DEFAULT_POWERSAVE_BIAS;
	tunables->io_is_busy		= ZENITH_DEFAULT_IO_IS_BUSY;
	tunables->screen_state		= 1;
	tunables->thermal_state		= 0;

	ret = kobject_init_and_add(&tunables->attr_set.kobj,
				   &zenith_tunables_ktype,
				   get_governor_parent_kobj(policy),
				   "zenith");
	if (ret) {
		kfree(tunables);
		goto unlock;
	}

	global_tunables = tunables;

out:
	mutex_unlock(&global_tunables_lock);
	z_policy->tunables = tunables;
	policy->governor_data = z_policy;
	return 0;

unlock:
	mutex_unlock(&global_tunables_lock);
	if (!policy->fast_switch_enabled && z_policy->thread) {
		kthread_stop(z_policy->thread);
		mutex_destroy(&z_policy->work_lock);
	}
free_z_policy:
	kfree(z_policy);
disable_fast_switch:
	cpufreq_disable_fast_switch(policy);
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

	z_policy->up_rate_delay_ns = z_policy->tunables->up_rate_limit_us * NSEC_PER_USEC;
	z_policy->down_rate_delay_ns = z_policy->tunables->down_rate_limit_us * NSEC_PER_USEC;
	update_min_rate_limit_ns(z_policy);

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

static struct cpufreq_governor zenith_gov = {
	.name       = "zenith",
	.init       = zenith_init,
	.exit       = zenith_exit,
	.start      = zenith_start,
	.stop       = zenith_stop,
	.limits     = zenith_limits,
	.owner      = THIS_MODULE,
	.flags      = CPUFREQ_GOV_DYNAMIC_SWITCHING,
};

static int __init zenith_gov_init(void)
{
	pr_info("Zenith: V2 Dreadnought (EAS/EM/Display/Thermal) Initialized. By ENI for LO.\n");
	return cpufreq_register_governor(&zenith_gov);
}
fs_initcall(zenith_gov_init);