// SPDX-License-Identifier: GPL-2.0
/*
 * Zenith CPUFreq Governor V2 (EAS/EM/Thermal/Display Hybrid)
 * Developed by ENI exclusively for LO.
 *
 * Architecture Additions:
 * 1. Energy Model (EM) Awareness: Reads mW costs from the device tree to prevent 
 * inefficient frequency spikes during thermal throttling.
 * 2. Display-State Awareness: `screen_state` sysfs hook forces deep-sleep
 * biases (raised up_threshold, powersave_bias) when the display is off.
 * 3. Dynamic Thermal Thresholding: `thermal_state` sysfs hook dynamically 
 * relaxes up_thresholds to let silicon breathe.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "sched.h"

#include <linux/cpufreq.h>
#include <linux/sched/cpufreq.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/irq_work.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/energy_model.h>
#include <linux/input.h>
#include <linux/atomic.h>
#include <linux/ktime.h>
#include <linux/notifier.h>
#include <linux/kernel_stat.h>

/* linux/fb.h transitively pulls linux/acpi.h, which redefines the
 * ACPI_PROBE_TABLE macro already defined by sched.h ->
 * asm-generic/vmlinux.lds.h. The linker-section variant from
 * vmlinux.lds.h is only meaningful in the kernel link stage and
 * unused in this translation unit, so undefining it before the
 * fb.h chain lets the acpi.h definition win without a warning.
 * Guarding the include on CONFIG_FB_NOTIFY also lets panels that
 * use a non-fb notifier framework skip the dependency entirely.
 */
#ifdef CONFIG_FB_NOTIFY
#undef ACPI_PROBE_TABLE
#undef ACPI_PROBE_TABLE_END
#include <linux/fb.h>
#endif
#include <trace/events/power.h>

#define CREATE_TRACE_POINTS
#include <trace/events/cpufreq_zenith.h>

/* Constants & Defaults */
/* Permille of SCHED_CAPACITY_SCALE at which iowait boost starts.
 * 125 == SCHED_CAPACITY_SCALE / 8, preserving the historical default.
 */
#define ZENITH_DEFAULT_IOWAIT_BOOST_MIN		125
#define ZENITH_DEFAULT_UP_THRESHOLD		80
#define ZENITH_DEFAULT_UP_THRESHOLD_HISPEED	0	/* disabled */
#define ZENITH_DEFAULT_DOWN_THRESHOLD		60
#define ZENITH_DEFAULT_HISPEED_FREQ		0	/* disabled */
#define ZENITH_DEFAULT_HISPEED_LOAD		90
#define ZENITH_EFF_BINS_MAX			4
#define ZENITH_CLIMB_MODE_SNAP			0	/* default */
#define ZENITH_CLIMB_MODE_STEP			1
#define ZENITH_PROFILE_CUSTOM			0	/* default */
#define ZENITH_PROFILE_PERFORMANCE		1
#define ZENITH_PROFILE_BALANCED			2
#define ZENITH_PROFILE_BATTERY			3
#define ZENITH_PROFILE_LEGACY			4
#define ZENITH_DEFAULT_CLIMB_MODE		ZENITH_CLIMB_MODE_SNAP
#define ZENITH_DEFAULT_FREQ_STEP_PCT		5
#define ZENITH_DEFAULT_THERMAL_AUTO		0
#define ZENITH_THERMAL_AUTO_PRESSURE_PCT	10
#define ZENITH_DEFAULT_UP_RATE_LIMIT_US		500
#define ZENITH_DEFAULT_DOWN_RATE_LIMIT_US	2000
#define ZENITH_DEFAULT_POWERSAVE_BIAS		0
#define ZENITH_DEFAULT_IO_IS_BUSY		1
#define ZENITH_DEFAULT_INPUT_BOOST_MS		80
#define ZENITH_DEFAULT_EFFICIENT_FREQ		0
#define ZENITH_DEFAULT_UP_DELAY_US		4000
#define ZENITH_DEFAULT_LIGHT_LOAD_FREQ		0
#define ZENITH_DEFAULT_LIGHT_LOAD_THRESHOLD	20
#define ZENITH_DEFAULT_SAMPLING_DOWN_FACTOR	1
#define ZENITH_MAX_SAMPLING_DOWN_FACTOR		10
#define ZENITH_DEFAULT_BIAS_LOAD_THRESHOLD	50

/* auto_tune classifier thresholds. Exposed as tunables so userspace can
 * tune what the observer considers "saturated" and the saturation /
 * input-event cutoffs that select performance vs battery.
 */
#define ZENITH_DEFAULT_AT_SAT_LOAD_PCT		70
#define ZENITH_DEFAULT_AT_HI_SAT_PCT		60
#define ZENITH_DEFAULT_AT_LO_SAT_PCT		10
#define ZENITH_DEFAULT_AT_HI_EVENTS_X2		4
#define ZENITH_DEFAULT_AT_LO_EVENTS_X2		1

/*
 * Zenith Tunables & State API
 */
struct zenith_tunables {
	struct gov_attr_set	attr_set;
	unsigned int		up_rate_limit_us;
	unsigned int		down_rate_limit_us;
	unsigned int		up_threshold;
	unsigned int		down_threshold;	/* hysteresis lower bound */

	/* Hispeed floor tier: when load >= hispeed_load (% of max_cap),
	 * ensure the chosen target_freq is at least hispeed_freq (kHz).
	 * hispeed_freq=0 disables the tier.
	 */
	unsigned int		hispeed_freq;
	unsigned int		hispeed_load;

	/* Secondary up_threshold applied only when policy->cur has
	 * already climbed to hispeed_freq or above. 0 disables the
	 * substitution and falls back to up_threshold at every bin.
	 *
	 * The common shape is up_threshold=70 to get from idle to
	 * hispeed_freq aggressively, with up_threshold_hispeed=90 to
	 * demand a clearly heavier workload before committing to
	 * policy->max. Requires hispeed_freq != 0.
	 */
	unsigned int		up_threshold_hispeed;

	/* Alternative climb mechanism when load crosses up_threshold.
	 * SNAP (0) pins policy->max (the original ondemand-style
	 * behaviour). STEP (1) bumps target_freq by freq_step_pct
	 * percent of policy->max per sample, giving a slower,
	 * conservative-style climb. STEP mode bypasses the
	 * brutal_active / down_threshold hysteresis.
	 */
	unsigned int		climb_mode;
	unsigned int		freq_step_pct;

	/* Last-applied preset, or CUSTOM if one was never written. The
	 * tunable does NOT auto-revert to CUSTOM when individual fields
	 * are later modified — the user can always check sysfs to see
	 * which recipe they last applied.
	 */
	unsigned int		active_profile;

	/* Permille (0..1000) of SCHED_CAPACITY_SCALE at which
	 * zenith_iowait_boost() arms and below which a doubling
	 * decay exits the boost. 125 (12.5%) matches the legacy
	 * SCHED_CAPACITY_SCALE / 8 constant. 0 disables the minimum
	 * floor but still allows the doubling climb to take effect
	 * from its first sample; use io_is_busy=0 to disable iowait
	 * boost wholesale instead.
	 */
	unsigned int		iowait_boost_min;

	/* When 1, a per-policy delayed_work periodically classifies the
	 * recent workload from load-saturation rate and input-event
	 * rate, and auto-selects performance / balanced / battery via
	 * zenith_apply_profile(). Default 0 (off).
	 */
	unsigned int		auto_tune;
	unsigned int		powersave_bias;
	unsigned int		io_is_busy;

	/* When 1, dampen the brutality-path load_pct by the fraction
	 * of recent CPU time spent in niced-user mode. Approximation
	 * of ondemand's ignore_nice_load for a PELT-based governor.
	 */
	unsigned int		ignore_nice_load;
	
	/* Zenith Environment API */
	unsigned int		screen_state;   /* 1 = ON, 0 = OFF */

	/* When 1, zenith subscribes to the fb notifier chain and updates
	 * screen_state automatically on FB_EVENT_BLANK. screen_state
	 * written from userspace still takes effect and is only
	 * overridden on the next blank/unblank event.
	 */
	unsigned int		screen_auto;

	unsigned int		thermal_state;  /* 0 = COOL, 1 = THROTTLING */

	/* When 1, thermal_state is additionally inferred from
	 * arch_scale_thermal_pressure() on every update_util tick.
	 * thermal_state=1 written from userspace still forces it.
	 */
	unsigned int		thermal_auto;

	/* Input boost duration (ms). 0 = disabled. */
	unsigned int		input_boost_ms;

	/* Efficient-frequency soft-cap ladder, up to ZENITH_EFF_BINS_MAX
	 * entries. Sorted ascending by frequency. The up_delay_us array
	 * is paired 1:1 with eff_freq; writing a single scalar to
	 * up_delay_us broadcasts it to every bin.
	 *
	 * eff_nr == 0 disables the ladder entirely (equivalent to the
	 * pre-ladder "efficient_freq=0" state). The scalar shadows
	 * efficient_freq / up_delay_us remain for backward-compatible
	 * sysfs reads: efficient_freq = eff_freq[0], up_delay_us =
	 * up_delay[0].
	 */
	unsigned int		efficient_freq;
	unsigned int		up_delay_us;
	unsigned int		eff_nr;
	unsigned int		eff_freq[ZENITH_EFF_BINS_MAX];
	unsigned int		eff_delay_us[ZENITH_EFF_BINS_MAX];

	/* Light-load hard cap. light_load_freq=0 disables. */
	unsigned int		light_load_freq;
	unsigned int		light_load_threshold;	/* in % of max_cap */

	/* Hold-at-max multiplier for down_rate_limit. 1 = disabled. */
	unsigned int		sampling_down_factor;

	/* powersave_bias only applies below this load (% of max_cap).
	 * 100 = always apply (legacy behaviour); 0 = never apply.
	 */
	unsigned int		bias_load_threshold;

	/* auto_tune observer thresholds (see per-field comments at the
	 * ZENITH_DEFAULT_AT_* macros). All are in percent except the
	 * events_x2 pair, which are integer event counts per 2 seconds
	 * over the classification window.
	 */
	unsigned int		auto_tune_sat_load_pct;
	unsigned int		auto_tune_hi_sat_pct;
	unsigned int		auto_tune_lo_sat_pct;
	unsigned int		auto_tune_hi_events_x2;
	unsigned int		auto_tune_lo_events_x2;
};

/*
 * Set by the input handler on every key/abs event. Read from the hot path
 * with atomic64_read so no governor lock is needed in the producer.
 */
static atomic64_t zenith_input_boost_until_ns = ATOMIC64_INIT(0);
static unsigned int zenith_input_boost_active_ms = ZENITH_DEFAULT_INPUT_BOOST_MS;

/* Monotonically-increasing global count of qualifying input events seen
 * by zenith_input_event. Auto-tune workers sample this periodically and
 * subtract their last-observed value to get an events-per-window rate.
 */
static atomic64_t zenith_auto_input_events = ATOMIC64_INIT(0);
#define ZENITH_AUTO_TUNE_PERIOD_MS	10000	/* classify every 10s  */
#define ZENITH_AUTO_TUNE_SAT_LOAD	70	/* load_pct >= 70 = saturated */
#define ZENITH_AUTO_TUNE_HI_SAT_PCT	60	/* gaming trigger */
#define ZENITH_AUTO_TUNE_LO_SAT_PCT	10	/* idle trigger */
#define ZENITH_AUTO_TUNE_HI_EVENTS_X2	(2 * 2)	/* > 2.0 events/s in 2s units */
#define ZENITH_AUTO_TUNE_LO_EVENTS_X2	1	/* < 0.5 events/s  */

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

	/* Per-bin unlock deadlines for the multi-step efficient_freq
	 * ladder. eff_unlock_at_ns[i] is the ktime_get_ns() time after
	 * which target_freq is allowed past tunables->eff_freq[i].
	 * Zero means "idle" — not currently gating. Indexed 0..eff_nr-1.
	 */
	u64			eff_unlock_at_ns[ZENITH_EFF_BINS_MAX];

	/* Multiplier currently applied to down_rate_delay_ns. Bumped to
	 * tunables->sampling_down_factor while sitting at policy->max,
	 * reset to 1 the moment we leave max. Mirrors ondemand.
	 */
	unsigned int		down_rate_mult;

	/* True once load crossed up_threshold. Stays true, holding us at
	 * policy->max, until load drops below down_threshold. Collapses
	 * to old snap-on-every-sample behaviour when
	 * down_threshold >= up_threshold.
	 */
	bool			brutal_active;

	/* Cached nice-load ratio for the policy (0..100). Sampled in
	 * the update_util hook when ignore_nice_load=1 and read from
	 * zenith_get_next_freq. Guarded by update_lock in the shared
	 * path; on the single path only one CPU writes it.
	 */
	unsigned int		nice_pct;

	/* Auto-tune observer state. Counters are incremented in the
	 * hot path when tunables->auto_tune=1; the delayed_work handler
	 * samples and resets them every ZENITH_AUTO_TUNE_PERIOD_MS and
	 * chooses a preset. Protected by update_lock (shared path) or
	 * single-writer (single path).
	 */
	unsigned int		at_samples_total;
	unsigned int		at_samples_saturated;
	u64			at_last_events;
	struct delayed_work	at_work;
};

struct zenith_cpu {
	struct update_util_data	update_util;
	struct zenith_policy	*z_policy;
	unsigned int		cpu;

	bool			iowait_boost_pending;
	unsigned int		iowait_boost;
	u64			last_update;
	unsigned long		bw_dl;

	/* Delta tracking for ignore_nice_load. Only read when the
	 * corresponding tunable is set.
	 */
	u64			prev_nice_time;
	u64			prev_nice_wall;
	unsigned long		max_capacity;
};

static DEFINE_PER_CPU(struct zenith_cpu, zenith_cpu);

/************************ Schedutil: I/O Wait & DL Logic ***********************/

/* Resolve the configured iowait floor for the policy owning z_cpu, in
 * absolute SCHED_CAPACITY_SCALE units. Called from every iowait path so
 * kept inline and trivial.
 */
static inline unsigned int zenith_iowait_floor(struct zenith_cpu *z_cpu)
{
	unsigned int permille = z_cpu->z_policy->tunables->iowait_boost_min;
	return (SCHED_CAPACITY_SCALE * permille) / 1000;
}

static bool zenith_iowait_reset(struct zenith_cpu *z_cpu, u64 time, bool set_iowait_boost)
{
	s64 delta_ns = time - z_cpu->last_update;
	if (delta_ns <= TICK_NSEC)
		return false;

	z_cpu->iowait_boost = set_iowait_boost ? zenith_iowait_floor(z_cpu) : 0;
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
	z_cpu->iowait_boost = zenith_iowait_floor(z_cpu);
}

static unsigned long zenith_iowait_apply(struct zenith_cpu *z_cpu, u64 time, unsigned long util, unsigned long max_cap)
{
	unsigned long boost;
	unsigned int floor = zenith_iowait_floor(z_cpu);

	if (!z_cpu->iowait_boost)
		return util;
	if (zenith_iowait_reset(z_cpu, time, false))
		return util;

	if (!z_cpu->iowait_boost_pending) {
		z_cpu->iowait_boost >>= 1;
		if (z_cpu->iowait_boost < floor) {
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

static unsigned long zenith_get_util(struct zenith_cpu *z_cpu)
{
	struct rq *rq = cpu_rq(z_cpu->cpu);
	unsigned long util = cpu_util_cfs(rq);
	unsigned long max = arch_scale_cpu_capacity(z_cpu->cpu);

	z_cpu->max_capacity = max;
	z_cpu->bw_dl = cpu_bw_dl(rq);

	return schedutil_cpu_util(z_cpu->cpu, util, max, FREQUENCY_UTIL, NULL);
}

/************************ Nice-load sampling *********************************/

/* Return the fraction (0..100) of wall time since the last call that
 * this CPU spent in niced-user mode. Uses the scheduler's cputime
 * accounting (same source as /proc/stat). Only called when
 * ignore_nice_load=1 so the cost is paid lazily.
 */
static unsigned int zenith_sample_nice_pct(struct zenith_cpu *z_cpu, u64 now)
{
	struct kernel_cpustat *kcs = &kcpustat_cpu(z_cpu->cpu);
	u64 cur_nice = kcpustat_field(kcs, CPUTIME_NICE, z_cpu->cpu);
	u64 nice_delta, wall_delta;
	unsigned int pct;

	if (unlikely(!z_cpu->prev_nice_wall)) {
		z_cpu->prev_nice_time = cur_nice;
		z_cpu->prev_nice_wall = now;
		return 0;
	}

	nice_delta = cur_nice - z_cpu->prev_nice_time;
	wall_delta = now - z_cpu->prev_nice_wall;
	z_cpu->prev_nice_time = cur_nice;
	z_cpu->prev_nice_wall = now;

	if (!wall_delta)
		return 0;
	if (nice_delta > wall_delta)
		nice_delta = wall_delta;

	pct = (unsigned int)((nice_delta * 100) / wall_delta);
	return pct > 100 ? 100 : pct;
}

/************************ Thermal State Resolution ***************************/

/* Return true when zenith should behave as if thermally throttled.
 *
 * Userspace-written thermal_state=1 always wins. When thermal_auto=1 is
 * set, we additionally consult arch_scale_thermal_pressure() on the
 * first CPU of the policy and consider the policy throttled when
 * pressure has eaten at least ZENITH_THERMAL_AUTO_PRESSURE_PCT of the
 * capacity. This lets the governor respond to the kernel thermal
 * framework (via arch_update_thermal_pressure) with no userspace in
 * the loop.
 */
static bool zenith_thermal_active(struct zenith_policy *z_policy)
{
	struct cpufreq_policy *policy = z_policy->policy;
	struct zenith_tunables *tunables = z_policy->tunables;
	unsigned long pressure, cap;
	int cpu;

	if (tunables->thermal_state)
		return true;
	if (!tunables->thermal_auto)
		return false;

	cpu = cpumask_first(policy->cpus);
	if (cpu >= nr_cpu_ids)
		return false;

	cap = arch_scale_cpu_capacity(cpu);
	if (!cap)
		return false;

	pressure = arch_scale_thermal_pressure(cpu);
	return (pressure * 100 / cap) >= ZENITH_THERMAL_AUTO_PRESSURE_PCT;
}

/************************ Energy Model (EM) Evaluation ***********************/

static unsigned int zenith_em_cap_freq(struct zenith_policy *z_policy, unsigned int target_freq)
{
	struct cpufreq_policy *policy = z_policy->policy;
	struct em_perf_domain *pd = em_cpu_get(policy->cpu);
	struct em_perf_state *ps;
	int i;

	/* If no Energy Model is registered or we aren't thermal throttling, skip */
	if (!pd || !zenith_thermal_active(z_policy))
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
	s64 down_delay = z_policy->down_rate_delay_ns *
			 (s64)max(z_policy->down_rate_mult, 1U);

	if (next_freq > z_policy->next_freq && delta_ns < z_policy->up_rate_delay_ns)
		return true;

	if (next_freq < z_policy->next_freq && delta_ns < down_delay)
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
	/* Tracepoint breadcrumb: updated at each decision branch. Read
	 * once at the end of the function when the event is enabled.
	 */
	const char *tp_path = "eas";
	unsigned int tp_load_pct = 0;

	/* Dynamic Environment Overrides */
	unsigned int dynamic_up_thresh = z_policy->tunables->up_threshold;
	unsigned int dynamic_bias = z_policy->tunables->powersave_bias;

	if (z_policy->tunables->screen_state == 0) {
		dynamic_up_thresh = 95; /* Hard to wake up */
		dynamic_bias = 500;     /* 50% penalty */
		z_policy->brutal_active = false; /* no hysteresis screen-off */
	} else if (zenith_thermal_active(z_policy)) {
		dynamic_up_thresh = 90; /* Relaxed for thermals */
	} else if (z_policy->tunables->up_threshold_hispeed &&
		   z_policy->tunables->hispeed_freq &&
		   policy->cur >= z_policy->tunables->hispeed_freq) {
		/* Above the hispeed floor, require the stiffer
		 * threshold before escalating all the way to
		 * policy->max. Screen-off and thermal overrides take
		 * precedence because both already pin an even
		 * higher value.
		 */
		dynamic_up_thresh = z_policy->tunables->up_threshold_hispeed;
	}

	if (max_cap)
		tp_load_pct = (unsigned int)((util * 100) / max_cap);

	/* 0. Input Boost — pin to policy->max for input_boost_ms after a key
	 * or touch event. Gated by screen_state so we don't wake clusters
	 * while the display is off.
	 */
	if (z_policy->tunables->input_boost_ms &&
	    z_policy->tunables->screen_state &&
	    ktime_get_ns() < (u64)atomic64_read(&zenith_input_boost_until_ns)) {
		freq = policy->max;
		tp_path = "input_boost";
		goto resolve;
	}

	/* 1. Ondemand Brutality (with hysteresis).
	 *
	 * Snap to policy->max when load crosses up_threshold and stay there
	 * while load remains above down_threshold. This creates a band
	 * around the transition so we do not ping-pong between policy->max
	 * and the bin just below it on every tick. Clearing brutal_active
	 * falls through to the EAS proportional path.
	 */
	if (max_cap) {
		unsigned int load_pct = (util * 100) / max_cap;

		if (z_policy->tunables->auto_tune) {
			z_policy->at_samples_total++;
			if (load_pct >= z_policy->tunables->auto_tune_sat_load_pct)
				z_policy->at_samples_saturated++;
		}

		/* ignore_nice_load: dampen the load percentage by the
		 * fraction of wall time recently spent in niced-user
		 * mode, so background niced work does not trigger
		 * snap-to-max. Approximate — PELT-based util already
		 * weighs niced tasks by their scheduler weight.
		 */
		if (z_policy->tunables->ignore_nice_load && z_policy->nice_pct)
			load_pct = load_pct * (100 - z_policy->nice_pct) / 100;

		if (load_pct >= dynamic_up_thresh) {
			if (z_policy->tunables->climb_mode ==
			    ZENITH_CLIMB_MODE_STEP) {
				/* Gentle climb: step by freq_step_pct of
				 * policy->max from the current bin.
				 * Bypasses hysteresis entirely.
				 */
				unsigned int step =
				    (policy->max *
				     z_policy->tunables->freq_step_pct) / 100;
				if (!step)
					step = 1;
				freq = policy->cur + step;
				if (freq > policy->max)
					freq = policy->max;
				z_policy->brutal_active = false;
				tp_path = "climb_step";
				goto resolve;
			}
			z_policy->brutal_active = true;
			freq = policy->max;
			tp_path = "snap_max";
			goto resolve;
		}

		if (z_policy->tunables->climb_mode == ZENITH_CLIMB_MODE_SNAP &&
		    z_policy->brutal_active &&
		    load_pct >= z_policy->tunables->down_threshold) {
			freq = policy->max;
			tp_path = "brutal_hold";
			goto resolve;
		}

		z_policy->brutal_active = false;
	}

	/* 2. Schedutil EAS Proportional Math with Headroom */
	if (arch_scale_freq_invariant())
		freq = policy->cpuinfo.max_freq;
	else
		freq = policy->cur + (policy->cur >> 2); 

	freq = map_util_freq(util, freq, max_cap);

	/* 2b. Hispeed floor — intermediate snap tier.
	 *
	 * When load has crossed hispeed_load but is still below
	 * up_threshold, we are in an "active but not saturated" regime.
	 * Rather than letting proportional math pick a freq based on a
	 * noisy PELT signal, floor the target at hispeed_freq so the
	 * cluster is at least at its "fast but efficient" bin. Above
	 * up_threshold we already went straight to policy->max in
	 * step 1, so this tier never competes with brutality.
	 *
	 * hispeed_freq=0 disables the tier.
	 */
	if (z_policy->tunables->hispeed_freq && max_cap) {
		unsigned int load_pct = (util * 100) / max_cap;
		if (load_pct >= z_policy->tunables->hispeed_load &&
		    freq < z_policy->tunables->hispeed_freq) {
			freq = z_policy->tunables->hispeed_freq;
			tp_path = "hispeed";
		}
	}

	/* 3. Powersave Bias.
	 *
	 * Only apply when current util is below bias_load_threshold so
	 * heavy work is not penalised. A threshold of 100 keeps the legacy
	 * "always-bias" behaviour; 0 disables the bias entirely without
	 * having to also write powersave_bias=0.
	 */
	if (dynamic_bias && max_cap &&
	    (util * 100) / max_cap < z_policy->tunables->bias_load_threshold) {
		margin = freq * dynamic_bias / 1000;
		freq = freq - margin;
	}

resolve:
	if (freq == z_policy->cached_raw_freq && !z_policy->need_freq_update)
		return z_policy->next_freq;

	z_policy->cached_raw_freq = freq;
	target_freq = cpufreq_driver_resolve_freq(policy, freq);

	/* 4. Efficient-frequency ladder (soft cap).
	 *
	 * The ladder is an array of (efficient_freq, up_delay_us) pairs
	 * sorted ascending by freq. For each bin i that the requested
	 * target_freq wants to cross, the governor holds at eff_freq[i]
	 * until the request has been sustained for eff_delay_us[i]. If
	 * target drops back below eff_freq[i] the corresponding
	 * deadline is cleared, so transient bursts do not accumulate
	 * climbing progress.
	 *
	 * eff_nr == 0 disables the ladder (identical to pre-ladder
	 * efficient_freq=0).
	 */
	if (z_policy->tunables->eff_nr) {
		unsigned int nr = z_policy->tunables->eff_nr;
		u64 now = ktime_get_ns();
		int i;

		if (nr > ZENITH_EFF_BINS_MAX)
			nr = ZENITH_EFF_BINS_MAX;

		for (i = 0; i < nr; i++) {
			unsigned int bin_freq = z_policy->tunables->eff_freq[i];
			u64 delay_ns = (u64)z_policy->tunables->eff_delay_us[i] *
				       NSEC_PER_USEC;

			if (target_freq <= bin_freq) {
				/* Target is at or below this bin. Reset
				 * its own and every higher bin's
				 * deadline so a later climb has to earn
				 * them again.
				 */
				int j;
				for (j = i; j < nr; j++)
					z_policy->eff_unlock_at_ns[j] = 0;
				break;
			}

			/* Target wants to cross bin i. Arm its deadline
			 * on first sight, hold at bin_freq until the
			 * sustained time expires.
			 */
			if (!z_policy->eff_unlock_at_ns[i]) {
				z_policy->eff_unlock_at_ns[i] = now + delay_ns;
				target_freq = bin_freq;
				break;
			}
			if (now < z_policy->eff_unlock_at_ns[i]) {
				target_freq = bin_freq;
				break;
			}
			/* Bin already unlocked; try the next one. */
		}
	}

	/* 5. Light-load hard cap.
	 *
	 * Independent of the efficient_freq soft cap (which gates climbs):
	 * when current util is below light_load_threshold, hard-clamp the
	 * resolved target_freq down to light_load_freq. Saves power on
	 * idle-ish workloads (background sync, screen-on hold) where PELT
	 * jitter would otherwise push us into a mid bin we do not need.
	 */
	if (z_policy->tunables->light_load_freq && max_cap &&
	    (util * 100) / max_cap < z_policy->tunables->light_load_threshold &&
	    target_freq > z_policy->tunables->light_load_freq) {
		target_freq = z_policy->tunables->light_load_freq;
		tp_path = "light_cap";
	}

	/* 6. Energy Model Validation */
	{
		unsigned int em_in = target_freq;
		target_freq = zenith_em_cap_freq(z_policy, target_freq);
		if (target_freq != em_in)
			tp_path = "em_cap";
	}

	/* 7. Sampling-down multiplier: while we are sitting at policy->max,
	 * extend the down-rate delay by sampling_down_factor so we do not
	 * ping-pong off the peak bin. Reset the moment we step away.
	 */
	if (target_freq >= policy->max)
		z_policy->down_rate_mult =
			max(z_policy->tunables->sampling_down_factor, 1U);
	else
		z_policy->down_rate_mult = 1;

	if (trace_zenith_decision_enabled())
		trace_zenith_decision(policy->cpu, tp_path, util, max_cap,
				      tp_load_pct, freq, target_freq);

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

	z_policy->nice_pct = tunables->ignore_nice_load ?
		zenith_sample_nice_pct(z_cpu, time) : 0;

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
		
		unsigned int nice_pct_max = 0;

		for_each_cpu(j, z_policy->policy->cpus) {
			struct zenith_cpu *j_z_cpu = &per_cpu(zenith_cpu, j);
			unsigned long j_util, j_max;

			j_util = zenith_get_util(j_z_cpu);
			j_max = j_z_cpu->max_capacity;
			j_util = zenith_iowait_apply(j_z_cpu, time, j_util, j_max);

			if (tunables->ignore_nice_load) {
				unsigned int p = zenith_sample_nice_pct(j_z_cpu, time);
				if (p > nice_pct_max)
					nice_pct_max = p;
			}

			if (j_util * max_cap > j_max * util) {
				util = j_util;
				max_cap = j_max;
			}
		}

		z_policy->nice_pct = tunables->ignore_nice_load ? nice_pct_max : 0;

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

/* Force the next zenith_get_next_freq() call on every policy sharing
 * this tunables set to recompute from scratch, bypassing the
 * cached_raw_freq shortcut. Call this from any sysfs _store that
 * changes a value which feeds into the freq decision, so the user's
 * write takes effect on the very next scheduler tick rather than
 * waiting for pre-resolve freq to drift. Must be called with the
 * attr_set->update_lock held (governor_store always takes it).
 */
static void zenith_invalidate_cache(struct gov_attr_set *attr_set)
{
	struct zenith_policy *z_pol;

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook)
		z_pol->need_freq_update = true;
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

/* Same as ZENITH_TUNABLE_UINT but invalidates the per-policy freq
 * cache after the write so the new value takes effect on the next
 * scheduler tick. Use this for fields that feed into the
 * zenith_get_next_freq() decision.
 */
#define ZENITH_TUNABLE_UINT_INVAL(_name) \
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
	zenith_invalidate_cache(attr_set); \
	return count; \
} \
static struct governor_attr _name = __ATTR_RW(_name)

ZENITH_TUNABLE_UINT_INVAL(io_is_busy);

static ssize_t iowait_boost_min_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->iowait_boost_min);
}

static ssize_t iowait_boost_min_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	/* 0..1000 permille of SCHED_CAPACITY_SCALE. >1000 would overshoot
	 * capacity on the first arm, which is never what we want.
	 */
	if (kstrtouint(buf, 10, &val) || val > 1000)
		return -EINVAL;
	t->iowait_boost_min = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr iowait_boost_min = __ATTR_RW(iowait_boost_min);

static ssize_t ignore_nice_load_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->ignore_nice_load);
}

static ssize_t ignore_nice_load_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->ignore_nice_load = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr ignore_nice_load = __ATTR_RW(ignore_nice_load);

/* Apply one of the preset recipes to all tunables in-place. Leaves
 * light_load_freq, hispeed_freq, efficient_freq ladder and other
 * device-specific frequencies untouched because their correct values
 * depend on the SoC's actual freq table. The user can layer those on
 * top after picking a profile.
 */
static void zenith_apply_profile(struct zenith_tunables *t, unsigned int prof)
{
	switch (prof) {
	case ZENITH_PROFILE_PERFORMANCE:
		t->up_rate_limit_us	= 200;
		t->down_rate_limit_us	= 4000;
		t->up_threshold		= 70;
		t->down_threshold	= 50;
		t->hispeed_load		= 80;
		t->climb_mode		= ZENITH_CLIMB_MODE_SNAP;
		t->freq_step_pct	= 10;
		t->powersave_bias	= 0;
		t->bias_load_threshold	= 50;
		t->ignore_nice_load	= 0;
		t->input_boost_ms	= 150;
		t->light_load_threshold	= 20;
		t->sampling_down_factor	= 4;
		t->thermal_auto		= 1;
		t->screen_auto		= 1;
		break;

	case ZENITH_PROFILE_BALANCED:
		t->up_rate_limit_us	= ZENITH_DEFAULT_UP_RATE_LIMIT_US;
		t->down_rate_limit_us	= ZENITH_DEFAULT_DOWN_RATE_LIMIT_US;
		t->up_threshold		= ZENITH_DEFAULT_UP_THRESHOLD;
		t->down_threshold	= ZENITH_DEFAULT_DOWN_THRESHOLD;
		t->hispeed_load		= ZENITH_DEFAULT_HISPEED_LOAD;
		t->climb_mode		= ZENITH_CLIMB_MODE_SNAP;
		t->freq_step_pct	= ZENITH_DEFAULT_FREQ_STEP_PCT;
		t->powersave_bias	= ZENITH_DEFAULT_POWERSAVE_BIAS;
		t->bias_load_threshold	= ZENITH_DEFAULT_BIAS_LOAD_THRESHOLD;
		t->ignore_nice_load	= 0;
		t->input_boost_ms	= ZENITH_DEFAULT_INPUT_BOOST_MS;
		t->light_load_threshold	= ZENITH_DEFAULT_LIGHT_LOAD_THRESHOLD;
		t->sampling_down_factor	= ZENITH_DEFAULT_SAMPLING_DOWN_FACTOR;
		t->thermal_auto		= ZENITH_DEFAULT_THERMAL_AUTO;
		t->screen_auto		= 0;
		break;

	case ZENITH_PROFILE_BATTERY:
		t->up_rate_limit_us	= 1000;
		t->down_rate_limit_us	= 1000;
		t->up_threshold		= 85;
		t->down_threshold	= 40;
		t->hispeed_load		= 95;
		t->climb_mode		= ZENITH_CLIMB_MODE_STEP;
		t->freq_step_pct	= 10;
		t->powersave_bias	= 100;	/* 10% */
		t->bias_load_threshold	= 40;
		t->ignore_nice_load	= 1;
		t->input_boost_ms	= 60;
		t->light_load_threshold	= 30;
		t->sampling_down_factor	= 1;
		t->thermal_auto		= 1;
		t->screen_auto		= 1;
		break;

	case ZENITH_PROFILE_LEGACY:
		/* Approximates cpufreq_ondemand: plain up_threshold with
		 * no hysteresis, no input boost, nice-load ignored.
		 */
		t->up_rate_limit_us	= 2000;
		t->down_rate_limit_us	= 4000;
		t->up_threshold		= 80;
		t->down_threshold	= 80;	/* collapses hysteresis */
		t->hispeed_load		= 90;
		t->climb_mode		= ZENITH_CLIMB_MODE_SNAP;
		t->freq_step_pct	= 5;
		t->powersave_bias	= 0;
		t->bias_load_threshold	= 50;
		t->ignore_nice_load	= 1;
		t->input_boost_ms	= 0;
		t->light_load_threshold	= 20;
		t->sampling_down_factor	= 1;
		t->thermal_auto		= 0;
		t->screen_auto		= 0;
		break;

	case ZENITH_PROFILE_CUSTOM:
	default:
		/* No mutation — leaving CUSTOM simply records intent. */
		return;
	}

	/* Mirror input_boost_ms to the governor-wide cache used by the
	 * input handler fast path.
	 */
	WRITE_ONCE(zenith_input_boost_active_ms, t->input_boost_ms);
}

/************************ Auto-tune observer *****************************/

/* Classify the workload seen since the last pass and pick a profile.
 * Runs from a delayed_work context, so it can hold the policy's
 * update_lock to sample the counters without racing the hot path.
 */
static void zenith_auto_tune_work(struct work_struct *w)
{
	struct zenith_policy *z_policy =
		container_of(to_delayed_work(w), struct zenith_policy, at_work);
	struct zenith_tunables *t = z_policy->tunables;
	unsigned int total, saturated, sat_pct;
	u64 events_now, events_delta;
	unsigned int events_rate_x2;
	unsigned int target;
	unsigned long flags;

	if (!t->auto_tune)
		return;	/* tunable turned off; stop the chain */

	raw_spin_lock_irqsave(&z_policy->update_lock, flags);
	total = z_policy->at_samples_total;
	saturated = z_policy->at_samples_saturated;
	z_policy->at_samples_total = 0;
	z_policy->at_samples_saturated = 0;
	raw_spin_unlock_irqrestore(&z_policy->update_lock, flags);

	events_now = atomic64_read(&zenith_auto_input_events);
	events_delta = events_now - z_policy->at_last_events;
	z_policy->at_last_events = events_now;

	sat_pct = total ? (saturated * 100 / total) : 0;
	/* events per 2s, i.e. half-events/s * 2, kept integer-friendly:
	 * ZENITH_AUTO_TUNE_HI_EVENTS_X2=4 corresponds to > 2.0/s, and
	 * ZENITH_AUTO_TUNE_LO_EVENTS_X2=1 corresponds to < 0.5/s, over
	 * the ZENITH_AUTO_TUNE_PERIOD_MS window (10s by default).
	 */
	events_rate_x2 = (unsigned int)((events_delta * 2000) /
					ZENITH_AUTO_TUNE_PERIOD_MS);

	if (sat_pct >= t->auto_tune_hi_sat_pct &&
	    events_rate_x2 >= t->auto_tune_hi_events_x2)
		target = ZENITH_PROFILE_PERFORMANCE;
	else if (sat_pct <= t->auto_tune_lo_sat_pct &&
		 events_rate_x2 <= t->auto_tune_lo_events_x2)
		target = ZENITH_PROFILE_BATTERY;
	else
		target = ZENITH_PROFILE_BALANCED;

	if (trace_zenith_auto_tune_enabled())
		trace_zenith_auto_tune(z_policy->policy->cpu, sat_pct,
				       events_rate_x2, t->active_profile,
				       target);

	if (target != t->active_profile) {
		zenith_apply_profile(t, target);
		t->active_profile = target;
	}

	/* Re-arm for the next classification window. */
	schedule_delayed_work(&z_policy->at_work,
			      msecs_to_jiffies(ZENITH_AUTO_TUNE_PERIOD_MS));
}

static ssize_t auto_tune_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->auto_tune);
}

static ssize_t auto_tune_store(struct gov_attr_set *attr_set,
			       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_policy;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	if (t->auto_tune == val)
		return count;
	t->auto_tune = val;

	list_for_each_entry(z_policy, &t->attr_set.policy_list, tunables_hook) {
		if (val) {
			z_policy->at_last_events =
				atomic64_read(&zenith_auto_input_events);
			z_policy->at_samples_total = 0;
			z_policy->at_samples_saturated = 0;
			schedule_delayed_work(&z_policy->at_work,
				msecs_to_jiffies(ZENITH_AUTO_TUNE_PERIOD_MS));
		} else {
			cancel_delayed_work_sync(&z_policy->at_work);
		}
	}

	return count;
}
static struct governor_attr auto_tune = __ATTR_RW(auto_tune);

/* auto_tune_* threshold tunables. The three *_pct fields are clamped to
 * the 0..100 range; the events_x2 fields accept any uint but only
 * values that can realistically occur in the 10 s observation window
 * are meaningful (events_x2 = events_per_2s, so 10 == 5 events/s).
 */
#define ZENITH_AT_PCT_STORE(_name) \
static ssize_t _name##_store(struct gov_attr_set *attr_set, \
			     const char *buf, size_t count) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	unsigned int val; \
	if (kstrtouint(buf, 10, &val) || val > 100) \
		return -EINVAL; \
	t->_name = val; \
	return count; \
}

#define ZENITH_AT_PCT_SHOW(_name) \
static ssize_t _name##_show(struct gov_attr_set *attr_set, char *buf) \
{ \
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->_name); \
}

#define ZENITH_AT_PCT_TUNABLE(_name) \
	ZENITH_AT_PCT_SHOW(_name) \
	ZENITH_AT_PCT_STORE(_name) \
	static struct governor_attr _name = __ATTR_RW(_name)

ZENITH_AT_PCT_TUNABLE(auto_tune_sat_load_pct);
ZENITH_AT_PCT_TUNABLE(auto_tune_hi_sat_pct);
ZENITH_AT_PCT_TUNABLE(auto_tune_lo_sat_pct);

ZENITH_TUNABLE_UINT(auto_tune_hi_events_x2);
ZENITH_TUNABLE_UINT(auto_tune_lo_events_x2);

static ssize_t profile_show(struct gov_attr_set *attr_set, char *buf)
{
	switch (to_zenith_tunables(attr_set)->active_profile) {
	case ZENITH_PROFILE_PERFORMANCE:	return sprintf(buf, "performance\n");
	case ZENITH_PROFILE_BALANCED:		return sprintf(buf, "balanced\n");
	case ZENITH_PROFILE_BATTERY:		return sprintf(buf, "battery\n");
	case ZENITH_PROFILE_LEGACY:		return sprintf(buf, "legacy\n");
	case ZENITH_PROFILE_CUSTOM:
	default:				return sprintf(buf, "custom\n");
	}
}

static ssize_t profile_store(struct gov_attr_set *attr_set,
			     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int prof;

	/* Accept the canonical name with optional trailing whitespace. */
	if (sysfs_streq(buf, "performance"))
		prof = ZENITH_PROFILE_PERFORMANCE;
	else if (sysfs_streq(buf, "balanced"))
		prof = ZENITH_PROFILE_BALANCED;
	else if (sysfs_streq(buf, "battery"))
		prof = ZENITH_PROFILE_BATTERY;
	else if (sysfs_streq(buf, "legacy"))
		prof = ZENITH_PROFILE_LEGACY;
	else if (sysfs_streq(buf, "custom"))
		prof = ZENITH_PROFILE_CUSTOM;
	else
		return -EINVAL;

	zenith_apply_profile(t, prof);
	t->active_profile = prof;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr profile = __ATTR_RW(profile);
ZENITH_TUNABLE_UINT_INVAL(screen_state);

static ssize_t screen_auto_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->screen_auto);
}

static ssize_t screen_auto_store(struct gov_attr_set *attr_set,
				 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->screen_auto = val;
	return count;
}
static struct governor_attr screen_auto = __ATTR_RW(screen_auto);
ZENITH_TUNABLE_UINT_INVAL(thermal_state);

static ssize_t thermal_auto_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->thermal_auto);
}

static ssize_t thermal_auto_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->thermal_auto = val;
	return count;
}
static struct governor_attr thermal_auto = __ATTR_RW(thermal_auto);

static ssize_t input_boost_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->input_boost_ms);
}

static ssize_t input_boost_ms_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1000)
		return -EINVAL;
	t->input_boost_ms = val;
	WRITE_ONCE(zenith_input_boost_active_ms, val);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr input_boost_ms = __ATTR_RW(input_boost_ms);

/* Parse up to ZENITH_EFF_BINS_MAX unsigned ints separated by whitespace
 * into out[], returning the number parsed. Extra tokens are ignored.
 * Returns -EINVAL if any token fails kstrtouint or if no tokens parse.
 */
static int zenith_parse_uint_list(const char *buf, unsigned int *out,
				  unsigned int max)
{
	char tmp[16];
	const char *p = buf;
	unsigned int nr = 0;
	int ret;

	while (*p && nr < max) {
		size_t len = 0;
		unsigned int val;

		while (*p == ' ' || *p == '\t' || *p == '\n')
			p++;
		if (!*p)
			break;

		while (p[len] && p[len] != ' ' && p[len] != '\t' &&
		       p[len] != '\n' && len < sizeof(tmp) - 1)
			len++;
		if (!len)
			break;
		memcpy(tmp, p, len);
		tmp[len] = '\0';
		p += len;

		ret = kstrtouint(tmp, 10, &val);
		if (ret)
			return -EINVAL;
		out[nr++] = val;
	}

	return nr ? (int)nr : -EINVAL;
}

static ssize_t efficient_freq_show(struct gov_attr_set *attr_set, char *buf)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int i;
	ssize_t len = 0;

	if (!t->eff_nr)
		return sprintf(buf, "0\n");

	for (i = 0; i < t->eff_nr; i++)
		len += sprintf(buf + len, "%u%c",
			       t->eff_freq[i],
			       (i + 1 == t->eff_nr) ? '\n' : ' ');
	return len;
}

static ssize_t efficient_freq_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int parsed[ZENITH_EFF_BINS_MAX];
	int n, i;

	n = zenith_parse_uint_list(buf, parsed, ZENITH_EFF_BINS_MAX);
	if (n < 0)
		return n;

	/* "efficient_freq=0" disables the ladder. */
	if (n == 1 && parsed[0] == 0) {
		t->eff_nr = 0;
		t->efficient_freq = 0;
		zenith_invalidate_cache(attr_set);
		return count;
	}

	/* Require strictly ascending sort — the ladder walks from low
	 * to high, and equal / decreasing entries would create
	 * unreachable bins.
	 */
	for (i = 1; i < n; i++) {
		if (parsed[i] <= parsed[i - 1])
			return -EINVAL;
	}

	for (i = 0; i < n; i++)
		t->eff_freq[i] = parsed[i];

	/* Broadcast the existing scalar up_delay_us to any new bins
	 * that don't yet have a delay. The user can then write a
	 * matching list to up_delay_us to override per-bin.
	 */
	for (i = 0; i < n; i++)
		if (!t->eff_delay_us[i])
			t->eff_delay_us[i] = t->up_delay_us;

	t->eff_nr = n;
	t->efficient_freq = t->eff_freq[0];
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr efficient_freq = __ATTR_RW(efficient_freq);

static ssize_t up_delay_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int i;
	ssize_t len = 0;

	if (!t->eff_nr)
		return sprintf(buf, "%u\n", t->up_delay_us);

	for (i = 0; i < t->eff_nr; i++)
		len += sprintf(buf + len, "%u%c",
			       t->eff_delay_us[i],
			       (i + 1 == t->eff_nr) ? '\n' : ' ');
	return len;
}

static ssize_t up_delay_us_store(struct gov_attr_set *attr_set,
				 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int parsed[ZENITH_EFF_BINS_MAX];
	int n, i;

	n = zenith_parse_uint_list(buf, parsed, ZENITH_EFF_BINS_MAX);
	if (n < 0)
		return n;

	for (i = 0; i < n; i++)
		if (parsed[i] > 1000000)
			return -EINVAL;

	if (n == 1) {
		/* Scalar write: broadcast to every existing bin and
		 * keep the scalar shadow up-to-date.
		 */
		t->up_delay_us = parsed[0];
		for (i = 0; i < ZENITH_EFF_BINS_MAX; i++)
			t->eff_delay_us[i] = parsed[0];
		zenith_invalidate_cache(attr_set);
		return count;
	}

	/* Vector write: must match eff_nr exactly. */
	if (!t->eff_nr || (unsigned int)n != t->eff_nr)
		return -EINVAL;

	for (i = 0; i < n; i++)
		t->eff_delay_us[i] = parsed[i];
	t->up_delay_us = parsed[0];
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr up_delay_us = __ATTR_RW(up_delay_us);

static ssize_t light_load_freq_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->light_load_freq);
}

static ssize_t light_load_freq_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->light_load_freq = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr light_load_freq = __ATTR_RW(light_load_freq);

static ssize_t light_load_threshold_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->light_load_threshold);
}

static ssize_t light_load_threshold_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	t->light_load_threshold = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr light_load_threshold = __ATTR_RW(light_load_threshold);

static ssize_t sampling_down_factor_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->sampling_down_factor);
}

static ssize_t sampling_down_factor_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val == 0 ||
	    val > ZENITH_MAX_SAMPLING_DOWN_FACTOR)
		return -EINVAL;
	t->sampling_down_factor = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr sampling_down_factor = __ATTR_RW(sampling_down_factor);

static ssize_t bias_load_threshold_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->bias_load_threshold);
}

static ssize_t bias_load_threshold_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	t->bias_load_threshold = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr bias_load_threshold = __ATTR_RW(bias_load_threshold);

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
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr up_threshold = __ATTR_RW(up_threshold);

static ssize_t up_threshold_hispeed_show(struct gov_attr_set *attr_set,
					 char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->up_threshold_hispeed);
}

static ssize_t up_threshold_hispeed_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	/* 0 = disabled (fall back to up_threshold). >100 is nonsense for
	 * a load percentage; up_threshold_hispeed < up_threshold is also
	 * nonsense because it would widen the hispeed capture zone
	 * rather than narrow it, but the user is free to do that if they
	 * really want to.
	 */
	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	t->up_threshold_hispeed = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr up_threshold_hispeed =
	__ATTR_RW(up_threshold_hispeed);

static ssize_t down_threshold_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->down_threshold);
}

static ssize_t down_threshold_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	t->down_threshold = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr down_threshold = __ATTR_RW(down_threshold);

static ssize_t hispeed_freq_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->hispeed_freq);
}

static ssize_t hispeed_freq_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->hispeed_freq = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr hispeed_freq = __ATTR_RW(hispeed_freq);

static ssize_t hispeed_load_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->hispeed_load);
}

static ssize_t hispeed_load_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val == 0 || val > 100)
		return -EINVAL;
	t->hispeed_load = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr hispeed_load = __ATTR_RW(hispeed_load);

static ssize_t climb_mode_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->climb_mode);
}

static ssize_t climb_mode_store(struct gov_attr_set *attr_set,
				const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_CLIMB_MODE_STEP)
		return -EINVAL;
	t->climb_mode = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr climb_mode = __ATTR_RW(climb_mode);

static ssize_t freq_step_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->freq_step_pct);
}

static ssize_t freq_step_pct_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val == 0 || val > 100)
		return -EINVAL;
	t->freq_step_pct = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr freq_step_pct = __ATTR_RW(freq_step_pct);

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
	zenith_invalidate_cache(attr_set);
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
		z_pol->up_rate_delay_ns = (u64)val * NSEC_PER_USEC;
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
		z_pol->down_rate_delay_ns = (u64)val * NSEC_PER_USEC;
		update_min_rate_limit_ns(z_pol);
	}
	return count;
}
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);

static struct attribute *zenith_attrs[] = {
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&up_threshold.attr,
	&up_threshold_hispeed.attr,
	&down_threshold.attr,
	&hispeed_freq.attr,
	&hispeed_load.attr,
	&climb_mode.attr,
	&freq_step_pct.attr,
	&profile.attr,
	&auto_tune.attr,
	&auto_tune_sat_load_pct.attr,
	&auto_tune_hi_sat_pct.attr,
	&auto_tune_lo_sat_pct.attr,
	&auto_tune_hi_events_x2.attr,
	&auto_tune_lo_events_x2.attr,
	&powersave_bias.attr,
	&io_is_busy.attr,
	&iowait_boost_min.attr,
	&ignore_nice_load.attr,
	&screen_state.attr,
	&screen_auto.attr,
	&thermal_state.attr,
	&thermal_auto.attr,
	&input_boost_ms.attr,
	&efficient_freq.attr,
	&up_delay_us.attr,
	&light_load_freq.attr,
	&light_load_threshold.attr,
	&sampling_down_factor.attr,
	&bias_load_threshold.attr,
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
	INIT_DELAYED_WORK(&z_policy->at_work, zenith_auto_tune_work);

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
	tunables->up_threshold_hispeed	= ZENITH_DEFAULT_UP_THRESHOLD_HISPEED;
	tunables->down_threshold	= ZENITH_DEFAULT_DOWN_THRESHOLD;
	tunables->hispeed_freq		= ZENITH_DEFAULT_HISPEED_FREQ;
	tunables->hispeed_load		= ZENITH_DEFAULT_HISPEED_LOAD;
	tunables->climb_mode		= ZENITH_DEFAULT_CLIMB_MODE;
	tunables->freq_step_pct		= ZENITH_DEFAULT_FREQ_STEP_PCT;
	tunables->active_profile	= ZENITH_PROFILE_CUSTOM;
	tunables->auto_tune		= 0;
	tunables->auto_tune_sat_load_pct = ZENITH_DEFAULT_AT_SAT_LOAD_PCT;
	tunables->auto_tune_hi_sat_pct	= ZENITH_DEFAULT_AT_HI_SAT_PCT;
	tunables->auto_tune_lo_sat_pct	= ZENITH_DEFAULT_AT_LO_SAT_PCT;
	tunables->auto_tune_hi_events_x2 = ZENITH_DEFAULT_AT_HI_EVENTS_X2;
	tunables->auto_tune_lo_events_x2 = ZENITH_DEFAULT_AT_LO_EVENTS_X2;
	tunables->powersave_bias	= ZENITH_DEFAULT_POWERSAVE_BIAS;
	tunables->io_is_busy		= ZENITH_DEFAULT_IO_IS_BUSY;
	tunables->iowait_boost_min	= ZENITH_DEFAULT_IOWAIT_BOOST_MIN;
	tunables->ignore_nice_load	= 0;
	tunables->screen_state		= 1;
	tunables->screen_auto		= 0;
	tunables->thermal_state		= 0;
	tunables->thermal_auto		= ZENITH_DEFAULT_THERMAL_AUTO;
	tunables->input_boost_ms	= ZENITH_DEFAULT_INPUT_BOOST_MS;
	tunables->efficient_freq	= ZENITH_DEFAULT_EFFICIENT_FREQ;
	tunables->up_delay_us		= ZENITH_DEFAULT_UP_DELAY_US;
	tunables->light_load_freq	= ZENITH_DEFAULT_LIGHT_LOAD_FREQ;
	tunables->light_load_threshold	= ZENITH_DEFAULT_LIGHT_LOAD_THRESHOLD;
	tunables->sampling_down_factor	= ZENITH_DEFAULT_SAMPLING_DOWN_FACTOR;
	tunables->bias_load_threshold	= ZENITH_DEFAULT_BIAS_LOAD_THRESHOLD;
	WRITE_ONCE(zenith_input_boost_active_ms, ZENITH_DEFAULT_INPUT_BOOST_MS);

	ret = kobject_init_and_add(&tunables->attr_set.kobj,
				   &zenith_tunables_ktype,
				   get_governor_parent_kobj(policy),
				   "zenith");
	if (ret) {
		/* kobject_init_and_add() always initialises the kobject
		 * refcount; on failure we must drop that reference via
		 * kobject_put(), which invokes zenith_tunables_free() and
		 * kfrees the backing tunables. Calling kfree() directly
		 * would bypass the release callback and leak any resources
		 * later attached to the ktype.
		 */
		kobject_put(&tunables->attr_set.kobj);
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

	/* Remove from the shared tunables' policy_list first so a concurrent
	 * sysfs store (e.g. auto_tune=1) can no longer iterate this policy
	 * and re-schedule at_work against it. Only after the list unlink
	 * is it safe to cancel the delayed work and free z_policy.
	 */
	mutex_lock(&global_tunables_lock);
	if (!gov_attr_set_put(&tunables->attr_set, &z_policy->tunables_hook))
		global_tunables = NULL;
	mutex_unlock(&global_tunables_lock);

	cancel_delayed_work_sync(&z_policy->at_work);

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

	z_policy->up_rate_delay_ns = (u64)z_policy->tunables->up_rate_limit_us * NSEC_PER_USEC;
	z_policy->down_rate_delay_ns = (u64)z_policy->tunables->down_rate_limit_us * NSEC_PER_USEC;
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

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ZENITH
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &zenith_gov;
}
#endif

/************************ Input Boost ***********************/

static void zenith_input_event(struct input_handle *handle, unsigned int type,
			       unsigned int code, int value)
{
	unsigned int active = READ_ONCE(zenith_input_boost_active_ms);
	u64 deadline;

	if (type != EV_KEY && type != EV_ABS && type != EV_REL)
		return;

	/* Always bump the auto-tune counter so a policy that enables
	 * auto_tune mid-session has recent data. Cheap atomic inc.
	 */
	atomic64_inc(&zenith_auto_input_events);

	if (!active)
		return;

	deadline = ktime_get_ns() + (u64)active * NSEC_PER_MSEC;
	atomic64_set(&zenith_input_boost_until_ns, deadline);
}

static int zenith_input_connect(struct input_handler *handler,
				struct input_dev *dev,
				const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "zenith";

	ret = input_register_handle(handle);
	if (ret)
		goto err_free;

	ret = input_open_device(handle);
	if (ret)
		goto err_unregister;

	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return ret;
}

static void zenith_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id zenith_input_ids[] = {
	/* Multitouch screens */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
				BIT_MASK(ABS_MT_POSITION_X) },
	},
	/* Touchpads */
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] = BIT_MASK(ABS_X) },
	},
	/* Keyboards */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};

static struct input_handler zenith_input_handler = {
	.event		= zenith_input_event,
	.connect	= zenith_input_connect,
	.disconnect	= zenith_input_disconnect,
	.name		= "zenith",
	.id_table	= zenith_input_ids,
};

/************************ FB blank notifier (screen_auto) ********************/

#ifdef CONFIG_FB_NOTIFY
static int zenith_fb_notifier_cb(struct notifier_block *nb,
				 unsigned long action, void *data)
{
	struct fb_event *evdata = data;
	int blank;
	unsigned int new_state;

	/* FB_EVENT_BLANK is the only blank event defined in
	 * android-common-5.10. Some older trees also ship
	 * FB_EARLY_EVENT_BLANK, but this one does not — including
	 * the symbol there breaks the build when CONFIG_FB_NOTIFY=y.
	 */
	if (action != FB_EVENT_BLANK)
		return NOTIFY_OK;
	if (!evdata || !evdata->data)
		return NOTIFY_OK;

	blank = *(int *)evdata->data;
	new_state = (blank == FB_BLANK_UNBLANK) ? 1 : 0;

	/* All zenith policies share one global_tunables (per-cluster
	 * clones hold a reference to the same struct), so a single write
	 * propagates everywhere.
	 */
	mutex_lock(&global_tunables_lock);
	if (global_tunables && global_tunables->screen_auto)
		WRITE_ONCE(global_tunables->screen_state, new_state);
	mutex_unlock(&global_tunables_lock);

	return NOTIFY_OK;
}

static struct notifier_block zenith_fb_notifier = {
	.notifier_call	= zenith_fb_notifier_cb,
	.priority	= 0,
};
#endif /* CONFIG_FB_NOTIFY */

static int __init zenith_gov_init(void)
{
	int ret;

	pr_info("Zenith: V2 Dreadnought (EAS/EM/Display/Thermal) Initialized. By ENI for LO.\n");

	ret = input_register_handler(&zenith_input_handler);
	if (ret)
		pr_warn("Zenith: input handler register failed (%d), boost disabled\n",
			ret);

#ifdef CONFIG_FB_NOTIFY
	ret = fb_register_client(&zenith_fb_notifier);
	if (ret)
		pr_warn("Zenith: fb notifier register failed (%d), screen_auto disabled\n",
			ret);
#endif

	return cpufreq_register_governor(&zenith_gov);
}
fs_initcall(zenith_gov_init);