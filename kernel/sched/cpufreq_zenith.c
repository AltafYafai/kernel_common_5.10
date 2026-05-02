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
#include <linux/jump_label.h>
#include <linux/atomic.h>
#include <linux/ktime.h>
#include <linux/math64.h>
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
/*
 * Optional drm_panel_notifier path.
 *
 * The Common Android Kernel 5.10 GKI tree does not ship a drm panel
 * notifier; it is a vendor-only mechanism (Qualcomm's
 * drm/drm_panel_notifier.h, MediaTek's panel_event_notifier, etc.).
 * Vendor builds that backport such a notifier framework can opt in
 * by defining CONFIG_DRM_PANEL_NOTIFY=y and providing a header at
 * <drm/drm_panel_notifier.h> that declares:
 *
 *     int  drm_panel_notifier_register(struct notifier_block *nb);
 *     int  drm_panel_notifier_unregister(struct notifier_block *nb);
 *     enum { DRM_PANEL_EVENT_BLANK = ..., };
 *     enum { DRM_PANEL_BLANK_UNBLANK = 0, DRM_PANEL_BLANK_POWERDOWN = ..., };
 *     struct drm_panel_notifier { void *data; };  (data points to int *blank)
 *
 * On stock GKI builds the config is undefined and this whole block
 * compiles out, leaving the legacy CONFIG_FB_NOTIFY path unchanged.
 */
#ifdef CONFIG_DRM_PANEL_NOTIFY
#include <drm/drm_panel_notifier.h>
#endif
#ifdef CONFIG_SCHED_PREFER_SILVER
#include <linux/prefer_silver.h>
#endif
#include <trace/events/power.h>

#define CREATE_TRACE_POINTS
#include <trace/events/cpufreq_zenith.h>

/* Constants & Defaults */
/* Permille of SCHED_CAPACITY_SCALE at which iowait boost starts.
 * 125 == SCHED_CAPACITY_SCALE / 8, preserving the historical default.
 */
#define ZENITH_DEFAULT_IOWAIT_BOOST_MIN		125
#define ZENITH_DEFAULT_IOWAIT_STACK_PCT		50	/* 0 = legacy max(util, boost) */

/* iowait_backoff_after_ms (default 0, off):
 *
 * The doubling-on-each-iowait-flag climb in zenith_iowait_boost()
 * has no upper bound other than SCHED_CAPACITY_SCALE.  On long
 * sustained-iowait workloads (level loads, app installs, big-file
 * syncs) the boost saturates near max for the duration, with
 * diminishing return: by the time the boost has doubled past the
 * point where extra freq materially reduces I/O wait, the cpu is
 * burning power on cycles that gain almost nothing.
 *
 * When set, this tunable starts shrinking the boost stack once a
 * single iowait episode has been live for N milliseconds.  The
 * doubling step in zenith_iowait_boost() flips to a halving step
 * once the timer elapses, so the boost decays toward the floor at
 * the same rate the apply path would decay it during quiet ticks.
 * iowait flag observations no longer keep extending the boost; the
 * episode dies on its own and the next fresh iowait re-arms from
 * the floor with a clean timer.
 *
 * 0 disables the backoff entirely (legacy behaviour: doubling
 * climbs to SCHED_CAPACITY_SCALE without an upper time bound).
 */
#define ZENITH_DEFAULT_IOWAIT_BACKOFF_AFTER_MS	0
#define ZENITH_DEFAULT_UP_THRESHOLD		75
#define ZENITH_DEFAULT_UP_THRESHOLD_HISPEED	0	/* disabled */
#define ZENITH_DEFAULT_DOWN_THRESHOLD		60
#define ZENITH_DEFAULT_HISPEED_FREQ		0	/* disabled */
#define ZENITH_DEFAULT_HISPEED_FREQ_PCT		55	/* fallback when hispeed_freq=0 */
#define ZENITH_DEFAULT_HISPEED_LOAD		65
#define ZENITH_DEFAULT_HISPEED_HYST_PCT		10	/* exit hysteresis margin */

/* hispeed_entry_streak (default 0, off):
 *
 * Symmetric entry-side hysteresis for the hispeed tier.  Today the
 * tier has only exit hysteresis (hispeed_hyst_pct): once load_pct
 * crosses hispeed_load it activates immediately, leaving the tier
 * vulnerable to single-sample noise spikes near the boundary.
 *
 * When set to N (>0), require load_pct >= hispeed_load to hold for
 * N+1 consecutive samples before flipping hispeed_active to true.
 * 0 preserves the historical immediate-flip behaviour.  Capped to
 * ZENITH_HISPEED_ENTRY_STREAK_MAX so the per-policy u8 counter
 * never overflows.
 */
#define ZENITH_DEFAULT_HISPEED_ENTRY_STREAK	0
#define ZENITH_HISPEED_ENTRY_STREAK_MAX		16

/* brutal_entry_streak (default 0, off):
 *
 * Symmetric to hispeed_entry_streak, but for the brutality
 * snap-to-max tier.  Today the brutality path flips
 * brutal_active=true and pins policy->max on the very first
 * sample where load_pct >= up_threshold, leaving the tier
 * vulnerable to single-sample spikes (scheduler wake-up
 * bursts, sampler quantisation noise) that pin the cluster
 * to max for the whole down_threshold hysteresis window.
 *
 * When set to N (>0), require load_pct >= up_threshold to
 * hold for N+1 consecutive samples before flipping
 * brutal_active to true.  0 preserves the historical
 * immediate-flip behaviour.  Only gates SNAP mode; STEP
 * climb mode is already gentle by design and is unaffected.
 * Exit hysteresis via down_threshold / brutal_active hold is
 * unchanged.  Capped to ZENITH_BRUTAL_ENTRY_STREAK_MAX so
 * the per-policy u8 counter never overflows.
 */
#define ZENITH_DEFAULT_BRUTAL_ENTRY_STREAK	0
#define ZENITH_BRUTAL_ENTRY_STREAK_MAX		16

/* up_threshold_adaptive (default 0, off):
 *
 * Variance-adaptive shaping of the brutality entry threshold.  The
 * static up_threshold is a single number that has to fit two
 * different workloads: bursty / interactive (UI scrolling, gestures,
 * frame pacing) where a low up_threshold is desirable so the cluster
 * climbs fast on a single hot sample, and sustained / steady
 * (transcoding, long compute) where a higher up_threshold avoids
 * pinning the cluster at max for the whole run.
 *
 * When set to N (1..30), zenith_get_next_freq() lowers the effective
 * up_threshold by up to N percent when the recent load signal is
 * bursty, leaving it unchanged on a steady signal.  The bursty/
 * steady signal is a rolling EWMA of |load_pct - prev_load_pct|;
 * high mean-absolute-change == bursty.  The adjustment is applied
 * only on the regular up_threshold path -- the screen-off (95),
 * thermal (90), and up_threshold_hispeed overrides are absolute
 * pinning values and stay verbatim.
 *
 * 0 disables the adjustment entirely (legacy: dynamic_up_thresh ==
 * tunables->up_threshold whenever no override fires).  Cap at 30 so
 * a runaway tunable can never lower up_threshold by more than 30%
 * of its value, which would be indistinguishable from "force snap"
 * behaviour.
 */
#define ZENITH_DEFAULT_UP_THRESHOLD_ADAPTIVE	0
#define ZENITH_UP_THRESHOLD_ADAPTIVE_MAX	30

/* Time-based cache TTL for the uclamp_{min,max} per-policy walks.  The
 * per-rq UCLAMP values are maintained by the scheduler on every
 * enqueue / dequeue, so a 1 ms staleness bound on the cached
 * aggregate is imperceptible to userspace (ADPF sessions are open
 * for tens of milliseconds to seconds) but drops the per-policy rq
 * walk from every freq eval down to once per millisecond.
 */
#define ZENITH_UCLAMP_CACHE_TTL_NS		(1 * NSEC_PER_MSEC)

/* Maximum number of bins in the efficient_freq soft-cap ladder.
 *
 * The ladder is the array of (eff_freq, eff_delay_us) pairs the
 * efficient_freq= / up_delay_us= sysfs nodes accept and that
 * zenith_get_next_freq() walks once per evaluation.  This number
 * bounds three things: the parser's parsed[] stack buffer, the
 * static eff_freq[] / eff_delay_us[] tables on struct
 * zenith_tunables, and the per-policy eff_unlock_at_ns[] deadline
 * table on struct zenith_policy.  The runtime value of eff_nr
 * (set by the parser, capped to this ceiling) decides how many of
 * those slots the hot path actually walks; a CSV with fewer entries
 * leaves the rest of the array unused but does not save any
 * footprint.
 *
 * Was 4, raised to 8 to fit the more finely-binned freq tables
 * found on Dimensity 9000+ / Snapdragon 8 Gen 3-class SoCs.  A bin
 * count above 8 is hard to tune by hand and burns d-cache on the
 * walk for diminishing return -- the cap is deliberate and not
 * runtime-configurable.  Memory cost vs the old value: 8 - 4 = 4
 * additional uints per array; tunables grows by 32 bytes (two
 * arrays) and zenith_policy by 32 bytes (one u64 array), totals
 * negligible against the rest of those structs.  Hot-path cost is
 * unchanged: the loop bound is eff_nr, not the array ceiling.
 */
#define ZENITH_EFF_BINS_MAX			8

/* Depth of the per-policy auto-tune classifier ring buffer.
 *
 * Each entry records one V1 classifier window (the worker fires every
 * ZENITH_AUTO_TUNE_PERIOD_MS = 10 s) plus the V2 state machine view at
 * that moment.  16 entries gives roughly 160 s of post-mortem visible
 * via the read-only at_log sysfs node, which covers a typical bench
 * run, an app launch, a bursty UI sequence or a thermal-recovery cycle
 * without any extra tooling.
 *
 * The ring is a single-writer/multi-reader buffer: only the per-policy
 * delayed_work worker pushes; sysfs readers walk it from oldest to
 * newest under no extra locking, accepting at most one window of
 * tearing on the wrap (rare and harmless for diagnostics).  Storage is
 * ~64 bytes per entry, so 16 entries cost ~1 KiB per policy.
 */
#define ZENITH_AT_LOG_NR			16

#define ZENITH_CLIMB_MODE_SNAP			0	/* default */
#define ZENITH_CLIMB_MODE_STEP			1
#define ZENITH_PROFILE_CUSTOM			0	/* default */
#define ZENITH_PROFILE_PERFORMANCE		1
#define ZENITH_PROFILE_BALANCED			2
#define ZENITH_PROFILE_BATTERY			3
#define ZENITH_PROFILE_LEGACY			4

/* Profile selected via the zenith.profile= kernel cmdline. Parsed by
 * zenith_setup_profile() at early_param time and consumed on the
 * first-init branch of zenith_init() so the governor comes up on the
 * requested preset before any userspace can write to the profile sysfs
 * node. Defaults to CUSTOM, which means "no cmdline override".
 */
static unsigned int zenith_cmdline_profile = ZENITH_PROFILE_CUSTOM;

/* Optional per-policy cmdline profile overrides parsed from
 * zenith.policy_profile=N:prof,M:prof,...  Indexed by the "policy
 * anchor cpu" (cpumask_first(policy->cpus) at init time -- what
 * SurfaceFlinger and the cpufreq sysfs tree already print as
 * policyN).  Each slot defaults to ZENITH_PROFILE_CUSTOM meaning
 * "no per-policy override; fall through to zenith_cmdline_profile".
 *
 * Using a fixed-size array indexed by CPU is intentional: the
 * parser runs at early_param time when per-cpu structures aren't
 * fully populated, and an NR_CPUS-sized u8 table costs one byte
 * per possible CPU on the kernel image (256 bytes on a common
 * aarch64 defconfig) -- cheaper than any dynamic allocation would
 * save, and lookup is a single load at init time.
 */
static u8 zenith_cmdline_policy_profile[NR_CPUS] = {
	[0 ... NR_CPUS - 1] = ZENITH_PROFILE_CUSTOM,
};

/* Static-branch fold for zero-default feature tunables.
 *
 * audio_aware, camera_aware, render_aware and psi_aware all default
 * to 0 (off) and are checked on every zenith_get_next_freq() call.
 * Folding them through a DEFINE_STATIC_KEY_FALSE turns the hot-path
 * "if (z_policy->tunables->X)" load-cmp-branch sequence into a
 * single never-taken jump while the feature is disabled, with the
 * cold path moved out of line for better i-cache density.
 *
 * The keys are governor-global (one zenith_tunables instance is
 * shared across all policies, see global_tunables_lock), so there
 * is no per-policy synchronisation question.  Each *_store callback
 * synchronises its key with the new tunable value via
 * static_branch_enable / static_branch_disable, which both sleep
 * acquiring cpus_read_lock() but are safe from sysfs store context.
 *
 * No init-time enable is needed: all four tunables default to 0 in
 * zenith_tunables_init() and zenith_set_profile_defaults() never
 * touches them, so the keys correctly start in the FALSE state.
 */
DEFINE_STATIC_KEY_FALSE(zenith_audio_aware_key);
DEFINE_STATIC_KEY_FALSE(zenith_camera_aware_key);
DEFINE_STATIC_KEY_FALSE(zenith_render_aware_key);
DEFINE_STATIC_KEY_FALSE(zenith_psi_aware_key);

/* Transition invariant for the four feature static keys above:
 *
 *   tunables->X (sysfs-visible scalar)  ==  static-key state of zenith_X_key
 *
 * is established by every *_store callback below using
 *
 *   t->X = val;
 *   zenith_set_static_key(&zenith_X_key, val);
 *
 * in that order — store the scalar first, then sync the key.  All
 * stores run under attr_set->update_lock (held by the gov_attr
 * dispatcher), so the two writes are serialised against each other
 * and against any other store on the same attr_set.
 *
 * The hot path reads the static key (single never-taken jump while
 * the feature is off), not the scalar, so a momentary tear between
 * the two writes can at worst cause one tick of the get_next_freq()
 * fast path to take the wrong branch.  Acceptable: feature flips are
 * rare (sysfs writes from system_server / init shell only), the wrong
 * branch is itself benign (returns or skips a tier), and the next
 * tick will see the consistent state.
 *
 * Implications for anyone adding a new feature key here:
 *   - The init state of every DEFINE_STATIC_KEY_FALSE is FALSE; do
 *     not flip the key in zenith_init() unless the matching scalar
 *     also defaults nonzero.  No init-time enable is needed for the
 *     four current keys (all four scalars default to 0 in
 *     zenith_tunables_init()).
 *   - Profile presets in zenith_apply_profile() must not silently
 *     toggle a feature scalar without also calling
 *     zenith_set_static_key(); doing so violates the invariant.  The
 *     four current presets (perf/balanced/battery/legacy) deliberately
 *     leave audio_aware / camera_aware / render_aware / psi_aware
 *     untouched for exactly this reason — they are user-managed
 *     opt-ins, not preset state.
 *   - static_branch_enable / static_branch_disable both sleep
 *     acquiring cpus_read_lock() but are safe from sysfs store
 *     context.  Do not call zenith_set_static_key() from the hot path
 *     or from any context that holds a spinlock; both will deadlock.
 */
static inline void zenith_set_static_key(struct static_key_false *key,
					 bool enable)
{
	if (enable)
		static_branch_enable(key);
	else
		static_branch_disable(key);
}

#define ZENITH_FEATURE_ENABLED(name)	\
	static_branch_unlikely(&zenith_##name##_key)

#define ZENITH_DEFAULT_CLIMB_MODE		ZENITH_CLIMB_MODE_SNAP
#define ZENITH_DEFAULT_FREQ_STEP_PCT		5

/* freq_step_adaptive (default 0, off):
 *
 * When STEP climb mode is selected, the per-sample step is a fixed
 * fraction of policy->max (freq_step_pct) regardless of how far
 * load_pct has overshot up_threshold.  A sample at load_pct = 76%
 * with up_threshold = 75% produces the same step as a sample at
 * load_pct = 99% -- the latter clearly wants to converge faster.
 *
 * When set to 1, the base step is scaled by (100 + overshoot)% where
 * overshoot is 100 * (load_pct - up_threshold) / (100 - up_threshold),
 * so:
 *   - At load_pct == up_threshold: 1.0x base step (no change)
 *   - At load_pct == 100:           2.0x base step (double)
 *   - Linearly interpolated in between.
 *
 * Only affects STEP climb mode; SNAP mode is load-independent by
 * design and unchanged.  Preserves a minimum step of 1 (same guard
 * as the base path) so a zero freq_step_pct cannot stall the climb.
 */
#define ZENITH_DEFAULT_FREQ_STEP_ADAPTIVE	0
#define ZENITH_DEFAULT_THERMAL_AUTO		1
#define ZENITH_THERMAL_AUTO_PRESSURE_PCT	10
#define ZENITH_DEFAULT_THERMAL_PRESSURE_CONTINUOUS	0

/* prefer_silver_aware defaults.  See struct zenith_tunables for
 * semantics.  Hot threshold of 50%% means the bump fires when at
 * least half of the recent prefer_silver decisions actually
 * redirected onto a silver core; hot bump of 5 points is small
 * enough to avoid a perceived step but large enough to noticeably
 * delay big-cluster downclock during sustained UI navigation.
 * Both knobs are tunable; the defaults are conservative.
 */
#define ZENITH_DEFAULT_PREFER_SILVER_AWARE			0
#define ZENITH_DEFAULT_PREFER_SILVER_HOT_THRESHOLD_PCT		50
#define ZENITH_DEFAULT_PREFER_SILVER_HOT_BUMP_PCT		5
#define ZENITH_PREFER_SILVER_HOT_BUMP_MAX_PCT			20

/* brutal_decay_ms upper bound.  500ms is generous: a longer window
 * is functionally indistinguishable from "no cliff exit" because
 * the underlying EAS / load signal will move the floor anyway.
 */
#define ZENITH_DEFAULT_BRUTAL_DECAY_MS				0
#define ZENITH_BRUTAL_DECAY_MS_MAX				500

/* thermal_util_derate (default 1, on):
 *
 * When set, zenith_get_util() scales down its output by the
 * fraction of capacity currently being eaten by SoC thermal
 * pressure (arch_scale_thermal_pressure(cpu) / arch_scale_cpu_capacity(cpu)).
 *
 * Without this, util keeps demanding 100% of the *thermal-throttled*
 * max, which causes the governor to pin policy->max while the
 * thermal framework drops the cap.  The result is a continuous
 * yo-yo: thermal lowers max -> we still pin max -> SoC stays hot ->
 * thermal lowers further.  Derating util smooths this loop because
 * we ask for less than the throttled max, giving the SoC a chance
 * to cool.
 *
 * Concretely, with 25% of capacity thermal-pressured, util is
 * scaled by (cap - pressure) / cap = 0.75.  A util of 800/1024
 * becomes 600/1024.  Frequency selection then targets the
 * derated demand instead of clamping to the (already throttled)
 * max.
 *
 * Only applies the scale when at least
 * ZENITH_THERMAL_DERATE_FLOOR_PCT of the capacity is pressured;
 * for sub-threshold pressure the cost of the multiply isn't
 * worth the precision.
 *
 * Tunable-gated 0/1.  Default 1 (on) -- the existing thermal_auto
 * tier already handles the "throttle hard" case, but it doesn't
 * smooth the dance.  This tier adds the smoothing.
 */
#define ZENITH_DEFAULT_THERMAL_UTIL_DERATE	1
#define ZENITH_THERMAL_DERATE_FLOOR_PCT		5

/* thermal_derate_rate_pct (default 0, off):
 *
 * The static thermal_util_derate scales util by the *current*
 * pressure level, which lags the actual thermal event: by the time
 * pressure has risen to the level where the derate bites, the
 * cluster has already spent the rising slope at full demand.  On
 * SoCs with fast pressure tracking (e.g. tsensor-driven thermal
 * frameworks) this is visible as an overshoot before the derate
 * catches up.
 *
 * thermal_derate_rate_pct adds a derivative term: when pressure
 * is rising sample-to-sample on a cpu, additionally scale util by
 * up to thermal_derate_rate_pct percent based on how big the
 * single-step rise was relative to capacity.  Same shape as the
 * level derate -- just on the slope instead of the value.  Caps
 * the additional reduction at thermal_derate_rate_pct so a single
 * pressure spike can never zero util.
 *
 *   rise_pct = ((pressure - prev_pressure) * 100) / max  // 0..100
 *   if (rise_pct > thermal_derate_rate_pct)
 *           rise_pct = thermal_derate_rate_pct;
 *   util_out = util_out * (100 - rise_pct) / 100;
 *
 * Only applied when the static thermal_util_derate also fires
 * (pressure >= ZENITH_THERMAL_DERATE_FLOOR_PCT and pressure < max),
 * so the rate term piggybacks on the existing gating and adds no
 * cost when the level derate is silent.  prev_pressure is per-cpu
 * and zero-initialised by zenith_start()'s memset, so the first
 * sample after attach sees rise_pct == 0 (no derivative kick on
 * cold start).
 *
 * 0 disables the derivative term entirely; the level derate is
 * unaffected.  Range 0..100; values >100 rejected by sysfs.  No
 * upper cap on the rate of rise itself -- the rate_pct clamp at
 * thermal_derate_rate_pct is the only bound that matters for the
 * output.
 */
#define ZENITH_DEFAULT_THERMAL_DERATE_RATE_PCT	0

/* freq_stability_margin_pct (default 3):
 *
 * When the resolved target_freq is within margin percent of policy->max
 * below the currently-requested frequency, keep the current request
 * instead of issuing a tiny downward transition.  This removes
 * bin-boundary oscillation where the target dances just below an OPP
 * edge and pays regulator / PLL transition cost for no perceptible
 * gain.  Upward transitions are never suppressed.
 *
 * 0 disables the margin entirely (legacy: every target != current
 * request can switch subject only to the rate limiter).  Range 0..10;
 * values above 10 are aggressive enough to feel sticky.
 */
#define ZENITH_DEFAULT_FREQ_STABILITY_MARGIN_PCT	3
#define ZENITH_FREQ_STABILITY_MARGIN_PCT_MAX		10

/* down_rate_adaptive (default 1, on):
 *
 * Scales the effective down_rate_delay_ns by the recent load variance
 * EWMA.  Bursty workloads get up to 2x the base delay, keeping freq
 * elevated between adjacent frame/render bursts; steady workloads use
 * the configured delay unchanged.
 *
 *   eff_delay = base_delay * (256 + min(var, 256)) / 256
 *
 * 0 disables the scaling and restores the fixed down-rate delay.
 */
#define ZENITH_DEFAULT_DOWN_RATE_ADAPTIVE	1

/* wakeup_boost (default 1, on):
 *
 * Detects idle-to-busy transitions where util jumps from below 10% to
 * at least 40% of capacity in one scheduler callback.  On detection,
 * the next two upward transitions bypass up_rate_limit so app launch,
 * screen-on and notification wakeups avoid the cold-start sample lag.
 *
 * 0 disables the detector.  The thresholds are expressed as capacity
 * percentages so they scale across ARM DynamIQ clusters.
 */
#define ZENITH_DEFAULT_WAKEUP_BOOST		1
#define ZENITH_WAKEUP_IDLE_THRESH_PCT		10
#define ZENITH_WAKEUP_BUSY_THRESH_PCT		40
#define ZENITH_WAKEUP_BOOST_TICKS		2

/* down_threshold_adaptive (default 0, off):
 *
 * Mirrors up_threshold_adaptive on the brutality exit side.  When set
 * to N (1..20), the effective down_threshold is lowered by up to N
 * percent under bursty load, widening the hysteresis band so max freq
 * holds through frame bursts.  Steady load leaves the configured
 * down_threshold unchanged.
 *
 * 0 disables the adjustment.
 */
#define ZENITH_DEFAULT_DOWN_THRESHOLD_ADAPTIVE	0
#define ZENITH_DOWN_THRESHOLD_ADAPTIVE_MAX	20

/* rate_limit_cluster_scale (default 1, on):
 *
 * Applies asymmetric cached rate limits to little-cluster policies on
 * heterogeneous SoCs: up_rate_delay_ns is doubled and
 * down_rate_delay_ns is halved.  Big / prime clusters keep the
 * configured limits.  On homogeneous SoCs every policy is treated as a
 * big cluster and this is a no-op.
 *
 * 0 disables the per-cluster scaling.
 */
#define ZENITH_DEFAULT_RATE_LIMIT_CLUSTER_SCALE	1
#define ZENITH_CLUSTER_LITTLE_THRESH_PCT	60

#define ZENITH_DEFAULT_UP_RATE_LIMIT_US		100
#define ZENITH_DEFAULT_DOWN_RATE_LIMIT_US	4000
#define ZENITH_DEFAULT_POWERSAVE_BIAS		0
#define ZENITH_DEFAULT_IO_IS_BUSY		1
#define ZENITH_DEFAULT_INPUT_BOOST_MS		80
#define ZENITH_DEFAULT_INPUT_BOOST_DECAY_MS	30

/* input_boost_decay_curve (default 0, linear):
 *
 * The input-boost decay path lowers a synthetic floor from the full
 * boost ceiling down to policy->min across input_boost_decay_ms.
 * Until now that ramp was strictly linear:
 *
 *   floor = ceiling - span * elapsed / decay_ns        (LINEAR)
 *
 * which drops fast at the start of the tail and slows near the
 * end -- the opposite of what a gesture actually wants.  The hand
 * leaves the screen, the compositor has already committed one or
 * two post-gesture frames at high freq, and we want the floor to
 * hold high for a short moment (keeping the render thread on a
 * comfortable bin for the settle frames) and then drop off
 * quickly at the end of the decay window.
 *
 * When set to 1 (CUBIC), the normalised elapsed time is cubed
 * before being consumed:
 *
 *   t256  = elapsed * 256 / decay_ns                   (0..256)
 *   cubic = t256^3 / 65536                             (0..256)
 *   floor = ceiling - span * cubic / 256               (CUBIC)
 *
 * At elapsed == 0 both curves give floor == ceiling; at
 * elapsed == decay_ns both give floor == policy->min.  The
 * midpoint differs sharply: LINEAR has dropped 50 %% at the
 * halfway mark, CUBIC has dropped ~12.5 %%.  The full-boost phase
 * (while remaining > decay_ns) is unaffected.
 *
 * All arithmetic is u32-bounded by design: t256 is capped at 256,
 * its cube at 16,777,216, the final per-step divisor fits a u32.
 */
#define ZENITH_DEFAULT_INPUT_BOOST_DECAY_CURVE	0
#define ZENITH_DEFAULT_INPUT_BOOST_BIG_ONLY	1
#define ZENITH_DEFAULT_INPUT_BOOST_CAP_PCT	80	/* 0 = no cap, pin to policy->max */
#define ZENITH_DEFAULT_EFFICIENT_FREQ		0

/* eff_bin_hyst_pct (default 0, off):
 *
 * The efficient_freq ladder releases a bin (and resets every higher
 * bin's wait-deadline) the moment the requested target_freq drops
 * to or below the bin's freq.  When userspace load sits right at a
 * bin boundary -- a 6 Hz frame-pacing thread asking for almost
 * exactly the bin freq -- this turns into ping-pong:
 *
 *   eval N   target = bin_freq + 1    arm bin, hold at bin_freq
 *   eval N+1 target = bin_freq        clear deadline, break
 *   eval N+2 target = bin_freq + 1    re-arm bin (full delay again)
 *   ...
 *
 * The bin never actually unlocks because the deadline keeps getting
 * reset.  The cluster sits one rung below where it should be.
 *
 * eff_bin_hyst_pct adds a release margin per bin: target must drop
 * to bin_freq * (100 - eff_bin_hyst_pct) / 100 before the deadline
 * is cleared.  Targets between that release threshold and bin_freq
 * land in a hysteresis band: the bin is *not* released (deadline is
 * preserved, so a re-cross doesn't have to re-arm) but the target
 * is also *not* held at bin_freq (so the operator gets the slightly
 * lower freq they asked for).  This breaks the ping-pong without
 * pinning the cluster up to the bin.
 *
 * Range 0..20.  20% is a generous upper bound -- bin spacing in
 * real freq tables tends to be larger than that, so a value of 20
 * gives the full hysteresis band; values higher would just clip to
 * the bin below.  0 disables the band entirely (legacy: any drop
 * to-or-below bin_freq releases the deadline).
 */
#define ZENITH_DEFAULT_EFF_BIN_HYST_PCT		0
#define ZENITH_EFF_BIN_HYST_PCT_MAX		20
#define ZENITH_DEFAULT_UP_DELAY_US		4000
#define ZENITH_DEFAULT_LIGHT_LOAD_FREQ		0
#define ZENITH_DEFAULT_LIGHT_LOAD_THRESHOLD	20
#define ZENITH_DEFAULT_SAMPLING_DOWN_FACTOR	2
#define ZENITH_MAX_SAMPLING_DOWN_FACTOR		10
#define ZENITH_DEFAULT_BOOST_EXIT_EXTEND	1	/* stretch down-rate after a boost ends */
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

/* auto_tune_scenario (default 0, off):
 *
 * When auto_tune=1 (the existing classifier worker is running) and
 * auto_tune_scenario=1, zenith_auto_tune_work() also samples the
 * detected scenario at classification time -- audio-thread enqueued,
 * camera HAL active (via the camera_active override or the comm
 * walk), render-thread active, and PSI memory-stall above
 * psi_mem_thresh -- and lets that scenario *override* the
 * load-saturation + input-rate target the vanilla classifier picks:
 *
 *   camera | render -> ZENITH_PROFILE_PERFORMANCE
 *   memstall (and not camera/render) -> ZENITH_PROFILE_BATTERY
 *   audio (and not camera/render/memstall) -> ZENITH_PROFILE_BALANCED
 *   no scenario -> classifier output unchanged
 *
 * The intent is the obvious one: when the device is actively
 * filming or driving a render pipeline, the user almost certainly
 * wants PERFORMANCE regardless of what the input-rate classifier
 * thinks; when it's purely playing audio in the background, BALANCED
 * is the natural floor (BATTERY would risk underrun); when memory
 * pressure is high the cycles wasted on stalls aren't worth the
 * energy.  The scenarios have a strict precedence (camera/render
 * beats memstall beats audio) to keep behaviour deterministic.
 *
 * Detection at classification time is a snapshot, not a window
 * average -- the comm walk caches (4 ms TTL each) reflect what's
 * running at the auto_tune sample point.  In practice that catches
 * the common case of "user opened the camera 8 seconds ago" cleanly
 * because the cameraserver / mtkcam-* processes stay enqueued.  For
 * scenarios that have already wound down by the moment we sample,
 * the classifier still picks via load and input-rate.
 *
 * Requires auto_tune=1.  Independent of audio_aware / camera_aware /
 * render_aware: the comm walks fire even when those gating flags
 * are 0, because here we're using them as detection signals, not as
 * floor/cap policy.  No KMI exposure.
 */
#define ZENITH_DEFAULT_AUTO_TUNE_SCENARIO	0

/* auto_tune_v2 safety layer (default 0, off):
 *
 * The legacy auto_tune path applies whole profile presets from one
 * classification window.  V2 keeps the same observer but adds a
 * bounded state machine: candidate targets must repeat for a small
 * hysteresis window, profile-specific guardrails clamp every automatic
 * write, and user-written knobs are skipped until the operator resets
 * overrides or changes profile.
 *
 * 0 preserves the legacy classifier path.  1 enables the bounded V2
 * state/actions without changing KMI or tracepoint ABI.
 */
#define ZENITH_DEFAULT_AUTO_TUNE_V2		0
#define ZENITH_DEFAULT_AT_HYSTERESIS_WINDOWS	2
#define ZENITH_DEFAULT_AT_COOLDOWN_WINDOWS	1
#define ZENITH_AT_HYSTERESIS_WINDOWS_MAX	8
#define ZENITH_AT_COOLDOWN_WINDOWS_MAX		8

#define ZENITH_DEFAULT_AT_CLUSTER_AWARE		1
#define ZENITH_DEFAULT_AT_V2_SIGNALS		1
#define ZENITH_DEFAULT_AT_THERMAL_SLOPE		1
#define ZENITH_DEFAULT_AT_THERMAL_PRESSURE_PCT	18
#define ZENITH_DEFAULT_AT_THERMAL_SLOPE_PCT	4
#define ZENITH_DEFAULT_AT_FRAME_PACING		1
#define ZENITH_DEFAULT_AT_SUSTAINED_GAMING	1
#define ZENITH_AT_THERMAL_PCT_MAX		100

#define ZENITH_AT_STATE_EFFICIENCY		0
#define ZENITH_AT_STATE_BALANCED		1
#define ZENITH_AT_STATE_LATENCY			2
#define ZENITH_AT_STATE_SUSTAINED_PERF		3
#define ZENITH_AT_STATE_THERMAL_RECOVERY	4

#define ZENITH_AT_REASON_CLASSIFIER		0
#define ZENITH_AT_REASON_CAMERA_RENDER		1
#define ZENITH_AT_REASON_MEMSTALL		2
#define ZENITH_AT_REASON_AUDIO			3
#define ZENITH_AT_REASON_VARIANCE		4
#define ZENITH_AT_REASON_THERMAL		5
#define ZENITH_AT_REASON_COOLDOWN		6
#define ZENITH_AT_REASON_HYSTERESIS		7
#define ZENITH_AT_REASON_SCREEN			8
#define ZENITH_AT_REASON_PSI			9
#define ZENITH_AT_REASON_FRAME			10
#define ZENITH_AT_REASON_GAME			11
#define ZENITH_AT_REASON_THERMAL_SLOPE		12

#define ZENITH_AT_FLAG_AUDIO			(1U << 0)
#define ZENITH_AT_FLAG_CAMERA			(1U << 1)
#define ZENITH_AT_FLAG_RENDER			(1U << 2)
#define ZENITH_AT_FLAG_MEMSTALL			(1U << 3)
#define ZENITH_AT_FLAG_THERMAL			(1U << 4)
#define ZENITH_AT_FLAG_SCREEN_OFF		(1U << 5)
#define ZENITH_AT_FLAG_PSI_CPU			(1U << 6)
#define ZENITH_AT_FLAG_PSI_IO			(1U << 7)
#define ZENITH_AT_FLAG_FRAME			(1U << 8)
#define ZENITH_AT_FLAG_GAME			(1U << 9)
#define ZENITH_AT_FLAG_THERMAL_SLOPE		(1U << 10)
#define ZENITH_AT_FLAG_LOCAL_ACTIONS		(1U << 11)
/* Set by the V1 classifier worker when prefer_silver_aware is on AND
 * the prefer_silver hit-rate over the last classifier window crossed
 * the prefer_silver_hot_threshold_pct cutoff.  Read-only signal; the
 * actual dynamic_up_thresh bump is applied directly in
 * zenith_get_next_freq() (the signal does not feed the V2 state
 * machine because prefer_silver redistribution is workload-dependent
 * and would race with the existing thermal / PSI / frame triggers).
 */
#define ZENITH_AT_FLAG_PREFER_SILVER_HOT	(1U << 12)

#define ZENITH_AT_OVERRIDE_UP_RATE		(1UL << 0)
#define ZENITH_AT_OVERRIDE_DOWN_RATE		(1UL << 1)
#define ZENITH_AT_OVERRIDE_UP_THRESHOLD		(1UL << 2)
#define ZENITH_AT_OVERRIDE_DOWN_THRESHOLD	(1UL << 3)
#define ZENITH_AT_OVERRIDE_INPUT_BOOST_MS	(1UL << 4)
#define ZENITH_AT_OVERRIDE_INPUT_BOOST_CAP	(1UL << 5)
#define ZENITH_AT_OVERRIDE_DOWN_ADAPTIVE	(1UL << 6)
#define ZENITH_AT_OVERRIDE_DOWN_THRESH_ADAPTIVE	(1UL << 7)
#define ZENITH_AT_OVERRIDE_FRAME_PACE		(1UL << 8)
#define ZENITH_AT_OVERRIDE_GAME_MODE		(1UL << 9)

#define ZENITH_CLUSTER_LITTLE			0
#define ZENITH_CLUSTER_BIG			1
#define ZENITH_CLUSTER_PRIME			2

/* kcpustat-derived hispeed-floor blend (see cpufreq_zenith.c "kcpustat
 * hispeed blend" section for the algorithm). The feature ships OFF;
 * userspace flips kcpustat_hispeed_enable=1 once trace data shows the
 * blend actually lifts cold-start frequencies on the target SoC.
 * Enabled by default to compensate PELT's 32 ms cold-start lag.
 *
 *   kcpustat_window_us     - observation window in microseconds. Each
 *                            window samples idle / wall delta from
 *                            kcpustat to compute a raw busy_pct (0..100).
 *                            Smaller windows respond faster but get
 *                            noisier. 4 ms matches reflex's default.
 *   kcpustat_filter_shift  - asymmetric EWMA shift on busy_pct. Up
 *                            transitions are instant; down transitions
 *                            decay by (filtered - measured) >> shift
 *                            per window. 0 disables the EWMA.
 *   kcpustat_hispeed_enable- master gate. 0 (default) = sampler runs
 *                            for tracing but does not influence freq.
 *                            1 = blend the decayed kcpustat util into
 *                            the PELT util consumed by every tier of
 *                            zenith_get_next_freq().
 */
#define ZENITH_DEFAULT_KCPUSTAT_WINDOW_US	4000
#define ZENITH_DEFAULT_KCPUSTAT_FILTER_SHIFT	1
#define ZENITH_DEFAULT_KCPUSTAT_HISPEED_ENABLE	1

/* util_math_v2 (default 1): when 1, zenith_get_util() folds the cfs_rq
 * runnable_avg into the util signal alongside util_avg / util_est, in
 * the same shape as 6.x cpu_util_cfs_boost(). Helps intermittent
 * tasks (UI thread + render thread spikes) without changing PELT or
 * util_est semantics. Enabled by default for better responsiveness
 * to short burst workloads (UI/render threads).
 */
#define ZENITH_DEFAULT_UTIL_MATH_V2		1

/* uclamp_min_respect (default 1): make zenith honour ADPF-style
 * uclamp_min hints more robustly than the stock schedutil_cpu_util()
 * path alone.  Two behaviours are gated by this tunable:
 *
 *   (a) an explicit final-freq floor of map_util_freq(uclamp_min_eff)
 *       applied after every other decision tier (powersave_bias,
 *       light_load_freq cap, efficient_freq ladder), so rate-limit
 *       windows and soft caps cannot erode ADPF hints;
 *   (b) when screen_state == 0, the aggressive
 *       dynamic_bias = 500 / up_threshold = 95 screen-off override is
 *       suppressed if the policy's effective uclamp_min is at least
 *       ZENITH_UCLAMP_MIN_MEANINGFUL_PCT of SCHED_CAPACITY_SCALE.
 *       This keeps Android's PerformanceHint sessions effective for
 *       legitimate screen-off work (audio decode, nav, sync) without
 *       opening the screen-off budget for every task.
 *
 * Set 0 to revert to the pre-patch behaviour (uclamp_min still flows
 * through schedutil_cpu_util() via the RQ aggregate, but no explicit
 * floor / screen-off suppression is applied on top).
 */
#define ZENITH_DEFAULT_UCLAMP_MIN_RESPECT	1

/* predict_util_pct (default 0, off): when non-zero (1..100), zenith
 * applies a lightweight one-step-ahead linear predictor to the
 * util signal returned by zenith_get_util() before it is consumed by
 * zenith_get_next_freq().  The predictor is
 *
 *    delta = util - prev_util
 *    pred  = util + (delta * predict_util_pct / 100)   // clamped to max
 *    util' = max(util, pred)                            // up-only
 *
 * The intent is to dampen the sawtooth pattern where the governor
 * undershoots a transient ramp by one sample window and chases it
 * across 3-4 windows before catching up.  Up-only ensures we never
 * predict the load below what we actually observed -- ramp-down
 * stays purely PELT-driven.  prev_util is held per zenith_cpu and
 * tracks the *unpredicted* value to keep delta a true sample-to-
 * sample derivative.
 *
 * 0 (default) disables the predictor; the 1..100 range covers the
 * useful spectrum where 50 is half-step-ahead, 100 is one-step-
 * ahead.  Values >100 are accepted but rarely helpful (the predictor
 * gets noisy on small deltas).
 */
#define ZENITH_DEFAULT_PREDICT_UTIL_PCT		0
#define ZENITH_PREDICT_UTIL_PCT_MAX		200

/* predict_util_smooth (default 0, off):
 *
 * The base predictor at ZENITH_DEFAULT_PREDICT_UTIL_PCT uses a
 * single-tap slope:
 *
 *   slope = util - prev_util
 *   pred  = util + slope * pct / 100
 *
 * A single-tap slope is jumpy: a one-sample outlier in util
 * produces a full-strength prediction on the very next tick.
 *
 * When set to 1, average the slope across the two most recent
 * taps:
 *
 *   slope1 = util       - prev_util
 *   slope2 = prev_util  - prev_util_2
 *   pred   = util + ((slope1 + slope2) / 2) * pct / 100
 *
 * Both slopes are computed only when strictly positive, so the
 * "up-only" invariant of the base predictor is preserved (a
 * ramp-down is never amplified).  prev_util_2 is maintained as
 * long as predict_util_pct > 0; toggling smooth on/off is free.
 *
 * Same cap / clamp / trace / prev_util update semantics as the
 * base predictor, so reverting to 0 returns to the historical
 * single-tap math with no lingering state.
 */
#define ZENITH_DEFAULT_PREDICT_UTIL_SMOOTH	0

/* render_aware (default 0, off) + render_floor_pct (default 70):
 *
 * When render_aware=1, zenith_get_next_freq() walks the policy's
 * cpumask and checks each cpu_curr(cpu)->comm against a small list of
 * known render / display-pipeline thread names.  If any matches and
 * render_floor_pct > 0, the final freq is floored at
 *
 *     policy->max * render_floor_pct / 100
 *
 * before the rate-limit gate.  The intent is to keep frame-pacing
 * threads on a frequency tier that delivers their next frame's
 * deadline rather than ramping up only after PELT catches up.
 *
 * The comm check is cached per-policy with a short TTL
 * (ZENITH_RENDER_CACHE_TTL_NS) so the strncmp loop runs at most once
 * every few milliseconds, well above scroll-frame budgets but cheap
 * enough that a hot scroll path doesn't spend any meaningful time on
 * comm matching.
 *
 * Set render_aware=0 to fully disable the feature (no walks, no
 * cache, no floor).  Set render_floor_pct=0 to leave the comm walk
 * running (for tracepoints) but apply no floor.
 */
#define ZENITH_DEFAULT_RENDER_AWARE		0
#define ZENITH_DEFAULT_RENDER_FLOOR_PCT		70
#define ZENITH_RENDER_CACHE_TTL_NS		(4 * NSEC_PER_MSEC)

/* game_mode (default 0, off):
 *
 * When game_mode=1, zenith applies two lightweight runtime overlays
 * that together match the heuristics most game-detection daemons
 * (Realme TouchBoost, OnePlus GameSpace, etc.) want:
 *
 *  (a) the effective hispeed_freq_pct used by zenith_eff_hispeed_freq()
 *      is multiplied by ZENITH_GAME_HISPEED_BOOST_PCT/100 -- by
 *      default a 10%% lift of the per-cluster auto-default floor;
 *  (b) the effective input_boost_decay_ms is multiplied by
 *      ZENITH_GAME_BOOST_DECAY_PCT/100 -- by default 130%%, so the
 *      trailing decay window is ~30%% longer to keep frametime
 *      stable across stick-flick / camera-pan inputs.
 *
 * The tunable accepts 0/1/2; userspace (a small gameswitch helper,
 * or a Realme `/proc/touchpanel/game_switch_enable` watcher) is
 * expected to flip it.  Level 2 ("turbo") stacks additional
 * runtime overrides on top of level 1 -- see the comment block
 * attached to ZENITH_GAME_L2_HISPEED_BOOST_PCT for the full list.
 * Values >2 are rejected by sysfs with -EINVAL.
 */
#define ZENITH_DEFAULT_GAME_MODE			0
#define ZENITH_GAME_HISPEED_BOOST_PCT		110
#define ZENITH_GAME_BOOST_DECAY_PCT		130

/* game_mode=2 ("turbo") stacks the game_mode=1 overlays on top of:
 *
 *   - a stronger hispeed boost multiplier (default 120%% vs 110%%),
 *   - a longer input_boost decay window (default 160%% vs 130%%),
 *   - effective input_boost_cap_pct treated as 0 (pin to policy->max
 *     during the full-boost phase regardless of the user-set cap),
 *   - effective climb_mode treated as SNAP for the brutality tier
 *     regardless of the user-set climb_mode.
 *
 * The stacked overrides are applied inline in the hot path and do
 * not mutate the underlying tunables, so switching back to 0 or 1
 * restores the user's original climb_mode / input_boost_cap_pct
 * bit-exactly.  Only the level 1 overlays (ZENITH_GAME_*_PCT above)
 * apply at game_mode=1; level 2 is a strict superset.
 */
#define ZENITH_GAME_L2_HISPEED_BOOST_PCT	120
#define ZENITH_GAME_L2_BOOST_DECAY_PCT		160
#define ZENITH_GAME_MODE_MAX			2

/* frame_budget_us (default 0, off) + frame_pace_floor_pct (default 0):
 *
 * Userspace-driven, lock-free, KMI-clean alternative to a real DRM
 * vblank hook.  The intent is the same as a frame-pacing governor:
 * keep the freq high enough that the render pipeline is never the
 * critical path of a frame's compute deadline.  The implementation
 * differs from real frame pacing in that nothing in the kernel sees
 * vblank events directly.  Instead, userspace (a small daemon that
 * watches /sys/class/drm/card0-DSI-1/vrefresh, or SurfaceFlinger if
 * patched, or the existing Realme display HAL) writes the current
 * vblank period in microseconds to frame_budget_us whenever the
 * panel switches refresh rate.
 *
 *   60 Hz   -> echo 16667 ...
 *   90 Hz   -> echo 11111 ...
 *  120 Hz   -> echo  8333 ...
 *  144 Hz   -> echo  6944 ...
 *  off      -> echo     0 ...
 *
 * The floor itself is computed adaptively: shorter budgets need a
 * higher floor because the same compute must finish in less wall
 * time.  The formula is
 *
 *   eff_pct = frame_pace_floor_pct * 16667 / frame_budget_us
 *   floor   = policy->max * min(eff_pct, 100) / 100
 *
 * so frame_pace_floor_pct is the *60 Hz baseline*; it auto-scales
 * upward at higher refresh rates without userspace re-tuning.
 *
 * frame_budget_us = 0 disables the feature regardless of
 * frame_pace_floor_pct.  frame_pace_floor_pct = 0 disables the
 * floor while keeping the budget set (useful for tracing the
 * tracepoint without changing freq).
 *
 * Floor is capped by uclamp_max downstream so an explicit ADPF
 * power-efficiency hint still wins.
 */
#define ZENITH_DEFAULT_FRAME_BUDGET_US		0
#define ZENITH_FRAME_BUDGET_US_MAX		50000

/* frame_budget_us_per_policy (default empty, off):
 *
 * frame_budget_us is global -- one vblank period applied to every
 * policy.  On big.LITTLE / 3-cluster SoCs the right value usually
 * differs per cluster: little wants 0 (no adaptive floor at all,
 * the cluster idles between frames), big wants the full 16667 /
 * 8333 / 6944 us depending on display refresh rate, prime in the
 * middle.  A single global value forces userspace to either over-
 * or under-floor at least one cluster.
 *
 * frame_budget_us_per_policy is a CSV override read as
 *
 *   anchor_cpu:budget_us[,anchor_cpu:budget_us]...
 *
 * where anchor_cpu is cpumask_first(policy->cpus) -- the same
 * "policyN" identifier the cpufreq sysfs tree already uses.  Stored
 * as a fixed-size array indexed by cpu, parsed once on store and
 * read lock-free in zenith_get_next_freq().  A non-zero entry for
 * the policy's anchor cpu overrides the global frame_budget_us
 * for that policy; a zero entry (the default) falls through to
 * the global value, preserving today's shape on every policy that
 * isn't called out in the CSV.
 *
 * Empty string clears all overrides.  Bounds: anchor_cpu < NR_CPUS,
 * budget_us <= ZENITH_FRAME_BUDGET_US_MAX.  Same disable semantics
 * as frame_budget_us itself: 0 means "no adaptive floor" for that
 * policy.
 */
#define ZENITH_DEFAULT_FRAME_PACE_FLOOR_PCT	0
#define ZENITH_FRAME_PACE_BASE_BUDGET_US	16667

/* psi_aware (default 0, off) + psi_mem_thresh (default 50)
 * + psi_cpu_thresh (default 0, off) + psi_io_thresh (default 0, off):
 *
 * When psi_aware=1, zenith_get_next_freq() reads the system-wide
 * pressure 10s averages from psi_system.avg[PSI_*_SOME][0].  Each
 * dimension has its own integer-percentage threshold; if the live
 * average is at or above the threshold for that dimension, the final
 * freq is *capped* at the effective hispeed floor.  Rationale:
 *
 *  - PSI_MEM_SOME: heavy memory stall.  Pushing above hispeed wastes
 *    energy on cycles that mostly stall waiting for memory; the
 *    workload is memory-bound, not compute-bound.
 *
 *  - PSI_CPU_SOME: oversubscribed runqueue.  More than one task is
 *    waiting on the cpu; ramping the freq above hispeed doesn't make
 *    runnable tasks run, it just burns power on the one that *is*
 *    running.  Useful on big.LITTLE where a small cluster gets piled
 *    on by a wakeup storm before the load balancer migrates anything.
 *
 *  - PSI_IO_SOME: I/O-bound stall.  The cpu is waiting on storage or
 *    block I/O; freq ramps don't reduce wait time.  Symmetric with
 *    PSI_MEM_SOME.
 *
 * When the hispeed tier is disabled (eff_hispeed == 0) the cap is
 * policy->max -- i.e. a no-op fallback.  The three caps stack: any
 * dimension over-threshold lowers the freq to the hispeed floor (we
 * don't subtract three times; the cap is at most one floor down).
 *
 * The reader is RCU-free and lock-free: psi_system.avg[][] is updated
 * by the avgs_work delayed work and a single READ_ONCE is sufficient
 * to get a coherent fixed-point value.  When CONFIG_PSI is off or
 * psi_disabled is set, the helper returns 0 and the caps never fire.
 *
 * 0..100 range; values >100 rejected by sysfs.  0 disables the cap
 * for that dimension even with psi_aware=1 (useful for tracing the
 * helpers without changing freq).  psi_cpu_thresh / psi_io_thresh
 * default to 0 so out-of-box behaviour matches pre-N1: only the mem
 * cap fires when psi_aware=1.
 */
#define ZENITH_DEFAULT_PSI_AWARE		0
#define ZENITH_DEFAULT_PSI_MEM_THRESH		50
#define ZENITH_DEFAULT_PSI_CPU_THRESH		0
#define ZENITH_DEFAULT_PSI_IO_THRESH		0

/* audio_aware (default 0, off) + audio_floor_pct (default 0) +
 * audio_cap_pct (default 0):
 *
 * When audio_aware=1, zenith_get_next_freq() walks the policy's
 * cpumask and checks each cpu_curr(cpu)->comm against a small list
 * of known Android audio-thread names (AudioOut_*, audioserver,
 * MediaCodec_*, OMX*, SoundPool, ...).  If any matches, the final
 * freq is clamped into a configurable band:
 *
 *     floor = policy->max * audio_floor_pct / 100   (0 = no floor)
 *     cap   = policy->max * audio_cap_pct   / 100   (0 = no cap)
 *
 * The point is to keep the freq an audio thread is running on as
 * stable as possible.  Audio buffers underrun when freq drops
 * mid-buffer; transient ramps to policy->max waste energy and
 * incur a freq-transition stall that itself can blow a frame.  A
 * modest floor (e.g. 40) prevents the underrun half; a modest cap
 * (e.g. 60) prevents the burst-to-max half.  Either alone is
 * useful; together they form a stable band.
 *
 * The cap is applied *before* the uclamp_max final cap so an
 * explicit ADPF power-efficiency hint can still walk it down
 * further.  The floor is applied alongside the render_floor /
 * frame_pace_floor tier and is subject to the same uclamp_max
 * downstream cap.
 *
 * The comm check is cached per-policy with TTL
 * ZENITH_AUDIO_CACHE_TTL_NS so a hot path (e.g. a 60 / 90 / 120 Hz
 * scroll) only does the strncmp loop a few times per second.
 *
 * Set audio_aware=0 to fully disable the feature (no walks, no
 * cache, no clamp).  Set audio_floor_pct=audio_cap_pct=0 to leave
 * the comm walk running (for tracepoints) but apply no clamp.
 */
#define ZENITH_DEFAULT_AUDIO_AWARE		0
#define ZENITH_DEFAULT_AUDIO_FLOOR_PCT		0
#define ZENITH_DEFAULT_AUDIO_CAP_PCT		0
#define ZENITH_AUDIO_CACHE_TTL_NS		(4 * NSEC_PER_MSEC)

/* camera_aware (default 0, off) + camera_active (default 0, auto)
 * + camera_floor_pct (default 0):
 *
 * When camera_aware=1, zenith_get_next_freq() walks the policy's
 * cpumask and checks each cpu_curr(cpu)->comm against a small list
 * of known Android camera HAL / framework thread names
 * (cameraserver, provider@N.M-se, provider.MTK*, mtkcam-*,
 * Camera2-*, CamX_*, ...).  If any matches, the final freq is
 * floored at (policy->max * camera_floor_pct / 100), mirroring
 * the render_aware tier.
 *
 * Camera workloads benefit from a stable high freq -- the capture
 * pipeline stalls when the freq dips below what its ISP/codec
 * pipeline needs.  No companion cap is provided (unlike audio):
 * camera bursts are short and infrequent, and capping freq there
 * costs frame-rate.
 *
 * The userspace override knob (camera_active) is provided because
 * vendor camera HALs sometimes use thread names that don't match
 * the comm table on every device.  Values:
 *
 *   0 (auto, default)  -- consult comm table
 *   1 (force-on)       -- floor always applied (skip comm walk)
 *   2 (force-off)      -- floor never applied (skip comm walk)
 *
 * The HAL or a Magisk module can write 1 on capture-start /
 * preview-start and 2 (or 0) on capture-stop, giving deterministic
 * behaviour without the kernel having to know every vendor name.
 *
 * The comm check is cached per-policy with TTL
 * ZENITH_CAMERA_CACHE_TTL_NS (mirrored from render).  Set
 * camera_aware=0 to fully disable; set camera_floor_pct=0 to leave
 * comm-walk + tracepoint visibility on but apply no floor.
 */
#define ZENITH_DEFAULT_CAMERA_AWARE		0
#define ZENITH_DEFAULT_CAMERA_ACTIVE		0
#define ZENITH_DEFAULT_CAMERA_FLOOR_PCT		0
#define ZENITH_CAMERA_CACHE_TTL_NS		(4 * NSEC_PER_MSEC)

#define ZENITH_CAMERA_OVERRIDE_AUTO		0
#define ZENITH_CAMERA_OVERRIDE_FORCE_ON		1
#define ZENITH_CAMERA_OVERRIDE_FORCE_OFF	2

/* boot_boost_ms (default 0, off):
 *
 * When non-zero, zenith pins the final freq to policy->max for the
 * first boot_boost_ms milliseconds after system boot.  Gated by
 * screen_state so it only applies after the screen comes on, and
 * skipped on small clusters when input_boost_big_only=1 (the existing
 * input-boost gate).  Intended to absorb the cold-cache, lots-of-zygote
 * boot path without paying for low-freq sample windows that PELT then
 * spends 200 ms catching up out of.
 *
 * boot_boost_ms is a one-shot: zenith_get_next_freq() compares
 * ktime_get_boottime_ns() against boot_boost_ms * NSEC_PER_MSEC and
 * disables the floor for everyone past that deadline.  Setting it to
 * 0 disables the feature; values up to a few minutes are accepted
 * but anything past 60_000 ms is wasteful.
 *
 * Recommended init.rc tune: 30000 (30 s).
 */
#define ZENITH_DEFAULT_BOOT_BOOST_MS		0
#define ZENITH_BOOT_BOOST_MAX_MS		300000

/* uclamp_max_respect (default 1): symmetric counterpart to
 * uclamp_min_respect.  When set, zenith_get_next_freq() applies an
 * explicit final-freq _cap_ derived from the RQ-aggregated uclamp_max
 * just before the ladder/rate-limit stage, mirroring the uclamp_min
 * floor.  This honours Android's PerformanceHint power-efficiency
 * hint (setPreferPowerEfficiency() -> per-task UCLAMP_MAX) for every
 * decision tier including the brutality snap, hispeed floor, and
 * input boost, which would otherwise walk over the cap.
 *
 * Set 0 to revert to pre-patch behaviour (uclamp_max still flows
 * through schedutil_cpu_util() via the RQ aggregate but no explicit
 * final cap is applied on top).
 */
#define ZENITH_DEFAULT_UCLAMP_MAX_RESPECT	1

/* uclamp_min threshold (in percent of SCHED_CAPACITY_SCALE) above
 * which the screen-off override suppression kicks in. 10 %% means the
 * task's ADPF hint has to reach uclamp_min >= ~102/1024 (about big-core
 * idle-loop capacity) before zenith considers it worth bypassing the
 * screen-off penalty.  Hardcoded rather than tunable -- it is a
 * definition of "meaningful", not a policy knob.
 */
#define ZENITH_UCLAMP_MIN_MEANINGFUL_PCT	10

/* kcpustat tunable bounds. window_us is clamped on store to keep the
 * sampler from thrashing or overflowing; filter_shift caps below the
 * width of an unsigned int.
 */
#define ZENITH_KCPUSTAT_WINDOW_MIN_US		1000
#define ZENITH_KCPUSTAT_WINDOW_MAX_US		100000
#define ZENITH_KCPUSTAT_FILTER_SHIFT_MAX	8

/*
 * Zenith Tunables & State API
 */
struct zenith_tunables {
	struct gov_attr_set	attr_set;
	unsigned int		up_rate_limit_us;
	unsigned int		down_rate_limit_us;
	unsigned int		up_threshold;

	/* See ZENITH_DEFAULT_UP_THRESHOLD_ADAPTIVE.  Magnitude of the
	 * variance-adaptive lowering applied to dynamic_up_thresh on
	 * bursty workloads, in percent of the static up_threshold.
	 * Range 0..ZENITH_UP_THRESHOLD_ADAPTIVE_MAX.  0 disables.
	 */
	unsigned int		up_threshold_adaptive;
	unsigned int		down_threshold;	/* hysteresis lower bound */

	/* Hispeed floor tier: when load >= hispeed_load (% of max_cap),
	 * ensure the chosen target_freq is at least hispeed_freq (kHz).
	 * hispeed_freq=0 disables the explicit tier.
	 *
	 * When hispeed_freq=0 and hispeed_freq_pct>0, the tier falls
	 * back to a per-cluster auto-default: the effective hispeed
	 * floor for the policy becomes (policy->max * hispeed_freq_pct
	 * / 100).  This gives sensible defaults on both the little and
	 * big clusters without userspace having to read policyN/max and
	 * write one absolute kHz value per policy.  hispeed_freq_pct=0
	 * preserves the legacy "tier disabled" semantics.
	 */
	unsigned int		hispeed_freq;
	unsigned int		hispeed_freq_pct;
	unsigned int		hispeed_load;

	/* Hysteresis margin (percent of max_cap) applied to hispeed_load
	 * on the _exit_ side of the tier.  Entering the tier still
	 * triggers at load_pct >= hispeed_load; leaving it requires
	 * load_pct < (hispeed_load - hispeed_hyst_pct).  Collapses to
	 * the legacy no-hysteresis behaviour at 0.  See
	 * ZENITH_DEFAULT_HISPEED_HYST_PCT.
	 */
	unsigned int		hispeed_hyst_pct;

	/* Symmetric entry-side hysteresis for the hispeed tier.  See
	 * ZENITH_DEFAULT_HISPEED_ENTRY_STREAK.  Capped on store to
	 * ZENITH_HISPEED_ENTRY_STREAK_MAX so the per-policy u8 counter
	 * cannot overflow.
	 */
	unsigned int		hispeed_entry_streak;

	/* Symmetric entry-side hysteresis for the brutality tier.  See
	 * ZENITH_DEFAULT_BRUTAL_ENTRY_STREAK.  Only gates SNAP climb
	 * mode; STEP mode is unaffected.  Capped on store to
	 * ZENITH_BRUTAL_ENTRY_STREAK_MAX so the per-policy u8 counter
	 * cannot overflow.
	 */
	unsigned int		brutal_entry_streak;

	/* Tail-decay window for the brutal-hold cliff exit, in
	 * milliseconds.  0 (default) preserves the historical hard-exit
	 * behaviour: the moment load_pct drops below the (possibly
	 * adaptive-shaped) eff_down threshold, brutal_active is cleared
	 * and the next sample's freq is whatever the EAS proportional
	 * math returns.  Non-zero arms a linear glide: policy->max at
	 * arm time, decaying toward the EAS-computed freq over
	 * brutal_decay_ms.  Eliminates the audible / visible drop that
	 * otherwise happens at the moment of cliff exit, especially on
	 * loads with bursty PELT signals.  Capped at
	 * ZENITH_BRUTAL_DECAY_MS_MAX in the sysfs store.
	 */
	unsigned int		brutal_decay_ms;

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

	/* Load-proportional scaling for the STEP climb step.  See
	 * ZENITH_DEFAULT_FREQ_STEP_ADAPTIVE.  0 = off (fixed step);
	 * 1 = on (step scales 1.0x .. 2.0x with overshoot).  Only
	 * meaningful when climb_mode == STEP.
	 */
	unsigned int		freq_step_adaptive;

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

	/* Percentage (0..100) of the iowait-derived boost that is added
	 * to util when stacking is active.  0 is the legacy behaviour
	 * (max(util, boost)); 100 is full stacking (util + boost, clamped
	 * to max_cap).  The default 50 is a conservative middle ground:
	 * when a CPU is already under compute load and also servicing
	 * I/O, it gets a half-weighted boost on top of its current util
	 * rather than the original "freq takes whichever is larger" which
	 * loses the I/O signal entirely when util is high.  Uses
	 * upstream 6.x schedutil's observation that a task that's both
	 * CPU-heavy and I/O-heavy needs more headroom than either demand
	 * alone would justify.
	 */
	unsigned int		iowait_stack_pct;

	/* See ZENITH_DEFAULT_IOWAIT_BACKOFF_AFTER_MS.  Time in ms after
	 * the start of a single iowait episode before the doubling-on-
	 * arm climb flips to a halving step.  0 disables the backoff.
	 */
	unsigned int		iowait_backoff_after_ms;

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

	/* When 1, replace the binary "thermal_active -> dynamic_up_thresh
	 * = 90" cliff with a linear ramp from up_threshold (cool, 0%
	 * pressure) to 90 (hot, 100% pressure).  Smooths long-session
	 * thermal throttling so users don't perceive a step in
	 * frequency the moment thermal_active flips on.  Default 0 to
	 * preserve legacy cliff behaviour for existing tunings.
	 */
	unsigned int		thermal_pressure_continuous;

	/* prefer_silver_aware: when 1, on big / prime cluster policies,
	 * raise dynamic_up_thresh by prefer_silver_hot_bump_pct percent
	 * (additive points) whenever the prefer_silver hit-rate over
	 * the last classifier window is at or above
	 * prefer_silver_hot_threshold_pct.  The intent is to reflect
	 * the fact that prefer_silver hides light load from the big
	 * cluster, so the big cluster sees an artificially "lighter"
	 * load and would otherwise downclock more aggressively than it
	 * should.  Little cluster policies are intentionally not bumped
	 * (they are already absorbing the redirected light tasks).
	 *
	 * Default 0 (off).  Has no effect when
	 * CONFIG_SCHED_PREFER_SILVER=n: the worker does not import
	 * prefer_silver_get_hit_miss() in that build, so the cached
	 * hit-rate stays at 0 and the bump never fires regardless of
	 * the tunable value.
	 */
	unsigned int		prefer_silver_aware;
	unsigned int		prefer_silver_hot_threshold_pct;
	unsigned int		prefer_silver_hot_bump_pct;

	/* See ZENITH_DEFAULT_THERMAL_UTIL_DERATE comment block.  When
	 * set, zenith_get_util() scales util_out by the (cap - pressure)
	 * / cap fraction whenever pressure exceeds
	 * ZENITH_THERMAL_DERATE_FLOOR_PCT.  Smooths the thermal dance
	 * by asking for less than the throttled max.
	 */
	unsigned int		thermal_util_derate;

	/* See ZENITH_DEFAULT_THERMAL_DERATE_RATE_PCT.  Magnitude of the
	 * derivative-term scaling applied on top of thermal_util_derate
	 * when pressure is rising sample-to-sample, in percent.  0..100,
	 * 0 disables.  Read lock-free in the derate site.
	 */
	unsigned int		thermal_derate_rate_pct;

	/* See ZENITH_DEFAULT_FREQ_STABILITY_MARGIN_PCT.  Percent of
	 * policy->max below which tiny downward transitions are held at
	 * the current request.  0 disables; range 0..10.
	 */
	unsigned int		freq_stability_margin_pct;

	/* See ZENITH_DEFAULT_DOWN_RATE_ADAPTIVE.  When set, bursty load
	 * variance stretches the effective down-rate delay up to 2x.
	 */
	unsigned int		down_rate_adaptive;

	/* See ZENITH_DEFAULT_WAKEUP_BOOST.  When set, idle-to-busy
	 * transitions arm a short up-rate bypass countdown.
	 */
	unsigned int		wakeup_boost;

	/* See ZENITH_DEFAULT_DOWN_THRESHOLD_ADAPTIVE.  Percent by which
	 * bursty load can lower the brutality exit threshold.  0 disables.
	 */
	unsigned int		down_threshold_adaptive;

	/* See ZENITH_DEFAULT_RATE_LIMIT_CLUSTER_SCALE.  When set, little
	 * clusters use asymmetric cached up/down rate delays.
	 */
	unsigned int		rate_limit_cluster_scale;

	/* Input boost duration (ms). 0 = disabled. */
	unsigned int		input_boost_ms;
	unsigned int		input_boost_decay_ms;

	/* Shape of the input_boost decay-phase floor.  0 = linear
	 * (legacy), 1 = cubic ease-in (floor holds high, drops fast at
	 * the tail).  See ZENITH_DEFAULT_INPUT_BOOST_DECAY_CURVE for
	 * semantics.  Range-checked 0..1 on store.
	 */
	unsigned int		input_boost_decay_curve;

	/* When 1 (default), input boost is applied only to policies whose
	 * top arch_scale_cpu_capacity equals SCHED_CAPACITY_SCALE -- i.e.
	 * the system's highest-capacity cluster(s).  On a homogeneous SoC
	 * every policy matches and behaviour is unchanged.  On big /
	 * little, this stops every touch from dragging the little cluster
	 * to policy->max, which rarely helps UI latency (UI / render run
	 * on big) and wastes energy on a cluster that is almost always in
	 * light-load territory at the moment of a tap.
	 *
	 * Set 0 to restore pre-patch behaviour (boost every policy).
	 */
	unsigned int		input_boost_big_only;

	/* Per-profile cap on the input-boost ceiling, expressed as a
	 * percentage of policy->max.  When non-zero, the full-pin phase
	 * targets policy->max * input_boost_cap_pct / 100 instead of
	 * policy->max, and the trailing decay phase ramps from that
	 * capped ceiling down to policy->min over input_boost_decay_ms.
	 * 0 (legacy / PERFORMANCE) means "no cap" -- pin to policy->max.
	 * Sized per-profile so BALANCED / BATTERY get a moderate boost
	 * (PELT can still climb past it under genuine load via the
	 * normal eval tiers, since the boost only sets a floor in the
	 * decay window) without spending the energy of a max-pin on
	 * every tap.  Range 0..100; values > 100 rejected by sysfs.
	 */
	unsigned int		input_boost_cap_pct;

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

	/* See ZENITH_DEFAULT_EFF_BIN_HYST_PCT.  Per-bin release margin
	 * for the efficient_freq ladder, in percent.  0..20.  0 keeps
	 * the legacy "any drop releases" shape.
	 */
	unsigned int		eff_bin_hyst_pct;
	unsigned int		eff_freq[ZENITH_EFF_BINS_MAX];
	unsigned int		eff_delay_us[ZENITH_EFF_BINS_MAX];

	/* Light-load hard cap. light_load_freq=0 disables. */
	unsigned int		light_load_freq;
	unsigned int		light_load_threshold;	/* in % of max_cap */

	/* Hold-at-max multiplier for down_rate_limit. 1 = disabled. */
	unsigned int		sampling_down_factor;

	/* When 1 (default), keep the sampling_down_factor multiplier in
	 * effect for one stretched down-rate window after an input boost
	 * exits, even after target_freq has fallen below policy->max.  The
	 * stretched window is sampling_down_factor * down_rate_limit_us
	 * long.  Mirrors the rationale for sit-at-max stretching: PELT
	 * needs a moment to catch up after a synthetic peak (the boost),
	 * and dropping the multiplier the instant target falls produces
	 * a tail-stutter on gestures whose load profile is bursty around
	 * the boost expiry.  Set 0 to restore pre-patch behaviour.
	 */
	unsigned int		boost_exit_extend;

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

	/* See ZENITH_DEFAULT_AUTO_TUNE_V2.  V2 keeps auto-tune bounded by
	 * profile guardrails, hysteresis/cooldown and user override masks.
	 */
	unsigned int		auto_tune_v2;
	unsigned int		auto_tune_hysteresis_windows;
	unsigned int		auto_tune_cooldown_windows;
	unsigned long		auto_tune_override_mask;
	unsigned int		auto_tune_cluster_aware;
	unsigned int		auto_tune_v2_signals;
	unsigned int		auto_tune_thermal_slope;
	unsigned int		auto_tune_thermal_pressure_pct;
	unsigned int		auto_tune_thermal_slope_pct;
	unsigned int		auto_tune_frame_pacing;
	unsigned int		auto_tune_sustained_gaming;

	/* See ZENITH_DEFAULT_AUTO_TUNE_SCENARIO comment block.  Master
	 * gate for the scenario overlay applied on top of the vanilla
	 * load + input-rate classifier in zenith_auto_tune_work().
	 * Requires auto_tune=1; ignored otherwise.
	 */
	unsigned int		auto_tune_scenario;

	/* kcpustat hispeed blend tunables (see ZENITH_DEFAULT_KCPUSTAT_*
	 * comments for semantics). All three default to safe values:
	 * sampler runs at 4 ms windows with EWMA shift=1, but the blend
	 * is gated off until userspace flips kcpustat_hispeed_enable.
	 */
	unsigned int		kcpustat_window_us;
	unsigned int		kcpustat_filter_shift;
	unsigned int		kcpustat_hispeed_enable;

	/* util_math_v2: 0 (default) keeps the historical
	 * cpu_util_cfs() input unchanged; 1 enables the
	 * 6.x-style runnable-aware util computation in
	 * zenith_get_util().
	 */
	unsigned int		util_math_v2;

	/* uclamp_min_respect: see the ZENITH_DEFAULT_UCLAMP_MIN_RESPECT
	 * block at the top of this file for full semantics. In short:
	 *   1 (default) = apply an explicit final-freq floor derived
	 *                  from RQ-aggregated uclamp_min, AND suppress
	 *                  the screen-off penalty when a meaningful
	 *                  uclamp_min is set;
	 *   0           = rely solely on schedutil_cpu_util() RQ uclamp
	 *                  aggregation (pre-G.1 behaviour).
	 */
	unsigned int		uclamp_min_respect;
	unsigned int		uclamp_max_respect;

	/* See ZENITH_DEFAULT_PREDICT_UTIL_PCT comment block. 0 = off. */
	unsigned int		predict_util_pct;

	/* See ZENITH_DEFAULT_PREDICT_UTIL_SMOOTH comment block.  0 = off
	 * (single-tap), 1 = on (two-tap averaged slope).  Ignored when
	 * predict_util_pct == 0.
	 */
	unsigned int		predict_util_smooth;

	/* See ZENITH_DEFAULT_RENDER_AWARE / ZENITH_DEFAULT_RENDER_FLOOR_PCT. */
	unsigned int		render_aware;
	unsigned int		render_floor_pct;

	/* See ZENITH_DEFAULT_GAME_MODE. 0/1, normalised on store. */
	unsigned int		game_mode;

	/* See ZENITH_DEFAULT_PSI_AWARE / ZENITH_DEFAULT_PSI_MEM_THRESH /
	 * ZENITH_DEFAULT_PSI_CPU_THRESH / ZENITH_DEFAULT_PSI_IO_THRESH.
	 * All thresholds are 0..100 integer percent of the 10s SOME
	 * average.  0 disables that dimension's cap.
	 */
	unsigned int		psi_aware;
	unsigned int		psi_mem_thresh;
	unsigned int		psi_cpu_thresh;
	unsigned int		psi_io_thresh;

	/* See ZENITH_DEFAULT_AUDIO_AWARE / ZENITH_DEFAULT_AUDIO_FLOOR_PCT
	 * / ZENITH_DEFAULT_AUDIO_CAP_PCT.  Both *_pct fields range 0..100;
	 * 0 in either disables that side of the band.
	 */
	unsigned int		audio_aware;
	unsigned int		audio_floor_pct;
	unsigned int		audio_cap_pct;

	/* See ZENITH_DEFAULT_CAMERA_AWARE / ZENITH_DEFAULT_CAMERA_ACTIVE
	 * / ZENITH_DEFAULT_CAMERA_FLOOR_PCT.  camera_active is the
	 * userspace override (0 auto, 1 force-on, 2 force-off);
	 * camera_floor_pct ranges 0..100.
	 */
	unsigned int		camera_aware;
	unsigned int		camera_active;
	unsigned int		camera_floor_pct;

	/* See ZENITH_DEFAULT_BOOT_BOOST_MS. 0 disables the one-shot. */
	unsigned int		boot_boost_ms;

	/* See ZENITH_DEFAULT_FRAME_BUDGET_US. Userspace writes the
	 * current vblank period in microseconds.  0 disables.
	 */
	unsigned int		frame_budget_us;

	/* See ZENITH_DEFAULT_FRAME_BUDGET_US/ frame_budget_us_per_policy
	 * comment block.  Indexed by cpumask_first(policy->cpus).  A
	 * non-zero entry overrides frame_budget_us for that policy;
	 * zero falls through to the global value.  All entries default
	 * to 0 (no override) at tunables_init() time.  Read lock-free
	 * in zenith_get_next_freq() under READ_ONCE; written under
	 * global_tunables_lock by the sysfs store path.
	 */
	unsigned int		frame_budget_us_per_policy[NR_CPUS];
	unsigned int		frame_pace_floor_pct;
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

/* Per-policy decision-stat buckets exposed via the readonly
 * `zenith_stats` sysfs node.  See struct zenith_policy::stats[] for
 * the storage and zenith_path_to_bucket() for the tp_path -> bucket
 * mapping.  ZENITH_STAT_OTHER is a catch-all so a future tier added
 * without a bucket mapping still gets counted (against decisions,
 * but not against any specific tier).
 */
enum zenith_stat_idx {
	ZENITH_STAT_DECISIONS,		/* total get_next_freq() calls */
	ZENITH_STAT_CACHE_HITS,		/* cached_raw_freq early-return */
	ZENITH_STAT_INPUT_BOOST,	/* input_boost / input_boost_decay */
	ZENITH_STAT_BRUTAL,		/* snap_max / brutal_hold / climb_step */
	ZENITH_STAT_HISPEED,		/* hispeed */
	ZENITH_STAT_FRAME_PACE,		/* frame_pace */
	ZENITH_STAT_AUDIO,		/* audio_floor / audio_cap */
	ZENITH_STAT_RENDER_CAMERA,	/* render_floor / camera_floor */
	ZENITH_STAT_UCLAMP,		/* uclamp_min_floor / uclamp_max_cap */
	ZENITH_STAT_PSI,		/* psi_*_cap */
	ZENITH_STAT_BOOT_BOOST,		/* boot_boost */
	ZENITH_STAT_LIGHT_CAP,		/* light_cap */
	ZENITH_STAT_EM_CAP,		/* em_cap */
	ZENITH_STAT_EAS,		/* fall-through, no override */
	ZENITH_STAT_OTHER,		/* unmapped tp_path */
	ZENITH_STAT_NR
};

/* One sample written by the auto-tune classifier worker into the
 * per-policy at_log ring (see ZENITH_AT_LOG_NR).  Mirrors the
 * at_last_* mirrors on struct zenith_policy at the moment the worker
 * resolved the new V1 target / V2 state, so userspace can correlate a
 * decision against the signals that drove it without bpftrace.
 */
struct zenith_at_log_entry {
	u64		ts_ns;
	u32		flags;			/* ZENITH_AT_FLAG_* mask */
	u32		var_x256;
	u32		thermal_pressure;	/* 0..1024 */
	u16		sat_pct;		/* 0..100 */
	u16		events_rate_x2;
	u16		thermal_slope;		/* signed-stored-as-u16 */
	u8		v1_target;		/* enum profile id */
	u8		v2_from_state;
	u8		v2_to_state;
	u8		reason;			/* enum at_reason */
	u8		emergency;		/* 1 if state_overridden by slope */
	u8		_pad[3];
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

	/* Hysteresis state for the hispeed tier (step 2b).  Sticky: goes
	 * true when load_pct first crosses hispeed_load with freq below
	 * the effective hispeed floor; stays true while load_pct stays
	 * above (hispeed_load - hispeed_hyst_pct).  Drops back to false
	 * only when the margin is crossed.  Prevents the tier from
	 * flapping in/out on noisy load trajectories that dance around
	 * hispeed_load, the same way brutal_active does for up_threshold.
	 */
	bool			hispeed_active;

	/* Entry-side streak counter for hispeed activation.  Increments
	 * on every sample where load_pct >= hispeed_load, resets when
	 * load_pct < hispeed_load.  hispeed_active flips to true only
	 * when streak > tunables->hispeed_entry_streak, providing
	 * symmetric entry/exit hysteresis.  Capped to a small u8 to
	 * avoid wraparound on long sustained-load runs.
	 */
	u8			hispeed_entry_count;

	/* Entry-side streak counter for brutality activation.  Mirror of
	 * hispeed_entry_count: increments on every sample where
	 * load_pct >= up_threshold while brutal_active is false, resets
	 * when load_pct drops below up_threshold OR when brutal_active
	 * flips true (past entry, no more entry credit to accumulate).
	 * Gates the SNAP snap-to-max path only when
	 * tunables->brutal_entry_streak > 0.  Capped to a small u8 to
	 * avoid wraparound on sustained-above-threshold runs that never
	 * quite reach the streak cap (shouldn't happen with the default
	 * cap of 16 but defensive against future cap bumps).
	 */
	u8			brutal_entry_count;

	/* Time-bounded cache for the per-policy uclamp_{min,max}
	 * aggregations.  Each walk is O(n_cpus_in_policy) rq reads
	 * (cheap, no locks, no cachelines dirtied) but the eval path
	 * can run every rate_limit_us, i.e. 10 000 Hz on up_rate path;
	 * caching for ZENITH_UCLAMP_CACHE_TTL_NS drops the rq walks by
	 * ~10x on an 8-CPU policy with zero user-visible staleness.
	 * Valid when uclamp_cache_valid_ns != 0 and now < valid_ns + TTL.
	 */
	unsigned long		cached_uclamp_min;
	unsigned long		cached_uclamp_max;
	u64			uclamp_cache_stamp_ns;

	/* Cached nice-load ratio for the policy (0..100). Sampled in
	 * the update_util hook when ignore_nice_load=1 and read from
	 * zenith_get_next_freq. Guarded by update_lock in the shared
	 * path; on the single path only one CPU writes it.
	 */
	unsigned int		nice_pct;

	/* Auto-tune observer state. Counters are incremented in the
	 * hot path when tunables->auto_tune=1; the delayed_work handler
	 * samples and resets them every ZENITH_AUTO_TUNE_PERIOD_MS and
	 * chooses a preset.
	 *
	 * They are atomic because the single-CPU update path runs
	 * zenith_get_next_freq() without holding update_lock, while the
	 * delayed_work handler reads+resets them (and runs on any CPU).
	 * atomic_inc in the hot path and atomic_xchg(..., 0) in the
	 * worker give us lockless correctness for the sample window.
	 */
	atomic_t		at_samples_total;
	atomic_t		at_samples_saturated;
	u64			at_last_events;
	unsigned int		at_last_total;
	unsigned int		at_last_saturated;
	unsigned int		at_last_sat_pct;
	unsigned int		at_last_events_rate_x2;
	unsigned int		at_last_target;
	unsigned int		at_last_state;
	unsigned int		at_pending_state;
	unsigned int		at_pending_windows;
	unsigned int		at_cooldown_left;
	unsigned int		at_last_reason;
	unsigned int		at_last_flags;
	unsigned int		at_last_var_x256;
	unsigned int		at_last_psi_cpu;
	unsigned int		at_last_psi_io;
	unsigned int		at_last_psi_mem;
	unsigned int		at_last_thermal_pressure;
	unsigned int		at_last_thermal_slope;
	unsigned int		at_last_frame_budget_us;
	unsigned int		at_effective_up_rate_limit_us;
	unsigned int		at_effective_down_rate_limit_us;
	unsigned int		at_effective_up_threshold;
	unsigned int		at_effective_down_threshold;
	unsigned int		at_effective_input_boost_ms;
	unsigned int		at_effective_input_boost_cap_pct;
	unsigned int		at_effective_down_rate_adaptive;
	unsigned int		at_effective_down_threshold_adaptive;
	unsigned int		at_effective_frame_pace_floor_pct;
	unsigned int		at_effective_game_mode;
	bool			at_local_actions;
	struct delayed_work	at_work;

	/* Auto-tune classifier ring buffer.  Single-writer (the
	 * delayed_work worker), multi-reader (sysfs at_log readers).
	 * at_log_head is the next write slot; at_log_count is the total
	 * number of entries pushed since the last reset, capped at the
	 * ring depth on read.  Both are reset by writing to
	 * zenith_stats_reset.
	 */
	struct zenith_at_log_entry at_log[ZENITH_AT_LOG_NR];
	unsigned int		at_log_head;
	unsigned int		at_log_count;

	/* brutal_decay_ms tail-glide deadline.  0 means no decay is
	 * armed and zenith_get_next_freq() takes the fast path; non-zero
	 * is an absolute ktime_get_ns() value at which the decay floor
	 * stops applying.  Single-writer (zenith_get_next_freq() under
	 * the policy's update_lock), single-reader (same site).
	 */
	u64			brutal_decay_until_ns;
	unsigned int		brutal_decay_arm_ms;

	/* prefer_silver_aware coordination state.  Snapshot of the
	 * global prefer_silver hit / miss counters at the previous V1
	 * classifier window, plus the resulting hit-rate (0..100) for
	 * the most-recent window.  The hit-rate is read on the hot
	 * path by zenith_get_next_freq() and only updated by the worker,
	 * so the read is unsynchronised but bounded to the previous
	 * complete window.  When CONFIG_SCHED_PREFER_SILVER=n these
	 * fields stay at 0 (the worker never updates them) and the
	 * downstream bump never fires.
	 */
	unsigned int		ps_prev_hit;
	unsigned int		ps_prev_miss;
	unsigned int		ps_hit_rate_pct;

	/* Cached topology bit: true when any CPU in the policy has
	 * arch_scale_cpu_capacity == SCHED_CAPACITY_SCALE, i.e. the
	 * policy belongs to (one of) the system's highest-capacity
	 * cluster(s).  Computed once at zenith_start() time from the
	 * policy's cpumask and kept for the lifetime of the policy.
	 * Used by the input-boost gate (input_boost_big_only) to skip
	 * boosting small-cluster policies on heterogeneous SoCs.
	 */
	bool			is_big_cluster;

	/* Cached rate-limit scale factors applied when
	 * rate_limit_cluster_scale is enabled.  1/0 means unscaled;
	 * little clusters use 2/1 so upward ramps are less eager and
	 * downward ramps save power sooner.
	 */
	unsigned int		up_rate_scale;
	unsigned int		down_rate_scale_shift;
	unsigned int		cluster_class;

	/* Cached per-policy result of the render-aware comm walk.  Valid
	 * for ZENITH_RENDER_CACHE_TTL_NS after render_cache_stamp_ns.
	 * Refreshed on the next zenith_get_next_freq() call past the TTL.
	 * Zero stamp means "never sampled".
	 */
	bool			render_active;
	u64			render_cache_stamp_ns;

	/* Cached per-policy result of the audio-aware comm walk.  Same
	 * shape as the render cache above; TTL is
	 * ZENITH_AUDIO_CACHE_TTL_NS.  Zero stamp means "never sampled".
	 */
	bool			audio_active;
	u64			audio_cache_stamp_ns;

	/* Cached per-policy result of the camera-aware comm walk.
	 * Holds the *raw* comm-match result, before the userspace
	 * override (camera_active=auto/force-on/force-off) is applied.
	 * TTL is ZENITH_CAMERA_CACHE_TTL_NS.  Zero stamp means "never
	 * sampled".
	 */
	bool			camera_auto_match;
	u64			camera_cache_stamp_ns;

	/* Last seen zenith_input_boost_until_ns deadline observed inside
	 * an active boost window for this policy.  Latched in the input
	 * boost step (0) and consumed by the sampling-down step (7) to
	 * keep the down-rate multiplier elevated for one stretched
	 * down-rate window after the boost exits, smoothing the boost
	 * tail when target_freq immediately falls below policy->max.
	 * Cleared once the stretched window passes.  Zero means "no
	 * recent boost to extend".
	 */
	u64			boost_active_until_ns;

	/* Variance-adaptive up_threshold state (see
	 * ZENITH_DEFAULT_UP_THRESHOLD_ADAPTIVE).  load_var_ewma_x256 is
	 * an EWMA (alpha=1/8) of |load_pct - prev| in fixed-point
	 * 1/256ths -- so a value of 256 == 1.0 percentage-point of
	 * average sample-to-sample swing.  Read at the top of
	 * zenith_get_next_freq() to bias dynamic_up_thresh, updated at
	 * the bottom with the current sample's tp_load_pct.  Both fields
	 * zero-initialised by zenith_start()'s memset.
	 */
	unsigned int		load_var_ewma_x256;
	unsigned int		last_load_pct;

	/* Per-policy decision stats.  Bumped from zenith_get_next_freq()
	 * once per evaluation; total decisions and cache_hits are
	 * always counted, the tier buckets are mapped from the final
	 * tp_path string by zenith_path_to_bucket().  Reset to zero on
	 * zenith_start() so the values reflect the current attach
	 * cycle.  No locking: zenith_get_next_freq() is serialised
	 * per-policy by the cpufreq core (the leader cpu in a shared
	 * policy, the only cpu in a single policy), so a plain
	 * unsigned long ++ is race-free.
	 */
	unsigned long		stats[ZENITH_STAT_NR];
};

struct zenith_cpu {
	struct update_util_data	update_util;
	struct zenith_policy	*z_policy;
	unsigned int		cpu;

	bool			iowait_boost_pending;
	unsigned int		iowait_boost;

	/* Wall-time the current iowait boost episode first armed; zero
	 * means "no episode in progress".  Stamped in zenith_iowait_boost()
	 * on the 0->floor transition, cleared whenever iowait_boost
	 * itself is cleared (zenith_iowait_reset / zenith_iowait_apply).
	 * Read by zenith_iowait_boost() to decide whether the
	 * iowait_backoff_after_ms timer has elapsed and the doubling
	 * step should flip to a halving step.
	 */
	u64			iowait_boost_first_ns;
	u64			last_update;
	unsigned long		bw_dl;

	/* Delta tracking for ignore_nice_load. Only read when the
	 * corresponding tunable is set.
	 */
	u64			prev_nice_time;
	u64			prev_nice_wall;
	unsigned long		max_capacity;

	/* Previous *unpredicted* zenith_get_util() output for this CPU,
	 * used as the second tap of the one-step-ahead predictor.  Only
	 * read/written when tunables->predict_util_pct != 0.  Initialised
	 * to zero by the kzalloc-style allocation in zenith_start().
	 */
	unsigned long		prev_util;

	/* Previous arch_scale_thermal_pressure() observation, used by
	 * the thermal_derate_rate_pct derivative term to compute a
	 * sample-to-sample rise.  Zero-initialised by zenith_start()'s
	 * memset; updated unconditionally whenever the static derate
	 * runs, so toggling the rate tunable doesn't see a stale
	 * prev_pressure for the first sample.
	 */
	unsigned long		prev_thermal_pressure;

	/* Tap from two samples ago, used only when
	 * tunables->predict_util_smooth is set and predict_util_pct != 0.
	 * Maintained in lockstep with prev_util (push the old prev_util
	 * into prev_util_2 before overwriting prev_util) so toggling the
	 * smooth tunable does not require any per-cpu init handshake.
	 * Initialised to zero by the kzalloc-style allocation in
	 * zenith_start().
	 */
	unsigned long		prev_util_2;

	/* Wakeup-boost state.  prev_util is independent from the predictor
	 * taps above so the detector works even when predict_util_pct=0.
	 * The countdown is decremented on upward transitions that bypass
	 * up_rate_limit.
	 */
	unsigned long		wakeup_prev_util;
	u8			wakeup_boost_ticks;

	/* kcpustat hispeed-blend sampler state (consumed by
	 * zenith_kcpustat_sample / zenith_kcpustat_blend). Two-phase
	 * windowed measurement: phase 1 clears at window expiry and
	 * arms hispeed_active for an immediate post-reset sample on
	 * the next callback.  Filtered busy_pct feeds an asymmetric
	 * EWMA (instant up, slow-decay down by filter_shift).
	 *
	 * hispeed_start_ns marks t=0 of the >>(elapsed_ms/32) decay
	 * applied in zenith_kcpustat_blend(); hispeed_idle_windows
	 * provides a one-window grace period before clearing the
	 * decay timer, so rapid idle/busy spinning doesn't keep the
	 * floor latched at full strength.
	 *
	 * All fields are zero-initialised by zenith_start()'s
	 * memset, which is the desired post-policy-attach state.
	 */
	u64			kc_prev_idle_time;	/* in usec */
	u64			kc_prev_wall_time;	/* in usec */
	unsigned int		kc_busy_pct;		/* raw, last window */
	unsigned int		kc_filtered_busy_pct;	/* EWMA-smoothed */
	bool			kc_hispeed_active;
	u64			kc_hispeed_start_ns;
	unsigned int		kc_idle_windows;
};

static DEFINE_PER_CPU(struct zenith_cpu, zenith_cpu);

static unsigned int zenith_tunable_or_local(struct zenith_policy *z_policy,
					    unsigned int tunable,
					    unsigned int local);

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
	z_cpu->iowait_boost_first_ns = set_iowait_boost ? time : 0;
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
		unsigned int after_ms =
			z_cpu->z_policy->tunables->iowait_backoff_after_ms;
		u64 first = z_cpu->iowait_boost_first_ns;

		/* Sustained-iowait backoff: once the episode has been
		 * live for after_ms milliseconds, halve instead of
		 * doubling so the boost decays toward the floor and
		 * eventually clears.  See ZENITH_DEFAULT_IOWAIT_BACKOFF_
		 * AFTER_MS for the why.  after_ms=0 keeps the legacy
		 * unbounded-doubling behaviour.
		 */
		if (after_ms && first &&
		    (time - first) > (u64)after_ms * NSEC_PER_MSEC) {
			z_cpu->iowait_boost >>= 1;
			if (z_cpu->iowait_boost <
			    zenith_iowait_floor(z_cpu)) {
				z_cpu->iowait_boost = 0;
				z_cpu->iowait_boost_first_ns = 0;
			}
		} else {
			z_cpu->iowait_boost = min_t(unsigned int,
				z_cpu->iowait_boost << 1, SCHED_CAPACITY_SCALE);
		}
		return;
	}
	z_cpu->iowait_boost = zenith_iowait_floor(z_cpu);
	z_cpu->iowait_boost_first_ns = time;
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
			z_cpu->iowait_boost_first_ns = 0;
			return util;
		}
	}

	z_cpu->iowait_boost_pending = false;
	boost = (z_cpu->iowait_boost * max_cap) >> SCHED_CAPACITY_SHIFT;

	{
		unsigned int stack_pct =
			z_cpu->z_policy->tunables->iowait_stack_pct;

		if (stack_pct && stack_pct <= 100) {
			/* Blend: util + (boost * stack_pct / 100), clamped
			 * to max_cap.  Take max with the legacy result so
			 * we never do worse than the old path on a
			 * util-light / iowait-heavy workload.
			 */
			unsigned long stacked = util +
				((boost * stack_pct) / 100);

			if (stacked > max_cap)
				stacked = max_cap;
			boost = max3(boost, util, stacked);
		} else {
			boost = max(boost, util);
		}
	}
	boost = uclamp_rq_util_with(cpu_rq(z_cpu->cpu), boost, NULL);
	return boost;
}

static inline void zenith_ignore_dl_rate_limit(struct zenith_cpu *z_cpu, struct zenith_policy *z_policy)
{
	if (cpu_bw_dl(cpu_rq(z_cpu->cpu)) > z_cpu->bw_dl)
		WRITE_ONCE(z_policy->limits_changed, true);
}

/*
 * Compute the cfs_rq util_cfs input fed into schedutil_cpu_util().
 *
 * v1 (default): cpu_util_cfs(rq), which on 5.10 returns
 *
 *      util_avg, optionally maxed with util_est.enqueued when the
 *      UTIL_EST sched_feat is on.
 *
 * v2 (tunable->util_math_v2 = 1): replicates the 6.x-style
 * cpu_util_cfs_boost() shape to give zenith a more responsive signal
 * for short, intermittent tasks (Android UI/render threads). The v2
 * formula is
 *
 *      util_v2 = max(util_avg,
 *                    util_est.enqueued (& ~UTIL_AVG_UNCHANGED),
 *                    runnable_avg)
 *
 * runnable_avg is what tasks contribute *while runnable* (regardless
 * of whether they are currently running), so spikes from
 * intermittent threads land in the util signal one PELT half-life
 * earlier than they do via util_avg alone. The UTIL_AVG_UNCHANGED
 * MSB on util_est.enqueued is masked defensively (cfs_rq sums tasks'
 * enqueued so the bit shouldn't be set there in practice, but
 * guarding against future kernel changes is cheap).
 *
 * Flipping the tunable does not change PELT or util_est accounting --
 * only what zenith feeds into the proportional math. It is safe to
 * toggle at runtime; zenith_invalidate_cache() is hit by the _store
 * path so the next callback recomputes immediately.
 */
static unsigned long zenith_get_util(struct zenith_cpu *z_cpu)
{
	struct rq *rq = cpu_rq(z_cpu->cpu);
	unsigned long util, util_out;
	unsigned long max = arch_scale_cpu_capacity(z_cpu->cpu);
	unsigned int predict_pct;

	z_cpu->max_capacity = max;
	z_cpu->bw_dl = cpu_bw_dl(rq);

	if (z_cpu->z_policy->tunables->util_math_v2) {
		unsigned long util_avg = READ_ONCE(rq->cfs.avg.util_avg);
		unsigned long runnable = READ_ONCE(rq->cfs.avg.runnable_avg);

		util = util_avg;
		if (sched_feat(UTIL_EST)) {
			unsigned long enq = READ_ONCE(rq->cfs.avg.util_est.enqueued);

			enq &= ~UTIL_AVG_UNCHANGED;
			util = max(util, enq);
		}
		util = max(util, runnable);
	} else {
		util = cpu_util_cfs(rq);
	}

	util_out = schedutil_cpu_util(z_cpu->cpu, util, max, FREQUENCY_UTIL, NULL);

	/* Thermal-pressure-aware util derate.  When the SoC thermal
	 * framework has eaten a meaningful fraction of capacity, scale
	 * util_out by (max - pressure) / max so the freq decision
	 * targets what we can actually deliver instead of pinning to
	 * the (already throttled) policy->max.  See the
	 * ZENITH_DEFAULT_THERMAL_UTIL_DERATE comment block.
	 *
	 * Sub-floor pressure is ignored to avoid the multiply cost on
	 * the noise.  trace_zenith_thermal_derate fires only when the
	 * derate actually changes util.
	 */
	if (READ_ONCE(z_cpu->z_policy->tunables->thermal_util_derate) && max) {
		unsigned long pressure = arch_scale_thermal_pressure(z_cpu->cpu);
		unsigned long pressure_pct = (pressure * 100) / max;

		if (pressure_pct >= ZENITH_THERMAL_DERATE_FLOOR_PCT &&
		    pressure < max) {
			unsigned long avail = max - pressure;
			unsigned long derated = (util_out * avail) / max;
			unsigned int rate_pct = READ_ONCE(
				z_cpu->z_policy->tunables->
					thermal_derate_rate_pct);

			/* Derivative term: when pressure is rising,
			 * scale derated further by the single-step rise
			 * relative to capacity, capped to rate_pct.  See
			 * ZENITH_DEFAULT_THERMAL_DERATE_RATE_PCT for the
			 * why.  rate_pct == 0 keeps the legacy level-only
			 * shape; the rate-of-change term piggybacks on
			 * the existing gate so it adds no cost when off.
			 */
			if (rate_pct &&
			    pressure > z_cpu->prev_thermal_pressure) {
				unsigned long rise =
					pressure - z_cpu->prev_thermal_pressure;
				unsigned long rise_pct =
					(rise * 100) / max;

				if (rate_pct > 100)
					rate_pct = 100;
				if (rise_pct > rate_pct)
					rise_pct = rate_pct;
				derated = (derated * (100 - rise_pct)) / 100;
			}

			if (trace_zenith_thermal_derate_enabled())
				trace_zenith_thermal_derate(z_cpu->cpu,
							    util_out, derated,
							    (unsigned int)pressure_pct);
			util_out = derated;
		}
		z_cpu->prev_thermal_pressure = pressure;
	}

	/* One-step-ahead linear predictor (up-only). See the
	 * ZENITH_DEFAULT_PREDICT_UTIL_PCT comment block at the top of
	 * this file for semantics. predict_pct == 0 short-circuits the
	 * whole block; non-zero loads the previous unpredicted value
	 * and applies pred = util + delta * pct / 100, taking max with
	 * the observed util so we never under-predict on a ramp-down.
	 * prev_util is updated unconditionally inside the gated block
	 * so disabling the predictor re-arms it cleanly on next enable.
	 */
	predict_pct = READ_ONCE(z_cpu->z_policy->tunables->predict_util_pct);
	if (predict_pct) {
		unsigned long prev = z_cpu->prev_util;
		unsigned int smooth = READ_ONCE(
			z_cpu->z_policy->tunables->predict_util_smooth);

		if (predict_pct > ZENITH_PREDICT_UTIL_PCT_MAX)
			predict_pct = ZENITH_PREDICT_UTIL_PCT_MAX;

		if (util_out > prev) {
			unsigned long slope1 = util_out - prev;
			unsigned long eff_slope = slope1;
			unsigned long pred;

			/* Two-tap smoothing (I7).  Average slope1 with
			 * the previous observed slope only when it was
			 * also positive -- preserves the base predictor's
			 * up-only invariant (we never smooth in a
			 * negative-slope memory).  Cheap: one add, one
			 * shift, no divide.
			 */
			if (smooth && prev > z_cpu->prev_util_2) {
				unsigned long slope2 =
					prev - z_cpu->prev_util_2;
				eff_slope = (slope1 + slope2) >> 1;
			}

			pred = util_out + (eff_slope * predict_pct) / 100;
			if (pred > max)
				pred = max;
			if (trace_zenith_predict_enabled())
				trace_zenith_predict(z_cpu->cpu, predict_pct,
						     util_out, pred);
			z_cpu->prev_util_2 = prev;
			z_cpu->prev_util = util_out;
			util_out = pred;
		} else {
			z_cpu->prev_util_2 = prev;
			z_cpu->prev_util = util_out;
		}
	}

	return util_out;
}

/************************ kcpustat hispeed sampler *********************
 *
 * Adapted from reflex (firelzrd, MIT-compatible / GPL-2.0):
 *
 *   https://github.com/firelzrd/reflex/blob/main/patches/0001-Reflex-CPUFreq-Governor-v0.3.0r2.patch
 *
 * Reflex's insight: PELT util has a 32 ms half-life, so on a sudden
 * busy-from-idle transition the scheduler util signal lags real load
 * by ~96-200 ms. kcpustat (kernel idle-time accounting) gives us the
 * raw busy ratio over the last observation window without smoothing,
 * which is perfect as a *temporary* util floor. To avoid double
 * counting once PELT catches up, we decay the floor with PELT's own
 * 32 ms half-life so total coverage stays ~100% across the ramp.
 *
 * Reflex implements the decay in log-domain with a 256-entry LUT to
 * get 1 ms granularity.  For Android phones we trade that precision
 * for simplicity: we decay in 32 ms quanta with a single right-shift,
 * which is exact at half-life boundaries and a few percent off in
 * between.  That's well below noise floor for cpufreq decisions.
 *
 * The integration point is the upcoming patch that calls
 * zenith_kcpustat_blend() in zenith_update_{single,shared}.  This
 * patch adds the sampler and the blend helper but doesn't call
 * either, so it is a pure no-op until the integration patch lands.
 */

/* Cap the decay shift so >> shift always produces 0 when the
 * contribution should be negligible.  After 8 half-lives the floor is
 * 1/256 of its initial value, well past the point of mattering.
 */
#define ZENITH_KC_DECAY_HALF_LIFE_MS	32
#define ZENITH_KC_DECAY_MAX_SHIFT	8

/*
 * Sample CPU busy ratio over the last window via kcpustat.
 *
 * Two-phase: when the window has elapsed, phase 1 latches the new
 * prev_idle / prev_wall snapshot and arms hispeed_active.  Phase 2
 * runs on the next callback and computes the actual busy_pct delta;
 * this avoids racing time-of-day skew between the snapshot reset and
 * the busy% calculation in the same callback.
 *
 * busy_pct flows through an asymmetric EWMA (instant up, decay down
 * by filter_shift).  filter_shift==0 disables the EWMA and uses the
 * raw value.
 *
 * Updates the hispeed decay timer:
 *   - any non-zero filtered busy_pct refreshes log_hispeed and starts
 *     (or keeps) hispeed_start_ns;
 *   - two consecutive idle windows clear the timer so the floor goes
 *     fully transparent until activity resumes.
 *
 * Caller must serialise (single path: only one CPU writes; shared
 * path: caller holds update_lock).
 */
static void zenith_kcpustat_sample(struct zenith_cpu *z_cpu,
				   unsigned int window_us,
				   unsigned int filter_shift, u64 time)
{
	u64 cur_idle, cur_wall;
	u64 wall_delta, idle_delta;

	cur_idle = get_cpu_idle_time(z_cpu->cpu, &cur_wall, 1);
	/* kc_prev_wall_time and cur_wall are u64 microseconds; the
	 * delta can exceed UINT_MAX on long-idle CPUs (~71 minutes).
	 * Truncating to unsigned int there made wall_delta wrap to a
	 * tiny value and immediately falsely satisfy the
	 * wall_delta >= window_us phase-1 condition, so the sampler
	 * silently re-armed instead of producing a real busy_pct.
	 * Keep everything u64 through the divide.
	 */
	wall_delta = cur_wall - z_cpu->kc_prev_wall_time;

	if (wall_delta >= window_us) {
		/*
		 * Phase 1: window elapsed.  Latch the new prev_*
		 * snapshot and let the next callback take the actual
		 * busy delta.  We don't touch the decay timer here:
		 * the momentary busy_pct=0 is a measurement artefact,
		 * not a genuine idle signal.
		 */
		z_cpu->kc_busy_pct = 0;
		z_cpu->kc_hispeed_active = true;
		z_cpu->kc_prev_idle_time = cur_idle;
		z_cpu->kc_prev_wall_time = cur_wall;
		return;
	}

	/* Within the current window. Skip unless phase 2 is armed. */
	if (!z_cpu->kc_hispeed_active)
		return;

	z_cpu->kc_hispeed_active = false;

	idle_delta = (cur_idle > z_cpu->kc_prev_idle_time) ?
		     (cur_idle - z_cpu->kc_prev_idle_time) : 0;

	z_cpu->kc_busy_pct = (wall_delta > idle_delta) ?
		(unsigned int)div64_u64(100ULL * (wall_delta - idle_delta),
					wall_delta) : 0;

	z_cpu->kc_prev_idle_time = cur_idle;
	z_cpu->kc_prev_wall_time = cur_wall;

	/* Asymmetric EWMA: instant up, configurable decay down. */
	if (!filter_shift ||
	    z_cpu->kc_busy_pct >= z_cpu->kc_filtered_busy_pct) {
		z_cpu->kc_filtered_busy_pct = z_cpu->kc_busy_pct;
	} else {
		unsigned int step =
			(z_cpu->kc_filtered_busy_pct - z_cpu->kc_busy_pct)
			>> filter_shift;
		z_cpu->kc_filtered_busy_pct -= step ? step :
			(z_cpu->kc_filtered_busy_pct - z_cpu->kc_busy_pct);
	}

	/*
	 * Decay timer with one-window grace period: avoid resetting
	 * hispeed_start_ns on every transient idle window so a busy
	 * task with sub-window idle gaps still sees a coherent decay
	 * trajectory.
	 */
	if (z_cpu->kc_filtered_busy_pct) {
		z_cpu->kc_idle_windows = 0;
		if (!z_cpu->kc_hispeed_start_ns)
			z_cpu->kc_hispeed_start_ns = time;
	} else if (++z_cpu->kc_idle_windows >= 2) {
		z_cpu->kc_hispeed_start_ns = 0;
		z_cpu->kc_filtered_busy_pct = 0;
	}
}

/*
 * Blend PELT util with a kcpustat-derived util floor that decays at
 * PELT's 32 ms half-life.  Returns pelt_util unchanged when the
 * blend is inactive (zero busy%, no start timestamp, or decay fully
 * elapsed).  When active, returns pelt_util plus the decayed floor,
 * capped at the raw kcpustat-implied util (so the blend can never
 * exceed the actual measured busy fraction).
 */
static unsigned long zenith_kcpustat_blend(struct zenith_cpu *z_cpu,
					   unsigned long pelt_util,
					   unsigned long max_cap, u64 time)
{
	unsigned long hispeed_util, decayed;
	u64 elapsed_ns;
	unsigned int half_lives;

	if (!z_cpu->kc_filtered_busy_pct || !z_cpu->kc_hispeed_start_ns ||
	    !max_cap)
		return pelt_util;

	hispeed_util = (max_cap * z_cpu->kc_filtered_busy_pct) / 100u;
	if (hispeed_util <= pelt_util)
		return pelt_util;

	elapsed_ns = time - z_cpu->kc_hispeed_start_ns;
	half_lives = (unsigned int)
		(elapsed_ns / (ZENITH_KC_DECAY_HALF_LIFE_MS * NSEC_PER_MSEC));

	if (half_lives >= ZENITH_KC_DECAY_MAX_SHIFT)
		return pelt_util;

	decayed = hispeed_util >> half_lives;
	if (decayed <= pelt_util)
		return pelt_util;

	return min(pelt_util + decayed, hispeed_util);
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

static unsigned int zenith_policy_thermal_pressure_pct(struct zenith_policy *z_policy)
{
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned long pressure;
	unsigned long cap;
	int cpu;

	cpu = cpumask_first(policy->cpus);
	if (cpu >= nr_cpu_ids)
		return 0;

	cap = arch_scale_cpu_capacity(cpu);
	if (!cap)
		return 0;

	pressure = arch_scale_thermal_pressure(cpu);
	return min_t(unsigned int, (pressure * 100) / cap, 100);
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

/* Spike detection threshold: when the requested frequency jump exceeds
 * policy->max >> ZENITH_SPIKE_SHIFT, bypass the up_rate_limit entirely.
 * This gives instant response to load spikes (task wakeup, game frame
 * start, UI touch) while still rate-limiting small, noise-driven
 * oscillations.
 */
#define ZENITH_SPIKE_SHIFT	3	/* 1 << 3 = divide by 8 = 12.5% */

static bool zenith_up_down_rate_limit(struct zenith_policy *z_policy, u64 time, unsigned int next_freq)
{
	s64 delta_ns = time - z_policy->last_freq_update_time;
	struct zenith_tunables *tunables = z_policy->tunables;

	/* Snapshot the cached delays once.  Concurrent writers are
	 * profile_store / auto_tune / up_rate_limit_us_store; readers
	 * are this hot path and zenith_should_update_freq().  Without
	 * READ_ONCE the compiler may reload between the comparison and
	 * the down_delay multiply, yielding inconsistent decisions.
	 */
	s64 up_delay = READ_ONCE(z_policy->up_rate_delay_ns);
	s64 down_delay = READ_ONCE(z_policy->down_rate_delay_ns) *
			 (s64)max(z_policy->down_rate_mult, 1U);

	if (READ_ONCE(tunables->down_rate_adaptive)) {
		unsigned int var = READ_ONCE(z_policy->load_var_ewma_x256);

		if (var > 256)
			var = 256;
		down_delay = (down_delay * (256 + var)) / 256;
	}

	if (next_freq > z_policy->next_freq) {
		unsigned int spike = z_policy->policy->max >> ZENITH_SPIKE_SHIFT;
		unsigned int cpu;
		struct zenith_cpu *z_cpu;

		for_each_cpu(cpu, z_policy->policy->cpus) {
			z_cpu = &per_cpu(zenith_cpu, cpu);

			if (z_cpu->wakeup_boost_ticks) {
				z_cpu->wakeup_boost_ticks--;
				return false;
			}
		}
		if (next_freq - z_policy->next_freq >= spike)
			return false;
		if (delta_ns < up_delay)
			return true;
	}

	if (next_freq < z_policy->next_freq && delta_ns < down_delay)
		return true;

	return false;
}

static bool zenith_should_update_freq(struct zenith_policy *z_policy, u64 time)
{
	s64 delta_ns;

	if (!cpufreq_this_cpu_can_update(z_policy->policy))
		return false;

	/* Pair the smp_store_release() in zenith_limits() with an
	 * smp_load_acquire() here so the new policy->{min,max} written
	 * by cpufreq_policy_apply_limits() is observed by the
	 * subsequent eval on this CPU.  The previous smp_wmb()/smp_mb()
	 * pair only ordered prior stores; it did not guarantee that the
	 * load of policy->{min,max} below the flag check was observed
	 * after the store of those fields above the flag store on the
	 * writer side.
	 */
	if (unlikely(smp_load_acquire(&z_policy->limits_changed))) {
		WRITE_ONCE(z_policy->limits_changed, false);
		z_policy->need_freq_update = true;
		return true;
	}

	if (z_policy->work_in_progress)
		return true;

	delta_ns = time - z_policy->last_freq_update_time;
	return delta_ns >= READ_ONCE(z_policy->min_rate_limit_ns);
}

/*
 * Return the max RQ-aggregated UCLAMP_MIN (in capacity units) across all
 * CPUs in this policy.  Uses uclamp_rq_get() which is just a READ_ONCE
 * on the rq->uclamp[UCLAMP_MIN].value field already maintained by the
 * scheduler on every enqueue/dequeue -- no locking needed, no
 * measurable fast-path cost even on 8-CPU policies.
 *
 * Returns 0 if CONFIG_UCLAMP_TASK is off (uclamp_rq_get stub returns 0)
 * or if uclamp is compiled in but no task on the policy has set
 * uclamp_min.  Callers use the 0 return as "no floor to apply".
 */
/* Refresh both per-policy uclamp aggregates in a single rq walk when
 * the cache is cold or stale.  Called from the uclamp_min / uclamp_max
 * helpers below; not meant to be invoked directly.
 */
static void zenith_uclamp_cache_refresh(struct zenith_policy *z_policy)
{
#ifdef CONFIG_UCLAMP_TASK
	unsigned long max_umin = 0;
	unsigned long max_umax = 0;
	int cpu;

	if (!uclamp_is_used()) {
		z_policy->cached_uclamp_min = 0;
		z_policy->cached_uclamp_max = SCHED_CAPACITY_SCALE;
		z_policy->uclamp_cache_stamp_ns = ktime_get_ns();
		return;
	}

	for_each_cpu(cpu, z_policy->policy->cpus) {
		struct rq *rq = cpu_rq(cpu);
		unsigned long umin = uclamp_rq_get(rq, UCLAMP_MIN);
		unsigned long umax = uclamp_rq_get(rq, UCLAMP_MAX);

		if (umin > max_umin)
			max_umin = umin;
		if (umax > max_umax)
			max_umax = umax;
	}
	z_policy->cached_uclamp_min = max_umin;
	z_policy->cached_uclamp_max = max_umax ? max_umax : SCHED_CAPACITY_SCALE;
	z_policy->uclamp_cache_stamp_ns = ktime_get_ns();
#else
	z_policy->cached_uclamp_min = 0;
	z_policy->cached_uclamp_max = SCHED_CAPACITY_SCALE;
	z_policy->uclamp_cache_stamp_ns = ktime_get_ns();
#endif
}

static inline bool zenith_uclamp_cache_fresh(struct zenith_policy *z_policy)
{
	u64 stamp = z_policy->uclamp_cache_stamp_ns;

	if (!stamp)
		return false;
	return ktime_get_ns() - stamp < ZENITH_UCLAMP_CACHE_TTL_NS;
}

static unsigned long zenith_policy_uclamp_min(struct zenith_policy *z_policy)
{
	if (!zenith_uclamp_cache_fresh(z_policy))
		zenith_uclamp_cache_refresh(z_policy);
	return z_policy->cached_uclamp_min;
}

/* Collect the effective uclamp_max for the policy.
 *
 * uclamp_max is Android's opt-in power-saving hint
 * (PerformanceHint.setPreferPowerEfficiency() -> per-task UCLAMP_MAX).
 * uclamp_rq_get(rq, UCLAMP_MAX) returns the maximum UCLAMP_MAX over
 * currently enqueued tasks, which honours the rule "if any task on
 * this rq wants no cap, don't cap".  The same rule has to hold at
 * policy granularity: if any CPU in the policy has a task that
 * doesn't want the freq capped, we must not cap the freq for the
 * whole cluster.  That's an _max_ reduction across per-rq values.
 *
 * SCHED_CAPACITY_SCALE is the canonical "no cap" sentinel (tasks
 * with no explicit UCLAMP_MAX set).  Callers treat any value
 * >= SCHED_CAPACITY_SCALE as "tier disabled, apply no freq cap".
 *
 * Returns SCHED_CAPACITY_SCALE if CONFIG_UCLAMP_TASK is off or uclamp
 * is not in use -- callers interpret that as "no cap".
 */
static unsigned long zenith_policy_uclamp_max(struct zenith_policy *z_policy)
{
	if (!zenith_uclamp_cache_fresh(z_policy))
		zenith_uclamp_cache_refresh(z_policy);
	return z_policy->cached_uclamp_max;
}

/* Effective hispeed floor in kHz for this policy.  If tunables->hispeed_freq
 * is set explicitly, honour it verbatim (legacy behaviour).  Otherwise fall
 * back to the per-cluster auto-default: (policy->max * hispeed_freq_pct / 100).
 * Returns 0 when the tier is disabled (both absolute and percentage values
 * are zero, or hispeed_freq_pct is zero while hispeed_freq is zero).
 *
 * When game_mode=1, the percentage path is multiplied by
 * ZENITH_GAME_HISPEED_BOOST_PCT/100 so userspace game-detection
 * daemons can lift the per-cluster auto-default floor without
 * touching hispeed_freq_pct itself.  The absolute hispeed_freq path
 * is left untouched: a userspace value written there is honoured
 * verbatim even with game_mode=1.
 */
static inline unsigned int zenith_eff_hispeed_freq(struct zenith_policy *z_policy)
{
	unsigned int eff = z_policy->tunables->hispeed_freq;
	unsigned int pct = z_policy->tunables->hispeed_freq_pct;

	if (!eff && pct) {
		unsigned int gm = z_policy->tunables->game_mode;

		if (gm >= 2)
			pct = (pct * ZENITH_GAME_L2_HISPEED_BOOST_PCT) / 100;
		else if (gm == 1)
			pct = (pct * ZENITH_GAME_HISPEED_BOOST_PCT) / 100;
		eff = (z_policy->policy->max * pct) / 100;
		if (eff > z_policy->policy->max)
			eff = z_policy->policy->max;
	}
	return eff;
}

/* Read the system-wide memory pressure 10s average from PSI as an
 * integer percentage (0..100).  Returns 0 when CONFIG_PSI is off, when
 * psi_disabled is set, or when the value isn't yet populated (early
 * boot).  Lock-free single READ_ONCE -- the avgs_work aggregator is
 * what writes to psi_system.avg[][] and we tolerate up to 2 s of
 * staleness on the read.
 */
static inline unsigned int zenith_psi_mem_some_pct(void)
{
#ifdef CONFIG_PSI
	unsigned long avg;

	if (static_branch_likely(&psi_disabled))
		return 0;

	avg = READ_ONCE(psi_system.avg[PSI_MEM_SOME][0]);
	return (unsigned int)LOAD_INT(avg);
#else
	return 0;
#endif
}

/* Same shape as zenith_psi_mem_some_pct() for the PSI_CPU_SOME and
 * PSI_IO_SOME dimensions.  See the psi_aware / psi_*_thresh comment
 * block for what each pressure source means.  Both helpers are
 * RCU-free, lock-free, and tolerate CONFIG_PSI=n at compile time.
 */
static inline unsigned int zenith_psi_cpu_some_pct(void)
{
#ifdef CONFIG_PSI
	unsigned long avg;

	if (static_branch_likely(&psi_disabled))
		return 0;

	avg = READ_ONCE(psi_system.avg[PSI_CPU_SOME][0]);
	return (unsigned int)LOAD_INT(avg);
#else
	return 0;
#endif
}

static inline unsigned int zenith_psi_io_some_pct(void)
{
#ifdef CONFIG_PSI
	unsigned long avg;

	if (static_branch_likely(&psi_disabled))
		return 0;

	avg = READ_ONCE(psi_system.avg[PSI_IO_SOME][0]);
	return (unsigned int)LOAD_INT(avg);
#else
	return 0;
#endif
}

/* RCU-protected, sysfs-configurable comm-prefix lists.
 *
 * The render / audio / camera comm tables used to be three plain
 * `static const char * const X[]` arrays compiled into the kernel.
 * Vendors using non-AOSP userspace -- MTK skins, Qualcomm OEM
 * builds, OEM-rebrand audio servers -- needed kernel rebuilds just
 * to add their thread names.  This block reworks the storage as a
 * trio of RCU-protected tables, populated at init from the original
 * arrays (so out-of-box behaviour is unchanged) and replaceable
 * lock-free via three new sysfs nodes (render_comms / audio_comms
 * / camera_comms, comma-separated).
 *
 * The reader (each of the three zenith_policy_has_X helpers) does:
 *   rcu_read_lock();
 *   table = rcu_dereference(zenith_<X>_table);
 *   for (i = 0; i < table->nr; i++) strncmp(curr->comm, table->entries[i], ...);
 *   rcu_read_unlock();
 *
 * The writer (each of the three sysfs_store helpers) builds an
 * entirely new struct zenith_comm_table from the CSV, RCU-swaps the
 * pointer, then kfree_rcu()'s the old one.  No hot-path allocation,
 * no lock contention with the readers, no synchronize_rcu() at
 * commit time -- the readers walk a stable snapshot until their
 * grace period closes, the kfree_rcu callback drops the old table
 * once everyone has moved on.
 *
 * Storage layout: each table is a single allocation containing the
 * pointer array plus the raw NUL-separated buffer the entries[]
 * pointers index into, sized to ZENITH_COMM_LIST_MAX entries and
 * ZENITH_COMM_BUF_MAX raw bytes.  Empty CSV (just "\n" or "")
 * resets to defaults; a parse error fails the entire write so
 * userspace doesn't see a half-applied table.
 */
#define ZENITH_COMM_LIST_MAX		32
#define ZENITH_COMM_BUF_MAX		1024

struct zenith_comm_table {
	struct rcu_head	rcu;
	unsigned int	nr;
	const char	*entries[ZENITH_COMM_LIST_MAX];
	char		raw[ZENITH_COMM_BUF_MAX];
};

static struct zenith_comm_table __rcu *zenith_render_table;
static struct zenith_comm_table __rcu *zenith_audio_table;
static struct zenith_comm_table __rcu *zenith_camera_table;
static DEFINE_MUTEX(zenith_comm_table_lock);

static struct zenith_comm_table *
zenith_alloc_comm_table_from_defaults(const char * const *defaults,
				      size_t nr_defaults)
{
	struct zenith_comm_table *t;
	size_t off = 0;
	unsigned int i;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t)
		return NULL;

	for (i = 0; i < nr_defaults && i < ZENITH_COMM_LIST_MAX; i++) {
		size_t len = strlen(defaults[i]) + 1;

		if (off + len > ZENITH_COMM_BUF_MAX)
			break;
		memcpy(t->raw + off, defaults[i], len);
		t->entries[i] = t->raw + off;
		off += len;
		t->nr = i + 1;
	}
	return t;
}

/* Parse a CSV (entries separated by ',' or whitespace) into a fresh
 * zenith_comm_table.  Returns NULL on alloc failure, ERR_PTR on
 * parse error.  Caller owns the table and must rcu_assign_pointer
 * + kfree_rcu the old one to publish.
 */
static struct zenith_comm_table *
zenith_alloc_comm_table_from_csv(const char *buf, size_t count)
{
	struct zenith_comm_table *t;
	size_t off = 0;
	const char *p = buf;
	const char *end = buf + count;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t)
		return NULL;

	while (p < end && t->nr < ZENITH_COMM_LIST_MAX) {
		const char *tok_start;
		size_t len;

		while (p < end && (*p == ',' || *p == ' ' ||
				   *p == '\t' || *p == '\n'))
			p++;
		if (p == end)
			break;

		tok_start = p;
		while (p < end && *p != ',' && *p != ' ' &&
		       *p != '\t' && *p != '\n')
			p++;
		len = p - tok_start;
		if (!len)
			continue;
		if (off + len + 1 > ZENITH_COMM_BUF_MAX) {
			kfree(t);
			return ERR_PTR(-ENOSPC);
		}
		memcpy(t->raw + off, tok_start, len);
		t->raw[off + len] = '\0';
		t->entries[t->nr++] = t->raw + off;
		off += len + 1;
	}
	return t;
}

static ssize_t zenith_show_comm_table(struct zenith_comm_table __rcu **slot,
				      char *buf)
{
	struct zenith_comm_table *t;
	ssize_t len = 0;
	unsigned int i;
	bool first = true;

	rcu_read_lock();
	t = rcu_dereference(*slot);
	if (t) {
		for (i = 0; i < t->nr; i++) {
			len += scnprintf(buf + len, PAGE_SIZE - len - 1,
					 "%s%s", first ? "" : ",",
					 t->entries[i]);
			first = false;
		}
	}
	rcu_read_unlock();
	len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
	return len;
}

static ssize_t
zenith_store_comm_table(struct zenith_comm_table __rcu **slot,
			const char *const *defaults, size_t nr_defaults,
			const char *buf, size_t count)
{
	struct zenith_comm_table *new_t;
	struct zenith_comm_table *old_t;
	const char *p = buf;
	const char *end = buf + count;

	while (p < end && (*p == ' ' || *p == '\t' || *p == '\n'))
		p++;
	if (p == end)
		new_t = zenith_alloc_comm_table_from_defaults(defaults,
							      nr_defaults);
	else
		new_t = zenith_alloc_comm_table_from_csv(buf, count);

	if (!new_t)
		return -ENOMEM;
	if (IS_ERR(new_t))
		return PTR_ERR(new_t);

	mutex_lock(&zenith_comm_table_lock);
	old_t = rcu_dereference_protected(*slot,
			lockdep_is_held(&zenith_comm_table_lock));
	rcu_assign_pointer(*slot, new_t);
	mutex_unlock(&zenith_comm_table_lock);

	if (old_t)
		kfree_rcu(old_t, rcu);
	return count;
}

/* List of comm prefixes treated as render / display-pipeline threads.
 * Matched by strncmp() over the first N characters where N is the
 * length of the table entry, so the userspace task only needs to
 * have its first N chars match (Android's "RenderThread NNN" naming
 * for libui's per-app render thread, for instance, matches the
 * "RenderThread" prefix).  Used as the seed values for the RCU
 * zenith_render_table at zenith_gov_init() time and as the reset
 * target when the render_comms sysfs node is written empty; live
 * matching always reads the RCU table.
 */
static const char * const zenith_render_comms[] = {
	"RenderThread",
	"surfaceflinger",
	"RenderEngine",
	"mali-cmar-back",
};

/* Walk the policy's online cpumask and check each cpu_curr's comm
 * against zenith_render_comms[].  Returns true on the first match.
 * The result is cached for ZENITH_RENDER_CACHE_TTL_NS so a hot path
 * (e.g. a 60 / 90 / 120 Hz scroll) only does the strncmp loop a few
 * times per second.  Caller is expected to gate the call on
 * tunables->render_aware != 0; this helper does not re-check that.
 */
static bool zenith_policy_has_render(struct zenith_policy *z_policy)
{
	u64 now = ktime_get_ns();
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned int cpu;
	bool match = false;

	if (z_policy->render_cache_stamp_ns &&
	    now - z_policy->render_cache_stamp_ns < ZENITH_RENDER_CACHE_TTL_NS)
		return z_policy->render_active;

	rcu_read_lock();
	{
		struct zenith_comm_table *t =
			rcu_dereference(zenith_render_table);

		for_each_cpu(cpu, policy->cpus) {
			struct task_struct *curr = READ_ONCE(cpu_curr(cpu));
			unsigned int i;

			if (!curr || !t)
				continue;
			for (i = 0; i < t->nr; i++) {
				const char *needle = t->entries[i];

				if (!strncmp(curr->comm, needle,
					     strlen(needle))) {
					match = true;
					break;
				}
			}
			if (match)
				break;
		}
	}
	rcu_read_unlock();

	z_policy->render_active = match;
	z_policy->render_cache_stamp_ns = now;
	return match;
}

/* Audio low-jitter comm match.  Same shape as zenith_render_comms[]:
 * a NUL-terminated table of comm prefixes; strncmp() walks each
 * cpu_curr->comm against each entry up to the table prefix length.
 *
 * Entries are picked from the standard Android audio thread names:
 *   - AudioOut_*       per-AudioFlinger fast/normal mixer threads
 *   - AudioMixer       AudioFlinger mixer threads (older naming)
 *   - audioserver      AudioFlinger main thread
 *   - audio_server     vendor variant of the same
 *   - MediaCodec_*     framework media codec callback threads
 *   - OMX*             OpenMAX vendor codec threads
 *   - SoundPool        framework SoundPool worker
 *   - PlaybackThread   AudioFlinger playback thread
 *   - RecordThread     AudioFlinger record thread
 *
 * Order is tuned for cache-friendliness on phone workloads (the most
 * common per-frame matches first).
 */
static const char * const zenith_audio_comms[] = {
	"AudioOut_",
	"AudioMixer",
	"audioserver",
	"audio_server",
	"MediaCodec_",
	"OMX",
	"SoundPool",
	"PlaybackThread",
	"RecordThread",
};

/* Walk the policy's online cpumask and check each cpu_curr's comm
 * against zenith_audio_comms[].  Returns true on the first match.
 * Cached for ZENITH_AUDIO_CACHE_TTL_NS so the strncmp loop runs
 * once every few milliseconds at most.  Caller is expected to gate
 * the call on tunables->audio_aware != 0; this helper does not
 * re-check that.
 */
static bool zenith_policy_has_audio(struct zenith_policy *z_policy)
{
	u64 now = ktime_get_ns();
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned int cpu;
	bool match = false;

	if (z_policy->audio_cache_stamp_ns &&
	    now - z_policy->audio_cache_stamp_ns < ZENITH_AUDIO_CACHE_TTL_NS)
		return z_policy->audio_active;

	rcu_read_lock();
	{
		struct zenith_comm_table *t =
			rcu_dereference(zenith_audio_table);

		for_each_cpu(cpu, policy->cpus) {
			struct task_struct *curr = READ_ONCE(cpu_curr(cpu));
			unsigned int i;

			if (!curr || !t)
				continue;
			for (i = 0; i < t->nr; i++) {
				const char *needle = t->entries[i];

				if (!strncmp(curr->comm, needle,
					     strlen(needle))) {
					match = true;
					break;
				}
			}
			if (match)
				break;
		}
	}
	rcu_read_unlock();

	z_policy->audio_active = match;
	z_policy->audio_cache_stamp_ns = now;
	return match;
}

/* Camera capture-pipeline comm match.  Picks names commonly used by
 * Android camera framework / HAL processes:
 *   - cameraserver       framework cameraserver process
 *   - cameraprovider     newer Treble cameraprovider
 *   - provider@          HIDL camera HAL service threads
 *                        ("provider@2.4-se", "provider@2.5-se", ...)
 *   - provider.MTK       MediaTek vendor variant
 *   - mtkcam-            MediaTek ISP/camera daemon threads
 *   - mtkcamutil         MediaTek camera utility threads
 *   - Camera2-           framework Camera2 internal threads
 *   - CamX_              Qualcomm CamX HAL threads
 *   - CamX-              CamX subsystem threads (alt naming)
 *   - vendor.qti.camera  Qualcomm vendor camera service
 *
 * Order is tuned for cache-friendliness: most common matches first.
 */
static const char * const zenith_camera_comms[] = {
	"cameraserver",
	"cameraprovider",
	"provider@",
	"provider.MTK",
	"mtkcam-",
	"mtkcamutil",
	"Camera2-",
	"CamX_",
	"CamX-",
	"vendor.qti.camera",
};

/* Walk the policy's online cpumask and check each cpu_curr's comm
 * against zenith_camera_comms[].  Returns true on the first match.
 * Cached for ZENITH_CAMERA_CACHE_TTL_NS.  Caller is expected to
 * gate on tunables->camera_aware != 0; this helper does not
 * re-check that.  The returned value is the *raw* comm-match
 * decision; the caller applies the camera_active override on top.
 */
static bool zenith_policy_has_camera(struct zenith_policy *z_policy)
{
	u64 now = ktime_get_ns();
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned int cpu;
	bool match = false;

	if (z_policy->camera_cache_stamp_ns &&
	    now - z_policy->camera_cache_stamp_ns < ZENITH_CAMERA_CACHE_TTL_NS)
		return z_policy->camera_auto_match;

	rcu_read_lock();
	for_each_cpu(cpu, policy->cpus) {
		struct task_struct *curr = READ_ONCE(cpu_curr(cpu));
		int i;

		if (!curr)
			continue;
		{
			struct zenith_comm_table *t =
				rcu_dereference(zenith_camera_table);

			if (!t)
				continue;
			for (i = 0; i < t->nr; i++) {
				const char *needle = t->entries[i];

				if (!strncmp(curr->comm, needle,
					     strlen(needle))) {
					match = true;
					break;
				}
			}
		}
		if (match)
			break;
	}
	rcu_read_unlock();

	z_policy->camera_auto_match = match;
	z_policy->camera_cache_stamp_ns = now;
	return match;
}

/* Predicate used by the cached_raw_freq shortcut in
 * zenith_get_next_freq().  Returns true when the efficient_freq
 * ladder has any armed bin deadline; in that case the cache hit
 * cannot be used because the ladder loop must run to drain
 * deadlines on schedule.  Walking ZENITH_EFF_BINS_MAX (small) once
 * per tick is cheap and avoids the latch hazard.
 */
static bool zenith_ladder_pending(struct zenith_policy *z_policy)
{
	unsigned int nr = z_policy->tunables->eff_nr;
	int i;

	if (!nr)
		return false;
	if (nr > ZENITH_EFF_BINS_MAX)
		nr = ZENITH_EFF_BINS_MAX;
	for (i = 0; i < nr; i++)
		if (z_policy->eff_unlock_at_ns[i])
			return true;
	return false;
}

/* Map a tp_path string to a stats bucket.  Called once per
 * zenith_get_next_freq() evaluation right before return, so the
 * cost is one O(few-strcmp) chain per decision -- negligible vs.
 * the rest of the eval pass.  Order is by frequency-of-hit so the
 * common case (eas / hispeed) short-circuits early.  Catches
 * everything; unknown tags fall through to ZENITH_STAT_OTHER.
 */
static enum zenith_stat_idx zenith_path_to_bucket(const char *path)
{
	if (unlikely(!path))
		return ZENITH_STAT_EAS;
	if (!strcmp(path, "eas"))
		return ZENITH_STAT_EAS;
	if (!strcmp(path, "hispeed"))
		return ZENITH_STAT_HISPEED;
	if (!strncmp(path, "input_boost", 11))	/* input_boost / _decay */
		return ZENITH_STAT_INPUT_BOOST;
	if (!strcmp(path, "snap_max") ||
	    !strcmp(path, "brutal_hold") ||
	    !strcmp(path, "climb_step"))
		return ZENITH_STAT_BRUTAL;
	if (!strcmp(path, "frame_pace"))
		return ZENITH_STAT_FRAME_PACE;
	if (!strcmp(path, "audio_floor") || !strcmp(path, "audio_cap"))
		return ZENITH_STAT_AUDIO;
	if (!strcmp(path, "render_floor") || !strcmp(path, "camera_floor"))
		return ZENITH_STAT_RENDER_CAMERA;
	if (!strcmp(path, "uclamp_min_floor") ||
	    !strcmp(path, "uclamp_max_cap"))
		return ZENITH_STAT_UCLAMP;
	if (!strncmp(path, "psi_", 4))		/* psi_mem_cap / _cpu / _io */
		return ZENITH_STAT_PSI;
	if (!strcmp(path, "boot_boost"))
		return ZENITH_STAT_BOOT_BOOST;
	if (!strcmp(path, "light_cap"))
		return ZENITH_STAT_LIGHT_CAP;
	if (!strcmp(path, "em_cap"))
		return ZENITH_STAT_EM_CAP;
	return ZENITH_STAT_OTHER;
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
	/* Set when freq is pinned by an explicit user-experience tier
	 * (input_boost full-pin, brutality snap_max / brutal_hold,
	 * climb_step).  Suppresses the post-resolve efficient-freq
	 * ladder and light-load hard cap so those tiers can't clip a
	 * boost back down to a lower bin.  EM validation and the
	 * sampling-down multiplier still run; both are correctness
	 * tiers, not user-experience clips.
	 */
	bool pin_to_target = false;

	/* Dynamic Environment Overrides */
	unsigned int dynamic_up_thresh = zenith_tunable_or_local(z_policy,
		z_policy->tunables->up_threshold,
		z_policy->at_effective_up_threshold);
	unsigned int dynamic_bias = z_policy->tunables->powersave_bias;

	/* ADPF / uclamp_min floor.  Sampled once here so every decision
	 * tier below (screen-off override, light-load cap, powersave_bias,
	 * final resolve) sees a consistent view.  zero when CONFIG_UCLAMP_TASK
	 * is off, when no task on the policy has set uclamp_min, or when
	 * the governor-level tunable disables respect entirely.
	 */
	unsigned long uclamp_min = z_policy->tunables->uclamp_min_respect ?
		zenith_policy_uclamp_min(z_policy) : 0;
	bool uclamp_min_meaningful = uclamp_min >=
		((SCHED_CAPACITY_SCALE * ZENITH_UCLAMP_MIN_MEANINGFUL_PCT) / 100);

	/* Input-boost decay floor.  Computed in the input-boost block
	 * below when we are in the trailing decay window of an active
	 * boost; applied as a minimum on the final freq just before the
	 * resolve label.  Zero means no floor (full-boost phase, or no
	 * boost at all).
	 */
	unsigned int input_boost_floor = 0;

	/* ADPF / uclamp_max cap.  Sampled once so every decision tier
	 * below sees a consistent view.  SCHED_CAPACITY_SCALE means
	 * "no cap" -- the helper returns that sentinel when uclamp is
	 * not in use, when no task has set a UCLAMP_MAX, or when the
	 * governor-level respect tunable is off.
	 */
	unsigned long uclamp_max = z_policy->tunables->uclamp_max_respect ?
		zenith_policy_uclamp_max(z_policy) : SCHED_CAPACITY_SCALE;

	if (z_policy->tunables->screen_state == 0 && !uclamp_min_meaningful) {
		dynamic_up_thresh = 95; /* Hard to wake up */
		dynamic_bias = 500;     /* 50% penalty */
		z_policy->brutal_active = false; /* no hysteresis screen-off */
	} else if (z_policy->tunables->screen_state == 0) {
		/* Screen is off but userspace has set a meaningful ADPF
		 * uclamp_min on at least one task in this policy --
		 * respect the hint.  Skip the 50 %% penalty and the
		 * 95 %% up_threshold bump; fall through to the normal
		 * eval path.  The final-freq floor applied below still
		 * guarantees we deliver at least the uclamp_min-implied
		 * frequency.
		 */
		z_policy->brutal_active = false;
	} else if (zenith_thermal_active(z_policy)) {
		/* Hot.  Default behaviour: snap dynamic_up_thresh to 90
		 * (the legacy cliff).  When thermal_pressure_continuous
		 * is set, ramp from the policy's normal up_threshold
		 * (at 0%% pressure) to 90 (at 100%% pressure) using the
		 * same arch_scale_thermal_pressure()-derived percentage
		 * the auto-tune V2 classifier uses; that removes the
		 * audible / observable step that otherwise happens the
		 * moment thermal_active flips on after a long burst.
		 */
		if (z_policy->tunables->thermal_pressure_continuous) {
			unsigned int floor = zenith_tunable_or_local(z_policy,
				z_policy->tunables->up_threshold,
				z_policy->at_effective_up_threshold);
			unsigned int pct =
				zenith_policy_thermal_pressure_pct(z_policy);

			if (pct > 100)
				pct = 100;
			if (floor < 90)
				dynamic_up_thresh = floor +
					((90U - floor) * pct) / 100U;
			else
				dynamic_up_thresh = floor;
		} else {
			dynamic_up_thresh = 90; /* Relaxed for thermals */
		}
	} else if (z_policy->tunables->up_threshold_hispeed &&
		   zenith_eff_hispeed_freq(z_policy) &&
		   policy->cur >= zenith_eff_hispeed_freq(z_policy)) {
		/* Above the hispeed floor, require the stiffer
		 * threshold before escalating all the way to
		 * policy->max. Screen-off and thermal overrides take
		 * precedence because both already pin an even
		 * higher value.
		 */
		dynamic_up_thresh = z_policy->tunables->up_threshold_hispeed;
	} else if (z_policy->tunables->up_threshold_adaptive &&
		   dynamic_up_thresh == zenith_tunable_or_local(z_policy,
				z_policy->tunables->up_threshold,
				z_policy->at_effective_up_threshold)) {
		/* Variance-adaptive shaping: lower dynamic_up_thresh by
		 * up to up_threshold_adaptive percent of its value when
		 * the recent load signal is bursty.  See
		 * ZENITH_DEFAULT_UP_THRESHOLD_ADAPTIVE for semantics.
		 * Skipped when any of the harder overrides above is in
		 * effect (those are absolute pinning values and must not
		 * be softened).  load_var_ewma_x256 is in 1/256ths of a
		 * percentage-point of mean abs change; 30*256 == fully
		 * saturated bursty signal.
		 */
		unsigned int adaptive =
			z_policy->tunables->up_threshold_adaptive;
		unsigned int var = z_policy->load_var_ewma_x256 / 256;
		unsigned int swing;

		if (adaptive > ZENITH_UP_THRESHOLD_ADAPTIVE_MAX)
			adaptive = ZENITH_UP_THRESHOLD_ADAPTIVE_MAX;
		if (var > ZENITH_UP_THRESHOLD_ADAPTIVE_MAX)
			var = ZENITH_UP_THRESHOLD_ADAPTIVE_MAX;
		/* swing = up_threshold * adaptive% * (var/30)
		 *       = up_threshold * adaptive * var
		 *         / (100 * ZENITH_UP_THRESHOLD_ADAPTIVE_MAX)
		 */
		swing = ((unsigned int)dynamic_up_thresh * adaptive * var) /
			(100u * ZENITH_UP_THRESHOLD_ADAPTIVE_MAX);
		if (swing < dynamic_up_thresh)
			dynamic_up_thresh -= swing;
	}

	/* prefer_silver_aware coordination: when prefer_silver is hot
	 * and this policy belongs to a big / prime cluster, raise
	 * dynamic_up_thresh by prefer_silver_hot_bump_pct points
	 * (clamped to ZENITH_PREFER_SILVER_HOT_BUMP_MAX_PCT) so the
	 * big cluster down-clocks less aggressively during sustained
	 * UI / app workloads where prefer_silver is steering the
	 * light wake-ups onto the silver/LITTLE cluster.  Skipped on
	 * the little cluster (already absorbing the redirected work)
	 * and skipped whenever a harder override above has pinned
	 * dynamic_up_thresh strictly higher than the natural
	 * up_threshold (screen-off, thermal cliff, hispeed pin) —
	 * those values are absolute and must not be inflated further.
	 *
	 * The (dynamic_up_thresh <= natural) test deliberately allows
	 * the bump to ride on top of the variance-adaptive shaping
	 * lower in the same chain (which only ever lowers
	 * dynamic_up_thresh below natural), preserving its smoothing
	 * effect while restoring the climb resistance prefer_silver
	 * was eroding by hiding light load from this cluster.
	 */
	if (z_policy->tunables->prefer_silver_aware &&
	    z_policy->cluster_class != ZENITH_CLUSTER_LITTLE &&
	    z_policy->ps_hit_rate_pct >=
		    z_policy->tunables->prefer_silver_hot_threshold_pct) {
		unsigned int natural = zenith_tunable_or_local(z_policy,
			z_policy->tunables->up_threshold,
			z_policy->at_effective_up_threshold);

		if (dynamic_up_thresh <= natural) {
			unsigned int bump =
			    z_policy->tunables->prefer_silver_hot_bump_pct;

			if (bump > ZENITH_PREFER_SILVER_HOT_BUMP_MAX_PCT)
				bump = ZENITH_PREFER_SILVER_HOT_BUMP_MAX_PCT;
			if (dynamic_up_thresh + bump <= 95)
				dynamic_up_thresh += bump;
			else
				dynamic_up_thresh = 95;
		}
	}

	if (max_cap)
		tp_load_pct = (unsigned int)((util * 100) / max_cap);

	/* 0. Input Boost — pin to policy->max for the non-decay portion of
	 * input_boost_ms after a key or touch event, then linearly ramp
	 * down across the trailing input_boost_decay_ms so the gesture
	 * tail doesn't cliff-drop back to the load-dependent target.
	 * Gated by screen_state so we don't wake clusters while the
	 * display is off, and optionally gated by input_boost_big_only
	 * so small-cluster policies skip the boost on heterogeneous SoCs.
	 */
	if (zenith_tunable_or_local(z_policy, z_policy->tunables->input_boost_ms,
				    z_policy->at_effective_input_boost_ms) &&
	    z_policy->tunables->screen_state &&
	    (!z_policy->tunables->input_boost_big_only ||
	     z_policy->is_big_cluster)) {
		u64 now = ktime_get_ns();
		u64 until = (u64)atomic64_read(&zenith_input_boost_until_ns);

		if (now < until) {
			unsigned int decay_ms =
				z_policy->tunables->input_boost_decay_ms;
			u64 decay_ns;
			u64 remaining = until - now;
			unsigned int cap_pct = zenith_tunable_or_local(
				z_policy, z_policy->tunables->input_boost_cap_pct,
				z_policy->at_effective_input_boost_cap_pct);
			unsigned int gm = zenith_tunable_or_local(
				z_policy, z_policy->tunables->game_mode,
				z_policy->at_effective_game_mode);
			unsigned int boost_ceiling;

			/* game_mode=2 (turbo) overrides the user-set cap and
			 * pins the full-boost phase to policy->max regardless
			 * of input_boost_cap_pct.  Runtime-only; the stored
			 * tunable is left untouched.
			 */
			if (gm >= 2)
				cap_pct = 0;
			boost_ceiling = (cap_pct && cap_pct <= 100) ?
				(policy->max / 100) * cap_pct : policy->max;

			/* game_mode decay stretch: lengthen the trailing decay
			 * window.  Level 1 uses ZENITH_GAME_BOOST_DECAY_PCT
			 * (default 130%%, ~30%% longer); level 2 uses
			 * ZENITH_GAME_L2_BOOST_DECAY_PCT (default 160%%,
			 * ~60%% longer) so input boosts hold the floor even
			 * longer across stick-flick / camera pan inputs.
			 * Pure no-op when game_mode=0.
			 */
			if (gm >= 2 && decay_ms)
				decay_ms = (decay_ms *
					    ZENITH_GAME_L2_BOOST_DECAY_PCT) / 100;
			else if (gm == 1 && decay_ms)
				decay_ms = (decay_ms *
					    ZENITH_GAME_BOOST_DECAY_PCT) / 100;
			decay_ns = (u64)decay_ms * NSEC_PER_MSEC;

			/* Latch the deadline for the boost-exit hold-down
			 * (step 7).  Refreshed on every active-boost tick so
			 * the stretched down-rate window starts from the
			 * actual boost expiry, not from when the latch was
			 * first set.
			 */
			z_policy->boost_active_until_ns = until;

			/* A capped ceiling that lands below policy->min would
			 * push the decay floor negative; clamp to min so the
			 * floor always remains a no-op or upward force.
			 */
			if (boost_ceiling < policy->min)
				boost_ceiling = policy->min;

			if (remaining > decay_ns) {
				/* Full-boost phase: pin to the capped ceiling
				 * (or policy->max when no cap is set).
				 */
				freq = boost_ceiling;
				tp_path = "input_boost";
				pin_to_target = true;
				goto apply_uclamp_max_cap;
			} else if (decay_ns) {
				/* Decay phase: ramp a floor from boost_ceiling
				 * down toward policy->min over the trailing
				 * decay_ns.  Shape is picked by
				 * input_boost_decay_curve: 0 = linear (the
				 * historical behaviour), 1 = cubic ease-in
				 * (floor holds high for most of the window,
				 * drops fast at the tail).  Normal eval runs
				 * after this point and may pick a higher freq;
				 * the floor only kicks in if the load has
				 * already dropped so far that eval undershoots
				 * the ramp.
				 */
				u64 elapsed = decay_ns - remaining;
				u64 span = boost_ceiling - policy->min;
				unsigned int dropped;

				if (z_policy->tunables->input_boost_decay_curve) {
					u32 t256 = (u32)div64_u64(elapsed * 256,
									  decay_ns);
					u32 cubic;

					if (t256 > 256)
						t256 = 256;
					cubic = (t256 * t256 * t256) >> 16;
					if (cubic > 256)
						cubic = 256;
					dropped = (unsigned int)
						((span * cubic) >> 8);
				} else {
					dropped = (unsigned int)
						div64_u64(span * elapsed,
							  decay_ns);
				}

				input_boost_floor = boost_ceiling - dropped;
			} else {
				/* Decay window not configured: original cliff. */
				freq = boost_ceiling;
				tp_path = "input_boost";
				pin_to_target = true;
				goto apply_uclamp_max_cap;
			}
		}
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
			atomic_inc(&z_policy->at_samples_total);
			if (load_pct >= z_policy->tunables->auto_tune_sat_load_pct)
				atomic_inc(&z_policy->at_samples_saturated);
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
			unsigned int b_streak =
				z_policy->tunables->brutal_entry_streak;
			unsigned int climb_mode =
				z_policy->tunables->climb_mode;

			/* game_mode=2 (turbo) forces SNAP climb regardless of
			 * the user-set climb_mode, so the brutality path
			 * always pins policy->max on threshold crossing.
			 * Runtime-only; the stored tunable is left untouched.
			 */
			if (zenith_tunable_or_local(z_policy,
					z_policy->tunables->game_mode,
					z_policy->at_effective_game_mode) >= 2)
				climb_mode = ZENITH_CLIMB_MODE_SNAP;

			if (climb_mode == ZENITH_CLIMB_MODE_STEP) {
				/* Gentle climb: step by freq_step_pct of
				 * policy->max from the current bin.
				 * Bypasses hysteresis entirely, including
				 * brutal_entry_streak -- STEP mode is
				 * already gentle by design so there is
				 * nothing to debounce.
				 */
				unsigned int step =
				    (policy->max *
				     z_policy->tunables->freq_step_pct) / 100;

				/* Load-proportional scaling (I4).  When
				 * freq_step_adaptive is set, widen the step
				 * linearly with overshoot above the active
				 * up_threshold so genuinely heavy load
				 * converges faster without changing the
				 * light-overshoot behaviour at the boundary.
				 * dynamic_up_thresh already reflects the
				 * screen-off / thermal / hispeed overrides
				 * above, so the span denominator is always
				 * the effective ceiling for this sample.
				 */
				if (z_policy->tunables->freq_step_adaptive &&
				    dynamic_up_thresh < 100) {
					unsigned int span =
						100 - dynamic_up_thresh;
					unsigned int overshoot =
						load_pct > dynamic_up_thresh ?
						load_pct - dynamic_up_thresh : 0;
					if (overshoot > span)
						overshoot = span;
					step += (step * overshoot) / span;
				}

				if (!step)
					step = 1;
				freq = policy->cur + step;
				if (freq > policy->max)
					freq = policy->max;
				z_policy->brutal_active = false;
				z_policy->brutal_entry_count = 0;
				tp_path = "climb_step";
				pin_to_target = true;
				goto apply_uclamp_max_cap;
			}

			/* Entry-side streak hysteresis for the brutality
			 * snap-to-max path.  Mirror of the hispeed tier:
			 * increment the entry counter each qualifying
			 * sample (saturate at the cap to avoid overflow),
			 * and only flip brutal_active once the counter
			 * exceeds tunables->brutal_entry_streak.
			 * b_streak == 0 collapses to the historical
			 * immediate-flip behaviour because the fresh
			 * counter crosses 1 > 0 on the very first sample.
			 * Counter reset sites: the STEP path above, the
			 * below-threshold fall-through, and the brutal_hold
			 * exit (all three mean "not in entry territory
			 * anymore").
			 */
			if (!z_policy->brutal_active) {
				if (z_policy->brutal_entry_count <
				    ZENITH_BRUTAL_ENTRY_STREAK_MAX)
					z_policy->brutal_entry_count++;
				if (z_policy->brutal_entry_count <= b_streak) {
					/* Streak not satisfied yet -- fall
					 * through to the hispeed / EAS path
					 * below without flipping brutal_active.
					 * No goto: the hispeed tier gets a
					 * chance to apply its own floor.
					 */
					goto brutal_entry_deferred;
				}
			}

			z_policy->brutal_active = true;
			z_policy->brutal_entry_count = 0;
			freq = policy->max;
			tp_path = "snap_max";
			pin_to_target = true;
			goto apply_uclamp_max_cap;
		}
		z_policy->brutal_entry_count = 0;

brutal_entry_deferred:
		if ((z_policy->tunables->climb_mode == ZENITH_CLIMB_MODE_SNAP ||
		     zenith_tunable_or_local(z_policy,
				z_policy->tunables->game_mode,
				z_policy->at_effective_game_mode) >= 2) &&
		    z_policy->brutal_active) {
			unsigned int eff_down =
				zenith_tunable_or_local(z_policy,
					READ_ONCE(z_policy->tunables->down_threshold),
					z_policy->at_effective_down_threshold);
			unsigned int adaptive = zenith_tunable_or_local(z_policy,
				READ_ONCE(z_policy->tunables->down_threshold_adaptive),
				z_policy->at_effective_down_threshold_adaptive);

			if (adaptive) {
				unsigned int var =
					z_policy->load_var_ewma_x256 / 256;
				unsigned int swing;

				if (adaptive > ZENITH_DOWN_THRESHOLD_ADAPTIVE_MAX)
					adaptive = ZENITH_DOWN_THRESHOLD_ADAPTIVE_MAX;
				if (var > ZENITH_DOWN_THRESHOLD_ADAPTIVE_MAX)
					var = ZENITH_DOWN_THRESHOLD_ADAPTIVE_MAX;
				swing = (eff_down * adaptive * var) /
					(100u * ZENITH_DOWN_THRESHOLD_ADAPTIVE_MAX);
				if (swing < eff_down)
					eff_down -= swing;
			}

			if (load_pct >= eff_down) {
				freq = policy->max;
				tp_path = "brutal_hold";
				pin_to_target = true;
				goto apply_uclamp_max_cap;
			}
		}

		/* Brutal-hold cliff exit.  When tunables->brutal_decay_ms
		 * is non-zero, arm a tail-glide deadline so the EAS
		 * post-floor below tapers freq from policy->max down to
		 * the load-dependent target across the configured window
		 * instead of cliff-dropping in a single tick.  The
		 * deadline is sticky: it survives until either expired
		 * or replaced by a fresh brutal-hold re-entry (which
		 * implicitly clears it on the next exit).  No-op when
		 * brutal_decay_ms == 0.
		 */
		if (z_policy->brutal_active &&
		    z_policy->tunables->brutal_decay_ms) {
			unsigned int decay_ms =
				z_policy->tunables->brutal_decay_ms;

			if (decay_ms > ZENITH_BRUTAL_DECAY_MS_MAX)
				decay_ms = ZENITH_BRUTAL_DECAY_MS_MAX;
			z_policy->brutal_decay_arm_ms = decay_ms;
			z_policy->brutal_decay_until_ns = ktime_get_ns() +
				(u64)decay_ms * NSEC_PER_MSEC;
		}
		z_policy->brutal_active = false;
	}

	/* 2. Schedutil EAS Proportional Math with Headroom */
	if (arch_scale_freq_invariant())
		freq = policy->cpuinfo.max_freq;
	else
		freq = policy->cur + (policy->cur >> 2); 

	freq = map_util_freq(util, freq, max_cap);

	/* 2a. Brutal-hold tail glide.  When the cliff exit above armed
	 * a brutal_decay_ms deadline, linearly interpolate a floor
	 * between policy->max (at arm time) and the EAS-computed freq
	 * (at expiry).  Eliminates the audible / visible drop that the
	 * legacy cliff produces on bursty workloads.  Self-disarms once
	 * the deadline passes.  When the deadline is unarmed (the
	 * common case), this whole block is a single zero-test branch.
	 */
	if (z_policy->brutal_decay_until_ns) {
		u64 now = ktime_get_ns();

		if (now >= z_policy->brutal_decay_until_ns) {
			z_policy->brutal_decay_until_ns = 0;
			z_policy->brutal_decay_arm_ms = 0;
		} else if (z_policy->brutal_decay_arm_ms) {
			u64 total_ns = (u64)z_policy->brutal_decay_arm_ms *
				NSEC_PER_MSEC;
			u64 remaining_ns =
				z_policy->brutal_decay_until_ns - now;
			unsigned int max_freq = policy->max;
			u64 span;

			if (max_freq > freq && total_ns) {
				span = (u64)(max_freq - freq) * remaining_ns;
				span = div64_u64(span, total_ns);
				if ((u64)freq + span > max_freq)
					freq = max_freq;
				else
					freq = freq + (unsigned int)span;
			}
		}
	}

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
	{
		unsigned int eff_hispeed = zenith_eff_hispeed_freq(z_policy);

		if (eff_hispeed && max_cap) {
			unsigned int load_pct = (util * 100) / max_cap;
			unsigned int entry = z_policy->tunables->hispeed_load;
			unsigned int hyst  = z_policy->tunables->hispeed_hyst_pct;
			unsigned int exit  = hyst < entry ? entry - hyst : 0;
			unsigned int streak =
				z_policy->tunables->hispeed_entry_streak;

			/* Entry-side streak hysteresis.  When streak == 0
			 * the historical immediate-flip behaviour is
			 * preserved (hispeed_entry_count crosses 1 > 0 on
			 * the very first qualifying sample).  When
			 * streak == N>0, require load_pct >= entry to
			 * hold for N+1 consecutive ticks before flipping.
			 * Counter is reset whenever load_pct drops below
			 * entry, and saturates at the streak cap so it
			 * cannot wrap.
			 */
			if (load_pct >= entry) {
				if (z_policy->hispeed_entry_count <
				    ZENITH_HISPEED_ENTRY_STREAK_MAX)
					z_policy->hispeed_entry_count++;
			} else {
				z_policy->hispeed_entry_count = 0;
			}

			if (!z_policy->hispeed_active &&
			    load_pct >= entry &&
			    z_policy->hispeed_entry_count > streak)
				z_policy->hispeed_active = true;
			else if (z_policy->hispeed_active && load_pct < exit)
				z_policy->hispeed_active = false;

			if (z_policy->hispeed_active && freq < eff_hispeed) {
				freq = eff_hispeed;
				tp_path = "hispeed";
			}
		} else {
			/* Tier disabled or max_cap == 0: drop the sticky bit
			 * so we don't carry it across a disable/enable cycle.
			 * Reset the streak counter too so we don't carry
			 * partial entry credit across a disable.
			 */
			z_policy->hispeed_active = false;
			z_policy->hispeed_entry_count = 0;
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

	/* 3b. uclamp_min floor (ADPF PerformanceHint).
	 *
	 * After every other tier has had its say, guarantee that the
	 * chosen freq is at least what uclamp_min would imply via
	 * map_util_freq().  Catches three concrete cases schedutil's
	 * RQ-aggregate path alone would miss:
	 *
	 *   - light_load_freq cap undershooting an ADPF hint;
	 *   - powersave_bias shaving the proportional freq below the hint;
	 *   - cached_raw_freq short-circuit below returning a rate-limited
	 *     stale value while uclamp_min has been raised in between ticks.
	 *
	 * Applied regardless of screen_state: the boolean above has
	 * already picked whether the screen-off override suppression
	 * fires, but the floor is useful in screen-on paths too (e.g.
	 * a light-load cap would otherwise clip a meaningful hint).
	 */
	if (uclamp_min && max_cap) {
		unsigned int uclamp_floor = map_util_freq(uclamp_min,
							  policy->cpuinfo.max_freq,
							  max_cap);
		if (freq < uclamp_floor) {
			freq = uclamp_floor;
			tp_path = "uclamp_min_floor";
		}
	}

	/* 3c. Input-boost decay floor.  When a boost is in its trailing
	 * decay window, the ramp floor computed in step 0 overrides
	 * anything lower the eval tiers produced.  Applied last so it
	 * doesn't short-circuit the normal load-demand logic when the
	 * load genuinely calls for more than the ramp allows.
	 */
	if (input_boost_floor && freq < input_boost_floor) {
		freq = input_boost_floor;
		tp_path = "input_boost_decay";
	}

	/* 3c0. Boot-boost floor.  When boot_boost_ms != 0 and we are
	 * still inside the boot_boost window (measured from
	 * ktime_get_boottime_ns()), pin the freq to policy->max.  Gated
	 * by screen_state and reuses the input_boost_big_only topology
	 * gate so the small cluster does not wake to max during boot.
	 */
	if (z_policy->tunables->boot_boost_ms &&
	    z_policy->tunables->screen_state &&
	    (!z_policy->tunables->input_boost_big_only ||
	     z_policy->is_big_cluster)) {
		u64 deadline_ns = (u64)z_policy->tunables->boot_boost_ms *
				  NSEC_PER_MSEC;

		if (ktime_get_boottime_ns() < deadline_ns) {
			if (freq < policy->max) {
				freq = policy->max;
				tp_path = "boot_boost";
			}
		}
	}

	/* 3c''. Adaptive frame-budget floor.  Userspace-driven; sees
	 * the current vblank period via tunables->frame_budget_us and
	 * the calibrated 60 Hz baseline floor via
	 * tunables->frame_pace_floor_pct.  The effective floor scales
	 * inversely with the budget so 90 / 120 / 144 Hz refresh rates
	 * automatically lift the floor.  See the comment block at the
	 * top of the file for the formula.
	 */
	/* Read both tunables exactly once.  Without READ_ONCE the
	 * compiler is free to re-fetch frame_budget_us between the
	 * gating check and the divide below; if userspace writes 0 in
	 * that window the divide oopses.  Same reasoning for base_pct
	 * (a 0 here just skips the floor, but a torn read past the
	 * upper bound check could yield eff_pct overflow).
	 */
	{
		unsigned int anchor = cpumask_first(policy->cpus);
		unsigned int budget_us = (anchor < NR_CPUS) ?
			READ_ONCE(z_policy->tunables->
				frame_budget_us_per_policy[anchor]) : 0;
		unsigned int base_pct =
			zenith_tunable_or_local(z_policy,
				READ_ONCE(z_policy->tunables->frame_pace_floor_pct),
				z_policy->at_effective_frame_pace_floor_pct);

		/* Per-policy override of zero falls through to the
		 * global frame_budget_us.  See ZENITH_DEFAULT_FRAME_
		 * BUDGET_US per-policy comment block.
		 */
		if (!budget_us)
			budget_us =
			  READ_ONCE(z_policy->tunables->frame_budget_us);

		if (budget_us && base_pct) {
			unsigned int eff_pct;
			unsigned int fp_floor;

			eff_pct = (base_pct *
				   ZENITH_FRAME_PACE_BASE_BUDGET_US) /
				  budget_us;
			if (eff_pct > 100)
				eff_pct = 100;
			fp_floor = (policy->max * eff_pct) / 100;
			if (fp_floor > policy->max)
				fp_floor = policy->max;
			if (trace_zenith_frame_pace_enabled())
				trace_zenith_frame_pace(
					cpumask_first(policy->cpus),
					budget_us, eff_pct, fp_floor);
			if (freq < fp_floor) {
				freq = fp_floor;
				tp_path = "frame_pace";
			}
		}
	}

	/* 3c''. Audio low-jitter floor.  When audio_aware=1 and any CPU
	 * in this policy is currently running a known audio thread
	 * (AudioOut_*, audioserver, MediaCodec_*, ...), apply a freq
	 * floor of (policy->max * audio_floor_pct / 100).  Caches the
	 * comm walk for ZENITH_AUDIO_CACHE_TTL_NS.  The audio cap_pct
	 * companion is applied separately below, just before the
	 * uclamp_max final cap, so an explicit ADPF hint can still
	 * walk it down further.  audio_floor_pct=0 leaves the comm
	 * walk running (for tracepoint visibility) but applies no
	 * floor.
	 */
	if (ZENITH_FEATURE_ENABLED(audio_aware)) {
		bool has_audio = zenith_policy_has_audio(z_policy);
		unsigned int af_pct = z_policy->tunables->audio_floor_pct;
		unsigned int ac_pct = z_policy->tunables->audio_cap_pct;
		unsigned int af = af_pct ? (policy->max * af_pct) / 100 : 0;
		unsigned int ac = ac_pct ? (policy->max * ac_pct) / 100 : 0;

		if (af > policy->max)
			af = policy->max;
		if (ac > policy->max)
			ac = policy->max;
		if (trace_zenith_audio_band_enabled())
			trace_zenith_audio_band(
				cpumask_first(policy->cpus),
				has_audio, af_pct, ac_pct,
				has_audio ? af : 0,
				has_audio ? ac : 0);
		if (has_audio && af && freq < af) {
			freq = af;
			tp_path = "audio_floor";
		}
	}

	/* 3c'. Render-thread / display-pipeline floor.  When
	 * render_aware=1 and any CPU in this policy is currently running
	 * a known render / display-pipeline thread (RenderThread,
	 * surfaceflinger, ...), apply a freq floor of
	 * (policy->max * render_floor_pct / 100).  Caches the comm walk
	 * for ZENITH_RENDER_CACHE_TTL_NS to keep the hot path cheap.
	 * Floor is still capped by the uclamp_max tier below.
	 */
	if (ZENITH_FEATURE_ENABLED(render_aware) &&
	    z_policy->tunables->render_floor_pct) {
		bool has_render = zenith_policy_has_render(z_policy);
		unsigned int rf = (policy->max *
				   z_policy->tunables->render_floor_pct) /
				  100;

		if (rf > policy->max)
			rf = policy->max;
		if (trace_zenith_render_floor_enabled())
			trace_zenith_render_floor(
				cpumask_first(policy->cpus),
				has_render,
				z_policy->tunables->render_floor_pct,
				has_render ? rf : 0);
		if (has_render && freq < rf) {
			freq = rf;
			tp_path = "render_floor";
		}
	}

	/* 3c''''. Camera capture-pipeline floor.  When camera_aware=1
	 * and either (camera_active=force-on) or (camera_active=auto
	 * && comm walk finds a known camera HAL/framework thread on
	 * the policy), apply a freq floor of
	 * (policy->max * camera_floor_pct / 100).  No companion cap:
	 * camera bursts benefit from full freq headroom; capping
	 * costs frame-rate.  Floor is still subject to the uclamp_max
	 * downstream cap.
	 *
	 * When camera_active=force-off, the comm walk is skipped and
	 * no floor is applied even if the table would have matched.
	 * The override exists because vendor camera HALs sometimes
	 * use thread names that don't match the table.
	 */
	if (ZENITH_FEATURE_ENABLED(camera_aware)) {
		unsigned int override = z_policy->tunables->camera_active;
		bool auto_match = false;
		bool active;
		unsigned int cf_pct = z_policy->tunables->camera_floor_pct;
		unsigned int cf;

		if (override == ZENITH_CAMERA_OVERRIDE_FORCE_OFF) {
			active = false;
		} else if (override == ZENITH_CAMERA_OVERRIDE_FORCE_ON) {
			active = true;
		} else {
			auto_match = zenith_policy_has_camera(z_policy);
			active = auto_match;
		}

		cf = cf_pct ? (policy->max * cf_pct) / 100 : 0;
		if (cf > policy->max)
			cf = policy->max;

		if (trace_zenith_camera_floor_enabled())
			trace_zenith_camera_floor(
				cpumask_first(policy->cpus),
				active, auto_match, override, cf_pct,
				active ? cf : 0);

		if (active && cf && freq < cf) {
			freq = cf;
			tp_path = "camera_floor";
		}
	}

	/* 3c'''. Audio low-jitter cap.  Companion to the audio floor
	 * tier above: when audio_aware=1, an audio thread is enqueued
	 * on the policy, and audio_cap_pct > 0, cap freq at
	 * (policy->max * audio_cap_pct / 100).  Applied after every
	 * floor tier so it can pull the freq down even when render_floor
	 * / boot_boost / input_boost would otherwise hold it higher.
	 * Applied *before* the uclamp_max final cap so an explicit ADPF
	 * power-efficiency hint can still walk it down further.  When
	 * the user sets audio_cap_pct < audio_floor_pct, the cap takes
	 * precedence (the floor block ran earlier; this block then
	 * pulls back).
	 */
	if (ZENITH_FEATURE_ENABLED(audio_aware) &&
	    z_policy->tunables->audio_cap_pct) {
		unsigned int ac = (policy->max *
				   z_policy->tunables->audio_cap_pct) / 100;

		if (ac > policy->max)
			ac = policy->max;
		if (z_policy->audio_active && freq > ac) {
			freq = ac;
			tp_path = "audio_cap";
		}
	}

apply_uclamp_max_cap:
	/* 3d. uclamp_max final-freq cap.  Applied after every other tier
	 * so that brutality snap, hispeed floor, input boost, and the
	 * uclamp_min / input_boost_decay floors can't walk over an
	 * explicit power-efficiency hint.  A uclamp_min floor higher
	 * than the uclamp_max cap wins by construction (floor applies
	 * first, cap would clamp it down below uclamp_min only when the
	 * two hints disagree, and per-task uclamp validation already
	 * prevents that at the scheduler layer).
	 *
	 * Reachable both from the natural fall-through path AND from
	 * the input_boost / brutality "goto" sites, so an ADPF
	 * uclamp_max hint can walk down even an explicitly-pinned
	 * boost target (the user's "save power on this thread"
	 * intent should beat the governor's "this is interactive"
	 * heuristic).
	 */
	if (uclamp_max < SCHED_CAPACITY_SCALE && max_cap) {
		unsigned int uclamp_cap = map_util_freq(uclamp_max,
							policy->cpuinfo.max_freq,
							max_cap);
		if (freq > uclamp_cap) {
			freq = uclamp_cap;
			tp_path = "uclamp_max_cap";
		}
	}

	/* 3e. PSI memory-pressure cap.  When psi_aware=1 and the system
	 * is over the configured 10s memory-pressure threshold, cap the
	 * final freq at the effective hispeed floor (or policy->max as
	 * fallback when the hispeed tier is disabled).  Rationale: under
	 * heavy memstall, going above hispeed mostly burns energy on
	 * cycles that stall waiting for memory.  Boot-boost (3c0) sits
	 * higher in the chain so the boot window is preserved even with
	 * psi_aware=1.  pin_to_target=true (input_boost / brutality)
	 * skips the PSI cap so a touch-driven boost wins even under
	 * memstall; the uclamp_max cap above is still authoritative.
	 */
	if (!pin_to_target &&
	    ZENITH_FEATURE_ENABLED(psi_aware) &&
	    (z_policy->tunables->psi_mem_thresh ||
	     z_policy->tunables->psi_cpu_thresh ||
	     z_policy->tunables->psi_io_thresh)) {
		const char *psi_tag = NULL;

		if (z_policy->tunables->psi_mem_thresh &&
		    zenith_psi_mem_some_pct() >=
		    z_policy->tunables->psi_mem_thresh)
			psi_tag = "psi_mem_cap";
		else if (z_policy->tunables->psi_cpu_thresh &&
			 zenith_psi_cpu_some_pct() >=
			 z_policy->tunables->psi_cpu_thresh)
			psi_tag = "psi_cpu_cap";
		else if (z_policy->tunables->psi_io_thresh &&
			 zenith_psi_io_some_pct() >=
			 z_policy->tunables->psi_io_thresh)
			psi_tag = "psi_io_cap";

		if (psi_tag) {
			unsigned int psi_cap =
				zenith_eff_hispeed_freq(z_policy);

			if (!psi_cap)
				psi_cap = policy->max;
			if (freq > psi_cap) {
				freq = psi_cap;
				tp_path = psi_tag;
			}
		}
	}

	/* cached_raw_freq shortcut: when the pre-resolve freq matches
	 * the value we cached on the previous tick AND nothing has
	 * marked need_freq_update, the post-resolve tiers (ladder,
	 * light_cap, EM, sampling-down) all produced the same answer
	 * last tick, so we can return the cached final value.
	 *
	 * EXCEPTION: if the efficient_freq ladder has any armed bin
	 * deadline (eff_unlock_at_ns[i] != 0), we MUST run the ladder
	 * loop again so deadlines can release on schedule.  Otherwise a
	 * sustained sub-up_threshold load that holds freq steady would
	 * latch the ladder at its current bin forever, because the
	 * cache hit short-circuits past the loop that consumes them.
	 */
	if (freq == z_policy->cached_raw_freq && !z_policy->need_freq_update &&
	    !zenith_ladder_pending(z_policy)) {
		z_policy->stats[ZENITH_STAT_DECISIONS]++;
		z_policy->stats[ZENITH_STAT_CACHE_HITS]++;
		/* Emit the same summary tracepoint on cache-hit so a
		 * trace consumer sees a continuous record of decisions
		 * rather than gaps every time the cache shortcut wins.
		 * Gated on trace_zenith_decision_enabled() so the cache
		 * hot path stays free when tracing is off.
		 */
		if (trace_zenith_decision_enabled()) {
			unsigned int kc_pct = per_cpu(zenith_cpu, policy->cpu)
						.kc_filtered_busy_pct;
			trace_zenith_decision(policy->cpu, tp_path, util,
					      max_cap, tp_load_pct, freq,
					      z_policy->next_freq, kc_pct,
					      z_policy->cached_uclamp_min,
					      z_policy->cached_uclamp_max,
					      true);
		}
		return z_policy->next_freq;
	}

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
	 * efficient_freq=0).  pin_to_target=true (input_boost full-pin,
	 * brutality snap_max / brutal_hold, climb_step) skips the
	 * ladder so the user-experience tier wins.
	 */
	if (!pin_to_target && z_policy->tunables->eff_nr) {
		unsigned int nr = z_policy->tunables->eff_nr;
		u64 now = ktime_get_ns();
		int i;

		if (nr > ZENITH_EFF_BINS_MAX)
			nr = ZENITH_EFF_BINS_MAX;

		for (i = 0; i < nr; i++) {
			unsigned int bin_freq = z_policy->tunables->eff_freq[i];
			u64 delay_ns = (u64)z_policy->tunables->eff_delay_us[i] *
				       NSEC_PER_USEC;

			/* Clamp the bin to this policy's max.  The
			 * efficient_freq table is set on the global
			 * tunables and may serve multiple policies with
			 * different policy->max values.  Bins above
			 * policy->max would otherwise be permanently
			 * unreachable (target_freq is already bounded
			 * by policy->max upstream), silently disabling
			 * the upper rungs of the ladder for the smaller
			 * cluster.  Collapsing to policy->max gives the
			 * operator the intuitive behaviour: "the table
			 * extends through this cluster's max".
			 */
			if (bin_freq > policy->max)
				bin_freq = policy->max;

			if (target_freq <= bin_freq) {
				/* Target is at or below this bin.  With
				 * hysteresis enabled, only release the
				 * bin (clear its and every higher bin's
				 * wait-deadline) if target dropped past
				 * bin_freq * (100 - hyst_pct) / 100.
				 * Otherwise the deadline is preserved
				 * but target_freq is left as-is, so a
				 * re-cross of bin_freq doesn't have to
				 * re-arm the bin.  See ZENITH_DEFAULT_
				 * EFF_BIN_HYST_PCT for the why.
				 */
				unsigned int hyst = READ_ONCE(
					z_policy->tunables->eff_bin_hyst_pct);

				if (hyst) {
					unsigned int margin =
						(bin_freq * hyst) / 100;
					unsigned int release = bin_freq -
						margin;

					if (target_freq > release)
						break;
				}
				{
					int j;
					for (j = i; j < nr; j++)
						z_policy->eff_unlock_at_ns[j] = 0;
				}
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
	 * pin_to_target=true skips this cap for the same reason as the
	 * efficient-freq ladder above.
	 */
	if (!pin_to_target &&
	    z_policy->tunables->light_load_freq && max_cap &&
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

	/* 7. Sampling-down multiplier: extend the down-rate delay by
	 * sampling_down_factor while we are either (a) sitting at
	 * policy->max, or (b) within one stretched down-rate window of
	 * a recently-exited input boost (boost_exit_extend).  The latter
	 * smooths the gesture tail: after the input-boost full-pin phase
	 * ends, normal eval may briefly pick a much lower freq while
	 * PELT catches up to the post-boost workload, and dropping the
	 * sampling multiplier the instant target falls below max
	 * produces a perceptible undershoot.  Reset the multiplier (and
	 * clear the latch) the moment both conditions are false.
	 */
	{
		unsigned int sdf = max(z_policy->tunables->sampling_down_factor,
					1U);
		bool boost_exit_active = false;

		if (z_policy->tunables->boost_exit_extend &&
		    z_policy->boost_active_until_ns) {
			u64 now_ns = ktime_get_ns();
			u64 stretch_ns = (u64)z_policy->tunables->down_rate_limit_us *
					 NSEC_PER_USEC * sdf;

			if (now_ns < z_policy->boost_active_until_ns +
				     stretch_ns)
				boost_exit_active = true;
			else
				z_policy->boost_active_until_ns = 0;
		}

		if (target_freq >= policy->max || boost_exit_active)
			z_policy->down_rate_mult = sdf;
		else
			z_policy->down_rate_mult = 1;
	}

	if (trace_zenith_decision_enabled()) {
		/* Report the leader CPU's filtered kcpustat busy% so a
		 * trace consumer can tell at a glance whether the
		 * decision was lifted by the kcpustat blend.  Reads 0
		 * when the feature is off (sampler is gated by
		 * kcpustat_hispeed_enable in update_util) or when no
		 * recent activity has populated the sampler.
		 */
		unsigned int kc_pct =
			per_cpu(zenith_cpu, policy->cpu).kc_filtered_busy_pct;

		trace_zenith_decision(policy->cpu, tp_path, util, max_cap,
				      tp_load_pct, freq, target_freq, kc_pct,
				      z_policy->cached_uclamp_min,
				      z_policy->cached_uclamp_max,
				      false);
	}

	z_policy->stats[ZENITH_STAT_DECISIONS]++;
	z_policy->stats[zenith_path_to_bucket(tp_path)]++;

	/* Update the variance EWMA used by the up_threshold_adaptive
	 * shaping at the top of the next eval.  Uses tp_load_pct as the
	 * input signal (computed earlier in this function).  Cheap: one
	 * abs-diff, one shift, one add.  See ZENITH_DEFAULT_UP_THRESHOLD_
	 * ADAPTIVE for what consumes load_var_ewma_x256.  Updated even
	 * when up_threshold_adaptive is 0 so flipping the tunable on
	 * doesn't see a stale-zero variance for the first 8 samples.
	 */
	{
		unsigned int prev = z_policy->last_load_pct;
		unsigned int delta = tp_load_pct > prev ?
				tp_load_pct - prev : prev - tp_load_pct;
		z_policy->load_var_ewma_x256 =
			(z_policy->load_var_ewma_x256 * 7 + delta * 256) / 8;
		z_policy->last_load_pct = tp_load_pct;
	}

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

	if (next_freq < z_policy->next_freq) {
		unsigned int margin_pct =
			READ_ONCE(z_policy->tunables->freq_stability_margin_pct);

		if (margin_pct) {
			unsigned int margin;

			if (margin_pct > ZENITH_FREQ_STABILITY_MARGIN_PCT_MAX)
				margin_pct = ZENITH_FREQ_STABILITY_MARGIN_PCT_MAX;
			margin = (z_policy->policy->max * margin_pct) / 100;
			if (z_policy->next_freq - next_freq <= margin)
				return;
		}
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
	if (READ_ONCE(tunables->wakeup_boost) && max_cap) {
		unsigned int cur_pct = (unsigned int)((util * 100) / max_cap);
		unsigned int prev_pct = z_cpu->wakeup_prev_util ?
			(unsigned int)((z_cpu->wakeup_prev_util * 100) /
				       max_cap) : 0;

		if (prev_pct < ZENITH_WAKEUP_IDLE_THRESH_PCT &&
		    cur_pct >= ZENITH_WAKEUP_BUSY_THRESH_PCT)
			z_cpu->wakeup_boost_ticks = ZENITH_WAKEUP_BOOST_TICKS;
	}
	z_cpu->wakeup_prev_util = util;

	if (tunables->kcpustat_hispeed_enable) {
		zenith_kcpustat_sample(z_cpu,
				       tunables->kcpustat_window_us,
				       tunables->kcpustat_filter_shift,
				       time);
		util = zenith_kcpustat_blend(z_cpu, util, max_cap, time);
	}

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
			if (READ_ONCE(tunables->wakeup_boost) && j_max) {
				unsigned int cur_pct =
					(unsigned int)((j_util * 100) / j_max);
				unsigned int prev_pct =
					j_z_cpu->wakeup_prev_util ?
					(unsigned int)((j_z_cpu->wakeup_prev_util *
							100) / j_max) : 0;

				if (prev_pct < ZENITH_WAKEUP_IDLE_THRESH_PCT &&
				    cur_pct >= ZENITH_WAKEUP_BUSY_THRESH_PCT)
					j_z_cpu->wakeup_boost_ticks =
						ZENITH_WAKEUP_BOOST_TICKS;
			}
			j_z_cpu->wakeup_prev_util = j_util;

			if (tunables->kcpustat_hispeed_enable) {
				zenith_kcpustat_sample(j_z_cpu,
					tunables->kcpustat_window_us,
					tunables->kcpustat_filter_shift,
					time);
				j_util = zenith_kcpustat_blend(j_z_cpu, j_util,
							       j_max, time);
			}

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

static inline struct zenith_tunables *to_zenith_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct zenith_tunables, attr_set);
}

/* Recompute z_policy->min_rate_limit_ns from the current cached
 * up/down delays.  Reads use READ_ONCE so concurrent updaters of
 * either delay (sysfs writers, profile switches, auto_tune) cannot
 * tear our snapshot, and the result is published with WRITE_ONCE
 * so the hot-path reader (zenith_should_update_freq) observes a
 * coherent value.  The min_rate_lock mutex previously used here is
 * gone: readers and writers are all single-instruction loads/stores
 * of a 64-bit aligned scalar, and the worst-case stale read costs
 * exactly one rate-limit tick of latency (already bounded by the
 * hot path anyway).
 */
static void update_min_rate_limit_ns(struct zenith_policy *z_policy)
{
	s64 up_ns = READ_ONCE(z_policy->up_rate_delay_ns);
	s64 down_ns = READ_ONCE(z_policy->down_rate_delay_ns);

	WRITE_ONCE(z_policy->min_rate_limit_ns, min(up_ns, down_ns));
}

static void zenith_update_cluster_rate_scale(struct zenith_policy *z_policy)
{
	unsigned int little_thresh =
		(SCHED_CAPACITY_SCALE * ZENITH_CLUSTER_LITTLE_THRESH_PCT) / 100;
	unsigned int cluster_cap = 0;
	unsigned int big_cap = 0;
	unsigned int cpu;

	for_each_cpu(cpu, z_policy->policy->cpus) {
		unsigned int cap = arch_scale_cpu_capacity(cpu);

		if (cap > cluster_cap)
			cluster_cap = cap;
	}
	for_each_possible_cpu(cpu) {
		unsigned int cap = arch_scale_cpu_capacity(cpu);

		if (cap > big_cap)
			big_cap = cap;
	}
	if (cluster_cap < little_thresh)
		z_policy->cluster_class = ZENITH_CLUSTER_LITTLE;
	else if (big_cap && cluster_cap >= big_cap)
		z_policy->cluster_class = ZENITH_CLUSTER_PRIME;
	else
		z_policy->cluster_class = ZENITH_CLUSTER_BIG;
	if (z_policy->tunables->rate_limit_cluster_scale &&
	    cluster_cap < little_thresh) {
		z_policy->up_rate_scale = 2;
		z_policy->down_rate_scale_shift = 1;
	} else {
		z_policy->up_rate_scale = 1;
		z_policy->down_rate_scale_shift = 0;
	}
}

static void zenith_update_rate_delay_ns(struct zenith_policy *z_policy)
{
	struct zenith_tunables *t = z_policy->tunables;
	unsigned int up_us = zenith_tunable_or_local(z_policy,
		t->up_rate_limit_us, z_policy->at_effective_up_rate_limit_us);
	unsigned int down_us = zenith_tunable_or_local(z_policy,
		t->down_rate_limit_us,
		z_policy->at_effective_down_rate_limit_us);
	s64 up_ns = (u64)up_us * NSEC_PER_USEC;
	s64 down_ns = (u64)down_us * NSEC_PER_USEC;

	up_ns *= max(z_policy->up_rate_scale, 1U);
	down_ns >>= z_policy->down_rate_scale_shift;
	WRITE_ONCE(z_policy->up_rate_delay_ns, up_ns);
	WRITE_ONCE(z_policy->down_rate_delay_ns, down_ns);
	update_min_rate_limit_ns(z_policy);
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

/* Refresh the per-policy {up,down}_rate_delay_ns caches on every
 * policy sharing this tunables set after a profile change has
 * mutated the {up,down}_rate_limit_us fields.  Without this, the
 * hot path keeps reading the previous profile's cached delays
 * (zenith_up_down_rate_limit() reads z_policy->up_rate_delay_ns,
 * not tunables->up_rate_limit_us).  Caller must hold the
 * attr_set->update_lock; sysfs profile_store always does.
 */
static void zenith_refresh_rate_delays(struct gov_attr_set *attr_set)
{
	struct zenith_policy *z_pol;

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		zenith_update_cluster_rate_scale(z_pol);
		zenith_update_rate_delay_ns(z_pol);
	}
}

/* Update the per-policy prefer_silver hit-rate snapshot.
 *
 * Called once per V1 classifier window from zenith_auto_tune_work().
 * Reads the global prefer_silver hit / miss atomic counters via the
 * accessor exported by kernel/sched/prefer_silver.c, computes the
 * delta against the previous window's snapshot, and stores the
 * resulting hit-rate as a percentage (0..100) on z_policy for
 * zenith_get_next_freq() to consume.  When the rate crosses the
 * tunable's hot threshold, ORs ZENITH_AT_FLAG_PREFER_SILVER_HOT
 * into *flags so the at_log dump and trace events reflect the
 * trigger.
 *
 * On builds without CONFIG_SCHED_PREFER_SILVER the accessor symbol
 * is unavailable, so the helper is a no-op stub and the cached
 * hit-rate stays at zero — the downstream bump in
 * zenith_get_next_freq() never fires regardless of the tunable.
 */
static void zenith_at_update_prefer_silver_rate(struct zenith_policy *z_policy,
						struct zenith_tunables *t,
						unsigned int *flags)
{
#ifdef CONFIG_SCHED_PREFER_SILVER
	unsigned int hit_now = 0, miss_now = 0;
	unsigned int hit_delta, miss_delta, total_delta;
	unsigned int rate;

	prefer_silver_get_hit_miss(&hit_now, &miss_now);

	hit_delta  = hit_now  - z_policy->ps_prev_hit;
	miss_delta = miss_now - z_policy->ps_prev_miss;
	total_delta = hit_delta + miss_delta;

	z_policy->ps_prev_hit  = hit_now;
	z_policy->ps_prev_miss = miss_now;

	if (!total_delta) {
		/* No prefer_silver activity in this window.  Decay the
		 * cached rate towards zero so a brief idle period
		 * cannot leave a stale "hot" reading behind.
		 */
		z_policy->ps_hit_rate_pct = 0;
		return;
	}

	rate = (hit_delta * 100U) / total_delta;
	if (rate > 100)
		rate = 100;
	z_policy->ps_hit_rate_pct = rate;

	if (t->prefer_silver_aware &&
	    rate >= t->prefer_silver_hot_threshold_pct)
		*flags |= ZENITH_AT_FLAG_PREFER_SILVER_HOT;
#else
	(void)z_policy;
	(void)t;
	(void)flags;
#endif
}

/* Push a one-shot sample into the per-policy auto-tune ring buffer.
 *
 * Called from the tail of zenith_auto_tune_work() after the V1
 * classifier has resolved a target and the V2 state machine has
 * settled on a state.  Reads the freshly-populated at_last_* mirrors
 * on z_policy so the caller does not have to redundantly thread the
 * same dozen-odd values; only from_state is taken as a parameter
 * because at_last_state has already been overwritten with the new
 * state by the time we get here.
 *
 * The ring is a simple head-advances-then-wraps circular buffer.
 * The worker is the only writer and it is single-shot per policy
 * (a delayed_work, not a timer with overlapping fire), so no
 * locking is needed on the writer side.  Sysfs readers walk the
 * ring under no extra lock and accept tearing on the wrap window;
 * the on-disk semantics are "last N samples observed by the worker",
 * which is exactly what diagnostics want.
 */
static void zenith_at_log_push(struct zenith_policy *z_policy,
			       unsigned int from_state,
			       unsigned int v1_target,
			       bool emergency)
{
	struct zenith_at_log_entry *e;
	unsigned int slot;

	slot = z_policy->at_log_head;
	if (slot >= ZENITH_AT_LOG_NR)
		slot = 0;
	e = &z_policy->at_log[slot];

	e->ts_ns		= ktime_get_ns();
	e->flags		= z_policy->at_last_flags;
	e->var_x256		= z_policy->at_last_var_x256;
	e->thermal_pressure	= z_policy->at_last_thermal_pressure;
	e->sat_pct		= z_policy->at_last_sat_pct;
	e->events_rate_x2	= z_policy->at_last_events_rate_x2;
	e->thermal_slope	= z_policy->at_last_thermal_slope;
	e->v1_target		= (u8)v1_target;
	e->v2_from_state	= (u8)from_state;
	e->v2_to_state		= (u8)z_policy->at_last_state;
	e->reason		= (u8)z_policy->at_last_reason;
	e->emergency		= emergency ? 1 : 0;

	z_policy->at_log_head = (slot + 1) % ZENITH_AT_LOG_NR;
	if (z_policy->at_log_count < ZENITH_AT_LOG_NR)
		z_policy->at_log_count++;
}

/* Reset all observability counters on a single policy.
 *
 * Clears both the per-policy zenith_stats[] array and the auto-tune
 * classifier ring.  Used by the zenith_stats_reset sysfs node so an
 * operator can mark a "start of measurement" point before a benchmark
 * without having to re-init the governor.  The decision-tier counters
 * are racy with the fast path (the per-CPU update path increments
 * them with a plain ++), but that race is bounded to a few lost
 * counts on the boundary which is acceptable for diagnostics.
 */
static void zenith_policy_observability_reset(struct zenith_policy *z_policy)
{
	memset(z_policy->stats, 0, sizeof(z_policy->stats));
	memset(z_policy->at_log, 0, sizeof(z_policy->at_log));
	z_policy->at_log_head = 0;
	z_policy->at_log_count = 0;
}

static void zenith_reset_local_actions(struct zenith_policy *z_policy)
{
	z_policy->at_effective_up_rate_limit_us = z_policy->tunables->up_rate_limit_us;
	z_policy->at_effective_down_rate_limit_us =
		z_policy->tunables->down_rate_limit_us;
	z_policy->at_effective_up_threshold = z_policy->tunables->up_threshold;
	z_policy->at_effective_down_threshold =
		z_policy->tunables->down_threshold;
	z_policy->at_effective_input_boost_ms =
		z_policy->tunables->input_boost_ms;
	z_policy->at_effective_input_boost_cap_pct =
		z_policy->tunables->input_boost_cap_pct;
	z_policy->at_effective_down_rate_adaptive =
		z_policy->tunables->down_rate_adaptive;
	z_policy->at_effective_down_threshold_adaptive =
		z_policy->tunables->down_threshold_adaptive;
	z_policy->at_effective_frame_pace_floor_pct =
		z_policy->tunables->frame_pace_floor_pct;
	z_policy->at_effective_game_mode = z_policy->tunables->game_mode;
	z_policy->at_local_actions = false;
	zenith_update_rate_delay_ns(z_policy);
}

/* Single-policy variant of zenith_refresh_rate_delays() for the
 * auto_tune classifier worker.  The worker runs from delayed_work
 * context and does NOT hold attr_set->update_lock, so iterating
 * policy_list there would race with concurrent gov_attr_set_get/put.
 * The per-policy z_policy that owns the worker is guaranteed live
 * (zenith_exit() cancels the worker before unlinking).  Other
 * policies sharing the same tunables pick up the new delays on
 * their own next auto_tune tick.
 */
static void zenith_refresh_rate_delays_one(struct zenith_policy *z_policy)
{
	zenith_update_cluster_rate_scale(z_policy);
	zenith_update_rate_delay_ns(z_policy);
}

static const char *zenith_profile_name(unsigned int profile)
{
	switch (profile) {
	case ZENITH_PROFILE_PERFORMANCE:
		return "performance";
	case ZENITH_PROFILE_BALANCED:
		return "balanced";
	case ZENITH_PROFILE_BATTERY:
		return "battery";
	case ZENITH_PROFILE_LEGACY:
		return "legacy";
	case ZENITH_PROFILE_CUSTOM:
	default:
		return "custom";
	}
}

static const char *zenith_at_state_name(unsigned int state)
{
	switch (state) {
	case ZENITH_AT_STATE_EFFICIENCY:
		return "efficiency";
	case ZENITH_AT_STATE_BALANCED:
		return "balanced";
	case ZENITH_AT_STATE_LATENCY:
		return "latency";
	case ZENITH_AT_STATE_SUSTAINED_PERF:
		return "sustained_perf";
	case ZENITH_AT_STATE_THERMAL_RECOVERY:
		return "thermal_recovery";
	default:
		return "unknown";
	}
}

static const char *zenith_at_reason_name(unsigned int reason)
{
	switch (reason) {
	case ZENITH_AT_REASON_CLASSIFIER:
		return "classifier";
	case ZENITH_AT_REASON_CAMERA_RENDER:
		return "camera_render";
	case ZENITH_AT_REASON_MEMSTALL:
		return "memstall";
	case ZENITH_AT_REASON_AUDIO:
		return "audio";
	case ZENITH_AT_REASON_VARIANCE:
		return "variance";
	case ZENITH_AT_REASON_THERMAL:
		return "thermal";
	case ZENITH_AT_REASON_COOLDOWN:
		return "cooldown";
	case ZENITH_AT_REASON_HYSTERESIS:
		return "hysteresis";
	case ZENITH_AT_REASON_SCREEN:
		return "screen";
	case ZENITH_AT_REASON_PSI:
		return "psi";
	case ZENITH_AT_REASON_FRAME:
		return "frame";
	case ZENITH_AT_REASON_GAME:
		return "game";
	case ZENITH_AT_REASON_THERMAL_SLOPE:
		return "thermal_slope";
	default:
		return "unknown";
	}
}

static unsigned int zenith_at_clamp(unsigned int val, unsigned int lo,
				    unsigned int hi)
{
	if (val < lo)
		return lo;
	if (val > hi)
		return hi;
	return val;
}

struct zenith_at_guardrails {
	unsigned int up_rate_min;
	unsigned int up_rate_max;
	unsigned int down_rate_min;
	unsigned int down_rate_max;
	unsigned int up_threshold_min;
	unsigned int up_threshold_max;
	unsigned int down_threshold_min;
	unsigned int down_threshold_max;
	unsigned int input_boost_min;
	unsigned int input_boost_max;
	unsigned int input_cap_min;
	unsigned int input_cap_max;
	unsigned int down_adaptive_min;
	unsigned int down_adaptive_max;
	unsigned int down_thresh_adaptive_min;
	unsigned int down_thresh_adaptive_max;
	unsigned int frame_floor_min;
	unsigned int frame_floor_max;
	unsigned int game_mode_min;
	unsigned int game_mode_max;
};

static void zenith_at_get_guardrails(unsigned int profile,
				     struct zenith_at_guardrails *g)
{
	switch (profile) {
	case ZENITH_PROFILE_PERFORMANCE:
		*g = (struct zenith_at_guardrails) {
			.up_rate_min = 0, .up_rate_max = 250,
			.down_rate_min = 4000, .down_rate_max = 12000,
			.up_threshold_min = 55, .up_threshold_max = 75,
			.down_threshold_min = 35, .down_threshold_max = 60,
			.input_boost_min = 100, .input_boost_max = 220,
			.input_cap_min = 0, .input_cap_max = 100,
			.down_adaptive_min = 1, .down_adaptive_max = 1,
			.down_thresh_adaptive_min = 5,
			.down_thresh_adaptive_max = 15,
			.frame_floor_min = 25, .frame_floor_max = 70,
			.game_mode_min = 0, .game_mode_max = 2,
		};
		break;
	case ZENITH_PROFILE_BATTERY:
		*g = (struct zenith_at_guardrails) {
			.up_rate_min = 250, .up_rate_max = 1000,
			.down_rate_min = 1000, .down_rate_max = 4000,
			.up_threshold_min = 78, .up_threshold_max = 95,
			.down_threshold_min = 35, .down_threshold_max = 55,
			.input_boost_min = 0, .input_boost_max = 80,
			.input_cap_min = 50, .input_cap_max = 75,
			.down_adaptive_min = 0, .down_adaptive_max = 1,
			.down_thresh_adaptive_min = 0,
			.down_thresh_adaptive_max = 5,
			.frame_floor_min = 0, .frame_floor_max = 35,
			.game_mode_min = 0, .game_mode_max = 1,
		};
		break;
	case ZENITH_PROFILE_LEGACY:
		*g = (struct zenith_at_guardrails) {
			.up_rate_min = 1000, .up_rate_max = 4000,
			.down_rate_min = 2000, .down_rate_max = 8000,
			.up_threshold_min = 75, .up_threshold_max = 90,
			.down_threshold_min = 70, .down_threshold_max = 90,
			.input_boost_min = 0, .input_boost_max = 0,
			.input_cap_min = 0, .input_cap_max = 0,
			.down_adaptive_min = 0, .down_adaptive_max = 0,
			.down_thresh_adaptive_min = 0,
			.down_thresh_adaptive_max = 0,
			.frame_floor_min = 0, .frame_floor_max = 0,
			.game_mode_min = 0, .game_mode_max = 0,
		};
		break;
	case ZENITH_PROFILE_CUSTOM:
	case ZENITH_PROFILE_BALANCED:
	default:
		*g = (struct zenith_at_guardrails) {
			.up_rate_min = 50, .up_rate_max = 500,
			.down_rate_min = 2500, .down_rate_max = 7000,
			.up_threshold_min = 65, .up_threshold_max = 85,
			.down_threshold_min = 45, .down_threshold_max = 70,
			.input_boost_min = 60, .input_boost_max = 160,
			.input_cap_min = 65, .input_cap_max = 90,
			.down_adaptive_min = 0, .down_adaptive_max = 1,
			.down_thresh_adaptive_min = 0,
			.down_thresh_adaptive_max = 10,
			.frame_floor_min = 15, .frame_floor_max = 55,
			.game_mode_min = 0, .game_mode_max = 2,
		};
		break;
	}
}

static void zenith_at_get_policy_guardrails(struct zenith_policy *z_policy,
					    unsigned int profile,
					    struct zenith_at_guardrails *g)
{
	zenith_at_get_guardrails(profile, g);
	if (!z_policy->tunables->auto_tune_cluster_aware)
		return;

	switch (z_policy->cluster_class) {
	case ZENITH_CLUSTER_LITTLE:
		g->up_rate_min = max(g->up_rate_min,
				     (g->up_rate_min + g->up_rate_max) / 2);
		g->input_boost_max = min(g->input_boost_max,
					 (g->input_boost_min +
					  g->input_boost_max) / 2);
		g->frame_floor_max = min(g->frame_floor_max,
					 (g->frame_floor_min +
					  g->frame_floor_max) / 2);
		g->game_mode_max = min(g->game_mode_max, 1U);
		break;
	case ZENITH_CLUSTER_PRIME:
		g->up_rate_max = min(g->up_rate_max,
				     (g->up_rate_min + g->up_rate_max) / 2);
		break;
	case ZENITH_CLUSTER_BIG:
	default:
		break;
	}
}

static unsigned int zenith_at_state_to_profile(unsigned int state)
{
	switch (state) {
	case ZENITH_AT_STATE_EFFICIENCY:
	case ZENITH_AT_STATE_THERMAL_RECOVERY:
		return ZENITH_PROFILE_BATTERY;
	case ZENITH_AT_STATE_LATENCY:
	case ZENITH_AT_STATE_SUSTAINED_PERF:
		return ZENITH_PROFILE_PERFORMANCE;
	case ZENITH_AT_STATE_BALANCED:
	default:
		return ZENITH_PROFILE_BALANCED;
	}
}

static unsigned int zenith_profile_to_at_state(unsigned int profile)
{
	switch (profile) {
	case ZENITH_PROFILE_PERFORMANCE:
		return ZENITH_AT_STATE_LATENCY;
	case ZENITH_PROFILE_BATTERY:
		return ZENITH_AT_STATE_EFFICIENCY;
	case ZENITH_PROFILE_BALANCED:
	case ZENITH_PROFILE_LEGACY:
	case ZENITH_PROFILE_CUSTOM:
	default:
		return ZENITH_AT_STATE_BALANCED;
	}
}

static const char *zenith_at_cluster_name(unsigned int cluster)
{
	switch (cluster) {
	case ZENITH_CLUSTER_LITTLE:
		return "little";
	case ZENITH_CLUSTER_BIG:
		return "big";
	case ZENITH_CLUSTER_PRIME:
		return "prime";
	default:
		return "unknown";
	}
}

static unsigned int zenith_at_profile_for_state(struct zenith_policy *z_policy,
						unsigned int state)
{
	if (!z_policy->tunables->auto_tune_cluster_aware)
		return zenith_at_state_to_profile(state);

	switch (z_policy->cluster_class) {
	case ZENITH_CLUSTER_LITTLE:
		if (state == ZENITH_AT_STATE_LATENCY ||
		    state == ZENITH_AT_STATE_SUSTAINED_PERF)
			return ZENITH_PROFILE_BALANCED;
		return zenith_at_state_to_profile(state);
	case ZENITH_CLUSTER_PRIME:
		if (state == ZENITH_AT_STATE_BALANCED)
			return ZENITH_PROFILE_PERFORMANCE;
		return zenith_at_state_to_profile(state);
	case ZENITH_CLUSTER_BIG:
	default:
		return zenith_at_state_to_profile(state);
	}
}

static void zenith_at_mark_override(struct zenith_tunables *t,
				    unsigned long bit)
{
	t->auto_tune_override_mask |= bit;
}

static bool zenith_at_set_uint(struct zenith_tunables *t, unsigned long bit,
			       unsigned int *field, unsigned int val,
			       unsigned int lo, unsigned int hi)
{
	if (t->auto_tune_override_mask & bit)
		return false;
	val = zenith_at_clamp(val, lo, hi);
	if (*field == val)
		return false;
	*field = val;
	return true;
}

static unsigned int zenith_tunable_or_local(struct zenith_policy *z_policy,
					    unsigned int tunable,
					    unsigned int local)
{
	return z_policy->at_local_actions ? local : tunable;
}

static void zenith_at_write_effective(struct zenith_policy *z_policy,
				      struct zenith_at_guardrails *g,
				      unsigned int up_rate,
				      unsigned int down_rate,
				      unsigned int up_th,
				      unsigned int down_th,
				      unsigned int boost_ms,
				      unsigned int boost_cap,
				      unsigned int down_adapt,
				      unsigned int down_th_adapt,
				      unsigned int frame_floor,
				      unsigned int game_mode)
{
	struct zenith_tunables *t = z_policy->tunables;

	z_policy->at_effective_up_rate_limit_us =
		(t->auto_tune_override_mask & ZENITH_AT_OVERRIDE_UP_RATE) ?
		t->up_rate_limit_us : zenith_at_clamp(up_rate,
						      g->up_rate_min,
						      g->up_rate_max);
	z_policy->at_effective_down_rate_limit_us =
		(t->auto_tune_override_mask & ZENITH_AT_OVERRIDE_DOWN_RATE) ?
		t->down_rate_limit_us : zenith_at_clamp(down_rate,
							g->down_rate_min,
							g->down_rate_max);
	z_policy->at_effective_up_threshold =
		(t->auto_tune_override_mask & ZENITH_AT_OVERRIDE_UP_THRESHOLD) ?
		t->up_threshold : zenith_at_clamp(up_th,
						  g->up_threshold_min,
						  g->up_threshold_max);
	z_policy->at_effective_down_threshold =
		(t->auto_tune_override_mask &
		 ZENITH_AT_OVERRIDE_DOWN_THRESHOLD) ?
		t->down_threshold : zenith_at_clamp(down_th,
						    g->down_threshold_min,
						    g->down_threshold_max);
	z_policy->at_effective_input_boost_ms =
		(t->auto_tune_override_mask & ZENITH_AT_OVERRIDE_INPUT_BOOST_MS) ?
		t->input_boost_ms : zenith_at_clamp(boost_ms,
						    g->input_boost_min,
						    g->input_boost_max);
	z_policy->at_effective_input_boost_cap_pct =
		(t->auto_tune_override_mask & ZENITH_AT_OVERRIDE_INPUT_BOOST_CAP) ?
		t->input_boost_cap_pct : zenith_at_clamp(boost_cap,
							 g->input_cap_min,
							 g->input_cap_max);
	z_policy->at_effective_down_rate_adaptive =
		(t->auto_tune_override_mask & ZENITH_AT_OVERRIDE_DOWN_ADAPTIVE) ?
		t->down_rate_adaptive : zenith_at_clamp(down_adapt,
							g->down_adaptive_min,
							g->down_adaptive_max);
	z_policy->at_effective_down_threshold_adaptive =
		(t->auto_tune_override_mask &
		 ZENITH_AT_OVERRIDE_DOWN_THRESH_ADAPTIVE) ?
		t->down_threshold_adaptive :
		zenith_at_clamp(down_th_adapt, g->down_thresh_adaptive_min,
				g->down_thresh_adaptive_max);
	z_policy->at_effective_frame_pace_floor_pct =
		(t->auto_tune_override_mask & ZENITH_AT_OVERRIDE_FRAME_PACE) ?
		t->frame_pace_floor_pct : zenith_at_clamp(frame_floor,
							  g->frame_floor_min,
							  g->frame_floor_max);
	z_policy->at_effective_game_mode =
		(t->auto_tune_override_mask & ZENITH_AT_OVERRIDE_GAME_MODE) ?
		t->game_mode : zenith_at_clamp(game_mode, g->game_mode_min,
					       g->game_mode_max);
	z_policy->at_local_actions = true;
	zenith_update_rate_delay_ns(z_policy);
	z_policy->need_freq_update = true;
}

static bool zenith_at_apply_actions(struct zenith_policy *z_policy,
				    unsigned int state)
{
	struct zenith_tunables *t = z_policy->tunables;
	struct zenith_at_guardrails g;
	unsigned int up_rate, down_rate, up_th, down_th;
	unsigned int boost_ms, boost_cap, down_adapt, down_th_adapt;
	unsigned int frame_floor = 0;
	unsigned int game_mode = 0;
	bool rate_changed = false;
	bool changed = false;

	zenith_at_get_policy_guardrails(z_policy, t->auto_tune_cluster_aware ?
					zenith_at_profile_for_state(z_policy,
								    state) :
					t->active_profile, &g);
	switch (state) {
	case ZENITH_AT_STATE_EFFICIENCY:
		up_rate = g.up_rate_max;
		down_rate = g.down_rate_min;
		up_th = g.up_threshold_max;
		down_th = g.down_threshold_min;
		boost_ms = g.input_boost_min;
		boost_cap = g.input_cap_min;
		down_adapt = g.down_adaptive_min;
		down_th_adapt = g.down_thresh_adaptive_min;
		frame_floor = g.frame_floor_min;
		game_mode = g.game_mode_min;
		break;
	case ZENITH_AT_STATE_LATENCY:
		up_rate = g.up_rate_min;
		down_rate = (g.down_rate_min + g.down_rate_max) / 2;
		up_th = g.up_threshold_min;
		down_th = g.down_threshold_min;
		boost_ms = g.input_boost_max;
		boost_cap = g.input_cap_max;
		down_adapt = g.down_adaptive_max;
		down_th_adapt = g.down_thresh_adaptive_max;
		frame_floor = (g.frame_floor_min + g.frame_floor_max) / 2;
		game_mode = min_t(unsigned int, g.game_mode_max, 1);
		break;
	case ZENITH_AT_STATE_SUSTAINED_PERF:
		up_rate = g.up_rate_min;
		down_rate = g.down_rate_max;
		up_th = g.up_threshold_min;
		down_th = g.down_threshold_max;
		boost_ms = g.input_boost_max;
		boost_cap = g.input_cap_max;
		down_adapt = g.down_adaptive_max;
		down_th_adapt = g.down_thresh_adaptive_max;
		frame_floor = g.frame_floor_max;
		game_mode = g.game_mode_max;
		break;
	case ZENITH_AT_STATE_THERMAL_RECOVERY:
		up_rate = g.up_rate_max;
		down_rate = g.down_rate_min;
		up_th = g.up_threshold_max;
		down_th = g.down_threshold_min;
		boost_ms = g.input_boost_min;
		boost_cap = g.input_cap_min;
		down_adapt = g.down_adaptive_min;
		down_th_adapt = g.down_thresh_adaptive_min;
		frame_floor = g.frame_floor_min;
		game_mode = g.game_mode_min;
		break;
	case ZENITH_AT_STATE_BALANCED:
	default:
		up_rate = (g.up_rate_min + g.up_rate_max) / 2;
		down_rate = (g.down_rate_min + g.down_rate_max) / 2;
		up_th = (g.up_threshold_min + g.up_threshold_max) / 2;
		down_th = (g.down_threshold_min + g.down_threshold_max) / 2;
		boost_ms = (g.input_boost_min + g.input_boost_max) / 2;
		boost_cap = (g.input_cap_min + g.input_cap_max) / 2;
		down_adapt = g.down_adaptive_max;
		down_th_adapt = (g.down_thresh_adaptive_min +
				  g.down_thresh_adaptive_max) / 2;
		frame_floor = (g.frame_floor_min + g.frame_floor_max) / 2;
		game_mode = min_t(unsigned int, g.game_mode_max, 1);
		break;
	}

	if (t->auto_tune_cluster_aware) {
		if (z_policy->cluster_class == ZENITH_CLUSTER_LITTLE) {
			up_rate = max(up_rate, (g.up_rate_min + g.up_rate_max) / 2);
			down_rate = g.down_rate_min;
			boost_ms = min(boost_ms,
				       (g.input_boost_min + g.input_boost_max) / 2);
			frame_floor = min(frame_floor,
					  (g.frame_floor_min + g.frame_floor_max) / 2);
			game_mode = min(game_mode, 1U);
		} else if (z_policy->cluster_class == ZENITH_CLUSTER_PRIME) {
			up_rate = g.up_rate_min;
			if (state == ZENITH_AT_STATE_LATENCY ||
			    state == ZENITH_AT_STATE_SUSTAINED_PERF) {
				boost_ms = g.input_boost_max;
				frame_floor = g.frame_floor_max;
			}
		}
	}

	if (t->auto_tune_frame_pacing && z_policy->at_last_frame_budget_us &&
	    (state == ZENITH_AT_STATE_LATENCY ||
	     state == ZENITH_AT_STATE_SUSTAINED_PERF))
		frame_floor = g.frame_floor_max;
	if (t->auto_tune_sustained_gaming &&
	    state == ZENITH_AT_STATE_SUSTAINED_PERF)
		game_mode = g.game_mode_max;
	if (t->auto_tune_thermal_slope &&
	    (z_policy->at_last_flags & ZENITH_AT_FLAG_THERMAL_SLOPE)) {
		down_rate = g.down_rate_min;
		down_th = g.down_threshold_min;
		boost_ms = g.input_boost_min;
		boost_cap = g.input_cap_min;
		frame_floor = g.frame_floor_min;
		game_mode = g.game_mode_min;
	}

	if (t->auto_tune_cluster_aware) {
		zenith_at_write_effective(z_policy, &g, up_rate, down_rate,
					  up_th, down_th, boost_ms, boost_cap,
					  down_adapt, down_th_adapt,
					  frame_floor, game_mode);
		return true;
	}

	rate_changed |= zenith_at_set_uint(t, ZENITH_AT_OVERRIDE_UP_RATE,
					   &t->up_rate_limit_us, up_rate,
					   g.up_rate_min, g.up_rate_max);
	rate_changed |= zenith_at_set_uint(t, ZENITH_AT_OVERRIDE_DOWN_RATE,
					   &t->down_rate_limit_us, down_rate,
					   g.down_rate_min, g.down_rate_max);
	changed |= zenith_at_set_uint(t, ZENITH_AT_OVERRIDE_UP_THRESHOLD,
				      &t->up_threshold, up_th,
				      g.up_threshold_min,
				      g.up_threshold_max);
	changed |= zenith_at_set_uint(t, ZENITH_AT_OVERRIDE_DOWN_THRESHOLD,
				      &t->down_threshold, down_th,
				      g.down_threshold_min,
				      g.down_threshold_max);
	changed |= zenith_at_set_uint(t, ZENITH_AT_OVERRIDE_INPUT_BOOST_MS,
				      &t->input_boost_ms, boost_ms,
				      g.input_boost_min, g.input_boost_max);
	changed |= zenith_at_set_uint(t, ZENITH_AT_OVERRIDE_INPUT_BOOST_CAP,
				      &t->input_boost_cap_pct, boost_cap,
				      g.input_cap_min, g.input_cap_max);
	changed |= zenith_at_set_uint(t, ZENITH_AT_OVERRIDE_DOWN_ADAPTIVE,
				      &t->down_rate_adaptive, down_adapt,
				      g.down_adaptive_min, g.down_adaptive_max);
	changed |= zenith_at_set_uint(t,
				      ZENITH_AT_OVERRIDE_DOWN_THRESH_ADAPTIVE,
				      &t->down_threshold_adaptive,
				      down_th_adapt,
				      g.down_thresh_adaptive_min,
				      g.down_thresh_adaptive_max);
	changed |= zenith_at_set_uint(t, ZENITH_AT_OVERRIDE_FRAME_PACE,
				      &t->frame_pace_floor_pct, frame_floor,
				      g.frame_floor_min, g.frame_floor_max);
	changed |= zenith_at_set_uint(t, ZENITH_AT_OVERRIDE_GAME_MODE,
				      &t->game_mode, game_mode,
				      g.game_mode_min, g.game_mode_max);
	if (rate_changed)
		zenith_refresh_rate_delays_one(z_policy);
	if (changed || rate_changed) {
		WRITE_ONCE(zenith_input_boost_active_ms, t->input_boost_ms);
		z_policy->need_freq_update = true;
	}
	return changed || rate_changed;
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

static ssize_t iowait_stack_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->iowait_stack_pct);
}

static ssize_t iowait_stack_pct_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	t->iowait_stack_pct = val;
	return count;
}
static struct governor_attr iowait_stack_pct = __ATTR_RW(iowait_stack_pct);

static ssize_t iowait_backoff_after_ms_show(struct gov_attr_set *attr_set,
					    char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->iowait_backoff_after_ms);
}

static ssize_t iowait_backoff_after_ms_store(struct gov_attr_set *attr_set,
					     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	/* 0 disables the backoff.  Cap at 60_000 ms (1 minute): an
	 * iowait episode lasting that long without a single quiet tick
	 * is exotic enough that the user almost certainly wants the
	 * backoff to kick in well before then; values above the cap
	 * are rejected so a typo doesn't silently disable the feature
	 * for hours.
	 */
	if (kstrtouint(buf, 10, &val) || val > 60000)
		return -EINVAL;
	t->iowait_backoff_after_ms = val;
	return count;
}
static struct governor_attr iowait_backoff_after_ms =
	__ATTR_RW(iowait_backoff_after_ms);

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
	struct zenith_profile_defaults {
		unsigned int profile;
		unsigned int up_rate_limit_us;
		unsigned int down_rate_limit_us;
		unsigned int up_threshold;
		unsigned int down_threshold;
		unsigned int hispeed_freq_pct;
		unsigned int hispeed_load;
		unsigned int climb_mode;
		unsigned int freq_step_pct;
		unsigned int powersave_bias;
		unsigned int bias_load_threshold;
		unsigned int ignore_nice_load;
		unsigned int input_boost_ms;
		unsigned int input_boost_decay_ms;
		unsigned int input_boost_cap_pct;
		unsigned int light_load_threshold;
		unsigned int sampling_down_factor;
		unsigned int thermal_auto;
		unsigned int screen_auto;
		unsigned int util_math_v2;
		unsigned int kcpustat_hispeed_enable;
		unsigned int down_rate_adaptive;
		unsigned int wakeup_boost;
		unsigned int down_threshold_adaptive;
		unsigned int rate_limit_cluster_scale;
	};
	static const struct zenith_profile_defaults profiles[] = {
		{
			.profile = ZENITH_PROFILE_PERFORMANCE,
			.up_rate_limit_us = 0,
			.down_rate_limit_us = 8000,
			.up_threshold = 65,
			.down_threshold = 45,
			.hispeed_freq_pct = 60,
			.hispeed_load = 55,
			.climb_mode = ZENITH_CLIMB_MODE_SNAP,
			.freq_step_pct = 15,
			.powersave_bias = 0,
			.bias_load_threshold = 50,
			.ignore_nice_load = 0,
			.input_boost_ms = 150,
			.input_boost_decay_ms = 50,
			.input_boost_cap_pct = 0,
			.light_load_threshold = 15,
			.sampling_down_factor = 4,
			.thermal_auto = 1,
			.screen_auto = 1,
			.util_math_v2 = 1,
			.kcpustat_hispeed_enable = 1,
			.down_rate_adaptive = 1,
			.wakeup_boost = 1,
			.down_threshold_adaptive = 10,
			.rate_limit_cluster_scale = 1,
		},
		{
			.profile = ZENITH_PROFILE_BALANCED,
			.up_rate_limit_us = ZENITH_DEFAULT_UP_RATE_LIMIT_US,
			.down_rate_limit_us = ZENITH_DEFAULT_DOWN_RATE_LIMIT_US,
			.up_threshold = ZENITH_DEFAULT_UP_THRESHOLD,
			.down_threshold = ZENITH_DEFAULT_DOWN_THRESHOLD,
			.hispeed_freq_pct = ZENITH_DEFAULT_HISPEED_FREQ_PCT,
			.hispeed_load = ZENITH_DEFAULT_HISPEED_LOAD,
			.climb_mode = ZENITH_CLIMB_MODE_SNAP,
			.freq_step_pct = ZENITH_DEFAULT_FREQ_STEP_PCT,
			.powersave_bias = 50,
			.bias_load_threshold = 40,
			.ignore_nice_load = 1,
			.input_boost_ms = ZENITH_DEFAULT_INPUT_BOOST_MS,
			.input_boost_decay_ms = ZENITH_DEFAULT_INPUT_BOOST_DECAY_MS,
			.input_boost_cap_pct = ZENITH_DEFAULT_INPUT_BOOST_CAP_PCT,
			.light_load_threshold = ZENITH_DEFAULT_LIGHT_LOAD_THRESHOLD,
			.sampling_down_factor = ZENITH_DEFAULT_SAMPLING_DOWN_FACTOR,
			.thermal_auto = 1,
			.screen_auto = 1,
			.util_math_v2 = 1,
			.kcpustat_hispeed_enable = 1,
			.down_rate_adaptive = 1,
			.wakeup_boost = 1,
			.down_threshold_adaptive = 5,
			.rate_limit_cluster_scale = 1,
		},
		{
			.profile = ZENITH_PROFILE_BATTERY,
			.up_rate_limit_us = 500,
			.down_rate_limit_us = 2000,
			.up_threshold = 85,
			.down_threshold = 40,
			.hispeed_freq_pct = 0,
			.hispeed_load = 75,
			.climb_mode = ZENITH_CLIMB_MODE_STEP,
			.freq_step_pct = 8,
			.powersave_bias = 150,
			.bias_load_threshold = 35,
			.ignore_nice_load = 1,
			.input_boost_ms = 40,
			.input_boost_decay_ms = 10,
			.input_boost_cap_pct = 60,
			.light_load_threshold = 30,
			.sampling_down_factor = 1,
			.thermal_auto = 1,
			.screen_auto = 1,
			.util_math_v2 = 1,
			.kcpustat_hispeed_enable = 0,
			.down_rate_adaptive = 0,
			.wakeup_boost = 1,
			.down_threshold_adaptive = 0,
			.rate_limit_cluster_scale = 1,
		},
		{
			.profile = ZENITH_PROFILE_LEGACY,
			.up_rate_limit_us = 2000,
			.down_rate_limit_us = 4000,
			.up_threshold = 80,
			.down_threshold = 80,
			.hispeed_freq_pct = 0,
			.hispeed_load = 90,
			.climb_mode = ZENITH_CLIMB_MODE_SNAP,
			.freq_step_pct = 5,
			.powersave_bias = 0,
			.bias_load_threshold = 50,
			.ignore_nice_load = 1,
			.input_boost_ms = 0,
			.input_boost_decay_ms = 0,
			.input_boost_cap_pct = 0,
			.light_load_threshold = 20,
			.sampling_down_factor = 1,
			.thermal_auto = 0,
			.screen_auto = 0,
			.util_math_v2 = 0,
			.kcpustat_hispeed_enable = 0,
			.down_rate_adaptive = 0,
			.wakeup_boost = 1,
			.down_threshold_adaptive = 0,
			.rate_limit_cluster_scale = 1,
		},
	};
	const struct zenith_profile_defaults *p = NULL;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(profiles); i++) {
		if (profiles[i].profile == prof) {
			p = &profiles[i];
			break;
		}
	}
	if (!p)
		return;

	t->up_rate_limit_us	= p->up_rate_limit_us;
	t->down_rate_limit_us	= p->down_rate_limit_us;
	t->up_threshold		= p->up_threshold;
	t->down_threshold	= p->down_threshold;
	t->hispeed_freq_pct	= p->hispeed_freq_pct;
	t->hispeed_load		= p->hispeed_load;
	t->climb_mode		= p->climb_mode;
	t->freq_step_pct	= p->freq_step_pct;
	t->powersave_bias	= p->powersave_bias;
	t->bias_load_threshold	= p->bias_load_threshold;
	t->ignore_nice_load	= p->ignore_nice_load;
	t->input_boost_ms	= p->input_boost_ms;
	t->input_boost_decay_ms	= p->input_boost_decay_ms;
	t->input_boost_cap_pct	= p->input_boost_cap_pct;
	t->light_load_threshold	= p->light_load_threshold;
	t->sampling_down_factor	= p->sampling_down_factor;
	t->thermal_auto		= p->thermal_auto;
	t->screen_auto		= p->screen_auto;
	t->util_math_v2		= p->util_math_v2;
	t->kcpustat_hispeed_enable = p->kcpustat_hispeed_enable;
	t->down_rate_adaptive	= p->down_rate_adaptive;
	t->wakeup_boost		= p->wakeup_boost;
	t->down_threshold_adaptive = p->down_threshold_adaptive;
	t->rate_limit_cluster_scale = p->rate_limit_cluster_scale;

	/* Mirror input_boost_ms to the governor-wide cache used by the
	 * input handler fast path.
	 */
	WRITE_ONCE(zenith_input_boost_active_ms, t->input_boost_ms);
}

/* early_param("zenith.profile", ...) — accepts one of the canonical
 * preset names (performance / balanced / battery / legacy / custom).
 * Anything else is ignored and leaves zenith_cmdline_profile at CUSTOM.
 *
 * Returning 1 tells the early-param core "consumed, do not append to
 * the residual cmdline"; returning 0 would leak the value into init
 * env via the unknown-param fallback.
 */
static int __init zenith_setup_profile(char *s)
{
	if (!s)
		return 1;
	if (!strcmp(s, "performance"))
		zenith_cmdline_profile = ZENITH_PROFILE_PERFORMANCE;
	else if (!strcmp(s, "balanced"))
		zenith_cmdline_profile = ZENITH_PROFILE_BALANCED;
	else if (!strcmp(s, "battery"))
		zenith_cmdline_profile = ZENITH_PROFILE_BATTERY;
	else if (!strcmp(s, "legacy"))
		zenith_cmdline_profile = ZENITH_PROFILE_LEGACY;
	else if (!strcmp(s, "custom"))
		zenith_cmdline_profile = ZENITH_PROFILE_CUSTOM;
	else {
		/* Unknown preset.  Reset explicitly to CUSTOM so a
		 * subsequent zenith.profile= on the cmdline (or a
		 * future caller chaining into this routine) cannot
		 * leak a previously-parsed value.  Belt and braces:
		 * the variable is already file-static and starts at
		 * CUSTOM, but pinning the reset on every unknown-input
		 * branch removes the entire class of "is this still
		 * the default?" questions.
		 */
		zenith_cmdline_profile = ZENITH_PROFILE_CUSTOM;
		pr_warn("zenith.profile=%s: unknown preset, ignored\n", s);
	}
	return 1;
}
early_param("zenith.profile", zenith_setup_profile);

/* Parse a single "performance|balanced|battery|legacy|custom" name
 * into a profile id.  Returns ZENITH_PROFILE_CUSTOM on unknown
 * input so callers can use the result as a tri-state ("apply" /
 * "explicit-custom" / "ignore") without another strcmp pass.
 */
static unsigned int __init zenith_parse_profile_name(const char *s)
{
	if (!strcmp(s, "performance"))
		return ZENITH_PROFILE_PERFORMANCE;
	if (!strcmp(s, "balanced"))
		return ZENITH_PROFILE_BALANCED;
	if (!strcmp(s, "battery"))
		return ZENITH_PROFILE_BATTERY;
	if (!strcmp(s, "legacy"))
		return ZENITH_PROFILE_LEGACY;
	if (!strcmp(s, "custom"))
		return ZENITH_PROFILE_CUSTOM;
	return ZENITH_PROFILE_CUSTOM;
}

/* early_param("zenith.policy_profile", ...) -- asymmetric preset
 * list.  Accepts a comma-separated list of "CPU:name" pairs where
 * CPU is the anchor-cpu of a cpufreq policy (cpumask_first) and
 * name is any of the canonical profile names accepted by
 * zenith.profile.  Example on a 4+3+1 big.LITTLE:
 *
 *   zenith.policy_profile=0:battery,4:balanced,7:performance
 *
 * Silently skips pairs with an out-of-range CPU index or an
 * unknown name.  Tokenising is done in-place against a stable
 * early_param scratch buffer (the cmdline is already copied by
 * the boot allocator).  Any slot not mentioned stays at CUSTOM
 * and therefore falls through to the global zenith.profile= (or
 * to the unprofiled default when that too is CUSTOM).
 */
static int __init zenith_setup_policy_profile(char *s)
{
	char *p, *next;

	if (!s)
		return 1;

	for (p = s; p && *p; p = next) {
		unsigned int cpu, prof;
		char *colon;

		next = strchr(p, ',');
		if (next)
			*next++ = '\0';

		colon = strchr(p, ':');
		if (!colon)
			continue;
		*colon++ = '\0';

		if (kstrtouint(p, 10, &cpu) || cpu >= NR_CPUS)
			continue;

		prof = zenith_parse_profile_name(colon);
		/* CUSTOM from the parser means either "user wrote
		 * custom" (explicit) or "unknown name" (skipped).  In
		 * both cases leaving the slot at its CUSTOM default
		 * is correct: CUSTOM means "no override".  Distinguish
		 * the explicit case only if we later want to force a
		 * CUSTOM override over the global profile; today we
		 * don't, so both collapse to the same behaviour.
		 */
		zenith_cmdline_policy_profile[cpu] = (u8)prof;
	}
	return 1;
}
early_param("zenith.policy_profile", zenith_setup_policy_profile);

/************************ Auto-tune observer *****************************/

/* Classify the workload seen since the last pass and pick a profile.
 * Runs from a delayed_work context on an arbitrary CPU. The per-policy
 * sample counters are atomics, so no lock is needed to sample+reset
 * them here even though zenith_update_single() increments them without
 * holding update_lock.
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
	unsigned int state;
	unsigned int reason = ZENITH_AT_REASON_CLASSIFIER;
	unsigned int flags = 0;
	bool audio = false;
	bool camera = false;
	bool render = false;
	bool memstall = false;
	bool psi_cpu = false;
	bool psi_io = false;
	bool frame_active = false;
	bool screen_off = false;
	bool thermal_slope = false;
	bool thermal;
	unsigned int psi_mem_pct = 0;
	unsigned int psi_cpu_pct = 0;
	unsigned int psi_io_pct = 0;
	unsigned int thermal_pressure = 0;
	unsigned int thermal_delta = 0;
	unsigned int frame_budget_us = 0;
	unsigned int at_from_state = z_policy->at_last_state;
	bool at_emergency = false;

	if (!t->auto_tune)
		return;	/* tunable turned off; stop the chain */

	total = (unsigned int)atomic_xchg(&z_policy->at_samples_total, 0);
	saturated = (unsigned int)atomic_xchg(&z_policy->at_samples_saturated, 0);

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
	state = zenith_profile_to_at_state(target);

	if (trace_zenith_auto_tune_enabled())
		trace_zenith_auto_tune(z_policy->policy->cpu, sat_pct,
				       events_rate_x2, t->active_profile,
				       target);

	if (t->auto_tune_v2 && t->auto_tune_v2_signals) {
		unsigned int anchor = cpumask_first(z_policy->policy->cpus);

		screen_off = !READ_ONCE(t->screen_state);
		if (screen_off)
			flags |= ZENITH_AT_FLAG_SCREEN_OFF;

		if (t->psi_mem_thresh) {
			psi_mem_pct = zenith_psi_mem_some_pct();
			memstall = psi_mem_pct >= t->psi_mem_thresh;
		}
		if (t->psi_cpu_thresh) {
			psi_cpu_pct = zenith_psi_cpu_some_pct();
			psi_cpu = psi_cpu_pct >= t->psi_cpu_thresh;
		}
		if (t->psi_io_thresh) {
			psi_io_pct = zenith_psi_io_some_pct();
			psi_io = psi_io_pct >= t->psi_io_thresh;
		}
		if (psi_cpu)
			flags |= ZENITH_AT_FLAG_PSI_CPU;
		if (psi_io)
			flags |= ZENITH_AT_FLAG_PSI_IO;

		if (t->auto_tune_frame_pacing) {
			if (cpu_possible(anchor))
				frame_budget_us = READ_ONCE(
					t->frame_budget_us_per_policy[anchor]);
			if (!frame_budget_us)
				frame_budget_us = READ_ONCE(t->frame_budget_us);
			frame_active = frame_budget_us &&
				       READ_ONCE(t->frame_pace_floor_pct);
			if (frame_active)
				flags |= ZENITH_AT_FLAG_FRAME;
		}

		if (t->auto_tune_sustained_gaming && READ_ONCE(t->game_mode)) {
			flags |= ZENITH_AT_FLAG_GAME;
			if (state == ZENITH_AT_STATE_LATENCY) {
				state = ZENITH_AT_STATE_SUSTAINED_PERF;
				reason = ZENITH_AT_REASON_GAME;
			}
		}
	}

	/* Scenario overlay.  V2 samples the same signals even when the
	 * legacy overlay gate is off, because diagnostics and state choice
	 * both benefit from knowing why a policy was held back or boosted.
	 */
	if (t->auto_tune_scenario || t->auto_tune_v2) {
		unsigned int cam_override = t->camera_active;
		unsigned int prev = target;

		audio = zenith_policy_has_audio(z_policy);
		render = zenith_policy_has_render(z_policy);
		if (cam_override == ZENITH_CAMERA_OVERRIDE_FORCE_ON)
			camera = true;
		else if (cam_override == ZENITH_CAMERA_OVERRIDE_FORCE_OFF)
			camera = false;
		else
			camera = zenith_policy_has_camera(z_policy);

		if (!psi_mem_pct && t->psi_mem_thresh)
			psi_mem_pct = zenith_psi_mem_some_pct();
		if (t->psi_mem_thresh)
			memstall = psi_mem_pct >= t->psi_mem_thresh;

		if (camera || render)
			target = ZENITH_PROFILE_PERFORMANCE;
		else if (memstall)
			target = ZENITH_PROFILE_BATTERY;
		else if (audio)
			target = ZENITH_PROFILE_BALANCED;
		if (camera)
			flags |= ZENITH_AT_FLAG_CAMERA;
		if (render)
			flags |= ZENITH_AT_FLAG_RENDER;
		if (audio)
			flags |= ZENITH_AT_FLAG_AUDIO;
		if (memstall)
			flags |= ZENITH_AT_FLAG_MEMSTALL;
		if (target != prev) {
			if (camera || render)
				reason = ZENITH_AT_REASON_CAMERA_RENDER;
			else if (memstall)
				reason = ZENITH_AT_REASON_MEMSTALL;
			else if (audio)
				reason = ZENITH_AT_REASON_AUDIO;
		}
		if (!(t->auto_tune_v2 && t->auto_tune_sustained_gaming &&
		      state == ZENITH_AT_STATE_SUSTAINED_PERF))
			state = zenith_profile_to_at_state(target);

		if (trace_zenith_auto_tune_scenario_enabled())
			trace_zenith_auto_tune_scenario(
				z_policy->policy->cpu, audio, camera,
				render, memstall, prev, target);
	}

	thermal = READ_ONCE(t->thermal_state);
	if (t->auto_tune_v2 && t->auto_tune_thermal_slope) {
		thermal_pressure = zenith_policy_thermal_pressure_pct(z_policy);
		if (thermal_pressure > z_policy->at_last_thermal_pressure)
			thermal_delta = thermal_pressure -
				z_policy->at_last_thermal_pressure;
		thermal_slope = thermal_pressure >=
				t->auto_tune_thermal_pressure_pct ||
			thermal_delta >= t->auto_tune_thermal_slope_pct;
		if (thermal_slope)
			flags |= ZENITH_AT_FLAG_THERMAL_SLOPE;
	}
	if (thermal) {
		flags |= ZENITH_AT_FLAG_THERMAL;
		if (t->auto_tune_v2) {
			state = ZENITH_AT_STATE_THERMAL_RECOVERY;
			target = zenith_at_profile_for_state(z_policy, state);
			reason = ZENITH_AT_REASON_THERMAL;
		}
	} else if (t->auto_tune_v2 && thermal_slope) {
		state = ZENITH_AT_STATE_THERMAL_RECOVERY;
		target = zenith_at_profile_for_state(z_policy, state);
		reason = ZENITH_AT_REASON_THERMAL_SLOPE;
	} else if (t->auto_tune_v2 && screen_off) {
		state = ZENITH_AT_STATE_EFFICIENCY;
		target = zenith_at_profile_for_state(z_policy, state);
		reason = ZENITH_AT_REASON_SCREEN;
	} else if (t->auto_tune_v2 && (psi_cpu || psi_io)) {
		state = psi_cpu ? ZENITH_AT_STATE_SUSTAINED_PERF :
			ZENITH_AT_STATE_EFFICIENCY;
		target = zenith_at_profile_for_state(z_policy, state);
		reason = ZENITH_AT_REASON_PSI;
	} else if (t->auto_tune_v2 && frame_active &&
		   state == ZENITH_AT_STATE_LATENCY) {
		state = ZENITH_AT_STATE_SUSTAINED_PERF;
		target = zenith_at_profile_for_state(z_policy, state);
		reason = ZENITH_AT_REASON_FRAME;
	} else if (t->auto_tune_v2 &&
		   z_policy->load_var_ewma_x256 >= 768 &&
		   state == ZENITH_AT_STATE_LATENCY) {
		state = ZENITH_AT_STATE_SUSTAINED_PERF;
		target = zenith_at_profile_for_state(z_policy, state);
		reason = ZENITH_AT_REASON_VARIANCE;
	}

	zenith_at_update_prefer_silver_rate(z_policy, t, &flags);

	z_policy->at_last_total = total;
	z_policy->at_last_saturated = saturated;
	z_policy->at_last_sat_pct = sat_pct;
	z_policy->at_last_events_rate_x2 = events_rate_x2;
	z_policy->at_last_target = target;
	z_policy->at_last_flags = flags;
	z_policy->at_last_var_x256 = z_policy->load_var_ewma_x256;
	z_policy->at_last_psi_cpu = psi_cpu_pct;
	z_policy->at_last_psi_io = psi_io_pct;
	z_policy->at_last_psi_mem = psi_mem_pct;
	z_policy->at_last_thermal_slope = thermal_delta;
	z_policy->at_last_frame_budget_us = frame_budget_us;
	z_policy->at_last_thermal_pressure = thermal_pressure;

	if (t->auto_tune_v2) {
		unsigned int need = t->auto_tune_hysteresis_windows;
		bool emergency = state == ZENITH_AT_STATE_THERMAL_RECOVERY ||
				 state == ZENITH_AT_STATE_SUSTAINED_PERF;

		if (need > ZENITH_AT_HYSTERESIS_WINDOWS_MAX)
			need = ZENITH_AT_HYSTERESIS_WINDOWS_MAX;
		if (z_policy->at_pending_state != state) {
			z_policy->at_pending_state = state;
			z_policy->at_pending_windows = 1;
		} else if (z_policy->at_pending_windows < need) {
			z_policy->at_pending_windows++;
		}
		if (!emergency && z_policy->at_cooldown_left) {
			z_policy->at_cooldown_left--;
			z_policy->at_last_reason = ZENITH_AT_REASON_COOLDOWN;
			goto rearm;
		}
		if (z_policy->at_pending_windows < need) {
			z_policy->at_last_reason = ZENITH_AT_REASON_HYSTERESIS;
			goto rearm;
		}
		if (state != z_policy->at_last_state) {
			unsigned int old_state = z_policy->at_last_state;

			z_policy->at_last_state = state;
			z_policy->at_cooldown_left =
				min_t(unsigned int,
				      t->auto_tune_cooldown_windows,
				      ZENITH_AT_COOLDOWN_WINDOWS_MAX);
			at_emergency = emergency;
			if (!t->auto_tune_cluster_aware &&
			    target != t->active_profile) {
				zenith_apply_profile(t, target);
				t->active_profile = target;
				zenith_refresh_rate_delays_one(z_policy);
			}
			if (trace_zenith_auto_tune_v2_enabled())
				trace_zenith_auto_tune_v2(
					z_policy->policy->cpu,
					z_policy->cluster_class, old_state,
					state, reason, flags, target);
		}
		z_policy->at_last_reason = reason;
		zenith_at_apply_actions(z_policy, state);
		goto rearm;
	}

	if (target != t->active_profile) {
		zenith_apply_profile(t, target);
		t->active_profile = target;
		z_policy->at_local_actions = false;
		/* Profile mutated tunables->{up,down}_rate_limit_us;
		 * refresh the per-policy rate-delay cache for *this*
		 * policy so the new limits take effect on the next tick.
		 * Other policies sharing the same tunables pick up the
		 * change on their own next auto_tune tick.  We cannot
		 * iterate attr_set->policy_list from this context
		 * (no lock held) without racing gov_attr_set_get/put.
		 */
		zenith_refresh_rate_delays_one(z_policy);
	}
	z_policy->at_last_state = zenith_profile_to_at_state(t->active_profile);
	z_policy->at_last_reason = reason;

	/* Re-arm for the next classification window. */
rearm:
	zenith_at_log_push(z_policy, at_from_state, z_policy->at_last_target,
			   at_emergency);
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
			atomic_set(&z_policy->at_samples_total, 0);
			atomic_set(&z_policy->at_samples_saturated, 0);
			z_policy->at_pending_windows = 0;
			z_policy->at_cooldown_left = 0;
			schedule_delayed_work(&z_policy->at_work,
				msecs_to_jiffies(ZENITH_AUTO_TUNE_PERIOD_MS));
		} else {
			cancel_delayed_work_sync(&z_policy->at_work);
		}
	}

	return count;
}
static struct governor_attr auto_tune = __ATTR_RW(auto_tune);

static ssize_t auto_tune_v2_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_tune_v2);
}

static ssize_t auto_tune_v2_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->auto_tune_v2 = val;
	return count;
}
static struct governor_attr auto_tune_v2 = __ATTR_RW(auto_tune_v2);

static ssize_t auto_tune_hysteresis_windows_show(struct gov_attr_set *attr_set,
						 char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_tune_hysteresis_windows);
}

static ssize_t auto_tune_hysteresis_windows_store(struct gov_attr_set *attr_set,
						  const char *buf,
						  size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_AT_HYSTERESIS_WINDOWS_MAX)
		return -EINVAL;
	t->auto_tune_hysteresis_windows = val;
	return count;
}
static struct governor_attr auto_tune_hysteresis_windows =
	__ATTR_RW(auto_tune_hysteresis_windows);

static ssize_t auto_tune_cooldown_windows_show(struct gov_attr_set *attr_set,
					       char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_tune_cooldown_windows);
}

static ssize_t auto_tune_cooldown_windows_store(struct gov_attr_set *attr_set,
						const char *buf,
						size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_AT_COOLDOWN_WINDOWS_MAX)
		return -EINVAL;
	t->auto_tune_cooldown_windows = val;
	return count;
}
static struct governor_attr auto_tune_cooldown_windows =
	__ATTR_RW(auto_tune_cooldown_windows);

static ssize_t auto_tune_cluster_aware_show(struct gov_attr_set *attr_set,
					    char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_tune_cluster_aware);
}

static ssize_t auto_tune_cluster_aware_store(struct gov_attr_set *attr_set,
					     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_pol;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->auto_tune_cluster_aware = val;
	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook)
		zenith_reset_local_actions(z_pol);
	zenith_refresh_rate_delays(attr_set);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr auto_tune_cluster_aware =
	__ATTR_RW(auto_tune_cluster_aware);

static ssize_t auto_tune_v2_signals_show(struct gov_attr_set *attr_set,
					 char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_tune_v2_signals);
}

static ssize_t auto_tune_v2_signals_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->auto_tune_v2_signals = val;
	return count;
}
static struct governor_attr auto_tune_v2_signals =
	__ATTR_RW(auto_tune_v2_signals);

static ssize_t auto_tune_thermal_slope_show(struct gov_attr_set *attr_set,
					    char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_tune_thermal_slope);
}

static ssize_t auto_tune_thermal_slope_store(struct gov_attr_set *attr_set,
					     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->auto_tune_thermal_slope = val;
	return count;
}
static struct governor_attr auto_tune_thermal_slope =
	__ATTR_RW(auto_tune_thermal_slope);

static ssize_t auto_tune_thermal_pressure_pct_show(struct gov_attr_set *attr_set,
						   char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->
		       auto_tune_thermal_pressure_pct);
}

static ssize_t auto_tune_thermal_pressure_pct_store(struct gov_attr_set *attr_set,
						    const char *buf,
						    size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_AT_THERMAL_PCT_MAX)
		return -EINVAL;
	t->auto_tune_thermal_pressure_pct = val;
	return count;
}
static struct governor_attr auto_tune_thermal_pressure_pct =
	__ATTR_RW(auto_tune_thermal_pressure_pct);

static ssize_t auto_tune_thermal_slope_pct_show(struct gov_attr_set *attr_set,
						char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->
		       auto_tune_thermal_slope_pct);
}

static ssize_t auto_tune_thermal_slope_pct_store(struct gov_attr_set *attr_set,
						 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_AT_THERMAL_PCT_MAX)
		return -EINVAL;
	t->auto_tune_thermal_slope_pct = val;
	return count;
}
static struct governor_attr auto_tune_thermal_slope_pct =
	__ATTR_RW(auto_tune_thermal_slope_pct);

static ssize_t auto_tune_frame_pacing_show(struct gov_attr_set *attr_set,
					   char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_tune_frame_pacing);
}

static ssize_t auto_tune_frame_pacing_store(struct gov_attr_set *attr_set,
					    const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->auto_tune_frame_pacing = val;
	return count;
}
static struct governor_attr auto_tune_frame_pacing =
	__ATTR_RW(auto_tune_frame_pacing);

static ssize_t auto_tune_sustained_gaming_show(struct gov_attr_set *attr_set,
					       char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->
		       auto_tune_sustained_gaming);
}

static ssize_t auto_tune_sustained_gaming_store(struct gov_attr_set *attr_set,
						const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->auto_tune_sustained_gaming = val;
	return count;
}
static struct governor_attr auto_tune_sustained_gaming =
	__ATTR_RW(auto_tune_sustained_gaming);

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

/* auto_tune_scenario sysfs knob.  Strict 0/1 boolean; non-zero
 * values normalised to 1 on store.  Effective only when auto_tune=1.
 * See ZENITH_DEFAULT_AUTO_TUNE_SCENARIO comment block for the
 * detection logic and scenario precedence.
 */
static ssize_t auto_tune_scenario_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_tune_scenario);
}

static ssize_t auto_tune_scenario_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->auto_tune_scenario = val;
	return count;
}
static struct governor_attr auto_tune_scenario =
	__ATTR_RW(auto_tune_scenario);

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
	struct zenith_policy *z_policy;
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

	/* Idempotent early-return: writing the currently-active
	 * profile is a no-op.  Skipping zenith_apply_profile() avoids
	 * re-stomping the tunables (which would lose any per-knob
	 * userspace tweaks the operator layered on top of the
	 * preset), and skipping zenith_refresh_rate_delays() avoids a
	 * tunables-list walk under attr_set->update_lock for nothing.
	 * The active_profile field is already correct by definition.
	 */
	if (t->active_profile == prof)
		return count;

	zenith_apply_profile(t, prof);
	t->active_profile = prof;
	t->auto_tune_override_mask = 0;
	list_for_each_entry(z_policy, &attr_set->policy_list, tunables_hook) {
		zenith_reset_local_actions(z_policy);
		z_policy->at_last_state = ZENITH_AT_STATE_BALANCED;
		z_policy->at_pending_state = ZENITH_AT_STATE_BALANCED;
		z_policy->at_pending_windows = 0;
		z_policy->at_cooldown_left = 0;
	}
	/* Profile may have mutated tunables->{up,down}_rate_limit_us;
	 * refresh the per-policy rate-delay cache on every policy
	 * sharing this tunables set so the new limits take effect on
	 * the next tick rather than persisting the previous profile's
	 * cached delays.  attr_set->update_lock is held by the
	 * governor_store wrapper, so iterating policy_list is safe.
	 */
	zenith_refresh_rate_delays(attr_set);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr profile = __ATTR_RW(profile);

static ssize_t auto_tune_status_show(struct gov_attr_set *attr_set, char *buf)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_pol;
	ssize_t len = 0;

	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "auto_tune=%u\n", t->auto_tune);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "auto_tune_v2=%u\n", t->auto_tune_v2);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "v2_knobs=cluster:%u signals:%u thermal_slope:%u frame:%u gaming:%u\n",
			 t->auto_tune_cluster_aware,
			 t->auto_tune_v2_signals,
			 t->auto_tune_thermal_slope,
			 t->auto_tune_frame_pacing,
			 t->auto_tune_sustained_gaming);
	len += scnprintf(buf + len, PAGE_SIZE - len, "profile=%s\n",
			 zenith_profile_name(t->active_profile));
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "override_mask=0x%lx\n", t->auto_tune_override_mask);
	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "policy%u(%s): state=%s pending=%s pending_windows=%u cooldown=%u reason=%s target=%s samples=%u saturated=%u sat_pct=%u events_x2=%u flags=0x%x var_x256=%u psi=%u/%u/%u thermal=%u+%u frame_us=%u local=%u eff_rate=%u/%u eff_thresh=%u/%u eff_boost=%u/%u eff_frame=%u eff_game=%u\n",
				 z_pol->policy->cpu,
				 zenith_at_cluster_name(z_pol->cluster_class),
				 zenith_at_state_name(z_pol->at_last_state),
				 zenith_at_state_name(z_pol->at_pending_state),
				 z_pol->at_pending_windows,
				 z_pol->at_cooldown_left,
				 zenith_at_reason_name(z_pol->at_last_reason),
				 zenith_profile_name(z_pol->at_last_target),
				 z_pol->at_last_total,
				 z_pol->at_last_saturated,
				 z_pol->at_last_sat_pct,
				 z_pol->at_last_events_rate_x2,
				 z_pol->at_last_flags,
				 z_pol->at_last_var_x256,
				 z_pol->at_last_psi_cpu,
				 z_pol->at_last_psi_io,
				 z_pol->at_last_psi_mem,
				 z_pol->at_last_thermal_pressure,
				 z_pol->at_last_thermal_slope,
				 z_pol->at_last_frame_budget_us,
				 z_pol->at_local_actions,
				 z_pol->at_effective_up_rate_limit_us,
				 z_pol->at_effective_down_rate_limit_us,
				 z_pol->at_effective_up_threshold,
				 z_pol->at_effective_down_threshold,
				 z_pol->at_effective_input_boost_ms,
				 z_pol->at_effective_input_boost_cap_pct,
				 z_pol->at_effective_frame_pace_floor_pct,
				 z_pol->at_effective_game_mode);
		if (len >= PAGE_SIZE)
			break;
	}
	return len;
}
static struct governor_attr auto_tune_status = __ATTR_RO(auto_tune_status);

static ssize_t auto_tune_reset_overrides_store(struct gov_attr_set *attr_set,
					       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_pol;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	if (val)
		t->auto_tune_override_mask = 0;
	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook)
		zenith_reset_local_actions(z_pol);
	return count;
}
static struct governor_attr auto_tune_reset_overrides =
	__ATTR_WO(auto_tune_reset_overrides);

/* profile_values: readonly dump of the hardcoded preset tables.
 *
 * Invokes zenith_apply_profile() against a stack-scratch tunables
 * struct for each preset (performance, balanced, battery, legacy)
 * and prints the resulting knob values in "name=value" form,
 * one line per profile.  The CUSTOM profile is intentionally
 * skipped: it is a marker ("no preset has been applied") and has
 * no canonical values.
 *
 * zenith_apply_profile() has one side effect -- it stamps
 * zenith_input_boost_active_ms with the preset's input_boost_ms.
 * Save / restore that mirror around the loop so reading this node
 * never perturbs the running boost window.  The scratch struct
 * itself is stack-local so there is no tunables-race window.
 */
static ssize_t profile_values_show(struct gov_attr_set *attr_set, char *buf)
{
	struct zenith_tunables scratch;
	u32 saved_active_ms = READ_ONCE(zenith_input_boost_active_ms);
	ssize_t len = 0;
	int i;
	static const struct {
		unsigned int id;
		const char *name;
	} profs[] = {
		{ ZENITH_PROFILE_PERFORMANCE, "performance" },
		{ ZENITH_PROFILE_BALANCED,    "balanced" },
		{ ZENITH_PROFILE_BATTERY,     "battery" },
		{ ZENITH_PROFILE_LEGACY,      "legacy" },
	};

	for (i = 0; i < ARRAY_SIZE(profs); i++) {
		memset(&scratch, 0, sizeof(scratch));
		zenith_apply_profile(&scratch, profs[i].id);
		len += scnprintf(buf + len, PAGE_SIZE - len,
			"%s: up_rate_limit_us=%u down_rate_limit_us=%u "
			"up_threshold=%u down_threshold=%u "
			"hispeed_freq_pct=%u hispeed_load=%u "
			"climb_mode=%u freq_step_pct=%u "
			"powersave_bias=%u bias_load_threshold=%u "
			"ignore_nice_load=%u input_boost_ms=%u "
			"input_boost_decay_ms=%u input_boost_cap_pct=%u "
			"light_load_threshold=%u sampling_down_factor=%u "
			"thermal_auto=%u screen_auto=%u util_math_v2=%u "
			"kcpustat_hispeed_enable=%u\n",
			profs[i].name,
			scratch.up_rate_limit_us, scratch.down_rate_limit_us,
			scratch.up_threshold, scratch.down_threshold,
			scratch.hispeed_freq_pct, scratch.hispeed_load,
			scratch.climb_mode, scratch.freq_step_pct,
			scratch.powersave_bias, scratch.bias_load_threshold,
			scratch.ignore_nice_load, scratch.input_boost_ms,
			scratch.input_boost_decay_ms,
			scratch.input_boost_cap_pct,
			scratch.light_load_threshold,
			scratch.sampling_down_factor,
			scratch.thermal_auto, scratch.screen_auto,
			scratch.util_math_v2,
			scratch.kcpustat_hispeed_enable);
		len += scnprintf(buf + len, PAGE_SIZE - len, "%s-extra: ",
				 profs[i].name);
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "down_rate_adaptive=%u ",
				 scratch.down_rate_adaptive);
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "wakeup_boost=%u ", scratch.wakeup_boost);
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "down_threshold_adaptive=%u ",
				 scratch.down_threshold_adaptive);
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "rate_limit_cluster_scale=%u\n",
				 scratch.rate_limit_cluster_scale);
	}

	/* Restore the boost-active mirror that zenith_apply_profile()
	 * stamps on every call.  Use WRITE_ONCE to match the writer
	 * semantics elsewhere in the file.
	 */
	WRITE_ONCE(zenith_input_boost_active_ms, saved_active_ms);
	return len;
}
static struct governor_attr profile_values = __ATTR_RO(profile_values);

/* zenith_stats: readonly per-policy decision-stat dump.  One line
 * per bucket, "name=count", emitted in enum order so userspace
 * scrapers can read field-by-name without depending on a fixed
 * column count.  Counters reset on zenith_start() so values reflect
 * the current attach cycle.  See enum zenith_stat_idx for what each
 * bucket includes.
 *
 * Each policy carries its own gov_attr_set, so this attribute is
 * per-policy automatically; we walk attr_set->policy_list for the
 * (currently always one) z_policy that owns the values.  The first
 * entry's stats are reported; if multiple policies were ever to
 * share a tunables instance, later entries are summed in.
 */
static ssize_t zenith_stats_show(struct gov_attr_set *attr_set, char *buf)
{
	static const char * const zenith_stat_names[] = {
		[ZENITH_STAT_DECISIONS]		= "decisions",
		[ZENITH_STAT_CACHE_HITS]	= "cache_hits",
		[ZENITH_STAT_INPUT_BOOST]	= "input_boost",
		[ZENITH_STAT_BRUTAL]		= "brutal",
		[ZENITH_STAT_HISPEED]		= "hispeed",
		[ZENITH_STAT_FRAME_PACE]	= "frame_pace",
		[ZENITH_STAT_AUDIO]		= "audio",
		[ZENITH_STAT_RENDER_CAMERA]	= "render_camera",
		[ZENITH_STAT_UCLAMP]		= "uclamp",
		[ZENITH_STAT_PSI]		= "psi",
		[ZENITH_STAT_BOOT_BOOST]	= "boot_boost",
		[ZENITH_STAT_LIGHT_CAP]		= "light_cap",
		[ZENITH_STAT_EM_CAP]		= "em_cap",
		[ZENITH_STAT_EAS]		= "eas",
		[ZENITH_STAT_OTHER]		= "other",
	};
	unsigned long sum[ZENITH_STAT_NR] = { 0 };
	struct zenith_policy *z_pol;
	ssize_t len = 0;
	unsigned int i;

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		for (i = 0; i < ZENITH_STAT_NR; i++)
			sum[i] += z_pol->stats[i];
	}

	for (i = 0; i < ZENITH_STAT_NR; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%s=%lu\n",
				 zenith_stat_names[i], sum[i]);
		if (len >= PAGE_SIZE)
			break;
	}
	return len;
}
static struct governor_attr zenith_stats = __ATTR_RO(zenith_stats);

/* Reset all per-policy observability counters: zenith_stats[] and the
 * auto-tune classifier ring (at_log).  Write any non-zero value to
 * trigger; writing 0 is a no-op so a misfired "echo > stats_reset"
 * can't accidentally erase the data the operator was about to read.
 */
static ssize_t zenith_stats_reset_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct zenith_policy *z_pol;
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (!val)
		return count;
	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook)
		zenith_policy_observability_reset(z_pol);
	return count;
}
static struct governor_attr zenith_stats_reset =
	__ATTR_WO(zenith_stats_reset);

/* Read-only dump of the per-policy auto-tune classifier ring buffer.
 *
 * One line per entry, oldest first, capped at ZENITH_AT_LOG_NR per
 * policy.  Format chosen to fit comfortably under PAGE_SIZE for the
 * common HMP topology (2 policies × 16 entries) while remaining
 * grep-friendly: every key is name=value with no quoted strings or
 * commas.
 */
static ssize_t at_log_show(struct gov_attr_set *attr_set, char *buf)
{
	struct zenith_policy *z_pol;
	ssize_t len = 0;

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		unsigned int count = z_pol->at_log_count;
		unsigned int head = z_pol->at_log_head;
		unsigned int start, i;

		if (count > ZENITH_AT_LOG_NR)
			count = ZENITH_AT_LOG_NR;
		start = (count == ZENITH_AT_LOG_NR) ? head : 0;

		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "policy%u(%s): %u entries\n",
				 z_pol->policy->cpu,
				 zenith_at_cluster_name(z_pol->cluster_class),
				 count);
		if (len >= PAGE_SIZE)
			break;

		for (i = 0; i < count; i++) {
			struct zenith_at_log_entry *e =
				&z_pol->at_log[(start + i) % ZENITH_AT_LOG_NR];

			len += scnprintf(buf + len, PAGE_SIZE - len,
				"  ts_ns=%llu reason=%s from=%s to=%s target=%s sat=%u evx2=%u thp=%u slope=%u var=%u flags=0x%x emerg=%u\n",
				(unsigned long long)e->ts_ns,
				zenith_at_reason_name(e->reason),
				zenith_at_state_name(e->v2_from_state),
				zenith_at_state_name(e->v2_to_state),
				zenith_profile_name(e->v1_target),
				e->sat_pct,
				e->events_rate_x2,
				e->thermal_pressure,
				e->thermal_slope,
				e->var_x256,
				e->flags,
				e->emergency);
			if (len >= PAGE_SIZE)
				break;
		}
		if (len >= PAGE_SIZE)
			break;
	}
	return len;
}
static struct governor_attr at_log = __ATTR_RO(at_log);

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

/* thermal_pressure_continuous sysfs knob.  Strict 0/1 boolean.  When
 * 1, dynamic_up_thresh ramps linearly from the policy's normal
 * up_threshold (at 0 %% thermal pressure) to 90 (at 100 %% thermal
 * pressure) instead of cliff-jumping to 90 the instant
 * zenith_thermal_active() flips true.  See the field comment on
 * struct zenith_tunables for the rationale.
 */
static ssize_t thermal_pressure_continuous_show(struct gov_attr_set *attr_set,
						char *buf)
{
	return sprintf(buf, "%u\n",
		to_zenith_tunables(attr_set)->thermal_pressure_continuous);
}

static ssize_t thermal_pressure_continuous_store(struct gov_attr_set *attr_set,
						 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->thermal_pressure_continuous = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr thermal_pressure_continuous =
	__ATTR_RW(thermal_pressure_continuous);

/* prefer_silver_aware: strict 0/1.  See struct zenith_tunables for
 * semantics.  When CONFIG_SCHED_PREFER_SILVER=n the field is still
 * stored and round-tripped via sysfs so userspace tools that probe
 * the governor's tunable list don't choke on a missing node, but
 * the run-time bump path is dead because the worker stub never
 * updates ps_hit_rate_pct.
 */
static ssize_t prefer_silver_aware_show(struct gov_attr_set *attr_set,
					char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->prefer_silver_aware);
}

static ssize_t prefer_silver_aware_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->prefer_silver_aware = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr prefer_silver_aware =
	__ATTR_RW(prefer_silver_aware);

/* prefer_silver_hot_threshold_pct: 0..100.  When the per-window
 * prefer_silver hit-rate is at or above this percentage,
 * prefer_silver_aware fires the bump on big / prime clusters.
 */
static ssize_t prefer_silver_hot_threshold_pct_show(
		struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		to_zenith_tunables(attr_set)->prefer_silver_hot_threshold_pct);
}

static ssize_t prefer_silver_hot_threshold_pct_store(
		struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	t->prefer_silver_hot_threshold_pct = val;
	return count;
}
static struct governor_attr prefer_silver_hot_threshold_pct =
	__ATTR_RW(prefer_silver_hot_threshold_pct);

/* prefer_silver_hot_bump_pct: 0..ZENITH_PREFER_SILVER_HOT_BUMP_MAX_PCT.
 * Additive points added to dynamic_up_thresh on big / prime clusters
 * when the prefer_silver hit-rate is hot.  Clamped to the max in the
 * fast path; this store enforces the same range so userspace gets
 * an early -EINVAL on out-of-range values.
 */
static ssize_t prefer_silver_hot_bump_pct_show(
		struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		to_zenith_tunables(attr_set)->prefer_silver_hot_bump_pct);
}

static ssize_t prefer_silver_hot_bump_pct_store(
		struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PREFER_SILVER_HOT_BUMP_MAX_PCT)
		return -EINVAL;
	t->prefer_silver_hot_bump_pct = val;
	return count;
}
static struct governor_attr prefer_silver_hot_bump_pct =
	__ATTR_RW(prefer_silver_hot_bump_pct);

/* thermal_util_derate sysfs knob.  Strict 0/1 boolean.  See the
 * ZENITH_DEFAULT_THERMAL_UTIL_DERATE comment block for semantics.
 */
static ssize_t thermal_util_derate_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->thermal_util_derate);
}

static ssize_t thermal_util_derate_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->thermal_util_derate = val;
	return count;
}
static struct governor_attr thermal_util_derate =
	__ATTR_RW(thermal_util_derate);

static ssize_t thermal_derate_rate_pct_show(struct gov_attr_set *attr_set,
					    char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->thermal_derate_rate_pct);
}

static ssize_t thermal_derate_rate_pct_store(struct gov_attr_set *attr_set,
					     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	t->thermal_derate_rate_pct = val;
	return count;
}
static struct governor_attr thermal_derate_rate_pct =
	__ATTR_RW(thermal_derate_rate_pct);

static ssize_t freq_stability_margin_pct_show(struct gov_attr_set *attr_set,
					      char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
				freq_stability_margin_pct);
}

static ssize_t freq_stability_margin_pct_store(struct gov_attr_set *attr_set,
					       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_FREQ_STABILITY_MARGIN_PCT_MAX)
		return -EINVAL;
	t->freq_stability_margin_pct = val;
	return count;
}
static struct governor_attr freq_stability_margin_pct =
	__ATTR_RW(freq_stability_margin_pct);

static ssize_t down_rate_adaptive_show(struct gov_attr_set *attr_set,
				       char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->down_rate_adaptive);
}

static ssize_t down_rate_adaptive_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->down_rate_adaptive = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_DOWN_ADAPTIVE);
	return count;
}
static struct governor_attr down_rate_adaptive =
	__ATTR_RW(down_rate_adaptive);

static ssize_t wakeup_boost_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->wakeup_boost);
}

static ssize_t wakeup_boost_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->wakeup_boost = val;
	return count;
}
static struct governor_attr wakeup_boost = __ATTR_RW(wakeup_boost);

static ssize_t rate_limit_cluster_scale_show(struct gov_attr_set *attr_set,
					     char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->rate_limit_cluster_scale);
}

static ssize_t rate_limit_cluster_scale_store(struct gov_attr_set *attr_set,
					     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->rate_limit_cluster_scale = val;
	zenith_refresh_rate_delays(attr_set);
	return count;
}
static struct governor_attr rate_limit_cluster_scale =
	__ATTR_RW(rate_limit_cluster_scale);

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
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_INPUT_BOOST_MS);
	WRITE_ONCE(zenith_input_boost_active_ms, val);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr input_boost_ms = __ATTR_RW(input_boost_ms);

static ssize_t input_boost_decay_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->input_boost_decay_ms);
}

static ssize_t input_boost_decay_ms_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1000)
		return -EINVAL;
	t->input_boost_decay_ms = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr input_boost_decay_ms =
	__ATTR_RW(input_boost_decay_ms);

/* input_boost_decay_curve sysfs knob.  0 = linear (legacy),
 * 1 = cubic ease-in.  See ZENITH_DEFAULT_INPUT_BOOST_DECAY_CURVE.
 */
static ssize_t input_boost_decay_curve_show(struct gov_attr_set *attr_set,
					    char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->input_boost_decay_curve);
}

static ssize_t input_boost_decay_curve_store(struct gov_attr_set *attr_set,
					     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->input_boost_decay_curve = val;
	return count;
}
static struct governor_attr input_boost_decay_curve =
	__ATTR_RW(input_boost_decay_curve);

static ssize_t input_boost_big_only_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->input_boost_big_only);
}

static ssize_t input_boost_big_only_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->input_boost_big_only = val;
	return count;
}
static struct governor_attr input_boost_big_only =
	__ATTR_RW(input_boost_big_only);

static ssize_t input_boost_cap_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->input_boost_cap_pct);
}

static ssize_t input_boost_cap_pct_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	t->input_boost_cap_pct = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_INPUT_BOOST_CAP);
	return count;
}
static struct governor_attr input_boost_cap_pct =
	__ATTR_RW(input_boost_cap_pct);

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

static ssize_t eff_bin_hyst_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->eff_bin_hyst_pct);
}

static ssize_t eff_bin_hyst_pct_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_EFF_BIN_HYST_PCT_MAX)
		return -EINVAL;
	t->eff_bin_hyst_pct = val;
	return count;
}
static struct governor_attr eff_bin_hyst_pct = __ATTR_RW(eff_bin_hyst_pct);

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

static ssize_t boost_exit_extend_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->boost_exit_extend);
}

static ssize_t boost_exit_extend_store(struct gov_attr_set *attr_set,
				       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->boost_exit_extend = val;
	return count;
}
static struct governor_attr boost_exit_extend = __ATTR_RW(boost_exit_extend);

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
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_UP_THRESHOLD);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr up_threshold = __ATTR_RW(up_threshold);

static ssize_t up_threshold_adaptive_show(struct gov_attr_set *attr_set,
					  char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->up_threshold_adaptive);
}

static ssize_t up_threshold_adaptive_store(struct gov_attr_set *attr_set,
					   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_UP_THRESHOLD_ADAPTIVE_MAX)
		return -EINVAL;
	t->up_threshold_adaptive = val;
	return count;
}
static struct governor_attr up_threshold_adaptive =
	__ATTR_RW(up_threshold_adaptive);

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
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_DOWN_THRESHOLD);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr down_threshold = __ATTR_RW(down_threshold);

static ssize_t down_threshold_adaptive_show(struct gov_attr_set *attr_set,
					    char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->down_threshold_adaptive);
}

static ssize_t down_threshold_adaptive_store(struct gov_attr_set *attr_set,
					     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_DOWN_THRESHOLD_ADAPTIVE_MAX)
		return -EINVAL;
	t->down_threshold_adaptive = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_DOWN_THRESH_ADAPTIVE);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr down_threshold_adaptive =
	__ATTR_RW(down_threshold_adaptive);

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

static ssize_t hispeed_freq_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->hispeed_freq_pct);
}

static ssize_t hispeed_freq_pct_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	t->hispeed_freq_pct = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr hispeed_freq_pct = __ATTR_RW(hispeed_freq_pct);

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

static ssize_t hispeed_hyst_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->hispeed_hyst_pct);
}

static ssize_t hispeed_hyst_pct_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	t->hispeed_hyst_pct = val;
	return count;
}
static struct governor_attr hispeed_hyst_pct = __ATTR_RW(hispeed_hyst_pct);

/* hispeed_entry_streak sysfs knob.  See ZENITH_DEFAULT_HISPEED_ENTRY_STREAK
 * for semantics.  Capped to ZENITH_HISPEED_ENTRY_STREAK_MAX on store
 * so the per-policy u8 streak counter cannot overflow.
 */
static ssize_t hispeed_entry_streak_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->hispeed_entry_streak);
}

static ssize_t hispeed_entry_streak_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_HISPEED_ENTRY_STREAK_MAX)
		return -EINVAL;
	t->hispeed_entry_streak = val;
	return count;
}
static struct governor_attr hispeed_entry_streak =
	__ATTR_RW(hispeed_entry_streak);

/* brutal_entry_streak sysfs knob.  See ZENITH_DEFAULT_BRUTAL_ENTRY_STREAK
 * for semantics.  Capped to ZENITH_BRUTAL_ENTRY_STREAK_MAX on store
 * so the per-policy u8 streak counter cannot overflow.
 */
static ssize_t brutal_entry_streak_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->brutal_entry_streak);
}

static ssize_t brutal_entry_streak_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_BRUTAL_ENTRY_STREAK_MAX)
		return -EINVAL;
	t->brutal_entry_streak = val;
	return count;
}
static struct governor_attr brutal_entry_streak =
	__ATTR_RW(brutal_entry_streak);

/* brutal_decay_ms sysfs knob.  Range 0..ZENITH_BRUTAL_DECAY_MS_MAX.
 * 0 disables the tail-glide and restores the legacy hard cliff
 * exit; non-zero arms a linear ramp from policy->max down to the
 * EAS-computed freq across the configured window on every
 * brutal-hold cliff exit.  See struct zenith_tunables for details.
 */
static ssize_t brutal_decay_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->brutal_decay_ms);
}

static ssize_t brutal_decay_ms_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_BRUTAL_DECAY_MS_MAX)
		return -EINVAL;
	t->brutal_decay_ms = val;
	return count;
}
static struct governor_attr brutal_decay_ms = __ATTR_RW(brutal_decay_ms);

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

/* freq_step_adaptive sysfs knob.  0/1 only.  See
 * ZENITH_DEFAULT_FREQ_STEP_ADAPTIVE for semantics.
 */
static ssize_t freq_step_adaptive_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->freq_step_adaptive);
}

static ssize_t freq_step_adaptive_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->freq_step_adaptive = val;
	return count;
}
static struct governor_attr freq_step_adaptive =
	__ATTR_RW(freq_step_adaptive);

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
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_UP_RATE);

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook)
		zenith_update_rate_delay_ns(z_pol);
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
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_DOWN_RATE);

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook)
		zenith_update_rate_delay_ns(z_pol);
	return count;
}
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);

/* kcpustat tunables. _store paths invalidate the freq cache so a
 * userspace write takes effect on the very next scheduler tick rather
 * than waiting for the prev_freq cache shortcut to drift. window_us is
 * clamped to a sane range; filter_shift is capped to keep the >>shift
 * idiom well-defined; hispeed_enable is a strict boolean.
 */
static ssize_t kcpustat_window_us_show(struct gov_attr_set *attr_set,
				       char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->kcpustat_window_us);
}

static ssize_t kcpustat_window_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val < ZENITH_KCPUSTAT_WINDOW_MIN_US ||
	    val > ZENITH_KCPUSTAT_WINDOW_MAX_US)
		return -EINVAL;
	t->kcpustat_window_us = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr kcpustat_window_us =
	__ATTR_RW(kcpustat_window_us);

static ssize_t kcpustat_filter_shift_show(struct gov_attr_set *attr_set,
					  char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->kcpustat_filter_shift);
}

static ssize_t kcpustat_filter_shift_store(struct gov_attr_set *attr_set,
					   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > ZENITH_KCPUSTAT_FILTER_SHIFT_MAX)
		return -EINVAL;
	t->kcpustat_filter_shift = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr kcpustat_filter_shift =
	__ATTR_RW(kcpustat_filter_shift);

static ssize_t kcpustat_hispeed_enable_show(struct gov_attr_set *attr_set,
					    char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->kcpustat_hispeed_enable);
}

static ssize_t kcpustat_hispeed_enable_store(struct gov_attr_set *attr_set,
					     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->kcpustat_hispeed_enable = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr kcpustat_hispeed_enable =
	__ATTR_RW(kcpustat_hispeed_enable);

/* Strict-bool tunable selecting v1 (legacy cpu_util_cfs()) vs v2
 * (6.x-style runnable-aware util) input to schedutil_cpu_util in
 * zenith_get_util().  Invalidates the prev_freq cache so toggles
 * take effect on the next tick.
 */
static ssize_t util_math_v2_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->util_math_v2);
}

static ssize_t util_math_v2_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->util_math_v2 = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr util_math_v2 = __ATTR_RW(util_math_v2);

/* predict_util_pct sysfs knob.  See ZENITH_DEFAULT_PREDICT_UTIL_PCT
 * comment block at the top of the file for semantics.  Range
 * 0..ZENITH_PREDICT_UTIL_PCT_MAX; values above the cap are rejected
 * outright rather than silently clamped, so userspace gets a clear
 * EINVAL on out-of-range writes.  The freq cache is invalidated so
 * a toggle takes effect on the very next zenith_update tick.
 */
static ssize_t predict_util_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->predict_util_pct);
}

static ssize_t predict_util_pct_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > ZENITH_PREDICT_UTIL_PCT_MAX)
		return -EINVAL;
	t->predict_util_pct = val;
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr predict_util_pct = __ATTR_RW(predict_util_pct);

/* predict_util_smooth sysfs knob.  0/1 only.  See
 * ZENITH_DEFAULT_PREDICT_UTIL_SMOOTH for semantics.
 */
static ssize_t predict_util_smooth_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->predict_util_smooth);
}

static ssize_t predict_util_smooth_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->predict_util_smooth = val;
	return count;
}
static struct governor_attr predict_util_smooth =
	__ATTR_RW(predict_util_smooth);

/* render_aware sysfs knob.  Strict 0/1 boolean; non-zero values are
 * normalised to 1 on store so userspace can echo any truthy integer.
 * No cache invalidation is required: tunables->render_aware is read
 * fresh on every zenith_get_next_freq() call, so the next scheduler
 * tick already sees the new value.
 */
static ssize_t render_aware_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->render_aware);
}

static ssize_t render_aware_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->render_aware = val;
	zenith_set_static_key(&zenith_render_aware_key, val);
	return count;
}
static struct governor_attr render_aware = __ATTR_RW(render_aware);

/* render_floor_pct sysfs knob.  Range 0..100; 0 leaves the comm
 * walk running but applies no floor.  Out-of-range values rejected
 * with EINVAL so userspace gets a clear error.
 */
static ssize_t render_floor_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->render_floor_pct);
}

static ssize_t render_floor_pct_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	t->render_floor_pct = val;
	return count;
}
static struct governor_attr render_floor_pct = __ATTR_RW(render_floor_pct);

/* audio_aware sysfs knob.  Strict 0/1 boolean; non-zero values are
 * normalised to 1 on store.  No cache invalidation: tunables->audio_aware
 * is read fresh on every zenith_get_next_freq() call.  Toggling from 1
 * to 0 leaves the per-policy audio_active cache stale, but its TTL
 * (ZENITH_AUDIO_CACHE_TTL_NS) is short and the cap/floor are gated on
 * tunables->audio_aware first so the stale state is unreachable.
 */
static ssize_t audio_aware_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->audio_aware);
}

static ssize_t audio_aware_store(struct gov_attr_set *attr_set,
				 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->audio_aware = val;
	zenith_set_static_key(&zenith_audio_aware_key, val);
	return count;
}
static struct governor_attr audio_aware = __ATTR_RW(audio_aware);

/* audio_floor_pct sysfs knob.  Range 0..100; 0 leaves the comm walk
 * running (when audio_aware=1) but applies no floor.  Out-of-range
 * values rejected with EINVAL.  audio_floor_pct does NOT have to be
 * <= audio_cap_pct: if the user inverts them, the cap still wins
 * because it runs after the floor in zenith_get_next_freq().
 */
static ssize_t audio_floor_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->audio_floor_pct);
}

static ssize_t audio_floor_pct_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	t->audio_floor_pct = val;
	return count;
}
static struct governor_attr audio_floor_pct = __ATTR_RW(audio_floor_pct);

/* audio_cap_pct sysfs knob.  Range 0..100; 0 disables the cap (the
 * floor side of the band can still apply alone).  Applied before the
 * uclamp_max final cap so ADPF power-efficiency hints still win.
 */
static ssize_t audio_cap_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->audio_cap_pct);
}

static ssize_t audio_cap_pct_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	t->audio_cap_pct = val;
	return count;
}
static struct governor_attr audio_cap_pct = __ATTR_RW(audio_cap_pct);

/* camera_aware sysfs knob.  Strict 0/1 boolean; non-zero values
 * normalised to 1 on store.
 */
static ssize_t camera_aware_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->camera_aware);
}

static ssize_t camera_aware_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->camera_aware = val;
	zenith_set_static_key(&zenith_camera_aware_key, val);
	return count;
}
static struct governor_attr camera_aware = __ATTR_RW(camera_aware);

/* render_comms / audio_comms / camera_comms sysfs knobs.  CSV of
 * comma-separated comm prefixes.  See the zenith_comm_table comment
 * block above zenith_render_comms[] for the RCU semantics and
 * defaults.  Empty write resets to the in-tree default list.
 */
static ssize_t render_comms_show(struct gov_attr_set *attr_set, char *buf)
{
	return zenith_show_comm_table(&zenith_render_table, buf);
}

static ssize_t render_comms_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	return zenith_store_comm_table(&zenith_render_table,
				       zenith_render_comms,
				       ARRAY_SIZE(zenith_render_comms),
				       buf, count);
}
static struct governor_attr render_comms = __ATTR_RW(render_comms);

static ssize_t audio_comms_show(struct gov_attr_set *attr_set, char *buf)
{
	return zenith_show_comm_table(&zenith_audio_table, buf);
}

static ssize_t audio_comms_store(struct gov_attr_set *attr_set,
				 const char *buf, size_t count)
{
	return zenith_store_comm_table(&zenith_audio_table,
				       zenith_audio_comms,
				       ARRAY_SIZE(zenith_audio_comms),
				       buf, count);
}
static struct governor_attr audio_comms = __ATTR_RW(audio_comms);

static ssize_t camera_comms_show(struct gov_attr_set *attr_set, char *buf)
{
	return zenith_show_comm_table(&zenith_camera_table, buf);
}

static ssize_t camera_comms_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	return zenith_store_comm_table(&zenith_camera_table,
				       zenith_camera_comms,
				       ARRAY_SIZE(zenith_camera_comms),
				       buf, count);
}
static struct governor_attr camera_comms = __ATTR_RW(camera_comms);

/* camera_active sysfs knob.  Tri-state override:
 *   0  ZENITH_CAMERA_OVERRIDE_AUTO        consult comm table
 *   1  ZENITH_CAMERA_OVERRIDE_FORCE_ON    floor always applied
 *   2  ZENITH_CAMERA_OVERRIDE_FORCE_OFF   floor never applied
 * Out-of-range values (>=3) rejected with EINVAL.
 */
static ssize_t camera_active_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->camera_active);
}

static ssize_t camera_active_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > ZENITH_CAMERA_OVERRIDE_FORCE_OFF)
		return -EINVAL;
	t->camera_active = val;
	return count;
}
static struct governor_attr camera_active = __ATTR_RW(camera_active);

/* camera_floor_pct sysfs knob.  Range 0..100; 0 leaves the comm
 * walk running (when camera_aware=1) but applies no floor.
 */
static ssize_t camera_floor_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->camera_floor_pct);
}

static ssize_t camera_floor_pct_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	t->camera_floor_pct = val;
	return count;
}
static struct governor_attr camera_floor_pct = __ATTR_RW(camera_floor_pct);

/* game_mode sysfs knob.  Strict 0/1 boolean.  See
 * ZENITH_DEFAULT_GAME_MODE comment block for the per-tier overlays
 * that flip behaviour when this is set.  No cache invalidation
 * required: the value is consumed inline in the freq path.
 */
static ssize_t game_mode_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->game_mode);
}

static ssize_t game_mode_store(struct gov_attr_set *attr_set,
			       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;
	unsigned int prev;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_GAME_MODE_MAX)
		return -EINVAL;
	prev = t->game_mode;
	t->game_mode = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_GAME_MODE);
	if (prev != t->game_mode)
		trace_zenith_game_mode(smp_processor_id(), t->game_mode);
	return count;
}
static struct governor_attr game_mode = __ATTR_RW(game_mode);

/* psi_aware sysfs knob.  Strict 0/1 boolean.  See ZENITH_DEFAULT_PSI_AWARE
 * comment block for semantics.  No cache invalidation -- the value is
 * consumed inline at zenith_get_next_freq() time.
 */
static ssize_t psi_aware_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->psi_aware);
}

static ssize_t psi_aware_store(struct gov_attr_set *attr_set,
			       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->psi_aware = val;
	zenith_set_static_key(&zenith_psi_aware_key, val);
	return count;
}
static struct governor_attr psi_aware = __ATTR_RW(psi_aware);

/* psi_mem_thresh sysfs knob.  Range 0..100 (integer percentage of
 * the PSI 10s some-stall average).  0 disables the cap even with
 * psi_aware=1, useful for tracing the helper without changing freq.
 */
static ssize_t psi_mem_thresh_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->psi_mem_thresh);
}

static ssize_t psi_mem_thresh_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	t->psi_mem_thresh = val;
	return count;
}
static struct governor_attr psi_mem_thresh = __ATTR_RW(psi_mem_thresh);

/* psi_cpu_thresh / psi_io_thresh sysfs knobs.  Same shape as
 * psi_mem_thresh: 0..100 integer percent, 0 disables that dimension's
 * cap.  See ZENITH_DEFAULT_PSI_AWARE comment block for semantics.
 */
static ssize_t psi_cpu_thresh_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->psi_cpu_thresh);
}

static ssize_t psi_cpu_thresh_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	t->psi_cpu_thresh = val;
	return count;
}
static struct governor_attr psi_cpu_thresh = __ATTR_RW(psi_cpu_thresh);

static ssize_t psi_io_thresh_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->psi_io_thresh);
}

static ssize_t psi_io_thresh_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	t->psi_io_thresh = val;
	return count;
}
static struct governor_attr psi_io_thresh = __ATTR_RW(psi_io_thresh);

/* boot_boost_ms sysfs knob.  See ZENITH_DEFAULT_BOOT_BOOST_MS comment
 * block for semantics.  Range 0..ZENITH_BOOT_BOOST_MAX_MS;
 * out-of-range values rejected with EINVAL so userspace gets a clear
 * error rather than a silent clamp.  No cache invalidation: the value
 * is consumed inline by the eval path on every tick.
 */
static ssize_t boot_boost_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->boot_boost_ms);
}

static ssize_t boot_boost_ms_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > ZENITH_BOOT_BOOST_MAX_MS)
		return -EINVAL;
	t->boot_boost_ms = val;
	return count;
}
static struct governor_attr boot_boost_ms = __ATTR_RW(boot_boost_ms);

/* frame_budget_us sysfs knob.  Range 0..ZENITH_FRAME_BUDGET_US_MAX
 * (50 ms).  Userspace writes the current vblank period in
 * microseconds whenever the panel changes refresh rate.  0 disables
 * the adaptive frame-budget floor outright.  See the
 * ZENITH_DEFAULT_FRAME_BUDGET_US comment block at the top of the file
 * for typical values per refresh rate.
 */
static ssize_t frame_budget_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->frame_budget_us);
}

static ssize_t frame_budget_us_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > ZENITH_FRAME_BUDGET_US_MAX)
		return -EINVAL;
	WRITE_ONCE(t->frame_budget_us, val);
	return count;
}
static struct governor_attr frame_budget_us = __ATTR_RW(frame_budget_us);

/* frame_budget_us_per_policy sysfs knob.  CSV "cpu:budget_us[,...]"
 * with the per-policy override semantics described in the comment
 * block at the top of the file.  Empty write clears all overrides.
 */
static ssize_t frame_budget_us_per_policy_show(struct gov_attr_set *attr_set,
					       char *buf)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	ssize_t len = 0;
	unsigned int cpu;
	bool first = true;

	for (cpu = 0; cpu < NR_CPUS; cpu++) {
		unsigned int v = t->frame_budget_us_per_policy[cpu];

		if (!v)
			continue;
		len += sprintf(buf + len, "%s%u:%u",
			       first ? "" : ",", cpu, v);
		first = false;
	}
	len += sprintf(buf + len, "\n");
	return len;
}

static ssize_t frame_budget_us_per_policy_store(struct gov_attr_set *attr_set,
						const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int parsed[NR_CPUS] = { 0 };
	const char *p = buf;
	const char *end = buf + count;
	unsigned int cpu;

	/* Empty write (just "\n" or "") clears all overrides. */
	while (p < end && (*p == ' ' || *p == '\t' || *p == '\n'))
		p++;
	if (p == end)
		goto commit;

	while (p < end) {
		unsigned int anchor;
		unsigned int val;
		char *colon;
		char *comma;
		char token[32];
		size_t tlen;

		comma = strnchr(p, end - p, ',');
		tlen = comma ? (size_t)(comma - p) : (size_t)(end - p);
		if (tlen >= sizeof(token))
			return -EINVAL;
		memcpy(token, p, tlen);
		token[tlen] = '\0';
		/* Strip trailing whitespace / newline. */
		while (tlen && (token[tlen - 1] == ' ' ||
				token[tlen - 1] == '\t' ||
				token[tlen - 1] == '\n'))
			token[--tlen] = '\0';
		if (!tlen) {
			p = comma ? comma + 1 : end;
			continue;
		}

		colon = strchr(token, ':');
		if (!colon)
			return -EINVAL;
		*colon = '\0';
		if (kstrtouint(token, 10, &anchor))
			return -EINVAL;
		if (kstrtouint(colon + 1, 10, &val))
			return -EINVAL;
		if (anchor >= NR_CPUS || val > ZENITH_FRAME_BUDGET_US_MAX)
			return -EINVAL;
		parsed[anchor] = val;
		p = comma ? comma + 1 : end;
	}

commit:
	for (cpu = 0; cpu < NR_CPUS; cpu++)
		WRITE_ONCE(t->frame_budget_us_per_policy[cpu], parsed[cpu]);
	return count;
}
static struct governor_attr frame_budget_us_per_policy =
	__ATTR_RW(frame_budget_us_per_policy);

/* frame_pace_floor_pct sysfs knob.  Range 0..100; the value is the
 * 60 Hz baseline floor as a percent of policy->max.  The kernel
 * scales this inversely with frame_budget_us, so the same value
 * gives a higher effective floor at higher refresh rates.  0
 * disables the floor while leaving the tracepoint live.
 */
static ssize_t frame_pace_floor_pct_show(struct gov_attr_set *attr_set,
					 char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->frame_pace_floor_pct);
}

static ssize_t frame_pace_floor_pct_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 100)
		return -EINVAL;
	WRITE_ONCE(t->frame_pace_floor_pct, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_FRAME_PACE);
	return count;
}
static struct governor_attr frame_pace_floor_pct =
	__ATTR_RW(frame_pace_floor_pct);

/*
 * uclamp_min_respect sysfs knob.  See the ZENITH_DEFAULT_UCLAMP_MIN_RESPECT
 * comment block at the top of this file for full semantics.  Normalised to
 * 0/1 on store.  No cache invalidation required -- the value is re-read
 * from tunables on every zenith_get_next_freq() call.
 */
static ssize_t uclamp_min_respect_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->uclamp_min_respect);
}

static ssize_t uclamp_min_respect_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->uclamp_min_respect = val;
	return count;
}
static struct governor_attr uclamp_min_respect = __ATTR_RW(uclamp_min_respect);

/*
 * uclamp_max_respect sysfs knob.  See the ZENITH_DEFAULT_UCLAMP_MAX_RESPECT
 * comment block at the top of this file for full semantics.  Normalised to
 * 0/1 on store.
 */
static ssize_t uclamp_max_respect_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->uclamp_max_respect);
}

static ssize_t uclamp_max_respect_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->uclamp_max_respect = val;
	return count;
}
static struct governor_attr uclamp_max_respect = __ATTR_RW(uclamp_max_respect);

static struct attribute *zenith_attrs[] = {
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&up_threshold.attr,
	&up_threshold_adaptive.attr,
	&up_threshold_hispeed.attr,
	&down_threshold.attr,
	&hispeed_freq.attr,
	&hispeed_freq_pct.attr,
	&hispeed_load.attr,
	&hispeed_hyst_pct.attr,
	&hispeed_entry_streak.attr,
	&brutal_entry_streak.attr,
	&brutal_decay_ms.attr,
	&climb_mode.attr,
	&freq_step_pct.attr,
	&freq_step_adaptive.attr,
	&profile.attr,
	&profile_values.attr,
	&zenith_stats.attr,
	&zenith_stats_reset.attr,
	&at_log.attr,
	&auto_tune_status.attr,
	&auto_tune_reset_overrides.attr,
	&auto_tune.attr,
	&auto_tune_v2.attr,
	&auto_tune_hysteresis_windows.attr,
	&auto_tune_cooldown_windows.attr,
	&auto_tune_cluster_aware.attr,
	&auto_tune_v2_signals.attr,
	&auto_tune_thermal_slope.attr,
	&auto_tune_thermal_pressure_pct.attr,
	&auto_tune_thermal_slope_pct.attr,
	&auto_tune_frame_pacing.attr,
	&auto_tune_sustained_gaming.attr,
	&auto_tune_sat_load_pct.attr,
	&auto_tune_hi_sat_pct.attr,
	&auto_tune_lo_sat_pct.attr,
	&auto_tune_hi_events_x2.attr,
	&auto_tune_lo_events_x2.attr,
	&auto_tune_scenario.attr,
	&powersave_bias.attr,
	&io_is_busy.attr,
	&iowait_boost_min.attr,
	&iowait_stack_pct.attr,
	&iowait_backoff_after_ms.attr,
	&ignore_nice_load.attr,
	&screen_state.attr,
	&screen_auto.attr,
	&thermal_state.attr,
	&thermal_auto.attr,
	&thermal_pressure_continuous.attr,
	&prefer_silver_aware.attr,
	&prefer_silver_hot_threshold_pct.attr,
	&prefer_silver_hot_bump_pct.attr,
	&thermal_util_derate.attr,
	&thermal_derate_rate_pct.attr,
	&freq_stability_margin_pct.attr,
	&down_rate_adaptive.attr,
	&wakeup_boost.attr,
	&down_threshold_adaptive.attr,
	&rate_limit_cluster_scale.attr,
	&input_boost_ms.attr,
	&input_boost_decay_ms.attr,
	&input_boost_decay_curve.attr,
	&input_boost_big_only.attr,
	&input_boost_cap_pct.attr,
	&efficient_freq.attr,
	&eff_bin_hyst_pct.attr,
	&up_delay_us.attr,
	&light_load_freq.attr,
	&light_load_threshold.attr,
	&sampling_down_factor.attr,
	&boost_exit_extend.attr,
	&bias_load_threshold.attr,
	&kcpustat_window_us.attr,
	&kcpustat_filter_shift.attr,
	&kcpustat_hispeed_enable.attr,
	&util_math_v2.attr,
	&uclamp_min_respect.attr,
	&uclamp_max_respect.attr,
	&predict_util_pct.attr,
	&predict_util_smooth.attr,
	&render_aware.attr,
	&render_comms.attr,
	&render_floor_pct.attr,
	&audio_aware.attr,
	&audio_comms.attr,
	&audio_floor_pct.attr,
	&audio_cap_pct.attr,
	&camera_aware.attr,
	&camera_comms.attr,
	&camera_active.attr,
	&camera_floor_pct.attr,
	&game_mode.attr,
	&psi_aware.attr,
	&psi_mem_thresh.attr,
	&psi_cpu_thresh.attr,
	&psi_io_thresh.attr,
	&boot_boost_ms.attr,
	&frame_budget_us.attr,
	&frame_budget_us_per_policy.attr,
	&frame_pace_floor_pct.attr,
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
	tunables->up_threshold_adaptive	= ZENITH_DEFAULT_UP_THRESHOLD_ADAPTIVE;
	tunables->up_threshold_hispeed	= ZENITH_DEFAULT_UP_THRESHOLD_HISPEED;
	tunables->down_threshold	= ZENITH_DEFAULT_DOWN_THRESHOLD;
	tunables->hispeed_freq		= ZENITH_DEFAULT_HISPEED_FREQ;
	tunables->hispeed_freq_pct	= ZENITH_DEFAULT_HISPEED_FREQ_PCT;
	tunables->hispeed_load		= ZENITH_DEFAULT_HISPEED_LOAD;
	tunables->hispeed_hyst_pct	= ZENITH_DEFAULT_HISPEED_HYST_PCT;
	tunables->hispeed_entry_streak	= ZENITH_DEFAULT_HISPEED_ENTRY_STREAK;
	tunables->brutal_entry_streak	= ZENITH_DEFAULT_BRUTAL_ENTRY_STREAK;
	tunables->climb_mode		= ZENITH_DEFAULT_CLIMB_MODE;
	tunables->freq_step_pct		= ZENITH_DEFAULT_FREQ_STEP_PCT;
	tunables->freq_step_adaptive	= ZENITH_DEFAULT_FREQ_STEP_ADAPTIVE;
	tunables->active_profile	= ZENITH_PROFILE_CUSTOM;
	tunables->auto_tune		= 1;
	tunables->auto_tune_sat_load_pct = ZENITH_DEFAULT_AT_SAT_LOAD_PCT;
	tunables->auto_tune_hi_sat_pct	= ZENITH_DEFAULT_AT_HI_SAT_PCT;
	tunables->auto_tune_lo_sat_pct	= ZENITH_DEFAULT_AT_LO_SAT_PCT;
	tunables->auto_tune_hi_events_x2 = ZENITH_DEFAULT_AT_HI_EVENTS_X2;
	tunables->auto_tune_lo_events_x2 = ZENITH_DEFAULT_AT_LO_EVENTS_X2;
	tunables->auto_tune_v2		= ZENITH_DEFAULT_AUTO_TUNE_V2;
	tunables->auto_tune_hysteresis_windows =
		ZENITH_DEFAULT_AT_HYSTERESIS_WINDOWS;
	tunables->auto_tune_cooldown_windows =
		ZENITH_DEFAULT_AT_COOLDOWN_WINDOWS;
	tunables->auto_tune_cluster_aware =
		ZENITH_DEFAULT_AT_CLUSTER_AWARE;
	tunables->auto_tune_v2_signals =
		ZENITH_DEFAULT_AT_V2_SIGNALS;
	tunables->auto_tune_thermal_slope =
		ZENITH_DEFAULT_AT_THERMAL_SLOPE;
	tunables->auto_tune_thermal_pressure_pct =
		ZENITH_DEFAULT_AT_THERMAL_PRESSURE_PCT;
	tunables->auto_tune_thermal_slope_pct =
		ZENITH_DEFAULT_AT_THERMAL_SLOPE_PCT;
	tunables->auto_tune_frame_pacing =
		ZENITH_DEFAULT_AT_FRAME_PACING;
	tunables->auto_tune_sustained_gaming =
		ZENITH_DEFAULT_AT_SUSTAINED_GAMING;
	tunables->auto_tune_scenario	= ZENITH_DEFAULT_AUTO_TUNE_SCENARIO;
	tunables->powersave_bias	= ZENITH_DEFAULT_POWERSAVE_BIAS;
	tunables->io_is_busy		= ZENITH_DEFAULT_IO_IS_BUSY;
	tunables->iowait_boost_min	= ZENITH_DEFAULT_IOWAIT_BOOST_MIN;
	tunables->iowait_stack_pct	= ZENITH_DEFAULT_IOWAIT_STACK_PCT;
	tunables->iowait_backoff_after_ms = ZENITH_DEFAULT_IOWAIT_BACKOFF_AFTER_MS;
	tunables->ignore_nice_load	= 0;
	tunables->screen_state		= 1;
	tunables->screen_auto		= 1;
	tunables->thermal_state		= 0;
	tunables->thermal_auto		= ZENITH_DEFAULT_THERMAL_AUTO;
	tunables->thermal_pressure_continuous =
		ZENITH_DEFAULT_THERMAL_PRESSURE_CONTINUOUS;
	tunables->prefer_silver_aware	= ZENITH_DEFAULT_PREFER_SILVER_AWARE;
	tunables->prefer_silver_hot_threshold_pct =
		ZENITH_DEFAULT_PREFER_SILVER_HOT_THRESHOLD_PCT;
	tunables->prefer_silver_hot_bump_pct =
		ZENITH_DEFAULT_PREFER_SILVER_HOT_BUMP_PCT;
	tunables->brutal_decay_ms	= ZENITH_DEFAULT_BRUTAL_DECAY_MS;
	tunables->thermal_util_derate	= ZENITH_DEFAULT_THERMAL_UTIL_DERATE;
	tunables->thermal_derate_rate_pct = ZENITH_DEFAULT_THERMAL_DERATE_RATE_PCT;
	tunables->freq_stability_margin_pct = ZENITH_DEFAULT_FREQ_STABILITY_MARGIN_PCT;
	tunables->down_rate_adaptive	= ZENITH_DEFAULT_DOWN_RATE_ADAPTIVE;
	tunables->wakeup_boost		= ZENITH_DEFAULT_WAKEUP_BOOST;
	tunables->down_threshold_adaptive = ZENITH_DEFAULT_DOWN_THRESHOLD_ADAPTIVE;
	tunables->rate_limit_cluster_scale = ZENITH_DEFAULT_RATE_LIMIT_CLUSTER_SCALE;
	tunables->input_boost_ms	= ZENITH_DEFAULT_INPUT_BOOST_MS;
	tunables->input_boost_decay_ms	= ZENITH_DEFAULT_INPUT_BOOST_DECAY_MS;
	tunables->input_boost_decay_curve = ZENITH_DEFAULT_INPUT_BOOST_DECAY_CURVE;
	tunables->input_boost_big_only	= ZENITH_DEFAULT_INPUT_BOOST_BIG_ONLY;
	tunables->input_boost_cap_pct	= ZENITH_DEFAULT_INPUT_BOOST_CAP_PCT;
	tunables->efficient_freq	= ZENITH_DEFAULT_EFFICIENT_FREQ;
	tunables->eff_bin_hyst_pct	= ZENITH_DEFAULT_EFF_BIN_HYST_PCT;
	tunables->up_delay_us		= ZENITH_DEFAULT_UP_DELAY_US;
	tunables->light_load_freq	= ZENITH_DEFAULT_LIGHT_LOAD_FREQ;
	tunables->light_load_threshold	= ZENITH_DEFAULT_LIGHT_LOAD_THRESHOLD;
	tunables->sampling_down_factor	= ZENITH_DEFAULT_SAMPLING_DOWN_FACTOR;
	tunables->boost_exit_extend	= ZENITH_DEFAULT_BOOST_EXIT_EXTEND;
	tunables->bias_load_threshold	= ZENITH_DEFAULT_BIAS_LOAD_THRESHOLD;
	tunables->kcpustat_window_us	= ZENITH_DEFAULT_KCPUSTAT_WINDOW_US;
	tunables->kcpustat_filter_shift	= ZENITH_DEFAULT_KCPUSTAT_FILTER_SHIFT;
	tunables->kcpustat_hispeed_enable = ZENITH_DEFAULT_KCPUSTAT_HISPEED_ENABLE;
	tunables->util_math_v2		= ZENITH_DEFAULT_UTIL_MATH_V2;
	tunables->uclamp_min_respect	= ZENITH_DEFAULT_UCLAMP_MIN_RESPECT;
	tunables->uclamp_max_respect	= ZENITH_DEFAULT_UCLAMP_MAX_RESPECT;
	tunables->predict_util_pct	= ZENITH_DEFAULT_PREDICT_UTIL_PCT;
	tunables->predict_util_smooth	= ZENITH_DEFAULT_PREDICT_UTIL_SMOOTH;
	tunables->render_aware		= ZENITH_DEFAULT_RENDER_AWARE;
	tunables->render_floor_pct	= ZENITH_DEFAULT_RENDER_FLOOR_PCT;
	tunables->audio_aware		= ZENITH_DEFAULT_AUDIO_AWARE;
	tunables->audio_floor_pct	= ZENITH_DEFAULT_AUDIO_FLOOR_PCT;
	tunables->audio_cap_pct		= ZENITH_DEFAULT_AUDIO_CAP_PCT;
	tunables->camera_aware		= ZENITH_DEFAULT_CAMERA_AWARE;
	tunables->camera_active		= ZENITH_DEFAULT_CAMERA_ACTIVE;
	tunables->camera_floor_pct	= ZENITH_DEFAULT_CAMERA_FLOOR_PCT;
	tunables->game_mode		= ZENITH_DEFAULT_GAME_MODE;
	tunables->psi_aware		= ZENITH_DEFAULT_PSI_AWARE;
	tunables->psi_mem_thresh	= ZENITH_DEFAULT_PSI_MEM_THRESH;
	tunables->psi_cpu_thresh	= ZENITH_DEFAULT_PSI_CPU_THRESH;
	tunables->psi_io_thresh		= ZENITH_DEFAULT_PSI_IO_THRESH;
	tunables->boot_boost_ms		= ZENITH_DEFAULT_BOOT_BOOST_MS;
	tunables->frame_budget_us	= ZENITH_DEFAULT_FRAME_BUDGET_US;
	tunables->frame_pace_floor_pct	= ZENITH_DEFAULT_FRAME_PACE_FLOOR_PCT;
	WRITE_ONCE(zenith_input_boost_active_ms, ZENITH_DEFAULT_INPUT_BOOST_MS);

	/* Apply a cmdline-picked preset before the sysfs attr set is
	 * published, so userspace sees the cmdline-picked preset as the
	 * initial state of the profile node.
	 *
	 * Precedence:
	 *   1. zenith.policy_profile=N:prof wins for the matching
	 *      anchor cpu (cpumask_first(policy->cpus)) -- asymmetric
	 *      big.LITTLE presets, one policy at a time.
	 *   2. zenith.profile= applies to every unmatched policy
	 *      (the historical global behaviour).
	 *   3. No cmdline override -> CUSTOM (historical default).
	 */
	{
		unsigned int anchor = cpumask_first(policy->cpus);
		unsigned int chosen = ZENITH_PROFILE_CUSTOM;

		if (anchor < NR_CPUS &&
		    zenith_cmdline_policy_profile[anchor] !=
		    ZENITH_PROFILE_CUSTOM)
			chosen = zenith_cmdline_policy_profile[anchor];
		else if (zenith_cmdline_profile != ZENITH_PROFILE_CUSTOM)
			chosen = zenith_cmdline_profile;

		if (chosen != ZENITH_PROFILE_CUSTOM) {
			zenith_apply_profile(tunables, chosen);
			tunables->active_profile = chosen;
		}
	}

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
	zenith_update_cluster_rate_scale(z_policy);
	zenith_reset_local_actions(z_policy);
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

	z_policy->last_freq_update_time = 0;
	z_policy->next_freq = 0;
	z_policy->work_in_progress = false;
	z_policy->limits_changed = false;
	z_policy->cached_raw_freq = 0;
	z_policy->need_freq_update = cpufreq_driver_test_flags(CPUFREQ_NEED_UPDATE_LIMITS);

	/* Cache the cluster-topology bit used by input_boost_big_only.
	 * SCHED_CAPACITY_SCALE is the normalised top capacity; any CPU
	 * in the policy hitting that value means this policy belongs
	 * to the system's highest-capacity cluster(s).
	 */
	z_policy->is_big_cluster = false;
	for_each_cpu(cpu, policy->cpus) {
		if (arch_scale_cpu_capacity(cpu) >= SCHED_CAPACITY_SCALE) {
			z_policy->is_big_cluster = true;
			break;
		}
	}
	zenith_update_cluster_rate_scale(z_policy);
	zenith_reset_local_actions(z_policy);

	/* Zero the uclamp cache so zenith_policy_uclamp_{min,max} refresh
	 * on the first eval after start rather than returning stale
	 * zeros cached from a previous attach cycle.
	 */
	z_policy->cached_uclamp_min = 0;
	z_policy->cached_uclamp_max = SCHED_CAPACITY_SCALE;
	z_policy->uclamp_cache_stamp_ns = 0;

	memset(z_policy->stats, 0, sizeof(z_policy->stats));
	z_policy->at_last_state =
		zenith_profile_to_at_state(z_policy->tunables->active_profile);
	z_policy->at_pending_state = z_policy->at_last_state;
	z_policy->at_pending_windows = 0;
	z_policy->at_cooldown_left = 0;
	z_policy->at_last_target = z_policy->tunables->active_profile;

	for_each_cpu(cpu, policy->cpus) {
		struct zenith_cpu *z_cpu = &per_cpu(zenith_cpu, cpu);
		memset(z_cpu, 0, sizeof(*z_cpu));
		z_cpu->cpu = cpu;
		z_cpu->z_policy = z_policy;
		
		cpufreq_add_update_util_hook(cpu, &z_cpu->update_util, 
			policy_is_shared(policy) ? zenith_update_shared : zenith_update_single);
	}

	/* Arm the auto-tune classifier when the tunable is enabled.  The
	 * default tunables_init() sets auto_tune=1, but the only place
	 * that schedules at_work is auto_tune_store() on a 0->1
	 * transition.  Without this hook, the classifier silently never
	 * runs on stock defaults; userspace has to write 0 then 1 to
	 * arm it.  Mirror the body of the val=1 branch in
	 * auto_tune_store() so start-time and runtime behaviour agree.
	 */
	if (z_policy->tunables->auto_tune) {
		z_policy->at_last_events =
			atomic64_read(&zenith_auto_input_events);
		atomic_set(&z_policy->at_samples_total, 0);
		atomic_set(&z_policy->at_samples_saturated, 0);
		z_policy->at_pending_windows = 0;
		z_policy->at_cooldown_left = 0;
		schedule_delayed_work(&z_policy->at_work,
			msecs_to_jiffies(ZENITH_AUTO_TUNE_PERIOD_MS));
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
	/* Release-store the flag so the subsequent acquire-load in
	 * zenith_should_update_freq() observes every write to
	 * policy->{min,max} that cpufreq_policy_apply_limits() made
	 * above.  smp_wmb() previously used here is a write-write
	 * barrier that does NOT establish a happens-before edge with
	 * the read side; release/acquire is the documented idiom.
	 */
	smp_store_release(&z_policy->limits_changed, true);
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

/* Common back-end shared by every panel-event source.
 *
 * Both the legacy fb_notifier callback and the optional
 * drm_panel_notifier callback funnel here so the screen_state write
 * happens in exactly one place.  Splitting them into a helper also
 * makes it cheap for vendor / out-of-tree drivers to deliver panel
 * events directly without registering a notifier (e.g. a vendor
 * mode-set handler can call this from its own ioctl path; the
 * function has no module-level dependencies).
 */
static void zenith_panel_blank_event(int blank, unsigned int unblank_value)
{
	unsigned int new_state = (blank == (int)unblank_value) ? 1 : 0;

	/* All zenith policies share one global_tunables (per-cluster
	 * clones hold a reference to the same struct), so a single write
	 * propagates everywhere.
	 */
	mutex_lock(&global_tunables_lock);
	if (global_tunables && global_tunables->screen_auto)
		WRITE_ONCE(global_tunables->screen_state, new_state);
	mutex_unlock(&global_tunables_lock);
}

#ifdef CONFIG_FB_NOTIFY
static int zenith_fb_notifier_cb(struct notifier_block *nb,
				 unsigned long action, void *data)
{
	struct fb_event *evdata = data;
	int blank;

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
	zenith_panel_blank_event(blank, FB_BLANK_UNBLANK);

	return NOTIFY_OK;
}

static struct notifier_block zenith_fb_notifier = {
	.notifier_call	= zenith_fb_notifier_cb,
	.priority	= 0,
};
#endif /* CONFIG_FB_NOTIFY */

#ifdef CONFIG_DRM_PANEL_NOTIFY
/* drm_panel_notifier callback.
 *
 * Called by the vendor drm panel notifier chain on display blank
 * transitions.  The vendor convention (Qualcomm and most adopters) is
 * to deliver a struct drm_panel_notifier whose .data field points to
 * an int holding DRM_PANEL_BLANK_UNBLANK, DRM_PANEL_BLANK_POWERDOWN
 * or DRM_PANEL_BLANK_LP.  Treat anything that is not UNBLANK as a
 * powered-off panel so screen_state goes to 0.
 */
static int zenith_drm_panel_notifier_cb(struct notifier_block *nb,
					unsigned long action, void *data)
{
	struct drm_panel_notifier *evdata = data;
	int blank;

	if (action != DRM_PANEL_EVENT_BLANK)
		return NOTIFY_OK;
	if (!evdata || !evdata->data)
		return NOTIFY_OK;

	blank = *(int *)evdata->data;
	zenith_panel_blank_event(blank, DRM_PANEL_BLANK_UNBLANK);

	return NOTIFY_OK;
}

static struct notifier_block zenith_drm_notifier = {
	.notifier_call	= zenith_drm_panel_notifier_cb,
	.priority	= 0,
};
#endif /* CONFIG_DRM_PANEL_NOTIFY */

static int __init zenith_gov_init(void)
{
	int ret;
	bool input_registered = false;
#ifdef CONFIG_FB_NOTIFY
	bool fb_registered = false;
#endif
#ifdef CONFIG_DRM_PANEL_NOTIFY
	bool drm_registered = false;
#endif

	pr_info("Zenith: V2 Dreadnought (EAS/EM/Display/Thermal) Initialized. By ENI for LO.\n");

	/* Allocate the initial RCU comm tables from the in-tree default
	 * arrays.  Failure here is non-fatal: zenith_policy_has_X()
	 * checks for a NULL table on the read side, so the awareness
	 * features simply skip the comm walk and fall through to the
	 * existing override paths (camera_active, render override, etc.)
	 * until userspace populates the tables via the sysfs nodes.
	 */
	rcu_assign_pointer(zenith_render_table,
		zenith_alloc_comm_table_from_defaults(zenith_render_comms,
			ARRAY_SIZE(zenith_render_comms)));
	rcu_assign_pointer(zenith_audio_table,
		zenith_alloc_comm_table_from_defaults(zenith_audio_comms,
			ARRAY_SIZE(zenith_audio_comms)));
	rcu_assign_pointer(zenith_camera_table,
		zenith_alloc_comm_table_from_defaults(zenith_camera_comms,
			ARRAY_SIZE(zenith_camera_comms)));

	ret = input_register_handler(&zenith_input_handler);
	if (ret)
		pr_warn("Zenith: input handler register failed (%d), boost disabled\n",
			ret);
	else
		input_registered = true;

	/* Panel-state delivery: register the drm_panel_notifier path first
	 * (preferred when available because the fb notifier chain is
	 * deprecated upstream and absent on most modern vendor builds), and
	 * fall back to fb_register_client() if drm is unavailable or its
	 * registration fails.  Registering both at once would cause every
	 * blank/unblank event to write screen_state twice, so the fb
	 * registration is skipped when drm is live.
	 */
#ifdef CONFIG_DRM_PANEL_NOTIFY
	ret = drm_panel_notifier_register(&zenith_drm_notifier);
	if (ret)
		pr_warn("Zenith: drm panel notifier register failed (%d), trying fb fallback\n",
			ret);
	else
		drm_registered = true;
#endif
#ifdef CONFIG_FB_NOTIFY
	if (
#ifdef CONFIG_DRM_PANEL_NOTIFY
	    !drm_registered &&
#endif
	    1) {
		ret = fb_register_client(&zenith_fb_notifier);
		if (ret)
			pr_warn("Zenith: fb notifier register failed (%d), screen_auto disabled\n",
				ret);
		else
			fb_registered = true;
	}
#endif
#if !defined(CONFIG_FB_NOTIFY) && !defined(CONFIG_DRM_PANEL_NOTIFY)
	/* screen_auto stores still accept 0/1 (the field is plain
	 * bookkeeping for userspace introspection) but no notifier
	 * will ever flip screen_state.  Print once at init so an
	 * operator wondering "why is screen_auto=1 not affecting the
	 * frequency" can grep dmesg and find the answer immediately
	 * rather than chasing the runtime path.
	 */
	pr_info("Zenith: CONFIG_FB_NOTIFY=n and CONFIG_DRM_PANEL_NOTIFY=n, screen_auto is bookkeeping-only (no panel events delivered)\n");
#endif
#ifdef CONFIG_DRM_PANEL_NOTIFY
	if (drm_registered)
		pr_info("Zenith: screen_auto wired through drm panel notifier\n");
#endif

	ret = cpufreq_register_governor(&zenith_gov);
	if (ret) {
		/* Roll back the input + fb / drm hooks we successfully
		 * registered above so that a probe failure here
		 * leaves no dangling notifier / handler bound to a
		 * governor that does not exist.  Previously the
		 * function returned the error and silently leaked
		 * both registrations; subsequent module-style
		 * insmod/rmmod cycles would double-register.
		 */
#ifdef CONFIG_DRM_PANEL_NOTIFY
		if (drm_registered)
			drm_panel_notifier_unregister(&zenith_drm_notifier);
#endif
#ifdef CONFIG_FB_NOTIFY
		if (fb_registered)
			fb_unregister_client(&zenith_fb_notifier);
#endif
		if (input_registered)
			input_unregister_handler(&zenith_input_handler);
		pr_err("Zenith: cpufreq_register_governor failed (%d)\n", ret);
		return ret;
	}
	return 0;
}
fs_initcall(zenith_gov_init);