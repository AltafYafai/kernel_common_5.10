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
#include <trace/events/power.h>

#define CREATE_TRACE_POINTS
#include <trace/events/cpufreq_zenith.h>

/* Constants & Defaults */
/* Permille of SCHED_CAPACITY_SCALE at which iowait boost starts.
 * 125 == SCHED_CAPACITY_SCALE / 8, preserving the historical default.
 */
#define ZENITH_DEFAULT_IOWAIT_BOOST_MIN		125
#define ZENITH_DEFAULT_IOWAIT_STACK_PCT		50	/* 0 = legacy max(util, boost) */
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

/* Time-based cache TTL for the uclamp_{min,max} per-policy walks.  The
 * per-rq UCLAMP values are maintained by the scheduler on every
 * enqueue / dequeue, so a 1 ms staleness bound on the cached
 * aggregate is imperceptible to userspace (ADPF sessions are open
 * for tens of milliseconds to seconds) but drops the per-policy rq
 * walk from every freq eval down to once per millisecond.
 */
#define ZENITH_UCLAMP_CACHE_TTL_NS		(1 * NSEC_PER_MSEC)
#define ZENITH_EFF_BINS_MAX			4
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
#define ZENITH_DEFAULT_THERMAL_AUTO		1
#define ZENITH_THERMAL_AUTO_PRESSURE_PCT	10

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
#define ZENITH_DEFAULT_UP_RATE_LIMIT_US		100
#define ZENITH_DEFAULT_DOWN_RATE_LIMIT_US	4000
#define ZENITH_DEFAULT_POWERSAVE_BIAS		0
#define ZENITH_DEFAULT_IO_IS_BUSY		1
#define ZENITH_DEFAULT_INPUT_BOOST_MS		80
#define ZENITH_DEFAULT_INPUT_BOOST_DECAY_MS	30
#define ZENITH_DEFAULT_INPUT_BOOST_BIG_ONLY	1
#define ZENITH_DEFAULT_INPUT_BOOST_CAP_PCT	80	/* 0 = no cap, pin to policy->max */
#define ZENITH_DEFAULT_EFFICIENT_FREQ		0
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
 * The tunable itself is just a 0/1 flag; userspace (a small
 * gameswitch helper, or a Realme `/proc/touchpanel/game_switch_enable`
 * watcher) is expected to flip it.  Values >1 are normalised to 1.
 */
#define ZENITH_DEFAULT_GAME_MODE			0
#define ZENITH_GAME_HISPEED_BOOST_PCT		110
#define ZENITH_GAME_BOOST_DECAY_PCT		130

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
#define ZENITH_DEFAULT_FRAME_PACE_FLOOR_PCT	0
#define ZENITH_FRAME_PACE_BASE_BUDGET_US	16667

/* psi_aware (default 0, off) + psi_mem_thresh (default 50):
 *
 * When psi_aware=1, zenith_get_next_freq() reads the system-wide
 * memory-pressure 10s average from psi_system.avg[PSI_MEM_SOME][0].
 * If the integer percentage is at or above psi_mem_thresh, the final
 * freq is *capped* at the effective hispeed floor.  Rationale: under
 * heavy memory stall, pushing the CPU above hispeed wastes energy on
 * cycles that mostly stall waiting for memory; the workload is
 * memory-bound, not compute-bound.  When the hispeed tier is disabled
 * (eff_hispeed == 0) the cap is policy->max -- i.e. a no-op fallback.
 *
 * The reader is RCU-free and lock-free: psi_system.avg[][] is updated
 * by the avgs_work delayed work and a single READ_ONCE is sufficient
 * to get a coherent fixed-point value.  When CONFIG_PSI is off or
 * psi_disabled is set, the helper returns 0 and the cap never fires.
 *
 * 0..100 range; values >100 rejected by sysfs.  0 disables the cap
 * even with psi_aware=1 (useful for tracing without changing freq).
 */
#define ZENITH_DEFAULT_PSI_AWARE		0
#define ZENITH_DEFAULT_PSI_MEM_THRESH		50

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

	/* See ZENITH_DEFAULT_THERMAL_UTIL_DERATE comment block.  When
	 * set, zenith_get_util() scales util_out by the (cap - pressure)
	 * / cap fraction whenever pressure exceeds
	 * ZENITH_THERMAL_DERATE_FLOOR_PCT.  Smooths the thermal dance
	 * by asking for less than the throttled max.
	 */
	unsigned int		thermal_util_derate;

	/* Input boost duration (ms). 0 = disabled. */
	unsigned int		input_boost_ms;
	unsigned int		input_boost_decay_ms;

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

	/* See ZENITH_DEFAULT_RENDER_AWARE / ZENITH_DEFAULT_RENDER_FLOOR_PCT. */
	unsigned int		render_aware;
	unsigned int		render_floor_pct;

	/* See ZENITH_DEFAULT_GAME_MODE. 0/1, normalised on store. */
	unsigned int		game_mode;

	/* See ZENITH_DEFAULT_PSI_AWARE / ZENITH_DEFAULT_PSI_MEM_THRESH. */
	unsigned int		psi_aware;
	unsigned int		psi_mem_thresh;

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
	struct delayed_work	at_work;

	/* Cached topology bit: true when any CPU in the policy has
	 * arch_scale_cpu_capacity == SCHED_CAPACITY_SCALE, i.e. the
	 * policy belongs to (one of) the system's highest-capacity
	 * cluster(s).  Computed once at zenith_start() time from the
	 * policy's cpumask and kept for the lifetime of the policy.
	 * Used by the input-boost gate (input_boost_big_only) to skip
	 * boosting small-cluster policies on heterogeneous SoCs.
	 */
	bool			is_big_cluster;

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

	/* Previous *unpredicted* zenith_get_util() output for this CPU,
	 * used as the second tap of the one-step-ahead predictor.  Only
	 * read/written when tunables->predict_util_pct != 0.  Initialised
	 * to zero by the kzalloc-style allocation in zenith_start().
	 */
	unsigned long		prev_util;

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

			if (trace_zenith_thermal_derate_enabled())
				trace_zenith_thermal_derate(z_cpu->cpu,
							    util_out, derated,
							    (unsigned int)pressure_pct);
			util_out = derated;
		}
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

		if (predict_pct > ZENITH_PREDICT_UTIL_PCT_MAX)
			predict_pct = ZENITH_PREDICT_UTIL_PCT_MAX;

		if (util_out > prev) {
			unsigned long delta = util_out - prev;
			unsigned long pred = util_out +
					     (delta * predict_pct) / 100;

			if (pred > max)
				pred = max;
			if (trace_zenith_predict_enabled())
				trace_zenith_predict(z_cpu->cpu, predict_pct,
						     util_out, pred);
			z_cpu->prev_util = util_out;
			util_out = pred;
		} else {
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
	unsigned int wall_delta, idle_delta;

	cur_idle = get_cpu_idle_time(z_cpu->cpu, &cur_wall, 1);
	wall_delta = (unsigned int)(cur_wall - z_cpu->kc_prev_wall_time);

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
		     (unsigned int)(cur_idle - z_cpu->kc_prev_idle_time) : 0;

	z_cpu->kc_busy_pct = (wall_delta > idle_delta) ?
		((100u * (wall_delta - idle_delta)) / wall_delta) : 0;

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
	s64 down_delay = z_policy->down_rate_delay_ns *
			 (s64)max(z_policy->down_rate_mult, 1U);

	if (next_freq > z_policy->next_freq) {
		unsigned int spike = z_policy->policy->max >> ZENITH_SPIKE_SHIFT;

		if (next_freq - z_policy->next_freq >= spike)
			return false;
		if (delta_ns < z_policy->up_rate_delay_ns)
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
		if (z_policy->tunables->game_mode)
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

/* List of comm prefixes treated as render / display-pipeline threads.
 * Matched by strncmp() over the first N characters where N is the
 * length of the table entry, so the userspace task only needs to
 * have its first N chars match (Android's "RenderThread NNN" naming
 * for libui's per-app render thread, for instance, matches the
 * "RenderThread" prefix).  Order of entries is irrelevant for
 * correctness; keep the most common first for cache-friendliness.
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
	for_each_cpu(cpu, policy->cpus) {
		struct task_struct *curr = READ_ONCE(cpu_curr(cpu));
		int i;

		if (!curr)
			continue;
		for (i = 0; i < ARRAY_SIZE(zenith_render_comms); i++) {
			const char *needle = zenith_render_comms[i];

			if (!strncmp(curr->comm, needle, strlen(needle))) {
				match = true;
				break;
			}
		}
		if (match)
			break;
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
	for_each_cpu(cpu, policy->cpus) {
		struct task_struct *curr = READ_ONCE(cpu_curr(cpu));
		int i;

		if (!curr)
			continue;
		for (i = 0; i < ARRAY_SIZE(zenith_audio_comms); i++) {
			const char *needle = zenith_audio_comms[i];

			if (!strncmp(curr->comm, needle, strlen(needle))) {
				match = true;
				break;
			}
		}
		if (match)
			break;
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
		for (i = 0; i < ARRAY_SIZE(zenith_camera_comms); i++) {
			const char *needle = zenith_camera_comms[i];

			if (!strncmp(curr->comm, needle, strlen(needle))) {
				match = true;
				break;
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
		dynamic_up_thresh = 90; /* Relaxed for thermals */
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
	if (z_policy->tunables->input_boost_ms &&
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
			unsigned int cap_pct =
				z_policy->tunables->input_boost_cap_pct;
			unsigned int boost_ceiling = (cap_pct && cap_pct <= 100) ?
				(policy->max / 100) * cap_pct : policy->max;

			/* game_mode stretch: lengthen the trailing decay
			 * window by ZENITH_GAME_BOOST_DECAY_PCT (default
			 * 130%%, i.e. ~30%% longer) so input boosts hold
			 * the freq floor longer across stick-flick / camera
			 * pan inputs.  Pure no-op when game_mode=0.
			 */
			if (z_policy->tunables->game_mode && decay_ms)
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
				goto resolve;
			} else if (decay_ns) {
				/* Decay phase: linearly ramp a floor from
				 * boost_ceiling down toward policy->min over
				 * the trailing decay_ns.  Normal eval runs
				 * after this point and may pick a higher freq;
				 * the floor only kicks in if the load has
				 * already dropped so far that eval undershoots
				 * the ramp.
				 */
				u64 elapsed = decay_ns - remaining;
				u64 span = boost_ceiling - policy->min;

				input_boost_floor = boost_ceiling -
					(unsigned int)div64_u64(span * elapsed,
								decay_ns);
			} else {
				/* Decay window not configured: original cliff. */
				freq = boost_ceiling;
				tp_path = "input_boost";
				goto resolve;
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
	if (z_policy->tunables->frame_budget_us &&
	    z_policy->tunables->frame_pace_floor_pct) {
		unsigned int budget_us = z_policy->tunables->frame_budget_us;
		unsigned int base_pct =
			z_policy->tunables->frame_pace_floor_pct;
		unsigned int eff_pct;
		unsigned int fp_floor;

		eff_pct = (base_pct * ZENITH_FRAME_PACE_BASE_BUDGET_US) /
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

	/* 3d. uclamp_max final-freq cap.  Applied after every other tier
	 * so that brutality snap, hispeed floor, input boost, and the
	 * uclamp_min / input_boost_decay floors can't walk over an
	 * explicit power-efficiency hint.  A uclamp_min floor higher
	 * than the uclamp_max cap wins by construction (floor applies
	 * first, cap would clamp it down below uclamp_min only when the
	 * two hints disagree, and per-task uclamp validation already
	 * prevents that at the scheduler layer).
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
	 * psi_aware=1.
	 */
	if (ZENITH_FEATURE_ENABLED(psi_aware) &&
	    z_policy->tunables->psi_mem_thresh) {
		unsigned int mem_some = zenith_psi_mem_some_pct();

		if (mem_some >= z_policy->tunables->psi_mem_thresh) {
			unsigned int psi_cap = zenith_eff_hispeed_freq(z_policy);

			if (!psi_cap)
				psi_cap = policy->max;
			if (freq > psi_cap) {
				freq = psi_cap;
				tp_path = "psi_mem_cap";
			}
		}
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
				      tp_load_pct, freq, target_freq, kc_pct);
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
		t->up_rate_limit_us	= 0;
		t->down_rate_limit_us	= 8000;
		t->up_threshold		= 65;
		t->down_threshold	= 45;
		t->hispeed_freq_pct	= 60;	/* engage tier at 60%% policy->max */
		t->hispeed_load		= 55;	/* must stay < up_threshold */
		t->climb_mode		= ZENITH_CLIMB_MODE_SNAP;
		t->freq_step_pct	= 15;
		t->powersave_bias	= 0;
		t->bias_load_threshold	= 50;
		t->ignore_nice_load	= 0;
		t->input_boost_ms	= 150;
		t->input_boost_decay_ms	= 50;
		t->input_boost_cap_pct	= 0;	/* PERFORMANCE: pin all the way to max */
		t->light_load_threshold	= 15;
		t->sampling_down_factor	= 4;
		t->thermal_auto		= 1;
		t->screen_auto		= 1;
		t->util_math_v2		= 1;
		t->kcpustat_hispeed_enable = 1;
		break;

	case ZENITH_PROFILE_BALANCED:
		t->up_rate_limit_us	= ZENITH_DEFAULT_UP_RATE_LIMIT_US;
		t->down_rate_limit_us	= ZENITH_DEFAULT_DOWN_RATE_LIMIT_US;
		t->up_threshold		= ZENITH_DEFAULT_UP_THRESHOLD;
		t->down_threshold	= ZENITH_DEFAULT_DOWN_THRESHOLD;
		t->hispeed_freq_pct	= ZENITH_DEFAULT_HISPEED_FREQ_PCT;
		t->hispeed_load		= ZENITH_DEFAULT_HISPEED_LOAD;
		t->climb_mode		= ZENITH_CLIMB_MODE_SNAP;
		t->freq_step_pct	= ZENITH_DEFAULT_FREQ_STEP_PCT;
		t->powersave_bias	= 50;	/* 5% gentle bias */
		t->bias_load_threshold	= 40;
		t->ignore_nice_load	= 1;
		t->input_boost_ms	= ZENITH_DEFAULT_INPUT_BOOST_MS;
		t->input_boost_decay_ms	= ZENITH_DEFAULT_INPUT_BOOST_DECAY_MS;
		t->input_boost_cap_pct	= ZENITH_DEFAULT_INPUT_BOOST_CAP_PCT;
		t->light_load_threshold	= ZENITH_DEFAULT_LIGHT_LOAD_THRESHOLD;
		t->sampling_down_factor	= ZENITH_DEFAULT_SAMPLING_DOWN_FACTOR;
		t->thermal_auto		= 1;
		t->screen_auto		= 1;
		t->util_math_v2		= 1;
		t->kcpustat_hispeed_enable = 1;
		break;

	case ZENITH_PROFILE_BATTERY:
		t->up_rate_limit_us	= 500;
		t->down_rate_limit_us	= 2000;
		t->up_threshold		= 85;
		t->down_threshold	= 40;
		t->hispeed_freq_pct	= 0;	/* battery: skip tier entirely */
		t->hispeed_load		= 75;	/* must stay < up_threshold */
		t->climb_mode		= ZENITH_CLIMB_MODE_STEP;
		t->freq_step_pct	= 8;
		t->powersave_bias	= 150;	/* 15% */
		t->bias_load_threshold	= 35;
		t->ignore_nice_load	= 1;
		t->input_boost_ms	= 40;
		t->input_boost_decay_ms	= 10;
		t->input_boost_cap_pct	= 60;	/* BATTERY: cap boost ceiling at 60%% of max */
		t->light_load_threshold	= 30;
		t->sampling_down_factor	= 1;
		t->thermal_auto		= 1;
		t->screen_auto		= 1;
		t->util_math_v2		= 1;
		t->kcpustat_hispeed_enable = 0;
		break;

	case ZENITH_PROFILE_LEGACY:
		/* Approximates cpufreq_ondemand: plain up_threshold with
		 * no hysteresis, no input boost, nice-load ignored.
		 */
		t->up_rate_limit_us	= 2000;
		t->down_rate_limit_us	= 4000;
		t->up_threshold		= 80;
		t->down_threshold	= 80;	/* collapses hysteresis */
		t->hispeed_freq_pct	= 0;	/* legacy: plain up_threshold only */
		t->hispeed_load		= 90;
		t->climb_mode		= ZENITH_CLIMB_MODE_SNAP;
		t->freq_step_pct	= 5;
		t->powersave_bias	= 0;
		t->bias_load_threshold	= 50;
		t->ignore_nice_load	= 1;
		t->input_boost_ms	= 0;
		t->input_boost_decay_ms	= 0;
		t->input_boost_cap_pct	= 0;	/* LEGACY: boost disabled, cap is moot */
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
	else
		pr_warn("zenith.profile=%s: unknown preset, ignored\n", s);
	return 1;
}
early_param("zenith.profile", zenith_setup_profile);

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

	if (trace_zenith_auto_tune_enabled())
		trace_zenith_auto_tune(z_policy->policy->cpu, sat_pct,
				       events_rate_x2, t->active_profile,
				       target);

	/* Scenario overlay (auto_tune_scenario=1).  Sample the four
	 * scenarios at classification time and let strict precedence
	 * pick a target that overrides the load + input-rate result.
	 * Each comm walk caches its result for a few ms in the per-policy
	 * cache, so calling them here is cheap (one walk per scenario at
	 * the 10s classifier tick) and never re-walks if the hot path
	 * just ran them.
	 */
	if (t->auto_tune_scenario) {
		bool audio = zenith_policy_has_audio(z_policy);
		unsigned int cam_override = t->camera_active;
		bool camera;
		bool render = zenith_policy_has_render(z_policy);
		bool memstall = false;
		unsigned int prev = target;

		if (cam_override == ZENITH_CAMERA_OVERRIDE_FORCE_ON)
			camera = true;
		else if (cam_override == ZENITH_CAMERA_OVERRIDE_FORCE_OFF)
			camera = false;
		else
			camera = zenith_policy_has_camera(z_policy);

		if (t->psi_mem_thresh) {
			unsigned int mem_some = zenith_psi_mem_some_pct();

			memstall = (mem_some >= t->psi_mem_thresh);
		}

		if (camera || render)
			target = ZENITH_PROFILE_PERFORMANCE;
		else if (memstall)
			target = ZENITH_PROFILE_BATTERY;
		else if (audio)
			target = ZENITH_PROFILE_BALANCED;
		/* else: leave target as the classifier's pick */

		if (trace_zenith_auto_tune_scenario_enabled())
			trace_zenith_auto_tune_scenario(
				z_policy->policy->cpu, audio, camera,
				render, memstall, prev, target);
	}

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
			atomic_set(&z_policy->at_samples_total, 0);
			atomic_set(&z_policy->at_samples_saturated, 0);
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

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->auto_tune_scenario = !!val;
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

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->input_boost_big_only = !!val;
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

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->boost_exit_extend = !!val;
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

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->kcpustat_hispeed_enable = !!val;
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

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->util_math_v2 = !!val;
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

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->render_aware = !!val;
	zenith_set_static_key(&zenith_render_aware_key, !!val);
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

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->audio_aware = !!val;
	zenith_set_static_key(&zenith_audio_aware_key, !!val);
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

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->camera_aware = !!val;
	zenith_set_static_key(&zenith_camera_aware_key, !!val);
	return count;
}
static struct governor_attr camera_aware = __ATTR_RW(camera_aware);

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

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	prev = t->game_mode;
	t->game_mode = !!val;
	if (prev != t->game_mode)
		trace_zenith_game_mode(smp_processor_id(), !!t->game_mode);
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

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->psi_aware = !!val;
	zenith_set_static_key(&zenith_psi_aware_key, !!val);
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
	t->frame_budget_us = val;
	return count;
}
static struct governor_attr frame_budget_us = __ATTR_RW(frame_budget_us);

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
	t->frame_pace_floor_pct = val;
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

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->uclamp_min_respect = !!val;
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

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->uclamp_max_respect = !!val;
	return count;
}
static struct governor_attr uclamp_max_respect = __ATTR_RW(uclamp_max_respect);

static struct attribute *zenith_attrs[] = {
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&up_threshold.attr,
	&up_threshold_hispeed.attr,
	&down_threshold.attr,
	&hispeed_freq.attr,
	&hispeed_freq_pct.attr,
	&hispeed_load.attr,
	&hispeed_hyst_pct.attr,
	&hispeed_entry_streak.attr,
	&climb_mode.attr,
	&freq_step_pct.attr,
	&profile.attr,
	&auto_tune.attr,
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
	&ignore_nice_load.attr,
	&screen_state.attr,
	&screen_auto.attr,
	&thermal_state.attr,
	&thermal_auto.attr,
	&thermal_util_derate.attr,
	&input_boost_ms.attr,
	&input_boost_decay_ms.attr,
	&input_boost_big_only.attr,
	&input_boost_cap_pct.attr,
	&efficient_freq.attr,
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
	&render_aware.attr,
	&render_floor_pct.attr,
	&audio_aware.attr,
	&audio_floor_pct.attr,
	&audio_cap_pct.attr,
	&camera_aware.attr,
	&camera_active.attr,
	&camera_floor_pct.attr,
	&game_mode.attr,
	&psi_aware.attr,
	&psi_mem_thresh.attr,
	&boot_boost_ms.attr,
	&frame_budget_us.attr,
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
	tunables->up_threshold_hispeed	= ZENITH_DEFAULT_UP_THRESHOLD_HISPEED;
	tunables->down_threshold	= ZENITH_DEFAULT_DOWN_THRESHOLD;
	tunables->hispeed_freq		= ZENITH_DEFAULT_HISPEED_FREQ;
	tunables->hispeed_freq_pct	= ZENITH_DEFAULT_HISPEED_FREQ_PCT;
	tunables->hispeed_load		= ZENITH_DEFAULT_HISPEED_LOAD;
	tunables->hispeed_hyst_pct	= ZENITH_DEFAULT_HISPEED_HYST_PCT;
	tunables->hispeed_entry_streak	= ZENITH_DEFAULT_HISPEED_ENTRY_STREAK;
	tunables->climb_mode		= ZENITH_DEFAULT_CLIMB_MODE;
	tunables->freq_step_pct		= ZENITH_DEFAULT_FREQ_STEP_PCT;
	tunables->active_profile	= ZENITH_PROFILE_CUSTOM;
	tunables->auto_tune		= 1;
	tunables->auto_tune_sat_load_pct = ZENITH_DEFAULT_AT_SAT_LOAD_PCT;
	tunables->auto_tune_hi_sat_pct	= ZENITH_DEFAULT_AT_HI_SAT_PCT;
	tunables->auto_tune_lo_sat_pct	= ZENITH_DEFAULT_AT_LO_SAT_PCT;
	tunables->auto_tune_hi_events_x2 = ZENITH_DEFAULT_AT_HI_EVENTS_X2;
	tunables->auto_tune_lo_events_x2 = ZENITH_DEFAULT_AT_LO_EVENTS_X2;
	tunables->auto_tune_scenario	= ZENITH_DEFAULT_AUTO_TUNE_SCENARIO;
	tunables->powersave_bias	= ZENITH_DEFAULT_POWERSAVE_BIAS;
	tunables->io_is_busy		= ZENITH_DEFAULT_IO_IS_BUSY;
	tunables->iowait_boost_min	= ZENITH_DEFAULT_IOWAIT_BOOST_MIN;
	tunables->iowait_stack_pct	= ZENITH_DEFAULT_IOWAIT_STACK_PCT;
	tunables->ignore_nice_load	= 0;
	tunables->screen_state		= 1;
	tunables->screen_auto		= 1;
	tunables->thermal_state		= 0;
	tunables->thermal_auto		= ZENITH_DEFAULT_THERMAL_AUTO;
	tunables->thermal_util_derate	= ZENITH_DEFAULT_THERMAL_UTIL_DERATE;
	tunables->input_boost_ms	= ZENITH_DEFAULT_INPUT_BOOST_MS;
	tunables->input_boost_decay_ms	= ZENITH_DEFAULT_INPUT_BOOST_DECAY_MS;
	tunables->input_boost_big_only	= ZENITH_DEFAULT_INPUT_BOOST_BIG_ONLY;
	tunables->input_boost_cap_pct	= ZENITH_DEFAULT_INPUT_BOOST_CAP_PCT;
	tunables->efficient_freq	= ZENITH_DEFAULT_EFFICIENT_FREQ;
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
	tunables->boot_boost_ms		= ZENITH_DEFAULT_BOOT_BOOST_MS;
	tunables->frame_budget_us	= ZENITH_DEFAULT_FRAME_BUDGET_US;
	tunables->frame_pace_floor_pct	= ZENITH_DEFAULT_FRAME_PACE_FLOOR_PCT;
	WRITE_ONCE(zenith_input_boost_active_ms, ZENITH_DEFAULT_INPUT_BOOST_MS);

	/* If zenith.profile= was passed on the kernel cmdline, apply it
	 * now (once, on the first policy that triggers global_tunables
	 * creation). This happens before the sysfs attr set is published
	 * by kobject_init_and_add() below, so userspace sees the
	 * cmdline-picked preset as the initial state of the profile node.
	 */
	if (zenith_cmdline_profile != ZENITH_PROFILE_CUSTOM) {
		zenith_apply_profile(tunables, zenith_cmdline_profile);
		tunables->active_profile = zenith_cmdline_profile;
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

	/* Zero the uclamp cache so zenith_policy_uclamp_{min,max} refresh
	 * on the first eval after start rather than returning stale
	 * zeros cached from a previous attach cycle.
	 */
	z_policy->cached_uclamp_min = 0;
	z_policy->cached_uclamp_max = SCHED_CAPACITY_SCALE;
	z_policy->uclamp_cache_stamp_ns = 0;

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