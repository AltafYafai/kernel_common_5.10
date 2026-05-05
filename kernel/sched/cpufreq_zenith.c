// SPDX-License-Identifier: GPL-2.0
/*
 * Zenith CPUFreq Governor (Zenithed-V4)
 * Originally developed by ENI exclusively for LO.
 * V3+ continued by XTENSEI.
 *
 * Hybrid governor with Energy Model awareness, display-state coupling,
 * thermal-aware throttling, and a self-calibrating multi-layer
 * auto-tune stack:
 *
 *   - V1 classifier      load + input rate; per-profile target every
 *                        ZENITH_AUTO_TUNE_PERIOD_MS
 *   - V2 state machine   per-cluster {efficiency, balanced, latency,
 *                        sustained_perf, thermal_recovery}, scenario
 *                        overlay (camera/render/audio/memstall),
 *                        cluster-aware capping, hysteresis + cooldown
 *   - V3 self-tuner      observes V2 transition rate and bumps
 *                        hysteresis/cooldown offsets to fit live load
 *
 * On top of the auto-tune stack:
 *
 *   - Glides (round/U/Z) frequency-shaping helpers
 *   - K1 migration_floor sticky cluster-arrival floor
 *   - K2 psi_cpu_floor   PSI-CPU-stall floor
 *   - K3 frame_overrun   vblank-driven rescue (drm_handle_vblank
 *                        producer hook in drivers/gpu/drm/drm_vblank.c)
 *   - M1 psi_mem_cap     memory-pressure cap
 *   - M2 uclamp respect  peer_ramp / migration_floor uclamp_min sub-gates
 *   - M3 peer_ramp_off   screen-off peer_ramp window
 *   - M5 K3 deep tier    deep-streak frame_overrun amplification
 *
 * Producer/consumer split:
 *
 *   - Producers: input_handler, drm_handle_vblank, screen_state,
 *     thermal_state, PSI sampler, kcpustat, comm-walk
 *   - Consumers: zenith_get_next_freq() (per-tick) and
 *     zenith_auto_tune_work() (per-classifier-window)
 *
 * Telemetry: 12 trace events under include/trace/events/cpufreq_zenith.h
 * plus the auto_tune_status, at_log, last_decision_path, profile_values,
 * zenith_stats, zenith_input_stats, auto_tune_v3_state, game_auto_state
 * RO sysfs attrs for live diagnosis.
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

/* peak_headroom_rescue (default 1, on):
 *
 * Watchdog tier that rescues the cluster from sustained-high-util
 * starvation.  When the load-based + hispeed pipeline leaves the
 * cluster well below policy->max even though load_pct is pegged
 * (auto-tune profile cap, calibration drift, accumulated down-rate
 * progress, or efficient_freq ladder gating), this tier forces a
 * one-shot up-shift toward policy->max.
 *
 * Trigger condition (both must hold for STARVE_STREAK + 1
 * consecutive samples):
 *
 *   load_pct >= ZENITH_PEAK_HEADROOM_STARVE_LOAD_PCT (default 90)
 *   freq < (policy->max * ZENITH_PEAK_HEADROOM_FREQ_FLOOR_PCT / 100)
 *     (default 85, i.e. cluster freq is below 85%% of policy->max)
 *
 * Rescue action: bump freq up to (policy->max *
 * ZENITH_PEAK_HEADROOM_JUMP_PCT / 100) (default 100, i.e. pin to
 * policy->max) and arm a hold-down deadline so a second rescue
 * cannot fire within ZENITH_PEAK_HEADROOM_HOLD_MS (default 50 ms).
 *
 * Bounded by:
 *   - The streak counter is u8 and saturates at
 *     ZENITH_PEAK_HEADROOM_STREAK_MAX so it cannot wrap on long
 *     sustained runs.
 *   - The downstream caps (uclamp_max, light_cap, audio_cap,
 *     em_cap, PSI cap) all apply after the rescue, so userspace
 *     power hints and the EM validator stay authoritative.
 *   - The rescue is skipped when pin_to_target is already true
 *     (input_boost full-pin / brutality snap_max / brutal_hold);
 *     those paths have already pinned the cluster high.
 *
 * 0 disables the watchdog entirely (legacy behaviour: nothing
 * lifts the cluster off the load-based + hispeed pipeline output).
 */
#define ZENITH_DEFAULT_PEAK_HEADROOM_RESCUE		1
#define ZENITH_DEFAULT_PEAK_HEADROOM_STARVE_LOAD_PCT	90
#define ZENITH_DEFAULT_PEAK_HEADROOM_FREQ_FLOOR_PCT	85
#define ZENITH_DEFAULT_PEAK_HEADROOM_STARVE_STREAK	3
#define ZENITH_DEFAULT_PEAK_HEADROOM_JUMP_PCT		100
#define ZENITH_DEFAULT_PEAK_HEADROOM_HOLD_MS		50
#define ZENITH_DEFAULT_PEAK_HEADROOM_PREARM		1
#define ZENITH_PEAK_HEADROOM_STREAK_MAX			16
#define ZENITH_PEAK_HEADROOM_HOLD_MS_MAX		1000

/* Predictive up-shift via util-trend ring (tier 2a').
 *
 * The peak-headroom rescue (tier 2c) is reactive: it waits for
 * peak_headroom_starve_streak consecutive saturated samples below
 * the freq floor before lifting the cluster, which on a 16 ms
 * sampling cadence is 48..64 ms of starvation before the rescue
 * lands.  The pre-arm tier 2b' shaves one window off the front of
 * that.  This tier shaves two more by acting on a *trend*, not on
 * a level: when the recent util signal is rising fast enough to
 * predict that hispeed_load is about to be crossed, lift the
 * cluster to eff_hispeed_freq one or two ticks before the level-
 * triggered hispeed tier would have done so.
 *
 * Trigger:
 *   delta_x256 = (newest - oldest) over the last predict_up_window
 *                samples, expressed as 256ths of max_cap so the
 *                threshold is unitless.  The compare is
 *                delta_x256 >= predict_up_thresh.
 *   freq < eff_hispeed_freq (otherwise the lift is a no-op).
 *   peak_starve_count == 0 (don't double-fire with rescue / pre-
 *                arm; let those handle it once starvation has
 *                begun).
 *   peak_rescue_until_ns has expired (don't refire inside a
 *                rescue hold-down window).
 *
 * Effect:
 *   Lift freq up to eff_hispeed_freq, set tp_path = "predict_up".
 *   Counter slot ZENITH_STAT_PREDICT_UP records the firing.  The
 *   subsequent hispeed tier (2b) will keep the floor in place if
 *   load_pct actually does cross hispeed_load on the next tick;
 *   otherwise the cluster naturally falls back through the EAS
 *   ladder after the rate-limit window closes.
 *
 * Risk:
 *   Oscillation when the window is short enough to react to PELT
 *   noise.  Mitigated by:
 *     - Gating on peak_starve_count == 0 keeps predict_up from
 *       fighting rescue / pre-arm during sustained-high regimes.
 *     - Default thresh of 64 ( == 25%% of max_cap rise across the
 *       window) is conservative; testers can dial it down to 32
 *       (~12.5%%) for more eager prediction.
 *     - The lift is to eff_hispeed_freq, not policy->max -- the
 *       cluster still has the rescue / brutality tiers above it
 *       when the trend turns out to be a real climb.
 *
 * 0 disables the tier (legacy behaviour: hispeed entry waits for
 * the level signal to arrive).  predict_up_window minimum is 2
 * (need at least two samples to compute a delta) and maximum is
 * ZENITH_PREDICT_UP_WINDOW_MAX so the per-policy ring buffer
 * stays small.
 */
#define ZENITH_DEFAULT_PREDICT_UP_THRESH		64
#define ZENITH_DEFAULT_PREDICT_UP_WINDOW		4
#define ZENITH_PREDICT_UP_THRESH_MAX			255
#define ZENITH_PREDICT_UP_WINDOW_MIN			2
#define ZENITH_PREDICT_UP_WINDOW_MAX			8

/* peak_hysteresis_streak / peak_step_down_pct
 * (defaults 3 / 95, [Stage 4 / Patch E]):
 *
 * Peak-return hysteresis.  When the previous evaluation pinned
 * the cluster at or near peak (freq >=
 * ZENITH_PEAK_HYSTERESIS_PEAK_THRESH_PCT of policy->max) and the
 * current evaluation wants to drop sharply, hold a soft floor
 * at (prev_freq * peak_step_down_pct / 100) for the next
 * peak_hysteresis_streak samples.  After the streak drains, the
 * cluster falls naturally through the lower tiers.
 *
 * Goal: smooth the off-peak descent on bursty workloads.  A
 * render thread that just finished a frame and is now idle
 * waiting for vblank produces a single sample of low load while
 * the next frame is still queued; the natural descent would
 * plunge to EAS-suggested freq and then bounce back up the next
 * sample.  Hysteresis trades a few ms of higher freq for a
 * smoother descent and far fewer freq transitions.
 *
 * Either tunable at 0 disables the tier (legacy).  Range checks
 * keep streak in 0..ZENITH_PEAK_HYSTERESIS_STREAK_MAX and
 * step_down_pct in 0..100.
 */
#define ZENITH_DEFAULT_PEAK_HYSTERESIS_STREAK		3
#define ZENITH_DEFAULT_PEAK_STEP_DOWN_PCT		95
#define ZENITH_PEAK_HYSTERESIS_STREAK_MAX		16
#define ZENITH_PEAK_HYSTERESIS_PEAK_THRESH_PCT		90

/* boost_idle_thresh / boost_idle_streak
 * (defaults 15 / 3, [Stage 4 / Patch F]):
 *
 * Boost early-exit on persistent idle.  When an input boost is
 * armed (now < zenith_input_boost_until_ns) but the cluster has
 * sat below load_pct boost_idle_thresh for boost_idle_streak
 * consecutive ticks, the per-policy tier-0 preempts the boost
 * on this tick instead of pinning to the boost ceiling.
 *
 * Goal: stop pinning the cluster to peak when the workload that
 * triggered the boost has clearly drained.  A user tap that
 * launches an app is a typical case: the launch animation
 * finishes well before input_boost_ms expires, but the natural
 * idle that follows would still see the cluster pinned at the
 * boost ceiling for the rest of the window.  Boost early-exit
 * trims the energy tail without affecting the launch-frame
 * latency the boost was actually for.
 *
 * Per-policy preemption only: the global
 * zenith_input_boost_until_ns is intentionally left armed so
 * other clusters that are still busy keep their boost.  Each
 * policy makes its own idle-streak decision.
 *
 * Either tunable at 0 disables the early-exit (legacy
 * boost-honoured-to-its-deadline behaviour).  Range checks:
 * thresh in 0..100 (load percent), streak in
 * 0..ZENITH_BOOST_IDLE_STREAK_MAX.
 */
#define ZENITH_DEFAULT_BOOST_IDLE_THRESH		15
#define ZENITH_DEFAULT_BOOST_IDLE_STREAK		3
#define ZENITH_BOOST_IDLE_STREAK_MAX			16

/* bg_util_scale_pct
 * (default 100 = off, [Stage 4 / Patch G]):
 *
 * Background-task util scaling.  When the display is off
 * (tunables->screen_state == 0), scale the util signal returned
 * by zenith_get_util() down to bg_util_scale_pct percent of its
 * natural value.  All downstream tiers (brutality, EAS,
 * ladder) see the lower signal, so freq decisions during
 * screen-off run further from policy->max for the same
 * underlying load.
 *
 * Goal: trim energy on background sync / wake-lock work that
 * runs while the device is locked.  These workloads are
 * typically not user-perceivable; running them on a slightly
 * cheaper freq point is a free energy win.
 *
 * Bypassed when screen_state == 1 so display-on responsiveness
 * is unchanged.  100 (default) is a no-op (full util passes
 * through).  0 is rejected by the sysfs store -- the kernel
 * already has cpu-idle paths for the "no work" case; this knob
 * is for *scaling* not for *gating*.  Range checked at
 * 1..100 to avoid that footgun.
 *
 * Reads via READ_ONCE on the eval hot path (zenith_get_util()
 * is called once per CPU per evaluation tick).
 */
#define ZENITH_DEFAULT_BG_UTIL_SCALE_PCT		100
#define ZENITH_BG_UTIL_SCALE_PCT_MIN			1

/* sleeper_tail_thresh_us / sleeper_tail_pct
 * (defaults 0 / 90, [Stage 4 / Patch H]):
 *
 * Sleeper-tail shaving.  When the cluster has been idle (no
 * runnable load_pct samples) for sleeper_tail_thresh_us
 * microseconds, shave the next freq decision down by
 * sleeper_tail_pct / 100, clamped at policy->min.  Saves
 * leakage on sleep entry by parking the cluster one DVFS rung
 * lower than it would otherwise sit on the wake-up tick.
 *
 * thresh_us == 0 (default) disables the tier; sleeper_tail_pct
 * is bounded in 50..100 (anything below 50 would slam the
 * cluster too low on wake and re-up immediately, which is the
 * opposite of what we want).
 *
 * Reads via READ_ONCE on the eval hot path; writes via
 * WRITE_ONCE from sysfs and from zenith_apply_profile().
 */
#define ZENITH_DEFAULT_SLEEPER_TAIL_THRESH_US		0
#define ZENITH_SLEEPER_TAIL_THRESH_US_MAX		100000
#define ZENITH_DEFAULT_SLEEPER_TAIL_PCT			90
#define ZENITH_SLEEPER_TAIL_PCT_MIN			50
#define ZENITH_SLEEPER_TAIL_PCT_MAX			100

/* peer_ramp_window_ms / peer_ramp_floor_pct
 * (defaults 25 / 60, [Stage 4 / Patch D]):
 *
 * Multi-cluster pre-arm coordination.  Cross-cluster IPC chains
 * (binder hops between an app-side BIG worker and a service-side
 * PRIME worker, audio pipelines feeding a render thread, etc.)
 * tend to need both clusters at a usable freq within a few
 * milliseconds of each other.  In the legacy path, when one
 * cluster is woken into peak by predict_up / peak_prearm /
 * peak_rescue, the peer cluster has to climb from idle by
 * itself, paying a full hispeed warm-up window before its half
 * of the IPC chain runs at speed.
 *
 * The arming side: when a cluster lifts to eff_hispeed (or
 * higher) via one of the peak tiers, stamp a deadline on the
 * peer cluster's slot.  The reading side: while that deadline
 * has not expired, the peer applies a soft floor at
 * peer_ramp_floor_pct of policy->max so it is no longer sitting
 * at idle when the cross-cluster wake arrives.  Class-based, not
 * policy-based: BIG arms PRIME and vice versa.  LITTLE does not
 * participate -- it is rarely on the producing side of a peer-
 * ramp-worthy IPC chain, and floor-arming it would interfere
 * with the bg_util_scale_pct screen-off path on devices that
 * route low-priority work to the small cluster.
 *
 * Self-disarms on the deadline.  No streak / debounce: the
 * triggers (predict_up trend window, peak_starve_count >=
 * starve_streak, peak_prearm gate) already require multiple
 * samples worth of evidence, so by the time the peer fires we
 * have all the confirmation we need.  Re-armings just bump the
 * deadline forward; harmless.
 *
 * peer_ramp_window_ms == 0 disables both sides (no arming
 * writes, no floor reads).  peer_ramp_floor_pct == 0 disables
 * just the floor (deadlines still get stamped but never
 * fire) -- mostly useful for trace consumers that want to see
 * the arming events without having the floor influence freq.
 *
 * 100 ms / 100% are the upper bounds.  The 100 ms cap is
 * loose: at peer_ramp_floor_pct=60 the floor only matters when
 * the natural freq would be below 60% of policy->max, which on
 * a real workload is a small fraction of the window.  The
 * 100% cap on the floor is the obvious one (anything higher is
 * just policy->max).
 */
#define ZENITH_DEFAULT_PEER_RAMP_WINDOW_MS		25
#define ZENITH_PEER_RAMP_WINDOW_MS_MAX			100
#define ZENITH_DEFAULT_PEER_RAMP_FLOOR_PCT		60
#define ZENITH_PEER_RAMP_FLOOR_PCT_MAX			100

/* peer_ramp_window_off_ms (default 0, [Stage 5 / Patch M3]):
 *
 * Screen-state-aware override of peer_ramp_window_ms.  When the
 * screen is off the cross-cluster IPC chains peer_ramp exists to
 * accelerate are mostly absent: there is no compositor, no app
 * render thread, no input handler.  Pre-arming a peer cluster in
 * that regime burns idle big-cluster freq for nothing.
 *
 * Same shape as screen_off_glide_ms: when tunables->screen_state
 * is 0 the peer-ramp arming and floor-eval paths use this value
 * in place of peer_ramp_window_ms.  Default 0 means peer_ramp is
 * fully suppressed while the screen is off (no arm writes, no
 * floor reads).  Set equal to peer_ramp_window_ms to restore the
 * pre-Stage-5 always-on behaviour byte-identically.  Range
 * 0..ZENITH_PEER_RAMP_WINDOW_MS_MAX, same upper bound as the
 * screen-on knob since the off variant is just a different value
 * for the same physical timer.
 *
 * Energy-only refinement: cannot raise the peer-ramp floor higher
 * than the screen-on path already does, so this knob can never
 * hurt responsiveness; it can only stop spending energy on a
 * cluster the user is not looking at.
 */
#define ZENITH_DEFAULT_PEER_RAMP_WINDOW_OFF_MS		0

/* migration_jump_pct / migration_floor_window_ms / migration_floor_pct
 * (defaults 20 / 30 / 60, [Stage 4 / Patch K1]):
 *
 * Migration-arrival soft floor.  When a high-util task migrates
 * between CPUs, the source CPU's util drops on the next sample
 * (the task is gone) but the destination's util takes ~32 ms
 * (one PELT half-life) to fully reflect the new load.  In that
 * gap, the destination cluster's eval can pick a freq based on
 * stale-low aggregate util.
 *
 * On every per-CPU update_util tick the governor compares the
 * current util to the value seen on the previous tick.  If the
 * sample-to-sample jump exceeds migration_jump_pct of the CPU's
 * max_capacity, treat it as evidence that a new task just landed
 * here and stamp a deadline on this policy's
 * migration_in_until_ns slot.  While that deadline holds, the
 * eval applies a soft floor at migration_floor_pct of
 * policy->max so the cluster is not running at idle freq for
 * the first half of the new task's PELT warm-up.
 *
 * Per-policy, not class-level: this tracks "task arrived here"
 * regardless of which cluster the task came from.  Composes
 * cleanly with peer_ramp (Patch D) which is the cross-cluster
 * IPC case; peer_ramp arms the *peer* of a ramping cluster,
 * migration_floor arms the *self* of an arrival.  No conflict.
 *
 * Self-disarms by deadline.  Re-armings just bump the deadline
 * forward.  Single-CPU policies (1+1+1 topologies, etc.) work
 * the same way: a task moving onto the only CPU in the policy
 * still triggers a util jump on that CPU.
 *
 * migration_jump_pct == 0 disables both sides (no arming
 * writes, no floor reads).  migration_floor_pct == 0 leaves the
 * stamping in place but suppresses the floor.
 *
 * The default jump threshold is 20%, which is the boundary
 * where empirically (a) PELT half-life dynamics + 4-CPU
 * averaging start producing visible per-CPU spikes from a
 * single task, and (b) routine util oscillation (sched_yield
 * loops, 1-tick-on-1-tick-off micro-bursts) tends to stay
 * below.  Window 30 ms covers most of one PELT half-life.
 */
#define ZENITH_DEFAULT_MIGRATION_JUMP_PCT		20
#define ZENITH_MIGRATION_JUMP_PCT_MAX			100
#define ZENITH_DEFAULT_MIGRATION_FLOOR_WINDOW_MS	30
#define ZENITH_MIGRATION_FLOOR_WINDOW_MS_MAX		100
#define ZENITH_DEFAULT_MIGRATION_FLOOR_PCT		60
#define ZENITH_MIGRATION_FLOOR_PCT_MAX			100

/* psi_cpu_floor_thresh (default 0, off, [Stage 4 / Patch K2]):
 *
 * PSI-CPU-aware sustained-pressure floor.  Mirror of the existing
 * psi_cpu_thresh cap but pointing the other direction.  When
 * zenith_psi_cpu_some_pct() (the 10s-EWMA of system-wide CPU
 * stall %) is at or above psi_cpu_floor_thresh, lift freq to
 * zenith_eff_hispeed_freq().
 *
 * Why this sits next to psi_cpu_thresh and not next to predict_up
 * or peak_rescue: PSI's 10 s smoothing window is too slow for
 * sub-second decisions, so this tier is by design only useful
 * for *sustained* CPU pressure -- gaming + background sync,
 * screen-record + foreground app, multi-app multitasking with a
 * background compile, etc.  Predict_up / peak_rescue / peak_prearm
 * cover the sub-second up-decisions; this tier covers the
 * "we've been queueing for 10+ seconds and util is still under
 * hispeed entry threshold" case where the existing tiers don't
 * fire because aggregate util doesn't capture queueing pressure
 * cleanly.
 *
 * Conservative default (0, off): the user has to opt in.  Set
 * via the per-profile mirror in zenith_apply_profile() so PERF
 * gets a moderate threshold automatically and BAT/LEG keep it
 * disabled.  When set, only fires when zenith_eff_hispeed_freq()
 * is non-zero; if hispeed is unconfigured the tier no-ops rather
 * than fall back to policy->max (which would be too aggressive
 * for what is, after all, a smoothed-pressure signal).
 */
#define ZENITH_DEFAULT_PSI_CPU_FLOOR_THRESH		0
#define ZENITH_PSI_CPU_FLOOR_THRESH_MAX			100

/* frame_overrun_slack_us / frame_overrun_window_ms /
 * frame_overrun_floor_pct (defaults 0 / 50 / 80,
 * [Stage 4 / Patch K3]):
 *
 * Frame-budget overrun rescue.  Companion to the existing
 * frame_pace_floor tier (Stage 3) -- frame_pace arms a floor
 * sized to fit one frame in the budget, this tier corrects
 * after a frame missed.
 *
 * Detection runs in zenith_drm_vblank_event(), a new exported
 * symbol the panel driver / display HAL is expected to call on
 * every vblank.  The first call after governor attach (or after
 * the screen-state stale guard fires) just records the timestamp
 * and returns.  On subsequent calls the elapsed wall-clock delta
 * is compared against (zenith_drm_vblank_us + slack_us); if the
 * gap is wider, the renderer missed the budget.  Stamp a
 * deadline frame_overrun_window_ms in the future on the file-
 * scope zenith_frame_overrun_until_ns slot.  While the deadline
 * holds, every cluster's eval lifts to a soft floor at
 * frame_overrun_floor_pct of policy->max.
 *
 * Why a *new* exported function and not a reuse of
 * zenith_set_drm_vblank_us(): the existing setter takes the
 * vblank *period* in microseconds and is only called on
 * refresh-rate transitions (60 Hz <-> 120 Hz).  This patch needs
 * a *per-vblank* event, which is a different concept.  Decoupled
 * so a panel driver can wire up either, both, or neither
 * depending on what it knows.  When the panel driver does not
 * call zenith_drm_vblank_event(), the entire detection path is
 * a no-op (the deadline is never stamped) and the rest of the
 * governor behaves exactly as before -- same fail-safe shape as
 * the existing zenith_set_drm_vblank_us() path.
 *
 * frame_overrun_slack_us == 0 disables stamping (the producer
 * still updates last_vblank_ns so the next 0 -> non-zero
 * configuration change starts cleanly).
 * frame_overrun_floor_pct == 0 leaves stamping but suppresses
 * the floor.
 *
 * Cold-boot default for slack_us is 0 (off).  Per-profile
 * values turn it on: BALANCED uses 4000 us, which is roughly a
 * third of a 60 Hz vblank period (16667 us).  Smaller and
 * routine driver / scheduler jitter trips the detector; larger
 * and an actual single missed frame (16667 us late) doesn't
 * trip it.  Window 50 ms covers about three 60 Hz frames or six
 * 120 Hz frames -- enough recovery time after a single miss
 * without holding a high-freq pin past a brief stall.
 */
#define ZENITH_DEFAULT_FRAME_OVERRUN_SLACK_US		0
#define ZENITH_FRAME_OVERRUN_SLACK_US_MAX		16667
#define ZENITH_DEFAULT_FRAME_OVERRUN_WINDOW_MS		50
#define ZENITH_FRAME_OVERRUN_WINDOW_MS_MAX		200
#define ZENITH_DEFAULT_FRAME_OVERRUN_FLOOR_PCT		80
#define ZENITH_FRAME_OVERRUN_FLOOR_PCT_MAX		100

/* frame_overrun_deep_streak / frame_overrun_deep_floor_pct
 * (defaults 0 / 100, [Stage 5 / Patch M5]):
 *
 * Sub-knob inside K3.  A single overrun is plausibly a one-shot
 * scheduler / GC / page-fault hiccup; the standard K3 floor at
 * frame_overrun_floor_pct (typ. 80%) is sized for that case.  N
 * consecutive overruns at the same panel period is qualitatively
 * different -- something is sustained-overloaded -- and the
 * recovery floor should escalate.
 *
 * Implementation: the producer (zenith_drm_vblank_event())
 * already detects per-vblank overruns and stamps a deadline.
 * Track a governor-wide consecutive-overrun streak: bump on
 * each overrun, reset on each within-budget vblank.  In the
 * consumer (the K3 read site in zenith_get_next_freq()), once
 * the streak crosses frame_overrun_deep_streak the floor lifts
 * from frame_overrun_floor_pct to frame_overrun_deep_floor_pct.
 *
 * frame_overrun_deep_streak == 0 disables the deep tier
 * entirely (the consumer never reads the streak atomic).  The
 * default of 0 keeps Stage 4 K3 byte-identical for users who
 * don't opt in.  PERFORMANCE profile arms it at 2 / 100% --
 * two consecutive misses at 60 Hz is ~33 ms of stutter, well
 * outside any plausible jitter explanation, and the user has
 * already opted into the energy / responsiveness trade by
 * picking PERFORMANCE.  All other profiles ship at 0.
 *
 * Capped at 16 to keep streak overflow a non-issue (atomic_t
 * is 31 bits but practically the read-site comparison only
 * cares about reaching the threshold; once past, the streak
 * keeps bumping until reset and the comparison stays true).
 * deep_floor_pct is capped at 100 (anything higher is just
 * policy->max again) and lower-bounded by the read site at
 * the existing frame_overrun_floor_pct -- the deep tier never
 * produces a *lower* floor than the standard K3 floor.
 *
 * Sits inside K3's V2 arming gate: if V2 has disarmed K3 for
 * this state (eff_floor_pct == 0), the deep tier is also
 * inactive because the read-site short-circuit fires before
 * the streak comparison.  No new V2 tier bit; this is an
 * amplification of K3, not a separate tier.
 */
#define ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_STREAK	0
#define ZENITH_FRAME_OVERRUN_DEEP_STREAK_MAX		16
#define ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_FLOOR_PCT	100
#define ZENITH_FRAME_OVERRUN_DEEP_FLOOR_PCT_MAX	100

/* peer_ramp_uclamp_min_respect / migration_floor_uclamp_min_respect
 * (defaults 1 / 1, [Stage 5 / Patch M2]):
 *
 * Per-tier sub-gates that let the peer_ramp (Patch D) and
 * migration_floor (Patch K1) floors respect a task's
 * uclamp_min hint.  Independent of the existing
 * uclamp_min_respect master gate (which controls the
 * *final-freq* floor at line ~6811): these gates only affect
 * the per-tier intermediate floors.
 *
 * Effective floor at each site becomes:
 *   max(static_floor_pct, uclamp_min_as_pct_of_max)
 *
 * When the inbound / on-policy task has uclamp_min == 0 (the
 * common case) the max() is a no-op and behaviour is byte-
 * identical to Stage 4.  When the task has an explicit ADPF-
 * driven uclamp_min, the new path lifts the floor *up* toward
 * what the scheduler already owes the task -- this can never
 * produce a *lower* floor than the static knob alone.
 *
 * Why two bools and not one: peer_ramp and migration_floor
 * are independently configurable in the existing per-profile
 * presets; some profiles arm one without the other (e.g.
 * BATTERY disables migration_floor but a future profile might
 * keep peer_ramp at non-zero).  Two bools keep the matrix
 * clean.
 *
 * Default 1 because the uclamp-respecting path is strictly a
 * floor-raise, never a floor-lower.  Set 0 to revert to the
 * pre-Stage-5 byte-identical behaviour for that specific tier.
 */
#define ZENITH_DEFAULT_PEER_RAMP_UCLAMP_MIN_RESPECT		1
#define ZENITH_DEFAULT_MIGRATION_FLOOR_UCLAMP_MIN_RESPECT	1

/* psi_mem_cap_thresh / psi_mem_cap_pct / psi_mem_cap_window_ms
 * (defaults 0 / 80 / 1000, [Stage 5 / Patch M1]):
 *
 * Symmetric companion to the existing psi_mem_thresh predicate.
 * The decision chain already gates the *up-push* on memstall via
 * zenith_psi_mem_some_pct() >= psi_mem_thresh.  But the cap side
 * (uclamp_max / light_cap / audio_cap / em_cap / existing
 * psi-cap-at-hispeed) doesn't pull the *final* freq down when
 * memstall climbs past a separate, lower threshold.  Pushing CPU
 * into peak under heavy paging just deepens the stall: the CPU
 * has nothing useful to do while it waits on mm.  This tier adds
 * a final-freq cap that fires exactly there.
 *
 * Knob shape:
 *   psi_mem_cap_thresh    (0..100, default 0 = off)
 *   psi_mem_cap_pct       (50..100, default 80%)
 *   psi_mem_cap_window_ms (100..5000, default 1000 ms)
 *
 * When psi_aware == 1 (master gate) AND psi_mem_cap_thresh > 0
 * AND zenith_psi_mem_some_pct() >= psi_mem_cap_thresh, the eval
 * path stamps z_policy->psi_mem_cap_until_ns with a deadline
 * `now + psi_mem_cap_window_ms`.  While that deadline has not
 * expired, the final freq is capped at psi_mem_cap_pct of
 * policy->max.  Once the EWMA falls below thresh and the window
 * lapses, the cap releases without further hysteresis.
 *
 * Defaults:
 *   - psi_mem_cap_thresh = 0 (off) so the patch is fully inert
 *     out of the box.  Users who opt into psi_aware = 1 already
 *     accept the existing PSI cap above; this tier is a stricter
 *     opt-in.
 *   - psi_mem_cap_pct = 80% so the cap is meaningful but not
 *     punitive (a healthy floor for browser / scrolling under
 *     mild memstall).  PERFORMANCE bumps to 90% for the user
 *     who has explicitly picked PERF.
 *   - psi_mem_cap_window_ms = 1000 ms so the cap stays in place
 *     long enough to absorb a full mmap_sem / kswapd burst
 *     without flapping every tick.  Aligned with the 10s EWMA
 *     timescale: the EWMA's natural reset-to-zero is glacial,
 *     so the window is what governs cap release.
 *
 * Range max (5000 ms) caps the worst-case stuck-cap to 5 s, the
 * order of magnitude where the user would notice "phone is slow"
 * regardless of governor reasoning.
 *
 * V2 tier mapping:
 *   ZENITH_AT_STATE_EFFICIENCY   -> arm  (back off on stall is
 *                                  exactly the EFFICIENCY job)
 *   ZENITH_AT_STATE_BALANCED     -> arm
 *   ZENITH_AT_STATE_THERMAL_RECOVERY -> arm
 *   ZENITH_AT_STATE_LATENCY      -> disarm (this is a cap, not
 *                                  a floor; LATENCY does not
 *                                  want any extra caps)
 *   ZENITH_AT_STATE_SUSTAINED_PERF -> disarm (user has explicitly
 *                                    asked for top-end)
 *   FRAME / GAME flag bypass     -> disarm (frame-pacing and game
 *                                  overrides need full headroom)
 *
 * Cannot regress at default: psi_mem_cap_thresh == 0 short-
 * circuits before any read of the EWMA; psi_aware == 0 short-
 * circuits a level higher; auto_tune_v2_tiers == 0 bypasses the
 * tier bit entirely.  When the tier *is* armed and fires, the
 * cap is bounded below by policy->min and above by policy->max,
 * so a misconfigured psi_mem_cap_pct cannot drop the policy
 * below its natural floor.
 */
#define ZENITH_DEFAULT_PSI_MEM_CAP_THRESH		0
#define ZENITH_PSI_MEM_CAP_THRESH_MAX			100
#define ZENITH_DEFAULT_PSI_MEM_CAP_PCT			80
#define ZENITH_PSI_MEM_CAP_PCT_MIN			50
#define ZENITH_PSI_MEM_CAP_PCT_MAX			100
#define ZENITH_DEFAULT_PSI_MEM_CAP_WINDOW_MS		1000
#define ZENITH_PSI_MEM_CAP_WINDOW_MS_MIN		100
#define ZENITH_PSI_MEM_CAP_WINDOW_MS_MAX		5000

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

/* M2 V2 state-transition history ring depth.  32 entries = 512 B per
 * policy (16 B / entry, see struct zenith_policy::at_history).  Sized
 * to span ~30 s of typical phone-class bursty workloads (camera open,
 * scroll, app launch) so a userspace triager reading the file can see
 * the last interesting cluster of state changes without round-tripping
 * to dmesg / perfetto.
 */
#define ZENITH_AT_HISTORY_NR			32

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
 * Init-time invariant:
 *
 *   - zenith_camera_aware_key and zenith_psi_aware_key match scalars
 *     that still default to 0, so they correctly start FALSE without
 *     any explicit init-time enable.
 *   - zenith_audio_aware_key and zenith_render_aware_key match scalars
 *     that were flipped to default 1 in the wave-2 auto-defaults
 *     round, so the keys must be explicitly enabled in zenith_init()
 *     after the tunable defaults are written, otherwise the hot path
 *     would read the scalar as 1 but skip the branch via the still-FALSE
 *     key.  zenith_init() now calls zenith_set_static_key() against
 *     each scalar's value (idempotent across re-attaches).
 *   - zenith_game_auto_key and zenith_auto_tune_v3_key match scalars
 *     that were flipped to non-zero defaults in the wave-7
 *     auto-defaults round (game_auto = 1, auto_tune_v3 = 2), so the
 *     same init-time sync rule applies: zenith_init() must enable
 *     the key against the default scalar value.  The auto_tune_v3
 *     key is binary even though the scalar is tri-valued (0/1/2);
 *     zenith_set_static_key() treats any non-zero value as TRUE, so
 *     observe-only mode (scalar = 1) and apply mode (scalar = 2)
 *     both produce key = TRUE.  See ZENITH_DEFAULT_AUTO_TUNE_V3
 *     comment block.
 *   - zenith_set_profile_defaults() never touches any of the six
 *     scalars (they are user-managed opt-ins, not preset state), so
 *     no profile-apply path needs to re-sync the keys.
 */
DEFINE_STATIC_KEY_FALSE(zenith_audio_aware_key);
DEFINE_STATIC_KEY_FALSE(zenith_camera_aware_key);
DEFINE_STATIC_KEY_FALSE(zenith_render_aware_key);
DEFINE_STATIC_KEY_FALSE(zenith_psi_aware_key);
DEFINE_STATIC_KEY_FALSE(zenith_game_auto_key);
DEFINE_STATIC_KEY_FALSE(zenith_auto_tune_v3_key);

/* Transition invariant for the six feature static keys above:
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
 *   - The init state of every DEFINE_STATIC_KEY_FALSE is FALSE.  If
 *     the matching scalar defaults to a non-zero value, the key must
 *     be explicitly enabled at init time (after tunables defaults are
 *     written), otherwise the hot path will read the scalar as 1 but
 *     skip the static-branch body.  zenith_init() syncs the
 *     audio_aware / render_aware keys against their default scalars
 *     for exactly this reason.
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
/*
 * thermal_pressure_continuous default flipped from 0 to 1 in the
 * wave-2 auto-defaults round.  The legacy hard-cliff path snapped
 * dynamic_up_thresh to 90% the moment thermal_state turned on; the
 * continuous path linearly ramps from up_threshold (at 0% pressure)
 * to 90% (at 100% pressure) using the same arch_scale_thermal_pressure
 * percentage that V2 consumes.  No KMI exposure; the runtime path is
 * gated on thermal_auto and a non-zero pressure reading.
 */
#define ZENITH_DEFAULT_THERMAL_PRESSURE_CONTINUOUS	1

/* prefer_silver_aware defaults.  See struct zenith_tunables for
 * semantics.  Hot threshold of 50%% means the bump fires when at
 * least half of the recent prefer_silver decisions actually
 * redirected onto a silver core; hot bump of 5 points is small
 * enough to avoid a perceived step but large enough to noticeably
 * delay big-cluster downclock during sustained UI navigation.
 * Both knobs are tunable; the defaults are conservative.
 *
 * Default flipped from 0 to 1 in the wave-1 auto-defaults round so
 * the prefer_silver coordination kicks in out of the box on builds
 * that have CONFIG_SCHED_PREFER_SILVER=y.  The bump is cluster-aware
 * (only big/prime clusters react) and gated on the silver-cpu hit
 * rate exceeding prefer_silver_hot_threshold_pct, so on devices
 * without prefer_silver, the runtime path is a no-op.
 */
#define ZENITH_DEFAULT_PREFER_SILVER_AWARE			1
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

/* wakeup_boost_ms upper bound.  200ms is past the perceptible
 * threshold for a wakeup transition; longer windows just bleed
 * into the steady-state climb logic and waste battery.
 */
#define ZENITH_DEFAULT_WAKEUP_BOOST_MS		0
#define ZENITH_WAKEUP_BOOST_MS_MAX		200

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

/* Multiplier for the effective down-rate delay while an input
 * boost is active.  Range 100..1000 (percent).  100 = no extension
 * (legacy behaviour); 200 (the default) = double the down-rate
 * delay during the input_boost full-pin window; 500 = quintuple.
 *
 * Rationale: within the input_boost_until_ns window the cluster is
 * pinned high precisely because the user is actively interacting.
 * Letting the down-rate gate fire at its normal cadence inside
 * that window pulls the cluster off peak the moment a sample's
 * load proportional math drops below the previous freq -- which
 * happens between every render tick and the next on a typical
 * scroll / swipe -- producing the "post-tap cliff drop" pattern
 * users describe as stutter even though the user is still
 * interacting.  Multiplying down_rate_delay during the boost
 * window holds the cluster up across the gaps without changing
 * any other tier.
 *
 * Self-disarms: once now >= input_boost_until_ns the multiplier
 * disappears on the very next call into zenith_up_down_rate_limit,
 * so the steady-state idle path is unaffected.  Capped at 1000%%
 * (10x) on store so a runaway value can not effectively pin the
 * cluster forever.
 */
#define ZENITH_DEFAULT_INPUT_BOOST_DOWN_RATE_MULT_PCT	200
#define ZENITH_INPUT_BOOST_DOWN_RATE_MULT_PCT_MAX	1000
#define ZENITH_DEFAULT_POWERSAVE_BIAS		0

/* Screen-on softening of powersave_bias.  Default 50: halve the
 * effective bias whenever the screen is on (tunables->screen_state
 * == 1), leaving the configured value in full effect on screen-off
 * and on the thermal-active path.  Rationale: powersave_bias
 * values inherited from the BALANCED (50, i.e. 5 %% shave) and
 * BATTERY (150, 15 %% shave) profiles are tuned for the average
 * over screen-on + screen-off duty cycles, but the screen-on path
 * itself is the single most responsiveness-sensitive window the
 * cluster has.  Halving the screen-on shave keeps the configured
 * profile's screen-off behaviour intact (where the existing 500 /
 * 50 %% screen-off override already dominates), while removing
 * about half of the steady-state shave that's been quietly
 * suppressing the cluster's peak ramp during interactive use.
 *
 * Range 0..100 (percent).  100 = no softening (legacy behaviour);
 * 50 = halve; 0 = zero out the bias entirely on screen-on.
 */
#define ZENITH_DEFAULT_SCREEN_ON_BIAS_PCT	50
#define ZENITH_DEFAULT_IO_IS_BUSY		1
#define ZENITH_DEFAULT_INPUT_BOOST_MS		80
#define ZENITH_DEFAULT_INPUT_BOOST_DECAY_MS	30

/* input_boost_touchdown_extra_ms (default 50, [Stage 4 / Patch C]):
 *
 * Touchdown vs coordinate-stream differentiation.  An EV_KEY/
 * BTN_TOUCH press (touchdown) is the user's "I just started
 * interacting" signal: latency from touchdown to first frame is
 * the visible feel of the device.  An EV_ABS coordinate stream
 * mid-gesture is "I'm already interacting" -- the cluster should
 * already be on a high tier from the touchdown that started the
 * gesture, so a per-coordinate widen-the-window effort is wasted
 * energy.
 *
 * This knob extends the input-boost active window by an extra
 * input_boost_touchdown_extra_ms milliseconds *only* on the
 * touchdown event.  Coordinate-stream EV_ABS events use the
 * unmodified input_boost_ms window plus the existing quiet-period
 * extension (see ZENITH_INPUT_QUIET_BOOST_MULT_PCT).
 *
 * 0 disables the touchdown extra entirely (legacy behaviour:
 * touchdown gets the same window as a coordinate event).  Capped
 * at ZENITH_INPUT_BOOST_TOUCHDOWN_EXTRA_MS_MAX so a runaway echo
 * can't accidentally pin the cluster up for seconds.
 */
#define ZENITH_DEFAULT_INPUT_BOOST_TOUCHDOWN_EXTRA_MS	50
#define ZENITH_INPUT_BOOST_TOUCHDOWN_EXTRA_MS_MAX	500

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

/* Input-boost quiet-period extension (always-on, compile-time
 * constants).  When zenith_input_event observes that the gap since
 * the previous input event exceeds ZENITH_INPUT_QUIET_THRESHOLD_MS,
 * the next event's full-pin window is widened by
 * ZENITH_INPUT_QUIET_BOOST_MULT_PCT (>= 100) and clamped at
 * ZENITH_INPUT_QUIET_BOOST_MAX_MS so a misconfigured input_boost_ms
 * cannot grow the window without bound.
 *
 * Rationale: the very first tap / key / scroll after the user has
 * been idle (reading, watching a static frame) is the single
 * highest-leverage responsiveness window the governor has.  By the
 * second tap of a sustained interaction the cluster is already
 * pinned by either input_boost or the load-driven hispeed tier; the
 * marginal value of an extra-long boost on each subsequent tap is
 * small.  Extending only the first-after-quiet event keeps the
 * average power impact tiny while making the "device feels slow when
 * I pick it up" failure mode go away.
 *
 * QUIET_THRESHOLD_MS = 1000: anything shorter than this and the
 * existing input_boost_ms / input_boost_decay_ms windows already
 * cover the gap.  At 1 second of no input, even an input_boost_ms
 * of 200 ms (the maximum sane value) has fully decayed and the
 * cluster is back on natural shaping, so the next event genuinely
 * is a "wake-from-quiet" event.
 *
 * MULT_PCT = 200: doubles the active phase.  Conservative; could be
 * higher but doubling hits a clear "whole-frame" extra (16 ms at
 * 60 Hz on top of an 80 ms default = ~6 frames worth) without
 * spending energy beyond the natural decay tail.
 *
 * MAX_MS = 250: hard ceiling.  Guards against the extension scaling
 * an input_boost_ms that has been raised by userspace beyond a sane
 * range.  Past 250 ms the input_boost_decay_ms window dominates the
 * energy bill anyway, so capping the extension here costs nothing.
 */
#define ZENITH_INPUT_QUIET_THRESHOLD_MS		1000
#define ZENITH_INPUT_QUIET_BOOST_MULT_PCT	200
#define ZENITH_INPUT_QUIET_BOOST_MAX_MS		250

/* ZENITH_DEFAULT_INPUT_BOOST_CAP_PCT controls the ceiling of the
 * full-pin phase of an active input boost: the first input_boost_ms
 * after a key / touch event.  0 means "no cap" -- pin to
 * policy->max for the duration of the full-pin phase, then decay
 * across input_boost_decay_ms back to policy->min.  Tester reports
 * of "device runs cold and never reaches peak frequency under
 * sustained interactive load (gameplay touch, fast scrolling)" trace
 * back to a non-zero cap eating the top of the cluster's range
 * during the very window where the user is actively asking for it.
 *
 * Default 0 (no cap, pin to policy->max).  Userspace setpoints, the
 * BALANCED / BATTERY profiles, and per-policy local overrides via
 * profile_values can still cap the ceiling lower for power-sensitive
 * configurations.  The full-pin phase is short (default 80 ms) and
 * the trailing decay phase is shorter still (default 30 ms), so
 * "pin to policy->max on every input event" is bounded in time and
 * downstream caps (uclamp_max, audio_cap, em_cap, light_cap,
 * thermal_state) all apply on top.
 */
#define ZENITH_DEFAULT_INPUT_BOOST_CAP_PCT	0
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
 *
 * Default flipped from 0 to 1 in the wave-1 auto-defaults round so
 * V1-only builds (auto_tune_v2=0) also benefit from scenario-aware
 * profile selection.  Floor/cap behaviour is still off by default
 * because audio_floor_pct, render_floor_pct and camera_floor_pct
 * stay at 0 -- only the profile-bias path is enabled.
 */
#define ZENITH_DEFAULT_AUTO_TUNE_SCENARIO	1

/* auto_tune_v2 safety layer (default 1, on):
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
 *
 * Default flipped from 0 to 1 in the wave-1 auto-defaults round.  V2
 * only adjusts knobs whose user-set value is the per-knob default;
 * any operator who has pinned a value via sysfs continues to win
 * outright.  hysteresis_windows + cooldown_windows + override_mask
 * remain in place to bound state-thrash.  Set the knob back to 0 in
 * init.zenith.rc to lock the legacy classifier path.
 */
#define ZENITH_DEFAULT_AUTO_TUNE_V2		1
#define ZENITH_DEFAULT_AT_HYSTERESIS_WINDOWS	2
#define ZENITH_DEFAULT_AT_COOLDOWN_WINDOWS	1
#define ZENITH_AT_HYSTERESIS_WINDOWS_MAX	8
#define ZENITH_AT_COOLDOWN_WINDOWS_MAX		8

/* Variance-promotion threshold (load_var_ewma_x256, units of 1/256
 * of the squared-deviation EWMA reported by the V1 classifier).
 * When the ewma is at-or-above this value AND the V2 state machine
 * is currently in LATENCY, V2 promotes to SUSTAINED_PERF on the
 * variance signal alone (reason=variance) -- the rationale is that
 * an oscillating workload spends too much time crossing thresholds
 * for LATENCY's per-tick reaction; SUSTAINED_PERF holds the freq
 * up across the bursts.
 *
 * 768 was chosen empirically as a good knee for phone workloads:
 * background music (steady) sits ~200, app launch / scroll
 * (medium burst) ~500, camera viewfinder / 3D ~900-1500.  Lower
 * values promote eagerly and risk holding sustained_perf longer
 * than necessary; higher values delay promotion and risk under-
 * frequency on bursty workloads.
 *
 * Tunable from userspace via auto_tune_v2_var_promote_thresh sysfs;
 * 0 disables variance-driven promotion entirely.
 */
#define ZENITH_DEFAULT_AT_V2_VAR_PROMOTE_THRESH	768
#define ZENITH_AT_V2_VAR_PROMOTE_THRESH_MAX	65535U

/* auto_tune_v3 (default 2, apply):
 *
 * Self-calibrating layer on top of V2.  Reads the per-policy at_log
 * ring (ZENITH_AT_LOG_NR entries, each one V1-window wide) once per
 * ZENITH_AT_V3_INTERVAL_NS and counts V2 state transitions inside
 * the window.  Based on observed transition rate, V3 maintains
 * bounded signed offsets to two V2 reaction knobs:
 *
 *   - auto_tune_hysteresis_windows  (ZENITH_DEFAULT_AT_HYSTERESIS_WINDOWS = 2)
 *   - auto_tune_cooldown_windows    (ZENITH_DEFAULT_AT_COOLDOWN_WINDOWS  = 1)
 *
 * If V2 was observed thrashing (transitions >= ZENITH_AT_V3_THRASH_HI),
 * the offsets bump up by one (more hysteresis, slower reaction).  If
 * V2 was observed sticky (transitions <= ZENITH_AT_V3_THRASH_LO), the
 * offsets bump down by one (less hysteresis, faster reaction).  Either
 * way the offset is clamped to [ZENITH_AT_V3_OFFSET_MIN ..
 * ZENITH_AT_V3_OFFSET_MAX] and the resulting eff_value is clamped to
 * the existing ZENITH_AT_HYSTERESIS_WINDOWS_MAX / _COOLDOWN_WINDOWS_MAX
 * caps and to a >=1 floor.
 *
 * Three modes via the auto_tune_v3 scalar:
 *
 *   - 0  off                  -- no observation, no adjustments.
 *   - 1  observe-only         -- collects stats, exposes them via
 *                                auto_tune_v3_state, does NOT apply
 *                                offsets.  Equivalent to a dry-run.
 *   - 2  observe + apply      -- collects stats AND applies the
 *                                bounded offsets to the V2 reaction
 *                                knobs.  This is the wave-7 default.
 *
 * Default 2 (apply, wave-7 round): once per
 * auto_tune_v3_interval_ms (default 60 s) the calibrator nudges
 * V2's effective hysteresis/cooldown windows toward whatever fits
 * the live workload.  Bounded offsets ([-1, +4]) and a >=1 floor on
 * the resulting effective value mean V3 cannot push V2 into a
 * state-change-impossible configuration even at the worst extreme.
 * The scalar is gated by the zenith_auto_tune_v3_key static branch
 * (FALSE while scalar = 0) so the calibration tail in
 * zenith_auto_tune_work() collapses to a single never-taken jump
 * per V1 window when an operator turns the feature off.
 *
 * Init-time invariant: the static key zenith_auto_tune_v3_key must
 * be synced TRUE in zenith_init() against the default-non-zero
 * scalar (same pattern as wave-2 audio_aware / render_aware).  See
 * the DEFINE_STATIC_KEY_FALSE comment block.
 *
 * Tunable surface:
 *   - auto_tune_v3              RW 0/1/2  master gate / mode
 *   - auto_tune_v3_state        RO        snapshot of current
 *                                         observed transitions/window
 *                                         and the two live offsets
 *   - auto_tune_v3_interval_ms  RW        calibration period (default
 *                                         60000, clamped to [10000,
 *                                         600000])
 */
#define ZENITH_DEFAULT_AUTO_TUNE_V3		2
#define ZENITH_AT_V3_MODE_OFF			0
#define ZENITH_AT_V3_MODE_OBSERVE		1
#define ZENITH_AT_V3_MODE_APPLY			2
#define ZENITH_AT_V3_MODE_MAX			2

#define ZENITH_DEFAULT_AT_V3_INTERVAL_MS	60000
#define ZENITH_AT_V3_INTERVAL_MIN_MS		10000
#define ZENITH_AT_V3_INTERVAL_MAX_MS		600000

/* Transition-rate thresholds.  Counted within the ZENITH_AT_LOG_NR
 * (=16) window which spans up to ~16 V1 cycles (~160 s with the
 * default V1 cadence).  HI/LO are absolute counts, not rates per
 * unit time -- the calibration cadence is stable enough that
 * counts work fine.
 */
#define ZENITH_AT_V3_THRASH_HI			6
#define ZENITH_AT_V3_THRASH_LO			1

/* Bounded signed offset range for the two V2 reaction knobs. */
#define ZENITH_AT_V3_OFFSET_MIN			(-1)
#define ZENITH_AT_V3_OFFSET_MAX			(+4)

#define ZENITH_DEFAULT_AT_CLUSTER_AWARE		1
#define ZENITH_DEFAULT_AT_V2_SIGNALS		1
#define ZENITH_DEFAULT_AT_THERMAL_SLOPE		1
#define ZENITH_DEFAULT_AT_THERMAL_PRESSURE_PCT	18
#define ZENITH_DEFAULT_AT_THERMAL_SLOPE_PCT	4
#define ZENITH_DEFAULT_AT_FRAME_PACING		1
#define ZENITH_DEFAULT_AT_SUSTAINED_GAMING	1
#define ZENITH_AT_THERMAL_PCT_MAX		100

/* auto_tune_v2_glides (default 1, on):
 *
 * Master gate for V2-driven population of the round-U-z10 "glide" /
 * coordination knobs:
 *
 *   brutal_decay_ms, wakeup_boost_ms, boot_boost_decay_ms,
 *   screen_off_glide_ms, thermal_pressure_continuous,
 *   prefer_silver_aware, frame_budget_us_auto.
 *
 * On stock systems all seven knobs default to 0 (legacy hard
 * cliffs / off).  Without auto_tune_v2_glides the consumer has to
 * hand-tune each one via sysfs to opt into the new behaviour.
 *
 * When auto_tune_v2_glides is 1 (default), zenith_auto_tune_work()
 * populates per-policy "effective" copies of these knobs based on
 * the current V2 state and signal flags; consumers fall back to the
 * effective copy whenever the user-set tunable is 0.  Writes a
 * non-zero value to any of the seven tunables continue to win
 * outright -- the V2-derived copy is only consulted on the 0
 * (default) value.  Set auto_tune_v2_glides to 0 to lock all seven
 * back to byte-identical legacy behaviour at the cost of having to
 * hand-tune anything you want enabled.
 *
 * Costs nothing on auto_tune_v2=0 systems: zenith_at_apply_glides()
 * is only called from the V2 worker path.
 */
#define ZENITH_DEFAULT_AUTO_TUNE_V2_GLIDES	1

/* auto_tune_v2_tiers (default 1, on) -- Patch L:
 *
 * Sibling of auto_tune_v2_glides for the Stage 4 K1 / K2 / K3
 * floor tiers.  Same pattern, different consumer set:
 *
 *   K1 -> migration_jump_pct, migration_floor_window_ms,
 *         migration_floor_pct
 *   K2 -> psi_cpu_floor_thresh
 *   K3 -> frame_overrun_slack_us, frame_overrun_window_ms,
 *         frame_overrun_floor_pct
 *
 * Difference from glides: these are *floor / lift* knobs, not
 * shape knobs.  Cold-boot defaults are non-zero on some profiles
 * (e.g. PERFORMANCE arms migration with jump=15) so the glide
 * accessor's "user value wins if non-zero" rule would let the
 * profile-set value defeat V2 every time.  Patch L instead uses
 * an *armed-mask* model: the V2 worker writes a bitmask of which
 * tiers should be active for the current state / flag set, and
 * the read site returns the profile-set tunable value if armed,
 * 0 (off) if disarmed.  User sysfs writes set per-knob bits in
 * tunables->auto_tune_override_mask which lock the read to the
 * tunable value regardless of V2.
 *
 * State / flag -> armed tiers mapping (also documented inline at
 * the ZENITH_AT_TIER_* defines):
 *
 *   LATENCY              -> migration + psi_cpu_floor
 *   FRAME flag           -> migration + frame_overrun
 *   GAME flag            -> migration + frame_overrun
 *   anything else        -> all disarmed
 *
 * Set auto_tune_v2_tiers to 0 to lock all three back to pure
 * profile-driven behaviour (the post-K3, pre-Patch-L state).
 *
 * Costs nothing on auto_tune_v2=0 systems: zenith_at_apply_tiers()
 * is only called from the V2 worker path.
 */
#define ZENITH_DEFAULT_AUTO_TUNE_V2_TIERS	1

/* Effective values applied by zenith_at_apply_glides() per state.
 * Picked to match the round-U-z10 doc recommendations and keep all
 * seven knobs inside their documented sysfs ranges.
 */
#define ZENITH_AT_GLIDE_BRUTAL_DECAY_MS		150
#define ZENITH_AT_GLIDE_WAKEUP_BOOST_MS		50
#define ZENITH_AT_GLIDE_BOOT_BOOST_DECAY_MS	5000
#define ZENITH_AT_GLIDE_SCREEN_OFF_MS		300
#define ZENITH_AT_GLIDE_THERMAL_PRESSURE_PCT	25

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

/* Patch L: V2-classifier "tier" override bits.
 *
 * The Stage 4 K1 / K2 / K3 floor tiers are profile-driven knobs --
 * the user picks PERFORMANCE / BALANCED / BATTERY / LEGACY and
 * zenith_apply_profile() writes the per-knob value.  Patch L lets
 * the V2 state classifier *additionally* arm or disarm those
 * tiers per-state (e.g. arm migration_floor in LATENCY, disarm it
 * in EFFICIENCY) without disturbing the profile-set values.
 *
 * Each bit, when set in tunables->auto_tune_override_mask, locks
 * the matching knob to whatever the user wrote via sysfs -- the
 * V2 worker stops touching it.  Profile changes clear the entire
 * mask in zenith_apply_profile() (existing behaviour), so a
 * profile flip rearms V2 as if the user had never overridden.
 *
 * (Bits 0..9 are the existing V2 actions overrides; bits 10..16
 * are the new tier overrides.)
 */
#define ZENITH_AT_OVERRIDE_MIGRATION_JUMP	BIT(10)
#define ZENITH_AT_OVERRIDE_MIGRATION_FLOOR_WIN	BIT(11)
#define ZENITH_AT_OVERRIDE_MIGRATION_FLOOR_PCT	BIT(12)
#define ZENITH_AT_OVERRIDE_PSI_CPU_FLOOR	BIT(13)
#define ZENITH_AT_OVERRIDE_FRAME_OVR_SLACK	BIT(14)
#define ZENITH_AT_OVERRIDE_FRAME_OVR_WINDOW	BIT(15)
#define ZENITH_AT_OVERRIDE_FRAME_OVR_FLOOR	BIT(16)

/* Patch M1: PSI-mem light cap.  Three new override bits, sitting
 * at the end of the existing tier-overrides bank.  Same shape as
 * the K1/K2/K3 override bits above: a sysfs write to any of the
 * three knobs ORs in its bit, and the V2 worker stops touching
 * that specific knob until the next profile flip clears the
 * mask.  Profile-flip-clears-mask is implemented in
 * zenith_apply_profile() (existing code, no edit needed here).
 */
#define ZENITH_AT_OVERRIDE_PSI_MEM_CAP_THRESH	BIT(17)
#define ZENITH_AT_OVERRIDE_PSI_MEM_CAP_PCT	BIT(18)
#define ZENITH_AT_OVERRIDE_PSI_MEM_CAP_WINDOW	BIT(19)

/* Patch L: V2-classifier tier-armed bitmask.
 *
 * Written by zenith_at_apply_tiers() per V2 worker pass; read by
 * the K1 / K2 / K3 reading sites in zenith_get_next_freq() via
 * zenith_tier_value().  Mapping (state / flag -> armed tiers):
 *
 *   LATENCY              -> migration + psi_cpu_floor
 *   FRAME flag (any)     -> migration + frame_overrun
 *   GAME flag (any)      -> migration + frame_overrun
 *   EFFICIENCY/BALANCED  -> none
 *   THERMAL_RECOVERY     -> none
 *   SUSTAINED_PERF       -> none (intentional: SUSTAINED_PERF
 *                          already pins via the actions path,
 *                          adding tier floors on top is double-
 *                          counting)
 *
 * A bit being clear means the tier is *disarmed* and the read
 * site treats the knob as 0 (off) for the duration of the V2
 * window, regardless of the profile-set value.  When the user
 * overrides via sysfs the override mask gates first and the tier
 * mask is irrelevant for that knob.
 */
#define ZENITH_AT_TIER_MIGRATION		BIT(0)
#define ZENITH_AT_TIER_PSI_CPU_FLOOR		BIT(1)
#define ZENITH_AT_TIER_FRAME_OVERRUN		BIT(2)
/* Patch M1: PSI-mem cap tier.  Armed in EFFICIENCY / BALANCED /
 * THERMAL_RECOVERY (states where backing off on memstall is the
 * desired behaviour); disarmed in LATENCY / SUSTAINED_PERF and
 * under the FRAME / GAME flag bypass.  Cap is final-freq, not a
 * predicate; disarm here turns the read-site into a no-op for
 * the duration of the V2 window regardless of the profile-set
 * thresh value.
 */
#define ZENITH_AT_TIER_PSI_MEM_CAP		BIT(3)

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
 *
 * Default flipped from 0 to 10 in the wave-2 auto-defaults round.
 * 10 is intentionally mild: a tenth-of-one-step lookahead.  Combined
 * with predict_util_smooth=1 (also flipped to 1 in wave-2), the
 * resulting predictor is two-tap-averaged and only adds util on a
 * positive slope, so a downward util ramp is never amplified.  Set
 * this to 0 to disable the predictor entirely; the rest of the
 * governor path is unchanged.
 */
#define ZENITH_DEFAULT_PREDICT_UTIL_PCT		10
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
 *
 * Default flipped from 0 to 1 in the wave-2 auto-defaults round.
 * The smooth path is gated on predict_util_pct > 0, so this default
 * is a no-op until the predictor itself is enabled.  Pair with the
 * wave-2 predict_util_pct default of 10 for a mild two-tap-averaged
 * predictor.
 */
#define ZENITH_DEFAULT_PREDICT_UTIL_SMOOTH	1

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
 *
 * Default flipped from 0 to 1 in the wave-2 auto-defaults round so
 * SurfaceFlinger / RenderThread / RenderEngine / mali-cmar-back are
 * picked up by the comm walk out of the box.  The actual freq floor
 * only fires when one of those threads is the cpu_curr at the moment
 * of a cpufreq decision (cached for ZENITH_RENDER_CACHE_TTL_NS), so
 * idle screens see no floor; only active rendering windows do.  The
 * render_floor_pct default is unchanged (70%) and continues to win
 * over the V2 effective up_threshold tier.
 */
#define ZENITH_DEFAULT_RENDER_AWARE		1
#define ZENITH_DEFAULT_RENDER_FLOOR_PCT		70
#define ZENITH_RENDER_CACHE_TTL_NS		(4 * NSEC_PER_MSEC)

/* render_floor_min_runtime_ms (default 50, [Stage 4 / Patch B]):
 *
 * Debounce window for the render-thread floor.  When a render
 * thread first becomes the cpu_curr after a quiet period, the
 * floor is *not* applied until the thread has been observed for
 * at least render_floor_min_runtime_ms milliseconds.  A render
 * thread that rises and falls inside the debounce window (e.g.
 * an idle SurfaceFlinger flush, a one-shot RenderEngine wakeup)
 * never floors the cluster.
 *
 * 0 disables the debounce: the floor fires the moment the
 * render thread is picked up, which is the original Wave-2
 * behaviour.  Capped at ZENITH_RENDER_FLOOR_MIN_RUNTIME_MS_MAX
 * so a bad echo can't push the debounce into the seconds range
 * and silently disable the floor for whole frames.
 *
 * The debounce is tracked by a per-policy stamp
 * (render_first_seen_ns) that is taken on the first sample with
 * has_render==true and reset to 0 on the first sample with
 * has_render==false.  No timers, no work_struct: the check is a
 * single ktime_get_ns() comparison on the existing eval path.
 */
#define ZENITH_DEFAULT_RENDER_FLOOR_MIN_RUNTIME_MS	50
#define ZENITH_RENDER_FLOOR_MIN_RUNTIME_MS_MAX		1000

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

/* game_auto (default 1, on):
 *
 * In-kernel heuristic for raising the effective game_mode without a
 * userspace gameswitch helper.  Walks each policy's online cpus and
 * matches cpu_curr->comm against the rcu-protected zenith_game_auto
 * comm table (see zenith_game_auto_comms[] for the seed list).  When
 * the same cpufreq decision sees the comm match for at least
 * ZENITH_GAME_AUTO_DETECT_STREAK consecutive calls, the global latch
 * zenith_game_auto_active_until_ns is set to
 * (now + ZENITH_GAME_AUTO_ACTIVE_TTL_NS).
 *
 * While the latch is in the future, zenith_eff_game_mode() reports
 * max(user game_mode, V2 effective, 1) -- so the existing game_mode=1
 * overlays (hispeed boost + input_boost decay stretch) apply
 * automatically.  The latch is auto-renewed on every fresh detection;
 * absent re-detection, it expires after ZENITH_GAME_AUTO_ACTIVE_TTL_NS
 * and the system reverts to the user / V2 game_mode value.
 *
 * Default 1 (on, wave-7 round): the seed comm list is conservative
 * (Unity / Unreal main / il2cpp / GameThread) and the worst-case
 * false-positive cost is a 5-second level-1 game_mode bump that
 * cannot push V2 into a state-change-impossible configuration.  The
 * static key zenith_game_auto_key still gates the comm walk so the
 * hot-path cost when no game thread is present is a single bounded
 * for_each_cpu() with an early break on first match.
 *
 * Init-time invariant: the static key zenith_game_auto_key must be
 * synced TRUE in zenith_init() against the default-1 scalar (same
 * pattern as wave-2 audio_aware / render_aware).  See the
 * DEFINE_STATIC_KEY_FALSE comment block.
 *
 * Tunable surface:
 *   - game_auto         RW 0/1   master gate
 *   - game_auto_state   RO 0/1   shows the current global latch state
 *   - game_auto_comms   RW CSV   comm prefix table (RCU-swapped on
 *                                store like render_comms / audio_comms)
 */
#define ZENITH_DEFAULT_GAME_AUTO		1
#define ZENITH_GAME_AUTO_CACHE_TTL_NS		(4 * NSEC_PER_MSEC)
#define ZENITH_GAME_AUTO_DETECT_STREAK		32
#define ZENITH_GAME_AUTO_ACTIVE_TTL_NS		(5ULL * NSEC_PER_SEC)

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

/* frame_budget_us_auto (default 0, off):
 *
 * When 1, the adaptive frame-budget floor in zenith_get_next_freq()
 * uses the cached refresh-rate value at zenith_drm_vblank_us in
 * preference to tunables->frame_budget_us / the per-policy override.
 * The cached value is the most recent vblank period (in us) reported
 * by the display driver via the exported zenith_set_drm_vblank_us()
 * kernel API; when zero (driver hasn't reported yet) the auto path
 * falls back to the userspace-set frame_budget_us so existing
 * tunings keep working.
 *
 * The "_drm" naming reflects the source of truth: any driver that
 * owns the active panel mode (drm-bridge, mipi-dsi panel, or vendor
 * display HAL upstreaming via drm) calls zenith_set_drm_vblank_us()
 * on every vblank-period change.  Eliminates the userspace
 * round-trip that otherwise loses 90 / 120 / 144 Hz bumps until the
 * HAL relays them to /sys/devices/.../zenith/frame_budget_us.
 */
#define ZENITH_DEFAULT_FRAME_BUDGET_US_AUTO	0

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
 *
 * Default flipped from 0 to 1 in the wave-2 auto-defaults round.
 * audio_floor_pct and audio_cap_pct stay at 0 by default, so the
 * runtime impact is just the comm walk (cached for
 * ZENITH_AUDIO_CACHE_TTL_NS) and the V2 scenario flag -- no actual
 * frequency clamp is applied unless the operator opts in by writing
 * a non-zero floor / cap.
 */
#define ZENITH_DEFAULT_AUDIO_AWARE		1
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

/* screen_off_glide_ms upper bound.  2s past the 1->0 transition is
 * generous: AOD / blanking handlers in Android typically settle in
 * well under 500ms.  0 keeps the historical cliff exactly.
 */
#define ZENITH_DEFAULT_SCREEN_OFF_GLIDE_MS	0
#define ZENITH_SCREEN_OFF_GLIDE_MS_MAX		2000

/* boot_boost_decay_ms upper bound.  30s is enough for the slowest
 * Android cold-boot animation; anything longer would conflict with
 * the screen_off detection that may legitimately follow boot.
 */
#define ZENITH_DEFAULT_BOOT_BOOST_DECAY_MS	0
#define ZENITH_BOOT_BOOST_DECAY_MS_MAX		30000
#define ZENITH_BOOT_BOOST_MAX_MS		300000

/* boot_complete latch (wave-3 follow-up):
 *
 * boot_boost_ms is a wall-clock one-shot that pins the cluster to
 * policy->max for boot_boost_ms milliseconds after boottime origin
 * regardless of whether the platform has actually finished booting.
 * On fast devices this leaves a noticeable energy / heat tail at
 * the end of the wall-clock window after Android has settled.
 *
 * The boot_complete latch lets either userspace or the in-kernel
 * auto-tune worker snap the boost deadline forward to "now" the
 * moment boot is observed complete:
 *
 *   - userspace:  init.zenith.rc raises the latch from
 *                 on property:sys.boot_completed=1 by writing 1 to
 *                 the boot_complete sysfs knob.
 *   - in-kernel:  the auto-tune worker observes a calm streak of at
 *                 least ZENITH_BOOT_COMPLETE_CALM_WINDOWS consecutive
 *                 EFFICIENCY windows past a small grace period and
 *                 raises the latch on the first qualifying policy.
 *
 * Once raised, the boot-boost path in zenith_get_next_freq() snaps
 * the deadline forward to zenith_boot_complete_ns -- the cluster
 * therefore transitions from Phase 1 (pin to max) to Phase 2 (decay
 * via boot_boost_decay_ms) immediately, instead of cliff-cutting to
 * load-derived freq the way "write boot_boost_ms 0" would.
 *
 * boot_complete_auto gates the in-kernel calm-detect path; default 1.
 * Set to 0 to require an explicit userspace write before the boost
 * is considered complete.
 */
#define ZENITH_DEFAULT_BOOT_COMPLETE_AUTO	1
#define ZENITH_BOOT_COMPLETE_CALM_WINDOWS	2
#define ZENITH_BOOT_COMPLETE_GRACE_NS \
	((u64)5000 * NSEC_PER_MSEC)

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

	/* Watchdog gate for the peak-headroom rescue tier.  See
	 * ZENITH_DEFAULT_PEAK_HEADROOM_RESCUE for full semantics.
	 * 1 enables the rescue (the default); 0 disables it entirely.
	 * The rescue is also gated by max_cap and policy->max being
	 * non-zero and pin_to_target being false.
	 */
	unsigned int		peak_headroom_rescue;

	/* Per-policy parameters for the peak-headroom rescue tier.
	 * See ZENITH_DEFAULT_PEAK_HEADROOM_* for full semantics and
	 * default values.  All five exist as sysfs knobs so a tester
	 * can A/B-tune the rescue at runtime without rebuilding the
	 * kernel.
	 *
	 *   peak_headroom_starve_load_pct (1..100, default 90)
	 *     Minimum load_pct at which a sample counts as "starving".
	 *     A value of 100 means only fully-saturated samples count;
	 *     0 is rejected on store (rescue would fire continuously).
	 *
	 *   peak_headroom_freq_floor_pct (1..100, default 85)
	 *     Cluster freq must be below this fraction of policy->max
	 *     for a sample to count as "starving".  100 means rescue
	 *     fires whenever freq < policy->max (always-on under
	 *     starvation); values closer to 0 require the cluster to
	 *     be very deep below peak before the rescue triggers.
	 *
	 *   peak_headroom_starve_streak (0..PEAK_HEADROOM_STREAK_MAX,
	 *     default 3)
	 *     Number of consecutive starving samples required before
	 *     the rescue may fire.  The streak counter must exceed
	 *     this value, so the effective wait is streak+1 windows.
	 *     0 fires on the very first starving sample.
	 *
	 *   peak_headroom_jump_pct (1..100, default 100)
	 *     Target as a percentage of policy->max when the rescue
	 *     fires.  100 pins to policy->max; lower values rescue
	 *     to an intermediate freq.  0 is rejected on store
	 *     (rescue with target 0 makes no sense).
	 *
	 *   peak_headroom_hold_ms (0..PEAK_HEADROOM_HOLD_MS_MAX,
	 *     default 50)
	 *     Minimum gap between two consecutive rescue fires, in
	 *     milliseconds.  Prevents stacking rescues on adjacent
	 *     ticks before the cpufreq driver has applied the
	 *     previous request.  0 disables the hold-down (rescue
	 *     can fire on every starving sample past the streak).
	 *
	 * The streak field is u8 for cache locality, so the sysfs
	 * store caps at PEAK_HEADROOM_STREAK_MAX (16); hold_ms caps
	 * at PEAK_HEADROOM_HOLD_MS_MAX (1000) so a runaway value
	 * cannot effectively pin the rescue off forever.
	 */
	unsigned int		peak_headroom_starve_load_pct;
	unsigned int		peak_headroom_freq_floor_pct;
	unsigned int		peak_headroom_starve_streak;
	unsigned int		peak_headroom_jump_pct;
	unsigned int		peak_headroom_hold_ms;

	/* Pre-arm tier for the peak-headroom rescue.  When 1 (the
	 * default), an early softer intervention fires while the
	 * starvation streak is accumulating but has not yet crossed
	 * peak_headroom_starve_streak.  Specifically: if the previous
	 * sample was starving (peak_starve_count > 0) and the current
	 * cluster freq is still below an effective hispeed_freq value
	 * that would itself escape the starvation floor, lift freq up
	 * to that hispeed value before the rescue tier even runs.
	 *
	 * Effect: a graduated response curve.  Sample 1 of starvation
	 * pulls the cluster into the hispeed band; subsequent samples
	 * either clear (because hispeed was enough) or progress to the
	 * full rescue (because hispeed wasn't enough and the streak
	 * crosses).  The pre-arm self-disables the moment it actually
	 * works -- once freq >= floor_freq, the next sample's starve
	 * check evaluates false and peak_starve_count resets to 0,
	 * which is also the gate that lets the pre-arm fire, so the
	 * intervention is naturally one-shot per starvation episode.
	 *
	 * 0 disables the pre-arm: starvation episodes go directly to
	 * the full rescue after the streak hits.  Mostly useful for
	 * debugging or for tracker A/B comparisons of the rescue tier
	 * alone vs. the rescue+pre-arm pair.  Also automatically a
	 * no-op when hispeed_freq is unconfigured (eff_hispeed == 0)
	 * or when the configured eff_hispeed value is itself below the
	 * floor_freq (lifting to a sub-floor hispeed wouldn't escape
	 * starvation, so we let the rescue handle it).
	 */
	unsigned int		peak_headroom_prearm;

	/* Predictive up-shift trend gate (tier 2a').  See
	 * ZENITH_DEFAULT_PREDICT_UP_THRESH for full semantics.  When
	 * non-zero, zenith_get_next_freq() compares the rise across
	 * the last predict_up_window util samples against this
	 * threshold (expressed as 256ths of max_cap so the value is
	 * unitless: 64 == ~25%% of max_cap rise across the window).
	 * 0 disables the tier; max ZENITH_PREDICT_UP_THRESH_MAX (255)
	 * is the largest value the unitless trend can take, at which
	 * point the tier effectively never fires (would require a
	 * full max_cap rise across the window).
	 */
	unsigned int		predict_up_thresh;

	/* Window size for the predict_up trend ring, in samples.
	 * Must be in [ZENITH_PREDICT_UP_WINDOW_MIN ..
	 * ZENITH_PREDICT_UP_WINDOW_MAX].  The trend compare reads the
	 * util sample written predict_up_window ticks ago against the
	 * one written this tick; out-of-range values are rejected on
	 * sysfs store.  A larger window smooths PELT noise out of the
	 * trend signal at the cost of one more tick of warm-up after
	 * a cold attach.
	 */
	unsigned int		predict_up_window;

	/* See ZENITH_DEFAULT_PEAK_HYSTERESIS_STREAK /
	 * ZENITH_DEFAULT_PEAK_STEP_DOWN_PCT (Patch E).  Either at 0
	 * disables the tier (legacy descent-from-peak).  Range
	 * checks: streak in 0..ZENITH_PEAK_HYSTERESIS_STREAK_MAX,
	 * step_down_pct in 0..100.  Reads via READ_ONCE on the eval
	 * hot path; writes via WRITE_ONCE from sysfs and from
	 * zenith_apply_profile().
	 */
	unsigned int		peak_hysteresis_streak;
	unsigned int		peak_step_down_pct;

	/* See ZENITH_DEFAULT_BOOST_IDLE_THRESH /
	 * ZENITH_DEFAULT_BOOST_IDLE_STREAK (Patch F).  Either at 0
	 * disables the boost early-exit (boost is honoured to its
	 * deadline regardless of in-cluster idle).  Range checks:
	 * thresh in 0..100 (load percent), streak in
	 * 0..ZENITH_BOOST_IDLE_STREAK_MAX.  Reads via READ_ONCE on
	 * the eval hot path.
	 */
	unsigned int		boost_idle_thresh;
	unsigned int		boost_idle_streak;

	/* See ZENITH_DEFAULT_BG_UTIL_SCALE_PCT (Patch G).  Scales
	 * zenith_get_util() output down to this percent of natural
	 * util when tunables->screen_state == 0.  Range 1..100.
	 * 100 (default) is a no-op pass-through.  Reads via
	 * READ_ONCE on the eval hot path.
	 */
	unsigned int		bg_util_scale_pct;

	/* See ZENITH_DEFAULT_SLEEPER_TAIL_THRESH_US /
	 * ZENITH_DEFAULT_SLEEPER_TAIL_PCT (Patch H).  thresh_us == 0
	 * disables.  pct bounded in 50..100; eval path reads via
	 * READ_ONCE.
	 */
	unsigned int		sleeper_tail_thresh_us;
	unsigned int		sleeper_tail_pct;

	/* See ZENITH_DEFAULT_PEER_RAMP_WINDOW_MS /
	 * ZENITH_DEFAULT_PEER_RAMP_FLOOR_PCT (Patch D).
	 * peer_ramp_window_ms == 0 disables both sides of the multi-
	 * cluster pre-arm coordination.  peer_ramp_floor_pct == 0
	 * suppresses the floor while leaving the deadline writes in
	 * place.  Reads via READ_ONCE on the eval hot path; writes
	 * via WRITE_ONCE from sysfs and zenith_apply_profile().
	 */
	unsigned int		peer_ramp_window_ms;
	unsigned int		peer_ramp_floor_pct;

	/* See ZENITH_DEFAULT_PEER_RAMP_WINDOW_OFF_MS (Patch M3).
	 * Screen-state-aware shadow of peer_ramp_window_ms.  Read
	 * via READ_ONCE in zenith_peer_ramp_effective_window_ms()
	 * whenever the arm path or the floor-eval path needs the
	 * effective window length; written via WRITE_ONCE from sysfs
	 * and zenith_apply_profile().
	 */
	unsigned int		peer_ramp_window_off_ms;

	/* See the migration_* macro block (Patch K1).  jump_pct == 0
	 * disables both sides; floor_pct == 0 suppresses the floor
	 * while leaving the per-CPU stamping in place.  All three
	 * are READ_ONCE on the eval / per-CPU update_util paths.
	 */
	unsigned int		migration_jump_pct;
	unsigned int		migration_floor_window_ms;
	unsigned int		migration_floor_pct;

	/* See ZENITH_DEFAULT_PSI_CPU_FLOOR_THRESH (Patch K2).  0
	 * disables the tier.  Read once on the eval path; written
	 * via WRITE_ONCE from sysfs and zenith_apply_profile().
	 */
	unsigned int		psi_cpu_floor_thresh;

	/* See the frame_overrun_* macro block (Patch K3).
	 * slack_us == 0 disables stamping (no overrun event ever
	 * arms a deadline); floor_pct == 0 leaves stamping in
	 * place but suppresses the floor.  All three are READ_ONCE
	 * on the eval / vblank-event paths.
	 */
	unsigned int		frame_overrun_slack_us;
	unsigned int		frame_overrun_window_ms;
	unsigned int		frame_overrun_floor_pct;

	/* See ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_STREAK /
	 * ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_FLOOR_PCT (Patch M5).
	 * Sub-knob inside K3.  deep_streak == 0 disables the deep
	 * tier (consumer never reads the streak atomic).  When
	 * armed, the deep tier amplifies an active K3 floor on
	 * sustained overrun runs.  Reads via READ_ONCE on the eval
	 * hot path; writes via WRITE_ONCE from sysfs and
	 * zenith_apply_profile().
	 */
	unsigned int		frame_overrun_deep_streak;
	unsigned int		frame_overrun_deep_floor_pct;

	/* See ZENITH_DEFAULT_PEER_RAMP_UCLAMP_MIN_RESPECT /
	 * ZENITH_DEFAULT_MIGRATION_FLOOR_UCLAMP_MIN_RESPECT
	 * (Patch M2).  Independent per-tier sub-gates: when set,
	 * the peer_ramp / migration_floor read sites compute their
	 * effective floor as max(static_pct, uclamp_min_pct).
	 * Default 1 because the uclamp path is a floor-raise; set
	 * 0 to revert to the static-pct-only Stage 4 behaviour for
	 * that specific tier without touching the master
	 * uclamp_min_respect gate.  Reads via READ_ONCE on the
	 * eval hot path; writes via WRITE_ONCE from sysfs.
	 */
	unsigned int		peer_ramp_uclamp_min_respect;
	unsigned int		migration_floor_uclamp_min_respect;

	/* See ZENITH_DEFAULT_PSI_MEM_CAP_* (Patch M1).  Three-tunable
	 * triple for the PSI-mem light cap tier.  All three reads
	 * are gated behind the existing psi_aware master gate AND
	 * the V2 PSI_MEM_CAP tier bit (via zenith_tier_value()).
	 * thresh == 0 short-circuits before any EWMA read, keeping
	 * the disabled path free.  Reads via READ_ONCE on the eval
	 * hot path; writes via WRITE_ONCE from sysfs and
	 * zenith_apply_profile().
	 */
	unsigned int		psi_mem_cap_thresh;
	unsigned int		psi_mem_cap_pct;
	unsigned int		psi_mem_cap_window_ms;

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

	/* Screen-on multiplier applied to powersave_bias before tier 3
	 * (Powersave Bias) consumes it.  Range 0..100 (percent).  When
	 * tunables->screen_state == 1 and the screen-off / thermal
	 * overrides have not replaced dynamic_bias, the effective
	 * dynamic_bias is scaled by screen_on_bias_pct / 100.  See
	 * ZENITH_DEFAULT_SCREEN_ON_BIAS_PCT for the full rationale.
	 *
	 * 100 disables the softening entirely (legacy behaviour: full
	 * configured bias applies on screen-on); 50 (the default)
	 * halves the bias on screen-on; 0 zeroes the bias on
	 * screen-on (full responsiveness, no shave).  Values above 100
	 * are rejected on store; the range is clamped on the read side
	 * defensively in case userspace bypasses the sysfs validator.
	 */
	unsigned int		screen_on_bias_pct;

	unsigned int		io_is_busy;

	/* When 1, dampen the brutality-path load_pct by the fraction
	 * of recent CPU time spent in niced-user mode. Approximation
	 * of ondemand's ignore_nice_load for a PELT-based governor.
	 */
	unsigned int		ignore_nice_load;
	
	/* Zenith Environment API */
	unsigned int		screen_state;   /* 1 = ON, 0 = OFF */

	/* Soft-glide window for the 1 -> 0 transition on screen_state.
	 * 0 (default) preserves the legacy hard cliff: as soon as
	 * screen_state is observed at 0, dynamic_up_thresh snaps to
	 * 95 and dynamic_bias snaps to 500 (the 50%% powersave
	 * penalty).  Non-zero arms a linear ramp on both: starting
	 * from the natural up_threshold / powersave_bias at the
	 * moment screen_state went to 0, climbing to the cliff
	 * targets over screen_off_glide_ms.  Eliminates the cliff
	 * that otherwise lands the moment AOD / panel-blank handlers
	 * stamp screen_state=0 while userspace work is still
	 * winding down.  Capped at ZENITH_SCREEN_OFF_GLIDE_MS_MAX in
	 * the sysfs store.
	 */
	unsigned int		screen_off_glide_ms;

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

	/* Time-based extension of wakeup_boost.  When 0 (default), the
	 * legacy tick-based ZENITH_WAKEUP_BOOST_TICKS countdown is used
	 * verbatim.  When non-zero, the detection sites additionally
	 * arm a per-CPU deadline at now + wakeup_boost_ms; the rate-
	 * limit bypass remains active until either the tick counter
	 * reaches zero AND the deadline has expired.  Lets userspace
	 * tune the bypass duration in real wall-clock time, regardless
	 * of how often zenith_freq_throttle() actually runs (which on
	 * Android can vary widely with up_rate_limit and topology).
	 * Capped at ZENITH_WAKEUP_BOOST_MS_MAX in the sysfs store.
	 */
	unsigned int		wakeup_boost_ms;

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

	/* Touchdown-vs-coordinate-stream extra window for the input
	 * boost (Patch C).  Range
	 * 0..ZENITH_INPUT_BOOST_TOUCHDOWN_EXTRA_MS_MAX.  See
	 * ZENITH_DEFAULT_INPUT_BOOST_TOUCHDOWN_EXTRA_MS for semantics.
	 * Mirrored to zenith_input_boost_touchdown_extra_ms_cache on
	 * store and on profile apply, read by zenith_input_event() on
	 * the EV_KEY/BTN_TOUCH press path.
	 */
	unsigned int		input_boost_touchdown_extra_ms;

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

	/* Multiplier for the effective down-rate delay applied to the
	 * cluster while an input boost is active (now <
	 * zenith_input_boost_until_ns).  Range 100..1000 (percent).
	 * 100 disables the extension and restores the legacy
	 * "down_rate_delay during boost == down_rate_delay outside
	 * boost" behaviour.  See ZENITH_DEFAULT_INPUT_BOOST_DOWN_RATE_-
	 * MULT_PCT for the full rationale; the extension is gated by
	 * input_boost_big_only the same way the boost itself is, so
	 * configurations that suppress the boost on LITTLE also keep
	 * LITTLE on its normal down-rate cadence.
	 */
	unsigned int		input_boost_down_rate_mult_pct;

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
	/* See ZENITH_DEFAULT_AT_V2_VAR_PROMOTE_THRESH.  load_var_ewma_x256
	 * threshold above which V2 promotes LATENCY to SUSTAINED_PERF on
	 * the variance signal alone.  0 disables variance-driven promotion.
	 */
	unsigned int		auto_tune_v2_var_promote_thresh;

	/* See ZENITH_DEFAULT_AUTO_TUNE_V3 comment block.  0/1/2 master
	 * gate for V3 self-calibration; the *_store callback also
	 * syncs the zenith_auto_tune_v3_key static key (FALSE when 0).
	 * auto_tune_v3_interval_ms is the calibration period; clamped
	 * to [ZENITH_AT_V3_INTERVAL_MIN_MS, ZENITH_AT_V3_INTERVAL_MAX_MS]
	 * on store.
	 */
	unsigned int		auto_tune_v3;
	unsigned int		auto_tune_v3_interval_ms;
	unsigned long		auto_tune_override_mask;
	unsigned int		auto_tune_cluster_aware;
	unsigned int		auto_tune_v2_signals;
	unsigned int		auto_tune_thermal_slope;
	unsigned int		auto_tune_thermal_pressure_pct;
	unsigned int		auto_tune_thermal_slope_pct;
	unsigned int		auto_tune_frame_pacing;
	unsigned int		auto_tune_sustained_gaming;

	/* See ZENITH_DEFAULT_AUTO_TUNE_V2_GLIDES.  When 1 (default), the
	 * V2 worker populates per-policy effective values for the
	 * round-U-z10 glide / coordination knobs (brutal_decay_ms,
	 * wakeup_boost_ms, boot_boost_decay_ms, screen_off_glide_ms,
	 * thermal_pressure_continuous, prefer_silver_aware,
	 * frame_budget_us_auto) based on V2 state.  Consumers use the
	 * effective value only when the user-set tunable is 0.  0 here
	 * locks all seven back to legacy behaviour exactly.  Ignored
	 * unless auto_tune_v2 is also 1.
	 */
	unsigned int		auto_tune_v2_glides;

	/* See ZENITH_DEFAULT_AUTO_TUNE_V2_TIERS comment block (Patch L).
	 * When 1 (default), the V2 worker writes a per-policy bitmask
	 * of *armed* Stage-4 floor tiers (K1 / K2 / K3) based on the
	 * just-resolved V2 state plus the FRAME / GAME flags.  K1/K2/K3
	 * read sites consult the bitmask via zenith_tier_value() and
	 * treat the underlying tunable as 0 (off) when the matching
	 * tier bit is clear.  User sysfs writes to any of the seven
	 * tier knobs set per-knob bits in auto_tune_override_mask;
	 * those bits suppress the V2 gating and the read returns the
	 * tunable value directly (Profile changes clear the entire
	 * mask, the existing zenith_apply_profile() behaviour, so a
	 * profile flip rearms V2).  Set to 0 to revert all three tiers
	 * to pure profile-driven behaviour.  Ignored unless
	 * auto_tune_v2 is also 1.
	 */
	unsigned int		auto_tune_v2_tiers;

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

	/* See ZENITH_DEFAULT_RENDER_FLOOR_MIN_RUNTIME_MS.  Debounce
	 * window in milliseconds.  Reads via READ_ONCE on the eval
	 * hot path; writes via WRITE_ONCE from sysfs and from
	 * zenith_apply_profile().  Range
	 * 0..ZENITH_RENDER_FLOOR_MIN_RUNTIME_MS_MAX.
	 */
	unsigned int		render_floor_min_runtime_ms;

	/* See ZENITH_DEFAULT_GAME_MODE. 0/1, normalised on store. */
	unsigned int		game_mode;

	/* See ZENITH_DEFAULT_GAME_AUTO comment block.  0/1 master gate
	 * for the in-kernel game detector.  Flipped via the matching
	 * sysfs node; the *_store callback also syncs the
	 * zenith_game_auto_key static key (FALSE when scalar = 0).
	 */
	unsigned int		game_auto;

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

	/* See ZENITH_DEFAULT_BOOT_COMPLETE_AUTO comment block.  Gates
	 * the in-kernel calm-detect arm of the boot_complete latch in
	 * zenith_auto_tune_work().  Default 1.  Setting to 0 disables
	 * the in-kernel arm; userspace writes to the boot_complete
	 * sysfs knob still work.
	 */
	unsigned int		boot_complete_auto;

	/* See ZENITH_DEFAULT_BOOT_BOOST_MS. 0 disables the one-shot. */
	unsigned int		boot_boost_ms;

	/* Trailing decay window for boot_boost, in milliseconds.  When 0
	 * (default), the boot-boost ends as a hard cliff at
	 * boot_boost_ms.  When non-zero, after boot_boost_ms expires
	 * zenith_get_next_freq() applies a linear floor that ramps from
	 * policy->max down to policy->min over boot_boost_decay_ms,
	 * mirroring input_boost_decay_ms's tail behaviour.  Useful when
	 * userspace boot animations or surface-flinger initial paints
	 * land just past boot_boost_ms and would otherwise see the
	 * sudden drop.  Capped at ZENITH_BOOT_BOOST_DECAY_MS_MAX.
	 */
	unsigned int		boot_boost_decay_ms;

	/* See ZENITH_DEFAULT_FRAME_BUDGET_US. Userspace writes the
	 * current vblank period in microseconds.  0 disables.
	 */
	unsigned int		frame_budget_us;

	/* See ZENITH_DEFAULT_FRAME_BUDGET_US_AUTO.  When 1, the adaptive
	 * frame-budget floor uses the cached drm-side vblank period
	 * (zenith_drm_vblank_us) instead of frame_budget_us /
	 * frame_budget_us_per_policy whenever the cached value is
	 * non-zero.  Lets a drm panel driver populate the rate without
	 * a userspace round-trip.  0 (default) preserves the legacy
	 * userspace-driven behaviour exactly.
	 */
	unsigned int		frame_budget_us_auto;

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

/*
 * Wall-clock timestamp of the last input event observed by
 * zenith_input_event.  Producer: zenith_input_event itself, on every
 * EV_KEY / EV_ABS / EV_REL event.  Consumer: also zenith_input_event,
 * to detect a "first input after quiet period" and grant that first
 * event an extended full-pin window (see ZENITH_INPUT_QUIET_*).
 *
 * Initialised to 0 so the very first event after boot is always
 * treated as a quiet-period entry.  Maintained as atomic64 so the
 * producer can read-then-write without holding any lock; multiple
 * input devices can fire concurrently with no consistency hazard
 * worse than two adjacent events both deciding the gap was long
 * enough to extend, which is harmless.
 */
static atomic64_t zenith_input_last_event_ns = ATOMIC64_INIT(0);

/*
 * Peer-ramp deadlines (Patch D).  One slot per BIG/PRIME class.
 * The slot named for class X holds the deadline that X should
 * apply -- i.e. it is *written by X's peer* when the peer ramps,
 * and *read by X* on its next eval.  Concretely:
 *
 *   BIG ramps   -> writes zenith_peer_ramp_until_ns_prime
 *   PRIME ramps -> writes zenith_peer_ramp_until_ns_big
 *   BIG eval    -> reads  zenith_peer_ramp_until_ns_big
 *   PRIME eval  -> reads  zenith_peer_ramp_until_ns_prime
 *
 * LITTLE neither writes nor reads.  See section "peer_ramp_*"
 * in the macro block at the top of the file for the rationale.
 *
 * Both stay 0 until the first ramp; deadlines are a forward
 * wall-clock ns and naturally expire as ktime_get_ns() advances
 * past them.  No reset path is needed: the read side is just a
 * "is now < until" compare.
 */
static atomic64_t zenith_peer_ramp_until_ns_big   = ATOMIC64_INIT(0);
static atomic64_t zenith_peer_ramp_until_ns_prime = ATOMIC64_INIT(0);

/* Frame-overrun rescue (Patch K3) -- file-scope atomics.
 *
 * zenith_last_vblank_ns: timestamp of the previous
 * zenith_drm_vblank_event() call.  0 means "no previous call",
 * which the producer treats as "first vblank, just record and
 * return".  Cleared (set back to 0) by the screen-state stale
 * guard so a sleep / blank period doesn't leave a stale
 * timestamp that would look like a multi-second overrun on
 * resume.
 *
 * zenith_frame_overrun_until_ns: deadline.  Stamped by the
 * producer when an overrun is detected, read by every cluster's
 * floor tier in zenith_get_next_freq() to apply a soft floor.
 * Self-disarms by deadline; ktime_get_ns() advancing past the
 * value is the disarm.
 *
 * Both are governor-wide (not per-policy) because frame
 * overruns are observed at the display layer, which sits above
 * the cpufreq policy partitioning -- a missed frame benefits
 * from lifting both BIG and PRIME, not just one.
 */
static atomic64_t zenith_last_vblank_ns          = ATOMIC64_INIT(0);
static atomic64_t zenith_frame_overrun_until_ns  = ATOMIC64_INIT(0);

/* Frame-overrun deep tier (Patch M5) -- governor-wide streak
 * counter.  Bumped by zenith_drm_vblank_event() each time a
 * vblank gap exceeds the budget; reset to 0 when a vblank
 * arrives within budget.  Read by the K3 block in
 * zenith_get_next_freq() and compared against the per-policy
 * frame_overrun_deep_streak knob.  Same governor-wide-vs-per-
 * policy reasoning as the deadline atomic above: frame events
 * sit at the display layer above cluster partitioning, and a
 * sustained-overrun signal benefits both clusters.
 *
 * Initialised to 0; a fresh policy attach therefore starts in
 * the non-deep-tier state regardless of any previous policy's
 * history, which is the conservative default.  Never decrements
 * other than the within-budget reset, so no torn-read protection
 * needed beyond atomic_t's natural alignment guarantee.
 */
static atomic_t zenith_frame_overrun_streak       = ATOMIC_INIT(0);

/*
 * Cached drm-panel vblank period, in microseconds.  Producer:
 * display drivers / panel bridges call zenith_set_drm_vblank_us()
 * whenever the active vblank period changes (e.g. on a 60->120Hz
 * mode switch in DRM).  Consumer: zenith_get_next_freq()'s
 * adaptive frame-budget floor when tunables->frame_budget_us_auto
 * is non-zero.  Zero means "no driver reported yet"; the auto path
 * then falls back to the userspace-set frame_budget_us so existing
 * tunings keep working.
 *
 * Lockless read on the consumer side; producers use atomic_set()
 * with no ordering requirements other than "the latest write wins".
 */
static atomic_t zenith_drm_vblank_us = ATOMIC_INIT(0);

/* Boot-completion latch.  See ZENITH_DEFAULT_BOOT_COMPLETE_AUTO
 * comment block above struct zenith_tunables.  Both globals are
 * read lock-free from zenith_get_next_freq() (the boost path) and
 * written either from the boot_complete sysfs *_store callback or
 * from zenith_auto_tune_work() once the calm streak qualifies.
 */
static atomic_t zenith_boot_complete = ATOMIC_INIT(0);
static u64 zenith_boot_complete_ns;

/* In-kernel game detector global latch.  See the
 * ZENITH_DEFAULT_GAME_AUTO comment block.  Read lock-free via
 * READ_ONCE() from zenith_eff_game_mode() (the helper consumed by
 * the hot-path readers of game_mode), and written either from the
 * hot-path detector zenith_policy_game_auto_tick() or from the
 * game_auto sysfs *_store callback (which clears the latch on
 * disable so a stale latch does not survive game_auto = 0).
 *
 * The value is the boottime nanosecond deadline at which the latch
 * expires.  Zero means "no game detected".  The compare/expire
 * check is "now < latch", so wraparound is not a concern in any
 * realistic uptime.
 */
static u64 zenith_game_auto_active_until_ns;

/* True if the in-kernel game detector latch is currently in the
 * future, i.e. a recent fresh detection has happened and is still
 * within ZENITH_GAME_AUTO_ACTIVE_TTL_NS.  Lock-free; readers tolerate
 * a stale value by at most one cpufreq tick.
 */
static bool zenith_game_auto_active(void)
{
	u64 until = READ_ONCE(zenith_game_auto_active_until_ns);

	if (!until)
		return false;
	return ktime_get_ns() < until;
}

/* Returns the effective game_mode used by all hot-path overlays:
 *   max(user_or_v2_game_mode, in-kernel auto detector)
 *
 * base_gm is whatever the existing zenith_tunable_or_local() pair
 * (t->game_mode vs z_policy->at_effective_game_mode) returned -- the
 * auto detector only ever bumps an under-1 result up to 1.  Higher
 * user / V2 values are preserved verbatim.  The static-branch gate
 * means a no-op single never-taken jump when game_auto = 0.
 */
static inline unsigned int zenith_eff_game_mode(unsigned int base_gm)
{
	if (static_branch_unlikely(&zenith_game_auto_key) &&
	    base_gm < 1 &&
	    zenith_game_auto_active())
		return 1;
	return base_gm;
}

/**
 * zenith_set_drm_vblank_us - publish active panel vblank period to zenith
 * @us: vblank period in microseconds; 0 clears the cache.
 *
 * Display drivers / drm-panel bridges call this whenever the active
 * panel's vblank period changes (e.g. 60 -> 120Hz switch via
 * drm_atomic_commit_tail).  Lock-free; safe to call from any context
 * including atomic.  Values larger than ZENITH_FRAME_BUDGET_US_MAX
 * (50ms, ~20Hz) are silently clamped down because anything above
 * that is past the useful rate-shaping range and would push the
 * eff_pct calculation in the consumer to 0 anyway.
 */
void zenith_set_drm_vblank_us(unsigned int us)
{
	if (us > ZENITH_FRAME_BUDGET_US_MAX)
		us = ZENITH_FRAME_BUDGET_US_MAX;
	atomic_set(&zenith_drm_vblank_us, (int)us);
}
EXPORT_SYMBOL_GPL(zenith_set_drm_vblank_us);

/* Governor-wide caches for the frame-overrun knobs (Patch K3).
 * The producer (zenith_drm_vblank_event()) runs from the display
 * driver context with no struct zenith_policy in scope; if the
 * arming logic needed a tunables lookup it would have to walk
 * the policy list.  Mirror the active values into file-scope
 * unsigned ints instead, written by sysfs store and
 * zenith_apply_profile(); same shape as
 * zenith_input_boost_active_ms above.
 *
 * Multiple policies sharing one governor-wide cache is fine
 * because the per-policy frame_overrun_slack_us /
 * frame_overrun_window_ms values are expected to be uniform
 * across clusters -- they describe a property of the display,
 * not of a specific cluster.
 */
static unsigned int zenith_frame_overrun_slack_us_cache =
	ZENITH_DEFAULT_FRAME_OVERRUN_SLACK_US;
static unsigned int zenith_frame_overrun_window_ms_cache =
	ZENITH_DEFAULT_FRAME_OVERRUN_WINDOW_MS;

/**
 * zenith_drm_vblank_event - notify zenith of a panel vblank
 *
 * Display drivers / drm-panel bridges call this from the per-
 * vblank IRQ handler so the governor can detect frame budget
 * overruns -- e.g. compositor + render thread missed a frame
 * and the next vblank arrives a full extra period late.
 *
 * Lock-free; safe to call from any context including IRQ.  When
 * the panel driver does not call this, the entire detection
 * path is a no-op (the deadline atomic stays at 0 and the
 * floor tier never fires).  Same fail-safe shape as
 * zenith_set_drm_vblank_us().
 *
 * The first call after policy attach (or after the screen-
 * state stale guard clears zenith_last_vblank_ns) just records
 * the timestamp and returns.  On every subsequent call the
 * elapsed wall-clock delta is compared against the cached
 * vblank period plus tunables->frame_overrun_slack_us; if the
 * gap is wider, the renderer missed the frame budget and a
 * deadline is stamped on zenith_frame_overrun_until_ns.
 */
void zenith_drm_vblank_event(void)
{
	unsigned int slack_us =
		READ_ONCE(zenith_frame_overrun_slack_us_cache);
	unsigned int window_ms;
	unsigned int period_us;
	u64 now_ns = ktime_get_ns();
	u64 last_ns = (u64)atomic64_read(&zenith_last_vblank_ns);
	u64 delta_ns;

	atomic64_set(&zenith_last_vblank_ns, (s64)now_ns);
	if (!slack_us || !last_ns || now_ns <= last_ns)
		return;
	period_us = (unsigned int)atomic_read(&zenith_drm_vblank_us);
	if (!period_us)
		return;
	delta_ns = now_ns - last_ns;
	if (delta_ns <= ((u64)period_us + slack_us) * NSEC_PER_USEC) {
		/* Within budget -- reset the deep-tier streak (Patch M5).
		 * A single good frame breaks any "sustained" pattern the
		 * deep tier was tracking, so the deep floor should
		 * back off on the very next eval.
		 */
		atomic_set(&zenith_frame_overrun_streak, 0);
		return;
	}
	window_ms = READ_ONCE(zenith_frame_overrun_window_ms_cache);
	if (!window_ms)
		return;
	atomic64_set(&zenith_frame_overrun_until_ns,
		     (s64)(now_ns + (u64)window_ms * NSEC_PER_MSEC));
	/* Track consecutive overruns for the deep-tier consumer in
	 * zenith_get_next_freq() (Patch M5).  Bumping is post-stamp
	 * so window_ms == 0 (K3 floor suppressed but stamping still
	 * occurs) does not feed the deep tier either: deep is an
	 * amplification of K3 and inherits its arming gate.
	 */
	atomic_inc(&zenith_frame_overrun_streak);
}
EXPORT_SYMBOL_GPL(zenith_drm_vblank_event);

static unsigned int zenith_input_boost_active_ms = ZENITH_DEFAULT_INPUT_BOOST_MS;

/* Governor-wide cache for the touchdown-extra knob (Patch C).
 * Mirrored from t->input_boost_touchdown_extra_ms by sysfs store
 * and zenith_apply_profile().  Read by zenith_input_event() on
 * the EV_KEY/BTN_TOUCH down path.  Stored as plain unsigned int
 * with READ_ONCE/WRITE_ONCE; the racy reader doesn't care if it
 * sees a stale value across the store window.
 */
static unsigned int zenith_input_boost_touchdown_extra_ms_cache =
	ZENITH_DEFAULT_INPUT_BOOST_TOUCHDOWN_EXTRA_MS;

/* Monotonically-increasing global count of qualifying input events seen
 * by zenith_input_event. Auto-tune workers sample this periodically and
 * subtract their last-observed value to get an events-per-window rate.
 */
static atomic64_t zenith_auto_input_events = ATOMIC64_INIT(0);
#define ZENITH_AUTO_TUNE_PERIOD_MS	10000	/* classify every 10s  */

/* Stage 4 / Patch I -- governor-wide input observability counters.
 *
 * Each counter is a monotonic atomic64; readers get a snapshot via
 * the `zenith_input_stats` sysfs node and subtract last-observed
 * values to compute rates.  Counters never reset; rollover on a 64-
 * bit atomic is "never" in practice.  The cost on the input path is
 * one atomic64_inc per counter touched; in the trace-disabled hot
 * path that's three increments per qualifying event, all cheap.
 *
 * Counters:
 *
 *   zenith_in_events_total
 *     Every EV_KEY/EV_ABS/EV_REL event seen by zenith_input_event,
 *     regardless of whether the boost is enabled.
 *   zenith_in_boosts_armed
 *     Events that wrote zenith_input_boost_until_ns (i.e.
 *     active != 0 at event time).
 *   zenith_in_boosts_quiet_extended
 *     Subset of armed boosts where the quiet-period extension
 *     widened the boost window past the configured active duration.
 *   zenith_in_boosts_skipped_disabled
 *     Events that fell through because input_boost_ms == 0
 *     (boost feature disabled at event time).
 *   zenith_in_boosts_early_exit
 *     [Stage 4 / Patch F] Per-policy decisions where the
 *     persistent-idle streak (boost_idle_low_streak) crossed
 *     the boost_idle_streak threshold and the policy preempted
 *     the still-armed input boost on its tier-0 path this tick.
 *     The global zenith_input_boost_until_ns is left untouched;
 *     other policies continue to honour the boost.
 */
static atomic64_t zenith_in_events_total = ATOMIC64_INIT(0);
static atomic64_t zenith_in_boosts_armed = ATOMIC64_INIT(0);
static atomic64_t zenith_in_boosts_quiet_extended = ATOMIC64_INIT(0);
static atomic64_t zenith_in_boosts_skipped_disabled = ATOMIC64_INIT(0);
static atomic64_t zenith_in_boosts_early_exit = ATOMIC64_INIT(0);

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
	/* Stage 4 / Patch I additions.  Split out the tp_paths
	 * previously bucketed into ZENITH_STAT_OTHER (predict_up,
	 * peak_prearm, peak_rescue) so testers can read the
	 * fire-rate of each Stage-3+ tier independently of the
	 * legacy buckets.
	 */
	ZENITH_STAT_PREDICT_UP,		/* predict_up */
	ZENITH_STAT_PEAK_PREARM,	/* peak_prearm */
	ZENITH_STAT_PEAK_RESCUE,	/* peak_rescue */
	ZENITH_STAT_PEAK_HYST,		/* peak_hyst (Patch E) */
	ZENITH_STAT_PEER_RAMP,		/* peer_ramp (Patch D) */
	ZENITH_STAT_MIGRATION_FLOOR,	/* migration_floor (Patch K1) */
	ZENITH_STAT_PSI_CPU_FLOOR,	/* psi_cpu_floor (Patch K2) */
	ZENITH_STAT_FRAME_OVERRUN,	/* frame_overrun (Patch K3) */
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

	/* Entry-side streak counter for the peak-headroom rescue tier.
	 * Increments on every sample where the cluster is starving
	 * (load_pct >= STARVE_LOAD_PCT and freq < FREQ_FLOOR_PCT of
	 * policy->max), resets when either condition breaks.  The
	 * rescue fires only when the streak exceeds STARVE_STREAK,
	 * giving STARVE_STREAK+1 consecutive sample windows of
	 * starvation as the entry hysteresis.  Capped to a small u8 to
	 * avoid wraparound on indefinitely-sustained heavy load.
	 */
	u8			peak_starve_count;

	/* Hold-down deadline for the peak-headroom rescue tier.  Stamped
	 * to ktime_get_ns() + PEAK_HEADROOM_HOLD_MS at the moment the
	 * rescue fires.  While now < this deadline, repeated streak
	 * crossings are observed (the streak counter still increments)
	 * but no second rescue freq-bump is applied.  Eliminates the
	 * pathological case where a rescue lifts freq, the very next
	 * sample observes load_pct still >= STARVE_LOAD_PCT and freq
	 * still < FLOOR_PCT (because the cpufreq driver hasn't applied
	 * the previous request yet), and another rescue fires --
	 * stacking two unwanted up-shifts on what should be a single
	 * rescue event.  Cleared (set to 0) at policy init.
	 */
	u64			peak_rescue_until_ns;

	/* Migration-arrival soft-floor deadline (Patch K1).  Stamped
	 * by the per-CPU update_util callback whenever any CPU in
	 * this policy sees a sample-to-sample util jump exceeding
	 * tunables->migration_jump_pct of its max_capacity.  Read by
	 * the migration_floor tier in zenith_get_next_freq() to
	 * decide whether to apply the soft floor.  Self-disarms by
	 * deadline; ktime_get_ns() advancing past the value is the
	 * disarm.  Cleared (set to 0) at policy init.  Updated under
	 * the per-policy update_lock by the shared callback;
	 * unlocked but ordered by raw_spin_lock acquire/release in
	 * the single-CPU callback.  No torn-write hazard either way:
	 * a 64-bit write on a 64-bit kernel is atomic, and the field
	 * is only ever monotonic-forward written.
	 */
	u64			migration_in_until_ns;

	/* PSI-mem cap deadline (Patch M1).  Stamped by the eval path
	 * when zenith_psi_mem_some_pct() crosses psi_mem_cap_thresh
	 * (and master psi_aware + V2 PSI_MEM_CAP tier gates pass);
	 * read by the same eval path one tier later as
	 * `now < deadline -> cap fires`.  Same per-policy / single-
	 * writer reasoning as migration_in_until_ns above: each
	 * policy stamps its own deadline (so the BIG / PRIME caps
	 * release independently), and the eval is serialized by
	 * update_lock.
	 *
	 * Cleared (set to 0) at policy init by zenith_alloc_-
	 * policy(); never decremented other than by the deadline
	 * comparison expiring, so no torn-write hazard on a 64-bit
	 * kernel and natural alignment guarantee on 32-bit.
	 */
	u64			psi_mem_cap_until_ns;

	/* Util-trend ring for the predictive up-shift tier (2a').  The
	 * tail of zenith_get_next_freq() pushes the current sample's
	 * util into util_history[util_history_idx] and advances the
	 * index modulo predict_up_window.  util_history_count tracks
	 * how many slots have been written since policy attach (or
	 * since the last shrink-on-window-store), so the tier can wait
	 * for the ring to warm up before evaluating a trend on
	 * unwritten zeroes.  Capped at ZENITH_PREDICT_UP_WINDOW_MAX so
	 * the storage cost is bounded.
	 */
	unsigned long		util_history[ZENITH_PREDICT_UP_WINDOW_MAX];
	unsigned int		util_history_idx;
	unsigned int		util_history_count;

	/* Render-thread floor debounce stamp [Stage 4 / Patch B].
	 * Set to ktime_get_ns() on the first eval where
	 * zenith_policy_has_render() returns true after a quiet
	 * period (previous sample saw has_render==false).  Reset to
	 * 0 the first time has_render returns false, so the debounce
	 * window restarts on each fresh render-thread arrival.
	 * Compared against tunables->render_floor_min_runtime_ms
	 * before the floor is allowed to fire.  Zero-initialised at
	 * policy alloc; no special teardown.
	 */
	u64			render_first_seen_ns;

	/* Peak-return hysteresis streak counter [Stage 4 / Patch E].
	 * Increments on every consecutive sample where the previous
	 * cached_raw_freq was at peak class
	 * (>= ZENITH_PEAK_HYSTERESIS_PEAK_THRESH_PCT * policy->max
	 * / 100) and the current freq wants to drop sharply.  When
	 * < tunables->peak_hysteresis_streak, the soft floor at
	 * (prev_freq * peak_step_down_pct / 100) is applied; when
	 * the streak drains, the cluster falls naturally through
	 * the lower tiers.  Reset on any sample where the previous
	 * freq is below peak class, or when the freq tier already
	 * computes a value at or above the soft floor.  Capped at
	 * ZENITH_PEAK_HYSTERESIS_STREAK_MAX so a stuck-near-peak
	 * regime cannot accumulate unbounded streak credit.
	 */
	unsigned int		peak_low_streak;

	/* Anchor freq for the peak-return hysteresis tier
	 * [Stage 4 / Patch E].  Captured on the transition from a
	 * peak-class previous freq to a non-peak natural freq, then
	 * used as the source for the soft floor
	 * (anchor * peak_step_down_pct / 100) until the streak
	 * drains.  Cleared on streak drain or on a sample where the
	 * computed freq is already at or above the soft floor (no
	 * hysteresis needed).  0 means "no anchor armed".
	 */
	unsigned int		peak_hyst_anchor_freq;

	/* Persistent-idle streak counter for boost early-exit
	 * [Stage 4 / Patch F].  Increments on every tick where an
	 * input boost is armed (now < zenith_input_boost_until_ns)
	 * AND the cluster's load_pct is below
	 * tunables->boost_idle_thresh.  Reset on boost expiry, on
	 * any non-idle sample inside the boost window, or after a
	 * preemption tick fires.  Capped at
	 * ZENITH_BOOST_IDLE_STREAK_MAX so a stuck-near-zero workload
	 * cannot accumulate unbounded streak credit.
	 */
	unsigned int		boost_idle_low_streak;

	/* Last-runnable timestamp [Stage 4 / Patch H].  Stamped
	 * with ktime_get_ns() in zenith_get_next_freq() whenever
	 * tp_load_pct > 0 (the cluster has any util).  Read by the
	 * sleeper-tail tier to gate the freq shave.
	 */
	u64			last_runnable_ns;

	/* Last decision tag chosen by zenith_get_next_freq() on this
	 * policy [Stage 4 / Patch J].  Stamped right after the
	 * stats[] update at the end of every eval (cached and
	 * uncached paths both update it).  Read-only sysfs node
	 * "last_decision_path" dumps the current value per-policy,
	 * formatted "policy<cpu>(<cluster>): <tag>\n".  Pointers
	 * are to .rodata string literals so storage is a single
	 * pointer assignment, no copy.  Reads use READ_ONCE for
	 * coherency with the eval-side WRITE_ONCE.
	 */
	const char		*last_decision_path;

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
	/* M1: time-in-state accounting.  at_state_residency_ns[s] is the
	 * cumulative wall-clock time spent in V2 state s since governor
	 * start (or since the last reset).  Updated lazily at every V2
	 * commit point: on transition from old to new, accumulate the
	 * (now - last_change_ns) delta into residency[old_state] and
	 * stamp last_change_ns := now.  Cheap (one ktime_get + 5 u64
	 * stores per actual transition, NOT per tick).  Surfaced
	 * read-only via auto_tune_state_residency sysfs as a CSV that
	 * userspace tools can sample at low frequency to compute
	 * percent-time-in-state.
	 */
	u64			at_state_residency_ns[5];
	u64			at_state_last_change_ns;
	/* M2: V2 state transition history ring.  Last
	 * ZENITH_AT_HISTORY_NR commits per policy, expressed as
	 * { ts_boottime_ns, from_state, to_state, reason, flags }
	 * tuples.  Surfaced read-only via auto_tune_state_history
	 * sysfs as one line per entry, most recent first.
	 *
	 * Each entry is 16 bytes; 32 entries = 512 B per policy
	 * (typically 2 policies = 1 KB total).  Cheap.
	 */
	struct {
		u64		ts_ns;
		u32		flags;
		u8		from;
		u8		to;
		u8		reason;
		u8		_pad;
	}			at_history[ZENITH_AT_HISTORY_NR];
	unsigned int		at_history_head;
	unsigned int		at_history_count;
	/* at_last_applied_state is the V2 state AFTER the cluster-aware
	 * demotion path runs.  When auto_tune_cluster_aware = 1 the
	 * little cluster takes LATENCY/SUSTAINED_PERF -> BALANCED and the
	 * prime cluster takes BALANCED -> LATENCY; the at_last_state
	 * field holds the pre-demotion V2 state (so V3 / hysteresis /
	 * cooldown all see the canonical decision), and this field
	 * holds what was actually applied via zenith_at_apply_actions().
	 * Surfaced through auto_tune_status as applied_state= so an
	 * operator reading the file can tell at-a-glance which cluster
	 * was demoted vs the V2 logical state.  When cluster_aware is
	 * off, applied_state == at_last_state.
	 */
	unsigned int		at_last_applied_state;
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

	/* Per-policy consecutive-EFFICIENCY-window counter for the
	 * boot_complete in-kernel calm detector.  See the
	 * ZENITH_DEFAULT_BOOT_COMPLETE_AUTO comment block.  Reset to 0
	 * whenever the resolved V2 state is anything other than
	 * EFFICIENCY; the auto detector latches zenith_boot_complete on
	 * the first per-policy worker that hits
	 * ZENITH_BOOT_COMPLETE_CALM_WINDOWS past the grace period.
	 */
	unsigned int		at_boot_calm_streak;

	/* V3 self-calibration state (see ZENITH_DEFAULT_AUTO_TUNE_V3
	 * comment block).  Updated at the tail of zenith_auto_tune_work()
	 * once per (auto_tune_v3_interval_ms) wall-clock period when the
	 * v3 key is enabled.  All fields are owned by the v2 worker
	 * thread and read either from the same thread (calibration
	 * step) or via sysfs *_show under the gov_attr_set rwsem (which
	 * guarantees a consistent snapshot).
	 *
	 *   at_v3_last_calib_ns    -- boottime ns of last calibration
	 *   at_v3_last_transitions -- transitions counted in last window
	 *   at_v3_hyst_offset      -- signed offset on hysteresis_windows
	 *   at_v3_cool_offset      -- signed offset on cooldown_windows
	 *
	 * Offsets are clamped to [ZENITH_AT_V3_OFFSET_MIN ..
	 * ZENITH_AT_V3_OFFSET_MAX] and only consumed when the v3 mode
	 * is APPLY (== 2).
	 */
	u64			at_v3_last_calib_ns;
	unsigned int		at_v3_last_transitions;
	signed char		at_v3_hyst_offset;
	signed char		at_v3_cool_offset;

	/* round-U-z10 glide / coordination knobs auto-driven by the V2
	 * worker via zenith_at_apply_glides() when
	 * tunables->auto_tune_v2_glides is on.  Consumers fall through
	 * to these only when the user-set tunable is 0; otherwise the
	 * user value wins outright.  at_local_glides_active gates the
	 * fall-through; cleared on auto_tune_v2 disable so the next
	 * unrelated freq tick stops consulting stale values.
	 */
	bool			at_local_glides_active;
	unsigned int		at_local_brutal_decay_ms;
	unsigned int		at_local_wakeup_boost_ms;
	unsigned int		at_local_boot_boost_decay_ms;
	unsigned int		at_local_screen_off_glide_ms;
	unsigned int		at_local_thermal_pressure_continuous;
	unsigned int		at_local_prefer_silver_aware;
	unsigned int		at_local_frame_budget_us_auto;

	/* Patch L: Stage-4 K1/K2/K3 tier-armed bitmask, written by
	 * zenith_at_apply_tiers() per V2 worker pass.  See the
	 * ZENITH_AT_TIER_* comment block.  Read by zenith_tier_value()
	 * from the K1/K2/K3 read sites in zenith_get_next_freq().
	 *
	 * at_local_tiers_active gates the read: false (cleared on V2
	 * disable, profile change, or auto_tune_v2_tiers=0) means the
	 * K1/K2/K3 reads return the underlying tunable verbatim, the
	 * pre-Patch-L behaviour.  true means consult the armed mask.
	 *
	 * Single-writer (V2 worker) / single-reader (eval path under
	 * update_lock); staleness is bounded by ZENITH_AUTO_TUNE_PERIOD_MS.
	 */
	bool			at_local_tiers_active;
	unsigned long		at_local_tier_armed_mask;
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

	/* screen_off_glide_ms tracking.  screen_state_last is the
	 * screen_state value seen on the previous zenith_get_next_freq()
	 * tick; transitions are detected by comparing tunables-> against
	 * this snapshot.  screen_off_arm_ns is set to ktime_get_ns() at
	 * the moment of the 1 -> 0 transition and cleared on the
	 * 0 -> 1 transition; non-zero on the screen-off branch means
	 * the glide ramp is candidate for application.  Single-writer,
	 * single-reader (zenith_get_next_freq() under update_lock).
	 */
	u64			screen_off_arm_ns;
	unsigned int		screen_state_last;

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

	/* Cached per-policy result of the game-auto comm walk + streak
	 * counter for the in-kernel game detector.  See
	 * ZENITH_DEFAULT_GAME_AUTO comment block.  TTL is
	 * ZENITH_GAME_AUTO_CACHE_TTL_NS for the comm-match cache; the
	 * streak counter increments on each hot-path call that observes
	 * a match (subject to the cache TTL) and resets to 0 on the
	 * first miss.  When it reaches ZENITH_GAME_AUTO_DETECT_STREAK,
	 * the global zenith_game_auto_active_until_ns latch is renewed
	 * for ZENITH_GAME_AUTO_ACTIVE_TTL_NS and the streak resets.
	 */
	bool			game_auto_match;
	u64			game_auto_cache_stamp_ns;
	unsigned int		game_auto_streak;

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

	/* Previous post-iowait_apply util on this CPU, used by the
	 * migration-arrival detector (Patch K1).  Independent from
	 * wakeup_prev_util because that field is only updated when
	 * tunables->wakeup_boost is set, and the migration detector
	 * needs to run regardless.  Zero-initialised by zenith_start()'s
	 * memset, which means the very first tick after policy attach
	 * looks like a "jump from 0" -- harmless: the migration tier
	 * disarms by 30 ms (default) wall-clock and any cluster
	 * starting from idle benefits from a brief floor anyway.
	 */
	unsigned long		migration_prev_util;

	/* Deadline mirror of wakeup_boost_ticks.  0 means no
	 * wall-clock-based bypass armed; non-zero is an absolute
	 * ktime_get_ns() value at which the time-based portion of the
	 * up-rate bypass stops applying.  Set together with
	 * wakeup_boost_ticks at the detection sites when
	 * tunables->wakeup_boost_ms is non-zero.
	 */
	u64			wakeup_boost_until_ns;

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
static unsigned int zenith_glide_value(struct zenith_policy *z_policy,
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

	/* Patch G: background-task util scaling.  When the display
	 * is off, scale util_out down to bg_util_scale_pct percent
	 * of its natural value so the downstream freq decision
	 * lands lower for the same underlying load.  Bypassed when
	 * the screen is on so display-on responsiveness is
	 * unchanged.  100 is a pass-through; the multiply path is
	 * skipped to avoid the cost on the common case.
	 */
	{
		unsigned int scale_pct =
			READ_ONCE(z_cpu->z_policy->tunables->bg_util_scale_pct);
		unsigned int screen =
			READ_ONCE(z_cpu->z_policy->tunables->screen_state);

		if (!screen && scale_pct && scale_pct < 100)
			util_out = (util_out * scale_pct) / 100;
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

	/* Input-boost-aware down-rate extension.  While the
	 * input_boost full-pin window is in effect (now <
	 * zenith_input_boost_until_ns), multiply down_delay by
	 * tunables->input_boost_down_rate_mult_pct / 100.  Holds the
	 * cluster up across the inter-frame gaps inside an active
	 * scroll / swipe.  See ZENITH_DEFAULT_INPUT_BOOST_DOWN_RATE_-
	 * MULT_PCT for the full rationale.  No-op when:
	 *   - the multiplier is at 100%% (legacy behaviour);
	 *   - no input boost is active (until == 0 or expired);
	 *   - the configured cluster gate (input_boost_big_only)
	 *     would suppress the boost itself for this cluster
	 *     (LITTLE in the default config, where down-rate
	 *     extension would just delay parking the cluster).
	 */
	{
		unsigned int mult_pct =
			READ_ONCE(tunables->input_boost_down_rate_mult_pct);

		if (mult_pct > 100 &&
		    (!READ_ONCE(tunables->input_boost_big_only) ||
		     z_policy->is_big_cluster)) {
			u64 until = (u64)atomic64_read(
					&zenith_input_boost_until_ns);

			if (until && (u64)time < until)
				down_delay = (down_delay * mult_pct) / 100;
		}
	}

	if (next_freq > z_policy->next_freq) {
		unsigned int spike = z_policy->policy->max >> ZENITH_SPIKE_SHIFT;
		unsigned int cpu;
		struct zenith_cpu *z_cpu;

		for_each_cpu(cpu, z_policy->policy->cpus) {
			z_cpu = &per_cpu(zenith_cpu, cpu);

			/* Tick-based bypass: legacy fast path that bypasses
			 * the up-rate limit for ZENITH_WAKEUP_BOOST_TICKS
			 * upward transitions after an idle->busy detection.
			 */
			if (z_cpu->wakeup_boost_ticks) {
				z_cpu->wakeup_boost_ticks--;
				return false;
			}

			/* Wall-clock-based bypass armed by wakeup_boost_ms.
			 * Stays active until the deadline passes; no need
			 * to decrement.  Self-disarms on first sample past
			 * the deadline so subsequent ticks fall through to
			 * the normal up_rate_limit gate.
			 */
			if (z_cpu->wakeup_boost_until_ns) {
				if (ktime_get_ns() <
				    z_cpu->wakeup_boost_until_ns)
					return false;
				z_cpu->wakeup_boost_until_ns = 0;
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
		unsigned int gm = zenith_eff_game_mode(
				z_policy->tunables->game_mode);

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
static struct zenith_comm_table __rcu *zenith_game_auto_table;
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
 *   - vendor.qti.audi  Qualcomm vendor audio HAL (truncated)
 *   - vendor.google.a  Tensor / Pixel vendor audio HAL (truncated)
 *   - vendor.oplus.au  OPlus / OnePlus / Realme audio HAL family
 *   - audio.hw.servic  Samsung audio.hw service (truncated)
 *
 * Order is tuned for cache-friendliness on phone workloads (the most
 * common per-frame matches first).  Default-list extensions are
 * runtime-augmentable via the audio_comms RW sysfs (CSV format).
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
	"vendor.qti.audi",
	"vendor.google.a",
	"vendor.oplus.au",
	"audio.hw.servic",
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
 *   - camerahalserver    Pixel/Tensor vendor camera HAL daemon
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
 *   - vendor.qti.hardwa  Qualcomm 8-gen+ truncated comm
 *                        (vendor.qti.hardware.camera.provider@*)
 *   - vendor.oplus.cam   OPlus / OnePlus / Realme camera HAL family
 *   - vendor.samsung.ca  Samsung Camera HAL (truncated to 16 chars)
 *
 * Order is tuned for cache-friendliness: most common matches first.
 * Default-list extensions are runtime-augmentable via the
 * camera_comms RW sysfs (CSV format).
 */
static const char * const zenith_camera_comms[] = {
	"cameraserver",
	"camerahalserver",
	"cameraprovider",
	"provider@",
	"provider.MTK",
	"mtkcam-",
	"mtkcamutil",
	"Camera2-",
	"CamX_",
	"CamX-",
	"vendor.qti.camera",
	"vendor.qti.hardwa",
	"vendor.oplus.cam",
	"vendor.samsung.ca",
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

/* Default seed list for the in-kernel game detector.  See the
 * ZENITH_DEFAULT_GAME_AUTO comment block.  Used at zenith_gov_init()
 * time and as the reset target when game_auto_comms is written empty;
 * live matching always reads the RCU table.
 *
 * Entries are tuned for typical Android game engines:
 *
 *   - UnityMain          Unity main thread (most common Unity name)
 *   - UnityGfxDeviceW    Unity gfx device worker
 *   - il2cpp             Unity IL2CPP scripting backend worker
 *   - GameThread         Unreal Engine main thread (also some custom
 *                        engines).  This is one Android system-wide
 *                        contender that stylistically conflicts with
 *                        Android's own RenderThread; we keep it on
 *                        the assumption that it is dominant on big
 *                        cores only when an actual Unreal title is
 *                        running.  Userspace can drop it via the
 *                        game_auto_comms knob if it conflicts.
 */
static const char * const zenith_game_auto_comms[] = {
	"UnityMain",
	"UnityGfxDeviceW",
	"il2cpp",
	"GameThread",
};

/* Hot-path comm walk used by the in-kernel game detector.  TTL'd
 * for ZENITH_GAME_AUTO_CACHE_TTL_NS so a 60/120/144 Hz cpufreq
 * decision rate only does the strncmp loop a few times per second.
 * Caller gates on the static_branch_unlikely(zenith_game_auto_key)
 * branch and the live tunables->game_auto scalar; this helper does
 * not re-check either.  Returns the raw comm-match boolean for the
 * caller (zenith_policy_game_auto_tick) to feed into the streak
 * counter.
 */
static bool zenith_policy_has_game_auto(struct zenith_policy *z_policy)
{
	u64 now = ktime_get_ns();
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned int cpu;
	bool match = false;

	if (z_policy->game_auto_cache_stamp_ns &&
	    now - z_policy->game_auto_cache_stamp_ns <
	    ZENITH_GAME_AUTO_CACHE_TTL_NS)
		return z_policy->game_auto_match;

	rcu_read_lock();
	{
		struct zenith_comm_table *t =
			rcu_dereference(zenith_game_auto_table);

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

	z_policy->game_auto_match = match;
	z_policy->game_auto_cache_stamp_ns = now;
	return match;
}

/* Per-policy hot-path tick for the in-kernel game detector.  Called
 * once per zenith_get_next_freq() invocation, gated by the
 * zenith_game_auto_key static branch and the live tunables->game_auto
 * scalar.  Maintains the per-policy streak counter and renews the
 * global zenith_game_auto_active_until_ns latch when the streak
 * crosses ZENITH_GAME_AUTO_DETECT_STREAK.  Resets streak on miss so
 * we only latch on a sustained match.
 *
 * Streak reset on detection (rather than continued increment) keeps
 * the atomic write rate bounded -- one write per
 * (DETECT_STREAK * cache_TTL) at the most pessimistic, ~128 ms
 * worst case at the default knob values.
 */
static void zenith_policy_game_auto_tick(struct zenith_policy *z_policy)
{
	bool match = zenith_policy_has_game_auto(z_policy);
	u64 until;

	if (match)
		z_policy->game_auto_streak++;
	else
		z_policy->game_auto_streak = 0;

	if (z_policy->game_auto_streak >= ZENITH_GAME_AUTO_DETECT_STREAK) {
		until = ktime_get_ns() + ZENITH_GAME_AUTO_ACTIVE_TTL_NS;
		WRITE_ONCE(zenith_game_auto_active_until_ns, until);
		z_policy->game_auto_streak = 0;
	}
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
	if (!strcmp(path, "predict_up"))
		return ZENITH_STAT_PREDICT_UP;
	if (!strcmp(path, "peak_prearm"))
		return ZENITH_STAT_PEAK_PREARM;
	if (!strcmp(path, "peak_rescue"))
		return ZENITH_STAT_PEAK_RESCUE;
	if (!strcmp(path, "peak_hyst"))
		return ZENITH_STAT_PEAK_HYST;
	if (!strcmp(path, "peer_ramp"))
		return ZENITH_STAT_PEER_RAMP;
	if (!strcmp(path, "migration_floor"))
		return ZENITH_STAT_MIGRATION_FLOOR;
	if (!strcmp(path, "psi_cpu_floor"))
		return ZENITH_STAT_PSI_CPU_FLOOR;
	if (!strcmp(path, "frame_overrun"))
		return ZENITH_STAT_FRAME_OVERRUN;
	return ZENITH_STAT_OTHER;
}

/* Cached "does this SoC have a dedicated BIG / mid cluster?" probe.
 *
 * On a 3+-cluster topology (1+3+4 et al) at least one cluster sits
 * strictly between the LITTLE and PRIME capacity bands, so its policy
 * is classified ZENITH_CLUSTER_BIG by zenith_update_cluster_rate_scale().
 * On a 2-cluster (true big.LITTLE) topology the lone non-LITTLE
 * cluster is at max_cap and gets classified ZENITH_CLUSTER_PRIME -- the
 * BIG class is unused.
 *
 * The prefer_silver_aware bump path needs to distinguish these two cases
 * so it can fire on the lowest non-LITTLE cluster in either topology
 * (BIG on tri-cluster, PRIME on 2-cluster) without wrongly inflating
 * up_threshold on PRIME when a separate BIG cluster also exists.
 *
 * Topology is invariant after boot, so the result is computed once on
 * the first call and cached.  capacity_orig is read via
 * arch_scale_cpu_capacity() which matches what
 * zenith_update_cluster_rate_scale() uses, keeping classification and
 * topology probe consistent on every SoC.
 */
static bool zenith_topology_has_big_class(void)
{
	/*
	 * State: 0 = unknown, 1 = false, 2 = true.  Encoding both the
	 * cached value and its validity in a single atomic_t means a
	 * concurrent caller cannot observe "valid" without also seeing
	 * the corresponding result -- side-stepping the
	 * smp_wmb / smp_rmb pairing that a separate (cached_result,
	 * cached_valid) pair would require on weakly-ordered ARM64.
	 * Topology is invariant after boot, so multiple racing first
	 * callers all compute the same result and converge.
	 */
	static atomic_t cached = ATOMIC_INIT(0);
	int snap = atomic_read(&cached);
	unsigned int little_thresh, big_cap = 0, second_cap = 0, cpu;
	bool has_big;

	if (snap)
		return snap == 2;

	little_thresh =
		(SCHED_CAPACITY_SCALE * ZENITH_CLUSTER_LITTLE_THRESH_PCT) / 100;

	for_each_possible_cpu(cpu) {
		unsigned int cap = arch_scale_cpu_capacity(cpu);

		if (cap > big_cap) {
			second_cap = big_cap;
			big_cap = cap;
		} else if (cap < big_cap && cap > second_cap) {
			second_cap = cap;
		}
	}

	/* A dedicated BIG class exists when there is a non-LITTLE
	 * capacity level strictly below the system maximum.  On
	 * 2-cluster phones second_cap is either zero (single non-LITTLE
	 * cluster) or below little_thresh (every non-max CPU was a
	 * LITTLE-equivalent).  On 3+-cluster phones second_cap sits
	 * comfortably above little_thresh and below big_cap.
	 */
	has_big = (second_cap >= little_thresh && second_cap < big_cap);
	atomic_set(&cached, has_big ? 2 : 1);
	return has_big;
}

/* Peak-return hysteresis (Patch E).  Pulled out of
 * zenith_get_next_freq() so the deeply-nested gating logic can be
 * expressed without overflowing checkpatch's max-tab limit.
 *
 * Pre: caller has computed freq through the lower freq-tier
 * passes.  Post: returns the freq with the soft floor applied
 * (or the input freq unchanged if the tier is disabled or the
 * cluster isn't in a peak-exit transition).  *tp_path is set to
 * "peak_hyst" iff the soft floor fired.
 */
static unsigned int
zenith_apply_peak_hysteresis(struct zenith_policy *z_policy,
			     struct cpufreq_policy *policy,
			     unsigned int freq, bool pin_to_target,
			     const char **tp_path)
{
	unsigned int hyst_streak =
		READ_ONCE(z_policy->tunables->peak_hysteresis_streak);
	unsigned int step_down_pct =
		READ_ONCE(z_policy->tunables->peak_step_down_pct);
	unsigned int prev, peak_thresh, anchor, floor_freq;

	if (!hyst_streak || !step_down_pct ||
	    !policy->max || pin_to_target) {
		z_policy->peak_hyst_anchor_freq = 0;
		z_policy->peak_low_streak = 0;
		return freq;
	}

	prev = z_policy->cached_raw_freq;
	peak_thresh = (policy->max / 100) *
		ZENITH_PEAK_HYSTERESIS_PEAK_THRESH_PCT;

	/* (Re)anchor on every sample where the previous
	 * cached_raw_freq sits in the peak class.  This lets the
	 * anchor track the cluster while it is pinned at peak, then
	 * survive the descent because the cap-to-floor write below
	 * pushes prev out of the peak class on subsequent ticks.
	 */
	if (prev >= peak_thresh) {
		z_policy->peak_hyst_anchor_freq = prev;
		z_policy->peak_low_streak = 0;
	}

	anchor = z_policy->peak_hyst_anchor_freq;
	if (!anchor)
		return freq;

	floor_freq = (anchor / 100) * step_down_pct;
	if (floor_freq > policy->max)
		floor_freq = policy->max;

	if (freq >= floor_freq) {
		/* Natural freq is at or above the soft floor; release
		 * the anchor.
		 */
		z_policy->peak_hyst_anchor_freq = 0;
		z_policy->peak_low_streak = 0;
		return freq;
	}

	if (z_policy->peak_low_streak >= hyst_streak) {
		/* Streak drained; release and let the natural descent
		 * resume.
		 */
		z_policy->peak_hyst_anchor_freq = 0;
		z_policy->peak_low_streak = 0;
		return freq;
	}

	if (z_policy->peak_low_streak < ZENITH_PEAK_HYSTERESIS_STREAK_MAX)
		z_policy->peak_low_streak++;
	*tp_path = "peak_hyst";
	return floor_freq;
}

/* Peer-ramp helpers (Patch D).  Translate a cluster_class to the
 * deadline atomic each side of the protocol cares about.
 *
 * peer_atomic_for() picks the slot that the *peer* of the given
 * cluster will read on its next eval.  An arming write goes here.
 *
 * self_atomic_for() picks the slot that the given cluster reads
 * itself.  This was written by the cluster's peer the last time
 * the peer ramped.
 *
 * Returns NULL for LITTLE on both, since LITTLE neither arms nor
 * is armed under this scheme (see the macro block at the top of
 * the file for why).
 */
static atomic64_t *
zenith_peer_ramp_peer_atomic(unsigned int cluster_class)
{
	switch (cluster_class) {
	case ZENITH_CLUSTER_BIG:
		return &zenith_peer_ramp_until_ns_prime;
	case ZENITH_CLUSTER_PRIME:
		return &zenith_peer_ramp_until_ns_big;
	default:
		return NULL;
	}
}

static atomic64_t *
zenith_peer_ramp_self_atomic(unsigned int cluster_class)
{
	switch (cluster_class) {
	case ZENITH_CLUSTER_BIG:
		return &zenith_peer_ramp_until_ns_big;
	case ZENITH_CLUSTER_PRIME:
		return &zenith_peer_ramp_until_ns_prime;
	default:
		return NULL;
	}
}

/* uclamp_min expressed as a percent of policy->max, suitable for
 * folding into the existing static-pct floors via max() at the
 * peer_ramp / migration_floor read sites (Patch M2).  Returns 0
 * when the policy has no uclamp_min set or when uclamp is not
 * compiled in (zenith_policy_uclamp_min() handles both cases),
 * which keeps the max() call a no-op.
 *
 * Independent of the master uclamp_min_respect knob; the new
 * per-tier bools at the call sites are gated separately.  Cheap:
 * one cache read, one mul + div.  Computed per call site rather
 * than once per zenith_get_next_freq() because both sites are
 * already short-circuited by their static-pct == 0 guard, and
 * the helper itself is called from inside an existing if-guard.
 */
static unsigned int
zenith_uclamp_min_pct_of_max(struct zenith_policy *z_policy)
{
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned long umin = zenith_policy_uclamp_min(z_policy);
	unsigned int max_cap;
	unsigned int umin_freq;

	if (!umin || !policy || !policy->max)
		return 0;
	max_cap = arch_scale_cpu_capacity(cpumask_first(policy->cpus));
	if (!max_cap)
		return 0;
	umin_freq = map_util_freq(umin, policy->cpuinfo.max_freq, max_cap);
	if (!umin_freq)
		return 0;
	return (umin_freq * 100U) / policy->max;
}

/* Effective peer_ramp window length, accounting for screen state
 * (Patch M3).  When the screen is on, the legacy peer_ramp_window_ms
 * applies.  When the screen is off, the shadow knob takes over.
 * Both reads are READ_ONCE so a concurrent sysfs write cannot tear
 * the value across the arming and floor-reading paths even though
 * those paths run on different CPUs.
 *
 * Returning 0 disables peer_ramp on the calling path: the arm
 * function bails on a 0 window; the floor-eval site treats 0 as
 * "no floor" via the existing tunable->peer_ramp_window_ms == 0
 * short-circuit (replaced here by the same check on the effective
 * value).  This is the design lever that makes peer_ramp_window_off_ms
 * == 0 fully suppress peer_ramp while the screen is off without
 * touching the existing screen-on path.
 */
static unsigned int
zenith_peer_ramp_effective_window_ms(const struct zenith_tunables *t)
{
	if (READ_ONCE(t->screen_state))
		return READ_ONCE(t->peer_ramp_window_ms);
	return READ_ONCE(t->peer_ramp_window_off_ms);
}

/* Stamp a deadline on the peer cluster's slot.  Called from the
 * three peak tiers (predict_up, peak_prearm, peak_rescue) right
 * after they decide to lift the cluster.  Cheap: one tunable
 * read, one switch, one atomic64_set.  Re-armings just bump the
 * deadline forward, so concurrent stamps from the same cluster
 * (e.g. predict_up on tick N then peak_prearm on tick N+1) end
 * up with the latest deadline winning, which is what we want.
 *
 * Gated entirely on the effective window length: when screen is
 * on this is peer_ramp_window_ms, when screen is off it is
 * peer_ramp_window_off_ms (default 0, so screen-off arms are
 * suppressed by default).  Either way, set the relevant knob to
 * 0 and this function is a couple of branches and a return.
 */
static void
zenith_peer_ramp_arm(struct zenith_policy *z_policy, u64 now_ns)
{
	unsigned int window_ms =
		zenith_peer_ramp_effective_window_ms(z_policy->tunables);
	atomic64_t *peer;

	if (!window_ms)
		return;
	peer = zenith_peer_ramp_peer_atomic(z_policy->cluster_class);
	if (!peer)
		return;
	atomic64_set(peer, now_ns + (u64)window_ms * NSEC_PER_MSEC);
}

/* Migration-arrival detector (Patch K1).  Called once per CPU per
 * update_util tick, after iowait_apply has folded its boost into
 * util but before kcpustat_blend or the wakeup-boost detector run.
 *
 * Compares util against the previous tick's value on the same CPU.
 * If the upward jump exceeds tunables->migration_jump_pct of
 * max_cap, treat it as evidence a task just landed here and stamp
 * z_policy->migration_in_until_ns with a deadline N ms in the
 * future.  N == migration_floor_window_ms.
 *
 * Always updates migration_prev_util so the next tick has a fresh
 * comparison baseline regardless of whether the threshold tripped.
 *
 * No-op when migration_jump_pct == 0 or max_cap == 0 (impossible
 * but cheap to guard).  Caller is responsible for whatever
 * locking the surrounding update_util path needs; this helper
 * does not take any.
 */
static void
zenith_migration_arrival_check(struct zenith_cpu *z_cpu,
			       unsigned long util, unsigned long max_cap,
			       struct zenith_policy *z_policy)
{
	unsigned int jump_pct =
		READ_ONCE(z_policy->tunables->migration_jump_pct);
	unsigned int window_ms;
	unsigned long prev = z_cpu->migration_prev_util;

	z_cpu->migration_prev_util = util;
	if (!jump_pct || !max_cap)
		return;
	if (util <= prev)
		return;
	if ((util - prev) * 100 < (unsigned long)jump_pct * max_cap)
		return;
	window_ms = READ_ONCE(z_policy->tunables->migration_floor_window_ms);
	if (!window_ms)
		return;
	z_policy->migration_in_until_ns =
		ktime_get_ns() + (u64)window_ms * NSEC_PER_MSEC;
}

/* Forward decl: zenith_tier_value() is the V2 tier-classifier
 * accessor, defined later in the file alongside the rest of the
 * auto_tune_v2 worker.  zenith_get_next_freq() (and a handful of
 * its sub-blocks below) read knobs through it, so we need the
 * prototype visible here.  Definition lives near the V2 worker so
 * the override-mask / tier-bit semantics are documented in one
 * place; the declaration just exposes the symbol earlier without
 * hoisting the whole body up out of context.
 */
static unsigned int zenith_tier_value(struct zenith_policy *z_policy,
				      unsigned int tunable,
				      unsigned long override_bit,
				      unsigned long tier_bit);

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

	/* In-kernel game detector tick.  Gated by the static branch
	 * (default FALSE while game_auto = 0) and the live tunables
	 * scalar (defends against a momentary tear during a sysfs
	 * store; the branch can be true while the scalar transitions
	 * back to 0).  Maintains the per-policy streak counter and
	 * renews the global zenith_game_auto_active_until_ns latch on
	 * sustained match.  See the ZENITH_DEFAULT_GAME_AUTO comment
	 * block.  Order: must follow the local declarations above and
	 * precede any other executable code in this function so the
	 * the latter is allowed to declare additional locals without
	 * tripping -Wdeclaration-after-statement.
	 */
	if (static_branch_unlikely(&zenith_game_auto_key) &&
	    READ_ONCE(z_policy->tunables->game_auto))
		zenith_policy_game_auto_tick(z_policy);

	{
		/* Screen-off glide tracking.  Detect 1 -> 0 / 0 -> 1
		 * transitions on tunables->screen_state and stamp
		 * screen_off_arm_ns at the moment we go to 0 so the
		 * glide block below can interpolate towards the cliff
		 * targets across screen_off_glide_ms.  Cleared on the
		 * way back up.  Default 0 leaves both sides unarmed
		 * which makes this a no-op.
		 */
		unsigned int cur_screen = z_policy->tunables->screen_state;

		if (z_policy->screen_state_last && !cur_screen)
			z_policy->screen_off_arm_ns = ktime_get_ns();
		else if (!z_policy->screen_state_last && cur_screen)
			z_policy->screen_off_arm_ns = 0;
		/* Patch K3 stale guard: clear any cached vblank
		 * timestamp on either edge so a multi-second sleep
		 * doesn't make the first event after resume look
		 * like a giant overrun.  Cheap unconditional store on
		 * a transition (rare, observable as a tunable flip),
		 * and harmless even when frame_overrun_slack_us is 0
		 * since the producer treats last_ns == 0 as "first
		 * event".
		 */
		if (z_policy->screen_state_last != cur_screen)
			atomic64_set(&zenith_last_vblank_ns, 0);
		/* Audit fix M7: on the 0 -> 1 (resume) edge, reset the
		 * V1 classifier counters and the V2 pending-window
		 * accumulator.  Without this, the first auto_tune window
		 * after a long suspend / blank would aggregate samples
		 * collected pre-suspend (when the screen was on and the
		 * workload was real) with a window-sized post-resume
		 * gap (where the device had been idle), producing a
		 * misleadingly low sat_pct and biasing V1 toward
		 * EFFICIENCY for one extra window after resume.
		 *
		 * Treat resume as a fresh measurement: clear the atomic
		 * sample counters and the pending-window state, leaving
		 * at_last_state / at_last_applied_state untouched (so
		 * the just-restored state survives the reset and the
		 * cooldown timer continues to gate further moves).
		 *
		 * Cheap: 4 atomic_sets and 1 unsigned-int store, only on
		 * a transition (not every tick).  No effect on the
		 * 1 -> 0 (suspend) edge -- those samples are still
		 * useful for the screen-off glide path.
		 */
		if (!z_policy->screen_state_last && cur_screen) {
			atomic_set(&z_policy->at_samples_total, 0);
			atomic_set(&z_policy->at_samples_saturated, 0);
			z_policy->at_last_events =
				atomic64_read(&zenith_auto_input_events);
			z_policy->at_pending_windows = 0;
		}
		z_policy->screen_state_last = cur_screen;
	}

	if (z_policy->tunables->screen_state == 0 && !uclamp_min_meaningful) {
		unsigned int glide_ms = zenith_glide_value(z_policy,
				z_policy->tunables->screen_off_glide_ms,
				z_policy->at_local_screen_off_glide_ms);
		u64 now = z_policy->screen_off_arm_ns ? ktime_get_ns() : 0;
		u64 elapsed_ns = (z_policy->screen_off_arm_ns &&
				  now > z_policy->screen_off_arm_ns) ?
				  now - z_policy->screen_off_arm_ns : 0;
		u64 glide_ns = (u64)glide_ms * NSEC_PER_MSEC;

		if (glide_ms && glide_ns && elapsed_ns < glide_ns) {
			/* Glide phase: ramp dynamic_up_thresh from the
			 * natural up_threshold (at the moment the screen
			 * went off) up to the legacy 95 cliff target
			 * across screen_off_glide_ms; mirror the same
			 * proportional ramp on dynamic_bias from the
			 * configured powersave_bias up to 500 (50 %%
			 * penalty).  Eliminates the cliff-on-cliff that
			 * happens when AOD / blanking handlers stamp
			 * screen_state=0 while userspace work is still
			 * winding down.  After glide_ms elapses the next
			 * tick re-enters this branch with elapsed_ns >=
			 * glide_ns and the legacy assignments below take
			 * effect verbatim.
			 */
			unsigned int floor =
				zenith_tunable_or_local(z_policy,
				    z_policy->tunables->up_threshold,
				    z_policy->at_effective_up_threshold);
			unsigned int bias_floor =
				z_policy->tunables->powersave_bias;
			u64 t256 = div64_u64(elapsed_ns * 256ULL, glide_ns);

			if (t256 > 256)
				t256 = 256;
			if (floor < 95)
				dynamic_up_thresh = floor +
				    (unsigned int)(((95U - floor) * t256) >> 8);
			else
				dynamic_up_thresh = floor;
			if (bias_floor < 500)
				dynamic_bias = bias_floor +
				    (unsigned int)(((500U - bias_floor) * t256)
							>> 8);
			else
				dynamic_bias = bias_floor;
		} else {
			dynamic_up_thresh = 95; /* Hard to wake up */
			dynamic_bias = 500;     /* 50% penalty */
		}
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
		if (zenith_glide_value(z_policy,
				z_policy->tunables->thermal_pressure_continuous,
				z_policy->at_local_thermal_pressure_continuous)) {
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
	 * and this policy belongs to the BIG / mid cluster, raise
	 * dynamic_up_thresh by prefer_silver_hot_bump_pct points
	 * (clamped to ZENITH_PREFER_SILVER_HOT_BUMP_MAX_PCT) so the
	 * big cluster down-clocks less aggressively during sustained
	 * UI / app workloads where prefer_silver is steering the
	 * light wake-ups onto the silver/LITTLE cluster.
	 *
	 * Fires on the lowest non-LITTLE cluster only:
	 *
	 *   - 3+-cluster topology (1+3+4 et al): cluster_class == BIG.
	 *     PRIME is excluded -- prefer_silver only redirects *light*
	 *     wake-ups onto silver (the heavy-task gate in
	 *     find_best_silver_cpu() rejects anything above
	 *     sysctl_heavy_task_thresh), so the work it hides from the
	 *     rest of the system is BIG-cluster work, never PRIME work.
	 *     Inflating up_threshold on PRIME would just delay
	 *     down-shifts on the highest-leakage cluster: pure power
	 *     tax with no perf return.
	 *   - 2-cluster topology (true big.LITTLE): no BIG class exists
	 *     and the lone non-LITTLE cluster is classified PRIME.  Fall
	 *     through to PRIME there so the bump still fires on the
	 *     cluster that absorbs the heavy work, exactly as before
	 *     this restriction was introduced.  The
	 *     zenith_topology_has_big_class() probe distinguishes the
	 *     two cases at runtime via capacity_orig, with the result
	 *     cached for the lifetime of the kernel.
	 *
	 * Also skipped whenever a harder override above has pinned
	 * dynamic_up_thresh strictly higher than the natural
	 * up_threshold (screen-off, thermal cliff, hispeed pin) --
	 * those values are absolute and must not be inflated further.
	 *
	 * The (dynamic_up_thresh <= natural) test deliberately allows
	 * the bump to ride on top of the variance-adaptive shaping
	 * lower in the same chain (which only ever lowers
	 * dynamic_up_thresh below natural), preserving its smoothing
	 * effect while restoring the climb resistance prefer_silver
	 * was eroding by hiding light load from this cluster.
	 */
	if (zenith_glide_value(z_policy,
			z_policy->tunables->prefer_silver_aware,
			z_policy->at_local_prefer_silver_aware) &&
	    (z_policy->cluster_class == ZENITH_CLUSTER_BIG ||
	     (z_policy->cluster_class == ZENITH_CLUSTER_PRIME &&
	      !zenith_topology_has_big_class())) &&
	    z_policy->ps_hit_rate_pct >=
		    z_policy->tunables->prefer_silver_hot_threshold_pct) {
		unsigned int natural = zenith_tunable_or_local(z_policy,
			z_policy->tunables->up_threshold,
			z_policy->at_effective_up_threshold);

		if (dynamic_up_thresh <= natural) {
			unsigned int bump_max =
			    z_policy->tunables->prefer_silver_hot_bump_pct;
			unsigned int rate     = z_policy->ps_hit_rate_pct;
			unsigned int thresh   =
			    z_policy->tunables->prefer_silver_hot_threshold_pct;
			unsigned int bump;

			if (bump_max > ZENITH_PREFER_SILVER_HOT_BUMP_MAX_PCT)
				bump_max = ZENITH_PREFER_SILVER_HOT_BUMP_MAX_PCT;

			/*
			 * Linear ramp: at hit_rate == threshold, bump = 0;
			 * at hit_rate == 100, bump = bump_max.  Replaces the
			 * step function (flat bump_max above threshold, 0
			 * below) so the tail of the ramp is smooth, a 51%
			 * hit rate doesn't yield the same up_threshold
			 * inflation as a 99% one, and the worst-case bump
			 * is unchanged from the previous fixed form.
			 *
			 * thresh >= 100 would make the denominator zero or
			 * negative; the outer >=-test already required
			 * rate >= thresh to enter this branch, so for
			 * thresh == 100 the only legal rate is also 100,
			 * collapsing the ramp to bump_max as an exact case.
			 * Treat thresh > 100 (impossible per the sysfs
			 * store handler's 0..100 clamp, but defensive)
			 * as no bump.
			 */
			if (rate <= thresh || thresh > 100)
				bump = 0;
			else if (thresh == 100)
				bump = bump_max;
			else
				bump = (bump_max * (rate - thresh)) /
				       (100u - thresh);

			if (dynamic_up_thresh + bump <= 95)
				dynamic_up_thresh += bump;
			else
				dynamic_up_thresh = 95;
		}
	}

	if (max_cap)
		tp_load_pct = (unsigned int)((util * 100) / max_cap);

	/* Patch H: stamp last_runnable_ns whenever the cluster has
	 * any util.  Read by the post-tier sleeper-tail shave to
	 * decide whether the cluster has been idle long enough to
	 * justify shaving the next freq decision.
	 */
	if (tp_load_pct > 0)
		z_policy->last_runnable_ns = ktime_get_ns();

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
			unsigned int gm = zenith_eff_game_mode(
				zenith_tunable_or_local(z_policy,
					z_policy->tunables->game_mode,
					z_policy->at_effective_game_mode));
			unsigned int boost_ceiling;
			unsigned int idle_streak_th =
				READ_ONCE(z_policy->tunables->boost_idle_streak);

			/* Patch F: persistent-idle preemption.  If a
			 * boost is still armed but the cluster has been
			 * idle for boost_idle_streak ticks, skip the
			 * boost on this tier-0 path so the lower tiers
			 * pick the freq.  The global until_ns is left
			 * armed; other policies still see the boost.
			 */
			if (idle_streak_th &&
			    z_policy->boost_idle_low_streak >= idle_streak_th) {
				atomic64_inc(&zenith_in_boosts_early_exit);
				z_policy->boost_idle_low_streak = 0;
				goto skip_input_boost;
			}

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

			/* Patch F: update the persistent-idle streak.
			 * Increment when the cluster's load_pct sits
			 * below boost_idle_thresh; reset otherwise.
			 * Capped at ZENITH_BOOST_IDLE_STREAK_MAX so a
			 * stuck-near-zero workload can't accumulate
			 * unbounded streak credit.
			 */
			{
				unsigned int idle_thresh =
					READ_ONCE(z_policy->tunables->boost_idle_thresh);

				if (idle_thresh && tp_load_pct < idle_thresh) {
					if (z_policy->boost_idle_low_streak <
					    ZENITH_BOOST_IDLE_STREAK_MAX)
						z_policy->boost_idle_low_streak++;
				} else {
					z_policy->boost_idle_low_streak = 0;
				}
			}

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
		} else {
			/* Boost expired naturally; drop any accumulated
			 * idle-streak credit so the next boost arming
			 * starts from zero (Patch F).
			 */
			z_policy->boost_idle_low_streak = 0;
		}
	} else {
		/* Boost feature gated off (input_boost_ms == 0,
		 * screen off, or input_boost_big_only excluded this
		 * cluster): clear the idle streak so a later re-enable
		 * sees a fresh count.
		 */
		z_policy->boost_idle_low_streak = 0;
	}
skip_input_boost:

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
			if (zenith_eff_game_mode(
					zenith_tunable_or_local(z_policy,
						z_policy->tunables->game_mode,
						z_policy->at_effective_game_mode)) >= 2)
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
		     zenith_eff_game_mode(
			zenith_tunable_or_local(z_policy,
				z_policy->tunables->game_mode,
				z_policy->at_effective_game_mode)) >= 2) &&
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
		if (z_policy->brutal_active) {
			unsigned int decay_ms = zenith_glide_value(z_policy,
				z_policy->tunables->brutal_decay_ms,
				z_policy->at_local_brutal_decay_ms);

			if (decay_ms) {
				if (decay_ms > ZENITH_BRUTAL_DECAY_MS_MAX)
					decay_ms =
						ZENITH_BRUTAL_DECAY_MS_MAX;
				z_policy->brutal_decay_arm_ms = decay_ms;
				z_policy->brutal_decay_until_ns =
					ktime_get_ns() +
					(u64)decay_ms * NSEC_PER_MSEC;
			}
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

	/* 2a'. Predictive up-shift via util-trend ring.  Lifts the
	 * cluster up to eff_hispeed_freq before the level-triggered
	 * hispeed tier (2b) catches a rising workload.  See the block
	 * comment above ZENITH_DEFAULT_PREDICT_UP_THRESH for the full
	 * rationale.
	 *
	 * Trigger:
	 *   - tunables->predict_up_thresh > 0 (gate);
	 *   - max_cap and policy->max non-zero (init guard);
	 *   - pin_to_target false (don't fight an explicit pin tier);
	 *   - eff_hispeed_freq configured and freq < eff_hispeed_freq;
	 *   - peak_starve_count == 0 (don't double-fire with rescue
	 *     / pre-arm during sustained-high regimes);
	 *   - peak_rescue_until_ns expired (don't refire inside a
	 *     rescue hold-down window);
	 *   - util_history_count >= predict_up_window (warm-up gate);
	 *   - delta_x256 = ((newest - oldest) * 256) / max_cap >=
	 *     predict_up_thresh.
	 *
	 * Effect: lift freq to eff_hispeed_freq, set tp_path
	 * "predict_up".  The hispeed tier (2b) sees freq already at
	 * the floor and naturally treats the lift as a continuation;
	 * the rescue tier (2c) sees freq lifted out of the starvation
	 * window and resets peak_starve_count on its first sample.
	 */
	if (z_policy->tunables->predict_up_thresh &&
	    max_cap && policy->max && !pin_to_target) {
		unsigned int window = z_policy->tunables->predict_up_window;
		unsigned int eff_hispeed = zenith_eff_hispeed_freq(z_policy);
		u64 now_ns = ktime_get_ns();

		if (window < ZENITH_PREDICT_UP_WINDOW_MIN)
			window = ZENITH_PREDICT_UP_WINDOW_MIN;
		else if (window > ZENITH_PREDICT_UP_WINDOW_MAX)
			window = ZENITH_PREDICT_UP_WINDOW_MAX;

		if (eff_hispeed && freq < eff_hispeed &&
		    z_policy->peak_starve_count == 0 &&
		    now_ns >= z_policy->peak_rescue_until_ns &&
		    z_policy->util_history_count >= window) {
			unsigned int idx = z_policy->util_history_idx;
			unsigned int newest_idx =
				(idx + ZENITH_PREDICT_UP_WINDOW_MAX - 1) %
				ZENITH_PREDICT_UP_WINDOW_MAX;
			unsigned int oldest_idx =
				(idx + ZENITH_PREDICT_UP_WINDOW_MAX - window) %
				ZENITH_PREDICT_UP_WINDOW_MAX;
			unsigned long newest = z_policy->util_history[newest_idx];
			unsigned long oldest = z_policy->util_history[oldest_idx];

			if (newest > oldest) {
				unsigned long delta = newest - oldest;
				unsigned int delta_x256 = (unsigned int)
					((delta * 256) / max_cap);

				if (delta_x256 >=
				    z_policy->tunables->predict_up_thresh) {
					freq = eff_hispeed;
					tp_path = "predict_up";
					/* Patch D: arm the peer cluster
					 * for cross-cluster IPC chains.
					 * No-op when peer_ramp_window_ms
					 * is 0 or this is LITTLE.
					 */
					zenith_peer_ramp_arm(z_policy, now_ns);
				}
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

	/* 2b'. Peak-headroom pre-arm.  Soft early intervention that
	 * lifts the cluster up to eff_hispeed_freq while the
	 * starvation streak is accumulating but has not yet crossed
	 * peak_headroom_starve_streak.  See peak_headroom_prearm in
	 * struct zenith_tunables for the full rationale.
	 *
	 * Trigger: gate is on, rescue gate is on (so peak_starve_count
	 * is being maintained at all), max_cap and policy->max non-
	 * zero, pin_to_target is false, eff_hispeed is configured,
	 * peak_starve_count > 0 (last sample was starving), current
	 * freq is below eff_hispeed, and lifting to eff_hispeed would
	 * actually escape the starvation floor (eff_hispeed >=
	 * floor_freq -- otherwise the rescue tier 2c would just
	 * re-flag the cluster as starving on the next sample, the
	 * hispeed pull would be wasted energy, and the rescue would
	 * be delayed).
	 *
	 * Effect: a one-shot per-episode graduated response.  Once the
	 * pull lands and freq >= floor_freq, 2c's starve check
	 * resolves false and peak_starve_count resets to 0, which is
	 * also the gate that lets this pre-arm fire, so the
	 * intervention won't repeat until a fresh starvation episode
	 * begins.  If eff_hispeed wasn't enough (peak_starve_count
	 * keeps climbing past the streak), the rescue tier in 2c
	 * fires as normal.
	 */
	if (z_policy->tunables->peak_headroom_rescue &&
	    z_policy->tunables->peak_headroom_prearm &&
	    max_cap && policy->max && !pin_to_target &&
	    z_policy->peak_starve_count > 0) {
		unsigned int eff_hispeed = zenith_eff_hispeed_freq(z_policy);
		unsigned int floor_pct =
			z_policy->tunables->peak_headroom_freq_floor_pct;
		unsigned int floor_freq =
			(policy->max / 100) * floor_pct;

		if (eff_hispeed && eff_hispeed >= floor_freq &&
		    freq < eff_hispeed) {
			freq = eff_hispeed;
			tp_path = "peak_prearm";
			/* Patch D: arm the peer cluster's deadline
			 * so its next eval picks up a soft floor.
			 */
			zenith_peer_ramp_arm(z_policy, ktime_get_ns());
		}
	}

	/* 2c. Peak-headroom rescue.  Watchdog tier that lifts the
	 * cluster off a sustained-high-util / sub-peak floor.  See
	 * ZENITH_DEFAULT_PEAK_HEADROOM_RESCUE for the full rationale.
	 *
	 * Skipped when:
	 *   - The tunable gate is off (peak_headroom_rescue == 0).
	 *   - max_cap or policy->max is 0 (init / no freq table).
	 *   - pin_to_target is true (input_boost / brutality already
	 *     pinned the cluster, no rescue needed).
	 *
	 * Streak-and-hold-down design mirrors the hispeed and brutality
	 * tiers above (hispeed_entry_count / brutal_entry_count) so
	 * single-sample noise can't fire a rescue, and a fired rescue
	 * can't restack on the very next tick before the cpufreq driver
	 * has a chance to apply the previous request.
	 */
	if (z_policy->tunables->peak_headroom_rescue &&
	    max_cap && policy->max && !pin_to_target) {
		unsigned int load_pct = (util * 100) / max_cap;
		unsigned int starve_load =
			z_policy->tunables->peak_headroom_starve_load_pct;
		unsigned int floor_pct =
			z_policy->tunables->peak_headroom_freq_floor_pct;
		unsigned int streak =
			z_policy->tunables->peak_headroom_starve_streak;
		unsigned int jump_pct =
			z_policy->tunables->peak_headroom_jump_pct;
		unsigned int hold_ms =
			z_policy->tunables->peak_headroom_hold_ms;
		unsigned int floor_freq =
			(policy->max / 100) * floor_pct;
		bool starving = (load_pct >= starve_load) &&
				(freq < floor_freq);

		if (starving) {
			if (z_policy->peak_starve_count <
			    ZENITH_PEAK_HEADROOM_STREAK_MAX)
				z_policy->peak_starve_count++;
		} else {
			z_policy->peak_starve_count = 0;
		}

		if (z_policy->peak_starve_count > streak) {
			u64 now_ns = ktime_get_ns();

			if (now_ns >= z_policy->peak_rescue_until_ns) {
				unsigned int rescue_freq =
					(policy->max / 100) * jump_pct;

				if (rescue_freq > policy->max || !jump_pct)
					rescue_freq = policy->max;
				if (freq < rescue_freq) {
					freq = rescue_freq;
					tp_path = "peak_rescue";
					z_policy->peak_rescue_until_ns = now_ns +
						(u64)hold_ms * NSEC_PER_MSEC;
					/* Patch D: arm peer cluster.  Same
					 * now_ns the rescue computed its
					 * own hold-down off of, so the
					 * deadlines are aligned.
					 */
					zenith_peer_ramp_arm(z_policy, now_ns);
				}
			}
		}
	} else {
		/* Tunable disabled, max_cap == 0, or pin_to_target
		 * already covers the cluster: drop streak credit so we
		 * don't carry partial entry across a disable cycle or a
		 * boost-pin window.
		 */
		z_policy->peak_starve_count = 0;
	}

	/* 3. Powersave Bias.
	 *
	 * Only apply when current util is below bias_load_threshold so
	 * heavy work is not penalised. A threshold of 100 keeps the legacy
	 * "always-bias" behaviour; 0 disables the bias entirely without
	 * having to also write powersave_bias=0.
	 *
	 * 3a. Screen-on bias softening.  When the screen is on
	 * (tunables->screen_state == 1), scale dynamic_bias down by
	 * screen_on_bias_pct / 100.  See ZENITH_DEFAULT_SCREEN_ON_BIAS_PCT
	 * for the rationale.  No-op when:
	 *   - screen_state != 1 (screen-off and uclamp-meaningful screen-
	 *     off paths are unaffected; the screen-off override at
	 *     dynamic_bias = 500 has already taken effect upstream).
	 *   - dynamic_bias == 0 (bias was already cleared upstream).
	 *   - screen_on_bias_pct >= 100 (no softening configured).
	 * The defensive >= 100 short-circuit also covers the case where
	 * userspace bypasses the sysfs store validator and writes a
	 * bogus value above 100.
	 */
	if (z_policy->tunables->screen_state == 1 && dynamic_bias &&
	    z_policy->tunables->screen_on_bias_pct < 100) {
		dynamic_bias = (dynamic_bias *
				z_policy->tunables->screen_on_bias_pct) / 100;
	}

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
		u64 boot_now = ktime_get_boottime_ns();
		u64 deadline_ns = (u64)z_policy->tunables->boot_boost_ms *
				  NSEC_PER_MSEC;

		/* boot_complete latch (sysfs-driven or in-kernel calm-detect).
		 * When raised, snap the deadline forward to the latch
		 * timestamp so the cluster transitions from Phase 1
		 * (pin to max) to Phase 2 (decay via boot_boost_decay_ms)
		 * immediately, instead of cliff-cutting to load-derived
		 * the way "write boot_boost_ms 0" would.
		 */
		if (atomic_read(&zenith_boot_complete)) {
			u64 latch_ns = READ_ONCE(zenith_boot_complete_ns);

			if (latch_ns && latch_ns < deadline_ns)
				deadline_ns = latch_ns;
		}

		if (boot_now < deadline_ns) {
			if (freq < policy->max) {
				freq = policy->max;
				tp_path = "boot_boost";
			}
		} else {
			/* Boot-boost decay tail.  Mirror of input_boost's
			 * decay phase but pinned to wall-clock boottime so
			 * the ramp shape is independent of how often
			 * zenith_get_next_freq() runs.  Linearly drops a
			 * floor from policy->max to policy->min across
			 * boot_boost_decay_ms after the hard pin window
			 * expires; eliminates the cliff that otherwise
			 * dumps boot freq from policy->max to load-derived
			 * the moment boot_boost_ms passes.
			 */
			unsigned int decay_ms = zenith_glide_value(z_policy,
				z_policy->tunables->boot_boost_decay_ms,
				z_policy->at_local_boot_boost_decay_ms);
			u64 decay_ns = (u64)decay_ms * NSEC_PER_MSEC;
			u64 elapsed_post = boot_now - deadline_ns;

			if (decay_ns && elapsed_post < decay_ns) {
				u64 span = policy->max - policy->min;
				u64 dropped = div64_u64(span * elapsed_post,
							decay_ns);
				unsigned int floor_freq;

				if (dropped >= span)
					floor_freq = policy->min;
				else
					floor_freq = policy->max -
						     (unsigned int)dropped;
				if (freq < floor_freq) {
					freq = floor_freq;
					tp_path = "boot_boost_decay";
				}
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

		/* frame_budget_us_auto: when 1, prefer the drm-side
		 * cached vblank period over the userspace-set value.
		 * Drivers populate it via zenith_set_drm_vblank_us().
		 * The auto path falls back to budget_us silently when
		 * the cache is empty (e.g. boot before drm has made
		 * its first commit), so the floor still works on
		 * stock-tuned systems where userspace writes the rate.
		 */
		if (zenith_glide_value(z_policy,
				READ_ONCE(z_policy->tunables->
					  frame_budget_us_auto),
				z_policy->at_local_frame_budget_us_auto)) {
			unsigned int auto_us = (unsigned int)
				atomic_read(&zenith_drm_vblank_us);

			if (auto_us)
				budget_us = auto_us;
		}

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
	 *
	 * Patch B: debounce the floor by render_floor_min_runtime_ms.
	 * Stamp the first-seen time on the false->true transition, clear
	 * on any sample where has_render is false.  Apply the floor only
	 * when the render thread has been observed for at least the
	 * debounce window; this filters one-shot SurfaceFlinger flushes
	 * and idle RenderEngine wakes that would otherwise bounce the
	 * cluster up to render_floor_pct for a single sample.
	 */
	if (ZENITH_FEATURE_ENABLED(render_aware) &&
	    z_policy->tunables->render_floor_pct) {
		bool has_render = zenith_policy_has_render(z_policy);
		unsigned int rf = (policy->max *
				   z_policy->tunables->render_floor_pct) /
				  100;
		unsigned int debounce_ms =
			READ_ONCE(z_policy->tunables->render_floor_min_runtime_ms);
		bool debounce_ok = true;
		u64 now_ns_render;

		if (rf > policy->max)
			rf = policy->max;

		if (has_render) {
			now_ns_render = ktime_get_ns();
			if (z_policy->render_first_seen_ns == 0)
				z_policy->render_first_seen_ns = now_ns_render;
			if (debounce_ms) {
				u64 thresh = (u64)debounce_ms * NSEC_PER_MSEC;

				if (now_ns_render -
				    z_policy->render_first_seen_ns < thresh)
					debounce_ok = false;
			}
		} else {
			z_policy->render_first_seen_ns = 0;
		}

		if (trace_zenith_render_floor_enabled())
			trace_zenith_render_floor(
				cpumask_first(policy->cpus),
				has_render,
				z_policy->tunables->render_floor_pct,
				(has_render && debounce_ok) ? rf : 0);
		if (has_render && debounce_ok && freq < rf) {
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

	/* 3c'''''. Peer-ramp soft floor (Patch D).
	 *
	 * If the peer cluster (BIG <-> PRIME) ramped to peak in the
	 * recent past via predict_up / peak_prearm / peak_rescue, it
	 * stamped a deadline on this cluster's slot.  While that
	 * deadline has not expired, hold a soft floor at
	 * peer_ramp_floor_pct of policy->max so the IPC chain doesn't
	 * spend its first few samples stalled at idle freq waiting
	 * for the level signal to land.
	 *
	 * The atomic is a single ktime_get_ns() compare; both the
	 * window and floor knobs short-circuit when 0.  pin_to_target
	 * paths bypass: input_boost / brutality already pin the
	 * cluster higher, so layering a floor under them is wasted
	 * arithmetic.  LITTLE has no peer atomic so the lookup
	 * returns NULL and the tier is a single branch on that
	 * cluster.
	 */
	if (!pin_to_target && policy->max &&
	    zenith_peer_ramp_effective_window_ms(z_policy->tunables) &&
	    z_policy->tunables->peer_ramp_floor_pct) {
		atomic64_t *self =
			zenith_peer_ramp_self_atomic(z_policy->cluster_class);

		if (self) {
			u64 until = (u64)atomic64_read(self);
			u64 now_ns = ktime_get_ns();

			if (now_ns < until) {
				struct zenith_tunables *t = z_policy->tunables;
				unsigned int eff_pct = t->peer_ramp_floor_pct;
				unsigned int floor;

				/* Patch M2: optionally fold uclamp_min
				 * into the peer_ramp floor.  Helper returns
				 * 0 when no task has uclamp_min set, so
				 * the max() is a no-op in the common case.
				 */
				if (READ_ONCE(t->peer_ramp_uclamp_min_respect)) {
					unsigned int umin_pct =
						zenith_uclamp_min_pct_of_max(z_policy);

					if (umin_pct > eff_pct)
						eff_pct = umin_pct;
				}
				floor = (policy->max * eff_pct) / 100;
				if (floor > policy->max)
					floor = policy->max;
				if (freq < floor) {
					freq = floor;
					tp_path = "peer_ramp";
				}
			}
		}
	}

	/* 3c''''''. Migration-arrival soft floor (Patch K1).
	 *
	 * If a per-CPU update_util tick observed a util jump
	 * exceeding migration_jump_pct of max_capacity, it stamped a
	 * deadline on this policy's migration_in_until_ns.  While
	 * that deadline has not expired, hold a soft floor at
	 * migration_floor_pct of policy->max so the destination
	 * cluster runs at a sensible freq during the inbound task's
	 * PELT warm-up rather than picking idle freq from the
	 * stale-low aggregate util signal.
	 *
	 * Same pin_to_target bypass as peer_ramp: input_boost /
	 * brutality already pin freq higher, so layering a soft
	 * floor under them is wasted arithmetic.  Both knobs
	 * short-circuit when 0 (jump_pct == 0 means stamping is also
	 * off; floor_pct == 0 keeps stamping but suppresses the
	 * floor here so userspace can correlate stats vs. effect).
	 */
	{
		/* Patch L: gate the K1 read through the V2 tier
		 * accessor.  In LATENCY / FRAME / GAME states the
		 * tier mask returns the profile-set jump_pct /
		 * floor_pct unchanged; in EFFICIENCY / BALANCED /
		 * THERMAL_RECOVERY states the V2 worker has cleared
		 * the migration tier bit and the accessor returns 0,
		 * which short-circuits the floor below.  User sysfs
		 * overrides bypass the V2 gate per zenith_tier_value().
		 */
		unsigned int eff_jump = zenith_tier_value(z_policy,
				z_policy->tunables->migration_jump_pct,
				ZENITH_AT_OVERRIDE_MIGRATION_JUMP,
				ZENITH_AT_TIER_MIGRATION);
		unsigned int eff_floor_pct = zenith_tier_value(z_policy,
				z_policy->tunables->migration_floor_pct,
				ZENITH_AT_OVERRIDE_MIGRATION_FLOOR_PCT,
				ZENITH_AT_TIER_MIGRATION);

		if (!pin_to_target && policy->max && eff_jump &&
		    eff_floor_pct) {
			u64 until = z_policy->migration_in_until_ns;

			if (until && ktime_get_ns() < until) {
				struct zenith_tunables *t = z_policy->tunables;
				unsigned int eff_pct = eff_floor_pct;
				unsigned int floor;

				/* Patch M2: optionally fold uclamp_min into
				 * the migration_floor.  Same shape as the
				 * peer_ramp variant above; the helper
				 * returns 0 when no task on the policy has
				 * uclamp_min set, so max() is a no-op.
				 */
				if (READ_ONCE(t->migration_floor_uclamp_min_respect)) {
					unsigned int umin_pct =
						zenith_uclamp_min_pct_of_max(z_policy);

					if (umin_pct > eff_pct)
						eff_pct = umin_pct;
				}
				floor = (policy->max * eff_pct) / 100;
				if (floor > policy->max)
					floor = policy->max;
				if (freq < floor) {
					freq = floor;
					tp_path = "migration_floor";
				}
			}
		}
	}

	/* 3c'''''''. PSI-CPU sustained-pressure floor (Patch K2).
	 *
	 * When the system-wide PSI_CPU_SOME 10s EWMA is at or above
	 * tunables->psi_cpu_floor_thresh and a hispeed freq is
	 * configured, lift to it.  This addresses the workload class
	 * where aggregate util sits below hispeed-entry threshold
	 * but PSI shows lots of queueing -- multi-app multitasking,
	 * gaming + background sync, screen-record + foreground app
	 * -- which the existing util-driven tiers don't catch
	 * because util is "just running" rather than "running with
	 * waiters".
	 *
	 * Tier short-circuits in three places: feature-gate off,
	 * tunable == 0, eff_hispeed_freq() == 0.  pin_to_target
	 * paths bypass for the same reason as the other floor tiers
	 * (input_boost / brutality already pin higher).  The
	 * 10s-EWMA smoothing of the underlying signal is by design:
	 * predict_up / peak_rescue cover the sub-second case;
	 * this tier covers the steady-state queueing case.
	 */
	{
		/* Patch L: V2-gated K2 read.  Armed in LATENCY only
		 * (queueing during latency-sensitive workloads is
		 * exactly the case the floor was designed for); not
		 * armed during FRAME / GAME because the 10s EWMA
		 * smoothing would be a cross-talk signal during
		 * gameplay.
		 */
		unsigned int eff_thresh = zenith_tier_value(z_policy,
				z_policy->tunables->psi_cpu_floor_thresh,
				ZENITH_AT_OVERRIDE_PSI_CPU_FLOOR,
				ZENITH_AT_TIER_PSI_CPU_FLOOR);

		if (!pin_to_target &&
		    ZENITH_FEATURE_ENABLED(psi_aware) &&
		    eff_thresh) {
			if (zenith_psi_cpu_some_pct() >= eff_thresh) {
				unsigned int floor =
					zenith_eff_hispeed_freq(z_policy);

				if (floor && floor > policy->max)
					floor = policy->max;
				if (floor && freq < floor) {
					freq = floor;
					tp_path = "psi_cpu_floor";
				}
			}
		}
	}

	/* 3c''''''''. Frame-overrun rescue (Patch K3).
	 *
	 * If zenith_drm_vblank_event() observed a vblank gap wider
	 * than the configured budget, it stamped the governor-wide
	 * zenith_frame_overrun_until_ns deadline.  While that
	 * deadline holds, every cluster's eval lifts to a soft
	 * floor at frame_overrun_floor_pct of policy->max.
	 *
	 * Symmetric to peer_ramp -- the producer is governor-wide
	 * (frame events are observed at the display layer above
	 * the cluster partition), the read happens on every
	 * cluster's eval, and either of the per-policy knobs being
	 * 0 short-circuits the read.  The slack_us cache being 0
	 * means no overrun event ever stamped the deadline, so the
	 * read is effectively a no-op anyway -- the explicit
	 * floor_pct gate just avoids the atomic_read in that case.
	 */
	{
		/* Patch L: V2-gated K3 read.  Armed in FRAME / GAME
		 * states (where vblank-driven floors actually make
		 * sense); disarmed elsewhere.  Note the producer
		 * (zenith_drm_vblank_event()) is governor-wide and
		 * keeps stamping the deadline regardless -- the V2
		 * gate is only on the consumer side.  An overrun
		 * stamped during a non-FRAME state simply does not
		 * lift this cluster's freq for the deadline window.
		 */
		unsigned int eff_slack = zenith_tier_value(z_policy,
				z_policy->tunables->frame_overrun_slack_us,
				ZENITH_AT_OVERRIDE_FRAME_OVR_SLACK,
				ZENITH_AT_TIER_FRAME_OVERRUN);
		unsigned int eff_floor_pct = zenith_tier_value(z_policy,
				z_policy->tunables->frame_overrun_floor_pct,
				ZENITH_AT_OVERRIDE_FRAME_OVR_FLOOR,
				ZENITH_AT_TIER_FRAME_OVERRUN);

		if (!pin_to_target && policy->max && eff_slack &&
		    eff_floor_pct) {
			u64 until =
			      (u64)atomic64_read(&zenith_frame_overrun_until_ns);

			if (until && ktime_get_ns() < until) {
				unsigned int floor_pct = eff_floor_pct;
				unsigned int deep_streak =
					READ_ONCE(z_policy->tunables->frame_overrun_deep_streak);
				unsigned int floor;

				/* Patch M5: deep tier.  After deep_streak
				 * consecutive overruns the floor escalates
				 * to frame_overrun_deep_floor_pct (default
				 * 100%).  deep_streak == 0 short-circuits
				 * the comparison and the streak atomic is
				 * never read.  The lower bound of max(...)
				 * with eff_floor_pct guarantees the deep
				 * tier never produces a *lower* floor than
				 * the standard K3 floor would.
				 */
				if (deep_streak &&
				    (unsigned int)atomic_read(&zenith_frame_overrun_streak) >=
				    deep_streak) {
					unsigned int deep_pct = READ_ONCE(
						z_policy->tunables->frame_overrun_deep_floor_pct);

					if (deep_pct > floor_pct)
						floor_pct = deep_pct;
				}
				floor = (policy->max * floor_pct) / 100;
				if (floor > policy->max)
					floor = policy->max;
				if (freq < floor) {
					freq = floor;
					tp_path = "frame_overrun";
				}
			}
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

	/* 3e'. PSI-mem light cap (Patch M1).  Sits one tier above the
	 * existing PSI-cap-at-hispeed (3e below).  When psi_aware=1
	 * AND the V2 PSI_MEM_CAP tier is armed (or the user has
	 * sysfs-overridden the thresh) AND
	 * zenith_psi_mem_some_pct() >= psi_mem_cap_thresh, stamp the
	 * per-policy psi_mem_cap_until_ns deadline.  While the
	 * deadline holds, cap final freq at psi_mem_cap_pct of
	 * policy->max.  Once the EWMA falls and the window lapses,
	 * the cap releases without further hysteresis.
	 *
	 * Cheap when off: psi_aware == 0 short-circuits via
	 * ZENITH_FEATURE_ENABLED; eff_thresh == 0 from V2 disarm
	 * short-circuits the EWMA read; pin_to_target paths skip
	 * the entire block.  The deadline read is a single field
	 * compare per eval, identical pattern to migration_floor.
	 */
	{
		unsigned int eff_thresh = zenith_tier_value(z_policy,
				z_policy->tunables->psi_mem_cap_thresh,
				ZENITH_AT_OVERRIDE_PSI_MEM_CAP_THRESH,
				ZENITH_AT_TIER_PSI_MEM_CAP);

		if (!pin_to_target &&
		    ZENITH_FEATURE_ENABLED(psi_aware) &&
		    eff_thresh && policy->max) {
			unsigned int cap_pct = READ_ONCE(
				z_policy->tunables->psi_mem_cap_pct);
			unsigned int win_ms = READ_ONCE(
				z_policy->tunables->psi_mem_cap_window_ms);
			u64 now_ns = ktime_get_ns();

			/* Arm path: stamp the deadline whenever the
			 * EWMA is currently above thresh.  Subsequent
			 * ticks while the EWMA stays above thresh keep
			 * pushing the deadline forward; once it drops,
			 * the deadline holds the cap until win_ms has
			 * lapsed.  Stamping before the cap apply means
			 * a cap that just released and a new spike both
			 * land cleanly.
			 */
			if (zenith_psi_mem_some_pct() >= eff_thresh)
				z_policy->psi_mem_cap_until_ns =
					now_ns + (u64)win_ms * NSEC_PER_MSEC;

			if (cap_pct >= ZENITH_PSI_MEM_CAP_PCT_MIN &&
			    z_policy->psi_mem_cap_until_ns &&
			    now_ns < z_policy->psi_mem_cap_until_ns) {
				unsigned int psi_cap =
					(policy->max * cap_pct) / 100;

				if (psi_cap < policy->min)
					psi_cap = policy->min;
				if (psi_cap > policy->max)
					psi_cap = policy->max;
				if (freq > psi_cap) {
					freq = psi_cap;
					tp_path = "psi_mem_cap_light";
				}
			}
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

	/* X. Peak-return hysteresis (Patch E).
	 *
	 * If the previous evaluation pinned the cluster at peak class
	 * (cached_raw_freq >= ZENITH_PEAK_HYSTERESIS_PEAK_THRESH_PCT
	 * of policy->max) and the current freq wants to drop below the
	 * soft floor (prev * peak_step_down_pct / 100), hold the soft
	 * floor for the next peak_hysteresis_streak samples then
	 * release.  pin_to_target paths (input_boost full-pin,
	 * brutality, climb_step) bypass this tier so the user-
	 * experience tier wins.
	 *
	 * Cost in the disabled path is one branch; in the enabled
	 * path it's a few unsigned multiplies and one streak compare.
	 */
	freq = zenith_apply_peak_hysteresis(z_policy, policy, freq,
					    pin_to_target, &tp_path);

	/* Patch H: sleeper-tail shaving.  When the cluster has been
	 * idle for at least sleeper_tail_thresh_us microseconds and
	 * the freq we'd otherwise pick is above policy->min, shave
	 * the freq by sleeper_tail_pct / 100 (clamped at
	 * policy->min).  Bypassed when pin_to_target is set so user-
	 * experience boosts always win.
	 *
	 * thresh_us == 0 disables the tier; pct == 100 makes the
	 * shave a no-op so we early-out to avoid the multiply on
	 * the common case.
	 */
	if (!pin_to_target && freq > policy->min) {
		unsigned int thresh_us =
			READ_ONCE(z_policy->tunables->sleeper_tail_thresh_us);
		unsigned int shave_pct =
			READ_ONCE(z_policy->tunables->sleeper_tail_pct);

		if (thresh_us && shave_pct &&
		    shave_pct < ZENITH_SLEEPER_TAIL_PCT_MAX) {
			u64 now_ns = ktime_get_ns();
			u64 idle_ns = now_ns - z_policy->last_runnable_ns;
			u64 thresh_ns = (u64)thresh_us * NSEC_PER_USEC;

			if (idle_ns >= thresh_ns) {
				unsigned int shaved =
					(freq / 100) * shave_pct;

				if (shaved < policy->min)
					shaved = policy->min;
				if (shaved < freq) {
					freq = shaved;
					tp_path = "sleeper_tail";
				}
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
		/* Patch J: stamp the per-policy last decision tag on
		 * the cache-hit path too so the sysfs node reflects
		 * the most recent path even when the cache shortcut
		 * wins.  WRITE_ONCE pairs with READ_ONCE in the show
		 * handler.
		 */
		WRITE_ONCE(z_policy->last_decision_path, tp_path);
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
	/* Patch J: stamp the per-policy last decision tag.  Pairs
	 * with READ_ONCE in the last_decision_path sysfs handler.
	 */
	WRITE_ONCE(z_policy->last_decision_path, tp_path);

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

	/* Push the current util sample into the predict_up trend ring
	 * so the next eval can compare a window of samples and decide
	 * whether to fire tier 2a'.  Done unconditionally (not gated
	 * on tunables->predict_up_thresh) so flipping the tunable on
	 * after a quiet period doesn't observe a ring of unwritten
	 * zeroes.  util_history_count saturates at the ring size so
	 * the value is a clean "are we warmed up" gate.
	 */
	{
		unsigned int idx = z_policy->util_history_idx;

		z_policy->util_history[idx] = util;
		z_policy->util_history_idx =
			(idx + 1) % ZENITH_PREDICT_UP_WINDOW_MAX;
		if (z_policy->util_history_count <
		    ZENITH_PREDICT_UP_WINDOW_MAX)
			z_policy->util_history_count++;
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
	zenith_migration_arrival_check(z_cpu, util, max_cap, z_policy);
	if (READ_ONCE(tunables->wakeup_boost) && max_cap) {
		unsigned int cur_pct = (unsigned int)((util * 100) / max_cap);
		unsigned int prev_pct = z_cpu->wakeup_prev_util ?
			(unsigned int)((z_cpu->wakeup_prev_util * 100) /
				       max_cap) : 0;

		if (prev_pct < ZENITH_WAKEUP_IDLE_THRESH_PCT &&
		    cur_pct >= ZENITH_WAKEUP_BUSY_THRESH_PCT) {
			unsigned int ms = zenith_glide_value(z_policy,
				READ_ONCE(tunables->wakeup_boost_ms),
				z_policy->at_local_wakeup_boost_ms);

			z_cpu->wakeup_boost_ticks = ZENITH_WAKEUP_BOOST_TICKS;
			if (ms) {
				if (ms > ZENITH_WAKEUP_BOOST_MS_MAX)
					ms = ZENITH_WAKEUP_BOOST_MS_MAX;
				z_cpu->wakeup_boost_until_ns =
					ktime_get_ns() +
					(u64)ms * NSEC_PER_MSEC;
			}
		}
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
			zenith_migration_arrival_check(j_z_cpu, j_util,
						       j_max, z_policy);
			if (READ_ONCE(tunables->wakeup_boost) && j_max) {
				unsigned int cur_pct =
					(unsigned int)((j_util * 100) / j_max);
				unsigned int prev_pct =
					j_z_cpu->wakeup_prev_util ?
					(unsigned int)((j_z_cpu->wakeup_prev_util *
							100) / j_max) : 0;

				if (prev_pct < ZENITH_WAKEUP_IDLE_THRESH_PCT &&
				    cur_pct >= ZENITH_WAKEUP_BUSY_THRESH_PCT) {
					unsigned int ms = zenith_glide_value(
						z_policy,
						READ_ONCE(
						 tunables->wakeup_boost_ms),
						z_policy->
						 at_local_wakeup_boost_ms);

					j_z_cpu->wakeup_boost_ticks =
						ZENITH_WAKEUP_BOOST_TICKS;
					if (ms) {
						if (ms > ZENITH_WAKEUP_BOOST_MS_MAX)
							ms = ZENITH_WAKEUP_BOOST_MS_MAX;
						j_z_cpu->wakeup_boost_until_ns =
							ktime_get_ns() +
							(u64)ms * NSEC_PER_MSEC;
					}
				}
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

	/*
	 * Disable transition fast path: when sysctl_prefer_silver was
	 * flipped to 0 mid-run, the global hit / miss counters stop
	 * incrementing.  The (!total_delta) branch below would still
	 * correctly zero the cached rate -- but only on the next
	 * worker window, leaving the previous "hot" reading active
	 * for one full classifier cycle's worth of zenith_get_next_freq()
	 * calls.  Drop the cache immediately on observing the disable
	 * so the up-threshold bump path also flips off in the same
	 * window the user expected.
	 *
	 * The previous-snapshot fields are zeroed too so re-enabling
	 * prefer_silver later does not see a stale baseline that would
	 * make the next window's delta artificially large.
	 */
	if (!READ_ONCE(sysctl_prefer_silver)) {
		z_policy->ps_hit_rate_pct = 0;
		z_policy->ps_prev_hit     = 0;
		z_policy->ps_prev_miss    = 0;
		return;
	}

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

/* V3 self-calibration tail.
 *
 * Called from zenith_auto_tune_work() once per V1 window when V3 is
 * enabled (zenith_auto_tune_v3_key TRUE) and the auto_tune_v3 scalar
 * is non-zero (defended against a momentary tear during a sysfs
 * store).  Walks the per-policy at_log ring, counts V2 state
 * transitions, and -- if the wall-clock interval has elapsed since
 * the last calibration -- updates at_v3_hyst_offset / at_v3_cool_offset
 * within the bounded range [ZENITH_AT_V3_OFFSET_MIN,
 * ZENITH_AT_V3_OFFSET_MAX].
 *
 * Mode 1 (OBSERVE) updates at_v3_last_transitions and exposes the
 * value via auto_tune_v3_state but never adjusts the offsets.  Mode 2
 * (APPLY) does both.
 *
 * Cost: at most ZENITH_AT_LOG_NR (=16) loads and a small bounded
 * amount of arithmetic, gated by the wall-clock interval check
 * (default 60 s).  Single-threaded with the v2 worker so no locking
 * is required for the per-policy fields; sysfs *_show readers are
 * serialised under the gov_attr_set rwsem.
 */
static void zenith_at_v3_calibrate(struct zenith_policy *z_policy,
				   unsigned int mode)
{
	struct zenith_tunables *t = z_policy->tunables;
	u64 now = ktime_get_boottime_ns();
	u64 interval_ns;
	unsigned int interval_ms;
	unsigned int transitions = 0;
	unsigned int prev_state;
	unsigned int idx, count, head;
	bool has_prev = false;

	interval_ms = READ_ONCE(t->auto_tune_v3_interval_ms);
	if (interval_ms < ZENITH_AT_V3_INTERVAL_MIN_MS)
		interval_ms = ZENITH_AT_V3_INTERVAL_MIN_MS;
	if (interval_ms > ZENITH_AT_V3_INTERVAL_MAX_MS)
		interval_ms = ZENITH_AT_V3_INTERVAL_MAX_MS;
	interval_ns = (u64)interval_ms * NSEC_PER_MSEC;

	if (z_policy->at_v3_last_calib_ns &&
	    now - z_policy->at_v3_last_calib_ns < interval_ns)
		return;

	count = z_policy->at_log_count;
	head = z_policy->at_log_head;
	prev_state = 0;

	/* Walk oldest -> newest. */
	for (idx = 0; idx < count; idx++) {
		unsigned int slot;
		struct zenith_at_log_entry *e;

		if (count < ZENITH_AT_LOG_NR)
			slot = idx;
		else
			slot = (head + idx) % ZENITH_AT_LOG_NR;

		e = &z_policy->at_log[slot];
		if (!has_prev) {
			prev_state = e->v2_to_state;
			has_prev = true;
			continue;
		}
		if (e->v2_to_state != prev_state) {
			transitions++;
			prev_state = e->v2_to_state;
		}
	}

	z_policy->at_v3_last_transitions = transitions;
	z_policy->at_v3_last_calib_ns = now;

	if (mode != ZENITH_AT_V3_MODE_APPLY)
		return;

	/* Apply bounded nudge to offsets. */
	if (transitions >= ZENITH_AT_V3_THRASH_HI) {
		if (z_policy->at_v3_hyst_offset < ZENITH_AT_V3_OFFSET_MAX)
			z_policy->at_v3_hyst_offset++;
		if (z_policy->at_v3_cool_offset < ZENITH_AT_V3_OFFSET_MAX)
			z_policy->at_v3_cool_offset++;
	} else if (transitions <= ZENITH_AT_V3_THRASH_LO) {
		if (z_policy->at_v3_hyst_offset > ZENITH_AT_V3_OFFSET_MIN)
			z_policy->at_v3_hyst_offset--;
		if (z_policy->at_v3_cool_offset > ZENITH_AT_V3_OFFSET_MIN)
			z_policy->at_v3_cool_offset--;
	}
}

/* Effective hysteresis_windows / cooldown_windows after V3 nudge.
 *
 * Returns the unmodified base when V3 is OFF or in OBSERVE mode.  When
 * V3 mode is APPLY (== 2), adds the per-policy signed offset and clamps
 * the result to [1, ZENITH_AT_*_WINDOWS_MAX].  >=1 floor preserves the
 * V2 state machine invariant that at least one window of agreement is
 * required before a state change commits.
 */
static inline unsigned int
zenith_at_eff_hyst_windows(struct zenith_policy *z_policy, unsigned int base)
{
	int v;

	if (!static_branch_unlikely(&zenith_auto_tune_v3_key))
		return base;
	if (READ_ONCE(z_policy->tunables->auto_tune_v3) !=
	    ZENITH_AT_V3_MODE_APPLY)
		return base;

	v = (int)base + (int)z_policy->at_v3_hyst_offset;
	if (v < 1)
		v = 1;
	if (v > ZENITH_AT_HYSTERESIS_WINDOWS_MAX)
		v = ZENITH_AT_HYSTERESIS_WINDOWS_MAX;
	return (unsigned int)v;
}

static inline unsigned int
zenith_at_eff_cool_windows(struct zenith_policy *z_policy, unsigned int base)
{
	int v;

	if (!static_branch_unlikely(&zenith_auto_tune_v3_key))
		return base;
	if (READ_ONCE(z_policy->tunables->auto_tune_v3) !=
	    ZENITH_AT_V3_MODE_APPLY)
		return base;

	v = (int)base + (int)z_policy->at_v3_cool_offset;
	if (v < 1)
		v = 1;
	if (v > ZENITH_AT_COOLDOWN_WINDOWS_MAX)
		v = ZENITH_AT_COOLDOWN_WINDOWS_MAX;
	return (unsigned int)v;
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
	z_policy->at_local_glides_active = false;
	z_policy->at_local_tiers_active = false;
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

/* zenith_glide_value - auto_tune_v2_glides accessor for round-U-z10 knobs.
 *
 * Semantics (different from zenith_tunable_or_local!):
 *   - If user wrote a non-zero tunable value, that wins outright --
 *     always, regardless of whether glides are active.
 *   - If the user value is 0 (the default for all seven glide knobs)
 *     AND auto_tune_v2_glides is on AND the V2 worker has run at
 *     least once (at_local_glides_active == true), return the
 *     V2-derived value.
 *   - Otherwise return 0 (== legacy behaviour: the consumer's
 *     "feature off" path).
 *
 * Read in the freq-update hot path; the non-zero short-circuit
 * keeps it one branch + one load on the common case (user has
 * tuned the knob explicitly).
 */
static unsigned int zenith_glide_value(struct zenith_policy *z_policy,
				       unsigned int tunable,
				       unsigned int local)
{
	if (tunable)
		return tunable;
	if (READ_ONCE(z_policy->tunables->auto_tune_v2_glides) &&
	    z_policy->at_local_glides_active)
		return local;
	return 0;
}

/* zenith_tier_value - V2 tier-classifier accessor for Patch L knobs.
 *
 * Different shape from zenith_glide_value():
 *
 *   1. User sysfs override always wins.  When the matching bit is
 *      set in tunables->auto_tune_override_mask the read returns
 *      the tunable value verbatim and skips all V2 logic.  Profile
 *      changes clear the entire mask (zenith_apply_profile() at
 *      line ~10630), so a profile flip rearms V2 even after the
 *      user has poked individual tiers.
 *
 *   2. When auto_tune_v2_tiers is 0 OR at_local_tiers_active is
 *      false the read returns the tunable verbatim -- this is the
 *      pre-Patch-L code path, byte-identical.
 *
 *   3. Otherwise (V2 active, tier mask current) the read returns
 *      the tunable iff the matching tier bit is set in the V2's
 *      armed mask, else 0.  "Disarmed" means "treat as off for
 *      this V2 window" -- a knob the user enabled via profile but
 *      that V2 has decided is not appropriate for the current
 *      classified state.
 *
 * Read in the eval hot path; the override-mask short-circuit is
 * one branch + one load on the common case (no override).
 */
static unsigned int zenith_tier_value(struct zenith_policy *z_policy,
				      unsigned int tunable,
				      unsigned long override_bit,
				      unsigned long tier_bit)
{
	struct zenith_tunables *t = z_policy->tunables;

	if (t->auto_tune_override_mask & override_bit)
		return tunable;
	if (!READ_ONCE(t->auto_tune_v2_tiers))
		return tunable;
	if (!z_policy->at_local_tiers_active)
		return tunable;
	return (z_policy->at_local_tier_armed_mask & tier_bit) ?
		tunable : 0;
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

/* zenith_at_apply_glides - V2 driver for the round-U-z10 glide knobs.
 *
 * Called from the V2 worker tail (just after zenith_at_apply_actions)
 * when auto_tune_v2 && auto_tune_v2_glides.  Populates per-policy
 * effective copies of the seven glide knobs from the current state +
 * the at_last_flags bitmap so the freq-update hot path can fall
 * through to them when the user-set tunable is 0.
 *
 * Mapping (state -> effective values):
 *
 *   ZENITH_AT_STATE_LATENCY:
 *     brutal_decay_ms       = ZENITH_AT_GLIDE_BRUTAL_DECAY_MS
 *     wakeup_boost_ms       = ZENITH_AT_GLIDE_WAKEUP_BOOST_MS
 *
 *   ZENITH_AT_STATE_THERMAL_RECOVERY:
 *     thermal_pressure_continuous = 1
 *
 *   ZENITH_AT_STATE_EFFICIENCY / BALANCED:
 *     prefer_silver_aware    = 1 (when ps hit-rate >= configured
 *                                 threshold; gated by the existing
 *                                 ps_hit_rate_pct snapshot)
 *
 *   any state with (flags & ZENITH_AT_FLAG_FRAME) || game_mode:
 *     wakeup_boost_ms        = ZENITH_AT_GLIDE_WAKEUP_BOOST_MS
 *     frame_budget_us_auto   = 1 (when zenith_drm_vblank_us != 0)
 *
 *   pressure_pct >= ZENITH_AT_GLIDE_THERMAL_PRESSURE_PCT:
 *     thermal_pressure_continuous = 1
 *
 *   always (independent of state):
 *     screen_off_glide_ms    = ZENITH_AT_GLIDE_SCREEN_OFF_MS
 *     boot_boost_decay_ms    = ZENITH_AT_GLIDE_BOOT_BOOST_DECAY_MS
 *     (these are arm-time / one-shot knobs, not state-dependent)
 *
 * Sets at_local_glides_active to gate consumer fall-through.
 * Cheap: O(1) and writes scalar fields with no locking (single
 * writer in the V2 worker, single readers in the consumer paths
 * under update_lock; staleness cost is bounded by the V2 tick
 * cadence).
 */
static void zenith_at_apply_glides(struct zenith_policy *z_policy,
				   unsigned int state)
{
	unsigned int flags = z_policy->at_last_flags;
	unsigned int pressure_pct = z_policy->at_last_thermal_pressure;
	unsigned int brutal_ms = 0;
	unsigned int wakeup_ms = 0;
	unsigned int thermal_continuous = 0;
	unsigned int prefer_silver_aware_v = 0;
	unsigned int frame_auto_v = 0;

	switch (state) {
	case ZENITH_AT_STATE_LATENCY:
		brutal_ms = ZENITH_AT_GLIDE_BRUTAL_DECAY_MS;
		wakeup_ms = ZENITH_AT_GLIDE_WAKEUP_BOOST_MS;
		break;
	case ZENITH_AT_STATE_THERMAL_RECOVERY:
		thermal_continuous = 1;
		break;
	case ZENITH_AT_STATE_EFFICIENCY:
	case ZENITH_AT_STATE_BALANCED:
		if (z_policy->ps_hit_rate_pct >=
		    READ_ONCE(z_policy->tunables->
			      prefer_silver_hot_threshold_pct))
			prefer_silver_aware_v = 1;
		break;
	default:
		break;
	}

	if ((flags & ZENITH_AT_FLAG_FRAME) || (flags & ZENITH_AT_FLAG_GAME)) {
		if (!wakeup_ms)
			wakeup_ms = ZENITH_AT_GLIDE_WAKEUP_BOOST_MS;
		if (atomic_read(&zenith_drm_vblank_us))
			frame_auto_v = 1;
	}

	if (pressure_pct >= ZENITH_AT_GLIDE_THERMAL_PRESSURE_PCT)
		thermal_continuous = 1;

	z_policy->at_local_brutal_decay_ms = brutal_ms;
	z_policy->at_local_wakeup_boost_ms = wakeup_ms;
	z_policy->at_local_boot_boost_decay_ms =
		ZENITH_AT_GLIDE_BOOT_BOOST_DECAY_MS;
	z_policy->at_local_screen_off_glide_ms =
		ZENITH_AT_GLIDE_SCREEN_OFF_MS;
	z_policy->at_local_thermal_pressure_continuous = thermal_continuous;
	z_policy->at_local_prefer_silver_aware = prefer_silver_aware_v;
	z_policy->at_local_frame_budget_us_auto = frame_auto_v;
	z_policy->at_local_glides_active = true;
}

/* Patch L: V2-classifier tier-armer.  Sibling of
 * zenith_at_apply_glides().  Computes which Stage-4 K1/K2/K3
 * floor tiers should be armed for the current state / flag set
 * and stamps the result into z_policy->at_local_tier_armed_mask.
 *
 * Mapping rationale (also documented at the ZENITH_AT_TIER_*
 * defines, kept in sync with zenith_at_apply_glides()'s state
 * switch):
 *
 *   LATENCY: this is the "we want fast wakeups" V2 state.  K1
 *   compensates PELT migration lag on inbound tasks; K2 lifts
 *   to hispeed when sustained CPU pressure says queueing is
 *   real.  Both are fits.  K3 needs vblank events that LATENCY
 *   doesn't necessarily imply -- gated separately on the FRAME
 *   flag below.
 *
 *   FRAME / GAME flags (independent of state): K1 catches the
 *   render-thread scheduling shuffle that scaling between
 *   clusters tends to trigger; K3 catches missed frames once
 *   the panel driver wires zenith_drm_vblank_event().  K2 is
 *   intentionally NOT armed here -- frame work doesn't
 *   correlate with sustained PSI pressure, and the 10s EWMA
 *   would be a cross-talk signal we don't want during gameplay.
 *
 *   EFFICIENCY / BALANCED: the "be conservative on freq" V2
 *   states.  Disarming all three tiers preserves the user's
 *   intent (battery / mid-line behaviour) even when the
 *   profile selected was PERFORMANCE/BALANCED.
 *
 *   THERMAL_RECOVERY: the V2 actions path already pulls down
 *   freq with thermal_pressure_continuous; layering K1/K2/K3
 *   floors on top would fight that.  Disarmed.
 *
 *   SUSTAINED_PERF: the V2 actions path already pins freq
 *   high; K1/K2/K3 floors on top are double-counting (the
 *   resolved freq is already at-or-above any of the floor_pct
 *   ceilings).  Disarmed -- not because it would be wrong but
 *   because it would be redundant.
 *
 * Cheap: O(1), one bitmask write under the policy lock.  Single
 * writer (V2 worker), single reader (eval path under
 * update_lock).
 */
static void zenith_at_apply_tiers(struct zenith_policy *z_policy,
				  unsigned int state)
{
	unsigned int flags = z_policy->at_last_flags;
	unsigned long armed = 0;

	switch (state) {
	case ZENITH_AT_STATE_LATENCY:
		armed |= ZENITH_AT_TIER_MIGRATION;
		armed |= ZENITH_AT_TIER_PSI_CPU_FLOOR;
		break;
	case ZENITH_AT_STATE_EFFICIENCY:
	case ZENITH_AT_STATE_BALANCED:
	case ZENITH_AT_STATE_THERMAL_RECOVERY:
		/* Patch M1: PSI-mem light cap.  Three states where
		 * "back off on memstall" is the policy's job, not a
		 * regression: EFFICIENCY explicitly trades freq for
		 * energy; BALANCED is the all-rounder default;
		 * THERMAL_RECOVERY is already in cool-down.
		 */
		armed |= ZENITH_AT_TIER_PSI_MEM_CAP;
		break;
	default:
		break;
	}

	if (flags & (ZENITH_AT_FLAG_FRAME | ZENITH_AT_FLAG_GAME)) {
		armed |= ZENITH_AT_TIER_MIGRATION;
		armed |= ZENITH_AT_TIER_FRAME_OVERRUN;
		/* Patch M1: frame-pacing and game overrides need full
		 * headroom on the cap side, so unconditionally clear
		 * the PSI-mem cap bit even if the underlying state
		 * had armed it.  (BALANCED + FRAME flag is the common
		 * case here.)  An explicit clear, not a "reset armed",
		 * so other tiers stay armed.
		 */
		armed &= ~ZENITH_AT_TIER_PSI_MEM_CAP;
	}

	z_policy->at_local_tier_armed_mask = armed;
	z_policy->at_local_tiers_active = true;
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

/* Audit fix M6.  Bounded variant of ZENITH_TUNABLE_UINT.  Max value
 * is taken from a user-supplied compile-time constant so the cap is
 * documented in the same place the macro is invoked.  Compare to
 * the open-coded "kstrtouint(...) || val > FOO" pattern used by
 * iowait_boost_min, iowait_stack_pct, etc.
 */
#define ZENITH_TUNABLE_UINT_MAX(_name, _max) \
static ssize_t _name##_show(struct gov_attr_set *attr_set, char *buf) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	return sprintf(buf, "%u\n", t->_name); \
} \
static ssize_t _name##_store(struct gov_attr_set *attr_set, const char *buf, size_t count) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	unsigned int val; \
	if (kstrtouint(buf, 10, &val) || val > (_max)) \
		return -EINVAL; \
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

/* Audit fix M6.  Bounded variant of ZENITH_TUNABLE_UINT_INVAL for
 * boolean-style tunables (0 or 1 only).  Without the bound the
 * previous template would happily store UINT_MAX into a "screen on?"
 * field; downstream code uses the value with `if (t->screen_state)`
 * so any nonzero won the test, but writing a large number out via
 * the show() path then re-reading produced a confusing audit trail.
 */
#define ZENITH_TUNABLE_UINT_BOOL_INVAL(_name) \
static ssize_t _name##_show(struct gov_attr_set *attr_set, char *buf) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	return sprintf(buf, "%u\n", t->_name); \
} \
static ssize_t _name##_store(struct gov_attr_set *attr_set, const char *buf, size_t count) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	unsigned int val; \
	if (kstrtouint(buf, 10, &val) || val > 1) \
		return -EINVAL; \
	t->_name = val; \
	zenith_invalidate_cache(attr_set); \
	return count; \
} \
static struct governor_attr _name = __ATTR_RW(_name)

ZENITH_TUNABLE_UINT_BOOL_INVAL(io_is_busy);

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
		/* Stage 1 / Stage 2 additions, profile-driven so users
		 * never have to touch them: scenario-detected profile
		 * flips (camera/render -> PERFORMANCE, memstall ->
		 * BATTERY, audio -> BALANCED, game-mode sustained ->
		 * PERFORMANCE, thermal/screen-off/PSI -> ...) all
		 * re-apply this table via zenith_apply_profile().
		 */
		unsigned int peak_headroom_rescue;
		unsigned int peak_headroom_prearm;
		unsigned int peak_headroom_starve_load_pct;
		unsigned int peak_headroom_freq_floor_pct;
		unsigned int peak_headroom_starve_streak;
		unsigned int peak_headroom_jump_pct;
		unsigned int peak_headroom_hold_ms;
		unsigned int screen_on_bias_pct;
		unsigned int input_boost_down_rate_mult_pct;
		unsigned int predict_up_thresh;
		unsigned int predict_up_window;
		unsigned int render_floor_pct;
		unsigned int render_floor_min_runtime_ms;
		unsigned int input_boost_touchdown_extra_ms;
		unsigned int peak_hysteresis_streak;
		unsigned int peak_step_down_pct;
		unsigned int boost_idle_thresh;
		unsigned int boost_idle_streak;
		unsigned int bg_util_scale_pct;
		unsigned int sleeper_tail_thresh_us;
		unsigned int sleeper_tail_pct;
		unsigned int peer_ramp_window_ms;
		unsigned int peer_ramp_floor_pct;
		unsigned int peer_ramp_window_off_ms;
		unsigned int migration_jump_pct;
		unsigned int migration_floor_window_ms;
		unsigned int migration_floor_pct;
		unsigned int psi_cpu_floor_thresh;
		unsigned int frame_overrun_slack_us;
		unsigned int frame_overrun_window_ms;
		unsigned int frame_overrun_floor_pct;
		unsigned int frame_overrun_deep_streak;
		unsigned int frame_overrun_deep_floor_pct;
		unsigned int psi_mem_cap_thresh;
		unsigned int psi_mem_cap_pct;
		unsigned int psi_mem_cap_window_ms;
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
			/* Stage 1/2: aggressive rescue, no bias on
			 * screen, 3x down-rate during input boost.
			 */
			.peak_headroom_rescue = 1,
			.peak_headroom_prearm = 1,
			.peak_headroom_starve_load_pct = 88,
			.peak_headroom_freq_floor_pct = 80,
			.peak_headroom_starve_streak = 2,
			.peak_headroom_jump_pct = 100,
			.peak_headroom_hold_ms = 25,
			.screen_on_bias_pct = 0,
			.input_boost_down_rate_mult_pct = 300,
			/* Stage 4 / Patch A: PERFORMANCE wants eager
			 * prediction.  Lower thresh (48 vs 64 default)
			 * fires the lift on smaller rises; window stays
			 * at the default (4) so the trend is computed
			 * over the same warm-up period.
			 */
			.predict_up_thresh = 48,
			.predict_up_window = 4,
			/* Stage 4 / Patch B: PERFORMANCE keeps the
			 * render floor strong (80%% of max, vs 70%%
			 * default) and tightens the debounce to 20 ms
			 * so frame deadlines aren't lost to a slow
			 * floor-arm on transient render activity.
			 */
			.render_floor_pct = 80,
			.render_floor_min_runtime_ms = 20,
			/* Stage 4 / Patch C: PERFORMANCE adds an extra
			 * 80 ms on touchdown -- the user-perceived
			 * latency from finger-down to first frame is
			 * what makes a phone "feel snappy".
			 */
			.input_boost_touchdown_extra_ms = 80,
			/* Stage 4 / Patch E: PERFORMANCE pins the soft
			 * floor higher (97% of anchor) and holds it
			 * for 4 samples.  The cluster paid for the
			 * peak; an extra few ms of high freq is cheap
			 * compared to bouncing through a frame deadline
			 * because EAS plunged on a single low sample.
			 */
			.peak_hysteresis_streak = 4,
			.peak_step_down_pct = 97,
			/* Stage 4 / Patch F: PERFORMANCE disables the
			 * boost early-exit.  The whole point of the
			 * profile is to honour every UX boost in full;
			 * a few ticks of below-threshold load inside a
			 * boost window is not a reason to drop the
			 * cluster off the boost ceiling.
			 */
			.boost_idle_thresh = 0,
			.boost_idle_streak = 0,
			/* Stage 4 / Patch G: PERFORMANCE keeps full util
			 * even when the screen is off.  This profile is
			 * for users who want maximum responsiveness on
			 * unlock; trimming background util would slow
			 * the device's recovery from a deep-sleep wake.
			 */
			.bg_util_scale_pct = 100,
			/* Stage 4 / Patch H: PERFORMANCE disables sleeper-
			 * tail shaving so wake-up freq is unshaved.
			 */
			.sleeper_tail_thresh_us = 0,
			.sleeper_tail_pct = 100,
			/* Stage 4 / Patch D: PERFORMANCE widens the peer-
			 * ramp window to 40 ms and lifts the floor to 70%%
			 * of policy->max.  IPC chains under this profile
			 * are typically the latency-sensitive kind --
			 * binder hops on app launch, render -> compositor
			 * -> display -- so giving the peer cluster a
			 * bigger and slightly higher pre-arm is worth the
			 * energy.
			 */
			.peer_ramp_window_ms = 40,
			.peer_ramp_floor_pct = 70,
			/* Stage 5 / Patch M3: PERFORMANCE suppresses
			 * peer_ramp once the screen is off.  The IPC
			 * chains the screen-on PERF override widens for
			 * (compositor / render / input) are inactive
			 * with the display blanked, so even the most
			 * aggressive profile gives back the screen-off
			 * energy.  Set non-zero by sysfs to keep peer-
			 * arming warm during e.g. audio playback.
			 */
			.peer_ramp_window_off_ms = 0,
			/* Stage 4 / Patch K1: PERFORMANCE drops the
			 * jump threshold to 15%% (one of every six
			 * util-percent points instead of every five)
			 * so smaller migration arrivals still trip the
			 * floor, and widens the floor to 70%% over a
			 * 35 ms window.  Symmetric reasoning to the
			 * peer-ramp PERF override above: latency-
			 * sensitive workloads benefit more from
			 * absorbing the PELT-warm-up cost than from
			 * the small energy saving of letting the freq
			 * drift down.
			 */
			.migration_jump_pct = 15,
			.migration_floor_window_ms = 35,
			.migration_floor_pct = 70,
			/* Stage 4 / Patch K2: PERFORMANCE arms the
			 * PSI-CPU sustained-pressure floor at 40%%.
			 * The 10s EWMA at 40+%% means the system has
			 * been queueing for a sustained stretch -- on
			 * a PERF profile that is unambiguously a sign
			 * we should be at hispeed regardless of
			 * what aggregate util is saying.
			 */
			.psi_cpu_floor_thresh = 40,
			/* Stage 4 / Patch K3: PERFORMANCE tightens
			 * the slack to 3000 us (~18%% of a 60 Hz
			 * frame) so smaller misses still trip the
			 * floor, and lifts the floor to 90%% of
			 * policy->max over a 60 ms window.  PERF is
			 * the profile where chasing missed frames is
			 * actually useful; energy spent on a recovery
			 * floor is justified.
			 */
			.frame_overrun_slack_us = 3000,
			.frame_overrun_window_ms = 60,
			.frame_overrun_floor_pct = 90,
			/* Stage 5 / Patch M5: PERFORMANCE arms the K3
			 * deep tier at 2 / 100%%.  Two consecutive
			 * 60 Hz vblank gaps wider than 3 ms slack is
			 * ~33 ms of stutter -- well outside any
			 * plausible single-shot jitter explanation.
			 * Lifting the floor to 100%% of policy->max
			 * for the recovery window is the right
			 * trade for a profile the user has explicitly
			 * picked for max responsiveness.
			 */
			.frame_overrun_deep_streak = 2,
			.frame_overrun_deep_floor_pct = 100,
			/* Stage 5 / Patch M1: PERFORMANCE keeps the
			 * PSI-mem cap off (thresh = 0).  The whole
			 * point of PERF is full headroom; backing off
			 * on memstall is the wrong call here.  The
			 * pct/window are populated with sane values
			 * for forward compatibility if the user
			 * overrides thresh via sysfs.
			 */
			.psi_mem_cap_thresh = 0,
			.psi_mem_cap_pct = 90,
			.psi_mem_cap_window_ms = 1000,
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
			/* Stage 1/2: same compile-time defaults as
			 * ZENITH_DEFAULT_*; this is the cold-boot
			 * baseline.
			 */
			.peak_headroom_rescue =
				ZENITH_DEFAULT_PEAK_HEADROOM_RESCUE,
			.peak_headroom_prearm =
				ZENITH_DEFAULT_PEAK_HEADROOM_PREARM,
			.peak_headroom_starve_load_pct =
				ZENITH_DEFAULT_PEAK_HEADROOM_STARVE_LOAD_PCT,
			.peak_headroom_freq_floor_pct =
				ZENITH_DEFAULT_PEAK_HEADROOM_FREQ_FLOOR_PCT,
			.peak_headroom_starve_streak =
				ZENITH_DEFAULT_PEAK_HEADROOM_STARVE_STREAK,
			.peak_headroom_jump_pct =
				ZENITH_DEFAULT_PEAK_HEADROOM_JUMP_PCT,
			.peak_headroom_hold_ms =
				ZENITH_DEFAULT_PEAK_HEADROOM_HOLD_MS,
			.screen_on_bias_pct =
				ZENITH_DEFAULT_SCREEN_ON_BIAS_PCT,
			.input_boost_down_rate_mult_pct =
				ZENITH_DEFAULT_INPUT_BOOST_DOWN_RATE_MULT_PCT,
			/* Stage 4 / Patch A: BALANCED matches the cold-
			 * boot ZENITH_DEFAULT_* values exactly so a
			 * plain compile produces the same behaviour as
			 * an explicit echo balanced > profile.
			 */
			.predict_up_thresh =
				ZENITH_DEFAULT_PREDICT_UP_THRESH,
			.predict_up_window =
				ZENITH_DEFAULT_PREDICT_UP_WINDOW,
			/* Stage 4 / Patch B: BALANCED matches cold-
			 * boot defaults (70%% floor, 50 ms debounce).
			 */
			.render_floor_pct =
				ZENITH_DEFAULT_RENDER_FLOOR_PCT,
			.render_floor_min_runtime_ms =
				ZENITH_DEFAULT_RENDER_FLOOR_MIN_RUNTIME_MS,
			/* Stage 4 / Patch C: BALANCED matches cold-boot
			 * default (50 ms touchdown extra).
			 */
			.input_boost_touchdown_extra_ms =
				ZENITH_DEFAULT_INPUT_BOOST_TOUCHDOWN_EXTRA_MS,
			/* Stage 4 / Patch E: BALANCED matches cold-boot
			 * defaults (3-sample streak, 95% step-down).
			 */
			.peak_hysteresis_streak =
				ZENITH_DEFAULT_PEAK_HYSTERESIS_STREAK,
			.peak_step_down_pct =
				ZENITH_DEFAULT_PEAK_STEP_DOWN_PCT,
			/* Stage 4 / Patch F: BALANCED matches cold-boot
			 * defaults (15% threshold, 3-tick streak).  This
			 * is the energy-optimisation profile, so trimming
			 * the boost tail when load actually drained makes
			 * sense as a default.
			 */
			.boost_idle_thresh =
				ZENITH_DEFAULT_BOOST_IDLE_THRESH,
			.boost_idle_streak =
				ZENITH_DEFAULT_BOOST_IDLE_STREAK,
			/* Stage 4 / Patch G: BALANCED scales screen-off
			 * util to 75% so background sync work runs on a
			 * lower freq tier while the device is locked.
			 */
			.bg_util_scale_pct = 75,
			/* Stage 4 / Patch H: BALANCED arms sleeper-tail
			 * shaving with a 20 ms idle threshold and a 90%
			 * shave.  Mild trim on the wake-up tick.
			 */
			.sleeper_tail_thresh_us = 20000,
			.sleeper_tail_pct = 90,
			/* Stage 4 / Patch D: BALANCED matches cold-boot
			 * defaults (25 ms window, 60%% floor).  Mild
			 * pre-arm: enough to shave warm-up latency on
			 * common cross-cluster wakes without paying for
			 * a full hispeed pin on every peer event.
			 */
			.peer_ramp_window_ms =
				ZENITH_DEFAULT_PEER_RAMP_WINDOW_MS,
			.peer_ramp_floor_pct =
				ZENITH_DEFAULT_PEER_RAMP_FLOOR_PCT,
			/* Stage 5 / Patch M3: BALANCED takes the cold-
			 * boot screen-off default (suppress peer_ramp).
			 * BALANCED is the all-day profile and the screen
			 * spends most of that day off.
			 */
			.peer_ramp_window_off_ms =
				ZENITH_DEFAULT_PEER_RAMP_WINDOW_OFF_MS,
			/* Stage 4 / Patch K1: BALANCED matches cold-
			 * boot defaults (20%% jump, 30 ms window, 60%%
			 * floor).  Sensible middle ground: catches the
			 * common one-task-arrived migration without
			 * over-firing on routine util oscillation.
			 */
			.migration_jump_pct =
				ZENITH_DEFAULT_MIGRATION_JUMP_PCT,
			.migration_floor_window_ms =
				ZENITH_DEFAULT_MIGRATION_FLOOR_WINDOW_MS,
			.migration_floor_pct =
				ZENITH_DEFAULT_MIGRATION_FLOOR_PCT,
			/* Stage 4 / Patch K2: BALANCED leaves the
			 * PSI-CPU floor disabled (the cold-boot
			 * default).  The energy cost of a hispeed pin
			 * triggered by a smoothed signal isn't worth
			 * paying on the everyday profile -- predict_up
			 * and the existing peak tiers cover the cases
			 * a balanced workload actually needs.
			 */
			.psi_cpu_floor_thresh =
				ZENITH_DEFAULT_PSI_CPU_FLOOR_THRESH,
			/* Stage 4 / Patch K3: BALANCED arms the
			 * frame-overrun rescue with 4000 us slack
			 * (~24%% of a 60 Hz frame), 50 ms window,
			 * 80%% floor.  Mid-range slack catches
			 * actual misses without tripping on routine
			 * driver jitter; mid-range floor balances
			 * recovery latency against energy cost.
			 */
			.frame_overrun_slack_us = 4000,
			.frame_overrun_window_ms =
				ZENITH_DEFAULT_FRAME_OVERRUN_WINDOW_MS,
			.frame_overrun_floor_pct =
				ZENITH_DEFAULT_FRAME_OVERRUN_FLOOR_PCT,
			/* Stage 5 / Patch M5: BALANCED keeps the deep
			 * tier at the cold-boot off default.  K3 alone
			 * is sufficient for the BALANCED energy /
			 * latency trade-off; users who want the deep
			 * escalation can opt in via sysfs without
			 * switching to PERFORMANCE.
			 */
			.frame_overrun_deep_streak =
				ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_STREAK,
			.frame_overrun_deep_floor_pct =
				ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_FLOOR_PCT,
			/* Stage 5 / Patch M1: BALANCED keeps the
			 * PSI-mem cap at the cold-boot off default.
			 * BALANCED's existing tier mapping arms the
			 * tier bit, so a sysfs flip of thresh > 0 is
			 * sufficient to opt in without flipping the
			 * profile.
			 */
			.psi_mem_cap_thresh =
				ZENITH_DEFAULT_PSI_MEM_CAP_THRESH,
			.psi_mem_cap_pct =
				ZENITH_DEFAULT_PSI_MEM_CAP_PCT,
			.psi_mem_cap_window_ms =
				ZENITH_DEFAULT_PSI_MEM_CAP_WINDOW_MS,
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
			/* Stage 1/2: keep the rescue safety net (the
			 * tester's "never reaches peak" complaint
			 * matters even on BATTERY) but disable the
			 * pre-arm and bound the rescue to 90%% of max
			 * to save the peak-freq energy slope.  Looser
			 * starve_streak (5) and longer hold (100ms)
			 * stop rescue churn from waking the cluster
			 * unnecessarily.  Light bias softening (80%%)
			 * keeps screen-on responsive without paying
			 * the BALANCED energy cost.
			 */
			.peak_headroom_rescue = 1,
			.peak_headroom_prearm = 0,
			.peak_headroom_starve_load_pct = 92,
			.peak_headroom_freq_floor_pct = 90,
			.peak_headroom_starve_streak = 5,
			.peak_headroom_jump_pct = 90,
			.peak_headroom_hold_ms = 100,
			.screen_on_bias_pct = 80,
			.input_boost_down_rate_mult_pct = 150,
			/* Stage 4 / Patch A: BATTERY disables prediction.
			 * The pre-shift would burn frame-edge energy that
			 * the rest of the BATTERY profile is specifically
			 * trying to avoid, and the level-triggered
			 * hispeed tier is good enough at this freq cap.
			 */
			.predict_up_thresh = 0,
			.predict_up_window =
				ZENITH_DEFAULT_PREDICT_UP_WINDOW,
			/* Stage 4 / Patch B: BATTERY softens the render
			 * floor to 50%% and stretches the debounce to
			 * 100 ms.  Render activity still floors the
			 * cluster but only after the workload is sticky;
			 * one-shot SurfaceFlinger flushes don't pay the
			 * floor.
			 */
			.render_floor_pct = 50,
			.render_floor_min_runtime_ms = 100,
			/* Stage 4 / Patch C: BATTERY drops the touchdown
			 * extra to 30 ms.  Touchdown latency still gets
			 * a small bonus, but the energy cost of an
			 * 80 ms extra-pin tail is too high for the
			 * battery profile.
			 */
			.input_boost_touchdown_extra_ms = 30,
			/* Stage 4 / Patch E: BATTERY shortens the
			 * streak to 2 samples and steepens the step
			 * down to 90% so the cluster falls off peak
			 * faster.  Hysteresis is still useful (one
			 * sample of low load isn't enough to confirm
			 * the workload is gone) but we don't want to
			 * pay 4 samples of extra freq.
			 */
			.peak_hysteresis_streak = 2,
			.peak_step_down_pct = 90,
			/* Stage 4 / Patch F: BATTERY uses an aggressive
			 * early-exit (25% threshold, 2-tick streak) so
			 * the boost ceiling is dropped as soon as the
			 * launch animation visibly drains.  Combined
			 * with the lower input_boost_ms in this profile
			 * the energy tail of a tap-and-release gesture
			 * is closer to a hard cliff than a 60 ms decay.
			 */
			.boost_idle_thresh = 25,
			.boost_idle_streak = 2,
			/* Stage 4 / Patch G: BATTERY scales screen-off
			 * util to 60% -- aggressive but appropriate
			 * for the battery profile where the user has
			 * already opted in to slower performance.
			 */
			.bg_util_scale_pct = 60,
			/* Stage 4 / Patch H: BATTERY uses an aggressive
			 * 10 ms threshold and 80% shave for maximum
			 * leakage savings.
			 */
			.sleeper_tail_thresh_us = 10000,
			.sleeper_tail_pct = 80,
			/* Stage 4 / Patch D: BATTERY disables peer-ramp.
			 * The user has opted in to slower performance,
			 * and the energy cost of holding the peer at
			 * a 60%%+ floor for 25 ms after every BIG/PRIME
			 * peak is exactly the kind of overhead this
			 * profile exists to avoid.
			 */
			.peer_ramp_window_ms = 0,
			.peer_ramp_floor_pct = 0,
			/* Stage 5 / Patch M3: BATTERY mirrors the
			 * peer_ramp_window_ms = 0 stance for the screen-
			 * off path; redundant given peer_ramp is already
			 * off, but kept explicit so a sysfs tweak that
			 * lifts peer_ramp_window_ms in this profile does
			 * not silently un-suppress the screen-off arm.
			 */
			.peer_ramp_window_off_ms = 0,
			/* Stage 4 / Patch K1: BATTERY disables the
			 * migration-arrival floor for the same reason
			 * peer-ramp is off here.  PELT warm-up
			 * latency is exactly the cost the user is
			 * trading away when picking this profile.
			 */
			.migration_jump_pct = 0,
			.migration_floor_window_ms = 0,
			.migration_floor_pct = 0,
			/* Stage 4 / Patch K2: BATTERY keeps the PSI-CPU
			 * floor disabled.  Sustained pressure under
			 * BATTERY is exactly when the user wants the
			 * governor to *not* lift -- the queueing is
			 * the price of running at conservative freq.
			 */
			.psi_cpu_floor_thresh = 0,
			/* Stage 4 / Patch K3: BATTERY disables the
			 * frame-overrun rescue.  Same trade as
			 * peer-ramp / migration-arrival above:
			 * recovery freq is exactly the cost the
			 * user is opting out of when picking BATTERY.
			 */
			.frame_overrun_slack_us = 0,
			.frame_overrun_window_ms = 0,
			.frame_overrun_floor_pct = 0,
			/* Stage 5 / Patch M5: BATTERY keeps the deep
			 * tier off for the same reason K3 itself is
			 * off here.  Redundant given K3's outer gate
			 * already short-circuits, but kept explicit so
			 * a sysfs tweak that lifts frame_overrun_*
			 * does not silently bring deep along.
			 */
			.frame_overrun_deep_streak = 0,
			.frame_overrun_deep_floor_pct = 100,
			/* Stage 5 / Patch M1: BATTERY arms the PSI-
			 * mem cap aggressively (thresh = 50%, cap to
			 * 70% of policy->max for 1.5 s).  Battery is
			 * the profile where backing off on memstall
			 * matches the user's stated preference: spend
			 * less power on cycles that would just stall
			 * on mm.  The wider window captures full
			 * kswapd / lowmem-killer bursts.
			 */
			.psi_mem_cap_thresh = 50,
			.psi_mem_cap_pct = 70,
			.psi_mem_cap_window_ms = 1500,
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
			/* Stage 1/2: LEGACY restores pre-Stage-1
			 * behaviour wholesale.  Rescue and pre-arm
			 * disabled (rescue gate off cancels the entire
			 * peak-headroom path; pre-arm follows for
			 * defence in depth).  screen_on_bias_pct = 100
			 * disables the bias softening (effective bias
			 * == raw bias).  input_boost_down_rate_mult_-
			 * pct = 100 disables the down-rate extension
			 * (effective down_delay == raw down_delay
			 * during boost).  The diagnostic /
			 * sub-rescue knobs are populated with the
			 * loosest values for forward compatibility
			 * (in case rescue is turned back on by sysfs).
			 */
			.peak_headroom_rescue = 0,
			.peak_headroom_prearm = 0,
			.peak_headroom_starve_load_pct = 95,
			.peak_headroom_freq_floor_pct = 95,
			.peak_headroom_starve_streak = 16,
			.peak_headroom_jump_pct = 100,
			.peak_headroom_hold_ms = 200,
			.screen_on_bias_pct = 100,
			.input_boost_down_rate_mult_pct = 100,
			/* Stage 4 / Patch A: LEGACY disables prediction
			 * (no Stage 4 features in the historical-
			 * compatibility profile).
			 */
			.predict_up_thresh = 0,
			.predict_up_window =
				ZENITH_DEFAULT_PREDICT_UP_WINDOW,
			/* Stage 4 / Patch B: LEGACY disables the floor
			 * outright (render_floor_pct=0) since the floor
			 * is a Stage-1+ feature.  The debounce knob is
			 * left at 0 (no debounce) for forward
			 * compatibility if the user re-enables the
			 * floor through sysfs.
			 */
			.render_floor_pct = 0,
			.render_floor_min_runtime_ms = 0,
			/* Stage 4 / Patch C: LEGACY disables the
			 * touchdown extra (legacy boost cadence
			 * unchanged).
			 */
			.input_boost_touchdown_extra_ms = 0,
			/* Stage 4 / Patch E: LEGACY disables peak-
			 * return hysteresis entirely so the descent
			 * shape is identical to the pre-Stage-4
			 * governor.
			 */
			.peak_hysteresis_streak = 0,
			.peak_step_down_pct = 0,
			/* Stage 4 / Patch F: LEGACY disables the boost
			 * early-exit; boosts are honoured to their
			 * configured deadline.
			 */
			.boost_idle_thresh = 0,
			.boost_idle_streak = 0,
			/* Stage 4 / Patch G: LEGACY keeps full util
			 * regardless of screen state (legacy governor
			 * had no notion of screen-off util scaling).
			 */
			.bg_util_scale_pct = 100,
			/* Stage 4 / Patch H: LEGACY disables sleeper-tail
			 * shaving (legacy governor had no notion).
			 */
			.sleeper_tail_thresh_us = 0,
			.sleeper_tail_pct = 100,
			/* Stage 4 / Patch D: LEGACY disables peer-ramp.
			 * Pre-Stage-1 governor had no cross-cluster
			 * coordination; LEGACY preserves that behaviour
			 * end-to-end.
			 */
			.peer_ramp_window_ms = 0,
			.peer_ramp_floor_pct = 0,
			/* Stage 5 / Patch M3: LEGACY likewise leaves the
			 * screen-off shadow at 0.  Pre-Stage-1 governor
			 * had no peer_ramp at all, screen-on or screen-
			 * off, so this preserves that absence end-to-
			 * end.
			 */
			.peer_ramp_window_off_ms = 0,
			/* Stage 4 / Patch K1: LEGACY disables the
			 * migration-arrival floor.  Pre-Stage-1
			 * governor had no PELT-warm-up compensation;
			 * LEGACY preserves that.
			 */
			.migration_jump_pct = 0,
			.migration_floor_window_ms = 0,
			.migration_floor_pct = 0,
			/* Stage 4 / Patch K2: LEGACY disables the
			 * PSI-CPU floor.  Pre-Stage-1 governor had no
			 * pressure-aware lifts.  LEGACY preserves
			 * that.
			 */
			.psi_cpu_floor_thresh = 0,
			/* Stage 4 / Patch K3: LEGACY disables the
			 * frame-overrun rescue.  Pre-Stage-1 governor
			 * had no display-layer awareness; LEGACY
			 * preserves that.
			 */
			.frame_overrun_slack_us = 0,
			.frame_overrun_window_ms = 0,
			.frame_overrun_floor_pct = 0,
			/* Stage 5 / Patch M5: LEGACY mirrors the K3
			 * disable for the deep tier.  Pre-Stage-1
			 * governor had no concept of consecutive-
			 * overrun amplification; LEGACY preserves that
			 * absence end-to-end.
			 */
			.frame_overrun_deep_streak = 0,
			.frame_overrun_deep_floor_pct = 100,
			/* Stage 5 / Patch M1: LEGACY keeps the PSI-mem
			 * cap fully off.  Pre-Stage-1 governor had no
			 * concept of memstall-driven freq capping;
			 * LEGACY preserves that.
			 */
			.psi_mem_cap_thresh = 0,
			.psi_mem_cap_pct = 80,
			.psi_mem_cap_window_ms = 1000,
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
	t->peak_headroom_rescue	= p->peak_headroom_rescue;
	t->peak_headroom_prearm	= p->peak_headroom_prearm;
	t->peak_headroom_starve_load_pct =
		p->peak_headroom_starve_load_pct;
	t->peak_headroom_freq_floor_pct =
		p->peak_headroom_freq_floor_pct;
	t->peak_headroom_starve_streak =
		p->peak_headroom_starve_streak;
	t->peak_headroom_jump_pct = p->peak_headroom_jump_pct;
	t->peak_headroom_hold_ms = p->peak_headroom_hold_ms;
	t->screen_on_bias_pct	= p->screen_on_bias_pct;
	t->input_boost_down_rate_mult_pct =
		p->input_boost_down_rate_mult_pct;
	WRITE_ONCE(t->predict_up_thresh, p->predict_up_thresh);
	WRITE_ONCE(t->predict_up_window, p->predict_up_window);
	t->render_floor_pct	= p->render_floor_pct;
	WRITE_ONCE(t->render_floor_min_runtime_ms,
		   p->render_floor_min_runtime_ms);
	WRITE_ONCE(t->input_boost_touchdown_extra_ms,
		   p->input_boost_touchdown_extra_ms);
	WRITE_ONCE(t->peak_hysteresis_streak,
		   p->peak_hysteresis_streak);
	WRITE_ONCE(t->peak_step_down_pct, p->peak_step_down_pct);
	WRITE_ONCE(t->boost_idle_thresh, p->boost_idle_thresh);
	WRITE_ONCE(t->boost_idle_streak, p->boost_idle_streak);
	WRITE_ONCE(t->bg_util_scale_pct, p->bg_util_scale_pct);
	WRITE_ONCE(t->sleeper_tail_thresh_us,
		   p->sleeper_tail_thresh_us);
	WRITE_ONCE(t->sleeper_tail_pct, p->sleeper_tail_pct);
	WRITE_ONCE(t->peer_ramp_window_ms, p->peer_ramp_window_ms);
	WRITE_ONCE(t->peer_ramp_floor_pct, p->peer_ramp_floor_pct);
	WRITE_ONCE(t->peer_ramp_window_off_ms,
		   p->peer_ramp_window_off_ms);
	WRITE_ONCE(t->migration_jump_pct, p->migration_jump_pct);
	WRITE_ONCE(t->migration_floor_window_ms,
		   p->migration_floor_window_ms);
	WRITE_ONCE(t->migration_floor_pct, p->migration_floor_pct);
	WRITE_ONCE(t->psi_cpu_floor_thresh, p->psi_cpu_floor_thresh);
	WRITE_ONCE(t->frame_overrun_slack_us, p->frame_overrun_slack_us);
	WRITE_ONCE(t->frame_overrun_window_ms,
		   p->frame_overrun_window_ms);
	WRITE_ONCE(t->frame_overrun_floor_pct,
		   p->frame_overrun_floor_pct);
	WRITE_ONCE(t->frame_overrun_deep_streak,
		   p->frame_overrun_deep_streak);
	WRITE_ONCE(t->frame_overrun_deep_floor_pct,
		   p->frame_overrun_deep_floor_pct);
	WRITE_ONCE(t->psi_mem_cap_thresh, p->psi_mem_cap_thresh);
	WRITE_ONCE(t->psi_mem_cap_pct, p->psi_mem_cap_pct);
	WRITE_ONCE(t->psi_mem_cap_window_ms,
		   p->psi_mem_cap_window_ms);
	WRITE_ONCE(zenith_frame_overrun_slack_us_cache,
		   p->frame_overrun_slack_us);
	WRITE_ONCE(zenith_frame_overrun_window_ms_cache,
		   p->frame_overrun_window_ms);

	/* Mirror input_boost_ms and input_boost_touchdown_extra_ms to
	 * the governor-wide caches used by the input handler fast
	 * path so a profile flip is picked up on the next event.
	 */
	WRITE_ONCE(zenith_input_boost_active_ms, t->input_boost_ms);
	WRITE_ONCE(zenith_input_boost_touchdown_extra_ms_cache,
		   t->input_boost_touchdown_extra_ms);
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
			/* Mirror the hot-path frame_budget_us_auto override
			 * so the V2 classifier sees the same effective
			 * budget as the floor itself (which already prefers
			 * the drm-side cache).  Without this, V2 would
			 * miss frame_active on auto setups whose userspace
			 * frame_budget_us is left at 0.
			 */
			if (zenith_glide_value(z_policy,
				READ_ONCE(t->frame_budget_us_auto),
				z_policy->at_local_frame_budget_us_auto)) {
				unsigned int auto_us = (unsigned int)
					atomic_read(&zenith_drm_vblank_us);

				if (auto_us)
					frame_budget_us = auto_us;
			}
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
		/* Reason reflects the strongest currently active scenario
		 * signal, not just the cause of the last target change.
		 * Without this, a long-held scenario (e.g. camera viewfinder
		 * pinned at PERFORMANCE for 30s) would show reason=classifier
		 * because the target stayed steady; that is misleading for
		 * post-hoc telemetry.  Severity order matches the target
		 * picker above (camera/render > memstall > audio).  Thermal,
		 * PSI, frame, screen, variance still override below; this
		 * is only the scenario-block default.
		 */
		if (camera || render)
			reason = ZENITH_AT_REASON_CAMERA_RENDER;
		else if (memstall)
			reason = ZENITH_AT_REASON_MEMSTALL;
		else if (audio)
			reason = ZENITH_AT_REASON_AUDIO;
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
		   t->auto_tune_v2_var_promote_thresh &&
		   z_policy->load_var_ewma_x256 >=
				t->auto_tune_v2_var_promote_thresh &&
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
		unsigned int need = zenith_at_eff_hyst_windows(z_policy,
						t->auto_tune_hysteresis_windows);
		bool emergency = state == ZENITH_AT_STATE_THERMAL_RECOVERY ||
				 state == ZENITH_AT_STATE_SUSTAINED_PERF;
		/* Audit fix M3: rising-edge fast-path commit.
		 *
		 * Compute the bits that turned ON in this window's flag
		 * bitmap (~at_last_flags & flags).  If any of the
		 * UI-perceptible / urgent scenarios just-armed -- camera,
		 * render, frame, thermal_slope, game, psi_cpu, memstall --
		 * treat the resulting V2 state as emergency for both the
		 * cooldown bypass and the hysteresis gate.  Without this,
		 * a 2-window default hysteresis means a camera flag that
		 * fires at t=0 has to wait until t=2*1.25 s = 2.5 s before
		 * V2 actually commits the new state, which is visible to
		 * users (camera open feels sluggish for the first second).
		 *
		 * The mask is intentionally narrow.  Plain saturation /
		 * variance promotions are NOT fast-pathed because those
		 * are exactly the workloads where stable hysteresis pays
		 * off (avoid bouncing between LATENCY and SUSTAINED_PERF
		 * on a single-tick spike).  Only signals from explicit
		 * producers (input, drm, scenario detectors, PSI) get the
		 * fast lane.
		 */
		unsigned int rising = flags & ~z_policy->at_last_flags;
		const unsigned int fastpath_mask =
			ZENITH_AT_FLAG_CAMERA |
			ZENITH_AT_FLAG_RENDER |
			ZENITH_AT_FLAG_FRAME  |
			ZENITH_AT_FLAG_THERMAL_SLOPE |
			ZENITH_AT_FLAG_GAME   |
			ZENITH_AT_FLAG_PSI_CPU |
			ZENITH_AT_FLAG_MEMSTALL;
		bool rising_fastpath = (rising & fastpath_mask) &&
			state != z_policy->at_last_state;

		if (need > ZENITH_AT_HYSTERESIS_WINDOWS_MAX)
			need = ZENITH_AT_HYSTERESIS_WINDOWS_MAX;
		if (z_policy->at_pending_state != state) {
			z_policy->at_pending_state = state;
			z_policy->at_pending_windows = 1;
		} else if (z_policy->at_pending_windows < need) {
			z_policy->at_pending_windows++;
		}
		if (rising_fastpath) {
			emergency = true;
			z_policy->at_pending_windows = need;
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
			u64 now_ns = ktime_get_ns();
			unsigned int hslot;

			/* M1: accumulate residency for the outgoing
			 * state.  The denominator can be 0 on the very
			 * first transition (last_change_ns hasn't been
			 * stamped yet) -- guard against that, otherwise
			 * a u64 underflow would inject ~146 years into
			 * residency[old_state].
			 */
			if (z_policy->at_state_last_change_ns &&
			    now_ns > z_policy->at_state_last_change_ns &&
			    old_state < ARRAY_SIZE(z_policy->at_state_residency_ns))
				z_policy->at_state_residency_ns[old_state] +=
					now_ns - z_policy->at_state_last_change_ns;
			z_policy->at_state_last_change_ns = now_ns;

			/* M2: push a history entry.  Lock-free single-
			 * writer ring (the V2 worker is the only writer
			 * for a given z_policy), reader (sysfs show)
			 * tolerates one entry of tearing on wrap which
			 * is acceptable for a diagnostic surface.
			 */
			hslot = z_policy->at_history_head;
			if (hslot >= ZENITH_AT_HISTORY_NR)
				hslot = 0;
			z_policy->at_history[hslot].ts_ns = now_ns;
			z_policy->at_history[hslot].flags = flags;
			z_policy->at_history[hslot].from = (u8)old_state;
			z_policy->at_history[hslot].to = (u8)state;
			z_policy->at_history[hslot].reason = (u8)reason;
			z_policy->at_history_head =
				(hslot + 1) % ZENITH_AT_HISTORY_NR;
			if (z_policy->at_history_count < ZENITH_AT_HISTORY_NR)
				z_policy->at_history_count++;

			z_policy->at_last_state = state;
			z_policy->at_cooldown_left =
				min_t(unsigned int,
				      zenith_at_eff_cool_windows(z_policy,
						t->auto_tune_cooldown_windows),
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
		/* Record the cluster-aware-demoted state for telemetry.
		 * applied_state mirrors at_last_state when cluster_aware
		 * is off, but reflects the LATENCY->BALANCED / BALANCED->
		 * LATENCY demotions when on.  Same expression the action
		 * picker uses internally (zenith_at_profile_for_state),
		 * mapped back via zenith_profile_to_at_state so the field
		 * is a state ID rather than a profile ID.
		 */
		z_policy->at_last_applied_state =
			zenith_profile_to_at_state(
				zenith_at_profile_for_state(z_policy, state));
		/* Populate the round-U-z10 glide knobs (brutal_decay_ms,
		 * wakeup_boost_ms, ...) from the just-resolved V2 state
		 * when auto_tune_v2_glides is on.  Cheap; gated so it
		 * remains free on systems that opt out.
		 */
		if (READ_ONCE(t->auto_tune_v2_glides))
			zenith_at_apply_glides(z_policy, state);
		else
			z_policy->at_local_glides_active = false;

		/* Patch L: arm / disarm Stage-4 K1/K2/K3 tiers based on
		 * the just-resolved V2 state and flag set.  Same gating
		 * shape as the glides above -- the tunable is its own
		 * master switch separate from auto_tune_v2.
		 */
		if (READ_ONCE(t->auto_tune_v2_tiers))
			zenith_at_apply_tiers(z_policy, state);
		else
			z_policy->at_local_tiers_active = false;

		/* Boot-complete calm detector.  Only runs while the
		 * latch is still down and the auto arm is enabled.
		 * Counts consecutive committed-EFFICIENCY windows on
		 * this policy; the first policy to reach the threshold
		 * past the boottime grace period raises the latch
		 * globally.  Subsequent policies / windows short-circuit
		 * on the atomic_read.
		 */
		if (!atomic_read(&zenith_boot_complete) &&
		    READ_ONCE(t->boot_complete_auto)) {
			if (state == ZENITH_AT_STATE_EFFICIENCY)
				z_policy->at_boot_calm_streak++;
			else
				z_policy->at_boot_calm_streak = 0;

			if (z_policy->at_boot_calm_streak >=
			    ZENITH_BOOT_COMPLETE_CALM_WINDOWS) {
				u64 now_ns = ktime_get_boottime_ns();

				if (now_ns >= ZENITH_BOOT_COMPLETE_GRACE_NS) {
					WRITE_ONCE(zenith_boot_complete_ns,
						   now_ns);
					atomic_set(&zenith_boot_complete, 1);
					pr_info("zenith: boot_complete latched (auto, calm=%u windows)\n",
						z_policy->at_boot_calm_streak);
				}
			}
		}
		goto rearm;
	}

	if (target != t->active_profile) {
		zenith_apply_profile(t, target);
		t->active_profile = target;
		z_policy->at_local_actions = false;
		z_policy->at_local_glides_active = false;
		z_policy->at_local_tiers_active = false;
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

	/* V3 self-calibration tail.  Gated by the static branch
	 * (FALSE while auto_tune_v3 = 0) and the live tunables scalar
	 * (defends against a momentary tear during a sysfs store; the
	 * branch can be true while the scalar transitions back to 0).
	 * Internal cadence gate (auto_tune_v3_interval_ms) limits the
	 * actual work to once per 10..600 s.  See ZENITH_DEFAULT_AUTO_TUNE_V3
	 * comment block.
	 */
	if (static_branch_unlikely(&zenith_auto_tune_v3_key)) {
		unsigned int v3_mode = READ_ONCE(t->auto_tune_v3);

		if (v3_mode != ZENITH_AT_V3_MODE_OFF)
			zenith_at_v3_calibrate(z_policy, v3_mode);
	}

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

/* auto_tune_v2_glides sysfs knob.  Boolean (0/1).  See
 * ZENITH_DEFAULT_AUTO_TUNE_V2_GLIDES.  Master gate for V2-driven
 * population of the round-U-z10 glide / coordination knobs;
 * defaults to 1 so the new soft-glide behaviour is on out of the
 * box without forcing operators to write seven separate sysfs
 * entries.  Per-knob user writes still take precedence.
 */
static ssize_t auto_tune_v2_glides_show(struct gov_attr_set *attr_set,
					char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_tune_v2_glides);
}

static ssize_t auto_tune_v2_glides_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->auto_tune_v2_glides = val;
	return count;
}
static struct governor_attr auto_tune_v2_glides =
	__ATTR_RW(auto_tune_v2_glides);

/* auto_tune_v2_tiers sysfs knob (Patch L).  Boolean (0/1).  See
 * ZENITH_DEFAULT_AUTO_TUNE_V2_TIERS.  Master gate for V2-driven
 * arming of the Stage-4 K1/K2/K3 floor tiers; defaults to 1 so
 * the V2 classifier shapes those tiers per state out of the box.
 * Set to 0 to lock all three tiers back to pure profile-driven
 * behaviour without disabling the rest of the V2 classifier.
 * Per-knob user sysfs writes still take precedence either way
 * (via the auto_tune_override_mask path).
 */
static ssize_t auto_tune_v2_tiers_show(struct gov_attr_set *attr_set,
				       char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_tune_v2_tiers);
}

static ssize_t auto_tune_v2_tiers_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->auto_tune_v2_tiers = val;
	return count;
}

static struct governor_attr auto_tune_v2_tiers =
	__ATTR_RW(auto_tune_v2_tiers);

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

static ssize_t auto_tune_v2_var_promote_thresh_show(struct gov_attr_set *attr_set,
						    char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_tune_v2_var_promote_thresh);
}

static ssize_t auto_tune_v2_var_promote_thresh_store(struct gov_attr_set *attr_set,
						     const char *buf,
						     size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_AT_V2_VAR_PROMOTE_THRESH_MAX)
		return -EINVAL;
	t->auto_tune_v2_var_promote_thresh = val;
	return count;
}

static struct governor_attr auto_tune_v2_var_promote_thresh =
	__ATTR_RW(auto_tune_v2_var_promote_thresh);

/* auto_tune_v3 sysfs knob (RW).  See ZENITH_DEFAULT_AUTO_TUNE_V3
 * comment block for full semantics.  Three accepted values:
 *
 *   0  off (default)
 *   1  observe-only (collect telemetry, do not adjust)
 *   2  apply       (collect telemetry AND adjust V2 hyst/cool windows)
 *
 * Store-side: clamps to [0, ZENITH_AT_V3_MODE_MAX], syncs the
 * zenith_auto_tune_v3_key static branch, and -- on a transition to 0
 * -- clears any accumulated offsets so a subsequent re-enable starts
 * from a clean baseline.
 */
static ssize_t auto_tune_v3_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->auto_tune_v3);
}

static ssize_t auto_tune_v3_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_policy;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_AT_V3_MODE_MAX)
		return -EINVAL;
	t->auto_tune_v3 = val;
	zenith_set_static_key(&zenith_auto_tune_v3_key, val);
	if (val == ZENITH_AT_V3_MODE_OFF) {
		list_for_each_entry(z_policy, &attr_set->policy_list,
				    tunables_hook) {
			z_policy->at_v3_hyst_offset = 0;
			z_policy->at_v3_cool_offset = 0;
			z_policy->at_v3_last_calib_ns = 0;
			z_policy->at_v3_last_transitions = 0;
		}
	}
	return count;
}
static struct governor_attr auto_tune_v3 = __ATTR_RW(auto_tune_v3);

/* auto_tune_v3_interval_ms sysfs knob (RW).  Calibration period in
 * milliseconds.  Default ZENITH_DEFAULT_AT_V3_INTERVAL_MS (60000);
 * clamped on store to [ZENITH_AT_V3_INTERVAL_MIN_MS,
 * ZENITH_AT_V3_INTERVAL_MAX_MS].
 */
static ssize_t auto_tune_v3_interval_ms_show(struct gov_attr_set *attr_set,
					     char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_tune_v3_interval_ms);
}

static ssize_t auto_tune_v3_interval_ms_store(struct gov_attr_set *attr_set,
					      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val < ZENITH_AT_V3_INTERVAL_MIN_MS)
		val = ZENITH_AT_V3_INTERVAL_MIN_MS;
	if (val > ZENITH_AT_V3_INTERVAL_MAX_MS)
		val = ZENITH_AT_V3_INTERVAL_MAX_MS;
	t->auto_tune_v3_interval_ms = val;
	return count;
}
static struct governor_attr auto_tune_v3_interval_ms =
	__ATTR_RW(auto_tune_v3_interval_ms);

/* auto_tune_v3_state sysfs knob (RO).  One line per online policy on
 * which V3 has been observed at least once:
 *
 *   policy<N>: transitions=<n> hyst_offset=<-1..+4> cool_offset=<-1..+4>
 *
 * transitions is the V2 state-transition count from the most recent
 * calibration window; the offsets are the live signed nudges (only
 * actually applied when auto_tune_v3 == 2).
 */
static ssize_t auto_tune_v3_state_show(struct gov_attr_set *attr_set,
				       char *buf)
{
	struct zenith_policy *z_policy;
	ssize_t pos = 0;

	list_for_each_entry(z_policy, &attr_set->policy_list, tunables_hook) {
		pos += scnprintf(buf + pos, PAGE_SIZE - pos,
			"policy%u: transitions=%u hyst_offset=%d cool_offset=%d\n",
			z_policy->policy ? z_policy->policy->cpu : 0,
			z_policy->at_v3_last_transitions,
			(int)z_policy->at_v3_hyst_offset,
			(int)z_policy->at_v3_cool_offset);
		if (pos >= PAGE_SIZE - 80)
			break;
	}
	return pos;
}
static struct governor_attr auto_tune_v3_state =
	__ATTR_RO(auto_tune_v3_state);

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

/* Input event rate thresholds, doubled.  Practical max in normal
 * use is a few hundred (touch event stream is 50-200 / s, doubled
 * gives 100-400); 65535 is a generous upper bound that still keeps
 * the field well inside u16 range so future packing into the V2
 * status struct stays free.
 */
ZENITH_TUNABLE_UINT_MAX(auto_tune_hi_events_x2, 65535);
ZENITH_TUNABLE_UINT_MAX(auto_tune_lo_events_x2, 65535);

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
		z_policy->at_last_applied_state = ZENITH_AT_STATE_BALANCED;
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

/* Bump ZENITH_AT_STATUS_FORMAT_VERSION whenever the layout of
 * auto_tune_status changes (new fields, reordering, renaming) so
 * userspace parsers can opt in / fail soft on unknown versions.
 * Field additions in trailing positions stay backwards-compatible
 * within the same major version.
 */
#define ZENITH_AT_STATUS_FORMAT_VERSION		1

static ssize_t auto_tune_status_show(struct gov_attr_set *attr_set, char *buf)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_pol;
	ssize_t len = 0;

	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "version=%u\n", ZENITH_AT_STATUS_FORMAT_VERSION);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "auto_tune=%u\n", t->auto_tune);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "auto_tune_v2=%u glides=%u tiers=%u\n",
			 t->auto_tune_v2, t->auto_tune_v2_glides,
			 t->auto_tune_v2_tiers);
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
				 "policy%u(%s): state=%s applied_state=%s pending=%s pending_windows=%u cooldown=%u reason=%s target=%s samples=%u saturated=%u sat_pct=%u events_x2=%u flags=0x%x var_x256=%u psi=%u/%u/%u thermal=%u+%u frame_us=%u local=%u eff_rate=%u/%u eff_thresh=%u/%u eff_boost=%u/%u eff_frame=%u eff_game=%u\n",
				 z_pol->policy->cpu,
				 zenith_at_cluster_name(z_pol->cluster_class),
				 zenith_at_state_name(z_pol->at_last_state),
				 zenith_at_state_name(z_pol->at_last_applied_state),
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

/* M1: time-in-state RO sysfs.  Format: one line per (policy, state)
 * tuple:
 *
 *   policy<cpu>(<cluster>) state=<state> residency_ns=<ns>
 *
 * Values are cumulative nanoseconds since governor start.  The
 * still-current state's counter is "live" -- it does not include
 * the time elapsed since the last commit (that delta is implicit
 * in the difference between sum-of-counters and uptime).
 * Userspace tools should treat this as monotonically non-decreasing
 * and compute differences across two reads to derive percent-time-
 * in-state for an arbitrary window.
 *
 * One line per state keeps each scnprintf() narrow enough to fit
 * in 100 columns and matches the at_log / state_history layout that
 * other zenith RO surfaces use, so awk / cut pipelines can be
 * reused.
 *
 * No reset hook -- a fresh boot zeroes the values, and
 * profile_store() does not zap residency (so cross-profile
 * comparisons stay legible).
 */
static ssize_t auto_tune_state_residency_show(struct gov_attr_set *attr_set,
					      char *buf)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_pol;
	ssize_t len = 0;
	unsigned int s;

	list_for_each_entry(z_pol, &t->attr_set.policy_list, tunables_hook) {
		for (s = 0; s < ARRAY_SIZE(z_pol->at_state_residency_ns); s++) {
			len += scnprintf(buf + len, PAGE_SIZE - len,
					 "policy%u(%s) state=%s residency_ns=%llu\n",
					 z_pol->policy->cpu,
					 zenith_at_cluster_name(
						z_pol->cluster_class),
					 zenith_at_state_name(s),
					 z_pol->at_state_residency_ns[s]);
			if (len >= PAGE_SIZE)
				return len;
		}
	}
	return len;
}

static struct governor_attr auto_tune_state_residency =
	__ATTR_RO(auto_tune_state_residency);

/* M2: V2 state-transition history RO sysfs.  One line per recorded
 * transition, most recent first, format:
 *
 *   policy<cpu>(<cluster>) ts=<ns> from=<state> to=<state> reason=<r> flags=0x<f>
 *
 * Bounded to ZENITH_AT_HISTORY_NR entries per policy.  The output
 * may exceed PAGE_SIZE on a fully-populated 8-cluster system; the
 * scnprintf early-out below truncates cleanly.
 *
 * Lock-free single-writer ring (V2 worker), reader (this show) may
 * see one torn entry on wrap.  Acceptable for diagnostics.
 */
static ssize_t auto_tune_state_history_show(struct gov_attr_set *attr_set,
					    char *buf)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_pol;
	ssize_t len = 0;
	unsigned int i, slot, head, count;

	list_for_each_entry(z_pol, &t->attr_set.policy_list, tunables_hook) {
		head = z_pol->at_history_head;
		count = z_pol->at_history_count;
		for (i = 0; i < count; i++) {
			/* Walk newest -> oldest.  Newest entry is at
			 * (head - 1) mod NR; subtract i more to step
			 * back through the ring.
			 */
			slot = (head + ZENITH_AT_HISTORY_NR - 1 - i) %
			       ZENITH_AT_HISTORY_NR;
			len += scnprintf(buf + len, PAGE_SIZE - len,
					 "policy%u(%s) ts=%llu from=%s to=%s reason=%s flags=0x%x\n",
					 z_pol->policy->cpu,
					 zenith_at_cluster_name(z_pol->cluster_class),
					 z_pol->at_history[slot].ts_ns,
					 zenith_at_state_name(z_pol->at_history[slot].from),
					 zenith_at_state_name(z_pol->at_history[slot].to),
					 zenith_at_reason_name(z_pol->at_history[slot].reason),
					 z_pol->at_history[slot].flags);
			if (len >= PAGE_SIZE)
				return len;
		}
	}
	return len;
}

static struct governor_attr auto_tune_state_history =
	__ATTR_RO(auto_tune_state_history);

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
	u32 saved_touchdown_extra_ms =
		READ_ONCE(zenith_input_boost_touchdown_extra_ms_cache);
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

	/* Restore the boost-active and touchdown-extra mirrors that
	 * zenith_apply_profile() stamps on every call.  Use WRITE_ONCE
	 * to match the writer semantics elsewhere in the file.
	 */
	WRITE_ONCE(zenith_input_boost_active_ms, saved_active_ms);
	WRITE_ONCE(zenith_input_boost_touchdown_extra_ms_cache,
		   saved_touchdown_extra_ms);
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
		[ZENITH_STAT_PREDICT_UP]	= "predict_up",
		[ZENITH_STAT_PEAK_PREARM]	= "peak_prearm",
		[ZENITH_STAT_PEAK_RESCUE]	= "peak_rescue",
		[ZENITH_STAT_PEAK_HYST]		= "peak_hyst",
		[ZENITH_STAT_PEER_RAMP]		= "peer_ramp",
		[ZENITH_STAT_MIGRATION_FLOOR]	= "migration_floor",
		[ZENITH_STAT_PSI_CPU_FLOOR]	= "psi_cpu_floor",
		[ZENITH_STAT_FRAME_OVERRUN]	= "frame_overrun",
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

/* Stage 4 / Patch I -- governor-wide input observability sysfs node.
 *
 * Read-only; emits one "name=value" line per atomic counter so
 * userspace can scrape by field name.  All counters are
 * monotonic-since-boot atomic64s so the reader's job is to subtract
 * a stored snapshot to get a rate.  See the block comment above
 * zenith_in_events_total for what each counter records.
 *
 * The counters are governor-wide (one set, not per-policy) because
 * input events are observed once per dispatch and applied to all
 * policies that have boosting enabled.  Putting the node on the
 * gov_attr_set means it appears under every cpufreq policy
 * directory but reads the same global atomics.
 */
static ssize_t zenith_input_stats_show(struct gov_attr_set *attr_set,
				       char *buf)
{
	return sprintf(buf,
		"events_total=%llu\n"
		"boosts_armed=%llu\n"
		"boosts_quiet_extended=%llu\n"
		"boosts_skipped_disabled=%llu\n"
		"boosts_early_exit=%llu\n",
		(unsigned long long)atomic64_read(&zenith_in_events_total),
		(unsigned long long)atomic64_read(&zenith_in_boosts_armed),
		(unsigned long long)atomic64_read(
			&zenith_in_boosts_quiet_extended),
		(unsigned long long)atomic64_read(
			&zenith_in_boosts_skipped_disabled),
		(unsigned long long)atomic64_read(
			&zenith_in_boosts_early_exit));
}

static struct governor_attr zenith_input_stats =
	__ATTR_RO(zenith_input_stats);

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

/* last_decision_path sysfs node (Patch J).  Read-only.  Dumps
 * one line per policy in attr_set->policy_list with the most
 * recent tp_path tag chosen by zenith_get_next_freq() on that
 * policy.  Format:
 *
 *   policy<cpu>(<cluster>): <tag>
 *
 * Tag pointers are .rodata literals stamped via WRITE_ONCE in
 * the eval path; reads use READ_ONCE so a torn pointer is
 * impossible.  Initial value before the first eval tick is
 * "init", set in zenith_init().
 */
static ssize_t last_decision_path_show(struct gov_attr_set *attr_set,
				       char *buf)
{
	struct zenith_policy *z_pol;
	ssize_t len = 0;

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		const char *tag = READ_ONCE(z_pol->last_decision_path);

		if (!tag)
			tag = "init";
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "policy%u(%s): %s\n",
				 z_pol->policy->cpu,
				 zenith_at_cluster_name(z_pol->cluster_class),
				 tag);
		if (len >= PAGE_SIZE)
			break;
	}
	return len;
}

static struct governor_attr last_decision_path =
	__ATTR_RO(last_decision_path);

ZENITH_TUNABLE_UINT_BOOL_INVAL(screen_state);

/* screen_off_glide_ms sysfs knob.  Range
 * 0..ZENITH_SCREEN_OFF_GLIDE_MS_MAX.  See struct zenith_tunables for
 * semantics.  0 keeps the legacy hard cliff at the screen-state 1->0
 * edge; non-zero arms a linear ramp on both dynamic_up_thresh and
 * dynamic_bias from the natural up_threshold / powersave_bias to
 * the cliff targets across the configured window.
 */
static ssize_t screen_off_glide_ms_show(struct gov_attr_set *attr_set,
					char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->screen_off_glide_ms);
}

static ssize_t screen_off_glide_ms_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_SCREEN_OFF_GLIDE_MS_MAX)
		return -EINVAL;
	t->screen_off_glide_ms = val;
	return count;
}
static struct governor_attr screen_off_glide_ms =
	__ATTR_RW(screen_off_glide_ms);

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
ZENITH_TUNABLE_UINT_BOOL_INVAL(thermal_state);

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

/* wakeup_boost_ms sysfs knob.  Range 0..ZENITH_WAKEUP_BOOST_MS_MAX.
 * 0 disables the wall-clock bypass and leaves only the legacy
 * tick-based ZENITH_WAKEUP_BOOST_TICKS countdown.  Non-zero arms a
 * deadline at the detection sites; the up-rate bypass holds until
 * either the tick counter expires or the deadline lapses.
 */
static ssize_t wakeup_boost_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->wakeup_boost_ms);
}

static ssize_t wakeup_boost_ms_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_WAKEUP_BOOST_MS_MAX)
		return -EINVAL;
	t->wakeup_boost_ms = val;
	return count;
}
static struct governor_attr wakeup_boost_ms = __ATTR_RW(wakeup_boost_ms);

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

/* input_boost_touchdown_extra_ms sysfs knob (Patch C).
 *
 * Extends the input-boost active window by an extra
 * input_boost_touchdown_extra_ms ms on EV_KEY/BTN_TOUCH press.
 * 0 disables the touchdown extra.  Capped at
 * ZENITH_INPUT_BOOST_TOUCHDOWN_EXTRA_MS_MAX so a runaway echo
 * can't pin the cluster up indefinitely.
 *
 * Mirrors the stored value to
 * zenith_input_boost_touchdown_extra_ms_cache so the input fast
 * path doesn't have to walk the tunables list.
 */
static ssize_t
input_boost_touchdown_extra_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->input_boost_touchdown_extra_ms);
}

static ssize_t
input_boost_touchdown_extra_ms_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_INPUT_BOOST_TOUCHDOWN_EXTRA_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->input_boost_touchdown_extra_ms, val);
	WRITE_ONCE(zenith_input_boost_touchdown_extra_ms_cache, val);
	return count;
}

static struct governor_attr input_boost_touchdown_extra_ms =
	__ATTR_RW(input_boost_touchdown_extra_ms);

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

/* input_boost_down_rate_mult_pct sysfs knob.  Range
 * 100..ZENITH_INPUT_BOOST_DOWN_RATE_MULT_PCT_MAX (1000).  See
 * ZENITH_DEFAULT_INPUT_BOOST_DOWN_RATE_MULT_PCT for full semantics.
 */
static ssize_t input_boost_down_rate_mult_pct_show(struct gov_attr_set *attr_set,
						   char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->input_boost_down_rate_mult_pct);
}

static ssize_t input_boost_down_rate_mult_pct_store(struct gov_attr_set *attr_set,
						    const char *buf,
						    size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val < 100 ||
	    val > ZENITH_INPUT_BOOST_DOWN_RATE_MULT_PCT_MAX)
		return -EINVAL;
	t->input_boost_down_rate_mult_pct = val;
	return count;
}

static struct governor_attr input_boost_down_rate_mult_pct =
	__ATTR_RW(input_boost_down_rate_mult_pct);

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

/* peak_headroom_rescue sysfs knob.  Boolean gate for the watchdog
 * tier that lifts the cluster off a sustained-high-util / sub-peak
 * floor.  See ZENITH_DEFAULT_PEAK_HEADROOM_RESCUE for the full
 * rationale.  Accepts 0 (disabled) or 1 (enabled, default); any
 * other value is rejected.
 */
static ssize_t peak_headroom_rescue_show(struct gov_attr_set *attr_set,
					 char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peak_headroom_rescue);
}

static ssize_t peak_headroom_rescue_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->peak_headroom_rescue = val;
	return count;
}

static struct governor_attr peak_headroom_rescue =
	__ATTR_RW(peak_headroom_rescue);

/* peak_headroom_starve_load_pct sysfs knob.  Minimum cluster
 * load_pct (0..100, util / max_cap * 100) at which a sample counts
 * as "starving" for the peak-headroom rescue tier.  Accepts 1..100;
 * 0 is rejected (rescue would fire on every sample).
 */
static ssize_t peak_headroom_starve_load_pct_show(struct gov_attr_set *attr_set,
						  char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peak_headroom_starve_load_pct);
}

static ssize_t peak_headroom_starve_load_pct_store(struct gov_attr_set *attr_set,
						   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val == 0 || val > 100)
		return -EINVAL;
	t->peak_headroom_starve_load_pct = val;
	return count;
}

static struct governor_attr peak_headroom_starve_load_pct =
	__ATTR_RW(peak_headroom_starve_load_pct);

/* peak_headroom_freq_floor_pct sysfs knob.  Maximum fraction of
 * policy->max (1..100) below which the cluster freq must sit before
 * a sample counts as "starving" for the rescue tier.  100 means
 * "any time freq < policy->max"; smaller values demand a deeper
 * sub-peak gap before triggering.  0 is rejected (any positive freq
 * would be above 0% of policy->max so the freq guard would never
 * fire).
 */
static ssize_t peak_headroom_freq_floor_pct_show(struct gov_attr_set *attr_set,
						 char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peak_headroom_freq_floor_pct);
}

static ssize_t peak_headroom_freq_floor_pct_store(struct gov_attr_set *attr_set,
						  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val == 0 || val > 100)
		return -EINVAL;
	t->peak_headroom_freq_floor_pct = val;
	return count;
}

static struct governor_attr peak_headroom_freq_floor_pct =
	__ATTR_RW(peak_headroom_freq_floor_pct);

/* peak_headroom_starve_streak sysfs knob.  Number of consecutive
 * starving samples required before the rescue may fire.  Accepts
 * 0..PEAK_HEADROOM_STREAK_MAX (16) since the underlying counter is
 * a u8 saturating at that value.  0 fires on the very first
 * starving sample.
 */
static ssize_t peak_headroom_starve_streak_show(struct gov_attr_set *attr_set,
						char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peak_headroom_starve_streak);
}

static ssize_t peak_headroom_starve_streak_store(struct gov_attr_set *attr_set,
						 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_PEAK_HEADROOM_STREAK_MAX)
		return -EINVAL;
	t->peak_headroom_starve_streak = val;
	return count;
}

static struct governor_attr peak_headroom_starve_streak =
	__ATTR_RW(peak_headroom_starve_streak);

/* peak_headroom_jump_pct sysfs knob.  Target as percentage of
 * policy->max for the rescue freq.  Accepts 1..100; 100 pins to
 * policy->max (the default), 50 rescues to half of policy->max,
 * etc.  0 is rejected (rescue with target 0 would never raise
 * freq).
 */
static ssize_t peak_headroom_jump_pct_show(struct gov_attr_set *attr_set,
					   char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peak_headroom_jump_pct);
}

static ssize_t peak_headroom_jump_pct_store(struct gov_attr_set *attr_set,
					    const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val == 0 || val > 100)
		return -EINVAL;
	t->peak_headroom_jump_pct = val;
	return count;
}

static struct governor_attr peak_headroom_jump_pct =
	__ATTR_RW(peak_headroom_jump_pct);

/* peak_headroom_hold_ms sysfs knob.  Minimum gap between two
 * rescue fires, in milliseconds.  Accepts
 * 0..PEAK_HEADROOM_HOLD_MS_MAX (1000); 0 disables the hold-down
 * (rescue can fire on every starving sample past the streak),
 * larger values rate-limit more aggressively.
 */
static ssize_t peak_headroom_hold_ms_show(struct gov_attr_set *attr_set,
					  char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peak_headroom_hold_ms);
}

static ssize_t peak_headroom_hold_ms_store(struct gov_attr_set *attr_set,
					   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PEAK_HEADROOM_HOLD_MS_MAX)
		return -EINVAL;
	t->peak_headroom_hold_ms = val;
	return count;
}

static struct governor_attr peak_headroom_hold_ms =
	__ATTR_RW(peak_headroom_hold_ms);

/* peak_headroom_prearm sysfs knob.  Boolean gate for the soft early
 * intervention tier (2b') that lifts the cluster to eff_hispeed_freq
 * while the starvation streak is accumulating but has not yet
 * crossed peak_headroom_starve_streak.  Accepts 0 or 1 only.
 */
static ssize_t peak_headroom_prearm_show(struct gov_attr_set *attr_set,
					 char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peak_headroom_prearm);
}

static ssize_t peak_headroom_prearm_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->peak_headroom_prearm = val;
	return count;
}

static struct governor_attr peak_headroom_prearm =
	__ATTR_RW(peak_headroom_prearm);

/* predict_up_thresh sysfs knob.  Trend threshold for the
 * predictive up-shift tier (2a') in 256ths of max_cap; see the
 * block comment above ZENITH_DEFAULT_PREDICT_UP_THRESH for what
 * the unit means and how the trigger is gated.  0 disables the
 * tier, max ZENITH_PREDICT_UP_THRESH_MAX (255).
 */
static ssize_t predict_up_thresh_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->predict_up_thresh);
}

static ssize_t predict_up_thresh_store(struct gov_attr_set *attr_set,
				       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_PREDICT_UP_THRESH_MAX)
		return -EINVAL;
	WRITE_ONCE(t->predict_up_thresh, val);
	return count;
}

static struct governor_attr predict_up_thresh =
	__ATTR_RW(predict_up_thresh);

/* predict_up_window sysfs knob.  Window size for the predict_up
 * trend ring, in samples.  Range
 * [ZENITH_PREDICT_UP_WINDOW_MIN .. ZENITH_PREDICT_UP_WINDOW_MAX]
 * (2..8).  Out-of-range values are rejected.  Larger windows
 * smooth PELT noise out of the trend signal at the cost of one
 * more tick of warm-up after a cold attach.
 */
static ssize_t predict_up_window_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->predict_up_window);
}

static ssize_t predict_up_window_store(struct gov_attr_set *attr_set,
				       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val < ZENITH_PREDICT_UP_WINDOW_MIN ||
	    val > ZENITH_PREDICT_UP_WINDOW_MAX)
		return -EINVAL;
	WRITE_ONCE(t->predict_up_window, val);
	return count;
}

static struct governor_attr predict_up_window =
	__ATTR_RW(predict_up_window);

/* peak_hysteresis_streak sysfs knob (Patch E).
 * Range 0..ZENITH_PEAK_HYSTERESIS_STREAK_MAX.  Number of
 * consecutive samples after a peak-class previous freq for
 * which the soft floor is held; 0 disables the hysteresis tier.
 */
static ssize_t
peak_hysteresis_streak_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peak_hysteresis_streak);
}

static ssize_t
peak_hysteresis_streak_store(struct gov_attr_set *attr_set,
			     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PEAK_HYSTERESIS_STREAK_MAX)
		return -EINVAL;
	WRITE_ONCE(t->peak_hysteresis_streak, val);
	return count;
}

static struct governor_attr peak_hysteresis_streak =
	__ATTR_RW(peak_hysteresis_streak);

/* peak_step_down_pct sysfs knob (Patch E).
 * Range 0..100.  Soft-floor freq for the hysteresis tier as a
 * percentage of the peak-anchor freq; 0 disables the tier.  100
 * makes the floor identical to the anchor (the cluster won't
 * descend at all during the streak).  95 (default) gives a 5%
 * stair-step descent.
 */
static ssize_t
peak_step_down_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peak_step_down_pct);
}

static ssize_t
peak_step_down_pct_store(struct gov_attr_set *attr_set,
			 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	WRITE_ONCE(t->peak_step_down_pct, val);
	return count;
}

static struct governor_attr peak_step_down_pct =
	__ATTR_RW(peak_step_down_pct);

/* boost_idle_thresh sysfs knob (Patch F).
 * Range 0..100.  load_pct below which a boost-active tick is
 * counted toward the persistent-idle streak.  0 disables the
 * boost early-exit (boost is always honoured to its deadline).
 */
static ssize_t
boost_idle_thresh_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->boost_idle_thresh);
}

static ssize_t
boost_idle_thresh_store(struct gov_attr_set *attr_set,
			const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	WRITE_ONCE(t->boost_idle_thresh, val);
	return count;
}

static struct governor_attr boost_idle_thresh =
	__ATTR_RW(boost_idle_thresh);

/* boost_idle_streak sysfs knob (Patch F).
 * Range 0..ZENITH_BOOST_IDLE_STREAK_MAX.  Number of consecutive
 * idle ticks (load_pct < boost_idle_thresh) inside an active
 * boost window before the policy preempts the boost on its
 * tier-0 path.  0 disables the boost early-exit.
 */
static ssize_t
boost_idle_streak_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->boost_idle_streak);
}

static ssize_t
boost_idle_streak_store(struct gov_attr_set *attr_set,
			const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_BOOST_IDLE_STREAK_MAX)
		return -EINVAL;
	WRITE_ONCE(t->boost_idle_streak, val);
	return count;
}

static struct governor_attr boost_idle_streak =
	__ATTR_RW(boost_idle_streak);

/* bg_util_scale_pct sysfs knob (Patch G).
 * Range ZENITH_BG_UTIL_SCALE_PCT_MIN..100.  Scales the util
 * signal returned by zenith_get_util() down to this percent
 * when the display is off.  100 is a pass-through (default).
 * 0 is rejected -- this is a scale knob, not a gate.
 */
static ssize_t
bg_util_scale_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->bg_util_scale_pct);
}

static ssize_t
bg_util_scale_pct_store(struct gov_attr_set *attr_set,
			const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val < ZENITH_BG_UTIL_SCALE_PCT_MIN || val > 100)
		return -EINVAL;
	WRITE_ONCE(t->bg_util_scale_pct, val);
	return count;
}

static struct governor_attr bg_util_scale_pct =
	__ATTR_RW(bg_util_scale_pct);

/* sleeper_tail_thresh_us sysfs knob (Patch H).
 * Range 0..ZENITH_SLEEPER_TAIL_THRESH_US_MAX.  0 disables.
 */
static ssize_t
sleeper_tail_thresh_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->sleeper_tail_thresh_us);
}

static ssize_t
sleeper_tail_thresh_us_store(struct gov_attr_set *attr_set,
			     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_SLEEPER_TAIL_THRESH_US_MAX)
		return -EINVAL;
	WRITE_ONCE(t->sleeper_tail_thresh_us, val);
	return count;
}

static struct governor_attr sleeper_tail_thresh_us =
	__ATTR_RW(sleeper_tail_thresh_us);

/* sleeper_tail_pct sysfs knob (Patch H).
 * Range ZENITH_SLEEPER_TAIL_PCT_MIN..ZENITH_SLEEPER_TAIL_PCT_MAX.
 */
static ssize_t
sleeper_tail_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->sleeper_tail_pct);
}

static ssize_t
sleeper_tail_pct_store(struct gov_attr_set *attr_set,
		       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val < ZENITH_SLEEPER_TAIL_PCT_MIN ||
	    val > ZENITH_SLEEPER_TAIL_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->sleeper_tail_pct, val);
	return count;
}

static struct governor_attr sleeper_tail_pct =
	__ATTR_RW(sleeper_tail_pct);

/* peer_ramp_window_ms sysfs knob (Patch D).
 * Range 0..ZENITH_PEER_RAMP_WINDOW_MS_MAX (100).  0 disables the
 * peer-ramp coordination on both sides (no arming writes from
 * the peak tiers, no floor reads).  Non-zero values are the
 * post-peak window during which the peer cluster applies the
 * soft floor after this cluster ramps.
 */
static ssize_t
peer_ramp_window_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peer_ramp_window_ms);
}

static ssize_t
peer_ramp_window_ms_store(struct gov_attr_set *attr_set,
			  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PEER_RAMP_WINDOW_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->peer_ramp_window_ms, val);
	return count;
}

static struct governor_attr peer_ramp_window_ms =
	__ATTR_RW(peer_ramp_window_ms);

/* peer_ramp_window_off_ms sysfs knob (Patch M3).
 * Range 0..ZENITH_PEER_RAMP_WINDOW_MS_MAX (100).  Screen-state-
 * aware shadow of peer_ramp_window_ms: when tunables->screen_state
 * is 0 the peer-ramp arming and floor-eval paths use this value
 * instead.  0 (default) suppresses peer_ramp entirely while the
 * screen is off.  Set equal to peer_ramp_window_ms to restore the
 * pre-Stage-5 always-on behaviour byte-identically.
 */
static ssize_t
peer_ramp_window_off_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peer_ramp_window_off_ms);
}

static ssize_t
peer_ramp_window_off_ms_store(struct gov_attr_set *attr_set,
			      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PEER_RAMP_WINDOW_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->peer_ramp_window_off_ms, val);
	return count;
}

static struct governor_attr peer_ramp_window_off_ms =
	__ATTR_RW(peer_ramp_window_off_ms);

/* peer_ramp_floor_pct sysfs knob (Patch D).
 * Range 0..ZENITH_PEER_RAMP_FLOOR_PCT_MAX (100).  Soft floor as
 * a percent of policy->max applied while a peer-ramp deadline
 * is active.  0 leaves the deadlines being stamped (visible to
 * trace consumers) but suppresses the floor itself.
 */
static ssize_t
peer_ramp_floor_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->peer_ramp_floor_pct);
}

static ssize_t
peer_ramp_floor_pct_store(struct gov_attr_set *attr_set,
			  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PEER_RAMP_FLOOR_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->peer_ramp_floor_pct, val);
	return count;
}

static struct governor_attr peer_ramp_floor_pct =
	__ATTR_RW(peer_ramp_floor_pct);

/* migration_jump_pct (Patch K1).  Range 0..100.  0 disables both
 * the per-CPU stamping and the floor read.
 */
static ssize_t
migration_jump_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->migration_jump_pct);
}

static ssize_t
migration_jump_pct_store(struct gov_attr_set *attr_set,
			 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_MIGRATION_JUMP_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->migration_jump_pct, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_MIGRATION_JUMP);
	return count;
}

static struct governor_attr migration_jump_pct =
	__ATTR_RW(migration_jump_pct);

/* migration_floor_window_ms (Patch K1).  Range 0..100.  Length of
 * the post-arrival window in milliseconds.  0 disables the
 * stamping (the detector still updates migration_prev_util but
 * does not arm a deadline).
 */
static ssize_t
migration_floor_window_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
		       migration_floor_window_ms);
}

static ssize_t
migration_floor_window_ms_store(struct gov_attr_set *attr_set,
				const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_MIGRATION_FLOOR_WINDOW_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->migration_floor_window_ms, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_MIGRATION_FLOOR_WIN);
	return count;
}

static struct governor_attr migration_floor_window_ms =
	__ATTR_RW(migration_floor_window_ms);

/* migration_floor_pct (Patch K1).  Range 0..100.  Soft floor as
 * a percent of policy->max while a migration deadline is active.
 * 0 leaves the deadline arming visible to stats / trace but
 * suppresses the freq adjustment.
 */
static ssize_t
migration_floor_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->migration_floor_pct);
}

static ssize_t
migration_floor_pct_store(struct gov_attr_set *attr_set,
			  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_MIGRATION_FLOOR_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->migration_floor_pct, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_MIGRATION_FLOOR_PCT);
	return count;
}

static struct governor_attr migration_floor_pct =
	__ATTR_RW(migration_floor_pct);

/* psi_cpu_floor_thresh (Patch K2).  Range 0..100.  0 disables.
 * When >= this percent of system-wide PSI_CPU_SOME 10s EWMA is
 * observed, the eval lifts to zenith_eff_hispeed_freq().  Mirror
 * of psi_cpu_thresh on the floor side.  See the
 * ZENITH_DEFAULT_PSI_CPU_FLOOR_THRESH comment block for the full
 * rationale.
 */
static ssize_t
psi_cpu_floor_thresh_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->psi_cpu_floor_thresh);
}

static ssize_t
psi_cpu_floor_thresh_store(struct gov_attr_set *attr_set,
			   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PSI_CPU_FLOOR_THRESH_MAX)
		return -EINVAL;
	WRITE_ONCE(t->psi_cpu_floor_thresh, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_PSI_CPU_FLOOR);
	return count;
}

static struct governor_attr psi_cpu_floor_thresh =
	__ATTR_RW(psi_cpu_floor_thresh);

/* frame_overrun_slack_us (Patch K3).  Range 0..16667 us.
 * Tolerance beyond the cached vblank period before treating a
 * gap as an overrun.  0 disables stamping (the producer still
 * updates last_vblank_ns).  Mirrored into the governor-wide
 * cache so the producer can read it without a tunables lookup.
 */
static ssize_t
frame_overrun_slack_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->frame_overrun_slack_us);
}

static ssize_t
frame_overrun_slack_us_store(struct gov_attr_set *attr_set,
			     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_FRAME_OVERRUN_SLACK_US_MAX)
		return -EINVAL;
	WRITE_ONCE(t->frame_overrun_slack_us, val);
	WRITE_ONCE(zenith_frame_overrun_slack_us_cache, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_FRAME_OVR_SLACK);
	return count;
}

static struct governor_attr frame_overrun_slack_us =
	__ATTR_RW(frame_overrun_slack_us);

/* frame_overrun_window_ms (Patch K3).  Range 0..200 ms.  How
 * long the soft floor holds after an overrun is detected.  0
 * suppresses stamping but still updates last_vblank_ns.  Also
 * mirrored to the governor-wide cache.
 */
static ssize_t
frame_overrun_window_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
		       frame_overrun_window_ms);
}

static ssize_t
frame_overrun_window_ms_store(struct gov_attr_set *attr_set,
			      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_FRAME_OVERRUN_WINDOW_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->frame_overrun_window_ms, val);
	WRITE_ONCE(zenith_frame_overrun_window_ms_cache, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_FRAME_OVR_WINDOW);
	return count;
}

static struct governor_attr frame_overrun_window_ms =
	__ATTR_RW(frame_overrun_window_ms);

/* frame_overrun_floor_pct (Patch K3).  Range 0..100.  Soft
 * floor as a percent of policy->max while the overrun deadline
 * is active.  0 leaves stamping in place but suppresses the
 * floor.
 */
static ssize_t
frame_overrun_floor_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
		       frame_overrun_floor_pct);
}

static ssize_t
frame_overrun_floor_pct_store(struct gov_attr_set *attr_set,
			      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_FRAME_OVERRUN_FLOOR_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->frame_overrun_floor_pct, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_FRAME_OVR_FLOOR);
	return count;
}

static struct governor_attr frame_overrun_floor_pct =
	__ATTR_RW(frame_overrun_floor_pct);

/* frame_overrun_deep_streak sysfs knob (Patch M5).  Range
 * 0..ZENITH_FRAME_OVERRUN_DEEP_STREAK_MAX (16).  When non-zero,
 * after this many consecutive overruns the K3 floor escalates
 * from frame_overrun_floor_pct to frame_overrun_deep_floor_pct.
 * 0 disables the deep tier (the consumer never reads the streak
 * atomic).  See the macro block above struct zenith_tunables
 * for the full design rationale.
 */
static ssize_t
frame_overrun_deep_streak_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
		       frame_overrun_deep_streak);
}

static ssize_t
frame_overrun_deep_streak_store(struct gov_attr_set *attr_set,
				const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_FRAME_OVERRUN_DEEP_STREAK_MAX)
		return -EINVAL;
	WRITE_ONCE(t->frame_overrun_deep_streak, val);
	return count;
}

static struct governor_attr frame_overrun_deep_streak =
	__ATTR_RW(frame_overrun_deep_streak);

/* frame_overrun_deep_floor_pct sysfs knob (Patch M5).  Range
 * 0..ZENITH_FRAME_OVERRUN_DEEP_FLOOR_PCT_MAX (100).  Floor as a
 * percent of policy->max applied while the K3 deadline is active
 * AND the consecutive-overrun streak has crossed
 * frame_overrun_deep_streak.  Effective floor is
 * max(deep_floor_pct, frame_overrun_floor_pct) so the deep tier
 * never produces a lower floor than the standard K3 floor.
 */
static ssize_t
frame_overrun_deep_floor_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
		       frame_overrun_deep_floor_pct);
}

static ssize_t
frame_overrun_deep_floor_pct_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_FRAME_OVERRUN_DEEP_FLOOR_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->frame_overrun_deep_floor_pct, val);
	return count;
}

static struct governor_attr frame_overrun_deep_floor_pct =
	__ATTR_RW(frame_overrun_deep_floor_pct);

/* psi_mem_cap_thresh / psi_mem_cap_pct / psi_mem_cap_window_ms
 * sysfs knobs (Patch M1).  thresh ranges 0..100 (0 = off);
 * pct ranges 50..100; window_ms ranges 100..5000.  All three
 * mark their override bit on store so the V2 worker stops
 * touching the value until the next profile flip clears the
 * mask.  See the macro block above for the full design.
 */
static ssize_t
psi_mem_cap_thresh_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->psi_mem_cap_thresh);
}

static ssize_t
psi_mem_cap_thresh_store(struct gov_attr_set *attr_set,
			 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PSI_MEM_CAP_THRESH_MAX)
		return -EINVAL;
	WRITE_ONCE(t->psi_mem_cap_thresh, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_PSI_MEM_CAP_THRESH);
	return count;
}

static struct governor_attr psi_mem_cap_thresh =
	__ATTR_RW(psi_mem_cap_thresh);

static ssize_t
psi_mem_cap_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->psi_mem_cap_pct);
}

static ssize_t
psi_mem_cap_pct_store(struct gov_attr_set *attr_set,
		      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val < ZENITH_PSI_MEM_CAP_PCT_MIN ||
	    val > ZENITH_PSI_MEM_CAP_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->psi_mem_cap_pct, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_PSI_MEM_CAP_PCT);
	return count;
}

static struct governor_attr psi_mem_cap_pct =
	__ATTR_RW(psi_mem_cap_pct);

static ssize_t
psi_mem_cap_window_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->psi_mem_cap_window_ms);
}

static ssize_t
psi_mem_cap_window_ms_store(struct gov_attr_set *attr_set,
			    const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val < ZENITH_PSI_MEM_CAP_WINDOW_MS_MIN ||
	    val > ZENITH_PSI_MEM_CAP_WINDOW_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->psi_mem_cap_window_ms, val);
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_PSI_MEM_CAP_WINDOW);
	return count;
}

static struct governor_attr psi_mem_cap_window_ms =
	__ATTR_RW(psi_mem_cap_window_ms);

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

/* screen_on_bias_pct sysfs knob.  See struct zenith_tunables doc and
 * ZENITH_DEFAULT_SCREEN_ON_BIAS_PCT for full semantics.  Range
 * 0..100 (percent).  100 disables the softening (legacy behaviour);
 * 50 (the default) halves the configured powersave_bias whenever
 * the screen is on; 0 zeroes the bias on screen-on.
 */
static ssize_t screen_on_bias_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->screen_on_bias_pct);
}

static ssize_t screen_on_bias_pct_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 100)
		return -EINVAL;
	t->screen_on_bias_pct = val;
	zenith_invalidate_cache(attr_set);
	return count;
}

static struct governor_attr screen_on_bias_pct =
	__ATTR_RW(screen_on_bias_pct);

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

/* render_floor_min_runtime_ms sysfs knob.  Debounce window in
 * milliseconds for the render-thread floor; 0 disables the
 * debounce so the floor fires the moment a render thread is
 * picked up.  See ZENITH_DEFAULT_RENDER_FLOOR_MIN_RUNTIME_MS for
 * details.  Capped at ZENITH_RENDER_FLOOR_MIN_RUNTIME_MS_MAX.
 */
static ssize_t
render_floor_min_runtime_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->render_floor_min_runtime_ms);
}

static ssize_t
render_floor_min_runtime_ms_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_RENDER_FLOOR_MIN_RUNTIME_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->render_floor_min_runtime_ms, val);
	return count;
}

static struct governor_attr render_floor_min_runtime_ms =
	__ATTR_RW(render_floor_min_runtime_ms);

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

static ssize_t game_auto_comms_show(struct gov_attr_set *attr_set, char *buf)
{
	return zenith_show_comm_table(&zenith_game_auto_table, buf);
}

static ssize_t game_auto_comms_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	return zenith_store_comm_table(&zenith_game_auto_table,
				       zenith_game_auto_comms,
				       ARRAY_SIZE(zenith_game_auto_comms),
				       buf, count);
}
static struct governor_attr game_auto_comms = __ATTR_RW(game_auto_comms);

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

/* game_auto sysfs knob.  Master gate for the in-kernel game
 * detector.  See the ZENITH_DEFAULT_GAME_AUTO comment block.  Strict
 * 0/1 boolean.  Syncs the zenith_game_auto_key static branch and,
 * on disable, clears the global zenith_game_auto_active_until_ns
 * latch so a stale "game active" state does not survive game_auto =
 * 0 (otherwise a write of 0 would leave the helper still returning
 * 1 until the existing TTL expired).
 */
static ssize_t game_auto_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_zenith_tunables(attr_set)->game_auto);
}

static ssize_t game_auto_store(struct gov_attr_set *attr_set,
			       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->game_auto = val;
	zenith_set_static_key(&zenith_game_auto_key, val);
	if (!val)
		WRITE_ONCE(zenith_game_auto_active_until_ns, 0);
	return count;
}
static struct governor_attr game_auto = __ATTR_RW(game_auto);

/* game_auto_state sysfs knob.  Read-only view of the global latch
 * state.  Returns 1 when the in-kernel detector currently considers
 * a game active (i.e. a fresh detection landed within
 * ZENITH_GAME_AUTO_ACTIVE_TTL_NS), 0 otherwise.  Useful for tooling
 * that wants to confirm the detector fired, distinct from a manual
 * game_mode write.
 */
static ssize_t game_auto_state_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", zenith_game_auto_active() ? 1 : 0);
}
static struct governor_attr game_auto_state = __ATTR_RO(game_auto_state);

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

/* boot_boost_decay_ms sysfs knob.  Range
 * 0..ZENITH_BOOT_BOOST_DECAY_MS_MAX.  See struct zenith_tunables for
 * semantics.  Mirror of input_boost_decay_ms but for the boot-pin
 * window: 0 keeps the legacy hard cliff at boot_boost_ms, non-zero
 * applies a linear floor ramp from policy->max to policy->min after
 * the pin expires.
 */
static ssize_t boot_boost_decay_ms_show(struct gov_attr_set *attr_set,
					char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->boot_boost_decay_ms);
}

static ssize_t boot_boost_decay_ms_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_BOOT_BOOST_DECAY_MS_MAX)
		return -EINVAL;
	t->boot_boost_decay_ms = val;
	return count;
}
static struct governor_attr boot_boost_decay_ms =
	__ATTR_RW(boot_boost_decay_ms);

/* boot_complete sysfs knob.  See ZENITH_DEFAULT_BOOT_COMPLETE_AUTO
 * comment block.  Read returns the global latch state shared across
 * all policies (any policy's node is equivalent).  Write 1 to raise
 * the latch from userspace -- typical use is from
 * on property:sys.boot_completed=1 in init.zenith.rc.  Write 0 to
 * lower the latch (useful for testing the boot-boost path on a
 * running system without a reboot).  Values >1 are rejected.
 *
 * Raising the latch (write 1) sets zenith_boot_complete_ns to
 * ktime_get_boottime_ns() so the boost path snaps the deadline to
 * the latch timestamp.  Lowering (write 0) clears both the atomic
 * and the timestamp; the boost behaves as if the latch had never
 * been raised, except that the wall-clock has advanced -- a
 * post-deadline boot would simply re-enter the existing
 * boot_boost_decay_ms tail.
 */
static ssize_t boot_complete_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", atomic_read(&zenith_boot_complete));
}

static ssize_t boot_complete_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	if (val) {
		WRITE_ONCE(zenith_boot_complete_ns,
			   ktime_get_boottime_ns());
		atomic_set(&zenith_boot_complete, 1);
	} else {
		atomic_set(&zenith_boot_complete, 0);
		WRITE_ONCE(zenith_boot_complete_ns, 0);
	}
	return count;
}
static struct governor_attr boot_complete = __ATTR_RW(boot_complete);

/* boot_complete_auto sysfs knob.  See ZENITH_DEFAULT_BOOT_COMPLETE_AUTO
 * comment block.  Per-tunables (per-attr-set), unlike the global
 * boot_complete latch above.  Default 1: the auto-tune worker may
 * raise the latch on the first policy that observes
 * ZENITH_BOOT_COMPLETE_CALM_WINDOWS consecutive EFFICIENCY windows
 * past the grace period.  Set to 0 to require an explicit userspace
 * write to boot_complete; the calm streak counter still increments
 * but never raises the latch.
 */
static ssize_t boot_complete_auto_show(struct gov_attr_set *attr_set,
				       char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->boot_complete_auto);
}

static ssize_t boot_complete_auto_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->boot_complete_auto = val;
	return count;
}
static struct governor_attr boot_complete_auto =
	__ATTR_RW(boot_complete_auto);

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

/* frame_budget_us_auto sysfs knob.  Boolean (0/1).  When 1, the
 * adaptive frame-budget floor and the auto-tune V2 classifier both
 * prefer the drm-side cached vblank period (zenith_drm_vblank_us)
 * over the userspace-set frame_budget_us.  Falls back to the
 * userspace value when the cache is empty so existing tunings keep
 * working on systems whose drm driver doesn't yet call
 * zenith_set_drm_vblank_us().
 */
static ssize_t frame_budget_us_auto_show(struct gov_attr_set *attr_set,
					 char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->frame_budget_us_auto);
}

static ssize_t frame_budget_us_auto_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->frame_budget_us_auto, val);
	return count;
}
static struct governor_attr frame_budget_us_auto =
	__ATTR_RW(frame_budget_us_auto);

/* drm_vblank_us read-only debug knob.  Mirrors the cached
 * zenith_drm_vblank_us atomic so userspace can verify the drm
 * driver actually called zenith_set_drm_vblank_us().  Read-only:
 * userspace tuning should write frame_budget_us instead.
 */
static ssize_t drm_vblank_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       (unsigned int)atomic_read(&zenith_drm_vblank_us));
}
static struct governor_attr drm_vblank_us = __ATTR_RO(drm_vblank_us);

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

/* peer_ramp_uclamp_min_respect sysfs knob (Patch M2).  Range
 * 0..1.  When set, the peer_ramp floor is computed as
 * max(peer_ramp_floor_pct, uclamp_min_pct).  When 0, the
 * floor uses peer_ramp_floor_pct verbatim (Stage 4 behaviour).
 * Independent of the master uclamp_min_respect knob.
 */
static ssize_t
peer_ramp_uclamp_min_respect_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
		       peer_ramp_uclamp_min_respect);
}

static ssize_t
peer_ramp_uclamp_min_respect_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->peer_ramp_uclamp_min_respect, val);
	return count;
}

static struct governor_attr peer_ramp_uclamp_min_respect =
	__ATTR_RW(peer_ramp_uclamp_min_respect);

/* migration_floor_uclamp_min_respect sysfs knob (Patch M2).
 * Range 0..1.  Same shape as peer_ramp_uclamp_min_respect above
 * but for the migration_floor read site.  Independent of both
 * the master uclamp_min_respect knob and the peer_ramp variant.
 */
static ssize_t
migration_floor_uclamp_min_respect_show(struct gov_attr_set *attr_set,
					char *buf)
{
	return sprintf(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
		       migration_floor_uclamp_min_respect);
}

static ssize_t
migration_floor_uclamp_min_respect_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->migration_floor_uclamp_min_respect, val);
	return count;
}

static struct governor_attr migration_floor_uclamp_min_respect =
	__ATTR_RW(migration_floor_uclamp_min_respect);

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
	&peak_headroom_rescue.attr,
	&peak_headroom_starve_load_pct.attr,
	&peak_headroom_freq_floor_pct.attr,
	&peak_headroom_starve_streak.attr,
	&peak_headroom_jump_pct.attr,
	&peak_headroom_hold_ms.attr,
	&peak_headroom_prearm.attr,
	&predict_up_thresh.attr,
	&predict_up_window.attr,
	&peak_hysteresis_streak.attr,
	&peak_step_down_pct.attr,
	&boost_idle_thresh.attr,
	&boost_idle_streak.attr,
	&bg_util_scale_pct.attr,
	&sleeper_tail_thresh_us.attr,
	&sleeper_tail_pct.attr,
	&peer_ramp_window_ms.attr,
	&peer_ramp_floor_pct.attr,
	&peer_ramp_window_off_ms.attr,
	&migration_jump_pct.attr,
	&migration_floor_window_ms.attr,
	&migration_floor_pct.attr,
	&psi_cpu_floor_thresh.attr,
	&frame_overrun_slack_us.attr,
	&frame_overrun_window_ms.attr,
	&frame_overrun_floor_pct.attr,
	&frame_overrun_deep_streak.attr,
	&frame_overrun_deep_floor_pct.attr,
	&psi_mem_cap_thresh.attr,
	&psi_mem_cap_pct.attr,
	&psi_mem_cap_window_ms.attr,
	&brutal_decay_ms.attr,
	&climb_mode.attr,
	&freq_step_pct.attr,
	&freq_step_adaptive.attr,
	&profile.attr,
	&profile_values.attr,
	&zenith_stats.attr,
	&zenith_stats_reset.attr,
	&zenith_input_stats.attr,
	&at_log.attr,
	&last_decision_path.attr,
	&auto_tune_status.attr,
	&auto_tune_state_residency.attr,
	&auto_tune_state_history.attr,
	&auto_tune_reset_overrides.attr,
	&auto_tune.attr,
	&auto_tune_v2.attr,
	&auto_tune_v2_glides.attr,
	&auto_tune_v2_tiers.attr,
	&auto_tune_hysteresis_windows.attr,
	&auto_tune_cooldown_windows.attr,
	&auto_tune_v2_var_promote_thresh.attr,
	&auto_tune_v3.attr,
	&auto_tune_v3_interval_ms.attr,
	&auto_tune_v3_state.attr,
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
	&screen_on_bias_pct.attr,
	&io_is_busy.attr,
	&iowait_boost_min.attr,
	&iowait_stack_pct.attr,
	&iowait_backoff_after_ms.attr,
	&ignore_nice_load.attr,
	&screen_state.attr,
	&screen_off_glide_ms.attr,
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
	&wakeup_boost_ms.attr,
	&down_threshold_adaptive.attr,
	&rate_limit_cluster_scale.attr,
	&input_boost_ms.attr,
	&input_boost_decay_ms.attr,
	&input_boost_touchdown_extra_ms.attr,
	&input_boost_decay_curve.attr,
	&input_boost_big_only.attr,
	&input_boost_cap_pct.attr,
	&input_boost_down_rate_mult_pct.attr,
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
	&peer_ramp_uclamp_min_respect.attr,
	&migration_floor_uclamp_min_respect.attr,
	&uclamp_max_respect.attr,
	&predict_util_pct.attr,
	&predict_util_smooth.attr,
	&render_aware.attr,
	&render_comms.attr,
	&render_floor_pct.attr,
	&render_floor_min_runtime_ms.attr,
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
	&boot_boost_decay_ms.attr,
	&boot_complete.attr,
	&boot_complete_auto.attr,
	&game_auto.attr,
	&game_auto_state.attr,
	&game_auto_comms.attr,
	&frame_budget_us.attr,
	&frame_budget_us_auto.attr,
	&drm_vblank_us.attr,
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
	/* Patch J: prime the per-policy decision tag so the sysfs
	 * node returns a meaningful value before the first eval.
	 */
	z_policy->last_decision_path = "init";

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
	tunables->peak_headroom_rescue	= ZENITH_DEFAULT_PEAK_HEADROOM_RESCUE;
	tunables->peak_headroom_starve_load_pct =
		ZENITH_DEFAULT_PEAK_HEADROOM_STARVE_LOAD_PCT;
	tunables->peak_headroom_freq_floor_pct =
		ZENITH_DEFAULT_PEAK_HEADROOM_FREQ_FLOOR_PCT;
	tunables->peak_headroom_starve_streak =
		ZENITH_DEFAULT_PEAK_HEADROOM_STARVE_STREAK;
	tunables->peak_headroom_jump_pct =
		ZENITH_DEFAULT_PEAK_HEADROOM_JUMP_PCT;
	tunables->peak_headroom_hold_ms =
		ZENITH_DEFAULT_PEAK_HEADROOM_HOLD_MS;
	tunables->peak_headroom_prearm =
		ZENITH_DEFAULT_PEAK_HEADROOM_PREARM;
	tunables->predict_up_thresh	= ZENITH_DEFAULT_PREDICT_UP_THRESH;
	tunables->predict_up_window	= ZENITH_DEFAULT_PREDICT_UP_WINDOW;
	tunables->peak_hysteresis_streak =
		ZENITH_DEFAULT_PEAK_HYSTERESIS_STREAK;
	tunables->peak_step_down_pct	= ZENITH_DEFAULT_PEAK_STEP_DOWN_PCT;
	tunables->boost_idle_thresh	= ZENITH_DEFAULT_BOOST_IDLE_THRESH;
	tunables->boost_idle_streak	= ZENITH_DEFAULT_BOOST_IDLE_STREAK;
	tunables->bg_util_scale_pct	= ZENITH_DEFAULT_BG_UTIL_SCALE_PCT;
	tunables->sleeper_tail_thresh_us =
		ZENITH_DEFAULT_SLEEPER_TAIL_THRESH_US;
	tunables->sleeper_tail_pct	= ZENITH_DEFAULT_SLEEPER_TAIL_PCT;
	tunables->peer_ramp_window_ms	=
		ZENITH_DEFAULT_PEER_RAMP_WINDOW_MS;
	tunables->peer_ramp_floor_pct	=
		ZENITH_DEFAULT_PEER_RAMP_FLOOR_PCT;
	tunables->peer_ramp_window_off_ms =
		ZENITH_DEFAULT_PEER_RAMP_WINDOW_OFF_MS;
	tunables->migration_jump_pct	=
		ZENITH_DEFAULT_MIGRATION_JUMP_PCT;
	tunables->migration_floor_window_ms =
		ZENITH_DEFAULT_MIGRATION_FLOOR_WINDOW_MS;
	tunables->migration_floor_pct	=
		ZENITH_DEFAULT_MIGRATION_FLOOR_PCT;
	tunables->psi_cpu_floor_thresh	=
		ZENITH_DEFAULT_PSI_CPU_FLOOR_THRESH;
	tunables->frame_overrun_slack_us =
		ZENITH_DEFAULT_FRAME_OVERRUN_SLACK_US;
	tunables->frame_overrun_window_ms =
		ZENITH_DEFAULT_FRAME_OVERRUN_WINDOW_MS;
	tunables->frame_overrun_floor_pct =
		ZENITH_DEFAULT_FRAME_OVERRUN_FLOOR_PCT;
	tunables->frame_overrun_deep_streak =
		ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_STREAK;
	tunables->frame_overrun_deep_floor_pct =
		ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_FLOOR_PCT;
	tunables->psi_mem_cap_thresh =
		ZENITH_DEFAULT_PSI_MEM_CAP_THRESH;
	tunables->psi_mem_cap_pct =
		ZENITH_DEFAULT_PSI_MEM_CAP_PCT;
	tunables->psi_mem_cap_window_ms =
		ZENITH_DEFAULT_PSI_MEM_CAP_WINDOW_MS;
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
	tunables->auto_tune_v2_glides	= ZENITH_DEFAULT_AUTO_TUNE_V2_GLIDES;
	tunables->auto_tune_v2_tiers	= ZENITH_DEFAULT_AUTO_TUNE_V2_TIERS;
	tunables->auto_tune_hysteresis_windows =
		ZENITH_DEFAULT_AT_HYSTERESIS_WINDOWS;
	tunables->auto_tune_cooldown_windows =
		ZENITH_DEFAULT_AT_COOLDOWN_WINDOWS;
	tunables->auto_tune_v2_var_promote_thresh =
		ZENITH_DEFAULT_AT_V2_VAR_PROMOTE_THRESH;
	tunables->auto_tune_v3 = ZENITH_DEFAULT_AUTO_TUNE_V3;
	tunables->auto_tune_v3_interval_ms =
		ZENITH_DEFAULT_AT_V3_INTERVAL_MS;
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
	tunables->screen_on_bias_pct	= ZENITH_DEFAULT_SCREEN_ON_BIAS_PCT;
	tunables->io_is_busy		= ZENITH_DEFAULT_IO_IS_BUSY;
	tunables->iowait_boost_min	= ZENITH_DEFAULT_IOWAIT_BOOST_MIN;
	tunables->iowait_stack_pct	= ZENITH_DEFAULT_IOWAIT_STACK_PCT;
	tunables->iowait_backoff_after_ms = ZENITH_DEFAULT_IOWAIT_BACKOFF_AFTER_MS;
	tunables->ignore_nice_load	= 0;
	tunables->screen_state		= 1;
	tunables->screen_off_glide_ms	= ZENITH_DEFAULT_SCREEN_OFF_GLIDE_MS;
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
	tunables->wakeup_boost_ms	= ZENITH_DEFAULT_WAKEUP_BOOST_MS;
	tunables->down_threshold_adaptive = ZENITH_DEFAULT_DOWN_THRESHOLD_ADAPTIVE;
	tunables->rate_limit_cluster_scale = ZENITH_DEFAULT_RATE_LIMIT_CLUSTER_SCALE;
	tunables->input_boost_ms	= ZENITH_DEFAULT_INPUT_BOOST_MS;
	tunables->input_boost_decay_ms	= ZENITH_DEFAULT_INPUT_BOOST_DECAY_MS;
	tunables->input_boost_touchdown_extra_ms =
		ZENITH_DEFAULT_INPUT_BOOST_TOUCHDOWN_EXTRA_MS;
	tunables->input_boost_decay_curve = ZENITH_DEFAULT_INPUT_BOOST_DECAY_CURVE;
	tunables->input_boost_big_only	= ZENITH_DEFAULT_INPUT_BOOST_BIG_ONLY;
	tunables->input_boost_cap_pct	= ZENITH_DEFAULT_INPUT_BOOST_CAP_PCT;
	tunables->input_boost_down_rate_mult_pct =
		ZENITH_DEFAULT_INPUT_BOOST_DOWN_RATE_MULT_PCT;
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
	tunables->peer_ramp_uclamp_min_respect =
		ZENITH_DEFAULT_PEER_RAMP_UCLAMP_MIN_RESPECT;
	tunables->migration_floor_uclamp_min_respect =
		ZENITH_DEFAULT_MIGRATION_FLOOR_UCLAMP_MIN_RESPECT;
	tunables->uclamp_max_respect	= ZENITH_DEFAULT_UCLAMP_MAX_RESPECT;
	tunables->predict_util_pct	= ZENITH_DEFAULT_PREDICT_UTIL_PCT;
	tunables->predict_util_smooth	= ZENITH_DEFAULT_PREDICT_UTIL_SMOOTH;
	tunables->render_aware		= ZENITH_DEFAULT_RENDER_AWARE;
	tunables->render_floor_pct	= ZENITH_DEFAULT_RENDER_FLOOR_PCT;
	tunables->render_floor_min_runtime_ms =
		ZENITH_DEFAULT_RENDER_FLOOR_MIN_RUNTIME_MS;
	tunables->audio_aware		= ZENITH_DEFAULT_AUDIO_AWARE;
	tunables->audio_floor_pct	= ZENITH_DEFAULT_AUDIO_FLOOR_PCT;
	tunables->audio_cap_pct		= ZENITH_DEFAULT_AUDIO_CAP_PCT;
	tunables->camera_aware		= ZENITH_DEFAULT_CAMERA_AWARE;
	tunables->camera_active		= ZENITH_DEFAULT_CAMERA_ACTIVE;
	tunables->camera_floor_pct	= ZENITH_DEFAULT_CAMERA_FLOOR_PCT;
	tunables->game_mode		= ZENITH_DEFAULT_GAME_MODE;
	tunables->game_auto		= ZENITH_DEFAULT_GAME_AUTO;
	tunables->psi_aware		= ZENITH_DEFAULT_PSI_AWARE;
	tunables->psi_mem_thresh	= ZENITH_DEFAULT_PSI_MEM_THRESH;
	tunables->psi_cpu_thresh	= ZENITH_DEFAULT_PSI_CPU_THRESH;
	tunables->psi_io_thresh		= ZENITH_DEFAULT_PSI_IO_THRESH;
	tunables->boot_boost_ms		= ZENITH_DEFAULT_BOOT_BOOST_MS;
	tunables->boot_boost_decay_ms	= ZENITH_DEFAULT_BOOT_BOOST_DECAY_MS;
	tunables->boot_complete_auto	= ZENITH_DEFAULT_BOOT_COMPLETE_AUTO;
	tunables->frame_budget_us	= ZENITH_DEFAULT_FRAME_BUDGET_US;
	tunables->frame_budget_us_auto	= ZENITH_DEFAULT_FRAME_BUDGET_US_AUTO;
	tunables->frame_pace_floor_pct	= ZENITH_DEFAULT_FRAME_PACE_FLOOR_PCT;
	WRITE_ONCE(zenith_input_boost_active_ms, ZENITH_DEFAULT_INPUT_BOOST_MS);
	WRITE_ONCE(zenith_input_boost_touchdown_extra_ms_cache,
		   ZENITH_DEFAULT_INPUT_BOOST_TOUCHDOWN_EXTRA_MS);

	/* Sync the audio_aware / render_aware / game_auto / auto_tune_v3
	 * static keys against their default scalars.  See the comment
	 * above DEFINE_STATIC_KEY_FALSE for the invariant: scalars whose
	 * default is non-zero need an explicit init-time key enable.
	 * audio_aware / render_aware were flipped to 1 in wave-2;
	 * game_auto was flipped to 1 and auto_tune_v3 to 2 in wave-7.
	 * camera_aware and psi_aware still default to 0 so their keys
	 * remain FALSE; we do not call them here.  Idempotent across
	 * re-attaches: zenith_set_static_key() is a no-op if the key is
	 * already in the requested state.  zenith_set_static_key()
	 * coerces non-zero scalars (including the auto_tune_v3 = 2
	 * APPLY mode) to TRUE, which is the correct branch state for
	 * any non-OFF mode.
	 */
	zenith_set_static_key(&zenith_audio_aware_key,
			      tunables->audio_aware);
	zenith_set_static_key(&zenith_render_aware_key,
			      tunables->render_aware);
	zenith_set_static_key(&zenith_game_auto_key,
			      tunables->game_auto);
	zenith_set_static_key(&zenith_auto_tune_v3_key,
			      tunables->auto_tune_v3);

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
	u64 now_ns, last_ns, deadline;
	unsigned int effective_ms;
	bool was_quiet = false;
	unsigned int gap_ms = 0;
	unsigned int source = 0;

	if (type != EV_KEY && type != EV_ABS && type != EV_REL)
		return;

	/* Always bump the auto-tune counter so a policy that enables
	 * auto_tune mid-session has recent data. Cheap atomic inc.
	 */
	atomic64_inc(&zenith_auto_input_events);
	atomic64_inc(&zenith_in_events_total);

	if (!active) {
		atomic64_inc(&zenith_in_boosts_skipped_disabled);
		return;
	}

	now_ns = ktime_get_ns();
	last_ns = (u64)atomic64_read(&zenith_input_last_event_ns);
	atomic64_set(&zenith_input_last_event_ns, now_ns);

	/* Quiet-period extension.  When the gap since the previous
	 * event exceeds ZENITH_INPUT_QUIET_THRESHOLD_MS (or this is the
	 * very first event after boot, last_ns == 0), widen the
	 * full-pin window by ZENITH_INPUT_QUIET_BOOST_MULT_PCT and clip
	 * at ZENITH_INPUT_QUIET_BOOST_MAX_MS.  Sustained-interaction
	 * events (gap < threshold) keep the original active duration.
	 */
	effective_ms = active;
	if (!last_ns ||
	    now_ns - last_ns >=
	    (u64)ZENITH_INPUT_QUIET_THRESHOLD_MS * NSEC_PER_MSEC) {
		unsigned int extended = (active *
					 ZENITH_INPUT_QUIET_BOOST_MULT_PCT) /
					100;

		if (extended > ZENITH_INPUT_QUIET_BOOST_MAX_MS)
			extended = ZENITH_INPUT_QUIET_BOOST_MAX_MS;
		if (extended > effective_ms)
			effective_ms = extended;
		was_quiet = true;
	}

	/* Touchdown detection (Patch C).  Only the EV_KEY/BTN_TOUCH
	 * press counts as a touchdown.  Coordinate-stream EV_ABS and
	 * BTN_TOUCH release (value == 0) take the unmodified path.
	 * The extra is added on top of any quiet-period extension --
	 * a touchdown after a long quiet gap gets both bonuses, which
	 * is exactly what the user feels (cold start of a gesture).
	 */
	if (type == EV_KEY && code == BTN_TOUCH && value == 1) {
		unsigned int extra =
			READ_ONCE(zenith_input_boost_touchdown_extra_ms_cache);

		if (extra) {
			u64 widened64 = (u64)effective_ms + extra;

			if (widened64 > U32_MAX)
				widened64 = U32_MAX;
			effective_ms = (unsigned int)widened64;
		}
	}

	deadline = now_ns + (u64)effective_ms * NSEC_PER_MSEC;
	atomic64_set(&zenith_input_boost_until_ns, deadline);
	atomic64_inc(&zenith_in_boosts_armed);
	if (effective_ms > active)
		atomic64_inc(&zenith_in_boosts_quiet_extended);

	/* Observability: emit a tracepoint capturing the boost arming
	 * decision.  Default-disabled; consumers (testers /
	 * developers) opt in via
	 *
	 *   echo 1 > /sys/kernel/debug/tracing/events/cpufreq_zenith/zenith_input_boost/enable
	 *
	 * The tracepoint is gated by trace_zenith_input_boost_enabled()
	 * so the cost in the disabled case is a single conditional
	 * branch on a static-key.  All emit-side computations
	 * (gap_ms, source) are guarded by the same gate so they
	 * cannot show up in the hot path when nobody is tracing.
	 */
	if (trace_zenith_input_boost_enabled()) {
		u64 gap_ns;

		if (last_ns && now_ns > last_ns)
			gap_ns = now_ns - last_ns;
		else
			gap_ns = 0;
		if (gap_ns) {
			u64 gap_ms_u64 = div_u64(gap_ns, NSEC_PER_MSEC);

			gap_ms = gap_ms_u64 > U32_MAX ? U32_MAX :
				 (unsigned int)gap_ms_u64;
		}

		switch (type) {
		case EV_ABS:
			source = 1; /* touchscreen */
			break;
		case EV_KEY:
			source = 2; /* key */
			break;
		default:
			source = 0; /* generic / EV_REL et al */
			break;
		}

		trace_zenith_input_boost(effective_ms, active,
					 effective_ms > active,
					 was_quiet, gap_ms, source);
	}
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
 *
 * __maybe_unused is required because the only in-tree callers live
 * inside CONFIG_FB_NOTIFY and CONFIG_DRM_PANEL_NOTIFY blocks; when
 * both are disabled the function has no in-tree caller and a -Werror
 * build would otherwise fail with -Wunused-function.  Out-of-tree
 * consumers (vendor mode-set / panel ioctl handlers) still get the
 * helper they need.
 */
static void __maybe_unused
zenith_panel_blank_event(int blank, unsigned int unblank_value)
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

	/* Boot banner.  Printed once at governor init, picked up by
	 * `dmesg | grep -i zenith`.  Free-verse on top, the heart-init
	 * flow + ASCII shomy block underneath; every line carries the
	 * "Zenith" prefix so the whole multi-line banner survives a
	 * grep.  The bloody-moon quote is printed as two physical dmesg
	 * lines because the single-line form is over 100 cols and
	 * checkpatch dislikes the adjacent-string-literal split.
	 */
	pr_info("Zenith : The brain has started.\n");
	pr_info("Zenith : The heart of zenith is yet to be initialized.\n");
	pr_info("Zenith : It might hurt to love someone from the shadows.\n");
	pr_info("Zenith : But love chose her.\n");
	pr_info("Zenith : I chose her.\n");
	pr_info("Zenith : Not because i was forced to.\n");
	pr_info("Zenith : Because i wanted to.\n");
	pr_info("Zenith : Even if it hurts.\n");
	pr_info("Zenith : Love really is a powerful thing.\n");
	pr_info("Zenith : \"Lost beneath the haunting light of a bloody moon,\n");
	pr_info("Zenith :  wandering through the echoes of time.\"\n");
	pr_info("Zenith :\n");
	pr_info("Zenith : Initializing the heart of zenith...\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :    ***   ***\n");
	pr_info("Zenith :   ***** *****\n");
	pr_info("Zenith :   ***********\n");
	pr_info("Zenith :    *********\n");
	pr_info("Zenith :     *******\n");
	pr_info("Zenith :      *****\n");
	pr_info("Zenith :       ***\n");
	pr_info("Zenith :        *\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :     ~ shomy ~\n");
	pr_info("Zenith :\n");
	pr_info("Zenith : The heart of zenith has been initialized.\n");
	pr_info("Zenith : Yet it can never be mine.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith : Welcome aboard, Zenith: Built by XTENSEI, for shomy.\n");

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
	rcu_assign_pointer(zenith_game_auto_table,
		zenith_alloc_comm_table_from_defaults(zenith_game_auto_comms,
			ARRAY_SIZE(zenith_game_auto_comms)));

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