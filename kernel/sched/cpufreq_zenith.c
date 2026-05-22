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
#include <linux/cpufreq_zenith.h>
#include <linux/sched/cpufreq.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/irq_work.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/cgroup.h>
#include <linux/energy_model.h>
#include <linux/input.h>
#include <linux/perf_event.h>
#include <linux/jump_label.h>
#include <linux/atomic.h>
#include <linux/bits.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/sysfs.h>
#include <linux/hikari.h>
#include <linux/notifier.h>
#include "cpufreq_zenith_internal.h"
#include <linux/power_supply.h>
#include <linux/time.h>
#include <linux/kernel_stat.h>
#include <linux/thermal.h>

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
#include <trace/events/sched.h>

#define CREATE_TRACE_POINTS
#include <trace/events/cpufreq_zenith.h>
#undef CREATE_TRACE_POINTS

/* Vendor-hook headers (B9-1 topology, B9-2 sched, B9-3 cpuidle, B9-3+
 * power).  Pulled in *after* CREATE_TRACE_POINTS has been undef'd
 * above, so each header only DECLAREs its android_vh_* /
 * android_rvh_* tracepoints (the #include <trace/define_trace.h> at
 * the bottom of every trace/hooks header is a no-op when
 * CREATE_TRACE_POINTS is not defined).  The canonical owner of every
 * __traceiter_android_* / __tracepoint_android_* symbol referenced
 * below is drivers/android/vendor_hooks.o, which sets
 * CREATE_TRACE_POINTS itself before pulling these same headers in
 * (drivers/android/vendor_hooks.c).  Defining them here would emit a
 * second copy of those symbols and the link would fail with duplicate
 * definitions in kernel/built-in.a vs drivers/built-in.a.
 *
 * All four headers are always pulled in regardless of
 * CONFIG_ANDROID_VENDOR_HOOKS: when the vendor-hook infrastructure is
 * compiled out, the trace_android_vh_* / register_trace_android_vh_*
 * macros collapse to empty / -ENODEV, and the per-tunable enable gates
 * (vh_arch_freq_scale_enable, vh_uclamp_observer_enable,
 * vh_cpu_idle_enable, vh_freq_qos_enable, vh_sched_move_task_enable,
 * vh_scheduler_tick_enable)
 * become runtime no-ops.
 *
 * Mirror of the pattern used by kernel/sched/core.c, which also defines
 * its own trace events via CREATE_TRACE_POINTS for <trace/events/sched.h>
 * and then includes <trace/hooks/sched.h> + <trace/hooks/dtask.h> as
 * declare-only consumers.
 */
#include <trace/hooks/topology.h>
#include <trace/hooks/sched.h>
#include <trace/hooks/cpuidle.h>
#include <trace/hooks/power.h>

/* Constants & Defaults */
/* Permille of SCHED_CAPACITY_SCALE at which iowait boost starts.
 * 125 == SCHED_CAPACITY_SCALE / 8, preserving the historical default.
 */

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
/* Defensive sysfs upper bound for hispeed_freq (kHz).  50 GHz is well past
 * any real CPU; rejects UINT_MAX-style garbage at the sysfs layer.  Values
 * below this still receive the existing consumer-side comparison against
 * policy->cur / policy->max.
 */

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

/* Patch 1.3 cluster-wake-pulse: when zenith_get_next_freq() is
 * entered after a >= cluster_wake_pulse_idle_ms gap (the cluster
 * was deeply idle), arm a soft floor at cluster_wake_pulse_floor_-
 * pct of policy->max for cluster_wake_pulse_ms milliseconds.
 *
 * The floor absorbs the PELT warm-up cost: the first util sample
 * after a long idle is by construction near zero and would normally
 * pin freq at min, even when the workload that just woke needs the
 * cluster (the EAS resolve catches up only on the second/third
 * sample, by which point a frame deadline can already be at risk).
 *
 * Defaults: cluster_wake_pulse_ms = 40 (one-frame budget on a
 * 60 Hz panel), cluster_wake_pulse_idle_ms = 80 (only fire after
 * a "real" deep idle, not just a burst of two consecutive idle
 * sample windows), cluster_wake_pulse_floor_pct = 55 (just above
 * a typical hispeed-entry band).
 *
 * Disabled by setting cluster_wake_pulse_ms = 0; profile-baked so
 * BATTERY and LEGACY hide the tier entirely while PERFORMANCE
 * widens it.
 */

/* Patch 1.2 batt_hold_scale_pct: percentage scale applied to
 * peak-rescue / peak-prearm hold-down millisecond budgets when
 * the system is running on battery.  Defaults to 100 (identity,
 * preserves pre-1.2 behaviour) so a no-op for users on AC, lab
 * boards, or hardware without a power_supply driver registered.
 * Bounded to 50..300 (50 -> halve the hold, 300 -> triple it).
 * Profile-baked via zenith_apply_profile() so users on PERFORMANCE
 * never have hold extended on battery, BALANCED gets a 1.2x scale,
 * BATTERY gets 1.8x to keep cores at floor longer when discharging,
 * LEGACY stays at 100 (identity, pre-V4 hybrid behaviour).
 */

/* Wave A charger-aware floor.  Companion to the existing
 * batt_hold_scale_pct / on_battery infrastructure: when
 * charger_aware == 1 AND the lazy AC-vs-battery cache reports the
 * system is on AC (zenith_on_battery == 0), a freq floor of
 * (policy->max * charger_floor_pct / 100) is applied alongside the
 * existing audio / render / migration floors.  Both knobs default 0
 * (off) so a fresh boot is bit-identical to pre-Wave-A behaviour;
 * users opt in by writing 1 + floor_pct via sysfs.
 *
 * Rationale: while the device is plugged in, the energy cost of
 * holding hispeed approaches zero (the charger is feeding both the
 * battery and the SoC), so a configurable floor delivers extra
 * responsiveness with no on-battery cost.  Thermal still wins above
 * charger_floor_pct because the thermal_state / auto_thermal_cap
 * chain runs after the floor and can walk it back down if the SoC
 * overheats.  audio_floor / render_floor / migration_floor all run
 * before this site, so the charger floor only applies when none of
 * the situational floors have already raised freq above
 * charger_floor_pct.
 */

/* Wave A cgroup-aware top-app floor.  Replaces the fragile comm-walk
 * heuristic for foreground detection (game_auto / render_aware /
 * audio_aware all match by thread name) with a rock-solid cgroup
 * read.  Android's ActivityManager assigns every UI-visible app's
 * threads to the cpuset cgroup directory "top-app"; system /
 * background threads land in "foreground", "background", or
 * "system-background".  Reading current->cgroups->subsys
 * [cpuset_cgrp_id]->cgroup->kn->name == "top-app" answers "is the
 * user looking at this task right now" without depending on which
 * APK happened to be installed or how its threads are named.
 *
 * When top_app_aware == 1 AND any CPU in the policy is currently
 * running a task in the top-app cpuset cgroup, apply a freq floor
 * of (policy->max * top_app_floor_pct / 100) alongside the existing
 * audio / render / camera / charger floors.  Cached for
 * ZENITH_TOP_APP_CACHE_TTL_NS to keep the hot path cheap (one
 * task_css() + strcmp() per cached interval, not per tick).
 *
 * Both knobs default 0 so the tier is opt-in and a fresh boot is
 * bit-identical to pre-Wave-A behaviour.  Requires CONFIG_CPUSETS=y
 * in the kernel (mandatory on Android GKI; on CONFIG_CPUSETS=n the
 * helper always returns false and the floor never applies).
 *
 * The cgroup name is fixed at "top-app" because every Android
 * release since Lollipop has used that exact directory name; if
 * a vendor renames it, set top_app_aware=0 and use the comm-walk
 * floors instead.
 */

/* Wave A render-thread util tracker.  More selective sibling of the
 * existing render_floor: the comm-walk-based render_floor fires
 * whenever a known render thread is observed on the policy, even
 * if the thread is sitting idle in its main loop (paused video,
 * static UI).  This tracker re-uses the same comm walk but adds a
 * second gate -- the matched task's PELT se.avg.util_avg must be
 * >= render_thread_util_thresh -- before applying a separate,
 * higher floor (render_thread_util_floor_pct).
 *
 * Threshold is in 1/SCHED_CAPACITY_SCALE units (0..1024) matching
 * util_avg's scale.  Typical values:
 *   256  =  25% of a CPU's capacity, low filter -- catches any
 *           render thread above background idle.
 *   512  =  50% capacity, medium filter -- catches active
 *           rendering (60 Hz scroll, video playback).
 *   768  =  75% capacity, strict filter -- catches heavy GPU
 *           workloads (gaming, complex animations) only.
 *
 * Reuses the cached zenith_policy_has_render() walk; the helper
 * also stores the matched task's util_avg in
 * z_policy->render_matched_util_avg, so this tier adds at most a
 * single load after the cache hit.  All three knobs default 0 so
 * a fresh boot is bit-identical to pre-Wave-A behaviour and the
 * tier is opt-in.  Requires render_aware=1 in addition to
 * render_thread_util_aware=1, because the comm-walk must run for
 * util_avg to be observed.
 */

/* Wave B PMU IPC tracker.  Per-CPU hardware perf_event counters
 * (instructions retired / CPU cycles) sampled once per
 * zenith_auto_tune_work() pass.  IPC == instructions / cycles is the
 * canonical efficiency signal: high IPC means the workload is
 * compute-bound and benefits from extra freq headroom; low IPC means
 * the workload is memory-bound or stalled and extra freq mostly burns
 * energy without helping throughput.  This tier raises a freq floor
 * when measured IPC crosses pmu_ipc_thresh, so user-tunable workloads
 * that want "freq when it actually pays" can opt in.
 *
 * IPC is reported as percent (100 = 1.0 IPC).  Modern Cortex-A series
 * cores typically run 0.5..2.0 IPC under common Android workloads, so
 * a default thresh of 100 (1.0 IPC) catches "actually using the CPU"
 * without firing on memory-bound stalls.  Cap at 1000 (10.0 IPC)
 * which is well above any real-world value -- the cap exists only to
 * keep the percent representation in unsigned int range.
 *
 * Gated on CONFIG_PERF_EVENTS at build time.  When the kernel is
 * built without perf_events, all helpers compile to no-ops, the
 * tunables remain visible but their effect is constant zero, and
 * the floor never applies.  Per-CPU events are allocated lazily in
 * zenith_start() and torn down in zenith_stop(); allocation
 * failures (PMU not exposed by the SoC, perf locked down) are
 * silently tolerated and the floor never applies on those CPUs.
 */

/* Wave B EAS / Energy Model integration.  Reads the per-policy
 * struct em_perf_domain (registered by the cpufreq driver via
 * em_dev_register_perf_domain()) to locate the energy-knee OPP --
 * the performance state with the lowest 'cost' field, where cost ==
 * power * max_freq / freq is pre-computed by EM at registration
 * time.  Below the knee, voltage scaling makes the OPP
 * inefficient in joules-per-instruction; above the knee, voltage
 * scales linearly with freq while capacity scales sub-linearly,
 * so cost rises again.  The knee is therefore the most
 * energy-efficient sustained operating point.
 *
 * em_floor_pct applies a freq floor at (em_knee_freq *
 * em_floor_pct / 100) so the policy never undershoots the
 * energy-knee for a sustained workload.  Capped at 200% to allow
 * the user to express "a bit above the knee for safety margin".
 *
 * Both knobs default 0 so the tier is opt-in and a fresh boot is
 * bit-identical to pre-Wave-B behaviour.  Gated on
 * CONFIG_ENERGY_MODEL at build time; on CONFIG_ENERGY_MODEL=n the
 * helper returns a constant zero (em_cpu_get() returns NULL
 * unconditionally) and the floor never applies.
 *
 * Caches the knee freq per-policy at zenith_start() to avoid the
 * em->table walk on every cpufreq tick.  When the cpufreq driver
 * registers an EM after zenith_start() has run (rare; most boards
 * register the EM early during cpufreq driver probe), the cache
 * is refreshed lazily on the first floor application.
 */

/* Patch 1.10 quiet-hours cap.  Two start / end knobs (in minutes
 * since 00:00 UTC, range 0..1439) define a daily window; while
 * inside that window, freq is capped at quiet_hours_cap_pct of
 * policy->max.  When start == end, the window is zero-length and
 * the tier is disabled (the default).  When start > end the
 * window wraps midnight (e.g. 22:00 .. 06:00).
 *
 * UTC is the reference because the kernel only knows wall time;
 * userspace converts the user's local sleep window to UTC and
 * writes the two knobs at boot.  This avoids dragging timezone
 * state into the governor.
 *
 * quiet_hours_screen_off_only (default 1) gates the cap on
 * tunables->screen_state == 0, so an unintended throttle never
 * lands while the user is actively interacting -- the use case
 * is "slow the CPU while the phone is sleeping next to the bed",
 * not "throttle the device mid-call".  Setting it to 0 enables
 * the cap regardless of screen state.
 *
 * Profile-baked: PERFORMANCE / LEGACY keep cap_pct = 100 (no cap),
 * BALANCED holds at 70 %% (mild residency push if a window is
 * configured), BATTERY at 55 %% (aggressive).  The window itself
 * is *not* profile-baked because the user's quiet hours are
 * personal -- profiles only own the cap depth.
 */

/* Patch 1.9 fg-transition pulse.  When a foreground (top-app)
 * task is woken for the first time after fork() -- detected via
 * a sched_wakeup_new tracepoint probe with uclamp_eff_value()
 * as the foreground proxy -- arm a one-shot freq floor for
 * fg_transition_pulse_ms milliseconds at fg_transition_pulse_pct
 * of policy->max.  Smooths app-launch / activity-start latency
 * by giving the cluster the freshly-forked task lands on a
 * brief headroom window above PELT cold-start.
 *
 * fg_transition_pulse_ms == 0 disables the tier (BATTERY /
 * LEGACY profiles).  fg_transition_pulse_pct == 0 also disables
 * the floor application; both knobs are profile-baked.
 */

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

/* pelt_rising_edge_thresh (default 32) + pelt_rising_edge_min_pct
 * (default 50): companion to predict_up that catches sharp single-
 * sample slope-up events that the rolling-window delta dilutes.
 *
 * Rationale: predict_up integrates over predict_up_window samples
 * (4 by default), so a workload that takes ~8 samples to reach the
 * cumulative threshold will not lift until the half-way point.  A
 * cold cluster wake-up or a freshly-foregrounded GUI task often
 * shows the steepest util_avg slope on the *first* sample after
 * the cluster left idle; the rolling window misses that with too-
 * conservative thresholds.
 *
 * The rising-edge tier checks the slope between the two newest
 * util_history samples ((newest - prev) * 256 / max_cap).  Crossing
 * pelt_rising_edge_thresh AND newest >= pelt_rising_edge_min_pct *
 * max_cap / 100 lifts the cluster to eff_hispeed_freq with
 * tp_path "pelt_edge".  The min_pct gate prevents firing on
 * tiny-base spikes (e.g. a 5%% util cluster jumping to 8%% in one
 * sample looks steep on the slope but is not actionable).
 *
 * 0 disables the tier (legacy behaviour: only the rolling-window
 * predict_up fires).  Max ZENITH_PELT_RISING_EDGE_THRESH_MAX (255)
 * is the same domain as predict_up_thresh.
 */

/* dl_task_floor_pct (default 0, range 0..100, [Patch C6]):
 *
 * SCHED_DEADLINE awareness floor.  When any CPU in the policy
 * has a SCHED_DEADLINE task on its rq (rq->dl.dl_nr_running >
 * 0), lift freq to (policy->max * dl_task_floor_pct / 100).
 *
 * Rationale: schedutil_cpu_util() already adds cpu_bw_dl() to
 * the util signal so DL bandwidth requirements feed into the
 * proportional math automatically.  That math guarantees
 * *average* DL throughput, but a freshly-woken DL task takes
 * roughly one PELT half-life (~32 ms) for its util_avg
 * contribution to fully land, during which the proportional
 * math runs against a stale picture and can miss the first
 * deadline.  Lifting to a per-policy floor closes that
 * responsiveness gap without forcing policy->max for every
 * DL task in the system.
 *
 * 0 disables the tier (legacy behaviour: rely solely on the
 * schedutil_cpu_util DL bandwidth contribution).  Max 100
 * (== policy->max).  pin_to_target paths skip the floor
 * because input_boost / brutality already pin higher.
 *
 * Profile bakes: PERFORMANCE=100, GAMING=100, AUDIO=80,
 * BALANCED=0, BATTERY=0, LEGACY=0.  Cold-boot default 0 keeps
 * legacy behaviour for CUSTOM users; flipping to a profile
 * that enables it does not require any further sysfs work.
 */

/* io_floor_hyst_ms / io_floor_hyst_pct
 * (defaults 0 / 50, [Patch C9]):
 *
 * Sticky-floor hysteresis sibling for the iowait_boost path.
 * iowait_boost starts at iowait_boost_min, doubles on each
 * SCHED_CPUFREQ_IOWAIT flag, halves on idle samples; once the
 * episode ends the boost decays to 0 within a few PELT periods,
 * after which the freq drops back to the level signal.  For
 * sustained block IO (file-system flush, sqlite WAL replay,
 * media transcode) the level signal often does not pin a high
 * freq because the worker thread is mostly D-state -- the boost
 * was carrying the freq.  When the boost decays the freq drops,
 * latency on the next IO batch jumps, and the boost has to ramp
 * again.
 *
 * The hysteresis floor stamps a deadline 'now + io_floor_hyst_ms'
 * on the policy whenever zenith_iowait_boost() arms a positive
 * boost.  While that deadline has not expired, lift freq to
 * (policy->max * io_floor_hyst_pct / 100), tp_path "io_floor".
 *
 * 0 ms disables the tier (legacy: rely solely on iowait_boost
 * decay).  Max ZENITH_IO_FLOOR_HYST_MS_MAX (2000 ms; beyond that
 * the floor would routinely outlive the IO episode).  pct in
 * 0..100; 0 disables the floor effect even if window is set.
 *
 * Profile bakes (auto-tune):
 *   PERFORMANCE:  500 ms / 70%%
 *   BALANCED:     200 ms / 50%%
 *   BATTERY:        0 ms /  0%%   (off, energy frame)
 *   LEGACY:         0 ms /  0%%   (historical-compat)
 *   GAMING:       300 ms / 60%%
 *   AUDIO:        500 ms / 60%%   (sustained DAC ring writes)
 *   CUSTOM:         0 ms / 50%%   (cold-boot off; pct populated
 *                                  for forward-compat)
 */

/* vh_arch_freq_scale_enable (default 0, [Patch B9-1]):
 *
 * Master 0/1 gate for the android_vh_arch_set_freq_scale vendor-hook
 * observer.  When 1, every realisation of a per-cluster
 * frequency-scale change (the value the scheduler caches for capacity
 * accounting) drops into zenith_probe_arch_set_freq_scale(), updates
 * z_policy->vh_arch_freq_scale_last, and -- if the climb crossed
 * ZENITH_VH_ARCH_FREQ_SCALE_STEP -- arms the peer cluster's
 * peer_ramp window via zenith_peer_ramp_arm().  When 0 the probe is
 * still installed (the cost is one branch on `enable`) but performs
 * no work.
 *
 * Why default 0: the hook fires from arbitrary scheduler context and
 * is purely additive on top of the existing decision-time peer_ramp
 * arming.  Cold-boot users keep the historical timing; opting in is
 * a single sysfs write or a profile flip.
 *
 * Profile bakes (auto-tune):
 *   PERFORMANCE:  1   (track realisation; pre-arm peer aggressively)
 *   BALANCED:     0   (cold-boot default; opt-in only)
 *   BATTERY:      0   (extra cross-cluster wakes are not worth it)
 *   LEGACY:       0   (historical-compat)
 *   GAMING:       1   (tighten cross-cluster coupling for input/render)
 *   AUDIO:        0   (audio path benefits from steady cluster, not
 *                      cross-cluster pre-arm)
 *   CUSTOM:       0   (cold-boot opt-in)
 *
 * ZENITH_VH_ARCH_FREQ_SCALE_STEP gates the peer-ramp arm: only a
 * climb of >= 51/1024 of SCHED_CAPACITY_SCALE counts (~5%, which
 * filters governor-noise re-evaluations of the same OPP without
 * dropping real cluster ramps).
 */

/* vh_uclamp_observer_enable (default 0, [Patch B9-2]):
 *
 * Master 0/1 gate for the android_vh_setscheduler_uclamp vendor-hook
 * observer.  When 1, every userspace uclamp_min raise (the path
 * Android Dynamic Performance Framework -- ADPF -- uses to express
 * "this thread needs more headroom") drops into
 * zenith_probe_setscheduler_uclamp(), looks up the task's current
 * CPU's policy, and -- if that CPU is in a zenith-driven policy --
 * arms peer_ramp on that policy's peer cluster.  Synchronous
 * peer-cluster pre-arm: previously zenith only saw the raised
 * uclamp_min after PELT propagation moved task util upwards, which
 * can take 4..32 ms on a hot game thread.  When 0 the probe is
 * still installed (one branch on `enable`) but performs no work.
 *
 * Filtering inside the probe:
 *   - clamp_id != UCLAMP_MIN -> ignore (uclamp_max raises do not
 *     justify a peer-cluster arm; they only constrain the task's
 *     own cluster downward).
 *   - value == 0 -> ignore (a clear, not a raise).
 *
 * Profile bakes (auto-tune):
 *   PERFORMANCE:  1   (synchronous ADPF response, no PELT lag)
 *   BALANCED:     0   (cold-boot default; opt-in only)
 *   BATTERY:      0   (extra cross-cluster wakes are not worth it)
 *   LEGACY:       0   (historical-compat)
 *   GAMING:       1   (tighten ADPF-triggered cluster coupling --
 *                      this is the headline workload for the hook)
 *   AUDIO:        0   (audio path benefits from steady cluster, not
 *                      cross-cluster pre-arm)
 *   CUSTOM:       0   (cold-boot opt-in)
 */

/* vh_cpu_idle_enable (default 1, [Patch B9-3]):
 *
 * Master 0/1 gate for the android_vh_cpu_idle_enter /
 * android_vh_cpu_idle_exit vendor-hook observer pair.  When 1, every
 * cpuidle exit on a CPU belonging to a zenith-driven policy stamps
 * the residency of that idle period (exit_ns - enter_ns) into
 * z_policy->vh_cpu_idle_last_residency_ns.  zenith_get_next_freq()
 * then suppresses cluster_wake_pulse arming when the cluster just
 * emerged from a deep idle (>= ZENITH_VH_CPU_IDLE_RESIDENCY_LONG_NS).
 *
 * Rationale: cluster_wake_pulse exists to compensate for cold-cache
 * latency after a brief micro-idle.  After a deep idle the workload
 * waking the cluster is fresh / sparse and the next eval will tell
 * us the actual demand within one rate window -- forcing a pulse
 * floor would just waste energy ramping past real demand.  This
 * argument applies equally to every profile, so the gate is enabled
 * across the board out of the box; the sysfs knob remains as a
 * runtime kill-switch in case a regression needs to be triaged
 * without a rebuild.
 *
 * Read-only observer: the enter probe does NOT mutate the cpuidle
 * state index passed by the hook (the hook permits mutation; we
 * leave cpuidle's own selection untouched).  When 0 the probe is
 * still installed (one branch on `enable`) but performs no work.
 *
 * Profile bakes (auto-tune):
 *   PERFORMANCE:  1   (fresh-demand reading after deep idle, no
 *                      pulse waste)
 *   BALANCED:     1   (cold-boot default; same energy-saving
 *                      argument as PERFORMANCE)
 *   BATTERY:      1   (energy-sensitive profile -- skipping
 *                      wasteful ramps directly serves the goal)
 *   LEGACY:       1   (historical-compat profile keeps the new
 *                      gate on; runtime kill-switch via sysfs is
 *                      always available)
 *   GAMING:       1   (frame-pace stability beats cold-cache pulse)
 *   AUDIO:        1   (audio path also benefits from skipping
 *                      pulses after long idles between callbacks)
 *   CUSTOM:       1   (cold-boot inherits the default constant)
 *
 * ZENITH_VH_CPU_IDLE_RESIDENCY_LONG_NS gates the suppression: only
 * an idle period >= 4 ms counts as "deep" enough to gate the
 * pulse.  Brief idles (<<4 ms) leave cwp arming untouched.
 */

/* vh_freq_qos_enable (default 0, [Patch B9-3+]):
 *
 * Master 0/1 gate for the android_vh_freq_qos_update_request vendor-
 * hook observer.  When 1, every freq-QoS update against a zenith-
 * driven cpufreq policy that raises FREQ_QOS_MIN to a value at or
 * above ZENITH_VH_FREQ_QOS_MIN_PCT of cpuinfo.max_freq stamps a
 * pressure window timestamp on the per-tunables atomic
 * vh_freq_qos_pressure_until_ns.  zenith_auto_classify() then biases
 * to PERFORMANCE while the timestamp is still ahead of the current
 * ktime_get_ns(), so the auto-selector can pivot in response to a
 * deliberate vendor / thermal / ADPF "I want sustained high freq"
 * signal rather than waiting for PELT load to climb.
 *
 * Read-only observer: the probe never mutates req or value; it only
 * stamps a timestamp on a hit.  When 0 the probe is still installed
 * (one branch on `enable`) but performs no work, and the auto-
 * selector consume side likewise short-circuits.
 *
 * Profile bakes (auto-tune):
 *   PERFORMANCE:  1   (probe runs for telemetry symmetry; auto-
 *                      classify check is a no-op self-pivot)
 *   BALANCED:     1   (default profile; this is where the AUTO
 *                      pivot to PERFORMANCE on QoS pressure
 *                      actually matters -- the engine sits in
 *                      BALANCED most of the time)
 *   BATTERY:      0   (explicit "ignore vendor pressure, save
 *                      battery"; user-chosen profile must win)
 *   LEGACY:       0   (historical-compat profile keeps the new
 *                      gate off; runtime opt-in via sysfs is
 *                      always available)
 *   GAMING:       0   (already aggressive headroom; redundant)
 *   AUDIO:        0   (audio takes precedence in the cascade
 *                      anyway; the bake is a no-op)
 *   CUSTOM:       0   (cold-boot inherits the default constant)
 *
 * Note: only FREQ_QOS_MIN raises trigger the pressure stamp.
 * FREQ_QOS_MAX updates (typically thermal / battery caps lowering
 * the ceiling) are deliberately ignored -- a thermal cap should not
 * perversely pivot the engine to PERFORMANCE.
 *
 * ZENITH_VH_FREQ_QOS_MIN_PCT (default 75) is the threshold relative
 * to policy->cpuinfo.max_freq; a request below this is not "high"
 * pressure and is ignored.  ZENITH_VH_FREQ_QOS_WINDOW_MS (default
 * 2000) is how long a single hit keeps the pressure flag armed; it
 * matches the default auto_hysteresis_ms so a transient single
 * request lands in the noise the auto-selector already debounces,
 * while a sustained sequence of QoS raises (HAL polling at 250 ms
 * cadence, ADPF push) keeps the flag continuously armed.
 */

/* vh_sched_move_task_enable (default 0, [Patch B9-5]):
 *
 * Master 0/1 gate for the android_vh_sched_move_task vendor-hook
 * observer.  When 1, every cgroup move that lands a task whose
 * task_cpu() belongs to a zenith-driven cpufreq policy stamps a
 * jiffies timestamp on z_policy->vh_sched_move_task_last_jiffies.
 * Cgroup churn on Android (Activity#onResume / shell cpuset
 * reassignment / top-app promotion) clusters tightly around the
 * "user just brought an app forward" instant; observing this churn
 * synchronously, instead of waiting for the auto-selector worker
 * to next tick, gives AUTO mode a low-latency foreground-transition
 * signal that does not require polling.
 *
 * Read-only observer: the probe never mutates the task; it only
 * stamps the timestamp on a hit.  When 0 the probe is still
 * installed (one branch on `enable`) but performs no work and the
 * timestamp never moves off zero.
 *
 * Hook flavor: this is android_vh_sched_move_task (a regular
 * DECLARE_HOOK, multiple registrants permitted).  The audit-list
 * adjacent candidates -- android_rvh_after_enqueue_task,
 * android_rvh_after_dequeue_task, android_rvh_wake_up_new_task --
 * are deliberately not wired here: they are DECLARE_RESTRICTED_HOOKs
 * (single registrant only, never unregisterable), so a kernel-image
 * registration would permanently monopolise them and block every
 * vendor SoC kernel module that wants to register the same hook for
 * production scheduling decisions.
 *
 * Profile bakes (auto-tune):
 *   PERFORMANCE:  0   (no AUTO mode active under explicit
 *                      PERFORMANCE; the probe is observability-only,
 *                      cold-boot opt-in only)
 *   BALANCED:     0   (default profile; AUTO is the consumer of
 *                      this signal but the gate stays opt-in until
 *                      shipping data confirms the timestamp is
 *                      consumed without false positives)
 *   BATTERY:      0   (energy-sensitive profile; observability-only
 *                      probes default off)
 *   LEGACY:       0   (historical-compat profile keeps the new
 *                      gate off; runtime opt-in via sysfs is
 *                      always available)
 *   GAMING:       0   (game cgroup placement is upstream of the
 *                      profile pivot, not downstream; this signal
 *                      does not change a GAMING decision)
 *   AUDIO:        0   (audio path doesn't pivot on cgroup churn)
 *   CUSTOM:       0   (cold-boot inherits the default constant)
 *
 * Hot-path note: android_vh_sched_move_task fires once per
 * cgroup-move (sched_move_task() in core.c), which on Android is
 * sub-Hz in steady state and bursts to a few tens of events per
 * second during app launches / activity transitions.  The probe is
 * lock-free (cpufreq_cpu_get_raw + READ_ONCE on governor_data, then
 * a single WRITE_ONCE to a per-policy field), so even worst-case
 * burst rate is irrelevant to scheduler-tick budget.
 */

/* vh_scheduler_tick_enable (default 0, [Patch B9-4]):
 *
 * Master 0/1 gate for the android_vh_scheduler_tick vendor-hook
 * observer.  When 1, every scheduler tick that fires on a CPU
 * belonging to a zenith-driven cpufreq policy stamps the wall-time
 * (ktime_get_ns()) of that tick on z_cpu->vh_scheduler_tick_last_ns
 * and bumps z_cpu->vh_scheduler_tick_count by one.  Per-CPU storage:
 * the only writer is the local CPU's tick handler, so no atomics
 * are needed; remote-CPU readers use READ_ONCE.
 *
 * Read-only observer: the probe never mutates the rq, the task, or
 * any scheduler state; it only stamps the timestamp / count pair on
 * a hit.  When 0 the probe is still installed (one branch on
 * `enable`) but performs no work and the per-CPU fields stay at
 * zero.
 *
 * Hook flavor: this is android_vh_scheduler_tick (a regular
 * DECLARE_HOOK, multiple registrants permitted).  The audit-list
 * adjacent candidates -- android_rvh_after_enqueue_task,
 * android_rvh_after_dequeue_task, android_rvh_wake_up_new_task --
 * are deliberately not wired here: they are DECLARE_RESTRICTED_HOOKs
 * (single registrant only, never unregisterable), so a kernel-image
 * registration would permanently monopolise them and block every
 * vendor SoC kernel module that wants to register the same hook for
 * production scheduling decisions.
 *
 * Profile bakes (auto-tune):
 *   PERFORMANCE:  0   (no AUTO mode active under explicit
 *                      PERFORMANCE; the probe is observability-only,
 *                      cold-boot opt-in only)
 *   BALANCED:     0   (default profile; gate stays opt-in until
 *                      shipping data confirms the count + timestamp
 *                      pair is consumed without false positives)
 *   BATTERY:      0   (energy-sensitive profile; observability-only
 *                      probes default off -- HZ * num_CPUs hits/s
 *                      means even a no-op probe path is a permanent
 *                      branch in the tick fast path)
 *   LEGACY:       0   (historical-compat profile keeps the new
 *                      gate off; runtime opt-in via sysfs is
 *                      always available)
 *   GAMING:       0   (already heavily-tuned tick treatment elsewhere
 *                      in the governor; no need to layer another
 *                      observer on top by default)
 *   AUDIO:        0   (audio path doesn't pivot on tick recency)
 *   CUSTOM:       0   (cold-boot inherits the default constant)
 *
 * Hot-path note: android_vh_scheduler_tick fires once per scheduler
 * tick, i.e. HZ * num_present_cpus calls per second (250 * 8 = 2000
 * /s on a typical Android arm64 board).  This is the strictest hot
 * path of any zenith vendor-hook observer.  The probe contract is
 * therefore: cpufreq_cpu_get_raw + READ_ONCE on governor_data and
 * tunable gate, then a single ktime_get_ns() and two WRITE_ONCEs to
 * per-CPU fields.  No mutex, no spinlock, no atomic_*.  Tick context
 * runs preempt-disabled with no rq lock held (the hook fires after
 * rq_unlock + trigger_load_balance in scheduler_tick()), so the
 * probe is already in a sleepless / lock-free regime; the only
 * remaining cost is the function call itself and the branch on the
 * enable gate.
 */

/* Patch B-AUTO-3: auto-selector engine cadence and hysteresis.
 *
 * auto_eval_ms (default 500): the deferrable workqueue runs the
 * classifier once every auto_eval_ms milliseconds when
 * active_profile == ZENITH_PROFILE_AUTO.  500 ms is the minimum
 * cadence that reliably catches the audio / camera open events
 * without polling so often that we waste wakeups.
 *
 * auto_hysteresis_ms (default 2000): the classifier's chosen
 * target must hold for at least auto_hysteresis_ms before zenith
 * commits the profile switch.  This debounces transient bursts
 * (e.g. a 1 s notification ping that briefly trips the audio
 * detector) so the device does not flap profiles every few
 * hundred ms.
 *
 * Both fields accept 0 -- 0 disables the engine wholesale (the
 * worker re-arms but exits the classifier early).  B-AUTO-5
 * promotes auto_eval_ms / auto_hysteresis_ms to profile-baked
 * defaults; until then they live as the universal defaults
 * defined here.
 *
 * Bounds:
 *   auto_eval_ms        100 .. 60000
 *   auto_hysteresis_ms  0   .. 60000
 *
 * The lower bound on auto_eval_ms keeps the worker out of the
 * 1-cpu-pinned-to-pollworker territory; the upper bound on either
 * cap keeps integration testing tractable.
 */

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

/* Time-based cache TTL for the uclamp_{min,max} per-policy walks.  The
 * per-rq UCLAMP values are maintained by the scheduler on every
 * enqueue / dequeue, so a 1 ms staleness bound on the cached
 * aggregate is imperceptible to userspace (ADPF sessions are open
 * for tens of milliseconds to seconds) but drops the per-policy rq
 * walk from every freq eval down to once per millisecond.
 */

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

/* M2 V2 state-transition history ring depth.  32 entries = 512 B per
 * policy (16 B / entry, see struct zenith_policy::at_history).  Sized
 * to span ~30 s of typical phone-class bursty workloads (camera open,
 * scroll, app launch) so a userspace triager reading the file can see
 * the last interesting cluster of state changes without round-tripping
 * to dmesg / perfetto.
 */


/* Patch B-AUTO-2: auto-profile selector meta-state.
 *
 * When active_profile == ZENITH_PROFILE_AUTO the auto-selector
 * engine (B-AUTO-3 / B-AUTO-4) drives profile bakes from observed
 * device state -- audio activity, game-engine threads, render-
 * thread saturation, foreground input recency, screen state, and
 * (B-AUTO-4) battery level / charging state.  The user sees a
 * single sysfs knob ("echo auto > profile") and zenith picks the
 * most appropriate concrete profile (BALANCED, PERFORMANCE,
 * BATTERY, GAMING, or AUDIO) on a 500 ms cadence with 2000 ms
 * hysteresis.
 *
 * AUTO is a *meta* profile: zenith_apply_profile(t, AUTO) is never
 * called.  Instead the engine writes the chosen concrete target
 * into tunables->auto_target and applies that.  active_profile
 * stays at AUTO; auto_target reflects the engine's current pick.
 *
 * Manual profiles (PERFORMANCE / BALANCED / BATTERY / LEGACY /
 * GAMING / AUDIO / CUSTOM) take precedence on write -- writing any
 * concrete profile to the profile sysfs node disengages auto until
 * the user explicitly writes "auto" again.
 *
 * LEGACY and CUSTOM are never auto-picked -- they are explicit
 * opt-out paths reserved for advanced users who have layered their
 * own per-knob tweaks on top.
 */

/* Profile selected via the zenith.profile= kernel cmdline. Parsed by
 * zenith_setup_profile() at early_param time and consumed on the
 * first-init branch of zenith_init() so the governor comes up on the
 * requested preset before any userspace can write to the profile sysfs
 * node. Defaults to CUSTOM, which means "no cmdline override".
 */
unsigned int zenith_cmdline_profile = ZENITH_PROFILE_CUSTOM;

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
u8 zenith_cmdline_policy_profile[NR_CPUS] = {
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
 *   - zenith_audio_aware_key and zenith_render_aware_key match scalars
 *     that were flipped to default 1 in the wave-2 auto-defaults
 *     round, so the keys must be explicitly enabled in zenith_init()
 *     after the tunable defaults are written, otherwise the hot path
 *     would read the scalar as 1 but skip the branch via the still-FALSE
 *     key.  zenith_init() now calls zenith_set_static_key() against
 *     each scalar's value (idempotent across re-attaches).
 *   - zenith_camera_aware_key and zenith_psi_aware_key match scalars
 *     that were flipped to default 1 in the auto-defaults round that
 *     mirrored audio_aware / render_aware, so the same init-time sync
 *     rule applies: zenith_init() must enable the key against the
 *     default scalar value.
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
 *   - zenith_thermal_aware_key matches thermal_aware, which defaults
 *     to 1 (the master gate is on out of the box).  Same init-time
 *     sync rule as audio_aware / render_aware: zenith_init() must
 *     call zenith_set_static_key() so the key starts TRUE; otherwise
 *     the gated thermal mechanisms (thermal_util_derate,
 *     auto_thermal_cap, the V2 THERMAL_RECOVERY transitions, and
 *     zenith_thermal_active()) would read the scalar as 1 but skip
 *     the branch via the still-FALSE key, silently disabling thermal
 *     handling on a default install.
 *   - zenith_set_profile_defaults() never touches any of the seven
 *     scalars (they are user-managed opt-ins, not preset state), so
 *     no profile-apply path needs to re-sync the keys.
 */
DEFINE_STATIC_KEY_FALSE(zenith_audio_aware_key);
DEFINE_STATIC_KEY_FALSE(zenith_camera_aware_key);
DEFINE_STATIC_KEY_FALSE(zenith_render_aware_key);
DEFINE_STATIC_KEY_FALSE(zenith_psi_aware_key);
DEFINE_STATIC_KEY_FALSE(zenith_game_auto_key);
DEFINE_STATIC_KEY_FALSE(zenith_auto_tune_v3_key);
DEFINE_STATIC_KEY_FALSE(zenith_thermal_aware_key);
/* Patch K: master gate for game_perf_burst.  Defaults TRUE because
 * the matching scalar tunables->game_perf_burst defaults to 1 (the
 * user requested "all automatic"); zenith_init() syncs the key to
 * the scalar at attach time exactly as audio_aware / render_aware /
 * etc. above.  When the master is 0 the static branch costs nothing
 * on the hot path -- the FSM evaluator and floor application both
 * sit inside ZENITH_FEATURE_ENABLED(game_perf_burst) blocks.
 */
DEFINE_STATIC_KEY_FALSE(zenith_game_perf_burst_key);

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
 *     audio_aware / render_aware / camera_aware / psi_aware /
 *     game_auto / auto_tune_v3 keys against their default scalars
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

	static_branch_likely(&zenith_##name##_key)


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
/*
 * thermal_aware: master gate for the cluster of thermal mechanisms
 * the governor exposes as separate tunables.  When 1 (default), the
 * gated mechanisms are active subject to their own per-mechanism
 * tunables; when 0, all of the following short-circuit to no-ops
 * regardless of their per-mechanism switch:
 *
 *   - thermal_util_derate (level term in zenith_get_util())
 *   - thermal_derate_rate_pct (slope term, lives inside the same
 *     thermal_util_derate block, so the master gate covers it)
 *   - the thermal_pressure_continuous up_thresh ramp inside the
 *     screen-off / zenith_thermal_active() path (gated indirectly
 *     because zenith_thermal_active() short-circuits to false when
 *     the master gate is off)
 *   - auto_thermal_cap (target_freq cap in zenith_get_next_freq())
 *   - the auto_tune_v2 THERMAL_RECOVERY state transitions
 *
 * Default 1 to preserve the current shipping behaviour: every
 * mechanism whose individual tunable is on stays on without any
 * userspace flip.  Audited 2026-05-07 (zenith-tunables-audit) as the
 * single naming/master-gate cleanup that lets userspace turn off all
 * thermal-driven freq adjustment at once for benchmarking, captures,
 * or thermal-test rigs without having to know the names of every
 * thermal sub-tunable.  Strict 0/1 boolean.  Static-key gated for
 * branchless cost when on; sysfs store calls
 * zenith_set_static_key() to keep the key state in sync with the
 * scalar.
 */
/*
 * thermal_pressure_continuous default flipped from 0 to 1 in the
 * wave-2 auto-defaults round.  The legacy hard-cliff path snapped
 * dynamic_up_thresh to 90% the moment thermal_state turned on; the
 * continuous path linearly ramps from up_threshold (at 0% pressure)
 * to 90% (at 100% pressure) using the same arch_scale_thermal_pressure
 * percentage that V2 consumes.  No KMI exposure; the runtime path is
 * gated on thermal_auto and a non-zero pressure reading.
 */

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

/* brutal_decay_ms upper bound.  500ms is generous: a longer window
 * is functionally indistinguishable from "no cliff exit" because
 * the underlying EAS / load signal will move the floor anyway.
 */

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

/* auto_thermal_cap (default 0, off):
 *
 * Final-stage hard cap on target_freq when the per-policy thermal
 * pressure (arch_scale_thermal_pressure() expressed as a
 * percentage of capacity) is sustained at or above
 * auto_thermal_cap_pressure_pct.  Layered AFTER the level /
 * derivative util_derate paths and the V2 THERMAL_RECOVERY state
 * machine, this tier acts as an absolute upper bound on freq.
 * Workloads that race past those mechanisms (input_boost,
 * peak_headroom_rescue, frame_overrun, dl_task_floor) cannot pin
 * policy->max indefinitely once thermal pressure exceeds the
 * configured threshold while this gate is on.
 *
 * Default is OFF (=0) so existing tunings are unchanged on
 * upgrade.  Operators that observe sustained thermal climbs in
 * spite of the V2 state machine flip auto_thermal_cap=1 per policy
 * and tune the threshold / cap pair to taste:
 *
 *   echo 1 > /sys/devices/system/cpu/cpufreq/policy0/zenith/\
 *           auto_thermal_cap
 *   echo 50 > .../auto_thermal_cap_pressure_pct  (fire at >= 50%%)
 *   echo 80 > .../auto_thermal_cap_freq_pct      (cap to 80%% of max)
 *
 * Pressure threshold is bounded ZENITH_AUTO_THERMAL_CAP_PRESSURE_-
 * PCT_{MIN,MAX} (1..100); freq cap is bounded ZENITH_AUTO_-
 * THERMAL_CAP_FREQ_PCT_{MIN,MAX} (50..100) so an accidental
 * "= 0" cannot zero the cluster.
 *
 * The cap is applied AFTER em_cap (Step 6) so the EM ladder still
 * has a chance to validate the resolved freq against the energy
 * model; auto_thermal_cap then clamps to the smaller of (em_cap
 * result, policy->max * auto_thermal_cap_freq_pct / 100).
 *
 * tp_path = "auto_thermal_cap" when the cap fires.  Counted in
 * ZENITH_STAT_AUTO_THERMAL_CAP via zenith_path_to_bucket() and
 * surfaced as auto_thermal_cap=N in the zenith_stats sysfs node.
 *
 * No KMI exposure (governor-private sysfs).  The runtime path is
 * gated on the boolean tunable; when 0 it short-circuits before
 * any pressure read, costing a single READ_ONCE per call.
 */

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

/* wakeup_boost_ms upper bound.  200ms is past the perceptible
 * threshold for a wakeup transition; longer windows just bleed
 * into the steady-state climb logic and waste battery.
 */

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

/* Defensive sysfs upper bound for up_rate_limit_us and down_rate_limit_us
 * (microseconds).  60 seconds is well past any sane cpufreq sampling
 * cadence; the goal is to reject UINT_MAX-style garbage at the sysfs
 * layer rather than to impose a tight policy.
 */

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
/* Defensive sysfs upper bound for light_load_freq (kHz).  50 GHz is well
 * past any real CPU; rejects UINT_MAX-style garbage at the sysfs layer.
 */

/* auto_tune classifier thresholds. Exposed as tunables so userspace can
 * tune what the observer considers "saturated" and the saturation /
 * input-event cutoffs that select performance vs battery.
 */

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

/* F2: PELT util-rising trend default.  25% window-to-window growth
 * is conservative -- a cold app launch typically pushes 50-100%, a
 * web first-paint 30-60%.  Setting to 0 disables the signal cleanly
 * (the V1 worker computes the delta but never raises the flag).
 */

/* F3: render-thread RT-priority floor (per-policy uclamp-min-style).
 *
 * When auto_tune_render_rt_floor_pct > 0 AND the V2 state machine
 * has committed to LATENCY or SUSTAINED_PERF AND ZENITH_AT_FLAG_RENDER
 * is currently active in at_last_flags, raise the freq floor to
 * (policy->max * auto_tune_render_rt_floor_pct / 100).  This is the
 * lower-risk variant of the original F3 design: instead of touching
 * task->sched_class via sched_setscheduler_nocheck() (which would
 * collide with audio_server's RT-priority inheritance trees), we
 * publish a per-policy uclamp-min-style hint that simply guarantees
 * RenderThread / surfaceflinger sees enough freq headroom to run
 * uninterrupted by background CFS tasks while V2 says the workload
 * is responsiveness-critical.
 *
 * Difference from render_floor_pct: render_floor_pct fires whenever
 * a render thread is observed running, regardless of V2 state.  The
 * F3 floor fires only after V2 has *already* committed to LATENCY
 * or SUSTAINED_PERF, so the user has signalled "this is jank-
 * sensitive workload".  In that regime, even a single CFS preemption
 * of a RenderThread is visible as a frame stutter; the floor adds
 * the headroom that makes the preemption window survivable.
 *
 * Default 0 (off, conservative).  Range 0..100; 0 disables the floor
 * cleanly.  Recommended user value if enabling: 85..95.  The floor
 * is OR-ed with (max of) the existing render_floor_pct floor; if
 * F3 is on and conditions fire, F3 always wins.
 */

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


/* Transition-rate thresholds.  Counted within the ZENITH_AT_LOG_NR
 * (=16) window which spans up to ~16 V1 cycles (~160 s with the
 * default V1 cadence).  HI/LO are absolute counts, not rates per
 * unit time -- the calibration cadence is stable enough that
 * counts work fine.
 */

/* Bounded signed offset range for the two V2 reaction knobs. */

/* Per-policy V3 calibration ring depth.  Each slot records one
 * zenith_at_v3_calibrate() invocation (both OBSERVE and APPLY modes,
 * so operators can see when V3 was running and how it nudged the
 * offsets).  Eight entries cover ~8 minutes of history at the
 * default 60 s interval and ~80 minutes at the 600 s clamp; small
 * enough to fit the read inside a sysfs PAGE_SIZE comfortably with
 * the per-policy header line.  Bumping this is cheap (8 bytes per
 * entry) but anything past PAGE_SIZE bytes will get truncated by
 * the show handler's bound check.
 */


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

/* Effective values applied by zenith_at_apply_glides() per state.
 * Picked to match the round-U-z10 doc recommendations and keep all
 * seven knobs inside their documented sysfs ranges.
 */



/* Set by the V1 classifier worker when prefer_silver_aware is on AND
 * the prefer_silver hit-rate over the last classifier window crossed
 * the prefer_silver_hot_threshold_pct cutoff.  Read-only signal; the
 * actual dynamic_up_thresh bump is applied directly in
 * zenith_get_next_freq() (the signal does not feed the V2 state
 * machine because prefer_silver redistribution is workload-dependent
 * and would race with the existing thermal / PSI / frame triggers).
 */
/* Audit fix F2: PELT-derived util-rising trend signal.
 *
 * Set by the V1 auto-tune worker when the policy-wide util average
 * delta between this window and the last window exceeded
 * t->auto_tune_util_rising_thresh_pct.  Used by V2 to bias toward
 * LATENCY when load is rapidly ramping (e.g. cold app launch, web
 * page first-paint, game scene transition) before sat_pct fully
 * crosses the hi_sat_pct threshold.
 *
 * Read-only flag like THERMAL_SLOPE and PREFER_SILVER_HOT; the
 * actual state-machine consumption happens in zenith_v2_propose()
 * (specifically the BALANCED -> LATENCY edge).  Bit chosen to
 * leave the LSB nibble for stable scenario flags (camera/audio/
 * render/etc.) and the next nibble for environmental signals.
 */


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

/* Patch M1: PSI-mem light cap.  Three new override bits, sitting
 * at the end of the existing tier-overrides bank.  Same shape as
 * the K1/K2/K3 override bits above: a sysfs write to any of the
 * three knobs ORs in its bit, and the V2 worker stops touching
 * that specific knob until the next profile flip clears the
 * mask.  Profile-flip-clears-mask is implemented in
 * zenith_apply_profile() (existing code, no edit needed here).
 */

/* Floor/cap-knob override fence.  Five sysfs knobs that are read on
 * the freq-update hot path but are not driven by any V2 / V3 path
 * today.  The bits sit dormant: a sysfs write to any of the five
 * knobs ORs in its bit, and any future zenith_at_set_uint() caller
 * gated on these bits will short-circuit, leaving the user value
 * alone.  Profile-flip-clears-mask is in zenith_apply_profile()
 * already (existing code, no edit needed here).
 */

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
/* Patch M1: PSI-mem cap tier.  Armed in EFFICIENCY / BALANCED /
 * THERMAL_RECOVERY (states where backing off on memstall is the
 * desired behaviour); disarmed in LATENCY / SUSTAINED_PERF and
 * under the FRAME / GAME flag bypass.  Cap is final-freq, not a
 * predicate; disarm here turns the read-site into a no-op for
 * the duration of the V2 window regardless of the profile-set
 * thresh value.
 */


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

/* util_math_v2 (default 1): when 1, zenith_get_util() folds the cfs_rq
 * runnable_avg into the util signal alongside util_avg / util_est, in
 * the same shape as 6.x cpu_util_cfs_boost(). Helps intermittent
 * tasks (UI thread + render thread spikes) without changing PELT or
 * util_est semantics. Enabled by default for better responsiveness
 * to short burst workloads (UI/render threads).
 */

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

/* psi_aware (default 1, on) + psi_mem_thresh (default 50)
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
 *
 * Default flipped from 0 to 1 in the auto-defaults round that
 * accompanies camera_aware: zenith is intended to be self-tuning, and
 * leaving the gate off meant the memstall cap shipped dormant.  The
 * out-of-box impact is a memstall reader gated by psi_mem_thresh = 50
 * (i.e. only fires under sustained heavy memory pressure); the cpu
 * and io caps remain off-by-threshold (0).
 */

/* psi_cgroup_path (default "" = use system-wide PSI):
 *
 * When non-empty, names a cgroup-v2 path (relative to the unified
 * hierarchy root, e.g. "/foreground") whose per-cgroup PSI averages
 * the zenith_psi_*_some_pct() consumers should read in place of
 * psi_system.avg[][].  Resolved at sysfs-store and at
 * zenith_apply_profile() time via cgroup_get_from_path(); the
 * resolved cgroup pointer is cached in zenith_psi_cgroup (RCU-
 * protected) so the hot-path read is still close to a single
 * READ_ONCE on the EWMA word.
 *
 * Empty string is the safe default and reproduces the pre-B10
 * behaviour exactly: the helpers fall back to &psi_system, which is
 * what every existing call site read before this knob existed.
 *
 * Path resolution failures (cgroup-v2 not mounted, path missing,
 * cgroup-v1-only system) are silently demoted to the empty/system-
 * wide path; no -ENOENT to userspace.  This keeps profile-baked
 * defaults safe across vendor cgroup naming variants.
 *
 * Path length cap is a generous compromise between cgroup hierarchy
 * depth and inline-buffer cost (one buffer per attr_set, not per
 * policy).
 */

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

/* Patch B7-2: decision-ring depth.  Per-policy circular buffer of
 * the last ZENITH_DEC_RING_NR (path, lat_ns) entries.  Power-of-two
 * so head advance is a single AND.  Storage budget: 32 entries *
 * sizeof(struct zenith_dec_ring_entry) per policy (~512 B).
 */

/* camera_aware (default 1, on) + camera_active (default 0, auto)
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
 *
 * Default flipped from 0 to 1 in the auto-defaults round that
 * accompanies psi_aware: zenith is intended to be self-tuning, and
 * shipping the gate off meant the camera floor never fired without
 * an explicit sysfs poke.  Out-of-box impact is a comm-walk on the
 * eval-cadence path (cached for ZENITH_CAMERA_CACHE_TTL_NS); no
 * frequency clamp is applied unless camera_floor_pct is also set
 * to a non-zero value (it stays at 0 by default).
 */


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

/* screen_off_glide_ms upper bound.  2s past the 1->0 transition is
 * generous: AOD / blanking handlers in Android typically settle in
 * well under 500ms.  0 keeps the historical cliff exactly.
 */

/* boot_boost_decay_ms upper bound.  30s is enough for the slowest
 * Android cold-boot animation; anything longer would conflict with
 * the screen_off detection that may legitimately follow boot.
 */

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

/* uclamp_min threshold (in percent of SCHED_CAPACITY_SCALE) above
 * which the screen-off override suppression kicks in. 10 %% means the
 * task's ADPF hint has to reach uclamp_min >= ~102/1024 (about big-core
 * idle-loop capacity) before zenith considers it worth bypassing the
 * screen-off penalty.  Hardcoded rather than tunable -- it is a
 * definition of "meaningful", not a policy knob.
 */

/* kcpustat tunable bounds. window_us is clamped on store to keep the
 * sampler from thrashing or overflowing; filter_shift caps below the
 * width of an unsigned int.
 */

/* Patch L: master switch for human-readable zenith state logging.
 * When ZENITH_DEFAULT_VERBOSE_LOG is non-zero, profile-change /
 * profile-bake-summary / 7-master-switch-flip events emit a
 * pr_info() line tagged "zenith:" to dmesg (forwarded to logcat
 * under the KERNEL tag on Android).  Default 0 so production
 * builds do not get spammed; flipping verbose_log=1 at runtime
 * makes the logging hot.  Plain RW boolean tunable; not profile-
 * baked since logging policy is operator preference, not workload-
 * driven.
 */

/* Patch K: game / sustained-high-load performance burst.  Five
 * tunables form one mechanism:
 *
 *   game_perf_burst                       master 0/1, default 1
 *   game_perf_burst_floor_pct             freq floor while ARMED
 *   game_perf_burst_thermal_ceiling_dc    skin-temp guardrail (mC)
 *   game_perf_burst_disarm_grace_ms       sustained-clear hold for disarm
 *   game_perf_burst_cooldown_ms           post-disarm step-down glide
 *
 * Detection is a multi-signal AND gate evaluated once per
 * zenith_get_next_freq() invocation (250us..few-ms cadence
 * depending on load / kthread / fast-switch path):
 *
 *   A.  zenith_eff_game_mode() != 0
 *       (manual game_mode write or in-kernel game_auto latched on a
 *        sustained known-game comm-walk match -- see ZENITH_DEFAULT_-
 *        GAME_AUTO comment block).
 *   B.  Sustained big-cluster util >= 70% for >= 2 seconds.
 *       (Read from z_policy->last_load_pct, which is stamped every
 *        tick at the bottom of zenith_get_next_freq() with the load
 *        used by the up-thresh decision -- so this Signal sees the
 *        *previous* tick's load, exactly the right thing for a
 *        sustained-window gate.)
 *   C.  Not video-only / audio-pinned -- the existing audio_aware
 *       sticky-active deadline is treated as a video-pin proxy and
 *       suppresses arming when audio is hot but no game signal
 *       arrived first.  In practice this filters Netflix / VLC
 *       playback that would otherwise trip Signal B alone.
 *
 * Trigger: A AND (B for >= 2s) AND (NOT C-suppressing).
 *
 * Disarm: !A OR (B-clear for >= disarm_grace_ms).  Both edges are
 * cheap loads on the hot path.  After disarm the FSM enters a
 * COOLDOWN glide that linearly steps the floor down to 0 over
 * cooldown_ms milliseconds, so freq doesn't whiplash on Alt+Tab.
 *
 * Latency expectations:
 *   worst case  ~2s     (Signal B requirement) + 1 hot-path tick
 *   best case   ~250ms  (already-armed game_auto + already-saturated
 *                        big cluster, hot path runs straight to ARM)
 *   disarm      ~1s     (default disarm_grace_ms; clamps tail floor)
 *
 * Thermal guardrail (option (c) hybrid):
 *   - At zenith_start() resolve "cpu<N>-thermal" against the kernel
 *     thermal subsystem where N == cpumask_first(policy->cpus).
 *     Cache the resulting struct thermal_zone_device * in
 *     z_policy->gpb_tzd.  IS_ERR / NULL means "no zone for this
 *     policy" (foreign SoC, thermal subsystem not registered yet,
 *     etc.) and the helper falls back to arch_scale_thermal_pressure
 *     converted to a synthetic dC scale (30..70 dC across the 0..100
 *     pressure range) so the guardrail still works on platforms
 *     without a per-cluster thermal zone.
 *   - When the live skin temp >= ceiling_dc, the burst floor is
 *     suppressed for this tick and the existing auto_thermal_cap /
 *     thermal_util_derate path stays in charge.  ARMED state is
 *     retained -- as soon as temp drops below the ceiling on a
 *     subsequent tick the floor re-engages without re-arming.
 *
 * Defaults reasoning:
 *   - master = 1 because the user asked for "all automatic"; the
 *     gate keeps it from firing outside detected games / sustained
 *     load.
 *   - floor_pct = 85 sits above hispeed_freq for every preset and
 *     below policy->max enough to leave the existing thermal path
 *     useful headroom.
 *   - thermal_ceiling_dc = 48000 (48 dC) is the user's 45..50 dC
 *     range midpoint.
 *   - disarm_grace_ms = 1000 keeps Alt+Tab snappy without bouncing
 *     on a single sub-threshold tick.
 *   - cooldown_ms = 5000 lets the cluster drift back without freq
 *     whiplash; matches the existing peak_headroom / brutal_decay
 *     scale.
 */
/* Patch M: Schmitt-trigger exit threshold for Signal B.  Once the
 * sustained-arming counter has stamped, hold it through dips above
 * this floor so a scene oscillating in the 65..72% band keeps
 * accumulating sustained time instead of resetting on every dip.
 * Enter at _B_THRESHOLD_PCT (70), exit at _B_EXIT_THRESHOLD_PCT (60).
 */
/* Patch M: lazy retry interval for the per-policy thermal zone cache.
 * If thermal-core registers after the governor (boot-ordering quirk
 * on some Tensor builds), zenith_start()'s one-shot resolution leaves
 * gpb_tzd NULL and the guardrail permanently falls back to
 * arch_scale_thermal_pressure().  The hot-path helper retries
 * resolution at most once per this interval until a zone is bound.
 */

/* FSM states for game_perf_burst.  Read from sysfs as the
 * game_perf_burst_state RO node and from the floor helper to
 * decide what (if any) freq floor to apply this tick.
 */

/* Patch M: last-disarm reason tokens for the per-policy stats node.
 * Recorded by the FSM evaluator at every ARMED -> COOLDOWN edge so
 * userspace tuning of disarm_grace_ms / cooldown_ms can tell whether
 * users disarm by Alt+Tab/Home (FAST) or by load actually subsiding
 * (SUSTAINED).  NONE = no disarm has happened in the current attach.
 */

/*
 * Zenith Tunables & State API
 */

/*
 * Set by the input handler on every key/abs event. Read from the hot path
 * with atomic64_read so no governor lock is needed in the producer.
 */
atomic64_t zenith_input_boost_until_ns = ATOMIC64_INIT(0);

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
atomic64_t zenith_input_last_event_ns = ATOMIC64_INIT(0);

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
atomic64_t zenith_peer_ramp_until_ns_big   = ATOMIC64_INIT(0);
atomic64_t zenith_peer_ramp_until_ns_prime = ATOMIC64_INIT(0);

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
atomic64_t zenith_last_vblank_ns          = ATOMIC64_INIT(0);
atomic64_t zenith_frame_overrun_until_ns  = ATOMIC64_INIT(0);

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
atomic_t zenith_frame_overrun_streak       = ATOMIC_INIT(0);

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
atomic_t zenith_drm_vblank_us = ATOMIC_INIT(0);

/* Boot-completion latch.  See ZENITH_DEFAULT_BOOT_COMPLETE_AUTO
 * comment block above struct zenith_tunables.  Both globals are
 * read lock-free from zenith_get_next_freq() (the boost path) and
 * written either from the boot_complete sysfs *_store callback or
 * from zenith_auto_tune_work() once the calm streak qualifies.
 */
atomic_t zenith_boot_complete = ATOMIC_INIT(0);
u64 zenith_boot_complete_ns;

/* AC-vs-battery state cache (Patch 1.2).  Refreshed once per
 * auto_tune window (ZENITH_AUTO_TUNE_PERIOD_MS, default 10 s) by
 * zenith_auto_tune_work() via power_supply_is_system_supplied():
 *   0  -> AC power, system-supplied (default)
 *   1  -> running on battery
 * Read lock-free from zenith_batt_scaled() in the peak-rescue and
 * peak-prearm hot paths to scale hold-down deadlines when the
 * batt_hold_scale_pct profile knob requests it.  When no power
 * supply driver is registered the helper returns -ENODEV /
 * -ENOSYS and the cache stays at 0 (AC), so the default path
 * exactly preserves pre-1.2 behaviour on systems without a battery
 * (laptops on a dock, lab boards, AVDs).
 */
atomic_t zenith_on_battery = ATOMIC_INIT(0);

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
u64 zenith_game_auto_active_until_ns;

/* True if the in-kernel game detector latch is currently in the
 * future, i.e. a recent fresh detection has happened and is still
 * within ZENITH_GAME_AUTO_ACTIVE_TTL_NS.  Lock-free; readers tolerate
 * a stale value by at most one cpufreq tick.
 */
bool zenith_game_auto_active(void)
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
inline unsigned int zenith_eff_game_mode(unsigned int base_gm)
{
	if (static_branch_likely(&zenith_game_auto_key) &&
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

/* Audit fix K4: v4l2 fd-open hook callbacks.  Strong symbols that
 * override the weak stubs in drivers/media/v4l2-core/v4l2-dev.c.
 *
 * Both callbacks are called outside of v4l2's videodev_lock so they
 * can run on any context cheaply (just an atomic_inc / atomic_dec).
 * No filtering by vfl_type or v4l2_dev capabilities -- any
 * /dev/video* open counts.  See the zenith_v4l2_active_fds comment
 * for the false-positive analysis (short answer: harmless, all
 * v4l2 capture workloads benefit from LATENCY).
 *
 * Exported so the v4l2 driver (which is built from a different
 * compilation unit) can resolve them via weak-symbol override.  No
 * other in-kernel caller is expected; the EXPORT_SYMBOL_GPL is for
 * the link-time relaxation, not for module use.
 */
/* Opaque forward declaration: zenith does not look inside vdev, just
 * passes it through.  Avoids dragging linux/videodev2.h into a
 * cpufreq governor TU.
 */
struct video_device;

/* Forward declarations of the K4/K5 vendor-hook strong defs below.
 * The matching __weak prototypes live inline at the call sites
 * (drivers/media/v4l2-core/v4l2-dev.c and sound/core/pcm_native.c)
 * so callers stay zero-knowledge of the governor.  Declaring them
 * here makes the local definitions visible to -Wmissing-prototypes
 * and sparse, and ensures cpufreq_zenith.c's view of the prototype
 * is internally consistent.
 */
void zenith_v4l2_open_notify(struct video_device *vdev);
void zenith_v4l2_release_notify(struct video_device *vdev);
void zenith_alsa_pcm_open_notify(int stream);
void zenith_alsa_pcm_release_notify(int stream);

/* Tentative declarations of the K4/K5 refcount atomics.  Their
 * defining declarations (with ATOMIC_INIT(0)) live further down the
 * file next to the rest of the auto-tune state; the function bodies
 * below were originally written assuming forward visibility, which
 * the C tentative-definition rule grants only when the file-scope
 * tentative is visible at first use.  These two lines provide that
 * tentative visibility so the open / release notify functions
 * compile cleanly under CONFIG_CPU_FREQ_GOV_ZENITH=y.
 */
atomic_t zenith_v4l2_active_fds;
atomic_t zenith_alsa_active_fds;

void zenith_v4l2_open_notify(struct video_device *vdev)
{
	(void)vdev;	/* unused; we don't filter by vfl_type yet */
	atomic_inc(&zenith_v4l2_active_fds);
}
EXPORT_SYMBOL_GPL(zenith_v4l2_open_notify);

void zenith_v4l2_release_notify(struct video_device *vdev)
{
	int v;

	(void)vdev;
	v = atomic_dec_return(&zenith_v4l2_active_fds);
	if (unlikely(v < 0))
		atomic_set(&zenith_v4l2_active_fds, 0);
}
EXPORT_SYMBOL_GPL(zenith_v4l2_release_notify);

/* K5: ALSA pcm-open / pcm-release notify hooks.  Same shape as K4.
 * The `stream` parameter is the SNDRV_PCM_STREAM_PLAYBACK / CAPTURE
 * direction; we don't currently distinguish them (any active
 * stream counts), but the parameter is preserved for forward
 * compatibility (a future patch could split capture vs playback
 * for capture-only audio scenarios like voice memos).
 */
void zenith_alsa_pcm_open_notify(int stream)
{
	(void)stream;
	atomic_inc(&zenith_alsa_active_fds);
}
EXPORT_SYMBOL_GPL(zenith_alsa_pcm_open_notify);

void zenith_alsa_pcm_release_notify(int stream)
{
	int v;

	(void)stream;
	v = atomic_dec_return(&zenith_alsa_active_fds);
	if (unlikely(v < 0))
		atomic_set(&zenith_alsa_active_fds, 0);
}
EXPORT_SYMBOL_GPL(zenith_alsa_pcm_release_notify);

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
unsigned int zenith_frame_overrun_slack_us_cache =
	ZENITH_DEFAULT_FRAME_OVERRUN_SLACK_US;
unsigned int zenith_frame_overrun_window_ms_cache =
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

unsigned int zenith_input_boost_active_ms = ZENITH_DEFAULT_INPUT_BOOST_MS;

/* Governor-wide cache for the touchdown-extra knob (Patch C).
 * Mirrored from t->input_boost_touchdown_extra_ms by sysfs store
 * and zenith_apply_profile().  Read by zenith_input_event() on
 * the EV_KEY/BTN_TOUCH down path.  Stored as plain unsigned int
 * with READ_ONCE/WRITE_ONCE; the racy reader doesn't care if it
 * sees a stale value across the store window.
 */
unsigned int zenith_input_boost_touchdown_extra_ms_cache =
	ZENITH_DEFAULT_INPUT_BOOST_TOUCHDOWN_EXTRA_MS;

/* Monotonically-increasing global count of qualifying input events seen
 * by zenith_input_event. Auto-tune workers sample this periodically and
 * subtract their last-observed value to get an events-per-window rate.
 */
atomic64_t zenith_auto_input_events = ATOMIC64_INIT(0);

/* Audit fix K4: deterministic camera detection via v4l2 fd-open hook.
 *
 * The runqueue-snapshot comm-walk in zenith_policy_has_camera() is a
 * probabilistic signal -- camera HALs that sleep most of the time
 * (cameraserver, camerahalserver) are rarely on-CPU when the walk
 * runs, so the cache fills with `false` and the camera flag never
 * fires.  K4 wires zenith into v4l2-core via two weak-symbol notify
 * callbacks (drivers/media/v4l2-core/v4l2-dev.c).  Every successful
 * v4l2 fd open bumps this refcount; release decrements.  Any nonzero
 * value is treated by the auto_tune scenario block as "camera active"
 * regardless of the comm-walk result.
 *
 * Counter is signed so a tearing race during release that would
 * otherwise underflow to UINT_MAX is observable as a negative value
 * during diagnosis instead of silently looking like a stuck camera.
 * The notify functions clamp to >= 0 on every store.
 *
 * False-positive surface: any process that opens /dev/videoN
 * (USB webcam, screen recorder sink, software encoder using v4l2
 * codec node) is also accounted.  All such workloads are similarly
 * media-bandwidth-bound and benefit from the LATENCY state, so the
 * "false positive" actually does the right thing.
 */
atomic_t zenith_v4l2_active_fds = ATOMIC_INIT(0);

/* Audit fix K5: deterministic audio detection via ALSA pcm-open hook.
 *
 * Sibling to K4 -- same atomic refcount pattern, drives the audio
 * scenario flag.  See zenith_alsa_pcm_open_notify() below for the
 * exported strong symbols, and sound/core/pcm_native.c for the
 * weak symbols that hook into snd_pcm_*_open / snd_pcm_release.
 *
 * Why bother when audio_server already lights up the comm-walk
 * reliably:
 *   - first window after open: audio_server may not have started
 *     mixing yet, comm-walk misses, audio flag stays 0 for a full
 *     V1 window.
 *   - last window after close: audio_server already drained, but
 *     V1 doesn't know the stream is gone until the next walk.
 *
 * K5 closes both edges to single-tick precision.
 */
atomic_t zenith_alsa_active_fds = ATOMIC_INIT(0);

/* Audit fix F1: scenario-active classifier window.
 *
 * When any scenario flag (camera, render, frame, game, memstall,
 * thermal_slope, psi_cpu) is set, drop the V1 classifier reschedule
 * cadence from the default 10 s down to ~1.5 s.  Real-world bursty
 * workloads (camera open, app launch, scroll) finish in 1-3 s on
 * modern phone-class hardware; the 10 s default means V1 runs
 * exactly once during the burst, sees a half-saturated window, and
 * picks BALANCED -- making LATENCY commit only after the burst is
 * already over.
 *
 * This faster window only applies to the *V1 reschedule* cadence;
 * the V2 hysteresis windows continue to count in the same units (so
 * a 2-window hysteresis is now ~3 s instead of ~20 s).  V3
 * calibration interval is unchanged because V3 already has its own
 * timer (auto_tune_v3_interval_ms).
 */

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
atomic64_t zenith_in_events_total = ATOMIC64_INIT(0);
atomic64_t zenith_in_boosts_armed = ATOMIC64_INIT(0);
atomic64_t zenith_in_boosts_quiet_extended = ATOMIC64_INIT(0);
atomic64_t zenith_in_boosts_skipped_disabled = ATOMIC64_INIT(0);
atomic64_t zenith_in_boosts_early_exit = ATOMIC64_INIT(0);

/* Per-policy decision-stat buckets exposed via the readonly
 * `zenith_stats` sysfs node.  See struct zenith_policy::stats[] for
 * the storage and zenith_path_to_bucket() for the tp_path -> bucket
 * mapping.  ZENITH_STAT_OTHER is a catch-all so a future tier added
 * without a bucket mapping still gets counted (against decisions,
 * but not against any specific tier).
 */

/* One sample written by the auto-tune classifier worker into the
 * per-policy at_log ring (see ZENITH_AT_LOG_NR).  Mirrors the
 * at_last_* mirrors on struct zenith_policy at the moment the worker
 * resolved the new V1 target / V2 state, so userspace can correlate a
 * decision against the signals that drove it without bpftrace.

/* Patch J: per-policy V3 calibration ring entry.  One slot per
 * zenith_at_v3_calibrate() invocation, recording the boottime ns,
 * the V2 transition count counted from the at_log walk, the V3 mode
 * the calibration ran under (OBSERVE / APPLY -- mode 0 OFF never
 * pushes), and the hyst/cool offset before and after any APPLY-mode
 * nudge.  before == after on OBSERVE-mode entries and on
 * rail-clamped APPLY entries; the differential lets the reader
 * trivially see "when did V3 actually move me" without bpftrace.

static DEFINE_PER_CPU(struct zenith_cpu, zenith_cpu);

/* Wave B PMU IPC tracker per-CPU state.  See the comment block above
 * ZENITH_DEFAULT_PMU_AWARE for the full rationale.  Allocated by
 * zenith_pmu_init_cpu() at zenith_start() time; released by
 * zenith_pmu_exit_cpu() at zenith_stop() time.  Sampled once per
 * zenith_auto_tune_work() pass; the resulting IPC is cached in
 * ipc_pct (1.0 IPC == 100) and read by zenith_policy_max_ipc_pct().
 *
 * On CONFIG_PERF_EVENTS=n the perf_event pointers are absent and
 * the helper functions all collapse to no-ops via the #else branch
 * below.

static DEFINE_PER_CPU(struct zenith_pmu_state, zenith_pmu);

/* Wave B PMU IPC tracker forward declarations.  The bodies live next
 * to zenith_init() / zenith_start() because they share their lifecycle;
 * the call sites in zenith_auto_tune_work() and zenith_get_next_freq()
 * are upstream of the definitions in source order, so a forward
 * declaration is required.
 */
int zenith_pmu_init_cpu(unsigned int cpu);
void zenith_pmu_exit_cpu(unsigned int cpu);
void zenith_pmu_sample_cpu(unsigned int cpu);
unsigned int zenith_policy_max_ipc_pct(struct zenith_policy *z_policy);

/* Wave B EAS / Energy Model integration.  Resolve the energy-knee
 * frequency of the policy's perf_domain (the OPP with the lowest
 * em->table[].cost field).  Cached in z_policy->em_knee_freq so the
 * em->table walk runs at most once per zenith_start() pass.  Returns
 * 0 if no EM is registered for the policy or if all costs are zero
 * (badly-formed EM).  See the comment block above
 * ZENITH_DEFAULT_EM_AWARE for the full rationale.
 */
unsigned int zenith_em_knee_freq(struct zenith_policy *z_policy);

unsigned int zenith_tunable_or_local(struct zenith_policy *z_policy,
					    unsigned int tunable,
					    unsigned int local);
unsigned int zenith_glide_value(struct zenith_policy *z_policy,
				       unsigned int tunable,
				       unsigned int local);

/************************ Schedutil: I/O Wait & DL Logic ***********************/

/* Resolve the configured iowait floor for the policy owning z_cpu, in
 * absolute SCHED_CAPACITY_SCALE units. Called from every iowait path so
 * kept inline and trivial.
 */
inline unsigned int zenith_iowait_floor(struct zenith_cpu *z_cpu)
{
	unsigned int permille = z_cpu->z_policy->tunables->iowait_boost_min;

	return (SCHED_CAPACITY_SCALE * permille) / 1000;
}

bool zenith_iowait_reset(struct zenith_cpu *z_cpu, u64 time, bool set_iowait_boost)
{
	s64 delta_ns = time - z_cpu->last_update;

	if (delta_ns <= TICK_NSEC)
		return false;

	z_cpu->iowait_boost = set_iowait_boost ? zenith_iowait_floor(z_cpu) : 0;
	z_cpu->iowait_boost_pending = set_iowait_boost;
	z_cpu->iowait_boost_first_ns = set_iowait_boost ? time : 0;
	return true;
}

void zenith_iowait_boost(struct zenith_cpu *z_cpu, u64 time,
				unsigned int flags, unsigned int io_is_busy)
{
	bool set_iowait_boost = (flags & SCHED_CPUFREQ_IOWAIT) && io_is_busy;

	/* Patch C9: stamp the io_floor hysteresis deadline on every
	 * iowait sample where the boost path would arm.  Bumps a
	 * sliding window forward; once the iowait_boost decays away,
	 * the floor outlives it for io_floor_hyst_ms past the last
	 * arming sample.  io_floor_hyst_ms == 0 stamps a 0 deadline,
	 * which the read-side check in zenith_get_next_freq() treats
	 * as no-floor (legacy behaviour).
	 *
	 * z_cpu->z_policy is established non-NULL by both callers
	 * (zenith_update_single, zenith_update_shared dereference
	 * z_policy->tunables before calling us); zenith_iowait_floor()
	 * and zenith_iowait_apply() likewise dereference it
	 * unconditionally, so we follow the same convention here.
	 */
	if (set_iowait_boost) {
		unsigned int hyst_ms =
			z_cpu->z_policy->tunables->io_floor_hyst_ms;

		if (hyst_ms)
			z_cpu->z_policy->io_floor_until_ns = time +
				(u64)hyst_ms * NSEC_PER_MSEC;
	}

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

static unsigned long zenith_iowait_apply(struct zenith_cpu *z_cpu, u64 time,
					 unsigned long util, unsigned long max_cap)
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

inline void zenith_ignore_dl_rate_limit(struct zenith_cpu *z_cpu,
					       struct zenith_policy *z_policy)
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
unsigned long zenith_get_util(struct zenith_cpu *z_cpu)
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
	 * ZENITH_DEFAULT_THERMAL_UTIL_DERATE comment block.  Wrapped
	 * by the thermal_aware master gate so a userspace flip to 0
	 * disables both the level term and the slope term inside this
	 * block at zero hot-path cost when the master gate is on (the
	 * common case).
	 *
	 * Sub-floor pressure is ignored to avoid the multiply cost on
	 * the noise.  trace_zenith_thermal_derate fires only when the
	 * derate actually changes util.
	 */
	if (ZENITH_FEATURE_ENABLED(thermal_aware) &&
	    READ_ONCE(z_cpu->z_policy->tunables->thermal_util_derate) && max) {
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
unsigned int zenith_sample_nice_pct(struct zenith_cpu *z_cpu, u64 now)
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
bool zenith_thermal_active(struct zenith_policy *z_policy)
{
	struct cpufreq_policy *policy = z_policy->policy;
	struct zenith_tunables *tunables = z_policy->tunables;
	unsigned long pressure, cap;
	bool active;
	int cpu;

	/* Master gate.  When thermal_aware == 0 every consumer of this
	 * helper (the screen-off thermal cliff path, the V2 sustained
	 * paths that read pressure, anything that calls
	 * zenith_thermal_active() directly) sees "cool" regardless of
	 * actual pressure or thermal_state.  Static-key gated for
	 * branchless cost when on, the common case.  Also clear the
	 * sysfs mirror so userspace never sees a stale 1 after a
	 * userspace flip of thermal_aware to 0.
	 */
	if (!ZENITH_FEATURE_ENABLED(thermal_aware)) {
		WRITE_ONCE(tunables->thermal_active, 0);
		return false;
	}

	if (tunables->thermal_state) {
		WRITE_ONCE(tunables->thermal_active, 1);
		return true;
	}
	if (!tunables->thermal_auto) {
		WRITE_ONCE(tunables->thermal_active, 0);
		return false;
	}

	cpu = cpumask_first(policy->cpus);
	if (cpu >= nr_cpu_ids) {
		WRITE_ONCE(tunables->thermal_active, 0);
		return false;
	}

	cap = arch_scale_cpu_capacity(cpu);
	if (!cap) {
		WRITE_ONCE(tunables->thermal_active, 0);
		return false;
	}

	pressure = arch_scale_thermal_pressure(cpu);
	active = (pressure * 100 / cap) >= ZENITH_THERMAL_AUTO_PRESSURE_PCT;
	WRITE_ONCE(tunables->thermal_active, active ? 1 : 0);
	return active;
}

unsigned int zenith_policy_thermal_pressure_pct(struct zenith_policy *z_policy)
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

unsigned int zenith_em_cap_freq(struct zenith_policy *z_policy, unsigned int target_freq)
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
			/*
			 * If this state consumes disproportionately high power (heuristic:
			 * if it's the absolute highest state and we are throttling), cap it
			 * to the previous state to save mW.
			 */
			if (i == pd->nr_perf_states - 1 && i > 0)
				return pd->table[i - 1].frequency;
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

bool zenith_up_down_rate_limit(struct zenith_policy *z_policy, u64 time,
				      unsigned int next_freq)
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
void zenith_uclamp_cache_refresh(struct zenith_policy *z_policy)
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
inline unsigned int zenith_eff_hispeed_freq(struct zenith_policy *z_policy)
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

/* B10-3: optional per-cgroup PSI source (see ZENITH_PSI_CGROUP_PATH_MAX
 * and psi_cgroup_path).
 *
 * zenith_psi_cgroup is the resolved cgroup whose embedded psi_group
 * (cgroup->psi) the three zenith_psi_*_some_pct() helpers below
 * dereference under rcu_read_lock().  NULL means "use psi_system",
 * which is the pre-B10 behaviour and the default at boot.
 *
 * Gated on CONFIG_PSI && CONFIG_CGROUPS.  Without CONFIG_CGROUPS the
 * struct cgroup type is forward-declared only and cgroup_psi(),
 * cgroup_get_from_path(), cgroup_put() are absent; the cached pointer
 * and apply helper degrade to no-ops, the picker returns &psi_system
 * unconditionally, and the sysfs store accepts (and silently
 * discards) any path.
 *
 * Lifecycle:
 *   - zenith_psi_cgroup_apply() is the only writer.  It serialises
 *     mutators with zenith_psi_cgroup_lock, holds a refcount on the
 *     cached cgroup (via cgroup_get_from_path), drops the previous
 *     refcount with cgroup_put() after a synchronize_rcu() so the
 *     readers below have left their grace period.
 *   - zenith_psi_cgroup_active_path is the most-recently-applied path
 *     string.  Identical-path stores are no-ops, so repeat profile-
 *     bake or sysfs writes don't churn the cgroup ref.
 *   - The cgroup ref lives as long as the path is set; zenith is
 *     built-in (Kconfig: bool), so we never run a teardown path.
 *
 * Hot-path readers do a single rcu_read_lock() / rcu_dereference() /
 * READ_ONCE() / rcu_read_unlock(); RCU read-side critical sections
 * are essentially free under PREEMPT_RCU (no atomic, no memory
 * barrier on the load side), so this stays cheap when the feature
 * is on, and is identical to the pre-B10 read when the cached
 * pointer is NULL.
 */
#if defined(CONFIG_PSI) && defined(CONFIG_CGROUPS)
static struct cgroup __rcu *zenith_psi_cgroup;
static DEFINE_MUTEX(zenith_psi_cgroup_lock);
static char zenith_psi_cgroup_active_path[ZENITH_PSI_CGROUP_PATH_MAX];
#endif

/* Replace the cached zenith_psi_cgroup pointer to match @path.
 *
 *   path == ""    -> drop to NULL (system-wide PSI), the safe default.
 *   path == X     -> resolve via cgroup_get_from_path(); on success
 *                    swap the cached pointer and drop the old refcount;
 *                    on failure (not mounted, missing, cgroup-v1-only)
 *                    leave the cache as-is and clear active_path so a
 *                    later identical-path store will retry.
 *
 * Caller is the sysfs store path or zenith_apply_profile().  Both run
 * outside any hot path.  No-op on identical path matches and on
 * !CONFIG_CGROUPS / !CONFIG_PSI builds.
 */
void zenith_psi_cgroup_apply(const char *path)
{
#if defined(CONFIG_PSI) && defined(CONFIG_CGROUPS)
	struct cgroup *new_cgrp = NULL;
	struct cgroup *old_cgrp;

	if (!path)
		path = "";

	mutex_lock(&zenith_psi_cgroup_lock);

	if (!strncmp(zenith_psi_cgroup_active_path, path,
		     ZENITH_PSI_CGROUP_PATH_MAX))
		goto out_unlock;

	if (path[0]) {
		new_cgrp = cgroup_get_from_path(path);
		if (IS_ERR(new_cgrp)) {
			new_cgrp = NULL;
			zenith_psi_cgroup_active_path[0] = '\0';
			goto out_unlock;
		}
	}

	old_cgrp = rcu_dereference_protected(zenith_psi_cgroup,
			lockdep_is_held(&zenith_psi_cgroup_lock));
	rcu_assign_pointer(zenith_psi_cgroup, new_cgrp);
	strscpy(zenith_psi_cgroup_active_path, path,
		sizeof(zenith_psi_cgroup_active_path));

	if (old_cgrp) {
		synchronize_rcu();
		cgroup_put(old_cgrp);
	}

out_unlock:
	mutex_unlock(&zenith_psi_cgroup_lock);
#endif
}

/* Pick the psi_group the zenith_psi_*_some_pct() helpers should read
 * from for the current call.  rcu_read_lock() must be held by the
 * caller; the returned pointer is only valid for the duration of
 * that read-side critical section.
 *
 * Falls back to &psi_system if the cgroup-v2 cached pointer is NULL
 * (or if CONFIG_CGROUPS=n), which is the pre-B10 behaviour and the
 * cold-boot default.
 */
#ifdef CONFIG_PSI
static __always_inline struct psi_group *zenith_psi_pick_group(void)
{
#ifdef CONFIG_CGROUPS
	struct cgroup *cgrp = rcu_dereference(zenith_psi_cgroup);

	return cgrp ? &cgrp->psi : &psi_system;
#else
	return &psi_system;
#endif
}
#endif

/* Read the memory pressure 10s average from PSI as an integer
 * percentage (0..100).  Returns 0 when CONFIG_PSI is off, when
 * psi_disabled is set, or when the value isn't yet populated (early
 * boot).
 *
 * Source psi_group is selected by zenith_psi_pick_group(): the
 * B10-3 cached cgroup-v2 group when one is configured via
 * psi_cgroup_path, otherwise psi_system (system-wide PSI, the
 * pre-B10 behaviour).  Single READ_ONCE on the EWMA word inside a
 * minimal rcu_read_lock() critical section -- the avgs_work
 * aggregator is what writes to ->avg[][] and we tolerate up to 2 s
 * of staleness on the read.
 */
inline unsigned int zenith_psi_mem_some_pct(void)
{
#ifdef CONFIG_PSI
	struct psi_group *grp;
	unsigned long avg;

	if (static_branch_likely(&psi_disabled))
		return 0;

	rcu_read_lock();
	grp = zenith_psi_pick_group();
	avg = READ_ONCE(grp->avg[PSI_MEM_SOME][0]);
	rcu_read_unlock();
	return (unsigned int)LOAD_INT(avg);
#else
	return 0;
#endif
}

/* Same shape as zenith_psi_mem_some_pct() for the PSI_CPU_SOME and
 * PSI_IO_SOME dimensions.  See the psi_aware / psi_*_thresh comment
 * block for what each pressure source means.  Both helpers honour
 * the same B10-3 cgroup-v2 cached pick, are lock-free outside the
 * RCU read-side, and tolerate CONFIG_PSI=n at compile time.
 */
inline unsigned int zenith_psi_cpu_some_pct(void)
{
#ifdef CONFIG_PSI
	struct psi_group *grp;
	unsigned long avg;

	if (static_branch_likely(&psi_disabled))
		return 0;

	rcu_read_lock();
	grp = zenith_psi_pick_group();
	avg = READ_ONCE(grp->avg[PSI_CPU_SOME][0]);
	rcu_read_unlock();
	return (unsigned int)LOAD_INT(avg);
#else
	return 0;
#endif
}

inline unsigned int zenith_psi_io_some_pct(void)
{
#ifdef CONFIG_PSI
	struct psi_group *grp;
	unsigned long avg;

	if (static_branch_likely(&psi_disabled))
		return 0;

	rcu_read_lock();
	grp = zenith_psi_pick_group();
	avg = READ_ONCE(grp->avg[PSI_IO_SOME][0]);
	rcu_read_unlock();
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
			len += sysfs_emit_at(buf, len,
					 "%s%s", first ? "" : ",",
					 t->entries[i]);
			first = false;
		}
	}
	rcu_read_unlock();
	len += sysfs_emit_at(buf, len, "\n");
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
 *
 * Coverage rationale (B-AUTO-1 expansion -- entries are vendor-
 * comprehensive so the zenith auto-profile selector cannot miss a
 * render-thread context, and so manual render_aware mode catches
 * non-AOSP graphics stacks too):
 *   - RenderThread         AOSP libui per-app render thread ("RenderThread N")
 *   - surfaceflinger       SurfaceFlinger main thread
 *   - RenderEngine         SurfaceFlinger render engine worker
 *   - mali-cmar-back       ARM Mali Bifrost / Valhall command-stream backend
 *   - GLThread             SurfaceView Java GL thread ("GLThread N"; cocos2d, libgdx, ...)
 *   - kgsl_worker_th       Qualcomm Adreno KGSL worker (truncated from kgsl_worker_thread)
 *   - kgsl-3d0             Qualcomm Adreno KGSL device thread
 *   - kbase_event          ARM Mali Bifrost / Valhall event-completion thread
 *   - composer-servic      HWC2 composer HAL service (truncated from composer-service)
 *   - vsync_thread         generic vsync producer thread
 *   - Choreographer        Android frame Choreographer thread
 *   - UnrealRenderTh       Unreal Engine render thread (truncated)
 *   - Cocos2d-Render       Cocos2d-x render thread (truncated)
 *   - CompositorThr        Chromium / WebView compositor thread (truncated)
 *
 * All entries are <= 15 chars to fit within task->comm[16] (the
 * trailing NUL leaves 15 usable bytes); strncmp() compares only
 * strlen(needle) bytes, so adding extra entries costs at most one
 * cache-miss-bounded strncmp loop iteration per CPU.  The hot path
 * is gated by ZENITH_RENDER_CACHE_TTL_NS so even a 14-entry walk
 * runs at most a few times per second per policy.
 */
static const char * const zenith_render_comms[] = {
	"RenderThread",
	"surfaceflinger",
	"RenderEngine",
	"mali-cmar-back",
	"GLThread",
	"kgsl_worker_th",
	"kgsl-3d0",
	"kbase_event",
	"composer-servic",
	"vsync_thread",
	"Choreographer",
	"UnrealRenderTh",
	"Cocos2d-Render",
	"CompositorThr",
};

/* Walk the policy's online cpumask and check each cpu_curr's comm
 * against zenith_render_comms[].  Returns true on the first match.
 * The result is cached for ZENITH_RENDER_CACHE_TTL_NS so a hot path
 * (e.g. a 60 / 90 / 120 Hz scroll) only does the strncmp loop a few
 * times per second.  Caller is expected to gate the call on
 * tunables->render_aware != 0; this helper does not re-check that.
 */
bool zenith_policy_has_render(struct zenith_policy *z_policy)
{
	u64 now = ktime_get_ns();
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned int cpu;
	bool match = false;
	unsigned int matched_util = 0;

	if (z_policy->render_cache_stamp_ns &&
	    now - z_policy->render_cache_stamp_ns < ZENITH_RENDER_CACHE_TTL_NS)
		return z_policy->render_active;

	rcu_read_lock();
	{
		struct zenith_comm_table *t =
			rcu_dereference(zenith_render_table);

		for_each_cpu(cpu, policy->cpus) {
			struct task_struct *curr = rcu_dereference(cpu_curr(cpu));
			unsigned int i;

			if (!curr || !t)
				continue;
			for (i = 0; i < t->nr; i++) {
				const char *needle = t->entries[i];

				if (!strncmp(curr->comm, needle,
					     strlen(needle))) {
					match = true;
					/* Wave A render-thread util
					 * tracker: capture the matched
					 * task's PELT util_avg in 1/1024
					 * units.  Single READ_ONCE() so
					 * the matched_util field is
					 * always fresh when render_-
					 * active is true.
					 */
					matched_util = (unsigned int)
						READ_ONCE(curr->se.avg.util_avg);
					break;
				}
			}
			if (match)
				break;
		}
	}
	rcu_read_unlock();

	z_policy->render_active = match;
	z_policy->render_matched_util_avg = matched_util;
	z_policy->render_cache_stamp_ns = now;
	return match;
}

/* Wave A cgroup-aware top-app helper.  Reads the cpuset cgroup the
 * task currently belongs to and checks whether the leaf directory
 * name is "top-app".  Caller must NOT hold rcu_read_lock; this
 * helper takes it internally (RCU is recursive, so calling from a
 * context that already holds the lock is also fine).
 *
 * Gated on CONFIG_CPUSETS because cpuset_cgrp_id is only defined
 * when the cpuset subsystem is built; on !CONFIG_CPUSETS the
 * helper compiles to a constant false and the floor never
 * applies.
 */
#if IS_ENABLED(CONFIG_CPUSETS)
static bool zenith_task_in_top_app(struct task_struct *t)
{
	struct cgroup_subsys_state *css;
	bool match = false;

	rcu_read_lock();
	css = task_css(t, cpuset_cgrp_id);
	if (css && css->cgroup && css->cgroup->kn) {
		const char *name = css->cgroup->kn->name;

		if (name && !strcmp(name, ZENITH_TOP_APP_CGROUP_NAME))
			match = true;
	}
	rcu_read_unlock();
	return match;
}
#else
static inline bool zenith_task_in_top_app(struct task_struct *t)
{
	return false;
}
#endif

/* Walk the policy's online cpumask and check each cpu_curr's cpuset
 * cgroup membership.  Returns true on the first match against the
 * "top-app" cgroup.  Result is cached for ZENITH_TOP_APP_CACHE_TTL_NS
 * (4 ms) so a hot path (e.g. a 60 / 90 / 120 Hz scroll) only does
 * the cgroup walk a few times per second per policy.  Caller is
 * expected to gate the call on tunables->top_app_aware != 0; this
 * helper does not re-check that.
 */
bool zenith_policy_has_top_app(struct zenith_policy *z_policy)
{
	u64 now = ktime_get_ns();
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned int cpu;
	bool match = false;

	if (z_policy->top_app_cache_stamp_ns &&
	    now - z_policy->top_app_cache_stamp_ns < ZENITH_TOP_APP_CACHE_TTL_NS)
		return z_policy->top_app_active;

	rcu_read_lock();
	for_each_cpu(cpu, policy->cpus) {
		struct task_struct *curr = rcu_dereference(cpu_curr(cpu));

		if (!curr)
			continue;
		if (zenith_task_in_top_app(curr)) {
			match = true;
			break;
		}
	}
	rcu_read_unlock();

	z_policy->top_app_active = match;
	z_policy->top_app_cache_stamp_ns = now;
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
 * B-AUTO-1 expansion (additions, all <= 15 chars to fit comm[16]):
 *   - fast_mixer       AudioFlinger FastMixer thread (low-latency path)
 *   - TrackBase        AudioFlinger TrackBase worker family
 *   - AudioTrack       libaudioclient JNI AudioTrack thread
 *   - AudioRecord      libaudioclient JNI AudioRecord thread
 *   - audioPolicySrv   AudioPolicyService main thread
 *   - audio.cb.thread  vendor audio callback thread (qcom / mtk family)
 *   - audio_track_thr  vendor track-driver thread (truncated)
 *   - vendor.mtk.audi  MediaTek vendor audio HAL (truncated)
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
	"fast_mixer",
	"TrackBase",
	"AudioTrack",
	"AudioRecord",
	"audioPolicySrv",
	"audio.cb.thread",
	"audio_track_thr",
	"vendor.mtk.audi",
};

/* Walk the policy's online cpumask and check each cpu_curr's comm
 * against zenith_audio_comms[].  Returns true on the first match.
 * Cached for ZENITH_AUDIO_CACHE_TTL_NS so the strncmp loop runs
 * once every few milliseconds at most.  Caller is expected to gate
 * the call on tunables->audio_aware != 0; this helper does not
 * re-check that.
 */
bool zenith_policy_has_audio(struct zenith_policy *z_policy)
{
	u64 now = ktime_get_ns();
	struct cpufreq_policy *policy = z_policy->policy;
	struct zenith_tunables *t_hyst = z_policy->tunables;
	unsigned int hyst_ms = t_hyst ? t_hyst->audio_hyst_ms : 0;
	unsigned int cpu;
	bool match = false;

	/* Audit fix K5: deterministic short-circuit on any open ALSA
	 * pcm fd.  Same pattern as K4: bypass the comm-walk and the
	 * cache TTL because the refcount is event-driven and always
	 * fresh.
	 */
	if (atomic_read(&zenith_alsa_active_fds) > 0) {
		z_policy->audio_active = true;
		z_policy->audio_cache_stamp_ns = now;
		if (hyst_ms)
			WRITE_ONCE(z_policy->audio_sticky_until_ns,
				   now + (u64)hyst_ms * NSEC_PER_MSEC);
		return true;
	}

	/* Patch B7-1: sticky audio-active window.  After a fresh
	 * positive detection the helper continues to report true for
	 * audio_hyst_ms past the last hit.  Bypasses the cache TTL
	 * (the sticky window is the strictly-longer guard).
	 */
	if (hyst_ms) {
		u64 until = READ_ONCE(z_policy->audio_sticky_until_ns);

		if (until && now < until)
			return true;
	}

	if (z_policy->audio_cache_stamp_ns &&
	    now - z_policy->audio_cache_stamp_ns < ZENITH_AUDIO_CACHE_TTL_NS)
		return z_policy->audio_active;

	rcu_read_lock();
	{
		struct zenith_comm_table *t =
			rcu_dereference(zenith_audio_table);

		for_each_cpu(cpu, policy->cpus) {
			struct task_struct *curr = rcu_dereference(cpu_curr(cpu));
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
	if (match && hyst_ms)
		WRITE_ONCE(z_policy->audio_sticky_until_ns,
			   now + (u64)hyst_ms * NSEC_PER_MSEC);
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
 * B-AUTO-1 expansion (additions, all <= 15 chars):
 *   - vendor.mtk.came  MediaTek camera HAL service (truncated)
 *   - vendor.google.c  Pixel / Tensor vendor.google.camera (truncated)
 *   - vendor.qti.imag  Qualcomm image processor service (truncated)
 *   - C2DColorConver   Qualcomm camera color-space conversion thread
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
	"vendor.mtk.came",
	"vendor.google.c",
	"vendor.qti.imag",
	"C2DColorConver",
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

	/* Audit fix K4: deterministic short-circuit.  Any open v4l2 fd
	 * means camera (or webcam, or v4l2 codec node, or screen
	 * recorder sink) is active; treat as match without doing the
	 * runqueue walk at all.  Bypasses the comm-walk cache too --
	 * the v4l2 hook updates atomically on every open / release so
	 * the value is always fresh, no TTL needed.
	 */
	if (atomic_read(&zenith_v4l2_active_fds) > 0) {
		z_policy->camera_auto_match = true;
		z_policy->camera_cache_stamp_ns = now;
		return true;
	}

	if (z_policy->camera_cache_stamp_ns &&
	    now - z_policy->camera_cache_stamp_ns < ZENITH_CAMERA_CACHE_TTL_NS)
		return z_policy->camera_auto_match;

	rcu_read_lock();
	for_each_cpu(cpu, policy->cpus) {
		struct task_struct *curr = rcu_dereference(cpu_curr(cpu));
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
 *
 * B-AUTO-1 expansion (additions, all <= 15 chars, vendor-comprehensive
 * so the auto-profile selector cannot miss a game-engine context):
 *
 *   - TaskGraphThr       Unreal Engine task-graph worker (truncated;
 *                        UE4/UE5 spawn TaskGraphThread N for parallel
 *                        engine tasks)
 *   - RHIThread          Unreal Engine Render Hardware Interface thread
 *                        (UE4/UE5; bridges renderer to GPU API backends)
 *   - Cocos2dxRender     Cocos2d-x render thread (truncated; engine name
 *                        used by many phone games shipped via cocos2d-x)
 *   - Job.Worker         Unity Burst job system worker thread
 *                        (DOTS / ECS / parallel-for jobs)
 *   - EnlightenWork      Unity Enlighten realtime-GI worker
 *
 * Patch M expansion (post-audit additions; all distinct from existing
 * Android system thread names so the prefix walk does not collide):
 *
 *   - RenderingThread    Unreal Engine alternate render thread name
 *                        (15 chars, exactly fits TASK_COMM_LEN-1).
 *                        Distinct from Android HWUI's "RenderThread"
 *                        (no "ing") -- the prefix walk uses strncmp
 *                        with strlen(needle) so the system thread
 *                        does not match this longer prefix.
 *   - Roblox             Roblox engine prefix.  Roblox spawns
 *                        threads named "RobloxAppMain", "RobloxRender",
 *                        "RobloxNetwork", etc., all of which prefix
 *                        with "Roblox".  One of the most popular
 *                        mobile titles globally; not covered by any
 *                        Unity / Unreal / Cocos2d engine matcher.
 *   - miHoYoSDK          miHoYo / HoYoverse common SDK thread.  Live
 *                        for Genshin Impact, Honkai: Star Rail,
 *                        Zenless Zone Zero, and Honkai Impact 3rd.
 *                        Belt-and-suspenders coverage on top of the
 *                        UnityMain match -- the SDK thread persists
 *                        through scenes where UnityMain is briefly
 *                        scheduled out (login flows, IAP, scene
 *                        transitions).
 */
static const char * const zenith_game_auto_comms[] = {
	"UnityMain",
	"UnityGfxDeviceW",
	"il2cpp",
	"GameThread",
	"TaskGraphThr",
	"RHIThread",
	"Cocos2dxRender",
	"Job.Worker",
	"EnlightenWork",
	"RenderingThread",
	"Roblox",
	"miHoYoSDK",
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
bool zenith_policy_has_game_auto(struct zenith_policy *z_policy)
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
			struct task_struct *curr = rcu_dereference(cpu_curr(cpu));
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
void zenith_policy_game_auto_tick(struct zenith_policy *z_policy)
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

/*
 * Kasumi (drivers/thermal/thermal_helpers.c) intercepts
 * thermal_zone_get_temp() and dampens the reported temperature.
 * For Zenith's game_perf_burst guardrail we want the *real* (un-
 * dampened) value so the FSM can't be fooled by a configured
 * Kasumi offset; the dampened value still goes to the framework /
 * userspace as before.  Forward-declared here (no public header
 * for Kasumi) -- defined and EXPORT_SYMBOL_GPL'd by Kasumi.
 *
 * Returns the last raw value seen by kasumi_dampen(), in millideg C.
 * Returns 0 if Kasumi has never dampened a reading (e.g. Kasumi
 * disabled, or no thermal zone the filter accepted has been read
 * yet) -- caller treats 0 as "fall back to whatever I have".
 */
#if IS_ENABLED(CONFIG_THERMAL)
extern int kasumi_get_last_real_mc(void);
extern void kasumi_apply_profile(unsigned int profile);
#else
static inline int kasumi_get_last_real_mc(void) { return 0; }
static inline void kasumi_apply_profile(unsigned int profile) { }
#endif

#if IS_ENABLED(CONFIG_IYASHI)
extern void iyashi_apply_profile(unsigned int profile);
#else
static inline void iyashi_apply_profile(unsigned int profile) { }
#endif

/* Patch K: live skin-temp readout for the game_perf_burst guardrail.
 * Returns millidegrees C.
 *
 * Primary path: thermal_zone_get_temp() against the per-policy zone
 * resolved at zenith_start() time.  This is the kernel thermal
 * subsystem's authoritative reading -- same number userspace would
 * see at /sys/class/thermal/thermal_zone<N>/temp.
 *
 * Patch K v2 (Zenith x Kasumi awareness): after a successful
 * thermal_zone_get_temp() we ask Kasumi for the last *real* (un-
 * dampened) reading and prefer it over the framework value.  This
 * lets Zenith make burst-guardrail decisions on the truth even
 * when Kasumi is veiling heat from the framework.  Tiny race
 * window: between our get_temp() returning and our
 * kasumi_get_last_real_mc() call, another CPU could re-dampen a
 * different zone and overwrite kasumi_last_real_mc.  The window is
 * sub-millisecond and the guardrail is approximate (FSM thresholds
 * are degrees apart), so this is acceptable.
 *
 * Fallback: if the zone is unresolved (NULL / IS_ERR -- foreign SoC,
 * thermal subsystem not registered yet, etc.) or the read returns
 * an error, synthesize an approximate dC value from
 * arch_scale_thermal_pressure().  The pressure is a 0..1024 capacity-
 * reduction scalar; mapping 0..100% pressure linearly to 30..70 dC
 * gives the burst guardrail a reasonable best-effort estimate even
 * on platforms where the thermal subsystem hasn't published a
 * per-cluster zone.
 *
 * Both paths are cheap (single load + one library call) so the
 * helper is safe to call once per zenith_get_next_freq().  No
 * caching layer here; the thermal subsystem already caches its own
 * sensor reads (driver dependent), and the fallback path is a
 * single arch_scale_thermal_pressure() read.
 */
static int zenith_gpb_get_temp_dc(struct zenith_policy *z_policy)
{
	struct thermal_zone_device *tzd = z_policy->gpb_tzd;
	unsigned int pct;
	int temp = 0;
	int real;

	/* Patch M: lazy retry if zenith_start() couldn't bind the
	 * per-cluster thermal zone (boot ordering: thermal-core may
	 * register after the governor on some Tensor builds).  Rate
	 * limited to one re-resolution per
	 * ZENITH_GPB_TZD_RETRY_INTERVAL_NS so a permanently-foreign SoC
	 * (no per-cpu thermal zones at all) costs only one
	 * thermal_zone_get_zone_by_name() per N seconds in the hot path.
	 */
	if (!tzd) {
		u64 now = ktime_get_ns();

		if (now >= z_policy->gpb_tzd_retry_at_ns) {
			char zone_name[16];
			unsigned int first_cpu =
				cpumask_first(z_policy->policy->cpus);

			scnprintf(zone_name, sizeof(zone_name),
				  "cpu%u-thermal", first_cpu);
			tzd = thermal_zone_get_zone_by_name(zone_name);
			if (IS_ERR(tzd))
				tzd = NULL;
			z_policy->gpb_tzd = tzd;
			z_policy->gpb_tzd_retry_at_ns =
				now + ZENITH_GPB_TZD_RETRY_INTERVAL_NS;
		}
	}

	if (tzd && !IS_ERR(tzd) && !thermal_zone_get_temp(tzd, &temp)) {
		real = kasumi_get_last_real_mc();
		return real > 0 ? real : temp;
	}

	pct = zenith_policy_thermal_pressure_pct(z_policy);
	if (pct > 100)
		pct = 100;
	return 30000 + (int)(pct * 400);
}

/* Patch K: game_perf_burst FSM evaluator.  Called once per
 * zenith_get_next_freq() invocation, gated by the
 * zenith_game_perf_burst_key static branch and the live tunables
 * scalar (defends against momentary tear during sysfs store).
 *
 * Reads:
 *   - zenith_eff_game_mode()                 Signal A
 *   - z_policy->last_load_pct (previous tick) Signal B
 *   - z_policy->audio_sticky_until_ns         Signal C suppressor
 *
 * Writes:
 *   - z_policy->gpb_state                    FSM state
 *   - z_policy->gpb_state_entry_ns           transition timestamp
 *   - z_policy->gpb_b_arm_first_seen_ns      Signal-B continuous-on
 *   - z_policy->gpb_b_disarm_first_seen_ns   Signal-B continuous-off
 *
 * No locking required: per-policy hot path is serialised by the
 * cpufreq core (single writer for this set of fields).
 *
 * Note: zenith_eff_game_mode() peeks at zenith_game_auto_active_until_ns
 * which is updated by zenith_policy_game_auto_tick() above; the
 * caller invokes the auto_tick first so a fresh tick's match has
 * already been latched when the FSM evaluator runs.
 */
void zenith_gpb_evaluate(struct zenith_policy *z_policy)
{
	struct zenith_tunables *t = z_policy->tunables;
	u64 now_ns = ktime_get_ns();
	bool sig_a, sig_b, sig_c_suppress;
	unsigned int load_pct;
	unsigned int disarm_grace_ms;
	unsigned int base_gm;

	/* Signal A: zenith_eff_game_mode().  Pass the manual game_mode
	 * scalar as the base; the helper bumps it to 1 when game_auto
	 * has latched a sustained known-game comm-walk match.  Read via
	 * READ_ONCE so a momentary tear during a sysfs game_mode store
	 * does not produce a transient false negative on this tick.
	 */
	base_gm = READ_ONCE(t->game_mode);
	sig_a = (zenith_eff_game_mode(base_gm) != 0);

	load_pct = READ_ONCE(z_policy->last_load_pct);

	/* Signal C suppressor: an active audio sticky window with NO
	 * game signal arriving first means "the user is watching
	 * something, not playing something".  When sig_a is already
	 * true (game_auto latched or manual game_mode write), the
	 * suppressor is moot -- a user can play a game while music
	 * plays, and we do not want to deny them the burst.
	 */
	sig_c_suppress = !sig_a &&
		(READ_ONCE(z_policy->audio_sticky_until_ns) > now_ns);

	/* Patch M: Schmitt-trigger Signal B.  The non-zero
	 * gpb_b_arm_first_seen_ns stamp doubles as the latch state.
	 *
	 *   off (stamp == 0):  arm only when load >= 70 (enter)
	 *   on  (stamp != 0):  release only when load < 60 (exit)
	 *
	 * This holds sig_b across dips inside the 60..70 hysteresis
	 * band so a scene oscillating in that range keeps accumulating
	 * sustained-arming time instead of resetting on every dip.
	 * The 2s sustained gate enforced at the IDLE -> ARMED edge
	 * below uses the same first-seen stamp, so once we cross 70
	 * and stay above 60, the timer keeps ticking.
	 */
	if (z_policy->gpb_b_arm_first_seen_ns) {
		if (load_pct < ZENITH_GAME_PERF_BURST_B_EXIT_THRESHOLD_PCT) {
			z_policy->gpb_b_arm_first_seen_ns = 0;
			sig_b = false;
		} else {
			sig_b = true;
		}
	} else {
		if (load_pct >= ZENITH_GAME_PERF_BURST_B_THRESHOLD_PCT) {
			z_policy->gpb_b_arm_first_seen_ns = now_ns;
			sig_b = true;
		} else {
			sig_b = false;
		}
	}

	/* Signal-B continuous-off tracking (only meaningful while
	 * ARMED).  Stamp first-seen on the ARMED-side falling edge,
	 * clear on any tick that re-observes B.
	 */
	if (z_policy->gpb_state == ZENITH_GPB_STATE_ARMED) {
		if (!sig_b) {
			if (z_policy->gpb_b_disarm_first_seen_ns == 0)
				z_policy->gpb_b_disarm_first_seen_ns = now_ns;
		} else {
			z_policy->gpb_b_disarm_first_seen_ns = 0;
		}
	} else {
		z_policy->gpb_b_disarm_first_seen_ns = 0;
	}

	disarm_grace_ms = READ_ONCE(t->game_perf_burst_disarm_grace_ms);

	switch (z_policy->gpb_state) {
	case ZENITH_GPB_STATE_IDLE:
		/* IDLE -> ARMED: A AND (B sustained for >= 2s) AND
		 * (NOT C suppressing).
		 */
		if (sig_a && !sig_c_suppress &&
		    z_policy->gpb_b_arm_first_seen_ns &&
		    (now_ns - z_policy->gpb_b_arm_first_seen_ns) >=
		    ZENITH_GAME_PERF_BURST_B_REQUIRED_NS) {
			z_policy->gpb_state = ZENITH_GPB_STATE_ARMED;
			z_policy->gpb_state_entry_ns = now_ns;
			z_policy->gpb_b_disarm_first_seen_ns = 0;
			z_policy->gpb_arm_count++;
		}
		break;
	case ZENITH_GPB_STATE_ARMED:
		/* Fast disarm: !A => COOLDOWN immediately.  This catches
		 * the user Alt+Tabbing / pressing Home -- game_auto's
		 * ACTIVE_TTL has expired or game_mode was written 0.
		 */
		if (!sig_a) {
			z_policy->gpb_state = ZENITH_GPB_STATE_COOLDOWN;
			z_policy->gpb_state_entry_ns = now_ns;
			z_policy->gpb_disarm_count++;
			z_policy->gpb_last_disarm_reason =
				ZENITH_GPB_DISARM_FAST;
			break;
		}
		/* Sustained-clear disarm: B has been false continuously
		 * for >= disarm_grace_ms.  Tail of a level / loading
		 * screen.  Glide back through COOLDOWN.
		 */
		if (z_policy->gpb_b_disarm_first_seen_ns &&
		    (now_ns - z_policy->gpb_b_disarm_first_seen_ns) >=
		    ((u64)disarm_grace_ms * NSEC_PER_MSEC)) {
			z_policy->gpb_state = ZENITH_GPB_STATE_COOLDOWN;
			z_policy->gpb_state_entry_ns = now_ns;
			z_policy->gpb_disarm_count++;
			z_policy->gpb_last_disarm_reason =
				ZENITH_GPB_DISARM_SUSTAINED;
		}
		break;
	case ZENITH_GPB_STATE_COOLDOWN:
		/* COOLDOWN -> ARMED on a fresh re-arm: user came back
		 * within the cooldown window.  Skip the 2s sustained
		 * gate this once (we were just here) so the floor
		 * re-engages without a second multi-second confidence
		 * build-up.
		 */
		if (sig_a && sig_b && !sig_c_suppress) {
			z_policy->gpb_state = ZENITH_GPB_STATE_ARMED;
			z_policy->gpb_state_entry_ns = now_ns;
			z_policy->gpb_b_disarm_first_seen_ns = 0;
			z_policy->gpb_arm_count++;
			break;
		}
		/* COOLDOWN -> IDLE when the glide window expires. */
		{
			unsigned int cooldown_ms =
				READ_ONCE(t->game_perf_burst_cooldown_ms);
			u64 elapsed = now_ns - z_policy->gpb_state_entry_ns;

			if (elapsed >= ((u64)cooldown_ms * NSEC_PER_MSEC)) {
				z_policy->gpb_state = ZENITH_GPB_STATE_IDLE;
				z_policy->gpb_state_entry_ns = now_ns;
				z_policy->gpb_idle_count++;
			}
		}
		break;
	default:
		/* Defensive: any unknown state -> IDLE. */
		z_policy->gpb_state = ZENITH_GPB_STATE_IDLE;
		z_policy->gpb_state_entry_ns = now_ns;
		break;
	}
}

/* Patch K: compute the per-tick freq floor contribution from the
 * game_perf_burst FSM.  Returns 0 when the FSM is IDLE, the master
 * is off, or the thermal guardrail is engaged this tick.  Otherwise
 * returns the floor freq in policy units (Hz).
 *
 * ARMED state: returns policy->max * floor_pct / 100.
 *
 * COOLDOWN state: linearly steps the floor from the ARMED level
 * down to 0 across cooldown_ms.  At t=0  ms post-disarm the floor
 * still equals the ARMED floor; at t=cooldown_ms it has reached 0.
 * This avoids freq whiplash when the user Alt+Tabs out.
 *
 * Thermal guardrail: when the live skin temp >= ceiling_dc, return
 * 0 -- the existing auto_thermal_cap / thermal_util_derate path
 * remains in charge.  The FSM stays in ARMED so the floor will
 * re-engage automatically once the temp drops back below ceiling.
 */
static unsigned int zenith_gpb_floor(struct zenith_policy *z_policy,
				     unsigned int policy_max)
{
	struct zenith_tunables *t = z_policy->tunables;
	unsigned int floor_pct;
	unsigned int floor;
	int temp_dc;
	int ceiling_dc;
	u8 state;

	state = z_policy->gpb_state;
	if (state == ZENITH_GPB_STATE_IDLE)
		return 0;

	floor_pct = READ_ONCE(t->game_perf_burst_floor_pct);
	if (!floor_pct || !policy_max)
		return 0;

	temp_dc = zenith_gpb_get_temp_dc(z_policy);
	ceiling_dc = (int)READ_ONCE(t->game_perf_burst_thermal_ceiling_dc);
	if (temp_dc >= ceiling_dc)
		return 0;

	floor = (policy_max / 100) * floor_pct;
	if (floor > policy_max)
		floor = policy_max;

	if (state == ZENITH_GPB_STATE_COOLDOWN) {
		unsigned int cooldown_ms =
			READ_ONCE(t->game_perf_burst_cooldown_ms);
		u64 cooldown_ns = (u64)cooldown_ms * NSEC_PER_MSEC;
		u64 now_ns = ktime_get_ns();
		u64 elapsed = now_ns - z_policy->gpb_state_entry_ns;

		if (!cooldown_ns || elapsed >= cooldown_ns)
			return 0;
		/* Linear ramp: floor *= (cooldown_ns - elapsed) / cooldown_ns
		 * Computed in u64 to avoid wrap on policy_max * remaining.
		 */
		{
			u64 remaining = cooldown_ns - elapsed;
			u64 scaled = ((u64)floor * remaining) / cooldown_ns;

			floor = (unsigned int)scaled;
		}
	}
	return floor;
}

/* Patch K: stringify the FSM state for sysfs read-back.  Stable
 * tokens so userspace tooling can grep / parse the state node.
 */
const char *zenith_gpb_state_name(u8 state)
{
	switch (state) {
	case ZENITH_GPB_STATE_IDLE:
		return "idle";
	case ZENITH_GPB_STATE_ARMED:
		return "ARMED";
	case ZENITH_GPB_STATE_COOLDOWN:
		return "COOLDOWN";
	default:
		return "?";
	}
}

/* Patch M: stringify the last-disarm reason for the stats sysfs.
 * Stable tokens so userspace tooling can grep / parse the field.
 */
const char *zenith_gpb_disarm_name(u8 reason)
{
	switch (reason) {
	case ZENITH_GPB_DISARM_NONE:
		return "none";
	case ZENITH_GPB_DISARM_FAST:
		return "fast";
	case ZENITH_GPB_DISARM_SUSTAINED:
		return "sustained";
	default:
		return "?";
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
	if (!strcmp(path, "auto_thermal_cap"))
		return ZENITH_STAT_AUTO_THERMAL_CAP;
	if (!strcmp(path, "quiet_hours_cap"))
		return ZENITH_STAT_QUIET_HOURS_CAP;
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

/* Reset the V1 sample counters and the V2 pending-window
 * accumulator to a fresh-window starting point.  Used by the
 * three call sites that all need the same fresh-classifier
 * substrate:
 *
 *   - the screen 0 -> 1 (resume) edge in zenith_get_next_freq()
 *     (audit fix M7 / M7b),
 *   - auto_tune_store() on the disable -> enable transition,
 *   - zenith_start() when auto_tune is enabled at policy bring-up.
 *
 * Leaves at_last_state / at_last_applied_state / at_cooldown_left
 * untouched so the just-recorded classification and any pending
 * post-transition cooldown survive the reset.  Callers that need
 * a complete classifier reset (boot, sysfs disable->enable) clear
 * at_cooldown_left themselves and reschedule at_work.
 */
static inline void zenith_at_v_reset_window(struct zenith_policy *z_policy)
{
	atomic_set(&z_policy->at_samples_total, 0);
	atomic_set(&z_policy->at_samples_saturated, 0);
	z_policy->at_last_events =
		atomic64_read(&zenith_auto_input_events);
	z_policy->at_pending_windows = 0;
}

/* Patch 1.10: return true when the wall clock is currently inside
 * the quiet-hours window described by [start_min, end_min) on a
 * 0..1439 minute-of-day grid (UTC).  Returns false when the
 * window is zero-length (start == end), which is the configured-
 * disabled state.  When start > end the window wraps midnight,
 * matching how a user would expect to write "22:00 to 06:00".
 *
 * Uses ktime_get_real_seconds() + time64_to_tm() to derive the
 * minute-of-day; both are read-only with no allocation, so the
 * helper is safe to call from the hot eval path.
 */
static inline bool zenith_in_quiet_hours(const struct zenith_tunables *t)
{
	unsigned int start = t->quiet_hours_start_min;
	unsigned int end = t->quiet_hours_end_min;
	struct tm tm;
	unsigned int now_min;

	if (start == end ||
	    start > ZENITH_QUIET_HOURS_MINUTE_MAX ||
	    end > ZENITH_QUIET_HOURS_MINUTE_MAX)
		return false;
	time64_to_tm(ktime_get_real_seconds(), 0, &tm);
	now_min = (unsigned int)tm.tm_hour * 60U +
		  (unsigned int)tm.tm_min;
	if (start < end)
		return now_min >= start && now_min < end;
	return now_min >= start || now_min < end;
}

/* Scale a hold-down millisecond budget by batt_hold_scale_pct when
 * the system is running on battery (Patch 1.2).  Returns @ms
 * unchanged when on AC, when scale_pct is 100, or when scale_pct
 * is 0 (treated as "no scaling configured" so an
 * accidentally-zeroed knob does not silently disable hold).  The
 * comparator (* / 100) keeps the math integer; saturating at
 * UINT_MAX is a non-issue because scale_pct is bounded to 50..300
 * and ms to ZENITH_PEAK_HEADROOM_HOLD_MS_MAX (a few hundred ms).
 */
static inline unsigned int zenith_batt_scaled(unsigned int ms,
					      unsigned int scale_pct)
{
	if (!atomic_read(&zenith_on_battery) || scale_pct == 100 || !scale_pct)
		return ms;
	return (unsigned int)(((u64)ms * scale_pct) / 100U);
}

/*
 * Forward declaration; defined near the bottom of this file
 * alongside the Hikari notifier subscription.  Used from
 * zenith_get_next_freq() below.
 */
static unsigned int zenith_hikari_policy_floor(struct cpufreq_policy *policy);

unsigned int zenith_get_next_freq(struct zenith_policy *z_policy,
					 unsigned long util, unsigned long max_cap)
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
	/* Patch 1.4: eval-entry timestamp for the decision-latency
	 * histogram.  Single ktime_get_ns() at function entry; the
	 * bucketing arithmetic at commit is constant-time so the
	 * total cost of the always-on histogram is one ktime read +
	 * one subtract + one bucket increment per eval.
	 */
	u64 dec_eval_start_ns = ktime_get_ns();
	unsigned int dynamic_up_thresh, dynamic_bias, input_boost_floor;
	unsigned long uclamp_min, uclamp_max;
	bool uclamp_min_meaningful;

	/* Patch 1.3 cluster-wake-pulse arm.  Compute now_ns once at
	 * the top of the eval and use it both for the gap measurement
	 * and the deadline stamp.  Skip the very first eval after
	 * policy bring-up (cluster_wake_last_eval_ns == 0) so a fresh
	 * policy that has never sampled doesn't trip a spurious pulse
	 * just because the field is zero.  cluster_wake_pulse_ms == 0
	 * short-circuits the arm; the floor application below is also
	 * gated by the deadline being non-zero, so the tier is a
	 * compile-time-shaped no-op when the profile disables it.
	 */
	{
		u64 now_arm_ns = dec_eval_start_ns;
		u64 prev = z_policy->cluster_wake_last_eval_ns;
		unsigned int pulse_ms =
			z_policy->tunables->cluster_wake_pulse_ms;
		unsigned int idle_ms =
			z_policy->tunables->cluster_wake_pulse_idle_ms;
		bool deep_idle_seen = false;

		/* Patch B9-3: suppress cluster_wake_pulse arm when the
		 * cluster just emerged from a deep cpuidle residency.
		 * The cwp tier exists to compensate for cold-cache latency
		 * after a brief micro-idle; a >= ZENITH_VH_CPU_IDLE_-
		 * RESIDENCY_LONG_NS idle period means the workload waking
		 * the cluster is fresh, not a continuation of a hot stream,
		 * and the next eval window will measure actual demand
		 * directly.  Forcing a pulse floor here would ramp past
		 * real demand and waste energy.  Reads are READ_ONCE on
		 * both gates; the residency aggregate is stamped lock-free
		 * by the cpu_idle_exit probe.  When vh_cpu_idle_enable is
		 * 0 the residency field never moves off zero so the gate
		 * is a compile-time-shaped no-op for that build.
		 */
		if (READ_ONCE(z_policy->tunables->vh_cpu_idle_enable) &&
		    READ_ONCE(z_policy->vh_cpu_idle_last_residency_ns) >=
			ZENITH_VH_CPU_IDLE_RESIDENCY_LONG_NS)
			deep_idle_seen = true;

		if (pulse_ms && prev && !deep_idle_seen &&
		    now_arm_ns - prev >=
			(u64)idle_ms * NSEC_PER_MSEC) {
			z_policy->cluster_wake_pulse_until_ns =
				now_arm_ns +
				(u64)pulse_ms * NSEC_PER_MSEC;
		}
		z_policy->cluster_wake_last_eval_ns = now_arm_ns;
	}

	/* Dynamic Environment Overrides */
	dynamic_up_thresh = zenith_tunable_or_local(z_policy,
		z_policy->tunables->up_threshold,
		z_policy->at_effective_up_threshold);
	dynamic_bias = z_policy->tunables->powersave_bias;

	/* ADPF / uclamp_min floor.  Sampled once here so every decision
	 * tier below (screen-off override, light-load cap, powersave_bias,
	 * final resolve) sees a consistent view.  zero when CONFIG_UCLAMP_TASK
	 * is off, when no task on the policy has set uclamp_min, or when
	 * the governor-level tunable disables respect entirely.
	 */
	uclamp_min = z_policy->tunables->uclamp_min_respect ?
		zenith_policy_uclamp_min(z_policy) : 0;
	uclamp_min_meaningful = uclamp_min >=
		((SCHED_CAPACITY_SCALE * ZENITH_UCLAMP_MIN_MEANINGFUL_PCT) / 100);

	/* Input-boost decay floor.  Computed in the input-boost block
	 * below when we are in the trailing decay window of an active
	 * boost; applied as a minimum on the final freq just before the
	 * resolve label.  Zero means no floor (full-boost phase, or no
	 * boost at all).
	 */
	input_boost_floor = 0;

	/* ADPF / uclamp_max cap.  Sampled once so every decision tier
	 * below sees a consistent view.  SCHED_CAPACITY_SCALE means
	 * "no cap" -- the helper returns that sentinel when uclamp is
	 * not in use, when no task has set a UCLAMP_MAX, or when the
	 * governor-level respect tunable is off.
	 */
	uclamp_max = z_policy->tunables->uclamp_max_respect ?
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
	 * latter is allowed to declare additional locals without
	 * tripping -Wdeclaration-after-statement.
	 */
	if (static_branch_likely(&zenith_game_auto_key) &&
	    READ_ONCE(z_policy->tunables->game_auto))
		zenith_policy_game_auto_tick(z_policy);

	/* Patch K: game_perf_burst FSM tick.  Must run AFTER the
	 * game_auto tick above so a fresh streak-driven latch on
	 * zenith_game_auto_active_until_ns is visible to Signal A
	 * (zenith_eff_game_mode()) on the same tick that detected it.
	 * Gated by the game_perf_burst static branch + live tunables
	 * scalar; both off => zero hot-path cost.  The FSM only
	 * reads load (Signal B) from z_policy->last_load_pct stamped
	 * by the previous tick, so the call site here -- before the
	 * current tick computes its load -- is correct.
	 */
	if (static_branch_likely(&zenith_game_perf_burst_key) &&
	    READ_ONCE(z_policy->tunables->game_perf_burst))
		zenith_gpb_evaluate(z_policy);

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
		 * Audit fix M7b extends the same reset edge to clear
		 * the per-policy streak / hysteresis state that would
		 * otherwise carry stale pre-suspend continuity into the
		 * first post-resume sample window: peak_starve_count
		 * (entry hysteresis for peak-headroom rescue),
		 * hispeed_entry_count and hispeed_active (entry / sticky
		 * state for the hispeed tier), brutal_entry_count and
		 * brutal_active (the up_threshold streak hysteresis).
		 * The deadline atomics in this family
		 * (input_boost_until_ns, peer_ramp_until_ns_*,
		 * frame_overrun_until_ns, peak_rescue_until_ns) already
		 * self-disarm: their absolute ktime_get_ns() deadlines
		 * are in the past after any non-trivial suspend, so a
		 * subsequent atomic64_read() naturally evaluates the
		 * boost as expired without an explicit clear.
		 *
		 * Cheap: 4 atomic_sets and 1 unsigned-int store on the
		 * V1/V2 path, plus 5 plain stores for the streak/
		 * hysteresis state, only on a transition (not every
		 * tick).  No effect on the 1 -> 0 (suspend) edge --
		 * those samples are still useful for the screen-off
		 * glide path.
		 */
		if (!z_policy->screen_state_last && cur_screen) {
			zenith_at_v_reset_window(z_policy);
			z_policy->peak_starve_count = 0;
			z_policy->hispeed_entry_count = 0;
			z_policy->brutal_entry_count = 0;
			z_policy->hispeed_active = false;
			z_policy->brutal_active = false;
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

	{
		unsigned long _vh_next_freq = 0;
		trace_android_vh_map_util_freq(util, freq, max_cap,
					       &_vh_next_freq, policy,
					       &z_policy->need_freq_update);
		if (_vh_next_freq)
			freq = _vh_next_freq;
		else
			freq = map_util_freq(util, freq, max_cap);
	}

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

			/* Patch C3: PELT rising-edge tier.  Catches a
			 * sharp single-sample slope-up that the
			 * rolling-window delta above dilutes.  Reuses
			 * newest_idx (already computed) and pulls the
			 * sample one position older so the slope is
			 * (newest - prev) / max_cap.  Only fires if
			 * predict_up did not already lift this tick
			 * (freq still below eff_hispeed) AND the slope
			 * test passes AND the absolute level guard
			 * (pelt_rising_edge_min_pct) is satisfied so we
			 * do not chase noise from a low base.
			 */
			if (freq < eff_hispeed &&
			    z_policy->tunables->pelt_rising_edge_thresh) {
				unsigned int prev_idx;
				unsigned long prev;

				prev_idx = (idx + ZENITH_PREDICT_UP_WINDOW_MAX - 2)
					% ZENITH_PREDICT_UP_WINDOW_MAX;
				prev = z_policy->util_history[prev_idx];

				if (newest > prev) {
					unsigned long edge = newest - prev;
					unsigned int edge_x256 = (unsigned int)
						((edge * 256) / max_cap);
					unsigned int newest_pct = (unsigned int)
						((newest * 100) / max_cap);

					if (edge_x256 >=
					    z_policy->tunables->pelt_rising_edge_thresh &&
					    newest_pct >=
					    z_policy->tunables->pelt_rising_edge_min_pct) {
						freq = eff_hispeed;
						tp_path = "pelt_edge";
						zenith_peer_ramp_arm(z_policy,
								     now_ns);
					}
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
			zenith_batt_scaled(
				z_policy->tunables->peak_headroom_hold_ms,
				z_policy->tunables->batt_hold_scale_pct);
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

	/* Wave A charger-aware floor.  When charger_aware == 1 AND the
	 * AC-vs-battery cache reports !on_battery, apply a freq floor of
	 * (policy->max * charger_floor_pct / 100).  The AC-vs-battery
	 * cache is updated lazily once per ZENITH_AUTO_TUNE_PERIOD by
	 * zenith_auto_tune_work() via power_supply_is_system_supplied(),
	 * so the floor follows the cable within roughly one auto_eval_ms
	 * window of plug / unplug.
	 *
	 * Both knobs default 0 so the tier is opt-in; thermal still wins
	 * downstream because auto_thermal_cap / thermal_state run after
	 * this site and can walk the floor back down if the SoC heats.
	 */
	if (z_policy->tunables->charger_aware &&
	    z_policy->tunables->charger_floor_pct &&
	    !atomic_read(&zenith_on_battery)) {
		unsigned int cf = (policy->max *
				   z_policy->tunables->charger_floor_pct) /
				  100;

		if (cf > policy->max)
			cf = policy->max;
		if (freq < cf) {
			freq = cf;
			tp_path = "charger_floor";
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

	/* Wave A render-thread util tracker.  More selective sibling
	 * of render_floor: applies a separate (typically higher) floor
	 * only when the matched render thread's PELT util_avg is at or
	 * above render_thread_util_thresh.  This filters out false
	 * positives where RenderThread is observed but is sitting idle
	 * in its main loop (paused video, static UI), where the
	 * unconditional render_floor over-floors.
	 *
	 * Reuses the zenith_policy_has_render() cache; the helper
	 * stores the matched task's util_avg in
	 * z_policy->render_matched_util_avg.  All three knobs default
	 * 0 so the tier is opt-in.  Requires render_aware=1 (otherwise
	 * the comm-walk does not run and util_avg is not observed).
	 */
	if (ZENITH_FEATURE_ENABLED(render_aware) &&
	    z_policy->tunables->render_thread_util_aware &&
	    z_policy->tunables->render_thread_util_thresh &&
	    z_policy->tunables->render_thread_util_floor_pct &&
	    zenith_policy_has_render(z_policy) &&
	    z_policy->render_matched_util_avg >=
	    z_policy->tunables->render_thread_util_thresh) {
		unsigned int rtuf =
			(policy->max *
			 z_policy->tunables->render_thread_util_floor_pct) /
			100;

		if (rtuf > policy->max)
			rtuf = policy->max;
		if (freq < rtuf) {
			freq = rtuf;
			tp_path = "render_thread_util_floor";
		}
	}

	/* Wave B PMU IPC tracker.  Apply a freq floor when measured
	 * IPC across the policy's CPUs is at or above pmu_ipc_thresh.
	 * The IPC cache is refreshed once per auto_tune window
	 * (zenith_pmu_sample_cpu() is called from zenith_auto_tune_-
	 * work()), so the floor follows workload-mix changes within
	 * roughly one auto_eval_ms after a transition.  Both knobs
	 * default 0 so the tier is opt-in; on CONFIG_PERF_EVENTS=n
	 * the helper returns a constant zero and the floor never
	 * applies.  See the comment block above ZENITH_DEFAULT_PMU_-
	 * AWARE for the full rationale.
	 */
	if (z_policy->tunables->pmu_aware &&
	    z_policy->tunables->pmu_ipc_thresh &&
	    z_policy->tunables->pmu_ipc_floor_pct &&
	    zenith_policy_max_ipc_pct(z_policy) >=
	    z_policy->tunables->pmu_ipc_thresh) {
		unsigned int pf =
			(policy->max *
			 z_policy->tunables->pmu_ipc_floor_pct) /
			100;

		if (pf > policy->max)
			pf = policy->max;
		if (freq < pf) {
			freq = pf;
			tp_path = "pmu_ipc_floor";
		}
	}

	/* Wave B EAS energy-knee floor.  When em_aware is on AND the
	 * cpufreq driver has registered an Energy Model for this
	 * policy AND em_floor_pct is non-zero, raise the freq floor to
	 * (em_knee_freq * em_floor_pct / 100).  The energy-knee is
	 * the OPP that minimises joules per instruction for sustained
	 * load -- below the knee, voltage scaling stops paying off
	 * and the policy wastes time without saving much energy.
	 * Floor still subject to the policy->max clamp downstream.
	 * See the comment block above ZENITH_DEFAULT_EM_AWARE for
	 * the full rationale.
	 */
	if (z_policy->tunables->em_aware &&
	    z_policy->tunables->em_floor_pct) {
		unsigned int knee = zenith_em_knee_freq(z_policy);

		if (knee) {
			unsigned int ef =
				(knee *
				 z_policy->tunables->em_floor_pct) /
				100;

			if (ef > policy->max)
				ef = policy->max;
			if (freq < ef) {
				freq = ef;
				tp_path = "em_floor";
			}
		}
	}

	/* Audit fix F3: V2-gated render-thread RT-priority floor.
	 *
	 * Per-policy uclamp-min-style boost.  When the tunable is enabled
	 * AND ZENITH_AT_FLAG_RENDER is currently active AND V2 has already
	 * committed to LATENCY or SUSTAINED_PERF, raise the freq floor to
	 * (policy->max * auto_tune_render_rt_floor_pct / 100).
	 *
	 * Logically a stricter sibling of render_floor_pct that fires only
	 * in the V2 states where a single CFS preemption of RenderThread
	 * is user-visible as a frame stutter.  See the
	 * ZENITH_DEFAULT_AT_RENDER_RT_FLOOR_PCT comment block for full
	 * semantics.  Default tunable is 0 (off); when enabled, this floor
	 * always wins over render_floor_pct because it is applied last in
	 * the floor chain.
	 */
	if (z_policy->tunables->auto_tune_render_rt_floor_pct &&
	    (z_policy->at_last_flags & ZENITH_AT_FLAG_RENDER) &&
	    (z_policy->at_last_state == ZENITH_AT_STATE_LATENCY ||
	     z_policy->at_last_state == ZENITH_AT_STATE_SUSTAINED_PERF)) {
		unsigned int rrf =
			(policy->max *
			 z_policy->tunables->auto_tune_render_rt_floor_pct) /
			100;

		if (rrf > policy->max)
			rrf = policy->max;
		if (freq < rrf) {
			freq = rrf;
			tp_path = "render_rt_floor";
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

	/* Wave A cgroup-aware top-app floor.  When top_app_aware == 1
	 * AND any CPU in the policy is currently running a task in the
	 * cpuset cgroup named "top-app", apply a freq floor of
	 * (policy->max * top_app_floor_pct / 100).  Cached for
	 * ZENITH_TOP_APP_CACHE_TTL_NS via
	 * zenith_policy_has_top_app() to keep the hot path cheap.
	 *
	 * Comm-walk floors (audio / render / camera / charger) all run
	 * before this site, so top_app_floor only fires when none of
	 * the more specific signals already lifted freq above
	 * top_app_floor_pct.  Both knobs default 0 so the tier is
	 * opt-in.  Requires CONFIG_CPUSETS=y; on CONFIG_CPUSETS=n the
	 * helper short-circuits to false and the floor never applies.
	 */
	if (z_policy->tunables->top_app_aware &&
	    z_policy->tunables->top_app_floor_pct &&
	    zenith_policy_has_top_app(z_policy)) {
		unsigned int taf = (policy->max *
				    z_policy->tunables->top_app_floor_pct) /
				   100;

		if (taf > policy->max)
			taf = policy->max;
		if (freq < taf) {
			freq = taf;
			tp_path = "top_app_floor";
		}
	}

	/* Patch K: game / sustained-high-load performance burst floor.
	 * When the FSM (evaluated up-stream from this function, near
	 * the game_auto tick) is ARMED, lift freq to the operator-
	 * configured floor.  COOLDOWN linearly steps the floor down to
	 * 0 across cooldown_ms.  zenith_gpb_floor() returns 0 when:
	 *   - master tunable is 0
	 *   - FSM is IDLE (no detected game / sustained load)
	 *   - thermal guardrail is engaged (skin temp >= ceiling)
	 * so the call is a no-op outside the active windows.  The
	 * pin_to_target gate skips the floor while a higher-priority
	 * pin (input boost / brutality) is in charge.
	 */
	if (!pin_to_target &&
	    ZENITH_FEATURE_ENABLED(game_perf_burst) &&
	    READ_ONCE(z_policy->tunables->game_perf_burst)) {
		unsigned int gpb = zenith_gpb_floor(z_policy, policy->max);

		if (gpb && freq < gpb) {
			freq = gpb;
			tp_path = "game_perf_burst_floor";
		}
	}

	/* Patch C6: SCHED_DEADLINE awareness floor.  Walks the
	 * policy CPU mask and looks for any rq with a non-zero
	 * dl.dl_nr_running.  When found, lift freq to
	 * (policy->max * dl_task_floor_pct / 100).  Race on
	 * dl_nr_running is harmless: read is a single load,
	 * decision is a heuristic responsiveness lift on top of
	 * the bandwidth math that schedutil_cpu_util already does
	 * for correctness.
	 *
	 * Tunable 0 (cold-boot default) disables; profile bakes
	 * default it on for PERFORMANCE / GAMING (100) and AUDIO
	 * (80).  pin_to_target skip avoids stacking under a
	 * higher pin from input_boost / brutality.
	 */
	if (!pin_to_target && policy->max &&
	    z_policy->tunables->dl_task_floor_pct) {
		int cpu_iter;
		bool dl_present = false;

		for_each_cpu(cpu_iter, policy->cpus) {
			if (READ_ONCE(cpu_rq(cpu_iter)->dl.dl_nr_running)) {
				dl_present = true;
				break;
			}
		}
		if (dl_present) {
			unsigned int dlf = (policy->max *
					    z_policy->tunables->dl_task_floor_pct) /
					   100;

			if (dlf > policy->max)
				dlf = policy->max;
			if (freq < dlf) {
				freq = dlf;
				tp_path = "dl_floor";
			}
		}
	}

	/* Patch C9: io_floor hysteresis sticky-floor.  When the
	 * iowait_boost path armed within the last io_floor_hyst_ms
	 * (zenith_iowait_boost() stamped io_floor_until_ns), lift
	 * freq to (policy->max * io_floor_hyst_pct / 100).  Both
	 * tunables 0 disables; pin_to_target paths skip (already
	 * pinned higher).
	 *
	 * Why this exists: iowait_boost decays to 0 within a few
	 * PELT periods after the last SCHED_CPUFREQ_IOWAIT, but
	 * sustained block IO (sqlite WAL replay, ext4 commit, media
	 * transcode) often goes through a small idle gap and then
	 * resumes; the boost has decayed and the level signal alone
	 * pins a low freq because the worker is mostly D-state.
	 * Holding the floor closes the gap.
	 */
	if (!pin_to_target && policy->max &&
	    z_policy->tunables->io_floor_hyst_ms &&
	    z_policy->tunables->io_floor_hyst_pct) {
		u64 until = z_policy->io_floor_until_ns;
		u64 now_ns = ktime_get_ns();

		if (until && now_ns < until) {
			unsigned int iof = (policy->max *
					    z_policy->tunables->io_floor_hyst_pct) /
					   100;

			if (iof > policy->max)
				iof = policy->max;
			if (freq < iof) {
				freq = iof;
				tp_path = "io_floor";
			}
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

	/* 3c''''''-pre. Cluster-wake-pulse soft floor (Patch 1.3).
	 *
	 * Mirrors the migration_floor mechanic below: if the arm block
	 * at the top of zenith_get_next_freq() detected a >= cluster_-
	 * wake_pulse_idle_ms gap since the last eval, it stamped a
	 * deadline on cluster_wake_pulse_until_ns.  While that
	 * deadline has not expired, hold a soft floor at
	 * cluster_wake_pulse_floor_pct of policy->max so the freshly-
	 * woken cluster runs above PELT-cold-start freq for the
	 * pulse window.
	 *
	 * Bypassed when pin_to_target (input_boost / brutality already
	 * own freq above any plausible pulse floor) or when the floor
	 * percentage is 0 (knob stamps but suppresses, mirroring
	 * peer_ramp / migration_floor).  Runs before the migration_-
	 * floor tier so a cluster that wakes AND receives an inbound
	 * migrating task picks the higher of the two floors.
	 */
	if (!pin_to_target && policy->max &&
	    z_policy->tunables->cluster_wake_pulse_floor_pct) {
		u64 until = z_policy->cluster_wake_pulse_until_ns;

		if (until && ktime_get_ns() < until) {
			unsigned int floor =
				(policy->max / 100) *
				z_policy->tunables->cluster_wake_pulse_floor_pct;

			if (floor > policy->max)
				floor = policy->max;
			if (freq < floor) {
				freq = floor;
				tp_path = "cluster_wake_pulse";
			}
		}
	}

	/* 3c''''''-pre-fg. Foreground-transition pulse soft floor
	 * (Patch 1.9).  Mirrors the cluster_wake_pulse mechanic:
	 * the sched_wakeup_new probe stamps fg_transition_pulse_-
	 * until_ns whenever a foreground task is woken for the
	 * first time after fork() on a CPU belonging to this policy.
	 * While the deadline has not expired, hold a soft floor at
	 * fg_transition_pulse_pct of policy->max so the freshly-
	 * forked top-app task picks up above PELT cold-start freq.
	 *
	 * Bypassed when pin_to_target (input_boost / brutality
	 * already at or above any plausible pulse floor) or when
	 * the floor percentage is 0 (knob stamps but suppresses).
	 * Reads the deadline with READ_ONCE since the writer is the
	 * sched_wakeup_new probe in arbitrary scheduler context;
	 * see the comment block above zenith_probe_wakeup_new for
	 * the full safety argument.
	 */
	if (!pin_to_target && policy->max &&
	    z_policy->tunables->fg_transition_pulse_pct) {
		u64 until = READ_ONCE(z_policy->fg_transition_pulse_until_ns);

		if (until && ktime_get_ns() < until) {
			unsigned int floor =
				(policy->max / 100) *
				z_policy->tunables->fg_transition_pulse_pct;

			if (floor > policy->max)
				floor = policy->max;
			if (freq < floor) {
				freq = floor;
				tp_path = "fg_transition_pulse";
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

	/* 3f. Quiet-hours cap (Patch 1.10).
	 *
	 * Hard freq cap inside the user-configured nightly window.
	 * pin_to_target tiers (input_boost / brutality) bypass the
	 * cap so a user explicitly poking the device mid-window still
	 * gets full responsiveness; passive evaluation is what gets
	 * throttled.  The screen_off_only gate is the second guard:
	 * with the default of 1, the cap only fires while
	 * tunables->screen_state == 0, so a quiet-hours window that
	 * accidentally overlaps an active call or alarm doesn't drag
	 * the cluster down.
	 *
	 * cap_pct is bounded floor 50 % (sysfs); cap_pct == 100 makes
	 * the tier a no-op and is the default, so without an explicit
	 * profile / sysfs override the tier is invisible.
	 */
	if (!pin_to_target && policy->max &&
	    z_policy->tunables->quiet_hours_cap_pct &&
	    z_policy->tunables->quiet_hours_cap_pct < 100 &&
	    (!z_policy->tunables->quiet_hours_screen_off_only ||
	     !READ_ONCE(z_policy->tunables->screen_state)) &&
	    zenith_in_quiet_hours(z_policy->tunables)) {
		unsigned int qh_cap = (policy->max / 100) *
			z_policy->tunables->quiet_hours_cap_pct;

		if (qh_cap < policy->min)
			qh_cap = policy->min;
		if (qh_cap > policy->max)
			qh_cap = policy->max;
		if (freq > qh_cap) {
			freq = qh_cap;
			tp_path = "quiet_hours_cap";
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
		/*
		 * Hikari floor application on the early-return path.
		 * Raises the cached freq if Hikari has a published
		 * wake-demand floor that exceeds it.  No-op when no
		 * floor is published or when Hikari is off.
		 */
		{
			unsigned int hf = zenith_hikari_policy_floor(policy);

			if (hf && hf > z_policy->next_freq)
				return hf;
		}
		return z_policy->next_freq;
	}

	z_policy->cached_raw_freq = freq;
	{
		unsigned int _vh_idx, _vh_l_freq, _vh_h_freq;

		_vh_l_freq = cpufreq_driver_resolve_freq(policy, freq);
		_vh_idx = cpufreq_frequency_table_target(policy, freq,
							 CPUFREQ_RELATION_H);
		_vh_h_freq = policy->freq_table[_vh_idx].frequency;
		_vh_h_freq = clamp(_vh_h_freq, policy->min, policy->max);
		if (_vh_l_freq <= _vh_h_freq || _vh_l_freq == policy->min)
			target_freq = _vh_l_freq;
		else if (mult_frac(100, freq - _vh_h_freq,
				   _vh_l_freq - _vh_h_freq) < 20)
			target_freq = _vh_h_freq;
		else
			target_freq = _vh_l_freq;
	}

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

	/* 6a. auto_thermal_cap (Path B): final-stage hard cap on
	 * target_freq when sustained thermal pressure exceeds the
	 * configured threshold.  Default off (auto_thermal_cap == 0);
	 * a single READ_ONCE short-circuits the pressure read on the
	 * fast path.  Applied AFTER em_cap so the energy model has
	 * already validated the freq; this tier just clamps the upper
	 * bound to policy->max * auto_thermal_cap_freq_pct / 100 when
	 * the pressure threshold is met.  See ZENITH_DEFAULT_AUTO_-
	 * THERMAL_CAP comment block.  Wrapped by the thermal_aware
	 * master gate (static-key) so the entire pressure read +
	 * threshold compare folds away when the master gate is off.
	 */
	if (ZENITH_FEATURE_ENABLED(thermal_aware) &&
	    READ_ONCE(z_policy->tunables->auto_thermal_cap)) {
		unsigned int p_pct =
			zenith_policy_thermal_pressure_pct(z_policy);
		unsigned int thresh = READ_ONCE(z_policy->tunables->
					auto_thermal_cap_pressure_pct);
		unsigned int cap_pct = READ_ONCE(z_policy->tunables->
					auto_thermal_cap_freq_pct);

		if (p_pct >= thresh && cap_pct < 100) {
			unsigned long cap_freq =
				((unsigned long)policy->max * cap_pct) /
				100UL;

			if (cap_freq && target_freq > cap_freq) {
				target_freq = cap_freq;
				tp_path = "auto_thermal_cap";
			}
		}
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

	/* Patch 1.4: decision-latency histogram.  Single ktime read
	 * + subtract + bucket increment.  3 fixed thresholds (10 us
	 * / 50 us / 100 us) cover the practical range of zenith eval
	 * costs on a 5.10 kernel; an eval that lands above 100 us is
	 * already pathological and the >=100us bucket is intentionally
	 * a fire-and-forget marker for those.
	 *
	 * Patch B7-2 piggy-backs on the same lat_ns sample to feed
	 * the decision-ring entry below; pulled out of the inner
	 * scope so dec_ring stamping can reuse it without re-reading
	 * ktime_get_ns().
	 */
	{
		u64 lat_ns = ktime_get_ns() - dec_eval_start_ns;
		unsigned int head;

		if (lat_ns < 10000ULL)
			z_policy->dec_lat_buckets[0]++;
		else if (lat_ns < 50000ULL)
			z_policy->dec_lat_buckets[1]++;
		else if (lat_ns < 100000ULL)
			z_policy->dec_lat_buckets[2]++;
		else
			z_policy->dec_lat_buckets[3]++;

		/* Patch B7-2: append (path, lat_ns) to the per-policy
		 * decision ring.  Clamp lat_ns to u32_max so the field
		 * truncation matches the storage type without wrapping.
		 * head advances under the same update_lock that gates
		 * this whole tail; readers use READ_ONCE on path and
		 * accept the corresponding lat_ns torn-write window
		 * (worst case: a brief mismatch resolved on the next
		 * read, perfectly fine for an observability ring).
		 */
		head = z_policy->dec_ring_head & ZENITH_DEC_RING_MASK;
		z_policy->dec_ring[head].lat_ns =
			(lat_ns > U32_MAX) ? U32_MAX : (u32)lat_ns;
		WRITE_ONCE(z_policy->dec_ring[head].path, tp_path);
		WRITE_ONCE(z_policy->dec_ring_head, head + 1);
	}
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

	/*
	 * Hikari floor application on the main return path.  Raises
	 * target_freq if Hikari has a published wake-demand floor
	 * that exceeds it.  No-op when no floor is published or when
	 * Hikari is off.  Applied AFTER all in-governor decision
	 * tiers so the floor acts as a hard, additive lower bound on
	 * the wake-time freq -- it can lift Zenith's choice but
	 * never lower it.
	 */
	{
		unsigned int hf = zenith_hikari_policy_floor(policy);

		if (hf && hf > target_freq)
			target_freq = hf;
	}

	return target_freq;
}

void zenith_execute_switch(struct zenith_policy *z_policy, u64 time, unsigned int next_freq)
{
	if (z_policy->need_freq_update) {
		z_policy->need_freq_update = false;
		if (z_policy->next_freq == next_freq &&
		    !cpufreq_driver_test_flags(CPUFREQ_NEED_UPDATE_LIMITS))
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

void zenith_update_single(struct update_util_data *hook, u64 time, unsigned int flags)
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

void zenith_update_shared(struct update_util_data *hook, u64 time, unsigned int flags)
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

			/*
			 * Skip siblings whose zenith_start() has not yet
			 * populated j_z_cpu->z_policy.  Without this guard,
			 * the first update tick that fires on a policy CPU
			 * before all sibling CPUs in the same policy have
			 * completed their per-CPU zenith_start() iteration
			 * dereferences NULL->tunables for the not-yet-
			 * initialized sibling and faults at
			 * NULL+offsetof(struct zenith_policy, tunables).
			 * The race window is closed by the two-loop ordering
			 * in zenith_start(), but keep the runtime check as
			 * belt-and-suspenders for any future code path that
			 * might transiently leave z_policy NULL.
			 */
			if (unlikely(!READ_ONCE(j_z_cpu->z_policy)))
				continue;

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

void zenith_work(struct kthread_work *work)
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

void zenith_irq_work(struct irq_work *irq_work)
{
	struct zenith_policy *z_policy = container_of(irq_work, struct zenith_policy, irq_work);

	kthread_queue_work(&z_policy->worker, &z_policy->work);
}
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
	thread = kthread_create(kthread_worker_fn, &z_policy->worker, "zenith:%d",
				cpumask_first(z_policy->policy->related_cpus));
	if (IS_ERR(thread))
		return PTR_ERR(thread);

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

/* Wave B PMU IPC tracker.  Allocate the per-CPU instructions and
 * cycles perf_events on this CPU.  Idempotent: if the events are
 * already allocated (re-attach on the same CPU after a governor
 * cycle), return success without re-allocating.  Allocation failures
 * (PMU not exposed by the SoC, perf locked down, OOM) leave the
 * pointers NULL; subsequent zenith_pmu_sample_cpu() calls return
 * early and the floor never applies on this CPU.  See the comment
 * block above ZENITH_DEFAULT_PMU_AWARE for the full rationale.
 */
#if IS_ENABLED(CONFIG_PERF_EVENTS)
int zenith_pmu_init_cpu(unsigned int cpu)
{
	struct zenith_pmu_state *st = per_cpu_ptr(&zenith_pmu, cpu);
	struct perf_event_attr inst_attr = {
		.type		= PERF_TYPE_HARDWARE,
		.config		= PERF_COUNT_HW_INSTRUCTIONS,
		.size		= sizeof(inst_attr),
		.pinned		= 1,
		.disabled	= 0,
		.exclude_idle	= 1,
	};
	struct perf_event_attr cycle_attr = inst_attr;

	cycle_attr.config = PERF_COUNT_HW_CPU_CYCLES;

	if (st->inst_event && st->cycle_event)
		return 0;

	if (!st->inst_event) {
		struct perf_event *e =
			perf_event_create_kernel_counter(&inst_attr, cpu,
							 NULL, NULL, NULL);

		if (IS_ERR(e)) {
			st->inst_event = NULL;
			return PTR_ERR(e);
		}
		st->inst_event = e;
	}
	if (!st->cycle_event) {
		struct perf_event *e =
			perf_event_create_kernel_counter(&cycle_attr, cpu,
							 NULL, NULL, NULL);

		if (IS_ERR(e)) {
			perf_event_release_kernel(st->inst_event);
			st->inst_event = NULL;
			st->cycle_event = NULL;
			return PTR_ERR(e);
		}
		st->cycle_event = e;
	}
	st->last_inst = 0;
	st->last_cycles = 0;
	st->ipc_pct = 0;
	return 0;
}

void zenith_pmu_exit_cpu(unsigned int cpu)
{
	struct zenith_pmu_state *st = per_cpu_ptr(&zenith_pmu, cpu);

	if (st->inst_event) {
		perf_event_release_kernel(st->inst_event);
		st->inst_event = NULL;
	}
	if (st->cycle_event) {
		perf_event_release_kernel(st->cycle_event);
		st->cycle_event = NULL;
	}
	st->last_inst = 0;
	st->last_cycles = 0;
	st->ipc_pct = 0;
}

/* Sample the per-CPU instructions and cycles counters and compute
 * IPC as a percentage.  Called from zenith_auto_tune_work() (process
 * context); perf_event_read_value() uses smp_call_function_single()
 * internally to read the counter on the target CPU, so this is safe
 * to call on any CPU regardless of which CPU owns the event.  Stores
 * the latest IPC in st->ipc_pct via WRITE_ONCE() so the lock-free
 * reader in zenith_policy_max_ipc_pct() sees a coherent value.
 */
void zenith_pmu_sample_cpu(unsigned int cpu)
{
	struct zenith_pmu_state *st = per_cpu_ptr(&zenith_pmu, cpu);
	u64 enabled, running, inst, cycles, di, dc;

	if (!st->inst_event || !st->cycle_event)
		return;

	inst   = perf_event_read_value(st->inst_event,  &enabled, &running);
	cycles = perf_event_read_value(st->cycle_event, &enabled, &running);

	di = inst   - st->last_inst;
	dc = cycles - st->last_cycles;
	st->last_inst   = inst;
	st->last_cycles = cycles;

	if (dc) {
		u64 r = div64_u64(di * 100, dc);

		if (r > ZENITH_PMU_IPC_THRESH_MAX)
			r = ZENITH_PMU_IPC_THRESH_MAX;
		WRITE_ONCE(st->ipc_pct, (unsigned int)r);
	}
}

unsigned int zenith_policy_max_ipc_pct(struct zenith_policy *z_policy)
{
	struct cpufreq_policy *policy = z_policy->policy;
	unsigned int cpu;
	unsigned int max_ipc = 0;

	for_each_cpu(cpu, policy->cpus) {
		struct zenith_pmu_state *st = per_cpu_ptr(&zenith_pmu, cpu);
		unsigned int ipc = READ_ONCE(st->ipc_pct);

		if (ipc > max_ipc)
			max_ipc = ipc;
	}
	return max_ipc;
}
#else
inline int zenith_pmu_init_cpu(unsigned int cpu)
{
	return 0;
}
inline void zenith_pmu_exit_cpu(unsigned int cpu)
{
}
inline void zenith_pmu_sample_cpu(unsigned int cpu)
{
}
static inline unsigned int
zenith_policy_max_ipc_pct(struct zenith_policy *z_policy)
{
	return 0;
}
#endif

/* Wave B EAS / Energy Model integration.  See the comment block
 * above ZENITH_DEFAULT_EM_AWARE for the full rationale.  On
 * CONFIG_ENERGY_MODEL=n, em_cpu_get() returns NULL unconditionally
 * (header-defined) and this function returns 0; the caller treats
 * 0 as "no EM" and the em_floor never applies.  No #if guard is
 * required at this site because the header itself stubs the API.
 */
unsigned int zenith_em_knee_freq(struct zenith_policy *z_policy)
{
	struct em_perf_domain *em;
	unsigned int cpu = cpumask_first(z_policy->policy->cpus);
	unsigned long best_cost = ULONG_MAX;
	unsigned int knee_freq = 0;
	int i;

	if (z_policy->em_knee_freq)
		return z_policy->em_knee_freq;

	em = em_cpu_get(cpu);
	if (!em || em->nr_perf_states <= 0 || !em->table)
		return 0;

	for (i = 0; i < em->nr_perf_states; i++) {
		unsigned long c = em->table[i].cost;

		if (c && c < best_cost) {
			best_cost = c;
			knee_freq = (unsigned int)em->table[i].frequency;
		}
	}

	/* Bad EM (all costs zero, or single-OPP table): fall back to
	 * the lowest registered freq, which is at least a meaningful
	 * lower bound for the knee.  Caller still scales by
	 * em_floor_pct so a 0% effective floor is fine.
	 */
	if (!knee_freq)
		knee_freq = (unsigned int)em->table[0].frequency;

	z_policy->em_knee_freq = knee_freq;
	return knee_freq;
}

int zenith_init(struct cpufreq_policy *policy)
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
	tunables->batt_hold_scale_pct =
		ZENITH_DEFAULT_BATT_HOLD_SCALE_PCT;
	tunables->charger_aware =
		ZENITH_DEFAULT_CHARGER_AWARE;
	tunables->charger_floor_pct =
		ZENITH_DEFAULT_CHARGER_FLOOR_PCT;
	tunables->top_app_aware =
		ZENITH_DEFAULT_TOP_APP_AWARE;
	tunables->top_app_floor_pct =
		ZENITH_DEFAULT_TOP_APP_FLOOR_PCT;
	tunables->render_thread_util_aware =
		ZENITH_DEFAULT_RENDER_THREAD_UTIL_AWARE;
	tunables->render_thread_util_thresh =
		ZENITH_DEFAULT_RENDER_THREAD_UTIL_THRESH;
	tunables->render_thread_util_floor_pct =
		ZENITH_DEFAULT_RENDER_THREAD_UTIL_FLOOR_PCT;
	tunables->pmu_aware =
		ZENITH_DEFAULT_PMU_AWARE;
	tunables->pmu_ipc_thresh =
		ZENITH_DEFAULT_PMU_IPC_THRESH;
	tunables->pmu_ipc_floor_pct =
		ZENITH_DEFAULT_PMU_IPC_FLOOR_PCT;
	tunables->em_aware =
		ZENITH_DEFAULT_EM_AWARE;
	tunables->em_floor_pct =
		ZENITH_DEFAULT_EM_FLOOR_PCT;
	tunables->cluster_wake_pulse_ms =
		ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_MS;
	tunables->cluster_wake_pulse_idle_ms =
		ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_IDLE_MS;
	tunables->cluster_wake_pulse_floor_pct =
		ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_FLOOR_PCT;
	tunables->quiet_hours_start_min =
		ZENITH_DEFAULT_QUIET_HOURS_START_MIN;
	tunables->quiet_hours_end_min =
		ZENITH_DEFAULT_QUIET_HOURS_END_MIN;
	tunables->quiet_hours_cap_pct =
		ZENITH_DEFAULT_QUIET_HOURS_CAP_PCT;
	tunables->quiet_hours_screen_off_only =
		ZENITH_DEFAULT_QUIET_HOURS_SCREEN_OFF_ONLY;
	tunables->fg_transition_pulse_ms =
		ZENITH_DEFAULT_FG_TRANSITION_PULSE_MS;
	tunables->fg_transition_pulse_pct =
		ZENITH_DEFAULT_FG_TRANSITION_PULSE_PCT;
	tunables->predict_up_thresh	= ZENITH_DEFAULT_PREDICT_UP_THRESH;
	tunables->predict_up_window	= ZENITH_DEFAULT_PREDICT_UP_WINDOW;
	tunables->pelt_rising_edge_thresh =
		ZENITH_DEFAULT_PELT_RISING_EDGE_THRESH;
	tunables->pelt_rising_edge_min_pct =
		ZENITH_DEFAULT_PELT_RISING_EDGE_MIN_PCT;
	tunables->dl_task_floor_pct	= ZENITH_DEFAULT_DL_TASK_FLOOR_PCT;
	tunables->io_floor_hyst_ms	= ZENITH_DEFAULT_IO_FLOOR_HYST_MS;
	tunables->io_floor_hyst_pct	= ZENITH_DEFAULT_IO_FLOOR_HYST_PCT;
	tunables->vh_arch_freq_scale_enable =
		ZENITH_DEFAULT_VH_ARCH_FREQ_SCALE_ENABLE;
	tunables->vh_uclamp_observer_enable =
		ZENITH_DEFAULT_VH_UCLAMP_OBSERVER_ENABLE;
	tunables->vh_cpu_idle_enable =
		ZENITH_DEFAULT_VH_CPU_IDLE_ENABLE;
	tunables->vh_freq_qos_enable =
		ZENITH_DEFAULT_VH_FREQ_QOS_ENABLE;
	tunables->vh_sched_move_task_enable =
		ZENITH_DEFAULT_VH_SCHED_MOVE_TASK_ENABLE;
	tunables->vh_scheduler_tick_enable =
		ZENITH_DEFAULT_VH_SCHEDULER_TICK_ENABLE;
	/* vh_freq_qos_pressure_until_ns is already 0 from kzalloc;
	 * 0 < any future ktime_get_ns() so the auto-classify check
	 * starts disarmed.  No explicit atomic64_set needed.
	 */
	/* Patch B-AUTO-2: seed auto_target so the auto_target sysfs
	 * node never reads 0 / "balanced" by accident on a fresh
	 * tunables alloc.  The actual cold-boot active_profile flip to
	 * ZENITH_PROFILE_AUTO lives in B-AUTO-5; here we just ensure
	 * the meta-state field is well-defined when a user writes
	 * "auto" to the profile sysfs node before any auto eval has
	 * landed.
	 */
	tunables->auto_target		= ZENITH_PROFILE_BALANCED;
	/* Patch B-AUTO-3: cadence + hysteresis defaults for the auto
	 * selector engine.  The eval_work itself is initialised below
	 * after the sysfs attr_set publication, but we need the
	 * tunables to carry sane values from this point so a
	 * subsequent profile_store("auto") schedules at the intended
	 * cadence rather than at jiffies-now (msecs_to_jiffies(0) is
	 * 0 jiffies, i.e. "run immediately on the next tick").
	 */
	tunables->auto_eval_ms		= ZENITH_DEFAULT_AUTO_EVAL_MS;
	tunables->auto_hysteresis_ms	= ZENITH_DEFAULT_AUTO_HYSTERESIS_MS;
	tunables->auto_pending_target	= ZENITH_PROFILE_BALANCED;
	tunables->auto_pending_first_seen_ns = 0;
	INIT_DEFERRABLE_WORK(&tunables->eval_work, zenith_auto_eval_work_fn);
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
	tunables->auto_tune_util_rising_thresh_pct =
		ZENITH_DEFAULT_AT_UTIL_RISING_THRESH_PCT;
	tunables->auto_tune_render_rt_floor_pct =
		ZENITH_DEFAULT_AT_RENDER_RT_FLOOR_PCT;
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
	tunables->thermal_aware		= ZENITH_DEFAULT_THERMAL_AWARE;
	tunables->thermal_active	= 0;
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
	tunables->auto_thermal_cap	= ZENITH_DEFAULT_AUTO_THERMAL_CAP;
	tunables->auto_thermal_cap_pressure_pct =
		ZENITH_DEFAULT_AUTO_THERMAL_CAP_PRESSURE_PCT;
	tunables->auto_thermal_cap_freq_pct =
		ZENITH_DEFAULT_AUTO_THERMAL_CAP_FREQ_PCT;
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
	tunables->audio_hyst_ms		= ZENITH_DEFAULT_AUDIO_HYST_MS;
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
	tunables->verbose_log		= ZENITH_DEFAULT_VERBOSE_LOG;
	/* Patch K: game_perf_burst defaults.  See the
	 * ZENITH_DEFAULT_GAME_PERF_BURST comment block for the
	 * rationale on each default.
	 */
	tunables->game_perf_burst	= ZENITH_DEFAULT_GAME_PERF_BURST;
	tunables->game_perf_burst_floor_pct =
		ZENITH_DEFAULT_GAME_PERF_BURST_FLOOR_PCT;
	tunables->game_perf_burst_thermal_ceiling_dc =
		ZENITH_DEFAULT_GAME_PERF_BURST_THERMAL_CEILING_DC;
	tunables->game_perf_burst_disarm_grace_ms =
		ZENITH_DEFAULT_GAME_PERF_BURST_DISARM_GRACE_MS;
	tunables->game_perf_burst_cooldown_ms =
		ZENITH_DEFAULT_GAME_PERF_BURST_COOLDOWN_MS;
	WRITE_ONCE(zenith_input_boost_active_ms, ZENITH_DEFAULT_INPUT_BOOST_MS);
	WRITE_ONCE(zenith_input_boost_touchdown_extra_ms_cache,
		   ZENITH_DEFAULT_INPUT_BOOST_TOUCHDOWN_EXTRA_MS);

	/* Sync the audio_aware / render_aware / camera_aware /
	 * psi_aware / game_auto / auto_tune_v3 / thermal_aware static
	 * keys against their default scalars.  See the comment above
	 * DEFINE_STATIC_KEY_FALSE for the invariant: scalars whose
	 * default is non-zero need an explicit init-time key enable.
	 * audio_aware / render_aware were flipped to 1 in wave-2;
	 * game_auto was flipped to 1 and auto_tune_v3 to 2 in wave-7;
	 * camera_aware and psi_aware were flipped to 1 in the
	 * auto-defaults round mirroring audio/render so all six
	 * detector branches ship live by default; thermal_aware
	 * defaults to 1 as the master gate over the thermal-mechanism
	 * cluster.  Idempotent across re-attaches:
	 * zenith_set_static_key() is a no-op if the key is already
	 * in the requested state.  zenith_set_static_key() coerces
	 * non-zero scalars (including the auto_tune_v3 = 2 APPLY
	 * mode) to TRUE, which is the correct branch state for any
	 * non-OFF mode.
	 */
	zenith_set_static_key(&zenith_audio_aware_key,
			      tunables->audio_aware);
	zenith_set_static_key(&zenith_render_aware_key,
			      tunables->render_aware);
	zenith_set_static_key(&zenith_camera_aware_key,
			      tunables->camera_aware);
	zenith_set_static_key(&zenith_psi_aware_key,
			      tunables->psi_aware);
	zenith_set_static_key(&zenith_game_auto_key,
			      tunables->game_auto);
	zenith_set_static_key(&zenith_auto_tune_v3_key,
			      tunables->auto_tune_v3);
	zenith_set_static_key(&zenith_thermal_aware_key,
			      tunables->thermal_aware);
	/* Patch K: same invariant for game_perf_burst.  Default = 1
	 * (master ON, "all automatic"); the FSM evaluator + floor
	 * application both sit inside ZENITH_FEATURE_ENABLED checks
	 * so this sync is what unlocks the hot-path body when the
	 * scalar is non-zero.  Idempotent across re-attaches.
	 */
	zenith_set_static_key(&zenith_game_perf_burst_key,
			      tunables->game_perf_burst);

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
		} else {
			 *     (per-policy override above wins)
			 *   - echo balanced > .../zenith/profile
			 *     (sysfs profile_store disengages auto)
			 *   - echo custom > .../zenith/profile
			 *     (CUSTOM is auto-immune by design)
			 *
			 * LEGACY and CUSTOM remain manual-only
			 * targets: the classifier never picks them,
			 * so a user's explicit "echo custom >
			 * profile" is preserved across the eval
			 * cadence.
			 */
			tunables->active_profile = ZENITH_PROFILE_AUTO;
			WRITE_ONCE(tunables->auto_target,
				   ZENITH_PROFILE_BALANCED);
			tunables->auto_pending_target =
				ZENITH_PROFILE_BALANCED;
			tunables->auto_pending_first_seen_ns = 0;
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

	/* Patch B-AUTO-5: arm the auto-selector worker if this fresh
	 * tunables container booted into AUTO (cold-boot default,
	 * see the chosen-selection block above).  schedule_delayed_-
	 * work runs first eval after one auto_eval_ms window so the
	 * device has time to settle on the BALANCED bake before the
	 * classifier starts steering.  Subsequent policies that
	 * attach to this tunables container do not re-schedule --
	 * the worker is one-per-tunables.
	 */
	if (tunables->active_profile == ZENITH_PROFILE_AUTO)
		schedule_delayed_work(&tunables->eval_work,
				      msecs_to_jiffies(tunables->auto_eval_ms ?
						       tunables->auto_eval_ms :
						       ZENITH_DEFAULT_AUTO_EVAL_MS));

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

void zenith_exit(struct cpufreq_policy *policy)
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

	/*
	 * Publish the NULL so every subsequent vendor-hook probe
	 * (vh_arch_set_freq_scale, vh_cpu_idle_*, vh_setscheduler_uclamp,
	 * vh_freq_qos_update_request, vh_sched_move_task,
	 * vh_scheduler_tick) sees it and bails out immediately.
	 *
	 * The probes are registered at module level (zenith_gov_init)
	 * and fire from tracepoint callbacks which execute inside an
	 * RCU-sched read-side critical section (preempt_disable /
	 * rcu_read_lock_sched).  A probe that loaded z_policy before
	 * we NULLed governor_data is still referencing valid memory
	 * here -- the kfree has not happened yet.  synchronize_rcu()
	 * below guarantees that every such in-flight callback has
	 * returned before we free z_policy, closing the
	 * load-then-use-after-free window.
	 *
	 * Cost: one grace period (~few ms) on the teardown path,
	 * which only runs on governor switch -- not on the hot path.
	 */
	policy->governor_data = NULL;

	synchronize_rcu();

	cpufreq_disable_fast_switch(policy);
	kfree(z_policy);
}

int zenith_start(struct cpufreq_policy *policy)
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
	memset(z_policy->dec_lat_buckets, 0,
	       sizeof(z_policy->dec_lat_buckets));
	z_policy->at_last_state =
		zenith_profile_to_at_state(z_policy->tunables->active_profile);
	z_policy->at_pending_state = z_policy->at_last_state;
	z_policy->at_pending_windows = 0;
	z_policy->at_cooldown_left = 0;
	z_policy->at_last_target = z_policy->tunables->active_profile;

	/*
	 * Two-pass attach to close the per-CPU init race.
	 *
	 * Pass 1 zeroes per-CPU state and publishes z_policy for every
	 * CPU in the policy *before* any update-util hook is registered.
	 * Pass 2 then registers the hooks.  Once the first hook is live,
	 * the scheduler is free to fire zenith_update_{single,shared} on
	 * any policy CPU; with z_policy already published on every
	 * sibling, the per-CPU iteration in zenith_update_shared cannot
	 * see a NULL z_policy.
	 *
	 * Previously this was a single loop, so the moment
	 * cpufreq_add_update_util_hook() ran for the first CPU the hook
	 * could fire on that CPU and iterate over a sibling whose
	 * z_policy had not yet been assigned -- causing a NULL deref at
	 * zenith_get_util+0x6c (read at NULL+offsetof(tunables)).  The
	 * race window was widened by any change that shifted CFS tick
	 * timing (e.g. removing BORE) and was observed at boot on MT6768
	 * (8-core, two clusters, shared policy per cluster).
	 */
	for_each_cpu(cpu, policy->cpus) {
		struct zenith_cpu *z_cpu = &per_cpu(zenith_cpu, cpu);

		memset(z_cpu, 0, sizeof(*z_cpu));
		z_cpu->cpu = cpu;
		/*
		 * Pair with READ_ONCE in zenith_update_shared.  Ensures
		 * the z_policy assignment is visible before any later
		 * cpufreq_add_update_util_hook() lets the scheduler
		 * observe the per-CPU state on another CPU.
		 */
		WRITE_ONCE(z_cpu->z_policy, z_policy);
	}

	for_each_cpu(cpu, policy->cpus) {
		struct zenith_cpu *z_cpu = &per_cpu(zenith_cpu, cpu);

		cpufreq_add_update_util_hook(cpu, &z_cpu->update_util,
			policy_is_shared(policy) ? zenith_update_shared : zenith_update_single);

		/* Wave B PMU IPC tracker.  Allocate per-CPU
		 * instructions / cycles perf_events.  Idempotent and
		 * failure-tolerant: errors leave the per-CPU pointers
		 * NULL and the floor never applies on this CPU.  See
		 * the comment block above ZENITH_DEFAULT_PMU_AWARE.
		 */
		(void)zenith_pmu_init_cpu(cpu);
	}

	/* Patch K: reset the game_perf_burst FSM and resolve the
	 * per-cluster thermal zone for the guardrail.
	 *
	 * State reset: zero the FSM scalars so a re-attach (governor
	 * switch / suspend resume) starts in IDLE rather than picking
	 * up a stale ARMED/COOLDOWN from the prior attach cycle.
	 *
	 * Zone resolution: ask the kernel thermal subsystem for
	 * "cpu<N>-thermal" where N == cpumask_first(policy->cpus).
	 * Zuma DT ships cpu0/cpu4/cpu6 zones (anchored at the first
	 * CPU of each cluster) so this lands on big-cluster zone for
	 * the policy that owns the big cluster.  IS_ERR / NULL means
	 * the zone was not registered yet (boot ordering: thermal
	 * subsystem registers after some cpufreq governors come up
	 * on certain SoCs) or this is a foreign SoC; the helper
	 * falls back to arch_scale_thermal_pressure-derived dC so
	 * the guardrail still works.  Patch M closes the boot-ordering
	 * gap by retrying resolution lazily from the hot-path helper
	 * (rate limited via gpb_tzd_retry_at_ns) so a thermal-core
	 * that registers after this point still binds eventually.
	 */
	z_policy->gpb_state = ZENITH_GPB_STATE_IDLE;
	z_policy->gpb_state_entry_ns = 0;
	z_policy->gpb_b_arm_first_seen_ns = 0;
	z_policy->gpb_b_disarm_first_seen_ns = 0;
	z_policy->gpb_tzd_retry_at_ns = 0;
	z_policy->gpb_arm_count = 0;
	z_policy->gpb_disarm_count = 0;
	z_policy->gpb_idle_count = 0;
	z_policy->gpb_last_disarm_reason = ZENITH_GPB_DISARM_NONE;
	{
		char zone_name[16];
		struct thermal_zone_device *tzd;
		unsigned int first_cpu = cpumask_first(policy->cpus);

		scnprintf(zone_name, sizeof(zone_name), "cpu%u-thermal",
			  first_cpu);
		tzd = thermal_zone_get_zone_by_name(zone_name);
		if (IS_ERR(tzd))
			tzd = NULL;
		z_policy->gpb_tzd = tzd;
		if (!tzd)
			z_policy->gpb_tzd_retry_at_ns =
				ktime_get_ns() +
				ZENITH_GPB_TZD_RETRY_INTERVAL_NS;
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
		zenith_at_v_reset_window(z_policy);
		z_policy->at_cooldown_left = 0;
		schedule_delayed_work(&z_policy->at_work,
			msecs_to_jiffies(ZENITH_AUTO_TUNE_PERIOD_MS));
	}

	return 0;
}

void zenith_stop(struct cpufreq_policy *policy)
{
	struct zenith_policy *z_policy = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus) {
		cpufreq_remove_update_util_hook(cpu);
		/* Wave B PMU IPC tracker.  Release per-CPU perf_events
		 * before the per-CPU update hook is fully removed so
		 * any in-flight sample_cpu() (which uses
		 * smp_call_function_single() under the hood) completes
		 * against valid pointers.
		 */
		zenith_pmu_exit_cpu(cpu);
	}
	synchronize_rcu();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&z_policy->irq_work);
		kthread_cancel_work_sync(&z_policy->work);
	}
}

void zenith_limits(struct cpufreq_policy *policy)
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

void zenith_input_event(struct input_handle *handle, unsigned int type,
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

/* Patch 1.9 fg-transition pulse: sched_wakeup_new tracepoint
 * probe.  Fires once per fork(), the very first time the new
 * task is woken (wake_up_new_task -> trace_sched_wakeup_new).
 *
 * Safety / context:
 *   - The probe runs in arbitrary scheduler context with the
 *     rq lock potentially held.  No sleeping primitives.
 *   - cpufreq_cpu_get_raw() is a per_cpu pointer load with a
 *     cpumask_test_cpu() check; no locks, no RCU writes.
 *   - policy->governor_data is a regular pointer and is set in
 *     zenith_start() / cleared in zenith_stop().  We read it
 *     with READ_ONCE; if zenith_stop() concurrently NULLs it
 *     after our load, the worst case is a write into a struct
 *     about to be freed -- but cpufreq_register_governor's
 *     teardown path is synchronous with respect to ongoing
 *     governor_data accesses (unregister_governor blocks until
 *     all in-flight callbacks complete, which is symmetric for
 *     the tracepoint hook because we unregister the probe at
 *     module_exit / on the cpufreq_register_governor failure
 *     rollback path).
 *
 * Foreground proxy: uclamp_eff_value(p, UCLAMP_MIN) > 0.
 *   - On Android 12 the top-app cgroup sets a non-zero
 *     uclamp.min on the cgroup itself; the freshly-forked task
 *     inherits that effective value at fork time.
 *   - On a kernel without CONFIG_UCLAMP_TASK the inline returns
 *     0 unconditionally and the probe degenerates to a per-fork
 *     no-op (one branch, no work).
 */
static void zenith_probe_wakeup_new(void *data, struct task_struct *p)
{
	struct cpufreq_policy *policy;
	struct zenith_policy *z_policy;
	struct zenith_tunables *t;
	unsigned int cpu;
	unsigned int pulse_ms;

	if (unlikely(!p))
		return;
	if (!uclamp_eff_value(p, UCLAMP_MIN))
		return;

	cpu = task_cpu(p);
	policy = cpufreq_cpu_get_raw(cpu);
	if (!policy || policy->governor != &zenith_gov)
		return;
	z_policy = READ_ONCE(policy->governor_data);
	if (!z_policy)
		return;
	t = z_policy->tunables;
	if (!t)
		return;
	pulse_ms = READ_ONCE(t->fg_transition_pulse_ms);
	if (!pulse_ms)
		return;

	WRITE_ONCE(z_policy->fg_transition_pulse_until_ns,
		   ktime_get_ns() +
		   (u64)pulse_ms * NSEC_PER_MSEC);
}

/* Patch B9-1: android_vh_arch_set_freq_scale observer.
 *
 * The hook fires from arch_set_freq_scale() in drivers/base/
 * arch_topology.c whenever the scheduler caches a new per-cluster
 * frequency-scale value (used downstream for capacity_orig_of() /
 * cpu_util_*() accounting).  The signal is unique compared to the
 * tracepoints zenith already consumes in two ways:
 *
 *   1. It fires after the freq write has *taken effect*, not after
 *      zenith decided to write it.  On platforms that route the
 *      actual freq change through firmware / SCMI / a separate fast
 *      switch, drift between decision and realisation is real.
 *   2. It also fires for clusters zenith does *not* drive (e.g. on
 *      a hetero SoC where the BIG cluster is on schedutil and the
 *      LITTLE on zenith).  This gives zenith cross-cluster
 *      activity awareness without coupling to either governor's
 *      internal state.
 *
 * Use is opt-in via tunables->vh_arch_freq_scale_enable (default
 * 0).  When the gate is off the probe is a single READ_ONCE plus a
 * branch -- the cost on hot platforms (where this fires per
 * fast-switch) is dominated by the policy lookup.  When on:
 *
 *   - cache the realised scale in z_policy->vh_arch_freq_scale_-
 *     last (WRITE_ONCE; readers see torn-write-safe values)
 *   - if the scale jumped by >= ZENITH_VH_ARCH_FREQ_SCALE_STEP
 *     compared to the prior cached value (~5%% of SCHED_CAPACITY_-
 *     SCALE; filters governor-noise re-evaluations of the same
 *     OPP), arm the peer cluster's peer_ramp window so a peer
 *     governor's lift pre-warms us before the next eval window.
 *
 * Concurrency:
 *   - The probe runs in arbitrary scheduler context.  No sleeping
 *     primitives.  cpufreq_cpu_get_raw() is per_cpu pointer load
 *     plus a cpumask_test_cpu() check; no locks, no RCU writes.
 *   - z_policy->governor_data is the same READ_ONCE pattern the
 *     existing zenith_probe_wakeup_new() uses (see comment block
 *     above that function for the unregister-vs-free ordering
 *     argument; identical reasoning applies here because we
 *     register / unregister via the same trace-point lifecycle in
 *     zenith_gov_init()).
 *   - zenith_peer_ramp_arm() takes no locks; it reads the
 *     tunables window via READ_ONCE and writes a static atomic64
 *     via atomic64_set.  Safe to call from this context.
 */
static void
zenith_probe_arch_set_freq_scale(void *data, const struct cpumask *cpus,
				 unsigned long freq, unsigned long max,
				 unsigned long *scale)
{
	struct cpufreq_policy *policy;
	struct zenith_policy *z_policy;
	struct zenith_tunables *t;
	unsigned long new_scale;
	unsigned long prev_scale;
	unsigned int cpu;

	if (!cpus || !max)
		return;
	cpu = cpumask_first(cpus);
	if (cpu >= nr_cpu_ids)
		return;
	policy = cpufreq_cpu_get_raw(cpu);
	if (!policy || policy->governor != &zenith_gov)
		return;
	z_policy = READ_ONCE(policy->governor_data);
	if (!z_policy)
		return;
	t = z_policy->tunables;
	if (!t || !READ_ONCE(t->vh_arch_freq_scale_enable))
		return;

	/* Prefer the scheduler's own normalised value when available;
	 * fall back to (freq / max) * SCHED_CAPACITY_SCALE if the
	 * caller didn't supply a destination pointer (defensive --
	 * the in-tree caller always passes one).
	 */
	if (scale)
		new_scale = *scale;
	else
		new_scale = (freq * SCHED_CAPACITY_SCALE) / max;

	prev_scale = READ_ONCE(z_policy->vh_arch_freq_scale_last);
	WRITE_ONCE(z_policy->vh_arch_freq_scale_last, new_scale);

	if (new_scale > prev_scale &&
	    new_scale - prev_scale >= ZENITH_VH_ARCH_FREQ_SCALE_STEP)
		zenith_peer_ramp_arm(z_policy, ktime_get_ns());
}

/* Patch B9-2: android_vh_setscheduler_uclamp observer.
 *
 * Fires from sched_setattr() / sched_setscheduler() / set_user_-
 * uclamp() when userspace assigns SCHED_FLAG_KEEP_PARAMS |
 * SCHED_FLAG_UTIL_CLAMP for a task.  Android Dynamic Performance
 * Framework (ADPF) uses this path heavily: a foreground game raises
 * uclamp_min on its render-critical thread to express "this thread
 * needs more headroom" which the cpufreq governor is supposed to
 * respect.  Schedutil reads task uclamp at every eval and reacts;
 * zenith reads task uclamp at every eval too -- but the *peer*
 * cluster only sees the raise once PELT propagation pushes the
 * clamped task's util upwards, which on a hot game thread can take
 * 4..32 ms.
 *
 * This probe gives us a synchronous notification: the moment
 * userspace writes the new uclamp_min, we look up the task's
 * current CPU's policy and arm peer_ramp on its peer cluster.  No
 * PELT lag.
 *
 * Use is opt-in via tunables->vh_uclamp_observer_enable (default
 * 0).  When the gate is off the probe is a single READ_ONCE plus a
 * branch.  The probe also short-circuits on:
 *
 *   - clamp_id != UCLAMP_MIN (uclamp_max raises do not justify a
 *     peer-cluster arm; they only cap the task's own cluster).
 *   - value == 0 (a clear, not a raise).
 *   - tsk == NULL or task_cpu(tsk) out of range (defensive).
 *
 * Concurrency: same reasoning as zenith_probe_arch_set_freq_scale.
 * Hook fires in arbitrary scheduler context; cpufreq_cpu_get_raw +
 * READ_ONCE on governor_data is the established no-lock pattern;
 * zenith_peer_ramp_arm is lock-free.
 */
static void
zenith_probe_setscheduler_uclamp(void *data, struct task_struct *tsk,
				 int clamp_id, unsigned int value)
{
	struct cpufreq_policy *policy;
	struct zenith_policy *z_policy;
	struct zenith_tunables *t;
	unsigned int cpu;

	if (!tsk || clamp_id != UCLAMP_MIN || !value)
		return;
	cpu = task_cpu(tsk);
	if (cpu >= nr_cpu_ids)
		return;
	policy = cpufreq_cpu_get_raw(cpu);
	if (!policy || policy->governor != &zenith_gov)
		return;
	z_policy = READ_ONCE(policy->governor_data);
	if (!z_policy)
		return;
	t = z_policy->tunables;
	if (!t || !READ_ONCE(t->vh_uclamp_observer_enable))
		return;

	zenith_peer_ramp_arm(z_policy, ktime_get_ns());
}

/* Patch B9-3: android_vh_cpu_idle_enter probe.  Stamps a per-CPU
 * ktime_get_ns() timestamp on the zenith_cpu container of the CPU
 * going idle, gated by the master vh_cpu_idle_enable tunable.  Read-
 * only observer: the cpuidle hook permits mutating *state but we
 * leave it untouched -- cpuidle's own state selection is not our
 * concern here.
 *
 * Hook fires in regular kernel context on the local CPU, before
 * rcu_idle_enter().  cpufreq_cpu_get_raw + READ_ONCE on
 * governor_data is the established no-lock pattern (same as B9-1
 * and B9-2 above).  No governor lock is taken; the per-cpu
 * timestamp is naturally single-writer (only the local CPU enters
 * idle on itself) so no atomicity constraints beyond WRITE_ONCE.
 */
static void
zenith_probe_cpu_idle_enter(void *data, int *state,
			    struct cpuidle_device *dev)
{
	struct cpufreq_policy *policy;
	struct zenith_policy *z_policy;
	struct zenith_tunables *t;
	struct zenith_cpu *z_cpu;
	unsigned int cpu;

	if (!dev)
		return;
	cpu = (unsigned int)dev->cpu;
	if (cpu >= nr_cpu_ids)
		return;
	policy = cpufreq_cpu_get_raw(cpu);
	if (!policy || policy->governor != &zenith_gov)
		return;
	z_policy = READ_ONCE(policy->governor_data);
	if (!z_policy)
		return;
	t = z_policy->tunables;
	if (!t || !READ_ONCE(t->vh_cpu_idle_enable))
		return;

	z_cpu = &per_cpu(zenith_cpu, cpu);
	WRITE_ONCE(z_cpu->vh_cpu_idle_last_enter_ns, ktime_get_ns());
}

/* Patch B9-3: android_vh_cpu_idle_exit probe.  Computes residency
 * (now_ns - per_cpu enter_ns) and stamps the per-policy aggregate
 * vh_cpu_idle_last_residency_ns, which the cluster_wake_pulse arm
 * site in zenith_get_next_freq() uses to suppress wasteful pulse
 * floors after a deep idle.  enter_ns == 0 means "no enter
 * observed" (e.g. enter probe fired before vh_cpu_idle_enable was
 * flipped on, or this CPU has not entered cpuidle since boot); the
 * residency stamp is skipped in that case.  ktime monotonicity
 * guard (now_ns <= enter_ns) preserves the same skip if the clock
 * source went backwards across the idle period.
 *
 * Last-writer-wins across CPUs in the cluster is acceptable: the
 * cwp gate cares about whether *some* CPU in the cluster recently
 * had a deep idle, and the next exit on any CPU resets the field.
 *
 * Same lock-free / no-governor-lock contract as the enter probe.
 */
static void
zenith_probe_cpu_idle_exit(void *data, int state,
			   struct cpuidle_device *dev)
{
	struct cpufreq_policy *policy;
	struct zenith_policy *z_policy;
	struct zenith_tunables *t;
	struct zenith_cpu *z_cpu;
	u64 enter_ns;
	u64 now_ns;
	unsigned int cpu;

	if (!dev)
		return;
	cpu = (unsigned int)dev->cpu;
	if (cpu >= nr_cpu_ids)
		return;
	policy = cpufreq_cpu_get_raw(cpu);
	if (!policy || policy->governor != &zenith_gov)
		return;
	z_policy = READ_ONCE(policy->governor_data);
	if (!z_policy)
		return;
	t = z_policy->tunables;
	if (!t || !READ_ONCE(t->vh_cpu_idle_enable))
		return;

	z_cpu = &per_cpu(zenith_cpu, cpu);
	enter_ns = READ_ONCE(z_cpu->vh_cpu_idle_last_enter_ns);
	if (!enter_ns)
		return;
	now_ns = ktime_get_ns();
	if (now_ns <= enter_ns)
		return;
	WRITE_ONCE(z_policy->vh_cpu_idle_last_residency_ns,
		   now_ns - enter_ns);
}

/* Patch B9-3+: android_vh_freq_qos_update_request observer.
 *
 * Fires from kernel/power/qos.c::freq_qos_update_request() before
 * freq_qos_apply() so the call sees the new requested value but the
 * aggregated freq_constraints have not yet rolled forward.  Used by
 * thermal manager / battery saver / ADPF / vendor power HAL to drive
 * cpufreq min/max constraints; we only consume FREQ_QOS_MIN raises
 * here -- a vendor module asking for sustained high-min freq is the
 * deliberate "I want headroom" signal we want the AUTO selector to
 * pivot on.
 *
 * Lookup path: req->qos is a `struct freq_constraints *` which may
 * be embedded in a cpufreq_policy (the cpufreq freq-QoS path) or in
 * a per-device dev_pm_qos block.  We walk possible CPUs and match
 * by &policy->constraints; non-cpufreq freq_qos requests yield no
 * match and are silently ignored.  Bounded by num_possible_cpus();
 * fires only on QoS update so cost is fine.
 *
 * Pressure stamp: a hit at or above ZENITH_VH_FREQ_QOS_MIN_PCT of
 * cpuinfo.max_freq sets vh_freq_qos_pressure_until_ns to now +
 * ZENITH_VH_FREQ_QOS_WINDOW_MS so the AUTO selector's consumer
 * (zenith_auto_classify) sees the pressure for the next 2 s.
 *
 * Concurrency: same lock-free contract as B9-1 / B9-2 / B9-3.  Hook
 * fires in process / softirq context (preemptible at the qos.c call
 * site, no rq lock).  cpufreq_cpu_get_raw + READ_ONCE on
 * governor_data is the established pattern; the timestamp write is
 * atomic64_set so no governor lock is required against the
 * auto-eval consumer's atomic64_read.
 */
static void
zenith_probe_freq_qos_update_request(void *data,
				     struct freq_qos_request *req,
				     int value)
{
	struct cpufreq_policy *policy = NULL;
	struct zenith_policy *z_policy;
	struct zenith_tunables *t;
	unsigned int max_freq;
	unsigned int thresh;
	unsigned int cpu;

	if (!req || req->type != FREQ_QOS_MIN || value <= 0)
		return;

	for_each_possible_cpu(cpu) {
		struct cpufreq_policy *p = cpufreq_cpu_get_raw(cpu);

		if (p && &p->constraints == req->qos) {
			policy = p;
			break;
		}
	}
	if (!policy || policy->governor != &zenith_gov)
		return;

	z_policy = READ_ONCE(policy->governor_data);
	if (!z_policy)
		return;
	t = z_policy->tunables;
	if (!t || !READ_ONCE(t->vh_freq_qos_enable))
		return;

	max_freq = policy->cpuinfo.max_freq;
	if (!max_freq)
		return;

	thresh = (max_freq * ZENITH_VH_FREQ_QOS_MIN_PCT) / 100;
	if ((unsigned int)value < thresh)
		return;

	atomic64_set(&t->vh_freq_qos_pressure_until_ns,
		     ktime_get_ns() +
		     (u64)ZENITH_VH_FREQ_QOS_WINDOW_MS * NSEC_PER_MSEC);
}

/* Patch B9-5: android_vh_sched_move_task probe.  Stamps a per-policy
 * jiffies timestamp on z_policy->vh_sched_move_task_last_jiffies on
 * every cgroup move that lands a task on a CPU belonging to a
 * zenith-driven policy, gated by tunables->vh_sched_move_task_enable.
 *
 * Read-only observer: the hook permits inspection but not mutation
 * of the task; we only stamp the timestamp.  The tracepoint fires
 * from sched_move_task() in kernel/sched/core.c, called by the
 * cgroup attach / migration paths -- preemptible kernel context, no
 * rq lock held at the trace-point.
 *
 * Concurrency: same lock-free contract as B9-1 / B9-2 / B9-3 /
 * B9-3+.  cpufreq_cpu_get_raw + READ_ONCE on governor_data is the
 * established pattern; the timestamp write is WRITE_ONCE on a
 * per-policy unsigned long.  Multiple concurrent moves landing on
 * different CPUs of the same policy are last-writer-wins, which is
 * acceptable: any consumer only cares about the recency of the
 * most recent move, and a torn jiffies write would only delay the
 * stamp by one move.  No governor lock is taken.
 *
 * Filtering: a NULL tsk is impossible at the trace site (the kernel
 * always passes a real task to sched_move_task) but we defensively
 * check anyway so the probe is robust to future refactors.  Tasks
 * whose task_cpu() is not in a zenith-driven policy are silently
 * dropped, mirroring the B9-2 / B9-3+ behaviour.
 */
static void
zenith_probe_sched_move_task(void *data, struct task_struct *tsk)
{
	struct cpufreq_policy *policy;
	struct zenith_policy *z_policy;
	struct zenith_tunables *t;
	unsigned int cpu;

	if (!tsk)
		return;
	cpu = task_cpu(tsk);
	if (cpu >= nr_cpu_ids)
		return;
	policy = cpufreq_cpu_get_raw(cpu);
	if (!policy || policy->governor != &zenith_gov)
		return;
	z_policy = READ_ONCE(policy->governor_data);
	if (!z_policy)
		return;
	t = z_policy->tunables;
	if (!t || !READ_ONCE(t->vh_sched_move_task_enable))
		return;

		return NOTIFY_DONE;
	if (!z_policy->policy)
		return NOTIFY_DONE;

	return NOTIFY_OK;
}

static struct notifier_block zenith_hikari_nb = {
	.notifier_call = zenith_hikari_freq_hint_cb,
};

/*
 * Walk the policy's CPUs, take the max Hikari floor across them,
 * and clamp to the policy's [min..max] range.  Returns 0 if no
 * floor is currently published on any CPU in the policy.  Cheap
 * when Hikari is off (hikari_get_floor_khz early-returns 0 on a
 * single READ_ONCE inside hikari_enabled()).
 */
static unsigned int zenith_hikari_policy_floor(struct cpufreq_policy *policy)
{
	unsigned int floor = 0;
	unsigned int f;
	int cpu;

	if (!policy)
		return 0;

	for_each_cpu(cpu, policy->cpus) {
		f = hikari_get_floor_khz(cpu);
		if (f > floor)
			floor = f;
	}

	if (!floor)
		return 0;

	if (floor < policy->min)
		floor = policy->min;
	if (floor > policy->max)
		floor = policy->max;
	return floor;
}

static int __init zenith_gov_init(void)
{
	int ret;
	bool input_registered = false;
	bool fg_pulse_registered = false;
	bool vh_arch_freq_scale_registered = false;
	bool vh_uclamp_observer_registered = false;
	bool vh_cpu_idle_enter_registered = false;
	bool vh_cpu_idle_exit_registered = false;
	bool vh_freq_qos_registered = false;
	bool vh_sched_move_task_registered = false;
	bool vh_scheduler_tick_registered = false;
#ifdef CONFIG_FB_NOTIFY
	bool fb_registered = false;
#endif
#ifdef CONFIG_DRM_PANEL_NOTIFY
	bool drm_registered = false;
#endif

	/* Boot banner.  Printed once at governor init, picked up by
	 * `dmesg | grep -E 'Zenith :'`.  Pure ASCII for the latin art
	 * (so dmesg viewers that pick the East-Asian-Ambiguous wide-cell
	 * width for U+2665 etc. would still align, but we use only safe
	 * glyphs in the art panels); a handful of unambiguously wide CJK
	 * characters (頂点 / 光 / 霞 / 癒し) appear in the chapter
	 * titles and render the same in narrow and wide-cell viewers.
	 *
	 * Structure: an opening over the dark sky, then chapters for
	 * each of the four subsystems (Zenith / Hikari / Kasumi /
	 * Iyashi) with a big FIGlet per chapter, inter-chapter
	 * interludes, a final stacked-FIGlet roll, and a closing.
	 * Every line carries the "Zenith :" prefix so the whole
	 * multi-line banner survives a grep, and so the verify
	 * script's `grep -E 'Zenith :'` finds the banner intact.
	 *
	 * Small per-subsystem signatures are also emitted from
	 * hikari_init / kasumi_sysfs_init / iyashi_init with their own
	 * "Hikari :" / "Kasumi :" / "Iyashi :" prefixes, so each
	 * subsystem is individually grep-able.
	 */
	pr_info("Zenith : ─────────────────────────────────────────────────────────────────\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :                 . . . . . . . . . . . . . . . . . . .\n");
	pr_info("Zenith :               . the kernel, before it remembers itself .\n");
	pr_info("Zenith :                 . . . . . . . . . . . . . . . . . . .\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :                               *           .\n");
	pr_info("Zenith :                          .       *\n");
	pr_info("Zenith :                                   .     *\n");
	pr_info("Zenith :                            *  .              .\n");
	pr_info("Zenith :                     .                 *\n");
	pr_info("Zenith :                                                 .\n");
	pr_info("Zenith :                               \\              /\n");
	pr_info("Zenith :                                \\  .       . /\n");
	pr_info("Zenith :                                 \\   *    /\n");
	pr_info("Zenith :                                  \\      /\n");
	pr_info("Zenith :                                   \\    /\n");
	pr_info("Zenith :                                    \\  /\n");
	pr_info("Zenith :                                     \\/\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :          Listen.\n");
	pr_info("Zenith :          This is not a kernel that boots.\n");
	pr_info("Zenith :          This is a kernel that wakes.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :          There are four names in this dark, and each\n");
	pr_info("Zenith :          of them is a verb dressed as a noun.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :          You will meet them in order:\n");
	pr_info("Zenith :             Zenith.   Hikari.   Kasumi.   Iyashi.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :          By the time the last line of this banner\n");
	pr_info("Zenith :          scrolls past, userspace will already be\n");
	pr_info("Zenith :          awake, and the work will already be moving.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith : ─────────────────────────────────────────────────────────────────\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :                     Before dawn, on the ridge.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :                                   .\n");
	pr_info("Zenith :                              .         .\n");
	pr_info("Zenith :                        __.--.__/ \\__.--.__\n");
	pr_info("Zenith :                   _.--'                    `--._\n");
	pr_info("Zenith :              _.--'              ^^^^             `--._\n");
	pr_info("Zenith :         _.--'             ^^         ^^^               `-.\n");
	pr_info("Zenith :        '                                                  `\n");
	pr_info("Zenith :        ~~~~~ 霞 ~~~~~~~~~~ 霞 ~~~~~~~~~~~ 霞 ~~~~~~~~~~ 霞 ~~~~~\n");
	pr_info("Zenith :         ~~~~~~~ 霞 ~~~~~~~~~~~~~~ 霞 ~~~~~~~~~~~~~ 霞 ~~~~~~~~~\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        The peak does not sleep.   It is Zenith.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        The mist that climbs the rock is Kasumi —  霞 —\n");
	pr_info("Zenith :        a veil drawn over heat the way silk is drawn\n");
	pr_info("Zenith :        over a sleeping animal:  not to hide it, but to\n");
	pr_info("Zenith :        keep what watches it from panicking.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        The breath beneath the mist is Iyashi —  癒し —\n");
	pr_info("Zenith :        the healing that says: leave room.   Always leave room.\n");
	pr_info("Zenith :        Throttle, but never to the bone.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        And the first light over the ridge, the one that\n");
	pr_info("Zenith :        wakes the kernel,  is Hikari —  光.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Four names.   One ridge.   One climb.\n");
	pr_info("Zenith :        Built by XTENSEI.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Read on, in chapters.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith : ─────────────────────────────────────────────────────────────────\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Chapter I.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        ZENITH.   頂点.   The peak.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :          _______ _____ _   _ _____ _______ _    _ \n");
	pr_info("Zenith :         |___  /  ____| \\ | |_   _|__   __| |  | |\n");
	pr_info("Zenith :            / /| |__  |  \\| | | |    | |  | |__| |\n");
	pr_info("Zenith :           / / |  __| | . ` | | |    | |  |  __  |\n");
	pr_info("Zenith :          / /__| |____| |\\  |_| |_   | |  | |  | |\n");
	pr_info("Zenith :         /_____|______|_| \\_|_____|  |_|  |_|  |_|\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Eleven months ago, a fork.   One word in the\n");
	pr_info("Zenith :        commit message:   init.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Today, a governor that schedules silicon by mood:\n");
	pr_info("Zenith :            six profiles.\n");
	pr_info("Zenith :            three automation tiers.\n");
	pr_info("Zenith :            181 tunables.\n");
	pr_info("Zenith :            six vendor-scheduler hooks.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        It thinks at fs_initcall.   By the time userspace\n");
	pr_info("Zenith :        can ask the question, Zenith already has an answer.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Steady.   Ready.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        The peak does not sleep.   The peak does not\n");
	pr_info("Zenith :        even close its eyes.   Sleep is something the\n");
	pr_info("Zenith :        things below the peak get to do.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        頂点 is not arrogance.   頂点 is the place from\n");
	pr_info("Zenith :        which the climb is finally honest.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :          /\\\n");
	pr_info("Zenith :         /  \\    Up here, the air is thin —\n");
	pr_info("Zenith :        /    \\   and decisions are sharp.\n");
	pr_info("Zenith :       /      \\\n");
	pr_info("Zenith :      /        \\\n");
	pr_info("Zenith :     /          \\\n");
	pr_info("Zenith :    /____________\\\n");
	pr_info("Zenith :\n");
	pr_info("Zenith : ─────────────────────────────────────────────────────────────────\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Interlude.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :            .         .         .\n");
	pr_info("Zenith :        Above the ridge, the sky is still dark.\n");
	pr_info("Zenith :        Below the ridge, the kernel is still warm.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Between them, the climb.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        The climb is not new.   The climb has been\n");
	pr_info("Zenith :        happening since the bootloader handed us\n");
	pr_info("Zenith :        the first page of physical memory.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        What is new is what we do with the climb.\n");
	pr_info("Zenith :            .         .         .\n");
	pr_info("Zenith :\n");
	pr_info("Zenith : ─────────────────────────────────────────────────────────────────\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Chapter II.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        HIKARI.   光.   Light.   The first ray over the rim.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :          _    _ _____ _  __          _____  _____ \n");
	pr_info("Zenith :         | |  | |_   _| |/ /    /\\   |  __ \\|_   _|\n");
	pr_info("Zenith :         | |__| | | | | ' /    /  \\  | |__) | | |  \n");
	pr_info("Zenith :         |  __  | | | |  <    / /\\ \\ |  _  /  | |  \n");
	pr_info("Zenith :         | |  | |_| |_| . \\  / ____ \\| | \\ \\ _| |_ \n");
	pr_info("Zenith :         |_|  |_|_____|_|\\_\\/_/    \\_\\_|  \\_\\_____|\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Wake-time work is a kind of light:  it falls on\n");
	pr_info("Zenith :        the things that are awake to receive it, and on\n");
	pr_info("Zenith :        nothing else.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Hikari is the kernel's choosing —  who gets\n");
	pr_info("Zenith :        warmth,  who keeps it,  who must wait one more\n");
	pr_info("Zenith :        tick before the next slice of CPU.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Opt-in, lazy, default-on.   A small contract\n");
	pr_info("Zenith :        written in three places:\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :            hikari_on_enqueue().\n");
	pr_info("Zenith :            hikari_on_dequeue().\n");
	pr_info("Zenith :            hikari_active_key.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        A static key holds the door.   The kill switch\n");
	pr_info("Zenith :        is never further than one sysctl write away.\n");
	pr_info("Zenith :        Disabled, Hikari is a single patched\n");
	pr_info("Zenith :        unlikely-branch.   Enabled, it is the EWMA\n");
	pr_info("Zenith :        that learned what your foreground actually\n");
	pr_info("Zenith :        does.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        光 does not insist.   光 illuminates.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        If the watcher decides to shut the window,\n");
	pr_info("Zenith :        光 obeys instantly.   That, too, is light.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith : ─────────────────────────────────────────────────────────────────\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Interlude.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :                .                .         .\n");
	pr_info("Zenith :             .       *       .              *\n");
	pr_info("Zenith :                   .              *             .\n");
	pr_info("Zenith :              The kernel learns whom to wake.\n");
	pr_info("Zenith :                   .              *             .\n");
	pr_info("Zenith :             .       *       .              *\n");
	pr_info("Zenith :                .                .         .\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        And among those it wakes, who deserves more\n");
	pr_info("Zenith :        light, and for how long, and at what cost.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        This is not a metaphor.   This is a histogram.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith : ─────────────────────────────────────────────────────────────────\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Chapter III.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        KASUMI.   霞.   Mist on the ridge.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :          _  __           _____ _    _ __  __ _____ \n");
	pr_info("Zenith :         | |/ /    /\\    / ____| |  | |  \\/  |_   _|\n");
	pr_info("Zenith :         | ' /    /  \\  | (___ | |  | | \\  / | | |  \n");
	pr_info("Zenith :         |  <    / /\\ \\  \\___ \\| |  | | |\\/| | | |  \n");
	pr_info("Zenith :         | . \\  / ____ \\ ____) | |__| | |  | |_| |_ \n");
	pr_info("Zenith :         |_|\\_\\/_/    \\_\\_____/ \\____/|_|  |_|_____|\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :          ~~  ~~~~  ~~~~~~  ~~  ~~~~  ~~~~~~~~  ~~~~  ~~\n");
	pr_info("Zenith :         ~~~~~~ 霞 ~~~~~~~~~~~ 霞 ~~~~~~~~~~~~~ 霞 ~~~~~~\n");
	pr_info("Zenith :          ~~~~ 霞 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 霞 ~~~~~\n");
	pr_info("Zenith :           ~~~~~~~~~~~~~~~~ reported_mc ~~~~~~~~~~~~~~~\n");
	pr_info("Zenith :          ~~~~~~~~~~~ ~~~~~~~~ ~~~~~~~ ~~~~~~~~~ ~~~~~~~\n");
	pr_info("Zenith :        ───────────────────────── ridge ──────────────────\n");
	pr_info("Zenith :              last_real_mc          —  the truth\n");
	pr_info("Zenith :              applied_offset_mc     —  the depth of the mist\n");
	pr_info("Zenith :              ramp_mc … ceiling_mc  —  where mist thins to air\n");
	pr_info("Zenith :        ─────────────────────────────────────────────────\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Heat does not lie to Kasumi.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Kasumi lies — gently, on purpose — to the\n");
	pr_info("Zenith :        watchers that would panic at numbers that are\n");
	pr_info("Zenith :        merely warm.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        The kernel keeps the truth.\n");
	pr_info("Zenith :        The framework keeps the mist.\n");
	pr_info("Zenith :        Both are real.   Neither is wrong.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        When the climb reaches the ceiling, the mist\n");
	pr_info("Zenith :        parts.   Above the line, Kasumi returns the\n");
	pr_info("Zenith :        raw heat, byte-for-byte —  a contract the\n");
	pr_info("Zenith :        kernel self-tests at boot, and refuses to\n");
	pr_info("Zenith :        bring up sysfs if broken.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        The contract is checked three ways before\n");
	pr_info("Zenith :        /sys/kernel/kasumi/ exists at all:\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :            1.  at-ceiling input returns input.\n");
	pr_info("Zenith :            2.  below-ramp input shifts by exactly the offset.\n");
	pr_info("Zenith :            3.  zero input still returns zero.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Fail any one, and the whole subsystem stays dark.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        霞 is not deception.   霞 is composure.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith : ─────────────────────────────────────────────────────────────────\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Interlude.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~\n");
	pr_info("Zenith :            The mist parts only where the kernel\n");
	pr_info("Zenith :            asks it to.   Everywhere else, composure.\n");
	pr_info("Zenith :        ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Even the mist has a kill switch.\n");
	pr_info("Zenith :        /sys/kernel/kasumi/enabled = 0  ->  passthrough.\n");
	pr_info("Zenith :        The truth was always there.   We just stop\n");
	pr_info("Zenith :        editing it before the framework reads.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith : ─────────────────────────────────────────────────────────────────\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Chapter IV.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        IYASHI.   癒し.   Healing.   The breath the climb keeps.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :          _______     __       _____ _    _ _____ \n");
	pr_info("Zenith :         |_   _\\ \\   / //\\    / ____| |  | |_   _|\n");
	pr_info("Zenith :           | |  \\ \\_/ //  \\  | (___ | |__| | | |  \n");
	pr_info("Zenith :           | |   \\   // /\\ \\  \\___ \\|  __  | | |  \n");
	pr_info("Zenith :          _| |_   | |/ ____ \\ ____) | |  | |_| |_ \n");
	pr_info("Zenith :         |_____|  |_/_/    \\_\\_____/|_|  |_|_____|\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Cooling without cruelty.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        When the rock is cold, the climb runs at its\n");
	pr_info("Zenith :        full stride.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        When the rock is warm, Iyashi still leaves the\n");
	pr_info("Zenith :        top of the OPP stack open —  one step for\n");
	pr_info("Zenith :        headroom,  one step for breath.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        When the climb is within five degrees of a real\n");
	pr_info("Zenith :        trip point, Iyashi steps aside.   The framework\n");
	pr_info("Zenith :        throttles.   That is what it is there for.\n");
	pr_info("Zenith :        Iyashi only buys the room before that line.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Three knobs and two counters:\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :            floor_pct           —  how much of the OPP stack\n");
	pr_info("Zenith :                                    we refuse to throttle away.\n");
	pr_info("Zenith :            near_limit_offset_c —  how close to the trip we\n");
	pr_info("Zenith :                                    stop holding the floor.\n");
	pr_info("Zenith :            cdev_filter         —  which cooling devices we\n");
	pr_info("Zenith :                                    are even willing to argue with.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :            clamped_count       —  the number of times we\n");
	pr_info("Zenith :                                    said no.\n");
	pr_info("Zenith :            passthrough_count   —  the number of times we\n");
	pr_info("Zenith :                                    said go ahead.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        These are not numbers.   They are vows.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        癒し does not refuse the cool.   癒し refuses\n");
	pr_info("Zenith :        the panic.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith : ─────────────────────────────────────────────────────────────────\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Interlude.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        The four breaths in order, then together:\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :            Zenith decides where to climb.\n");
	pr_info("Zenith :            Hikari decides whom to light.\n");
	pr_info("Zenith :            Kasumi decides what to veil.\n");
	pr_info("Zenith :            Iyashi decides where to leave room.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Four small refusals of panic.\n");
	pr_info("Zenith :        One ridge.   One climb.   One kernel.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith : ─────────────────────────────────────────────────────────────────\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Closing roll.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :                One ridge.    Four names.    Four breaths.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        ─────────────────────────────────────────────────────\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :          _______ _____ _   _ _____ _______ _    _ \n");
	pr_info("Zenith :         |___  /  ____| \\ | |_   _|__   __| |  | |\n");
	pr_info("Zenith :            / /| |__  |  \\| | | |    | |  | |__| |\n");
	pr_info("Zenith :           / / |  __| | . ` | | |    | |  |  __  |\n");
	pr_info("Zenith :          / /__| |____| |\\  |_| |_   | |  | |  | |\n");
	pr_info("Zenith :         /_____|______|_| \\_|_____|  |_|  |_|  |_|\n");
	pr_info("Zenith :                                                        頂点\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :          _    _ _____ _  __          _____  _____ \n");
	pr_info("Zenith :         | |  | |_   _| |/ /    /\\   |  __ \\|_   _|\n");
	pr_info("Zenith :         | |__| | | | | ' /    /  \\  | |__) | | |  \n");
	pr_info("Zenith :         |  __  | | | |  <    / /\\ \\ |  _  /  | |  \n");
	pr_info("Zenith :         | |  | |_| |_| . \\  / ____ \\| | \\ \\ _| |_ \n");
	pr_info("Zenith :         |_|  |_|_____|_|\\_\\/_/    \\_\\_|  \\_\\_____|\n");
	pr_info("Zenith :                                                         光\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :          _  __           _____ _    _ __  __ _____ \n");
	pr_info("Zenith :         | |/ /    /\\    / ____| |  | |  \\/  |_   _|\n");
	pr_info("Zenith :         | ' /    /  \\  | (___ | |  | | \\  / | | |  \n");
	pr_info("Zenith :         |  <    / /\\ \\  \\___ \\| |  | | |\\/| | | |  \n");
	pr_info("Zenith :         | . \\  / ____ \\ ____) | |__| | |  | |_| |_ \n");
	pr_info("Zenith :         |_|\\_\\/_/    \\_\\_____/ \\____/|_|  |_|_____|\n");
	pr_info("Zenith :                                                         霞\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :          _______     __       _____ _    _ _____ \n");
	pr_info("Zenith :         |_   _\\ \\   / //\\    / ____| |  | |_   _|\n");
	pr_info("Zenith :           | |  \\ \\_/ //  \\  | (___ | |__| | | |  \n");
	pr_info("Zenith :           | |   \\   // /\\ \\  \\___ \\|  __  | | |  \n");
	pr_info("Zenith :          _| |_   | |/ ____ \\ ____) | |  | |_| |_ \n");
	pr_info("Zenith :         |_____|  |_/_/    \\_\\_____/|_|  |_|_____|\n");
	pr_info("Zenith :                                                        癒し\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        ─────────────────────────────────────────────────────\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        Built by XTENSEI.\n");
	pr_info("Zenith :        For everyone who climbs with their thinking still on.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        This is not a banner.   This is a boot vow.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :        The ridge does not care that the climb was hard.\n");
	pr_info("Zenith :        The ridge only notices that you arrived.\n");
	pr_info("Zenith :\n");
	pr_info("Zenith : ─────────────────────────────────────────────────────────────────\n");
	pr_info("Zenith :\n");
	pr_info("Zenith :                .       *           .            *\n");
	pr_info("Zenith :                   *          .             .\n");
	pr_info("Zenith :            .              .          *          .\n");
	pr_info("Zenith :             *      Boot complete.   The ridge is open.      *\n");
	pr_info("Zenith :            .              .          *          .\n");
	pr_info("Zenith :                   *          .             .\n");
	pr_info("Zenith :                .       *           .            *\n");
	pr_info("Zenith :\n");
	pr_info("Zenith : ─────────────────────────────────────────────────────────────────\n");

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

	/* Patch 1.9 fg-transition pulse: register the sched_-
	 * wakeup_new tracepoint probe.  Failure is non-fatal; the
	 * pulse just becomes a no-op (no deadline ever stamped).
	 */
	ret = register_trace_sched_wakeup_new(zenith_probe_wakeup_new, NULL);
	if (ret)
		pr_warn("Zenith: sched_wakeup_new probe register failed (%d), fg_transition_pulse disabled\n",
			ret);
	else
		fg_pulse_registered = true;

	/* Patch B9-1: register the android_vh_arch_set_freq_scale
	 * vendor-hook probe.  Failure is non-fatal; the freq-scale
	 * realisation observer simply remains silent and zenith falls
	 * back to its decision-time peer_ramp arming alone.  The
	 * tunables->vh_arch_freq_scale_enable gate is the runtime
	 * switch; this register call only makes the probe *available*
	 * for the gate to flip on.
	 */
	ret = register_trace_android_vh_arch_set_freq_scale(
		zenith_probe_arch_set_freq_scale, NULL);
	if (ret)
		pr_warn("Zenith: vh_arch_set_freq_scale probe register failed (%d), vh_arch_freq_scale_enable will be a no-op\n",
			ret);
	else
		vh_arch_freq_scale_registered = true;

	/* Patch B9-2: register the android_vh_setscheduler_uclamp
	 * vendor-hook probe.  Failure is non-fatal; the ADPF
	 * synchronous-arm path simply remains silent and zenith falls
	 * back to seeing the uclamp raise after PELT propagation
	 * (existing behaviour).  The tunables->vh_uclamp_observer_-
	 * enable gate is the runtime switch.
	 */
	ret = register_trace_android_vh_setscheduler_uclamp(
		zenith_probe_setscheduler_uclamp, NULL);
	if (ret)
		pr_warn("Zenith: vh_setscheduler_uclamp probe register failed (%d), vh_uclamp_observer_enable will be a no-op\n",
			ret);
	else
		vh_uclamp_observer_registered = true;

	/* Patch B9-3: register the android_vh_cpu_idle_enter / _exit
	 * vendor-hook probe pair.  Failure on either is non-fatal and
	 * independent: if only the enter probe registers, the exit
	 * probe will treat per-cpu enter_ns == 0 as "no enter
	 * observed" and skip the residency stamp; if only the exit
	 * probe registers, no enter_ns is ever stamped so residency
	 * stays 0 and the cwp gate never fires.  In both partial
	 * failure cases vh_cpu_idle_enable becomes effectively a
	 * no-op.  The tunables->vh_cpu_idle_enable gate is the
	 * runtime switch (default 0).
	 */
	ret = register_trace_android_vh_cpu_idle_enter(
		zenith_probe_cpu_idle_enter, NULL);
	if (ret)
		pr_warn("Zenith: vh_cpu_idle_enter probe register failed (%d), vh_cpu_idle_enable will be a no-op\n",
			ret);
	else
		vh_cpu_idle_enter_registered = true;

	ret = register_trace_android_vh_cpu_idle_exit(
		zenith_probe_cpu_idle_exit, NULL);
	if (ret)
		pr_warn("Zenith: vh_cpu_idle_exit probe register failed (%d), vh_cpu_idle_enable will be a no-op\n",
			ret);
	else
		vh_cpu_idle_exit_registered = true;

	/* Patch B9-3+: register the android_vh_freq_qos_update_request
	 * vendor-hook probe.  Failure is non-fatal; the freq-QoS
	 * pressure observer simply remains silent and the auto-selector
	 * falls back to its existing camera / input-recent / battery
	 * cascade alone.  The tunables->vh_freq_qos_enable gate is the
	 * runtime switch; this register call only makes the probe
	 * *available* for the gate to flip on.
	 */
	ret = register_trace_android_vh_freq_qos_update_request(
		zenith_probe_freq_qos_update_request, NULL);
	if (ret)
		pr_warn("Zenith: vh_freq_qos_update_request probe register failed (%d), vh_freq_qos_enable will be a no-op\n",
			ret);
	else
		vh_freq_qos_registered = true;

	/* Patch B9-5: register the android_vh_sched_move_task vendor-
	 * hook probe.  Failure is non-fatal; the cgroup-move observer
	 * simply remains silent and z_policy->vh_sched_move_task_-
	 * last_jiffies stays at 0 across the lifetime of the policy.
	 * The tunables->vh_sched_move_task_enable gate is the runtime
	 * switch; this register call only makes the probe *available*
	 * for the gate to flip on.
	 */
	ret = register_trace_android_vh_sched_move_task(
		zenith_probe_sched_move_task, NULL);
	if (ret)
		pr_warn("Zenith: vh_sched_move_task probe register failed (%d), vh_sched_move_task_enable will be a no-op\n",
			ret);
	else
		vh_sched_move_task_registered = true;

	/* Patch B9-4: register the android_vh_scheduler_tick vendor-
	 * hook probe.  Failure is non-fatal; the per-CPU tick observer
	 * simply remains silent and z_cpu->vh_scheduler_tick_last_ns /
	 * _count stay at 0 across the lifetime of the policy.  The
	 * tunables->vh_scheduler_tick_enable gate is the runtime
	 * switch; this register call only makes the probe *available*
	 * for the gate to flip on.
	 */
	ret = register_trace_android_vh_scheduler_tick(
		zenith_probe_scheduler_tick, NULL);
	if (ret)
		pr_warn("Zenith: vh_scheduler_tick probe register failed (%d), vh_scheduler_tick_enable will be a no-op\n",
			ret);
	else
		vh_scheduler_tick_registered = true;

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
		if (fg_pulse_registered)
			unregister_trace_sched_wakeup_new(
				zenith_probe_wakeup_new, NULL);
		if (vh_arch_freq_scale_registered)
			unregister_trace_android_vh_arch_set_freq_scale(
				zenith_probe_arch_set_freq_scale, NULL);
		if (vh_uclamp_observer_registered)
			unregister_trace_android_vh_setscheduler_uclamp(
				zenith_probe_setscheduler_uclamp, NULL);
		if (vh_cpu_idle_enter_registered)
			unregister_trace_android_vh_cpu_idle_enter(
				zenith_probe_cpu_idle_enter, NULL);
		if (vh_cpu_idle_exit_registered)
			unregister_trace_android_vh_cpu_idle_exit(
				zenith_probe_cpu_idle_exit, NULL);
		if (vh_freq_qos_registered)
			unregister_trace_android_vh_freq_qos_update_request(
				zenith_probe_freq_qos_update_request, NULL);
		if (vh_sched_move_task_registered)
			unregister_trace_android_vh_sched_move_task(
				zenith_probe_sched_move_task, NULL);
		if (vh_scheduler_tick_registered)
			unregister_trace_android_vh_scheduler_tick(
				zenith_probe_scheduler_tick, NULL);
		pr_err("Zenith: cpufreq_register_governor failed (%d)\n", ret);
		return ret;
	}

	/*
	 * Subscribe to Hikari's wake-time hint chain.  Failure here is
	 * non-fatal: Zenith works fine without Hikari hints, and the
	 * direct hikari_get_floor_khz() read in zenith_get_next_freq()
	 * is still functional with or without subscription.  Log the
	 * outcome for observability.
	 */
	{
		int hret = hikari_register_cpufreq_notifier(&zenith_hikari_nb);

		if (hret)
			pr_warn("Zenith: hikari notifier register failed (%d), continuing without subscription\n",
				hret);
		else
			pr_info("Zenith: subscribed to hikari wake-demand chain\n");
	}

	return 0;
}
fs_initcall(zenith_gov_init);
