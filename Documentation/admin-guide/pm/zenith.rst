.. SPDX-License-Identifier: GPL-2.0

===========================
The Zenith CPUFreq Governor
===========================

:Author: ENI (V1, exclusively for LO)
:Maintainer: XTENSEI (V2/V3 round-U, exclusively for android12-5.10 kernels)

``zenith`` is a custom hybrid CPUFreq governor that combines techniques
from ``schedutil`` (EAS proportional scaling), ``ondemand`` (hard-
threshold snap to maximum), ``schedhorizon`` (efficient-frequency delay
ladder) and the legacy interactive governor (hispeed bias / climb-mode),
and adds an explicit input-event boost path on top.  It is built for
interactive-first Android UI on modern ARM flagship SoCs.

It lives in ``kernel/sched/cpufreq_zenith.c`` and is selected with
``CONFIG_CPU_FREQ_GOV_ZENITH=y``.  To make it the default at boot, set
``CONFIG_CPU_FREQ_DEFAULT_GOV_ZENITH=y`` in your defconfig.

A companion userspace integration script lives in
``zenith-docs/init.zenith.rc``; the broader phone-class kernel stack
(BORE, KSM, ADIOS, CAKE) is documented in
``zenith-docs/zenith-history.rst``.

Decision chain
==============

On every ``cpufreq_governor_update_util()`` callback, ``zenith`` walks
the following chain in ``zenith_get_next_freq()``::

    0. Input Boost          → pin to policy->max for input_boost_ms
                              after any EV_KEY/EV_ABS/EV_REL event
                              (with optional cubic-eased decay)
    1. Ondemand Brutality   → pin to policy->max once load crosses
                              up_threshold (or up_threshold_hispeed when
                              already above hispeed_freq), stay pinned
                              until load drops below down_threshold
                              (hysteresis, with optional entry streak)
    2. Hispeed Ladder       → if hispeed_freq is set and load crosses
                              hispeed_load, climb toward hispeed_freq
                              using climb_mode (snap or step, with
                              entry streak + exit hysteresis)
    3. EAS Proportional     → map_util_freq(util, max_freq, max_cap)
                              with optional capacity headroom; util
                              source picked by util_math_v2 and
                              optionally blended with the kcpustat
                              hispeed signal
    4. Render / Audio /     → per-tier floors and caps when the
       Camera / Game /        corresponding awareness flag is set
       PSI / Frame Pace
    5. Powersave Bias       → subtract powersave_bias / 1000 from freq,
                              but only when load < bias_load_threshold
    6. uclamp_min Floor     → ADPF / cgroup uclamp_min raised to floor
    7. Cache Resolve        → cpufreq_driver_resolve_freq()
    8. Efficient Cap        → hold at efficient_freq until the request
                              has been sustained for up_delay_us;
                              up to ZENITH_EFF_BINS_MAX bins with
                              individual delays and exit hysteresis
    9. Light-Load Cap       → hard-clamp to light_load_freq when load
                              is below light_load_threshold
   10. EM / Thermal Cap     → drop the top turbo bin when thermal_state
                              is set, derated by thermal_util_derate
   11. Sampling Down        → extend down_rate_limit by
                              sampling_down_factor while at max
                              (with optional boost_exit_extend)

The input-boost and brutality paths ``goto resolve``, skipping steps
2-6; all paths pass through the remaining steps.

Tracepoints
===========

``zenith`` ships its own trace event header at
``include/trace/events/cpufreq_zenith.h``.  All events are gated on the
runtime tracing-enabled state of their corresponding tracepoint, so the
fast-path cost is one ``static_branch`` test per event::

    zenith_decision         decision-tier breadcrumb (path, freq, load)
    zenith_auto_tune        per-window classifier sample
    zenith_auto_tune_v2     v2 state-machine transitions
    zenith_auto_tune_scenario  scenario classification (game/audio/...)
    zenith_thermal_derate   thermal-pressure-derived util scaling
    zenith_predict          two-tap smoothed predicted util
    zenith_render_floor     render-tier floor application
    zenith_frame_pace       frame-pace floor application
    zenith_audio_band       audio-tier floor / cap application
    zenith_camera_floor     camera-tier floor application
    zenith_game_mode        game-mode tier transitions

Tunables
========

All tunables are exposed under
``/sys/devices/system/cpu/cpufreq/zenith/``.  Writes are range-checked
and return ``-EINVAL`` on out-of-range input.  Defaults below are the
kernel-side defaults; the ``profile`` selector overrides several of
them at once when written.

Rate limiting
-------------

``up_rate_limit_us``
    Minimum time (µs) between two consecutive upward frequency changes.
    Default 500.  Lower = more responsive, more DVFS churn.

``down_rate_limit_us``
    Minimum time (µs) between two consecutive downward frequency
    changes.  Default 2000.  Higher = sticks at elevated frequencies
    longer, lower battery.

``sampling_down_factor``
    While ``target_freq`` is at ``policy->max``, the effective
    ``down_rate_limit_us`` is multiplied by this factor.  Stops
    ping-pong between max and the bin just below max under sustained
    heavy load.  Default 1 (disabled), range 1..10.

``rate_limit_cluster_scale``
    Boolean (0/1, default 0).  When 1, the little cluster gets tighter
    rate limits than the big cluster (rate divided by an internal
    cluster-scale factor) so it tracks short bursts more aggressively
    without affecting the big cluster's DVFS budget.

``down_rate_adaptive``
    Boolean (0/1, default 0).  When 1, recent load variance widens the
    effective ``down_rate_limit_us`` by up to 2× on bursty workloads,
    preserving freq between bursts so the next burst doesn't pay a
    DVFS-ramp penalty.

``freq_stability_margin_pct``
    Holds the current frequency when the new target falls within this
    percent of ``policy->max`` *below* current.  Prevents tiny single-
    bin oscillations at OPP boundaries.  Default 0 (disabled), range
    0..10.

Load thresholds
---------------

``up_threshold``
    Load (in % of CPU capacity) at or above which ``zenith`` jumps
    straight to ``policy->max`` (ondemand-style).  Default 80, range
    1..100.

``up_threshold_adaptive``
    Variance-adaptive shaping: lowers ``dynamic_up_thresh`` by up to
    this percent of its value when the recent load signal is bursty.
    Default 0 (disabled), range 0..ZENITH_UP_THRESHOLD_ADAPTIVE_MAX.

``up_threshold_hispeed``
    Optional second up-threshold that applies only after the policy is
    already above ``hispeed_freq``.  Used to make the climb above
    hispeed require a stronger sustained load than the climb to
    hispeed itself.  0 = disabled (default).  Range 0..100.

``down_threshold``
    Load (%) at or below which the brutality lock is released, letting
    EAS proportional scaling take over again.  Creates a hysteresis
    band between ``down_threshold`` and ``up_threshold``.  Default 60,
    range 0..100.  Set equal to ``up_threshold`` to disable hysteresis.

``down_threshold_adaptive``
    Variance-adaptive shaping for the brutality exit threshold.
    Default 0 (disabled), range 0..ZENITH_DOWN_THRESHOLD_ADAPTIVE_MAX.

Hispeed ladder
--------------

``hispeed_freq``
    Target frequency (kHz) the governor should aim for when load
    crosses ``hispeed_load``.  0 = disabled (default), in which case
    ``hispeed_freq_pct`` is consulted next.

``hispeed_freq_pct``
    Per-cluster auto-default for ``hispeed_freq`` when the absolute
    knob is 0: effective hispeed = ``policy->max * hispeed_freq_pct /
    100``.  Default 55.  Set both ``hispeed_freq`` and
    ``hispeed_freq_pct`` to 0 to disable the tier outright.

``hispeed_load``
    Load (%) at or above which the hispeed ladder activates.
    Default 65, range 1..100.

``hispeed_hyst_pct``
    Exit-side hysteresis margin (% of max_cap) on ``hispeed_load``.
    Entering still triggers at ``load_pct >= hispeed_load``; leaving
    requires ``load_pct < (hispeed_load - hispeed_hyst_pct)``.
    Default 10.

``hispeed_entry_streak``
    Symmetric entry-side hysteresis: require ``hispeed_load`` to hold
    for N+1 consecutive samples before activating.  Default 0 (off,
    legacy single-sample entry), range 0..ZENITH_HISPEED_ENTRY_STREAK_MAX.

``brutal_decay_ms``
    Tail-glide window in milliseconds (0..500, default 0).  When 0,
    the brutal-hold cliff exit is the legacy hard cliff: the moment
    ``load_pct`` drops below the (possibly adaptive-shaped) effective
    down threshold, ``brutal_active`` is cleared and the next sample's
    ``freq`` is whatever the EAS proportional math returns.  When
    non-zero, the cliff exit instead arms a linear glide: ``policy->max``
    at arm time, decaying toward the EAS-computed ``freq`` over
    ``brutal_decay_ms``.  Eliminates the audible / visible drop that
    the legacy cliff produces on bursty workloads.  The glide self-
    disarms once the deadline passes; while disarmed (the common case)
    the post-EAS check is a single zero-test branch.

``brutal_entry_streak``
    Same idea for the brutality tier (snap-to-max).  Only gates
    ``climb_mode = SNAP``; STEP mode is unaffected.  Default 0 (off).

``climb_mode``
    How ``zenith`` reaches the brutality target.  0 = SNAP (jump
    directly to ``policy->max`` on the next tick).  1 = STEP (climb
    one ``freq_step_pct``-sized step per tick, bypasses the brutality
    hysteresis).  Default 0.

``freq_step_pct``
    Size of each STEP-mode climb step expressed as a percent of
    ``policy->max``.  Default 5, range 1..100.

``freq_step_adaptive``
    Boolean (0/1, default 0).  When 1, the STEP step scales 1.0×..2.0×
    with the overshoot of load above ``up_threshold``.

Profile selector
----------------

``profile``
    Convenience knob that writes a coherent set of tunables in one go.
    Read returns the active profile name; write accepts one of:
    ``performance``, ``balanced``, ``battery``, ``legacy``, ``custom``.
    Writing any non-``custom`` profile sets the active profile and
    overwrites the dependent tunables; writing ``custom`` does *not*
    revert anything, it just marks the active profile as custom.
    Default is ``custom`` so individual sysfs writes are remembered.

``profile_values`` (read-only)
    Dump of the table-driven profile defaults.  Shows what each named
    profile would write, so userspace can preview a switch without
    committing it.

Auto-tune classifier (V1)
-------------------------

``zenith`` can passively classify the recent load profile (idle / light
/ busy / saturated) by sampling once every 10 s and counting how many
of the last N samples crossed thresholds.  When ``auto_tune = 1``, the
governor periodically rebalances ``up_threshold`` / ``down_threshold``
between two preset bands depending on whether recent input event rate
indicates the screen is being driven (UI work) or not (background).

``auto_tune``
    Enable / disable the V1 classifier worker.  0 (default) = off,
    1 = on.  Toggling on resets the sample counters.

``auto_tune_sat_load_pct``
    A 10 s sample is counted as "saturated" if average load reached
    this percentage.  Default 70, range 0..100.

``auto_tune_hi_sat_pct``
    Saturation rate (samples_saturated / samples_total, in percent) at
    or above which the classifier picks the *hi* band.  Default 60.

``auto_tune_lo_sat_pct``
    Saturation rate at or below which the classifier picks the *lo*
    band.  Default 10.  Between the two it stays in the previous band.

``auto_tune_hi_events_x2``
    Input event rate (events per 2 s, since the worker runs every
    10 s) that the classifier considers "screen actively driven".
    Default 4 (≈ 2 events/s).

``auto_tune_lo_events_x2``
    Input event rate at or below which the classifier considers the
    screen idle.  Default 1.

``auto_tune_scenario``
    Boolean (0/1, default 0).  When 1, the classifier's profile
    selection is biased by scenario flags (game / audio / camera /
    render) lifted from the awareness tiers.

Auto-tune state machine (V2)
----------------------------

V2 is a state machine layered on top of V1 that classifies into five
states — ``efficiency``, ``balanced``, ``latency``, ``sustained_perf``,
``thermal_recovery`` — and adjusts up to five knobs within per-profile
guardrails.

``auto_tune_v2``
    Boolean (0/1, default 0).  Enables the V2 state machine.  V1 must
    also be enabled (``auto_tune=1``) for V2 to run.

``auto_tune_hysteresis_windows``
    Require this many consecutive 10 s windows to agree before
    committing a state change.  Default 2, range 0..N.

``auto_tune_cooldown_windows``
    Quiet period (in windows) after a state change before another
    change is allowed.  ``thermal_recovery`` and ``sustained_perf``
    bypass this.  Default 1.

``auto_tune_cluster_aware``
    Boolean (0/1, default 0).  When 1, V2 actions are anchored on the
    first CPU of each policy and applied per cluster, so big and
    little can be in different states simultaneously.

``auto_tune_v2_signals``
    Bitmask of signals V2 may consult.  See ``ZENITH_AT_FLAG_*``
    in the source.  Default ``0`` (legacy: classifier only).

``auto_tune_thermal_slope``
    Boolean (0/1, default 0).  Enables the thermal-slope detector;
    when thermal pressure is rising fast, V2 jumps directly to
    ``thermal_recovery`` without waiting for hysteresis.

``auto_tune_thermal_pressure_pct``
    Threshold (%) of ``arch_scale_thermal_pressure()`` at which the
    thermal-slope detector arms.  Default 30.

``auto_tune_thermal_slope_pct``
    Rate-of-change (% per window) of thermal pressure that triggers
    early ``thermal_recovery`` entry.  Default 15.

``auto_tune_frame_pacing``
    Boolean (0/1, default 0).  When 1 and ``frame_budget_us`` is set,
    sustained frame-budget undershoot biases V2 toward ``latency``.

``auto_tune_sustained_gaming``
    Boolean (0/1, default 0).  When 1 and ``game_mode`` is non-zero
    for an extended window, V2 prefers the ``sustained_perf`` state.

``auto_tune_status`` (read-only)
    Diagnostic dump.  Shows current V2 state, pending state,
    confidence-window count, cooldown remaining, last reason, sample
    counters, signal flags, override mask, and active profile name.

``auto_tune_reset_overrides`` (write-only, write any non-zero)
    Clears the per-tunable user-override bitmask, allowing V2 to
    rewrite knobs the operator previously touched.

Efficient-frequency ladder
--------------------------

``efficient_freq``
    Upper soft cap (kHz).  Requests above this frequency are held at
    ``efficient_freq`` until the request has been sustained for
    ``up_delay_us``, only then allowed through.  0 = disabled
    (default).  Up to ``ZENITH_EFF_BINS_MAX`` (8) bins are supported
    via internal arrays ``eff_freq[]`` / ``eff_delay_us[]``; the
    sysfs knob writes the first bin only.

``up_delay_us``
    Time (µs) a ``>efficient_freq`` request must be sustained before
    the cap lifts.  Default 4000.  Range 0..1000000.

``eff_bin_hyst_pct``
    Exit-side hysteresis margin between adjacent efficient bins, as a
    percent of the higher bin's frequency.  Default 0 (no hysteresis,
    every tick re-evaluates from scratch).

Light-load cap
--------------

``light_load_freq``
    Hard upper cap (kHz) when current util is below
    ``light_load_threshold``.  Saves power on idle-ish workloads.
    0 = disabled (default).

``light_load_threshold``
    Load (%) below which the light-load cap applies.  Default 20,
    range 0..100.

Power-save bias
---------------

``powersave_bias``
    Fraction (in units of 1/1000) to subtract from the proportional
    scaling target after step 3.  Default 0.  Range 0..1000.

``bias_load_threshold``
    Load (%) below which ``powersave_bias`` is applied.  Above this
    threshold the bias is skipped so heavy work is not penalised.
    Default 50, range 0..100.  Set to 100 to always apply (legacy
    behaviour), or to 0 to never apply without touching
    ``powersave_bias``.

Input boost
-----------

``input_boost_ms``
    Duration (ms) to pin ``policy->max`` after any EV_KEY / EV_ABS /
    EV_REL event from the input subsystem.  Gated on ``screen_state``
    so screen-off never boosts.  Default 80, range 0..1000.

``input_boost_decay_ms``
    Length (ms) of the post-pin decay window during which the freq
    floor ramps from the boost ceiling back down to the natural eval.
    0 disables the decay (cliff exit).

``input_boost_decay_curve``
    Boolean (0/1, default 0).  When 1, the decay floor follows a
    cubic ease-in curve instead of linear.  Smoother visually,
    slightly more sustained boost.

``input_boost_big_only``
    Boolean (0/1, default 0).  When 1, only big / prime clusters
    receive the input boost; the little cluster eval path is
    untouched.

``input_boost_cap_pct``
    Cap on the boost ceiling expressed as percent of ``policy->max``.
    Default 100 (no cap).  Set lower to convert the input boost into
    a softer hispeed pin.

``boot_boost_decay_ms``
    Trailing decay window for ``boot_boost``, in milliseconds
    (0..30000, default 0).  When 0, the boot-boost ends as a hard
    cliff at ``boot_boost_ms``.  When non-zero, after
    ``boot_boost_ms`` expires the governor applies a linear floor
    that ramps from ``policy->max`` down to ``policy->min`` over
    ``boot_boost_decay_ms``, mirroring ``input_boost_decay_ms``'s
    tail behaviour for the boot pin.  Useful when boot animations
    or surface-flinger initial paints land just past
    ``boot_boost_ms`` and would otherwise see the sudden drop.

``boot_boost_ms``
    One-shot pin to ``policy->max`` for this many ms after boot,
    decays past the deadline.  Gated by ``screen_state`` and
    ``input_boost_big_only``.  0 = disabled, recommended 30000
    (30 s).

I/O wait handling
-----------------

``io_is_busy``
    Boolean (0/1, default 1).  When 1, ``SCHED_CPUFREQ_IOWAIT`` flags
    from the scheduler boost the frequency target.  When 0, IO-bound
    workloads get no boost.

``iowait_boost_min``
    Minimum util (in permille of max capacity, range 0..1000) the
    iowait boost may force.  Default 125 (12.5 %).  0 disables the
    floor and lets iowait boost ramp from the natural util signal.

``iowait_stack_pct``
    Percent (0..100) of the iowait-derived boost that is stacked on
    top of util when both are present.  Default 50.  0 reproduces
    the legacy ``max(util, boost)`` semantics.

``iowait_backoff_after_ms``
    When a single iowait episode has been live for this many ms, the
    doubling climb in ``zenith_iowait_boost()`` flips to a halving
    decay.  0 disables the backoff (legacy unbounded climb).

``ignore_nice_load``
    Boolean (0/1, default 0).  When 1, time spent on tasks with nice
    > 0 is excluded from the load calculation.  Useful when
    background tasks are intentionally low-priority and should not
    influence DVFS.

Environment hooks
-----------------

``screen_state``
    Userspace / notifier-driven flag indicating display state.  1 = on
    (default), 0 = off.  When 0, ``up_threshold`` is internally raised
    to 95 and ``powersave_bias`` is internally forced to 500 (50 %),
    so the governor becomes a very sleepy ondemand.  Also disables
    input boost.

``screen_off_glide_ms``
    Soft-glide window for the 1 -> 0 transition on ``screen_state``,
    in milliseconds (0..2000, default 0).  When 0, the screen-off
    cliff is the legacy hard step: ``dynamic_up_thresh`` snaps to 95
    and ``dynamic_bias`` snaps to 500 (the 50%% powersave penalty)
    on the very next sample after ``screen_state`` flips to 0.  When
    non-zero, both quantities ramp linearly from their natural values
    (``up_threshold`` / ``powersave_bias``) to the cliff targets
    across the configured window.  Eliminates the cliff that
    otherwise lands the moment AOD / panel-blank handlers stamp
    ``screen_state=0`` while userspace work is still winding down.

``screen_auto``
    Boolean (0/1, default 1).  When 1, ``zenith`` listens to the fb /
    drm-panel notifier chain and automatically updates
    ``screen_state`` on display blank/unblank.  When 0,
    ``screen_state`` is purely userspace-driven.

    The notifier source is selected at boot in this order:

    1. ``CONFIG_DRM_PANEL_NOTIFY=y`` (vendor builds with a
       ``drm_panel_notifier_register()`` API): preferred whenever
       registration succeeds.  ``dmesg`` shows
       ``screen_auto wired through drm panel notifier``.
    2. ``CONFIG_FB_NOTIFY=y``: fallback when drm is unavailable or
       fails registration.  Common on legacy / 5.10-stable trees.
    3. Neither: ``screen_state`` is bookkeeping-only and only
       userspace can flip it.  ``dmesg`` says so.

    Stock GKI 5.10 does not ship a drm panel notifier; vendor builds
    that backport one (Qualcomm / MediaTek / Samsung have their own)
    must define ``CONFIG_DRM_PANEL_NOTIFY=y`` and provide the
    matching ``<drm/drm_panel_notifier.h>`` header to opt in.

``thermal_pressure_continuous``
    Boolean (0/1, default 0).  When 0, ``zenith`` uses the legacy
    cliff: as soon as ``zenith_thermal_active()`` becomes true,
    ``dynamic_up_thresh`` snaps to 90 % regardless of the actual
    pressure level.  When 1, ``dynamic_up_thresh`` ramps linearly
    from the policy's ``up_threshold`` (at 0 % pressure) to 90 % (at
    100 % pressure), using the same
    ``arch_scale_thermal_pressure()``-derived percentage that the
    auto-tune V2 classifier consumes.  Smooths long-session thermal
    throttling: removes the audible / visible step that would
    otherwise occur the moment ``thermal_state`` flips on after a
    long burst.  The ramp end-points are intentionally hard-coded to
    match the cliff target so the new continuous mode never asks
    for a higher frequency than the legacy cliff would have allowed
    at peak pressure.

``prefer_silver_aware``
    Boolean (0/1, default 0).  When 1 and built with
    ``CONFIG_SCHED_PREFER_SILVER=y``, the auto-tune classifier worker
    samples the global ``prefer_silver`` hit / miss counters once per
    window and computes a hit-rate.  When the rate is at or above
    ``prefer_silver_hot_threshold_pct``, big / prime cluster policies
    raise ``dynamic_up_thresh`` by ``prefer_silver_hot_bump_pct``
    points (clamped at 95) so the big cluster down-clocks less
    aggressively during sustained UI / app navigation -- the regime
    where ``prefer_silver`` is steering most light wake-ups onto the
    silver / LITTLE cluster and the big cluster's measured load is
    artificially deflated.  Little cluster policies are intentionally
    not bumped (they are already absorbing the redirected work).
    When ``CONFIG_SCHED_PREFER_SILVER=n`` the worker stub leaves the
    cached hit-rate at zero and the bump never fires.

``prefer_silver_hot_threshold_pct``
    Hit-rate threshold (0..100, default 50).  Lower the value to
    fire the bump more aggressively, raise it to require a clearer
    signal that ``prefer_silver`` is actively redistributing load.

``prefer_silver_hot_bump_pct``
    Number of percentage points (0..20, default 5) added to
    ``dynamic_up_thresh`` on big / prime cluster policies when the
    bump fires.  Final ``dynamic_up_thresh`` is capped at 95 so the
    bump cannot exceed the screen-off cliff.

``thermal_state``
    Userspace / notifier-driven flag indicating thermal pressure.
    1 = hot, 0 = cool (default).  When 1, ``up_threshold`` is
    internally raised to 90 and the energy-model cap drops the top
    turbo bin.

``thermal_auto``
    Boolean (0/1, default 0).  When 1, ``zenith`` listens to thermal
    framework notifications and toggles ``thermal_state``
    automatically when the policy enters / exits a thermal-pressure
    band.

``thermal_util_derate``
    Util-side derating applied when ``thermal_state`` is set, in %
    (0..100).  Default 0 (no extra derating).  Independent of the
    EM-cap step.

``thermal_derate_rate_pct``
    Rate-of-change (% per tick) on the thermal-derate ramp; soft
    derivative term that smooths the binary thermal_state cliff.
    Default 0 (off).

``wakeup_boost``
    Boolean (0/1, default 0).  Brief bypass of ``up_rate_limit_us``
    on idle-to-busy transitions, so the first tick after a long idle
    can climb without the rate-limit clamp.

``wakeup_boost_ms``
    Wall-clock duration of the wakeup-boost bypass, in milliseconds
    (0..200, default 0).  When 0, the legacy tick-based bypass is the
    only mechanism: the next ``ZENITH_WAKEUP_BOOST_TICKS`` (currently
    2) upward transitions skip ``up_rate_limit`` after the detector
    fires.  When non-zero, the detection sites additionally arm a
    per-CPU ``ktime_get_ns()``-based deadline; the up-rate bypass
    stays active until both the tick counter has expired AND the
    deadline has lapsed.  Useful when ``up_rate_limit`` is small
    enough that two ticks elapse in microseconds, leaving the bypass
    too short to actually escape the cold-start sample.

kcpustat hispeed blend
----------------------

These three knobs control an optional blend of kernel-cpu-stat-derived
load (``/proc/stat`` style aggregation across user / sys / iowait /
softirq) into the governor's main util signal.  Useful when the
scheduler's PELT signal is artificially low because of CFS bandwidth
control, but kernel-side activity (network softirq, IRQ load) is high.

``kcpustat_hispeed_enable``
    Boolean (0/1, default 0).  When 1, the kcpustat blend is added on
    top of the EAS proportional util in step 3 of the decision chain.

``kcpustat_window_us``
    Sliding window length (µs) used to compute the kcpustat-derived
    load fraction.  Default 4000, range 1000..100000.  Smaller window
    = more reactive, more variance.

``kcpustat_filter_shift``
    Right-shift count applied to smooth the kcpustat-derived load
    over successive windows (``ema = ema - (ema >> shift) + new``).
    Default 1, range 0..8.  0 = no smoothing, higher = slower to
    react.

Util-math version
-----------------

``util_math_v2``
    Boolean (0/1, default 0).  Selects the util signal source for
    ``zenith_get_util()``.  0 = legacy ``cpu_util_cfs()``-based path
    (5.10 native).  1 = 6.x-style runnable-aware util that includes
    ``cpu_util_irq()`` and ``cpu_util_rt()`` contributions.  The v2
    signal is closer to mainline ``schedutil`` post-6.0.

uclamp / ADPF
-------------

``uclamp_min_respect``
    Boolean (0/1, default 1).  When 1, any task on this policy with a
    cgroup or ADPF-set ``uclamp_min`` raises a frequency floor
    proportional to that minimum.

``uclamp_max_respect``
    Boolean (0/1, default 0).  When 1, any task on this policy with a
    cgroup or ADPF-set ``uclamp_max`` caps the frequency target
    proportional to that maximum.

Predicted util
--------------

``predict_util_pct``
    Percentage (0..100, default 0 = off) of a two-tap util predictor
    blended into the EAS proportional target.  Mitigates ramp-up
    latency on bursty workloads at the cost of a small overshoot
    risk.

``predict_util_smooth``
    Boolean (0/1, default 0).  When 1, the predictor's two taps are
    additionally low-pass filtered before blending, trading peak
    accuracy for less noise.

Awareness tiers
---------------

Each awareness tier maintains an RCU-protected list of ``comm`` prefix
strings (mutable from sysfs).  When at least one task whose ``comm``
matches a prefix is runnable on the policy, the tier's floor / cap is
applied between the EAS proportional step and the post-resolve clips.

Render tier::

    render_aware       0/1, default 0.
    render_comms       \n-separated list of comm prefixes; default
                       includes "RenderEngine", "RenderThread",
                       "surfaceflinger", "GraphicsExecutor", etc.
    render_floor_pct   Floor as percent of policy->max when matched.
                       Default 0 (off).

Audio tier::

    audio_aware        0/1, default 0.
    audio_comms        Default includes "AudioFlinger", "AudioOut_",
                       "AudioRecord", "audio.hal", etc.
    audio_floor_pct    Floor (% of max) when matched.  Default 0.
    audio_cap_pct      Cap (% of max) when matched, e.g. for
                       low-latency audio threads that should not be
                       allowed to climb to max during low-load
                       intervals.  Default 0 (no cap).

Camera tier::

    camera_aware       0/1, default 0.
    camera_active      Userspace flag mirroring an active camera
                       session.  Read-write; userspace toggles based
                       on camera HAL state.
    camera_comms       Default includes "Camera HAL", "CameraServer",
                       "ImageProcessor", etc.
    camera_floor_pct   Floor (% of max) when matched.  Default 0.

Game mode::

    game_mode          0 = off, 1 = level-1 (input-boost emphasis,
                       2 = level-2 (sustained big-cluster bias).
                       Default 0.

Frame pacing
------------

``frame_budget_us``
    Frame budget (µs).  When set and ``predict_util_pct > 0``, sustained
    util below the frame-pace floor for longer than this budget
    raises a brief floor on ``policy->max`` to recover.  60 Hz =
    16667, 90 Hz = 11111, 120 Hz = 8333.  Default 0 (off).

``frame_budget_us_per_policy``
    CSV override of ``frame_budget_us`` per policy (e.g.
    ``policy0=16667,policy4=8333``).  Empty = use the global
    ``frame_budget_us``.

``frame_pace_floor_pct``
    Floor (% of policy->max) raised when the frame-pace timer fires.
    Default 0 (off).

PSI awareness
-------------

``psi_aware``
    Boolean (0/1, default 0).  Enables the PSI-aware tier.

``psi_mem_thresh``
    PSI memory-some pressure (%) at or above which a memory-stall
    floor is applied.  Default 0 (off).

``psi_cpu_thresh``
    PSI cpu-some pressure (%) trigger.  Default 0 (off).

``psi_io_thresh``
    PSI io-some pressure (%) trigger.  Default 0 (off).

Read-only diagnostics
---------------------

``zenith_stats``
    Per-policy decision-tier counters: total ticks, brutality entries,
    hispeed entries, EAS, input-boost, light-load cap hits, EM cap
    hits.  Read once from userspace after a benchmark to attribute
    the freq-time histogram to specific tiers.

``zenith_stats_reset`` (write-only)
    Write any non-zero value to clear ``zenith_stats`` and the
    ``at_log`` ring on every policy that shares this tunables set.
    Writing ``0`` is intentionally a no-op (avoids a stray
    ``echo > zenith_stats_reset`` erasing the data the operator was
    about to read).

``at_log``
    Per-policy ring buffer of the most recent auto-tune classifier
    samples.  Each sample is one ``ZENITH_AUTO_TUNE_PERIOD_MS`` window
    (10 s by default), the depth of the ring is
    ``ZENITH_AT_LOG_NR`` (16) so each policy keeps about 160 s of
    post-mortem.  Output is one header line per policy followed by
    one line per sample (oldest first), with ``key=value`` pairs::

        ts_ns=  ktime_get_ns() at push time
        reason= V1 / V2 reason name
        from=   V2 state at the start of the window
        to=     V2 state after this window's actions
        target= V1 picked profile name
        sat=    saturation rate (% of samples > saturation_load_pct)
        evx2=   input event rate (events / 2 s)
        thp=    arch_scale_thermal_pressure() at push (0..1024)
        slope=  thermal pressure delta vs the previous window
        var=    load variance EWMA × 256
        flags=  ZENITH_AT_FLAG_* signal mask
        emerg=  1 if a V2 emergency state was committed

Typical recipes
===============

Phone-class daily driver
------------------------

Active screen, mixed workload, expecting some bursty UI.  This is the
zenith-default sweet spot::

    echo balanced > /sys/.../zenith/profile
    echo 1        > /sys/.../zenith/auto_tune
    echo 1        > /sys/.../zenith/auto_tune_v2
    echo 1        > /sys/.../zenith/screen_auto
    echo 1        > /sys/.../zenith/thermal_auto
    echo 80       > /sys/.../zenith/input_boost_ms

Flagship SoC, gaming ROM
------------------------

Preserves peak performance, adds modest light-load savings::

    echo performance > /sys/.../zenith/profile
    echo 80          > /sys/.../zenith/up_threshold
    echo 60          > /sys/.../zenith/down_threshold
    echo 4           > /sys/.../zenith/sampling_down_factor
    echo 80          > /sys/.../zenith/input_boost_ms
    echo 0           > /sys/.../zenith/auto_tune
    # no light-load cap, no efficient_freq, no bias

Mid-range SoC, battery ROM
--------------------------

Caps idle-ish load aggressively, slower ramp-up::

    echo battery > /sys/.../zenith/profile
    echo 80      > /sys/.../zenith/up_threshold
    echo 50      > /sys/.../zenith/down_threshold
    echo 1       > /sys/.../zenith/sampling_down_factor
    echo 80      > /sys/.../zenith/input_boost_ms
    echo 691200  > /sys/.../zenith/light_load_freq
    echo 30      > /sys/.../zenith/light_load_threshold
    echo 100     > /sys/.../zenith/powersave_bias
    echo 40      > /sys/.../zenith/bias_load_threshold

Ondemand-style legacy
---------------------

Reproduces the pre-hysteresis behaviour for comparison::

    echo legacy > /sys/.../zenith/profile
    echo 80     > /sys/.../zenith/up_threshold
    echo 80     > /sys/.../zenith/down_threshold      # hysteresis off
    echo 1      > /sys/.../zenith/sampling_down_factor
    echo 0      > /sys/.../zenith/input_boost_ms      # boost off
    echo 100    > /sys/.../zenith/bias_load_threshold # always bias

Hispeed-ladder example
----------------------

Force the policy to climb to a known efficient frequency before
allowing brutality, useful when the silicon has a clear voltage cliff
at one bin above ``hispeed_freq``::

    echo 1804800 > /sys/.../zenith/hispeed_freq
    echo 70      > /sys/.../zenith/hispeed_load
    echo 1       > /sys/.../zenith/climb_mode      # step instead of snap
    echo 10      > /sys/.../zenith/freq_step_pct   # 10 % per update

kcpustat blend example
----------------------

Useful when softirq / IRQ load is the bottleneck (heavy networking,
USB bulk traffic) and the scheduler's PELT util alone underestimates
real CPU pressure::

    echo 1     > /sys/.../zenith/kcpustat_hispeed_enable
    echo 4000  > /sys/.../zenith/kcpustat_window_us
    echo 1     > /sys/.../zenith/kcpustat_filter_shift

Relationship to other governors
===============================

``schedutil``
    ``zenith`` inherits the PELT-based ``schedutil_cpu_util`` signal
    and the iowait-boost mechanism.  It differs in three ways: split
    up/down rate limits, an ondemand-style hard threshold for
    snap-to-max, and an explicit input-event boost.  With
    ``util_math_v2 = 1``, the util signal is essentially 6.x
    ``schedutil``'s.

``ondemand``
    ``zenith`` reuses ``sampling_down_factor`` and the
    ``up_threshold`` snap-to-max concept.  It does **not** use
    ondemand's per-policy kworker polling; like ``schedutil`` it runs
    off the scheduler's ``update_util`` hook.

``schedhorizon``
    ``zenith`` borrows the efficient-frequency delay concept and
    extends it to a ladder of up to 8 bins with optional exit
    hysteresis.

``interactive``
    ``zenith`` reuses the hispeed-frequency / hispeed-load idea and
    the climb-mode (snap vs step) toggle, but does not borrow
    interactive's target-load array — that's replaced by the simpler
    ``hispeed_load`` + ``up_threshold_hispeed`` pair plus
    symmetric entry-streak hysteresis.
