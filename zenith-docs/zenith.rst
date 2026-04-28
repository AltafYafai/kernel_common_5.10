.. SPDX-License-Identifier: GPL-2.0

===============================
The Zenith CPUFreq Governor
===============================

:Copyright: (C) 2026
:Author: XTENSEI (exclusively for android12-5.10 kernels)

``zenith`` is a custom hybrid CPUFreq governor that combines techniques from
``schedutil`` (EAS proportional scaling), ``ondemand`` (hard-threshold snap to
maximum), ``schedhorizon`` (efficient-frequency delay ladder) and the legacy
interactive governor (hispeed bias / climb-mode), and adds an explicit
input-event boost path on top. It is built for interactive-first Android UI
on modern ARM flagship SoCs.

It lives in ``kernel/sched/cpufreq_zenith.c`` and is selected with
``CONFIG_CPU_FREQ_GOV_ZENITH=y``. To make it the default at boot, set
``CONFIG_CPU_FREQ_DEFAULT_GOV_ZENITH=y`` in your defconfig.

Decision chain
==============

On every ``cpufreq_governor_update_util()`` callback, ``zenith`` walks the
following chain in ``zenith_get_next_freq()``::

    0. Input Boost          → pin to policy->max for input_boost_ms
                              after any EV_KEY/EV_ABS/EV_REL event
    1. Ondemand Brutality   → pin to policy->max once load crosses
                              up_threshold (or up_threshold_hispeed when
                              already above hispeed_freq), stay pinned
                              until load drops below down_threshold
                              (hysteresis)
    2. Hispeed Ladder       → if hispeed_freq is set and load >=
                              hispeed_load, climb toward hispeed_freq
                              using climb_mode (snap or step)
    3. EAS Proportional     → map_util_freq(util, max_freq, max_cap)
                              with optional capacity headroom; util
                              source picked by util_math_v2 (v1 = legacy
                              cpu_util_cfs(), v2 = 6.x-style runnable-
                              aware) and optionally blended with the
                              kcpustat hispeed signal
    4. Powersave Bias       → subtract powersave_bias / 1000 from freq,
                              but only when load < bias_load_threshold
    5. Cache Resolve        → cpufreq_driver_resolve_freq()
    6. Efficient Cap        → hold at efficient_freq until the request
                              has been sustained for up_delay_us
    7. Light-Load Cap       → hard-clamp to light_load_freq when load
                              is below light_load_threshold
    8. EM Cap               → drop the top turbo bin when
                              thermal_state is set
    9. Sampling Down        → extend down_rate_limit by
                              sampling_down_factor while at max

The input-boost and brutality paths ``goto resolve``, skipping steps
2-4; all paths pass through the remaining steps.

Tunables
========

All tunables are exposed under
``/sys/devices/system/cpu/cpufreq/zenith/``. Writes are range-checked and
return ``-EINVAL`` on out-of-range input. There are 35 tunables in total.
Defaults below are the kernel-side defaults; the ``profile`` selector
overrides several of them at once when written.

Rate limiting
-------------

``up_rate_limit_us``
    Minimum time (µs) between two consecutive upward frequency changes.
    Default 500. Lower = more responsive, more DVFS churn.

``down_rate_limit_us``
    Minimum time (µs) between two consecutive downward frequency changes.
    Default 2000. Higher = sticks at elevated frequencies longer, lower
    battery.

``sampling_down_factor``
    While ``target_freq`` is at ``policy->max``, the effective
    ``down_rate_limit_us`` is multiplied by this factor. Stops ping-pong
    between max and the bin just below max under sustained heavy load.
    Default 1 (disabled), range 1..10.

Load thresholds
---------------

``up_threshold``
    Load (in % of CPU capacity) at or above which ``zenith`` jumps straight
    to ``policy->max`` (ondemand-style). Default 80, range 1..100.

``up_threshold_hispeed``
    Optional second up-threshold that applies only after the policy is
    already above ``hispeed_freq``. Used to make the climb above hispeed
    require a stronger sustained load than the climb to hispeed itself.
    0 = disabled (default), so the regular ``up_threshold`` always
    applies. Range 0..100.

``down_threshold``
    Load (%) at or below which the brutality lock is released, letting
    EAS proportional scaling take over again. Creates a hysteresis band
    between ``down_threshold`` and ``up_threshold``. Default 60, range
    0..100. Set equal to ``up_threshold`` to disable hysteresis.

Hispeed ladder
--------------

``hispeed_freq``
    Target frequency (kHz) the governor should aim for when load crosses
    ``hispeed_load``. 0 = disabled (default), in which case the hispeed
    ladder is skipped entirely and the governor goes straight from EAS
    proportional to brutality.

``hispeed_load``
    Load (%) at or above which the hispeed ladder activates. Default 90,
    range 1..100.

``climb_mode``
    How ``zenith`` reaches ``hispeed_freq``. 0 = snap (default — go
    directly to the target on the next freq update), 1 = step (climb
    one ``freq_step_pct``-sized step per update).

``freq_step_pct``
    Size of each climb-mode step expressed as a percentage of
    ``policy->max``. Only used when ``climb_mode = 1``. Default 5,
    range 1..100.

Auto-tune classifier
--------------------

``zenith`` can passively classify the recent load profile (idle / light /
busy / saturated) by sampling once every 10 s and counting how many of
the last N samples crossed thresholds. When ``auto_tune = 1``, the
governor periodically rebalances ``up_threshold`` / ``down_threshold``
between two preset bands depending on whether recent input event rate
indicates the screen is being driven (UI work) or not (background work).

``auto_tune``
    Enable / disable the classifier worker. 0 (default) = off, 1 = on.
    Toggling on resets the sample counters.

``auto_tune_sat_load_pct``
    A 10 s sample is counted as "saturated" if average load reached this
    percentage. Default 70, range 0..100.

``auto_tune_hi_sat_pct``
    Saturation rate (samples_saturated / samples_total, in percent) at
    or above which the classifier picks the *hi* band. Default 60.

``auto_tune_lo_sat_pct``
    Saturation rate at or below which the classifier picks the *lo*
    band. Default 10. Between the two it stays in the previous band.

``auto_tune_hi_events_x2``
    Input event rate (events per 2 s, since the worker runs every 10 s)
    that the classifier considers "screen actively driven". Default 4
    (≈ 2 events/s).

``auto_tune_lo_events_x2``
    Input event rate at or below which the classifier considers the
    screen idle. Default 1.

Profile selector
----------------

``profile``
    Convenience knob that writes a coherent set of tunables in one go.
    Read returns the active profile name; write accepts one of:
    ``performance``, ``balanced``, ``battery``, ``legacy``, ``custom``.
    Writing any non-``custom`` profile sets the active profile and
    overwrites the dependent tunables; writing ``custom`` does *not*
    revert anything, it just marks the active profile as custom. Default
    is ``custom`` so individual sysfs writes are remembered.

Efficient-frequency ladder
--------------------------

``efficient_freq``
    Upper soft cap (kHz). Requests above this frequency are held at
    ``efficient_freq`` until the request has been sustained for
    ``up_delay_us``, only then allowed through. 0 = disabled (default).

``up_delay_us``
    Time (µs) a ``>efficient_freq`` request must be sustained before the
    cap lifts. Default 4000. Range 0..1000000.

Light-load cap
--------------

``light_load_freq``
    Hard upper cap (kHz) when current util is below
    ``light_load_threshold``. Saves power on idle-ish workloads. 0 =
    disabled (default).

``light_load_threshold``
    Load (%) below which the light-load cap applies. Default 20, range
    0..100.

Power-save bias
---------------

``powersave_bias``
    Fraction (in units of 1/1000) to subtract from the proportional
    scaling target after step 3. Default 0. Range 0..1000.

``bias_load_threshold``
    Load (%) below which ``powersave_bias`` is applied. Above this
    threshold the bias is skipped so heavy work is not penalised.
    Default 50, range 0..100. Set to 100 to always apply (legacy
    behaviour), or to 0 to never apply without touching
    ``powersave_bias``.

Input boost
-----------

``input_boost_ms``
    Duration (ms) to pin ``policy->max`` after any EV_KEY / EV_ABS /
    EV_REL event from the input subsystem. Gated on ``screen_state`` so
    screen-off never boosts. Default 80, range 0..1000. Set to 0 to
    disable.

I/O wait handling
-----------------

``io_is_busy``
    Boolean (0/1, default 1). When 1, ``SCHED_CPUFREQ_IOWAIT`` flags
    from the scheduler boost the frequency target. When 0, IO-bound
    workloads get no boost.

``iowait_boost_min``
    Minimum util (in percent of max capacity, range 0..100) the iowait
    boost may force. Default 125 (capped to 100 internally — value above
    100 just means "always force a meaningful boost"). 0 disables the
    floor and lets iowait boost ramp from the natural util signal.

``ignore_nice_load``
    Boolean (0/1, default 0). When 1, time spent on tasks with nice > 0
    is excluded from the load calculation. Useful when background tasks
    are intentionally low-priority and should not influence DVFS.

Environment hooks
-----------------

``screen_state``
    Userspace / notifier-driven flag indicating display state. 1 = on
    (default), 0 = off. When 0, ``up_threshold`` is internally raised to
    95 and ``powersave_bias`` is internally forced to 500 (50%), so the
    governor becomes a very sleepy ondemand. Also disables input boost.

``screen_auto``
    Boolean (0/1, default 1). When 1, ``zenith`` listens to the fb /
    drm-panel notifier chain and automatically updates ``screen_state``
    on display blank/unblank. When 0, ``screen_state`` is purely
    userspace-driven.

``thermal_state``
    Userspace / notifier-driven flag indicating thermal pressure. 1 =
    hot, 0 = cool (default). When 1, ``up_threshold`` is internally
    raised to 90 and the energy-model cap drops the top turbo bin.

``thermal_auto``
    Boolean (0/1, default 0). When 1, ``zenith`` listens to thermal
    framework notifications and automatically toggles ``thermal_state``
    when the policy enters / exits a thermal-pressure band. When 0,
    ``thermal_state`` is purely userspace-driven.

kcpustat hispeed blend
----------------------

These three knobs control an optional blend of kernel-cpu-stat-derived
load (``/proc/stat`` style aggregation across user / sys / iowait /
softirq) into the governor's main util signal. Useful when the
scheduler's PELT signal is artificially low because of CFS bandwidth
control, but kernel-side activity (network softirq, IRQ load) is high.

``kcpustat_hispeed_enable``
    Boolean (0/1, default 0). When 1, the kcpustat blend is added on top
    of the EAS proportional util in step 3 of the decision chain.

``kcpustat_window_us``
    Sliding window length (µs) used to compute the kcpustat-derived load
    fraction. Default 4000, range 1000..100000. Smaller window = more
    reactive, more variance.

``kcpustat_filter_shift``
    Right-shift count applied to smooth the kcpustat-derived load over
    successive windows (``ema = ema - (ema >> shift) + new``). Default
    1, range 0..8. 0 = no smoothing, higher = slower to react.

Util-math version
-----------------

``util_math_v2``
    Boolean (0/1, default 0). Selects the util signal source for
    ``zenith_get_util()``. 0 = legacy ``cpu_util_cfs()``-based path
    (5.10 native). 1 = 6.x-style runnable-aware util that includes
    ``cpu_util_irq()`` and ``cpu_util_rt()`` contributions.  The v2
    signal is closer to mainline ``schedutil`` post-6.0.

Typical recipes
===============

Phone-class daily driver
------------------------

Active screen, mixed workload, expecting some bursty UI. This is the
zenith-default sweet spot::

    echo balanced > /sys/.../zenith/profile
    echo 1        > /sys/.../zenith/auto_tune
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
    ``zenith`` inherits the PELT-based ``schedutil_cpu_util`` signal and
    the iowait-boost mechanism. It differs in three ways: split
    up/down rate limits, an ondemand-style hard threshold for snap-to-max,
    and an explicit input-event boost. With ``util_math_v2 = 1``, the
    util signal is essentially 6.x ``schedutil``'s.

``ondemand``
    ``zenith`` reuses ``sampling_down_factor`` and the ``up_threshold``
    snap-to-max concept. It does **not** use ondemand's per-policy
    kworker polling; like ``schedutil`` it runs off the scheduler's
    ``update_util`` hook.

``schedhorizon``
    ``zenith`` borrows the efficient-frequency delay concept but
    collapses the multi-step ladder to a single step for predictability
    and simpler userspace parsing.

``interactive``
    ``zenith`` reuses the hispeed-frequency / hispeed-load idea and the
    climb-mode (snap vs step) toggle, but does not borrow interactive's
    target-load array — that's replaced by the simpler
    ``hispeed_load`` + ``up_threshold_hispeed`` pair.
