.. SPDX-License-Identifier: GPL-2.0

===============================
The Zenith CPUFreq Governor
===============================

:Copyright: |copy| 2025
:Author: ENI (exclusively for LO)

``zenith`` is a custom hybrid CPUFreq governor that combines techniques from
``schedutil`` (EAS proportional scaling), ``ondemand`` (hard-threshold snap to
maximum), ``schedhorizon`` (efficient-frequency delay ladder) and the legacy
interactive governor (hispeed bias), and adds an explicit input-event boost
path on top. It is built for interactive-first Android UI on modern ARM
flagship SoCs.

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
                              up_threshold, stay pinned until load
                              drops below down_threshold (hysteresis)
    2. EAS Proportional     → map_util_freq(util, max_freq, max_cap)
                              with optional capacity headroom
    3. Powersave Bias       → subtract powersave_bias / 1000 from freq,
                              but only when load < bias_load_threshold
    4. Cache Resolve        → cpufreq_driver_resolve_freq()
    5. Efficient Cap        → hold at efficient_freq until the request
                              has been sustained for up_delay_us
    6. Light-Load Cap       → hard-clamp to light_load_freq when load
                              is below light_load_threshold
    7. EM Cap               → drop the top turbo bin when
                              thermal_state is set
    8. Sampling Down        → extend down_rate_limit by
                              sampling_down_factor while at max

The input-boost and brutality paths ``goto resolve``, skipping steps
2 and 3; all paths pass through the remaining steps.

Tunables
========

All tunables are exposed under
``/sys/devices/system/cpu/cpufreq/zenith/``. Writes are range-checked and
return ``-EINVAL`` on out-of-range input.

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

``down_threshold``
    Load (%) at or below which the brutality lock is released, letting
    EAS proportional scaling take over again. Creates a hysteresis band
    between ``down_threshold`` and ``up_threshold``. Default 60, range
    0..100. Set equal to ``up_threshold`` to disable hysteresis.

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
    scaling target after step 2. Default 0. Range 0..1000.

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

Environment hooks
-----------------

``screen_state``
    Userspace / notifier-driven flag indicating display state. 1 = on
    (default), 0 = off. When 0, ``up_threshold`` is internally raised to
    95 and ``powersave_bias`` is internally forced to 500 (50%), so the
    governor becomes a very sleepy ondemand.

``thermal_state``
    Userspace / notifier-driven flag indicating thermal pressure. 1 =
    hot, 0 = cool (default). When 1, ``up_threshold`` is internally
    raised to 90 and the energy-model cap drops the top turbo bin.

``io_is_busy``
    Boolean (0/1, default 1). When 1, SCHED_CPUFREQ_IOWAIT flags from
    schedutil boost the frequency target. When 0, IO-bound workloads
    get no boost.

Typical recipes
===============

Flagship SoC, gaming ROM
------------------------

Preserves peak performance, adds modest light-load savings::

    echo 80   > /sys/.../zenith/up_threshold
    echo 60   > /sys/.../zenith/down_threshold
    echo 4    > /sys/.../zenith/sampling_down_factor
    echo 80   > /sys/.../zenith/input_boost_ms
    # no light-load cap, no efficient_freq, no bias

Mid-range SoC, battery ROM
--------------------------

Caps idle-ish load aggressively, slower ramp-up::

    echo 80     > /sys/.../zenith/up_threshold
    echo 50     > /sys/.../zenith/down_threshold
    echo 1      > /sys/.../zenith/sampling_down_factor
    echo 80     > /sys/.../zenith/input_boost_ms
    echo 691200 > /sys/.../zenith/light_load_freq
    echo 30     > /sys/.../zenith/light_load_threshold
    echo 100    > /sys/.../zenith/powersave_bias
    echo 40     > /sys/.../zenith/bias_load_threshold

Ondemand-style legacy
---------------------

Reproduces the pre-hysteresis behaviour for comparison::

    echo 80  > /sys/.../zenith/up_threshold
    echo 80  > /sys/.../zenith/down_threshold      # hysteresis off
    echo 1   > /sys/.../zenith/sampling_down_factor
    echo 0   > /sys/.../zenith/input_boost_ms      # boost off
    echo 100 > /sys/.../zenith/bias_load_threshold # always bias

Relationship to other governors
===============================

``schedutil``
    ``zenith`` inherits the PELT-based ``schedutil_cpu_util`` signal and
    the iowait-boost mechanism. It differs in three ways: split
    up/down rate limits, an ondemand-style hard threshold for snap-to-max,
    and an explicit input-event boost.

``ondemand``
    ``zenith`` reuses ``sampling_down_factor`` and the ``up_threshold``
    snap-to-max concept. It does **not** use ondemand's per-policy
    kworker polling; like ``schedutil`` it runs off the scheduler's
    ``update_util`` hook.

``schedhorizon``
    ``zenith`` borrows the efficient-frequency delay concept but
    collapses the multi-step ladder to a single step for predictability
    and simpler userspace parsing.
