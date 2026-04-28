SPDX-License-Identifier: GPL-2.0

================================================
The Zenith Kernel Stack: tunables outside zenith
================================================

Copyright: (C) 2026
Author: XTENSEI (exclusively for android12-5.10 kernels)

The ``zenith`` CPUFreq governor (see :doc:`zenith`) is one piece of a
larger phone-class kernel stack. This document describes the rest of
the stack: the BORE scheduler, KSM, the ADIOS I/O scheduler, CAKE as
default qdisc, and the kernel-baked ``mm`` / ``TCP`` / ``net.core``
defaults that ``zenith`` ships with.

Each section lists the tunables, what they do, the kernel-default
value (after this kernel's modifications), and when you might want to
retune.

BORE — Burst-Oriented Response Enhancer
=======================================

``BORE`` is a CFS scheduler modification by Masahito S (firelzrd) that
tracks a per-task "burst penalty" — a moving signal of how
front-loaded each task's CPU consumption is. Tasks with a heavy burst
penalty are deprioritised at vruntime placement so they cannot drown
out latency-sensitive peers. BORE is enabled in this kernel via
``CONFIG_SCHED_BORE=y`` and lives in ``kernel/sched/fair.c``.

The android12-5.10 backport is by Pzqqt and includes the
``ANDROID_KABI_USE`` macro reservation that keeps ``struct sched_entity``
ABI-clean for vendor module compatibility.

All BORE tunables live under ``/proc/sys/kernel/`` and are read/write
to root only. Reads from non-root may return EACCES on Android due to
SELinux MAC policy even when file permissions are 0644 — this is
expected; use ``adb shell su -c "cat …"`` for inspection.

``sched_bore``
    Master switch. ``1`` (default) = BORE active; ``0`` = behave as
    stock CFS. Toggleable at runtime.

``sched_burst_exclude_kthreads``
    ``1`` (default) = kernel threads are not subjected to the burst
    penalty. ``0`` = all tasks. Default ``1`` is correct for phones
    where kthreads (workqueues, irq threads) should not be penalised
    for being bursty.

``sched_burst_smoothness_long``
    Smoothing factor (right-shift count) applied to the burst penalty
    over long observation windows. Default ``1``. Higher = slower to
    react, more stable; lower = more reactive, more noise.

``sched_burst_smoothness_short``
    Same as above but for short observation windows. Default ``0``
    (no extra smoothing on short bursts — they are the signal we
    *want* to react to).

``sched_burst_fork_atavistic``
    Fork-time burst inheritance policy. Default ``2`` =
    fork-and-decay: child inherits parent's burst score, then decays
    over its first scheduling windows. ``0`` = clean slate, ``1`` =
    full inherit. ``2`` is best for app-launch / shell-fork patterns.

``sched_burst_penalty_offset``
    Constant subtracted from the raw burst score before it becomes
    the penalty. Default ``22`` (units of 1/64 of a vruntime
    quantum). Higher = more lenient toward bursts; lower = harsher.

``sched_burst_penalty_scale``
    Multiplier (in units of 1/1024) applied to the post-offset burst
    score. Default ``1280`` (≈ 1.25× scale). Higher = stronger
    penalty; lower = weaker.

``sched_burst_cache_lifetime``
    How long (ns) a per-task burst-cache entry stays valid after the
    task last ran. Default ``60000000`` (60 ms). Longer = better
    burst tracking across short sleeps, more memory pressure.

When to retune
--------------

Most users should leave the BORE defaults. Retune only if you observe:

* **Latency spikes on UI under sustained background work** — try
  raising ``sched_burst_penalty_scale`` toward 1536.
* **Audio glitches in voice/music apps under load** — try lowering
  ``sched_burst_penalty_offset`` toward 16.
* **Suspicion that BORE is misbehaving** — set ``sched_bore=0`` for
  an A/B comparison without rebooting.

KSM — Kernel Samepage Merging
=============================

KSM scans anonymous private memory for identical pages and merges
duplicates into a single physical copy. On phones running zRAM, this
trades a small periodic CPU cost for a sometimes-significant
reduction in zRAM working set. Enabled in this kernel via
``CONFIG_KSM=y``.

KSM is **off by default** at boot — userspace must opt in by writing
``1`` to ``/sys/kernel/mm/ksm/run``. Android systems usually do this
from a vendor init script after ``boot_completed``.

All KSM tunables live under ``/sys/kernel/mm/ksm/``.

``run``
    Master switch. ``0`` = stop and unmerge (default), ``1`` = scan
    and merge. Writes to ``2`` unmerge then stop.

``pages_to_scan``
    Number of pages scanned in one wakeup of the ksmd kthread.
    Default ``100``. Higher = faster convergence, more CPU; lower =
    background-friendlier, slower convergence.

``sleep_millisecs``
    How long ksmd sleeps between batches. Default ``200`` in this
    kernel (upstream default is 20). The ``200`` value is appropriate
    for phones where ksmd should be a low-priority background task.

``merge_across_nodes``
    NUMA control. ``1`` (default) = allow merging across NUMA nodes.
    Phones are UMA so this is a no-op.

``pages_shared`` / ``pages_sharing`` / ``pages_unshared`` / ``pages_volatile`` / ``full_scans``
    Read-only statistics. Useful for measuring whether KSM is doing
    real work.

KSM Advisor (mainline 6.7+ backport)
------------------------------------

This kernel backports the KSM Advisor (Stefan Roesch, SuSE). When
enabled, the advisor automatically retunes ``pages_to_scan`` based on
how long each full scan takes versus a configured target, so users no
longer need to babysit ``pages_to_scan`` to balance dedup latency
against ksmd CPU cost.

Default is **disabled** — KSM behaves exactly as before until the
advisor is opted into.

``advisor_mode``
    ``[none] scan-time`` (default) — advisor disabled, manual tuning
    only. Write ``scan-time`` to enable the scan-time advisor. Write
    ``none`` to disable. Switching modes resets the advisor context
    and rebases ``pages_to_scan`` to either the default (none) or
    ``advisor_min_pages_to_scan`` (scan-time).

``advisor_max_cpu``
    Cap on percent of one CPU that ksmd may consume on average,
    enforced by capping the next ``pages_to_scan`` value.
    Default ``70``.

``advisor_min_pages_to_scan``
    Floor for ``pages_to_scan`` when the advisor is active. Default
    ``500``.

``advisor_max_pages_to_scan``
    Ceiling for ``pages_to_scan`` when the advisor is active.
    Default ``30000``.

``advisor_target_scan_time``
    Target time (seconds) to complete one full scan of all candidate
    pages. Default ``200``. **Most important parameter.** Lower =
    advisor scans more aggressively (more dedup, more CPU); higher =
    less aggressive.

When the advisor is active in ``scan-time`` mode, writes to
``pages_to_scan`` are refused with ``-EINVAL`` (the advisor owns the
value). Disable the advisor to set ``pages_to_scan`` manually again.

When to retune (advisor)
~~~~~~~~~~~~~~~~~~~~~~~~

* **You want hands-off KSM** — set ``advisor_mode=scan-time``, leave
  the rest at defaults. ksmd will rebalance itself.
* **You want more aggressive dedup at higher CPU cost** — keep
  advisor enabled, lower ``advisor_target_scan_time`` to 60.
* **You want more conservative dedup** — raise
  ``advisor_target_scan_time`` toward 600. Or lower
  ``advisor_max_cpu`` to 30.
* **Mainline reference**: see ``Documentation/admin-guide/mm/ksm.rst``
  upstream.

When to retune
--------------

* **High dedup ratio expected** (lots of long-lived web tabs, similar
  apps) → raise ``pages_to_scan`` to 200.
* **High ksmd CPU on idle phone** → raise ``sleep_millisecs`` toward
  1000.
* **Low dedup ratio** (single foreground app, small user-space) →
  consider just leaving ksm off.

ADIOS — Adaptive Deadline I/O Scheduler
=======================================

ADIOS is the I/O scheduler that this kernel selects by default for
single-queue (``nr_hw_queues == 1``) block devices, including
``/dev/sd*`` (UFS), ``/dev/mmcblk*`` (eMMC), and zRAM. It lives in
``block/adios.c`` and is built in via ``CONFIG_MQ_IOSCHED_ADIOS=y``
plus the in-kernel default-elevator pin in ``block/elevator.c``.

ADIOS's central idea is to track per-request deadlines but adaptively
relax them under sustained queue depth so a writer cannot starve a
reader (and vice versa). It outperforms ``mq-deadline`` on
phone-class workloads where read-write interleaving is dense.

Tunables are exposed under ``/sys/block/<dev>/queue/iosched/``.
Per-device knobs:

``read_expire_us``
    Time (µs) until a read request is considered expired and gets
    deadline priority. Default 500 ms. Lower = more aggressive
    read-priority, may starve writes; higher = balanced.

``write_expire_us``
    Same for writes. Default 5 s.

``writes_starved``
    Maximum number of consecutive read batches dispatched before a
    write batch is forced. Default 2. Lower = more write fairness,
    higher latency on reads under write pressure.

``front_merges``
    ``1`` (default) = enable front merges (new request inserted in
    front of an existing one); ``0`` = back merges only.

When to retune
--------------

* **UFS devices that frequently spike to high write queue depth**
  (e.g. media-recording apps) → raise ``writes_starved`` to 4.
* **Reads feel laggy during sustained writes** → lower
  ``read_expire_us`` to 250000 (250 ms).

Note: this kernel also drops the upstream-incorrect
``ELEVATOR_F_MQ_AWARE`` flag from ADIOS (commit f4c4c6603), since
ADIOS is single-queue and shouldn't advertise multi-queue awareness.

CAKE — Common Applications Kept Enhanced (default qdisc)
========================================================

CAKE is a sophisticated qdisc that combines fair queueing, COBALT AQM,
and bandwidth shaping. This kernel makes CAKE the **default qdisc**
for all netdevs via ``CONFIG_DEFAULT_NET_SCH="cake"`` (replacing the
upstream default of ``fq_codel``).

The replacement is automatic: every netdev that comes up gets a CAKE
qdisc as its root, applied via the kernel's ``default_qdisc_ops``
hook. To verify::

    cat /proc/sys/net/core/default_qdisc        # cake
    tc qdisc show dev wlan0                     # qdisc cake 0: …

To override per-device (e.g. switch ``wlan0`` to ``fq_codel`` for
A/B testing)::

    tc qdisc replace dev wlan0 root fq_codel

CAKE has many parameters — see ``tc-cake(8)`` man page for the
complete list. The ones most relevant on Android:

``bandwidth <rate>``
    Tells CAKE the link's downstream / upstream rate so it can shape
    correctly. ``unlimited`` (default) = let CAKE auto-detect from
    netdev queue length. For Wi-Fi this is fine. For cellular, where
    the actual rate varies massively, ``unlimited`` is also fine but
    you may set a known-good cap if your carrier rate-limits.

``flowblind`` / ``srchost`` / ``dsthost`` / ``hosts`` / ``flows`` / ``dual-srchost`` / ``dual-dsthost`` / ``triple-isolate``
    Flow isolation policy. Default is ``triple-isolate`` for the
    initial qdisc; this is correct for phone use.

``rtt`` / ``datacentre`` / ``lan`` / ``metro`` / ``regional`` / ``internet`` / ``oceanic`` / ``satellite`` / ``interplanetary``
    RTT tier. Default ``internet`` is correct; lowering to ``metro``
    only helps on a controlled LAN and would worsen cellular.

When to retune
--------------

* **You know your carrier's downstream rate cap** → set
  ``bandwidth <N>mbit`` on the cellular interface.
* **Persistent bufferbloat under sustained download** → switch the
  Wi-Fi qdisc to ``cake bandwidth 50mbit`` (or whatever your AP
  delivers) instead of unlimited.

mm — Memory management phone-class defaults
===========================================

This kernel ships with the following ``mm`` defaults baked in (no
sysctl write needed at boot to take effect; the values are the
kernel-side initialisers in ``mm/vmscan.c``, ``mm/page_alloc.c``,
``mm/page-writeback.c``, and ``fs/dcache.c``):

``vm.swappiness = 100``
    Upstream default 60. Phones with zRAM benefit from very
    aggressive swapping because zRAM is much cheaper than dropping
    file pages and re-reading them from flash on reuse. ``100`` is
    the legitimate ceiling — higher just means "always prefer
    anon-eviction over file-eviction".

``vm.dirty_ratio = 10``
    Upstream default 20. Maximum percentage of system memory that may
    be dirty before writers are stalled. Lower on phones to keep
    flush bursts smaller and less laggy on small NAND.

``vm.dirty_background_ratio = 5``
    Upstream default 10. Percentage at which background writeback
    starts. Halved here for the same reason.

``vm.watermark_scale_factor = 100``
    Upstream default 10. Zone watermark scaling. Larger value = more
    aggressive kswapd, fewer direct-reclaim stalls. Phones see big
    UX wins from raising this since direct reclaim is the worst
    cause of frame drops.

``vm.vfs_cache_pressure = 200``
    Upstream default 100. How aggressively the kernel reclaims
    inode/dentry cache vs page cache. Doubled on phones because the
    user-visible benefit of preserving inode cache is much smaller
    than the cost of pinning anon memory.

``vm.min_free_kbytes`` is left at the kernel auto-computed value
(based on memory size).

When to retune
--------------

* **High zRAM device, lots of background apps** → no change, this is
  the optimised case.
* **Low-memory device dropping foreground apps too eagerly** → lower
  ``vm.swappiness`` toward 80, raise ``vm.dirty_ratio`` toward 15.
* **Persistent ``kswapd`` CPU usage** → lower ``vm.watermark_scale_factor``
  toward 50.

TCP phone-class defaults
========================

This kernel sets the following ``net.ipv4.tcp_*`` defaults at
``init_net`` namespace creation time (in ``net/ipv4/tcp_ipv4.c``):

``net.ipv4.tcp_slow_start_after_idle = 0``
    Upstream default 1. Disables the cwnd reset when a connection is
    idle, so HTTP/2 connection-reuse and websocket pings aren't
    starting from cwnd=10 every time. **The single biggest perceived
    speedup** for phones with persistent connection workloads (chat
    apps, push, websockets).

``net.ipv4.tcp_nometrics_save = 1``
    Upstream default 0. Stops the kernel from caching per-host TCP
    metrics in the route cache. Each new connection starts fresh
    rather than inheriting a possibly-stale RTT estimate from a
    previous session, which is appropriate for phones that move
    between networks frequently.

``net.ipv4.tcp_fastopen = 0x3`` (TFO_CLIENT_ENABLE | TFO_SERVER_ENABLE)
    Upstream default 0x1 (CLIENT only). Enables TFO server-side too
    so apps that use the phone as a TCP server (rare but possible —
    ``adb tcpip``, AirPlay receivers, etc.) can serve a SYN+data
    handshake.

``net.ipv4.tcp_fastopen_blackhole_timeout = 0``
    Upstream default non-zero. Disables the TFO blackhole detector
    backoff on phones, since the cost of one round-trip on a
    blackholed cellular link is acceptable and the detector causes
    weeks-long disabling of TFO on flaky networks.

``net.ipv4.tcp_limit_output_bytes`` is bumped to 4 MB (commit
``d2e8c1962``) so per-socket output queueing is not the bottleneck
on 5G / Wi-Fi 6.

The default congestion control is pinned to **BBR** in
``arch/arm64/configs/gki_defconfig`` via
``CONFIG_DEFAULT_TCP_CONG="bbr"``. This is the in-tree v1 BBR; v3
was tried and reverted for kABI reasons.

When to retune
--------------

* **You measure regression on lossy cellular** → switch to ``cubic``
  with ``echo cubic > /proc/sys/net/ipv4/tcp_congestion_control``.
* **Latency-sensitive workload (gaming, voice)** → consider
  lowering ``tcp_limit_output_bytes`` back to 1 MB.

net.core phone-class defaults
=============================

These live in ``net/core/sock.c`` and ``net/core/dev.c``:

``net.core.rmem_max = 16777216`` (16 MiB)
    Upstream default 256 KiB. Maximum SO_RCVBUF a userspace process
    may set on a socket. Required for high-BDP transfers on Wi-Fi 6
    / 5G NR. The default was sized for a 100 Mbit-era internet.

``net.core.wmem_max = 16777216`` (16 MiB)
    Same for SO_SNDBUF.

``net.core.netdev_max_backlog = 16384``
    Upstream default 1000. Per-CPU input packet backlog before the
    kernel starts dropping. Raised here because softirq is sometimes
    starved by foreground app work on phones; a deeper backlog avoids
    drops during transient stalls.




