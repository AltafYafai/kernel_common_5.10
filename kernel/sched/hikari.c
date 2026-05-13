// SPDX-License-Identifier: GPL-2.0-only
/*
 * kernel/sched/hikari.c - Hikari wake-time policy engine.
 *
 * Hikari observes per-task wake-to-run wait time and reacts via
 * three non-vruntime actuators:
 *
 *   1. uclamp_min boost on the waking task for a short window
 *   2. cpufreq frequency floor hint published via an atomic
 *      notifier chain (a governor like cpufreq_zenith may
 *      subscribe to it)
 *   3. big.LITTLE wake-up CPU placement steering
 *
 * It does NOT modify vruntime, prio, or any CFS fairness state.
 * BORE retains full authority over fairness.
 *
 * Default at runtime is disabled.  When enabled, only tasks that
 * have been opted in (via /proc/<pid>/hikari_enable, top-app
 * cgroup auto-opt-in, or the kernel API) see any actuator fire on
 * them.
 *
 * The hot-path entry points (hikari_on_enqueue / hikari_on_dequeue
 * / hikari_on_wake_up / hikari_select_cpu) early-return on a single
 * READ_ONCE of the master enable flag, so when Hikari is off the
 * cost is one predictable branch.
 *
 * Concurrency model:
 *   - Per-task fields (sched.h slots 2/3) are touched only while
 *     the task's rq lock is held (enqueue/dequeue/wake hooks all
 *     run under it), or while the task is current on its own CPU.
 *     READ_ONCE/WRITE_ONCE without atomics is sufficient.
 *   - Per-CPU state lives in DEFINE_PER_CPU and is touched only
 *     from the matching CPU's scheduler hot path, or under
 *     this_cpu_ptr()/per_cpu_ptr() with appropriate care from
 *     other CPUs for read-only observability reads.
 *   - Sysctl values are plain u32 read with READ_ONCE on hot
 *     paths.  No locking required.
 *   - Kill flag uses atomic_t for cross-CPU consistency.
 *
 * Init ordering (see hikari_init()) mirrors the Zenith hardening
 * pattern: every piece of state is initialised before the master
 * `init_complete` flag is set; every hot path reads `init_complete`
 * first.
 */

#define pr_fmt(fmt) "hikari: " fmt

#include <linux/atomic.h>
#include <linux/cgroup.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/fs.h>
#include <linux/hikari.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/notifier.h>
#include <linux/percpu.h>
#include <linux/pid.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/sched/topology.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/sysctl.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "sched.h"

/*
 * Per-CPU state.  All u64 timestamps are jiffies values, compared
 * with time_after / time_before helpers so wraparound is handled.
 *
 * wake_demand_ewma_ns is a u32 nanoseconds EWMA, saturating at
 * U32_MAX.  ~4.29s upper bound -- any task waiting longer than
 * that on enqueue almost certainly isn't a task we want to chase.
 */
struct hikari_pcpu {
	u32		wake_demand_ewma_ns;
	unsigned int	wake_floor_khz;
	unsigned long	wake_floor_until_jiffies;
	unsigned long	audio_active_until_jiffies;
	atomic_t	boost_count;
	atomic_t	hint_count;
};

static DEFINE_PER_CPU(struct hikari_pcpu, hikari_pcpu);

/*
 * Sysctl-backed tunables.  All u32, all proc_douintvec_minmax.
 *
 * The "_value" suffix on hikari_enable_value is intentional: the
 * accessor hikari_enabled() reads it via READ_ONCE.  Sysctl writes
 * pass through proc_douintvec_minmax which is a WRITE_ONCE-like
 * store on the slow path.
 */
static unsigned int hikari_enable_value;
static unsigned int hikari_wake_threshold_us = 1000;
static unsigned int hikari_uclamp_boost_pct = 30;
static unsigned int hikari_uclamp_ttl_ms = 16;
static unsigned int hikari_floor_khz_cluster0 = 800000;
static unsigned int hikari_floor_khz_cluster1 = 1200000;
static unsigned int hikari_floor_ttl_ms = 50;
static unsigned int hikari_placement_enable;
static unsigned int hikari_audio_intensify = 1;
static unsigned int hikari_topapp_auto_optin = 1;
static unsigned int hikari_topapp_auto_optout;
static unsigned int hikari_ewma_shift = 3;

static const unsigned int hikari_uint_zero = 0;
static const unsigned int hikari_uint_one  = 1;
static const unsigned int hikari_threshold_min = 100;
static const unsigned int hikari_threshold_max = 100000;
static const unsigned int hikari_boost_pct_max = 50;
static const unsigned int hikari_uclamp_ttl_min = 1;
static const unsigned int hikari_uclamp_ttl_max = 200;
static const unsigned int hikari_floor_khz_max_c0 = 2000000;
static const unsigned int hikari_floor_khz_max_c1 = 3000000;
static const unsigned int hikari_floor_ttl_min = 1;
static const unsigned int hikari_floor_ttl_max = 500;
static const unsigned int hikari_ewma_shift_min = 1;
static const unsigned int hikari_ewma_shift_max = 7;

/*
 * Master state.
 *
 * init_complete is set ONCE, after every piece of init has run
 * and is visible to all CPUs.  Hot paths read it first; reading a
 * stale "false" value just means we early-return early in boot.
 * Reading a stale "true" value never happens because the store is
 * the last init step.
 *
 * kill_flag is set if self-disable triggers.  Once set it is
 * never cleared without a reboot; the master enable sysctl can be
 * flipped back to 1 by an admin who wants to clear the lockout
 * explicitly (see hikari_enable_sysctl_handler).
 */
static bool			hikari_init_complete __read_mostly;
static atomic_t			hikari_kill_flag = ATOMIC_INIT(0);
static u8			hikari_disable_reason_global __read_mostly;
static cpumask_t		hikari_big_cluster __read_mostly;
static cpumask_t		hikari_little_cluster __read_mostly;
static int			hikari_max_big_cpu __read_mostly = -1;

static ATOMIC_NOTIFIER_HEAD(hikari_cpufreq_chain);

static inline bool hikari_is_killed(void)
{
	return atomic_read(&hikari_kill_flag) != 0;
}

static inline void hikari_self_disable(u8 reason)
{
	/* Cmpxchg-ish: only the first failing path records the reason. */
	if (atomic_xchg(&hikari_kill_flag, 1) == 0) {
		hikari_disable_reason_global = reason;
		pr_err_once("self-disabled (reason=%u)\n", reason);
#ifdef CONFIG_HIKARI_DEBUG
		BUG();
#endif
	}
}

bool hikari_enabled(void)
{
	if (!READ_ONCE(hikari_init_complete))
		return false;
	if (hikari_is_killed())
		return false;
	return READ_ONCE(hikari_enable_value) != 0;
}
EXPORT_SYMBOL_GPL(hikari_enabled);

/*
 * Truncate jiffies to u32 for storage in task_struct slot.
 * On HZ=250 this wraps every ~198 days, which exceeds any
 * reasonable boost TTL by orders of magnitude.
 */
static inline u32 hikari_now_token(void)
{
	return (u32)((unsigned long)jiffies);
}

static inline bool hikari_token_active(u32 expiry_token)
{
	u32 now = hikari_now_token();

	if (!expiry_token)
		return false;
	return (s32)(expiry_token - now) > 0;
}

static inline u32 hikari_token_add_ms(unsigned int ms)
{
	u32 ticks = (u32)msecs_to_jiffies(ms);
	u32 expiry = hikari_now_token() + ticks;

	/* avoid the "0 means unset" sentinel */
	if (!expiry)
		expiry = 1;
	return expiry;
}

static inline bool hikari_task_active(struct task_struct *p)
{
	u32 flags;

	if (!hikari_enabled())
		return false;
	if (!p)
		return false;
	flags = READ_ONCE(p->hikari_flags);
	return (flags & HIKARI_FLAG_OPT_IN) != 0;
}

static void hikari_set_flag(struct task_struct *p, u32 bit, bool on);

static inline bool hikari_in_top_app(struct task_struct *p)
{
	return (READ_ONCE(p->hikari_flags) & HIKARI_FLAG_FOREGROUND) != 0;
}

/*
 * Lazy top-app cgroup auto-opt-in.  Called from hikari_on_enqueue()
 * under rq_lock, where p->sched_task_group is stable.
 *
 * Walks task_group(p)->css.cgroup->kn->name and compares to
 * "top-app".  Cost: one pointer chain + 7-byte strcmp, only when
 * hikari_topapp_auto_optin is set.
 *
 * State transitions:
 *   enter top-app  -> set FOREGROUND + OPT_IN
 *   leave top-app  -> clear FOREGROUND only (OPT_IN may be
 *                     user-set via /proc and must survive)
 */
static inline void hikari_lazy_topapp_update(struct task_struct *p)
{
#ifdef CONFIG_CGROUP_SCHED
	struct task_group *tg;
	struct cgroup *cgrp;
	bool in_topapp;
	u32 flags;

	if (!READ_ONCE(hikari_topapp_auto_optin))
		return;

	tg = task_group(p);
	if (!tg)
		return;
	cgrp = tg->css.cgroup;
	if (!cgrp || !cgrp->kn || !cgrp->kn->name)
		return;

	in_topapp = (strcmp(cgrp->kn->name, "top-app") == 0);
	flags = READ_ONCE(p->hikari_flags);

	if (in_topapp && !(flags & HIKARI_FLAG_FOREGROUND))
		hikari_set_flag(p, HIKARI_FLAG_FOREGROUND | HIKARI_FLAG_OPT_IN,
				true);
	else if (!in_topapp && (flags & HIKARI_FLAG_FOREGROUND))
		hikari_set_flag(p, HIKARI_FLAG_FOREGROUND, false);
#endif
}

static inline unsigned int hikari_uclamp_boost_value(void)
{
	unsigned int pct = READ_ONCE(hikari_uclamp_boost_pct);

	if (pct > hikari_boost_pct_max)
		pct = hikari_boost_pct_max;
	return (SCHED_CAPACITY_SCALE * pct) / 100;
}

/*
 * Returns the additional uclamp_min that Hikari is currently
 * boosting on this task, or 0 if no boost is active.  Safe to call
 * from the uclamp_eff_get() hot path.
 */
unsigned int hikari_uclamp_boost_amount(struct task_struct *p)
{
	u32 until;

	if (!IS_ENABLED(CONFIG_HIKARI_UCLAMP))
		return 0;
	if (!hikari_enabled())
		return 0;
	if (!p)
		return 0;

	until = READ_ONCE(p->hikari_boost_until_ns);
	if (!hikari_token_active(until)) {
		/* Lazy clear: avoid storing through a hot read but on
		 * the next non-hot visit we'll zero it.
		 */
		return 0;
	}
	return hikari_uclamp_boost_value();
}
EXPORT_SYMBOL_GPL(hikari_uclamp_boost_amount);

static inline void hikari_apply_uclamp_boost(struct task_struct *p)
{
	if (!IS_ENABLED(CONFIG_HIKARI_UCLAMP))
		return;
	WRITE_ONCE(p->hikari_boost_until_ns,
		   hikari_token_add_ms(READ_ONCE(hikari_uclamp_ttl_ms)));
	atomic_inc(&this_cpu_ptr(&hikari_pcpu)->boost_count);
}

static inline void hikari_publish_freq_hint(unsigned int cpu, u32 demand_ns)
{
	struct hikari_freq_hint hint;
	unsigned int floor;

	if (!IS_ENABLED(CONFIG_HIKARI_ZENITH_HINT))
		return;
	if (cpu >= nr_cpu_ids)
		return;

	floor = cpumask_test_cpu(cpu, &hikari_big_cluster)
		? READ_ONCE(hikari_floor_khz_cluster1)
		: READ_ONCE(hikari_floor_khz_cluster0);

	if (!floor)
		return;

	hint.cpu       = cpu;
	hint.floor_khz = floor;
	hint.ttl_ms    = READ_ONCE(hikari_floor_ttl_ms);
	hint.demand_ns = demand_ns;

	atomic_inc(&per_cpu_ptr(&hikari_pcpu, cpu)->hint_count);
	WRITE_ONCE(per_cpu_ptr(&hikari_pcpu, cpu)->wake_floor_khz, floor);
	WRITE_ONCE(per_cpu_ptr(&hikari_pcpu, cpu)->wake_floor_until_jiffies,
		   jiffies + msecs_to_jiffies(hint.ttl_ms));

	atomic_notifier_call_chain(&hikari_cpufreq_chain,
				   HIKARI_NOTIFIER_WAKE_DEMAND, &hint);
}

unsigned int hikari_get_floor_khz(unsigned int cpu)
{
	struct hikari_pcpu *pc;
	unsigned long until;
	unsigned int khz;

	if (!IS_ENABLED(CONFIG_HIKARI_ZENITH_HINT))
		return 0;
	if (!hikari_enabled())
		return 0;
	if (cpu >= nr_cpu_ids)
		return 0;

	pc = per_cpu_ptr(&hikari_pcpu, cpu);
	until = READ_ONCE(pc->wake_floor_until_jiffies);
	if (!until || time_after_eq(jiffies, until))
		return 0;

	khz = READ_ONCE(pc->wake_floor_khz);
	return khz;
}
EXPORT_SYMBOL_GPL(hikari_get_floor_khz);

/*
 * Mark the CPU `p` is currently running on as audio-active for
 * the next ttl_ms (using the floor TTL for symmetry).  Called
 * from hikari_mark_audio() and from the scheduler when we
 * observe an audio-tagged task being enqueued.
 */
static inline void hikari_pcpu_mark_audio(unsigned int cpu)
{
	if (cpu >= nr_cpu_ids)
		return;
	WRITE_ONCE(per_cpu_ptr(&hikari_pcpu, cpu)->audio_active_until_jiffies,
		   jiffies + msecs_to_jiffies(READ_ONCE(hikari_floor_ttl_ms)));
}

static inline bool hikari_cpu_is_audio_active(unsigned int cpu)
{
	unsigned long until;

	if (cpu >= nr_cpu_ids)
		return false;
	until = READ_ONCE(per_cpu_ptr(&hikari_pcpu, cpu)->audio_active_until_jiffies);
	return until && time_before(jiffies, until);
}

/* ------------------------------------------------------------ */
/* Hot-path hooks called from the scheduler.                    */
/* ------------------------------------------------------------ */

void hikari_on_enqueue(struct task_struct *p, struct rq *rq)
{
	u32 now32;

	if (!hikari_enabled())
		return;
	if (!p || !rq)
		return;

	hikari_lazy_topapp_update(p);

	if (!(READ_ONCE(p->hikari_flags) & HIKARI_FLAG_OPT_IN))
		return;

	now32 = (u32)rq_clock_task(rq);
	WRITE_ONCE(p->hikari_last_enqueue_ns, now32 ? now32 : 1);

	/*
	 * Lazy boost-expiry clear: if a stale "active" boost from
	 * before a long sleep is still set, clear it now.  This is
	 * the only safety against u32-jiffies wraparound for
	 * boost_until.
	 */
	if (p->hikari_boost_until_ns &&
	    !hikari_token_active(p->hikari_boost_until_ns))
		WRITE_ONCE(p->hikari_boost_until_ns, 0);

	if (READ_ONCE(p->hikari_flags) & HIKARI_FLAG_AUDIO_TAGGED)
		hikari_pcpu_mark_audio(task_cpu(p));
}
EXPORT_SYMBOL_GPL(hikari_on_enqueue);

void hikari_on_dequeue(struct task_struct *p, struct rq *rq)
{
	u32 last, now32, delta, ewma, threshold_ns;

	if (!hikari_task_active(p))
		return;
	if (!rq)
		return;

	last = READ_ONCE(p->hikari_last_enqueue_ns);
	if (!last)
		return;

	now32 = (u32)rq_clock_task(rq);
	/*
	 * Signed wraparound-safe subtraction.  delta is "time
	 * spent waiting on the rq from enqueue to dequeue (pick)".
	 * If negative or zero, ignore (clock skew during migration
	 * or a degenerate sample).
	 */
	if ((s32)(now32 - last) <= 0)
		return;
	delta = now32 - last;

	/*
	 * EWMA with tunable alpha = 1/(1<<shift).  Default shift=3
	 * gives alpha=1/8: new = (7*old + delta) / 8.  Lower shift
	 * = more reactive, higher = lazier smoothing.
	 * Saturates at U32_MAX naturally because all values are u32.
	 */
	{
		unsigned int shift = READ_ONCE(hikari_ewma_shift);
		u32 weight;

		if (shift < hikari_ewma_shift_min)
			shift = hikari_ewma_shift_min;
		if (shift > hikari_ewma_shift_max)
			shift = hikari_ewma_shift_max;
		weight = (1u << shift) - 1;

		ewma = READ_ONCE(p->hikari_wait_ewma_ns);
		if (ewma > U32_MAX - delta) {
			ewma = U32_MAX;
		} else {
			ewma = ((ewma * weight) + delta) >> shift;
		}
	}
	WRITE_ONCE(p->hikari_wait_ewma_ns, ewma);
	WRITE_ONCE(p->hikari_last_enqueue_ns, 0);

	threshold_ns = READ_ONCE(hikari_wake_threshold_us);
	if (threshold_ns > U32_MAX / 1000)
		threshold_ns = U32_MAX / 1000;
	threshold_ns *= 1000;

	if (ewma > threshold_ns) {
		hikari_apply_uclamp_boost(p);
		hikari_publish_freq_hint(task_cpu(p), ewma);
	}
}
EXPORT_SYMBOL_GPL(hikari_on_dequeue);

void hikari_on_wake_up(struct task_struct *p, int target_cpu)
{
	if (!hikari_task_active(p))
		return;
	if (target_cpu < 0 || target_cpu >= nr_cpu_ids)
		return;

	if (READ_ONCE(p->hikari_flags) & HIKARI_FLAG_AUDIO_TAGGED)
		hikari_pcpu_mark_audio(target_cpu);
}
EXPORT_SYMBOL_GPL(hikari_on_wake_up);

/*
 * Big.LITTLE placement.  Engage condition:
 *
 *   placement_enable=1
 *   AND task is opted in
 *   AND task's recent EWMA exceeds the wake threshold
 *   AND (task is foreground OR a sibling CPU is audio-active)
 *
 * If engaged, returns a preferred CPU drawn from the big-cluster
 * intersected with the task's allowed mask.  If the intersection
 * is empty (task is pinned to little cores) returns -1.
 */
int hikari_select_cpu(struct task_struct *p, int prev_cpu, int wake_flags)
{
	u32 ewma, threshold_ns;
	int cpu;

	if (!IS_ENABLED(CONFIG_HIKARI_PLACEMENT))
		return -1;
	if (!READ_ONCE(hikari_placement_enable))
		return -1;
	if (!hikari_task_active(p))
		return -1;

	ewma = READ_ONCE(p->hikari_wait_ewma_ns);
	threshold_ns = READ_ONCE(hikari_wake_threshold_us) * 1000;
	if (ewma <= threshold_ns)
		return -1;

	if (!hikari_in_top_app(p) &&
	    !hikari_cpu_is_audio_active(prev_cpu) &&
	    !(READ_ONCE(p->hikari_flags) & HIKARI_FLAG_AUDIO_TAGGED))
		return -1;

	if (cpumask_empty(&hikari_big_cluster))
		return -1;

	for_each_cpu_and(cpu, &hikari_big_cluster, p->cpus_ptr) {
		if (!cpu_online(cpu))
			continue;
		return cpu;
	}
	return -1;
}
EXPORT_SYMBOL_GPL(hikari_select_cpu);

/* ------------------------------------------------------------ */
/* Setter API.                                                  */
/* ------------------------------------------------------------ */

static void hikari_set_flag(struct task_struct *p, u32 bit, bool on)
{
	u32 cur, new;

	if (!p)
		return;
	do {
		cur = READ_ONCE(p->hikari_flags);
		new = on ? (cur | bit) : (cur & ~bit);
		if (new == cur)
			return;
	} while (cmpxchg(&p->hikari_flags, cur, new) != cur);
}

void hikari_set_opt_in(struct task_struct *p, bool opt_in)
{
	hikari_set_flag(p, HIKARI_FLAG_OPT_IN, opt_in);
}
EXPORT_SYMBOL_GPL(hikari_set_opt_in);

void hikari_mark_audio(struct task_struct *p, bool tagged)
{
	hikari_set_flag(p, HIKARI_FLAG_AUDIO_TAGGED, tagged);
	if (tagged && p)
		hikari_pcpu_mark_audio(task_cpu(p));
}
EXPORT_SYMBOL_GPL(hikari_mark_audio);

void hikari_mark_foreground(struct task_struct *p, bool tagged)
{
	hikari_set_flag(p, HIKARI_FLAG_FOREGROUND, tagged);

	/*
	 * Top-app auto-opt-in: tagging foreground also opts the
	 * task in for Hikari if the auto-opt-in sysctl is set.
	 * Conversely, untagging foreground does NOT opt out --
	 * the user may have explicitly opted in via /proc.
	 */
	if (tagged && READ_ONCE(hikari_topapp_auto_optin))
		hikari_set_flag(p, HIKARI_FLAG_OPT_IN, true);
}
EXPORT_SYMBOL_GPL(hikari_mark_foreground);

/* ------------------------------------------------------------ */
/* Notifier registration.                                       */
/* ------------------------------------------------------------ */

int hikari_register_cpufreq_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&hikari_cpufreq_chain, nb);
}
EXPORT_SYMBOL_GPL(hikari_register_cpufreq_notifier);

int hikari_unregister_cpufreq_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&hikari_cpufreq_chain, nb);
}
EXPORT_SYMBOL_GPL(hikari_unregister_cpufreq_notifier);

/* ------------------------------------------------------------ */
/* Sanity check.                                                */
/* ------------------------------------------------------------ */

static void hikari_sanity_check_pcpu(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		struct hikari_pcpu *pc = per_cpu_ptr(&hikari_pcpu, cpu);

		if (!pc) {
			hikari_self_disable(HIKARI_DISABLE_SANITY_PCPU);
			return;
		}
	}
}

/* ------------------------------------------------------------ */
/* Sysctls.                                                     */
/* ------------------------------------------------------------ */

static int hikari_enable_sysctl_handler(struct ctl_table *table, int write,
					void *buffer, size_t *lenp, loff_t *ppos)
{
	int ret = proc_douintvec_minmax(table, write, buffer, lenp, ppos);

	if (write && !ret) {
		if (READ_ONCE(hikari_enable_value) != 0)
			atomic_set(&hikari_kill_flag, 0);
	}
	return ret;
}

static struct ctl_table hikari_sysctl_table[] = {
	{
		.procname	= "hikari_enable",
		.data		= &hikari_enable_value,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= hikari_enable_sysctl_handler,
		.extra1		= (void *)&hikari_uint_zero,
		.extra2		= (void *)&hikari_uint_one,
	},
	{
		.procname	= "hikari_wake_threshold_us",
		.data		= &hikari_wake_threshold_us,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_threshold_min,
		.extra2		= (void *)&hikari_threshold_max,
	},
	{
		.procname	= "hikari_uclamp_boost_pct",
		.data		= &hikari_uclamp_boost_pct,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_uint_zero,
		.extra2		= (void *)&hikari_boost_pct_max,
	},
	{
		.procname	= "hikari_uclamp_ttl_ms",
		.data		= &hikari_uclamp_ttl_ms,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_uclamp_ttl_min,
		.extra2		= (void *)&hikari_uclamp_ttl_max,
	},
	{
		.procname	= "hikari_floor_khz_cluster0",
		.data		= &hikari_floor_khz_cluster0,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_uint_zero,
		.extra2		= (void *)&hikari_floor_khz_max_c0,
	},
	{
		.procname	= "hikari_floor_khz_cluster1",
		.data		= &hikari_floor_khz_cluster1,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_uint_zero,
		.extra2		= (void *)&hikari_floor_khz_max_c1,
	},
	{
		.procname	= "hikari_floor_ttl_ms",
		.data		= &hikari_floor_ttl_ms,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_floor_ttl_min,
		.extra2		= (void *)&hikari_floor_ttl_max,
	},
	{
		.procname	= "hikari_placement_enable",
		.data		= &hikari_placement_enable,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_uint_zero,
		.extra2		= (void *)&hikari_uint_one,
	},
	{
		.procname	= "hikari_audio_intensify",
		.data		= &hikari_audio_intensify,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_uint_zero,
		.extra2		= (void *)&hikari_uint_one,
	},
	{
		.procname	= "hikari_ewma_shift",
		.data		= &hikari_ewma_shift,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_ewma_shift_min,
		.extra2		= (void *)&hikari_ewma_shift_max,
	},
	{
		.procname	= "hikari_topapp_auto_optin",
		.data		= &hikari_topapp_auto_optin,
		.maxlen		= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_douintvec_minmax,
		.extra1		= (void *)&hikari_uint_zero,
		.extra2		= (void *)&hikari_uint_one,
	},
	{ }
};

/* ------------------------------------------------------------ */
/* Init.                                                        */
/* ------------------------------------------------------------ */

static void __init hikari_discover_clusters(void)
{
	unsigned long max_cap = 0, min_cap = ULONG_MAX;
	int cpu;

	cpumask_clear(&hikari_big_cluster);
	cpumask_clear(&hikari_little_cluster);

	for_each_possible_cpu(cpu) {
		unsigned long cap = arch_scale_cpu_capacity(cpu);

		if (cap > max_cap)
			max_cap = cap;
		if (cap < min_cap)
			min_cap = cap;
	}

	if (max_cap == min_cap) {
		/* uniprocessor / SMP-symmetric: no big cluster. */
		pr_info("symmetric capacity, big.LITTLE placement disabled\n");
		return;
	}

	for_each_possible_cpu(cpu) {
		unsigned long cap = arch_scale_cpu_capacity(cpu);

		if (cap == max_cap) {
			cpumask_set_cpu(cpu, &hikari_big_cluster);
			hikari_max_big_cpu = cpu;
		} else {
			cpumask_set_cpu(cpu, &hikari_little_cluster);
		}
	}

	pr_info("big cluster: %*pbl, little cluster: %*pbl\n",
		cpumask_pr_args(&hikari_big_cluster),
		cpumask_pr_args(&hikari_little_cluster));
}

/* ------------------------------------------------------------ */
/* Per-task /proc helpers (callable from fs/proc/base.c).       */
/* ------------------------------------------------------------ */

/*
 * Print a human-readable per-task stats blob to @m.  Output
 * format is one "key: value" line per attribute; the keys are
 * stable for parsing.
 */
void hikari_seq_print_stats(struct seq_file *m, struct task_struct *p)
{
	u32 flags, ewma, last_enq, boost_until;

	if (!p) {
		seq_puts(m, "hikari: invalid task\n");
		return;
	}

	flags       = READ_ONCE(p->hikari_flags);
	ewma        = READ_ONCE(p->hikari_wait_ewma_ns);
	last_enq    = READ_ONCE(p->hikari_last_enqueue_ns);
	boost_until = READ_ONCE(p->hikari_boost_until_ns);

	seq_printf(m, "hikari_enabled_global: %u\n",
		   hikari_enabled() ? 1U : 0U);
	seq_printf(m, "hikari_opt_in: %u\n",
		   (flags & HIKARI_FLAG_OPT_IN) ? 1U : 0U);
	seq_printf(m, "hikari_audio_tagged: %u\n",
		   (flags & HIKARI_FLAG_AUDIO_TAGGED) ? 1U : 0U);
	seq_printf(m, "hikari_foreground: %u\n",
		   (flags & HIKARI_FLAG_FOREGROUND) ? 1U : 0U);
	seq_printf(m, "hikari_wait_ewma_ns: %u\n", ewma);
	seq_printf(m, "hikari_last_enqueue_ns: %u\n", last_enq);
	seq_printf(m, "hikari_boost_active: %u\n",
		   hikari_token_active(boost_until) ? 1U : 0U);
	seq_printf(m, "hikari_boost_until_jiffies: %u\n", boost_until);
}
EXPORT_SYMBOL_GPL(hikari_seq_print_stats);

/* Helper for /proc/<pid>/hikari_enable + /proc/<pid>/hikari_audio. */
u32 hikari_task_get_flag(struct task_struct *p, u32 bit)
{
	if (!p)
		return 0;
	return (READ_ONCE(p->hikari_flags) & bit) ? 1U : 0U;
}
EXPORT_SYMBOL_GPL(hikari_task_get_flag);

void hikari_task_set_flag(struct task_struct *p, u32 bit, bool on)
{
	hikari_set_flag(p, bit, on);
}
EXPORT_SYMBOL_GPL(hikari_task_set_flag);

/* ------------------------------------------------------------ */
/* /sys/kernel/hikari/ observability.                           */
/* ------------------------------------------------------------ */

static struct kobject *hikari_kobj;

static ssize_t enabled_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	return sysfs_emit(buf, "%u\n", hikari_enabled() ? 1U : 0U);
}

static ssize_t disabled_reason_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n",
			  (unsigned int)READ_ONCE(hikari_disable_reason_global));
}

static ssize_t version_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	return sysfs_emit(buf, "%s\n", "hikari-v2");
}

static ssize_t total_boost_count_show(struct kobject *kobj,
				      struct kobj_attribute *attr, char *buf)
{
	unsigned long long sum = 0;
	int cpu;

	for_each_possible_cpu(cpu)
		sum += (unsigned long long)
			atomic_read(&per_cpu_ptr(&hikari_pcpu, cpu)->boost_count);
	return sysfs_emit(buf, "%llu\n", sum);
}

static ssize_t total_hint_count_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	unsigned long long sum = 0;
	int cpu;

	for_each_possible_cpu(cpu)
		sum += (unsigned long long)
			atomic_read(&per_cpu_ptr(&hikari_pcpu, cpu)->hint_count);
	return sysfs_emit(buf, "%llu\n", sum);
}

static ssize_t big_cluster_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%*pbl\n",
			  cpumask_pr_args(&hikari_big_cluster));
}

static ssize_t little_cluster_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%*pbl\n",
			  cpumask_pr_args(&hikari_little_cluster));
}

static struct kobj_attribute hikari_attr_enabled =
	__ATTR(enabled, 0444, enabled_show, NULL);
static struct kobj_attribute hikari_attr_disabled_reason =
	__ATTR(disabled_reason, 0444, disabled_reason_show, NULL);
static struct kobj_attribute hikari_attr_version =
	__ATTR(version, 0444, version_show, NULL);
static struct kobj_attribute hikari_attr_total_boost_count =
	__ATTR(total_boost_count, 0444, total_boost_count_show, NULL);
static struct kobj_attribute hikari_attr_total_hint_count =
	__ATTR(total_hint_count, 0444, total_hint_count_show, NULL);
static struct kobj_attribute hikari_attr_big_cluster =
	__ATTR(big_cluster, 0444, big_cluster_show, NULL);
static struct kobj_attribute hikari_attr_little_cluster =
	__ATTR(little_cluster, 0444, little_cluster_show, NULL);

static struct attribute *hikari_sysfs_attrs[] = {
	&hikari_attr_enabled.attr,
	&hikari_attr_disabled_reason.attr,
	&hikari_attr_version.attr,
	&hikari_attr_total_boost_count.attr,
	&hikari_attr_total_hint_count.attr,
	&hikari_attr_big_cluster.attr,
	&hikari_attr_little_cluster.attr,
	NULL,
};

static const struct attribute_group hikari_sysfs_group = {
	.attrs = hikari_sysfs_attrs,
};

static int __init hikari_sysfs_init(void)
{
	int ret;

	hikari_kobj = kobject_create_and_add("hikari", kernel_kobj);
	if (!hikari_kobj) {
		pr_warn("failed to create /sys/kernel/hikari\n");
		return -ENOMEM;
	}

	ret = sysfs_create_group(hikari_kobj, &hikari_sysfs_group);
	if (ret) {
		kobject_put(hikari_kobj);
		hikari_kobj = NULL;
		pr_warn("failed to create /sys/kernel/hikari attributes (%d)\n",
			ret);
		return ret;
	}

	return 0;
}

static int __init hikari_init(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		struct hikari_pcpu *pc = per_cpu_ptr(&hikari_pcpu, cpu);

		memset(pc, 0, sizeof(*pc));
		atomic_set(&pc->boost_count, 0);
		atomic_set(&pc->hint_count, 0);
	}

	hikari_discover_clusters();

	if (!register_sysctl("kernel", hikari_sysctl_table)) {
		pr_err("failed to register sysctl entries\n");
		return -ENOMEM;
	}

	/*
	 * Sysfs init: non-fatal on failure.  /sys/kernel/hikari is
	 * observability-only; if it fails to create, the rest of
	 * Hikari is still functional.  The warning is logged.
	 */
	(void)hikari_sysfs_init();

	hikari_sanity_check_pcpu();
	if (hikari_is_killed()) {
		pr_err("init sanity check failed\n");
		return -EINVAL;
	}

	smp_wmb();
	WRITE_ONCE(hikari_init_complete, true);

	pr_info("initialised (master enable=%u)\n",
		READ_ONCE(hikari_enable_value));
	return 0;
}
late_initcall(hikari_init);
