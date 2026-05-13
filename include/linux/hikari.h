/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * include/linux/hikari.h - Hikari wake-time policy engine public API.
 *
 * Hikari observes per-task wake-to-run wait time and reacts via
 * non-vruntime actuators:
 *
 *   - uclamp_min boost (per-task, time-limited)
 *   - cpufreq frequency floor hint (via notifier chain, picked up by
 *     a cpufreq governor if present)
 *   - big.LITTLE wake-up CPU placement steering
 *
 * Hikari deliberately does NOT modify vruntime, prio, or any CFS
 * fairness state.  BORE retains full authority over fairness.
 *
 * Default state at runtime: disabled.  Set kernel.hikari_enable=1
 * to activate.  Even with the master switch on, each task must be
 * explicitly opted in (per-PID flag, top-app cgroup auto-opt-in,
 * or kernel API tag) for actuators to fire on it.
 *
 * All hooks early-return when the kernel was built without HIKARI
 * (the stub versions below).  When built but disabled at runtime
 * they early-return after one READ_ONCE.  Cost on the hot path is
 * a predictable branch.
 */
#ifndef _LINUX_HIKARI_H
#define _LINUX_HIKARI_H

#include <linux/types.h>
#include <linux/cpumask.h>

struct task_struct;
struct rq;

/* Bits packed into task_struct::hikari_flags. */
#define HIKARI_FLAG_OPT_IN		(1u << 0)
#define HIKARI_FLAG_AUDIO_TAGGED	(1u << 1)
#define HIKARI_FLAG_FOREGROUND		(1u << 2)
/* Bits 3..31 reserved for future use; must stay zero in current code. */

/*
 * Reasons Hikari may self-disable at runtime.  Exposed via
 * /sys/kernel/hikari/disabled_reason and recorded in dmesg once.
 */
#define HIKARI_DISABLE_NONE		0
#define HIKARI_DISABLE_SANITY_PCPU	1
#define HIKARI_DISABLE_SANITY_TASK	2
#define HIKARI_DISABLE_SANITY_BOOST	3
#define HIKARI_DISABLE_SANITY_NOTIFIER	4
#define HIKARI_DISABLE_USER		5

/*
 * Notifier chain payload for cpufreq governor consumers.  A
 * governor (e.g. cpufreq_zenith) registers with
 * hikari_register_cpufreq_notifier(); when Hikari observes a
 * wake-time demand on a CPU it fires the chain with this struct
 * as the data argument.
 *
 * floor_khz is a *hint*, not a guarantee.  The consumer is free
 * to clamp it, ignore it, or apply it for less than ttl_ms.
 */
struct hikari_freq_hint {
	unsigned int cpu;
	unsigned int floor_khz;
	unsigned int ttl_ms;
	unsigned int demand_ns;
};

#define HIKARI_NOTIFIER_WAKE_DEMAND	1

#ifdef CONFIG_HIKARI

/* Public observer hooks called from the scheduler. */
void hikari_on_enqueue(struct task_struct *p, struct rq *rq);
void hikari_on_dequeue(struct task_struct *p, struct rq *rq);
void hikari_on_wake_up(struct task_struct *p, int target_cpu);

/*
 * Wake-up CPU placement helper.  Returns a CPU to target for
 * placement, or -1 if Hikari has no opinion (caller should fall
 * back to vanilla scheduler logic).  Never returns an invalid or
 * offline CPU; if it cannot honour the placement preference it
 * returns -1 instead.
 */
int hikari_select_cpu(struct task_struct *p, int prev_cpu, int wake_flags);

/* Kernel-side API for tagging an audio task. */
void hikari_mark_audio(struct task_struct *p, bool tagged);

/* Kernel-side API for tagging a foreground task. */
void hikari_mark_foreground(struct task_struct *p, bool tagged);

/* Kernel-side opt-in/opt-out API. */
void hikari_set_opt_in(struct task_struct *p, bool opt_in);

/*
 * Notifier chain registration.  Governors call this once at init.
 * Callbacks receive an event constant (currently only
 * HIKARI_NOTIFIER_WAKE_DEMAND) and a pointer to a
 * struct hikari_freq_hint.
 */
struct notifier_block;
int hikari_register_cpufreq_notifier(struct notifier_block *nb);
int hikari_unregister_cpufreq_notifier(struct notifier_block *nb);

/* Master enable check, useful for callers that want to skip work. */
bool hikari_enabled(void);

#else /* !CONFIG_HIKARI */

static inline void hikari_on_enqueue(struct task_struct *p, struct rq *rq) { }
static inline void hikari_on_dequeue(struct task_struct *p, struct rq *rq) { }
static inline void hikari_on_wake_up(struct task_struct *p, int target_cpu) { }
static inline int  hikari_select_cpu(struct task_struct *p, int prev_cpu,
				     int wake_flags) { return -1; }
static inline void hikari_mark_audio(struct task_struct *p, bool tagged) { }
static inline void hikari_mark_foreground(struct task_struct *p, bool tagged) { }
static inline void hikari_set_opt_in(struct task_struct *p, bool opt_in) { }

struct notifier_block;
static inline int hikari_register_cpufreq_notifier(struct notifier_block *nb)
	{ return 0; }
static inline int hikari_unregister_cpufreq_notifier(struct notifier_block *nb)
	{ return 0; }

static inline bool hikari_enabled(void) { return false; }

#endif /* CONFIG_HIKARI */

#endif /* _LINUX_HIKARI_H */
