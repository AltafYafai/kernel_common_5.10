
/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Zenith cpufreq governor - sysfs interface, adaptive tuning, and auto-eval.
 *
 * Split from cpufreq_zenith.c for maintainability.  This file contains the
 * sysfs show/store callbacks, attribute groups, the adaptive-tuning
 * infrastructure (zenith_at_*), and the auto-classifier evaluation work fn.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "cpufreq_zenith_internal.h"

/************************** Sysfs Interface & Tunables ************************/

struct zenith_tunables *global_tunables;
DEFINE_MUTEX(global_tunables_lock);

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
void update_min_rate_limit_ns(struct zenith_policy *z_policy)
{
	s64 up_ns = READ_ONCE(z_policy->up_rate_delay_ns);
	s64 down_ns = READ_ONCE(z_policy->down_rate_delay_ns);

	WRITE_ONCE(z_policy->min_rate_limit_ns, min(up_ns, down_ns));
}

void zenith_update_cluster_rate_scale(struct zenith_policy *z_policy)
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

void zenith_update_rate_delay_ns(struct zenith_policy *z_policy)
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
void zenith_invalidate_cache(struct gov_attr_set *attr_set)
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
void zenith_refresh_rate_delays(struct gov_attr_set *attr_set)
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
void zenith_at_update_prefer_silver_rate(struct zenith_policy *z_policy,
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
void zenith_at_log_push(struct zenith_policy *z_policy,
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
void zenith_at_v3_calibrate(struct zenith_policy *z_policy,
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
	/* Patch J: snapshot of the live offsets before any APPLY-mode
	 * nudge below; recorded in the calibration ring at the tail of
	 * this function so userspace can audit V3 drift.
	 */
	signed char hyst_before, cool_before;
	struct zenith_at_v3_calib_log_entry *log_entry;
	unsigned int log_slot;

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

	hyst_before = z_policy->at_v3_hyst_offset;
	cool_before = z_policy->at_v3_cool_offset;

	/* Apply bounded nudge to offsets only in APPLY mode.  OBSERVE
	 * mode still falls through to the calibration-ring push below
	 * with before == after so userspace can see "V3 ran but did
	 * not nudge because mode is OBSERVE".
	 */
	if (mode == ZENITH_AT_V3_MODE_APPLY) {
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

	/* Patch J: push a calibration record on every tick (both
	 * OBSERVE and APPLY) so userspace can audit V3 drift without
	 * ftrace.  Single-writer ring -- no locking needed against
	 * sysfs readers, who walk the ring under the gov_attr_set
	 * rwsem and accept the same wrap-window tearing the existing
	 * at_log path tolerates.
	 */
	log_slot = z_policy->at_v3_calib_log_head;
	if (log_slot >= ZENITH_AT_V3_CALIB_LOG_NR)
		log_slot = 0;
	log_entry = &z_policy->at_v3_calib_log[log_slot];
	log_entry->ts_ns	= now;
	log_entry->transitions	= transitions;
	log_entry->mode		= (u8)mode;
	log_entry->hyst_before	= hyst_before;
	log_entry->hyst_after	= z_policy->at_v3_hyst_offset;
	log_entry->cool_before	= cool_before;
	log_entry->cool_after	= z_policy->at_v3_cool_offset;
	z_policy->at_v3_calib_log_head =
		(log_slot + 1) % ZENITH_AT_V3_CALIB_LOG_NR;
	if (z_policy->at_v3_calib_log_count < ZENITH_AT_V3_CALIB_LOG_NR)
		z_policy->at_v3_calib_log_count++;
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

	if (!static_branch_likely(&zenith_auto_tune_v3_key))
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

	if (!static_branch_likely(&zenith_auto_tune_v3_key))
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
void zenith_policy_observability_reset(struct zenith_policy *z_policy)
{
	memset(z_policy->stats, 0, sizeof(z_policy->stats));
	memset(z_policy->dec_lat_buckets, 0,
	       sizeof(z_policy->dec_lat_buckets));
	memset(z_policy->at_log, 0, sizeof(z_policy->at_log));
	z_policy->at_log_head = 0;
	z_policy->at_log_count = 0;
	/* Patch J: clear the V3 calibration audit ring on the same
	 * sysfs reset path.  Same rationale as the at_log clear above:
	 * keep all observability surfaces aligned so an operator can
	 * read a clean baseline from any of them on the next eval tick.
	 */
	memset(z_policy->at_v3_calib_log, 0,
	       sizeof(z_policy->at_v3_calib_log));
	z_policy->at_v3_calib_log_head = 0;
	z_policy->at_v3_calib_log_count = 0;
	/* Patch B7-2: clear the per-policy decision ring on the same
	 * sysfs reset path that clears stats / dec_lat_buckets.  Keeps
	 * the three observability surfaces aligned so an operator can
	 * "echo 1 > zenith_stats_reset" and read a clean baseline from
	 * any of them on the next eval tick.
	 */
	memset(z_policy->dec_ring, 0, sizeof(z_policy->dec_ring));
	z_policy->dec_ring_head = 0;
}

void zenith_reset_local_actions(struct zenith_policy *z_policy)
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
void zenith_refresh_rate_delays_one(struct zenith_policy *z_policy)
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
	case ZENITH_PROFILE_GAMING:
		return "gaming";
	case ZENITH_PROFILE_AUDIO:
		return "audio";
	case ZENITH_PROFILE_CUSTOM:
	default:
		return "custom";
	}
}

/* Patch L: human-readable zenith logging helpers.  All three are
 * gated on READ_ONCE(t->verbose_log) so production builds (where
 * verbose_log defaults to 0) pay only the cost of one load + one
 * branch on the cold profile_store / master-store paths.  The
 * pr_info() format strings deliberately match across the three
 * helpers so a userspace log scraper can grep "zenith: " and parse
 * a stable structure.
 *
 * Patch M: emit via pr_info_ratelimited() so a pathological caller
 * (auto_tune flapping the profile, or a userspace tool fanning out
 * master-switch stores) cannot spam dmesg.  Default rate-limit is
 * 10 messages per 5 seconds (DEFAULT_RATELIMIT_BURST /
 * DEFAULT_RATELIMIT_INTERVAL); excess events are coalesced with a
 * "callbacks suppressed" trailer so the trail is still meaningful.
 *
 * profile_change: emitted by profile_store on the user-write path
 *                 *before* the bake runs, so the dmesg trail reads
 *                 "switching X -> Y" / "applied Y profile: ..." in
 *                 chronological order.
 * profile_applied: emitted at the tail of zenith_apply_profile()
 *                  with a one-line summary of the major bake
 *                  results.  Fires for AUTO-driven applies too,
 *                  giving the operator visibility into the auto
 *                  selector's decisions.
 * master_flip:    emitted by each of the seven master-switch stores
 *                 (audio_aware / render_aware / camera_aware /
 *                 psi_aware / game_auto / auto_tune_v3 /
 *                 thermal_aware) after a value change is committed,
 *                 with old / new values both included so the trail
 *                 is meaningful even under fast successive flips.
 */
inline void zenith_log_profile_change(struct zenith_tunables *t,
					     unsigned int old_prof,
					     unsigned int new_prof)
{
	if (!READ_ONCE(t->verbose_log))
		return;
	pr_info_ratelimited("zenith: switching profile %s -> %s\n",
		zenith_profile_name(old_prof),
		zenith_profile_name(new_prof));
}

inline void zenith_log_profile_applied(struct zenith_tunables *t,
					      unsigned int prof)
{
	if (!READ_ONCE(t->verbose_log))
		return;
	pr_info_ratelimited("zenith: applied %s profile: hispeed_freq_pct=%u up_threshold=%u down_threshold=%u climb_mode=%u freq_step_pct=%u powersave_bias=%u up_rate_limit_us=%u down_rate_limit_us=%u wakeup_boost=%u down_threshold_adaptive=%u\n",
		zenith_profile_name(prof),
		t->hispeed_freq_pct, t->up_threshold, t->down_threshold,
		t->climb_mode, t->freq_step_pct, t->powersave_bias,
		t->up_rate_limit_us, t->down_rate_limit_us,
		t->wakeup_boost, t->down_threshold_adaptive);
}

inline void zenith_log_master_flip(struct zenith_tunables *t,
					  const char *name,
					  unsigned int old_val,
					  unsigned int new_val)
{
	if (!READ_ONCE(t->verbose_log))
		return;
	pr_info_ratelimited("zenith: master %s %u -> %u\n",
		name, old_val, new_val);
}

const char *zenith_at_state_name(unsigned int state)
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

const char *zenith_at_reason_name(unsigned int reason)
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

unsigned int zenith_at_clamp(unsigned int val, unsigned int lo,
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

void zenith_at_get_guardrails(unsigned int profile,
				     struct zenith_at_guardrails *g)
{
	switch (profile) {
	case ZENITH_PROFILE_PERFORMANCE:
	case ZENITH_PROFILE_GAMING:
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
	case ZENITH_PROFILE_AUDIO:
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

void zenith_at_get_policy_guardrails(struct zenith_policy *z_policy,
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

unsigned int zenith_at_state_to_profile(unsigned int state)
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

unsigned int zenith_profile_to_at_state(unsigned int profile)
{
	switch (profile) {
	case ZENITH_PROFILE_PERFORMANCE:
	case ZENITH_PROFILE_GAMING:
		return ZENITH_AT_STATE_LATENCY;
	case ZENITH_PROFILE_BATTERY:
		return ZENITH_AT_STATE_EFFICIENCY;
	case ZENITH_PROFILE_BALANCED:
	case ZENITH_PROFILE_LEGACY:
	case ZENITH_PROFILE_AUDIO:
	case ZENITH_PROFILE_CUSTOM:
	default:
		return ZENITH_AT_STATE_BALANCED;
	}
}

const char *zenith_at_cluster_name(unsigned int cluster)
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

unsigned int zenith_at_profile_for_state(struct zenith_policy *z_policy,
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

void zenith_at_mark_override(struct zenith_tunables *t,
				    unsigned long bit)
{
	t->auto_tune_override_mask |= bit;
}

bool zenith_at_set_uint(struct zenith_tunables *t, unsigned long bit,
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

unsigned int zenith_tunable_or_local(struct zenith_policy *z_policy,
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
unsigned int zenith_glide_value(struct zenith_policy *z_policy,
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

void zenith_at_write_effective(struct zenith_policy *z_policy,
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

bool zenith_at_apply_actions(struct zenith_policy *z_policy,
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
void zenith_at_apply_glides(struct zenith_policy *z_policy,
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
void zenith_at_apply_tiers(struct zenith_policy *z_policy,
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
	return sysfs_emit(buf, "%u\n", t->_name); \
} \
static ssize_t _name##_store(struct gov_attr_set *attr_set, const char *buf, size_t count) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	unsigned int val; \
\
	if (kstrtouint(buf, 10, &val)) \
		return -EINVAL; \
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
	return sysfs_emit(buf, "%u\n", t->_name); \
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
	return sysfs_emit(buf, "%u\n", t->_name); \
} \
static ssize_t _name##_store(struct gov_attr_set *attr_set, const char *buf, size_t count) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	unsigned int val; \
\
	if (kstrtouint(buf, 10, &val)) \
		return -EINVAL; \
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
	return sysfs_emit(buf, "%u\n", t->_name); \
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

/* Combined ZENITH_TUNABLE_UINT_MAX + ZENITH_TUNABLE_UINT_INVAL: clamp
 * the stored value at _max and invalidate the per-policy freq cache
 * so the new value takes effect on the next scheduler tick.  Use for
 * fields that have a non-trivial upper bound *and* feed into the
 * zenith_get_next_freq() decision (e.g. hispeed_freq, light_load_freq,
 * climb_mode, powersave_bias).
 */
#define ZENITH_TUNABLE_UINT_MAX_INVAL(_name, _max) \
static ssize_t _name##_show(struct gov_attr_set *attr_set, char *buf) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	return sysfs_emit(buf, "%u\n", t->_name); \
} \
static ssize_t _name##_store(struct gov_attr_set *attr_set, const char *buf, size_t count) \
{ \
	struct zenith_tunables *t = to_zenith_tunables(attr_set); \
	unsigned int val; \
	if (kstrtouint(buf, 10, &val) || val > (_max)) \
		return -EINVAL; \
	t->_name = val; \
	zenith_invalidate_cache(attr_set); \
	return count; \
} \
static struct governor_attr _name = __ATTR_RW(_name)

ZENITH_TUNABLE_UINT_BOOL_INVAL(io_is_busy);

static ssize_t iowait_boost_min_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
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

ZENITH_TUNABLE_UINT_MAX(iowait_stack_pct, 100);

static ssize_t iowait_backoff_after_ms_show(struct gov_attr_set *attr_set,
					    char *buf)
{
	return sysfs_emit(buf, "%u\n",
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

ZENITH_TUNABLE_UINT_BOOL_INVAL(ignore_nice_load);

/* Apply one of the preset recipes to all tunables in-place. Leaves
 * light_load_freq, hispeed_freq, efficient_freq ladder and other
 * device-specific frequencies untouched because their correct values
 * depend on the SoC's actual freq table. The user can layer those on
 * top after picking a profile.
 *
 * Profile design intent (re-derived from the four preset tables
 * below; all values that don't appear in the per-profile table fall
 * back to the module-level ZENITH_DEFAULT_* values).  This block is
 * the one place that documents each profile's stance per tier;
 * individual tunable comments and tracepoints describe the mechanic,
 * but the per-profile *values* belong here so a reader picking a
 * profile can see the whole budget at once.
 *
 *   PERFORMANCE  Aggressive everywhere.  up_threshold=65, hispeed
 *                tier engages at load 55, climb mode SNAP, freq_step
 *                15 %% per tick, no powersave bias, input boost
 *                150 ms with full decay tail, peak-headroom rescue
 *                AND prearm both on, render-floor and frame-overrun
 *                tiers tuned tighter, peer-ramp window 80 ms with
 *                screen-off variant disabled, predictive up tier
 *                tighter, PSI floors armed but with hispeed-headroom
 *                guards.  Tradeoff: best burst latency, highest
 *                steady-state power.  Preferred for scenario flips
 *                triggered by camera_aware / render_aware / sustained
 *                game-mode.
 *
 *   BALANCED     The default.  up_threshold ~75, hispeed tier at
 *                load 65, climb mode SNAP, freq_step 10 %%, mild
 *                powersave bias, input boost 100 ms with shorter
 *                decay, peak-headroom rescue on but prearm gated by
 *                hysteresis streak, peer-ramp window 50 ms with a
 *                short screen-off variant, frame-overrun deep tier
 *                gated by streak.  Tradeoff: best 95th-pct latency
 *                while keeping idle power close to BATTERY.
 *                Preferred for the audio-detect scenario flip.
 *
 *   BATTERY      Conservative everywhere it matters for residency.
 *                up_threshold raised, climb mode STEP not SNAP,
 *                freq_step ~5-7 %%, larger powersave bias, input
 *                boost shorter, peak-headroom rescue off (only
 *                prearm fires), render-floor gated by min runtime,
 *                peer-ramp window ~30 ms (just long enough to catch
 *                a real cluster wake without paying for noise),
 *                frame-overrun off, PSI memstall cap armed
 *                aggressively to throttle when memstall is the
 *                bottleneck.  Tradeoff: lowest screen-off and idle
 *                power, deeper drops on bursty foreground work.
 *                Preferred for memstall-detected scenario flip
 *                (audio + memstall + screen-off all push toward
 *                BATTERY).
 *
 *   LEGACY       Bug-for-bug compatible with pre-V4 zenith.  All
 *                post-V4 tiers (peak-headroom prearm, peer-ramp,
 *                migration-floor, frame-overrun, PSI floors,
 *                render-floor, predict-up) gated to 0 so the
 *                runtime path matches the original schedutil-plus-
 *                ondemand-plus-reflex hybrid.  Useful as a
 *                regression baseline and for users who have an
 *                existing tuning workflow that depends on the V4
 *                tiers being silent.
 *
 *   CUSTOM       (not in this table)  No profile applied.  Every
 *                tunable carries whatever sysfs left it at, which
 *                in practice is the ZENITH_DEFAULT_* values from
 *                the top of this file unless the user overwrote
 *                them.  This is the value reported by the profile
 *                sysfs node when no preset has ever been applied.
 *
 * Adding a new tunable that should track the profile: add a field
 * to struct zenith_profile_defaults below, fill in the value in all
 * four preset tables, and copy it from p->foo to t->foo at the end
 * of this function.  Forgetting any of those three steps will leave
 * the field at its module default for that profile, which is
 * almost always wrong.
 *
 * Profile-baked vs. global-only tunable split (audited 2026-05-07,
 * zenith-tunables-audit).  struct zenith_tunables exposes ~180
 * tunables; this function bakes the subset whose semantics differ
 * by user-picked profile.  Everything else is "global-only": set
 * once at zenith_create_tunables_data() time from the
 * ZENITH_DEFAULT_* constants and only mutated by direct sysfs
 * writes (no profile path touches them).
 *
 * BAKED (81 fields, in the same order as struct
 * zenith_profile_defaults below).  Adding a field here without
 * updating all four preset tables triggers the warning above; a
 * static assert is not feasible because the struct is anonymous
 * inside this function.
 *
 *   V1 base (legacy schedutil-plus tier, 24 fields):
 *     up_rate_limit_us, down_rate_limit_us, up_threshold,
 *     down_threshold, hispeed_freq_pct, hispeed_load, climb_mode,
 *     freq_step_pct, powersave_bias, bias_load_threshold,
 *     ignore_nice_load, input_boost_ms, input_boost_decay_ms,
 *     input_boost_cap_pct, light_load_threshold,
 *     sampling_down_factor, thermal_auto, screen_auto,
 *     util_math_v2, kcpustat_hispeed_enable, down_rate_adaptive,
 *     wakeup_boost, down_threshold_adaptive,
 *     rate_limit_cluster_scale.
 *
 *   Stage 1/2 (peak-headroom + scenario shaping, 17 fields):
 *     peak_headroom_rescue, peak_headroom_prearm,
 *     peak_headroom_starve_load_pct,
 *     peak_headroom_freq_floor_pct,
 *     peak_headroom_starve_streak, peak_headroom_jump_pct,
 *     peak_headroom_hold_ms, batt_hold_scale_pct,
 *     cluster_wake_pulse_ms, cluster_wake_pulse_idle_ms,
 *     cluster_wake_pulse_floor_pct, quiet_hours_cap_pct,
 *     quiet_hours_screen_off_only, fg_transition_pulse_ms,
 *     fg_transition_pulse_pct, screen_on_bias_pct,
 *     input_boost_down_rate_mult_pct.
 *
 *   Stage 4 / Stage 5 (predict + rising-edge + DL + IO + render
 *   + peak-hyst + boost-idle + bg-util + sleeper + peer-ramp +
 *   migration-floor + PSI-CPU floor + frame-overrun + PSI-mem
 *   cap, 32 fields):
 *     predict_up_thresh, predict_up_window, pelt_rising_edge_thresh,
 *     pelt_rising_edge_min_pct, dl_task_floor_pct,
 *     io_floor_hyst_ms, io_floor_hyst_pct, render_floor_pct,
 *     render_floor_min_runtime_ms, input_boost_touchdown_extra_ms,
 *     peak_hysteresis_streak, peak_step_down_pct,
 *     boost_idle_thresh, boost_idle_streak, bg_util_scale_pct,
 *     sleeper_tail_thresh_us, sleeper_tail_pct,
 *     peer_ramp_window_ms, peer_ramp_floor_pct,
 *     peer_ramp_window_off_ms, migration_jump_pct,
 *     migration_floor_window_ms, migration_floor_pct,
 *     psi_cpu_floor_thresh, frame_overrun_slack_us,
 *     frame_overrun_window_ms, frame_overrun_floor_pct,
 *     frame_overrun_deep_streak, frame_overrun_deep_floor_pct,
 *     psi_mem_cap_thresh, psi_mem_cap_pct, psi_mem_cap_window_ms.
 *
 *   Audio + vendor-hooks (7 fields):
 *     audio_hyst_ms, vh_arch_freq_scale_enable,
 *     vh_uclamp_observer_enable, vh_cpu_idle_enable,
 *     vh_freq_qos_enable, vh_sched_move_task_enable,
 *     vh_scheduler_tick_enable.
 *
 *   Patch B10-3 (1 field):
 *     psi_cgroup_path.
 *
 * GLOBAL-ONLY (~100 fields).  Not enumerated individually; the
 * categories are:
 *
 *   - User intent / opt-in flags whose semantics are device-wide
 *     and not per-profile: audio_aware, render_aware, camera_aware,
 *     psi_aware, prefer_silver_aware, game_auto, auto_tune_v2,
 *     auto_tune_v3 (all of the seven static-key-gated aware-flags
 *     and tier switches plus auto_tune_cluster_aware,
 *     auto_tune_v2_signals, auto_tune_frame_pacing,
 *     auto_tune_sustained_gaming).
 *
 *   - Mode-style scalars whose semantics are device-wide:
 *     thermal_state, thermal_active, thermal_aware, screen_state,
 *     freq_step_adaptive, climb_mode_brutal_*, profile (the
 *     reported profile node itself), profile_auto.
 *
 *   - Hardware-shaped freqs that depend on the SoC's freq table,
 *     not the user's intent: hispeed_freq, light_load_freq,
 *     efficient_freq[N], eff_bin_*, up_delay_us.
 *
 *   - Auto-tune V2/V3 internals that the auto-tune state machine
 *     manages directly (auto_tune_eval_ms,
 *     auto_tune_hysteresis_ms, auto_tune_hysteresis_windows,
 *     auto_tune_cooldown_windows, auto_tune_v2_var_promote_thresh,
 *     auto_tune_util_rising_thresh_pct,
 *     auto_tune_render_rt_floor_pct, auto_tune_v3_interval_ms,
 *     auto_tune_v3_state, auto_tune_thermal_slope,
 *     auto_tune_thermal_pressure_pct,
 *     auto_tune_thermal_slope_pct, auto_tune_sat_load_pct,
 *     auto_tune_hi_sat_pct, auto_tune_lo_sat_pct,
 *     auto_tune_hi_events_x2, auto_tune_lo_events_x2,
 *     auto_tune_scenario, all of the at_log_* sysfs RO mirrors).
 *
 *   - PSI thresholds and adaptive-up internals that already key off
 *     other profile-baked tiers: psi_mem_thresh, psi_cpu_thresh,
 *     psi_io_thresh, up_threshold_adaptive (the adaptive value, as
 *     opposed to the on/off knob which is profile-baked).
 *
 *   - Boot-time and frame-budget knobs that are global by design:
 *     boot_boost_ms, boot_boost_decay_ms, boot_complete_auto,
 *     frame_budget_us, frame_budget_us_auto, frame_pace_floor_pct,
 *     fg_*_pct ratios that are not in the bake set above.
 *
 *   - Boost / wakeup secondary knobs whose primary on/off lives in
 *     the bake set: input_boost_decay_curve, input_boost_big_only,
 *     wakeup_boost_ms, brutal_decay_ms, hispeed_*_streak,
 *     hispeed_hyst_pct, hispeed_entry_streak, freq_stability_-
 *     margin_pct, sampling_down_factor's helpers (already-baked
 *     factor itself is the bake-set entry), boost_exit_extend.
 *
 *   - Stats / observability: zenith_stats_*, *_active mirrors
 *     written from the runtime path, prefer_silver_*_pct, and the
 *     prefer_silver_hot_threshold_pct / prefer_silver_hot_bump_pct
 *     pair (the on/off prefer_silver_aware lives in the aware-flag
 *     opt-in group above).
 *
 * Rule of thumb for new tunables: if the right value depends on
 * what the user picked in /sys/.../zenith/profile, add it to the
 * bake set below.  If the right value depends on the device's
 * hardware (freq table, cluster topology) or on the user's intent
 * to enable an opt-in detection path (an aware-flag), keep it
 * global-only.  Default values for both groups belong in the
 * ZENITH_DEFAULT_* block at the top of this file; the four preset
 * tables only need to override values that differ from the
 * BALANCED row, but the current convention is to fill every cell
 * for readability.
 */
void zenith_apply_profile(struct zenith_tunables *t, unsigned int prof)
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
		unsigned int batt_hold_scale_pct;
		unsigned int cluster_wake_pulse_ms;
		unsigned int cluster_wake_pulse_idle_ms;
		unsigned int cluster_wake_pulse_floor_pct;
		unsigned int quiet_hours_cap_pct;
		unsigned int quiet_hours_screen_off_only;
		unsigned int fg_transition_pulse_ms;
		unsigned int fg_transition_pulse_pct;
		unsigned int screen_on_bias_pct;
		unsigned int input_boost_down_rate_mult_pct;
		unsigned int predict_up_thresh;
		unsigned int predict_up_window;
		unsigned int pelt_rising_edge_thresh;
		unsigned int pelt_rising_edge_min_pct;
		unsigned int dl_task_floor_pct;
		unsigned int io_floor_hyst_ms;
		unsigned int io_floor_hyst_pct;
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
		unsigned int audio_hyst_ms;
		unsigned int vh_arch_freq_scale_enable;
		unsigned int vh_uclamp_observer_enable;
		unsigned int vh_cpu_idle_enable;
		unsigned int vh_freq_qos_enable;
		unsigned int vh_sched_move_task_enable;
		unsigned int vh_scheduler_tick_enable;
		/* Patch B10-3: per-profile cgroup-v2 path the
		 * zenith_psi_*_some_pct() helpers should read from.
		 * NULL or "" means "use system-wide PSI" (the safe
		 * default and the pre-B10 behaviour); a non-empty
		 * cgroup-v2 path is resolved + cached at apply time
		 * via zenith_psi_cgroup_apply().  Profile-bake here
		 * is overridable per policy via the psi_cgroup_path
		 * sysfs node.
		 */
		const char *psi_cgroup_path;
	};
	const struct zenith_profile_defaults profiles[] = {
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
			.batt_hold_scale_pct = 100,
			.cluster_wake_pulse_ms = 80,
			.cluster_wake_pulse_idle_ms = 60,
			.cluster_wake_pulse_floor_pct = 70,
			.quiet_hours_cap_pct = 100,
			.quiet_hours_screen_off_only = 1,
			.fg_transition_pulse_ms = 50,
			.fg_transition_pulse_pct = 75,
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
			/* Patch C3: PERFORMANCE wants the rising-edge
			 * tier to fire on smaller per-sample slopes
			 * (24 vs 32 default) and from a lower absolute
			 * level (40%% of max_cap vs 50%% default), so
			 * a fresh foreground task barely starts the
			 * rise and we already have eff_hispeed under
			 * it.  Effort budget: bigger battery hit if a
			 * jitter sample fires the lift, but PERF is
			 * the profile that pays that cost willingly.
			 */
			.pelt_rising_edge_thresh = 24,
			.pelt_rising_edge_min_pct = 40,
			/* Patch C6: PERFORMANCE pins to policy->max
			 * the moment any DL task is detected.  Frame-
			 * work / kernel timers using SCHED_DEADLINE
			 * (e.g. PipeWire RT, V4L2 streamer threads on
			 * the BIG cluster) get a hard freq floor so
			 * the first wake doesn't miss its deadline.
			 */
			.dl_task_floor_pct = 100,
			/* Patch C9: PERFORMANCE holds a 70%% floor for
			 * 500 ms past the last iowait sample.  Block
			 * IO bursts on PERF (e.g. apt-update, large
			 * file copies) carry the freq instead of
			 * collapsing into the level signal between
			 * batches.
			 */
			.io_floor_hyst_ms = 500,
			.io_floor_hyst_pct = 70,
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
			.audio_hyst_ms = 250,
			.vh_arch_freq_scale_enable = 1,
			.vh_uclamp_observer_enable = 1,
			.vh_cpu_idle_enable = 1,
			.vh_freq_qos_enable = 1,
			.vh_sched_move_task_enable = 0,
			.vh_scheduler_tick_enable = 0,
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
			.batt_hold_scale_pct = 120,
			.cluster_wake_pulse_ms =
				ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_MS,
			.cluster_wake_pulse_idle_ms =
				ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_IDLE_MS,
			.cluster_wake_pulse_floor_pct =
				ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_FLOOR_PCT,
			.quiet_hours_cap_pct = 70,
			.quiet_hours_screen_off_only = 1,
			.fg_transition_pulse_ms =
				ZENITH_DEFAULT_FG_TRANSITION_PULSE_MS,
			.fg_transition_pulse_pct =
				ZENITH_DEFAULT_FG_TRANSITION_PULSE_PCT,
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
			/* Patch C3: BALANCED matches cold-boot defaults
			 * (32 thresh, 50%% level gate).
			 */
			.pelt_rising_edge_thresh =
				ZENITH_DEFAULT_PELT_RISING_EDGE_THRESH,
			.pelt_rising_edge_min_pct =
				ZENITH_DEFAULT_PELT_RISING_EDGE_MIN_PCT,
			/* Patch C6: BALANCED matches cold-boot default
			 * (off).  The DL bandwidth math via
			 * schedutil_cpu_util() handles average-throughput
			 * for DL tasks; the floor is an opt-in
			 * responsiveness boost reserved for the
			 * profiles that explicitly want it.
			 */
			.dl_task_floor_pct =
				ZENITH_DEFAULT_DL_TASK_FLOOR_PCT,
			/* Patch C9: BALANCED holds a 50%% floor for
			 * 200 ms.  Default-magnitude responsiveness
			 * for moderate IO without paying the energy
			 * frame of PERFORMANCE.
			 */
			.io_floor_hyst_ms = 200,
			.io_floor_hyst_pct = 50,
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
			.audio_hyst_ms = ZENITH_DEFAULT_AUDIO_HYST_MS,
			.vh_arch_freq_scale_enable =
				ZENITH_DEFAULT_VH_ARCH_FREQ_SCALE_ENABLE,
			.vh_uclamp_observer_enable =
				ZENITH_DEFAULT_VH_UCLAMP_OBSERVER_ENABLE,
			.vh_cpu_idle_enable =
				ZENITH_DEFAULT_VH_CPU_IDLE_ENABLE,
			/* See ZENITH_DEFAULT_VH_FREQ_QOS_ENABLE for the
			 * BALANCED-vs-cold-boot rationale: the cold-boot
			 * default is 0 (opt-in), but the BALANCED bake is
			 * explicitly 1 because BALANCED is the AUTO
			 * engine's default resting state, and the AUTO
			 * pivot to PERFORMANCE on QoS pressure is what
			 * makes this observer useful in the first place.
			 */
			.vh_freq_qos_enable = 1,
			.vh_sched_move_task_enable = 0,
			.vh_scheduler_tick_enable = 0,
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
			.batt_hold_scale_pct = 180,
			.cluster_wake_pulse_ms = 0,
			.cluster_wake_pulse_idle_ms = 0,
			.cluster_wake_pulse_floor_pct = 0,
			.quiet_hours_cap_pct = 55,
			.quiet_hours_screen_off_only = 1,
			.fg_transition_pulse_ms = 0,
			.fg_transition_pulse_pct = 0,
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
			/* Patch C3: BATTERY also disables the rising-
			 * edge tier; same energy-frame argument as
			 * predict_up.  The hispeed level tier alone is
			 * the right speed/energy trade for this profile.
			 */
			.pelt_rising_edge_thresh = 0,
			.pelt_rising_edge_min_pct =
				ZENITH_DEFAULT_PELT_RISING_EDGE_MIN_PCT,
			/* Patch C6: BATTERY keeps the floor off; DL
			 * bandwidth math is sufficient for the energy
			 * frame this profile targets.
			 */
			.dl_task_floor_pct = 0,
			/* Patch C9: BATTERY disables IO floor
			 * hysteresis — energy frame.
			 */
			.io_floor_hyst_ms = 0,
			.io_floor_hyst_pct = 0,
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
			.audio_hyst_ms = 100,
			.vh_arch_freq_scale_enable = 0,
			.vh_uclamp_observer_enable = 0,
			.vh_cpu_idle_enable = 1,
			.vh_freq_qos_enable = 0,
			.vh_sched_move_task_enable = 0,
			.vh_scheduler_tick_enable = 0,
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
			.batt_hold_scale_pct = 100,
			.cluster_wake_pulse_ms = 0,
			.cluster_wake_pulse_idle_ms = 0,
			.cluster_wake_pulse_floor_pct = 0,
			.quiet_hours_cap_pct = 100,
			.quiet_hours_screen_off_only = 1,
			.fg_transition_pulse_ms = 0,
			.fg_transition_pulse_pct = 0,
			.screen_on_bias_pct = 100,
			.input_boost_down_rate_mult_pct = 100,
			/* Stage 4 / Patch A: LEGACY disables prediction
			 * (no Stage 4 features in the historical-
			 * compatibility profile).
			 */
			.predict_up_thresh = 0,
			.predict_up_window =
				ZENITH_DEFAULT_PREDICT_UP_WINDOW,
			/* Patch C3: LEGACY disables the rising-edge
			 * tier as well -- the historical-compat profile
			 * keeps every Patch-C3-and-later tier dormant.
			 */
			.pelt_rising_edge_thresh = 0,
			.pelt_rising_edge_min_pct =
				ZENITH_DEFAULT_PELT_RISING_EDGE_MIN_PCT,
			/* Patch C6: LEGACY also disables the DL floor
			 * (historical-compat profile).
			 */
			.dl_task_floor_pct = 0,
			/* Patch C9: LEGACY off (historical-compat). */
			.io_floor_hyst_ms = 0,
			.io_floor_hyst_pct = 0,
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
			.audio_hyst_ms = 0,
			.vh_arch_freq_scale_enable = 0,
			.vh_uclamp_observer_enable = 0,
			.vh_cpu_idle_enable = 1,
			.vh_freq_qos_enable = 0,
			.vh_sched_move_task_enable = 0,
			.vh_scheduler_tick_enable = 0,
		},
		{
			/* Patch 4.1: GAMING profile.
			 *
			 * Purpose: maximum frame stability under
			 * sustained foreground game load.  Branches
			 * from PERFORMANCE with stronger frame-overrun
			 * tiering, deeper cluster-wake / fg pulses,
			 * full screen-on background util, and AC-vs-
			 * battery hold scaling pinned at identity (the
			 * profile is for plugged-in / docked play; the
			 * user explicitly opted into the energy cost).
			 *
			 * Auto-tune state: LATENCY (mapped via
			 * zenith_profile_to_at_state).  Guardrails:
			 * shared with PERFORMANCE (mapped via
			 * zenith_at_get_guardrails).
			 */
			.profile = ZENITH_PROFILE_GAMING,
			.up_rate_limit_us = 0,
			.down_rate_limit_us = 8000,
			.up_threshold = 60,
			.down_threshold = 45,
			.hispeed_freq_pct = 65,
			.hispeed_load = 50,
			.climb_mode = ZENITH_CLIMB_MODE_SNAP,
			.freq_step_pct = 18,
			.powersave_bias = 0,
			.bias_load_threshold = 50,
			.ignore_nice_load = 0,
			.input_boost_ms = 180,
			.input_boost_decay_ms = 60,
			.input_boost_cap_pct = 0,
			.light_load_threshold = 12,
			.sampling_down_factor = 4,
			.thermal_auto = 1,
			.screen_auto = 1,
			.util_math_v2 = 1,
			.kcpustat_hispeed_enable = 1,
			.down_rate_adaptive = 1,
			.wakeup_boost = 1,
			.down_threshold_adaptive = 12,
			.rate_limit_cluster_scale = 1,
			/* Aggressive peak-headroom: lower starve_load_-
			 * pct (85 vs PERF 88), shorter streak (1 vs 2),
			 * higher floor (90 vs PERF 80), longer hold
			 * (35 ms vs PERF 25).  A single below-floor
			 * sample inside a sustained game load is a
			 * frame at risk; recover before EAS sees it.
			 */
			.peak_headroom_rescue = 1,
			.peak_headroom_prearm = 1,
			.peak_headroom_starve_load_pct = 85,
			.peak_headroom_freq_floor_pct = 90,
			.peak_headroom_starve_streak = 1,
			.peak_headroom_jump_pct = 100,
			.peak_headroom_hold_ms = 35,
			.batt_hold_scale_pct = 100,
			.cluster_wake_pulse_ms = 100,
			.cluster_wake_pulse_idle_ms = 50,
			.cluster_wake_pulse_floor_pct = 80,
			.quiet_hours_cap_pct = 100,
			.quiet_hours_screen_off_only = 1,
			.fg_transition_pulse_ms = 60,
			.fg_transition_pulse_pct = 80,
			.screen_on_bias_pct = 0,
			.input_boost_down_rate_mult_pct = 350,
			.predict_up_thresh = 40,
			.predict_up_window = 4,
			/* Patch C3: GAMING is the most aggressive
			 * profile for the rising-edge tier (20 / 35).
			 * Slope as small as ~8%% per sample fires the
			 * lift, and the level gate drops to 35%% of
			 * max_cap so a freshly-foregrounded game frame
			 * snaps to eff_hispeed before the level-trig
			 * tier even sees it.
			 */
			.pelt_rising_edge_thresh = 20,
			.pelt_rising_edge_min_pct = 35,
			/* Patch C6: GAMING pins to policy->max on DL
			 * task presence -- some game engines pin a
			 * compositor-pacer thread to SCHED_DEADLINE
			 * to lock vblank cadence; the floor guarantees
			 * its first wake hits target frequency.
			 */
			.dl_task_floor_pct = 100,
			/* Patch C9: GAMING 300 ms / 60%% — game asset
			 * loading and save/restore bursts keep the
			 * freq elevated through batch gaps.
			 */
			.io_floor_hyst_ms = 300,
			.io_floor_hyst_pct = 60,
			.render_floor_pct = 85,
			.render_floor_min_runtime_ms = 15,
			.input_boost_touchdown_extra_ms = 100,
			.peak_hysteresis_streak = 5,
			.peak_step_down_pct = 98,
			.boost_idle_thresh = 0,
			.boost_idle_streak = 0,
			.bg_util_scale_pct = 100,
			.sleeper_tail_thresh_us = 0,
			.sleeper_tail_pct = 100,
			.peer_ramp_window_ms = 50,
			.peer_ramp_floor_pct = 75,
			/* Keep peer_ramp warm even with screen blanked.
			 * Game-mode often suspends rendering briefly
			 * (loading screens / menu transitions) where
			 * the IPC chain is still active; suppressing
			 * peer_ramp the moment the panel reports
			 * screen-off would defeat the profile.
			 */
			.peer_ramp_window_off_ms = 30,
			.migration_jump_pct = 12,
			.migration_floor_window_ms = 40,
			.migration_floor_pct = 75,
			.psi_cpu_floor_thresh = 35,
			/* Frame-overrun tiering tightened across the
			 * board.  Slack 2500 us = ~15%% of a 60 Hz
			 * frame; window 60 ms; floor 95%%.  Deep tier
			 * arms on a single consecutive overrun (vs
			 * PERF's 2) at full 100%%.
			 */
			.frame_overrun_slack_us = 2500,
			.frame_overrun_window_ms = 60,
			.frame_overrun_floor_pct = 95,
			.frame_overrun_deep_streak = 1,
			.frame_overrun_deep_floor_pct = 100,
			/* PSI-mem cap stays off (same reasoning as
			 * PERFORMANCE).  Forward-compatible defaults
			 * for the pct/window if a sysfs override
			 * arms thresh.
			 */
			.psi_mem_cap_thresh = 0,
			.psi_mem_cap_pct = 90,
			.psi_mem_cap_window_ms = 1000,
			.audio_hyst_ms = 250,
			.vh_arch_freq_scale_enable = 1,
			.vh_uclamp_observer_enable = 1,
			.vh_cpu_idle_enable = 1,
			/* Patch B9-3+: GAMING already provides aggressive
			 * headroom; QoS-pressure pivot to PERFORMANCE is
			 * redundant.  Bake off.
			 */
			.vh_freq_qos_enable = 0,
			.vh_sched_move_task_enable = 0,
			.vh_scheduler_tick_enable = 0,
		},
		{
			/* Patch 4.2: AUDIO profile.
			 *
			 * Purpose: pinhole-tight wake-cadence guarantees
			 * for ALSA / aaudio callback threads (typical
			 * pcm period ~ 5..20 ms).  Branches from
			 * BALANCED with the screen-off / sleeper-tail /
			 * peer-ramp-off knobs all softened so the
			 * cluster doesn't drift into states that cost
			 * an audio underrun on the first wake of every
			 * period.  Frame-overrun stays at BALANCED
			 * (audio doesn't care about vblank).  Render
			 * floor disabled (audio threads aren't
			 * RENDER_PRIO).  PSI-mem cap stays off.
			 *
			 * Auto-tune state: BALANCED (mapped via
			 * zenith_profile_to_at_state).  Guardrails:
			 * shared with BALANCED / CUSTOM.
			 */
			.profile = ZENITH_PROFILE_AUDIO,
			.up_rate_limit_us = 250,
			.down_rate_limit_us = 8000,
			.up_threshold = 70,
			.down_threshold = 50,
			.hispeed_freq_pct = ZENITH_DEFAULT_HISPEED_FREQ_PCT,
			.hispeed_load = ZENITH_DEFAULT_HISPEED_LOAD,
			.climb_mode = ZENITH_CLIMB_MODE_SNAP,
			.freq_step_pct = ZENITH_DEFAULT_FREQ_STEP_PCT,
			.powersave_bias = 30,
			.bias_load_threshold = 45,
			.ignore_nice_load = 1,
			.input_boost_ms = 50,
			.input_boost_decay_ms = 30,
			.input_boost_cap_pct = 70,
			.light_load_threshold = 18,
			.sampling_down_factor = 4,
			.thermal_auto = 1,
			.screen_auto = 1,
			.util_math_v2 = 1,
			.kcpustat_hispeed_enable = 1,
			.down_rate_adaptive = 1,
			.wakeup_boost = 1,
			.down_threshold_adaptive = 5,
			.rate_limit_cluster_scale = 1,
			/* Mild peak-headroom: lift the floor slightly
			 * over BALANCED so the audio worker cluster
			 * recovers fast on a brief peak, but keep
			 * starve_load_pct higher so we don't fire on
			 * routine cadence.
			 */
			.peak_headroom_rescue =
				ZENITH_DEFAULT_PEAK_HEADROOM_RESCUE,
			.peak_headroom_prearm =
				ZENITH_DEFAULT_PEAK_HEADROOM_PREARM,
			.peak_headroom_starve_load_pct = 90,
			.peak_headroom_freq_floor_pct = 75,
			.peak_headroom_starve_streak = 3,
			.peak_headroom_jump_pct = 100,
			.peak_headroom_hold_ms = 30,
			.batt_hold_scale_pct = 110,
			.cluster_wake_pulse_ms =
				ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_MS,
			.cluster_wake_pulse_idle_ms =
				ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_IDLE_MS,
			.cluster_wake_pulse_floor_pct = 65,
			.quiet_hours_cap_pct = 85,
			.quiet_hours_screen_off_only = 1,
			.fg_transition_pulse_ms =
				ZENITH_DEFAULT_FG_TRANSITION_PULSE_MS,
			.fg_transition_pulse_pct =
				ZENITH_DEFAULT_FG_TRANSITION_PULSE_PCT,
			.screen_on_bias_pct =
				ZENITH_DEFAULT_SCREEN_ON_BIAS_PCT,
			.input_boost_down_rate_mult_pct =
				ZENITH_DEFAULT_INPUT_BOOST_DOWN_RATE_MULT_PCT,
			.predict_up_thresh =
				ZENITH_DEFAULT_PREDICT_UP_THRESH,
			.predict_up_window =
				ZENITH_DEFAULT_PREDICT_UP_WINDOW,
			/* Patch C3: AUDIO matches BALANCED defaults.
			 * Audio worker bursts are predictable enough
			 * that the rolling-window predict_up handles
			 * them without the rising-edge tier needing to
			 * over-react.  Keeping the knob default-armed
			 * means a transient game/UI burst alongside
			 * audio playback is still caught.
			 */
			.pelt_rising_edge_thresh =
				ZENITH_DEFAULT_PELT_RISING_EDGE_THRESH,
			.pelt_rising_edge_min_pct =
				ZENITH_DEFAULT_PELT_RISING_EDGE_MIN_PCT,
			/* Patch C6: AUDIO uses an 80%% DL floor.  ALSA
			 * RT/DL paths and JACK-style audio servers
			 * pin a per-period worker on SCHED_DEADLINE
			 * with sub-ms periods; the 80%% floor gives
			 * those threads guaranteed headroom without
			 * pinning the cluster to absolute max for
			 * everything else on the device.
			 */
			.dl_task_floor_pct = 80,
			/* Patch C9: AUDIO 500 ms / 60%% — sustained
			 * DAC ring-buffer writes are iowait-heavy but
			 * util-light (D-state dominant); the floor
			 * prevents freq drops between DMA periods.
			 */
			.io_floor_hyst_ms = 500,
			.io_floor_hyst_pct = 60,
			/* Render floor off: audio worker is not the
			 * RENDER_PRIO thread that renderer-floor is
			 * scoped to.  Keep the floor knob populated
			 * for forward-compat in case sysfs flips it
			 * during an audio + game session, but cold-
			 * start the AUDIO profile with it off.
			 */
			.render_floor_pct = 0,
			.render_floor_min_runtime_ms =
				ZENITH_DEFAULT_RENDER_FLOOR_MIN_RUNTIME_MS,
			.input_boost_touchdown_extra_ms = 30,
			.peak_hysteresis_streak =
				ZENITH_DEFAULT_PEAK_HYSTERESIS_STREAK,
			.peak_step_down_pct =
				ZENITH_DEFAULT_PEAK_STEP_DOWN_PCT,
			.boost_idle_thresh =
				ZENITH_DEFAULT_BOOST_IDLE_THRESH,
			.boost_idle_streak =
				ZENITH_DEFAULT_BOOST_IDLE_STREAK,
			/* Background-util scale stays high under AUDIO.
			 * The ALSA / aaudio worker keeps running with
			 * the screen off; trimming it to the BALANCED
			 * 75%% bg_util_scale risks dropping the
			 * cluster into a slower tier on the very
			 * cadence the worker depends on.
			 */
			.bg_util_scale_pct = 90,
			/* Sleeper-tail shaving disabled.  Audio
			 * callbacks wake on a tight period; shaving
			 * the wake-tick freq is exactly the kind of
			 * micro-saving that turns into audible
			 * underruns.
			 */
			.sleeper_tail_thresh_us = 0,
			.sleeper_tail_pct = 100,
			/* Peer-ramp: keep both windows armed.  Audio
			 * worker often migrates LITTLE -> BIG on a
			 * spike (resampler / mixer), and the IPC chain
			 * to the audio HAL stays warm with the screen
			 * off.  Suppressing peer_ramp_off here would
			 * defeat the profile.
			 */
			.peer_ramp_window_ms =
				ZENITH_DEFAULT_PEER_RAMP_WINDOW_MS,
			.peer_ramp_floor_pct = 65,
			.peer_ramp_window_off_ms = 25,
			.migration_jump_pct =
				ZENITH_DEFAULT_MIGRATION_JUMP_PCT,
			.migration_floor_window_ms =
				ZENITH_DEFAULT_MIGRATION_FLOOR_WINDOW_MS,
			.migration_floor_pct =
				ZENITH_DEFAULT_MIGRATION_FLOOR_PCT,
			.psi_cpu_floor_thresh =
				ZENITH_DEFAULT_PSI_CPU_FLOOR_THRESH,
			/* Frame-overrun unchanged from BALANCED;
			 * audio cadence is independent of vblank.
			 */
			.frame_overrun_slack_us = 4000,
			.frame_overrun_window_ms =
				ZENITH_DEFAULT_FRAME_OVERRUN_WINDOW_MS,
			.frame_overrun_floor_pct =
				ZENITH_DEFAULT_FRAME_OVERRUN_FLOOR_PCT,
			.frame_overrun_deep_streak = 0,
			.frame_overrun_deep_floor_pct = 100,
			/* PSI-mem cap off: capping the cluster on
			 * memstall risks underruns the same way
			 * sleeper-tail shaving would.  pct/window
			 * left at sensible defaults for forward
			 * compat with sysfs overrides.
			 */
			.psi_mem_cap_thresh = 0,
			.psi_mem_cap_pct = 80,
			.psi_mem_cap_window_ms = 1000,
			.audio_hyst_ms = 750,
			.vh_arch_freq_scale_enable = 0,
			.vh_uclamp_observer_enable = 0,
			.vh_cpu_idle_enable = 1,
			.vh_freq_qos_enable = 0,
			.vh_sched_move_task_enable = 0,
			.vh_scheduler_tick_enable = 0,
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
	t->batt_hold_scale_pct	= p->batt_hold_scale_pct;
	t->cluster_wake_pulse_ms = p->cluster_wake_pulse_ms;
	t->cluster_wake_pulse_idle_ms = p->cluster_wake_pulse_idle_ms;
	t->cluster_wake_pulse_floor_pct = p->cluster_wake_pulse_floor_pct;
	t->quiet_hours_cap_pct	= p->quiet_hours_cap_pct;
	t->quiet_hours_screen_off_only = p->quiet_hours_screen_off_only;
	t->fg_transition_pulse_ms = p->fg_transition_pulse_ms;
	t->fg_transition_pulse_pct = p->fg_transition_pulse_pct;
	t->screen_on_bias_pct	= p->screen_on_bias_pct;
	t->input_boost_down_rate_mult_pct =
		p->input_boost_down_rate_mult_pct;
	WRITE_ONCE(t->predict_up_thresh, p->predict_up_thresh);
	WRITE_ONCE(t->predict_up_window, p->predict_up_window);
	WRITE_ONCE(t->pelt_rising_edge_thresh, p->pelt_rising_edge_thresh);
	WRITE_ONCE(t->pelt_rising_edge_min_pct, p->pelt_rising_edge_min_pct);
	WRITE_ONCE(t->dl_task_floor_pct, p->dl_task_floor_pct);
	WRITE_ONCE(t->io_floor_hyst_ms, p->io_floor_hyst_ms);
	WRITE_ONCE(t->io_floor_hyst_pct, p->io_floor_hyst_pct);
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
	WRITE_ONCE(t->audio_hyst_ms, p->audio_hyst_ms);
	WRITE_ONCE(t->vh_arch_freq_scale_enable,
		   p->vh_arch_freq_scale_enable);
	WRITE_ONCE(t->vh_uclamp_observer_enable,
		   p->vh_uclamp_observer_enable);
	WRITE_ONCE(t->vh_cpu_idle_enable,
		   p->vh_cpu_idle_enable);
	WRITE_ONCE(t->vh_freq_qos_enable,
		   p->vh_freq_qos_enable);
	WRITE_ONCE(t->vh_sched_move_task_enable,
		   p->vh_sched_move_task_enable);
	WRITE_ONCE(t->vh_scheduler_tick_enable,
		   p->vh_scheduler_tick_enable);
	WRITE_ONCE(zenith_frame_overrun_slack_us_cache,
		   p->frame_overrun_slack_us);
	WRITE_ONCE(zenith_frame_overrun_window_ms_cache,
		   p->frame_overrun_window_ms);
	/* Patch B10-3: copy the profile's cgroup-v2 PSI path into
	 * the per-tunables buffer (NULL bake -> empty string =
	 * system-wide PSI = pre-B10 behaviour) and refresh the
	 * file-scope cached cgroup pointer.  Identical-path
	 * applies are no-ops inside zenith_psi_cgroup_apply() so
	 * profile-bake from multiple policies doesn't churn the
	 * cgroup ref.
	 */
	{
		const char *cg_path = p->psi_cgroup_path ?: "";

		strscpy(t->psi_cgroup_path, cg_path,
			sizeof(t->psi_cgroup_path));
		zenith_psi_cgroup_apply(cg_path);
	}

	/* Mirror input_boost_ms and input_boost_touchdown_extra_ms to
	 * the governor-wide caches used by the input handler fast
	 * path so a profile flip is picked up on the next event.
	 */
	WRITE_ONCE(zenith_input_boost_active_ms, t->input_boost_ms);
	WRITE_ONCE(zenith_input_boost_touchdown_extra_ms_cache,
		   t->input_boost_touchdown_extra_ms);

	/* Patch L: emit a one-line summary of the bake result so the
	 * operator can confirm in dmesg / logcat which profile is
	 * live and what the major tunables were set to.  Fires on
	 * every apply path -- user-driven profile_store, AUTO-driven
	 * worker, and the early-balanced-then-AUTO bootstrap inside
	 * profile_store's AUTO branch.  Gated on verbose_log inside
	 * the helper.
	 */
	zenith_log_profile_applied(t, prof);

	/* Propagate the profile to the thermal stack so Kasumi's
	 * dampening window and Iyashi's performance floor track
	 * the governor's intent.  BALANCED bakes the compile-time
	 * defaults in both subsystems so this is a no-op on the
	 * cold-boot path.
	 */
	kasumi_apply_profile(prof);
	iyashi_apply_profile(prof);
	hikari_apply_profile(prof);
}

/* Patch B-AUTO-4: auto-selector classifier (priority cascade).
 *
 * Returns the concrete profile zenith should run while
 * active_profile == ZENITH_PROFILE_AUTO.  Reads governor-wide
 * signals plus a per-policy walk of attr_set->policy_list and
 * picks one of:
 *
 *   AUDIO       any process holds an open ALSA fd (atomic
 *               refcount maintained by the snd_pcm_open /
 *               snd_pcm_release vendor hooks).  Highest priority
 *               -- audio glitches are the most user-visible
 *               regression and AUDIO bake is the lowest-jitter
 *               profile.
 *   GAMING      any policy currently sees a game-engine thread
 *               on a runqueue (Unity / Unreal / Cocos2d / etc;
 *               see zenith_game_auto_comms[]) OR has render-
 *               thread saturation while the v4l2 fd refcount is
 *               zero (camera takes precedence over render via
 *               PERFORMANCE).  Game profile prioritises sustained
 *               throughput + frame-pacing over efficiency.
 *   PERFORMANCE recent input event (< 1500 ms) AND screen on,
 *               OR camera fd open (capture pipelines need
 *               headroom but not the GAMING bake).
 *   BATTERY     screen off AND running on battery.  Most
 *               aggressive efficiency bake; safe because the
 *               user is by definition not interacting.
 *   BALANCED    fallback when no signal fires.  Sane default
 *               that handles screen-on idle, charger-attached
 *               idle, and any state the more specific
 *               classifiers do not catch.
 *
 * LEGACY and CUSTOM are never picked -- they are explicit opt-
 * out paths.
 *
 * The classifier is read-only with respect to tunables and z_-
 * policy state (no field is mutated; no lock is taken beyond the
 * list-walk RCU-equivalent under attr_set->update_lock that the
 * caller already holds).  All atomic / shared-state reads are
 * READ_ONCE / atomic_read / atomic64_read and racing values are
 * tolerated -- the worst case is a single 500 ms eval window
 * delayed pick, debounced again by auto_hysteresis_ms.
 *
 * Recency window for PERFORMANCE: 1500 ms.  Long enough to ride
 * through a single scroll gesture's quiet phase, short enough
 * that a finished interaction does not pin PERFORMANCE forever.
 */
#define ZENITH_AUTO_INPUT_RECENT_NS	(1500ULL * NSEC_PER_MSEC)

static unsigned int zenith_auto_classify(struct zenith_tunables *t)
{
	struct zenith_policy *z_policy;
	bool audio;
	bool camera;
	bool game = false;
	bool render = false;
	bool input_recent;
	bool screen_on;
	bool on_battery;
	u64 now_ns;
	u64 last_input_ns;

	audio = atomic_read(&zenith_alsa_active_fds) > 0;
	if (audio)
		return ZENITH_PROFILE_AUDIO;

	camera = atomic_read(&zenith_v4l2_active_fds) > 0;
	on_battery = atomic_read(&zenith_on_battery) > 0;
	screen_on = READ_ONCE(t->screen_state) != 0;
	now_ns = ktime_get_ns();
	last_input_ns = atomic64_read(&zenith_input_last_event_ns);
	input_recent = last_input_ns &&
		       (now_ns - last_input_ns) < ZENITH_AUTO_INPUT_RECENT_NS;

	/* Game / render walk over attr_set->policy_list.  The caller
	 * (zenith_auto_eval_work_fn) holds attr_set->update_lock so
	 * the policy_list is stable here; the per-policy detectors
	 * are already cache-TTL'd so a 2 - 4 policy walk costs at
	 * most one strncmp loop per per-policy cache miss (sub-second
	 * window).  Render is OR'd into game when no camera fd is
	 * open -- a saturated render thread without a camera capture
	 * happening is dominated by the GAMING bake; with a camera
	 * fd open the camera takes precedence and the cascade lands
	 * on PERFORMANCE.
	 */
	list_for_each_entry(z_policy, &t->attr_set.policy_list,
			    tunables_hook) {
		if (zenith_policy_has_game_auto(z_policy)) {
			game = true;
			break;
		}
		if (!camera && zenith_policy_has_render(z_policy))
			render = true;
	}
	if (game || render)
		return ZENITH_PROFILE_GAMING;

	if (camera)
		return ZENITH_PROFILE_PERFORMANCE;

	/* Patch B9-3+: high-pressure FREQ_QOS_MIN raised by a vendor /
	 * thermal / ADPF requester within the last
	 * ZENITH_VH_FREQ_QOS_WINDOW_MS.  Tunable-gated
	 * (vh_freq_qos_enable) so the consumer side mirrors the
	 * probe's own write-side gate.  pressure_until_ns is 0 on
	 * cold boot (kzalloc); the s64 comparison treats 0 < now_ns
	 * correctly without wrap.
	 */
	if (READ_ONCE(t->vh_freq_qos_enable) &&
	    atomic64_read(&t->vh_freq_qos_pressure_until_ns) > (s64)now_ns)
		return ZENITH_PROFILE_PERFORMANCE;

	if (input_recent && screen_on)
		return ZENITH_PROFILE_PERFORMANCE;

	if (!screen_on && on_battery)
		return ZENITH_PROFILE_BATTERY;

	return ZENITH_PROFILE_BALANCED;
}

/* Patch B-AUTO-3: auto-selector worker.
 *
 * Runs on the system unbound deferrable workqueue at
 * t->auto_eval_ms cadence whenever active_profile ==
 * ZENITH_PROFILE_AUTO.  See the eval_work comment in struct
 * zenith_tunables for the design notes; this is the worker body
 * itself.
 *
 * Off-switch path:
 *   - active_profile != AUTO  -> return without rearming.  This
 *     is the path that fires when the user writes "balanced" /
 *     "performance" / etc to the profile sysfs node.
 *     profile_store will subsequently cancel_delayed_work_sync
 *     to drain any in-flight invocation.
 *
 * On-path:
 *   - target = zenith_auto_classify(t)  (B-AUTO-3 stub: BALANCED;
 *                                        B-AUTO-4: cascade)
 *   - hysteresis: if target == auto_target the engine is already
 *     converged; clear pending state and rearm.  Otherwise the
 *     pending target's first_seen timestamp gates the commit
 *     until auto_hysteresis_ms has elapsed.  When the timer
 *     fires we commit -- zenith_apply_profile under
 *     attr_set->update_lock plus zenith_refresh_rate_delays /
 *     zenith_invalidate_cache so cached per-policy state for the
 *     old profile does not leak forward.
 *   - active_profile *stays* at AUTO across commits; only
 *     auto_target reflects the engine's current pick.
 *
 * Cadence-zero path:
 *   - if auto_eval_ms == 0 the engine is paused.  The worker
 *     still rearms (default cadence 500 ms) so a userspace write
 *     to a non-zero auto_eval_ms re-enters the active path
 *     promptly, but the classifier is skipped and no profile
 *     mutation runs.  This makes auto_eval_ms = 0 a runtime
 *     enable/disable toggle without changing the work struct
 *     lifecycle.
 *
 * Concurrency:
 *   - active_profile is read once outside the lock as an early-
 *     out so the cancel-in-progress path (profile_store on AUTO
 *     exit) does not deadlock on update_lock waiting for
 *     cancel_delayed_work_sync to drain.
 *   - all profile-bake mutation paths take attr_set->update_lock
 *     (mutex, sleepable) just like profile_store does, so the
 *     two writers serialise cleanly.
 */
void zenith_auto_eval_work_fn(struct work_struct *w)
{
	struct zenith_tunables *t = container_of(to_delayed_work(w),
						 struct zenith_tunables,
						 eval_work);
	unsigned int eval_ms;
	unsigned int hyst_ms;
	unsigned int target;
	unsigned int current_target;
	unsigned int user_eval_ms;

	/* Off-switch: read active_profile lock-free; if userspace has
	 * already pivoted to a manual profile we exit immediately
	 * without rearming so cancel_delayed_work_sync converges.
	 */
	if (READ_ONCE(t->active_profile) != ZENITH_PROFILE_AUTO)
		return;

	user_eval_ms = READ_ONCE(t->auto_eval_ms);
	hyst_ms = READ_ONCE(t->auto_hysteresis_ms);
	eval_ms = user_eval_ms ? user_eval_ms : ZENITH_DEFAULT_AUTO_EVAL_MS;

	/* Cadence-zero gate: auto_eval_ms == 0 is the runtime pause
	 * switch.  We still rearm at the default cadence so a future
	 * userspace write to a non-zero auto_eval_ms re-enters the
	 * active path promptly, but we skip the classifier and the
	 * profile commit so no tunable mutation happens while paused.
	 */
	if (user_eval_ms == 0)
		goto rearm;

	target = zenith_auto_classify(t);

	mutex_lock(&t->attr_set.update_lock);

	/* Re-check active_profile under the lock to close the race
	 * between the lock-free early-out above and a concurrent
	 * profile_store that took the lock first to switch us to a
	 * manual profile.  When the race fires we drop the lock and
	 * exit without rearming -- profile_store's
	 * cancel_delayed_work_sync follow-up will then converge.
	 */
	if (t->active_profile != ZENITH_PROFILE_AUTO) {
		mutex_unlock(&t->attr_set.update_lock);
		return;
	}

	current_target = READ_ONCE(t->auto_target);

	if (target == current_target) {
		/* Already converged on this target; clear any pending
		 * hysteresis tracking so a fresh divergence starts
		 * with a clean first_seen timestamp.
		 */
		t->auto_pending_target = current_target;
		t->auto_pending_first_seen_ns = 0;
		goto unlock;
	}

	if (t->auto_pending_target != target) {
		t->auto_pending_target = target;
		t->auto_pending_first_seen_ns = ktime_get_ns();
		goto unlock;
	}

	/* Pending target unchanged across this window; check whether
	 * the hysteresis timer has elapsed.
	 */
	if (hyst_ms) {
		u64 now = ktime_get_ns();
		u64 first_seen = t->auto_pending_first_seen_ns;
		u64 hyst_ns = (u64)hyst_ms * NSEC_PER_MSEC;

		if (!first_seen || now - first_seen < hyst_ns)
			goto unlock;
	}

	/* Hysteresis satisfied -- commit the new profile bake.
	 * active_profile stays at AUTO so the engine keeps running.
	 */
	zenith_apply_profile(t, target);
	WRITE_ONCE(t->auto_target, target);
	t->auto_pending_target = target;
	t->auto_pending_first_seen_ns = 0;

	zenith_refresh_rate_delays(&t->attr_set);
	zenith_invalidate_cache(&t->attr_set);

unlock:
	mutex_unlock(&t->attr_set.update_lock);

rearm:
	/* Final guard against the cancel_delayed_work_sync race --
	 * if active_profile was flipped out of AUTO while we held
	 * the lock above, do not rearm.
	 */
	if (READ_ONCE(t->active_profile) == ZENITH_PROFILE_AUTO)
		schedule_delayed_work(&t->eval_work,
				      msecs_to_jiffies(eval_ms));
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
	if (!strcmp(s, "performance")) {
		zenith_cmdline_profile = ZENITH_PROFILE_PERFORMANCE;
	} else if (!strcmp(s, "balanced")) {
		zenith_cmdline_profile = ZENITH_PROFILE_BALANCED;
	} else if (!strcmp(s, "battery")) {
		zenith_cmdline_profile = ZENITH_PROFILE_BATTERY;
	} else if (!strcmp(s, "legacy")) {
		zenith_cmdline_profile = ZENITH_PROFILE_LEGACY;
	} else if (!strcmp(s, "gaming")) {
		zenith_cmdline_profile = ZENITH_PROFILE_GAMING;
	} else if (!strcmp(s, "audio")) {
		zenith_cmdline_profile = ZENITH_PROFILE_AUDIO;
	} else if (!strcmp(s, "custom")) {
		zenith_cmdline_profile = ZENITH_PROFILE_CUSTOM;
	} else {
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
unsigned int __init zenith_parse_profile_name(const char *s)
{
	if (!strcmp(s, "performance"))
		return ZENITH_PROFILE_PERFORMANCE;
	if (!strcmp(s, "balanced"))
		return ZENITH_PROFILE_BALANCED;
	if (!strcmp(s, "battery"))
		return ZENITH_PROFILE_BATTERY;
	if (!strcmp(s, "legacy"))
		return ZENITH_PROFILE_LEGACY;
	if (!strcmp(s, "gaming"))
		return ZENITH_PROFILE_GAMING;
	if (!strcmp(s, "audio"))
		return ZENITH_PROFILE_AUDIO;
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

