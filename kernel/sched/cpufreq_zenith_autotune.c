/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Zenith cpufreq governor - auto-tune observer and tunables ktype.
 *
 * Split from cpufreq_zenith.c for maintainability.  This file contains the
 * auto-tune delayed-work classifier (zenith_auto_tune_work), V2 state-machine
 * transition logic, auto-tune-related sysfs show/store, and the
 * zenith_tunables_ktype release path.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "cpufreq_zenith_internal.h"

/************************ Auto-tune observer *****************************/

/* Classify the workload seen since the last pass and pick a profile.
 * Runs from a delayed_work context on an arbitrary CPU. The per-policy
 * sample counters are atomics, so no lock is needed to sample+reset
 * them here even though zenith_update_single() increments them without
 * holding update_lock.
 */
void zenith_auto_tune_work(struct work_struct *w)
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
	bool thermal = false;
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

	/* Patch 1.2: refresh the AC-vs-battery cache once per
	 * auto_tune window.  Cheap (a single power_supply iterator
	 * walk per 10 s), keeps the hot path lock-free, and is the
	 * earliest point in the periodic chain where it makes sense
	 * to update -- the per-policy worker is the only periodic
	 * timer in the governor and it already runs only when
	 * auto_tune is on.  When no power supply driver is
	 * registered the helper returns -ENODEV / -ENOSYS and the
	 * cache stays at 0 (AC), so default behaviour is preserved.
	 */
	{
		int psy = power_supply_is_system_supplied();

		if (psy >= 0)
			atomic_set(&zenith_on_battery, psy ? 0 : 1);
	}

	/* Wave B PMU IPC tracker.  Sample per-CPU instructions /
	 * cycles perf_events once per auto_tune window.  Cheap
	 * (perf_event_read_value() walks one IPI per CPU; total cost
	 * for a 4-CPU policy is roughly 4 IPIs per ZENITH_AUTO_TUNE_-
	 * PERIOD_MS, 10 s by default).  Updates the per-CPU ipc_pct
	 * cache that zenith_get_next_freq()'s pmu_ipc_floor block
	 * reads via zenith_policy_max_ipc_pct().  When pmu_aware is 0
	 * the sampling still runs but the result is unused (always
	 * cheap to compute, and keeps the cache fresh in case the
	 * tunable is flipped on at runtime).  Compiles out on
	 * CONFIG_PERF_EVENTS=n via the #else branch in the helper.
	 */
	{
		unsigned int cpu;

		for_each_cpu(cpu, z_policy->policy->cpus)
			zenith_pmu_sample_cpu(cpu);
	}

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

	/* Audit fix F2: PELT util-rising trend.  Sum per-cpu PELT
	 * util_avg across this policy and compare against the previous
	 * window's value.  Raise ZENITH_AT_FLAG_UTIL_RISING when the
	 * delta exceeds the configured percentage.  Done above the V2
	 * signal block so the flag is available to the state machine.
	 *
	 * Math: pct_growth = ((cur - prev) * 100) / max(prev, 1).
	 * Zero out the previous-sample on the first window after init
	 * so the first comparison doesn't see 100% growth from 0->N.
	 */
	{
		unsigned int cpu;
		unsigned long util_sum = 0;

		for_each_cpu(cpu, z_policy->policy->cpus) {
			struct rq *rq = cpu_rq(cpu);

			util_sum += READ_ONCE(rq->cfs.avg.util_avg);
		}

		if (z_policy->at_last_util_sum &&
		    t->auto_tune_util_rising_thresh_pct) {
			unsigned long prev = z_policy->at_last_util_sum;
			unsigned long delta = util_sum > prev ?
				util_sum - prev : 0;
			unsigned int pct = prev ?
				(unsigned int)((delta * 100) / prev) : 0;

			if (pct >= t->auto_tune_util_rising_thresh_pct)
				flags |= ZENITH_AT_FLAG_UTIL_RISING;
		}
		z_policy->at_last_util_sum = util_sum;
	}

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

	/* Master gate covers both the level signal (thermal_state) and
	 * the slope signal (auto_tune_thermal_slope) the V2 evaluator
	 * uses to drive the THERMAL_RECOVERY state.  When thermal_aware
	 * == 0 we pin both inputs to 0 here so the THERMAL_RECOVERY
	 * branches below never fire; the screen / PSI / frame /
	 * variance branches are unaffected because they don't depend
	 * on either signal.  See ZENITH_DEFAULT_THERMAL_AWARE comment
	 * block.
	 */
	if (ZENITH_FEATURE_ENABLED(thermal_aware)) {
		thermal = READ_ONCE(t->thermal_state);
		if (t->auto_tune_v2 && t->auto_tune_thermal_slope) {
			thermal_pressure =
				zenith_policy_thermal_pressure_pct(z_policy);
			if (thermal_pressure > z_policy->at_last_thermal_pressure)
				thermal_delta = thermal_pressure -
					z_policy->at_last_thermal_pressure;
			thermal_slope = thermal_pressure >=
					t->auto_tune_thermal_pressure_pct ||
				thermal_delta >= t->auto_tune_thermal_slope_pct;
			if (thermal_slope)
				flags |= ZENITH_AT_FLAG_THERMAL_SLOPE;
		}
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
	if (static_branch_likely(&zenith_auto_tune_v3_key)) {
		unsigned int v3_mode = READ_ONCE(t->auto_tune_v3);

		if (v3_mode != ZENITH_AT_V3_MODE_OFF)
			zenith_at_v3_calibrate(z_policy, v3_mode);
	}

	{
		/* F1: pick the faster reschedule cadence whenever a
		 * scenario flag is active in the just-completed window.
		 * The check uses at_last_flags (set above by the
		 * decision path), so a scenario that ended IN this
		 * window still gets one fast follow-up window before
		 * we settle back to the slow cadence -- catches the
		 * case where the burst leaves residual variance worth
		 * a quick re-eval.
		 */
		const unsigned int fast_mask =
			ZENITH_AT_FLAG_CAMERA |
			ZENITH_AT_FLAG_RENDER |
			ZENITH_AT_FLAG_FRAME  |
			ZENITH_AT_FLAG_GAME   |
			ZENITH_AT_FLAG_MEMSTALL |
			ZENITH_AT_FLAG_THERMAL_SLOPE |
			ZENITH_AT_FLAG_PSI_CPU;
		unsigned int period =
			(z_policy->at_last_flags & fast_mask) ?
			ZENITH_AUTO_TUNE_FAST_PERIOD_MS :
			ZENITH_AUTO_TUNE_PERIOD_MS;

		schedule_delayed_work(&z_policy->at_work,
				      msecs_to_jiffies(period));
	}
}

static ssize_t auto_tune_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->auto_tune);
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
			zenith_at_v_reset_window(z_policy);
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

ZENITH_TUNABLE_UINT_MAX(auto_tune_v2, 1);

/* auto_tune_v2_glides sysfs knob.  Boolean (0/1).  See
 * ZENITH_DEFAULT_AUTO_TUNE_V2_GLIDES.  Master gate for V2-driven
 * population of the round-U-z10 glide / coordination knobs;
 * defaults to 1 so the new soft-glide behaviour is on out of the
 * box without forcing operators to write seven separate sysfs
 * entries.  Per-knob user writes still take precedence.
 */
ZENITH_TUNABLE_UINT_MAX(auto_tune_v2_glides, 1);

/* auto_tune_v2_tiers sysfs knob (Patch L).  Boolean (0/1).  See
 * ZENITH_DEFAULT_AUTO_TUNE_V2_TIERS.  Master gate for V2-driven
 * arming of the Stage-4 K1/K2/K3 floor tiers; defaults to 1 so
 * the V2 classifier shapes those tiers per state out of the box.
 * Set to 0 to lock all three tiers back to pure profile-driven
 * behaviour without disabling the rest of the V2 classifier.
 * Per-knob user sysfs writes still take precedence either way
 * (via the auto_tune_override_mask path).
 */
ZENITH_TUNABLE_UINT_MAX(auto_tune_v2_tiers, 1);

static ssize_t auto_tune_hysteresis_windows_show(struct gov_attr_set *attr_set,
						 char *buf)
{
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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

/* Audit fix F2: PELT util-rising trend threshold sysfs knob.
 *
 * Read-only show / write-with-clamp store.  See the
 * ZENITH_DEFAULT_AT_UTIL_RISING_THRESH_PCT comment block for
 * semantics.  0 disables the signal cleanly.
 */
static ssize_t auto_tune_util_rising_thresh_pct_show(struct gov_attr_set *attr_set,
						     char *buf)
{
	return sysfs_emit(buf, "%u\n",
		to_zenith_tunables(attr_set)->auto_tune_util_rising_thresh_pct);
}

static ssize_t auto_tune_util_rising_thresh_pct_store(struct gov_attr_set *attr_set,
						      const char *buf,
						      size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_AT_UTIL_RISING_THRESH_PCT_MAX)
		return -EINVAL;
	t->auto_tune_util_rising_thresh_pct = val;
	return count;
}

static struct governor_attr auto_tune_util_rising_thresh_pct =
	__ATTR_RW(auto_tune_util_rising_thresh_pct);

/* Audit fix F3: render-thread RT-priority floor (uclamp-min-style)
 * sysfs knob.  Range 0..100; 0 disables the floor cleanly.  See the
 * ZENITH_DEFAULT_AT_RENDER_RT_FLOOR_PCT comment block for semantics.
 */
static ssize_t auto_tune_render_rt_floor_pct_show(struct gov_attr_set *attr_set,
						  char *buf)
{
	return sysfs_emit(buf, "%u\n",
		to_zenith_tunables(attr_set)->auto_tune_render_rt_floor_pct);
}

static ssize_t auto_tune_render_rt_floor_pct_store(struct gov_attr_set *attr_set,
						   const char *buf,
						   size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_AT_RENDER_RT_FLOOR_PCT_MAX)
		return -EINVAL;
	t->auto_tune_render_rt_floor_pct = val;
	return count;
}

static struct governor_attr auto_tune_render_rt_floor_pct =
	__ATTR_RW(auto_tune_render_rt_floor_pct);

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
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->auto_tune_v3);
}

static ssize_t auto_tune_v3_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_policy;
	unsigned int old;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_AT_V3_MODE_MAX)
		return -EINVAL;
	old = t->auto_tune_v3;
	t->auto_tune_v3 = val;
	zenith_set_static_key(&zenith_auto_tune_v3_key, val);
	if (val == ZENITH_AT_V3_MODE_OFF) {
		list_for_each_entry(z_policy, &attr_set->policy_list,
				    tunables_hook) {
			z_policy->at_v3_hyst_offset = 0;
			z_policy->at_v3_cool_offset = 0;
			z_policy->at_v3_last_calib_ns = 0;
			z_policy->at_v3_last_transitions = 0;
			/* Patch J: clear the calibration audit ring on the
			 * V3 master-disable transition so a subsequent
			 * re-enable starts the audit trail from a clean
			 * baseline, matching the rest of the V3 fields
			 * cleared here.
			 */
			memset(z_policy->at_v3_calib_log, 0,
			       sizeof(z_policy->at_v3_calib_log));
			z_policy->at_v3_calib_log_head = 0;
			z_policy->at_v3_calib_log_count = 0;
		}
	}
	/* V3 mode flips alter the offsets fed into
	 * zenith_at_eff_hyst_windows() / zenith_at_eff_cool_windows()
	 * on the freq-decision path.  Drop any cached effective
	 * windows so the next tick recomputes on the new mode --
	 * matters most on MODE_APPLY -> MODE_OFF and MODE_DRY ->
	 * MODE_APPLY transitions where the offset surface changes
	 * shape under the cache.
	 */
	zenith_invalidate_cache(attr_set);
	if (old != val)
		zenith_log_master_flip(t, "auto_tune_v3", old, val);
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
	return sysfs_emit(buf, "%u\n",
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
		pos += sysfs_emit_at(buf, pos,
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

/* Patch J: auto_tune_v3_calib_log RO sysfs.  Dumps the per-policy
 * V3 calibration audit ring -- one line per calibration tick,
 * oldest first, capped at ZENITH_AT_V3_CALIB_LOG_NR entries.
 * Format chosen to fit comfortably under PAGE_SIZE for the common
 * HMP topology (2 policies x 8 entries) while remaining grep
 * friendly: every key is name=value with no quoted strings or
 * commas.
 *
 * Mode is reported as the integer ZENITH_AT_V3_MODE_* value so a
 * scraper can index into it directly; hyst/cool deltas are emitted
 * as (before -> after) pairs so the operator can see at a glance
 * which entries actually moved the offsets and which were OBSERVE
 * passes or rail-clamped APPLY passes.
 */
static ssize_t auto_tune_v3_calib_log_show(struct gov_attr_set *attr_set,
					   char *buf)
{
	struct zenith_policy *z_pol;
	ssize_t len = 0;

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		unsigned int count = z_pol->at_v3_calib_log_count;
		unsigned int head = z_pol->at_v3_calib_log_head;
		unsigned int start, i;

		if (count > ZENITH_AT_V3_CALIB_LOG_NR)
			count = ZENITH_AT_V3_CALIB_LOG_NR;
		start = (count == ZENITH_AT_V3_CALIB_LOG_NR) ? head : 0;

		len += sysfs_emit_at(buf, len,
				 "policy%u: %u entries\n",
				 z_pol->policy ? z_pol->policy->cpu : 0,
				 count);
		if (len >= PAGE_SIZE)
			break;

		for (i = 0; i < count; i++) {
			struct zenith_at_v3_calib_log_entry *e =
				&z_pol->at_v3_calib_log[
					(start + i) %
					ZENITH_AT_V3_CALIB_LOG_NR];

			len += sysfs_emit_at(buf, len,
				"  ts_ns=%llu mode=%u trans=%u hyst=%d->%d cool=%d->%d\n",
				(unsigned long long)e->ts_ns,
				(unsigned int)e->mode,
				e->transitions,
				(int)e->hyst_before, (int)e->hyst_after,
				(int)e->cool_before, (int)e->cool_after);
			if (len >= PAGE_SIZE)
				break;
		}
		if (len >= PAGE_SIZE)
			break;
	}
	return len;
}
static struct governor_attr auto_tune_v3_calib_log =
	__ATTR_RO(auto_tune_v3_calib_log);

static ssize_t auto_tune_cluster_aware_show(struct gov_attr_set *attr_set,
					    char *buf)
{
	return sysfs_emit(buf, "%u\n",
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

ZENITH_TUNABLE_UINT_MAX(auto_tune_v2_signals, 1);

ZENITH_TUNABLE_UINT_MAX(auto_tune_thermal_slope, 1);

static ssize_t auto_tune_thermal_pressure_pct_show(struct gov_attr_set *attr_set,
						   char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->
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
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->
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

ZENITH_TUNABLE_UINT_MAX(auto_tune_frame_pacing, 1);

static ssize_t auto_tune_sustained_gaming_show(struct gov_attr_set *attr_set,
					       char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->
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
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->_name); \
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
ZENITH_TUNABLE_UINT_MAX(auto_tune_scenario, 1);

static ssize_t profile_show(struct gov_attr_set *attr_set, char *buf)
{
	switch (to_zenith_tunables(attr_set)->active_profile) {
	case ZENITH_PROFILE_PERFORMANCE:	return sysfs_emit(buf, "performance\n");
	case ZENITH_PROFILE_BALANCED:		return sysfs_emit(buf, "balanced\n");
	case ZENITH_PROFILE_BATTERY:		return sysfs_emit(buf, "battery\n");
	case ZENITH_PROFILE_LEGACY:		return sysfs_emit(buf, "legacy\n");
	case ZENITH_PROFILE_GAMING:		return sysfs_emit(buf, "gaming\n");
	case ZENITH_PROFILE_AUDIO:		return sysfs_emit(buf, "audio\n");
	/* Patch B-AUTO-2: AUTO is the meta-profile that engages the
	 * auto-selector engine.  Userspace sees "auto"; the concrete
	 * profile the engine has applied is exposed separately via
	 * the auto_target RO sysfs node.
	 */
	case ZENITH_PROFILE_AUTO:		return sysfs_emit(buf, "auto\n");
	case ZENITH_PROFILE_CUSTOM:
	default:				return sysfs_emit(buf, "custom\n");
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
	else if (sysfs_streq(buf, "gaming"))
		prof = ZENITH_PROFILE_GAMING;
	else if (sysfs_streq(buf, "audio"))
		prof = ZENITH_PROFILE_AUDIO;
	else if (sysfs_streq(buf, "custom"))
		prof = ZENITH_PROFILE_CUSTOM;
	/* Patch B-AUTO-2: "auto" is the meta-profile that engages the
	 * auto-selector engine (B-AUTO-3 / B-AUTO-4).  On entry we
	 * apply the BALANCED bake immediately so the device runs on a
	 * known-safe baseline until the engine's first eval lands;
	 * active_profile then becomes AUTO so the engine knows it is
	 * free to pick a concrete target.
	 */
	else if (sysfs_streq(buf, "auto"))
		prof = ZENITH_PROFILE_AUTO;
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

	/* Patch L: log the profile transition before the bake runs so
	 * the dmesg trail reads chronologically against the
	 * subsequent "applied" summary line emitted by
	 * zenith_apply_profile().  Gated on verbose_log inside the
	 * helper.
	 */
	zenith_log_profile_change(t, t->active_profile, prof);

	/* Patch B-AUTO-2: AUTO is a meta-profile, not a tunables bake.
	 * Apply the BALANCED preset immediately (so the device is on a
	 * known-safe baseline before the auto-selector engine's first
	 * eval lands), then mark active_profile = AUTO and stamp the
	 * auto_target so the engine has a starting point.  The engine
	 * itself (B-AUTO-3 / B-AUTO-4) will refine the target on its
	 * 500 ms eval cadence with 2000 ms hysteresis.
	 */
	if (prof == ZENITH_PROFILE_AUTO) {
		zenith_apply_profile(t, ZENITH_PROFILE_BALANCED);
		WRITE_ONCE(t->auto_target, ZENITH_PROFILE_BALANCED);
		t->auto_pending_target = ZENITH_PROFILE_BALANCED;
		t->auto_pending_first_seen_ns = 0;
	} else {
		zenith_apply_profile(t, prof);
	}
	t->active_profile = prof;
	t->auto_tune_override_mask = 0;
	/* Patch B-AUTO-3: AUTO entry scheduling.  On entry we kick
	 * the eval worker so the first classifier run lands after
	 * one auto_eval_ms window rather than waiting for some
	 * external tick.  schedule_delayed_work is idempotent if the
	 * worker was already pending.
	 *
	 * On AUTO exit we *do not* call cancel_delayed_work_sync from
	 * here -- governor_store holds attr_set->update_lock across
	 * this entire path, the worker also acquires that lock to
	 * apply profile mutations, and a sync cancel while the worker
	 * is waiting on the same lock would deadlock.  Instead we
	 * rely on the worker's own lock-free
	 * READ_ONCE(active_profile) early-out: at most one stale
	 * worker invocation runs after the profile flip, observes
	 * active_profile != AUTO, and self-cancels (no rearm).  The
	 * synchronous drain happens later in zenith_tunables_free,
	 * which runs from the kobject release path with no lock
	 * held.
	 */
	if (prof == ZENITH_PROFILE_AUTO)
		schedule_delayed_work(&t->eval_work,
				      msecs_to_jiffies(t->auto_eval_ms ?
						       t->auto_eval_ms :
						       ZENITH_DEFAULT_AUTO_EVAL_MS));
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

/* Patch L: verbose_log RW sysfs.  Plain 0/1 boolean.  When 1, the
 * three helpers (zenith_log_profile_change, zenith_log_profile_-
 * applied, zenith_log_master_flip) emit "zenith:" prefixed
 * pr_info() lines on the user-driven sysfs write paths.  Default 0
 * so production builds aren't spammed.  Read via READ_ONCE inside
 * the helpers; written via WRITE_ONCE here.
 */
static ssize_t verbose_log_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->verbose_log);
}

static ssize_t verbose_log_store(struct gov_attr_set *attr_set,
				 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->verbose_log, val);
	return count;
}
static struct governor_attr verbose_log = __ATTR_RW(verbose_log);

/* Patch K: game_perf_burst master sysfs.  0/1 boolean.  See the
 * long ZENITH_DEFAULT_GAME_PERF_BURST comment block for the full
 * mechanism.  Toggling this also flips the matching static branch
 * so the FSM tick + floor application drop off the hot path entirely
 * when the master is 0.  Verbose-log gated dmesg trail emitted on
 * every flip (Patch L integration).
 */
static ssize_t game_perf_burst_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->game_perf_burst);
}

static ssize_t game_perf_burst_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int old;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	old = t->game_perf_burst;
	WRITE_ONCE(t->game_perf_burst, val);
	zenith_set_static_key(&zenith_game_perf_burst_key, val);
	if (old != val)
		zenith_log_master_flip(t, "game_perf_burst", old, val);
	return count;
}
static struct governor_attr game_perf_burst = __ATTR_RW(game_perf_burst);

/* Patch K: game_perf_burst_floor_pct sysfs.  Clamped to
 * [ZENITH_GAME_PERF_BURST_FLOOR_PCT_MIN .. _MAX].  WRITE_ONCE'd
 * because the value is read on the per-tick hot path inside the
 * FSM floor helper.
 */
static ssize_t game_perf_burst_floor_pct_show(struct gov_attr_set *attr_set,
					      char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->game_perf_burst_floor_pct);
}

static ssize_t game_perf_burst_floor_pct_store(struct gov_attr_set *attr_set,
					       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val < ZENITH_GAME_PERF_BURST_FLOOR_PCT_MIN)
		val = ZENITH_GAME_PERF_BURST_FLOOR_PCT_MIN;
	if (val > ZENITH_GAME_PERF_BURST_FLOOR_PCT_MAX)
		val = ZENITH_GAME_PERF_BURST_FLOOR_PCT_MAX;
	WRITE_ONCE(t->game_perf_burst_floor_pct, val);
	return count;
}
static struct governor_attr game_perf_burst_floor_pct =
	__ATTR_RW(game_perf_burst_floor_pct);

/* Patch K: game_perf_burst_thermal_ceiling_dc sysfs.  Millidegrees C
 * (matches the kernel thermal subsystem's unit, so this knob plumbs
 * straight through to thermal_zone_get_temp() comparisons).  Clamp
 * range 40..60 dC keeps the user from shooting themselves in the
 * foot with absurd values; the user requested a 45..50 dC operating
 * range so the default 48000 sits at the midpoint.
 */
static ssize_t game_perf_burst_thermal_ceiling_dc_show(
	struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
				game_perf_burst_thermal_ceiling_dc);
}

static ssize_t game_perf_burst_thermal_ceiling_dc_store(
	struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val < ZENITH_GAME_PERF_BURST_THERMAL_CEILING_DC_MIN)
		val = ZENITH_GAME_PERF_BURST_THERMAL_CEILING_DC_MIN;
	if (val > ZENITH_GAME_PERF_BURST_THERMAL_CEILING_DC_MAX)
		val = ZENITH_GAME_PERF_BURST_THERMAL_CEILING_DC_MAX;
	WRITE_ONCE(t->game_perf_burst_thermal_ceiling_dc, val);
	return count;
}
static struct governor_attr game_perf_burst_thermal_ceiling_dc =
	__ATTR_RW(game_perf_burst_thermal_ceiling_dc);

/* Patch K: game_perf_burst_disarm_grace_ms sysfs.  Hold time after
 * Signal B drops before transitioning ARMED -> COOLDOWN.  Default
 * 1000 keeps Alt+Tab snappy.  Clamp [0, 10000] (10s upper bound is
 * already absurd for "Alt+Tab grace"; anything more should be a
 * cooldown_ms tweak instead).
 */
static ssize_t game_perf_burst_disarm_grace_ms_show(
	struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
				game_perf_burst_disarm_grace_ms);
}

static ssize_t game_perf_burst_disarm_grace_ms_store(
	struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > ZENITH_GAME_PERF_BURST_DISARM_GRACE_MS_MAX)
		val = ZENITH_GAME_PERF_BURST_DISARM_GRACE_MS_MAX;
	WRITE_ONCE(t->game_perf_burst_disarm_grace_ms, val);
	return count;
}
static struct governor_attr game_perf_burst_disarm_grace_ms =
	__ATTR_RW(game_perf_burst_disarm_grace_ms);

/* Patch K: game_perf_burst_cooldown_ms sysfs.  Length of the
 * COOLDOWN-state linear floor glide back to 0.  Clamp [0, 60000]
 * (1 minute upper bound; longer than that is effectively a stuck
 * floor).  0 disables the glide -- floor drops to 0 immediately on
 * disarm, which is rarely what you want but is supported.
 */
static ssize_t game_perf_burst_cooldown_ms_show(struct gov_attr_set *attr_set,
						char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
				game_perf_burst_cooldown_ms);
}

static ssize_t game_perf_burst_cooldown_ms_store(struct gov_attr_set *attr_set,
						 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > ZENITH_GAME_PERF_BURST_COOLDOWN_MS_MAX)
		val = ZENITH_GAME_PERF_BURST_COOLDOWN_MS_MAX;
	WRITE_ONCE(t->game_perf_burst_cooldown_ms, val);
	return count;
}
static struct governor_attr game_perf_burst_cooldown_ms =
	__ATTR_RW(game_perf_burst_cooldown_ms);

/* Patch K: game_perf_burst_state RO sysfs.  Live FSM state across
 * all attached policies.  Walks the attr_set policy list and prints
 * one line per policy: "<cluster_first_cpu> <state>".  When all
 * policies are IDLE this returns a single "idle" line so userspace
 * tooling has a stable shape to grep / parse.
 *
 * Stable tokens (zenith_gpb_state_name): "idle" / "ARMED" /
 * "COOLDOWN".  Order-stable per policy attach order.
 */
static ssize_t game_perf_burst_state_show(struct gov_attr_set *attr_set,
					  char *buf)
{
	struct zenith_policy *z_policy;
	ssize_t off = 0;
	bool any_active = false;

	list_for_each_entry(z_policy, &attr_set->policy_list, tunables_hook) {
		int first_cpu = cpumask_first(z_policy->policy->cpus);
		const char *name = zenith_gpb_state_name(z_policy->gpb_state);

		off += sysfs_emit_at(buf, off, "%d %s\n",
				 first_cpu, name);
		if (z_policy->gpb_state != ZENITH_GPB_STATE_IDLE)
			any_active = true;
		if (off >= PAGE_SIZE - 32)
			break;
	}
	if (!any_active && off == 0)
		off = sysfs_emit(buf, "idle\n");
	return off;
}
static struct governor_attr game_perf_burst_state =
	__ATTR_RO(game_perf_burst_state);

/* Patch M: game_perf_burst_stats RO sysfs.  Per-policy lifetime
 * counters and the last ARMED -> COOLDOWN reason, exposed for
 * empirical tuning of disarm_grace_ms / cooldown_ms.  One line per
 * attached policy:
 *
 *   "<cluster_first_cpu> state=<name> arm=<n> disarm=<n>
 *    idle=<n> last_disarm=<token>\n"
 *
 * Token grammar:
 *   state         "idle" | "ARMED" | "COOLDOWN"
 *   last_disarm   "none" | "fast" | "sustained"
 *
 * arm:    IDLE -> ARMED + COOLDOWN -> ARMED transitions
 * disarm: ARMED -> COOLDOWN transitions
 * idle:   COOLDOWN -> IDLE transitions (full-glide completions;
 *         arm > idle means we have re-armed during cooldown).
 *
 * Zero-initialised per attach (zenith_start) so a governor switch
 * resets the counters and userspace can compute rates over a known
 * baseline.  Order-stable per policy attach order.
 */
static ssize_t game_perf_burst_stats_show(struct gov_attr_set *attr_set,
					  char *buf)
{
	struct zenith_policy *z_policy;
	ssize_t off = 0;

	list_for_each_entry(z_policy, &attr_set->policy_list, tunables_hook) {
		int first_cpu = cpumask_first(z_policy->policy->cpus);
		const char *state_name =
			zenith_gpb_state_name(z_policy->gpb_state);
		const char *disarm_name =
			zenith_gpb_disarm_name(
				z_policy->gpb_last_disarm_reason);

		off += sysfs_emit_at(buf, off,
				 "%d state=%s arm=%u disarm=%u idle=%u last_disarm=%s\n",
				 first_cpu, state_name,
				 z_policy->gpb_arm_count,
				 z_policy->gpb_disarm_count,
				 z_policy->gpb_idle_count,
				 disarm_name);
		if (off >= PAGE_SIZE - 64)
			break;
	}
	return off;
}
static struct governor_attr game_perf_burst_stats =
	__ATTR_RO(game_perf_burst_stats);

/* Patch B-AUTO-2: auto_target RO sysfs.  When active_profile ==
 * ZENITH_PROFILE_AUTO this prints the concrete profile the auto-
 * selector engine has currently applied (BALANCED, PERFORMANCE,
 * BATTERY, GAMING, AUDIO).  When active_profile != AUTO this still
 * returns the last value the engine wrote -- it is a debug-only
 * window into the engine state.  Read with READ_ONCE so a torn
 * write from the engine worker cannot produce a malformed string.
 */
static ssize_t auto_target_show(struct gov_attr_set *attr_set, char *buf)
{
	unsigned int target = READ_ONCE(to_zenith_tunables(attr_set)->auto_target);

	switch (target) {
	case ZENITH_PROFILE_PERFORMANCE:	return sysfs_emit(buf, "performance\n");
	case ZENITH_PROFILE_BALANCED:		return sysfs_emit(buf, "balanced\n");
	case ZENITH_PROFILE_BATTERY:		return sysfs_emit(buf, "battery\n");
	case ZENITH_PROFILE_GAMING:		return sysfs_emit(buf, "gaming\n");
	case ZENITH_PROFILE_AUDIO:		return sysfs_emit(buf, "audio\n");
	default:				return sysfs_emit(buf, "balanced\n");
	}
}
static struct governor_attr auto_target = __ATTR_RO(auto_target);

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

	len += sysfs_emit_at(buf, len,
			 "version=%u\n", ZENITH_AT_STATUS_FORMAT_VERSION);
	len += sysfs_emit_at(buf, len,
			 "auto_tune=%u\n", t->auto_tune);
	len += sysfs_emit_at(buf, len,
			 "auto_tune_v2=%u glides=%u tiers=%u\n",
			 t->auto_tune_v2, t->auto_tune_v2_glides,
			 t->auto_tune_v2_tiers);
	len += sysfs_emit_at(buf, len,
			 "v2_knobs=cluster:%u signals:%u thermal_slope:%u frame:%u gaming:%u\n",
			 t->auto_tune_cluster_aware,
			 t->auto_tune_v2_signals,
			 t->auto_tune_thermal_slope,
			 t->auto_tune_frame_pacing,
			 t->auto_tune_sustained_gaming);
	len += sysfs_emit_at(buf, len, "profile=%s\n",
			 zenith_profile_name(t->active_profile));
	len += sysfs_emit_at(buf, len,
			 "override_mask=0x%lx\n", t->auto_tune_override_mask);
	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		len += sysfs_emit_at(buf, len,
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
			len += sysfs_emit_at(buf, len,
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
			len += sysfs_emit_at(buf, len,
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
		len += sysfs_emit_at(buf, len,
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
		len += sysfs_emit_at(buf, len, "%s-extra: ",
				 profs[i].name);
		len += sysfs_emit_at(buf, len,
				 "down_rate_adaptive=%u ",
				 scratch.down_rate_adaptive);
		len += sysfs_emit_at(buf, len,
				 "wakeup_boost=%u ", scratch.wakeup_boost);
		len += sysfs_emit_at(buf, len,
				 "down_threshold_adaptive=%u ",
				 scratch.down_threshold_adaptive);
		len += sysfs_emit_at(buf, len,
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
		[ZENITH_STAT_AUTO_THERMAL_CAP]	= "auto_thermal_cap",
		[ZENITH_STAT_QUIET_HOURS_CAP]	= "quiet_hours_cap",
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
		len += sysfs_emit_at(buf, len, "%s=%lu\n",
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
	return sysfs_emit(buf,
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

		len += sysfs_emit_at(buf, len,
				 "policy%u(%s): %u entries\n",
				 z_pol->policy->cpu,
				 zenith_at_cluster_name(z_pol->cluster_class),
				 count);
		if (len >= PAGE_SIZE)
			break;

		for (i = 0; i < count; i++) {
			struct zenith_at_log_entry *e =
				&z_pol->at_log[(start + i) % ZENITH_AT_LOG_NR];

			len += sysfs_emit_at(buf, len,
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
		len += sysfs_emit_at(buf, len,
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

/* Patch B7-2: decision_ring sysfs node.  Read-only.  Dumps the
 * per-policy ring of the last ZENITH_DEC_RING_NR (path, lat_us)
 * pairs newest-first.  Format:
 *
 *   policy<cpu>(<cluster>):
 *     <tag> <lat_us>
 *     ...
 *
 * The ring is a power-of-two circular buffer; entries with a NULL
 * path are uninitialised (the ring has not yet wrapped through
 * those slots since the policy was created) and are skipped.
 *
 * Reader is single-shot per sysfs read; the eval path keeps
 * advancing concurrently, so the snapshot is best-effort.
 * READ_ONCE on the path pointer makes the read torn-write-safe;
 * lat_ns is sampled without strict ordering relative to path
 * which can occasionally pair a path with the lat_ns from the
 * neighbouring slot (tolerable for an observability dump).
 */
static ssize_t decision_ring_show(struct gov_attr_set *attr_set, char *buf)
{
	struct zenith_policy *z_pol;
	ssize_t len = 0;

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		unsigned int head = READ_ONCE(z_pol->dec_ring_head);
		unsigned int i;

		len += sysfs_emit_at(buf, len,
				 "policy%u(%s):\n",
				 z_pol->policy->cpu,
				 zenith_at_cluster_name(z_pol->cluster_class));
		if (len >= PAGE_SIZE)
			break;

		for (i = 0; i < ZENITH_DEC_RING_NR; i++) {
			unsigned int idx =
				(head - 1 - i) & ZENITH_DEC_RING_MASK;
			const char *p = READ_ONCE(z_pol->dec_ring[idx].path);
			u32 lat_ns = z_pol->dec_ring[idx].lat_ns;

			if (!p)
				continue;
			len += sysfs_emit_at(buf, len,
					 "  %s %u\n", p, lat_ns / 1000);
			if (len >= PAGE_SIZE)
				break;
		}
		if (len >= PAGE_SIZE)
			break;
	}
	return len;
}

static struct governor_attr decision_ring = __ATTR_RO(decision_ring);

/* Patch C8: decision_confidence sysfs node.  Read-only.  Walks the
 * per-policy dec_ring (ZENITH_DEC_RING_NR == 32 entries) and tallies
 * how many of the last 32 evals were won by each tp_path.  Output
 * is one line per distinct path actually seen, sorted by descending
 * count, with absolute count and percent-of-window:
 *
 *   policy0(little):
 *     hispeed       18 56%
 *     predict_up     6 18%
 *     pelt_edge      4 12%
 *     dl_floor       2  6%
 *     ...
 *
 * Useful for diagnosing which tier is doing the work in a given
 * load profile -- e.g. confirming pelt_edge is firing on cold-wake
 * scrolling, or checking that dl_floor is dormant on BALANCED.
 *
 * Cost: one PAGE_SIZE buffer pass per policy on read.  No locking
 * (READ_ONCE on path; tally is on stack).  Pointer-compare is
 * sufficient because tp_path is always a string literal -- gcc
 * pools all literals so the same path always points to the same
 * address; if a future tier writes a non-literal, strcmp would be
 * needed but the existing dec_ring sample logic stores the literal
 * pointer as well, so any path captured into the ring is also a
 * literal.
 */
#define ZENITH_CONF_TALLY_MAX	16

static ssize_t
decision_confidence_show(struct gov_attr_set *attr_set, char *buf)
{
	struct zenith_policy *z_pol;
	ssize_t len = 0;

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook) {
		struct {
			const char	*path;
			unsigned int	count;
		} tally[ZENITH_CONF_TALLY_MAX];
		unsigned int n_tally = 0;
		unsigned int total = 0;
		unsigned int head = READ_ONCE(z_pol->dec_ring_head);
		unsigned int i, j;

		memset(tally, 0, sizeof(tally));

		for (i = 0; i < ZENITH_DEC_RING_NR; i++) {
			unsigned int idx =
				(head - 1 - i) & ZENITH_DEC_RING_MASK;
			const char *p = READ_ONCE(z_pol->dec_ring[idx].path);

			if (!p)
				continue;
			total++;

			for (j = 0; j < n_tally; j++) {
				if (tally[j].path == p) {
					tally[j].count++;
					break;
				}
			}
			if (j == n_tally && n_tally < ZENITH_CONF_TALLY_MAX) {
				tally[n_tally].path = p;
				tally[n_tally].count = 1;
				n_tally++;
			}
		}

		for (i = 0; i + 1 < n_tally; i++) {
			unsigned int max_j = i;

			for (j = i + 1; j < n_tally; j++) {
				if (tally[j].count > tally[max_j].count)
					max_j = j;
			}
			if (max_j != i) {
				const char *tp = tally[i].path;
				unsigned int tc = tally[i].count;

				tally[i].path = tally[max_j].path;
				tally[i].count = tally[max_j].count;
				tally[max_j].path = tp;
				tally[max_j].count = tc;
			}
		}

		len += sysfs_emit_at(buf, len,
				 "policy%u(%s) total=%u:\n",
				 z_pol->policy->cpu,
				 zenith_at_cluster_name(z_pol->cluster_class),
				 total);
		if (len >= PAGE_SIZE)
			break;

		for (i = 0; i < n_tally; i++) {
			unsigned int pct = total ?
				(tally[i].count * 100) / total : 0;

			len += sysfs_emit_at(buf, len,
					 "  %-16s %3u %3u%%\n",
					 tally[i].path, tally[i].count, pct);
			if (len >= PAGE_SIZE)
				break;
		}
		if (len >= PAGE_SIZE)
			break;
	}
	return len;
}

static struct governor_attr decision_confidence =
	__ATTR_RO(decision_confidence);

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
	return sysfs_emit(buf, "%u\n",
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

ZENITH_TUNABLE_UINT_MAX(screen_auto, 1);
ZENITH_TUNABLE_UINT_BOOL_INVAL(thermal_state);

ZENITH_TUNABLE_UINT_MAX(thermal_auto, 1);

/* thermal_pressure_continuous sysfs knob.  Strict 0/1 boolean.  When
 * 1, dynamic_up_thresh ramps linearly from the policy's normal
 * up_threshold (at 0 %% thermal pressure) to 90 (at 100 %% thermal
 * pressure) instead of cliff-jumping to 90 the instant
 * zenith_thermal_active() flips true.  See the field comment on
 * struct zenith_tunables for the rationale.
 */
ZENITH_TUNABLE_UINT_BOOL_INVAL(thermal_pressure_continuous);

/* thermal_aware sysfs knob.  Master gate over the cluster of
 * thermal-driven freq adjustments (thermal_util_derate, the
 * thermal_pressure_continuous up_thresh ramp, auto_thermal_cap, the
 * V2 THERMAL_RECOVERY transitions).  Strict 0/1 boolean.  Mirrors
 * the value into zenith_thermal_aware_key so the gated branches
 * fold to no-ops in the off case.  See ZENITH_DEFAULT_THERMAL_AWARE
 * for rationale.
 */
static ssize_t thermal_aware_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->thermal_aware);
}

static ssize_t thermal_aware_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int old;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	old = t->thermal_aware;
	t->thermal_aware = val;
	zenith_set_static_key(&zenith_thermal_aware_key, val);
	/* Master-switch flips alter the freq-decision surface fed by
	 * the thermal cluster (thermal_util_derate, the
	 * thermal_pressure_continuous up_thresh ramp, auto_thermal_cap,
	 * V2 THERMAL_RECOVERY).  Drop any cached snapshots so the
	 * next tick recomputes on the new gate state, matching the
	 * symmetry of thermal_pressure_continuous_store above.
	 */
	zenith_invalidate_cache(attr_set);
	if (old != val)
		zenith_log_master_flip(t, "thermal_aware", old, val);
	return count;
}
static struct governor_attr thermal_aware = __ATTR_RW(thermal_aware);

/* thermal_active sysfs knob.  Read-only mirror of
 * zenith_thermal_active(): 1 when the governor currently believes
 * thermal pressure is at or above the threshold, 0 otherwise.
 * Written by the kernel from zenith_thermal_active() on every call.
 * Always 0 when thermal_aware is 0.  Read with READ_ONCE because
 * the writes from per-policy update paths can race with sysfs
 * reads, and tearing the unsigned int read across CPUs is benign
 * but observable.
 */
static ssize_t thermal_active_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       READ_ONCE(to_zenith_tunables(attr_set)->thermal_active));
}
static struct governor_attr thermal_active = __ATTR_RO(thermal_active);

/* prefer_silver_aware: strict 0/1.  See struct zenith_tunables for
 * semantics.  When CONFIG_SCHED_PREFER_SILVER=n the field is still
 * stored and round-tripped via sysfs so userspace tools that probe
 * the governor's tunable list don't choke on a missing node, but
 * the run-time bump path is dead because the worker stub never
 * updates ps_hit_rate_pct.
 */
ZENITH_TUNABLE_UINT_BOOL_INVAL(prefer_silver_aware);

/* prefer_silver_hot_threshold_pct: 0..100.  When the per-window
 * prefer_silver hit-rate is at or above this percentage,
 * prefer_silver_aware fires the bump on big / prime clusters.
 */
static ssize_t prefer_silver_hot_threshold_pct_show(
		struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
ZENITH_TUNABLE_UINT_MAX(thermal_util_derate, 1);

ZENITH_TUNABLE_UINT_MAX(thermal_derate_rate_pct, 100);

static ssize_t auto_thermal_cap_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_thermal_cap);
}

static ssize_t auto_thermal_cap_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	t->auto_thermal_cap = val;
	/* Cap master switch governs whether the auto_thermal_cap_*_pct
	 * pair feeds the freq decision; flip needs a cache drop so the
	 * next tick recomputes the headroom on the new gate state.
	 */
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr auto_thermal_cap =
	__ATTR_RW(auto_thermal_cap);

static ssize_t auto_thermal_cap_pressure_pct_show(struct gov_attr_set *attr_set,
						  char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->
			       auto_thermal_cap_pressure_pct);
}

static ssize_t auto_thermal_cap_pressure_pct_store(struct gov_attr_set *attr_set,
						   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val < ZENITH_AUTO_THERMAL_CAP_PRESSURE_PCT_MIN ||
	    val > ZENITH_AUTO_THERMAL_CAP_PRESSURE_PCT_MAX)
		return -EINVAL;
	t->auto_thermal_cap_pressure_pct = val;
	return count;
}
static struct governor_attr auto_thermal_cap_pressure_pct =
	__ATTR_RW(auto_thermal_cap_pressure_pct);

static ssize_t auto_thermal_cap_freq_pct_show(struct gov_attr_set *attr_set,
					      char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->auto_thermal_cap_freq_pct);
}

static ssize_t auto_thermal_cap_freq_pct_store(struct gov_attr_set *attr_set,
					       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val < ZENITH_AUTO_THERMAL_CAP_FREQ_PCT_MIN ||
	    val > ZENITH_AUTO_THERMAL_CAP_FREQ_PCT_MAX)
		return -EINVAL;
	t->auto_thermal_cap_freq_pct = val;
	return count;
}
static struct governor_attr auto_thermal_cap_freq_pct =
	__ATTR_RW(auto_thermal_cap_freq_pct);

static ssize_t freq_stability_margin_pct_show(struct gov_attr_set *attr_set,
					      char *buf)
{
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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

ZENITH_TUNABLE_UINT_MAX(wakeup_boost, 1);

/* wakeup_boost_ms sysfs knob.  Range 0..ZENITH_WAKEUP_BOOST_MS_MAX.
 * 0 disables the wall-clock bypass and leaves only the legacy
 * tick-based ZENITH_WAKEUP_BOOST_TICKS countdown.  Non-zero arms a
 * deadline at the detection sites; the up-rate bypass holds until
 * either the tick counter expires or the deadline lapses.
 */
ZENITH_TUNABLE_UINT_MAX(wakeup_boost_ms, ZENITH_WAKEUP_BOOST_MS_MAX);

static ssize_t rate_limit_cluster_scale_show(struct gov_attr_set *attr_set,
					     char *buf)
{
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->input_boost_ms);
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

ZENITH_TUNABLE_UINT_MAX_INVAL(input_boost_decay_ms, 1000);

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
	return sysfs_emit(buf, "%u\n",
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
ZENITH_TUNABLE_UINT_MAX(input_boost_decay_curve, 1);

ZENITH_TUNABLE_UINT_MAX(input_boost_big_only, 1);

static ssize_t input_boost_cap_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
		return sysfs_emit(buf, "0\n");

	for (i = 0; i < t->eff_nr; i++)
		len += sysfs_emit_at(buf, len, "%u%c",
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
	return sysfs_emit(buf, "%u\n",
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
		return sysfs_emit(buf, "%u\n", t->up_delay_us);

	for (i = 0; i < t->eff_nr; i++)
		len += sysfs_emit_at(buf, len, "%u%c",
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

ZENITH_TUNABLE_UINT_MAX_INVAL(light_load_freq, ZENITH_LIGHT_LOAD_FREQ_MAX);

ZENITH_TUNABLE_UINT_MAX_INVAL(light_load_threshold, 100);

static ssize_t sampling_down_factor_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
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

ZENITH_TUNABLE_UINT_MAX(boost_exit_extend, 1);

ZENITH_TUNABLE_UINT_MAX_INVAL(bias_load_threshold, 100);

static ssize_t up_threshold_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->up_threshold);
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->down_threshold);
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
	return sysfs_emit(buf, "%u\n",
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

ZENITH_TUNABLE_UINT_MAX_INVAL(hispeed_freq, ZENITH_HISPEED_FREQ_MAX);

ZENITH_TUNABLE_UINT_MAX_INVAL(hispeed_freq_pct, 100);

static ssize_t hispeed_load_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->hispeed_load);
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

ZENITH_TUNABLE_UINT_MAX(hispeed_hyst_pct, 100);

/* hispeed_entry_streak sysfs knob.  See ZENITH_DEFAULT_HISPEED_ENTRY_STREAK
 * for semantics.  Capped to ZENITH_HISPEED_ENTRY_STREAK_MAX on store
 * so the per-policy u8 streak counter cannot overflow.
 */
static ssize_t hispeed_entry_streak_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
ZENITH_TUNABLE_UINT_MAX(peak_headroom_rescue, 1);

/* peak_headroom_starve_load_pct sysfs knob.  Minimum cluster
 * load_pct (0..100, util / max_cap * 100) at which a sample counts
 * as "starving" for the peak-headroom rescue tier.  Accepts 1..100;
 * 0 is rejected (rescue would fire on every sample).
 */
static ssize_t peak_headroom_starve_load_pct_show(struct gov_attr_set *attr_set,
						  char *buf)
{
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
ZENITH_TUNABLE_UINT_MAX(peak_headroom_starve_streak, ZENITH_PEAK_HEADROOM_STREAK_MAX);

/* peak_headroom_jump_pct sysfs knob.  Target as percentage of
 * policy->max for the rescue freq.  Accepts 1..100; 100 pins to
 * policy->max (the default), 50 rescues to half of policy->max,
 * etc.  0 is rejected (rescue with target 0 would never raise
 * freq).
 */
static ssize_t peak_headroom_jump_pct_show(struct gov_attr_set *attr_set,
					   char *buf)
{
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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

/* batt_hold_scale_pct sysfs knob (Patch 1.2).  Percentage applied
 * to peak-rescue hold-down deadlines when the system is on
 * battery (zenith_on_battery == 1).  Profile-baked: PERFORMANCE
 * keeps 100 (no scaling), BALANCED extends to 120, BATTERY to
 * 180, LEGACY 100.  Accepts 50..300.
 */
static ssize_t batt_hold_scale_pct_show(struct gov_attr_set *attr_set,
					char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->batt_hold_scale_pct);
}

static ssize_t batt_hold_scale_pct_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val < ZENITH_BATT_HOLD_SCALE_PCT_MIN ||
	    val > ZENITH_BATT_HOLD_SCALE_PCT_MAX)
		return -EINVAL;
	t->batt_hold_scale_pct = val;
	return count;
}

static struct governor_attr batt_hold_scale_pct =
	__ATTR_RW(batt_hold_scale_pct);

/* Wave A charger-aware floor knobs.  charger_aware is a 0/1 gate;
 * charger_floor_pct is the floor as a percentage of policy->max,
 * applied when the gate is on AND the AC-vs-battery cache reports
 * !on_battery.  See the comment block above
 * ZENITH_DEFAULT_CHARGER_AWARE for the full rationale.
 */
ZENITH_TUNABLE_UINT_MAX(charger_aware, 1);
ZENITH_TUNABLE_UINT_MAX(charger_floor_pct, ZENITH_CHARGER_FLOOR_PCT_MAX);

/* Wave A cgroup-aware top-app floor knobs.  top_app_aware is a 0/1
 * gate; top_app_floor_pct is the floor as a percentage of
 * policy->max, applied when the gate is on AND any CPU in the
 * policy is currently running a task in the cpuset cgroup named
 * "top-app".  See the comment block above
 * ZENITH_DEFAULT_TOP_APP_AWARE for the full rationale.
 */
ZENITH_TUNABLE_UINT_MAX(top_app_aware, 1);
ZENITH_TUNABLE_UINT_MAX(top_app_floor_pct, ZENITH_TOP_APP_FLOOR_PCT_MAX);

/* Wave A render-thread util tracker knobs.  render_thread_util_aware
 * is a 0/1 gate; render_thread_util_thresh is the util_avg threshold
 * (1/SCHED_CAPACITY_SCALE units, 0..1024); render_thread_util_floor_pct
 * is the floor as a percentage of policy->max applied when the gate
 * is on AND a render thread is observed AND its util_avg >= thresh.
 * See the comment block above ZENITH_DEFAULT_RENDER_THREAD_UTIL_AWARE
 * for the full rationale.
 */
ZENITH_TUNABLE_UINT_MAX(render_thread_util_aware, 1);
ZENITH_TUNABLE_UINT_MAX(render_thread_util_thresh,
			ZENITH_RENDER_THREAD_UTIL_THRESH_MAX);
ZENITH_TUNABLE_UINT_MAX(render_thread_util_floor_pct,
			ZENITH_RENDER_THREAD_UTIL_FLOOR_PCT_MAX);

/* Wave B PMU IPC tracker knobs.  pmu_aware is a 0/1 gate; pmu_ipc_-
 * thresh is the IPC threshold in percent (100 = 1.0 IPC, default
 * 100, max 1000); pmu_ipc_floor_pct is the floor as a percentage of
 * policy->max applied when the gate is on AND the policy's max
 * sampled IPC across CPUs is >= thresh.  See the comment block above
 * ZENITH_DEFAULT_PMU_AWARE for the full rationale.
 */
ZENITH_TUNABLE_UINT_MAX(pmu_aware, 1);
ZENITH_TUNABLE_UINT_MAX(pmu_ipc_thresh, ZENITH_PMU_IPC_THRESH_MAX);
ZENITH_TUNABLE_UINT_MAX(pmu_ipc_floor_pct, ZENITH_PMU_IPC_FLOOR_PCT_MAX);

/* Wave B EAS energy-knee floor knobs.  em_aware is a 0/1 gate;
 * em_floor_pct is the floor as a percentage of the policy's energy-
 * knee freq, capped at 200% to allow the user to express "a bit
 * above the knee for safety margin".  See the comment block above
 * ZENITH_DEFAULT_EM_AWARE for the full rationale.
 */
ZENITH_TUNABLE_UINT_MAX(em_aware, 1);
ZENITH_TUNABLE_UINT_MAX(em_floor_pct, ZENITH_EM_FLOOR_PCT_MAX);

/* on_battery sysfs read-only diagnostic (Patch 1.2).  Reports the
 * current AC-vs-battery cache state (0 = AC / system-supplied, 1
 * = on battery).  Updated lazily once per ZENITH_AUTO_TUNE_PERIOD
 * by zenith_auto_tune_work().  Useful for an operator wondering
 * why a battery-aware tunable is or isn't engaging.
 */
static ssize_t on_battery_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       (unsigned int)atomic_read(&zenith_on_battery));
}

static struct governor_attr on_battery = __ATTR_RO(on_battery);

/* cluster_wake_pulse_ms sysfs knob (Patch 1.3).  Width of the soft
 * floor armed when the cluster wakes from a >= cluster_wake_pulse_-
 * idle_ms gap.  Accepts 0 (disable the tier) up to ZENITH_CLUSTER_-
 * WAKE_PULSE_MS_MAX (200 ms).
 */
static ssize_t cluster_wake_pulse_ms_show(struct gov_attr_set *attr_set,
					  char *buf)
{
	return sysfs_emit(buf, "%u\n",
		to_zenith_tunables(attr_set)->cluster_wake_pulse_ms);
}

static ssize_t cluster_wake_pulse_ms_store(struct gov_attr_set *attr_set,
					   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_CLUSTER_WAKE_PULSE_MS_MAX)
		return -EINVAL;
	t->cluster_wake_pulse_ms = val;
	return count;
}

static struct governor_attr cluster_wake_pulse_ms =
	__ATTR_RW(cluster_wake_pulse_ms);

/* cluster_wake_pulse_idle_ms sysfs knob.  Minimum gap between
 * consecutive evals required before the wake-pulse arms.  Bounded
 * to 0..ZENITH_CLUSTER_WAKE_PULSE_IDLE_MS_MAX (1000 ms).  0 means
 * "any gap qualifies", which combined with cluster_wake_pulse_ms
 * non-zero would arm the pulse on every eval; users wanting that
 * effect should set both knobs explicitly.
 */
static ssize_t cluster_wake_pulse_idle_ms_show(struct gov_attr_set *attr_set,
					       char *buf)
{
	return sysfs_emit(buf, "%u\n",
		to_zenith_tunables(attr_set)->cluster_wake_pulse_idle_ms);
}

static ssize_t cluster_wake_pulse_idle_ms_store(struct gov_attr_set *attr_set,
						const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_CLUSTER_WAKE_PULSE_IDLE_MS_MAX)
		return -EINVAL;
	t->cluster_wake_pulse_idle_ms = val;
	return count;
}

static struct governor_attr cluster_wake_pulse_idle_ms =
	__ATTR_RW(cluster_wake_pulse_idle_ms);

/* cluster_wake_pulse_floor_pct sysfs knob.  Floor as percentage of
 * policy->max held for the wake-pulse window.  Accepts 0..100;
 * 0 stamps the deadline but suppresses the floor application,
 * mirroring the peer_ramp / migration_floor knob shape.
 */
ZENITH_TUNABLE_UINT_MAX(cluster_wake_pulse_floor_pct, 100);

/* quiet_hours_start_min / quiet_hours_end_min sysfs knobs (Patch
 * 1.10).  Both accept 0..1439 (minutes since 00:00 UTC).  When
 * start == end, the tier is disabled.  When start > end, the
 * window wraps midnight.  See zenith_in_quiet_hours().
 */
static ssize_t quiet_hours_start_min_show(struct gov_attr_set *attr_set,
					  char *buf)
{
	return sysfs_emit(buf, "%u\n",
		to_zenith_tunables(attr_set)->quiet_hours_start_min);
}

static ssize_t quiet_hours_start_min_store(struct gov_attr_set *attr_set,
					   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_QUIET_HOURS_MINUTE_MAX)
		return -EINVAL;
	t->quiet_hours_start_min = val;
	return count;
}

static struct governor_attr quiet_hours_start_min =
	__ATTR_RW(quiet_hours_start_min);

static ssize_t quiet_hours_end_min_show(struct gov_attr_set *attr_set,
					char *buf)
{
	return sysfs_emit(buf, "%u\n",
		to_zenith_tunables(attr_set)->quiet_hours_end_min);
}

static ssize_t quiet_hours_end_min_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_QUIET_HOURS_MINUTE_MAX)
		return -EINVAL;
	t->quiet_hours_end_min = val;
	return count;
}

static struct governor_attr quiet_hours_end_min =
	__ATTR_RW(quiet_hours_end_min);

/* quiet_hours_cap_pct: 50..100, profile-baked.  100 disables the
 * cap (no-op, default).  Smaller values cap freq harder during
 * the window.  Floor of 50 mirrors batt_hold_scale_pct's lower
 * bound and avoids surprising users with a near-min cap.
 */
static ssize_t quiet_hours_cap_pct_show(struct gov_attr_set *attr_set,
					char *buf)
{
	return sysfs_emit(buf, "%u\n",
		to_zenith_tunables(attr_set)->quiet_hours_cap_pct);
}

static ssize_t quiet_hours_cap_pct_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val < ZENITH_QUIET_HOURS_CAP_PCT_MIN || val > 100)
		return -EINVAL;
	t->quiet_hours_cap_pct = val;
	return count;
}

static struct governor_attr quiet_hours_cap_pct =
	__ATTR_RW(quiet_hours_cap_pct);

/* quiet_hours_screen_off_only: 0 / 1.  Default 1 -- the cap only
 * fires while the screen is off, so a window that overlaps an
 * active call / alarm doesn't drag the cluster down.
 */
ZENITH_TUNABLE_UINT_MAX(quiet_hours_screen_off_only, 1);

/* Patch 1.4: decision_latency_hist sysfs node.
 *
 * Read-only.  Prints four numbers separated by spaces, terminated
 * with a newline:
 *
 *   <count_lt10us> <count_10_50us> <count_50_100us> <count_ge100us>
 *
 * Each count is the lifetime number of zenith_get_next_freq()
 * evals whose end-to-end cost (from function entry to commit
 * point) landed in the corresponding bucket on the policy that
 * owns this attribute set.  Reset on policy attach (zenith_start),
 * so the values reflect the current attach cycle only.
 *
 * Single-line space-separated format keeps userspace parsers
 * trivial (awk '{print $1, $2, $3, $4}'); no need to walk a
 * multi-line table.  A separate decisions counter (the existing
 * stats[] / decisions_total node) lets userspace compute the per-
 * eval cost distribution.
 */
static ssize_t decision_latency_hist_show(struct gov_attr_set *attr_set,
					  char *buf)
{
	struct zenith_policy *z_policy;
	unsigned long b0 = 0, b1 = 0, b2 = 0, b3 = 0;

	/* Same single-leader-cpu policy iteration pattern used by
	 * the existing stats sysfs handlers (e.g. decisions_total).
	 * The list is stable while the gov_attr_set is alive (held
	 * by the sysfs read), so a plain list_for_each_entry without
	 * an extra lock is correct.
	 */
	list_for_each_entry(z_policy, &attr_set->policy_list, tunables_hook) {
		b0 += z_policy->dec_lat_buckets[0];
		b1 += z_policy->dec_lat_buckets[1];
		b2 += z_policy->dec_lat_buckets[2];
		b3 += z_policy->dec_lat_buckets[3];
	}
	return sysfs_emit(buf, "%lu %lu %lu %lu\n", b0, b1, b2, b3);
}

static struct governor_attr decision_latency_hist =
	__ATTR_RO(decision_latency_hist);

/* Patch 1.9 fg-transition pulse sysfs knobs (RW, profile-baked).
 *
 * fg_transition_pulse_ms:
 *   Pulse duration in milliseconds.  0 disables the producer
 *   (the sched_wakeup_new probe never stamps a deadline) and is
 *   the BATTERY / LEGACY default.  Bounded to 0..200 to keep an
 *   accidentally-large value from holding the floor for an
 *   unreasonable stretch.
 *
 * fg_transition_pulse_pct:
 *   Floor depth in percent of policy->max.  0..100; 0 disables
 *   the consumer (deadline still gets stamped, but no floor is
 *   applied).
 */
static ssize_t fg_transition_pulse_ms_show(struct gov_attr_set *attr_set,
					   char *buf)
{
	return sysfs_emit(buf, "%u\n",
		to_zenith_tunables(attr_set)->fg_transition_pulse_ms);
}

static ssize_t fg_transition_pulse_ms_store(struct gov_attr_set *attr_set,
					    const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_FG_TRANSITION_PULSE_MS_MAX)
		return -EINVAL;
	t->fg_transition_pulse_ms = val;
	return count;
}

static struct governor_attr fg_transition_pulse_ms =
	__ATTR_RW(fg_transition_pulse_ms);

static ssize_t fg_transition_pulse_pct_show(struct gov_attr_set *attr_set,
					    char *buf)
{
	return sysfs_emit(buf, "%u\n",
		to_zenith_tunables(attr_set)->fg_transition_pulse_pct);
}

static ssize_t fg_transition_pulse_pct_store(struct gov_attr_set *attr_set,
					     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_FG_TRANSITION_PULSE_PCT_MAX)
		return -EINVAL;
	t->fg_transition_pulse_pct = val;
	return count;
}

static struct governor_attr fg_transition_pulse_pct =
	__ATTR_RW(fg_transition_pulse_pct);

/* peak_headroom_prearm sysfs knob.  Boolean gate for the soft early
 * intervention tier (2b') that lifts the cluster to eff_hispeed_freq
 * while the starvation streak is accumulating but has not yet
 * crossed peak_headroom_starve_streak.  Accepts 0 or 1 only.
 */
ZENITH_TUNABLE_UINT_MAX(peak_headroom_prearm, 1);

/* predict_up_thresh sysfs knob.  Trend threshold for the
 * predictive up-shift tier (2a') in 256ths of max_cap; see the
 * block comment above ZENITH_DEFAULT_PREDICT_UP_THRESH for what
 * the unit means and how the trigger is gated.  0 disables the
 * tier, max ZENITH_PREDICT_UP_THRESH_MAX (255).
 */
static ssize_t predict_up_thresh_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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

/* pelt_rising_edge_thresh sysfs knob (Patch C3).  See block
 * comment above ZENITH_DEFAULT_PELT_RISING_EDGE_THRESH for the
 * full semantics.  0 disables the tier; max
 * ZENITH_PELT_RISING_EDGE_THRESH_MAX (255).
 */
static ssize_t
pelt_rising_edge_thresh_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->pelt_rising_edge_thresh);
}

static ssize_t
pelt_rising_edge_thresh_store(struct gov_attr_set *attr_set,
			      const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PELT_RISING_EDGE_THRESH_MAX)
		return -EINVAL;
	WRITE_ONCE(t->pelt_rising_edge_thresh, val);
	return count;
}

static struct governor_attr pelt_rising_edge_thresh =
	__ATTR_RW(pelt_rising_edge_thresh);

/* pelt_rising_edge_min_pct sysfs knob (Patch C3).  Absolute-level
 * gate for the rising-edge tier; the tier only fires when the
 * newest util sample is at least this percent of max_cap.  Range
 * 0..ZENITH_PELT_RISING_EDGE_MIN_PCT_MAX (100).
 */
static ssize_t
pelt_rising_edge_min_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->pelt_rising_edge_min_pct);
}

static ssize_t
pelt_rising_edge_min_pct_store(struct gov_attr_set *attr_set,
			       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_PELT_RISING_EDGE_MIN_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->pelt_rising_edge_min_pct, val);
	return count;
}

static struct governor_attr pelt_rising_edge_min_pct =
	__ATTR_RW(pelt_rising_edge_min_pct);

/* dl_task_floor_pct sysfs knob (Patch C6).  Range 0..100.  When
 * any CPU in the policy has a SCHED_DEADLINE task, lift freq to
 * (policy->max * dl_task_floor_pct / 100).  0 disables the floor;
 * see ZENITH_DEFAULT_DL_TASK_FLOOR_PCT for the full block comment.
 */
static ssize_t
dl_task_floor_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->dl_task_floor_pct);
}

static ssize_t
dl_task_floor_pct_store(struct gov_attr_set *attr_set,
			const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_DL_TASK_FLOOR_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->dl_task_floor_pct, val);
	return count;
}

static struct governor_attr dl_task_floor_pct =
	__ATTR_RW(dl_task_floor_pct);

/* io_floor_hyst_ms sysfs knob (Patch C9).  Range 0..2000.  When
 * non-zero, every iowait_boost arming stamps a deadline that
 * keeps the io_floor tier active for io_floor_hyst_ms past the
 * last arming sample.  See ZENITH_DEFAULT_IO_FLOOR_HYST_MS for
 * the full block comment.
 */
static ssize_t
io_floor_hyst_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->io_floor_hyst_ms);
}

static ssize_t
io_floor_hyst_ms_store(struct gov_attr_set *attr_set,
		       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_IO_FLOOR_HYST_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->io_floor_hyst_ms, val);
	return count;
}

static struct governor_attr io_floor_hyst_ms =
	__ATTR_RW(io_floor_hyst_ms);

/* io_floor_hyst_pct sysfs knob (Patch C9).  Range 0..100.  Floor
 * value as a percentage of policy->max while the io_floor_until_ns
 * deadline has not expired.  0 disables the floor effect (a
 * non-zero deadline still gets stamped but no lift happens).
 */
static ssize_t
io_floor_hyst_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->io_floor_hyst_pct);
}

static ssize_t
io_floor_hyst_pct_store(struct gov_attr_set *attr_set,
			const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) ||
	    val > ZENITH_IO_FLOOR_HYST_PCT_MAX)
		return -EINVAL;
	WRITE_ONCE(t->io_floor_hyst_pct, val);
	return count;
}

static struct governor_attr io_floor_hyst_pct =
	__ATTR_RW(io_floor_hyst_pct);

/* peak_hysteresis_streak sysfs knob (Patch E).
 * Range 0..ZENITH_PEAK_HYSTERESIS_STREAK_MAX.  Number of
 * consecutive samples after a peak-class previous freq for
 * which the soft floor is held; 0 disables the hysteresis tier.
 */
static ssize_t
peak_hysteresis_streak_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
ZENITH_TUNABLE_UINT_MAX(brutal_decay_ms, ZENITH_BRUTAL_DECAY_MS_MAX);

ZENITH_TUNABLE_UINT_MAX_INVAL(climb_mode, ZENITH_CLIMB_MODE_STEP);

static ssize_t freq_step_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->freq_step_pct);
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
ZENITH_TUNABLE_UINT_MAX(freq_step_adaptive, 1);

ZENITH_TUNABLE_UINT_MAX_INVAL(powersave_bias, 1000);

/* screen_on_bias_pct sysfs knob.  See struct zenith_tunables doc and
 * ZENITH_DEFAULT_SCREEN_ON_BIAS_PCT for full semantics.  Range
 * 0..100 (percent).  100 disables the softening (legacy behaviour);
 * 50 (the default) halves the configured powersave_bias whenever
 * the screen is on; 0 zeroes the bias on screen-on.
 */
ZENITH_TUNABLE_UINT_MAX_INVAL(screen_on_bias_pct, 100);

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->up_rate_limit_us);
}

static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_pol;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_RATE_LIMIT_US_MAX)
		return -EINVAL;
	t->up_rate_limit_us = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_UP_RATE);

	list_for_each_entry(z_pol, &attr_set->policy_list, tunables_hook)
		zenith_update_rate_delay_ns(z_pol);
	return count;
}
static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->down_rate_limit_us);
}

static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	struct zenith_policy *z_pol;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > ZENITH_RATE_LIMIT_US_MAX)
		return -EINVAL;
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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

ZENITH_TUNABLE_UINT_BOOL_INVAL(kcpustat_hispeed_enable);

/* Strict-bool tunable selecting v1 (legacy cpu_util_cfs()) vs v2
 * (6.x-style runnable-aware util) input to schedutil_cpu_util in
 * zenith_get_util().  Invalidates the prev_freq cache so toggles
 * take effect on the next tick.
 */
ZENITH_TUNABLE_UINT_BOOL_INVAL(util_math_v2);

/* predict_util_pct sysfs knob.  See ZENITH_DEFAULT_PREDICT_UTIL_PCT
 * comment block at the top of the file for semantics.  Range
 * 0..ZENITH_PREDICT_UTIL_PCT_MAX; values above the cap are rejected
 * outright rather than silently clamped, so userspace gets a clear
 * EINVAL on out-of-range writes.  The freq cache is invalidated so
 * a toggle takes effect on the very next zenith_update tick.
 */
static ssize_t predict_util_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
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
ZENITH_TUNABLE_UINT_MAX(predict_util_smooth, 1);

/* render_aware sysfs knob.  Strict 0/1 boolean; non-zero values are
 * normalised to 1 on store so userspace can echo any truthy integer.
 * No cache invalidation is required: tunables->render_aware is read
 * fresh on every zenith_get_next_freq() call, so the next scheduler
 * tick already sees the new value.
 */
static ssize_t render_aware_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->render_aware);
}

static ssize_t render_aware_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int old;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	old = t->render_aware;
	t->render_aware = val;
	zenith_set_static_key(&zenith_render_aware_key, val);
	if (old != val)
		zenith_log_master_flip(t, "render_aware", old, val);
	return count;
}
static struct governor_attr render_aware = __ATTR_RW(render_aware);

/* render_floor_pct sysfs knob.  Range 0..100; 0 leaves the comm
 * walk running but applies no floor.  Out-of-range values rejected
 * with EINVAL so userspace gets a clear error.
 */
static ssize_t render_floor_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
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
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_RENDER_FLOOR_PCT);
	zenith_invalidate_cache(attr_set);
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->audio_aware);
}

static ssize_t audio_aware_store(struct gov_attr_set *attr_set,
				 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int old;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	old = t->audio_aware;
	t->audio_aware = val;
	zenith_set_static_key(&zenith_audio_aware_key, val);
	if (old != val)
		zenith_log_master_flip(t, "audio_aware", old, val);
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
	return sysfs_emit(buf, "%u\n",
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
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_AUDIO_FLOOR_PCT);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr audio_floor_pct = __ATTR_RW(audio_floor_pct);

/* audio_cap_pct sysfs knob.  Range 0..100; 0 disables the cap (the
 * floor side of the band can still apply alone).  Applied before the
 * uclamp_max final cap so ADPF power-efficiency hints still win.
 */
static ssize_t audio_cap_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
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
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_AUDIO_CAP_PCT);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr audio_cap_pct = __ATTR_RW(audio_cap_pct);

/* Patch B7-1: audio_hyst_ms sysfs knob.  Range 0..2000 (capped via
 * ZENITH_AUDIO_HYST_MS_MAX); 0 disables the sticky window so the
 * helper falls back to the cache-TTL-only behaviour.  Profile-baked
 * to a sane value per profile (PERFORMANCE/BALANCED/GAMING 250 ms,
 * BATTERY 100 ms, AUDIO 750 ms, LEGACY 0).
 */
static ssize_t audio_hyst_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->audio_hyst_ms);
}

static ssize_t audio_hyst_ms_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > ZENITH_AUDIO_HYST_MS_MAX)
		return -EINVAL;
	t->audio_hyst_ms = val;
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_AUDIO_HYST_MS);
	zenith_invalidate_cache(attr_set);
	return count;
}
static struct governor_attr audio_hyst_ms = __ATTR_RW(audio_hyst_ms);

/* Patch B9-1: vh_arch_freq_scale_enable sysfs knob.  Strict 0/1
 * boolean; gates the android_vh_arch_set_freq_scale vendor-hook
 * observer (see ZENITH_DEFAULT_VH_ARCH_FREQ_SCALE_ENABLE for the
 * full semantics and profile bakes).  Stored via plain assignment;
 * the probe reads it with READ_ONCE so a torn write would only
 * delay the gate flip by one realisation event.
 */
static ssize_t
vh_arch_freq_scale_enable_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->vh_arch_freq_scale_enable);
}

static ssize_t
vh_arch_freq_scale_enable_store(struct gov_attr_set *attr_set,
				const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->vh_arch_freq_scale_enable, val);
	return count;
}
static struct governor_attr vh_arch_freq_scale_enable =
	__ATTR_RW(vh_arch_freq_scale_enable);

/* Patch B9-2: vh_uclamp_observer_enable sysfs knob.  Strict 0/1
 * boolean; gates the android_vh_setscheduler_uclamp vendor-hook
 * observer (see ZENITH_DEFAULT_VH_UCLAMP_OBSERVER_ENABLE for full
 * semantics and profile bakes).  Stored via WRITE_ONCE; the probe
 * reads it with READ_ONCE so a torn write would only delay the
 * gate flip by one ADPF write.
 */
static ssize_t
vh_uclamp_observer_enable_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->vh_uclamp_observer_enable);
}

static ssize_t
vh_uclamp_observer_enable_store(struct gov_attr_set *attr_set,
				const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->vh_uclamp_observer_enable, val);
	return count;
}
static struct governor_attr vh_uclamp_observer_enable =
	__ATTR_RW(vh_uclamp_observer_enable);

/* Patch B9-3: vh_cpu_idle_enable sysfs knob.  Strict 0/1 boolean;
 * gates the android_vh_cpu_idle_enter / android_vh_cpu_idle_exit
 * vendor-hook observer pair (see ZENITH_DEFAULT_VH_CPU_IDLE_ENABLE
 * for full semantics and profile bakes).  Stored via WRITE_ONCE;
 * both probes and the cwp arm-site reader use READ_ONCE so a torn
 * write would at worst delay the gate flip by one cpuidle exit /
 * one eval window.
 */
static ssize_t
vh_cpu_idle_enable_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->vh_cpu_idle_enable);
}

static ssize_t
vh_cpu_idle_enable_store(struct gov_attr_set *attr_set,
			 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->vh_cpu_idle_enable, val);
	return count;
}

static struct governor_attr vh_cpu_idle_enable =
	__ATTR_RW(vh_cpu_idle_enable);

/* Patch B9-3+: vh_freq_qos_enable sysfs knob.  Strict 0/1 boolean;
 * gates the android_vh_freq_qos_update_request vendor-hook observer
 * (see ZENITH_DEFAULT_VH_FREQ_QOS_ENABLE for full semantics and
 * profile bakes).  Stored via WRITE_ONCE; the probe and the auto-
 * classify consumer both read with READ_ONCE so a torn write would
 * at worst delay the gate flip by one QoS update / one auto-eval
 * window.
 */
static ssize_t
vh_freq_qos_enable_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->vh_freq_qos_enable);
}

static ssize_t
vh_freq_qos_enable_store(struct gov_attr_set *attr_set,
			 const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->vh_freq_qos_enable, val);
	return count;
}

static struct governor_attr vh_freq_qos_enable =
	__ATTR_RW(vh_freq_qos_enable);

/* Patch B9-5: vh_sched_move_task_enable sysfs knob.  Strict 0/1
 * boolean; gates the android_vh_sched_move_task vendor-hook observer
 * (see ZENITH_DEFAULT_VH_SCHED_MOVE_TASK_ENABLE for full semantics
 * and profile bakes).  Stored via WRITE_ONCE; the probe reads with
 * READ_ONCE so a torn write would at worst delay the gate flip by
 * one cgroup move.
 */
static ssize_t
vh_sched_move_task_enable_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->vh_sched_move_task_enable);
}

static ssize_t
vh_sched_move_task_enable_store(struct gov_attr_set *attr_set,
				const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->vh_sched_move_task_enable, val);
	return count;
}

static struct governor_attr vh_sched_move_task_enable =
	__ATTR_RW(vh_sched_move_task_enable);

/* Patch B9-4: vh_scheduler_tick_enable sysfs knob.  Strict 0/1
 * boolean; gates the android_vh_scheduler_tick vendor-hook observer
 * (see ZENITH_DEFAULT_VH_SCHEDULER_TICK_ENABLE for full semantics
 * and profile bakes).  Stored via WRITE_ONCE; the probe reads with
 * READ_ONCE so a torn write would at worst delay the gate flip by
 * one tick (4 ms at HZ=250).
 */
static ssize_t
vh_scheduler_tick_enable_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->vh_scheduler_tick_enable);
}

static ssize_t
vh_scheduler_tick_enable_store(struct gov_attr_set *attr_set,
			       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	WRITE_ONCE(t->vh_scheduler_tick_enable, val);
	return count;
}

static struct governor_attr vh_scheduler_tick_enable =
	__ATTR_RW(vh_scheduler_tick_enable);

/* Patch B-AUTO-3: auto_eval_ms RW sysfs.  Cadence at which the
 * auto-selector engine runs its classifier when active_profile ==
 * ZENITH_PROFILE_AUTO.  Bounded by ZENITH_AUTO_EVAL_MS_{MIN,MAX}.
 * 0 is the runtime pause (worker still rearms but does not
 * classify or commit).  Reads / writes are READ_ONCE / WRITE_ONCE
 * because the worker reads this lock-free.
 */
static ssize_t auto_eval_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       READ_ONCE(to_zenith_tunables(attr_set)->auto_eval_ms));
}

static ssize_t auto_eval_ms_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val && val < ZENITH_AUTO_EVAL_MS_MIN)
		return -EINVAL;
	if (val > ZENITH_AUTO_EVAL_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->auto_eval_ms, val);
	return count;
}
static struct governor_attr auto_eval_ms = __ATTR_RW(auto_eval_ms);

/* Patch B-AUTO-3: auto_hysteresis_ms RW sysfs.  How long the
 * auto-selector classifier's chosen target must hold before
 * zenith commits the profile switch.  Bounded by
 * ZENITH_AUTO_HYSTERESIS_MS_{MIN,MAX}; 0 disables hysteresis (the
 * classifier's pick lands on the very next eval window).
 */
static ssize_t auto_hysteresis_ms_show(struct gov_attr_set *attr_set,
				       char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       READ_ONCE(to_zenith_tunables(attr_set)->auto_hysteresis_ms));
}

static ssize_t auto_hysteresis_ms_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > ZENITH_AUTO_HYSTERESIS_MS_MAX)
		return -EINVAL;
	WRITE_ONCE(t->auto_hysteresis_ms, val);
	return count;
}
static struct governor_attr auto_hysteresis_ms =
	__ATTR_RW(auto_hysteresis_ms);

/* camera_aware sysfs knob.  Strict 0/1 boolean; non-zero values
 * normalised to 1 on store.
 */
static ssize_t camera_aware_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
		       to_zenith_tunables(attr_set)->camera_aware);
}

static ssize_t camera_aware_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int old;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	old = t->camera_aware;
	t->camera_aware = val;
	zenith_set_static_key(&zenith_camera_aware_key, val);
	if (old != val)
		zenith_log_master_flip(t, "camera_aware", old, val);
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	zenith_at_mark_override(t, ZENITH_AT_OVERRIDE_CAMERA_FLOOR_PCT);
	zenith_invalidate_cache(attr_set);
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
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->game_mode);
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
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->game_auto);
}

static ssize_t game_auto_store(struct gov_attr_set *attr_set,
			       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int old;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	old = t->game_auto;
	t->game_auto = val;
	zenith_set_static_key(&zenith_game_auto_key, val);
	if (!val)
		WRITE_ONCE(zenith_game_auto_active_until_ns, 0);
	if (old != val)
		zenith_log_master_flip(t, "game_auto", old, val);
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
	return sysfs_emit(buf, "%u\n", zenith_game_auto_active() ? 1 : 0);
}
static struct governor_attr game_auto_state = __ATTR_RO(game_auto_state);

/* psi_aware sysfs knob.  Strict 0/1 boolean.  See ZENITH_DEFAULT_PSI_AWARE
 * comment block for semantics.  No cache invalidation -- the value is
 * consumed inline at zenith_get_next_freq() time.
 */
static ssize_t psi_aware_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n", to_zenith_tunables(attr_set)->psi_aware);
}

static ssize_t psi_aware_store(struct gov_attr_set *attr_set,
			       const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	unsigned int old;
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;
	old = t->psi_aware;
	t->psi_aware = val;
	zenith_set_static_key(&zenith_psi_aware_key, val);
	if (old != val)
		zenith_log_master_flip(t, "psi_aware", old, val);
	return count;
}
static struct governor_attr psi_aware = __ATTR_RW(psi_aware);

/* psi_mem_thresh sysfs knob.  Range 0..100 (integer percentage of
 * the PSI 10s some-stall average).  0 disables the cap even with
 * psi_aware=1, useful for tracing the helper without changing freq.
 */
static ssize_t psi_mem_thresh_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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

/* psi_cgroup_path sysfs knob (Patch B10-3).
 *
 * Read returns the per-tunables psi_cgroup_path string (the path
 * most recently stored on this attr_set, profile-baked or sysfs-
 * written).  Newline-terminated, like every other zenith string knob.
 *
 * Write copies the supplied string into t->psi_cgroup_path (after
 * stripping a trailing newline) and calls zenith_psi_cgroup_apply()
 * to refresh the file-scope cached cgroup pointer.  Empty string
 * (just "\n" or "") drops to NULL = system-wide PSI = pre-B10
 * behaviour.  -EINVAL on a too-long path; resolution failures
 * (cgroup-v2 not mounted, missing cgroup) are silently demoted to
 * the empty/system-wide path inside zenith_psi_cgroup_apply().
 */
static ssize_t psi_cgroup_path_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%s\n",
		       to_zenith_tunables(attr_set)->psi_cgroup_path);
}

static ssize_t psi_cgroup_path_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	struct zenith_tunables *t = to_zenith_tunables(attr_set);
	char path[ZENITH_PSI_CGROUP_PATH_MAX];
	ssize_t len;

	len = strscpy(path, buf, sizeof(path));
	if (len < 0)
		return -EINVAL;

	/* Strip trailing newline, if any. */
	if (len > 0 && path[len - 1] == '\n')
		path[len - 1] = '\0';

	strscpy(t->psi_cgroup_path, path, sizeof(t->psi_cgroup_path));
	zenith_psi_cgroup_apply(path);

	return count;
}
static struct governor_attr psi_cgroup_path = __ATTR_RW(psi_cgroup_path);

/* boot_boost_ms sysfs knob.  See ZENITH_DEFAULT_BOOT_BOOST_MS comment
 * block for semantics.  Range 0..ZENITH_BOOT_BOOST_MAX_MS;
 * out-of-range values rejected with EINVAL so userspace gets a clear
 * error rather than a silent clamp.  No cache invalidation: the value
 * is consumed inline by the eval path on every tick.
 */
static ssize_t boot_boost_ms_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n", atomic_read(&zenith_boot_complete));
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
ZENITH_TUNABLE_UINT_MAX(boot_complete_auto, 1);

/* frame_budget_us sysfs knob.  Range 0..ZENITH_FRAME_BUDGET_US_MAX
 * (50 ms).  Userspace writes the current vblank period in
 * microseconds whenever the panel changes refresh rate.  0 disables
 * the adaptive frame-budget floor outright.  See the
 * ZENITH_DEFAULT_FRAME_BUDGET_US comment block at the top of the file
 * for typical values per refresh rate.
 */
static ssize_t frame_budget_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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

	/* Iterate only over CPUs that actually exist on this machine.
	 * Slots in [nr_cpu_ids, NR_CPUS) are kept at zero by the store
	 * side (input is rejected with -EINVAL above nr_cpu_ids), so
	 * skipping them here is functionally equivalent and avoids a
	 * read of zero-initialized storage we know cannot be non-zero.
	 * sysfs_emit_at is the bounded form of sprintf for sysfs show
	 * handlers: identical formatted output, but it cannot overrun
	 * the PAGE_SIZE buffer the sysfs core hands us.
	 */
	for_each_possible_cpu(cpu) {
		unsigned int v = t->frame_budget_us_per_policy[cpu];

		if (!v)
			continue;
		len += sysfs_emit_at(buf, len, "%s%u:%u",
				     first ? "" : ",", cpu, v);
		first = false;
	}
	len += sysfs_emit_at(buf, len, "\n");
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
		/* Reject input that targets a CPU index above the number
		 * of CPUs that actually exist on this machine.  The array
		 * is still sized [NR_CPUS], but slots in [nr_cpu_ids,
		 * NR_CPUS) are dead storage -- accepting them here would
		 * silently mask a typo / misconfigured boot string.
		 */
		if (anchor >= nr_cpu_ids || val > ZENITH_FRAME_BUDGET_US_MAX)
			return -EINVAL;
		parsed[anchor] = val;
		p = comma ? comma + 1 : end;
	}

commit:
	/* Only write to slots for CPUs that actually exist.  Slots in
	 * [nr_cpu_ids, NR_CPUS) stay at zero (the kzalloc value) for
	 * the lifetime of the tunables, because the bounds check above
	 * never lets the store path write a non-zero value into them.
	 */
	for_each_possible_cpu(cpu)
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
	return sysfs_emit(buf, "%u\n",
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
	zenith_invalidate_cache(attr_set);
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
ZENITH_TUNABLE_UINT_MAX(uclamp_min_respect, 1);

/* peer_ramp_uclamp_min_respect sysfs knob (Patch M2).  Range
 * 0..1.  When set, the peer_ramp floor is computed as
 * max(peer_ramp_floor_pct, uclamp_min_pct).  When 0, the
 * floor uses peer_ramp_floor_pct verbatim (Stage 4 behaviour).
 * Independent of the master uclamp_min_respect knob.
 */
static ssize_t
peer_ramp_uclamp_min_respect_show(struct gov_attr_set *attr_set, char *buf)
{
	return sysfs_emit(buf, "%u\n",
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
	return sysfs_emit(buf, "%u\n",
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
ZENITH_TUNABLE_UINT_MAX(uclamp_max_respect, 1);

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
	&batt_hold_scale_pct.attr,
	&on_battery.attr,
	&charger_aware.attr,
	&charger_floor_pct.attr,
	&top_app_aware.attr,
	&top_app_floor_pct.attr,
	&render_thread_util_aware.attr,
	&render_thread_util_thresh.attr,
	&render_thread_util_floor_pct.attr,
	&pmu_aware.attr,
	&pmu_ipc_thresh.attr,
	&pmu_ipc_floor_pct.attr,
	&em_aware.attr,
	&em_floor_pct.attr,
	&cluster_wake_pulse_ms.attr,
	&cluster_wake_pulse_idle_ms.attr,
	&cluster_wake_pulse_floor_pct.attr,
	&quiet_hours_start_min.attr,
	&quiet_hours_end_min.attr,
	&quiet_hours_cap_pct.attr,
	&quiet_hours_screen_off_only.attr,
	&decision_latency_hist.attr,
	&fg_transition_pulse_ms.attr,
	&fg_transition_pulse_pct.attr,
	&peak_headroom_prearm.attr,
	&predict_up_thresh.attr,
	&predict_up_window.attr,
	&pelt_rising_edge_thresh.attr,
	&pelt_rising_edge_min_pct.attr,
	&dl_task_floor_pct.attr,
	&io_floor_hyst_ms.attr,
	&io_floor_hyst_pct.attr,
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
	&verbose_log.attr,
	&game_perf_burst.attr,
	&game_perf_burst_floor_pct.attr,
	&game_perf_burst_thermal_ceiling_dc.attr,
	&game_perf_burst_disarm_grace_ms.attr,
	&game_perf_burst_cooldown_ms.attr,
	&game_perf_burst_state.attr,
	&game_perf_burst_stats.attr,
	&auto_target.attr,
	&auto_eval_ms.attr,
	&auto_hysteresis_ms.attr,
	&profile_values.attr,
	&zenith_stats.attr,
	&zenith_stats_reset.attr,
	&zenith_input_stats.attr,
	&at_log.attr,
	&last_decision_path.attr,
	&decision_ring.attr,
	&decision_confidence.attr,
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
	&auto_tune_util_rising_thresh_pct.attr,
	&auto_tune_render_rt_floor_pct.attr,
	&auto_tune_v3.attr,
	&auto_tune_v3_interval_ms.attr,
	&auto_tune_v3_state.attr,
	&auto_tune_v3_calib_log.attr,
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
	&thermal_aware.attr,
	&thermal_active.attr,
	&thermal_pressure_continuous.attr,
	&prefer_silver_aware.attr,
	&prefer_silver_hot_threshold_pct.attr,
	&prefer_silver_hot_bump_pct.attr,
	&thermal_util_derate.attr,
	&thermal_derate_rate_pct.attr,
	&auto_thermal_cap.attr,
	&auto_thermal_cap_pressure_pct.attr,
	&auto_thermal_cap_freq_pct.attr,
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
	&audio_hyst_ms.attr,
	&vh_arch_freq_scale_enable.attr,
	&vh_uclamp_observer_enable.attr,
	&vh_cpu_idle_enable.attr,
	&vh_freq_qos_enable.attr,
	&vh_sched_move_task_enable.attr,
	&vh_scheduler_tick_enable.attr,
	&camera_aware.attr,
	&camera_comms.attr,
	&camera_active.attr,
	&camera_floor_pct.attr,
	&game_mode.attr,
	&psi_aware.attr,
	&psi_mem_thresh.attr,
	&psi_cpu_thresh.attr,
	&psi_io_thresh.attr,
	&psi_cgroup_path.attr,
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

void zenith_tunables_free(struct kobject *kobj)
{
	struct zenith_tunables *t =
		to_zenith_tunables(container_of(kobj, struct gov_attr_set,
						kobj));

	/* Patch B-AUTO-3: drain the auto-selector worker before
	 * freeing the tunables container.  The worker re-arms itself
	 * unconditionally while active_profile == AUTO so we must
	 * flip active_profile to a non-AUTO value first (the kobject
	 * is going away, so any value will do); cancel_delayed_work_-
	 * sync then returns once the in-flight invocation has
	 * observed the new value via the rearm-side READ_ONCE guard.
	 */
	WRITE_ONCE(t->active_profile, ZENITH_PROFILE_CUSTOM);
	cancel_delayed_work_sync(&t->eval_work);

	kfree(t);
}

struct kobj_type zenith_tunables_ktype = {
	.default_groups = zenith_groups,
	.sysfs_ops = &governor_sysfs_ops,
