/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM cpufreq_zenith

#if !defined(_TRACE_CPUFREQ_ZENITH_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_CPUFREQ_ZENITH_H

#include <linux/tracepoint.h>

/* Main decision trace: records the path taken through
 * zenith_get_next_freq() and the freq it resolved. Emitted only when
 * the event is enabled (gated by trace_zenith_decision_enabled()).
 *
 * path strings are compile-time literals owned by the kernel text,
 * so __string / __assign_str stores a cheap per-record copy without
 * dynamic allocation.
 */
TRACE_EVENT(zenith_decision,

	TP_PROTO(int cpu, const char *path, unsigned long util,
		 unsigned long max_cap, unsigned int load_pct,
		 unsigned int freq_in, unsigned int freq_out,
		 unsigned int kcpustat_pct),

	TP_ARGS(cpu, path, util, max_cap, load_pct, freq_in, freq_out,
		kcpustat_pct),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__string(path,		path)
		__field(unsigned long,	util)
		__field(unsigned long,	max_cap)
		__field(unsigned int,	load_pct)
		__field(unsigned int,	freq_in)
		__field(unsigned int,	freq_out)
		__field(unsigned int,	kcpustat_pct)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__assign_str(path,	path);
		__entry->util		= util;
		__entry->max_cap	= max_cap;
		__entry->load_pct	= load_pct;
		__entry->freq_in	= freq_in;
		__entry->freq_out	= freq_out;
		__entry->kcpustat_pct	= kcpustat_pct;
	),

	TP_printk("cpu=%d path=%s util=%lu max=%lu load=%u%% in=%u out=%u kcpustat=%u%%",
		  __entry->cpu, __get_str(path), __entry->util,
		  __entry->max_cap, __entry->load_pct,
		  __entry->freq_in, __entry->freq_out,
		  __entry->kcpustat_pct)
);

/* Auto-tune classifier decision emitted once per ZENITH_AUTO_TUNE_PERIOD_MS. */
TRACE_EVENT(zenith_auto_tune,

	TP_PROTO(int cpu, unsigned int sat_pct, unsigned int events_rate_x2,
		 unsigned int prev_profile, unsigned int new_profile),

	TP_ARGS(cpu, sat_pct, events_rate_x2, prev_profile, new_profile),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(unsigned int,	sat_pct)
		__field(unsigned int,	events_rate_x2)
		__field(unsigned int,	prev_profile)
		__field(unsigned int,	new_profile)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->sat_pct	= sat_pct;
		__entry->events_rate_x2	= events_rate_x2;
		__entry->prev_profile	= prev_profile;
		__entry->new_profile	= new_profile;
	),

	TP_printk("cpu=%d sat=%u%% events_x2=%u prev=%u new=%u",
		  __entry->cpu, __entry->sat_pct, __entry->events_rate_x2,
		  __entry->prev_profile, __entry->new_profile)
);

/* Scenario-aware auto_tune classifier override.  Emitted from
 * zenith_auto_tune_work() when auto_tune_scenario=1 and a detected
 * scenario (audio / camera / render / mem-stall) overrides the
 * load-saturation + input-rate classifier.  prev_target is what the
 * vanilla classifier would have picked; scenario_target is what the
 * scenario bias substitutes.  When the two are equal, the overlay
 * was a no-op for this window (still emitted for tracing parity).
 */
TRACE_EVENT(zenith_auto_tune_scenario,

	TP_PROTO(int cpu, bool audio, bool camera, bool render,
		 bool memstall, unsigned int prev_target,
		 unsigned int scenario_target),

	TP_ARGS(cpu, audio, camera, render, memstall, prev_target,
		scenario_target),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(bool,		audio)
		__field(bool,		camera)
		__field(bool,		render)
		__field(bool,		memstall)
		__field(unsigned int,	prev_target)
		__field(unsigned int,	scenario_target)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->audio		= audio;
		__entry->camera		= camera;
		__entry->render		= render;
		__entry->memstall	= memstall;
		__entry->prev_target	= prev_target;
		__entry->scenario_target = scenario_target;
	),

	TP_printk("cpu=%d audio=%d camera=%d render=%d memstall=%d prev=%u scenario=%u",
		  __entry->cpu, __entry->audio, __entry->camera,
		  __entry->render, __entry->memstall,
		  __entry->prev_target, __entry->scenario_target)
);

/* Predictive-util one-step-ahead extrapolation (predict_util_pct). One
 * record per zenith_get_util() call when the predictor is enabled and
 * the predicted value differs from the observed util. Useful for
 * sanity-checking that the predictor isn't over-reaching during
 * monotonic ramps.
 */
TRACE_EVENT(zenith_predict,

	TP_PROTO(int cpu, unsigned int pct,
		 unsigned long util_obs, unsigned long util_pred),

	TP_ARGS(cpu, pct, util_obs, util_pred),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(unsigned int,	pct)
		__field(unsigned long,	util_obs)
		__field(unsigned long,	util_pred)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->pct		= pct;
		__entry->util_obs	= util_obs;
		__entry->util_pred	= util_pred;
	),

	TP_printk("cpu=%d pct=%u util_obs=%lu util_pred=%lu",
		  __entry->cpu, __entry->pct,
		  __entry->util_obs, __entry->util_pred)
);

/* Render-thread / display-pipeline floor activation. Emitted at most
 * once every ZENITH_RENDER_CACHE_TTL_NS per policy when render_aware=1
 * and the comm walk decides whether a render thread is currently
 * running on any of the policy's CPUs.
 */
TRACE_EVENT(zenith_render_floor,

	TP_PROTO(int cpu, bool active, unsigned int floor_pct,
		 unsigned int floor_freq),

	TP_ARGS(cpu, active, floor_pct, floor_freq),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(bool,		active)
		__field(unsigned int,	floor_pct)
		__field(unsigned int,	floor_freq)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->active		= active;
		__entry->floor_pct	= floor_pct;
		__entry->floor_freq	= floor_freq;
	),

	TP_printk("cpu=%d active=%d floor_pct=%u floor_freq=%u",
		  __entry->cpu, __entry->active,
		  __entry->floor_pct, __entry->floor_freq)
);

/* Adaptive frame-budget floor evaluation. Fires once per
 * zenith_get_next_freq() call when frame_budget_us and
 * frame_pace_floor_pct are both non-zero, regardless of whether the
 * floor actually moved the freq.  Useful for sanity-checking that
 * userspace is keeping the budget in sync with the panel's vrefresh.
 */
TRACE_EVENT(zenith_frame_pace,

	TP_PROTO(int cpu, unsigned int budget_us,
		 unsigned int eff_pct, unsigned int floor_freq),

	TP_ARGS(cpu, budget_us, eff_pct, floor_freq),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(unsigned int,	budget_us)
		__field(unsigned int,	eff_pct)
		__field(unsigned int,	floor_freq)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->budget_us	= budget_us;
		__entry->eff_pct	= eff_pct;
		__entry->floor_freq	= floor_freq;
	),

	TP_printk("cpu=%d budget_us=%u eff_pct=%u floor_freq=%u",
		  __entry->cpu, __entry->budget_us,
		  __entry->eff_pct, __entry->floor_freq)
);

/* Audio low-jitter floor/cap activation. Emitted at most once every
 * ZENITH_AUDIO_CACHE_TTL_NS per policy when audio_aware=1 and the
 * comm walk decides whether an audio thread is currently running on
 * any of the policy's CPUs. floor_freq / cap_freq are the resolved
 * absolute frequencies (kHz); 0 means "tier disabled".
 */
TRACE_EVENT(zenith_audio_band,

	TP_PROTO(int cpu, bool active, unsigned int floor_pct,
		 unsigned int cap_pct, unsigned int floor_freq,
		 unsigned int cap_freq),

	TP_ARGS(cpu, active, floor_pct, cap_pct, floor_freq, cap_freq),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(bool,		active)
		__field(unsigned int,	floor_pct)
		__field(unsigned int,	cap_pct)
		__field(unsigned int,	floor_freq)
		__field(unsigned int,	cap_freq)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->active		= active;
		__entry->floor_pct	= floor_pct;
		__entry->cap_pct	= cap_pct;
		__entry->floor_freq	= floor_freq;
		__entry->cap_freq	= cap_freq;
	),

	TP_printk("cpu=%d active=%d floor_pct=%u cap_pct=%u floor_freq=%u cap_freq=%u",
		  __entry->cpu, __entry->active,
		  __entry->floor_pct, __entry->cap_pct,
		  __entry->floor_freq, __entry->cap_freq)
);

/* Camera capture-pipeline floor activation. Emitted at most once
 * every ZENITH_CAMERA_CACHE_TTL_NS per policy when camera_aware=1.
 * 'active' is the resolved decision after applying the userspace
 * override (camera_active=auto/force-on/force-off); 'auto_match' is
 * the raw comm-walk result before override.
 */
TRACE_EVENT(zenith_camera_floor,

	TP_PROTO(int cpu, bool active, bool auto_match,
		 unsigned int override, unsigned int floor_pct,
		 unsigned int floor_freq),

	TP_ARGS(cpu, active, auto_match, override, floor_pct, floor_freq),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__field(bool,		active)
		__field(bool,		auto_match)
		__field(unsigned int,	override)
		__field(unsigned int,	floor_pct)
		__field(unsigned int,	floor_freq)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__entry->active		= active;
		__entry->auto_match	= auto_match;
		__entry->override	= override;
		__entry->floor_pct	= floor_pct;
		__entry->floor_freq	= floor_freq;
	),

	TP_printk("cpu=%d active=%d auto_match=%d override=%u floor_pct=%u floor_freq=%u",
		  __entry->cpu, __entry->active, __entry->auto_match,
		  __entry->override, __entry->floor_pct, __entry->floor_freq)
);

/* game_mode flip. Emitted whenever userspace writes a new value to
 * the game_mode sysfs node; lets you correlate frame-pacing diffs in
 * trace data with the moment the gameswitch helper armed/disarmed.
 */
TRACE_EVENT(zenith_game_mode,

	TP_PROTO(int cpu, bool active),

	TP_ARGS(cpu, active),

	TP_STRUCT__entry(
		__field(int,	cpu)
		__field(bool,	active)
	),

	TP_fast_assign(
		__entry->cpu	= cpu;
		__entry->active	= active;
	),

	TP_printk("cpu=%d active=%d", __entry->cpu, __entry->active)
);

#endif /* _TRACE_CPUFREQ_ZENITH_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
