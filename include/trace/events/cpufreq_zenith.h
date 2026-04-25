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
		 unsigned int freq_in, unsigned int freq_out),

	TP_ARGS(cpu, path, util, max_cap, load_pct, freq_in, freq_out),

	TP_STRUCT__entry(
		__field(int,		cpu)
		__string(path,		path)
		__field(unsigned long,	util)
		__field(unsigned long,	max_cap)
		__field(unsigned int,	load_pct)
		__field(unsigned int,	freq_in)
		__field(unsigned int,	freq_out)
	),

	TP_fast_assign(
		__entry->cpu		= cpu;
		__assign_str(path,	path);
		__entry->util		= util;
		__entry->max_cap	= max_cap;
		__entry->load_pct	= load_pct;
		__entry->freq_in	= freq_in;
		__entry->freq_out	= freq_out;
	),

	TP_printk("cpu=%d path=%s util=%lu max=%lu load=%u%% in=%u out=%u",
		  __entry->cpu, __get_str(path), __entry->util,
		  __entry->max_cap, __entry->load_pct,
		  __entry->freq_in, __entry->freq_out)
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

#endif /* _TRACE_CPUFREQ_ZENITH_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
