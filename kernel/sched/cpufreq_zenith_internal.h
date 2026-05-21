/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Zenith CPUFreq Governor — internal header
 *
 * Shared structs, enums, constants and extern declarations used by the
 * main governor module and its split-out subsystems (sysfs, auto-tune).
 *
 * Do not include outside kernel/sched/cpufreq_zenith*.c.
 */
#ifndef _CPUFREQ_ZENITH_INTERNAL_H
#define _CPUFREQ_ZENITH_INTERNAL_H

#include <linux/cpufreq.h>
#include <linux/jump_label.h>
#include <linux/atomic.h>
#include <linux/types.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/slab.h>

/* Trace event headers (declare-only: no CREATE_TRACE_POINTS) */
#include <trace/events/power.h>
#include <trace/events/sched.h>
#include <trace/events/cpufreq_zenith.h>

/* ─────────────────────────────────────────────
 * Forward declarations (opaque pointers)
 * ───────────────────────────────────────────── */
struct gov_attr_set;

/* ─────────────────────────────────────────────
 * Constants & Defaults
 * ───────────────────────────────────────────── */

/* Permille of SCHED_CAPACITY_SCALE at which iowait boost starts. */
#define ZENITH_DEFAULT_IOWAIT_BOOST_MIN		125
#define ZENITH_DEFAULT_IOWAIT_STACK_PCT		50

#define ZENITH_DEFAULT_IOWAIT_BACKOFF_AFTER_MS	0
#define ZENITH_DEFAULT_UP_THRESHOLD		75
#define ZENITH_DEFAULT_UP_THRESHOLD_HISPEED	0
#define ZENITH_DEFAULT_DOWN_THRESHOLD		60
#define ZENITH_DEFAULT_HISPEED_FREQ		0
#define ZENITH_HISPEED_FREQ_MAX			50000000U
#define ZENITH_DEFAULT_HISPEED_FREQ_PCT		55
#define ZENITH_DEFAULT_HISPEED_LOAD		65
#define ZENITH_DEFAULT_HISPEED_HYST_PCT		10
#define ZENITH_DEFAULT_HISPEED_ENTRY_STREAK	0
#define ZENITH_HISPEED_ENTRY_STREAK_MAX		16
#define ZENITH_DEFAULT_BRUTAL_ENTRY_STREAK	0
#define ZENITH_BRUTAL_ENTRY_STREAK_MAX		16
#define ZENITH_DEFAULT_PEAK_HEADROOM_RESCUE	1
#define ZENITH_DEFAULT_PEAK_HEADROOM_STARVE_LOAD_PCT	90
#define ZENITH_DEFAULT_PEAK_HEADROOM_FREQ_FLOOR_PCT	85
#define ZENITH_DEFAULT_PEAK_HEADROOM_STARVE_STREAK	3
#define ZENITH_DEFAULT_PEAK_HEADROOM_JUMP_PCT		100
#define ZENITH_DEFAULT_PEAK_HEADROOM_HOLD_MS		50
#define ZENITH_DEFAULT_PEAK_HEADROOM_PREARM		1
#define ZENITH_PEAK_HEADROOM_STREAK_MAX			16
#define ZENITH_PEAK_HEADROOM_HOLD_MS_MAX		1000
#define ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_MS		40
#define ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_IDLE_MS	80
#define ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_FLOOR_PCT	55
#define ZENITH_CLUSTER_WAKE_PULSE_MS_MAX		200
#define ZENITH_CLUSTER_WAKE_PULSE_IDLE_MS_MAX		1000
#define ZENITH_DEFAULT_BATT_HOLD_SCALE_PCT		100
#define ZENITH_BATT_HOLD_SCALE_PCT_MIN			50
#define ZENITH_BATT_HOLD_SCALE_PCT_MAX			300
#define ZENITH_DEFAULT_CHARGER_AWARE			0
#define ZENITH_DEFAULT_CHARGER_FLOOR_PCT		0
#define ZENITH_CHARGER_FLOOR_PCT_MAX			100
#define ZENITH_DEFAULT_TOP_APP_AWARE			0
#define ZENITH_DEFAULT_TOP_APP_FLOOR_PCT		0
#define ZENITH_TOP_APP_FLOOR_PCT_MAX			100
#define ZENITH_TOP_APP_CACHE_TTL_NS			(4 * NSEC_PER_MSEC)
#define ZENITH_TOP_APP_CGROUP_NAME			"top-app"
#define ZENITH_DEFAULT_RENDER_THREAD_UTIL_AWARE		0
#define ZENITH_DEFAULT_RENDER_THREAD_UTIL_THRESH	0
#define ZENITH_DEFAULT_RENDER_THREAD_UTIL_FLOOR_PCT	0
#define ZENITH_RENDER_THREAD_UTIL_THRESH_MAX		1024
#define ZENITH_RENDER_THREAD_UTIL_FLOOR_PCT_MAX		100
#define ZENITH_DEFAULT_PMU_AWARE			0
#define ZENITH_DEFAULT_PMU_IPC_THRESH			100
#define ZENITH_DEFAULT_PMU_IPC_FLOOR_PCT		0
#define ZENITH_PMU_IPC_THRESH_MAX			1000
#define ZENITH_PMU_IPC_FLOOR_PCT_MAX			100
#define ZENITH_DEFAULT_EM_AWARE				0
#define ZENITH_DEFAULT_EM_FLOOR_PCT			0
#define ZENITH_EM_FLOOR_PCT_MAX				200
#define ZENITH_DEFAULT_QUIET_HOURS_START_MIN		0
#define ZENITH_DEFAULT_QUIET_HOURS_END_MIN		0
#define ZENITH_DEFAULT_QUIET_HOURS_CAP_PCT		100
#define ZENITH_DEFAULT_QUIET_HOURS_SCREEN_OFF_ONLY	1
#define ZENITH_DEFAULT_FG_TRANSITION_PULSE_MS		30
#define ZENITH_DEFAULT_FG_TRANSITION_PULSE_PCT		65
#define ZENITH_FG_TRANSITION_PULSE_MS_MAX		200
#define ZENITH_FG_TRANSITION_PULSE_PCT_MAX		100
#define ZENITH_QUIET_HOURS_MINUTE_MAX			1439
#define ZENITH_QUIET_HOURS_CAP_PCT_MIN			50
#define ZENITH_DEFAULT_PREDICT_UP_THRESH		64
#define ZENITH_DEFAULT_PREDICT_UP_WINDOW		4
#define ZENITH_PREDICT_UP_THRESH_MAX			255
#define ZENITH_PREDICT_UP_WINDOW_MIN			2
#define ZENITH_PREDICT_UP_WINDOW_MAX			8
#define ZENITH_DEFAULT_PELT_RISING_EDGE_THRESH		32
#define ZENITH_DEFAULT_PELT_RISING_EDGE_MIN_PCT		50
#define ZENITH_PELT_RISING_EDGE_THRESH_MAX		255
#define ZENITH_PELT_RISING_EDGE_MIN_PCT_MAX		100
#define ZENITH_DEFAULT_DL_TASK_FLOOR_PCT		0
#define ZENITH_DL_TASK_FLOOR_PCT_MAX			100
#define ZENITH_DEFAULT_IO_FLOOR_HYST_MS			0
#define ZENITH_DEFAULT_IO_FLOOR_HYST_PCT		50
#define ZENITH_IO_FLOOR_HYST_MS_MAX			2000
#define ZENITH_IO_FLOOR_HYST_PCT_MAX			100
#define ZENITH_DEFAULT_VH_ARCH_FREQ_SCALE_ENABLE	0
#define ZENITH_VH_ARCH_FREQ_SCALE_STEP			51
#define ZENITH_DEFAULT_VH_UCLAMP_OBSERVER_ENABLE	0
#define ZENITH_DEFAULT_VH_CPU_IDLE_ENABLE		1
#define ZENITH_VH_CPU_IDLE_RESIDENCY_LONG_NS		(4ULL * NSEC_PER_MSEC)
#define ZENITH_DEFAULT_VH_FREQ_QOS_ENABLE		0
#define ZENITH_VH_FREQ_QOS_MIN_PCT			75
#define ZENITH_VH_FREQ_QOS_WINDOW_MS			2000
#define ZENITH_DEFAULT_VH_SCHED_MOVE_TASK_ENABLE	0
#define ZENITH_DEFAULT_VH_SCHEDULER_TICK_ENABLE		0
#define ZENITH_DEFAULT_AUTO_EVAL_MS			500
#define ZENITH_DEFAULT_AUTO_HYSTERESIS_MS		2000
#define ZENITH_AUTO_EVAL_MS_MIN				100
#define ZENITH_AUTO_EVAL_MS_MAX				60000
#define ZENITH_AUTO_HYSTERESIS_MS_MIN			0
#define ZENITH_AUTO_HYSTERESIS_MS_MAX			60000
#define ZENITH_DEFAULT_PEAK_HYSTERESIS_STREAK		3
#define ZENITH_DEFAULT_PEAK_STEP_DOWN_PCT		95
#define ZENITH_PEAK_HYSTERESIS_STREAK_MAX		16
#define ZENITH_PEAK_HYSTERESIS_PEAK_THRESH_PCT		90
#define ZENITH_DEFAULT_BOOST_IDLE_THRESH		15
#define ZENITH_DEFAULT_BOOST_IDLE_STREAK		3
#define ZENITH_BOOST_IDLE_STREAK_MAX			16
#define ZENITH_DEFAULT_BG_UTIL_SCALE_PCT		100
#define ZENITH_BG_UTIL_SCALE_PCT_MIN			1
#define ZENITH_DEFAULT_SLEEPER_TAIL_THRESH_US		0
#define ZENITH_SLEEPER_TAIL_THRESH_US_MAX		100000
#define ZENITH_DEFAULT_SLEEPER_TAIL_PCT			90
#define ZENITH_SLEEPER_TAIL_PCT_MIN			50
#define ZENITH_SLEEPER_TAIL_PCT_MAX			100
#define ZENITH_DEFAULT_PEER_RAMP_WINDOW_MS		25
#define ZENITH_PEER_RAMP_WINDOW_MS_MAX			100
#define ZENITH_DEFAULT_PEER_RAMP_FLOOR_PCT		60
#define ZENITH_PEER_RAMP_FLOOR_PCT_MAX			100
#define ZENITH_DEFAULT_PEER_RAMP_WINDOW_OFF_MS		0
#define ZENITH_DEFAULT_MIGRATION_JUMP_PCT		20
#define ZENITH_MIGRATION_JUMP_PCT_MAX			100
#define ZENITH_DEFAULT_MIGRATION_FLOOR_WINDOW_MS	30
#define ZENITH_MIGRATION_FLOOR_WINDOW_MS_MAX		100
#define ZENITH_DEFAULT_MIGRATION_FLOOR_PCT		60
#define ZENITH_MIGRATION_FLOOR_PCT_MAX			100
#define ZENITH_DEFAULT_PSI_CPU_FLOOR_THRESH		0
#define ZENITH_PSI_CPU_FLOOR_THRESH_MAX			100
#define ZENITH_DEFAULT_FRAME_OVERRUN_SLACK_US		0
#define ZENITH_FRAME_OVERRUN_SLACK_US_MAX		16667
#define ZENITH_DEFAULT_FRAME_OVERRUN_WINDOW_MS		50
#define ZENITH_FRAME_OVERRUN_WINDOW_MS_MAX		200
#define ZENITH_DEFAULT_FRAME_OVERRUN_FLOOR_PCT		80
#define ZENITH_FRAME_OVERRUN_FLOOR_PCT_MAX		100
#define ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_STREAK	0
#define ZENITH_FRAME_OVERRUN_DEEP_STREAK_MAX		16
#define ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_FLOOR_PCT	100
#define ZENITH_FRAME_OVERRUN_DEEP_FLOOR_PCT_MAX		100
#define ZENITH_DEFAULT_PEER_RAMP_UCLAMP_MIN_RESPECT	1
#define ZENITH_DEFAULT_MIGRATION_FLOOR_UCLAMP_MIN_RESPECT 1
#define ZENITH_DEFAULT_PSI_MEM_CAP_THRESH		0
#define ZENITH_PSI_MEM_CAP_THRESH_MAX			100
#define ZENITH_DEFAULT_PSI_MEM_CAP_PCT			80
#define ZENITH_PSI_MEM_CAP_PCT_MIN			50
#define ZENITH_PSI_MEM_CAP_PCT_MAX			100
#define ZENITH_DEFAULT_PSI_MEM_CAP_WINDOW_MS		1000
#define ZENITH_PSI_MEM_CAP_WINDOW_MS_MIN		100
#define ZENITH_PSI_MEM_CAP_WINDOW_MS_MAX		5000
#define ZENITH_DEFAULT_UP_THRESHOLD_ADAPTIVE		0
#define ZENITH_UP_THRESHOLD_ADAPTIVE_MAX		30
#define ZENITH_UCLAMP_CACHE_TTL_NS			(1 * NSEC_PER_MSEC)
#define ZENITH_EFF_BINS_MAX				8
#define ZENITH_AT_LOG_NR				16
#define ZENITH_AT_HISTORY_NR				32
#define ZENITH_DEC_RING_NR				16
#define ZENITH_CLIMB_MODE_SNAP				0
#define ZENITH_CLIMB_MODE_STEP				1
#define ZENITH_PROFILE_CUSTOM				0
#define ZENITH_PROFILE_PERFORMANCE			1
#define ZENITH_PROFILE_BALANCED				2
#define ZENITH_PROFILE_BATTERY				3
#define ZENITH_PROFILE_LEGACY				4
#define ZENITH_PROFILE_GAMING				5
#define ZENITH_PROFILE_AUDIO				6
#define ZENITH_PROFILE_AUTO				7
#define ZENITH_DEFAULT_CLIMB_MODE			ZENITH_CLIMB_MODE_SNAP
#define ZENITH_DEFAULT_FREQ_STEP_PCT			5
#define ZENITH_DEFAULT_FREQ_STEP_ADAPTIVE		0
#define ZENITH_DEFAULT_THERMAL_AUTO			1
#define ZENITH_THERMAL_AUTO_PRESSURE_PCT		10
#define ZENITH_DEFAULT_THERMAL_AWARE			1
#define ZENITH_DEFAULT_THERMAL_PRESSURE_CONTINUOUS	1
#define ZENITH_DEFAULT_PREFER_SILVER_AWARE		1
#define ZENITH_DEFAULT_PREFER_SILVER_HOT_THRESHOLD_PCT	50
#define ZENITH_DEFAULT_PREFER_SILVER_HOT_BUMP_PCT	5
#define ZENITH_PREFER_SILVER_HOT_BUMP_MAX_PCT		20
#define ZENITH_DEFAULT_BRUTAL_DECAY_MS			0
#define ZENITH_BRUTAL_DECAY_MS_MAX			500
#define ZENITH_DEFAULT_THERMAL_UTIL_DERATE		1
#define ZENITH_THERMAL_DERATE_FLOOR_PCT			5
#define ZENITH_DEFAULT_THERMAL_DERATE_RATE_PCT		25
#define ZENITH_DEFAULT_AUTO_THERMAL_CAP			0
#define ZENITH_DEFAULT_AUTO_THERMAL_CAP_PRESSURE_PCT	50
#define ZENITH_AUTO_THERMAL_CAP_PRESSURE_PCT_MIN	1
#define ZENITH_AUTO_THERMAL_CAP_PRESSURE_PCT_MAX	100
#define ZENITH_DEFAULT_AUTO_THERMAL_CAP_FREQ_PCT	80
#define ZENITH_AUTO_THERMAL_CAP_FREQ_PCT_MIN		50
#define ZENITH_AUTO_THERMAL_CAP_FREQ_PCT_MAX		100
#define ZENITH_DEFAULT_FREQ_STABILITY_MARGIN_PCT	3
#define ZENITH_FREQ_STABILITY_MARGIN_PCT_MAX		10
#define ZENITH_DEFAULT_DOWN_RATE_ADAPTIVE		1
#define ZENITH_DEFAULT_WAKEUP_BOOST			1
#define ZENITH_WAKEUP_IDLE_THRESH_PCT			10
#define ZENITH_WAKEUP_BUSY_THRESH_PCT			40
#define ZENITH_WAKEUP_BOOST_TICKS			2
#define ZENITH_DEFAULT_WAKEUP_BOOST_MS			0
#define ZENITH_WAKEUP_BOOST_MS_MAX			200
#define ZENITH_DEFAULT_DOWN_THRESHOLD_ADAPTIVE		0
#define ZENITH_DOWN_THRESHOLD_ADAPTIVE_MAX		20
#define ZENITH_DEFAULT_RATE_LIMIT_CLUSTER_SCALE		1
#define ZENITH_CLUSTER_LITTLE_THRESH_PCT		60
#define ZENITH_DEFAULT_UP_RATE_LIMIT_US			100
#define ZENITH_DEFAULT_DOWN_RATE_LIMIT_US		4000
#define ZENITH_RATE_LIMIT_US_MAX			60000000U
#define ZENITH_DEFAULT_INPUT_BOOST_MS			48
#define ZENITH_DEFAULT_INPUT_BOOST_DOWN_RATE_MULT	200
#define ZENITH_INPUT_BOOST_DOWN_RATE_MULT_MIN		100
#define ZENITH_INPUT_BOOST_DOWN_RATE_MULT_MAX		1000
#define ZENITH_DEFAULT_INPUT_BOOST_TOUCHDOWN_EXTRA_MS	0
#define ZENITH_INPUT_BOOST_EXTRA_MS_MAX			200
#define ZENITH_DEFAULT_SAMPLE_RATE_MS			16
#define ZENITH_DEFAULT_MAX_SAMPLE_RATE_MS		500
#define ZENITH_DEFAULT_INPUT_EVAL_MS			500
#define ZENITH_INPUT_EVAL_MS_MIN			100
#define ZENITH_INPUT_EVAL_MS_MAX			60000
#define ZENITH_DEFAULT_GP_AWARE				true
#define ZENITH_DEFAULT_GP_CORE_PCT			60
#define ZENITH_DEFAULT_GP_FREQ_PCT			70
#define ZENITH_GP_FREQ_PCT_MIN				10
#define ZENITH_DEFAULT_FRAME_PACE_FLOOR_PCT		25
#define ZENITH_FRAME_PACE_FLOOR_PCT_MAX			100
#define ZENITH_DEFAULT_FRAME_CTL_PCT			100
#define ZENITH_DEFAULT_FRAME_CTL_DELTA_MS		1000
#define ZENITH_DEFAULT_GP_EVAL_THRESH			1
#define ZENITH_DEFAULT_GP_EVAL_PERIOD_MS		100
#define ZENITH_DEFAULT_GP_BOOST_PCT			30
#define ZENITH_GP_BOOST_MIN_PCT			10
#define ZENITH_AUTO_TUNE_PERIOD_MS			10000
#define ZENITH_DEFAULT_AUTO_TUNE			1
#define ZENITH_DEFAULT_AUTO_TUNE_V3			2
#define ZENITH_AUTO_TUNE_V3_CALIB_PERIOD_MS		120000
#define ZENITH_AT_V3_CALIB_COOLDOWN_MS			30000
#define ZENITH_AT_V3_CALIB_LOG_NR			8
#define ZENITH_AT_V3_CALIB_RESET_MS			10000
#define ZENITH_AT_V3_HYST_OFFSET_MIN			-3
#define ZENITH_AT_V3_HYST_OFFSET_MAX			4
#define ZENITH_AT_V3_COOL_OFFSET_MIN			-3
#define ZENITH_AT_V3_COOL_OFFSET_MAX			4
#define ZENITH_COMM_LIST_MAX				128
#define ZENITH_COMM_BUF_MAX				2048
#define ZENITH_DEFAULT_GAME_AUTO			1
#define ZENITH_DEFAULT_CAMERA_AWARE			1
#define ZENITH_DEFAULT_AUDIO_AWARE			1
#define ZENITH_DEFAULT_RENDER_AWARE			1
#define ZENITH_DEFAULT_PSI_AWARE			1
#define ZENITH_DEFAULT_AUDIO_PULSE_FLOOR_PCT		0
#define ZENITH_DEFAULT_AUDIO_FLOOR_PCT			30
#define ZENITH_DEFAULT_CAMERA_FLOOR_PCT			0
#define ZENITH_DEFAULT_RENDER_FLOOR_PCT			20
#define ZENITH_DEFAULT_DECISION_RING_NR			16
#define ZENITH_DEFAULT_QUIET_EXTEND_MS			0
#define ZENITH_DEFAULT_INPUT_FLOOR_PCT			90
#define ZENITH_DEFAULT_BOOT_BOOST_DECAY_MS		0
#define ZENITH_DEFAULT_BOOT_BOOST_MS			0
#define ZENITH_BOOT_BOOST_MS_MAX			10000
#define ZENITH_DEFAULT_BOOT_BOOST_CAP_PCT		0
#define ZENITH_DEFAULT_LIGHT_CAP_PCT			75
#define ZENITH_LIGHT_CAP_PCT_MIN			0
#define ZENITH_DEFAULT_SCREEN_OFF_GLIDE_MS		0
#define ZENITH_SCREEN_OFF_GLIDE_MS_MAX			500
#define ZENITH_DEFAULT_MIN_SAMPLE_RATE_MS		1
#define ZENITH_DEFAULT_PEAK_PREARM_MS			0
#define ZENITH_DEFAULT_PEAK_PREARM_JUMP_PCT		0
#define ZENITH_DEFAULT_PEAK_PREARM_IDLE_PCT		90

/* ─────────────────────────────────────────────
 * Profile name table (shared by sysfs and core)
 * ───────────────────────────────────────────── */
extern const char *zenith_profile_name[];
extern const unsigned int zenith_profile_defaults[][2];

/* ─────────────────────────────────────────────
 * Enums
 * ───────────────────────────────────────────── */

enum zenith_stat_idx {
	ZENITH_STAT_DECISIONS,
	ZENITH_STAT_CACHE_HITS,
	ZENITH_STAT_INPUT_BOOST,
	ZENITH_STAT_BRUTAL,
	ZENITH_STAT_HISPEED,
	ZENITH_STAT_FRAME_PACE,
	ZENITH_STAT_AUDIO,
	ZENITH_STAT_RENDER_CAMERA,
	ZENITH_STAT_UCLAMP,
	ZENITH_STAT_PSI,
	ZENITH_STAT_BOOT_BOOST,
	ZENITH_STAT_LIGHT_CAP,
	ZENITH_STAT_EM_CAP,
	ZENITH_STAT_EAS,
	ZENITH_STAT_OTHER,
	ZENITH_STAT_PREDICT_UP,
	ZENITH_STAT_PEAK_PREARM,
	ZENITH_STAT_PEAK_RESCUE,
	ZENITH_STAT_PEAK_HYST,
	ZENITH_STAT_PEER_RAMP,
	ZENITH_STAT_MIGRATION_FLOOR,
	ZENITH_STAT_PSI_CPU_FLOOR,
	ZENITH_STAT_FRAME_OVERRUN,
	ZENITH_STAT_AUTO_THERMAL_CAP,
	ZENITH_STAT_QUIET_HOURS_CAP,
	ZENITH_STAT_NR
};

/* ─────────────────────────────────────────────
 * Core structs
 * ───────────────────────────────────────────── */

struct zenith_tunables {
	struct gov_attr_set	attr_set;
	unsigned int		up_rate_limit_us;
	unsigned int		down_rate_limit_us;
	unsigned int		up_threshold;
	unsigned int		up_threshold_adaptive;
	unsigned int		down_threshold;
	unsigned int		hispeed_freq;
	unsigned int		hispeed_freq_pct;
	unsigned int		hispeed_load;
	unsigned int		hispeed_hyst_pct;
	unsigned int		hispeed_entry_streak;
	unsigned int		brutal_entry_streak;
	unsigned int		peak_headroom_rescue;
	unsigned int		peak_headroom_starve_load_pct;
	unsigned int		peak_headroom_freq_floor_pct;
	unsigned int		peak_headroom_starve_streak;
	unsigned int		peak_headroom_jump_pct;
	unsigned int		peak_headroom_hold_ms;
	unsigned int		peak_headroom_prearm;
	unsigned int		peak_prearm_ms;
	unsigned int		peak_prearm_jump_pct;
	unsigned int		peak_prearm_idle_pct;
	unsigned int		batt_hold_scale_pct;
	unsigned int		charger_aware;
	unsigned int		charger_floor_pct;
	unsigned int		top_app_aware;
	unsigned int		top_app_floor_pct;
	unsigned int		render_thread_util_aware;
	unsigned int		render_thread_util_thresh;
	unsigned int		render_thread_util_floor_pct;
	unsigned int		pmu_aware;
	unsigned int		pmu_ipc_thresh;
	unsigned int		pmu_ipc_floor_pct;
	unsigned int		em_aware;
	unsigned int		em_floor_pct;
	unsigned int		cluster_wake_pulse_ms;
	unsigned int		cluster_wake_pulse_idle_ms;
	unsigned int		cluster_wake_pulse_floor_pct;
	unsigned int		quiet_hours_start_min;
	unsigned int		quiet_hours_end_min;
	unsigned int		quiet_hours_cap_pct;
	unsigned int		quiet_hours_screen_off_only;
	unsigned int		fg_transition_pulse_ms;
	unsigned int		fg_transition_pulse_pct;
	unsigned int		predict_up_thresh;
	unsigned int		predict_up_window;
	unsigned int		pelt_rising_edge_thresh;
	unsigned int		pelt_rising_edge_min_pct;
	unsigned int		dl_task_floor_pct;
	unsigned int		io_floor_hyst_ms;
	unsigned int		io_floor_hyst_pct;
	unsigned int		vh_arch_freq_scale_enable;
	unsigned int		vh_uclamp_observer_enable;
	unsigned int		vh_cpu_idle_enable;
	unsigned int		vh_freq_qos_enable;
	unsigned int		vh_sched_move_task_enable;
	unsigned int		vh_scheduler_tick_enable;
	unsigned int		auto_eval_ms;
	unsigned int		auto_hysteresis_ms;
	unsigned int		peak_hysteresis_streak;
	unsigned int		peak_step_down_pct;
	unsigned int		boost_idle_thresh;
	unsigned int		boost_idle_streak;
	unsigned int		bg_util_scale_pct;
	unsigned int		sleeper_tail_thresh_us;
	unsigned int		sleeper_tail_pct;
	unsigned int		peer_ramp_window_ms;
	unsigned int		peer_ramp_floor_pct;
	unsigned int		peer_ramp_window_off_ms;
	unsigned int		migration_jump_pct;
	unsigned int		migration_floor_window_ms;
	unsigned int		migration_floor_pct;
	unsigned int		psi_cpu_floor_thresh;
	unsigned int		frame_overrun_slack_us;
	unsigned int		frame_overrun_window_ms;
	unsigned int		frame_overrun_floor_pct;
	unsigned int		frame_overrun_deep_streak;
	unsigned int		frame_overrun_deep_floor_pct;
	unsigned int		peer_ramp_uclamp_min_respect;
	unsigned int		migration_floor_uclamp_min_respect;
	unsigned int		psi_mem_cap_thresh;
	unsigned int		psi_mem_cap_pct;
	unsigned int		psi_mem_cap_window_ms;
	unsigned int		up_threshold_adaptive;
	unsigned int		climb_mode;
	unsigned int		freq_step_pct;
	unsigned int		freq_step_adaptive;
	unsigned int		thermal_auto;
	unsigned int		thermal_aware;
	unsigned int		thermal_pressure_continuous;
	unsigned int		thermal_util_derate;
	unsigned int		thermal_derate_rate_pct;
	unsigned int		auto_thermal_cap;
	unsigned int		auto_thermal_cap_pressure_pct;
	unsigned int		auto_thermal_cap_freq_pct;
	unsigned int		freq_stability_margin_pct;
	unsigned int		down_rate_adaptive;
	unsigned int		wakeup_boost;
	unsigned int		wakeup_boost_ms;
	unsigned int		down_threshold_adaptive;
	unsigned int		rate_limit_cluster_scale;
	unsigned int		input_boost_ms;
	unsigned int		input_boost_down_rate_mult;
	unsigned int		input_boost_touchdown_extra_ms;
	unsigned int		input_boost_floor_pct;
	unsigned int		light_cap_pct;
	unsigned int		min_sample_rate_ms;
	unsigned int		max_sample_rate_ms;
	unsigned int		input_eval_ms;
	unsigned int		sample_rate_ms;
	unsigned int		gp_aware;
	unsigned int		gp_core_pct;
	unsigned int		gp_freq_pct;
	unsigned int		frame_pace_floor_pct;
	unsigned int		frame_ctl_pct;
	unsigned int		frame_ctl_delta_ms;
	unsigned int		gp_eval_thresh;
	unsigned int		gp_eval_period_ms;
	unsigned int		gp_boost_pct;
	unsigned int		auto_tune;
	unsigned int		auto_tune_hysteresis_windows;
	unsigned int		auto_tune_v3;
	unsigned int		game_auto;
	unsigned int		game_auto_comms;
	unsigned int		camera_aware;
	unsigned int		camera_comms;
	unsigned int		audio_aware;
	unsigned int		audio_comms;
	unsigned int		render_aware;
	unsigned int		psi_aware;
	unsigned int		audio_pulse_floor_pct;
	unsigned int		audio_floor_pct;
	unsigned int		camera_floor_pct;
	unsigned int		render_floor_pct;
	unsigned int		decision_ring_nr;
	unsigned int		quiet_extend_ms;
	unsigned int		boot_boost_decay_ms;
	unsigned int		boot_boost_ms;
	unsigned int		boot_boost_cap_pct;
	unsigned int		screen_off_glide_ms;
	unsigned int		screen_off_glide_ms_legacy;
	unsigned int		prefer_silver_aware;
	unsigned int		prefer_silver_hot_threshold_pct;
	unsigned int		prefer_silver_hot_bump_pct;
	unsigned int		brutal_decay_ms;
	unsigned int		brutal_decay_enabled;
	unsigned int		iowait_boost_min;
	unsigned int		iowait_stack_pct;
	unsigned int		iowait_backoff_after_ms;
	unsigned int		hispeed_freq_pct_override;
	unsigned int		screen_state;
	unsigned int		active_profile;
	unsigned int		auto_target;
	unsigned int		game_perf_burst;
	unsigned int		game_perf_burst_floor_pct;
	unsigned int		game_perf_burst_thermal_ceiling_dc;
	unsigned int		game_perf_burst_disarm_grace_ms;
	unsigned int		game_perf_burst_cooldown_ms;
};

struct zenith_dec_ring_entry {
	const char	*path;
	u32		lat_ns;
};

struct zenith_at_log_entry {
	u64		ts_ns;
	unsigned int	total;
	unsigned int	saturated;
	unsigned int	sat_pct;
	unsigned int	events_rate_x2;
	unsigned int	target;
	unsigned int	state;
	unsigned int	reason;
	unsigned int	flags;
	unsigned int	var_x256;
	unsigned int	psi_cpu;
	unsigned int	psi_io;
	unsigned int	psi_mem;
	unsigned int	thermal_pressure;
	unsigned int	thermal_slope;
	unsigned int	frame_budget_us;
	unsigned long	util_sum;
	u8		calib_result;
	u8		reserved[7];
};

struct zenith_at_v3_calib_log_entry {
	u64		ts_ns;
	signed char	hyst_offset;
	signed char	cool_offset;
	unsigned int	total_transitions;
	unsigned int	state;
	unsigned int	frame_budget_us;
};

struct zenith_at_guardrails {
	unsigned int		obs_window;
	unsigned int		hyst_windows;
	unsigned int		cool_windows;
	signed char		hyst_offset;
	signed char		cool_offset;
};

struct zenith_policy {
	struct cpufreq_policy	*policy;
	struct zenith_tunables	*tunables;
	struct list_head	tunables_hook;

	raw_spinlock_t		update_lock;
	u64			last_freq_update_time;
	s64			min_rate_limit_ns;
	s64			up_rate_delay_ns;
	s64			down_rate_delay_ns;
	unsigned int		next_freq;
	unsigned int		cached_raw_freq;

	struct irq_work		irq_work;
	struct kthread_work	work;
	struct mutex		work_lock;
	struct kthread_worker	worker;
	struct task_struct	*thread;
	bool			work_in_progress;

	bool			limits_changed;
	bool			need_freq_update;

	u64			eff_unlock_at_ns[ZENITH_EFF_BINS_MAX];
	unsigned int		down_rate_mult;
	bool			brutal_active;
	bool			hispeed_active;
	u8			hispeed_entry_count;
	u8			brutal_entry_count;
	u8			peak_starve_count;
	u64			peak_rescue_until_ns;
	u64			migration_in_until_ns;
	u64			cluster_wake_pulse_until_ns;
	u64			cluster_wake_last_eval_ns;
	u64			fg_transition_pulse_until_ns;
	u64			psi_mem_cap_until_ns;
	u64			io_floor_until_ns;
	unsigned long		util_history[ZENITH_PREDICT_UP_WINDOW_MAX];
	unsigned int		util_history_idx;
	unsigned int		util_history_count;
	u64			render_first_seen_ns;
	unsigned int		peak_low_streak;
	unsigned int		peak_hyst_anchor_freq;
	unsigned int		boost_idle_low_streak;
	u64			last_runnable_ns;
	const char		*last_decision_path;

	struct zenith_dec_ring_entry
				dec_ring[ZENITH_DEC_RING_NR];
	unsigned int		dec_ring_head;

	unsigned long		cached_uclamp_min;
	unsigned long		cached_uclamp_max;
	u64			uclamp_cache_stamp_ns;
	unsigned int		nice_pct;

	atomic_t		at_samples_total;
	atomic_t		at_samples_saturated;
	u64			at_last_events;
	unsigned int		at_last_total;
	unsigned int		at_last_saturated;
	unsigned int		at_last_sat_pct;
	unsigned int		at_last_events_rate_x2;
	unsigned int		at_last_target;
	unsigned int		at_last_state;
	unsigned long		at_last_util_sum;
	u64			at_state_residency_ns[5];
	u64			at_state_last_change_ns;
	struct {
		u64		ts_ns;
		u32		flags;
		u8		from;
		u8		to;
		u8		reason;
		u8		_pad;
	}			at_history[ZENITH_AT_HISTORY_NR];
	unsigned int		at_history_head;
	unsigned int		at_history_count;
	unsigned int		at_last_applied_state;
	unsigned int		at_pending_state;
	unsigned int		at_pending_windows;
	unsigned int		at_cooldown_left;
	unsigned int		at_last_reason;
	unsigned int		at_last_flags;
	unsigned int		at_last_var_x256;
	unsigned int		at_last_psi_cpu;
	unsigned int		at_last_psi_io;
	unsigned int		at_last_psi_mem;
	unsigned int		at_last_thermal_pressure;
	unsigned int		at_last_thermal_slope;
	unsigned int		at_last_frame_budget_us;
	unsigned int		at_effective_up_rate_limit_us;
	unsigned int		at_effective_down_rate_limit_us;
	unsigned int		at_effective_up_threshold;
	unsigned int		at_effective_down_threshold;
	unsigned int		at_effective_input_boost_ms;
	unsigned int		at_effective_input_boost_cap_pct;
	unsigned int		at_effective_down_rate_adaptive;
	unsigned int		at_effective_down_threshold_adaptive;
	unsigned int		at_effective_frame_pace_floor_pct;
	unsigned int		at_effective_game_mode;
	bool			at_local_actions;

	unsigned int		at_boot_calm_streak;
	u64			at_v3_last_calib_ns;
	unsigned int		at_v3_last_transitions;
	signed char		at_v3_hyst_offset;
	signed char		at_v3_cool_offset;

	struct zenith_at_v3_calib_log_entry
				at_v3_calib_log[ZENITH_AT_V3_CALIB_LOG_NR];
	unsigned int		at_v3_calib_log_head;
	unsigned int		at_v3_calib_log_count;

	bool			at_local_glides_active;
	unsigned int		at_local_brutal_decay_ms;
	unsigned int		at_local_wakeup_boost_ms;
	unsigned int		at_local_boot_boost_decay_ms;
	unsigned int		at_local_screen_off_glide_ms;
	unsigned int		at_local_thermal_pressure_continuous;
	unsigned int		at_local_prefer_silver_aware;
	unsigned int		at_local_frame_budget_us_auto;

	bool			at_local_tiers_active;
	unsigned long		at_local_tier_armed_mask;
	struct delayed_work	at_work;

	struct zenith_at_log_entry at_log[ZENITH_AT_LOG_NR];
	unsigned int		at_log_head;
	unsigned int		at_log_count;

	u64			brutal_decay_until_ns;
	unsigned int		brutal_decay_arm_ms;
	u64			screen_off_arm_ns;
	unsigned int		screen_state_last;
	unsigned int		ps_prev_hit;
	unsigned int		ps_prev_miss;
	unsigned int		ps_hit_rate_pct;
	bool			is_big_cluster;
	unsigned int		up_rate_scale;
	unsigned int		down_rate_scale_shift;
	unsigned int		cluster_class;
	bool			render_active;
	u64			render_cache_stamp_ns;
	unsigned int		render_matched_util_avg;
	bool			top_app_active;
	u64			top_app_cache_stamp_ns;
	unsigned int		em_knee_freq;
	bool			audio_active;
	u64			audio_cache_stamp_ns;
	u64			audio_sticky_until_ns;
	bool			camera_auto_match;
	u64			camera_cache_stamp_ns;
	bool			game_auto_match;
	u64			game_auto_cache_stamp_ns;
	unsigned int		game_auto_streak;
	u8			gpb_state;
	u64			gpb_state_entry_ns;
	u64			gpb_b_arm_first_seen_ns;
	u64			gpb_b_disarm_first_seen_ns;
	struct thermal_zone_device *gpb_tzd;
	u64			gpb_tzd_retry_at_ns;
	u32			gpb_arm_count;
	u32			gpb_disarm_count;
	u32			gpb_idle_count;
	u8			gpb_last_disarm_reason;
	u64			boost_active_until_ns;
	unsigned int		load_var_ewma_x256;
	unsigned int		last_load_pct;
	unsigned long		stats[ZENITH_STAT_NR];
	unsigned long		dec_lat_buckets[4];
	unsigned long		vh_arch_freq_scale_last;
	u64			vh_cpu_idle_last_residency_ns;
	unsigned long		vh_sched_move_task_last_jiffies;
};

struct zenith_cpu {
	struct update_util_data	update_util;
	struct zenith_policy	*z_policy;
	unsigned int		cpu;

	bool			iowait_boost_pending;
	unsigned int		iowait_boost;
	u64			last_update;
	unsigned long		prev_util;
	unsigned long		bw_dl;
	u64			iowait_boost_stamp_ns;
	u64			iowait_backoff_stamp_ns;
	u64			thermal_pressure;
	u64			prev_thermal_pressure;
	unsigned int		wakeup_boost_tick;
	bool			idle_state;
	u64			cluster_wake_pulse_cpu_idle_stamp_ns;
	u64			vh_scheduler_tick_last_ns;
	unsigned long		vh_scheduler_tick_count;
	unsigned int		thermal_derate_pressure_pct;
};

struct zenith_pmu_state {
#ifdef CONFIG_PERF_EVENTS
	struct perf_event	*event_inst;
	struct perf_event	*event_cycles;
#endif
	unsigned long long	last_inst;
	unsigned long long	last_cycles;
};

struct zenith_comm_table {
	struct rcu_head	rcu;
	unsigned int	nr;
	const char	*entries[ZENITH_COMM_LIST_MAX];
	char		raw[ZENITH_COMM_BUF_MAX];
};

/* ─────────────────────────────────────────────
 * Extern global variables
 * ───────────────────────────────────────────── */

extern struct zenith_tunables *global_tunables;
extern struct mutex global_tunables_lock;

/* Static keys */
extern struct static_key_false zenith_audio_aware_key;
extern struct static_key_false zenith_camera_aware_key;
extern struct static_key_false zenith_render_aware_key;
extern struct static_key_false zenith_psi_aware_key;
extern struct static_key_false zenith_game_auto_key;
extern struct static_key_false zenith_auto_tune_v3_key;
extern struct static_key_false zenith_thermal_aware_key;
extern struct static_key_false zenith_game_perf_burst_key;

/* Global atomic state */
extern atomic64_t zenith_input_boost_until_ns;
extern atomic64_t zenith_input_last_event_ns;
extern atomic64_t zenith_peer_ramp_until_ns_big;
extern atomic64_t zenith_peer_ramp_until_ns_prime;
extern atomic64_t zenith_last_vblank_ns;
extern atomic64_t zenith_frame_overrun_until_ns;
extern atomic_t zenith_frame_overrun_streak;
extern atomic_t zenith_drm_vblank_us;
extern atomic_t zenith_boot_complete;
extern u64 zenith_boot_complete_ns;
extern atomic_t zenith_on_battery;
extern u64 zenith_game_auto_active_until_ns;
extern atomic_t zenith_v4l2_active_fds;
extern atomic_t zenith_alsa_active_fds;
extern unsigned int zenith_frame_overrun_slack_us_cache;
extern unsigned int zenith_frame_overrun_window_ms_cache;
extern unsigned int zenith_input_boost_active_ms;
extern unsigned int zenith_input_boost_touchdown_extra_ms_cache;
extern atomic64_t zenith_auto_input_events;
extern atomic64_t zenith_in_events_total;
extern atomic64_t zenith_in_boosts_armed;
extern atomic64_t zenith_in_boosts_quiet_extended;
extern atomic64_t zenith_in_boosts_skipped_disabled;
extern atomic64_t zenith_in_boosts_early_exit;

extern unsigned int zenith_cmdline_profile;
extern u8 zenith_cmdline_policy_profile[];
extern struct attribute_group zenith_groups[];
extern struct kobj_type zenith_tunables_ktype;

/* ─────────────────────────────────────────────
 * Inline helpers shared by all files
 * ───────────────────────────────────────────── */

#define ZENITH_FEATURE_ENABLED(name)	\
	static_branch_likely(&zenith_##name##_key)

static inline void zenith_set_static_key(struct static_key_false *key,
					 bool enable)
{
	if (enable)
		static_branch_enable(key);
	else
		static_branch_disable(key);
}

static inline struct zenith_tunables *to_zenith_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct zenith_tunables, attr_set);
}

/* ─────────────────────────────────────────────
 * Extern function declarations (cross-file)
 * ───────────────────────────────────────────── */

/* From cpufreq_zenith.c (core algorithm) */
extern unsigned int zenith_get_next_freq(struct zenith_policy *z_policy,
					 unsigned int util,
					 unsigned int max_cap);
extern unsigned int zenith_eff_hispeed_freq(struct zenith_policy *z_policy);
extern unsigned int zenith_glide_value(struct zenith_policy *z_policy,
				       unsigned int freq);
extern unsigned int zenith_get_util(struct zenith_cpu *z_cpu);
extern void zenith_iowait_boost(struct zenith_cpu *z_cpu, u64 time,
				unsigned int flags);
extern bool zenith_iowait_reset(struct zenith_cpu *z_cpu, u64 time,
				bool set_iowait_boost);
extern unsigned int zenith_iowait_floor(struct zenith_cpu *z_cpu);
extern unsigned int zenith_policy_thermal_pressure_pct(struct zenith_policy *z_policy);
extern bool zenith_thermal_active(struct zenith_policy *z_policy);
extern bool zenith_up_down_rate_limit(struct zenith_policy *z_policy, u64 time,
				      unsigned int target_freq);
extern unsigned int zenith_em_cap_freq(struct zenith_policy *z_policy,
				       unsigned int target_freq);
extern unsigned int zenith_tunable_or_local(struct zenith_policy *z_policy,
					    unsigned int local,
					    unsigned int shared);
extern unsigned int zenith_policy_max_ipc_pct(struct zenith_policy *z_policy);
extern unsigned int zenith_psi_mem_some_pct(void);
extern unsigned int zenith_psi_cpu_some_pct(void);
extern unsigned int zenith_psi_io_some_pct(void);
extern unsigned int zenith_sample_nice_pct(struct zenith_cpu *z_cpu, u64 now);
extern unsigned int zenith_em_knee_freq(struct zenith_policy *z_policy);
extern bool zenith_policy_has_top_app(struct zenith_policy *z_policy);
extern bool zenith_policy_has_render(struct zenith_policy *z_policy);
extern bool zenith_policy_has_audio(struct zenith_policy *z_policy);
extern bool zenith_policy_has_game_auto(struct zenith_policy *z_policy);
extern bool zenith_game_auto_active(void);
extern unsigned int zenith_eff_game_mode(unsigned int base_gm);
extern void zenith_policy_game_auto_tick(struct zenith_policy *z_policy);
extern void zenith_gpb_evaluate(struct zenith_policy *z_policy);
extern void zenith_uclamp_cache_refresh(struct zenith_policy *z_policy);
extern void zenith_pmu_sample_cpu(unsigned int cpu);
extern int zenith_pmu_init_cpu(unsigned int cpu);
extern void zenith_pmu_exit_cpu(unsigned int cpu);
extern void zenith_psi_cgroup_apply(const char *path);
extern void zenith_update_single(struct update_util_data *hook, u64 time,
				 unsigned int flags);
extern void zenith_update_shared(struct update_util_data *hook, u64 time,
				 unsigned int flags);
extern void zenith_irq_work(struct irq_work *irq_work);
extern void zenith_work(struct kthread_work *work);
extern void zenith_execute_switch(struct zenith_policy *z_policy, u64 time,
				  unsigned int next_freq);
extern int zenith_init(struct cpufreq_policy *policy);
extern int zenith_exit(struct cpufreq_policy *policy);
extern int zenith_start(struct cpufreq_policy *policy);
extern void zenith_stop(struct cpufreq_policy *policy);
extern int zenith_limits(struct cpufreq_policy *policy);
extern void zenith_ignore_dl_rate_limit(struct zenith_cpu *z_cpu,
					unsigned int flags);

/* From cpufreq_zenith_sysfs.c (sysfs interface + adaptive tuning) */
extern void zenith_apply_profile(struct zenith_tunables *t, unsigned int prof);
extern void zenith_invalidate_cache(struct gov_attr_set *attr_set);
extern void zenith_refresh_rate_delays(struct gov_attr_set *attr_set);
extern void zenith_refresh_rate_delays_one(struct zenith_policy *z_policy);
extern void zenith_reset_local_actions(struct zenith_policy *z_policy);
extern void zenith_policy_observability_reset(struct zenith_policy *z_policy);
extern void update_min_rate_limit_ns(struct zenith_policy *z_policy);
extern void zenith_update_cluster_rate_scale(struct zenith_policy *z_policy);
extern void zenith_update_rate_delay_ns(struct zenith_policy *z_policy);
extern void zenith_log_profile_change(struct zenith_tunables *t,
				      unsigned int old_profile,
				      unsigned int new_profile);
extern void zenith_log_profile_applied(struct zenith_tunables *t,
				       unsigned int prof);
extern void zenith_log_master_flip(struct zenith_tunables *t,
				   unsigned int old_state,
				   unsigned int new_state);

/* From cpufreq_zenith_autotune.c (auto-tune observer) */
extern void zenith_auto_tune_work(struct work_struct *w);
extern void zenith_auto_eval_work_fn(struct work_struct *w);
extern void zenith_tunables_free(struct kobject *kobj);
extern unsigned int zenith_parse_profile_name(const char *name);
extern void zenith_profile_defaults(struct zenith_tunables *t,
				    unsigned int prof);
extern unsigned int zenith_profile_to_at_state(unsigned int profile);
extern unsigned int zenith_at_state_to_profile(unsigned int state);
extern const char *zenith_at_state_name(unsigned int state);
extern const char *zenith_at_reason_name(unsigned int reason);
extern const char *zenith_at_cluster_name(unsigned int cluster);
extern unsigned int zenith_at_profile_for_state(unsigned int state);
extern const char *zenith_gpb_state_name(u8 state);
extern const char *zenith_gpb_disarm_name(u8 reason);
extern void zenith_at_update_prefer_silver_rate(struct zenith_policy *z_policy,
						unsigned int hit_pct);
extern struct zenith_at_guardrails zenith_at_get_guardrails(struct zenith_policy *z_policy);
extern void zenith_at_get_policy_guardrails(struct zenith_policy *z_policy,
					    struct zenith_at_guardrails *gr);
extern void zenith_at_log_push(struct zenith_policy *z_policy, unsigned int state,
			       unsigned int reason, unsigned int flags);
extern void zenith_at_v3_calibrate(struct zenith_policy *z_policy);
extern void zenith_at_mark_override(struct zenith_policy *z_policy);
extern int zenith_at_set_uint(struct zenith_policy *z_policy,
			      const char *name, unsigned int val);
extern unsigned int zenith_at_eff_hyst_windows(struct zenith_policy *z_policy);
extern unsigned int zenith_at_eff_cool_windows(struct zenith_policy *z_policy);
extern void zenith_at_apply_actions(struct zenith_policy *z_policy,
				    unsigned int target, unsigned int reason,
				    unsigned int flags);
extern void zenith_at_apply_glides(struct zenith_policy *z_policy);
extern void zenith_at_apply_tiers(struct zenith_policy *z_policy);
extern void zenith_at_clamp(struct zenith_tunables *t, unsigned int prof);
extern unsigned int zenith_at_write_effective(struct zenith_policy *z_policy,
					      unsigned int *member,
					      unsigned int val);

/* Decision-path recording */
extern void zenith_input_event(struct input_handle *handle, unsigned int type,
			       unsigned int code, int value);

#endif /* _CPUFREQ_ZENITH_INTERNAL_H */
