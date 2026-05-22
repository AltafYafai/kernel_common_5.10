// SPDX-License-Identifier: GPL-2.0
/*
 * cpufreq_zenith_internal.h - Shared definitions for the Zenith CPUFreq governor
 * Auto-generated from cpufreq_zenith.c. Do not edit manually.
 */
#ifndef _CPUFREQ_ZENITH_INTERNAL_H
#define _CPUFREQ_ZENITH_INTERNAL_H

#include <linux/types.h>
#include <linux/cpufreq.h>
#include <linux/sched.h>
#include <linux/sched/loadavg.h>
#include <linux/jump_label.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/bits.h>
#include <linux/spinlock.h>
#include <linux/cpumask.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/percpu.h>
#include <linux/rbtree.h>
#include <linux/u64_stats_sync.h>
#include <linux/lockdep.h>
#include <uapi/linux/sched/types.h>

struct cpufreq_policy;
struct cpufreq_freqs;

#define ZENITH_DEFAULT_IOWAIT_BOOST_MIN		125
#define ZENITH_DEFAULT_IOWAIT_STACK_PCT		50	/* 0 = legacy max(util, boost) */
#define ZENITH_DEFAULT_IOWAIT_BACKOFF_AFTER_MS	0
#define ZENITH_DEFAULT_UP_THRESHOLD		75
#define ZENITH_DEFAULT_UP_THRESHOLD_HISPEED	0	/* disabled */
#define ZENITH_DEFAULT_DOWN_THRESHOLD		60
#define ZENITH_DEFAULT_HISPEED_FREQ		0	/* disabled */
#define ZENITH_HISPEED_FREQ_MAX			50000000U
#define ZENITH_DEFAULT_HISPEED_FREQ_PCT		55	/* fallback when hispeed_freq=0 */
#define ZENITH_DEFAULT_HISPEED_LOAD		65
#define ZENITH_DEFAULT_HISPEED_HYST_PCT		10	/* exit hysteresis margin */
#define ZENITH_DEFAULT_HISPEED_ENTRY_STREAK	0
#define ZENITH_HISPEED_ENTRY_STREAK_MAX		16
#define ZENITH_DEFAULT_BRUTAL_ENTRY_STREAK	0
#define ZENITH_BRUTAL_ENTRY_STREAK_MAX		16
#define ZENITH_DEFAULT_PEAK_HEADROOM_RESCUE		1
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
#define ZENITH_DEFAULT_AUTO_EVAL_MS		500
#define ZENITH_DEFAULT_AUTO_HYSTERESIS_MS	2000
#define ZENITH_AUTO_EVAL_MS_MIN			100
#define ZENITH_AUTO_EVAL_MS_MAX			60000
#define ZENITH_AUTO_HYSTERESIS_MS_MIN		0
#define ZENITH_AUTO_HYSTERESIS_MS_MAX		60000
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
#define ZENITH_FRAME_OVERRUN_DEEP_FLOOR_PCT_MAX	100
#define ZENITH_DEFAULT_PEER_RAMP_UCLAMP_MIN_RESPECT		1
#define ZENITH_DEFAULT_MIGRATION_FLOOR_UCLAMP_MIN_RESPECT	1
#define ZENITH_DEFAULT_PSI_MEM_CAP_THRESH		0
#define ZENITH_PSI_MEM_CAP_THRESH_MAX			100
#define ZENITH_DEFAULT_PSI_MEM_CAP_PCT			80
#define ZENITH_PSI_MEM_CAP_PCT_MIN			50
#define ZENITH_PSI_MEM_CAP_PCT_MAX			100
#define ZENITH_DEFAULT_PSI_MEM_CAP_WINDOW_MS		1000
#define ZENITH_PSI_MEM_CAP_WINDOW_MS_MIN		100
#define ZENITH_PSI_MEM_CAP_WINDOW_MS_MAX		5000
#define ZENITH_DEFAULT_UP_THRESHOLD_ADAPTIVE	0
#define ZENITH_UP_THRESHOLD_ADAPTIVE_MAX	30
#define ZENITH_UCLAMP_CACHE_TTL_NS		(1 * NSEC_PER_MSEC)
#define ZENITH_EFF_BINS_MAX			8
#define ZENITH_AT_LOG_NR			16
#define ZENITH_AT_HISTORY_NR			32
#define ZENITH_CLIMB_MODE_SNAP			0	/* default */
#define ZENITH_CLIMB_MODE_STEP			1
#define ZENITH_PROFILE_CUSTOM			0	/* default */
#define ZENITH_PROFILE_PERFORMANCE		1
#define ZENITH_PROFILE_BALANCED			2
#define ZENITH_PROFILE_BATTERY			3
#define ZENITH_PROFILE_LEGACY			4
#define ZENITH_PROFILE_GAMING			5
#define ZENITH_PROFILE_AUDIO			6
#define ZENITH_PROFILE_AUTO			7
#define ZENITH_FEATURE_ENABLED(name)	\
#define ZENITH_DEFAULT_CLIMB_MODE		ZENITH_CLIMB_MODE_SNAP
#define ZENITH_DEFAULT_FREQ_STEP_PCT		5
#define ZENITH_DEFAULT_FREQ_STEP_ADAPTIVE	0
#define ZENITH_DEFAULT_THERMAL_AUTO		1
#define ZENITH_THERMAL_AUTO_PRESSURE_PCT	10
#define ZENITH_DEFAULT_THERMAL_AWARE		1
#define ZENITH_DEFAULT_THERMAL_PRESSURE_CONTINUOUS	1
#define ZENITH_DEFAULT_PREFER_SILVER_AWARE			1
#define ZENITH_DEFAULT_PREFER_SILVER_HOT_THRESHOLD_PCT		50
#define ZENITH_DEFAULT_PREFER_SILVER_HOT_BUMP_PCT		5
#define ZENITH_PREFER_SILVER_HOT_BUMP_MAX_PCT			20
#define ZENITH_DEFAULT_BRUTAL_DECAY_MS				0
#define ZENITH_BRUTAL_DECAY_MS_MAX				500
#define ZENITH_DEFAULT_THERMAL_UTIL_DERATE	1
#define ZENITH_THERMAL_DERATE_FLOOR_PCT		5
#define ZENITH_DEFAULT_THERMAL_DERATE_RATE_PCT	25
#define ZENITH_DEFAULT_AUTO_THERMAL_CAP			0
#define ZENITH_DEFAULT_AUTO_THERMAL_CAP_PRESSURE_PCT	50
#define ZENITH_AUTO_THERMAL_CAP_PRESSURE_PCT_MIN	1
#define ZENITH_AUTO_THERMAL_CAP_PRESSURE_PCT_MAX	100
#define ZENITH_DEFAULT_AUTO_THERMAL_CAP_FREQ_PCT	80
#define ZENITH_AUTO_THERMAL_CAP_FREQ_PCT_MIN		50
#define ZENITH_AUTO_THERMAL_CAP_FREQ_PCT_MAX		100
#define ZENITH_DEFAULT_FREQ_STABILITY_MARGIN_PCT	3
#define ZENITH_FREQ_STABILITY_MARGIN_PCT_MAX		10
#define ZENITH_DEFAULT_DOWN_RATE_ADAPTIVE	1
#define ZENITH_DEFAULT_WAKEUP_BOOST		1
#define ZENITH_WAKEUP_IDLE_THRESH_PCT		10
#define ZENITH_WAKEUP_BUSY_THRESH_PCT		40
#define ZENITH_WAKEUP_BOOST_TICKS		2
#define ZENITH_DEFAULT_WAKEUP_BOOST_MS		0
#define ZENITH_WAKEUP_BOOST_MS_MAX		200
#define ZENITH_DEFAULT_DOWN_THRESHOLD_ADAPTIVE	0
#define ZENITH_DOWN_THRESHOLD_ADAPTIVE_MAX	20
#define ZENITH_DEFAULT_RATE_LIMIT_CLUSTER_SCALE	1
#define ZENITH_CLUSTER_LITTLE_THRESH_PCT	60
#define ZENITH_DEFAULT_UP_RATE_LIMIT_US		100
#define ZENITH_DEFAULT_DOWN_RATE_LIMIT_US	4000
#define ZENITH_RATE_LIMIT_US_MAX		60000000U
#define ZENITH_DEFAULT_INPUT_BOOST_DOWN_RATE_MULT_PCT	200
#define ZENITH_INPUT_BOOST_DOWN_RATE_MULT_PCT_MAX	1000
#define ZENITH_DEFAULT_POWERSAVE_BIAS		0
#define ZENITH_DEFAULT_SCREEN_ON_BIAS_PCT	50
#define ZENITH_DEFAULT_IO_IS_BUSY		1
#define ZENITH_DEFAULT_INPUT_BOOST_MS		80
#define ZENITH_DEFAULT_INPUT_BOOST_DECAY_MS	30
#define ZENITH_DEFAULT_INPUT_BOOST_TOUCHDOWN_EXTRA_MS	50
#define ZENITH_INPUT_BOOST_TOUCHDOWN_EXTRA_MS_MAX	500
#define ZENITH_DEFAULT_INPUT_BOOST_DECAY_CURVE	0
#define ZENITH_DEFAULT_INPUT_BOOST_BIG_ONLY	1
#define ZENITH_INPUT_QUIET_THRESHOLD_MS		1000
#define ZENITH_INPUT_QUIET_BOOST_MULT_PCT	200
#define ZENITH_INPUT_QUIET_BOOST_MAX_MS		250
#define ZENITH_DEFAULT_INPUT_BOOST_CAP_PCT	0
#define ZENITH_DEFAULT_EFFICIENT_FREQ		0
#define ZENITH_DEFAULT_EFF_BIN_HYST_PCT		0
#define ZENITH_EFF_BIN_HYST_PCT_MAX		20
#define ZENITH_DEFAULT_UP_DELAY_US		4000
#define ZENITH_DEFAULT_LIGHT_LOAD_FREQ		0
#define ZENITH_LIGHT_LOAD_FREQ_MAX		50000000U
#define ZENITH_DEFAULT_LIGHT_LOAD_THRESHOLD	20
#define ZENITH_DEFAULT_SAMPLING_DOWN_FACTOR	2
#define ZENITH_MAX_SAMPLING_DOWN_FACTOR		10
#define ZENITH_DEFAULT_BOOST_EXIT_EXTEND	1	/* stretch down-rate after a boost ends */
#define ZENITH_DEFAULT_BIAS_LOAD_THRESHOLD	50
#define ZENITH_DEFAULT_AT_SAT_LOAD_PCT		70
#define ZENITH_DEFAULT_AT_HI_SAT_PCT		60
#define ZENITH_DEFAULT_AT_LO_SAT_PCT		10
#define ZENITH_DEFAULT_AT_HI_EVENTS_X2		4
#define ZENITH_DEFAULT_AT_LO_EVENTS_X2		1
#define ZENITH_DEFAULT_AUTO_TUNE_SCENARIO	1
#define ZENITH_DEFAULT_AUTO_TUNE_V2		1
#define ZENITH_DEFAULT_AT_HYSTERESIS_WINDOWS	2
#define ZENITH_DEFAULT_AT_COOLDOWN_WINDOWS	1
#define ZENITH_AT_HYSTERESIS_WINDOWS_MAX	8
#define ZENITH_AT_COOLDOWN_WINDOWS_MAX		8
#define ZENITH_DEFAULT_AT_V2_VAR_PROMOTE_THRESH	768
#define ZENITH_AT_V2_VAR_PROMOTE_THRESH_MAX	65535U
#define ZENITH_DEFAULT_AT_UTIL_RISING_THRESH_PCT	25
#define ZENITH_AT_UTIL_RISING_THRESH_PCT_MAX	200U
#define ZENITH_DEFAULT_AT_RENDER_RT_FLOOR_PCT	0
#define ZENITH_AT_RENDER_RT_FLOOR_PCT_MAX	100U
#define ZENITH_DEFAULT_AUTO_TUNE_V3		2
#define ZENITH_AT_V3_MODE_OFF			0
#define ZENITH_AT_V3_MODE_OBSERVE		1
#define ZENITH_AT_V3_MODE_APPLY			2
#define ZENITH_AT_V3_MODE_MAX			2
#define ZENITH_DEFAULT_AT_V3_INTERVAL_MS	60000
#define ZENITH_AT_V3_INTERVAL_MIN_MS		10000
#define ZENITH_AT_V3_INTERVAL_MAX_MS		600000
#define ZENITH_AT_V3_THRASH_HI			6
#define ZENITH_AT_V3_THRASH_LO			1
#define ZENITH_AT_V3_OFFSET_MIN			(-1)
#define ZENITH_AT_V3_OFFSET_MAX			(+4)
#define ZENITH_AT_V3_CALIB_LOG_NR		8
#define ZENITH_DEFAULT_AT_CLUSTER_AWARE		1
#define ZENITH_DEFAULT_AT_V2_SIGNALS		1
#define ZENITH_DEFAULT_AT_THERMAL_SLOPE		1
#define ZENITH_DEFAULT_AT_THERMAL_PRESSURE_PCT	18
#define ZENITH_DEFAULT_AT_THERMAL_SLOPE_PCT	4
#define ZENITH_DEFAULT_AT_FRAME_PACING		1
#define ZENITH_DEFAULT_AT_SUSTAINED_GAMING	1
#define ZENITH_AT_THERMAL_PCT_MAX		100
#define ZENITH_DEFAULT_AUTO_TUNE_V2_GLIDES	1
#define ZENITH_DEFAULT_AUTO_TUNE_V2_TIERS	1
#define ZENITH_AT_GLIDE_BRUTAL_DECAY_MS		150
#define ZENITH_AT_GLIDE_WAKEUP_BOOST_MS		50
#define ZENITH_AT_GLIDE_BOOT_BOOST_DECAY_MS	5000
#define ZENITH_AT_GLIDE_SCREEN_OFF_MS		300
#define ZENITH_AT_GLIDE_THERMAL_PRESSURE_PCT	25
#define ZENITH_AT_STATE_EFFICIENCY		0
#define ZENITH_AT_STATE_BALANCED		1
#define ZENITH_AT_STATE_LATENCY			2
#define ZENITH_AT_STATE_SUSTAINED_PERF		3
#define ZENITH_AT_STATE_THERMAL_RECOVERY	4
#define ZENITH_AT_REASON_CLASSIFIER		0
#define ZENITH_AT_REASON_CAMERA_RENDER		1
#define ZENITH_AT_REASON_MEMSTALL		2
#define ZENITH_AT_REASON_AUDIO			3
#define ZENITH_AT_REASON_VARIANCE		4
#define ZENITH_AT_REASON_THERMAL		5
#define ZENITH_AT_REASON_COOLDOWN		6
#define ZENITH_AT_REASON_HYSTERESIS		7
#define ZENITH_AT_REASON_SCREEN			8
#define ZENITH_AT_REASON_PSI			9
#define ZENITH_AT_REASON_FRAME			10
#define ZENITH_AT_REASON_GAME			11
#define ZENITH_AT_REASON_THERMAL_SLOPE		12
#define ZENITH_AT_FLAG_AUDIO			BIT(0)
#define ZENITH_AT_FLAG_CAMERA			BIT(1)
#define ZENITH_AT_FLAG_RENDER			BIT(2)
#define ZENITH_AT_FLAG_MEMSTALL			BIT(3)
#define ZENITH_AT_FLAG_THERMAL			BIT(4)
#define ZENITH_AT_FLAG_SCREEN_OFF		BIT(5)
#define ZENITH_AT_FLAG_PSI_CPU			BIT(6)
#define ZENITH_AT_FLAG_PSI_IO			BIT(7)
#define ZENITH_AT_FLAG_FRAME			BIT(8)
#define ZENITH_AT_FLAG_GAME			BIT(9)
#define ZENITH_AT_FLAG_THERMAL_SLOPE		BIT(10)
#define ZENITH_AT_FLAG_LOCAL_ACTIONS		BIT(11)
#define ZENITH_AT_FLAG_PREFER_SILVER_HOT	BIT(12)
#define ZENITH_AT_FLAG_UTIL_RISING		BIT(13)
#define ZENITH_AT_OVERRIDE_UP_RATE		BIT(0)
#define ZENITH_AT_OVERRIDE_DOWN_RATE		BIT(1)
#define ZENITH_AT_OVERRIDE_UP_THRESHOLD		BIT(2)
#define ZENITH_AT_OVERRIDE_DOWN_THRESHOLD	BIT(3)
#define ZENITH_AT_OVERRIDE_INPUT_BOOST_MS	BIT(4)
#define ZENITH_AT_OVERRIDE_INPUT_BOOST_CAP	BIT(5)
#define ZENITH_AT_OVERRIDE_DOWN_ADAPTIVE	BIT(6)
#define ZENITH_AT_OVERRIDE_DOWN_THRESH_ADAPTIVE	BIT(7)
#define ZENITH_AT_OVERRIDE_FRAME_PACE		BIT(8)
#define ZENITH_AT_OVERRIDE_GAME_MODE		BIT(9)
#define ZENITH_AT_OVERRIDE_MIGRATION_JUMP	BIT(10)
#define ZENITH_AT_OVERRIDE_MIGRATION_FLOOR_WIN	BIT(11)
#define ZENITH_AT_OVERRIDE_MIGRATION_FLOOR_PCT	BIT(12)
#define ZENITH_AT_OVERRIDE_PSI_CPU_FLOOR	BIT(13)
#define ZENITH_AT_OVERRIDE_FRAME_OVR_SLACK	BIT(14)
#define ZENITH_AT_OVERRIDE_FRAME_OVR_WINDOW	BIT(15)
#define ZENITH_AT_OVERRIDE_FRAME_OVR_FLOOR	BIT(16)
#define ZENITH_AT_OVERRIDE_PSI_MEM_CAP_THRESH	BIT(17)
#define ZENITH_AT_OVERRIDE_PSI_MEM_CAP_PCT	BIT(18)
#define ZENITH_AT_OVERRIDE_PSI_MEM_CAP_WINDOW	BIT(19)
#define ZENITH_AT_OVERRIDE_RENDER_FLOOR_PCT	BIT(20)
#define ZENITH_AT_OVERRIDE_AUDIO_FLOOR_PCT	BIT(21)
#define ZENITH_AT_OVERRIDE_AUDIO_CAP_PCT	BIT(22)
#define ZENITH_AT_OVERRIDE_AUDIO_HYST_MS	BIT(23)
#define ZENITH_AT_OVERRIDE_CAMERA_FLOOR_PCT	BIT(24)
#define ZENITH_AT_TIER_MIGRATION		BIT(0)
#define ZENITH_AT_TIER_PSI_CPU_FLOOR		BIT(1)
#define ZENITH_AT_TIER_FRAME_OVERRUN		BIT(2)
#define ZENITH_AT_TIER_PSI_MEM_CAP		BIT(3)
#define ZENITH_CLUSTER_LITTLE			0
#define ZENITH_CLUSTER_BIG			1
#define ZENITH_CLUSTER_PRIME			2
#define ZENITH_DEFAULT_KCPUSTAT_WINDOW_US	4000
#define ZENITH_DEFAULT_KCPUSTAT_FILTER_SHIFT	1
#define ZENITH_DEFAULT_KCPUSTAT_HISPEED_ENABLE	1
#define ZENITH_DEFAULT_UTIL_MATH_V2		1
#define ZENITH_DEFAULT_UCLAMP_MIN_RESPECT	1
#define ZENITH_DEFAULT_PREDICT_UTIL_PCT		10
#define ZENITH_PREDICT_UTIL_PCT_MAX		200
#define ZENITH_DEFAULT_PREDICT_UTIL_SMOOTH	1
#define ZENITH_DEFAULT_RENDER_AWARE		1
#define ZENITH_DEFAULT_RENDER_FLOOR_PCT		70
#define ZENITH_RENDER_CACHE_TTL_NS		(4 * NSEC_PER_MSEC)
#define ZENITH_DEFAULT_RENDER_FLOOR_MIN_RUNTIME_MS	50
#define ZENITH_RENDER_FLOOR_MIN_RUNTIME_MS_MAX		1000
#define ZENITH_DEFAULT_GAME_MODE			0
#define ZENITH_GAME_HISPEED_BOOST_PCT		110
#define ZENITH_GAME_BOOST_DECAY_PCT		130
#define ZENITH_GAME_L2_HISPEED_BOOST_PCT	120
#define ZENITH_GAME_L2_BOOST_DECAY_PCT		160
#define ZENITH_GAME_MODE_MAX			2
#define ZENITH_DEFAULT_GAME_AUTO		1
#define ZENITH_GAME_AUTO_CACHE_TTL_NS		(4 * NSEC_PER_MSEC)
#define ZENITH_GAME_AUTO_DETECT_STREAK		32
#define ZENITH_GAME_AUTO_ACTIVE_TTL_NS		(5ULL * NSEC_PER_SEC)
#define ZENITH_DEFAULT_FRAME_BUDGET_US		0
#define ZENITH_FRAME_BUDGET_US_MAX		50000
#define ZENITH_DEFAULT_FRAME_BUDGET_US_AUTO	0
#define ZENITH_DEFAULT_FRAME_PACE_FLOOR_PCT	0
#define ZENITH_FRAME_PACE_BASE_BUDGET_US	16667
#define ZENITH_DEFAULT_PSI_AWARE		1
#define ZENITH_DEFAULT_PSI_MEM_THRESH		50
#define ZENITH_DEFAULT_PSI_CPU_THRESH		0
#define ZENITH_DEFAULT_PSI_IO_THRESH		0
#define ZENITH_PSI_CGROUP_PATH_MAX		128
#define ZENITH_DEFAULT_AUDIO_AWARE		1
#define ZENITH_DEFAULT_AUDIO_FLOOR_PCT		0
#define ZENITH_DEFAULT_AUDIO_CAP_PCT		0
#define ZENITH_AUDIO_CACHE_TTL_NS		(4 * NSEC_PER_MSEC)
#define ZENITH_DEFAULT_AUDIO_HYST_MS		250
#define ZENITH_AUDIO_HYST_MS_MAX		2000
#define ZENITH_DEC_RING_NR			32
#define ZENITH_DEC_RING_MASK			(ZENITH_DEC_RING_NR - 1)
#define ZENITH_DEFAULT_CAMERA_AWARE		1
#define ZENITH_DEFAULT_CAMERA_ACTIVE		0
#define ZENITH_DEFAULT_CAMERA_FLOOR_PCT		0
#define ZENITH_CAMERA_CACHE_TTL_NS		(4 * NSEC_PER_MSEC)
#define ZENITH_CAMERA_OVERRIDE_AUTO		0
#define ZENITH_CAMERA_OVERRIDE_FORCE_ON		1
#define ZENITH_CAMERA_OVERRIDE_FORCE_OFF	2
#define ZENITH_DEFAULT_BOOT_BOOST_MS		0
#define ZENITH_DEFAULT_SCREEN_OFF_GLIDE_MS	0
#define ZENITH_SCREEN_OFF_GLIDE_MS_MAX		2000
#define ZENITH_DEFAULT_BOOT_BOOST_DECAY_MS	0
#define ZENITH_BOOT_BOOST_DECAY_MS_MAX		30000
#define ZENITH_BOOT_BOOST_MAX_MS		300000
#define ZENITH_DEFAULT_BOOT_COMPLETE_AUTO	1
#define ZENITH_BOOT_COMPLETE_CALM_WINDOWS	2
#define ZENITH_BOOT_COMPLETE_GRACE_NS \
#define ZENITH_DEFAULT_UCLAMP_MAX_RESPECT	1
#define ZENITH_UCLAMP_MIN_MEANINGFUL_PCT	10
#define ZENITH_KCPUSTAT_WINDOW_MIN_US		1000
#define ZENITH_KCPUSTAT_WINDOW_MAX_US		100000
#define ZENITH_KCPUSTAT_FILTER_SHIFT_MAX	8
#define ZENITH_DEFAULT_VERBOSE_LOG		0
#define ZENITH_DEFAULT_GAME_PERF_BURST			1
#define ZENITH_DEFAULT_GAME_PERF_BURST_FLOOR_PCT		85
#define ZENITH_DEFAULT_GAME_PERF_BURST_THERMAL_CEILING_DC	48000
#define ZENITH_DEFAULT_GAME_PERF_BURST_DISARM_GRACE_MS	1000
#define ZENITH_DEFAULT_GAME_PERF_BURST_COOLDOWN_MS		5000
#define ZENITH_GAME_PERF_BURST_FLOOR_PCT_MIN		50
#define ZENITH_GAME_PERF_BURST_FLOOR_PCT_MAX		100
#define ZENITH_GAME_PERF_BURST_THERMAL_CEILING_DC_MIN	40000
#define ZENITH_GAME_PERF_BURST_THERMAL_CEILING_DC_MAX	60000
#define ZENITH_GAME_PERF_BURST_DISARM_GRACE_MS_MAX	10000
#define ZENITH_GAME_PERF_BURST_COOLDOWN_MS_MAX		60000
#define ZENITH_GAME_PERF_BURST_B_THRESHOLD_PCT		70
#define ZENITH_GAME_PERF_BURST_B_REQUIRED_NS		(2ULL * NSEC_PER_SEC)
#define ZENITH_GAME_PERF_BURST_B_EXIT_THRESHOLD_PCT	60
#define ZENITH_GPB_TZD_RETRY_INTERVAL_NS		(5ULL * NSEC_PER_SEC)
#define ZENITH_GPB_STATE_IDLE		0
#define ZENITH_GPB_STATE_ARMED		1
#define ZENITH_GPB_STATE_COOLDOWN	2
#define ZENITH_GPB_DISARM_NONE		0
#define ZENITH_GPB_DISARM_FAST		1
#define ZENITH_GPB_DISARM_SUSTAINED	2

struct zenith_tunables {
	struct gov_attr_set	attr_set;
	unsigned int		up_rate_limit_us;
	unsigned int		down_rate_limit_us;
	unsigned int		up_threshold;

	/* See ZENITH_DEFAULT_UP_THRESHOLD_ADAPTIVE.  Magnitude of the
	 * variance-adaptive lowering applied to dynamic_up_thresh on
	 * bursty workloads, in percent of the static up_threshold.
	 * Range 0..ZENITH_UP_THRESHOLD_ADAPTIVE_MAX.  0 disables.
	 */
	unsigned int		up_threshold_adaptive;
	unsigned int		down_threshold;	/* hysteresis lower bound */

	/* Hispeed floor tier: when load >= hispeed_load (% of max_cap),
	 * ensure the chosen target_freq is at least hispeed_freq (kHz).
	 * hispeed_freq=0 disables the explicit tier.
	 *
	 * When hispeed_freq=0 and hispeed_freq_pct>0, the tier falls
	 * back to a per-cluster auto-default: the effective hispeed
	 * floor for the policy becomes (policy->max * hispeed_freq_pct
	 * / 100).  This gives sensible defaults on both the little and
	 * big clusters without userspace having to read policyN/max and
	 * write one absolute kHz value per policy.  hispeed_freq_pct=0
	 * preserves the legacy "tier disabled" semantics.
	 */
	unsigned int		hispeed_freq;
	unsigned int		hispeed_freq_pct;
	unsigned int		hispeed_load;

	/* Hysteresis margin (percent of max_cap) applied to hispeed_load
	 * on the _exit_ side of the tier.  Entering the tier still
	 * triggers at load_pct >= hispeed_load; leaving it requires
	 * load_pct < (hispeed_load - hispeed_hyst_pct).  Collapses to
	 * the legacy no-hysteresis behaviour at 0.  See
	 * ZENITH_DEFAULT_HISPEED_HYST_PCT.
	 */
	unsigned int		hispeed_hyst_pct;

	/* Symmetric entry-side hysteresis for the hispeed tier.  See
	 * ZENITH_DEFAULT_HISPEED_ENTRY_STREAK.  Capped on store to
	 * ZENITH_HISPEED_ENTRY_STREAK_MAX so the per-policy u8 counter
	 * cannot overflow.
	 */
	unsigned int		hispeed_entry_streak;

	/* Symmetric entry-side hysteresis for the brutality tier.  See
	 * ZENITH_DEFAULT_BRUTAL_ENTRY_STREAK.  Only gates SNAP climb
	 * mode; STEP mode is unaffected.  Capped on store to
	 * ZENITH_BRUTAL_ENTRY_STREAK_MAX so the per-policy u8 counter
	 * cannot overflow.
	 */
	unsigned int		brutal_entry_streak;

	/* Watchdog gate for the peak-headroom rescue tier.  See
	 * ZENITH_DEFAULT_PEAK_HEADROOM_RESCUE for full semantics.
	 * 1 enables the rescue (the default); 0 disables it entirely.
	 * The rescue is also gated by max_cap and policy->max being
	 * non-zero and pin_to_target being false.
	 */
	unsigned int		peak_headroom_rescue;

	/* Per-policy parameters for the peak-headroom rescue tier.
	 * See ZENITH_DEFAULT_PEAK_HEADROOM_* for full semantics and
	 * default values.  All five exist as sysfs knobs so a tester
	 * can A/B-tune the rescue at runtime without rebuilding the
	 * kernel.
	 *
	 *   peak_headroom_starve_load_pct (1..100, default 90)
	 *     Minimum load_pct at which a sample counts as "starving".
	 *     A value of 100 means only fully-saturated samples count;
	 *     0 is rejected on store (rescue would fire continuously).
	 *
	 *   peak_headroom_freq_floor_pct (1..100, default 85)
	 *     Cluster freq must be below this fraction of policy->max
	 *     for a sample to count as "starving".  100 means rescue
	 *     fires whenever freq < policy->max (always-on under
	 *     starvation); values closer to 0 require the cluster to
	 *     be very deep below peak before the rescue triggers.
	 *
	 *   peak_headroom_starve_streak (0..PEAK_HEADROOM_STREAK_MAX,
	 *     default 3)
	 *     Number of consecutive starving samples required before
	 *     the rescue may fire.  The streak counter must exceed
	 *     this value, so the effective wait is streak+1 windows.
	 *     0 fires on the very first starving sample.
	 *
	 *   peak_headroom_jump_pct (1..100, default 100)
	 *     Target as a percentage of policy->max when the rescue
	 *     fires.  100 pins to policy->max; lower values rescue
	 *     to an intermediate freq.  0 is rejected on store
	 *     (rescue with target 0 makes no sense).
	 *
	 *   peak_headroom_hold_ms (0..PEAK_HEADROOM_HOLD_MS_MAX,
	 *     default 50)
	 *     Minimum gap between two consecutive rescue fires, in
	 *     milliseconds.  Prevents stacking rescues on adjacent
	 *     ticks before the cpufreq driver has applied the
	 *     previous request.  0 disables the hold-down (rescue
	 *     can fire on every starving sample past the streak).
	 *
	 * The streak field is u8 for cache locality, so the sysfs
	 * store caps at PEAK_HEADROOM_STREAK_MAX (16); hold_ms caps
	 * at PEAK_HEADROOM_HOLD_MS_MAX (1000) so a runaway value
	 * cannot effectively pin the rescue off forever.
	 */
	unsigned int		peak_headroom_starve_load_pct;
	unsigned int		peak_headroom_freq_floor_pct;
	unsigned int		peak_headroom_starve_streak;
	unsigned int		peak_headroom_jump_pct;
	unsigned int		peak_headroom_hold_ms;

	/* Patch 1.2 batt_hold_scale_pct.  Defaults to 100 (identity).
	 * When the AC-vs-battery cache reports zenith_on_battery == 1,
	 * peak-rescue / peak-prearm hold deadlines are scaled by this
	 * percentage before being applied.  Profile-baked; see the
	 * comment block above zenith_apply_profile() for per-profile
	 * values.  Bounded to ZENITH_BATT_HOLD_SCALE_PCT_{MIN,MAX}.
	 */
	unsigned int		batt_hold_scale_pct;

	/* Wave A charger-aware floor.  See the comment block above
	 * ZENITH_DEFAULT_CHARGER_AWARE for the full rationale.  Both
	 * default 0 so a fresh boot is bit-identical to pre-Wave-A
	 * behaviour.  charger_aware == 0 short-circuits the tier;
	 * charger_floor_pct == 0 leaves the gate live (for tracepoint
	 * visibility) but applies no floor.  Bounded 0..1 and 0..100
	 * respectively on store.
	 */
	unsigned int		charger_aware;
	unsigned int		charger_floor_pct;

	/* Wave A cgroup-aware top-app floor.  See the comment block
	 * above ZENITH_DEFAULT_TOP_APP_AWARE for the full rationale.
	 * Both default 0 so a fresh boot is bit-identical to pre-Wave-A
	 * behaviour.  top_app_aware == 0 short-circuits the tier;
	 * top_app_floor_pct == 0 leaves the gate live (for tracepoint
	 * visibility) but applies no floor.  Bounded 0..1 and 0..100
	 * respectively on store.  Requires CONFIG_CPUSETS=y; on
	 * CONFIG_CPUSETS=n the helper always returns false and the
	 * gate is effectively a no-op.
	 */
	unsigned int		top_app_aware;
	unsigned int		top_app_floor_pct;

	/* Wave A render-thread util tracker.  See the comment block
	 * above ZENITH_DEFAULT_RENDER_THREAD_UTIL_AWARE for the full
	 * rationale.  All three default 0 so a fresh boot is bit-
	 * identical to pre-Wave-A behaviour.  The tier requires
	 * render_aware=1 to function (the comm-walk must run to
	 * observe util_avg).  Bounded 0..1, 0..1024, 0..100
	 * respectively on store.
	 */
	unsigned int		render_thread_util_aware;
	unsigned int		render_thread_util_thresh;
	unsigned int		render_thread_util_floor_pct;

	/* Wave B PMU IPC tracker.  See the comment block above
	 * ZENITH_DEFAULT_PMU_AWARE for the full rationale.  All three
	 * default 0 (except pmu_ipc_thresh which defaults to 100 /
	 * 1.0 IPC because a 0 threshold would be meaningless) so a
	 * fresh boot is bit-identical to pre-Wave-B behaviour and the
	 * tier is opt-in.  Bounded 0..1, 0..1000, 0..100 respectively
	 * on store.
	 */
	unsigned int		pmu_aware;
	unsigned int		pmu_ipc_thresh;
	unsigned int		pmu_ipc_floor_pct;

	/* Wave B EAS / Energy Model integration.  See the comment
	 * block above ZENITH_DEFAULT_EM_AWARE for the full rationale.
	 * Both default 0 so a fresh boot is bit-identical to pre-
	 * Wave-B behaviour.  Bounded 0..1 and 0..200 respectively on
	 * store.
	 */
	unsigned int		em_aware;
	unsigned int		em_floor_pct;

	/* Patch 1.3 cluster-wake-pulse.  See the comment block above
	 * ZENITH_DEFAULT_CLUSTER_WAKE_PULSE_MS for the full rationale.
	 * cluster_wake_pulse_ms == 0 disables the tier entirely (so
	 * BATTERY / LEGACY profiles short-circuit at no runtime cost).
	 * All three knobs are profile-baked.
	 */
	unsigned int		cluster_wake_pulse_ms;
	unsigned int		cluster_wake_pulse_idle_ms;
	unsigned int		cluster_wake_pulse_floor_pct;

	/* Patch 1.10 quiet-hours cap.  See the comment block above
	 * ZENITH_DEFAULT_QUIET_HOURS_START_MIN for the full rationale.
	 * quiet_hours_start_min == quiet_hours_end_min disables the
	 * tier (default).  cap_pct is the only profile-baked knob in
	 * this group; the start / end window is user-personal.
	 */
	unsigned int		quiet_hours_start_min;
	unsigned int		quiet_hours_end_min;
	unsigned int		quiet_hours_cap_pct;
	unsigned int		quiet_hours_screen_off_only;

	/* Patch 1.9 fg-transition pulse.  See the comment block
	 * above ZENITH_DEFAULT_FG_TRANSITION_PULSE_MS for the full
	 * rationale.  Both knobs profile-baked; fg_transition_-
	 * pulse_ms == 0 disables the producer (no deadline ever
	 * stamped); fg_transition_pulse_pct == 0 disables the
	 * consumer (deadline stamped but no floor applied).
	 */
	unsigned int		fg_transition_pulse_ms;
	unsigned int		fg_transition_pulse_pct;

	/* Pre-arm tier for the peak-headroom rescue.  When 1 (the
	 * default), an early softer intervention fires while the
	 * starvation streak is accumulating but has not yet crossed
	 * peak_headroom_starve_streak.  Specifically: if the previous
	 * sample was starving (peak_starve_count > 0) and the current
	 * cluster freq is still below an effective hispeed_freq value
	 * that would itself escape the starvation floor, lift freq up
	 * to that hispeed value before the rescue tier even runs.
	 *
	 * Effect: a graduated response curve.  Sample 1 of starvation
	 * pulls the cluster into the hispeed band; subsequent samples
	 * either clear (because hispeed was enough) or progress to the
	 * full rescue (because hispeed wasn't enough and the streak
	 * crosses).  The pre-arm self-disables the moment it actually
	 * works -- once freq >= floor_freq, the next sample's starve
	 * check evaluates false and peak_starve_count resets to 0,
	 * which is also the gate that lets the pre-arm fire, so the
	 * intervention is naturally one-shot per starvation episode.
	 *
	 * 0 disables the pre-arm: starvation episodes go directly to
	 * the full rescue after the streak hits.  Mostly useful for
	 * debugging or for tracker A/B comparisons of the rescue tier
	 * alone vs. the rescue+pre-arm pair.  Also automatically a
	 * no-op when hispeed_freq is unconfigured (eff_hispeed == 0)
	 * or when the configured eff_hispeed value is itself below the
	 * floor_freq (lifting to a sub-floor hispeed wouldn't escape
	 * starvation, so we let the rescue handle it).
	 */
	unsigned int		peak_headroom_prearm;

	/* Predictive up-shift trend gate (tier 2a').  See
	 * ZENITH_DEFAULT_PREDICT_UP_THRESH for full semantics.  When
	 * non-zero, zenith_get_next_freq() compares the rise across
	 * the last predict_up_window util samples against this
	 * threshold (expressed as 256ths of max_cap so the value is
	 * unitless: 64 == ~25%% of max_cap rise across the window).
	 * 0 disables the tier; max ZENITH_PREDICT_UP_THRESH_MAX (255)
	 * is the largest value the unitless trend can take, at which
	 * point the tier effectively never fires (would require a
	 * full max_cap rise across the window).
	 */
	unsigned int		predict_up_thresh;

	/* Window size for the predict_up trend ring, in samples.
	 * Must be in [ZENITH_PREDICT_UP_WINDOW_MIN ..
	 * ZENITH_PREDICT_UP_WINDOW_MAX].  The trend compare reads the
	 * util sample written predict_up_window ticks ago against the
	 * one written this tick; out-of-range values are rejected on
	 * sysfs store.  A larger window smooths PELT noise out of the
	 * trend signal at the cost of one more tick of warm-up after
	 * a cold attach.
	 */
	unsigned int		predict_up_window;

	/* See ZENITH_DEFAULT_PELT_RISING_EDGE_THRESH /
	 * ZENITH_DEFAULT_PELT_RISING_EDGE_MIN_PCT (Patch C3).  Single-
	 * sample slope tier that lifts to eff_hispeed_freq when the
	 * delta between the two most-recent util_history samples
	 * crosses pelt_rising_edge_thresh AND the newest sample is
	 * already above pelt_rising_edge_min_pct of max_cap.  0 in
	 * thresh disables the tier (legacy behaviour: only the
	 * rolling-window predict_up fires).
	 */
	unsigned int		pelt_rising_edge_thresh;
	unsigned int		pelt_rising_edge_min_pct;

	/* See ZENITH_DEFAULT_DL_TASK_FLOOR_PCT (Patch C6).  When a
	 * SCHED_DEADLINE task is present on any CPU in the policy,
	 * lift freq to (policy->max * dl_task_floor_pct / 100).  0
	 * disables the floor; non-zero in 1..100 sets the
	 * percentage of policy->max used as the floor.
	 */
	unsigned int		dl_task_floor_pct;

	/* See ZENITH_DEFAULT_IO_FLOOR_HYST_MS /
	 * ZENITH_DEFAULT_IO_FLOOR_HYST_PCT (Patch C9).  Either at 0
	 * disables the IO floor hysteresis tier.  Range checks:
	 * io_floor_hyst_ms in 0..ZENITH_IO_FLOOR_HYST_MS_MAX,
	 * io_floor_hyst_pct in 0..ZENITH_IO_FLOOR_HYST_PCT_MAX.
	 * Read/write via READ_ONCE / WRITE_ONCE (no torn-write
	 * hazard on word-sized scalars).
	 */
	unsigned int		io_floor_hyst_ms;
	unsigned int		io_floor_hyst_pct;

	/* See ZENITH_DEFAULT_PEAK_HYSTERESIS_STREAK /
	 * ZENITH_DEFAULT_PEAK_STEP_DOWN_PCT (Patch E).  Either at 0
	 * disables the tier (legacy descent-from-peak).  Range
	 * checks: streak in 0..ZENITH_PEAK_HYSTERESIS_STREAK_MAX,
	 * step_down_pct in 0..100.  Reads via READ_ONCE on the eval
	 * hot path; writes via WRITE_ONCE from sysfs and from
	 * zenith_apply_profile().
	 */
	unsigned int		peak_hysteresis_streak;
	unsigned int		peak_step_down_pct;

	/* See ZENITH_DEFAULT_BOOST_IDLE_THRESH /
	 * ZENITH_DEFAULT_BOOST_IDLE_STREAK (Patch F).  Either at 0
	 * disables the boost early-exit (boost is honoured to its
	 * deadline regardless of in-cluster idle).  Range checks:
	 * thresh in 0..100 (load percent), streak in
	 * 0..ZENITH_BOOST_IDLE_STREAK_MAX.  Reads via READ_ONCE on
	 * the eval hot path.
	 */
	unsigned int		boost_idle_thresh;
	unsigned int		boost_idle_streak;

	/* See ZENITH_DEFAULT_BG_UTIL_SCALE_PCT (Patch G).  Scales
	 * zenith_get_util() output down to this percent of natural
	 * util when tunables->screen_state == 0.  Range 1..100.
	 * 100 (default) is a no-op pass-through.  Reads via
	 * READ_ONCE on the eval hot path.
	 */
	unsigned int		bg_util_scale_pct;

	/* See ZENITH_DEFAULT_SLEEPER_TAIL_THRESH_US /
	 * ZENITH_DEFAULT_SLEEPER_TAIL_PCT (Patch H).  thresh_us == 0
	 * disables.  pct bounded in 50..100; eval path reads via
	 * READ_ONCE.
	 */
	unsigned int		sleeper_tail_thresh_us;
	unsigned int		sleeper_tail_pct;

	/* See ZENITH_DEFAULT_PEER_RAMP_WINDOW_MS /
	 * ZENITH_DEFAULT_PEER_RAMP_FLOOR_PCT (Patch D).
	 * peer_ramp_window_ms == 0 disables both sides of the multi-
	 * cluster pre-arm coordination.  peer_ramp_floor_pct == 0
	 * suppresses the floor while leaving the deadline writes in
	 * place.  Reads via READ_ONCE on the eval hot path; writes
	 * via WRITE_ONCE from sysfs and zenith_apply_profile().
	 */
	unsigned int		peer_ramp_window_ms;
	unsigned int		peer_ramp_floor_pct;

	/* See ZENITH_DEFAULT_PEER_RAMP_WINDOW_OFF_MS (Patch M3).
	 * Screen-state-aware shadow of peer_ramp_window_ms.  Read
	 * via READ_ONCE in zenith_peer_ramp_effective_window_ms()
	 * whenever the arm path or the floor-eval path needs the
	 * effective window length; written via WRITE_ONCE from sysfs
	 * and zenith_apply_profile().
	 */
	unsigned int		peer_ramp_window_off_ms;

	/* See the migration_* macro block (Patch K1).  jump_pct == 0
	 * disables both sides; floor_pct == 0 suppresses the floor
	 * while leaving the per-CPU stamping in place.  All three
	 * are READ_ONCE on the eval / per-CPU update_util paths.
	 */
	unsigned int		migration_jump_pct;
	unsigned int		migration_floor_window_ms;
	unsigned int		migration_floor_pct;

	/* See ZENITH_DEFAULT_PSI_CPU_FLOOR_THRESH (Patch K2).  0
	 * disables the tier.  Read once on the eval path; written
	 * via WRITE_ONCE from sysfs and zenith_apply_profile().
	 */
	unsigned int		psi_cpu_floor_thresh;

	/* See the frame_overrun_* macro block (Patch K3).
	 * slack_us == 0 disables stamping (no overrun event ever
	 * arms a deadline); floor_pct == 0 leaves stamping in
	 * place but suppresses the floor.  All three are READ_ONCE
	 * on the eval / vblank-event paths.
	 */
	unsigned int		frame_overrun_slack_us;
	unsigned int		frame_overrun_window_ms;
	unsigned int		frame_overrun_floor_pct;

	/* See ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_STREAK /
	 * ZENITH_DEFAULT_FRAME_OVERRUN_DEEP_FLOOR_PCT (Patch M5).
	 * Sub-knob inside K3.  deep_streak == 0 disables the deep
	 * tier (consumer never reads the streak atomic).  When
	 * armed, the deep tier amplifies an active K3 floor on
	 * sustained overrun runs.  Reads via READ_ONCE on the eval
	 * hot path; writes via WRITE_ONCE from sysfs and
	 * zenith_apply_profile().
	 */
	unsigned int		frame_overrun_deep_streak;
	unsigned int		frame_overrun_deep_floor_pct;

	/* See ZENITH_DEFAULT_PEER_RAMP_UCLAMP_MIN_RESPECT /
	 * ZENITH_DEFAULT_MIGRATION_FLOOR_UCLAMP_MIN_RESPECT
	 * (Patch M2).  Independent per-tier sub-gates: when set,
	 * the peer_ramp / migration_floor read sites compute their
	 * effective floor as max(static_pct, uclamp_min_pct).
	 * Default 1 because the uclamp path is a floor-raise; set
	 * 0 to revert to the static-pct-only Stage 4 behaviour for
	 * that specific tier without touching the master
	 * uclamp_min_respect gate.  Reads via READ_ONCE on the
	 * eval hot path; writes via WRITE_ONCE from sysfs.
	 */
	unsigned int		peer_ramp_uclamp_min_respect;
	unsigned int		migration_floor_uclamp_min_respect;

	/* See ZENITH_DEFAULT_PSI_MEM_CAP_* (Patch M1).  Three-tunable
	 * triple for the PSI-mem light cap tier.  All three reads
	 * are gated behind the existing psi_aware master gate AND
	 * the V2 PSI_MEM_CAP tier bit (via zenith_tier_value()).
	 * thresh == 0 short-circuits before any EWMA read, keeping
	 * the disabled path free.  Reads via READ_ONCE on the eval
	 * hot path; writes via WRITE_ONCE from sysfs and
	 * zenith_apply_profile().
	 */
	unsigned int		psi_mem_cap_thresh;
	unsigned int		psi_mem_cap_pct;
	unsigned int		psi_mem_cap_window_ms;

	/* Tail-decay window for the brutal-hold cliff exit, in
	 * milliseconds.  0 (default) preserves the historical hard-exit
	 * behaviour: the moment load_pct drops below the (possibly
	 * adaptive-shaped) eff_down threshold, brutal_active is cleared
	 * and the next sample's freq is whatever the EAS proportional
	 * math returns.  Non-zero arms a linear glide: policy->max at
	 * arm time, decaying toward the EAS-computed freq over
	 * brutal_decay_ms.  Eliminates the audible / visible drop that
	 * otherwise happens at the moment of cliff exit, especially on
	 * loads with bursty PELT signals.  Capped at
	 * ZENITH_BRUTAL_DECAY_MS_MAX in the sysfs store.
	 */
	unsigned int		brutal_decay_ms;

	/* Secondary up_threshold applied only when policy->cur has
	 * already climbed to hispeed_freq or above. 0 disables the
	 * substitution and falls back to up_threshold at every bin.
	 *
	 * The common shape is up_threshold=70 to get from idle to
	 * hispeed_freq aggressively, with up_threshold_hispeed=90 to
	 * demand a clearly heavier workload before committing to
	 * policy->max. Requires hispeed_freq != 0.
	 */
	unsigned int		up_threshold_hispeed;

	/* Alternative climb mechanism when load crosses up_threshold.
	 * SNAP (0) pins policy->max (the original ondemand-style
	 * behaviour). STEP (1) bumps target_freq by freq_step_pct
	 * percent of policy->max per sample, giving a slower,
	 * conservative-style climb. STEP mode bypasses the
	 * brutal_active / down_threshold hysteresis.
	 */
	unsigned int		climb_mode;
	unsigned int		freq_step_pct;

	/* Load-proportional scaling for the STEP climb step.  See
	 * ZENITH_DEFAULT_FREQ_STEP_ADAPTIVE.  0 = off (fixed step);
	 * 1 = on (step scales 1.0x .. 2.0x with overshoot).  Only
	 * meaningful when climb_mode == STEP.
	 */
	unsigned int		freq_step_adaptive;

	/* Last-applied preset, or CUSTOM if one was never written. The
	 * tunable does NOT auto-revert to CUSTOM when individual fields
	 * are later modified — the user can always check sysfs to see
	 * which recipe they last applied.
	 */
	unsigned int		active_profile;

	/* Patch B-AUTO-2: auto-selector meta-state.  See ZENITH_PROFILE_AUTO.
	 *
	 * auto_target: when active_profile == ZENITH_PROFILE_AUTO the
	 * decision engine (B-AUTO-3 / B-AUTO-4) writes the most
	 * recently picked concrete target here (BALANCED, PERFORMANCE,
	 * BATTERY, GAMING, or AUDIO) and applies that profile via
	 * zenith_apply_profile().  Default ZENITH_PROFILE_BALANCED so
	 * cold boot is on a known-safe baseline before the first auto
	 * eval lands.  When active_profile != AUTO this field is
	 * stale; readers must check active_profile first.  Read via
	 * READ_ONCE on any path that races with the auto worker;
	 * written via WRITE_ONCE from the worker and from
	 * profile_store on AUTO entry.
	 */
	unsigned int		auto_target;

	/* Patch B-AUTO-3: deferrable workqueue + hysteresis tracking
	 * for the auto-selector engine.
	 *
	 * eval_work runs on the system unbound deferrable workqueue at
	 * a (default 500 ms, profile-baked) cadence whenever
	 * active_profile == ZENITH_PROFILE_AUTO.  It reads device-wide
	 * signals (B-AUTO-4 adds the classifier; B-AUTO-3 stubs it to
	 * return BALANCED) and, after auto_pending_target has held
	 * steady for >= auto_hysteresis_ms, applies the new profile
	 * via zenith_apply_profile() and stamps auto_target.
	 *
	 * eval_work is initialised in the tunables alloc path and
	 * cancelled in the tunables free path.  It re-arms itself at
	 * the end of every run (modulo active_profile == AUTO, which
	 * is the engine's only off-switch).  It is *not* scheduled
	 * automatically at alloc time -- profile_store on AUTO entry
	 * is the only producer of an initial schedule, and B-AUTO-5
	 * adds a cold-boot AUTO flip that triggers that path.
	 *
	 * Concurrency: the worker takes attr_set->update_lock around
	 * any mutation of profile-bake fields (zenith_apply_profile,
	 * zenith_refresh_rate_delays, zenith_invalidate_cache) so the
	 * existing sysfs profile_store path stays race-free.  The
	 * worker reads READ_ONCE(active_profile) outside the lock as
	 * an early-out so the cancel-in-progress path doesn't deadlock
	 * on update_lock with profile_store waiting for
	 * cancel_delayed_work_sync.
	 *
	 * auto_pending_target / auto_pending_first_seen_ns implement
	 * the hysteresis: when the classifier returns a different
	 * target than auto_target the new value lands in
	 * auto_pending_target with a fresh first_seen timestamp; if
	 * the same value re-appears in subsequent windows and
	 * (now - first_seen) >= auto_hysteresis_ms we commit.  When
	 * the classifier flaps back to auto_target the pending state
	 * resets.
	 */
	struct delayed_work	eval_work;
	unsigned int		auto_pending_target;
	u64			auto_pending_first_seen_ns;

	/* Patch B-AUTO-3: cadence + hysteresis tunables for the
	 * auto-selector engine.  See ZENITH_DEFAULT_AUTO_EVAL_MS and
	 * ZENITH_DEFAULT_AUTO_HYSTERESIS_MS for default rationale.
	 * Bounded by ZENITH_AUTO_EVAL_MS_{MIN,MAX} and
	 * ZENITH_AUTO_HYSTERESIS_MS_{MIN,MAX} on sysfs writes.
	 */
	unsigned int		auto_eval_ms;
	unsigned int		auto_hysteresis_ms;

	/* Permille (0..1000) of SCHED_CAPACITY_SCALE at which
	 * zenith_iowait_boost() arms and below which a doubling
	 * decay exits the boost. 125 (12.5%) matches the legacy
	 * SCHED_CAPACITY_SCALE / 8 constant. 0 disables the minimum
	 * floor but still allows the doubling climb to take effect
	 * from its first sample; use io_is_busy=0 to disable iowait
	 * boost wholesale instead.
	 */
	unsigned int		iowait_boost_min;

	/* Percentage (0..100) of the iowait-derived boost that is added
	 * to util when stacking is active.  0 is the legacy behaviour
	 * (max(util, boost)); 100 is full stacking (util + boost, clamped
	 * to max_cap).  The default 50 is a conservative middle ground:
	 * when a CPU is already under compute load and also servicing
	 * I/O, it gets a half-weighted boost on top of its current util
	 * rather than the original "freq takes whichever is larger" which
	 * loses the I/O signal entirely when util is high.  Uses
	 * upstream 6.x schedutil's observation that a task that's both
	 * CPU-heavy and I/O-heavy needs more headroom than either demand
	 * alone would justify.
	 */
	unsigned int		iowait_stack_pct;

	/* See ZENITH_DEFAULT_IOWAIT_BACKOFF_AFTER_MS.  Time in ms after
	 * the start of a single iowait episode before the doubling-on-
	 * arm climb flips to a halving step.  0 disables the backoff.
	 */
	unsigned int		iowait_backoff_after_ms;

	/* When 1, a per-policy delayed_work periodically classifies the
	 * recent workload from load-saturation rate and input-event
	 * rate, and auto-selects performance / balanced / battery via
	 * zenith_apply_profile(). Default 0 (off).
	 */
	unsigned int		auto_tune;
	unsigned int		powersave_bias;

	/* Screen-on multiplier applied to powersave_bias before tier 3
	 * (Powersave Bias) consumes it.  Range 0..100 (percent).  When
	 * tunables->screen_state == 1 and the screen-off / thermal
	 * overrides have not replaced dynamic_bias, the effective
	 * dynamic_bias is scaled by screen_on_bias_pct / 100.  See
	 * ZENITH_DEFAULT_SCREEN_ON_BIAS_PCT for the full rationale.
	 *
	 * 100 disables the softening entirely (legacy behaviour: full
	 * configured bias applies on screen-on); 50 (the default)
	 * halves the bias on screen-on; 0 zeroes the bias on
	 * screen-on (full responsiveness, no shave).  Values above 100
	 * are rejected on store; the range is clamped on the read side
	 * defensively in case userspace bypasses the sysfs validator.
	 */
	unsigned int		screen_on_bias_pct;

	unsigned int		io_is_busy;

	/* When 1, dampen the brutality-path load_pct by the fraction
	 * of recent CPU time spent in niced-user mode. Approximation
	 * of ondemand's ignore_nice_load for a PELT-based governor.
	 */
	unsigned int		ignore_nice_load;

	/* Zenith Environment API */
	unsigned int		screen_state;   /* 1 = ON, 0 = OFF */

	/* Soft-glide window for the 1 -> 0 transition on screen_state.
	 * 0 (default) preserves the legacy hard cliff: as soon as
	 * screen_state is observed at 0, dynamic_up_thresh snaps to
	 * 95 and dynamic_bias snaps to 500 (the 50%% powersave
	 * penalty).  Non-zero arms a linear ramp on both: starting
	 * from the natural up_threshold / powersave_bias at the
	 * moment screen_state went to 0, climbing to the cliff
	 * targets over screen_off_glide_ms.  Eliminates the cliff
	 * that otherwise lands the moment AOD / panel-blank handlers
	 * stamp screen_state=0 while userspace work is still
	 * winding down.  Capped at ZENITH_SCREEN_OFF_GLIDE_MS_MAX in
	 * the sysfs store.
	 */
	unsigned int		screen_off_glide_ms;

	/* When 1, zenith subscribes to the fb notifier chain and updates
	 * screen_state automatically on FB_EVENT_BLANK. screen_state
	 * written from userspace still takes effect and is only
	 * overridden on the next blank/unblank event.
	 */
	unsigned int		screen_auto;

	unsigned int		thermal_state;  /* 0 = COOL, 1 = THROTTLING */

	/* When 1, thermal_state is additionally inferred from
	 * arch_scale_thermal_pressure() on every update_util tick.
	 * thermal_state=1 written from userspace still forces it.
	 */
	unsigned int		thermal_auto;

	/* See ZENITH_DEFAULT_THERMAL_AWARE comment block.  Master gate
	 * for the cluster of thermal mechanisms (thermal_util_derate,
	 * the thermal_pressure_continuous up_thresh ramp,
	 * auto_thermal_cap, the V2 THERMAL_RECOVERY transitions, and
	 * zenith_thermal_active() itself).  Default 1.  Strict 0/1.
	 * Static-key gated via zenith_thermal_aware_key.
	 */
	unsigned int		thermal_aware;

	/* RO sysfs mirror.  Set on each call to zenith_thermal_active()
	 * to the function's return value, so userspace can observe
	 * whether the governor currently believes thermal pressure is
	 * high enough to justify the cluster of thermal mechanisms.
	 * Multiple policies may stomp on this from their own update
	 * paths; that race is benign because the field is observability
	 * only and not consumed by any decision tier.  Always 0 when
	 * thermal_aware is 0 (zenith_thermal_active() short-circuits
	 * before writing).
	 */
	unsigned int		thermal_active;

	/* When 1, replace the binary "thermal_active -> dynamic_up_thresh
	 * = 90" cliff with a linear ramp from up_threshold (cool, 0%
	 * pressure) to 90 (hot, 100% pressure).  Smooths long-session
	 * thermal throttling so users don't perceive a step in
	 * frequency the moment thermal_active flips on.  Default 0 to
	 * preserve legacy cliff behaviour for existing tunings.
	 */
	unsigned int		thermal_pressure_continuous;

	/* prefer_silver_aware: when 1, on big / prime cluster policies,
	 * raise dynamic_up_thresh by prefer_silver_hot_bump_pct percent
	 * (additive points) whenever the prefer_silver hit-rate over
	 * the last classifier window is at or above
	 * prefer_silver_hot_threshold_pct.  The intent is to reflect
	 * the fact that prefer_silver hides light load from the big
	 * cluster, so the big cluster sees an artificially "lighter"
	 * load and would otherwise downclock more aggressively than it
	 * should.  Little cluster policies are intentionally not bumped
	 * (they are already absorbing the redirected light tasks).
	 *
	 * Default 0 (off).  Has no effect when
	 * CONFIG_SCHED_PREFER_SILVER=n: the worker does not import
	 * prefer_silver_get_hit_miss() in that build, so the cached
	 * hit-rate stays at 0 and the bump never fires regardless of
	 * the tunable value.
	 */
	unsigned int		prefer_silver_aware;
	unsigned int		prefer_silver_hot_threshold_pct;
	unsigned int		prefer_silver_hot_bump_pct;

	/* See ZENITH_DEFAULT_THERMAL_UTIL_DERATE comment block.  When
	 * set, zenith_get_util() scales util_out by the (cap - pressure)
	 * / cap fraction whenever pressure exceeds
	 * ZENITH_THERMAL_DERATE_FLOOR_PCT.  Smooths the thermal dance
	 * by asking for less than the throttled max.
	 */
	unsigned int		thermal_util_derate;

	/* See ZENITH_DEFAULT_THERMAL_DERATE_RATE_PCT.  Magnitude of the
	 * derivative-term scaling applied on top of thermal_util_derate
	 * when pressure is rising sample-to-sample, in percent.  0..100,
	 * 0 disables.  Read lock-free in the derate site.
	 */
	unsigned int		thermal_derate_rate_pct;

	/* See ZENITH_DEFAULT_AUTO_THERMAL_CAP comment block.  Boolean
	 * gate; when 1, applies a hard cap on target_freq once the
	 * per-policy thermal pressure crosses auto_thermal_cap_-
	 * pressure_pct.  Default 0; flip to 1 via sysfs to opt in.
	 * Read with READ_ONCE in the eval-path tier so the lockless
	 * sysfs writer cannot tear the gate value across CPUs.
	 */
	unsigned int		auto_thermal_cap;

	/* Threshold in percent of arch_scale_thermal_pressure() at
	 * which auto_thermal_cap fires.  Bounded
	 * ZENITH_AUTO_THERMAL_CAP_PRESSURE_PCT_{MIN,MAX} (1..100).
	 * Read in the eval-path tier with READ_ONCE alongside
	 * auto_thermal_cap and auto_thermal_cap_freq_pct.
	 */
	unsigned int		auto_thermal_cap_pressure_pct;

	/* Cap as a percent of policy->max applied when auto_thermal_-
	 * cap fires.  Bounded ZENITH_AUTO_THERMAL_CAP_FREQ_PCT_{MIN,
	 * MAX} (50..100).  The min bound (50) prevents an accidental
	 * "= 0" from zeroing the cluster.
	 */
	unsigned int		auto_thermal_cap_freq_pct;

	/* See ZENITH_DEFAULT_FREQ_STABILITY_MARGIN_PCT.  Percent of
	 * policy->max below which tiny downward transitions are held at
	 * the current request.  0 disables; range 0..10.
	 */
	unsigned int		freq_stability_margin_pct;

	/* See ZENITH_DEFAULT_DOWN_RATE_ADAPTIVE.  When set, bursty load
	 * variance stretches the effective down-rate delay up to 2x.
	 */
	unsigned int		down_rate_adaptive;

	/* See ZENITH_DEFAULT_WAKEUP_BOOST.  When set, idle-to-busy
	 * transitions arm a short up-rate bypass countdown.
	 */
	unsigned int		wakeup_boost;

	/* Time-based extension of wakeup_boost.  When 0 (default), the
	 * legacy tick-based ZENITH_WAKEUP_BOOST_TICKS countdown is used
	 * verbatim.  When non-zero, the detection sites additionally
	 * arm a per-CPU deadline at now + wakeup_boost_ms; the rate-
	 * limit bypass remains active until either the tick counter
	 * reaches zero AND the deadline has expired.  Lets userspace
	 * tune the bypass duration in real wall-clock time, regardless
	 * of how often zenith_freq_throttle() actually runs (which on
	 * Android can vary widely with up_rate_limit and topology).
	 * Capped at ZENITH_WAKEUP_BOOST_MS_MAX in the sysfs store.
	 */
	unsigned int		wakeup_boost_ms;

	/* See ZENITH_DEFAULT_DOWN_THRESHOLD_ADAPTIVE.  Percent by which
	 * bursty load can lower the brutality exit threshold.  0 disables.
	 */
	unsigned int		down_threshold_adaptive;

	/* See ZENITH_DEFAULT_RATE_LIMIT_CLUSTER_SCALE.  When set, little
	 * clusters use asymmetric cached up/down rate delays.
	 */
	unsigned int		rate_limit_cluster_scale;

	/* Input boost duration (ms). 0 = disabled. */
	unsigned int		input_boost_ms;
	unsigned int		input_boost_decay_ms;

	/* Touchdown-vs-coordinate-stream extra window for the input
	 * boost (Patch C).  Range
	 * 0..ZENITH_INPUT_BOOST_TOUCHDOWN_EXTRA_MS_MAX.  See
	 * ZENITH_DEFAULT_INPUT_BOOST_TOUCHDOWN_EXTRA_MS for semantics.
	 * Mirrored to zenith_input_boost_touchdown_extra_ms_cache on
	 * store and on profile apply, read by zenith_input_event() on
	 * the EV_KEY/BTN_TOUCH press path.
	 */
	unsigned int		input_boost_touchdown_extra_ms;

	/* Shape of the input_boost decay-phase floor.  0 = linear
	 * (legacy), 1 = cubic ease-in (floor holds high, drops fast at
	 * the tail).  See ZENITH_DEFAULT_INPUT_BOOST_DECAY_CURVE for
	 * semantics.  Range-checked 0..1 on store.
	 */
	unsigned int		input_boost_decay_curve;

	/* When 1 (default), input boost is applied only to policies whose
	 * top arch_scale_cpu_capacity equals SCHED_CAPACITY_SCALE -- i.e.
	 * the system's highest-capacity cluster(s).  On a homogeneous SoC
	 * every policy matches and behaviour is unchanged.  On big /
	 * little, this stops every touch from dragging the little cluster
	 * to policy->max, which rarely helps UI latency (UI / render run
	 * on big) and wastes energy on a cluster that is almost always in
	 * light-load territory at the moment of a tap.
	 *
	 * Set 0 to restore pre-patch behaviour (boost every policy).
	 */
	unsigned int		input_boost_big_only;

	/* Per-profile cap on the input-boost ceiling, expressed as a
	 * percentage of policy->max.  When non-zero, the full-pin phase
	 * targets policy->max * input_boost_cap_pct / 100 instead of
	 * policy->max, and the trailing decay phase ramps from that
	 * capped ceiling down to policy->min over input_boost_decay_ms.
	 * 0 (legacy / PERFORMANCE) means "no cap" -- pin to policy->max.
	 * Sized per-profile so BALANCED / BATTERY get a moderate boost
	 * (PELT can still climb past it under genuine load via the
	 * normal eval tiers, since the boost only sets a floor in the
	 * decay window) without spending the energy of a max-pin on
	 * every tap.  Range 0..100; values > 100 rejected by sysfs.
	 */
	unsigned int		input_boost_cap_pct;

	/* Multiplier for the effective down-rate delay applied to the
	 * cluster while an input boost is active (now <
	 * zenith_input_boost_until_ns).  Range 100..1000 (percent).
	 * 100 disables the extension and restores the legacy
	 * "down_rate_delay during boost == down_rate_delay outside
	 * boost" behaviour.  See ZENITH_DEFAULT_INPUT_BOOST_DOWN_RATE_-
	 * MULT_PCT for the full rationale; the extension is gated by
	 * input_boost_big_only the same way the boost itself is, so
	 * configurations that suppress the boost on LITTLE also keep
	 * LITTLE on its normal down-rate cadence.
	 */
	unsigned int		input_boost_down_rate_mult_pct;

	/* Efficient-frequency soft-cap ladder, up to ZENITH_EFF_BINS_MAX
	 * entries. Sorted ascending by frequency. The up_delay_us array
	 * is paired 1:1 with eff_freq; writing a single scalar to
	 * up_delay_us broadcasts it to every bin.
	 *
	 * eff_nr == 0 disables the ladder entirely (equivalent to the
	 * pre-ladder "efficient_freq=0" state). The scalar shadows
	 * efficient_freq / up_delay_us remain for backward-compatible
	 * sysfs reads: efficient_freq = eff_freq[0], up_delay_us =
	 * up_delay[0].
	 */
	unsigned int		efficient_freq;
	unsigned int		up_delay_us;
	unsigned int		eff_nr;

	/* See ZENITH_DEFAULT_EFF_BIN_HYST_PCT.  Per-bin release margin
	 * for the efficient_freq ladder, in percent.  0..20.  0 keeps
	 * the legacy "any drop releases" shape.
	 */
	unsigned int		eff_bin_hyst_pct;
	unsigned int		eff_freq[ZENITH_EFF_BINS_MAX];
	unsigned int		eff_delay_us[ZENITH_EFF_BINS_MAX];

	/* Light-load hard cap. light_load_freq=0 disables. */
	unsigned int		light_load_freq;
	unsigned int		light_load_threshold;	/* in % of max_cap */

	/* Hold-at-max multiplier for down_rate_limit. 1 = disabled. */
	unsigned int		sampling_down_factor;

	/* When 1 (default), keep the sampling_down_factor multiplier in
	 * effect for one stretched down-rate window after an input boost
	 * exits, even after target_freq has fallen below policy->max.  The
	 * stretched window is sampling_down_factor * down_rate_limit_us
	 * long.  Mirrors the rationale for sit-at-max stretching: PELT
	 * needs a moment to catch up after a synthetic peak (the boost),
	 * and dropping the multiplier the instant target falls produces
	 * a tail-stutter on gestures whose load profile is bursty around
	 * the boost expiry.  Set 0 to restore pre-patch behaviour.
	 */
	unsigned int		boost_exit_extend;

	/* powersave_bias only applies below this load (% of max_cap).
	 * 100 = always apply (legacy behaviour); 0 = never apply.
	 */
	unsigned int		bias_load_threshold;

	/* auto_tune observer thresholds (see per-field comments at the
	 * ZENITH_DEFAULT_AT_* macros). All are in percent except the
	 * events_x2 pair, which are integer event counts per 2 seconds
	 * over the classification window.
	 */
	unsigned int		auto_tune_sat_load_pct;
	unsigned int		auto_tune_hi_sat_pct;
	unsigned int		auto_tune_lo_sat_pct;
	unsigned int		auto_tune_hi_events_x2;
	unsigned int		auto_tune_lo_events_x2;
	/* F2: util-rising trend threshold, percent.  When the policy-
	 * wide PELT util sum grows by more than this percentage between
	 * consecutive V1 windows, the worker raises ZENITH_AT_FLAG_
	 * UTIL_RISING.  Default 25 (i.e. >=25% increase in window-to-
	 * window mean util).  Setting to 0 disables the signal entirely
	 * (the flag never fires) without removing it from the surface.
	 * Range: 0..200.
	 */
	unsigned int		auto_tune_util_rising_thresh_pct;
	unsigned int		auto_tune_render_rt_floor_pct;

	/* See ZENITH_DEFAULT_AUTO_TUNE_V2.  V2 keeps auto-tune bounded by
	 * profile guardrails, hysteresis/cooldown and user override masks.
	 */
	unsigned int		auto_tune_v2;
	unsigned int		auto_tune_hysteresis_windows;
	unsigned int		auto_tune_cooldown_windows;
	/* See ZENITH_DEFAULT_AT_V2_VAR_PROMOTE_THRESH.  load_var_ewma_x256
	 * threshold above which V2 promotes LATENCY to SUSTAINED_PERF on
	 * the variance signal alone.  0 disables variance-driven promotion.
	 */
	unsigned int		auto_tune_v2_var_promote_thresh;

	/* See ZENITH_DEFAULT_AUTO_TUNE_V3 comment block.  0/1/2 master
	 * gate for V3 self-calibration; the *_store callback also
	 * syncs the zenith_auto_tune_v3_key static key (FALSE when 0).
	 * auto_tune_v3_interval_ms is the calibration period; clamped
	 * to [ZENITH_AT_V3_INTERVAL_MIN_MS, ZENITH_AT_V3_INTERVAL_MAX_MS]
	 * on store.
	 */
	unsigned int		auto_tune_v3;
	unsigned int		auto_tune_v3_interval_ms;
	unsigned long		auto_tune_override_mask;
	unsigned int		auto_tune_cluster_aware;
	unsigned int		auto_tune_v2_signals;
	unsigned int		auto_tune_thermal_slope;
	unsigned int		auto_tune_thermal_pressure_pct;
	unsigned int		auto_tune_thermal_slope_pct;
	unsigned int		auto_tune_frame_pacing;
	unsigned int		auto_tune_sustained_gaming;

	/* See ZENITH_DEFAULT_AUTO_TUNE_V2_GLIDES.  When 1 (default), the
	 * V2 worker populates per-policy effective values for the
	 * round-U-z10 glide / coordination knobs (brutal_decay_ms,
	 * wakeup_boost_ms, boot_boost_decay_ms, screen_off_glide_ms,
	 * thermal_pressure_continuous, prefer_silver_aware,
	 * frame_budget_us_auto) based on V2 state.  Consumers use the
	 * effective value only when the user-set tunable is 0.  0 here
	 * locks all seven back to legacy behaviour exactly.  Ignored
	 * unless auto_tune_v2 is also 1.
	 */
	unsigned int		auto_tune_v2_glides;

	/* See ZENITH_DEFAULT_AUTO_TUNE_V2_TIERS comment block (Patch L).
	 * When 1 (default), the V2 worker writes a per-policy bitmask
	 * of *armed* Stage-4 floor tiers (K1 / K2 / K3) based on the
	 * just-resolved V2 state plus the FRAME / GAME flags.  K1/K2/K3
	 * read sites consult the bitmask via zenith_tier_value() and
	 * treat the underlying tunable as 0 (off) when the matching
	 * tier bit is clear.  User sysfs writes to any of the seven
	 * tier knobs set per-knob bits in auto_tune_override_mask;
	 * those bits suppress the V2 gating and the read returns the
	 * tunable value directly (Profile changes clear the entire
	 * mask, the existing zenith_apply_profile() behaviour, so a
	 * profile flip rearms V2).  Set to 0 to revert all three tiers
	 * to pure profile-driven behaviour.  Ignored unless
	 * auto_tune_v2 is also 1.
	 */
	unsigned int		auto_tune_v2_tiers;

	/* See ZENITH_DEFAULT_AUTO_TUNE_SCENARIO comment block.  Master
	 * gate for the scenario overlay applied on top of the vanilla
	 * load + input-rate classifier in zenith_auto_tune_work().
	 * Requires auto_tune=1; ignored otherwise.
	 */
	unsigned int		auto_tune_scenario;

	/* kcpustat hispeed blend tunables (see ZENITH_DEFAULT_KCPUSTAT_*
	 * comments for semantics). All three default to safe values:
	 * sampler runs at 4 ms windows with EWMA shift=1, but the blend
	 * is gated off until userspace flips kcpustat_hispeed_enable.
	 */
	unsigned int		kcpustat_window_us;
	unsigned int		kcpustat_filter_shift;
	unsigned int		kcpustat_hispeed_enable;

	/* util_math_v2: 0 (default) keeps the historical
	 * cpu_util_cfs() input unchanged; 1 enables the
	 * 6.x-style runnable-aware util computation in
	 * zenith_get_util().
	 */
	unsigned int		util_math_v2;

	/* uclamp_min_respect: see the ZENITH_DEFAULT_UCLAMP_MIN_RESPECT
	 * block at the top of this file for full semantics. In short:
	 *   1 (default) = apply an explicit final-freq floor derived
	 *                  from RQ-aggregated uclamp_min, AND suppress
	 *                  the screen-off penalty when a meaningful
	 *                  uclamp_min is set;
	 *   0           = rely solely on schedutil_cpu_util() RQ uclamp
	 *                  aggregation (pre-G.1 behaviour).
	 */
	unsigned int		uclamp_min_respect;
	unsigned int		uclamp_max_respect;

	/* See ZENITH_DEFAULT_PREDICT_UTIL_PCT comment block. 0 = off. */
	unsigned int		predict_util_pct;

	/* See ZENITH_DEFAULT_PREDICT_UTIL_SMOOTH comment block.  0 = off
	 * (single-tap), 1 = on (two-tap averaged slope).  Ignored when
	 * predict_util_pct == 0.
	 */
	unsigned int		predict_util_smooth;

	/* See ZENITH_DEFAULT_RENDER_AWARE / ZENITH_DEFAULT_RENDER_FLOOR_PCT. */
	unsigned int		render_aware;
	unsigned int		render_floor_pct;

	/* See ZENITH_DEFAULT_RENDER_FLOOR_MIN_RUNTIME_MS.  Debounce
	 * window in milliseconds.  Reads via READ_ONCE on the eval
	 * hot path; writes via WRITE_ONCE from sysfs and from
	 * zenith_apply_profile().  Range
	 * 0..ZENITH_RENDER_FLOOR_MIN_RUNTIME_MS_MAX.
	 */
	unsigned int		render_floor_min_runtime_ms;

	/* See ZENITH_DEFAULT_GAME_MODE. 0/1, normalised on store. */
	unsigned int		game_mode;

	/* See ZENITH_DEFAULT_GAME_AUTO comment block.  0/1 master gate
	 * for the in-kernel game detector.  Flipped via the matching
	 * sysfs node; the *_store callback also syncs the
	 * zenith_game_auto_key static key (FALSE when scalar = 0).
	 */
	unsigned int		game_auto;

	/* See ZENITH_DEFAULT_PSI_AWARE / ZENITH_DEFAULT_PSI_MEM_THRESH /
	 * ZENITH_DEFAULT_PSI_CPU_THRESH / ZENITH_DEFAULT_PSI_IO_THRESH.
	 * All thresholds are 0..100 integer percent of the 10s SOME
	 * average.  0 disables that dimension's cap.
	 */
	unsigned int		psi_aware;
	unsigned int		psi_mem_thresh;
	unsigned int		psi_cpu_thresh;
	unsigned int		psi_io_thresh;

	/* See ZENITH_PSI_CGROUP_PATH_MAX (Patch B10-3).  Empty string =
	 * use system-wide PSI (psi_system); non-empty cgroup-v2 path =
	 * resolve via cgroup_get_from_path() at sysfs-store time and
	 * cache the resolved cgroup in zenith_psi_cgroup for the three
	 * zenith_psi_*_some_pct() helpers to read under rcu_read_lock().
	 *
	 * Buffer is inline (no kmalloc) so tunables free is unchanged
	 * (kfree(t) handles it).  All cross-policy state (the cached
	 * cgroup ref + the active_path no-op cache) lives in the file-
	 * scope zenith_psi_cgroup_* statics.  Last-writer-wins across
	 * policies; in practice all policies converge to the same path
	 * via zenith_apply_profile().
	 */
	char			psi_cgroup_path[ZENITH_PSI_CGROUP_PATH_MAX];

	/* See ZENITH_DEFAULT_AUDIO_AWARE / ZENITH_DEFAULT_AUDIO_FLOOR_PCT
	 * / ZENITH_DEFAULT_AUDIO_CAP_PCT.  Both *_pct fields range 0..100;
	 * 0 in either disables that side of the band.
	 */
	unsigned int		audio_aware;
	unsigned int		audio_floor_pct;
	unsigned int		audio_cap_pct;
	unsigned int		audio_hyst_ms;

	/* See ZENITH_DEFAULT_CAMERA_AWARE / ZENITH_DEFAULT_CAMERA_ACTIVE
	 * / ZENITH_DEFAULT_CAMERA_FLOOR_PCT.  camera_active is the
	 * userspace override (0 auto, 1 force-on, 2 force-off);
	 * camera_floor_pct ranges 0..100.
	 */
	unsigned int		camera_aware;
	unsigned int		camera_active;
	unsigned int		camera_floor_pct;

	/* See ZENITH_DEFAULT_BOOT_COMPLETE_AUTO comment block.  Gates
	 * the in-kernel calm-detect arm of the boot_complete latch in
	 * zenith_auto_tune_work().  Default 1.  Setting to 0 disables
	 * the in-kernel arm; userspace writes to the boot_complete
	 * sysfs knob still work.
	 */
	unsigned int		boot_complete_auto;

	/* See ZENITH_DEFAULT_BOOT_BOOST_MS. 0 disables the one-shot. */
	unsigned int		boot_boost_ms;

	/* Trailing decay window for boot_boost, in milliseconds.  When 0
	 * (default), the boot-boost ends as a hard cliff at
	 * boot_boost_ms.  When non-zero, after boot_boost_ms expires
	 * zenith_get_next_freq() applies a linear floor that ramps from
	 * policy->max down to policy->min over boot_boost_decay_ms,
	 * mirroring input_boost_decay_ms's tail behaviour.  Useful when
	 * userspace boot animations or surface-flinger initial paints
	 * land just past boot_boost_ms and would otherwise see the
	 * sudden drop.  Capped at ZENITH_BOOT_BOOST_DECAY_MS_MAX.
	 */
	unsigned int		boot_boost_decay_ms;

	/* See ZENITH_DEFAULT_FRAME_BUDGET_US. Userspace writes the
	 * current vblank period in microseconds.  0 disables.
	 */
	unsigned int		frame_budget_us;

	/* See ZENITH_DEFAULT_FRAME_BUDGET_US_AUTO.  When 1, the adaptive
	 * frame-budget floor uses the cached drm-side vblank period
	 * (zenith_drm_vblank_us) instead of frame_budget_us /
	 * frame_budget_us_per_policy whenever the cached value is
	 * non-zero.  Lets a drm panel driver populate the rate without
	 * a userspace round-trip.  0 (default) preserves the legacy
	 * userspace-driven behaviour exactly.
	 */
	unsigned int		frame_budget_us_auto;

	/* See ZENITH_DEFAULT_FRAME_BUDGET_US/ frame_budget_us_per_policy
	 * comment block.  Indexed by cpumask_first(policy->cpus).  A
	 * non-zero entry overrides frame_budget_us for that policy;
	 * zero falls through to the global value.  All entries default
	 * to 0 (no override) at tunables_init() time.  Read lock-free
	 * in zenith_get_next_freq() under READ_ONCE; written under
	 * global_tunables_lock by the sysfs store path.
	 */
	unsigned int		frame_budget_us_per_policy[NR_CPUS];
	unsigned int		frame_pace_floor_pct;

	/* See ZENITH_DEFAULT_VH_ARCH_FREQ_SCALE_ENABLE (Patch B9-1).
	 * Master 0/1 gate for the android_vh_arch_set_freq_scale
	 * vendor-hook observer.  When 0 the registered probe is a
	 * single-branch no-op; when 1 it caches the realised cluster
	 * freq scale and pre-arms peer_ramp on the local cluster.
	 * Read via READ_ONCE on the hot probe path; written via
	 * WRITE_ONCE from sysfs and from zenith_apply_profile().
	 */
	unsigned int		vh_arch_freq_scale_enable;

	/* See ZENITH_DEFAULT_VH_UCLAMP_OBSERVER_ENABLE (Patch B9-2).
	 * Master 0/1 gate for the android_vh_setscheduler_uclamp
	 * vendor-hook observer.  When 0 the registered probe is a
	 * single-branch no-op; when 1 a userspace uclamp_min raise on
	 * a task running in a zenith-driven policy synchronously arms
	 * peer_ramp on that policy's peer cluster (no PELT lag).
	 * Read via READ_ONCE on the probe path; written via
	 * WRITE_ONCE from sysfs and from zenith_apply_profile().
	 */
	unsigned int		vh_uclamp_observer_enable;

	/* See ZENITH_DEFAULT_VH_CPU_IDLE_ENABLE (Patch B9-3).
	 * Master 0/1 gate for the android_vh_cpu_idle_enter /
	 * android_vh_cpu_idle_exit vendor-hook observer pair.  When 0
	 * both registered probes are single-branch no-ops; when 1 the
	 * exit probe stamps idle residency into z_policy->vh_cpu_idle_-
	 * last_residency_ns, and zenith_get_next_freq() suppresses
	 * cluster_wake_pulse arming when that residency exceeds
	 * ZENITH_VH_CPU_IDLE_RESIDENCY_LONG_NS.  Read via READ_ONCE on
	 * both the probe path and the cwp arm site; written via
	 * WRITE_ONCE from sysfs and from zenith_apply_profile().
	 */
	unsigned int		vh_cpu_idle_enable;

	/* See ZENITH_DEFAULT_VH_FREQ_QOS_ENABLE (Patch B9-3+).
	 * Master 0/1 gate for the android_vh_freq_qos_update_request
	 * vendor-hook observer.  When 0 the registered probe is a
	 * single-branch no-op and the auto-classify consume side
	 * short-circuits; when 1 a high-min FREQ_QOS update against a
	 * zenith-driven policy stamps vh_freq_qos_pressure_until_ns and
	 * zenith_auto_classify() pivots to PERFORMANCE while the
	 * timestamp is still ahead of ktime_get_ns().  Read via
	 * READ_ONCE on both the probe path and the auto-classify
	 * consumer; written via WRITE_ONCE from sysfs and from
	 * zenith_apply_profile().
	 */
	unsigned int		vh_freq_qos_enable;

	/* Per-tunables freq-QoS pressure window (Patch B9-3+).
	 * Set by zenith_probe_freq_qos_update_request() to
	 * ktime_get_ns() + ZENITH_VH_FREQ_QOS_WINDOW_MS * NSEC_PER_MSEC
	 * on each high-min FREQ_QOS hit; read by
	 * zenith_auto_classify() against ktime_get_ns() to determine
	 * whether the AUTO selector should bias to PERFORMANCE.
	 * atomic64_t so the producer (probe in arbitrary scheduler
	 * context) and consumer (auto-eval worker, sleepable context)
	 * race-free without a governor lock.  Initialised to 0 by
	 * kzalloc(); 0 means "no pressure" because every real hit
	 * stamps a value > 0.
	 */
	atomic64_t		vh_freq_qos_pressure_until_ns;

	/* See ZENITH_DEFAULT_VH_SCHED_MOVE_TASK_ENABLE (Patch B9-5).
	 * Master 0/1 gate for the android_vh_sched_move_task vendor-
	 * hook observer.  When 0 the registered probe is a single-
	 * branch no-op; when 1 every cgroup move that lands a task on
	 * a CPU belonging to a zenith-driven policy stamps jiffies on
	 * z_policy->vh_sched_move_task_last_jiffies.  Read via
	 * READ_ONCE on the probe path; written via WRITE_ONCE from
	 * sysfs and from zenith_apply_profile().
	 */
	unsigned int		vh_sched_move_task_enable;

	/* See ZENITH_DEFAULT_VH_SCHEDULER_TICK_ENABLE (Patch B9-4).
	 * Master 0/1 gate for the android_vh_scheduler_tick vendor-
	 * hook observer.  When 0 the registered probe is a single-
	 * branch no-op; when 1 every scheduler tick on a CPU belonging
	 * to a zenith-driven policy stamps ktime_get_ns() on
	 * z_cpu->vh_scheduler_tick_last_ns and bumps z_cpu->vh_-
	 * scheduler_tick_count.  Read via READ_ONCE on the probe
	 * (HZ * num_CPUs hot path); written via WRITE_ONCE from sysfs
	 * and from zenith_apply_profile().
	 */
	unsigned int		vh_scheduler_tick_enable;

	/* Patch L: see ZENITH_DEFAULT_VERBOSE_LOG comment block.
	 * Operator-controlled gate for the human-readable zenith
	 * dmesg / logcat trail.  Read via READ_ONCE on the (cold)
	 * sysfs and profile_store paths; written via WRITE_ONCE
	 * from the verbose_log_store handler.
	 */
	unsigned int		verbose_log;

	/* Patch K: game / sustained-high-load performance burst.  See
	 * ZENITH_DEFAULT_GAME_PERF_BURST and the long comment block
	 * above ZENITH_GPB_STATE_IDLE for the full mechanism.  Five
	 * cooperating fields:
	 *
	 *   game_perf_burst                       master 0/1, default 1
	 *                                         (also gates the
	 *                                          zenith_game_perf_burst_-
	 *                                          key static branch)
	 *   game_perf_burst_floor_pct             freq floor while ARMED,
	 *                                         applied as
	 *                                         policy->max * pct / 100
	 *                                         in zenith_get_next_freq()
	 *   game_perf_burst_thermal_ceiling_dc    skin-temp guardrail,
	 *                                         millidegrees C
	 *                                         (matches the kernel
	 *                                          thermal subsystem unit)
	 *   game_perf_burst_disarm_grace_ms       sustained-clear hold
	 *                                         before transitioning out
	 *                                         of ARMED on Signal-B drop
	 *   game_perf_burst_cooldown_ms           length of the COOLDOWN
	 *                                         glide that linearly
	 *                                         steps the floor down to 0
	 *
	 * All five are read on the hot path inside
	 * ZENITH_FEATURE_ENABLED(game_perf_burst); writes are via
	 * WRITE_ONCE in their respective sysfs handlers.  Not profile-
	 * baked because the burst mechanic is workload-detection driven,
	 * not preset state -- presets that wanted to bias the floor would
	 * have to write to game_perf_burst_floor_pct explicitly.
	 */
	unsigned int		game_perf_burst;
	unsigned int		game_perf_burst_floor_pct;
	unsigned int		game_perf_burst_thermal_ceiling_dc;
	unsigned int		game_perf_burst_disarm_grace_ms;
	unsigned int		game_perf_burst_cooldown_ms;
};

enum zenith_stat_idx {
	ZENITH_STAT_DECISIONS,		/* total get_next_freq() calls */
	ZENITH_STAT_CACHE_HITS,		/* cached_raw_freq early-return */
	ZENITH_STAT_INPUT_BOOST,	/* input_boost / input_boost_decay */
	ZENITH_STAT_BRUTAL,		/* snap_max / brutal_hold / climb_step */
	ZENITH_STAT_HISPEED,		/* hispeed */
	ZENITH_STAT_FRAME_PACE,		/* frame_pace */
	ZENITH_STAT_AUDIO,		/* audio_floor / audio_cap */
	ZENITH_STAT_RENDER_CAMERA,	/* render_floor / camera_floor */
	ZENITH_STAT_UCLAMP,		/* uclamp_min_floor / uclamp_max_cap */
	ZENITH_STAT_PSI,		/* psi_*_cap */
	ZENITH_STAT_BOOT_BOOST,		/* boot_boost */
	ZENITH_STAT_LIGHT_CAP,		/* light_cap */
	ZENITH_STAT_EM_CAP,		/* em_cap */
	ZENITH_STAT_EAS,		/* fall-through, no override */
	ZENITH_STAT_OTHER,		/* unmapped tp_path */
	/* Stage 4 / Patch I additions.  Split out the tp_paths
	 * previously bucketed into ZENITH_STAT_OTHER (predict_up,
	 * peak_prearm, peak_rescue) so testers can read the
	 * fire-rate of each Stage-3+ tier independently of the
	 * legacy buckets.
	 */
	ZENITH_STAT_PREDICT_UP,		/* predict_up */
	ZENITH_STAT_PEAK_PREARM,	/* peak_prearm */
	ZENITH_STAT_PEAK_RESCUE,	/* peak_rescue */
	ZENITH_STAT_PEAK_HYST,		/* peak_hyst (Patch E) */
	ZENITH_STAT_PEER_RAMP,		/* peer_ramp (Patch D) */
	ZENITH_STAT_MIGRATION_FLOOR,	/* migration_floor (Patch K1) */
	ZENITH_STAT_PSI_CPU_FLOOR,	/* psi_cpu_floor (Patch K2) */
	ZENITH_STAT_FRAME_OVERRUN,	/* frame_overrun (Patch K3) */
	ZENITH_STAT_AUTO_THERMAL_CAP,	/* auto_thermal_cap (Path B) */
	ZENITH_STAT_QUIET_HOURS_CAP,	/* quiet_hours_cap (Patch 1.10) */
	ZENITH_STAT_NR
};


struct zenith_at_log_entry {
	u64		ts_ns;
	u32		flags;			/* ZENITH_AT_FLAG_* mask */
	u32		var_x256;
	u32		thermal_pressure;	/* 0..1024 */
	u16		sat_pct;		/* 0..100 */
	u16		events_rate_x2;
	u16		thermal_slope;		/* signed-stored-as-u16 */
	u8		v1_target;		/* enum profile id */
	u8		v2_from_state;
	u8		v2_to_state;
	u8		reason;			/* enum at_reason */
	u8		emergency;		/* 1 if state_overridden by slope */
	u8		_pad[3];
};


struct zenith_at_v3_calib_log_entry {
	u64		ts_ns;
	u32		transitions;
	u8		mode;			/* ZENITH_AT_V3_MODE_* */
	s8		hyst_before;
	s8		hyst_after;
	s8		cool_before;
	s8		cool_after;
	u8		_pad[3];
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
	/* Serialises the slow-path kthread_work against teardown */
	struct mutex		work_lock;
	struct kthread_worker	worker;
	struct task_struct	*thread;
	bool			work_in_progress;

	bool			limits_changed;
	bool			need_freq_update;

	/* Per-bin unlock deadlines for the multi-step efficient_freq
	 * ladder. eff_unlock_at_ns[i] is the ktime_get_ns() time after
	 * which target_freq is allowed past tunables->eff_freq[i].
	 * Zero means "idle" — not currently gating. Indexed 0..eff_nr-1.
	 */
	u64			eff_unlock_at_ns[ZENITH_EFF_BINS_MAX];

	/* Multiplier currently applied to down_rate_delay_ns. Bumped to
	 * tunables->sampling_down_factor while sitting at policy->max,
	 * reset to 1 the moment we leave max. Mirrors ondemand.
	 */
	unsigned int		down_rate_mult;

	/* True once load crossed up_threshold. Stays true, holding us at
	 * policy->max, until load drops below down_threshold. Collapses
	 * to old snap-on-every-sample behaviour when
	 * down_threshold >= up_threshold.
	 */
	bool			brutal_active;

	/* Hysteresis state for the hispeed tier (step 2b).  Sticky: goes
	 * true when load_pct first crosses hispeed_load with freq below
	 * the effective hispeed floor; stays true while load_pct stays
	 * above (hispeed_load - hispeed_hyst_pct).  Drops back to false
	 * only when the margin is crossed.  Prevents the tier from
	 * flapping in/out on noisy load trajectories that dance around
	 * hispeed_load, the same way brutal_active does for up_threshold.
	 */
	bool			hispeed_active;

	/* Entry-side streak counter for hispeed activation.  Increments
	 * on every sample where load_pct >= hispeed_load, resets when
	 * load_pct < hispeed_load.  hispeed_active flips to true only
	 * when streak > tunables->hispeed_entry_streak, providing
	 * symmetric entry/exit hysteresis.  Capped to a small u8 to
	 * avoid wraparound on long sustained-load runs.
	 */
	u8			hispeed_entry_count;

	/* Entry-side streak counter for brutality activation.  Mirror of
	 * hispeed_entry_count: increments on every sample where
	 * load_pct >= up_threshold while brutal_active is false, resets
	 * when load_pct drops below up_threshold OR when brutal_active
	 * flips true (past entry, no more entry credit to accumulate).
	 * Gates the SNAP snap-to-max path only when
	 * tunables->brutal_entry_streak > 0.  Capped to a small u8 to
	 * avoid wraparound on sustained-above-threshold runs that never
	 * quite reach the streak cap (shouldn't happen with the default
	 * cap of 16 but defensive against future cap bumps).
	 */
	u8			brutal_entry_count;

	/* Entry-side streak counter for the peak-headroom rescue tier.
	 * Increments on every sample where the cluster is starving
	 * (load_pct >= STARVE_LOAD_PCT and freq < FREQ_FLOOR_PCT of
	 * policy->max), resets when either condition breaks.  The
	 * rescue fires only when the streak exceeds STARVE_STREAK,
	 * giving STARVE_STREAK+1 consecutive sample windows of
	 * starvation as the entry hysteresis.  Capped to a small u8 to
	 * avoid wraparound on indefinitely-sustained heavy load.
	 */
	u8			peak_starve_count;

	/* Hold-down deadline for the peak-headroom rescue tier.  Stamped
	 * to ktime_get_ns() + PEAK_HEADROOM_HOLD_MS at the moment the
	 * rescue fires.  While now < this deadline, repeated streak
	 * crossings are observed (the streak counter still increments)
	 * but no second rescue freq-bump is applied.  Eliminates the
	 * pathological case where a rescue lifts freq, the very next
	 * sample observes load_pct still >= STARVE_LOAD_PCT and freq
	 * still < FLOOR_PCT (because the cpufreq driver hasn't applied
	 * the previous request yet), and another rescue fires --
	 * stacking two unwanted up-shifts on what should be a single
	 * rescue event.  Cleared (set to 0) at policy init.
	 */
	u64			peak_rescue_until_ns;

	/* Migration-arrival soft-floor deadline (Patch K1).  Stamped
	 * by the per-CPU update_util callback whenever any CPU in
	 * this policy sees a sample-to-sample util jump exceeding
	 * tunables->migration_jump_pct of its max_capacity.  Read by
	 * the migration_floor tier in zenith_get_next_freq() to
	 * decide whether to apply the soft floor.  Self-disarms by
	 * deadline; ktime_get_ns() advancing past the value is the
	 * disarm.  Cleared (set to 0) at policy init.  Updated under
	 * the per-policy update_lock by the shared callback;
	 * unlocked but ordered by raw_spin_lock acquire/release in
	 * the single-CPU callback.  No torn-write hazard either way:
	 * a 64-bit write on a 64-bit kernel is atomic, and the field
	 * is only ever monotonic-forward written.
	 */
	u64			migration_in_until_ns;

	/* Cluster-wake-pulse (Patch 1.3) deadlines.  cluster_wake_-
	 * pulse_until_ns is stamped at the top of zenith_get_next_-
	 * freq() when (a) cluster_wake_pulse_ms is non-zero and
	 * (b) the gap (now_ns - cluster_wake_last_eval_ns) is at
	 * or above cluster_wake_pulse_idle_ms.  cluster_wake_last_-
	 * eval_ns is updated on every eval (so a continuous active
	 * stream simply keeps refreshing the timestamp without ever
	 * arming the pulse).  Same single-writer-per-policy reason-
	 * ing as migration_in_until_ns above: both fields are
	 * touched only by the eval path under update_lock.
	 */
	u64			cluster_wake_pulse_until_ns;
	u64			cluster_wake_last_eval_ns;

	/* Patch 1.9 fg-transition pulse deadline.  Stamped by the
	 * sched_wakeup_new tracepoint probe (zenith_probe_wakeup_-
	 * new) when a foreground task is woken for the first time
	 * after fork() on a CPU that belongs to this policy.
	 *
	 * Cross-context writer: unlike the other deadline fields in
	 * this struct (which are touched only by the eval path
	 * under update_lock), this one is written from arbitrary
	 * scheduler context.  Both writer and the eval-path reader
	 * use WRITE_ONCE / READ_ONCE; on 64-bit the access is
	 * naturally torn-write-safe, on 32-bit the worst case is a
	 * sub-millisecond drift on the deadline read which is well
	 * under the resolution of fg_transition_pulse_ms.
	 */
	u64			fg_transition_pulse_until_ns;

	/* PSI-mem cap deadline (Patch M1).  Stamped by the eval path
	 * when zenith_psi_mem_some_pct() crosses psi_mem_cap_thresh
	 * (and master psi_aware + V2 PSI_MEM_CAP tier gates pass);
	 * read by the same eval path one tier later as
	 * `now < deadline -> cap fires`.  Same per-policy / single-
	 * writer reasoning as migration_in_until_ns above: each
	 * policy stamps its own deadline (so the BIG / PRIME caps
	 * release independently), and the eval is serialized by
	 * update_lock.
	 *
	 * Cleared (set to 0) at policy init by zenith_alloc_-
	 * policy(); never decremented other than by the deadline
	 * comparison expiring, so no torn-write hazard on a 64-bit
	 * kernel and natural alignment guarantee on 32-bit.
	 */
	u64			psi_mem_cap_until_ns;

	/* Patch C9: io_floor_hyst sticky-deadline.  Stamped by
	 * zenith_iowait_boost() on the 0->positive boost edge with
	 * 'now + io_floor_hyst_ms * NSEC_PER_MSEC'.  Read by
	 * zenith_get_next_freq() to gate the io_floor tier.  Same
	 * writer reasoning as psi_mem_cap_until_ns above; reads are
	 * naked u64 loads, harmless on a stale-by-one-tick read
	 * because the floor is a heuristic.
	 */
	u64			io_floor_until_ns;

	/* Util-trend ring for the predictive up-shift tier (2a').  The
	 * tail of zenith_get_next_freq() pushes the current sample's
	 * util into util_history[util_history_idx] and advances the
	 * index modulo predict_up_window.  util_history_count tracks
	 * how many slots have been written since policy attach (or
	 * since the last shrink-on-window-store), so the tier can wait
	 * for the ring to warm up before evaluating a trend on
	 * unwritten zeroes.  Capped at ZENITH_PREDICT_UP_WINDOW_MAX so
	 * the storage cost is bounded.
	 */
	unsigned long		util_history[ZENITH_PREDICT_UP_WINDOW_MAX];
	unsigned int		util_history_idx;
	unsigned int		util_history_count;

	/* Render-thread floor debounce stamp [Stage 4 / Patch B].
	 * Set to ktime_get_ns() on the first eval where
	 * zenith_policy_has_render() returns true after a quiet
	 * period (previous sample saw has_render==false).  Reset to
	 * 0 the first time has_render returns false, so the debounce
	 * window restarts on each fresh render-thread arrival.
	 * Compared against tunables->render_floor_min_runtime_ms
	 * before the floor is allowed to fire.  Zero-initialised at
	 * policy alloc; no special teardown.
	 */
	u64			render_first_seen_ns;

	/* Peak-return hysteresis streak counter [Stage 4 / Patch E].
	 * Increments on every consecutive sample where the previous
	 * cached_raw_freq was at peak class
	 * (>= ZENITH_PEAK_HYSTERESIS_PEAK_THRESH_PCT * policy->max
	 * / 100) and the current freq wants to drop sharply.  When
	 * < tunables->peak_hysteresis_streak, the soft floor at
	 * (prev_freq * peak_step_down_pct / 100) is applied; when
	 * the streak drains, the cluster falls naturally through
	 * the lower tiers.  Reset on any sample where the previous
	 * freq is below peak class, or when the freq tier already
	 * computes a value at or above the soft floor.  Capped at
	 * ZENITH_PEAK_HYSTERESIS_STREAK_MAX so a stuck-near-peak
	 * regime cannot accumulate unbounded streak credit.
	 */
	unsigned int		peak_low_streak;

	/* Anchor freq for the peak-return hysteresis tier
	 * [Stage 4 / Patch E].  Captured on the transition from a
	 * peak-class previous freq to a non-peak natural freq, then
	 * used as the source for the soft floor
	 * (anchor * peak_step_down_pct / 100) until the streak
	 * drains.  Cleared on streak drain or on a sample where the
	 * computed freq is already at or above the soft floor (no
	 * hysteresis needed).  0 means "no anchor armed".
	 */
	unsigned int		peak_hyst_anchor_freq;

	/* Persistent-idle streak counter for boost early-exit
	 * [Stage 4 / Patch F].  Increments on every tick where an
	 * input boost is armed (now < zenith_input_boost_until_ns)
	 * AND the cluster's load_pct is below
	 * tunables->boost_idle_thresh.  Reset on boost expiry, on
	 * any non-idle sample inside the boost window, or after a
	 * preemption tick fires.  Capped at
	 * ZENITH_BOOST_IDLE_STREAK_MAX so a stuck-near-zero workload
	 * cannot accumulate unbounded streak credit.
	 */
	unsigned int		boost_idle_low_streak;

	/* Last-runnable timestamp [Stage 4 / Patch H].  Stamped
	 * with ktime_get_ns() in zenith_get_next_freq() whenever
	 * tp_load_pct > 0 (the cluster has any util).  Read by the
	 * sleeper-tail tier to gate the freq shave.
	 */
	u64			last_runnable_ns;

	/* Last decision tag chosen by zenith_get_next_freq() on this
	 * policy [Stage 4 / Patch J].  Stamped right after the
	 * stats[] update at the end of every eval (cached and
	 * uncached paths both update it).  Read-only sysfs node
	 * "last_decision_path" dumps the current value per-policy,
	 * formatted "policy<cpu>(<cluster>): <tag>\n".  Pointers
	 * are to .rodata string literals so storage is a single
	 * pointer assignment, no copy.  Reads use READ_ONCE for
	 * coherency with the eval-side WRITE_ONCE.
	 */
	const char		*last_decision_path;

	/* Patch B7-2: decision-ring buffer.  Per-policy circular log
	 * of the last ZENITH_DEC_RING_NR (path, lat_ns) entries,
	 * paired so the sysfs reader can correlate which tier won
	 * with how long the eval took.  Updated in lockstep with
	 * dec_lat_buckets and last_decision_path at the end of every
	 * eval; head advances under the policy update_lock so the
	 * read side only needs READ_ONCE on path.  lat_ns is a 32-bit
	 * value (eval cost is bounded by tens of microseconds; truncating
	 * the high bits at u32_max ~= 4 s loses only pathological
	 * outliers, and they get clamped, not wrapped).
	 */
	struct zenith_dec_ring_entry {
		const char	*path;
		u32		lat_ns;
	} dec_ring[ZENITH_DEC_RING_NR];
	unsigned int		dec_ring_head;

	/* Time-bounded cache for the per-policy uclamp_{min,max}
	 * aggregations.  Each walk is O(n_cpus_in_policy) rq reads
	 * (cheap, no locks, no cachelines dirtied) but the eval path
	 * can run every rate_limit_us, i.e. 10 000 Hz on up_rate path;
	 * caching for ZENITH_UCLAMP_CACHE_TTL_NS drops the rq walks by
	 * ~10x on an 8-CPU policy with zero user-visible staleness.
	 * Valid when uclamp_cache_valid_ns != 0 and now < valid_ns + TTL.
	 */
	unsigned long		cached_uclamp_min;
	unsigned long		cached_uclamp_max;
	u64			uclamp_cache_stamp_ns;

	/* Cached nice-load ratio for the policy (0..100). Sampled in
	 * the update_util hook when ignore_nice_load=1 and read from
	 * zenith_get_next_freq. Guarded by update_lock in the shared
	 * path; on the single path only one CPU writes it.
	 */
	unsigned int		nice_pct;

	/* Auto-tune observer state. Counters are incremented in the
	 * hot path when tunables->auto_tune=1; the delayed_work handler
	 * samples and resets them every ZENITH_AUTO_TUNE_PERIOD_MS and
	 * chooses a preset.
	 *
	 * They are atomic because the single-CPU update path runs
	 * zenith_get_next_freq() without holding update_lock, while the
	 * delayed_work handler reads+resets them (and runs on any CPU).
	 * atomic_inc in the hot path and atomic_xchg(..., 0) in the
	 * worker give us lockless correctness for the sample window.
	 */
	atomic_t		at_samples_total;
	atomic_t		at_samples_saturated;
	u64			at_last_events;
	unsigned int		at_last_total;
	unsigned int		at_last_saturated;
	unsigned int		at_last_sat_pct;
	unsigned int		at_last_events_rate_x2;
	unsigned int		at_last_target;
	unsigned int		at_last_state;
	/* F2: PELT-derived util tracking.  Sum of per-cpu rq->cfs.avg
	 * .util_avg across this policy, sampled at every V1 work tick.
	 * Compared against the previous window's value to detect a
	 * rising-load trend before sat_pct fully saturates.  Stored
	 * as raw util sum (1024 * num_cpus capacity scale); rising
	 * threshold is normalised to a percentage in
	 * t->auto_tune_util_rising_thresh_pct.
	 */
	unsigned long		at_last_util_sum;
	/* M1: time-in-state accounting.  at_state_residency_ns[s] is the
	 * cumulative wall-clock time spent in V2 state s since governor
	 * start (or since the last reset).  Updated lazily at every V2
	 * commit point: on transition from old to new, accumulate the
	 * (now - last_change_ns) delta into residency[old_state] and
	 * stamp last_change_ns := now.  Cheap (one ktime_get + 5 u64
	 * stores per actual transition, NOT per tick).  Surfaced
	 * read-only via auto_tune_state_residency sysfs as a CSV that
	 * userspace tools can sample at low frequency to compute
	 * percent-time-in-state.
	 */
	u64			at_state_residency_ns[5];
	u64			at_state_last_change_ns;
	/* M2: V2 state transition history ring.  Last
	 * ZENITH_AT_HISTORY_NR commits per policy, expressed as
	 * { ts_boottime_ns, from_state, to_state, reason, flags }
	 * tuples.  Surfaced read-only via auto_tune_state_history
	 * sysfs as one line per entry, most recent first.
	 *
	 * Each entry is 16 bytes; 32 entries = 512 B per policy
	 * (typically 2 policies = 1 KB total).  Cheap.
	 */
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
	/* at_last_applied_state is the V2 state AFTER the cluster-aware
	 * demotion path runs.  When auto_tune_cluster_aware = 1 the
	 * little cluster takes LATENCY/SUSTAINED_PERF -> BALANCED and the
	 * prime cluster takes BALANCED -> LATENCY; the at_last_state
	 * field holds the pre-demotion V2 state (so V3 / hysteresis /
	 * cooldown all see the canonical decision), and this field
	 * holds what was actually applied via zenith_at_apply_actions().
	 * Surfaced through auto_tune_status as applied_state= so an
	 * operator reading the file can tell at-a-glance which cluster
	 * was demoted vs the V2 logical state.  When cluster_aware is
	 * off, applied_state == at_last_state.
	 */
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

	/* Per-policy consecutive-EFFICIENCY-window counter for the
	 * boot_complete in-kernel calm detector.  See the
	 * ZENITH_DEFAULT_BOOT_COMPLETE_AUTO comment block.  Reset to 0
	 * whenever the resolved V2 state is anything other than
	 * EFFICIENCY; the auto detector latches zenith_boot_complete on
	 * the first per-policy worker that hits
	 * ZENITH_BOOT_COMPLETE_CALM_WINDOWS past the grace period.
	 */
	unsigned int		at_boot_calm_streak;

	/* V3 self-calibration state (see ZENITH_DEFAULT_AUTO_TUNE_V3
	 * comment block).  Updated at the tail of zenith_auto_tune_work()
	 * once per (auto_tune_v3_interval_ms) wall-clock period when the
	 * v3 key is enabled.  All fields are owned by the v2 worker
	 * thread and read either from the same thread (calibration
	 * step) or via sysfs *_show under the gov_attr_set rwsem (which
	 * guarantees a consistent snapshot).
	 *
	 *   at_v3_last_calib_ns    -- boottime ns of last calibration
	 *   at_v3_last_transitions -- transitions counted in last window
	 *   at_v3_hyst_offset      -- signed offset on hysteresis_windows
	 *   at_v3_cool_offset      -- signed offset on cooldown_windows
	 *
	 * Offsets are clamped to [ZENITH_AT_V3_OFFSET_MIN ..
	 * ZENITH_AT_V3_OFFSET_MAX] and only consumed when the v3 mode
	 * is APPLY (== 2).
	 */
	u64			at_v3_last_calib_ns;
	unsigned int		at_v3_last_transitions;
	signed char		at_v3_hyst_offset;
	signed char		at_v3_cool_offset;

	/* Patch J: V3 calibration audit ring.  Pushed by
	 * zenith_at_v3_calibrate() at the end of every calibration
	 * tick (both OBSERVE and APPLY).  Surfaced via the
	 * auto_tune_v3_calib_log RO sysfs node so the operator can
	 * see V3's actual drift trajectory without ftrace.
	 * Single-writer (the v2 worker thread) / multi-reader (sysfs
	 * *_show under the gov_attr_set rwsem); reset alongside the
	 * other observability surfaces in
	 * zenith_policy_observability_reset() and on V3 master
	 * MODE_OFF transitions in auto_tune_v3_store.
	 */
	struct zenith_at_v3_calib_log_entry
				at_v3_calib_log[ZENITH_AT_V3_CALIB_LOG_NR];
	unsigned int		at_v3_calib_log_head;
	unsigned int		at_v3_calib_log_count;

	/* round-U-z10 glide / coordination knobs auto-driven by the V2
	 * worker via zenith_at_apply_glides() when
	 * tunables->auto_tune_v2_glides is on.  Consumers fall through
	 * to these only when the user-set tunable is 0; otherwise the
	 * user value wins outright.  at_local_glides_active gates the
	 * fall-through; cleared on auto_tune_v2 disable so the next
	 * unrelated freq tick stops consulting stale values.
	 */
	bool			at_local_glides_active;
	unsigned int		at_local_brutal_decay_ms;
	unsigned int		at_local_wakeup_boost_ms;
	unsigned int		at_local_boot_boost_decay_ms;
	unsigned int		at_local_screen_off_glide_ms;
	unsigned int		at_local_thermal_pressure_continuous;
	unsigned int		at_local_prefer_silver_aware;
	unsigned int		at_local_frame_budget_us_auto;

	/* Patch L: Stage-4 K1/K2/K3 tier-armed bitmask, written by
	 * zenith_at_apply_tiers() per V2 worker pass.  See the
	 * ZENITH_AT_TIER_* comment block.  Read by zenith_tier_value()
	 * from the K1/K2/K3 read sites in zenith_get_next_freq().
	 *
	 * at_local_tiers_active gates the read: false (cleared on V2
	 * disable, profile change, or auto_tune_v2_tiers=0) means the
	 * K1/K2/K3 reads return the underlying tunable verbatim, the
	 * pre-Patch-L behaviour.  true means consult the armed mask.
	 *
	 * Single-writer (V2 worker) / single-reader (eval path under
	 * update_lock); staleness is bounded by ZENITH_AUTO_TUNE_PERIOD_MS.
	 */
	bool			at_local_tiers_active;
	unsigned long		at_local_tier_armed_mask;
	struct delayed_work	at_work;

	/* Auto-tune classifier ring buffer.  Single-writer (the
	 * delayed_work worker), multi-reader (sysfs at_log readers).
	 * at_log_head is the next write slot; at_log_count is the total
	 * number of entries pushed since the last reset, capped at the
	 * ring depth on read.  Both are reset by writing to
	 * zenith_stats_reset.
	 */
	struct zenith_at_log_entry at_log[ZENITH_AT_LOG_NR];
	unsigned int		at_log_head;
	unsigned int		at_log_count;

	/* brutal_decay_ms tail-glide deadline.  0 means no decay is
	 * armed and zenith_get_next_freq() takes the fast path; non-zero
	 * is an absolute ktime_get_ns() value at which the decay floor
	 * stops applying.  Single-writer (zenith_get_next_freq() under
	 * the policy's update_lock), single-reader (same site).
	 */
	u64			brutal_decay_until_ns;
	unsigned int		brutal_decay_arm_ms;

	/* screen_off_glide_ms tracking.  screen_state_last is the
	 * screen_state value seen on the previous zenith_get_next_freq()
	 * tick; transitions are detected by comparing tunables-> against
	 * this snapshot.  screen_off_arm_ns is set to ktime_get_ns() at
	 * the moment of the 1 -> 0 transition and cleared on the
	 * 0 -> 1 transition; non-zero on the screen-off branch means
	 * the glide ramp is candidate for application.  Single-writer,
	 * single-reader (zenith_get_next_freq() under update_lock).
	 */
	u64			screen_off_arm_ns;
	unsigned int		screen_state_last;

	/* prefer_silver_aware coordination state.  Snapshot of the
	 * global prefer_silver hit / miss counters at the previous V1
	 * classifier window, plus the resulting hit-rate (0..100) for
	 * the most-recent window.  The hit-rate is read on the hot
	 * path by zenith_get_next_freq() and only updated by the worker,
	 * so the read is unsynchronised but bounded to the previous
	 * complete window.  When CONFIG_SCHED_PREFER_SILVER=n these
	 * fields stay at 0 (the worker never updates them) and the
	 * downstream bump never fires.
	 */
	unsigned int		ps_prev_hit;
	unsigned int		ps_prev_miss;
	unsigned int		ps_hit_rate_pct;

	/* Cached topology bit: true when any CPU in the policy has
	 * arch_scale_cpu_capacity == SCHED_CAPACITY_SCALE, i.e. the
	 * policy belongs to (one of) the system's highest-capacity
	 * cluster(s).  Computed once at zenith_start() time from the
	 * policy's cpumask and kept for the lifetime of the policy.
	 * Used by the input-boost gate (input_boost_big_only) to skip
	 * boosting small-cluster policies on heterogeneous SoCs.
	 */
	bool			is_big_cluster;

	/* Cached rate-limit scale factors applied when
	 * rate_limit_cluster_scale is enabled.  1/0 means unscaled;
	 * little clusters use 2/1 so upward ramps are less eager and
	 * downward ramps save power sooner.
	 */
	unsigned int		up_rate_scale;
	unsigned int		down_rate_scale_shift;
	unsigned int		cluster_class;

	/* Cached per-policy result of the render-aware comm walk.  Valid
	 * for ZENITH_RENDER_CACHE_TTL_NS after render_cache_stamp_ns.
	 * Refreshed on the next zenith_get_next_freq() call past the TTL.
	 * Zero stamp means "never sampled".
	 */
	bool			render_active;
	u64			render_cache_stamp_ns;

	/* Wave A render-thread util tracker.  Stores the
	 * se.avg.util_avg of the first comm-matched render task
	 * observed during zenith_policy_has_render()'s walk.  Refreshed
	 * alongside render_active / render_cache_stamp_ns; valid for
	 * ZENITH_RENDER_CACHE_TTL_NS.  Zero means "no matched task"
	 * (or never sampled); non-zero is the matched task's util in
	 * 1/SCHED_CAPACITY_SCALE units.
	 */
	unsigned int		render_matched_util_avg;

	/* Cached per-policy result of the cgroup-aware top-app walk.
	 * Valid for ZENITH_TOP_APP_CACHE_TTL_NS after
	 * top_app_cache_stamp_ns.  Refreshed on the next
	 * zenith_get_next_freq() call past the TTL.  Zero stamp means
	 * "never sampled".
	 */
	bool			top_app_active;
	u64			top_app_cache_stamp_ns;

	/* Wave B EAS energy-knee freq cache.  Populated at
	 * zenith_start() time from em_cpu_get(); refreshed lazily on
	 * the first em_floor application if the EM was not yet
	 * registered when zenith_start() ran.  Zero means "no EM"
	 * (or never sampled); non-zero is the energy-knee freq in
	 * KHz.
	 */
	unsigned int		em_knee_freq;

	/* Cached per-policy result of the audio-aware comm walk.  Same
	 * shape as the render cache above; TTL is
	 * ZENITH_AUDIO_CACHE_TTL_NS.  Zero stamp means "never sampled".
	 */
	bool			audio_active;
	u64			audio_cache_stamp_ns;

	/* Patch B7-1: sticky audio-active deadline.  When
	 * zenith_policy_has_audio() observes a positive detection
	 * (alsa fd > 0 or comm-walk match) we extend this deadline to
	 * now + audio_hyst_ms.  While now < audio_sticky_until_ns the
	 * helper returns true regardless of fresh signal, so a brief
	 * gap between two pcm releases / re-opens does not flip the
	 * cluster out of audio-aware mode.  audio_hyst_ms == 0
	 * disables the hysteresis (legacy behaviour).
	 */
	u64			audio_sticky_until_ns;

	/* Cached per-policy result of the camera-aware comm walk.
	 * Holds the *raw* comm-match result, before the userspace
	 * override (camera_active=auto/force-on/force-off) is applied.
	 * TTL is ZENITH_CAMERA_CACHE_TTL_NS.  Zero stamp means "never
	 * sampled".
	 */
	bool			camera_auto_match;
	u64			camera_cache_stamp_ns;

	/* Cached per-policy result of the game-auto comm walk + streak
	 * counter for the in-kernel game detector.  See
	 * ZENITH_DEFAULT_GAME_AUTO comment block.  TTL is
	 * ZENITH_GAME_AUTO_CACHE_TTL_NS for the comm-match cache; the
	 * streak counter increments on each hot-path call that observes
	 * a match (subject to the cache TTL) and resets to 0 on the
	 * first miss.  When it reaches ZENITH_GAME_AUTO_DETECT_STREAK,
	 * the global zenith_game_auto_active_until_ns latch is renewed
	 * for ZENITH_GAME_AUTO_ACTIVE_TTL_NS and the streak resets.
	 */
	bool			game_auto_match;
	u64			game_auto_cache_stamp_ns;
	unsigned int		game_auto_streak;

	/* Patch K: per-policy state for the game / sustained-high-load
	 * performance burst FSM.  See ZENITH_DEFAULT_GAME_PERF_BURST and
	 * the long comment block above ZENITH_GPB_STATE_IDLE for the
	 * full mechanism; this struct holds the live FSM state only.
	 *
	 *   gpb_state                ZENITH_GPB_STATE_{IDLE,ARMED,COOLDOWN}
	 *   gpb_state_entry_ns       ktime_get_ns() at the last FSM
	 *                            transition; used by the COOLDOWN
	 *                            glide to compute the linear ramp
	 *   gpb_b_arm_first_seen_ns  ktime_get_ns() at the first tick
	 *                            where Signal B (load >= 70%) was
	 *                            seen continuously; 0 = not seen
	 *   gpb_b_disarm_first_seen_ns
	 *                            ktime_get_ns() at the first ARMED
	 *                            tick where Signal B has dropped
	 *                            below 70%; 0 = currently still hot
	 *   gpb_tzd                  cached struct thermal_zone_device *
	 *                            for this policy's primary CPU
	 *                            ("cpu<N>-thermal"), resolved at
	 *                            zenith_start() and consulted by the
	 *                            thermal guardrail helper.  NULL means
	 *                            "no per-policy zone resolved", in
	 *                            which case the helper falls back to
	 *                            arch_scale_thermal_pressure().
	 *   gpb_tzd_retry_at_ns      ktime_get_ns() deadline for the next
	 *                            lazy zone-resolution retry when
	 *                            gpb_tzd is NULL.  Patch M closes the
	 *                            boot-ordering hole where thermal-core
	 *                            registers after the governor: the
	 *                            hot-path helper will re-attempt
	 *                            thermal_zone_get_zone_by_name() at
	 *                            most once per
	 *                            ZENITH_GPB_TZD_RETRY_INTERVAL_NS
	 *                            until a zone is bound.
	 *   gpb_arm_count            number of IDLE -> ARMED *and*
	 *                            COOLDOWN -> ARMED transitions since
	 *                            attach.  Bumped from the FSM
	 *                            evaluator at every rising edge.
	 *                            Counts re-arms so userspace can see
	 *                            "user keeps Alt+Tabbing back into
	 *                            the game" thrash.
	 *   gpb_disarm_count         number of ARMED -> COOLDOWN
	 *                            transitions since attach.  Equals
	 *                            gpb_arm_count when the FSM is
	 *                            currently IDLE/COOLDOWN, one less
	 *                            when ARMED.
	 *   gpb_idle_count           number of COOLDOWN -> IDLE
	 *                            transitions since attach.  Counts
	 *                            full glide completions; missing
	 *                            counts (vs disarm_count) are
	 *                            re-arms during cooldown.
	 *   gpb_last_disarm_reason   ZENITH_GPB_DISARM_{NONE,FAST,
	 *                            SUSTAINED}.  Recorded at every
	 *                            ARMED -> COOLDOWN edge.
	 *                            Patch M; surfaced by the
	 *                            game_perf_burst_stats RO sysfs.
	 *
	 * All ten fields are only touched from the per-policy hot path
	 * (zenith_get_next_freq() and the FSM evaluator it calls), which
	 * is serialised by the cpufreq core.  No locking required.  All
	 * are zero-initialised by zenith_start()'s memset path or
	 * explicitly cleared at attach.
	 */
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

	/* Last seen zenith_input_boost_until_ns deadline observed inside
	 * an active boost window for this policy.  Latched in the input
	 * boost step (0) and consumed by the sampling-down step (7) to
	 * keep the down-rate multiplier elevated for one stretched
	 * down-rate window after the boost exits, smoothing the boost
	 * tail when target_freq immediately falls below policy->max.
	 * Cleared once the stretched window passes.  Zero means "no
	 * recent boost to extend".
	 */
	u64			boost_active_until_ns;

	/* Variance-adaptive up_threshold state (see
	 * ZENITH_DEFAULT_UP_THRESHOLD_ADAPTIVE).  load_var_ewma_x256 is
	 * an EWMA (alpha=1/8) of |load_pct - prev| in fixed-point
	 * 1/256ths -- so a value of 256 == 1.0 percentage-point of
	 * average sample-to-sample swing.  Read at the top of
	 * zenith_get_next_freq() to bias dynamic_up_thresh, updated at
	 * the bottom with the current sample's tp_load_pct.  Both fields
	 * zero-initialised by zenith_start()'s memset.
	 */
	unsigned int		load_var_ewma_x256;
	unsigned int		last_load_pct;

	/* Per-policy decision stats.  Bumped from zenith_get_next_freq()
	 * once per evaluation; total decisions and cache_hits are
	 * always counted, the tier buckets are mapped from the final
	 * tp_path string by zenith_path_to_bucket().  Reset to zero on
	 * zenith_start() so the values reflect the current attach
	 * cycle.  No locking: zenith_get_next_freq() is serialised
	 * per-policy by the cpufreq core (the leader cpu in a shared
	 * policy, the only cpu in a single policy), so a plain
	 * unsigned long ++ is race-free.
	 */
	unsigned long		stats[ZENITH_STAT_NR];

	/* Patch 1.4 decision-latency histogram.  4 unsigned-long
	 * buckets covering the eval cost in nanoseconds:
	 *   [0]   <  10 us
	 *   [1]  10..< 50 us
	 *   [2]  50..<100 us
	 *   [3]  >=100 us
	 *
	 * Storage budget: 4 * sizeof(unsigned long) per policy.
	 * Bumped at the same commit point as stats[]; the same
	 * single-writer reasoning applies (per-policy serialised by
	 * the cpufreq core).  Sysfs read via decision_latency_hist.
	 * Reset to zero on zenith_start() alongside stats[].
	 */
	unsigned long		dec_lat_buckets[4];

	/* Patch B9-1: realised per-cluster freq scale, cached from the
	 * android_vh_arch_set_freq_scale vendor hook.  Stores the most
	 * recent SCHED_CAPACITY_SCALE-domain value (0..1024) the
	 * scheduler observed for this policy after a freq write
	 * actually took effect.  Updated under no governor lock from
	 * the hook callback (which can fire in arbitrary scheduler
	 * context); written via WRITE_ONCE so the eval-path readers
	 * (which hold update_lock) and the cross-cluster comparator in
	 * the probe (which does not) both see torn-write-safe values
	 * on 32-bit.  Read via READ_ONCE.
	 *
	 * Cleared (set to 0) at policy init by kzalloc(); never
	 * decremented other than by the hook overwriting it on the
	 * next realisation.  When tunables->vh_arch_freq_scale_enable
	 * is 0 this field never moves off zero.
	 */
	unsigned long		vh_arch_freq_scale_last;

	/* Patch B9-3: most recent cpuidle residency observed on a CPU
	 * belonging to this policy, in ns.  Stamped by zenith_probe_-
	 * cpu_idle_exit() as (now_ns - per_cpu enter_ns).  Read by the
	 * cluster_wake_pulse arm site in zenith_get_next_freq() with
	 * READ_ONCE; the cluster_wake_pulse is suppressed when this
	 * value crosses ZENITH_VH_CPU_IDLE_RESIDENCY_LONG_NS.  Cleared
	 * (set to 0) at policy init by kzalloc(); only written by the
	 * exit probe so a last-writer-wins race across CPUs in the
	 * cluster is acceptable (the next exit on any CPU resets it).
	 * When tunables->vh_cpu_idle_enable is 0 this field never
	 * moves off zero.
	 */
	u64			vh_cpu_idle_last_residency_ns;

	/* Patch B9-5: per-policy jiffies stamp updated by
	 * zenith_probe_sched_move_task() on every cgroup move that
	 * lands a task on a CPU belonging to this policy.  Cleared (set
	 * to 0) at policy init by kzalloc(); only written by the probe
	 * (under READ_ONCE / WRITE_ONCE, no governor lock taken).  A
	 * 0 value means "no cgroup move observed since boot or while
	 * this policy was zenith-driven".  Read with READ_ONCE from
	 * any consumer (the auto-selector worker is the intended
	 * future consumer; B9-5 only stamps -- no consumer is wired
	 * in this patch, by design).  When tunables->vh_sched_move_-
	 * task_enable is 0 this field never moves off zero.
	 */
	unsigned long		vh_sched_move_task_last_jiffies;
};


struct zenith_cpu {
	struct update_util_data	update_util;
	struct zenith_policy	*z_policy;
	unsigned int		cpu;

	bool			iowait_boost_pending;
	unsigned int		iowait_boost;

	/* Wall-time the current iowait boost episode first armed; zero
	 * means "no episode in progress".  Stamped in zenith_iowait_boost()
	 * on the 0->floor transition, cleared whenever iowait_boost
	 * itself is cleared (zenith_iowait_reset / zenith_iowait_apply).
	 * Read by zenith_iowait_boost() to decide whether the
	 * iowait_backoff_after_ms timer has elapsed and the doubling
	 * step should flip to a halving step.
	 */
	u64			iowait_boost_first_ns;
	u64			last_update;
	unsigned long		bw_dl;

	/* Delta tracking for ignore_nice_load. Only read when the
	 * corresponding tunable is set.
	 */
	u64			prev_nice_time;
	u64			prev_nice_wall;
	unsigned long		max_capacity;

	/* Previous *unpredicted* zenith_get_util() output for this CPU,
	 * used as the second tap of the one-step-ahead predictor.  Only
	 * read/written when tunables->predict_util_pct != 0.  Initialised
	 * to zero by the kzalloc-style allocation in zenith_start().
	 */
	unsigned long		prev_util;

	/* Previous arch_scale_thermal_pressure() observation, used by
	 * the thermal_derate_rate_pct derivative term to compute a
	 * sample-to-sample rise.  Zero-initialised by zenith_start()'s
	 * memset; updated unconditionally whenever the static derate
	 * runs, so toggling the rate tunable doesn't see a stale
	 * prev_pressure for the first sample.
	 */
	unsigned long		prev_thermal_pressure;

	/* Tap from two samples ago, used only when
	 * tunables->predict_util_smooth is set and predict_util_pct != 0.
	 * Maintained in lockstep with prev_util (push the old prev_util
	 * into prev_util_2 before overwriting prev_util) so toggling the
	 * smooth tunable does not require any per-cpu init handshake.
	 * Initialised to zero by the kzalloc-style allocation in
	 * zenith_start().
	 */
	unsigned long		prev_util_2;

	/* Wakeup-boost state.  prev_util is independent from the predictor
	 * taps above so the detector works even when predict_util_pct=0.
	 * The countdown is decremented on upward transitions that bypass
	 * up_rate_limit.
	 */
	unsigned long		wakeup_prev_util;
	u8			wakeup_boost_ticks;

	/* Previous post-iowait_apply util on this CPU, used by the
	 * migration-arrival detector (Patch K1).  Independent from
	 * wakeup_prev_util because that field is only updated when
	 * tunables->wakeup_boost is set, and the migration detector
	 * needs to run regardless.  Zero-initialised by zenith_start()'s
	 * memset, which means the very first tick after policy attach
	 * looks like a "jump from 0" -- harmless: the migration tier
	 * disarms by 30 ms (default) wall-clock and any cluster
	 * starting from idle benefits from a brief floor anyway.
	 */
	unsigned long		migration_prev_util;

	/* Deadline mirror of wakeup_boost_ticks.  0 means no
	 * wall-clock-based bypass armed; non-zero is an absolute
	 * ktime_get_ns() value at which the time-based portion of the
	 * up-rate bypass stops applying.  Set together with
	 * wakeup_boost_ticks at the detection sites when
	 * tunables->wakeup_boost_ms is non-zero.
	 */
	u64			wakeup_boost_until_ns;

	/* kcpustat hispeed-blend sampler state (consumed by
	 * zenith_kcpustat_sample / zenith_kcpustat_blend). Two-phase
	 * windowed measurement: phase 1 clears at window expiry and
	 * arms hispeed_active for an immediate post-reset sample on
	 * the next callback.  Filtered busy_pct feeds an asymmetric
	 * EWMA (instant up, slow-decay down by filter_shift).
	 *
	 * hispeed_start_ns marks t=0 of the >>(elapsed_ms/32) decay
	 * applied in zenith_kcpustat_blend(); hispeed_idle_windows
	 * provides a one-window grace period before clearing the
	 * decay timer, so rapid idle/busy spinning doesn't keep the
	 * floor latched at full strength.
	 *
	 * All fields are zero-initialised by zenith_start()'s
	 * memset, which is the desired post-policy-attach state.
	 */
	u64			kc_prev_idle_time;	/* in usec */
	u64			kc_prev_wall_time;	/* in usec */
	unsigned int		kc_busy_pct;		/* raw, last window */
	unsigned int		kc_filtered_busy_pct;	/* EWMA-smoothed */
	bool			kc_hispeed_active;
	u64			kc_hispeed_start_ns;
	unsigned int		kc_idle_windows;

	/* Patch B9-3: ktime_get_ns() timestamp of the most recent
	 * cpuidle entry observed on this CPU.  Stamped by zenith_-
	 * probe_cpu_idle_enter() and consumed exactly once by
	 * zenith_probe_cpu_idle_exit() to compute (exit_ns -
	 * enter_ns) before stamping the per-policy
	 * vh_cpu_idle_last_residency_ns aggregate.  Cleared (set to
	 * 0) at zenith_start() time by the kzalloc-style memset; the
	 * exit probe treats 0 as "no enter observed" and skips the
	 * residency stamp.  When tunables->vh_cpu_idle_enable is 0
	 * this field never moves off zero (the enter probe gates on
	 * the same READ_ONCE).
	 */
	u64			vh_cpu_idle_last_enter_ns;

	/* Patch B9-4: per-CPU last-tick timestamp + tick count.
	 * Stamped by zenith_probe_scheduler_tick() once per
	 * android_vh_scheduler_tick fire on the local CPU.  Single
	 * writer per CPU (the local CPU's tick handler), so no
	 * atomics needed; remote-CPU readers use READ_ONCE.  Cleared
	 * (set to 0) at zenith_start() time by the kzalloc-style
	 * memset.  When tunables->vh_scheduler_tick_enable is 0 these
	 * fields never move off zero.
	 */
	u64			vh_scheduler_tick_last_ns;
	unsigned long		vh_scheduler_tick_count;
};


struct zenith_pmu_state {
#if IS_ENABLED(CONFIG_PERF_EVENTS)
	struct perf_event	*inst_event;
	struct perf_event	*cycle_event;

/*
 * Global variables and static keys -- shared across all .c files
 */

extern struct zenith_tunables *global_tunables;
extern struct mutex global_tunables_lock;

extern struct static_key_false zenith_audio_aware_key;
extern struct static_key_false zenith_camera_aware_key;
extern struct static_key_false zenith_render_aware_key;
extern struct static_key_false zenith_psi_aware_key;
extern struct static_key_false zenith_game_auto_key;
extern struct static_key_false zenith_auto_tune_v3_key;
extern struct static_key_false zenith_thermal_aware_key;
extern struct static_key_false zenith_game_perf_burst_key;

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

/*
 * Inline helpers shared by all translation units
 */

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


/* Function prototypes for cross-file access */
extern unsigned int zenith_glide_value(struct zenith_policy *z_policy);
extern void zenith_pmu_sample_cpu(struct zenith_cpu *z_cpu);
extern bool zenith_policy_has_game_auto(struct zenith_policy *z_policy);
extern unsigned int zenith_policy_thermal_pressure_pct(struct zenith_policy *z_policy);
extern unsigned int zenith_tunable_or_local(unsigned int *tunable, unsigned int local);

#endif
	u64			last_inst;
	u64			last_cycles;
	unsigned int		ipc_pct;
};


struct zenith_comm_table {
	struct rcu_head	rcu;
	unsigned int	nr;
	const char	*entries[ZENITH_COMM_LIST_MAX];
	char		raw[ZENITH_COMM_BUF_MAX];
};


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
	static const struct zenith_profile_defaults profiles[] = {

/*
 * Cross-file function declarations
 * (Functions with -static removed for multi-file access)
 */

