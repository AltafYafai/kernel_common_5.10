// SPDX-License-Identifier: GPL-2.0-only
/*
 * Generic GPU devfreq governor auto-switcher for the zenith governor.
 *
 * Watches for zenith game-mode transitions via zenith_is_game_mode_active()
 * and automatically switches the GPU's devfreq governor:
 *   - game mode active  -> "performance"
 *   - game mode stopped -> "simple_ondemand" (with configurable idle timeout
 *     before falling to "powersave")
 *
 * Works with ANY GPU driver (Mali, Adreno, Panfrost, etc.) that registers
 * a devfreq device, not just Qualcomm's msm_gpu.
 *
 * Tunables are exposed under /sys/module/zenith_gpu_switch/parameters/
 */

#include <linux/module.h>
#include <linux/devfreq.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/cpufreq_zenith.h>
#include <linux/jiffies.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/compaction.h>

/*
 * Debug logging gated behind CONFIG_ZENITH_DEBUG_MSG.
 * Production (community) builds compile these out entirely.
 */
#ifdef CONFIG_ZENITH_DEBUG_MSG
static bool debug_log;
module_param(debug_log, bool, 0644);
MODULE_PARM_DESC(debug_log, "Enable verbose dmesg logging (default: false)");

#define gpu_debug(fmt, ...)	pr_debug("zenith_gpu_switch: " fmt, ##__VA_ARGS__)
#else
static const bool debug_log = false;
#define gpu_debug(fmt, ...)	no_printk(KERN_DEBUG "zenith_gpu_switch: " fmt, ##__VA_ARGS__)
#endif

/***** Tunables *****/

static char gpu_devfreq_name[DEVFREQ_NAME_LEN] = "";
module_param_string(gpu_devfreq_name, gpu_devfreq_name,
		    sizeof(gpu_devfreq_name), 0644);
MODULE_PARM_DESC(gpu_devfreq_name,
		 "GPU devfreq device name (empty = auto-detect)");

static unsigned int gpu_check_ms = 2000;
module_param(gpu_check_ms, uint, 0644);
MODULE_PARM_DESC(gpu_check_ms,
		 "Poll interval in ms (default: 2000)");

static char gpu_game_governor[DEVFREQ_NAME_LEN] = "performance";
module_param_string(gpu_game_governor, gpu_game_governor,
		    sizeof(gpu_game_governor), 0644);
MODULE_PARM_DESC(gpu_game_governor,
		 "Governor while game mode is active (default: performance)");

static char gpu_active_governor[DEVFREQ_NAME_LEN] = "simple_ondemand";
module_param_string(gpu_active_governor, gpu_active_governor,
		    sizeof(gpu_active_governor), 0644);
MODULE_PARM_DESC(gpu_active_governor,
		 "Governor while GPU active, not gaming (default: simple_ondemand)");

static char gpu_idle_governor[DEVFREQ_NAME_LEN] = "powersave";
module_param_string(gpu_idle_governor, gpu_idle_governor,
		    sizeof(gpu_idle_governor), 0644);
MODULE_PARM_DESC(gpu_idle_governor,
		 "Governor after idle timeout (default: powersave)");

static unsigned int gpu_idle_ms = 5000;
module_param(gpu_idle_ms, uint, 0644);
MODULE_PARM_DESC(gpu_idle_ms,
		 "Idle timeout in ms before powersave (default: 5000, 0=skip)");

/***** Internal state *****/

static struct delayed_work gpu_governor_work;
static unsigned long gpu_last_game_active_jiffies;
static bool gpu_prev_game_active;

/*
 * Resolve the GPU devfreq device.  If gpu_devfreq_name is set (non-empty),
 * use it as an exact match.  Otherwise auto-detect by scanning all devfreq
 * devices for GPU keywords (mali, gpu, kgsl, adreno, panfrost, ...).
 */
static struct devfreq *gpu_resolve_devfreq(void)
{
	if (gpu_devfreq_name[0])
		return devfreq_get_devfreq_by_name(gpu_devfreq_name);

	return devfreq_find_gpu_devfreq();
}

static void gpu_governor_worker(struct work_struct *work)
{
	struct devfreq *df;
	const char *target;
	bool game_active;
	unsigned long idle_jiffies;

	df = gpu_resolve_devfreq();
	if (IS_ERR(df)) {
		/* GPU not probed yet -- retry later */
		goto resched;
	}

	game_active = zenith_is_game_mode_active();

	/*
	 * Proactively wake kcompactd when game mode first activates.
	 * This pre-compacts memory before game assets need high-order
	 * (DMA/GPU) allocations, reducing launch-time stalls.
	 */
	if (game_active && !gpu_prev_game_active)
		wakeup_all_kcompactd();
	gpu_prev_game_active = game_active;

	if (game_active) {
		target = gpu_game_governor;
		gpu_last_game_active_jiffies = jiffies;
	} else {
		idle_jiffies = jiffies - gpu_last_game_active_jiffies;
		if (gpu_idle_ms > 0 &&
		    jiffies_to_msecs(idle_jiffies) >= gpu_idle_ms)
			target = gpu_idle_governor;
		else
			target = gpu_active_governor;
	}

	/* Only switch if the governor actually changed */
	if (strcmp(df->governor_name, target)) {
		pr_info("zenith_gpu_switch: %s: %s -> %s%s\n",
			dev_name(&df->dev), df->governor_name, target,
			game_active ? " (game on)" : "");
		devfreq_set_governor(df, target);
	} else {
		gpu_debug("%s: already %s (game=%d)\n",
			  dev_name(&df->dev), target, game_active);
	}

resched:
	queue_delayed_work(system_unbound_wq, &gpu_governor_work,
			   msecs_to_jiffies(gpu_check_ms));
}

static int __init zenith_gpu_switch_init(void)
{
	gpu_last_game_active_jiffies = jiffies;
	INIT_DELAYED_WORK(&gpu_governor_work, gpu_governor_worker);
	queue_delayed_work(system_unbound_wq, &gpu_governor_work,
			   msecs_to_jiffies(gpu_check_ms));

	pr_info("zenith_gpu_switch: watching '%s' every %u ms\n",
		gpu_devfreq_name, gpu_check_ms);
	return 0;
}

static void __exit zenith_gpu_switch_exit(void)
{
	cancel_delayed_work_sync(&gpu_governor_work);
	pr_info("zenith_gpu_switch: stopped\n");
}

module_init(zenith_gpu_switch_init);
module_exit(zenith_gpu_switch_exit);

MODULE_DESCRIPTION("Zenith GPU devfreq governor auto-switcher");
MODULE_AUTHOR("Codebuff");
MODULE_LICENSE("GPL v2");
