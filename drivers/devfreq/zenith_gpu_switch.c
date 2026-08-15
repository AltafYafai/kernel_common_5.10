// SPDX-License-Identifier: GPL-2.0-only
/*
 * Generic GPU devfreq governor auto-switcher for the zenith governor.
 *
 * Watches for zenith game-mode transitions via zenith_is_game_mode_active()
 * and automatically switches the GPU's devfreq governor:
 *   - game mode active  -> "performance"
 *   - game mode stopped -> "simple_ondemand" (with configurable idle timeout
 *     before falling to "simple_ondemand")
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
#include <linux/dcache.h>
#include <linux/mm.h>

/*
 * Exported VM tunables we adjust during game mode:
 *   vm_dirty_ratio              — max dirty page % before throttling writers
 *   dirty_background_ratio      — % at which background writeback starts
 *   dirty_writeback_interval    — periodic flusher wakeup interval (cs)
 *   dirty_expire_interval       — max age of dirty pages before writeback (cs)
 *   min_free_kbytes             — reserved free page pool size
 *   sysctl_vfs_cache_pressure   — how aggressively inode/dentry cache is reclaimed
 */
extern int vm_dirty_ratio;
extern int dirty_background_ratio;
extern unsigned int dirty_writeback_interval;
extern unsigned int dirty_expire_interval;
extern int min_free_kbytes;

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

static char gpu_idle_governor[DEVFREQ_NAME_LEN] = "simple_ondemand";
module_param_string(gpu_idle_governor, gpu_idle_governor,
		    sizeof(gpu_idle_governor), 0644);
MODULE_PARM_DESC(gpu_idle_governor,
		 "Governor after idle timeout (default: simple_ondemand)");

static unsigned int gpu_idle_ms = 5000;
module_param(gpu_idle_ms, uint, 0644);
MODULE_PARM_DESC(gpu_idle_ms,
		 "Idle timeout in ms before idle governor (default: 5000, 0=skip)");

/***** Game-mode memory tuning save state *****/

static int saved_dirty_ratio;
static int saved_dirty_bg_ratio;
static int saved_vfs_cache_pressure;
static unsigned int saved_dirty_writeback_interval;
static unsigned int saved_dirty_expire_interval;
static int saved_min_free_kbytes;
static bool game_tuning_active;

/*
 * During game mode, tighten dirty ratios and reduce VFS cache pressure
 * so that:
 *   - Writeback starts earlier and stays gentler  (smaller bursts)
 *   - File-backed pages (game assets) are evicted less aggressively
 *   - Dirty pages are flushed sooner to avoid a post-game fsync storm
 *   - More free memory is reserved for atomic allocations
 *
 * The normal defaults are dirty_ratio=20, dirty_bg=10, dirty_wb=1500cs,
 * dirty_expire=3000cs, min_free=~8-16MB, vfs_cache=100.
 * User-modified values are saved on game-enter and restored on game-exit.
 */
#define GAME_DIRTY_RATIO               10
#define GAME_DIRTY_BG_RATIO             5
#define GAME_VFS_CACHE_PRESSURE        50
#define GAME_DIRTY_WB_INTERVAL       300  /* 3 seconds in cs */
#define GAME_DIRTY_EXPIRE_INTERVAL   500  /* 5 seconds in cs */

/***** Internal state *****/

static struct delayed_work gpu_governor_work;
static struct notifier_block gpu_game_mode_nb;
static unsigned long gpu_last_game_active_jiffies;
static bool gpu_prev_game_active;
static bool gpu_exiting;

static unsigned int gpu_probe_retries;

/*
 * Governor availability fallback.  Some GKI builds do not register the
 * governor the device (or this driver) asks for (e.g. 'simple_ondemand'
 * when CONFIG_DEVFREQ_GOV_SIMPLE_ONDEMAND is off).  devfreq_set_governor()
 * then fails with -EINVAL and the device stays on the built-in 'dummy'
 * governor.  We record the governor the device came up with and fall back
 * to it instead of churning devfreq every tick.
 */
static char gpu_native_governor[DEVFREQ_NAME_LEN];
static bool gpu_native_saved;
static bool gpu_game_gov_missing;
static bool gpu_active_gov_missing;
static bool gpu_gov_warned;

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
	unsigned long total_ram_kb;
	int ret;

	df = gpu_resolve_devfreq();
	if (IS_ERR_OR_NULL(df)) {
		/* GPU not probed yet -- retry later (up to 12 times) */
		if (++gpu_probe_retries > 12) {
			pr_info("zenith_gpu_switch: no GPU devfreq found after 12 retries, "
				"disabling\n");
			return;
		}
		goto resched;
	}

	if (!gpu_native_saved) {
		strscpy(gpu_native_governor, df->governor_name,
			DEVFREQ_NAME_LEN);
		gpu_native_saved = true;
	}

	game_active = zenith_is_game_mode_active();

	/*
	 * Proactively wake kcompactd when game mode first activates.
	 * This pre-compacts memory before game assets need high-order
	 * (DMA/GPU) allocations, reducing launch-time stalls.
	 */
	if (game_active && !gpu_prev_game_active)
		wakeup_all_kcompactd();

	/*
	 * Game-mode memory tuning: tighten dirty ratios and reduce VFS cache
	 * pressure so writeback stays gentle and game assets remain cached.
	 *
	 * Safety: the min_free_kbytes boost is capped to never exceed 5% of
	 * total RAM.  On a 4 GB device that's ~200 MB -- the 5 MB bump is
	 * well within range, but the cap prevents pathological over-reservation
	 * on very low-RAM or misconfigured systems.
	 */
	if (game_active && !game_tuning_active) {
		/* Save current values */
		saved_dirty_ratio = vm_dirty_ratio;
		saved_dirty_bg_ratio = dirty_background_ratio;
		saved_vfs_cache_pressure = sysctl_vfs_cache_pressure;
		saved_dirty_writeback_interval = dirty_writeback_interval;
		saved_dirty_expire_interval = dirty_expire_interval;
		saved_min_free_kbytes = min_free_kbytes;

		/* Apply game-mode values */
		vm_dirty_ratio = GAME_DIRTY_RATIO;
		dirty_background_ratio = GAME_DIRTY_BG_RATIO;
		sysctl_vfs_cache_pressure = GAME_VFS_CACHE_PRESSURE;
		dirty_writeback_interval = GAME_DIRTY_WB_INTERVAL;
		dirty_expire_interval = GAME_DIRTY_EXPIRE_INTERVAL;

		total_ram_kb = (unsigned long)totalram_pages() * (PAGE_SIZE / 1024);
		min_free_kbytes = min(min_free_kbytes + 5120,
				       (int)(total_ram_kb * 5 / 100));

		game_tuning_active = true;

		gpu_debug("game mode mem tuning ON "
			 "(dirty_ratio=%d, dirty_bg=%d, vfs_cache=%d, "
			 "wb=%ucs, expire=%ucs, min_free=%d)\n",
			 vm_dirty_ratio, dirty_background_ratio,
			 sysctl_vfs_cache_pressure,
			 dirty_writeback_interval, dirty_expire_interval,
			 min_free_kbytes);
	} else if (!game_active && game_tuning_active) {
		/* Restore saved values */
		vm_dirty_ratio = saved_dirty_ratio;
		dirty_background_ratio = saved_dirty_bg_ratio;
		sysctl_vfs_cache_pressure = saved_vfs_cache_pressure;
		dirty_writeback_interval = saved_dirty_writeback_interval;
		dirty_expire_interval = saved_dirty_expire_interval;
		min_free_kbytes = saved_min_free_kbytes;

		game_tuning_active = false;

		gpu_debug("game mode mem tuning OFF "
			 "(dirty_ratio=%d, dirty_bg=%d, vfs_cache=%d, "
			 "wb=%ucs, expire=%ucs, min_free=%d)\n",
			 vm_dirty_ratio, dirty_background_ratio,
			 sysctl_vfs_cache_pressure,
			 dirty_writeback_interval, dirty_expire_interval,
			 min_free_kbytes);
	}

	gpu_prev_game_active = game_active;

	if (game_active) {
		target = gpu_game_governor;
		gpu_last_game_active_jiffies = jiffies;
	} else {
		unsigned long cur_freq = df->previous_freq;

		/*
		 * GPU activity detection: if the GPU's current frequency is
		 * above the minimum OPP, reset the idle timer so the
		 * governor stays on the active governor.
		 */
		if (cur_freq > df->scaling_min_freq)
			gpu_last_game_active_jiffies = jiffies;

		idle_jiffies = jiffies - gpu_last_game_active_jiffies;
		if (gpu_idle_ms > 0 &&
		    jiffies_to_msecs(idle_jiffies) >= gpu_idle_ms)
			target = gpu_idle_governor;
		else
			target = gpu_active_governor;
	}

	/*
	 * Fall back to the governor the device came up with when a
	 * requested governor is known to be unregistered or immutable
	 * in this kernel.  Without this, devfreq_set_governor() fails
	 * with -EINVAL and we re-attempt every tick (visible as a 5s
	 * 'dummy -> simple_ondemand' flap in dmesg on MTK GKI builds
	 * lacking the governor).
	 *
	 * Note: once a governor is marked unavailable, the governor-name
	 * params are effectively locked until reboot -- we stop probing
	 * the failing name to keep the flap dead.
	 */
	if (!game_active && gpu_active_gov_missing &&
	    (!strcmp(target, gpu_active_governor) ||
	     !strcmp(target, gpu_idle_governor)))
		target = gpu_native_governor;
	else if (game_active && gpu_game_gov_missing &&
		 !strcmp(target, gpu_game_governor))
		target = gpu_native_governor;

	/* Only switch if the governor actually changed */
	if (strcmp(df->governor_name, target)) {
		ret = devfreq_set_governor(df, target);
		if (ret == -EINVAL) {
			/*
			 * Governor not built into this kernel, or the
			 * device's governor is immutable.  Remember it so
			 * we stop retrying, and settle back on the native
			 * governor instead of leaving devfreq half-switched.
			 */
			if (game_active)
				gpu_game_gov_missing = true;
			else if (!strcmp(target, gpu_active_governor) ||
				 !strcmp(target, gpu_idle_governor))
				gpu_active_gov_missing = true;
			if (!gpu_gov_warned) {
				pr_warn("zenith_gpu_switch: %s: governor '%s' "
					"not available or immutable, keeping "
					"'%s'\n",
					dev_name(&df->dev), target,
					gpu_native_governor);
				gpu_gov_warned = true;
			}
			if (strcmp(df->governor_name, gpu_native_governor))
				devfreq_set_governor(df, gpu_native_governor);
		} else if (ret) {
			pr_warn("zenith_gpu_switch: %s: set governor '%s' "
				"failed (%d)\n", dev_name(&df->dev), target,
				ret);
		} else {
			pr_info("zenith_gpu_switch: %s: %s -> %s%s\n",
				dev_name(&df->dev), df->governor_name, target,
				game_active ? " (game on)" : "");
		}
	} else {
		gpu_debug("%s: already %s (game=%d)\n",
			  dev_name(&df->dev), target, game_active);
	}

resched:
	/* If the module is shutting down, skip re-queue so that
	 * cancel_delayed_work_sync converges on the first call.
	 * Without this check the work re-queues itself before the
	 * cancel completes, leaving a pending work item that would
	 * fire after module teardown — use-after-free.
	 */
	if (unlikely(gpu_exiting))
		return;

	/* Reschedule at 5s for idle/wake watchdog.
	 * The game_mode notifier handles immediate state transitions,
	 * so this is just a low-frequency safety net for TTL expiry
	 * and GPU idle/timeout tracking.
	 */
	queue_delayed_work(system_unbound_wq, &gpu_governor_work,
			   msecs_to_jiffies(5000));
}

static int gpu_game_mode_notifier_cb(struct notifier_block *nb,
				 unsigned long action, void *data)
{
	mod_delayed_work(system_unbound_wq, &gpu_governor_work, 0);
	printk(KERN_INFO "GrayRavens: zenith_gpu_switch: game mode %s\n",
	       action ? "ON" : "OFF");
	return NOTIFY_OK;
}

static int __init zenith_gpu_switch_init(void)
{
	gpu_game_mode_nb.notifier_call = gpu_game_mode_notifier_cb;
	gpu_game_mode_nb.priority = 0;
	zenith_register_game_mode_notifier(&gpu_game_mode_nb);

	gpu_last_game_active_jiffies = jiffies;
	INIT_DELAYED_WORK(&gpu_governor_work, gpu_governor_worker);
	queue_delayed_work(system_unbound_wq, &gpu_governor_work, 0);

	pr_info("zenith_gpu_switch: watching '%s' (notifier + 5s watchdog)\n",
		gpu_devfreq_name);
	return 0;
}

static void __exit zenith_gpu_switch_exit(void)
{
	/* Set the exit flag first so the work function stops
	 * re-queuing itself.  Then unregister the notifier
	 * (so no new events trigger mod_delayed_work), and
	 * finally cancel any in-flight or re-queued work.
	 */
	gpu_exiting = true;
	zenith_unregister_game_mode_notifier(&gpu_game_mode_nb);
	cancel_delayed_work_sync(&gpu_governor_work);
	pr_info("zenith_gpu_switch: stopped\n");
}

module_init(zenith_gpu_switch_init);
module_exit(zenith_gpu_switch_exit);

MODULE_DESCRIPTION("Zenith GPU devfreq governor auto-switcher");
MODULE_AUTHOR("Codebuff");
MODULE_LICENSE("GPL v2");
