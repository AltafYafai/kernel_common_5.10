// SPDX-License-Identifier: GPL-2.0-only
/*
 * zenith_readahead - Dynamic readahead tuning for app launch speed
 *
 * Adjusts the default readahead size based on the active Zenith profile.
 * When GAMING or PERFORMANCE profile is active, readahead is increased
 * to 512 KB for faster game asset loading.  When BATTERY profile is
 * active, readahead is reduced to 64 KB for lower I/O overhead.
 *
 * Reads /sys/block/<disk>/queue/read_ahead_kb and adjusts periodically.
 *
 * Fully automatic — zero manual tuning required.
 */
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/printk.h>
#include <linux/cpufreq_zenith.h>

static bool ra_enabled __read_mostly = true;
static unsigned int ra_interval_ms __read_mostly = 5000;
static unsigned int ra_game_kb __read_mostly = 512;
static unsigned int ra_perf_kb __read_mostly = 256;
static unsigned int ra_balanced_kb __read_mostly = 128;
static unsigned int ra_battery_kb __read_mostly = 64;
static char ra_disk[64] __read_mostly = "mmcblk0";

module_param_named(enabled, ra_enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable dynamic readahead (default: true)");
module_param_named(interval_ms, ra_interval_ms, uint, 0644);
MODULE_PARM_DESC(interval_ms, "Poll interval in ms (default: 5000)");
module_param_named(game_kb, ra_game_kb, uint, 0644);
MODULE_PARM_DESC(game_kb, "Readahead KB for GAMING profile (default: 512)");
module_param_named(perf_kb, ra_perf_kb, uint, 0644);
MODULE_PARM_DESC(perf_kb, "Readahead KB for PERFORMANCE profile (default: 256)");
module_param_named(balanced_kb, ra_balanced_kb, uint, 0644);
MODULE_PARM_DESC(balanced_kb, "Readahead KB for BALANCED profile (default: 128)");
module_param_named(battery_kb, ra_battery_kb, uint, 0644);
MODULE_PARM_DESC(battery_kb, "Readahead KB for BATTERY profile (default: 64)");
module_param_string(disk, ra_disk, sizeof(ra_disk), 0644);
MODULE_PARM_DESC(disk, "Block device name (default: mmcblk0)");

static struct task_struct *ra_thread;
static unsigned int ra_last_written;

static unsigned int ra_get_target(void)
{
	unsigned int profile = zenith_get_active_profile();

	switch (profile) {
	case ZENITH_PROFILE_GAMING:
		return READ_ONCE(ra_game_kb);
	case ZENITH_PROFILE_PERFORMANCE:
		return READ_ONCE(ra_perf_kb);
	case ZENITH_PROFILE_BATTERY:
		return READ_ONCE(ra_battery_kb);
	default:
		return READ_ONCE(ra_balanced_kb);
	}
}

static void ra_do_write(unsigned int kb)
{
	struct file *file;
	char buf[32];
	char path[128];
	loff_t pos = 0;
	int len;

	snprintf(path, sizeof(path), "/sys/block/%s/queue/read_ahead_kb",
		 ra_disk);

	file = filp_open(path, O_WRONLY, 0);
	if (IS_ERR(file))
		return;

	len = snprintf(buf, sizeof(buf), "%u\n", kb);
	kernel_write(file, buf, len, &pos);
	filp_close(file, NULL);
}

static int ra_worker(void *data)
{
	while (!kthread_should_stop()) {
		if (READ_ONCE(ra_enabled)) {
			unsigned int target = ra_get_target();

			if (target != ra_last_written) {
				ra_do_write(target);
				ra_last_written = target;
			}
		}

		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(READ_ONCE(ra_interval_ms)));
		__set_current_state(TASK_RUNNING);
	}
	return 0;
}

static int __init ra_init(void)
{
	/* Auto-detect block device if still at default */
	if (!strcmp(ra_disk, "mmcblk0")) {
		struct file *f;
		const char *candidates[] = {"sda", "sdb", "nvme0n1", "mmcblk0", NULL};
		int i;

		for (i = 0; candidates[i]; i++) {
			char path[64];
			snprintf(path, sizeof(path),
				 "/sys/block/%s/queue/read_ahead_kb", candidates[i]);
			f = filp_open(path, O_RDONLY, 0);
			if (!IS_ERR(f)) {
				filp_close(f, NULL);
				strscpy(ra_disk, candidates[i], sizeof(ra_disk));
				break;
			}
		}
	}

	ra_thread = kthread_run(ra_worker, NULL, "zenith_readahead");
	if (IS_ERR(ra_thread)) {
		pr_err("zenith_readahead: failed to start worker\n");
		return PTR_ERR(ra_thread);
	}

	pr_info("zenith_readahead: disk=%s interval=%ums\n",
		ra_disk, READ_ONCE(ra_interval_ms));
	return 0;
}

static void __exit ra_exit(void)
{
	if (ra_thread && !IS_ERR(ra_thread))
		kthread_stop(ra_thread);
}

module_init(ra_init);
module_exit(ra_exit);

MODULE_DESCRIPTION("Zenith Dynamic Readahead — auto-tune by profile");
MODULE_AUTHOR("XTENSEI");
MODULE_LICENSE("GPL v2");
