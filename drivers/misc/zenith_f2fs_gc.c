// SPDX-License-Identifier: GPL-2.0-only
/*
 * zenith_f2fs_gc - Profile-aware F2FS garbage collection trigger
 *
 * When the active Zenith profile switches to GAMING (or PERFORMANCE),
 * this module polls the governor and writes to /sys/fs/f2fs/<disk>/
 * gc_urgent to request aggressive GC.  When the profile switches away,
 * it restores gc_urgent to 0.
 *
 * Prevents in-game I/O stalls from background GC by front-loading
 * garbage collection before gameplay begins.
 *
 * Tunables (/sys/module/zenith_f2fs_gc/parameters/):
 *   enabled     — master switch (1/0, default 1)
 *   profiles    — comma-separated profile numbers that trigger GC
 *                 default: "5,1" (GAMING, PERFORMANCE)
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/dcache.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/cpufreq_zenith.h>

static bool f2fs_gc_enabled __read_mostly = true;
static char f2fs_gc_profiles[64] __read_mostly = "5,1";

module_param_named(enabled, f2fs_gc_enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable F2FS GC trigger (default: true)");
module_param_string(profiles, f2fs_gc_profiles, sizeof(f2fs_gc_profiles), 0644);
MODULE_PARM_DESC(profiles, "Comma-separated profile IDs that trigger GC (default: 5,1 = GAMING,PERFORMANCE)");

/* Profile IDs: 0=CUSTOM, 1=PERFORMANCE, 2=BALANCED, 3=BATTERY,
 * 4=LEGACY, 5=GAMING, 6=AUDIO, 7=AUTO
 */
#define MAX_TRIGGER_PROFILES 8

static unsigned int trigger_profiles[MAX_TRIGGER_PROFILES];
static int nr_trigger_profiles;

/* Polling kthread */
static struct task_struct *f2fs_gc_task;
static unsigned int last_profile = ZENITH_PROFILE_BALANCED;

static bool profile_matches(unsigned int active)
{
	int i;

	for (i = 0; i < nr_trigger_profiles; i++) {
		if (trigger_profiles[i] == active)
			return true;
	}
	return false;
}

static void f2fs_parse_profiles(void)
{
	char *buf, *tok;

	nr_trigger_profiles = 0;
	buf = kstrdup(f2fs_gc_profiles, GFP_KERNEL);
	if (!buf)
		return;

	tok = strsep(&buf, ",");
	while (tok && nr_trigger_profiles < MAX_TRIGGER_PROFILES) {
		unsigned int val;

		while (*tok == ' ')
			tok++;
		if (kstrtouint(tok, 10, &val) == 0)
			trigger_profiles[nr_trigger_profiles++] = val;
		tok = strsep(&buf, ",");
	}
	kfree(buf);
}

/* Write value to all known /sys/fs/f2fs/<disk>/gc_urgent paths */
static void f2fs_set_gc_urgent(unsigned int value)
{
	static const char * const disks[] = {
		"mmcblk0", "mmcblk1", "sda", "sdb",
		"nvme0n1", "nvme0n2", NULL
	};
	char val_buf[8];
	int len;
	int i;

	len = snprintf(val_buf, sizeof(val_buf), "%u\n", value);

	for (i = 0; disks[i]; i++) {
		struct file *f;
		char path[64];

		snprintf(path, sizeof(path),
			 "/sys/fs/f2fs/%s/gc_urgent", disks[i]);

		f = filp_open(path, O_WRONLY, 0);
		if (IS_ERR(f))
			continue;

		kernel_write(f, val_buf, len, 0);
		filp_close(f, NULL);

		pr_debug("zenith_f2fs_gc: wrote %s -> %s\n", val_buf, path);
	}
}

/* Polling kthread: check profile every 2s and trigger GC when needed */
static int f2fs_gc_kthread(void *data)
{
	while (!kthread_should_stop()) {
		unsigned int current_profile;

		if (!READ_ONCE(f2fs_gc_enabled)) {
			schedule_timeout_interruptible(HZ * 2);
			continue;
		}

		current_profile = zenith_get_active_profile();

		if (current_profile != last_profile) {
			bool should_gc = profile_matches(current_profile);
			bool was_gc = profile_matches(last_profile);

			if (should_gc && !was_gc) {
				/* Entering gaming/perf — trigger urgent GC */
				f2fs_set_gc_urgent(1);
				pr_debug("zenith_f2fs_gc: GC urgent ON (profile=%u)\n",
					 current_profile);
			} else if (!should_gc && was_gc) {
				/* Leaving gaming/perf — restore normal GC */
				f2fs_set_gc_urgent(0);
				pr_debug("zenith_f2fs_gc: GC urgent OFF (profile=%u)\n",
					 current_profile);
			}
			last_profile = current_profile;
		}

		schedule_timeout_interruptible(HZ * 2);
	}
	return 0;
}

static int __init f2fs_gc_init(void)
{
	f2fs_parse_profiles();

	last_profile = zenith_get_active_profile();

	f2fs_gc_task = kthread_run(f2fs_gc_kthread, NULL,
				   "zenith_f2fs_gc");
	if (IS_ERR(f2fs_gc_task)) {
		pr_err("zenith_f2fs_gc: failed to start kthread\n");
		return PTR_ERR(f2fs_gc_task);
	}

	pr_info("zenith_f2fs_gc: initialized (profiles=%s)\n",
		f2fs_gc_profiles);
	return 0;
}

static void __exit f2fs_gc_exit(void)
{
	if (f2fs_gc_task) {
		kthread_stop(f2fs_gc_task);
		f2fs_gc_task = NULL;
	}

	/* Restore default GC on exit */
	f2fs_set_gc_urgent(0);
	pr_info("zenith_f2fs_gc: unloaded\n");
}

module_init(f2fs_gc_init);
module_exit(f2fs_gc_exit);

MODULE_DESCRIPTION("Zenith F2FS GC Trigger — profile-aware garbage collection");
MODULE_AUTHOR("XTENSEI");
MODULE_LICENSE("GPL v2");
