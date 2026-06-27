// SPDX-License-Identifier: GPL-2.0-only
/*
 * zenith_adaptive_boost - Profile-aware input boost extension
 *
 * Dynamically adjusts the input_boost_ms duration based on the
 * active Zenith profile, via /sys/module/zenith_input_boost/
 * parameters/boost_ms.
 *
 *   GAMING / PERFORMANCE → extend boost to gamer_ms (default 150 ms)
 *   BATTERY              → shorten boost to battery_ms (default 25 ms)
 *   Other profiles       → keep the user's configured value
 *
 * Design: uses a polling kthread (same pattern as zenith_stune_boost.c
 * and zenith_f2fs_gc.c) rather than an input notifier, so it works
 * with the existing input_boost module without needing per-event
 * delivery.
 *
 * Tunables (/sys/module/zenith_adaptive_boost/parameters/):
 *   enabled       — master switch (1/0, default 1)
 *   gamer_ms      — boost_ms to apply under GAMING/PERFORMANCE
 *                   (default: 150)
 *   battery_ms    — boost_ms to apply under BATTERY (default: 25)
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/cpufreq_zenith.h>

static bool ab_enabled __read_mostly = true;
static unsigned int ab_gamer_ms __read_mostly = 150;
static unsigned int ab_battery_ms __read_mostly = 25;

module_param_named(enabled, ab_enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable adaptive boost (default: true)");
module_param_named(gamer_ms, ab_gamer_ms, uint, 0644);
MODULE_PARM_DESC(gamer_ms, "Input boost ms under GAMING/PERFORMANCE (default: 150)");
module_param_named(battery_ms, ab_battery_ms, uint, 0644);
MODULE_PARM_DESC(battery_ms, "Input boost ms under BATTERY (default: 25)");

/* Cache the user's original boost_ms so we can restore it */
static unsigned int saved_boost_ms;
static bool saved_valid;
static struct task_struct *ab_task;
static unsigned int last_ab_profile = ZENITH_PROFILE_BALANCED;

/* Write value to input_boost boost_ms sysfs parameter */
static void ab_set_boost_ms(unsigned int ms)
{
	struct file *f;
	char buf[16];
	int len;

	len = snprintf(buf, sizeof(buf), "%u\n", ms);

	f = filp_open("/sys/module/zenith_input_boost/parameters/boost_ms",
		      O_WRONLY, 0);
	if (IS_ERR(f))
		return;

	kernel_write(f, buf, len, 0);
	filp_close(f, NULL);

	pr_debug("zenith_adaptive_boost: boost_ms -> %u\n", ms);
}

/* Read current boost_ms from input_boost sysfs */
static unsigned int ab_get_boost_ms(void)
{
	struct file *f;
	char buf[16];
	loff_t pos = 0;
	int ret;

	f = filp_open("/sys/module/zenith_input_boost/parameters/boost_ms",
		      O_RDONLY, 0);
	if (IS_ERR(f))
		return 50; /* default fallback */

	ret = kernel_read(f, buf, sizeof(buf) - 1, &pos);
	filp_close(f, NULL);

	if (ret <= 0)
		return 50;

	buf[ret] = '\0';
	return simple_strtoul(buf, NULL, 10);
}

/* Polling kthread: check profile every 2s and adjust boost_ms */
static int ab_kthread(void *data)
{
	/* On first run, save the user's original boost_ms value */
	saved_boost_ms = ab_get_boost_ms();
	saved_valid = true;
	last_ab_profile = zenith_get_active_profile();

	while (!kthread_should_stop()) {
		unsigned int profile;

		if (!READ_ONCE(ab_enabled)) {
			schedule_timeout_interruptible(HZ * 2);
			continue;
		}

		profile = zenith_get_active_profile();

		if (profile != last_ab_profile) {
			switch (profile) {
			case ZENITH_PROFILE_GAMING:
			case ZENITH_PROFILE_PERFORMANCE:
				ab_set_boost_ms(READ_ONCE(ab_gamer_ms));
				break;
			case ZENITH_PROFILE_BATTERY:
				ab_set_boost_ms(READ_ONCE(ab_battery_ms));
				break;
			default:
				/* Restore user's original value */
				if (saved_valid)
					ab_set_boost_ms(saved_boost_ms);
				break;
			}
			last_ab_profile = profile;
		}

		schedule_timeout_interruptible(HZ * 2);
	}

	/* Restore original value on exit */
	if (saved_valid)
		ab_set_boost_ms(saved_boost_ms);

	return 0;
}

static int __init ab_init(void)
{
	ab_task = kthread_run(ab_kthread, NULL, "zenith_adaptive_boost");
	if (IS_ERR(ab_task)) {
		pr_err("zenith_adaptive_boost: failed to start kthread\n");
		return PTR_ERR(ab_task);
	}

	pr_info("zenith_adaptive_boost: initialized (gamer=%ums, battery=%ums)\n",
		ab_gamer_ms, ab_battery_ms);
	return 0;
}

static void __exit ab_exit(void)
{
	if (ab_task) {
		kthread_stop(ab_task);
		ab_task = NULL;
	}
	pr_info("zenith_adaptive_boost: unloaded\n");
}

module_init(ab_init);
module_exit(ab_exit);

MODULE_DESCRIPTION("Zenith Adaptive Boost — profile-aware input boost duration");
MODULE_AUTHOR("XTENSEI");
MODULE_LICENSE("GPL v2");
