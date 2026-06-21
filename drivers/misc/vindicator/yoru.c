// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/vindicator/yoru.c
 * Yoru (夜) — watchdog panic generator
 *
 * Starts a kernel timer on boot.  Userspace must periodically pet the
 * watchdog by writing to /sys/module/yoru/parameters/pet.  If the timer
 * expires, Yoru calls panic("Yoru: watchdog timeout") — which Kaisei
 * (回生) intercepts and redirects into recovery mode.
 *
 * Together Yoru + Kaisei cover both kernel panics AND hangs:
 *   - Kernel panic → Kaisei redirects to recovery
 *   - Kernel hang   → Yoru triggers panic → Kaisei redirects to recovery
 *
 * Module parameters:
 *   enabled   - Enable/disable the watchdog (default: true)
 *   timeout   - Watchdog timeout in seconds (default: 60)
 *   pet       - Write any value to pet the watchdog (write-only)
 *
 * Author: GrayRavens
 */

#define pr_fmt(fmt) "yoru: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/atomic.h>

/* ------------------------------------------------------------------ */
/* Tunables                                                           */
/* ------------------------------------------------------------------ */

static bool yoru_enabled = true;
module_param_named(enabled, yoru_enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable Yoru watchdog (default: true)");

static unsigned int yoru_timeout = 60;
module_param_named(timeout, yoru_timeout, uint, 0644);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds (default: 60)");

/* ------------------------------------------------------------------ */
/* Timer                                                              */
/* ------------------------------------------------------------------ */

static struct timer_list yoru_timer;

static void yoru_timer_cb(struct timer_list *t)
{
	panic("Yoru: watchdog timeout");
}

static void yoru_start(void)
{
	if (READ_ONCE(yoru_enabled))
		mod_timer(&yoru_timer,
			  jiffies + msecs_to_jiffies(READ_ONCE(yoru_timeout) * 1000));
}

/* ------------------------------------------------------------------ */
/* Pet — write-only module param with callback                        */
/* ------------------------------------------------------------------ */

static int yoru_pet_set(const char *val, const struct kernel_param *kp)
{
	unsigned int dummy;
	int ret;

	ret = kstrtouint(val, 0, &dummy);
	if (ret)
		return ret;

	if (READ_ONCE(yoru_enabled)) {
		pr_debug("petted (timeout=%us)\n", READ_ONCE(yoru_timeout));
		yoru_start();
	}

	return 0;
}

static const struct kernel_param_ops yoru_pet_ops = {
	.set = yoru_pet_set,
	.get = param_get_uint,
};
static unsigned int yoru_pet_val;
module_param_cb(pet, &yoru_pet_ops, &yoru_pet_val, 0200);
MODULE_PARM_DESC(pet, "Write any value to pet the watchdog (write-only)");

/* ------------------------------------------------------------------ */
/* Module init / exit                                                 */
/* ------------------------------------------------------------------ */

static int __init yoru_init(void)
{
	timer_setup(&yoru_timer, yoru_timer_cb, 0);
	yoru_start();

	pr_info("active (timeout=%us)%s\n", yoru_timeout,
		yoru_enabled ? "" : " — disabled at boot");
	return 0;
}
late_initcall(yoru_init);

static void __exit yoru_exit(void)
{
	del_timer_sync(&yoru_timer);
	pr_info("unloaded\n");
}
module_exit(yoru_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GrayRavens");
MODULE_DESCRIPTION("Yoru (夜) — watchdog panic generator");
