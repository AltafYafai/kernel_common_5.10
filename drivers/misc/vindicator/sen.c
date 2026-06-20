// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/vindicator/sen.c
 * Sen — Touch-based gaming frequency boost
 *
 * Intercepts touch input events and temporarily boosts CPU frequency
 * to maximum via the schedutil vendor hook. The boost duration is
 * configurable and automatically refreshed on each new touch event.
 *
 * Author: GrayRavens
 * Co-authored-by: Kanagawa Yamada <albert.wesley.dion@gmail.com>
 */

#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/input.h>
#include <trace/hooks/sched.h>

#define SEN_BOOST_DURATION_MS	1000

static bool sen_enabled = true;
module_param(sen_enabled, bool, 0644);
MODULE_PARM_DESC(sen_enabled, "Enable Sen gaming boost (default: true)");

static unsigned int sen_boost_ms = SEN_BOOST_DURATION_MS;
module_param(sen_boost_ms, uint, 0644);
MODULE_PARM_DESC(sen_boost_ms, "Touch boost duration in ms (default: 1000)");

static bool sen_is_boosted;
static DEFINE_SPINLOCK(sen_lock);
static struct delayed_work sen_boost_off_work;

/* Hook pointer exported by the input subsystem */
extern void (*sen_touch_hook)(unsigned int type, unsigned int code);

static void sen_vh_map_util_freq(void *data, unsigned long util,
				 unsigned long freq, unsigned long cap,
				 unsigned long *next_freq,
				 struct cpufreq_policy *policy,
				 bool *need_freq_update)
{
	unsigned long flags;
	bool active;

	if (!sen_enabled)
		return;

	spin_lock_irqsave(&sen_lock, flags);
	active = sen_is_boosted;
	spin_unlock_irqrestore(&sen_lock, flags);

	if (active && policy && next_freq && need_freq_update) {
		*next_freq = policy->cpuinfo.max_freq;
		*need_freq_update = true;
	}
}

static void sen_boost_off(struct work_struct *work)
{
	unsigned long flags;

	spin_lock_irqsave(&sen_lock, flags);
	sen_is_boosted = false;
	spin_unlock_irqrestore(&sen_lock, flags);

	pr_debug("sen: touch boost OFF\n");
}

static void sen_trigger_boost(unsigned int type, unsigned int code)
{
	unsigned long flags;

	if (!sen_enabled)
		return;

	/*
	 * Only fire on actual touch events, not on keyboard,
	 * gamepad, power button, or other non-touch inputs.
	 * Filter for absolute multi-touch position events.
	 */
	if (type != EV_ABS)
		return;
	if (code != ABS_MT_POSITION_X && code != ABS_MT_POSITION_Y &&
	    code != ABS_MT_TRACKING_ID && code != ABS_MT_TOUCH_MAJOR &&
	    code != ABS_MT_WIDTH_MAJOR && code != ABS_PRESSURE)
		return;

	spin_lock_irqsave(&sen_lock, flags);
	if (!sen_is_boosted)
		pr_debug("sen: touch boost ON\n");
	sen_is_boosted = true;
	spin_unlock_irqrestore(&sen_lock, flags);

	mod_delayed_work(system_wq, &sen_boost_off_work,
			 msecs_to_jiffies(sen_boost_ms));
}

static int __init sen_init(void)
{
	int ret;

	INIT_DELAYED_WORK(&sen_boost_off_work, sen_boost_off);

	ret = register_trace_android_vh_map_util_freq(
			sen_vh_map_util_freq, NULL);
	if (ret) {
		pr_err("sen: failed to register vendor hook (err=%d)\n", ret);
		return ret;
	}

	sen_touch_hook = sen_trigger_boost;

	pr_info("sen: active (boost=%ums)\n", sen_boost_ms);
	return 0;
}
late_initcall(sen_init);

static void __exit sen_exit(void)
{
	sen_touch_hook = NULL;

	unregister_trace_android_vh_map_util_freq(
			sen_vh_map_util_freq, NULL);
	cancel_delayed_work_sync(&sen_boost_off_work);

	pr_info("sen: unloaded\n");
}
module_exit(sen_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GrayRavens");
MODULE_DESCRIPTION("Sen — Touch-based gaming frequency boost");
