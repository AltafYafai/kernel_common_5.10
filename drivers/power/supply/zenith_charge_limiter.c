// SPDX-License-Identifier: GPL-2.0-only
/*
 * zenith_charge_limiter - Smart charging for battery longevity
 *
 * Monitors battery capacity via the power supply class.  When the
 * battery reaches limit_percent, disables charging by writing 0 to
 * the charger's input_suspend control file.  When capacity drops to
 * resume_percent, re-enables charging.
 *
 * Tunables (/sys/module/zenith_charge_limiter/parameters/):
 *   enabled         — master switch (1/0, default 0)
 *   limit_percent   — stop charging above this % (50-99, default 80)
 *   resume_percent  — resume charging below this % (50-99, default 75)
 *   suspend_node    — charger control file path (default: auto-detect)
 */
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/namei.h>

static bool charge_limiter_enabled;
static unsigned int charge_limit_percent __read_mostly = 80;
static unsigned int charge_resume_percent __read_mostly = 75;
static char charge_suspend_node[256] __read_mostly;

static struct delayed_work charge_work;
static bool charge_suspended;

module_param_named(enabled, charge_limiter_enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable charge limiter (default: false)");
module_param_named(limit_percent, charge_limit_percent, uint, 0644);
MODULE_PARM_DESC(limit_percent, "Stop charging above this %% (50-99, default 80)");
module_param_named(resume_percent, charge_resume_percent, uint, 0644);
MODULE_PARM_DESC(resume_percent, "Resume charging below this %% (50-99, default 75)");
module_param_string(suspend_node, charge_suspend_node,
		    sizeof(charge_suspend_node), 0644);
MODULE_PARM_DESC(suspend_node,
		 "Charger suspend control file (default: auto-detect)");

static void charge_set_suspend(bool suspend)
{
	struct file *file;
	char buf[4];
	loff_t pos = 0;

	file = filp_open(charge_suspend_node, O_WRONLY, 0);
	if (IS_ERR(file))
		return;

	buf[0] = suspend ? '1' : '0';
	buf[1] = '\n';
	kernel_write(file, buf, 2, &pos);
	filp_close(file, NULL);

	charge_suspended = suspend;
}

static void charge_worker(struct work_struct *work)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int cap, limit, resume;

	if (!READ_ONCE(charge_limiter_enabled)) {
		if (charge_suspended)
			charge_set_suspend(false);
		goto resched;
	}

	limit  = READ_ONCE(charge_limit_percent);
	resume = READ_ONCE(charge_resume_percent);

	if (limit < 50 || limit > 99 || resume < 50 || resume > 99)
		goto resched;

	psy = power_supply_get_by_name("battery");
	if (IS_ERR_OR_NULL(psy))
		goto resched;

	if (psy->desc->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val))
		goto put;
	cap = val.intval;

	if (psy->desc->get_property(psy, POWER_SUPPLY_PROP_STATUS, &val))
		goto put;

	if (val.intval == POWER_SUPPLY_STATUS_CHARGING ||
	    val.intval == POWER_SUPPLY_STATUS_FULL) {
		if (cap >= limit && !charge_suspended)
			charge_set_suspend(true);
	} else if (val.intval == POWER_SUPPLY_STATUS_NOT_CHARGING ||
		   val.intval == POWER_SUPPLY_STATUS_DISCHARGING) {
		if (cap <= resume && charge_suspended)
			charge_set_suspend(false);
	}

put:
	power_supply_put(psy);
resched:
	queue_delayed_work(system_unbound_wq, &charge_work,
			   msecs_to_jiffies(10000));
}

static int __init charge_limiter_init(void)
{
	/* Auto-detect suspend node */
	if (!charge_suspend_node[0]) {
		/* Common paths for Qualcomm and MediaTek chargers */
		const char *paths[] = {
			"/sys/class/power_supply/battery/input_suspend",
			"/sys/class/power_supply/battery/charging_enabled",
			"/sys/class/qcom-bms/input_suspend",
			NULL
		};
		int i;

		for (i = 0; paths[i]; i++) {
			struct path p;

			if (kern_path(paths[i], 0, &p) == 0) {
				path_put(&p);
				strscpy(charge_suspend_node, paths[i],
					sizeof(charge_suspend_node));
				break;
			}
		}

		if (!charge_suspend_node[0]) {
			pr_warn("zenith_charge_limiter: no suspend node found, "
				"set manually via suspend_node param\n");
		}
	}

	INIT_DELAYED_WORK(&charge_work, charge_worker);
	schedule_delayed_work(&charge_work, msecs_to_jiffies(30000));

	pr_info("zenith_charge_limiter: limit=%u%% resume=%u%% node=%s\n",
		READ_ONCE(charge_limit_percent),
		READ_ONCE(charge_resume_percent),
		charge_suspend_node[0] ? charge_suspend_node : "(none)");
	return 0;
}

static void __exit charge_limiter_exit(void)
{
	cancel_delayed_work_sync(&charge_work);
	if (charge_suspended)
		charge_set_suspend(false);
}

module_init(charge_limiter_init);
module_exit(charge_limiter_exit);

MODULE_DESCRIPTION("Zenith Charge Limiter — battery longevity");
MODULE_AUTHOR("XTENSEI");
MODULE_LICENSE("GPL v2");
