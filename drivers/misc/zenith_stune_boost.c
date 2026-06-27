// SPDX-License-Identifier: GPL-2.0-only
/*
 * zenith_stune_boost - Top-app stune boost
 *
 * When a task enters the top-app cgroup (foreground), writes a
 * configurable boost value to the stune cgroup control file,
 * making EAS prefer higher frequencies for foreground tasks.
 *
 * Tunables (/sys/module/zenith_stune_boost/parameters/):
 *   enabled       — master switch (1/0, default 1)
 *   boost_value   — schedtune.boost value (0-100, default 50)
 *   stune_path    — path to stune cgroup (default: /dev/stune)
 */
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/fdtable.h>
#include <linux/cgroup.h>
#include <linux/cpuset.h>
#include <linux/sched.h>

static bool stune_boost_enabled __read_mostly = true;
static unsigned int stune_boost_value __read_mostly = 50;
static char stune_path[256] = "/dev/stune";
static char top_app_path[256] = "/dev/stune/top-app";
static char boost_file[256] = "/dev/stune/top-app/schedtune.boost";

module_param_named(enabled, stune_boost_enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable stune boost (default: true)");
module_param_named(boost_value, stune_boost_value, uint, 0644);
MODULE_PARM_DESC(boost_value, "schedtune.boost value 0-100 (default: 50)");
module_param_string(stune_path, stune_path, sizeof(stune_path), 0644);
MODULE_PARM_DESC(stune_path, "Path to stune cgroup root (default: /dev/stune)");

static void stune_write_boost(unsigned int value)
{
	struct file *file;
	char buf[8];
	int len;
	loff_t pos = 0;

	if (!READ_ONCE(stune_boost_enabled))
		return;

	file = filp_open(boost_file, O_WRONLY, 0);
	if (IS_ERR(file))
		return;

	len = snprintf(buf, sizeof(buf), "%u\n", value);
	kernel_write(file, buf, len, &pos);
	filp_close(file, NULL);
}

static int stune_cgroup_events(struct notifier_block *nb,
			       unsigned long event, void *data)
{
	struct cgroup_subsys_state *css = data;

	if (event != CGROUP_EVENT_ATTACH)
		return NOTIFY_OK;

	if (!css || !css->cgroup || !css->cgroup->kn)
		return NOTIFY_OK;

	/* Match top-app cgroup by name */
	if (strcmp(css->cgroup->kn->name, "top-app") == 0) {
		unsigned long count = cgroup_task_count(css->cgroup);

		if (count > 0)
			stune_write_boost(READ_ONCE(stune_boost_value));
		else
			stune_write_boost(0);
	}

	return NOTIFY_OK;
}

static struct notifier_block stune_cgroup_nb = {
	.notifier_call = stune_cgroup_events,
};

static int __init stune_boost_init(void)
{
	int ret;

	/* Apply initial boost if top-app already has tasks */
	{
		struct cgroup_subsys_state *css;

		rcu_read_lock();
		css = cgroup_get_from_path("/top-app");
		if (!IS_ERR(css)) {
			if (cgroup_task_count(css->cgroup) > 0)
				stune_write_boost(READ_ONCE(stune_boost_value));
			cgroup_put(css);
		}
		rcu_read_unlock();
	}

	/* Register cgroup events notifier */
	ret = cgroup_add_event_notifier(&stune_cgroup_nb);
	if (ret) {
		pr_err("zenith_stune_boost: cgroup notifier failed (%d)\n", ret);
		return ret;
	}

	pr_info("zenith_stune_boost: boost=%u path=%s\n",
		READ_ONCE(stune_boost_value), boost_file);
	return 0;
}

static void __exit stune_boost_exit(void)
{
	cgroup_remove_event_notifier(&stune_cgroup_nb);
	stune_write_boost(0);
}

module_init(stune_boost_init);
module_exit(stune_boost_exit);

MODULE_DESCRIPTION("Zenith Stune Boost — boost top-app schedtune");
MODULE_AUTHOR("XTENSEI");
MODULE_LICENSE("GPL v2");
