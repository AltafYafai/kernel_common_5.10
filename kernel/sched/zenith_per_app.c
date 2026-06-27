// SPDX-License-Identifier: GPL-2.0-only
/*
 * zenith_per_app - Fully automatic per-app Zenith profile switching
 *
 * Called from the scheduler enqueue path (enqueue_task_fair) whenever
 * a task enters a runqueue.  If the task is in the top-app cgroup,
 * checks the task's comm (process name) against a built-in lookup table
 * and automatically switches the active Zenith profile.
 *
 * When the matched app leaves the top-app cgroup, the profile is
 * automatically reverted to the previously saved profile.
 *
 * Users can add/remove rules at runtime via /proc/zenith/per_app_profiles
 * (see the procfs interface) but the module works with zero configuration
 * and zero manual tuning.  Built-in default rules detect common gaming
 * and camera applications automatically.
 *
 * Profile IDs: 0=CUSTOM, 1=PERFORMANCE, 2=BALANCED, 3=BATTERY,
 *              4=LEGACY, 5=GAMING, 6=AUDIO, 7=AUTO
 */
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/cgroup.h>
#include <linux/cpufreq_zenith.h>

#define MAX_RULES 64
#define MAX_NAME_LEN 128

struct app_profile_rule {
	char name[MAX_NAME_LEN];
	unsigned int profile;
};

static struct app_profile_rule *app_rules;
static int app_nr_rules;
static DEFINE_MUTEX(app_rules_lock);

/* Track the currently active app for revert-on-exit */
static char current_app[MAX_NAME_LEN];
static unsigned int saved_profile = ZENITH_PROFILE_BALANCED;

/* ---- Built-in default rules (zero config needed) ---- */
static const struct app_profile_rule default_rules[] = {
	/* Gaming */
	{ "com.genshinimpact",			ZENITH_PROFILE_GAMING },
	{ "com.miHoYo.GenshinImpact",		ZENITH_PROFILE_GAMING },
	{ "com.miHoYo.Yuanshen",		ZENITH_PROFILE_GAMING },
	{ "com.tencent.tmgp.sgame",		ZENITH_PROFILE_GAMING },
	{ "com.tencent.tmgp.pubgmhd",		ZENITH_PROFILE_GAMING },
	{ "com.pubg.krmobile",			ZENITH_PROFILE_GAMING },
	{ "com.pubg.imobile",			ZENITH_PROFILE_GAMING },
	{ "com.activision.callofduty.shooter",	ZENITH_PROFILE_GAMING },
	{ "com.epicgames.fortnite",		ZENITH_PROFILE_GAMING },
	{ "com.mojang.minecraftpe",		ZENITH_PROFILE_GAMING },
	{ "com.riotgames.league.wildrift",	ZENITH_PROFILE_GAMING },
	{ "com.tencent.KiHan",			ZENITH_PROFILE_GAMING },
	/* Camera */
	{ "com.google.android.apps.camera",	ZENITH_PROFILE_PERFORMANCE },
	{ "org.codeaurora.snapcam",		ZENITH_PROFILE_PERFORMANCE },
	/* Audio */
	{ "com.spotify.music",			ZENITH_PROFILE_AUDIO },
	{ "com.apple.android.music",		ZENITH_PROFILE_AUDIO },
};
#define NR_DEFAULT_RULES (sizeof(default_rules) / sizeof(default_rules[0]))

/* ---- Lookup ---- */

int zenith_per_app_match(const char *comm)
{
	int i;

	if (!comm || !comm[0])
		return -1;

	for (i = 0; i < app_nr_rules; i++) {
		if (!strncmp(comm, app_rules[i].name, MAX_NAME_LEN - 1))
			return app_rules[i].profile;
	}
	return -1;
}
EXPORT_SYMBOL_GPL(zenith_per_app_match);

void zenith_per_app_foreground(const char *comm)
{
	int profile;

	if (!comm || !comm[0])
		return;

	profile = zenith_per_app_match(comm);
	if (profile >= 0) {
		strscpy(current_app, comm, sizeof(current_app));
		saved_profile = zenith_get_active_profile();
		zenith_set_profile(profile);
		pr_info_ratelimited("zenith_per_app: %s -> profile %d\n",
				    comm, profile);
	}
}
EXPORT_SYMBOL_GPL(zenith_per_app_foreground);

void zenith_per_app_background(const char *comm)
{
	if (!comm || !comm[0])
		return;

	if (!strncmp(current_app, comm, MAX_NAME_LEN - 1)) {
		char buf[MAX_NAME_LEN];
		unsigned int restore = saved_profile;

		strscpy(buf, current_app, sizeof(buf));
		current_app[0] = '\0';
		zenith_set_profile(restore);
		pr_info_ratelimited("zenith_per_app: %s exited, reverted to %d\n",
				    buf, restore);
	}
}
EXPORT_SYMBOL_GPL(zenith_per_app_background);

/*
 * zenith_per_app_hook_enqueue - Called from enqueue_task_fair() under rq_lock.
 *
 * Detects top-app cgroup membership by walking the task's cgroup path.
 * When a process moves into top-app, checks its comm against the rule
 * table and auto-switches profiles.
 */
void zenith_per_app_hook_enqueue(struct task_struct *p)
{
	struct cgroup_subsys_state *css;
	char comm[TASK_COMM_LEN];

	if (!p)
		return;

	rcu_read_lock();
	css = task_css(p, cpu_cgrp_id);
	if (!css || !css->cgroup || !css->cgroup->kn) {
		rcu_read_unlock();
		return;
	}

	if (strcmp(css->cgroup->kn->name, "top-app") == 0) {
		get_task_comm(comm, p);
		rcu_read_unlock();
		zenith_per_app_foreground(comm);
	} else {
		rcu_read_unlock();
	}
}
EXPORT_SYMBOL_GPL(zenith_per_app_hook_enqueue);

/* ---- procfs interface ---- */

static void app_clear_rules(void)
{
	mutex_lock(&app_rules_lock);
	app_nr_rules = NR_DEFAULT_RULES;
	mutex_unlock(&app_rules_lock);
}

static int app_add_rule(const char *str)
{
	char name[MAX_NAME_LEN];
	unsigned int profile;
	int ret;

	if (app_nr_rules >= MAX_RULES)
		return -ENOSPC;

	ret = sscanf(str, "%127[^=]=%u", name, &profile);
	if (ret != 2 || profile > 7)
		return -EINVAL;

	mutex_lock(&app_rules_lock);
	strscpy(app_rules[app_nr_rules].name, name, MAX_NAME_LEN);
	app_rules[app_nr_rules].profile = profile;
	app_nr_rules++;
	mutex_unlock(&app_rules_lock);

	return 0;
}

static int app_proc_show(struct seq_file *m, void *v)
{
	int i;

	mutex_lock(&app_rules_lock);
	for (i = 0; i < app_nr_rules; i++)
		seq_printf(m, "%s=%u\n",
			   app_rules[i].name, app_rules[i].profile);
	mutex_unlock(&app_rules_lock);

	return 0;
}

static ssize_t app_proc_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	char tmp[256];
	char *p, *tok;
	int ret = count;

	if (count >= sizeof(tmp))
		return -E2BIG;

	if (copy_from_user(tmp, buf, count))
		return -EFAULT;
	tmp[count] = '\0';

	if (count > 0 && tmp[count - 1] == '\n')
		tmp[count - 1] = '\0';

	p = tmp;
	while ((tok = strsep(&p, "\n")) != NULL) {
		while (*tok == ' ' || *tok == '\t')
			tok++;
		if (*tok == '\0')
			continue;

		if (strcmp(tok, "-clear") == 0) {
			app_clear_rules();
		} else if (strcmp(tok, "-reset") == 0) {
			app_clear_rules();
		} else {
			int err = app_add_rule(tok);
			if (err)
				ret = err;
		}
	}

	return ret;
}

static int app_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, app_proc_show, NULL);
}

static const struct proc_ops app_proc_fops = {
	.proc_open    = app_proc_open,
	.proc_read    = seq_read,
	.proc_write   = app_proc_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

static struct proc_dir_entry *app_proc_dir;

static int __init app_init(void)
{
	int i;

	app_rules = kcalloc(MAX_RULES, sizeof(*app_rules), GFP_KERNEL);
	if (!app_rules)
		return -ENOMEM;

	/* Load built-in defaults */
	for (i = 0; i < NR_DEFAULT_RULES; i++)
		app_rules[i] = default_rules[i];
	app_nr_rules = NR_DEFAULT_RULES;

	app_proc_dir = proc_mkdir("zenith", NULL);
	if (!app_proc_dir) {
		pr_err("zenith_per_app: failed to create /proc/zenith\n");
		kfree(app_rules);
		return -ENOMEM;
	}

	proc_create("per_app_profiles", 0644, app_proc_dir, &app_proc_fops);

	pr_info("zenith_per_app: auto mode, %d built-in rules, %d max\n",
		NR_DEFAULT_RULES, MAX_RULES);
	return 0;
}

static void __exit app_exit(void)
{
	remove_proc_entry("per_app_profiles", app_proc_dir);
	remove_proc_entry("zenith", NULL);
	kfree(app_rules);
}

module_init(app_init);
module_exit(app_exit);

MODULE_DESCRIPTION("Zenith Per-App Profile Auto-Switcher (zero tuning, built-in game/camera rules)");
MODULE_AUTHOR("XTENSEI");
MODULE_LICENSE("GPL v2");
