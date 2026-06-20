// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/vindicator/kaisei.c
 * Kaisei (回生) — panic-to-recovery redirector
 *
 * Registers a panic notifier that intercepts kernel panics and BUG()
 * conditions (via panic_on_oops), redirecting the subsequent reboot into
 * recovery mode via do_kernel_restart() with a configurable command.
 *
 * If the primary command fails, platform-specific restart handlers are
 * tried in a fallback chain (recovery → bootloader → download).  If all
 * fail, Kaisei's notifier returns and the normal panic flow continues
 * to emergency_restart().
 *
 * Panic reason and statistics are exposed via /sys/kernel/kaisei/ and,
 * when Herald is enabled, published as Android properties.
 *
 * Author: GrayRavens
 * Co-authored-by: Kanagawa Yamada <albert.wesley.dion@gmail.com>
 */

#define pr_fmt(fmt) "kaisei: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/string.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/atomic.h>
#include <linux/ktime.h>
extern struct atomic_notifier_head panic_notifier_list;

#include <linux/vindicator/herald.h>

/* ------------------------------------------------------------------ */
/* Tunables                                                           */
/* ------------------------------------------------------------------ */

static bool kaisei_enabled = true;
module_param_named(enabled, kaisei_enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable Kaisei panic-to-recovery (default: true)");

static char kaisei_cmd[64] = "recovery";
module_param_string(cmd, kaisei_cmd, sizeof(kaisei_cmd), 0644);
MODULE_PARM_DESC(cmd, "Primary reboot command (default: recovery)");

static bool kaisei_bootloop_protection = true;
module_param_named(bootloop_protection, kaisei_bootloop_protection, bool, 0644);
MODULE_PARM_DESC(bootloop_protection,
		 "Disable Kaisei after N consecutive failures to prevent bootloops"
		 " (default: true)");

static unsigned int kaisei_max_failures = 3;
module_param_named(max_failures, kaisei_max_failures, uint, 0644);
MODULE_PARM_DESC(max_failures,
		 "Max consecutive recovery failures before disabling (default: 3)");

/* ------------------------------------------------------------------ */
/* Fallback command table                                             */
/* ------------------------------------------------------------------ */

static const char * const kaisei_fallback_cmds[] = {
	"recovery",
	"bootloader",
	"download",
	NULL,		/* sentinel */
};

/* ------------------------------------------------------------------ */
/* Statistics                                                         */
/* ------------------------------------------------------------------ */

static atomic_t kaisei_trigger_count	= ATOMIC_INIT(0);
static atomic_t kaisei_recovery_fails	= ATOMIC_INIT(0);
static u64      kaisei_last_ts;
static char     kaisei_last_reason[256];
static bool     kaisei_disabled;	/* bootloop protection tripped */

/* ------------------------------------------------------------------ */
/* Sysfs                                                              */
/* ------------------------------------------------------------------ */

static struct kobject *kaisei_kobj;

static ssize_t trigger_count_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", atomic_read(&kaisei_trigger_count));
}
static struct kobj_attribute trigger_count_attr =
	__ATTR(trigger_count, 0444, trigger_count_show, NULL);

static ssize_t recovery_failures_show(struct kobject *kobj,
				      struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", atomic_read(&kaisei_recovery_fails));
}
static struct kobj_attribute recovery_failures_attr =
	__ATTR(recovery_failures, 0444, recovery_failures_show, NULL);

static ssize_t last_reason_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%s\n", kaisei_last_reason);
}
static struct kobj_attribute last_reason_attr =
	__ATTR(last_reason, 0444, last_reason_show, NULL);

static ssize_t last_timestamp_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%llu\n", kaisei_last_ts);
}
static struct kobj_attribute last_timestamp_attr =
	__ATTR(last_timestamp, 0444, last_timestamp_show, NULL);

static ssize_t status_show(struct kobject *kobj,
			   struct kobj_attribute *attr, char *buf)
{
	if (kaisei_disabled)
		return sysfs_emit(buf, "disabled (bootloop protection)\n");
	if (!READ_ONCE(kaisei_enabled))
		return sysfs_emit(buf, "disabled (user)\n");
	return sysfs_emit(buf, "active (cmd=%s, fails=%d, trigs=%d)\n",
			  kaisei_cmd,
			  atomic_read(&kaisei_recovery_fails),
			  atomic_read(&kaisei_trigger_count));
}
static struct kobj_attribute status_attr =
	__ATTR(status, 0444, status_show, NULL);

static struct attribute *kaisei_attrs[] = {
	&trigger_count_attr.attr,
	&recovery_failures_attr.attr,
	&last_reason_attr.attr,
	&last_timestamp_attr.attr,
	&status_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(kaisei);

/* ------------------------------------------------------------------ */
/* Panic notifier                                                     */
/* ------------------------------------------------------------------ */

static int kaisei_panic_cb(struct notifier_block *nb,
			   unsigned long action, void *data)
{
	const char * const *cmd;
	int fails;

	if (kaisei_disabled || !READ_ONCE(kaisei_enabled))
		return NOTIFY_DONE;

	/* Record the panic reason */
	if (data)
		strscpy(kaisei_last_reason, (const char *)data,
			sizeof(kaisei_last_reason));
	else
		strscpy(kaisei_last_reason, "(unknown)",
			sizeof(kaisei_last_reason));

	kaisei_last_ts = ktime_get_real_fast_ns();
	atomic_inc(&kaisei_trigger_count);

	pr_emerg("panic detected — reason: %s\n", kaisei_last_reason);

	/*
	 * Try each command in the fallback chain.  If any succeeds, the
	 * platform restart handler triggers an immediate reboot and we
	 * never return — the system is already rebooting into recovery.
	 */

	for (cmd = kaisei_fallback_cmds; *cmd; cmd++) {
		/* Use the module param only when on the primary command */
		const char *active_cmd = (cmd == kaisei_fallback_cmds)
					 ? kaisei_cmd : *cmd;

		pr_emerg("trying restart command '%s'\n", active_cmd);
		do_kernel_restart((char *)active_cmd);

		/*
		 * If we get here, the handler didn't restart. Log it and
		 * try the next fallback.
		 */
		pr_warn("do_kernel_restart(\"%s\") returned — trying next\n",
			active_cmd);
	}

	/* All commands failed */
	fails = atomic_inc_return(&kaisei_recovery_fails);

	pr_warn("all restart commands failed (%d consecutive failures)\n",
		fails);

	/* Bootloop protection: disable self after too many failures */
	if (kaisei_bootloop_protection &&
	    fails >= READ_ONCE(kaisei_max_failures)) {
		kaisei_disabled = true;
		pr_emerg("bootloop protection: Kaisei disabled after %d "
			 "failed recovery attempts\n", fails);
		pr_emerg("Re-enable via: echo 1 > "
			 "/sys/module/kaisei/parameters/enabled\n");
	}

	return NOTIFY_DONE;
}

static struct notifier_block kaisei_panic_nb = {
	.notifier_call = kaisei_panic_cb,
	.priority = INT_MAX,	/* run early in the panic notifier chain */
};

/* ------------------------------------------------------------------ */
/* Module init / exit                                                 */
/* ------------------------------------------------------------------ */

static int __init kaisei_init(void)
{
	int ret;

	/* P0: auto-enable panic_on_oops to catch BUG() as well */
	WRITE_ONCE(panic_on_oops, 1);
	pr_info("enabled panic_on_oops\n");

	/* Create sysfs directory /sys/kernel/kaisei/ */
	kaisei_kobj = kobject_create_and_add("kaisei", kernel_kobj);
	if (!kaisei_kobj) {
		pr_err("failed to create sysfs entry\n");
		/* Non-fatal: continue without sysfs */
	} else {
		ret = sysfs_create_groups(kaisei_kobj, kaisei_groups);
		if (ret) {
			pr_warn("failed to create sysfs groups (err=%d)\n", ret);
			kobject_put(kaisei_kobj);
			kaisei_kobj = NULL;
		}
	}

	/* Register panic notifier */
	ret = atomic_notifier_chain_register(&panic_notifier_list,
					     &kaisei_panic_nb);
	if (ret) {
		pr_err("failed to register panic notifier (err=%d)\n", ret);
		goto err_sysfs;
	}

#ifdef CONFIG_VINDICATOR_HERALD
	{
		char buf[32];

		herald_set_prop("sys.kaisei.status", "active");
		herald_set_prop("sys.kaisei.cmd", kaisei_cmd);

		snprintf(buf, sizeof(buf), "%u",
			 READ_ONCE(kaisei_max_failures));
		herald_set_prop("sys.kaisei.max_failures", buf);

		snprintf(buf, sizeof(buf), "%d",
			 atomic_read(&kaisei_trigger_count));
		herald_set_prop("sys.kaisei.trigger_count", buf);

		pr_info("herald properties published\n");
	}
#endif

	pr_info("active (cmd=\"%s\", bootloop=%s, max_fails=%u)\n",
		kaisei_cmd,
		kaisei_bootloop_protection ? "on" : "off",
		kaisei_max_failures);
	return 0;

err_sysfs:
	if (kaisei_kobj) {
		sysfs_remove_groups(kaisei_kobj, kaisei_groups);
		kobject_put(kaisei_kobj);
	}
	return ret;
}
late_initcall(kaisei_init);

static void __exit kaisei_exit(void)
{
	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &kaisei_panic_nb);

	if (kaisei_kobj) {
		sysfs_remove_groups(kaisei_kobj, kaisei_groups);
		kobject_put(kaisei_kobj);
	}

	pr_info("unloaded\n");
}
module_exit(kaisei_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GrayRavens");
MODULE_DESCRIPTION("Kaisei (回生) — panic-to-recovery redirector");
