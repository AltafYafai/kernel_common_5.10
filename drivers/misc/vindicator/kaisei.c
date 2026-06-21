// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/vindicator/kaisei.c
 * Kaisei (回生) — panic-to-recovery redirector
 *
 * Registers a panic notifier that intercepts kernel panics, redirecting
 * the subsequent reboot into recovery mode via do_kernel_restart() with
 * a configurable command string.
 *
 * If the primary command fails, platform-specific restart handlers are
 * tried in a fallback chain (recovery → bootloader → download).  If all
 * fail, the notifier returns and the normal panic flow falls through to
 * emergency_restart().
 *
 * Panic reason and statistics are exposed via /sys/kernel/kaisei/ and,
 * when CONFIG_VINDICATOR_HERALD is enabled, published as Android
 * properties.
 *
 * === Cross-reboot bootloop protection ===
 * Kaisei persists a bootloop counter across reboots using the last 64
 * bytes of the ramoops reserved-memory region (found via device tree).
 * After N consecutive boots that end in failed recovery, Kaisei
 * disables itself and logs a cleartext message to the console so the
 * next boot is clean.
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
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/types.h>

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

static bool kaisei_panic_on_oops;
module_param_named(panic_on_oops, kaisei_panic_on_oops, bool, 0644);
MODULE_PARM_DESC(panic_on_oops,
		 "Force panic_on_oops=1 on init so BUG() also triggers Kaisei"
		 " (default: false — set to true only after verifying"
		 " do_kernel_restart(\"recovery\") works on your platform)");

static bool kaisei_bootloop_protection = true;
module_param_named(bootloop_protection, kaisei_bootloop_protection, bool, 0644);
MODULE_PARM_DESC(bootloop_protection,
		 "Disable Kaisei after N consecutive cross-reboot failures"
		 " (default: true)");

static unsigned int kaisei_max_failures = 3;
module_param_named(max_failures, kaisei_max_failures, uint, 0644);
MODULE_PARM_DESC(max_failures,
		 "Max consecutive recovery failures before disabling"
		 " (default: 3)");

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
/* Cross-reboot bootloop counter (persisted in ramoops reserved mem)  */
/* ------------------------------------------------------------------ */

#define KAISEI_PERSISTENT_MAGIC	0x4B414953	/* "KAIS" */
#define KAISEI_PERSISTENT_SIZE	64		/* last 64 bytes of ramoops */

/*
 * Stored at the end of the ramoops reserved-memory region so it survives
 * warm reboots (DRAM is not zeroed across emergency_restart / PSCI reset).
 */
struct kaisei_persistent {
	u32 magic;		/* KAISEI_PERSISTENT_MAGIC */
	u32 version;		/* struct version (2) */
	u32 cross_counter;	/* consecutive cross-reboot failed-recovery boots */
	u32 last_panic;		/* 1 = previous boot ended in failed recovery */
	char last_reason[48];	/* truncated panic reason (persisted cross-reboot) */
};

/* Mapped pointer, NULL = not available / not found */
static struct kaisei_persistent *kaisei_persistent;

/* ------------------------------------------------------------------ */
/* Statistics                                                         */
/* ------------------------------------------------------------------ */

static atomic_t kaisei_trigger_count	= ATOMIC_INIT(0);
static atomic_t kaisei_recovery_fails	= ATOMIC_INIT(0);
static u64      kaisei_last_ts;
static char     kaisei_last_reason[256];
static bool     kaisei_disabled;	/* bootloop protection tripped */

/* ------------------------------------------------------------------ */
/* Persistent-memory helpers                                          */
/* ------------------------------------------------------------------ */

/*
 * Map the last 64 bytes of the ramoops reserved-memory region so we can
 * persist a bootloop counter across reboots.
 */
static void *kaisei_map_persistent(void)
{
	struct device_node *np;
	struct resource res;
	void *base;
	phys_addr_t offset;

	np = of_find_compatible_node(NULL, NULL, "ramoops");
	if (!np) {
		pr_info("no ramoops DT node found — cross-reboot bootloop "
			"protection unavailable\n");
		return NULL;
	}

	if (of_address_to_resource(np, 0, &res)) {
		pr_warn("failed to read ramoops resource\n");
		of_node_put(np);
		return NULL;
	}
	of_node_put(np);

	if (resource_size(&res) < KAISEI_PERSISTENT_SIZE) {
		pr_warn("ramoops region too small (%llu bytes) — bootloop "
			"protection unavailable\n",
			resource_size(&res));
		return NULL;
	}

	offset = resource_size(&res) - KAISEI_PERSISTENT_SIZE;
	base = memremap(res.start + offset, KAISEI_PERSISTENT_SIZE,
			MEMREMAP_WB);
	if (!base) {
		pr_warn("failed to memremap ramoops tail — bootloop "
			"protection unavailable\n");
		return NULL;
	}

	return base;
}

static void kaisei_unmap_persistent(void *ptr)
{
	if (ptr)
		memunmap(ptr);
}

/*
 * Read a u32 from persistent memory.  Returns 0 if mapping is unavailable.
 */
static inline u32 kaisei_pread(u32 *field)
{
	if (!kaisei_persistent)
		return 0;
	return READ_ONCE(*field);
}

/*
 * Write a u32 to persistent memory.  Safe for panic context (no
 * allocations, no locking).  Uses smp_wmb() to ensure the store is
 * visible before emergency_restart() performs the PSCI reset.
 * Silent no-op if mapping is unavailable.
 */
static inline void kaisei_pwrite(u32 *field, u32 val)
{
	if (!kaisei_persistent)
		return;
	smp_wmb();
	WRITE_ONCE(*field, val);
}

/*
 * Write a buffer to persistent memory.  Used to persist the panic
 * reason string across reboots.  Safe for panic context.
 */
static inline void kaisei_pwrite_buf(void *dst, const void *src, size_t len)
{
	if (!kaisei_persistent)
		return;
	smp_wmb();
	memcpy(dst, src, len);
}

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

static ssize_t cross_counter_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	u32 val = kaisei_pread(kaisei_persistent ?
			       &kaisei_persistent->cross_counter : NULL);
	return sysfs_emit(buf, "%u\n", val);
}
static struct kobj_attribute cross_counter_attr =
	__ATTR(cross_counter, 0444, cross_counter_show, NULL);

static struct attribute *kaisei_attrs[] = {
	&trigger_count_attr.attr,
	&recovery_failures_attr.attr,
	&last_reason_attr.attr,
	&last_timestamp_attr.attr,
	&status_attr.attr,
	&cross_counter_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(kaisei);

/* ------------------------------------------------------------------ */
/* Cross-reboot bootloop detection                                     */
/* ------------------------------------------------------------------ */

static void kaisei_check_cross_bootloop(void)
{
	u32 magic, ctr, last_panic;

	if (!kaisei_bootloop_protection)
		return;

	magic = kaisei_pread(&kaisei_persistent->magic);
	ctr   = kaisei_pread(&kaisei_persistent->cross_counter);
	last_panic = kaisei_pread(&kaisei_persistent->last_panic);

	if (magic == KAISEI_PERSISTENT_MAGIC && last_panic) {
		/*
		 * The previous boot ended in a failed recovery attempt.
		 * This is the second (or Nth) boot in a row that
		 * crashed and couldn't reach recovery.
		 */
		ctr++;

		if (ctr >= READ_ONCE(kaisei_max_failures)) {
			kaisei_disabled = true;
			pr_emerg("BOOTLOOP DETECTED: %u consecutive boots ended "
				 "in failed recovery — Kaisei self-disabled.\n",
				 ctr);
			pr_emerg("To re-enable on next boot:\n");
			pr_emerg("  # echo 1 > /sys/module/kaisei/parameters/"
				 "enabled\n");
			pr_emerg("  # echo 0 > /sys/kernel/kaisei/cross_counter"
				 "\n");

			/* Reset counter so next manual enable is clean */
			kaisei_pwrite(&kaisei_persistent->cross_counter, 0);
		} else {
			pr_warn("previous boot ended in failed recovery "
				"(counter=%u/%u)\n", ctr,
				READ_ONCE(kaisei_max_failures));
			kaisei_pwrite(&kaisei_persistent->cross_counter, ctr);
		}
	} else {
		/*
		 * Previous boot was clean (no failed recovery).  Reset
		 * the counter.
		 */
		if (magic == KAISEI_PERSISTENT_MAGIC && ctr != 0) {
			pr_info("clean boot — resetting cross-reboot counter "
				"(was %u)\n", ctr);
			kaisei_pwrite(&kaisei_persistent->cross_counter, 0);
		}
	}

	/* Prepare for THIS boot: clear the panic flag */
	kaisei_pwrite(&kaisei_persistent->last_panic, 0);

	/* Write magic on first use, or migrate from v1 */
	if (magic != KAISEI_PERSISTENT_MAGIC) {
		kaisei_pwrite(&kaisei_persistent->magic,
			      KAISEI_PERSISTENT_MAGIC);
		kaisei_pwrite(&kaisei_persistent->version, 2);
	} else if (magic == KAISEI_PERSISTENT_MAGIC &&
		   kaisei_pread(&kaisei_persistent->version) < 2) {
		/* Old v1 struct: clear the new last_reason field */
		memset(kaisei_persistent->last_reason, 0,
		       sizeof(kaisei_persistent->last_reason));
		kaisei_pwrite(&kaisei_persistent->version, 2);
	}
}

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
	 * never return.
	 */
	for (cmd = kaisei_fallback_cmds; *cmd; cmd++) {
		const char *active_cmd = (cmd == kaisei_fallback_cmds)
					 ? kaisei_cmd : *cmd;

		pr_emerg("trying restart command '%s'\n", active_cmd);
		do_kernel_restart((char *)active_cmd);

		pr_warn("do_kernel_restart(\"%s\") returned — trying next\n",
			active_cmd);
	}

	/*
	 * All commands failed.  Mark this boot as a failed recovery so
	 * the cross-reboot counter increments on next boot.
	 */
	fails = atomic_inc_return(&kaisei_recovery_fails);

	pr_warn("all restart commands failed (%d consecutive failures)\n",
		fails);

	/* In-session bootloop protection */
	if (kaisei_bootloop_protection &&
	    fails >= READ_ONCE(kaisei_max_failures)) {
		kaisei_disabled = true;
		pr_emerg("Kaisei disabled after %d failed recovery attempts "
			 "this session\n", fails);
	}

	/* Persist the failure for cross-reboot detection */
	kaisei_pwrite(&kaisei_persistent->last_panic, 1);

	/*
	 * Persist the panic reason string so it can be published via
	 * Herald on the next boot.
	 */
	kaisei_pwrite_buf(kaisei_persistent->last_reason,
			  kaisei_last_reason,
			  min(sizeof(kaisei_persistent->last_reason),
			      sizeof(kaisei_last_reason)));

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

	/*
	 * Map persistent memory BEFORE registering the notifier so
	 * kaisei_pwrite() is available immediately.
	 */
	kaisei_persistent = kaisei_map_persistent();

	/* Cross-reboot bootloop check */
	kaisei_check_cross_bootloop();

	/* If bootloop protection tripped, don't register the notifier */
	if (kaisei_disabled) {
		pr_emerg("self-disabled due to bootloop detection\n");
		/* Still create sysfs so the user can inspect/re-enable */
		goto create_sysfs;
	}

	/*
	 * P0 (opt-in): if the user explicitly enabled this, force
	 * panic_on_oops so BUG() also triggers Kaisei.
	 */
	if (kaisei_panic_on_oops) {
		WRITE_ONCE(panic_on_oops, 1);
		pr_info("forced panic_on_oops=1\n");
	} else {
		pr_info("panic_on_oops not forced (set kaisei.panic_on_oops=1 "
			"to enable)\n");
	}

	/* Register panic notifier */
	ret = atomic_notifier_chain_register(&panic_notifier_list,
					     &kaisei_panic_nb);
	if (ret) {
		pr_err("failed to register panic notifier (err=%d)\n", ret);
		goto err_unmap;
	}

create_sysfs:
	/* Create sysfs directory /sys/kernel/kaisei/ */
	kaisei_kobj = kobject_create_and_add("kaisei", kernel_kobj);
	if (!kaisei_kobj) {
		pr_err("failed to create sysfs entry\n");
		if (!kaisei_disabled) {
			/* Non-fatal if the notifier is already registered */
		}
	} else {
		ret = sysfs_create_groups(kaisei_kobj, kaisei_groups);
		if (ret) {
			pr_warn("failed to create sysfs groups (err=%d)\n", ret);
			kobject_put(kaisei_kobj);
			kaisei_kobj = NULL;
		}
	}

#ifdef CONFIG_VINDICATOR_HERALD
	{
		char buf[32];

		if (kaisei_disabled)
			herald_set_prop("sys.kaisei.status",
				       "disabled (bootloop)");
		else
			herald_set_prop("sys.kaisei.status", "active");

		herald_set_prop("sys.kaisei.cmd", kaisei_cmd);

		snprintf(buf, sizeof(buf), "%u",
			 READ_ONCE(kaisei_max_failures));
		herald_set_prop("sys.kaisei.max_failures", buf);

		snprintf(buf, sizeof(buf), "%u",
			 kaisei_pread(&kaisei_persistent->cross_counter));
		herald_set_prop("sys.kaisei.cross_counter", buf);

		/* Publish last panic reason from persistent storage */
		if (kaisei_persistent &&
		    kaisei_persistent->last_reason[0])
			herald_set_prop("sys.kaisei.last_reason",
				       kaisei_persistent->last_reason);

		pr_info("herald properties published\n");
	}
#endif

	if (kaisei_disabled)
		pr_warn("loaded but disabled (see dmesg for bootloop details)\n");
	else
		pr_info("active (cmd=\"%s\", panic_on_oops=%s, "
			"bootloop=%s, max_fails=%u)%s\n",
			kaisei_cmd,
			kaisei_panic_on_oops ? "yes" : "no",
			kaisei_bootloop_protection &&
				kaisei_persistent ? "on" : "off",
			kaisei_max_failures,
			kaisei_persistent ? "" :
				" (no cross-reboot protection)");
	return 0;

err_unmap:
	kaisei_unmap_persistent(kaisei_persistent);
	kaisei_persistent = NULL;
	return ret;
}
late_initcall(kaisei_init);

static void __exit kaisei_exit(void)
{
	if (!kaisei_disabled)
		atomic_notifier_chain_unregister(&panic_notifier_list,
						 &kaisei_panic_nb);

	if (kaisei_kobj) {
		sysfs_remove_groups(kaisei_kobj, kaisei_groups);
		kobject_put(kaisei_kobj);
	}

	kaisei_unmap_persistent(kaisei_persistent);
	kaisei_persistent = NULL;

	pr_info("unloaded\n");
}
module_exit(kaisei_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GrayRavens");
MODULE_DESCRIPTION("Kaisei (回生) — panic-to-recovery redirector");
