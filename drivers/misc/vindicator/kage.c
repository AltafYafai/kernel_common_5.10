// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/vindicator/kage.c
 * Kage — SUSFS path and mount hiding helper
 *
 * Provides a kernel API to dynamically hide filesystem paths and mounts
 * via SUSFS (KernelSU Security File System). All operations are chained
 * through Kiryuu to execute /data/adb/ksud with the appropriate susfs
 * subcommands, bypassing SELinux constraints.
 *
 * Author: GrayRavens
 * Co-authored-by: Kanagawa Yamada <albert.wesley.dion@gmail.com>
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/fs.h>

extern int kiryuu_exec(const char *cmd);

/*
 * Path to the ksud binary. Override via module param if KernelSU
 * is installed at a non-default location.
 */
static char ksud_path[256] = "/data/adb/ksud";
module_param_string(ksud_path, ksud_path, sizeof(ksud_path), 0644);
MODULE_PARM_DESC(ksud_path, "Path to the ksud binary");

/*
 * Lazy KSU availability check: cached after first probe.
 * 0 = not checked, 1 = available, -1 = not found.
 */
static int ksu_available;
static DEFINE_SPINLOCK(ksu_check_lock);

static bool ksu_is_available(void)
{
	struct file *f;
	unsigned long flags;
	int cached;

	spin_lock_irqsave(&ksu_check_lock, flags);
	cached = ksu_available;
	spin_unlock_irqrestore(&ksu_check_lock, flags);

	if (cached)
		return cached > 0;

	/* Probe once at first use */
	f = filp_open(ksud_path, O_RDONLY, 0);
	if (IS_ERR(f)) {
		spin_lock_irqsave(&ksu_check_lock, flags);
		ksu_available = -1;
		spin_unlock_irqrestore(&ksu_check_lock, flags);
		return false;
	}
	filp_close(f, NULL);

	spin_lock_irqsave(&ksu_check_lock, flags);
	ksu_available = 1;
	spin_unlock_irqrestore(&ksu_check_lock, flags);
	return true;
}

#define kage_pr_cmd_result(fmt, ...) \
	pr_debug("kage: " fmt, ##__VA_ARGS__)

/**
 * kage_hide_path - Hide a filesystem path via SUSFS
 * @path: Absolute path to hide (e.g. "/data/adb/modules")
 *
 * Returns: 0 on success, negative errno on error
 */
int kage_hide_path(const char *path)
{
	char cmd[256];

	if (!path)
		return -EINVAL;

	snprintf(cmd, sizeof(cmd), "%s susfs add_sus_path '%s'", ksud_path, path);

	if (!ksu_is_available())
		return -ENOENT;

	kage_pr_cmd_result("hiding path via SUSFS -> %s\n", path);
	return kiryuu_exec(cmd);
}
EXPORT_SYMBOL_GPL(kage_hide_path);

/**
 * kage_unhide_path - Remove a path from SUSFS hiding
 * @path: Previously hidden absolute path
 *
 * Returns: 0 on success, negative errno on error
 */
int kage_unhide_path(const char *path)
{
	char cmd[256];

	if (!path)
		return -EINVAL;

	snprintf(cmd, sizeof(cmd), "%s susfs rm_sus_path '%s'", ksud_path, path);

	if (!ksu_is_available())
		return -ENOENT;

	kage_pr_cmd_result("unhiding path via SUSFS -> %s\n", path);
	return kiryuu_exec(cmd);
}
EXPORT_SYMBOL_GPL(kage_unhide_path);

/**
 * kage_hide_mount - Hide a mount point via SUSFS
 * @path: Mount path to hide
 *
 * Returns: 0 on success, negative errno on error
 */
int kage_hide_mount(const char *path)
{
	char cmd[256];

	if (!path)
		return -EINVAL;

	snprintf(cmd, sizeof(cmd), "%s susfs add_sus_mount '%s'", ksud_path, path);

	if (!ksu_is_available())
		return -ENOENT;

	kage_pr_cmd_result("hiding mount via SUSFS -> %s\n", path);
	return kiryuu_exec(cmd);
}
EXPORT_SYMBOL_GPL(kage_hide_mount);

/**
 * kage_unhide_mount - Restore a previously hidden mount point
 * @path: Mount path to unhide
 *
 * Returns: 0 on success, negative errno on error
 */
int kage_unhide_mount(const char *path)
{
	char cmd[256];

	if (!path)
		return -EINVAL;

	snprintf(cmd, sizeof(cmd), "%s susfs rm_sus_mount '%s'", ksud_path, path);

	if (!ksu_is_available())
		return -ENOENT;

	kage_pr_cmd_result("unhiding mount via SUSFS -> %s\n", path);
	return kiryuu_exec(cmd);
}
EXPORT_SYMBOL_GPL(kage_unhide_mount);

/**
 * kage_hide_package - Convenience wrapper to hide common package paths
 * @pkg_name: Android package name (e.g. "com.google.android.gms")
 *
 * Adds standard SUSFS paths for hiding root from a specific app.
 * Returns: 0 on success, negative errno on error
 */
int kage_hide_package(const char *pkg_name)
{
	char cmd[512];
	int ret;

	if (!pkg_name)
		return -EINVAL;

	snprintf(cmd, sizeof(cmd),
		 "%s susfs add_sus_package '%s'", ksud_path, pkg_name);

	if (!ksu_is_available())
		return -ENOENT;

	kage_pr_cmd_result("hiding root from package -> %s\n", pkg_name);
	ret = kiryuu_exec(cmd);

	return ret;

}
EXPORT_SYMBOL_GPL(kage_hide_package);

static int __init kage_init(void)
{
	pr_debug("kage: SUSFS path/mount hiding helper initialized\n");
	return 0;
}

late_initcall(kage_init);

static void __exit kage_exit(void)
{
	pr_info("kage: unloaded\n");
}
module_exit(kage_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GrayRavens");
MODULE_DESCRIPTION("Kage — SUSFS path and mount hiding helper");
