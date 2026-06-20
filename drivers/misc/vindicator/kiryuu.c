// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/vindicator/kiryuu.c
 * Kiryuu — Native root command execution engine
 *
 * Executes shell commands from kernel context as UID 0 via
 * call_usermodehelper(). Bypasses SELinux constraints by running
 * in the kernel domain — intended for use with KernelSU/SUSFS
 * integration where userspace helpers (ksud, resetprop) must be
 * invoked with full privileges.
 *
 * Author: GrayRavens
 * Co-authored-by: Kanagawa Yamada <albert.wesley.dion@gmail.com>
 */

#include <linux/module.h>
#include <linux/kmod.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/delay.h>

/**
 * kiryuu_exec - Execute a shell command as root (blocking)
 * @cmd: The shell command string to execute
 *
 * Returns: exit code of the command (0 = success), negative errno on error
 *
 * Runs call_usermodehelper() with UMH_WAIT_PROC and a minimal environment
 * that grants PATH access to standard Android binaries. Blocks until the
 * command completes. The calling module must ensure @cmd is not
 * attacker-controlled; no quoting/escaping is performed.
 *
 * WARNING: This function BLOCKS the caller until the command completes.
 * It MUST NOT be called from atomic context, interrupt handlers,
 * while holding a spinlock, or from any context where sleeping is
 * forbidden. Use kiryuu_exec_async() for fire-and-forget cases.
 */
int kiryuu_exec(const char *cmd)
{
	char *argv[] = { "/system/bin/sh", "-c", (char *)cmd, NULL };
	char *envp[] = {
		"HOME=/",
		"PATH=/sbin:/vendor/bin:/system/sbin:/system/bin:/system/xbin",
		"ANDROID_ROOT=/system",
		"ANDROID_DATA=/data",
		NULL
	};
	int ret;

	if (!cmd)
		return -EINVAL;

	pr_debug("kiryuu: executing: %s\n", cmd);

	ret = call_usermodehelper(argv[0], argv, envp, UMH_WAIT_PROC);
	if (ret < 0)
		pr_warn("kiryuu: command failed (err=%d): %s\n", ret, cmd);
	else if (ret > 0)
		pr_debug("kiryuu: command exited with code %d: %s\n", ret, cmd);

	return ret;
}
EXPORT_SYMBOL_GPL(kiryuu_exec);

/**
 * kiryuu_exec_async - Execute a command asynchronously (fire-and-forget)
 * @cmd: The shell command string to execute
 *
 * Returns: 0 on submission success, negative errno on error
 *
 * The command runs as UMH_NO_WAIT — no return code is collected.
 * Useful for one-shot property changes where the exit status is
 * not critical. Safe to call from any context.
 */

int kiryuu_exec_async(const char *cmd)
{
	char *argv[] = { "/system/bin/sh", "-c", (char *)cmd, NULL };
	char *envp[] = {
		"HOME=/",
		"PATH=/sbin:/vendor/bin:/system/sbin:/system/bin:/system/xbin",
		"ANDROID_ROOT=/system",
		"ANDROID_DATA=/data",
		NULL
	};
	int ret;

	if (!cmd)
		return -EINVAL;

	pr_debug("kiryuu: async exec: %s\n", cmd);

	ret = call_usermodehelper(argv[0], argv, envp, UMH_NO_WAIT);
	if (ret < 0)
		pr_warn("kiryuu: async exec failed (err=%d): %s\n", ret, cmd);

	return ret;
}
EXPORT_SYMBOL_GPL(kiryuu_exec_async);

static int __init kiryuu_init(void)
{
	pr_info("kiryuu: Native root execution engine initialized\n");
	return 0;
}
late_initcall(kiryuu_init);

static void __exit kiryuu_exit(void)
{
	pr_info("kiryuu: unloaded\n");
}
module_exit(kiryuu_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GrayRavens");
MODULE_DESCRIPTION("Kiryuu — Native root command execution engine");
