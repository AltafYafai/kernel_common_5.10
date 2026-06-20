// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/vindicator/kaisei.c
 * Kaisei (回生) — panic-to-recovery redirector
 *
 * Registers a panic notifier that intercepts kernel panics, BUG(), and
 * other fatal errors and redirects the subsequent reboot into recovery
 * mode via do_kernel_restart() with a configurable command string.
 *
 * Platform-specific restart handlers (e.g., qpnp-poweron on Qualcomm,
 * PMIC drivers on MTK/Exynos, PSCI on generic arm64) interpret the
 * command and write the appropriate magic values to signal the bootloader
 * to boot into recovery — no userspace, no block I/O, no reserved memory.
 *
 * If the restart handler is not supported on the platform (returns
 * without restarting), the normal panic flow continues to
 * emergency_restart() and the device reboots normally.
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

extern struct atomic_notifier_head panic_notifier_list;

/* ------------------------------------------------------------------ */
/* Tunables                                                           */
/* ------------------------------------------------------------------ */

static bool kaisei_enabled = true;
module_param_named(enabled, kaisei_enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable Kaisei panic-to-recovery redirect (default: true)");

static char kaisei_cmd[64] = "recovery";
module_param_string(cmd, kaisei_cmd, sizeof(kaisei_cmd), 0644);
MODULE_PARM_DESC(cmd, "Reboot command passed to do_kernel_restart() (default: recovery)");

/* ------------------------------------------------------------------ */
/* Panic notifier                                                     */
/* ------------------------------------------------------------------ */

static int kaisei_panic_cb(struct notifier_block *nb,
			   unsigned long action, void *data)
{
	if (!READ_ONCE(kaisei_enabled))
		return NOTIFY_DONE;

	pr_emerg("panic detected — restarting with command '%s'\n",
		 kaisei_cmd);

	/*
	 * Call into the platform restart handler chain with the configured
	 * command. On supported platforms this writes the appropriate magic
	 * to PMIC/RTC registers (e.g., "recovery" → recovery mode) and
	 * triggers an immediate restart — we never return.
	 *
	 * If the platform doesn't support the command, the handler returns
	 * and we fall through to let the normal panic flow continue with
	 * emergency_restart().
	 */
	do_kernel_restart(kaisei_cmd);

	pr_warn("do_kernel_restart(\"%s\") returned — platform may not "
		"support this command, falling through to emergency_restart\n",
		kaisei_cmd);

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

	ret = atomic_notifier_chain_register(&panic_notifier_list,
					     &kaisei_panic_nb);
	if (ret) {
		pr_err("failed to register panic notifier (err=%d)\n", ret);
		return ret;
	}

	pr_info("active (cmd=\"%s\")\n", kaisei_cmd);
	return 0;
}
late_initcall(kaisei_init);

static void __exit kaisei_exit(void)
{
	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &kaisei_panic_nb);
	pr_info("unloaded\n");
}
module_exit(kaisei_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GrayRavens");
MODULE_DESCRIPTION("Kaisei (回生) — panic-to-recovery redirector");

