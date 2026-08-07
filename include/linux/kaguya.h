/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_KAGUYA_H
#define _LINUX_KAGUYA_H

#ifdef CONFIG_KAGUYA_CMDLINE_SPOOF
/**
 * kaguya_spoof_boot_args() - rewrite detection-sensitive boot arguments
 * @str: raw boot text (saved_command_line / saved_boot_config)
 * @bootconfig: true for the /proc/bootconfig "key = \"value\"" format,
 *		false for the /proc/cmdline key=value format
 *
 * Returns a kmalloc'd copy of @str with verifiedbootstate, device_state,
 * flash.locked, veritymode, selinux and warranty_bit rewritten to the
 * values kaguya enforces via resetprop, or NULL on allocation failure
 * (the caller must fall back to the original text).  The caller owns
 * the returned buffer and must kfree() it.
 *
 * Recovery processes always get the unmodified text; init-family
 * processes keep the real veritymode (see kernel/kaguya.c).
 */
char *kaguya_spoof_boot_args(const char *str, bool bootconfig);
#endif

#endif /* _LINUX_KAGUYA_H */
