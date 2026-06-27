/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * include/linux/zenith_per_app.h - Per-app Zenith profile auto-switcher.
 *
 * Automatically switches the active Zenith governor profile when
 * a matching application enters or leaves the top-app cgroup.
 *
 * Fully automatic — zero manual configuration required, but users
 * can add custom rules via /proc/zenith/per_app_profiles if desired.
 */
#ifndef _LINUX_ZENITH_PER_APP_H
#define _LINUX_ZENITH_PER_APP_H

struct task_struct;

#ifdef CONFIG_ZENITH_PER_APP

void zenith_per_app_hook_enqueue(struct task_struct *p);

#else /* !CONFIG_ZENITH_PER_APP */

static inline void zenith_per_app_hook_enqueue(struct task_struct *p) { }

#endif /* CONFIG_ZENITH_PER_APP */

#endif /* _LINUX_ZENITH_PER_APP_H */
