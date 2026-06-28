/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * include/linux/vindicator.h
 * Vindicator — sysfs / procfs enforcement framework.
 *
 * Kernel modules register enforce-callbacks that Vindicator's
 * hrtimer-driven watchdog re-applies at a configurable interval,
 * fighting vendor init.rc / HALs that revert kernel tunables.
 *
 * Author: GrayRavens
 */
#ifndef _LINUX_VINDICATOR_H
#define _LINUX_VINDICATOR_H

#include <linux/types.h>
#include <linux/jiffies.h>

/**
 * struct vindicator_enforce_ops - callback interface for an enforcement target.
 * @name:        Human-readable name (used in debugfs & log).
 * @enforce:     Called by the watchdog to re-apply the target value(s).
 *               Must be safe to call in atomic context (hrtimer callback).
 * @enforce_get: Optional.  Reads back the current value for debugfs
 *               reporting.  Returns 0 on success, -errno on failure.
 * @data:        Private data pointer passed to both callbacks.
 */
struct vindicator_enforce_ops {
	const char             *name;
	void (*enforce)(void *data);
	int  (*enforce_get)(void *data, char *buf, size_t size);
	void *data;
};

/**
 * vindicator_register - Register an enforcement target.
 * @ops: Callback bundle (must remain stable until unregister).
 *
 * Returns 0 on success, -EBUSY if the watchdog is already saturated,
 * -ENOMEM on allocation failure.
 *
 * Context: Any (may sleep).
 */
int vindicator_register(const struct vindicator_enforce_ops *ops);

/**
 * vindicator_unregister - Remove a previously registered target.
 * @ops: Must match the & passed to vindicator_register().
 *
 * Context: Any (may sleep).  Safe to call from module exit.
 */
void vindicator_unregister(const struct vindicator_enforce_ops *ops);

#endif /* _LINUX_VINDICATOR_H */
