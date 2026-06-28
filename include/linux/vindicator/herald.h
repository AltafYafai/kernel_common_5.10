/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * include/linux/vindicator/herald.h
 * Herald — SELinux-safe property relay from kernel to userspace.
 *
 * Kernel modules queue property changes via herald_set_prop();
 * the values are exposed under /sys/kernel/herald/ for a userspace
 * daemon or init.rc script to pick up and apply with setprop(1).
 *
 * Author: GrayRavens
 */
#ifndef _LINUX_HERALD_H
#define _LINUX_HERALD_H

#define HERALD_PROP_NAME_MAX  64
#define HERALD_PROP_VAL_MAX   92

/**
 * herald_set_prop - Queue an Android property change.
 * @name: Property name  (e.g. "debug.graphics.game_default_frame_rate").
 * @val:  Property value (e.g. "disabled").
 *
 * Returns 0 on success, -ENOMEM if the queue is full,
 * -EINVAL on bad arguments.
 *
 * Safe to call from atomic context.
 */
int herald_set_prop(const char *name, const char *val);

/**
 * herald_del_prop - Remove a pending property from the queue.
 * @name: Property name to remove.
 */
void herald_del_prop(const char *name);

#endif /* _LINUX_HERALD_H */
