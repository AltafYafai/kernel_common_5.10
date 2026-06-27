// SPDX-License-Identifier: GPL-2.0-only
/*
 * dynamic_fsync - Screen-state aware fsync toggle
 *
 * Disables fsync while the screen is on to eliminate write-back
 * stalls during UI-critical moments.  Re-enables fsync when the
 * screen turns off to guarantee data integrity before suspend.
 *
 * Uses the kernel's CONFIG_DYNAMIC_FSYNC infrastructure: the module
 * writes the screen state into dynamic_fsync_state and the core fsync
 * path (fs/sync.c) checks it before issuing any writeback.
 *
 * Userspace control:
 *   /sys/module/dynamic_fsync/parameters/enabled    (1/0, def 1)
 */
#include <linux/module.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/dynamic_fsync.h>

/*
 * Module parameter stored directly in the kernel's global so the
 * fs/sync.c hot path always reads the authoritative value.
 * module_param_cb with a custom handler is not needed because the
 * bool type already uses the same memory we point it at.
 */
module_param_named(enabled, dynamic_fsync_enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable dynamic fsync (default: true)");

static int dfsync_fb_notifier(struct notifier_block *nb,
			      unsigned long event, void *data)
{
	struct fb_event *ev = data;
	int new_state;

	if (event != FB_EVENT_BLANK)
		return NOTIFY_OK;

	if (!ev || !ev->data)
		return NOTIFY_OK;

	new_state = (*(int *)ev->data == FB_BLANK_UNBLANK) ? 2 : 1;
	WRITE_ONCE(dynamic_fsync_state, new_state);

	return NOTIFY_OK;
}

static struct notifier_block dfsync_fb_nb = {
	.notifier_call = dfsync_fb_notifier,
};

static int __init dfsync_init(void)
{
	int ret;

	/* Default: assume screen is on */
	WRITE_ONCE(dynamic_fsync_state, 2);

	ret = fb_register_client(&dfsync_fb_nb);
	if (ret) {
		pr_err("dynamic_fsync: fb_register_client failed (%d)\n", ret);
		return ret;
	}

	pr_info("dynamic_fsync: enabled=%d, screen=ON\n",
		(int)READ_ONCE(dynamic_fsync_enabled));
	return 0;
}

static void __exit dfsync_exit(void)
{
	fb_unregister_client(&dfsync_fb_nb);
	WRITE_ONCE(dynamic_fsync_state, 0);
}

module_init(dfsync_init);
module_exit(dfsync_exit);

MODULE_DESCRIPTION("Dynamic fsync — disable fsync when screen is on");
MODULE_AUTHOR("XTENSEI");
MODULE_LICENSE("GPL v2");
