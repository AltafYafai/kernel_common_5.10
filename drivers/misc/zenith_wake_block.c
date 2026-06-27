// SPDX-License-Identifier: GPL-2.0-only
/*
 * zenith_wake_block - Automatic kernel wakelock blocker
 *
 * Automatically blocks known battery-draining wakelocks.  Uses two
 * mechanisms:
 *
 *   1. kprobe on __pm_stay_awake — intercepts kernel wakeup sources
 *      at activation time and immediately relaxes blocked ones.
 *   2. Periodic /sys/power/wake_lock scanner — releases userspace
 *      wakelocks that match the blocklist.
 *
 * Fully automatic — no userspace interaction required.
 *
 * Tunables (/sys/module/zenith_wake_block/parameters/):
 *   enabled      — master switch (1/0, default 1)
 *   blocklist    — comma-separated wakelock name substrings to block
 *                  default: "wlan,radio-interface,bluedroid_timer,
 *                            nfc_timer,location,batchedLocation,
 *                            nfc_wake_lock,qcom_rx_wakelock,
 *                            netmgr_wakelock,wlan_wow_wake"
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/kprobes.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/pm_wakeup.h>

static bool wb_enabled __read_mostly = true;
static char wb_blocklist[512] __read_mostly =
	"wlan,radio-interface,bluedroid_timer,nfc_timer,"
	"location,batchedLocation,nfc_wake_lock,"
	"qcom_rx_wakelock,netmgr_wakelock,wlan_wow_wake";
static char wb_wake_lock_path[64] __read_mostly = "/sys/power/wake_lock";
static char wb_wake_unlock_path[64] __read_mostly = "/sys/power/wake_unlock";

module_param_named(enabled, wb_enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable wake lock blocker (default: true)");
module_param_string(blocklist, wb_blocklist, sizeof(wb_blocklist), 0644);
MODULE_PARM_DESC(blocklist, "Comma-separated wakelock name substrings to block");

#define MAX_BLOCKED 32
static char *wb_blocked_names[MAX_BLOCKED];
static int wb_nr_blocked;

/* ---- Blocklist management ---- */

static bool wb_is_blocked(const char *name)
{
	int i;

	if (!name || !name[0])
		return false;

	for (i = 0; i < wb_nr_blocked; i++) {
		if (strstr(name, wb_blocked_names[i]))
			return true;
	}
	return false;
}

static void wb_parse_blocklist(void)
{
	char *buf, *tok;
	int i;

	for (i = 0; i < wb_nr_blocked; i++)
		kfree(wb_blocked_names[i]);
	wb_nr_blocked = 0;

	buf = kstrdup(wb_blocklist, GFP_KERNEL);
	if (!buf)
		return;

	tok = strsep(&buf, ",");
	while (tok && wb_nr_blocked < MAX_BLOCKED) {
		while (*tok == ' ')
			tok++;
		if (*tok) {
			wb_blocked_names[wb_nr_blocked] = kstrdup(tok, GFP_KERNEL);
			if (wb_blocked_names[wb_nr_blocked])
				wb_nr_blocked++;
		}
		tok = strsep(&buf, ",");
	}
	kfree(buf);
}

/* ---- kprobe: intercept __pm_stay_awake ---- */

static struct kprobe wb_kp;

static int wb_kprobe_pre(struct kprobe *kp, struct pt_regs *regs)
{
	struct wakeup_source *ws;

	if (!READ_ONCE(wb_enabled))
		return 0;

	/*
	 * On arm64, the first argument (ws) is in x0.
	 * On x86_64, it's in di.
	 * We use the arch-independent pt_regs macros.
	 * If the arch isn't recognized, kprobe registration fails
	 * and we fall back gracefully.
	 */
#ifdef CONFIG_ARM64
	ws = (struct wakeup_source *)regs->regs[0];
#elif defined(CONFIG_X86_64)
	ws = (struct wakeup_source *)regs->di;
#else
	/* Unsupported arch — skip kprobe interceptor */
	return 0;
#endif

	if (!ws || !ws->name)
		return 0;

	if (wb_is_blocked(ws->name)) {
		pr_debug("zenith_wake_block: blocking %s\n", ws->name);
		__pm_relax(ws);
	}

	return 0;
}

/* ---- Workqueue: periodic userspace wakelock scanner ---- */

static struct delayed_work wb_work;
static struct workqueue_struct *wb_wq;

/*
 * Read /sys/power/wake_lock to get currently held userspace wakelocks,
 * then release any that match the blocklist.
 */
static void wb_scan_userspace_wakelocks(void)
{
	struct file *file;
	char buf[1024];
	loff_t pos = 0;
	ssize_t len;
	char *line;

	if (!READ_ONCE(wb_enabled))
		return;

	file = filp_open(wb_wake_lock_path, O_RDONLY, 0);
	if (IS_ERR(file))
		return;

	len = kernel_read(file, buf, sizeof(buf) - 1, &pos);
	filp_close(file, NULL);
	if (len <= 0)
		return;

	buf[len] = '\0';

	/* Each line is a wakelock name */
	line = strsep(&buf, "\n");
	while (line) {
		while (*line == ' ' || *line == '\t')
			line++;

		if (*line && wb_is_blocked(line)) {
			struct file *ufile;
			loff_t upos = 0;

			ufile = filp_open(wb_wake_unlock_path, O_WRONLY, 0);
			if (!IS_ERR(ufile)) {
				kernel_write(ufile, line, strlen(line), &upos);
				kernel_write(ufile, "\n", 1, &upos);
				filp_close(ufile, NULL);
			}
		}
		line = strsep(&buf, "\n");
	}
}

static void wb_worker(struct work_struct *work)
{
	wb_scan_userspace_wakelocks();
	queue_delayed_work(wb_wq, &wb_work, msecs_to_jiffies(5000));
}

/* ---- Init / Exit ---- */

static int __init wb_init(void)
{
	int ret;

	wb_parse_blocklist();

	if (wb_nr_blocked == 0) {
		pr_info("zenith_wake_block: no wakelocks in blocklist, disabled\n");
		return 0;
	}

	/* Register kprobe on __pm_stay_awake (non-fatal on failure) */
	memset(&wb_kp, 0, sizeof(wb_kp));
	wb_kp.pre_handler = wb_kprobe_pre;
	wb_kp.symbol_name = "__pm_stay_awake";
	ret = register_kprobe(&wb_kp);
	if (ret) {
		pr_info("zenith_wake_block: kprobe unavailable (%d), "
			"using userspace-only scanning\n", ret);
	} else {
		pr_info("zenith_wake_block: kprobe on __pm_stay_awake registered\n");
	}

	/* Start periodic userspace wakelock scanner */
	wb_wq = alloc_ordered_workqueue("zenith_wake_block", WQ_MEM_RECLAIM);
	if (!wb_wq) {
		pr_err("zenith_wake_block: failed to create workqueue\n");
		if (wb_kp.addr)
			unregister_kprobe(&wb_kp);
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&wb_work, wb_worker);
	queue_delayed_work(wb_wq, &wb_work, msecs_to_jiffies(10000));

	pr_info("zenith_wake_block: active, blocking %d patterns\n",
		wb_nr_blocked);
	return 0;
}

static void __exit wb_exit(void)
{
	int i;

	cancel_delayed_work_sync(&wb_work);
	if (wb_wq)
		destroy_workqueue(wb_wq);

	if (wb_kp.addr)
		unregister_kprobe(&wb_kp);

	for (i = 0; i < wb_nr_blocked; i++)
		kfree(wb_blocked_names[i]);
	wb_nr_blocked = 0;
}

module_init(wb_init);
module_exit(wb_exit);

MODULE_DESCRIPTION("Zenith Wake Lock Blocker (kprobe + userspace scanner)");
MODULE_AUTHOR("XTENSEI");
MODULE_LICENSE("GPL v2");
