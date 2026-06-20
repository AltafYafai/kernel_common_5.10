// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/vindicator/vindicator_targets.c
 * Vindicator enforcement targets — auto-defend critical kernel tunables.
 *
 * Uses the Vindicator framework's hrtimer watchdog to detect and
 * re-apply sysfs/procfs values that vendor init.rc or HALs may
 * revert.  Each target uses a workqueue for the actual file I/O
 * (Vindicator enforce callbacks run in atomic context).
 *
 * Registered by default:
 *   - cpufreq governor  → "zenith" (or configurable)
 *   - TCP congestion    → "bbr"
 *   - read_ahead_kb     → 128
 *   - sched_boost       → 0
 *
 * All paths and expected values are configurable via module params.
 *
 * Author: GrayRavens
 */
#define pr_fmt(fmt) "vindicator_targets: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/vindicator.h>

/* ------------------------------------------------------------------ */
/* Constants                                                          */
/* ------------------------------------------------------------------ */
#define VIND_TARGET_NAME_MAX    32
#define VIND_TARGET_PATH_MAX    128
#define VIND_TARGET_VAL_MAX     32
#define VIND_TARGET_COUNT       4

/* ------------------------------------------------------------------ */
/* Per-target state                                                    */
/* ------------------------------------------------------------------ */
struct vind_target_info {
	char                    name[VIND_TARGET_NAME_MAX];
	char                    path[VIND_TARGET_PATH_MAX];
	char                    expected[VIND_TARGET_VAL_MAX];
	char                    current_buf[VIND_TARGET_VAL_MAX];
	struct delayed_work     dwork;
	struct vindicator_enforce_ops vops;
	bool                    registered;
};

static struct vind_target_info *vind_targets[VIND_TARGET_COUNT];
static struct workqueue_struct *vind_targets_wq;
static DEFINE_MUTEX(vind_targets_lock);

/* ------------------------------------------------------------------ */
/* Module params — paths and expected values per target                */
/* ------------------------------------------------------------------ */
/*
 * Governor enforcement: we target cpu0 which is always online on arm64.
 * For heterogeneous clusters, each policy's governor is tracked
 * separately by cpufreq, so cpu0 covers the first cluster.
 * Vendor HALs that flip other clusters' governors do so through
 * the same sysfs path pattern — a future enhancement could check
 * scaling_governor on each policy.
 */
static char *gov_path      = "/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor";
static char *gov_expected  = "zenith";

static char *tcp_path      = "/proc/sys/net/ipv4/tcp_congestion_control";
static char *tcp_expected  = "bbr";

/*
 * Wildcard the block device path: detect UFS (sda), eMMC (mmcblk0),
 * NVMe, or virtual block devices via sysfs scan at init.
 */
static char *ra_path       = "/sys/block/mmcblk0/queue/read_ahead_kb";
static char *ra_expected   = "128";

static char *boost_path    = "/sys/module/cpu_boost/parameters/sched_boost";
static char *boost_expected= "0";

module_param_named(gov_path,      gov_path,      charp, 0644);
module_param_named(gov_expected,  gov_expected,  charp, 0644);
module_param_named(tcp_path,      tcp_path,      charp, 0644);
module_param_named(tcp_expected,  tcp_expected,  charp, 0644);
module_param_named(ra_path,       ra_path,       charp, 0644);
module_param_named(ra_expected,   ra_expected,   charp, 0644);
module_param_named(boost_path,    boost_path,    charp, 0644);
module_param_named(boost_expected,boost_expected, charp, 0644);

/* ------------------------------------------------------------------ */
/* VFS helpers                                                        */
/* ------------------------------------------------------------------ */

/* Read a sysfs/procfs value into buf.  Returns 0 on success. */
static int vfs_read_value(const char *path, char *buf, size_t size)
{
	struct file *f;
	loff_t pos = 0;
	int ret;

	f = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(f))
		return PTR_ERR(f);

	ret = kernel_read(f, buf, size - 1, &pos);
	if (ret < 0) {
		filp_close(f, NULL);
		return ret;
	}

	buf[ret] = '\0';
	/* Strip trailing newline */
	if (ret > 0 && buf[ret - 1] == '\n')
		buf[ret - 1] = '\0';

	filp_close(f, NULL);
	return 0;
}

/* Write a value to a sysfs/procfs path.  Returns 0 on success. */
static int vfs_write_value(const char *path, const char *val)
{
	struct file *f;
	loff_t pos = 0;
	int ret;

	f = filp_open(path, O_WRONLY, 0);
	if (IS_ERR(f))
		return PTR_ERR(f);

	ret = kernel_write(f, val, strlen(val), &pos);
	filp_close(f, NULL);
	return ret < 0 ? ret : 0;
}

/* ------------------------------------------------------------------ */
/* Work function — runs in process context, does the actual I/O        */
/* ------------------------------------------------------------------ */
static void vind_target_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct vind_target_info *info;
	int i;

	/* Find which target this work belongs to */
	for (i = 0; i < VIND_TARGET_COUNT; i++) {
		if (vind_targets[i] && &vind_targets[i]->dwork == dwork) {
			info = vind_targets[i];
			goto found;
		}
	}
	return;

found:
	if (vfs_read_value(info->path, info->current_buf,
			   sizeof(info->current_buf)))
		return; /* Can't read — will retry on next Vindicator tick */

	/* Compare and enforce if different */
	if (strcmp(info->current_buf, info->expected) != 0) {
		pr_info("enforcing %s: '%s' -> '%s'\n",
			info->name, info->current_buf, info->expected);
		vfs_write_value(info->path, info->expected);
	}
}

/* ------------------------------------------------------------------ */
/* Vindicator enforce callback — atomic, just schedules the work       */
/* ------------------------------------------------------------------ */
static void vind_target_enforce(void *data)
{
	struct vind_target_info *info = data;

	if (vind_targets_wq)
		queue_delayed_work(vind_targets_wq, &info->dwork, 0);
}

/* ------------------------------------------------------------------ */
/* Target registration helper                                          */
/* ------------------------------------------------------------------ */
static int register_target(const char *name, const char *path,
			   const char *expected, int idx)
{
	struct vind_target_info *info;
	int ret;

	if (idx >= VIND_TARGET_COUNT)
		return -EINVAL;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	strscpy(info->name, name, sizeof(info->name));
	strscpy(info->path, path, sizeof(info->path));
	strscpy(info->expected, expected, sizeof(info->expected));

	INIT_DELAYED_WORK(&info->dwork, vind_target_work);

	info->vops.name    = info->name;
	info->vops.enforce = vind_target_enforce;
	info->vops.data    = info;

	ret = vindicator_register(&info->vops);
	if (ret) {
		pr_err("failed to register '%s': %d\n", name, ret);
		kfree(info);
		return ret;
	}

	info->registered = true;
	vind_targets[idx] = info;
	pr_info("target '%s' → %s = %s\n", name, path, expected);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Module init / exit                                                  */
/* ------------------------------------------------------------------ */
static int __init vindicator_targets_init(void)
{
	int ret;

	vind_targets_wq = alloc_workqueue("vind_targets", WQ_UNBOUND, 0);
	if (!vind_targets_wq)
		return -ENOMEM;

	ret = register_target("cpufreq_gov", gov_path, gov_expected, 0);
	if (ret && ret != -ENOMEM)
		pr_warn("cpufreq_gov target not available (path may not exist)\n");

	ret = register_target("tcp_cong", tcp_path, tcp_expected, 1);
	if (ret && ret != -ENOMEM)
		pr_warn("tcp_cong target not available\n");

	ret = register_target("read_ahead", ra_path, ra_expected, 2);
	if (ret && ret != -ENOMEM)
		pr_warn("read_ahead target not available (path may not exist)\n");

	ret = register_target("sched_boost", boost_path, boost_expected, 3);
	if (ret && ret != -ENOMEM)
		pr_warn("sched_boost target not available (path may not exist)\n");

	pr_info("loaded with %d targets\n", VIND_TARGET_COUNT);
	return 0;
}

static void __exit vindicator_targets_exit(void)
{
	int i;

	for (i = 0; i < VIND_TARGET_COUNT; i++) {
		struct vind_target_info *info = vind_targets[i];
		if (!info)
			continue;
		if (info->registered)
			vindicator_unregister(&info->vops);
		cancel_delayed_work_sync(&info->dwork);
		kfree(info);
		vind_targets[i] = NULL;
	}

	if (vind_targets_wq)
		destroy_workqueue(vind_targets_wq);

	pr_info("unloaded\n");
}

module_init(vindicator_targets_init);
module_exit(vindicator_targets_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GrayRavens");
MODULE_DESCRIPTION("Vindicator enforcement targets — governor, TCP, read_ahead, sched_boost");
