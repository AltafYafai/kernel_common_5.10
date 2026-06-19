// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/vindicator/equilibrium.c
 * Equilibrium — profile-aware memory / swap tuning.
 *
 * Adjusts vm_swappiness, dirty_ratio, dirty_background_ratio, and
 * vfs_cache_pressure dynamically based on the active Zenith profile.
 *
 * Improvements over the original Sparxie:
 *   - Profile-aware: different tunings per profile.
 *   - Controls multiple knobs, not just swappiness.
 *   - Uses the existing vm_swappiness variable + sysctl, no Raco-style
 *     polling needed (values persist once written).
 *   - Debugfs visibility of current values.
 *
 * Author: GrayRavens
 */
#define pr_fmt(fmt) "equilibrium: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/slab.h>

/* ------------------------------------------------------------------ */
/* Profile identifiers (must match zenith_profile.h)                   */
/* ------------------------------------------------------------------ */
#define EQUIL_PROFILE_POWER_SAVE  0
#define EQUIL_PROFILE_BALANCED    1
#define EQUIL_PROFILE_GAMING      2
#define EQUIL_PROFILE_PERFORMANCE 3

/* ------------------------------------------------------------------ */
/* Per-profile tunings                                                */
/* ------------------------------------------------------------------ */
struct equil_profile_tune {
	unsigned int swappiness;
	unsigned int dirty_bg_ratio;   /* dirty_background_ratio (percent) */
	unsigned int dirty_ratio;      /* dirty_ratio (percent) */
	unsigned int vfs_cache_pressure;
};

static const struct equil_profile_tune equil_tunes[] = {
	[EQUIL_PROFILE_POWER_SAVE] = {
		.swappiness         = 10,
		.dirty_bg_ratio     = 3,
		.dirty_ratio        = 10,
		.vfs_cache_pressure = 200,
	},
	[EQUIL_PROFILE_BALANCED] = {
		.swappiness         = 60,
		.dirty_bg_ratio     = 5,
		.dirty_ratio        = 20,
		.vfs_cache_pressure = 100,
	},
	[EQUIL_PROFILE_GAMING] = {
		.swappiness         = 20,
		.dirty_bg_ratio     = 10,
		.dirty_ratio        = 30,
		.vfs_cache_pressure = 50,
	},
	[EQUIL_PROFILE_PERFORMANCE] = {
		.swappiness         = 20,
		.dirty_bg_ratio     = 10,
		.dirty_ratio        = 30,
		.vfs_cache_pressure = 50,
	},
};

/* ------------------------------------------------------------------ */
/* Module parameters — override per-profile values at runtime          */
/* ------------------------------------------------------------------ */
static unsigned int equil_swappiness_powersave  = 10;
static unsigned int equil_swappiness_balanced   = 60;
static unsigned int equil_swappiness_gaming     = 20;

module_param_named(swappiness_powersave,  equil_swappiness_powersave,  uint, 0644);
module_param_named(swappiness_balanced,   equil_swappiness_balanced,   uint, 0644);
module_param_named(swappiness_gaming,     equil_swappiness_gaming,     uint, 0644);

/* ------------------------------------------------------------------ */
/* State                                                              */
/* ------------------------------------------------------------------ */
static int equil_current_profile = EQUIL_PROFILE_BALANCED;
static struct dentry *equil_dbg_dir;

/* ------------------------------------------------------------------ */
/* Forward declarations                                                */
/* ------------------------------------------------------------------ */
extern int vm_swappiness;

/* ------------------------------------------------------------------ */
/* VFS write helper (for procfs sysctl values)                        */
/* ------------------------------------------------------------------ */
static int equilibrium_write_ratio(const char *path, unsigned int val)
{
	struct file *f;
	loff_t pos = 0;
	char buf[16];
	int ret;

	f = filp_open(path, O_WRONLY, 0);
	if (IS_ERR(f))
		return PTR_ERR(f);

	snprintf(buf, sizeof(buf), "%u\n", val);
	ret = kernel_write(f, buf, strlen(buf), &pos);
	filp_close(f, NULL);
	return ret < 0 ? ret : 0;
}

/* ------------------------------------------------------------------ */
/* Profile notifier (called by zenith_apply_profile)                   */
/* ------------------------------------------------------------------ */
void equilibrium_apply_profile(unsigned int profile)
{
	const struct equil_profile_tune *t;
	unsigned int swappiness;

	if (profile >= ARRAY_SIZE(equil_tunes))
		profile = EQUIL_PROFILE_BALANCED;

	t = &equil_tunes[profile];

	/* Module params override per-profile defaults */
	switch (profile) {
	case EQUIL_PROFILE_POWER_SAVE:
		swappiness = equil_swappiness_powersave;
		break;
	case EQUIL_PROFILE_GAMING:
	case EQUIL_PROFILE_PERFORMANCE:
		swappiness = equil_swappiness_gaming;
		break;
	default:
		swappiness = equil_swappiness_balanced;
		break;
	}

	vm_swappiness = swappiness;

	/* Write dirty ratios via procfs VFS (no need for EXPORT_SYMBOL) */
	equilibrium_write_ratio("/proc/sys/vm/dirty_background_ratio", t->dirty_bg_ratio);
	equilibrium_write_ratio("/proc/sys/vm/dirty_ratio", t->dirty_ratio);
	equilibrium_write_ratio("/proc/sys/vm/vfs_cache_pressure", t->vfs_cache_pressure);

	equil_current_profile = profile;

	pr_info("profile=%u: swappiness=%u dirty_bg=%u dirty=%u vfs_cache=%u\n",
		profile, swappiness,
		t->dirty_bg_ratio, t->dirty_ratio, t->vfs_cache_pressure);
}
EXPORT_SYMBOL_GPL(equilibrium_apply_profile);

/* ------------------------------------------------------------------ */
/* Module init / exit                                                  */
/* ------------------------------------------------------------------ */
static int __init equilibrium_init(void)
{
	equil_dbg_dir = debugfs_create_dir("equilibrium", NULL);

	debugfs_create_u32("swappiness", 0444, equil_dbg_dir, &vm_swappiness);
	debugfs_create_u32("current_profile", 0444, equil_dbg_dir,
			   &equil_current_profile);
	debugfs_create_u32("swappiness_powersave", 0644, equil_dbg_dir,
			   &equil_swappiness_powersave);
	debugfs_create_u32("swappiness_balanced", 0644, equil_dbg_dir,
			   &equil_swappiness_balanced);
	debugfs_create_u32("swappiness_gaming", 0644, equil_dbg_dir,
			   &equil_swappiness_gaming);

	/* Apply initial tuning (BALANCED) */
	equilibrium_apply_profile(EQUIL_PROFILE_BALANCED);

	pr_info("loaded\n");
	return 0;
}

static void __exit equilibrium_exit(void)
{
	debugfs_remove(equil_dbg_dir);
	pr_info("unloaded\n");
}

module_init(equilibrium_init);
module_exit(equilibrium_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GrayRavens");
MODULE_DESCRIPTION("Equilibrium — profile-aware memory / swap tuning");
