// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/vindicator/equilibrium.c
 * Equilibrium — profile-aware memory / swap tuning.
 *
 * Adjusts vm_swappiness, dirty_ratio, dirty_background_ratio, and
 * vfs_cache_pressure dynamically based on the active Zenith profile.
 *
 * Wired into the Zenith profile chain (zenith_apply_profile) alongside
 * Hikari, Kasumi, and IYASHI — fully automatic, zero user config needed.
 *
 * Improvements over the original Sparxie:
 *   - Profile-aware: different tunings per Zenith profile.
 *   - Controls 4 knobs, not just swappiness.
 *   - Uses vm_swappiness directly + VFS writes — no polling.
 *   - Module params override any per-profile value at runtime.
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
#include <linux/zenith_profiles.h>

/* ------------------------------------------------------------------ */
/* Per-profile tunings                                                */
/* ------------------------------------------------------------------ */
struct equil_profile_tune {
	unsigned int swappiness;
	unsigned int dirty_bg_ratio;
	unsigned int dirty_ratio;
	unsigned int vfs_cache_pressure;
};

static const struct equil_profile_tune equil_tunes[] = {
	[ZENITH_PROFILE_BALANCED] = {
		.swappiness         = 60,
		.dirty_bg_ratio     = 5,
		.dirty_ratio        = 20,
		.vfs_cache_pressure = 100,
	},
	[ZENITH_PROFILE_PERFORMANCE] = {
		.swappiness         = 20,
		.dirty_bg_ratio     = 10,
		.dirty_ratio        = 30,
		.vfs_cache_pressure = 50,
	},
	[ZENITH_PROFILE_BATTERY] = {
		.swappiness         = 10,
		.dirty_bg_ratio     = 3,
		.dirty_ratio        = 10,
		.vfs_cache_pressure = 200,
	},
	[ZENITH_PROFILE_GAMING] = {
		.swappiness         = 20,
		.dirty_bg_ratio     = 10,
		.dirty_ratio        = 30,
		.vfs_cache_pressure = 50,
	},
	[ZENITH_PROFILE_AUDIO] = {
		.swappiness         = 20,
		.dirty_bg_ratio     = 10,
		.dirty_ratio        = 30,
		.vfs_cache_pressure = 50,
	},
};

/* ------------------------------------------------------------------ */
/* Module parameters — override per-profile values at runtime          */
/* ------------------------------------------------------------------ */
static unsigned int equil_swappiness_performance = 20;
static unsigned int equil_swappiness_battery    = 10;
static unsigned int equil_swappiness_balanced   = 60;
static unsigned int equil_swappiness_gaming     = 20;
static unsigned int equil_swappiness_audio      = 20;

static unsigned int equil_dirty_bg_performance   = 10;
static unsigned int equil_dirty_bg_battery       = 3;
static unsigned int equil_dirty_bg_balanced      = 5;
static unsigned int equil_dirty_bg_gaming        = 10;
static unsigned int equil_dirty_bg_audio         = 10;

static unsigned int equil_dirty_ratio_performance  = 30;
static unsigned int equil_dirty_ratio_battery     = 10;
static unsigned int equil_dirty_ratio_balanced    = 20;
static unsigned int equil_dirty_ratio_gaming      = 30;
static unsigned int equil_dirty_ratio_audio       = 30;

static unsigned int equil_vfs_cache_performance   = 50;
static unsigned int equil_vfs_cache_battery       = 200;
static unsigned int equil_vfs_cache_balanced      = 100;
static unsigned int equil_vfs_cache_gaming        = 50;
static unsigned int equil_vfs_cache_audio         = 50;

module_param_named(swappiness_performance, equil_swappiness_performance, uint, 0644);
module_param_named(swappiness_battery,     equil_swappiness_battery,     uint, 0644);
module_param_named(swappiness_balanced,    equil_swappiness_balanced,    uint, 0644);
module_param_named(swappiness_gaming,      equil_swappiness_gaming,      uint, 0644);
module_param_named(swappiness_audio,       equil_swappiness_audio,       uint, 0644);

module_param_named(dirty_bg_performance, equil_dirty_bg_performance, uint, 0644);
module_param_named(dirty_bg_battery,     equil_dirty_bg_battery,     uint, 0644);
module_param_named(dirty_bg_balanced,    equil_dirty_bg_balanced,    uint, 0644);
module_param_named(dirty_bg_gaming,      equil_dirty_bg_gaming,      uint, 0644);
module_param_named(dirty_bg_audio,       equil_dirty_bg_audio,       uint, 0644);

module_param_named(dirty_ratio_performance, equil_dirty_ratio_performance, uint, 0644);
module_param_named(dirty_ratio_battery,     equil_dirty_ratio_battery,     uint, 0644);
module_param_named(dirty_ratio_balanced,    equil_dirty_ratio_balanced,    uint, 0644);
module_param_named(dirty_ratio_gaming,      equil_dirty_ratio_gaming,      uint, 0644);
module_param_named(dirty_ratio_audio,       equil_dirty_ratio_audio,       uint, 0644);

module_param_named(vfs_cache_performance, equil_vfs_cache_performance, uint, 0644);
module_param_named(vfs_cache_battery,     equil_vfs_cache_battery,     uint, 0644);
module_param_named(vfs_cache_balanced,    equil_vfs_cache_balanced,    uint, 0644);
module_param_named(vfs_cache_gaming,      equil_vfs_cache_gaming,      uint, 0644);
module_param_named(vfs_cache_audio,       equil_vfs_cache_audio,       uint, 0644);

/* ------------------------------------------------------------------ */
/* State                                                              */
/* ------------------------------------------------------------------ */
static int equil_current_profile = ZENITH_PROFILE_BALANCED;
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
/* Per-profile helper — get override param or fall back to table       */
/* ------------------------------------------------------------------ */
static unsigned int equil_override_swappiness(unsigned int profile)
{
	switch (zenith_resolve_profile(profile)) {
	case ZENITH_PROFILE_BATTERY:
		return equil_swappiness_battery;
	case ZENITH_PROFILE_GAMING:
		return equil_swappiness_gaming;
	case ZENITH_PROFILE_PERFORMANCE:
		return equil_swappiness_performance;
	case ZENITH_PROFILE_AUDIO:
		return equil_swappiness_audio;
	default:
		return equil_swappiness_balanced;
	}
}

static unsigned int equil_override_dirty_bg(unsigned int profile)
{
	switch (zenith_resolve_profile(profile)) {
	case ZENITH_PROFILE_BATTERY:
		return equil_dirty_bg_battery;
	case ZENITH_PROFILE_GAMING:
		return equil_dirty_bg_gaming;
	case ZENITH_PROFILE_PERFORMANCE:
		return equil_dirty_bg_performance;
	case ZENITH_PROFILE_AUDIO:
		return equil_dirty_bg_audio;
	default:
		return equil_dirty_bg_balanced;
	}
}

static unsigned int equil_override_dirty_ratio(unsigned int profile)
{
	switch (zenith_resolve_profile(profile)) {
	case ZENITH_PROFILE_BATTERY:
		return equil_dirty_ratio_battery;
	case ZENITH_PROFILE_GAMING:
		return equil_dirty_ratio_gaming;
	case ZENITH_PROFILE_PERFORMANCE:
		return equil_dirty_ratio_performance;
	case ZENITH_PROFILE_AUDIO:
		return equil_dirty_ratio_audio;
	default:
		return equil_dirty_ratio_balanced;
	}
}

static unsigned int equil_override_vfs_cache(unsigned int profile)
{
	switch (zenith_resolve_profile(profile)) {
	case ZENITH_PROFILE_BATTERY:
		return equil_vfs_cache_battery;
	case ZENITH_PROFILE_GAMING:
		return equil_vfs_cache_gaming;
	case ZENITH_PROFILE_PERFORMANCE:
		return equil_vfs_cache_performance;
	case ZENITH_PROFILE_AUDIO:
		return equil_vfs_cache_audio;
	default:
		return equil_vfs_cache_balanced;
	}
}

/* ------------------------------------------------------------------ */
/* Profile notifier (called by zenith_apply_profile)                   */
/* ------------------------------------------------------------------ */
void equilibrium_apply_profile(unsigned int profile)
{
	const struct equil_profile_tune *t = NULL;
	unsigned int swappiness, dirty_bg, dirty_ratio, vfs_cache;

	profile = zenith_resolve_profile(profile);

	/* Look up table entry for this profile */
	if (profile < ARRAY_SIZE(equil_tunes))
		t = &equil_tunes[profile];

	/* Fall back to BALANCED if no table entry */
	if (!t)
		t = &equil_tunes[ZENITH_PROFILE_BALANCED];

	/* Module params override table defaults */
	swappiness  = equil_override_swappiness(profile);
	dirty_bg    = equil_override_dirty_bg(profile);
	dirty_ratio = equil_override_dirty_ratio(profile);
	vfs_cache   = equil_override_vfs_cache(profile);

	/* Direct write — instant, no sysfs */
	vm_swappiness = swappiness;

	/* VFS writes for procfs knobs */
	equilibrium_write_ratio("/proc/sys/vm/dirty_background_ratio", dirty_bg);
	equilibrium_write_ratio("/proc/sys/vm/dirty_ratio", dirty_ratio);
	equilibrium_write_ratio("/proc/sys/vm/vfs_cache_pressure", vfs_cache);

	equil_current_profile = profile;

	pr_info("profile=%u: swappiness=%u dirty_bg=%u dirty=%u vfs_cache=%u\n",
		profile, swappiness, dirty_bg, dirty_ratio, vfs_cache);
}
EXPORT_SYMBOL_GPL(equilibrium_apply_profile);

/* ------------------------------------------------------------------ */
/* Module init / exit                                                  */
/* ------------------------------------------------------------------ */
static int __init equilibrium_init(void)
{
	equil_dbg_dir = debugfs_create_dir("equilibrium", NULL);

	debugfs_create_u32("swappiness",        0444, equil_dbg_dir, &vm_swappiness);
	debugfs_create_u32("current_profile",   0444, equil_dbg_dir,
			   &equil_current_profile);

	debugfs_create_u32("swappiness_performance", 0644, equil_dbg_dir,
			   &equil_swappiness_performance);
	debugfs_create_u32("swappiness_battery",   0644, equil_dbg_dir,
			   &equil_swappiness_battery);
	debugfs_create_u32("swappiness_balanced",  0644, equil_dbg_dir,
			   &equil_swappiness_balanced);
	debugfs_create_u32("swappiness_gaming",    0644, equil_dbg_dir,
			   &equil_swappiness_gaming);
	debugfs_create_u32("swappiness_audio",     0644, equil_dbg_dir,
			   &equil_swappiness_audio);

	debugfs_create_u32("dirty_bg_performance", 0644, equil_dbg_dir,
			   &equil_dirty_bg_performance);
	debugfs_create_u32("dirty_bg_battery",   0644, equil_dbg_dir,
			   &equil_dirty_bg_battery);
	debugfs_create_u32("dirty_bg_balanced",  0644, equil_dbg_dir,
			   &equil_dirty_bg_balanced);
	debugfs_create_u32("dirty_bg_gaming",    0644, equil_dbg_dir,
			   &equil_dirty_bg_gaming);
	debugfs_create_u32("dirty_bg_audio",     0644, equil_dbg_dir,
			   &equil_dirty_bg_audio);

	debugfs_create_u32("dirty_ratio_performance", 0644, equil_dbg_dir,
			   &equil_dirty_ratio_performance);
	debugfs_create_u32("dirty_ratio_battery",   0644, equil_dbg_dir,
			   &equil_dirty_ratio_battery);
	debugfs_create_u32("dirty_ratio_balanced",  0644, equil_dbg_dir,
			   &equil_dirty_ratio_balanced);
	debugfs_create_u32("dirty_ratio_gaming",    0644, equil_dbg_dir,
			   &equil_dirty_ratio_gaming);
	debugfs_create_u32("dirty_ratio_audio",     0644, equil_dbg_dir,
			   &equil_dirty_ratio_audio);

	debugfs_create_u32("vfs_cache_performance", 0644, equil_dbg_dir,
			   &equil_vfs_cache_performance);
	debugfs_create_u32("vfs_cache_battery",   0644, equil_dbg_dir,
			   &equil_vfs_cache_battery);
	debugfs_create_u32("vfs_cache_balanced",  0644, equil_dbg_dir,
			   &equil_vfs_cache_balanced);
	debugfs_create_u32("vfs_cache_gaming",    0644, equil_dbg_dir,
			   &equil_vfs_cache_gaming);
	debugfs_create_u32("vfs_cache_audio",     0644, equil_dbg_dir,
			   &equil_vfs_cache_audio);

	/* Apply initial tuning (BALANCED) */
	equilibrium_apply_profile(ZENITH_PROFILE_BALANCED);

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
