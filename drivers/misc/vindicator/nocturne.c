// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/vindicator/nocturne.c
 * Nocturne — screen-state power saving.
 *
 * Detects screen-on/off transitions via the PM suspend notifier
 * chain and applies configurable CPU frequency + cpuset throttling
 * when the display is off.
 *
 * Improvements over the original Tenebrion:
 *   - Uses PM suspend notifier (no sysfs polling).
 *   - Integrates with the Zenith profile system:
 *       GAMING profile  → disable throttling even with screen off.
 *       POWER_SAVE      → most aggressive throttling.
 *   - Per-profile tunables via sysfs.
 *   - freq_qos-based CPU frequency capping (not polluting cpufreq sysfs).
 *
 * Author: GrayRavens
 */
#define pr_fmt(fmt) "nocturne: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/pm_qos.h>
#include <linux/suspend.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/fs.h>
#include <linux/slab.h>

/* ------------------------------------------------------------------ */
/* Tunables                                                           */
/* ------------------------------------------------------------------ */
static bool noct_enabled = true;
module_param_named(enabled, noct_enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Master enable");

/* Frequency cap (KHz) applied when screen is off; 0 = no cap */
static unsigned int noct_freq_cap_khz;
module_param_named(freq_cap_khz, noct_freq_cap_khz, uint, 0644);
MODULE_PARM_DESC(freq_cap_khz, "Max CPU frequency (KHz) when screen off; 0 = no cap");

/* Cpuset restriction: restrict background + system-background to CPU0 */
static bool noct_restrict_cpusets = true;
module_param_named(restrict_cpusets, noct_restrict_cpusets, bool, 0644);
MODULE_PARM_DESC(restrict_cpusets, "Restrict background cpusets when screen off");

/* Suppress throttling when GAMING profile is active */
static bool noct_respect_gaming = true;
module_param_named(respect_gaming, noct_respect_gaming, bool, 0644);
MODULE_PARM_DESC(respect_gaming, "Skip throttling if GAMING profile is active");

/* ------------------------------------------------------------------ */
/* State                                                              */
/* ------------------------------------------------------------------ */
static bool screen_off;
static DEFINE_MUTEX(noct_lock);

/* freq_qos requests per policy */
static struct freq_qos_request *noct_qos_min;
static struct freq_qos_request *noct_qos_max;
static unsigned int noct_qos_nr_cpus;

/* ------------------------------------------------------------------ */
/* Forward declarations                                                */
/* ------------------------------------------------------------------ */
extern bool zenith_is_game_mode_active(void);

/* ------------------------------------------------------------------ */
/* VFS write helper (for cpuset)                                      */
/* ------------------------------------------------------------------ */
static int noct_write_file(const char *path, const char *buf)
{
	struct file *f;
	loff_t pos = 0;
	int ret;

	f = filp_open(path, O_WRONLY, 0);
	if (IS_ERR(f))
		return PTR_ERR(f);
	ret = kernel_write(f, buf, strlen(buf), &pos);
	filp_close(f, NULL);
	return ret < 0 ? ret : 0;
}

/* ------------------------------------------------------------------ */
/* freq_qos management                                                */
/* ------------------------------------------------------------------ */
static int noct_qos_init(void)
{
	unsigned int cpu;
	struct cpufreq_policy *policy;

	noct_qos_nr_cpus = nr_cpu_ids;
	noct_qos_min = kcalloc(noct_qos_nr_cpus, sizeof(*noct_qos_min), GFP_KERNEL);
	noct_qos_max = kcalloc(noct_qos_nr_cpus, sizeof(*noct_qos_max), GFP_KERNEL);
	if (!noct_qos_min || !noct_qos_max)
		goto fail;

	cpus_read_lock();
	for_each_online_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;
		if (policy->cpu == cpu) {
			freq_qos_add_request(&policy->constraints,
				&noct_qos_min[cpu], FREQ_QOS_MIN,
				policy->cpuinfo.min_freq);
			freq_qos_add_request(&policy->constraints,
				&noct_qos_max[cpu], FREQ_QOS_MAX,
				policy->cpuinfo.max_freq);
		}
		cpufreq_cpu_put(policy);
	}
	cpus_read_unlock();
	return 0;

fail:
	kfree(noct_qos_min);
	kfree(noct_qos_max);
	return -ENOMEM;
}

static void noct_qos_throttle(void)
{
	unsigned int cpu;
	struct cpufreq_policy *policy;

	if (!noct_freq_cap_khz)
		return;

	cpus_read_lock();
	for_each_online_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;
		if (policy->cpu == cpu) {
			/* Clamp max first, then raise min — avoids transient */
			freq_qos_update_request(&noct_qos_max[cpu],
				min(policy->cpuinfo.max_freq, noct_freq_cap_khz));
			freq_qos_update_request(&noct_qos_min[cpu],
				policy->cpuinfo.min_freq);
		}
		cpufreq_cpu_put(policy);
	}
	cpus_read_unlock();
}

static void noct_qos_restore(void)
{
	unsigned int cpu;
	struct cpufreq_policy *policy;

	if (!noct_freq_cap_khz)
		return;

	cpus_read_lock();
	for_each_online_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;
		if (policy->cpu == cpu) {
			freq_qos_update_request(&noct_qos_min[cpu],
				policy->cpuinfo.min_freq);
			freq_qos_update_request(&noct_qos_max[cpu],
				policy->cpuinfo.max_freq);
		}
		cpufreq_cpu_put(policy);
	}
	cpus_read_unlock();
}

/* ------------------------------------------------------------------ */
/* Screen state transition                                            */
/* ------------------------------------------------------------------ */
static void noct_screen_on(void)
{
	if (!screen_off)
		return;

	mutex_lock(&noct_lock);
	screen_off = false;
	noct_qos_restore();
	pr_info("screen ON — frequency limits restored\n");
	mutex_unlock(&noct_lock);
}

static void noct_screen_off(void)
{
	if (screen_off)
		return;

	mutex_lock(&noct_lock);
	screen_off = true;

	/* Respect gaming profile: don't throttle */
	if (noct_respect_gaming && zenith_is_game_mode_active()) {
		pr_info("screen OFF but GAMING profile — skipping throttle\n");
		mutex_unlock(&noct_lock);
		return;
	}

	noct_qos_throttle();

	/* Restrict background cpusets */
	if (noct_restrict_cpusets) {
		noct_write_file("/dev/cpuset/background/cpus", "0");
		noct_write_file("/dev/cpuset/system-background/cpus", "0");
		pr_info("background cpusets restricted to CPU0\n");
	}

	pr_info("screen OFF — throttling engaged%s\n",
		noct_freq_cap_khz ? "" : " (freq cap not configured)");
	mutex_unlock(&noct_lock);
}

/* ------------------------------------------------------------------ */
/* PM notifier                                                         */
/* ------------------------------------------------------------------ */
static int noct_pm_notifier(struct notifier_block *nb, unsigned long event, void *data)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
	case PM_HIBERNATION_PREPARE:
		noct_screen_off();
		break;
	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
		noct_screen_on();
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block noct_pm_nb = {
	.notifier_call = noct_pm_notifier,
};

/* ------------------------------------------------------------------ */
/* Module init / exit                                                  */
/* ------------------------------------------------------------------ */
static int __init nocturne_init(void)
{
	int ret;

	if (!noct_enabled) {
		pr_info("disabled by module param\n");
		return 0;
	}

	ret = noct_qos_init();
	if (ret)
		return ret;

	register_pm_notifier(&noct_pm_nb);
	pr_info("loaded (freq_cap=%u kHz, gaming_respect=%d, cpuset_restrict=%d)\n",
		noct_freq_cap_khz, noct_respect_gaming, noct_restrict_cpusets);
	return 0;
}

static void __exit nocturne_exit(void)
{
	unregister_pm_notifier(&noct_pm_nb);
	noct_screen_on(); /* restore any throttled resources */
	kfree(noct_qos_min);
	kfree(noct_qos_max);
	pr_info("unloaded\n");
}

module_init(nocturne_init);
module_exit(nocturne_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GrayRavens");
MODULE_DESCRIPTION("Nocturne — screen-state power saving");
