// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/vindicator/shun.c
 * Shun — Boot-time CPU frequency boost via PM QoS
 *
 * Locks all online CPU policies to maximum frequency for a configurable
 * duration after boot, then gracefully releases the lock. Uses PM QoS
 * requests to remain GKI 2.0 compatible — no direct sysfs writes.
 *
 * Author: GrayRavens
 * Co-authored-by: Kanagawa Yamada <albert.wesley.dion@gmail.com>
 */

#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>

#define SHUN_BOOST_DELAY_MS	10000
#define SHUN_REVERT_DELAY_MS	30000

static bool shun_enabled = true;
module_param(shun_enabled, bool, 0644);
MODULE_PARM_DESC(shun_enabled, "Enable Shun boot boost (default: true)");

static unsigned int shun_boost_ms = SHUN_BOOST_DELAY_MS;
module_param(shun_boost_ms, uint, 0644);
MODULE_PARM_DESC(shun_boost_ms,
		 "Delay before boost starts in ms (default: 10000)");

static unsigned int shun_revert_ms = SHUN_REVERT_DELAY_MS;
module_param(shun_revert_ms, uint, 0644);
MODULE_PARM_DESC(shun_revert_ms,
		 "Duration of boost in ms (default: 30000)");

static struct delayed_work shun_boost_work;
static struct delayed_work shun_revert_work;

/* Dynamic per-policy QoS tracking */
struct shun_qos_entry {
	unsigned int cpu;
	struct freq_qos_request min_req;
	struct freq_qos_request max_req;
	bool init;
};

static struct shun_qos_entry *shun_entries;
static unsigned int shun_nr_entries;

static struct shun_qos_entry *shun_find_entry(unsigned int cpu)
{
	unsigned int i;

	for (i = 0; i < shun_nr_entries; i++) {
		if (shun_entries[i].cpu == cpu && shun_entries[i].init)
			return &shun_entries[i];
	}
	return NULL;
}

static struct shun_qos_entry *shun_add_entry(unsigned int cpu,
					      struct cpufreq_policy *policy)
{
	struct shun_qos_entry *new_entries;
	unsigned int new_nr;

	new_nr = shun_nr_entries + 1;
	new_entries = krealloc(shun_entries,
			      new_nr * sizeof(*new_entries),
			      GFP_KERNEL);
	if (!new_entries)
		return NULL;

	shun_entries = new_entries;
	new_entries[shun_nr_entries].cpu = cpu;
	new_entries[shun_nr_entries].init = false;
	shun_nr_entries = new_nr;

	return &new_entries[shun_nr_entries - 1];
}

static void shun_do_boost(struct work_struct *work)
{
	unsigned int cpu;
	struct cpufreq_policy *policy;

	pr_info("shun: locking min=max via QoS\n");

	for_each_online_cpu(cpu) {
		struct shun_qos_entry *entry;

		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;

		if (policy->cpu == cpu) {
			entry = shun_find_entry(cpu);
			if (!entry)
				entry = shun_add_entry(cpu, policy);
			if (!entry) {
				cpufreq_cpu_put(policy);
				continue;
			}

			if (!entry->init) {
				freq_qos_add_request(&policy->constraints,
					&entry->max_req,
					FREQ_QOS_MAX,
					policy->cpuinfo.max_freq);
				freq_qos_add_request(&policy->constraints,
					&entry->min_req,
					FREQ_QOS_MIN,
					policy->cpuinfo.max_freq);
				entry->init = true;
			} else {
				freq_qos_update_request(&entry->max_req,
					policy->cpuinfo.max_freq);
				freq_qos_update_request(&entry->min_req,
					policy->cpuinfo.max_freq);
			}
			pr_debug("shun: policy%u locked to %u KHz\n",
				 cpu, policy->cpuinfo.max_freq);
		}
		cpufreq_cpu_put(policy);
	}

	schedule_delayed_work(&shun_revert_work,
			      msecs_to_jiffies(shun_revert_ms));
}

static void shun_do_revert(struct work_struct *work)
{
	unsigned int i;
	struct cpufreq_policy *policy;

	pr_info("shun: restoring — releasing QoS locks\n");

	for (i = 0; i < shun_nr_entries; i++) {
		struct shun_qos_entry *entry = &shun_entries[i];

		if (!entry->init)
			continue;

		policy = cpufreq_cpu_get(entry->cpu);
		if (!policy)
			continue;

		freq_qos_update_request(&entry->min_req,
			policy->cpuinfo.min_freq);
		freq_qos_update_request(&entry->max_req,
			policy->cpuinfo.max_freq);
		pr_debug("shun: policy%u restored min=%u max=%u KHz\n",
			 entry->cpu, policy->cpuinfo.min_freq,
			 policy->cpuinfo.max_freq);
		cpufreq_cpu_put(policy);
	}
}

static void shun_qos_cleanup(void)
{
	unsigned int i;

	for (i = 0; i < shun_nr_entries; i++) {
		struct shun_qos_entry *entry = &shun_entries[i];

		if (!entry->init)
			continue;
		freq_qos_remove_request(&entry->min_req);
		freq_qos_remove_request(&entry->max_req);
		entry->init = false;
	}

	kfree(shun_entries);
	shun_entries = NULL;
	shun_nr_entries = 0;
}

static int __init shun_init(void)
{
	if (!shun_enabled) {
		pr_info("shun: disabled\n");
		return 0;
	}

	shun_entries = NULL;
	shun_nr_entries = 0;

	INIT_DELAYED_WORK(&shun_boost_work, shun_do_boost);
	INIT_DELAYED_WORK(&shun_revert_work, shun_do_revert);

	schedule_delayed_work(&shun_boost_work,
			      msecs_to_jiffies(shun_boost_ms));

	pr_info("shun: active (boost+%ums, hold=%ums)\n",
		shun_boost_ms, shun_revert_ms);
	return 0;
}
late_initcall(shun_init);

static void __exit shun_exit(void)
{
	cancel_delayed_work_sync(&shun_boost_work);
	cancel_delayed_work_sync(&shun_revert_work);

	shun_qos_cleanup();

	pr_info("shun: unloaded\n");
}
module_exit(shun_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GrayRavens");
MODULE_DESCRIPTION("Shun — Boot-time CPU frequency boost via PM QoS");
