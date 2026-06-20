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

static struct freq_qos_request shun_min_req[NR_CPUS];
static struct freq_qos_request shun_max_req[NR_CPUS];
static bool shun_qos_init[NR_CPUS];

static void shun_do_boost(struct work_struct *work)
{
	unsigned int cpu;
	struct cpufreq_policy *policy;

	pr_info("shun: locking min=max via QoS\n");

	for_each_online_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;

		if (policy->cpu == cpu) {
			if (!shun_qos_init[cpu]) {
				freq_qos_add_request(&policy->constraints,
					&shun_max_req[cpu],
					FREQ_QOS_MAX,
					policy->cpuinfo.max_freq);
				freq_qos_add_request(&policy->constraints,
					&shun_min_req[cpu],
					FREQ_QOS_MIN,
					policy->cpuinfo.max_freq);
				shun_qos_init[cpu] = true;
			} else {
				freq_qos_update_request(&shun_max_req[cpu],
					policy->cpuinfo.max_freq);
				freq_qos_update_request(&shun_min_req[cpu],
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
	unsigned int cpu;
	struct cpufreq_policy *policy;

	pr_info("shun: restoring — releasing QoS locks\n");

	for_each_online_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;

		if (policy->cpu == cpu && shun_qos_init[cpu]) {
			freq_qos_update_request(&shun_min_req[cpu],
				policy->cpuinfo.min_freq);
			freq_qos_update_request(&shun_max_req[cpu],
				policy->cpuinfo.max_freq);
			pr_debug("shun: policy%u restored min=%u max=%u KHz\n",
				 cpu, policy->cpuinfo.min_freq,
				 policy->cpuinfo.max_freq);
		}
		cpufreq_cpu_put(policy);
	}
}

static void shun_qos_cleanup(void)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu) {
		if (shun_qos_init[cpu]) {
			freq_qos_remove_request(&shun_min_req[cpu]);
			freq_qos_remove_request(&shun_max_req[cpu]);
			shun_qos_init[cpu] = false;
		}
	}
}

static int __init shun_init(void)
{
	if (!shun_enabled) {
		pr_info("shun: disabled\n");
		return 0;
	}

	memset(shun_qos_init, 0, sizeof(shun_qos_init));

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
