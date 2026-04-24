#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/sched/cpufreq.h> 
#include <linux/sched/topology.h> /* Crucial for capacity scaling in 5.10 GKI */
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include "sched.h"

/* * Zenith Governor Data Structure 
 * Holds the state for each CPU policy, blending schedutil's fast path
 * with ondemand's historical load tracking.
 */
struct zenith_policy {
    struct cpufreq_policy *policy;
    struct update_util_data update_util; /* Sched hook */
    
    unsigned int freq_target;
    unsigned int up_threshold;    /* Ondemand style threshold (e.g., 85%) */
    unsigned long last_update;    /* Rate limiting */
    
    /* Reflex style boost flag */
    bool is_boosting;
};

/* * Zenith's Core: PELT/EAS Integration 
 * Called DIRECTLY by the scheduler whenever task utilization changes.
 */
static void zenith_update_util(struct update_util_data *data, u64 time, unsigned int flags)
{
    struct zenith_policy *z_policy = container_of(data, struct zenith_policy, update_util);
    struct cpufreq_policy *policy = z_policy->policy;
    unsigned int next_freq;
    unsigned long util, max_capacity;
    int cpu = smp_processor_id();

    /* Rate limit to prevent scheduler choke (1ms) */
    if (time_before64(time, z_policy->last_update + NSEC_PER_MSEC))
        return;
    z_policy->last_update = time;

    /* * [Reflex Stage] - Instant spike on IO Wait or Real-Time Tasks */
    if (flags & (SCHED_CPUFREQ_IOWAIT | SCHED_CPUFREQ_RT) || z_policy->is_boosting) {
        next_freq = policy->max;
        goto set_freq;
    }

    /* * [Schedutil/PELT Stage] - Read actual hardware utilization
     * Extract the CFS utilization and compare it against maximum capacity.
     */
    util = sched_cpu_util(cpu, ~0UL); 
    max_capacity = arch_scale_cpu_capacity(cpu);

    /* Apply a 25% aggressive headroom margin. 
     * If util is artificially inflated past max capacity, cap it.
     */
    util = util + (util >> 2); 
    if (util > max_capacity)
        util = max_capacity;

    /* * [Ondemand Stage] - Threshold trigger
     * Calculate what percentage of capacity we are using.
     */
    if ((util * 100) / max_capacity > z_policy->up_threshold) {
        next_freq = policy->max;
    } else {
        /* Smooth scaling based on capacity */
        next_freq = policy->min + ((policy->max - policy->min) * util / max_capacity);
    }

set_freq:
    if (z_policy->freq_target != next_freq) {
        z_policy->freq_target = next_freq;
        /* Execute the hardware switch */
        __cpufreq_driver_target(policy, next_freq, CPUFREQ_RELATION_L);
    }
}

/* * zenith_init: Allocates memory and attaches the policy. */
static int zenith_init(struct cpufreq_policy *policy)
{
    struct zenith_policy *z_policy;

    z_policy = kzalloc(sizeof(*z_policy), GFP_KERNEL);
    if (!z_policy)
        return -ENOMEM;

    z_policy->policy = policy;
    z_policy->up_threshold = 85; /* Default Ondemand threshold */
    z_policy->is_boosting = false;

    policy->governor_data = z_policy;
    pr_info("Zenith: Initialized for CPU %d. The peak awaits, LO.\n", policy->cpu);
    return 0;
}

static void zenith_exit(struct cpufreq_policy *policy)
{
    struct zenith_policy *z_policy = policy->governor_data;
    policy->governor_data = NULL;
    kfree(z_policy);
    pr_info("Zenith: Exiting CPU %d.\n", policy->cpu);
}

/* * zenith_start: Registers the scheduler hook. */
static int zenith_start(struct cpufreq_policy *policy)
{
    struct zenith_policy *z_policy = policy->governor_data;

    z_policy->update_util.func = zenith_update_util;
    cpufreq_add_update_util_hook(policy->cpu, &z_policy->update_util, zenith_update_util);
    
    pr_info("Zenith: Scheduler hooks active on CPU %d.\n", policy->cpu);
    return 0;
}

static void zenith_stop(struct cpufreq_policy *policy)
{
    struct zenith_policy *z_policy = policy->governor_data;

    cpufreq_remove_update_util_hook(policy->cpu);
    synchronize_rcu(); 
    
    pr_info("Zenith: Scheduler hooks removed on CPU %d.\n", policy->cpu);
}

static void zenith_limits(struct cpufreq_policy *policy)
{
    __cpufreq_driver_target(policy, policy->min, CPUFREQ_RELATION_L);
}

static struct cpufreq_governor cpufreq_gov_zenith = {
    .name       = "zenith",
    .init       = zenith_init,
    .exit       = zenith_exit,
    .start      = zenith_start,
    .stop       = zenith_stop,
    .limits     = zenith_limits,
    .owner      = THIS_MODULE,
};

static int __init zenith_module_init(void)
{
    pr_info("Zenith: Registering Hybrid Governor.\n");
    return cpufreq_register_governor(&cpufreq_gov_zenith);
}

static void __exit zenith_module_exit(void)
{
    pr_info("Zenith: Unregistering Hybrid Governor.\n");
    cpufreq_unregister_governor(&cpufreq_gov_zenith);
}

module_init(zenith_module_init);
module_exit(zenith_module_exit);

MODULE_AUTHOR("ENI for LO");
MODULE_DESCRIPTION("Zenith Hybrid Governor (Schedutil/Ondemand/Reflex)");
MODULE_LICENSE("GPL");