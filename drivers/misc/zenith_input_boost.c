// SPDX-License-Identifier: GPL-2.0-only
/*
 * zenith_input_boost - Touch-triggered CPU frequency boost
 *
 * Boosts all online CPUs to a configurable frequency for a short
 * duration when a touchscreen event is detected.  Catches the
 * initial burst of work from touch processing so cpufreq is already
 * ramped up by the time the app/task actually runs.
 *
 * Tunables (/sys/module/zenith_input_boost/parameters/):
 *   enabled         — master switch (1/0, default 1)
 *   boost_ms        — boost duration in ms (1-500, default 50)
 *   min_freq_khz    — minimum frequency to boost to (0 = max, default 0)
 */
#include <linux/module.h>
#include <linux/input.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/cpumask.h>
#include <linux/slab.h>

static bool input_boost_enabled __read_mostly = true;
static unsigned int input_boost_ms __read_mostly = 50;
static unsigned int input_boost_min_freq_khz __read_mostly;

module_param_named(enabled, input_boost_enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable input boost (default: true)");
module_param_named(boost_ms, input_boost_ms, uint, 0644);
MODULE_PARM_DESC(boost_ms, "Boost duration in ms (1-500, default: 50)");
module_param_named(boost_min_freq, input_boost_min_freq_khz, uint, 0644);
MODULE_PARM_DESC(boost_min_freq, "Boost min freq in kHz (0 = max freq, default: 0)");

/* Per-policy boost state */
struct ib_policy {
	struct cpufreq_policy *policy;
	unsigned int saved_min;
	struct delayed_work work;
};

static struct ib_policy *ib_policies;
static int ib_nr_policies;
static spinlock_t ib_lock;

static void ib_boost_end(struct work_struct *work)
{
	struct ib_policy *ib = container_of(work, struct ib_policy, work.work);

	if (!READ_ONCE(input_boost_enabled))
		return;

	spin_lock(&ib_lock);
	if (ib->policy) {
		ib->policy->min = ib->saved_min;
		cpufreq_update_policy(ib->policy->cpu);
	}
	spin_unlock(&ib_lock);
}

static void ib_boost_trigger(void)
{
	int i;

	if (!READ_ONCE(input_boost_enabled))
		return;

	for (i = 0; i < ib_nr_policies; i++) {
		struct ib_policy *ib = &ib_policies[i];
		unsigned int boost_freq;

		if (!ib->policy)
			continue;

		spin_lock(&ib_lock);

		/*
		 * Cancel any pending boost-end work so we extend the
		 * boost window rather than stacking timers.
		 *
		 * Use cancel_delayed_work() (non-sleeping) because this
		 * is called from the input_handler->event callback which
		 * runs in interrupt/softirq context.
		 */
		cancel_delayed_work(&ib->work);

		if (ib->policy->min != ib->saved_min)
			ib->saved_min = ib->policy->min;

		boost_freq = READ_ONCE(input_boost_min_freq_khz);
		if (!boost_freq)
			boost_freq = ib->policy->cpuinfo.max_freq;

		if (boost_freq > ib->policy->min) {
			ib->policy->min = min(boost_freq,
					     ib->policy->cpuinfo.max_freq);
			cpufreq_update_policy(ib->policy->cpu);
		}

		schedule_delayed_work(&ib->work,
				      msecs_to_jiffies(READ_ONCE(input_boost_ms)));

		spin_unlock(&ib_lock);
	}
}

/*
 * Input handler to catch touch events.  Replaces the unavailable
 * input_register_notifier() API with the standard input_handler
 * pattern where the ->event callback fires on every input event.
 *
 * Uses cancel_delayed_work() (non-sleeping) instead of
 * cancel_delayed_work_sync() because the ->event callback runs
 * in interrupt/softirq context where sleeping is not permitted.
 */
static int ib_input_connect(struct input_handler *handler,
			    struct input_dev *dev,
			    const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "zenith_input_boost";

	ret = input_register_handle(handle);
	if (ret) {
		kfree(handle);
		return ret;
	}

	ret = input_open_device(handle);
	if (ret) {
		input_unregister_handle(handle);
		kfree(handle);
		return ret;
	}

	return 0;
}

static void ib_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static void ib_input_event(struct input_handle *handle,
			   unsigned int type, unsigned int code, int value)
{
	/*
	 * Only trigger on touch events.  For EV_KEY it's a tap (BTN_TOUCH=1).
	 * For EV_ABS, fire once per touch start on ABS_MT_TRACKING_ID
	 * to avoid triggering on every coordinate update (X,Y,pressure)
	 * during a single gesture.
	 */
	if (type == EV_KEY && value == 1)
		ib_boost_trigger();
	else if (type == EV_ABS && code == ABS_MT_TRACKING_ID && value >= 0)
		ib_boost_trigger();
}

static const struct input_device_id ib_input_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { BIT_MASK(ABS_MT_POSITION_X) },
	},
	{ },
};

static struct input_handler ib_input_handler = {
	.event		= ib_input_event,
	.connect	= ib_input_connect,
	.disconnect	= ib_input_disconnect,
	.name		= "zenith_input_boost",
	.id_table	= ib_input_ids,
};

static void ib_cpufreq_policy_notifier(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;
	int i;

	if (event != CPUFREQ_CREATE_POLICY && event != CPUFREQ_REMOVE_POLICY)
		return;

	for (i = 0; i < ib_nr_policies; i++) {
		if (ib_policies[i].policy == policy) {
			if (event == CPUFREQ_REMOVE_POLICY) {
				cancel_delayed_work_sync(&ib_policies[i].work);
				ib_policies[i].policy = NULL;
			}
			return;
		}
	}

	/* New policy: find an empty slot */
	if (event == CPUFREQ_CREATE_POLICY) {
		for (i = 0; i < ib_nr_policies; i++) {
			if (!ib_policies[i].policy) {
				ib_policies[i].policy = policy;
				ib_policies[i].saved_min = policy->min;
				return;
			}
		}
	}
}

static struct notifier_block ib_cpufreq_nb = {
	.notifier_call = (int (*)(struct notifier_block *, unsigned long, void *))ib_cpufreq_policy_notifier,
};

static int __init ib_init(void)
{
	struct cpufreq_policy *policy;
	int cpu, ret;

	/* Count policies */
	for_each_possible_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (policy) {
			ib_nr_policies++;
			cpufreq_cpu_put(policy);
			/* Skip to next policy's CPUs */
			cpu = cpumask_last(policy->related_cpus);
		}
	}

	if (!ib_nr_policies) {
		pr_err("zenith_input_boost: no cpufreq policies found\n");
		return -ENODEV;
	}

	ib_policies = kcalloc(ib_nr_policies, sizeof(*ib_policies), GFP_KERNEL);
	if (!ib_policies)
		return -ENOMEM;

	spin_lock_init(&ib_lock);

	/* Populate policies */
	{
		int i = 0;
		struct cpufreq_policy *pol;

		for_each_possible_cpu(cpu) {
			pol = cpufreq_cpu_get(cpu);
			if (pol) {
				ib_policies[i].policy = pol;
				ib_policies[i].saved_min = pol->min;
				INIT_DELAYED_WORK(&ib_policies[i].work, ib_boost_end);
				i++;
				cpufreq_cpu_put(pol);
				cpu = cpumask_last(pol->related_cpus);
			}
		}
	}

	ret = input_register_handler(&ib_input_handler);
	if (ret) {
		pr_err("zenith_input_boost: input_register_handler failed (%d)\n", ret);
		goto free_policies;
	}

	ret = cpufreq_register_notifier(&ib_cpufreq_nb, CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		pr_warn("zenith_input_boost: cpufreq notifier failed (%d)\n", ret);

	pr_info("zenith_input_boost: registered (%u policies, boost=%ums)\n",
		ib_nr_policies, READ_ONCE(input_boost_ms));
	return 0;

free_policies:
	kfree(ib_policies);
	return ret;
}

static void __exit ib_exit(void)
{
	int i;

	input_unregister_handler(&ib_input_handler);

	for (i = 0; i < ib_nr_policies; i++)
		cancel_delayed_work_sync(&ib_policies[i].work);

	/* Restore saved mins */
	for (i = 0; i < ib_nr_policies; i++) {
		if (ib_policies[i].policy) {
			ib_policies[i].policy->min = ib_policies[i].saved_min;
			cpufreq_update_policy(ib_policies[i].policy->cpu);
		}
	}

	kfree(ib_policies);
}

module_init(ib_init);
module_exit(ib_exit);

MODULE_DESCRIPTION("Zenith Input Boost — boost CPU on touch");
MODULE_AUTHOR("XTENSEI");
MODULE_LICENSE("GPL v2");
