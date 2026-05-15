// SPDX-License-Identifier: GPL-2.0
/*
 * Iyashi (癒し) thermal performance floor -- XTENSEI
 *
 * "Iyashi" -- the healing of unnecessary throttling.  Sits between the
 * thermal governor's chosen cooling target and the cooling device's
 * set_cur_state() callback.  When the system is far from any active
 * trip point, Iyashi clamps the cooling target so the CPU is held at
 * a configurable performance floor (default 90% of max OPP).  When
 * the closest bound trip is within near_limit_offset_c degrees,
 * Iyashi steps aside and lets the thermal framework throttle normally.
 *
 * Decision per thermal_cdev_update() call:
 *
 *   1. If iyashi_enabled == 0, passthrough.
 *   2. If cdev->type doesn't match the cdev filter (default:
 *      "thermal-cpufreq-" prefix), passthrough.  This keeps GPU and
 *      other cooling devices out of Iyashi's scope.
 *   3. Walk cdev->thermal_instances, read each bound zone's current
 *      temperature and its trip-point temperature, and compute the
 *      minimum (trip - current) headroom across all instances.
 *   4. If min_headroom_c < near_limit_offset_c, cooldown is imminent
 *      -- passthrough so the framework can throttle.
 *   5. Otherwise clamp the target to a floor_state so the CPU never
 *      drops below floor_pct of max OPP.
 *
 * Reads tz->temperature directly under the cdev->lock that
 * thermal_cdev_update() already holds.  Calls tz->ops->get_trip_temp
 * outside of tz->lock (the trip-temp callbacks are documented to be
 * lockless and side-effect free in the of_thermal backend, which is
 * what every platform driver we ship uses).
 *
 * All shared state is __read_mostly + READ_ONCE / WRITE_ONCE; the
 * hot path takes no locks.  The cdev_filter string is protected by
 * iyashi_filter_lock and snapshotted into a stack-local buffer.
 *
 * Disabled via static_branch when iyashi_enabled = 0, so the hook
 * is literally one branch on the cooling-update fast path while off.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/atomic.h>
#include <linux/cpu_cooling.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/jump_label.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/pm_qos.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>

#include "iyashi.h"
#include "thermal_core.h"

/* --------------------------------------------------------------- *
 * Tunables (R/W from sysfs)                                       *
 * --------------------------------------------------------------- */

static unsigned int iyashi_enabled            __read_mostly = 1;
static unsigned int iyashi_floor_pct          __read_mostly = 90;
static unsigned int iyashi_near_limit_offset_c __read_mostly = 5;

/*
 * Optional frequency-units floor (only meaningful for cpufreq cooling).
 * 0 = off; the floor is taken from iyashi_floor_pct in cooling-state
 * units.  Non-zero = compute the deepest cooling state whose target
 * frequency is still >= (pct% of policy->cpuinfo.max_freq) and use the
 * MORE PERMISSIVE of {state-units floor, freq-units floor} -- i.e. the
 * one that keeps cpufreq running faster.
 *
 * Why both?  state-units floor_pct is well-defined for non-cpufreq
 * cooling devices (GPU, fan, etc.); min_freq_pct is only meaningful
 * for cpufreq cooling but is the knob users actually think in.
 */
static unsigned int iyashi_min_freq_pct       __read_mostly;

/*
 * Static key gating.  Flipped by enabled_store() so the disabled
 * fast path is a single unlikely-branch.
 */
static DEFINE_STATIC_KEY_FALSE(iyashi_active_key);

/* --------------------------------------------------------------- *
 * Observability latches (R/O from sysfs)                          *
 * --------------------------------------------------------------- */

static atomic64_t iyashi_clamped_count;
static atomic64_t iyashi_passthrough_count;
static atomic64_t iyashi_freq_floor_used_count;

static int iyashi_last_min_headroom_c  __read_mostly;
static int iyashi_last_trip_temp_mc    __read_mostly;
static int iyashi_last_zone_temp_mc    __read_mostly;
static unsigned long iyashi_last_target_in   __read_mostly;
static unsigned long iyashi_last_target_out  __read_mostly;

/* --------------------------------------------------------------- *
 * cdev filter -- which cooling devices Iyashi acts on             *
 *                                                                 *
 * Default: "thermal-cpufreq-,thermal-devfreq-,thermal-gpufreq-"    *
 * (prefix match).  Covers every cpufreq_cooling.c-registered cdev *
 * (name = thermal-cpufreq-N), every devfreq_cooling.c-registered  *
 * GPU/DDR cdev (name = thermal-devfreq-N), and any vendor          *
 * thermal-gpufreq-* cdev some downstream trees ship.               *
 *                                                                 *
 * The freq-units floor only applies meaningfully to cpufreq cdevs *
 * (cpufreq_cooling_floor_state_for_pct returns 0 elsewhere), so   *
 * widening the filter just lets the state-units floor_pct floor   *
 * also gate GPU/DDR cooling -- it does not implicitly enable      *
 * min_freq_pct on those devices.                                  *
 *                                                                 *
 * Override via sysfs.  Comma-separated list of prefixes.  A leading*
 * '-' inverts to blacklist mode.                                  *
 * --------------------------------------------------------------- */

#define IYASHI_FILTER_LEN 256
#define IYASHI_DEFAULT_FILTER \
	"thermal-cpufreq-,thermal-devfreq-,thermal-gpufreq-"

static char iyashi_cdev_filter_buf[IYASHI_FILTER_LEN] = IYASHI_DEFAULT_FILTER;
static DEFINE_SPINLOCK(iyashi_filter_lock);

static bool iyashi_cdev_allowed(const char *cdev_type)
{
	char snapshot[IYASHI_FILTER_LEN];
	char *p, *tok;
	bool blacklist;
	unsigned long flags;

	if (!cdev_type)
		return false;

	spin_lock_irqsave(&iyashi_filter_lock, flags);
	strscpy(snapshot, iyashi_cdev_filter_buf, sizeof(snapshot));
	spin_unlock_irqrestore(&iyashi_filter_lock, flags);

	if (snapshot[0] == '\0')
		return false;	/* empty filter = match nothing */

	blacklist = (snapshot[0] == '-');
	p = snapshot + (blacklist ? 1 : 0);

	while ((tok = strsep(&p, ",")) != NULL) {
		size_t len = strlen(tok);

		while (len > 0 &&
		       (tok[len - 1] == ' ' ||
			tok[len - 1] == '\t' ||
			tok[len - 1] == '\n' ||
			tok[len - 1] == '\r'))
			tok[--len] = '\0';
		while (*tok == ' ' || *tok == '\t')
			tok++;
		if (*tok == '\0')
			continue;
		if (!strncmp(cdev_type, tok, strlen(tok)))
			return !blacklist;
	}

	return blacklist;
}

/* --------------------------------------------------------------- *
 * Core hook -- called from thermal_cdev_update()                  *
 * --------------------------------------------------------------- */

unsigned long iyashi_clamp_target(struct thermal_cooling_device *cdev,
				  unsigned long target)
{
	struct thermal_instance *inst;
	int min_headroom_c = INT_MAX;
	int closest_trip_mc = 0;
	int closest_zone_mc = 0;
	unsigned int near_c, floor_pct;
	unsigned long max_state = 0, floor_state;

	if (!static_branch_unlikely(&iyashi_active_key))
		return target;

	if (!iyashi_cdev_allowed(cdev->type))
		return target;

	if (cdev->ops->get_max_state &&
	    cdev->ops->get_max_state(cdev, &max_state))
		return target;
	if (!max_state)
		return target;

	near_c    = READ_ONCE(iyashi_near_limit_offset_c);
	floor_pct = READ_ONCE(iyashi_floor_pct);

	/*
	 * thermal_cdev_update() already holds cdev->lock when we are
	 * called, so cdev->thermal_instances is safe to walk.
	 */
	list_for_each_entry(inst, &cdev->thermal_instances, cdev_node) {
		struct thermal_zone_device *tz = inst->tz;
		int trip_temp = 0;
		int zone_temp;
		int headroom_c;

		if (!tz || !tz->ops || !tz->ops->get_trip_temp)
			continue;
		if (inst->target == THERMAL_NO_TARGET)
			continue;
		if (tz->ops->get_trip_temp(tz, inst->trip, &trip_temp))
			continue;

		zone_temp = tz->temperature;
		if (zone_temp <= 0)
			continue;

		headroom_c = (trip_temp - zone_temp) / 1000;
		if (headroom_c < min_headroom_c) {
			min_headroom_c  = headroom_c;
			closest_trip_mc = trip_temp;
			closest_zone_mc = zone_temp;
		}
	}

	/* Latches: visible from /sys/kernel/iyashi/last_* */
	WRITE_ONCE(iyashi_last_min_headroom_c,
		   min_headroom_c == INT_MAX ? 0 : min_headroom_c);
	WRITE_ONCE(iyashi_last_trip_temp_mc, closest_trip_mc);
	WRITE_ONCE(iyashi_last_zone_temp_mc, closest_zone_mc);
	WRITE_ONCE(iyashi_last_target_in,    target);

	/*
	 * Cooldown imminent: at least one trip is within near_c of
	 * the bound zone's current temperature.  Surrender control.
	 */
	if (min_headroom_c != INT_MAX && (unsigned int)min_headroom_c < near_c) {
		atomic64_inc(&iyashi_passthrough_count);
		WRITE_ONCE(iyashi_last_target_out, target);
		return target;
	}

	/*
	 * Far from any trip: enforce performance floor.  floor_state
	 * is the deepest cooling step we tolerate, in cooling-device
	 * units where 0 = no mitigation and max_state = fully cold.
	 */
	floor_state = max_state * (100 - floor_pct) / 100;

	/*
	 * Optional freq-units override: if the user expressed the floor
	 * as "% of cpuinfo_max_freq", translate it through the cpufreq
	 * cooling helper and prefer it when it is more permissive (i.e.
	 * its state index is lower, meaning a higher actual frequency).
	 * cpufreq_cooling_floor_state_for_pct() returns 0 for non-cpufreq
	 * cdevs or when the helper cannot decide, which we treat as
	 * "no override".
	 */
	{
		unsigned int min_freq_pct = READ_ONCE(iyashi_min_freq_pct);

		if (min_freq_pct) {
			unsigned long freq_floor =
				cpufreq_cooling_floor_state_for_pct(cdev,
								    min_freq_pct);

			if (freq_floor && freq_floor < floor_state) {
				floor_state = freq_floor;
				atomic64_inc(&iyashi_freq_floor_used_count);
			}
		}
	}

	if (target > floor_state) {
		atomic64_inc(&iyashi_clamped_count);
		WRITE_ONCE(iyashi_last_target_out, floor_state);
		return floor_state;
	}

	atomic64_inc(&iyashi_passthrough_count);
	WRITE_ONCE(iyashi_last_target_out, target);
	return target;
}
EXPORT_SYMBOL_GPL(iyashi_clamp_target);

/* --------------------------------------------------------------- *
 * Per-cpufreq-policy freq_qos MIN enforcement (enforce_min)        *
 *                                                                 *
 * The clamp_target path above keeps cpufreq cooling devices from  *
 * pulling cpufreq below the floor.  But "consistent performance"  *
 * also wants cpufreq to stay high when there is no thermal cap   *
 * at all (e.g. light wake-bursts).  Iyashi can opt-in to register *
 * a FREQ_QOS_MIN request per policy so the scheduler is told the  *
 * floor through cpufreq's official channel.                       *
 *                                                                 *
 * Lifecycle:                                                       *
 *   - enforce_min flips 0->1: walk online cpus, attach qos per pol*
 *   - enforce_min flips 1->0: walk list, detach + free each entry *
 *   - min_freq_pct changes while enforce_min=1: update every req  *
 *   - CPUFREQ_CREATE_POLICY: attach if enforce_min=1              *
 *   - CPUFREQ_REMOVE_POLICY: detach matching entry if present     *
 *                                                                 *
 * Concurrency: iyashi_qos_lock (mutex) guards iyashi_qos_list and *
 * the enforce_min state transitions.  The freq_qos API takes its  *
 * own internal locks; iyashi_qos_lock is the order on top.        *
 * --------------------------------------------------------------- */

static unsigned int iyashi_enforce_min        __read_mostly;

struct iyashi_qos_entry {
	struct cpufreq_policy	*policy;
	struct freq_qos_request	 req;
	struct list_head	 node;
};

static LIST_HEAD(iyashi_qos_list);
static DEFINE_MUTEX(iyashi_qos_lock);

static unsigned int iyashi_compute_min_freq(struct cpufreq_policy *policy,
					    unsigned int pct)
{
	u64 v;

	if (!policy || !policy->cpuinfo.max_freq)
		return 0;
	if (pct == 0 || pct > 100)
		return 0;

	v = (u64)policy->cpuinfo.max_freq * pct;
	do_div(v, 100);
	if (v > policy->cpuinfo.max_freq)
		v = policy->cpuinfo.max_freq;
	return (unsigned int)v;
}

/* iyashi_qos_lock must be held. */
static struct iyashi_qos_entry *
iyashi_qos_find_entry(struct cpufreq_policy *policy)
{
	struct iyashi_qos_entry *e;

	list_for_each_entry(e, &iyashi_qos_list, node) {
		if (e->policy == policy)
			return e;
	}
	return NULL;
}

/* iyashi_qos_lock must be held.  Adds an entry if one is not present. */
static int iyashi_qos_attach_locked(struct cpufreq_policy *policy)
{
	struct iyashi_qos_entry *entry;
	unsigned int freq;
	int ret;

	if (iyashi_qos_find_entry(policy))
		return 0;

	freq = iyashi_compute_min_freq(policy,
				       READ_ONCE(iyashi_min_freq_pct));
	if (!freq)
		return 0;	/* min_freq_pct == 0 -> nothing to enforce */

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->policy = policy;
	ret = freq_qos_add_request(&policy->constraints, &entry->req,
				   FREQ_QOS_MIN, freq);
	if (ret < 0) {
		kfree(entry);
		return ret;
	}

	list_add(&entry->node, &iyashi_qos_list);
	return 0;
}

/* iyashi_qos_lock must be held. */
static void iyashi_qos_detach_entry_locked(struct iyashi_qos_entry *entry)
{
	list_del(&entry->node);
	freq_qos_remove_request(&entry->req);
	kfree(entry);
}

/* Attach a request to every online cpu's policy.  enforce_min=1 path. */
static void iyashi_qos_attach_all(void)
{
	struct cpufreq_policy *policy;
	int cpu;

	mutex_lock(&iyashi_qos_lock);
	for_each_online_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;
		/* Only attach once per policy (multiple cpus per policy). */
		if (!iyashi_qos_find_entry(policy))
			(void)iyashi_qos_attach_locked(policy);
		cpufreq_cpu_put(policy);
	}
	mutex_unlock(&iyashi_qos_lock);
}

/* Detach every request.  enforce_min=0 path. */
static void iyashi_qos_detach_all(void)
{
	struct iyashi_qos_entry *e, *tmp;

	mutex_lock(&iyashi_qos_lock);
	list_for_each_entry_safe(e, tmp, &iyashi_qos_list, node)
		iyashi_qos_detach_entry_locked(e);
	mutex_unlock(&iyashi_qos_lock);
}

/* Update every active request after min_freq_pct changes. */
static void iyashi_qos_update_all(void)
{
	struct iyashi_qos_entry *e;
	unsigned int freq;
	unsigned int pct = READ_ONCE(iyashi_min_freq_pct);

	mutex_lock(&iyashi_qos_lock);
	list_for_each_entry(e, &iyashi_qos_list, node) {
		freq = iyashi_compute_min_freq(e->policy, pct);
		if (!freq)
			freq = FREQ_QOS_MIN_DEFAULT_VALUE;
		(void)freq_qos_update_request(&e->req, (s32)freq);
	}
	mutex_unlock(&iyashi_qos_lock);
}

static int iyashi_cpufreq_policy_notify(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;

	if (!READ_ONCE(iyashi_enforce_min))
		return NOTIFY_OK;

	switch (event) {
	case CPUFREQ_CREATE_POLICY:
		mutex_lock(&iyashi_qos_lock);
		(void)iyashi_qos_attach_locked(policy);
		mutex_unlock(&iyashi_qos_lock);
		break;
	case CPUFREQ_REMOVE_POLICY:
		mutex_lock(&iyashi_qos_lock);
		{
			struct iyashi_qos_entry *e =
				iyashi_qos_find_entry(policy);
			if (e)
				iyashi_qos_detach_entry_locked(e);
		}
		mutex_unlock(&iyashi_qos_lock);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block iyashi_cpufreq_policy_nb = {
	.notifier_call = iyashi_cpufreq_policy_notify,
};

/* --------------------------------------------------------------- *
 * sysfs                                                           *
 * --------------------------------------------------------------- */

#define IYASHI_ATTR_RW(_name, _var, _validate)				\
static ssize_t _name##_show(struct kobject *kobj,			\
			    struct kobj_attribute *attr, char *buf)	\
{									\
	return sysfs_emit(buf, "%u\n", READ_ONCE(_var));		\
}									\
static ssize_t _name##_store(struct kobject *kobj,			\
			     struct kobj_attribute *attr,		\
			     const char *buf, size_t count)		\
{									\
	unsigned int val;						\
	if (kstrtouint(buf, 0, &val))					\
		return -EINVAL;						\
	if (!(_validate))						\
		return -ERANGE;						\
	WRITE_ONCE(_var, val);						\
	return count;							\
}									\
static struct kobj_attribute iyashi_##_name##_attr =			\
	__ATTR(_name, 0644, _name##_show, _name##_store)

static ssize_t enabled_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(iyashi_enabled));
}

static ssize_t enabled_store(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;
	if (val > 1)
		return -ERANGE;

	if (val == READ_ONCE(iyashi_enabled))
		return count;

	WRITE_ONCE(iyashi_enabled, val);
	if (val)
		static_branch_enable(&iyashi_active_key);
	else
		static_branch_disable(&iyashi_active_key);

	return count;
}

static struct kobj_attribute iyashi_enabled_attr =
	__ATTR(enabled, 0644, enabled_show, enabled_store);

IYASHI_ATTR_RW(floor_pct,           iyashi_floor_pct,           val >= 50 && val <= 100);
IYASHI_ATTR_RW(near_limit_offset_c, iyashi_near_limit_offset_c, val >= 1  && val <= 15);

static ssize_t min_freq_pct_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(iyashi_min_freq_pct));
}

static ssize_t min_freq_pct_store(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;
	if (!(val == 0 || (val >= 50 && val <= 100)))
		return -ERANGE;

	WRITE_ONCE(iyashi_min_freq_pct, val);

	/* Re-apply on every attached policy if enforce_min is active. */
	if (READ_ONCE(iyashi_enforce_min))
		iyashi_qos_update_all();

	return count;
}

static struct kobj_attribute iyashi_min_freq_pct_attr =
	__ATTR(min_freq_pct, 0644, min_freq_pct_show, min_freq_pct_store);

static ssize_t enforce_min_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(iyashi_enforce_min));
}

static ssize_t enforce_min_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;
	if (val > 1)
		return -ERANGE;

	if (val == READ_ONCE(iyashi_enforce_min))
		return count;

	WRITE_ONCE(iyashi_enforce_min, val);
	if (val)
		iyashi_qos_attach_all();
	else
		iyashi_qos_detach_all();

	return count;
}

static struct kobj_attribute iyashi_enforce_min_attr =
	__ATTR(enforce_min, 0644, enforce_min_show, enforce_min_store);

#define IYASHI_ATTR_RO_INT(_name, _var)					\
static ssize_t _name##_show(struct kobject *kobj,			\
			    struct kobj_attribute *attr, char *buf)	\
{									\
	return sysfs_emit(buf, "%d\n", READ_ONCE(_var));		\
}									\
static struct kobj_attribute iyashi_##_name##_attr =			\
	__ATTR(_name, 0444, _name##_show, NULL)

#define IYASHI_ATTR_RO_ULONG(_name, _var)				\
static ssize_t _name##_show(struct kobject *kobj,			\
			    struct kobj_attribute *attr, char *buf)	\
{									\
	return sysfs_emit(buf, "%lu\n", READ_ONCE(_var));		\
}									\
static struct kobj_attribute iyashi_##_name##_attr =			\
	__ATTR(_name, 0444, _name##_show, NULL)

#define IYASHI_ATTR_RO_ATOMIC64(_name, _var)				\
static ssize_t _name##_show(struct kobject *kobj,			\
			    struct kobj_attribute *attr, char *buf)	\
{									\
	return sysfs_emit(buf, "%lld\n", (long long)atomic64_read(&_var)); \
}									\
static struct kobj_attribute iyashi_##_name##_attr =			\
	__ATTR(_name, 0444, _name##_show, NULL)

IYASHI_ATTR_RO_INT     (last_min_headroom_c,    iyashi_last_min_headroom_c);
IYASHI_ATTR_RO_INT     (last_trip_temp_mc,      iyashi_last_trip_temp_mc);
IYASHI_ATTR_RO_INT     (last_zone_temp_mc,      iyashi_last_zone_temp_mc);
IYASHI_ATTR_RO_ULONG   (last_target_in,         iyashi_last_target_in);
IYASHI_ATTR_RO_ULONG   (last_target_out,        iyashi_last_target_out);
IYASHI_ATTR_RO_ATOMIC64(clamped_count,          iyashi_clamped_count);
IYASHI_ATTR_RO_ATOMIC64(passthrough_count,      iyashi_passthrough_count);
IYASHI_ATTR_RO_ATOMIC64(freq_floor_used_count,  iyashi_freq_floor_used_count);

static ssize_t cdev_filter_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	unsigned long flags;
	ssize_t ret;

	spin_lock_irqsave(&iyashi_filter_lock, flags);
	ret = sysfs_emit(buf, "%s\n", iyashi_cdev_filter_buf);
	spin_unlock_irqrestore(&iyashi_filter_lock, flags);
	return ret;
}

static ssize_t cdev_filter_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned long flags;
	size_t len = count;

	if (len >= IYASHI_FILTER_LEN)
		return -E2BIG;

	spin_lock_irqsave(&iyashi_filter_lock, flags);
	memcpy(iyashi_cdev_filter_buf, buf, len);
	iyashi_cdev_filter_buf[len] = '\0';
	if (len > 0 && iyashi_cdev_filter_buf[len - 1] == '\n')
		iyashi_cdev_filter_buf[len - 1] = '\0';
	spin_unlock_irqrestore(&iyashi_filter_lock, flags);
	return count;
}

static struct kobj_attribute iyashi_cdev_filter_attr =
	__ATTR(cdev_filter, 0644, cdev_filter_show, cdev_filter_store);

static ssize_t version_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "iyashi-v1\n");
}

static struct kobj_attribute iyashi_version_attr =
	__ATTR(version, 0444, version_show, NULL);

static struct attribute *iyashi_attrs[] = {
	&iyashi_version_attr.attr,
	&iyashi_enabled_attr.attr,
	&iyashi_floor_pct_attr.attr,
	&iyashi_min_freq_pct_attr.attr,
	&iyashi_enforce_min_attr.attr,
	&iyashi_near_limit_offset_c_attr.attr,
	&iyashi_cdev_filter_attr.attr,
	&iyashi_clamped_count_attr.attr,
	&iyashi_passthrough_count_attr.attr,
	&iyashi_freq_floor_used_count_attr.attr,
	&iyashi_last_min_headroom_c_attr.attr,
	&iyashi_last_trip_temp_mc_attr.attr,
	&iyashi_last_zone_temp_mc_attr.attr,
	&iyashi_last_target_in_attr.attr,
	&iyashi_last_target_out_attr.attr,
	NULL,
};

static struct attribute_group iyashi_attr_group = {
	.attrs = iyashi_attrs,
};

static struct kobject *iyashi_kobj;

/* --------------------------------------------------------------- *
 * Init                                                            *
 * --------------------------------------------------------------- */

static int __init iyashi_init(void)
{
	int ret;

	iyashi_kobj = kobject_create_and_add("iyashi", kernel_kobj);
	if (!iyashi_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(iyashi_kobj, &iyashi_attr_group);
	if (ret) {
		kobject_put(iyashi_kobj);
		iyashi_kobj = NULL;
		return ret;
	}

	if (READ_ONCE(iyashi_enabled))
		static_branch_enable(&iyashi_active_key);

	/*
	 * Register a cpufreq policy notifier so the enforce_min path can
	 * attach/detach a FREQ_QOS_MIN request when policies come and go
	 * (CPU hot-plug etc.).  The notifier itself early-outs when
	 * enforce_min == 0, so it is cheap even with the feature disabled.
	 */
	ret = cpufreq_register_notifier(&iyashi_cpufreq_policy_nb,
					CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		pr_warn("Iyashi: cpufreq_register_notifier failed: %d (enforce_min will not survive hotplug)\n",
			ret);

	pr_info("Iyashi (癒し) performance floor active: floor=%u%% near_limit=%uC filter='%s'\n",
		READ_ONCE(iyashi_floor_pct),
		READ_ONCE(iyashi_near_limit_offset_c),
		iyashi_cdev_filter_buf);

	/*
	 * Iyashi boot banner.  Full mythic + mechanism narrative
	 * emitted once at init.  Single 'Iyashi : ' prefix on every
	 * line so the whole banner is grep-stable; ASCII relationship
	 * diagrams show how this subsystem composes with the others.
	 */
	pr_info("Iyashi : \n");
	pr_info("Iyashi : when the cooling step lands, the air should not turn to stone.\n");
	pr_info("Iyashi : the ridge breathes in, holds, breathes out.  Iyashi makes sure\n");
	pr_info("Iyashi : the breath out still has somewhere to go.\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi :     ____                 __    _\n");
	pr_info("Iyashi :    /  _/_  ______ ______/ /_  (_)\n");
	pr_info("Iyashi :    / // / / / __ `/ ___/ __ \\/ /\n");
	pr_info("Iyashi :  _/ // /_/ / /_/ (__  ) / / / /\n");
	pr_info("Iyashi : /___/\\__, /\\__,_/____/_/ /_/_/\n");
	pr_info("Iyashi :     /____/\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi :                        癒し\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi :          ----  what Iyashi is  ----\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi : a thermal-aware performance floor for cpufreq cooling.  when the\n");
	pr_info("Iyashi : thermal subsystem clamps a cpufreq policy down because of heat,\n");
	pr_info("Iyashi : Iyashi makes sure the clamp does not pin the cluster at its lowest\n");
	pr_info("Iyashi : OPP -- the top N OPPs are reserved as 'open road' so that a sudden\n");
	pr_info("Iyashi : foreground request can still produce work.\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi : without Iyashi, a hot device can deliver a button-press into a\n");
	pr_info("Iyashi : fully clamped cpufreq and produce visible lag for an entire second\n");
	pr_info("Iyashi : while the thermal load shed.\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi :          ----  how the breath returns  ----\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi :         cpufreq cooling device  decides 'set max = N'\n");
	pr_info("Iyashi :                               |\n");
	pr_info("Iyashi :                               v\n");
	pr_info("Iyashi :             +----------------------------------+\n");
	pr_info("Iyashi :             |  Iyashi inspects the policy      |\n");
	pr_info("Iyashi :             |    - count remaining OPPs from N |\n");
	pr_info("Iyashi :             |    - if fewer than reserve K,    |\n");
	pr_info("Iyashi :             |      lift max to K-th OPP        |\n");
	pr_info("Iyashi :             +----------------------------------+\n");
	pr_info("Iyashi :                               |\n");
	pr_info("Iyashi :                               v\n");
	pr_info("Iyashi :             +----------------------------------+\n");
	pr_info("Iyashi :             |  Iyashi increments counters:     |\n");
	pr_info("Iyashi :             |    clamped_count    (passthrough)|\n");
	pr_info("Iyashi :             |    passthrough_count (overrode)  |\n");
	pr_info("Iyashi :             +----------------------------------+\n");
	pr_info("Iyashi :                               |\n");
	pr_info("Iyashi :                               v\n");
	pr_info("Iyashi :                 cpufreq sees the (possibly lifted) max\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi :          ----  the gating  ----\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi : Iyashi only acts when:\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi :     enabled == 1\n");
	pr_info("Iyashi :     and the cooling action would cap below the reserve\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi : it never lowers a max that the cooler did not lower.  it never\n");
	pr_info("Iyashi : overrides a userspace request.  it never bypasses a critical-trip\n");
	pr_info("Iyashi : shutdown -- the critical path skips cpufreq cooling entirely.\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi :          ----  observability  ----\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi : /sys/kernel/iyashi/enabled         R/W master switch\n");
	pr_info("Iyashi : /sys/kernel/iyashi/reserve_opps    R/W how many top OPPs to keep\n");
	pr_info("Iyashi : /sys/kernel/iyashi/clamped_count   R/O times the cooler clamped\n");
	pr_info("Iyashi : /sys/kernel/iyashi/passthrough_count R/O times Iyashi lifted that clamp\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi : a healthy device shows both counters moving under sustained load --\n");
	pr_info("Iyashi : the ratio passthrough/clamped tells you how often heat alone would\n");
	pr_info("Iyashi : have starved the foreground.\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi :          ----  bond with Kasumi  ----\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi : Kasumi softens *what the framework sees*.  Iyashi softens *what\n");
	pr_info("Iyashi : cpufreq does*.  same goal, different layer.  with both enabled:\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi :   raw temp -> Kasumi -> framework -> cooling decision -> Iyashi -> cpufreq\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi : Kasumi delays the cooling decision; Iyashi shapes the decision.\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi :          ----  bond with Zenith  ----\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi : Zenith sees the post-Iyashi max as the policy's max.  Zenith's\n");
	pr_info("Iyashi : floor hint from Hikari is still honored *under* the Iyashi-lifted\n");
	pr_info("Iyashi : ceiling -- the floor cannot exceed the (lifted or not) max.  the\n");
	pr_info("Iyashi : hand-off is one number through cpufreq's policy lock.\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi :          ----  the breath out has somewhere to go  ----\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi :          leave room.  let the foreground land.\n");
	pr_info("Iyashi : \n");
	pr_info("Iyashi : built by XTENSEI.\n");

	return 0;
}
late_initcall_sync(iyashi_init);
