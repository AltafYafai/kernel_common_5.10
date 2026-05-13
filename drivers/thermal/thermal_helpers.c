// SPDX-License-Identifier: GPL-2.0
/*
 *  thermal_helpers.c - helper functions to handle thermal devices
 *
 *  Copyright (C) 2016 Eduardo Valentin <edubezval@gmail.com>
 *
 *  Highly based on original thermal_core.c
 *  Copyright (C) 2008 Intel Corp
 *  Copyright (C) 2008 Zhang Rui <rui.zhang@intel.com>
 *  Copyright (C) 2008 Sujith Thomas <sujith.thomas@intel.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/device.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include <trace/events/thermal.h>

#include "thermal_core.h"

/*
 * Kasumi (霞) thermal dampening -- XTENSEI
 *
 * Delays thermal throttling by subtracting a configurable offset from
 * the reported temperature.  A linear ramp between ramp_start and
 * ceiling smoothly reduces the offset to zero so the real temperature
 * is reported once the safety ceiling is reached.
 *
 *   real < ramp_start : reported = real - offset        (full offset)
 *   ramp_start <= real < ceiling : offset tapers to 0   (smooth ramp)
 *   real >= ceiling   : reported = real                 (safety)
 */
static unsigned int kasumi_enable     __read_mostly = 1;
static unsigned int kasumi_offset_mc  __read_mostly = 15000;  /* 15 C  */
static unsigned int kasumi_ramp_mc    __read_mostly = 85000;  /* 85 C  */
static unsigned int kasumi_ceiling_mc __read_mostly = 95000;  /* 95 C  */

/*
 * Observability latches.  Updated at the end of every kasumi_dampen()
 * call; readable through /sys/kernel/kasumi/last_*.  R/O so userspace
 * cannot fabricate state.  WRITE_ONCE / READ_ONCE for tearing safety.
 */
static int kasumi_last_real_mc        __read_mostly;
static int kasumi_last_reported_mc    __read_mostly;
static int kasumi_applied_offset_mc   __read_mostly;

/*
 * Zone-type filter.  Comma-separated list of thermal_zone types.
 *   ""                       (default)  apply to all zones
 *   "mtktscpu,mtktspmic"     whitelist  apply only to listed zones
 *   "-battery,mtktsbattery"  blacklist  apply to all EXCEPT listed zones
 *
 * Protected by kasumi_filter_lock.  The hot path takes a stack-local
 * snapshot under the lock and parses without it.
 */
#define KASUMI_FILTER_LEN 256
static char kasumi_zone_filter_buf[KASUMI_FILTER_LEN];
static DEFINE_SPINLOCK(kasumi_filter_lock);

static bool kasumi_zone_allowed(const char *zone_type)
{
	char snapshot[KASUMI_FILTER_LEN];
	char *p, *tok;
	bool blacklist;
	unsigned long flags;

	if (!zone_type)
		return true;

	spin_lock_irqsave(&kasumi_filter_lock, flags);
	strscpy(snapshot, kasumi_zone_filter_buf, sizeof(snapshot));
	spin_unlock_irqrestore(&kasumi_filter_lock, flags);

	if (snapshot[0] == '\0')
		return true;

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
		if (!strcmp(tok, zone_type))
			return !blacklist;
	}

	return blacklist;
}

static int kasumi_dampen(int real)
{
	unsigned int offset, ramp, ceiling;
	int dampened;

	if (!READ_ONCE(kasumi_enable) || real <= 0)
		return real;

	offset  = READ_ONCE(kasumi_offset_mc);
	ramp    = READ_ONCE(kasumi_ramp_mc);
	ceiling = READ_ONCE(kasumi_ceiling_mc);

	if (real >= (int)ceiling) {
		WRITE_ONCE(kasumi_last_real_mc,      real);
		WRITE_ONCE(kasumi_last_reported_mc,  real);
		WRITE_ONCE(kasumi_applied_offset_mc, 0);
		return real;
	}

	if (real < (int)ramp) {
		dampened = real - (int)offset;
	} else {
		int range     = (int)ceiling - (int)ramp;
		int remaining = (int)ceiling - real;

		if (range > 0)
			dampened = real - (int)offset * remaining / range;
		else
			dampened = real;
	}

	if (dampened < 0)
		dampened = 0;

	WRITE_ONCE(kasumi_last_real_mc,      real);
	WRITE_ONCE(kasumi_last_reported_mc,  dampened);
	WRITE_ONCE(kasumi_applied_offset_mc, real - dampened);

	return dampened;
}

/*
 * Iyashi consumers read this; do not export to userspace through any
 * other channel than the /sys/kernel/kasumi/last_real_mc attribute.
 */
int kasumi_get_last_real_mc(void)
{
	return READ_ONCE(kasumi_last_real_mc);
}
EXPORT_SYMBOL_GPL(kasumi_get_last_real_mc);

int get_tz_trend(struct thermal_zone_device *tz, int trip)
{
	enum thermal_trend trend;

	if (tz->emul_temperature || !tz->ops->get_trend ||
	    tz->ops->get_trend(tz, trip, &trend)) {
		if (tz->temperature > tz->last_temperature)
			trend = THERMAL_TREND_RAISING;
		else if (tz->temperature < tz->last_temperature)
			trend = THERMAL_TREND_DROPPING;
		else
			trend = THERMAL_TREND_STABLE;
	}

	return trend;
}
EXPORT_SYMBOL(get_tz_trend);

struct thermal_instance *
get_thermal_instance(struct thermal_zone_device *tz,
		     struct thermal_cooling_device *cdev, int trip)
{
	struct thermal_instance *pos = NULL;
	struct thermal_instance *target_instance = NULL;

	mutex_lock(&tz->lock);
	mutex_lock(&cdev->lock);

	list_for_each_entry(pos, &tz->thermal_instances, tz_node) {
		if (pos->tz == tz && pos->trip == trip && pos->cdev == cdev) {
			target_instance = pos;
			break;
		}
	}

	mutex_unlock(&cdev->lock);
	mutex_unlock(&tz->lock);

	return target_instance;
}
EXPORT_SYMBOL(get_thermal_instance);

/**
 * thermal_zone_get_temp() - returns the temperature of a thermal zone
 * @tz: a valid pointer to a struct thermal_zone_device
 * @temp: a valid pointer to where to store the resulting temperature.
 *
 * When a valid thermal zone reference is passed, it will fetch its
 * temperature and fill @temp.
 *
 * Return: On success returns 0, an error code otherwise
 */
int thermal_zone_get_temp(struct thermal_zone_device *tz, int *temp)
{
	int ret = -EINVAL;
	int count;
	int crit_temp = INT_MAX;
	enum thermal_trip_type type;

	if (!tz || IS_ERR(tz) || !tz->ops->get_temp)
		goto exit;

	mutex_lock(&tz->lock);

	ret = tz->ops->get_temp(tz, temp);

	if (IS_ENABLED(CONFIG_THERMAL_EMULATION) && tz->emul_temperature) {
		for (count = 0; count < tz->trips; count++) {
			ret = tz->ops->get_trip_type(tz, count, &type);
			if (!ret && type == THERMAL_TRIP_CRITICAL) {
				ret = tz->ops->get_trip_temp(tz, count,
						&crit_temp);
				break;
			}
		}

		/*
		 * Only allow emulating a temperature when the real temperature
		 * is below the critical temperature so that the emulation code
		 * cannot hide critical conditions.
		 */
		if (!ret && *temp < crit_temp)
			*temp = tz->emul_temperature;
	}

	/* Kasumi thermal dampening (skip zones excluded by zone_filter) */
	if (!ret && kasumi_zone_allowed(tz->type))
		*temp = kasumi_dampen(*temp);

	mutex_unlock(&tz->lock);
exit:
	return ret;
}
EXPORT_SYMBOL_GPL(thermal_zone_get_temp);

/**
 * thermal_zone_set_trips - Computes the next trip points for the driver
 * @tz: a pointer to a thermal zone device structure
 *
 * The function computes the next temperature boundaries by browsing
 * the trip points. The result is the closer low and high trip points
 * to the current temperature. These values are passed to the backend
 * driver to let it set its own notification mechanism (usually an
 * interrupt).
 *
 * It does not return a value
 */
void thermal_zone_set_trips(struct thermal_zone_device *tz)
{
	int low = -INT_MAX;
	int high = INT_MAX;
	int trip_temp, hysteresis;
	int i, ret;

	mutex_lock(&tz->lock);

	if (!tz->ops->set_trips || !tz->ops->get_trip_hyst)
		goto exit;

	for (i = 0; i < tz->trips; i++) {
		int trip_low;

		tz->ops->get_trip_temp(tz, i, &trip_temp);
		tz->ops->get_trip_hyst(tz, i, &hysteresis);

		trip_low = trip_temp - hysteresis;

		if (trip_low < tz->temperature && trip_low > low)
			low = trip_low;

		if (trip_temp > tz->temperature && trip_temp < high)
			high = trip_temp;
	}

	/* No need to change trip points */
	if (tz->prev_low_trip == low && tz->prev_high_trip == high)
		goto exit;

	tz->prev_low_trip = low;
	tz->prev_high_trip = high;

	dev_dbg(&tz->device,
		"new temperature boundaries: %d < x < %d\n", low, high);

	/*
	 * Set a temperature window. When this window is left the driver
	 * must inform the thermal core via thermal_zone_device_update.
	 */
	ret = tz->ops->set_trips(tz, low, high);
	if (ret)
		dev_err(&tz->device, "Failed to set trips: %d\n", ret);

exit:
	mutex_unlock(&tz->lock);
}

static void thermal_cdev_set_cur_state(struct thermal_cooling_device *cdev,
				       int target)
{
	if (cdev->ops->set_cur_state(cdev, target))
		return;

	thermal_notify_cdev_state_update(cdev->id, target);
	thermal_cooling_device_stats_update(cdev, target);
}

void thermal_cdev_update(struct thermal_cooling_device *cdev)
{
	struct thermal_instance *instance;
	unsigned long target = 0;

	mutex_lock(&cdev->lock);
	/* cooling device is updated*/
	if (cdev->updated) {
		mutex_unlock(&cdev->lock);
		return;
	}

	/* Make sure cdev enters the deepest cooling state */
	list_for_each_entry(instance, &cdev->thermal_instances, cdev_node) {
		dev_dbg(&cdev->device, "zone%d->target=%lu\n",
			instance->tz->id, instance->target);
		if (instance->target == THERMAL_NO_TARGET)
			continue;
		if (instance->target > target)
			target = instance->target;
	}

	thermal_cdev_set_cur_state(cdev, target);

	cdev->updated = true;
	mutex_unlock(&cdev->lock);
	trace_cdev_update(cdev, target);
	dev_dbg(&cdev->device, "set to state %lu\n", target);
}
EXPORT_SYMBOL(thermal_cdev_update);

/**
 * thermal_zone_get_slope - return the slope attribute of the thermal zone
 * @tz: thermal zone device with the slope attribute
 *
 * Return: If the thermal zone device has a slope attribute, return it, else
 * return 1.
 */
int thermal_zone_get_slope(struct thermal_zone_device *tz)
{
	if (tz && tz->tzp)
		return tz->tzp->slope;
	return 1;
}
EXPORT_SYMBOL_GPL(thermal_zone_get_slope);

/**
 * thermal_zone_get_offset - return the offset attribute of the thermal zone
 * @tz: thermal zone device with the offset attribute
 *
 * Return: If the thermal zone device has a offset attribute, return it, else
 * return 0.
 */
int thermal_zone_get_offset(struct thermal_zone_device *tz)
{
	if (tz && tz->tzp)
		return tz->tzp->offset;
	return 0;
}
EXPORT_SYMBOL_GPL(thermal_zone_get_offset);

/* ---- Kasumi sysfs interface (/sys/kernel/kasumi/) ---- */

#define KASUMI_ATTR_RW(_name, _var)					\
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
	WRITE_ONCE(_var, val);						\
	return count;							\
}									\
static struct kobj_attribute kasumi_##_name##_attr =			\
	__ATTR(_name, 0644, _name##_show, _name##_store)

#define KASUMI_ATTR_RO(_name, _var)					\
static ssize_t _name##_show(struct kobject *kobj,			\
			    struct kobj_attribute *attr, char *buf)	\
{									\
	return sysfs_emit(buf, "%d\n", READ_ONCE(_var));		\
}									\
static struct kobj_attribute kasumi_##_name##_attr =			\
	__ATTR(_name, 0444, _name##_show, NULL)

KASUMI_ATTR_RW(enabled,    kasumi_enable);
KASUMI_ATTR_RW(offset_mc,  kasumi_offset_mc);
KASUMI_ATTR_RW(ramp_mc,    kasumi_ramp_mc);
KASUMI_ATTR_RW(ceiling_mc, kasumi_ceiling_mc);

KASUMI_ATTR_RO(last_real_mc,       kasumi_last_real_mc);
KASUMI_ATTR_RO(last_reported_mc,   kasumi_last_reported_mc);
KASUMI_ATTR_RO(applied_offset_mc,  kasumi_applied_offset_mc);

static ssize_t zone_filter_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	unsigned long flags;
	ssize_t ret;

	spin_lock_irqsave(&kasumi_filter_lock, flags);
	ret = sysfs_emit(buf, "%s\n", kasumi_zone_filter_buf);
	spin_unlock_irqrestore(&kasumi_filter_lock, flags);
	return ret;
}

static ssize_t zone_filter_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned long flags;
	size_t len = count;

	if (len >= KASUMI_FILTER_LEN)
		return -E2BIG;

	spin_lock_irqsave(&kasumi_filter_lock, flags);
	memcpy(kasumi_zone_filter_buf, buf, len);
	kasumi_zone_filter_buf[len] = '\0';
	/* strip a single trailing newline so 'echo "..." > zone_filter' DTRT */
	if (len > 0 && kasumi_zone_filter_buf[len - 1] == '\n')
		kasumi_zone_filter_buf[len - 1] = '\0';
	spin_unlock_irqrestore(&kasumi_filter_lock, flags);
	return count;
}

static struct kobj_attribute kasumi_zone_filter_attr =
	__ATTR(zone_filter, 0644, zone_filter_show, zone_filter_store);

static struct attribute *kasumi_attrs[] = {
	&kasumi_enabled_attr.attr,
	&kasumi_offset_mc_attr.attr,
	&kasumi_ramp_mc_attr.attr,
	&kasumi_ceiling_mc_attr.attr,
	&kasumi_last_real_mc_attr.attr,
	&kasumi_last_reported_mc_attr.attr,
	&kasumi_applied_offset_mc_attr.attr,
	&kasumi_zone_filter_attr.attr,
	NULL,
};

static struct attribute_group kasumi_attr_group = {
	.attrs = kasumi_attrs,
};

static struct kobject *kasumi_kobj;

/*
 * Safety self-test.  Runs once at init before any sysfs is exposed and
 * asserts the three contracts kasumi_dampen() makes:
 *
 *   1. at-or-above-ceiling input is returned unchanged (no dampening),
 *   2. below-ramp input gets exactly offset_mc subtracted,
 *   3. zero / negative input is passed through.
 *
 * If any of the three fails we refuse to expose /sys/kernel/kasumi at
 * all, which makes the failure loud (anyone scripting against the
 * sysfs files will see ENOENT) and prevents a misconfigured build
 * from quietly hiding throttling.  The dampening hook itself stays
 * active either way -- this is purely a paranoia check on the math.
 */
static int __init kasumi_safety_self_test(void)
{
	unsigned int ceiling = READ_ONCE(kasumi_ceiling_mc);
	unsigned int offset  = READ_ONCE(kasumi_offset_mc);
	unsigned int ramp    = READ_ONCE(kasumi_ramp_mc);
	unsigned int saved_enable;
	int at_ceiling = (int)ceiling;
	int above_ceiling = (int)ceiling + 1000;
	int below_ramp = (int)ramp - 5000;
	int r1, r2, r3, r4;
	int ret = 0;

	/*
	 * Force kasumi_enable=1 for the duration of the test so the
	 * result is independent of the default we ship.  Restore on exit.
	 */
	saved_enable = READ_ONCE(kasumi_enable);
	WRITE_ONCE(kasumi_enable, 1);

	/* 1. at-or-above-ceiling: must be returned unchanged. */
	r1 = kasumi_dampen(at_ceiling);
	r2 = kasumi_dampen(above_ceiling);
	if (r1 != at_ceiling || r2 != above_ceiling) {
		pr_err("kasumi: safety self-test FAILED: at_ceiling=%d->%d, above_ceiling=%d->%d (expected unchanged)\n",
		       at_ceiling, r1, above_ceiling, r2);
		ret = -EIO;
		goto out;
	}

	/* 2. below-ramp: must be exactly real - offset. */
	if (below_ramp > 0) {
		r3 = kasumi_dampen(below_ramp);
		if (r3 != below_ramp - (int)offset) {
			pr_err("kasumi: safety self-test FAILED: below_ramp=%d->%d (expected %d)\n",
			       below_ramp, r3, below_ramp - (int)offset);
			ret = -EIO;
			goto out;
		}
	}

	/* 3. zero / negative: must be passed through. */
	r4 = kasumi_dampen(0);
	if (r4 != 0) {
		pr_err("kasumi: safety self-test FAILED: zero input -> %d (expected 0)\n", r4);
		ret = -EIO;
		goto out;
	}

	pr_info("kasumi: safety self-test passed (ceiling=%u offset=%u ramp=%u)\n",
		ceiling, offset, ramp);
out:
	WRITE_ONCE(kasumi_enable, saved_enable);
	return ret;
}

static int __init kasumi_sysfs_init(void)
{
	int ret;

	ret = kasumi_safety_self_test();
	if (ret) {
		pr_err("kasumi: refusing to register sysfs (self-test failed)\n");
		return ret;
	}

	kasumi_kobj = kobject_create_and_add("kasumi", kernel_kobj);
	if (!kasumi_kobj)
		return -ENOMEM;
	return sysfs_create_group(kasumi_kobj, &kasumi_attr_group);
}
late_initcall(kasumi_sysfs_init);
