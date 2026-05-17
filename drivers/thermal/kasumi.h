/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Kasumi (霞) thermal dampening -- XTENSEI
 *
 * Internal interface between the thermal core and kasumi.c.
 * kasumi_zone_allowed() and kasumi_dampen() are called from
 * thermal_zone_get_temp() in thermal_helpers.c; the other two
 * are consumed by Zenith's cpufreq governor.
 */

#ifndef __KASUMI_H__
#define __KASUMI_H__

#include <linux/thermal.h>

#ifdef CONFIG_KASUMI
bool kasumi_zone_allowed(const char *zone_type);
int kasumi_dampen(int real, const char *zone_type);
int kasumi_get_last_real_mc(void);
void kasumi_apply_profile(unsigned int profile);
#else
static inline bool kasumi_zone_allowed(const char *zone_type)
{
	return true;
}
static inline int kasumi_dampen(int real, const char *zone_type)
{
	return real;
}
static inline int kasumi_get_last_real_mc(void)
{
	return 0;
}
static inline void kasumi_apply_profile(unsigned int profile)
{
}
#endif

#endif /* __KASUMI_H__ */
