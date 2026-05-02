/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Public kernel API for the zenith cpufreq governor.
 *
 * Currently consists of a single setter that display drivers /
 * drm-panel bridges call to publish the active panel vblank period
 * to zenith's adaptive frame-budget floor.  Stub-out when zenith is
 * not built so callers can stay unconditional.
 */
#ifndef _LINUX_CPUFREQ_ZENITH_H
#define _LINUX_CPUFREQ_ZENITH_H

#include <linux/types.h>

#if IS_ENABLED(CONFIG_CPU_FREQ_GOV_ZENITH)
extern void zenith_set_drm_vblank_us(unsigned int us);
#else
static inline void zenith_set_drm_vblank_us(unsigned int us) { }
#endif

#endif /* _LINUX_CPUFREQ_ZENITH_H */
