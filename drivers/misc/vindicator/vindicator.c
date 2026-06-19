// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/vindicator/vindicator.c
 * Vindicator — sysfs / procfs enforcement framework.
 *
 * Kernel modules register enforce-callbacks that Vindicator's
 * hrtimer-driven watchdog re-applies periodically, fighting
 * vendor init.rc / HALs that revert kernel tunables.
 *
 * Design highlights over the original Raco:
 *   - Per-target hrtimer (not a single poll thread with msleep).
 *   - RCU-protected target list (lock-free read in timer callback).
 *   - Adaptive backoff: starts at 50 ms, doubles to 5 s max.
 *   - Per-target max-retries (default 120) + auto-deregister.
 *   - debugfs statistics per target.
 *
 * Author: GrayRavens
 */
#define pr_fmt(fmt) "vindicator: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/hrtimer.h>
#include <linux/rcupdate.h>
#include <linux/debugfs.h>
#include <linux/atomic.h>
#include <linux/vindicator.h>

/* ------------------------------------------------------------------ */
/* Tunables                                                           */
/* ------------------------------------------------------------------ */
static unsigned int vind_interval_min_ms = 50;    /* highest frequency */
module_param_named(interval_min_ms, vind_interval_min_ms, uint, 0644);
MODULE_PARM_DESC(interval_min_ms, "Minimum enforcement interval (ms)");

static unsigned int vind_interval_max_ms = 5000;  /* after full backoff */
module_param_named(interval_max_ms, vind_interval_max_ms, uint, 0644);
MODULE_PARM_DESC(interval_max_ms, "Maximum enforcement interval (ms) after backoff");

static unsigned int vind_max_retries = 120;       /* ~35 min @ max interval */
module_param_named(max_retries, vind_max_retries, uint, 0644);
MODULE_PARM_DESC(max_retries, "Enforcements before auto-deregister (0 = infinite)");

static unsigned int vind_enabled = 1;
module_param_named(enabled, vind_enabled, uint, 0644);
MODULE_PARM_DESC(enabled, "Master enable (0 = pause all enforcement)");

/* ------------------------------------------------------------------ */
/* Target structure                                                   */
/* ------------------------------------------------------------------ */
struct vind_target {
	const struct vindicator_enforce_ops *ops;
	struct hrtimer              timer;
	atomic_t                    enforc_cnt;     /* total enforcements */
	atomic_t                    skip_cnt;       /* skipped (disabled) */
	unsigned int                interval_ms;    /* current interval */
	unsigned int                retries_done;
	struct list_head            list;           /* protected by vind_lock + RCU */
	struct dentry              *dbg_dentry;     /* debugfs directory */
};

static LIST_HEAD(vind_targets);
static DEFINE_MUTEX(vind_lock);                  /* serialises register / unregister */
static struct dentry *vind_dbg_dir;

/* ------------------------------------------------------------------ */
/* Enforcement timer callback                                          */
/* ------------------------------------------------------------------ */
static enum hrtimer_restart vind_timer_cb(struct hrtimer *timer)
{
	struct vind_target *tgt = container_of(timer, struct vind_target, timer);

	if (!READ_ONCE(vind_enabled)) {
		atomic_inc(&tgt->skip_cnt);
		goto resched;
	}

	/* Fire the enforce callback */
	tgt->ops->enforce(tgt->ops->data);
	atomic_inc(&tgt->enforc_cnt);

	/* Retry bookkeeping */
	if (READ_ONCE(vind_max_retries) &&
	    ++tgt->retries_done >= READ_ONCE(vind_max_retries)) {
		pr_info("'%s' reached max retries (%u), deregistering\n",
			tgt->ops->name, vind_max_retries);
		return HRTIMER_NORESTART; /* timer stops itself */
	}

	/* Adaptive backoff: double interval until max */
	tgt->interval_ms = min(tgt->interval_ms * 2, READ_ONCE(vind_interval_max_ms));

resched:
	hrtimer_forward_now(timer,
		ktime_set(0, tgt->interval_ms * 1000000UL));
	return HRTIMER_RESTART;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */
int vindicator_register(const struct vindicator_enforce_ops *ops)
{
	struct vind_target *tgt;
	struct dentry *dbg;

	if (!ops || !ops->enforce || !ops->name)
		return -EINVAL;

	tgt = kzalloc(sizeof(*tgt), GFP_KERNEL);
	if (!tgt)
		return -ENOMEM;

	tgt->ops = ops;
	tgt->interval_ms = READ_ONCE(vind_interval_min_ms);
	atomic_set(&tgt->enforc_cnt, 0);
	atomic_set(&tgt->skip_cnt, 0);

	/* debugfs entry */
	if (vind_dbg_dir) {
		dbg = debugfs_create_dir(ops->name, vind_dbg_dir);
		debugfs_create_atomic("enforcements", 0444, dbg, &tgt->enforc_cnt);
		debugfs_create_atomic("skipped",      0444, dbg, &tgt->skip_cnt);
		debugfs_create_u32("interval_ms",     0444, dbg, &tgt->interval_ms);
		tgt->dbg_dentry = dbg;
	}

	hrtimer_init(&tgt->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	tgt->timer.function = vind_timer_cb;

	mutex_lock(&vind_lock);
	list_add_tail_rcu(&tgt->list, &vind_targets);
	mutex_unlock(&vind_lock);

	hrtimer_start(&tgt->timer,
		ktime_set(0, tgt->interval_ms * 1000000UL),
		HRTIMER_MODE_REL);

	pr_info("registered '%s' (interval %u ms, max_retries %u)\n",
		ops->name, tgt->interval_ms, vind_max_retries);
	return 0;
}
EXPORT_SYMBOL_GPL(vindicator_register);

void vindicator_unregister(const struct vindicator_enforce_ops *ops)
{
	struct vind_target *tgt;

	if (!ops)
		return;

	mutex_lock(&vind_lock);
	list_for_each_entry_rcu(tgt, &vind_targets, list,
				lockdep_is_held(&vind_lock)) {
		if (tgt->ops == ops) {
			list_del_rcu(&tgt->list);
			mutex_unlock(&vind_lock);

			hrtimer_cancel(&tgt->timer);
			if (tgt->dbg_dentry)
				debugfs_remove(tgt->dbg_dentry);
			synchronize_rcu();
			pr_info("unregistered '%s'\n", ops->name);
			kfree(tgt);
			return;
		}
	}
	mutex_unlock(&vind_lock);
	pr_warn("unregister called but '%s' not found\n",
		ops->name ? ops->name : "(null)");
}
EXPORT_SYMBOL_GPL(vindicator_unregister);

/* ------------------------------------------------------------------ */
/* Module init / exit                                                  */
/* ------------------------------------------------------------------ */
static int __init vindicator_init(void)
{
	vind_dbg_dir = debugfs_create_dir("vindicator", NULL);
	pr_info("loaded (min=%ums max=%ums retries=%u)\n",
		vind_interval_min_ms, vind_interval_max_ms, vind_max_retries);
	return 0;
}

static void __exit vindicator_exit(void)
{
	struct vind_target *tgt, *tmp;

	mutex_lock(&vind_lock);
	list_for_each_entry_safe(tgt, tmp, &vind_targets, list) {
		list_del_rcu(&tgt->list);
		hrtimer_cancel(&tgt->timer);
		debugfs_remove(tgt->dbg_dentry);
		pr_info("auto-deregistered '%s' on module exit\n", tgt->ops->name);
		kfree(tgt);
	}
	mutex_unlock(&vind_lock);

	debugfs_remove(vind_dbg_dir);
	pr_info("unloaded\n");
}

module_init(vindicator_init);
module_exit(vindicator_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GrayRavens");
MODULE_DESCRIPTION("Vindicator — sysfs enforcement framework");
