// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/vindicator/herald.c
 * Herald — SELinux-safe property relay from kernel to userspace.
 *
 * Kernel modules queue property changes via herald_set_prop();
 * the values are exposed under /sys/kernel/herald/ for a userspace
 * daemon or init.rc script to pick up and apply with setprop(1).
 *
 * Improvements over the original Reine:
 *   - No call_usermodehelper() — SELinux safe.
 *   - Sysfs-based interface: /sys/kernel/herald/queue/ shows
 *     pending properties, /sys/kernel/herald/commit drains them.
 *   - Queue depth bounded (256 entries) prevents memory exhaustion.
 *   - Each entry is pairs of files (name + value) for easy parsing.
 *   - Supports deletion (herald_del_prop) for cancelling pending props.
 *
 * Userspace usage (in init.rc or daemon):
 *   while true; do
 *     for d in /sys/kernel/herald/queue/*/; do
 *       name=$(cat $d/name)
 *       val=$(cat $d/value)
 *       setprop $name $val
 *       echo 1 > $d/commit
 *     done
 *     sleep 2
 *   done
 *
 * Author: GrayRavens
 */
#define pr_fmt(fmt) "herald: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/atomic.h>
#include <linux/vindicator/herald.h>

/* ------------------------------------------------------------------ */
/* Constants                                                          */
/* ------------------------------------------------------------------ */
#define HERALD_QUEUE_MAX      256
#define HERALD_NAME_MAX       64
#define HERALD_VAL_MAX        92

/* ------------------------------------------------------------------ */
/* Per-entry structure                                                */
/* ------------------------------------------------------------------ */
struct herald_entry {
	char            name[HERALD_NAME_MAX];
	char            val[HERALD_VAL_MAX];
	struct kobject  kobj;
	struct list_head list;
	bool            committed;
};

static struct kobject *herald_kobj;
static struct kobject *herald_queue_kobj;

static LIST_HEAD(herald_pending_list);
static DEFINE_SPINLOCK(herald_lock);   /* protects pending_list */
static atomic_t herald_queue_depth = ATOMIC_INIT(0);

/* ------------------------------------------------------------------ */
/* Sysfs show/store for a single entry                                */
/* ------------------------------------------------------------------ */
static ssize_t herald_entry_name_show(struct kobject *kobj,
				      struct kobj_attribute *attr, char *buf)
{
	struct herald_entry *e = container_of(kobj, struct herald_entry, kobj);
	return sysfs_emit(buf, "%s\n", e->name);
}
static struct kobj_attribute herald_attr_name = __ATTR_RO(name);

static ssize_t herald_entry_val_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	struct herald_entry *e = container_of(kobj, struct herald_entry, kobj);
	return sysfs_emit(buf, "%s\n", e->val);
}
static struct kobj_attribute herald_attr_val = __ATTR_RO(value);

static ssize_t herald_entry_commit_store(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 const char *buf, size_t count)
{
	struct herald_entry *e = container_of(kobj, struct herald_entry, kobj);
	unsigned long flags;

	if (!e->committed) {
		spin_lock_irqsave(&herald_lock, flags);
		list_del(&e->list);
		e->committed = true;
		atomic_dec(&herald_queue_depth);
		spin_unlock_irqrestore(&herald_lock, flags);

		pr_info("committed: %s = %s\n", e->name, e->val);
		kobject_put(&e->kobj); /* free via release */
	}
	return count;
}
static struct kobj_attribute herald_attr_commit = __ATTR_WO(commit);

static struct attribute *herald_entry_attrs[] = {
	&herald_attr_name.attr,
	&herald_attr_val.attr,
	&herald_attr_commit.attr,
	NULL,
};
ATTRIBUTE_GROUPS(herald_entry);

static void herald_entry_release(struct kobject *kobj)
{
	struct herald_entry *e = container_of(kobj, struct herald_entry, kobj);
	kfree(e);
}

static struct kobj_type herald_entry_ktype = {
	.sysfs_ops = &kobj_sysfs_ops,
	.release   = herald_entry_release,
	.default_groups = herald_entry_groups,
};

/* ------------------------------------------------------------------ */
/* Queue depth sysfs (read-only)                                       */
/* ------------------------------------------------------------------ */
static ssize_t herald_depth_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", atomic_read(&herald_queue_depth));
}
static struct kobj_attribute herald_attr_depth = __ATTR_RO(queue_depth);

static struct attribute *herald_attrs[] = {
	&herald_attr_depth.attr,
	NULL,
};
ATTRIBUTE_GROUPS(herald);

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */
int herald_set_prop(const char *name, const char *val)
{
	struct herald_entry *e;
	unsigned long flags;
	int ret;

	if (!name || !val || !*name || !*val)
		return -EINVAL;

	if (strlen(name) >= HERALD_NAME_MAX ||
	    strlen(val) >= HERALD_VAL_MAX)
		return -EINVAL;

	if (atomic_read(&herald_queue_depth) >= HERALD_QUEUE_MAX)
		return -ENOMEM;

	/* Remove existing entry with same name before adding */
	herald_del_prop(name);

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (!e)
		return -ENOMEM;

	strscpy(e->name, name, sizeof(e->name));
	strscpy(e->val, val, sizeof(e->val));
	e->committed = false;

	ret = kobject_init_and_add(&e->kobj, &herald_entry_ktype,
				   herald_queue_kobj, "%s", name);
	if (ret) {
		kobject_put(&e->kobj);
		return ret;
	}

	spin_lock_irqsave(&herald_lock, flags);
	list_add_tail(&e->list, &herald_pending_list);
	atomic_inc(&herald_queue_depth);
	spin_unlock_irqrestore(&herald_lock, flags);

	pr_debug("queued: %s = %s (depth %d)\n",
		 name, val, atomic_read(&herald_queue_depth));
	return 0;
}
EXPORT_SYMBOL_GPL(herald_set_prop);

void herald_del_prop(const char *name)
{
	struct herald_entry *e;
	unsigned long flags;

	spin_lock_irqsave(&herald_lock, flags);
	list_for_each_entry(e, &herald_pending_list, list) {
		if (strcmp(e->name, name) == 0 && !e->committed) {
			list_del(&e->list);
			atomic_dec(&herald_queue_depth);
			spin_unlock_irqrestore(&herald_lock, flags);

			pr_info("deleted pending: %s\n", name);
			kobject_put(&e->kobj);
			return;
		}
	}
	spin_unlock_irqrestore(&herald_lock, flags);
}
EXPORT_SYMBOL_GPL(herald_del_prop);

/* ------------------------------------------------------------------ */
/* Module init / exit                                                  */
/* ------------------------------------------------------------------ */
static int __init herald_init(void)
{
	int ret;

	herald_kobj = kobject_create_and_add("herald", kernel_kobj);
	if (!herald_kobj)
		return -ENOMEM;

	ret = sysfs_create_groups(herald_kobj, herald_groups);
	if (ret)
		goto err_kobj;

	herald_queue_kobj = kobject_create_and_add("queue", herald_kobj);
	if (!herald_queue_kobj) {
		ret = -ENOMEM;
		goto err_groups;
	}

	pr_info("loaded (max queue depth %d)\n", HERALD_QUEUE_MAX);
	return 0;

err_groups:
	sysfs_remove_groups(herald_kobj, herald_groups);
err_kobj:
	kobject_put(herald_kobj);
	return ret;
}

static void __exit herald_exit(void)
{
	struct herald_entry *e, *tmp;
	unsigned long flags;

	/* Drain pending queue */
	spin_lock_irqsave(&herald_lock, flags);
	list_for_each_entry_safe(e, tmp, &herald_pending_list, list) {
		list_del(&e->list);
		atomic_dec(&herald_queue_depth);
		spin_unlock_irqrestore(&herald_lock, flags);

		kobject_put(&e->kobj);

		spin_lock_irqsave(&herald_lock, flags);
	}
	spin_unlock_irqrestore(&herald_lock, flags);

	kobject_put(herald_queue_kobj);
	sysfs_remove_groups(herald_kobj, herald_groups);
	kobject_put(herald_kobj);

	pr_info("unloaded\n");
}

module_init(herald_init);
module_exit(herald_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GrayRavens");
MODULE_DESCRIPTION("Herald — SELinux-safe kernel-to-userspace property relay");
