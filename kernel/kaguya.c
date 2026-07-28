// SPDX-License-Identifier: GPL-2.0
/*
 * kaguya — GrayRavens Kernel-Level Safeprop Enforcer
 *
 * Enforces Android system property overrides from the kernel via a
 * timer-based workqueue that periodically calls resetprop(1) through
 * call_usermodehelper().  This catches any userspace attempts to
 * overwrite the spoofed values (e.g. banking app root detections).
 *
 * The default property list includes the standard safeprop spoofs
 * for hiding bootloader unlock and enforcing device integrity.
 *
 * Sysfs: /sys/kernel/kaguya/
 *   enabled        (RW) — 0/1 master switch (default 1)
 *   interval_ms    (RW) — enforcement interval in ms (default 10000)
 *   props          (RO) — dump current enforced properties
 *   props_add      (WO) — add a property "name=value"
 *   props_del      (WO) — remove a property by name
 *   props_clear    (WO) — remove all custom properties
 */
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <linux/umh.h>

/* ------------------------------------------------------------------ */
/* Property store                                                      */
/* ------------------------------------------------------------------ */
#define KAGUYA_PROPS_MAX		32
#define KAGUYA_PROP_NAME_LEN		128
#define KAGUYA_PROP_VAL_LEN		128

struct kaguya_prop {
	char name[KAGUYA_PROP_NAME_LEN];
	char value[KAGUYA_PROP_VAL_LEN];
};

/* Default properties — standard safeprop spoofs */
static const struct kaguya_prop kaguya_default_props[] = {
	{ "ro.crypto.state",			"encrypted" },
	{ "ro.boot.verifiedbootstate",		"green" },
	{ "ro.boot.veritymode",			"enforcing" },
	{ "vendor.boot.vbmeta.device_state",	"locked" },
	{ "ro.secureboot.lockstate",		"locked" },
	{ "ro.boot.flash.locked",		"1" },
	{ "ro.boot.vbmeta.device_state",	"locked" },
	{ "ro.boot.selinux",			"enforcing" },
	{ "sys.oem_unlock_allowed",		"0" },
	{ "ro.boot.veritymode.managed",		"yes" },
	{ "ro.boot.warranty_bit",		"0" },
	{ "ro.vendor.boot.warranty_bit",	"0" },
	{ "ro.vendor.warranty_bit",		"0" },
	{ "ro.warranty_bit",			"0" },
};

#define KAGUYA_DEFAULT_COUNT \
	(sizeof(kaguya_default_props) / sizeof(kaguya_default_props[0]))

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */
static DEFINE_MUTEX(kaguya_lock);
static bool kaguya_enabled = true;
static unsigned int kaguya_interval_ms = 5000;
static struct delayed_work kaguya_work;

/* Dynamic property array — starts with defaults, can be extended */
static struct kaguya_prop kaguya_props[KAGUYA_PROPS_MAX];
static int kaguya_prop_count;

/* Environment for call_usermodehelper */
static char *kaguya_envp[] = {
	"HOME=/",
	"PATH=/sbin:/system/bin:/product/bin",
	NULL,
};

/* ------------------------------------------------------------------ */
/* Enforcement tick — re-applies all properties via resetprop           */
/* ------------------------------------------------------------------ */

/*
 * Max length of the batch shell command.  Each property adds
 * roughly 16 + len(name) + len(value) bytes for the resetprop
 * invocation.  4K covers ~32 medium-length properties with room
 * to spare.
 */
#define KAGUYA_CMD_BUF_SIZE 4096

/* Build argv for: sh -c "resetprop 'name' 'value' && resetprop ..." */
struct kaguya_batch_cmd {
	char *argv[4];
	char cmd_buf[KAGUYA_CMD_BUF_SIZE];
};

static void kaguya_enforce_work_fn(struct work_struct *work)
{
	int i;

	if (!kaguya_enabled)
		goto reschedule;

	mutex_lock(&kaguya_lock);

	if (kaguya_prop_count > 0) {
		struct kaguya_batch_cmd cmd;
		int pos = 0;

		/* Build a single shell command: resetprop 'n1' 'v1' && ... */
		pos += snprintf(cmd.cmd_buf + pos,
				sizeof(cmd.cmd_buf) - pos,
				"/system/bin/resetprop");

		for (i = 0; i < kaguya_prop_count &&
		     pos < (int)sizeof(cmd.cmd_buf) - 1; i++) {
			int rem = (int)sizeof(cmd.cmd_buf) - pos;
			int n = snprintf(cmd.cmd_buf + pos, rem,
					 " '%s' '%s' && /system/bin/resetprop",
					 kaguya_props[i].name,
					 kaguya_props[i].value);

			if (n < 0 || n >= rem) {
				pr_debug("kaguya: cmd buf full at prop %d\n", i);
				break;
			}
			pos += n;
		}

		/* Strip the trailing " && /system/bin/resetprop" (25 chars) */
#define KAGUYA_TRAILING_LEN (int)(sizeof(" && /system/bin/resetprop") - 1)
		if (pos >= KAGUYA_TRAILING_LEN) {
			pos -= KAGUYA_TRAILING_LEN;
			cmd.cmd_buf[pos] = '\0';
		}

		cmd.argv[0] = "/system/bin/sh";
		cmd.argv[1] = "-c";
		cmd.argv[2] = cmd.cmd_buf;
		cmd.argv[3] = NULL;

		call_usermodehelper(cmd.argv[0], cmd.argv,
				    kaguya_envp, UMH_NO_WAIT);
	}

	mutex_unlock(&kaguya_lock);

reschedule:
	if (kaguya_interval_ms > 0)
		schedule_delayed_work(&kaguya_work,
				      msecs_to_jiffies(kaguya_interval_ms));
}

/* ------------------------------------------------------------------ */
/* Sysfs                                                               */
/* ------------------------------------------------------------------ */
static struct kobject *kaguya_kobj;

static ssize_t enabled_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", kaguya_enabled);
}

static ssize_t enabled_store(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;
	kaguya_enabled = !!val;
	return count;
}
static struct kobj_attribute enabled_attr =
	__ATTR_RW(enabled);

static ssize_t interval_ms_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", kaguya_interval_ms);
}

static ssize_t interval_ms_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;
	if (val < 1000 || val > 60000)
		return -ERANGE;
	kaguya_interval_ms = val;
	return count;
}
static struct kobj_attribute interval_ms_attr =
	__ATTR_RW(interval_ms);

static ssize_t props_show(struct kobject *kobj,
			  struct kobj_attribute *attr, char *buf)
{
	int i, pos = 0;

	mutex_lock(&kaguya_lock);
	for (i = 0; i < kaguya_prop_count; i++)
		pos += sysfs_emit_at(buf, pos, "%s=%s\n",
				     kaguya_props[i].name,
				     kaguya_props[i].value);
	mutex_unlock(&kaguya_lock);
	return pos;
}
static struct kobj_attribute props_attr =
	__ATTR_RO(props);

static ssize_t props_add_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t count)
{
	char name[KAGUYA_PROP_NAME_LEN];
	char value[KAGUYA_PROP_VAL_LEN];
	const char *eq;
	unsigned int nlen, vlen;
	int i;

	/* Strip trailing newline */
	while (count > 0 && (buf[count - 1] == '\n' || buf[count - 1] == ' '))
		count--;

	eq = memchr(buf, '=', count);
	if (!eq || eq == buf)
		return -EINVAL;

	nlen = eq - buf;
	vlen = count - nlen - 1;

	if (nlen == 0 || nlen >= KAGUYA_PROP_NAME_LEN)
		return -EINVAL;
	if (vlen == 0 || vlen >= KAGUYA_PROP_VAL_LEN)
		return -EINVAL;

	memcpy(name, buf, nlen);
	name[nlen] = '\0';
	memcpy(value, eq + 1, vlen);
	value[vlen] = '\0';

	mutex_lock(&kaguya_lock);

	/* Update existing if present */
	for (i = 0; i < kaguya_prop_count; i++) {
		if (strcmp(kaguya_props[i].name, name) == 0) {
			strscpy(kaguya_props[i].value, value,
				sizeof(kaguya_props[i].value));
			mutex_unlock(&kaguya_lock);
			return count;
		}
	}

	/* Add new */
	if (kaguya_prop_count >= KAGUYA_PROPS_MAX) {
		mutex_unlock(&kaguya_lock);
		return -ENOSPC;
	}

	strscpy(kaguya_props[kaguya_prop_count].name, name,
		sizeof(kaguya_props[kaguya_prop_count].name));
	strscpy(kaguya_props[kaguya_prop_count].value, value,
		sizeof(kaguya_props[kaguya_prop_count].value));
	kaguya_prop_count++;

	mutex_unlock(&kaguya_lock);
	return count;
}
static struct kobj_attribute props_add_attr =
	__ATTR_WO(props_add);

static ssize_t props_del_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t count)
{
	char name[KAGUYA_PROP_NAME_LEN];
	int i;

	while (count > 0 && (buf[count - 1] == '\n' || buf[count - 1] == ' '))
		count--;

	if (count == 0 || count >= KAGUYA_PROP_NAME_LEN)
		return -EINVAL;

	memcpy(name, buf, count);
	name[count] = '\0';

	mutex_lock(&kaguya_lock);
	for (i = 0; i < kaguya_prop_count; i++) {
		if (strcmp(kaguya_props[i].name, name) == 0) {
			/* Shift down */
			if (i < kaguya_prop_count - 1)
				memmove(&kaguya_props[i],
					&kaguya_props[i + 1],
					(kaguya_prop_count - i - 1) *
					sizeof(kaguya_props[i]));
			kaguya_prop_count--;
			mutex_unlock(&kaguya_lock);
			return count;
		}
	}
	mutex_unlock(&kaguya_lock);
	return -ENOENT;
}
static struct kobj_attribute props_del_attr =
	__ATTR_WO(props_del);

static ssize_t props_clear_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	/* Reset to defaults */
	mutex_lock(&kaguya_lock);
	kaguya_prop_count = KAGUYA_DEFAULT_COUNT;
	memcpy(kaguya_props, kaguya_default_props,
	       sizeof(kaguya_default_props));
	mutex_unlock(&kaguya_lock);
	return count;
}
static struct kobj_attribute props_clear_attr =
	__ATTR_WO(props_clear);


static ssize_t enforce_now_store(struct kobject *kobj,
                                 struct kobj_attribute *attr,
                                 const char *buf, size_t count)
{
	if (!kaguya_enabled)
		return count;

	/* Trigger immediate enforcement */
	mod_delayed_work(system_wq, &kaguya_work, 0);
	return count;
}
static struct kobj_attribute enforce_now_attr =
	__ATTR_WO(enforce_now);

static ssize_t verify_show(struct kobject *kobj,
			   struct kobj_attribute *attr, char *buf)
{
	/* Note: this only shows what we WANT to enforce, not actual values.
	 * For actual verification use: getprop <name> from userspace.
	 * The kernel cannot read Android properties directly.
	 */
	return sysfs_emit(buf, "read-only. Use: getprop <name> from adb shell\n");
}
static struct kobj_attribute verify_attr =
	__ATTR_RO(verify);

static struct attribute *kaguya_attrs[] = {
	&enabled_attr.attr,
	&interval_ms_attr.attr,
	&props_attr.attr,
	&props_add_attr.attr,
	&props_del_attr.attr,
	&props_clear_attr.attr,
	&enforce_now_attr.attr,
	&verify_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(kaguya);

/* ------------------------------------------------------------------ */
/* Init / Exit                                                         */
/* ------------------------------------------------------------------ */
static int __init kaguya_init(void)
{
	int ret;

	/* Seed with defaults */
	BUILD_BUG_ON(KAGUYA_DEFAULT_COUNT > KAGUYA_PROPS_MAX);
	memcpy(kaguya_props, kaguya_default_props,
	       sizeof(kaguya_default_props));
	kaguya_prop_count = KAGUYA_DEFAULT_COUNT;

	kaguya_kobj = kobject_create_and_add("kaguya", kernel_kobj);
	if (!kaguya_kobj)
		return -ENOMEM;

	ret = sysfs_create_groups(kaguya_kobj, kaguya_groups);
	if (ret) {
		kobject_put(kaguya_kobj);
		return ret;
	}

	INIT_DELAYED_WORK(&kaguya_work, kaguya_enforce_work_fn);
	/*
	 * Use a longer initial delay (30s) so userspace has time to
	 * fully boot before call_usermodehelper fires.  Subsequent
	 * re-schedules use the configured kaguya_interval_ms.
	 */
	schedule_delayed_work(&kaguya_work,
			      msecs_to_jiffies(max(kaguya_interval_ms, 30000U)));

	pr_info("kaguya: loaded (%d default props, interval %u ms)\n",
		kaguya_prop_count, kaguya_interval_ms);
	return 0;
}

static void __exit kaguya_exit(void)
{
	/* Prevent work from re-queuing itself before we cancel it */
	kaguya_enabled = false;
	kaguya_interval_ms = 0;
	cancel_delayed_work_sync(&kaguya_work);

	sysfs_remove_groups(kaguya_kobj, kaguya_groups);
	kobject_put(kaguya_kobj);
	pr_info("kaguya: unloaded\n");
}

module_init(kaguya_init);
module_exit(kaguya_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Kaguya — GrayRavens Kernel Safeprop Enforcer");
MODULE_AUTHOR("GrayRavens");
