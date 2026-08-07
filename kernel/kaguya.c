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
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/kaguya.h>

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

/* Candidate resetprop locations, in priority order.  Magisk installs
 * /system/bin/resetprop on most setups; KernelSU-Next keeps its tools
 * under /data/adb/ksu/bin.  Additional locations can be appended here.
 */static const char *const kaguya_resetprop_candidates[] = {
	"/system/bin/resetprop",
	"/data/adb/magisk/resetprop",
	"/data/adb/ksu/bin/resetprop",
	NULL,
};

/* Resolved resetprop path once found ("" = unresolved).  Cached so the
 * enforcement tick only probes once after the first success; a missing
 * binary keeps re-probing so a late-installed resetprop (e.g. Magisk
 * flashed after boot) is picked up automatically.
 */
static char kaguya_resetprop_path[64];
static bool kaguya_resetprop_warned;

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

/*
 * Resolve a usable resetprop binary.  Only -ENOENT/-ENOTDIR count as
 * "missing": if the probe is denied by SELinux or fails for another
 * reason the binary may still exist, so we optimistically return the
 * path and let the exec attempt be the definitive test (the builder
 * ships matching ksu_allow rules for these locations).
 */
static const char *kaguya_resolve_resetprop(void)
{
	const char *const *p;

	if (kaguya_resetprop_path[0])
		return kaguya_resetprop_path;

	for (p = kaguya_resetprop_candidates; *p; p++) {
		struct file *f = filp_open(*p, O_RDONLY, 0);

		if (IS_ERR(f)) {
			if (PTR_ERR(f) == -ENOENT || PTR_ERR(f) == -ENOTDIR)
				continue;
			/* Not missing (e.g. SELinux denied the open) — cache it so
			 * the tick stops re-probing, and let the exec attempt be
			 * the definitive test.
			 */
			strscpy(kaguya_resetprop_path, *p,
				sizeof(kaguya_resetprop_path));
			return kaguya_resetprop_path;
		}
		filp_close(f, NULL);
		strscpy(kaguya_resetprop_path, *p,
			sizeof(kaguya_resetprop_path));
		pr_info("kaguya: using resetprop at %s\n",
			kaguya_resetprop_path);
		return kaguya_resetprop_path;
	}

	if (!kaguya_resetprop_warned) {
		pr_warn("kaguya: no resetprop found (%s, %s, %s) - safeprop "
			"enforcement disabled; install Magisk or KernelSU-Next "
			"tools\n",
			kaguya_resetprop_candidates[0],
			kaguya_resetprop_candidates[1],
			kaguya_resetprop_candidates[2]);
		kaguya_resetprop_warned = true;
	}
	return NULL;
}

static void kaguya_enforce_work_fn(struct work_struct *work)
{
	if (!kaguya_enabled)
		goto reschedule;

	mutex_lock(&kaguya_lock);

	if (kaguya_prop_count > 0) {
		const char *rp = kaguya_resolve_resetprop();

		if (rp) {
			struct kaguya_batch_cmd cmd;
			int pos = 0, i;

			/* Build: resetprop 'n1' 'v1' && resetprop 'n2' 'v2'.
			 * Note: if the chain ever truncates against the 4K buffer,
			 * sh parses the whole line first and no prop gets set.
			 */
			for (i = 0; i < kaguya_prop_count; i++) {
				int n;

				if (i == 0)
					n = snprintf(cmd.cmd_buf + pos,
						     sizeof(cmd.cmd_buf) - pos,
						     "%s '%s' '%s'", rp,
						     kaguya_props[i].name,
						     kaguya_props[i].value);
				else
					n = snprintf(cmd.cmd_buf + pos,
						     sizeof(cmd.cmd_buf) - pos,
						     " && %s '%s' '%s'", rp,
						     kaguya_props[i].name,
						     kaguya_props[i].value);

				if (n < 0 ||
				    n >= (int)sizeof(cmd.cmd_buf) - pos) {
					pr_debug("kaguya: cmd buf full at prop %d\n",
						 i);
					break;
				}
				pos += n;
			}

			cmd.argv[0] = "/system/bin/sh";
			cmd.argv[1] = "-c";
			cmd.argv[2] = cmd.cmd_buf;
			cmd.argv[3] = NULL;

			call_usermodehelper(cmd.argv[0], cmd.argv,
					    kaguya_envp, UMH_NO_WAIT);
		}
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

/* ------------------------------------------------------------------ */
/* Boot argument spoofing (/proc/cmdline, /proc/bootconfig)             */
/* ------------------------------------------------------------------ */
/*
 * kaguya enforces the ro.boot.* property values above via resetprop,
 * but /proc/cmdline and /proc/bootconfig are generated from the raw
 * boot text, NOT from the property store.  A detection that reads
 * either file would see the real values and, worse, spot a mismatch
 * between the (green) property and the (orange) raw text.  These
 * tables rewrite the raw text to the values kaguya enforces so both
 * layers always agree.
 *
 * Every replacement keeps the string the same length or shrinks it
 * except a few small ones (e.g. "verifiedbootstate=red"->green grows
 * by 3), so the +KAGUYA_SPOOF_SLACK headroom in
 * kaguya_spoof_boot_args() is always sufficient.
 */
#ifdef CONFIG_KAGUYA_CMDLINE_SPOOF
#define KAGUYA_SPOOF_SLACK		256

struct kaguya_spoof_pair {
	const char *from;
	const char *to;
};

/* /proc/cmdline format: key=value (androidboot.* prefix matched via strstr) */
static const struct kaguya_spoof_pair kaguya_spoof_cmdline[] = {
	{ "verifiedbootstate=orange",	"verifiedbootstate=green" },
	{ "verifiedbootstate=red",	"verifiedbootstate=green" },
	{ "veritymode=logging",		"veritymode=enforcing" },
	{ "veritymode=disabled",	"veritymode=enforcing" },
	{ "flash.locked=0",		"flash.locked=1" },
	{ "vbmeta.device_state=unlocked", "vbmeta.device_state=locked" },
	{ "selinux=permissive",		"selinux=enforcing" },
	{ "warranty_bit=1",		"warranty_bit=0" },
};

/* /proc/bootconfig format: key = "value" */
static const struct kaguya_spoof_pair kaguya_spoof_bootconfig[] = {
	{ "verifiedbootstate = \"orange\"", "verifiedbootstate = \"green\"" },
	{ "device_state = \"unlocked\"",	"device_state = \"locked\"" },
	{ "flash.locked = \"0\"",		"flash.locked = \"1\"" },
	{ "veritymode = \"logging\"",		"veritymode = \"enforcing\"" },
	{ "veritymode = \"disabled\"",	"veritymode = \"enforcing\"" },
	{ "selinux = \"permissive\"",		"selinux = \"enforcing\"" },
};

/* In-place memmove-based replace; handles growth and shrink. */
static void kaguya_safe_replace(char *str, const char *old_str,
				const char *new_str)
{
	char *pos;
	size_t old_len = strlen(old_str);
	size_t new_len = strlen(new_str);

	while ((pos = strstr(str, old_str))) {
		size_t tail_len = strlen(pos + old_len);

		memmove(pos + new_len, pos + old_len, tail_len + 1);
		memcpy(pos, new_str, new_len);
	}
}

char *kaguya_spoof_boot_args(const char *str, bool bootconfig)
{
	const struct kaguya_spoof_pair *pairs;
	size_t npairs, i;
	const char *comm = current->comm;
	bool is_init, is_recovery;
	char *buf;

	/* init-family processes keep the real veritymode so the boot chain
	 * sees the actual verity configuration; recovery tools need the
	 * real state to function, so they get no spoof at all.
	 */
	is_init = (current->pid == 1 || strstr(comm, "init") ||
		   strstr(comm, "ueventd") || strstr(comm, "vold"));
	is_recovery = (strstr(comm, "recovery") || strstr(comm, "twrp") ||
			strstr(comm, "orangefox") || strstr(comm, "pitchblack") ||
			strstr(comm, "shrp"));

	if (bootconfig) {
		pairs = kaguya_spoof_bootconfig;
		npairs = ARRAY_SIZE(kaguya_spoof_bootconfig);
	} else {
		pairs = kaguya_spoof_cmdline;
		npairs = ARRAY_SIZE(kaguya_spoof_cmdline);
	}

	buf = kmalloc(strlen(str) + KAGUYA_SPOOF_SLACK, GFP_KERNEL);
	if (!buf)
		return NULL;
	strscpy(buf, str, strlen(str) + 1);

	if (is_recovery)
		return buf;

	for (i = 0; i < npairs; i++) {
		if (is_init && strstr(pairs[i].from, "veritymode"))
			continue;
		kaguya_safe_replace(buf, pairs[i].from, pairs[i].to);
	}
	return buf;
}
#endif /* CONFIG_KAGUYA_CMDLINE_SPOOF */

module_init(kaguya_init);
module_exit(kaguya_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Kaguya — GrayRavens Kernel Safeprop Enforcer");
MODULE_AUTHOR("GrayRavens");
