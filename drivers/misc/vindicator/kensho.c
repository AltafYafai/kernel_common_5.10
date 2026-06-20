// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/vindicator/kensho.c
 * Kenshō — 監照: Kernel panic capture and forensic logging
 *
 * Kenshō captures kernel panics at the moment they occur and exposes
 * the full log via sysfs. It uses two kernel facilities:
 *
 *   1. kmsg_dumper — retrieves the entire kernel log buffer at panic
 *   2. panic notifier — captures the panic reason string
 *
 * After a panic (if the system hasn't fully power-cycled), the captured
 * log can be read from /sys/kernel/kensho/last_panic. This is invaluable
 * for developers debugging boot-time crashes that don't leave serial traces.
 *
 * For cross-reboot persistence, Kenshō can be extended to write to pstore
 * (v2).
 *
 * Author: GrayRavens
 * Co-authored-by: Kanagawa Yamada <albert.wesley.dion@gmail.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kmsg_dump.h>
#include <linux/panic_notifier.h>
#include <linux/notifier.h>

/*
 * Kernel 5.10 introduced include/linux/panic_notifier.h.
 * If this build complains about an undeclared panic_notifier_list,
 * uncomment the extern declaration below and remove the header above.
 */
/* extern struct atomic_notifier_head panic_notifier_list; */
#include <linux/sched.h>
#include <linux/sched/loadavg.h>
#include <linux/sched/stat.h>
#include <linux/utsname.h>
#include <linux/ktime.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timekeeping.h>
#include <linux/version.h>

#ifdef CONFIG_VINDICATOR_KENSHO_PERSIST
#include <linux/memblock.h>
#include <asm/cacheflush.h>
#endif

#define KENSHO_VERSION		"1.0"
#define KENSHO_BUF_SIZE		(128 * 1024)	/* 128 kB panic log buffer */
#define KENSHO_REASON_LEN	256

#ifdef CONFIG_VINDICATOR_KENSHO_PERSIST
#define KENSHO_PERSIST_SIZE	SZ_128K
#define KENSHO_PERSIST_MAGIC	0x4B454E53484FULL /* "KENSHO" */

/*
 * Header stored at the start of the reserved persistent memory region.
 * If magic matches on boot, the panic log survived the reboot.
 */
struct kensho_persist_header {
	u64	magic;
	u64	timestamp;
	u32	size;	/* bytes of log data following this header */
	u8	__pad[20];
} __packed;
#endif

/* ------------------------------------------------------------------ */
/* Data                                                               */
/* ------------------------------------------------------------------ */

/*
 * kensho_panic_buf  — pre-allocated buffer that holds the final panic log
 *                     (metadata header + kernel log buffer).
 * kensho_panic_len  — number of valid bytes in kensho_panic_buf.
 * kensho_crashed    — set to true once a panic has been captured.
 * kensho_reason     — the panic reason string passed to the notifier.
 */
static char *kensho_panic_buf;
static size_t kensho_panic_len;
static bool kensho_crashed;
static char kensho_reason[KENSHO_REASON_LEN];
static struct kobject *kensho_kobj;

/* kmsg dumper instance */
static struct kmsg_dumper kensho_dumper;

/* panic notifier block */
static struct notifier_block kensho_panic_nb;

#ifdef CONFIG_VINDICATOR_KENSHO_PERSIST
/* Persistent memory region — survives reboots */
static phys_addr_t kensho_persist_pa;
static void *kensho_persist_va;
#endif

/* ------------------------------------------------------------------ */
/* Helpers — metadata assembly                                        */
/* ------------------------------------------------------------------ */

/*
 * kensho_write_metadata — fill the header region of the panic buffer
 * with timestamp, kernel version, loadavg, and the panic reason.
 *
 * Called from atomic (panic) context — no sleeping, no GFP_KERNEL.
 */
static void kensho_write_metadata(void)
{
	struct timespec64 ts;
	unsigned long avg[3];
	int len;

	/* Timestamp — wall-clock time at panic */
	ktime_get_real_ts64(&ts);

	/* Load average snapshot */
	get_avenrun(avg, FIXED_1 / 100, 0);

	len = scnprintf(kensho_panic_buf, KENSHO_BUF_SIZE,
		"===== Kenshō Crash Capture v%s =====\n"
		"Kernel: %s %s\n"
		"Timestamp: %lld.%09ld UTC\n"
		"Uptime: %llu jiffies\n"
		"Loadavg: %lu.%02lu %lu.%02lu %lu.%02lu\n"
		"Reason:  %s\n"
		"====================================\n\n",
		KENSHO_VERSION,
		init_uts_ns.name.release,
		init_uts_ns.name.version,
		(long long)ts.tv_sec, ts.tv_nsec,
		(unsigned long long)jiffies,
		LOAD_INT(avg[0]), LOAD_FRAC(avg[0]),
		LOAD_INT(avg[1]), LOAD_FRAC(avg[1]),
		LOAD_INT(avg[2]), LOAD_FRAC(avg[2]),
		kensho_reason);

	if (len < 0)
		len = 0;
	else if (len >= KENSHO_BUF_SIZE)
		len = KENSHO_BUF_SIZE - 1;

	kensho_panic_len = (size_t)len;
	kensho_crashed = true;
}

/* ------------------------------------------------------------------ */
/* kmsg dumper callback                                               */
/* ------------------------------------------------------------------ */

/*
 * Called from panic() context — the kernel log buffer is still intact.
 * We drain it into the remaining space after our metadata header.
 */
static void kensho_dump_cb(struct kmsg_dumper *dumper,
			   enum kmsg_dump_reason reason)
{
	size_t len;
	size_t remaining;

	/* Only capture panics (and optionally oopses — check KMSG_DUMP_OOPS) */
	if (reason > KMSG_DUMP_PANIC)
		return;

	/* Write metadata header first */
	kensho_write_metadata();

	/* Fill the rest of the buffer with the kernel log */
	remaining = KENSHO_BUF_SIZE - kensho_panic_len;
	if (remaining < 64)
		return;	/* header alone filled it — unusual but possible */

	if (!kmsg_dump_get_buffer(dumper, true,
				  kensho_panic_buf + kensho_panic_len,
				  remaining - 1, &len))
		return;

	kensho_panic_len += len;
	kensho_panic_buf[kensho_panic_len] = '\0';
}

/* ------------------------------------------------------------------ */
/* Panic notifier                                                      */
/* ------------------------------------------------------------------ */

/*
 * Fires before kmsg_dump in the panic() sequence. We save the panic
 * reason string so kensho_write_metadata can include it.
 * Priority INT_MAX means we run first.
 */
static int kensho_panic_cb(struct notifier_block *nb,
			   unsigned long action, void *data)
{
	if (data)
		strscpy(kensho_reason, (const char *)data,
			sizeof(kensho_reason));
	return NOTIFY_DONE;
}

/* ------------------------------------------------------------------ */
/* Sysfs                                                               */
/* ------------------------------------------------------------------ */

/*
 * Use bin_attribute for last_panic so the entire 128 kB buffer is
 * accessible via read(), not truncated to PAGE_SIZE (4 kB).
 */
static ssize_t last_panic_read(struct file *filp, struct kobject *kobj,
			       struct bin_attribute *attr,
			       char *buf, loff_t off, size_t count)
{
	static const char no_crash[] = "No panic captured since boot.\n";

	if (!kensho_crashed) {
		if (off >= (loff_t)sizeof(no_crash))
			return 0;
		count = min_t(size_t, sizeof(no_crash) - (size_t)off, count);
		memcpy(buf, no_crash + off, count);
		return count;
	}

	if (off >= (loff_t)kensho_panic_len)
		return 0;

	count = min_t(size_t, count, kensho_panic_len - (size_t)off);
	memcpy(buf, kensho_panic_buf + off, count);
	return count;
}

static ssize_t crashed_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", kensho_crashed ? 1 : 0);
}

static ssize_t info_show(struct kobject *kobj,
			 struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
		"Kenshō v%s\n"
		"Status: %s\n"
		"Buffer: %u bytes\n",
		KENSHO_VERSION,
		kensho_crashed ? "CRASH CAPTURED" : "Standing by",
		(unsigned int)KENSHO_BUF_SIZE);
}

/* last_panic as a binary attribute — no PAGE_SIZE limit */
static BIN_ATTR(last_panic, 0444, last_panic_read, NULL);

static struct kobj_attribute kensho_crashed_attr =
	__ATTR_RO(crashed);
static struct kobj_attribute kensho_info_attr =
	__ATTR_RO(info);

/* bin_attrs must be in a separate list from regular attrs */
static struct attribute *kensho_attrs[] = {
	&kensho_crashed_attr.attr,
	&kensho_info_attr.attr,
	NULL,
};

static struct bin_attribute *kensho_bin_attrs[] = {
	&bin_attr_last_panic,
	NULL,
};

static const struct attribute_group kensho_group = {
	.attrs = kensho_attrs,
	.bin_attrs = kensho_bin_attrs,
};
static const struct attribute_group *kensho_groups[] = {
	&kensho_group,
	NULL,
};

/* ------------------------------------------------------------------ */
/* Persistent memory (survives reboots via reserved DRAM)              */
/* ------------------------------------------------------------------ */

#ifdef CONFIG_VINDICATOR_KENSHO_PERSIST

/*
 * Reserve 128 KB of physical memory very early, before any other
 * allocator is online.  This region will NOT be touched by the kernel
 * after this point, so its contents survive warm reboots.
 *
 * On the NEXT boot we check the magic — if it still matches, the
 * crash log made it across the reboot boundary.
 */
static int __init kensho_persist_early_init(void)
{
	kensho_persist_pa = memblock_phys_alloc(KENSHO_PERSIST_SIZE, SZ_4K);
	if (!kensho_persist_pa) {
		pr_warn("kensho: failed to reserve %u bytes for persistent "
			"storage\n", KENSHO_PERSIST_SIZE);
		return -ENOMEM;
	}

	kensho_persist_va = __va(kensho_persist_pa);

	pr_info("kensho: persistent region reserved at 0x%llx (%u bytes)\n",
		(unsigned long long)kensho_persist_pa,
		KENSHO_PERSIST_SIZE);

	/* The header/size check + import happens in kensho_init() */
	return 0;
}
early_initcall(kensho_persist_early_init);

/*
 * Attempt to import a previous crash log from the persistent region.
 * Called from kensho_init() after the panic buffer is allocated.
 */
static void kensho_persist_import(void)
{
	struct kensho_persist_header *hdr = kensho_persist_va;
	size_t data_size;

	if (!kensho_persist_va)
		return;

	if (hdr->magic != KENSHO_PERSIST_MAGIC) {
		/* No valid data — first boot or already consumed */
		memset(kensho_persist_va, 0, KENSHO_PERSIST_SIZE);
		return;
	}

	data_size = hdr->size;
	if (data_size == 0 ||
	    data_size > KENSHO_PERSIST_SIZE - sizeof(*hdr)) {
		/* Corrupt — clear and move on */
		memset(kensho_persist_va, 0, KENSHO_PERSIST_SIZE);
		return;
	}

	if (data_size >= KENSHO_BUF_SIZE)
		data_size = KENSHO_BUF_SIZE - 1;

	memcpy(kensho_panic_buf,
	       kensho_persist_va + sizeof(*hdr),
	       data_size);
	kensho_panic_buf[data_size] = '\0';
	kensho_panic_len = data_size;
	kensho_crashed = true;

	pr_info("kensho: recovered crash log from previous boot "
		"(%zu bytes, ts=%llu)\n", data_size,
		(unsigned long long)hdr->timestamp);

	/* Consume the data so we don't read it twice */
	memset(kensho_persist_va, 0, KENSHO_PERSIST_SIZE);
}

/*
 * Write the captured panic log to the persistent region.
 * Called from the kmsg dumper callback (panic context).
 * We flush the D-cache so the data is in DRAM before reboot.
 */
static void kensho_persist_save(void)
{
	struct kensho_persist_header *hdr = kensho_persist_va;
	size_t write_size;

	if (!kensho_persist_va || kensho_panic_len < 8)
		return;

	write_size = min_t(size_t, kensho_panic_len,
			   KENSHO_PERSIST_SIZE - sizeof(*hdr));

	hdr->magic = KENSHO_PERSIST_MAGIC;
	hdr->timestamp = ktime_get_real_seconds();
	hdr->size = write_size;

	memcpy(kensho_persist_va + sizeof(*hdr),
	       kensho_panic_buf, write_size);

	/*
	 * Ensure the data is visible in DRAM before the reboot hits.
	 *
	 * On ARM64, stores can sit in the CPU store buffer without
	 * reaching the cache.  __flush_dcache_area only cleans cache
	 * lines that already contain the data — if we skip the barrier,
	 * the memcpy payload may still be in the store buffer when the
	 * cache flush executes, and the actual data never makes it to
	 * DRAM.  The magic would be gone after reboot.
	 *
	 * wmb() drains the store buffer to the cache so that the
	 * subsequent dc civac instructions hit valid lines.
	 */
	wmb();
	__flush_dcache_area(kensho_persist_va, KENSHO_PERSIST_SIZE);
}

#else /* !CONFIG_VINDICATOR_KENSHO_PERSIST */

static void kensho_persist_import(void) { }
static void kensho_persist_save(void)  { }

#endif /* CONFIG_VINDICATOR_KENSHO_PERSIST */

/* ------------------------------------------------------------------ */
/* kmsg dumper callback (updated — persist save)                      */
/* ------------------------------------------------------------------ */

/*
 * Called from panic() context — the kernel log buffer is still intact.
 * We drain it into the remaining space after our metadata header,
 * then flush to persistent memory so it survives the reboot.
 */
static void kensho_dump_cb(struct kmsg_dumper *dumper,
			   enum kmsg_dump_reason reason)
{
	size_t len;
	size_t remaining;

	/* Only capture panics */
	if (reason > KMSG_DUMP_PANIC)
		return;

	/* Write metadata header */
	kensho_write_metadata();

	/* Fill the rest with kernel log */
	remaining = KENSHO_BUF_SIZE - kensho_panic_len;
	if (remaining >= 64) {
		if (kmsg_dump_get_buffer(dumper, true,
					 kensho_panic_buf + kensho_panic_len,
					 remaining - 1, &len)) {
			kensho_panic_len += len;
			kensho_panic_buf[kensho_panic_len] = '\0';
		}
	}

	/* Persist across reboot */
	kensho_persist_save();
}

/* ------------------------------------------------------------------ */
/* Panic notifier                                                      */
/* ------------------------------------------------------------------ */

/*
 * Fires before kmsg_dump in the panic() sequence. We save the panic
 * reason string so kensho_write_metadata can include it.
 * Priority INT_MAX means we run first.
 */
static int kensho_panic_cb(struct notifier_block *nb,
			   unsigned long action, void *data)
{
	if (data)
		strscpy(kensho_reason, (const char *)data,
			sizeof(kensho_reason));
	return NOTIFY_DONE;
}

/* ------------------------------------------------------------------ */
/* Sysfs                                                               */
/* ------------------------------------------------------------------ */

/*
 * Use bin_attribute for last_panic so the entire 128 kB buffer is
 * accessible via read(), not truncated to PAGE_SIZE (4 kB).
 */
static ssize_t last_panic_read(struct file *filp, struct kobject *kobj,
			       struct bin_attribute *attr,
			       char *buf, loff_t off, size_t count)
{
	static const char no_crash[] = "No panic captured since boot.\n";

	if (!kensho_crashed) {
		if (off >= (loff_t)sizeof(no_crash))
			return 0;
		count = min_t(size_t, sizeof(no_crash) - (size_t)off, count);
		memcpy(buf, no_crash + off, count);
		return count;
	}

	if (off >= (loff_t)kensho_panic_len)
		return 0;

	count = min_t(size_t, count, kensho_panic_len - (size_t)off);
	memcpy(buf, kensho_panic_buf + off, count);
	return count;
}

static ssize_t crashed_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", kensho_crashed ? 1 : 0);
}

static ssize_t info_show(struct kobject *kobj,
			 struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
		"Kenshō v%s\n"
		"Status: %s\n"
		"Buffer: %u bytes\n"
		"Persist: %s\n",
		KENSHO_VERSION,
		kensho_crashed ? "CRASH CAPTURED" : "Standing by",
		(unsigned int)KENSHO_BUF_SIZE,
		IS_ENABLED(CONFIG_VINDICATOR_KENSHO_PERSIST) ?
			"enabled" : "disabled");
}

/* last_panic as a binary attribute — no PAGE_SIZE limit */
static BIN_ATTR(last_panic, 0444, last_panic_read, NULL);

static struct kobj_attribute kensho_crashed_attr =
	__ATTR_RO(crashed);
static struct kobj_attribute kensho_info_attr =
	__ATTR_RO(info);

/* bin_attrs must be in a separate list from regular attrs */
static struct attribute *kensho_attrs[] = {
	&kensho_crashed_attr.attr,
	&kensho_info_attr.attr,
	NULL,
};

static struct bin_attribute *kensho_bin_attrs[] = {
	&bin_attr_last_panic,
	NULL,
};

static const struct attribute_group kensho_group = {
	.attrs = kensho_attrs,
	.bin_attrs = kensho_bin_attrs,
};
static const struct attribute_group *kensho_groups[] = {
	&kensho_group,
	NULL,
};

/* ------------------------------------------------------------------ */
/* Module init / exit                                                  */
/* ------------------------------------------------------------------ */

static int __init kensho_init(void)
{
	int ret;

	/* Pre-allocate the panic buffer — never freed */
	kensho_panic_buf = kmalloc(KENSHO_BUF_SIZE, GFP_KERNEL);
	if (!kensho_panic_buf)
		return -ENOMEM;

	/*
	 * Import any crash log that survived from a previous boot
	 * (only if CONFIG_VINDICATOR_KENSHO_PERSIST is enabled).
	 */
	kensho_persist_import();

	/* Sysfs — /sys/kernel/kensho/ */
	kensho_kobj = kobject_create_and_add("kensho", kernel_kobj);
	if (!kensho_kobj) {
		pr_warn("kensho: failed to create sysfs entry\n");
		kfree(kensho_panic_buf);
		kensho_panic_buf = NULL;
		return -ENOMEM;
	}

	ret = sysfs_create_groups(kensho_kobj, kensho_groups);
	if (ret) {
		pr_warn("kensho: failed to create sysfs attributes\n");
		kobject_put(kensho_kobj);
		kfree(kensho_panic_buf);
		kensho_panic_buf = NULL;
		return ret;
	}

	/* Panic notifier — run first to grab reason string */
	kensho_panic_nb.notifier_call = kensho_panic_cb;
	kensho_panic_nb.priority = INT_MAX;
	atomic_notifier_chain_register(&panic_notifier_list,
				       &kensho_panic_nb);

	/* kmsg dumper — capture the full log buffer at panic */
	kensho_dumper.dump = kensho_dump_cb;
	kensho_dumper.max_reason = KMSG_DUMP_PANIC;
	ret = kmsg_dump_register(&kensho_dumper);
	if (ret) {
		pr_warn("kensho: kmsg_dump_register failed (%d)\n", ret);
		atomic_notifier_chain_unregister(&panic_notifier_list,
						 &kensho_panic_nb);
		sysfs_remove_groups(kensho_kobj, kensho_groups);
		kobject_put(kensho_kobj);
		kfree(kensho_panic_buf);
		kensho_panic_buf = NULL;
		return ret;
	}

	/* Initialise metadata (will be overwritten on panic) */
	memset(kensho_reason, 0, sizeof(kensho_reason));

	pr_info("kensho: Crash capture ready (%u byte buffer%s)\n",
		KENSHO_BUF_SIZE,
		IS_ENABLED(CONFIG_VINDICATOR_KENSHO_PERSIST) ?
			", persistent storage active" : "");
	return 0;
}

static void __exit kensho_exit(void)
{
	kmsg_dump_unregister(&kensho_dumper);
	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &kensho_panic_nb);

	sysfs_remove_groups(kensho_kobj, kensho_groups);
	kobject_put(kensho_kobj);

	kfree(kensho_panic_buf);
	kensho_panic_buf = NULL;

	pr_info("kensho: unloaded\n");
}

module_init(kensho_init);
module_exit(kensho_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GrayRavens");
MODULE_DESCRIPTION("Kenshō — Kernel panic capture and forensic logging");
