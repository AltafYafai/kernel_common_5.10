// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/misc/vindicator/oto.c
 * Oto — Audio thread SCHED_FIFO boost + PM QoS + CPUSet manager
 *
 * Detects audio server threads and promotes them to SCHED_FIFO with
 * configurable priority. Engages a PM QoS latency lock to prevent
 * deep idle states during audio playback, and maintains dynamic CPUSet
 * masks for foreground/top-app to ensure audio threads have access
 * to all cores.
 *
 * Author: GrayRavens
 * Co-authored-by: Kanagawa Yamada <albert.wesley.dion@gmail.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/sched/signal.h>
#include <linux/pm_qos.h>
#include <linux/cpu.h>
#include <linux/rcupdate.h>
#include <linux/string.h>
#include <linux/atomic.h>
#include <linux/pid.h>
#include <uapi/linux/sched/types.h>

#define OTO_ENGAGE_DELAY_MS	20000
#define OTO_SCAN_MS		5000
#define OTO_PM_QOS_LATENCY_US	100
#define OTO_MAX_AUDIO_PIDS	64
#define OTO_FIFO_PRIORITY	2

static bool oto_enabled = true;
module_param(oto_enabled, bool, 0644);
MODULE_PARM_DESC(oto_enabled, "Enable Oto audio boost (default: true)");

static unsigned int oto_fifo_priority = OTO_FIFO_PRIORITY;
module_param(oto_fifo_priority, uint, 0644);
MODULE_PARM_DESC(oto_fifo_priority,
		 "SCHED_FIFO priority for audio threads (default: 2)");

static struct pm_qos_request oto_pm_qos;
static bool oto_pm_qos_active;
static struct task_struct *oto_thread;

static char oto_cpu_mask[32];

static const char * const oto_audio_threads[] = {
	"audioserver",
	"AudioOut",
	"AudioIn",
	"FastMixer",
	"FastCapture",
	"AudioFlinger",
	"AudioTrack",
	"AudioRecord",
	"audio.r_submix",
	"usb_audio_wq",
	"audio_hw_thread",
	"audio_hal_worker",
	NULL
};

static int oto_write_file(const char *path, const char *buf)
{
	struct file *f;
	loff_t pos = 0;
	int ret;

	f = filp_open(path, O_WRONLY, 0);
	if (IS_ERR(f))
		return PTR_ERR(f);

	ret = kernel_write(f, buf, strlen(buf), &pos);
	filp_close(f, NULL);
	return ret < 0 ? ret : 0;
}

static void oto_apply_cpuset_override(void)
{
	int total_cores = num_possible_cpus();

	snprintf(oto_cpu_mask, sizeof(oto_cpu_mask),
		 "0-%d", total_cores - 1);

	oto_write_file("/dev/cpuset/foreground/cpus", oto_cpu_mask);
	oto_write_file("/dev/cpuset/top-app/cpus", oto_cpu_mask);
	oto_write_file("/dev/cpuset/boost-app/cpus", oto_cpu_mask);

	pr_debug("oto: cpuset override applied -> %s\n", oto_cpu_mask);
}

static void oto_boost_audio_threads(void)
{
	struct task_struct *p;
	struct sched_param param = { .sched_priority = oto_fifo_priority };
	pid_t pids[OTO_MAX_AUDIO_PIDS];
	int count = 0;
	int boosted = 0;
	int i, j;

	rcu_read_lock();
	for_each_process(p) {
		if (count >= OTO_MAX_AUDIO_PIDS)
			break;
		for (i = 0; oto_audio_threads[i]; i++) {
			if (strncmp(p->comm, oto_audio_threads[i],
				    TASK_COMM_LEN) == 0) {
				pids[count++] = p->pid;
				break;
			}
		}
	}
	rcu_read_unlock();

	for (j = 0; j < count; j++) {
		rcu_read_lock();
		p = find_task_by_vpid(pids[j]);
		if (p)
			get_task_struct(p);
		rcu_read_unlock();

		if (!p)
			continue;

		if (!rt_task(p)) {
			sched_setscheduler_nocheck(p, SCHED_FIFO, &param);
			pr_debug("oto: boosted %s (pid %d) -> SCHED_FIFO prio %u\n",
				 p->comm, p->pid, oto_fifo_priority);
			boosted++;
		}
		put_task_struct(p);
	}

	if (boosted > 0)
		pr_debug("oto: %d audio thread(s) boosted to SCHED_FIFO\n",
			 boosted);
}

static void oto_pm_qos_engage(void)
{
	if (oto_pm_qos_active)
		return;

	cpu_latency_qos_add_request(&oto_pm_qos, OTO_PM_QOS_LATENCY_US);
	oto_pm_qos_active = true;
	pr_info("oto: PM QoS latency locked to %d us\n", OTO_PM_QOS_LATENCY_US);
}

static int oto_worker(void *data)
{
	pr_info("oto: standing by — engaging in %d ms\n", OTO_ENGAGE_DELAY_MS);
	msleep(OTO_ENGAGE_DELAY_MS);

	oto_pm_qos_engage();
	oto_apply_cpuset_override();

	while (!kthread_should_stop()) {
		oto_boost_audio_threads();

		/* Watchdog: detect and correct vendor cpuset rollback */
		{
			struct file *f;
			char current_mask[32] = {0};
			char *cleaned;

			f = filp_open("/dev/cpuset/foreground/cpus",
				      O_RDONLY, 0);
			if (!IS_ERR(f)) {
				loff_t pos = 0;
				kernel_read(f, current_mask,
					    sizeof(current_mask) - 1, &pos);
				filp_close(f, NULL);
			}

			cleaned = strim(current_mask);
			if (strlen(cleaned) > 0 &&
			    strstr(cleaned, oto_cpu_mask) == NULL) {
				pr_info("oto: watchdog caught cpuset rollback"
					" ('%s') — re-enforcing\n", cleaned);
				oto_apply_cpuset_override();
			}
		}

		msleep_interruptible(OTO_SCAN_MS);
	}
	return 0;
}

static int __init oto_init(void)
{
	if (!oto_enabled) {
		pr_info("oto: disabled via module param\n");
		return 0;
	}

	oto_thread = kthread_run(oto_worker, NULL, "oto_audio");
	if (IS_ERR(oto_thread)) {
		pr_err("oto: failed to start worker thread: %ld\n",
		       PTR_ERR(oto_thread));
		return PTR_ERR(oto_thread);
	}

	pr_info("oto: active (prio=%u, scan=%dms, engage=%dms)\n",
		oto_fifo_priority, OTO_SCAN_MS, OTO_ENGAGE_DELAY_MS);
	return 0;
}
late_initcall(oto_init);

static void __exit oto_exit(void)
{
	if (oto_thread)
		kthread_stop(oto_thread);

	if (oto_pm_qos_active) {
		cpu_latency_qos_remove_request(&oto_pm_qos);
		oto_pm_qos_active = false;
	}

	pr_info("oto: unloaded\n");
}
module_exit(oto_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GrayRavens");
MODULE_DESCRIPTION("Oto — Audio thread SCHED_FIFO boost + PM QoS + CPUSet manager");
