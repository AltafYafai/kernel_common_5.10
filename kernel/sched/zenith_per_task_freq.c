// SPDX-License-Identifier: GPL-2.0-only
/*
 * zenith_per_task_freq - Per-task minimum frequency floor
 *
 * Allows tasks to request a minimum CPU frequency via /proc/zenith/per_task_freq.
 * When a task with an active freq floor is on a CPU, Zenith governor applies
 * the floor as a scheduling hint.
 *
 * Interface: /proc/zenith/per_task_freq
 *   Write: "pid=<pid> freq=<kHz>" — set task minimum freq
 *   Write: "pid=<pid> freq=0"     — clear task minimum freq
 *   Read: shows all active task freq floors
 */
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/cpufreq_zenith.h>

#define MAX_FLOOR_ENTRIES 64

struct task_freq_floor {
	pid_t pid;
	unsigned int freq_khz;
};

static struct task_freq_floor *task_floors;
static int nr_task_floors;
static DEFINE_MUTEX(task_floors_lock);

int zenith_get_task_freq_floor(pid_t pid)
{
	int i;
	int nr = READ_ONCE(nr_task_floors);

	for (i = 0; i < nr; i++) {
		if (READ_ONCE(task_floors[i].pid) == pid)
			return READ_ONCE(task_floors[i].freq_khz);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(zenith_get_task_freq_floor);

static int task_freq_proc_show(struct seq_file *m, void *v)
{
	int i;

	mutex_lock(&task_floors_lock);
	for (i = 0; i < nr_task_floors; i++)
		seq_printf(m, "pid=%d freq=%u\n",
			   task_floors[i].pid, task_floors[i].freq_khz);
	mutex_unlock(&task_floors_lock);

	return 0;
}

static ssize_t task_freq_proc_write(struct file *file, const char __user *buf,
				    size_t count, loff_t *ppos)
{
	char tmp[128];
	pid_t pid;
	unsigned int freq;
	int ret;

	if (count >= sizeof(tmp))
		return -E2BIG;

	if (copy_from_user(tmp, buf, count))
		return -EFAULT;
	tmp[count] = '\0';

	ret = sscanf(tmp, "pid=%d freq=%u", &pid, &freq);
	if (ret != 2)
		return -EINVAL;

	mutex_lock(&task_floors_lock);		if (freq == 0) {
		/* Remove entry */
		int i;

		for (i = 0; i < nr_task_floors; i++) {
			if (READ_ONCE(task_floors[i].pid) == pid) {
				WRITE_ONCE(task_floors[i].pid,
					   task_floors[nr_task_floors - 1].pid);
				WRITE_ONCE(task_floors[i].freq_khz,
					   task_floors[nr_task_floors - 1].freq_khz);
				WRITE_ONCE(nr_task_floors, nr_task_floors - 1);
				break;
			}
		}
	} else {
		/* Add or update entry */
		int i;

		for (i = 0; i < nr_task_floors; i++) {
			if (READ_ONCE(task_floors[i].pid) == pid) {
				WRITE_ONCE(task_floors[i].freq_khz, freq);
				goto done;
			}
		}

		if (nr_task_floors < MAX_FLOOR_ENTRIES) {
			WRITE_ONCE(task_floors[nr_task_floors].pid, pid);
			WRITE_ONCE(task_floors[nr_task_floors].freq_khz, freq);
			WRITE_ONCE(nr_task_floors, nr_task_floors + 1);
		}
	}

done:
	mutex_unlock(&task_floors_lock);
	return count;
}

static int task_freq_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, task_freq_proc_show, NULL);
}

static const struct proc_ops task_freq_proc_fops = {
	.proc_open    = task_freq_proc_open,
	.proc_read    = seq_read,
	.proc_write   = task_freq_proc_write,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
};

static int __init per_task_freq_init(void)
{
	task_floors = kcalloc(MAX_FLOOR_ENTRIES, sizeof(*task_floors), GFP_KERNEL);
	if (!task_floors)
		return -ENOMEM;

	proc_create("per_task_freq", 0644, NULL, &task_freq_proc_fops);

	pr_info("zenith_per_task_freq: registered\n");
	return 0;
}

static void __exit per_task_freq_exit(void)
{
	remove_proc_entry("per_task_freq", NULL);
	kfree(task_floors);
}

module_init(per_task_freq_init);
module_exit(per_task_freq_exit);

MODULE_DESCRIPTION("Zenith Per-Task Frequency Floor");
MODULE_AUTHOR("XTENSEI");
MODULE_LICENSE("GPL v2");
