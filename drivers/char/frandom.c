// SPDX-License-Identifier: GPL-2.0-only
/*
 * frandom - Fast PRNG character device
 *
 * Provides /dev/frandom using the kernel's prandom_u32() for
 * non-security-critical fast random numbers.  ~5-10x faster than
 * /dev/urandom for workloads like ASLR, key generation in games,
 * and other high-frequency random reads.
 *
 * Tunables:
 *   /sys/module/frandom/parameters/buf_size  - read buffer size (4-4096)
 */

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/prandom.h>
#include <linux/uaccess.h>

#define FRANDOM_MIN_BUF  4
#define FRANDOM_MAX_BUF  4096

static unsigned int buf_size = 64;
module_param(buf_size, uint, 0644);
MODULE_PARM_DESC(buf_size, "Read buffer size (4-4096, default 64)");

static ssize_t frandom_read(struct file *file, char __user *buf,
			    size_t count, loff_t *ppos)
{
	u32 tmp[256];  /* max 1024 bytes stack buffer */
	size_t chunk, copied = 0;
	int i;

	while (count > 0) {
		chunk = min(count, sizeof(tmp));
		chunk = min(chunk, (size_t)READ_ONCE(buf_size));
		chunk = rounddown(chunk, sizeof(u32));
		if (chunk < sizeof(u32))
			break;

		for (i = 0; i < chunk / sizeof(u32); i++)
			tmp[i] = prandom_u32();

		if (copy_to_user(buf + copied, tmp, chunk))
			return copied ? copied : -EFAULT;

		copied += chunk;
		count  -= chunk;
	}

	return copied;
}

static const struct file_operations frandom_fops = {
	.owner  = THIS_MODULE,
	.read   = frandom_read,
	.llseek = noop_llseek,
};

static struct miscdevice frandom_misc = {
	.minor  = MISC_DYNAMIC_MINOR,
	.name   = "frandom",
	.fops   = &frandom_fops,
};

static int __init frandom_init(void)
{
	int ret;

	ret = misc_register(&frandom_misc);
	if (ret)
		pr_err("frandom: failed to register misc device: %d\n", ret);
	else
		pr_info("frandom: registered (buf_size=%u)\n",
			READ_ONCE(buf_size));
	return ret;
}

static void __exit frandom_exit(void)
{
	misc_deregister(&frandom_misc);
}

module_init(frandom_init);
module_exit(frandom_exit);

MODULE_DESCRIPTION("Fast PRNG character device (/dev/frandom)");
MODULE_AUTHOR("XTENSEI");
MODULE_LICENSE("GPL v2");
