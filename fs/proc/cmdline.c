// SPDX-License-Identifier: GPL-2.0
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/kaguya.h>

static int cmdline_proc_show(struct seq_file *m, void *v)
{
#ifdef CONFIG_KAGUYA_CMDLINE_SPOOF
	{
		char *spoofed = kaguya_spoof_boot_args(saved_command_line,
						       false);

		if (spoofed) {
			seq_puts(m, spoofed);
			kfree(spoofed);
		} else {
			seq_puts(m, saved_command_line);
		}
	}
#else
	seq_puts(m, saved_command_line);
#endif
	seq_putc(m, '\n');
	return 0;
}

static int __init proc_cmdline_init(void)
{
	proc_create_single("cmdline", 0, NULL, cmdline_proc_show);
	return 0;
}
fs_initcall(proc_cmdline_init);
