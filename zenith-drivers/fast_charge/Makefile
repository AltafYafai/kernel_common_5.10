# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for fast_charge — universal GKI charger current booster
#
# Build against GKI kernel source:
#   make -C /path/to/gki_kernel_src M=$(pwd) modules
#
# Cross-compile for aarch64:
#   make ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- \
#     -C /path/to/gki_kernel_src M=$(pwd) modules
#

obj-m += fast_charge.o

KBUILD_CFLAGS += -D__CHECK_ENDIAN__

all:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

install:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules_install
	depmod -a
