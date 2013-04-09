#
# .zd1211 - USB2.0 802.11b/g driver for Zydas ZD1211 chipsets
#
#
#

CC=arm-linux-gcc
CPP=arm-linux-g++
LD=arm-linux-ld
rm=rm -f -r

MODPATH := ${INSTALL_MOD_PATH}/lib/modules/2.6.24

# if the kernel is 2.6.x, turn on this
KERN_26=y

KERNEL_SOURCE=$(srctree)
#KERNEL_SOURCE=/usr/src/linux

# set to 1 for zd1211b
ZD1211REV_B=0

SRC_DIR=src
DEFINES=-D__KERNEL__ -DMODULE=1



ifeq ($(KERN_26), y)

ifeq ($(ZD1211REV_B),1)
MODULE = zd1211b.ko
endif
ifeq ($(ZD1211REV_B),0)
MODULE = zd1211.ko
endif

INCLUDES=-I$(KERNEL_SOURCE)/include -I$(SRC_DIR)/include/ -I$(SRC_DIR)

EXTRA_CFLAGS += -I$(PWD)/src/include

ifndef CONFIG_FRAME_POINTER
EXTRA_CFLAGS += -fomit-frame-pointer
endif

ifdef CONFIG_SMP
EXTRA_CFLAGS += -D__SMP__ -DSMP
endif

KDIR := $(srctree)
PWD := $(shell pwd)

WLAN_SRC=$(PWD)


EXTRA_CFLAGS += -O2 -Wall -Wstrict-prototypes -pipe
#EXTRA_CFLAGS += -Wa,-a,-ad -g
EXTRA_CFLAGS += -DZDCONF_WE_STAT_SUPPORT=1
EXTRA_CFLAGS += -DHOST_IF_USB
EXTRA_CFLAGS += -DAMAC
EXTRA_CFLAGS += -DGCCK
EXTRA_CFLAGS += -DOFDM
EXTRA_CFLAGS += -DHOSTAPD_SUPPORT
EXTRA_CFLAGS += -DUSE_EP4_SET_REG
EXTRA_CFLAGS += -DDOWNLOADFIRMWARE
EXTRA_CFLAGS += -DfTX_GAIN_OFDM=0
EXTRA_CFLAGS += -DfNEW_CODE_MAP=1
EXTRA_CFLAGS += -DfWRITE_WORD_REG=1
EXTRA_CFLAGS += -DfREAD_MUL_REG=1
EXTRA_CFLAGS += -DENHANCE_RX=1
ifeq ($(ZD1211REV_B),1)
	EXTRA_CFLAGS += -DZD1211B
endif
ifeq ($(ZD1211REV_B),0)
	EXTRA_CFLAGS += -DZD1211
endif
#EXTRA_CFLAGS += $(INCLUDES)

ifeq ($(ZD1211REV_B),1)
	obj-m := zd1211b.o
endif
ifeq ($(ZD1211REV_B),0)
	obj-m := zd1211.o
endif
zd1211-objs := $(SRC_DIR)/zd1205.o \
$(SRC_DIR)/zdasocsvc.o \
$(SRC_DIR)/zdauthreq.o \
$(SRC_DIR)/zdauthrsp.o \
$(SRC_DIR)/zdmmrx.o \
$(SRC_DIR)/zdshared.o \
$(SRC_DIR)/zdhci.o \
$(SRC_DIR)/zdglobal.o \
$(SRC_DIR)/zdencrypt.o \
$(SRC_DIR)/zdpmfilter.o \
$(SRC_DIR)/zdpsmon.o \
$(SRC_DIR)/zdsynch.o \
$(SRC_DIR)/zdbuf.o \
$(SRC_DIR)/zd1205_proc.o \
$(SRC_DIR)/zdhw.o \
$(SRC_DIR)/zddebug.o \
$(SRC_DIR)/zdtkipseed.o \
$(SRC_DIR)/zdmic.o \
$(SRC_DIR)/zdusb.o
ifeq ($(ZD1211REV_B),1)
zd1211-objs += $(SRC_DIR)/zd1211.o
zd1211b-objs = $(zd1211-objs)
endif
ifeq ($(ZD1211REV_B),0)
zd1211-objs += $(SRC_DIR)/zd1211.o
endif

all:

ifneq ($(KERNELRELEASE),)

else
ifndef ZD1211REV_B
		make both
else
		@echo -e $(KDIR)
		@echo -e $(PWD)
		@echo -e $(EXTRA_CFLAGS)
		@echo -e $(zd1211-objs)
		$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
endif

endif

else # kernel 2.4

INCLUDES=-I$(KERNEL_SOURCE)/include -I$(SRC_DIR)/include/ -I$(SRC_DIR)
ifeq ($(ZD1211REV_B),1)
	MODULE = zd1211b.o
endif
ifeq ($(ZD1211REV_B),0)
	MODULE = zd1211.o
endif

OBJECTS=$(SRC_DIR)/zd1205.o \
    $(SRC_DIR)/zdasocsvc.o \
    $(SRC_DIR)/zdauthreq.o \
    $(SRC_DIR)/zdauthrsp.o \
    $(SRC_DIR)/zdmmrx.o \
    $(SRC_DIR)/zdshared.o \
    $(SRC_DIR)/zdhci.o \
    $(SRC_DIR)/zdglobal.o \
    $(SRC_DIR)/zdencrypt.o \
    $(SRC_DIR)/zdpmfilter.o \
    $(SRC_DIR)/zdpsmon.o \
    $(SRC_DIR)/zdsynch.o \
    $(SRC_DIR)/zdbuf.o \
    $(SRC_DIR)/zd1205_proc.o \
    $(SRC_DIR)/zdhw.o \
    $(SRC_DIR)/zddebug.o \
    $(SRC_DIR)/zdtkipseed.o \
    $(SRC_DIR)/zdmic.o \
    $(SRC_DIR)/zdusb.o
    OBJECTS += $(SRC_DIR)/zd1211.o

CFLAGS=-O -Wall -Wstrict-prototypes -pipe # -Wa,-a,-ad -g

ifdef CONFIG_MODVERSIONS
CFLAGS += -DMODVERSIONS -include $(KERNEL_SOURCE)/include/linux/modversions.h   #kernel 2.4
endif

ifndef CONFIG_FRAME_POINTER
CFLAGS += -fomit-frame-pointer
endif

ifdef CONFIG_SMP
CFLAGS += -D__SMP__ -DSMP
endif

CFLAGS += -DZDCONF_WE_STAT_SUPPORT=1
CFLAGS += -DHOST_IF_USB
CFLAGS += -DAMAC
CFLAGS += -DGCCK
CFLAGS += -DOFDM
CFLAGS += -DHOSTAPD_SUPPORT
CFLAGS += -DUSE_EP4_SET_REG
CFLAGS += -DDOWNLOADFIRMWARE
CFLAGS += -DfTX_GAIN_OFDM=0
CFLAGS += -DfNEW_CODE_MAP=1
CFLAGS += -DfWRITE_WORD_REG=1
CFLAGS += -DfREAD_MUL_REG=1
ifeq ($(ZD1211REV_B),1)
	CFLAGS += -DZD1211B
endif
ifeq ($(ZD1211REV_B),0)
	CFLAGS += -DZD1211
endif
CFLAGS += -DENHANCE_RX=1

ifndef ZD1211REV_B
all:
	make both
else
all: $(MODULE)
endif

$(MODULE): $(OBJECTS)
	$(LD) -static  -r $(OBJECTS) -o $(MODULE)
	chmod -x $(MODULE)

%.o: %.c
	$(CC) -static $(CFLAGS) $(INCLUDES) $(DEFINES) $(DEBUG) -c $< -o $@

endif
both:
	make ZD1211REV_B=0
	make install ZD1211REV_B=0
	make clean
	make ZD1211REV_B=1
	make install ZD1211REV_B=1

menuconfig:
	/bin/sh scripts/Menuconfig config.in
inst:
	make
	make install



install: all
	mkdir -p $(MODPATH)/net
#	mkdir -p /etc/zd1211
	cp $(MODULE) $(MODPATH)/net
	depmod -a

#for apdbg
	gcc -o apdbg apdbg.c
	chmod +x apdbg
	cp ./apdbg /sbin/apdbg

clean:
	rm -rf .tmp_versions .*.cmd *.ko *.mod.c *.mod.o *.o $(SRC_DIR)/*.o  $(SRC_DIR)/.*.o.cmd
