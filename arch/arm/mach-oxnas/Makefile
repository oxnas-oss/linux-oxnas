#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y := oxnas.o irq.o time.o dma.o pci.o ahb_mon.o leon.o samba_reserve.o

obj-$(CONFIG_SYNOPSYS_GMAC) += gmac.o

gmac-objs := gmac-napi.o gmac_ethtool.o gmac_phy.o gmac_desc.o gmac_offload.o

obj-$(CONFIG_OXNAS_IBW) += ibw.o

obj-$(CONFIG_TACHO_THERM_AND_FAN) += thermAndFan.o

obj-$(CONFIG_I2S) += i2s.o

obj-$(CONFIG_CRYPTO_OXAESLRW) += cipher.o

obj-$(CONFIG_GPIO_TEST)	+= gpioTest.o

obj-$(CONFIG_I2S) += i2s.o

obj-$(CONFIG_DPE_TEST) += dpe_test.o

obj-$(CONFIG_OXNAS_AHB_MONITOR_MODULE) += oxnas-ahb-monitor.o

obj-$(CONFIG_OXNAS_USB_TEST_MODES) += usb-test-mode.o

obj-$(CONFIG_LEON_POWER_BUTTON_MONITOR) += power_button.o

obj-$(CONFIG_USER_RECOVERY_BUTTON_MONITOR) += user_recovery_button.o

obj-$(CONFIG_OXNAS_FRONT_LAMP_CONTROL) += leds.o

obj-$(CONFIG_WDC_FAN_OXNAS800) += wdc-fan.o

obj-$(CONFIG_WDC_LEDS_OXNAS800) += wdc-leds.o

obj-$(CONFIG_OXNAS_WD810_LEDS) += oxnas-wd810-leds.o

obj-$(CONFIG_WDC_LEDS_TRIGGER_SATA_DISK) += wdc-ledtrig-sata.o


# gpio control interface
obj-${CONFIG_ZYXEL_GPIO_CTRL} += gpio_ctrl/

obj-$(CONFIG_ZYXEL_HW_TEST) += hwTest.o
