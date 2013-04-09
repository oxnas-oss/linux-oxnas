#
# Makefile for the linux kernel.
#

# Define the master clock frequencies of the NAS chip - allow overriding from make invocation cmd line
PLL400 ?= 400000000
RPSCLK ?= 25000000

CFLAGS += -DNOMINAL_PLL400_FREQ=$(PLL400) -DNOMINAL_RPSCLK_FREQ=$(RPSCLK)

# Object file lists.

obj-y	:= oxnas.o irq.o time.o dma.o pci.o ahb_mon.o

obj-$(CONFIG_OXNAS_IBW) += ibw.o

obj-$(CONFIG_TACHO_THERM_AND_FAN) += thermAndFan.o

obj-$(CONFIG_SYNOPSYS_GMAC) += gmac-napi.o gmac_ethtool.o gmac_phy.o gmac_desc.o

obj-$(CONFIG_LEON_COPRO) += gmac_offload.o

obj-$(CONFIG_SUPPORT_LEON) += leon.o

obj-$(CONFIG_I2S) += i2s.o

obj-$(CONFIG_CRYPTO_OXAESLRW) += cipher.o

obj-$(CONFIG_GPIO_TEST)	+= gpioTest.o

obj-$(CONFIG_I2S)		+= i2s.o

obj-$(CONFIG_DPE_TEST) += dpe_test.o

obj-$(CONFIG_OXNAS_M41T0) += m41t00.o

obj-$(CONFIG_OXNAS_AHB_MONITOR_MODULE) += oxnas-ahb-monitor.o

obj-$(CONFIG_OXNAS_USB_TEST_MODES) += usb-test-mode.o

obj-$(CONFIG_LEON_POWER_BUTTON_MONITOR) += power_button.o

obj-$(CONFIG_WDC_FAN_OXNAS800) += wdc-fan.o

obj-$(CONFIG_WDC_LEDS_OXNAS800) += wdc-leds.o

obj-$(CONFIG_WDC_LEDS_TRIGGER_SATA_DISK) += wdc-ledtrig-sata.o
