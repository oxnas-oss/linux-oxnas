/*
 * linux/include/asm-arm/arch-oxnas/thecus_io.h
 *
 * Board support code for the Thecus N0204 platform.
 *
 * Author: citizen <citizen_lee@thecus.com>
 * Copyright (C) 2008 Thecus Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#ifndef __THECUS_IO_H
#define __THECUS_IO_H

#include <asm/hardware.h>

extern spinlock_t oxnas_gpio_spinlock;

#if defined(CONFIG_THECUS_N0204_IO) || defined(CONFIG_THECUS_N0204_IO_MODULE)
#define GPIO_N0204_USB_COPY_SUCCESS_LED	OX810_GPIO_0
#define GPIO_N0204_USB_COPY_FAIL_LED	OX810_GPIO_1
#define GPIO_N0204_DISK2_DETECT		OX810_GPIO_2
#define GPIO_N0204_DRAM_VENDER		OX810_GPIO_3
#define GPIO_N0204_DISK1_DETECT		OX810_GPIO_4
#define GPIO_N0204_USB_COPY_BTN		OX810_GPIO_5
#define GPIO_N0204_POWER_ON_BTN		OX810_GPIO_9
#define GPIO_N0204_FACTORY_JUMPER	OX810_GPIO_10
#define GPIO_N0204_USB_PWR_EN		OX810_GPIO_23
#define GPIO_N0204_USB_PWR_ERR		OX810_GPIO_24
#define GPIO_N0204_POWER_OFF		OX810_GPIO_25
#define GPIO_N0204_DISK1_ACT_LED	OX810_GPIO_26
#define GPIO_N0204_DISK2_ACT_LED	OX810_GPIO_27
#define GPIO_N0204_THERMAL		OX810_GPIO_29
#define GPIO_N0204_DISK1_FAIL_LED	OX810_GPIO_33
#define GPIO_N0204_DISK2_FAIL_LED	OX810_GPIO_34
#endif

#if defined(CONFIG_THECUS_N2200_IO) || defined(CONFIG_THECUS_N2200_IO_MODULE)
#define GPIO_N2200_USB_COPY_SUCCESS_LED	OX810_GPIO_0
#define GPIO_N2200_DISK2_DETECT		OX810_GPIO_2
#define GPIO_N2200_POWER_ON_BTN		OX810_GPIO_3
#define GPIO_N2200_DISK1_DETECT		OX810_GPIO_4
#define GPIO_N2200_USB_COPY_FAIL_LED	OX810_GPIO_23
#define GPIO_N2200_FACTORY_JUMPER	OX810_GPIO_24
#define GPIO_N2200_POWER_OFF		OX810_GPIO_25
#define GPIO_N2200_DISK1_ACT_LED	OX810_GPIO_26
#define GPIO_N2200_DISK2_ACT_LED	OX810_GPIO_27
#define GPIO_N2200_RESET_BTN		OX810_GPIO_28
#define GPIO_N2200_THERMAL		OX810_GPIO_29
#define GPIO_N2200_FAN			OX810_GPIO_30
#define GPIO_N2200_LCM_BACKLIGHT_EN	OX810_GPIO_31
//#define GPIO_N2200_HARD_RESET		OX810_GPIO_31
#define GPIO_N2200_USB_COPY_BTN		OX810_GPIO_32
#define GPIO_N2200_DISK1_FAIL_LED	OX810_GPIO_33
#define GPIO_N2200_DISK2_FAIL_LED	OX810_GPIO_34
#endif

static __inline__ void ox810_gpio_set_input(u32 gpiox)
{
  u32 SWITCH_NUM;
  u32 SWITCH_PRISEL_REG;
  u32 SWITCH_SECSEL_REG;
  u32 SWITCH_TERSEL_REG;
  u32 SWITCH_CLR_OE_REG;
  u32 DATA_REG;
  u32 SWITCH_MASK;

  if(gpiox < 32) { // 0 -31
    SWITCH_NUM=gpiox;
    SWITCH_PRISEL_REG=SYS_CTRL_GPIO_PRIMSEL_CTRL_0;
    SWITCH_SECSEL_REG=SYS_CTRL_GPIO_SECSEL_CTRL_0;
    SWITCH_TERSEL_REG=SYS_CTRL_GPIO_TERTSEL_CTRL_0;
    SWITCH_CLR_OE_REG=GPIO_A_OUTPUT_ENABLE_CLEAR;
    DATA_REG=GPIO_A_DATA;
  } else if (gpiox < 35) { // 32 - 34
    SWITCH_NUM=gpiox - 32;
    SWITCH_PRISEL_REG=SYS_CTRL_GPIO_PRIMSEL_CTRL_1;
    SWITCH_SECSEL_REG=SYS_CTRL_GPIO_SECSEL_CTRL_1;
    SWITCH_TERSEL_REG=SYS_CTRL_GPIO_TERTSEL_CTRL_1;
    SWITCH_CLR_OE_REG=GPIO_B_OUTPUT_ENABLE_CLEAR;
    DATA_REG=GPIO_B_DATA;
  } else {
    printk(KERN_ERR "gpiox not support");
    return;
  }
  SWITCH_MASK=(1UL << (SWITCH_NUM));

  spin_lock(&oxnas_gpio_spinlock);
  // Disable primary, secondary and teriary GPIO functions on switch lines
  writel(readl(SWITCH_PRISEL_REG) & ~SWITCH_MASK, SWITCH_PRISEL_REG);
  writel(readl(SWITCH_SECSEL_REG) & ~SWITCH_MASK, SWITCH_SECSEL_REG);
  writel(readl(SWITCH_TERSEL_REG) & ~SWITCH_MASK, SWITCH_TERSEL_REG);

  // Enable GPIO input on switch line
  writel(SWITCH_MASK, SWITCH_CLR_OE_REG);
  spin_unlock(&oxnas_gpio_spinlock);
}

static __inline__ void ox810_gpio_set_output(int gpiox)
{
  u32 SWITCH_NUM;
  u32 SWITCH_PRISEL_REG;
  u32 SWITCH_SECSEL_REG;
  u32 SWITCH_TERSEL_REG;
  u32 SWITCH_SET_OE_REG;
  u32 DATA_REG;
  u32 SWITCH_MASK;

  if(gpiox < 32) { // 0 -31
    SWITCH_NUM=gpiox;
    SWITCH_PRISEL_REG=SYS_CTRL_GPIO_PRIMSEL_CTRL_0;
    SWITCH_SECSEL_REG=SYS_CTRL_GPIO_SECSEL_CTRL_0;
    SWITCH_TERSEL_REG=SYS_CTRL_GPIO_TERTSEL_CTRL_0;
    SWITCH_SET_OE_REG=GPIO_A_OUTPUT_ENABLE_SET;
    DATA_REG=GPIO_A_DATA;
  } else if (gpiox < 35) { // 32 - 34
    SWITCH_NUM=gpiox - 32;
    SWITCH_PRISEL_REG=SYS_CTRL_GPIO_PRIMSEL_CTRL_1;
    SWITCH_SECSEL_REG=SYS_CTRL_GPIO_SECSEL_CTRL_1;
    SWITCH_TERSEL_REG=SYS_CTRL_GPIO_TERTSEL_CTRL_1;
    SWITCH_SET_OE_REG=GPIO_B_OUTPUT_ENABLE_SET;
    DATA_REG=GPIO_B_DATA;
  } else {
    printk(KERN_ERR "gpiox not support");
    return;
  }
  SWITCH_MASK= (1UL << (SWITCH_NUM));

  spin_lock(&oxnas_gpio_spinlock);
  // Disable primary, secondary and teriary GPIO functions on switch lines
  writel(readl(SWITCH_PRISEL_REG) & ~SWITCH_MASK, SWITCH_PRISEL_REG);
  writel(readl(SWITCH_SECSEL_REG) & ~SWITCH_MASK, SWITCH_SECSEL_REG);
  writel(readl(SWITCH_TERSEL_REG) & ~SWITCH_MASK, SWITCH_TERSEL_REG);

  // Enable GPIO output on switch line
  writel(SWITCH_MASK, SWITCH_SET_OE_REG);
  spin_unlock(&oxnas_gpio_spinlock);
}

static __inline__ void ox810_gpio_write_bit(int gpiox, int value)
{
  u32 SWITCH_NUM;
  u32 DATA_REG;
  u32 SWITCH_MASK;

  if(gpiox < 32) { // 0 -31
    SWITCH_NUM=gpiox;
    if(value==1)
        DATA_REG=GPIO_A_OUTPUT_SET;
    else
        DATA_REG=GPIO_A_OUTPUT_CLEAR;
  } else if (gpiox < 35) { // 32 - 34
    SWITCH_NUM=gpiox - 32;
    if(value==1)
        DATA_REG=GPIO_B_OUTPUT_SET;
    else
        DATA_REG=GPIO_B_OUTPUT_CLEAR;
  } else {
    printk(KERN_ERR  "gpiox not support");
    return;
  }
  SWITCH_MASK= (1UL << (SWITCH_NUM));

  spin_lock(&oxnas_gpio_spinlock);
  writel(SWITCH_MASK, DATA_REG);
  spin_unlock(&oxnas_gpio_spinlock);
}

static __inline__ u32 ox810_gpio_read_bit(int gpiox)
{
  u32 SWITCH_NUM;
  u32 DATA_REG;
  u32 SWITCH_MASK;
  u32 value;

  if(gpiox < 32) { // 0 -31
    SWITCH_NUM=gpiox;
    DATA_REG=GPIO_A_DATA;
  } else if (gpiox < 35) { // 32 - 34
    SWITCH_NUM=gpiox - 32;
    DATA_REG=GPIO_B_DATA;
  } else {
    printk(KERN_ERR  "gpiox not support");
    return 0;
  }
  SWITCH_MASK= (1UL << (SWITCH_NUM));

  spin_lock(&oxnas_gpio_spinlock);
  value = (readl(DATA_REG) & SWITCH_MASK) ? 1 : 0;
  spin_unlock(&oxnas_gpio_spinlock);

  return value;
}


#endif // __THECUS_IO_H
