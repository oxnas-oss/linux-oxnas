/*
 * linux/drivers/i2c/i2c-i2c_oxnas_bitbash.c
 *
 * Author: Abraham van der Merwe <abraham@2d3d.co.za>
 *
 * An I2C adapter driver for the 2d3D, Inc. StrongARM SA-1110
 * Development board (i2c_oxnas_bitbash).
 *
 * This source code is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <asm/hardware.h>
#include <asm/io.h>

#define I2C_OXNAS_BITBASH_I2C_SDA_OUT   (1UL << (CONFIG_OXNAS_I2C_SDA))
#define I2C_OXNAS_BITBASH_I2C_SCL_OUT   (1UL << (CONFIG_OXNAS_I2C_SCL))
#define	I2C_OXNAS_BB_PULSEWIDTH (40)
#define OPEN_COLLECTOR_CLOCK    1

extern spinlock_t oxnas_gpio_spinlock;

static void i2c_oxnas_bitbash_setsda(void *data,int state)
{
	//printk(KERN_INFO " $%si2c_oxnas_bitbash_setsda: %d\n",state?"G":"g",state);
	if (state) {
		// tristae as input to set line on bus
		writel(I2C_OXNAS_BITBASH_I2C_SDA_OUT, GPIO_A_OUTPUT_ENABLE_CLEAR);
	} else {
		// tristate as output (with latch to zero) to assert zero on the bus
		writel(I2C_OXNAS_BITBASH_I2C_SDA_OUT, GPIO_A_OUTPUT_CLEAR);
		writel(I2C_OXNAS_BITBASH_I2C_SDA_OUT, GPIO_A_OUTPUT_ENABLE_SET);
	}
}

static void i2c_oxnas_bitbash_setscl(void *data,int state)
{
#if OPEN_COLLECTOR_CLOCK
	//printk(KERN_INFO " $%si2c_oxnas_bitbash_setscl: %d\n",state?"Y":"y",state);
	if (state) {
		// tristae as input to set line on bus
		writel(I2C_OXNAS_BITBASH_I2C_SCL_OUT, GPIO_A_OUTPUT_ENABLE_CLEAR);
	} else {
		// tristate as output (with latch to zero) to assert zero on the bus
		writel(I2C_OXNAS_BITBASH_I2C_SCL_OUT, GPIO_A_OUTPUT_CLEAR);
		writel(I2C_OXNAS_BITBASH_I2C_SCL_OUT, GPIO_A_OUTPUT_ENABLE_SET);
	}
#else // driven clock
	//printk(KERN_INFO " $%si2c_oxnas_bitbash_setscl: %d\n",state?"Y":"y",state);
	if (state) {
		// tristae as input to set line on bus
		writel(I2C_OXNAS_BITBASH_I2C_SCL_OUT, GPIO_A_OUTPUT_SET);
		writel(I2C_OXNAS_BITBASH_I2C_SCL_OUT, GPIO_A_OUTPUT_ENABLE_CLEAR);
	} else {
		// tristate as output (with latch to zero) to assert zero on the bus
		writel(I2C_OXNAS_BITBASH_I2C_SCL_OUT, GPIO_A_OUTPUT_CLEAR);
		writel(I2C_OXNAS_BITBASH_I2C_SCL_OUT, GPIO_A_OUTPUT_ENABLE_SET);
	}

#endif
}

static int i2c_oxnas_bitbash_getsda(void *data)
{
	return ((readl(GPIO_A_DATA ) & I2C_OXNAS_BITBASH_I2C_SDA_OUT) != 0);
}

static int i2c_oxnas_bitbash_getscl(void *data)
{
	return ((readl(GPIO_A_DATA ) & I2C_OXNAS_BITBASH_I2C_SCL_OUT) != 0);
}

static struct i2c_algo_bit_data bit_i2c_oxnas_bitbash_data = {
	.setsda    = i2c_oxnas_bitbash_setsda,
	.setscl    = i2c_oxnas_bitbash_setscl,
	.getsda    = i2c_oxnas_bitbash_getsda,
	.getscl    = i2c_oxnas_bitbash_getscl,
	.udelay    = I2C_OXNAS_BB_PULSEWIDTH,
	.mdelay    = I2C_OXNAS_BB_PULSEWIDTH,
	.timeout   = HZ
};

static struct i2c_adapter i2c_oxnas_bitbash_ops = {
	.owner		= THIS_MODULE,
	.name		= "i2c_oxnas_bitbash adapter driver",
	.id		    = I2C_HW_B_OXNAS,
	.algo_data	= &bit_i2c_oxnas_bitbash_data,
};

static int __init i2c_oxnas_bitbash_init(void)
{
    unsigned long flags;
    unsigned long mask = I2C_OXNAS_BITBASH_I2C_SDA_OUT | I2C_OXNAS_BITBASH_I2C_SCL_OUT;

	printk(KERN_INFO "i2c_oxnas_bitbash_init: i2c OX800 driver init\n");

	/* Dedicate the GPIO over to i2c.
     * NOTE: This may be confusing, but we are not using the i2c core here we
     * are using bit-bashed GPIO, so we must disable the primary, secondary and
     * tertiary functions of the relevant GPIO pins
     */
    spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
    writel(readl(SYS_CTRL_GPIO_PRIMSEL_CTRL_0) & ~mask, SYS_CTRL_GPIO_PRIMSEL_CTRL_0);
    writel(readl(SYS_CTRL_GPIO_SECSEL_CTRL_0)  & ~mask, SYS_CTRL_GPIO_SECSEL_CTRL_0);
    writel(readl(SYS_CTRL_GPIO_TERTSEL_CTRL_0) & ~mask, SYS_CTRL_GPIO_TERTSEL_CTRL_0);
    spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

	i2c_oxnas_bitbash_setsda(NULL,1);
	i2c_oxnas_bitbash_setscl(NULL,1);
	return i2c_bit_add_bus(&i2c_oxnas_bitbash_ops);
}

static void __exit i2c_oxnas_bitbash_exit(void)
{
	printk(KERN_INFO "i2c_oxnas_bitbash_exit: i2c OX800 driver exit\n");
	i2c_oxnas_bitbash_setsda(NULL,1);
	i2c_oxnas_bitbash_setscl(NULL,1);
	i2c_bit_del_bus(&i2c_oxnas_bitbash_ops);
}

MODULE_AUTHOR ("Chris Ford <....oxsemni>");
MODULE_DESCRIPTION ("I2C-Bus adapter routines for i2c_oxnas_bitbash");
MODULE_LICENSE ("GPL");

module_init (i2c_oxnas_bitbash_init);
module_exit (i2c_oxnas_bitbash_exit);

