/*
 *  Copyright (C) 2008 Thecus Technology Corp.
 *
 *      Maintainer: citizen <citizen_lee@thecus.com>
 *
 *      Driver for io (include led, jumper ...) on Thecus N0204/N2200
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/arch/thecus_io.h>

MODULE_AUTHOR("citizen Lee <citizen_lee@thecus.com>");
MODULE_DESCRIPTION("Thecus N0204/N2200 MB Driver for board copy button operation");
MODULE_LICENSE("GPL");

#ifdef GPIO_N0204_USB_COPY_BTN
#if (GPIO_N0204_USB_COPY_BTN < 32)
#define SWITCH_NUM          GPIO_N0204_USB_COPY_BTN
#define IRQ_NUM             GPIO_1_INTERRUPT
#define INT_STATUS_REG      GPIO_A_INTERRUPT_STATUS_REGISTER
#define SWITCH_PRISEL_REG   SYS_CTRL_GPIO_PRIMSEL_CTRL_0
#define SWITCH_SECSEL_REG   SYS_CTRL_GPIO_SECSEL_CTRL_0
#define SWITCH_TERSEL_REG   SYS_CTRL_GPIO_TERTSEL_CTRL_0
#define SWITCH_CLR_OE_REG   GPIO_A_OUTPUT_ENABLE_CLEAR
#define DEBOUNCE_REG        GPIO_A_INPUT_DEBOUNCE_ENABLE
#define LEVEL_INT_REG       GPIO_A_LEVEL_INTERRUPT_ENABLE
#define FALLING_INT_REG     GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE
#define RISING_INT_REG	    GPIO_A_RISING_EDGE_ACTIVE_HIGH_ENABLE
#define DATA_REG            GPIO_A_DATA
#else
#define SWITCH_NUM          ((GPIO_N0204_USB_COPY_BTN) - 32)
#define IRQ_NUM             GPIO_2_INTERRUPT
#define INT_STATUS_REG      GPIO_B_INTERRUPT_STATUS_REGISTER
#define SWITCH_PRISEL_REG   SYS_CTRL_GPIO_PRIMSEL_CTRL_1
#define SWITCH_SECSEL_REG   SYS_CTRL_GPIO_SECSEL_CTRL_1
#define SWITCH_TERSEL_REG   SYS_CTRL_GPIO_TERTSEL_CTRL_1
#define SWITCH_CLR_OE_REG   GPIO_B_OUTPUT_ENABLE_CLEAR
#define DEBOUNCE_REG        GPIO_B_INPUT_DEBOUNCE_ENABLE
#define LEVEL_INT_REG       GPIO_B_LEVEL_INTERRUPT_ENABLE
#define FALLING_INT_REG     GPIO_B_FALLING_EDGE_ACTIVE_LOW_ENABLE
#define RISING_INT_REG	    GPIO_B_RISING_EDGE_ACTIVE_HIGH_ENABLE
#define DATA_REG            GPIO_B_DATA
#endif
#endif

#ifdef GPIO_N2200_USB_COPY_BTN
#if (GPIO_N2200_USB_COPY_BTN < 32)
#define SWITCH_NUM          GPIO_N2200_USB_COPY_BTN
#define IRQ_NUM             GPIO_1_INTERRUPT
#define INT_STATUS_REG      GPIO_A_INTERRUPT_STATUS_REGISTER
#define SWITCH_PRISEL_REG   SYS_CTRL_GPIO_PRIMSEL_CTRL_0
#define SWITCH_SECSEL_REG   SYS_CTRL_GPIO_SECSEL_CTRL_0
#define SWITCH_TERSEL_REG   SYS_CTRL_GPIO_TERTSEL_CTRL_0
#define SWITCH_CLR_OE_REG   GPIO_A_OUTPUT_ENABLE_CLEAR
#define DEBOUNCE_REG        GPIO_A_INPUT_DEBOUNCE_ENABLE
#define LEVEL_INT_REG       GPIO_A_LEVEL_INTERRUPT_ENABLE
#define FALLING_INT_REG     GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE
#define RISING_INT_REG	    GPIO_A_RISING_EDGE_ACTIVE_HIGH_ENABLE
#define DATA_REG            GPIO_A_DATA
#else
#define SWITCH_NUM          ((GPIO_N2200_USB_COPY_BTN) - 32)
#define IRQ_NUM             GPIO_2_INTERRUPT
#define INT_STATUS_REG      GPIO_B_INTERRUPT_STATUS_REGISTER
#define SWITCH_PRISEL_REG   SYS_CTRL_GPIO_PRIMSEL_CTRL_1
#define SWITCH_SECSEL_REG   SYS_CTRL_GPIO_SECSEL_CTRL_1
#define SWITCH_TERSEL_REG   SYS_CTRL_GPIO_TERTSEL_CTRL_1
#define SWITCH_CLR_OE_REG   GPIO_B_OUTPUT_ENABLE_CLEAR
#define DEBOUNCE_REG        GPIO_B_INPUT_DEBOUNCE_ENABLE
#define LEVEL_INT_REG       GPIO_B_LEVEL_INTERRUPT_ENABLE
#define FALLING_INT_REG     GPIO_B_FALLING_EDGE_ACTIVE_LOW_ENABLE
#define RISING_INT_REG	    GPIO_B_RISING_EDGE_ACTIVE_HIGH_ENABLE
#define DATA_REG            GPIO_B_DATA
#endif
#endif


#define SWITCH_MASK (1UL << (SWITCH_NUM))

#define TIMER_INTERVAL_JIFFIES  ((HZ) >> 3) /* An eigth of a second */
#define TIMER_COUNT_LIMIT       3           /* In eigths of a second */

extern spinlock_t oxnas_gpio_spinlock;

static unsigned long count;
static struct timer_list timer;

/** Have to use active low level interupt generation, as otherwise might miss
 *  interrupts that arrive concurrently with a PCI interrupt, as PCI interrupts
 *  are generated via GPIO pins and std PCI drivers will not know that there
 *  may be other pending GPIO interrupt sources waiting to be serviced and will
 *  simply return IRQ_HANDLED if they see themselves as having generated the
 *  interrupt, thus preventing later chained handlers from being called
 */
static irqreturn_t int_handler(int irq, void* dev_id)
{
	int status = IRQ_NONE;
	unsigned int int_status = readl((volatile unsigned long *)INT_STATUS_REG);

	/* Is the interrupt for us? */
	if (int_status & SWITCH_MASK) {
		/* Disable the copy button GPIO line interrupt */
		spin_lock(&oxnas_gpio_spinlock);
		writel(readl(RISING_INT_REG) & ~SWITCH_MASK, RISING_INT_REG);
		spin_unlock(&oxnas_gpio_spinlock);

		/* Zeroise button hold down counter */
		count = 0;

		/* Start hold down timer with a timeout of 1/8 second */
		mod_timer(&timer, jiffies + TIMER_INTERVAL_JIFFIES);

		/* Only mark interrupt as serviced if no other unmasked GPIO interrupts
		are pending */
		if (!readl((volatile unsigned long *)INT_STATUS_REG)) {
			status = IRQ_HANDLED;
		}
	}

	return status;
}

/*
 * Device driver object
 */
typedef struct copy_button_driver_s {
	/** sysfs dir tree root for copy button driver */
	struct kset kset;
	struct kobject copy_button;
} copy_button_driver_t;

static copy_button_driver_t copy_button_driver;

static void work_handler(struct work_struct * not_used) {
	kobject_uevent(&copy_button_driver.copy_button, KOBJ_OFFLINE);
}

DECLARE_WORK(copy_button_hotplug_work, work_handler);

static void timer_handler(unsigned long data)
{
	unsigned long flags;

	/* Is the copy button still pressed? */
	if ((readl(DATA_REG) & SWITCH_MASK)) {
printk("copy button count: %lu\n",count);
		/* Yes, so increment count of how many timer intervals have passed since
		copy button was pressed */
		if (++count == TIMER_COUNT_LIMIT) {
printk("copy button work\n");
			schedule_work(&copy_button_hotplug_work);

			mod_timer(&timer, jiffies + HZ);
		} else {
			/* Restart timer with a timeout of 1/8 second */
			mod_timer(&timer, jiffies + TIMER_INTERVAL_JIFFIES);
		}
	} else {
		/* The h/w debounced copy button has been released, so reenable the
		active low interrupt detection to trap the user's next attempt to
		copy down */
		spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
		writel(readl(RISING_INT_REG) | SWITCH_MASK, RISING_INT_REG);
		spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);
	}
}

static struct kobj_type ktype_copy_button = {
	.release = 0,
	.sysfs_ops = 0,
	.default_attrs = 0,
};

static int copy_button_hotplug_filter(struct kset* kset, struct kobject* kobj) {
	return get_ktype(kobj) == &ktype_copy_button;
}

static const char* copy_button_hotplug_name(struct kset* kset, struct kobject* kobj) {
	return "thecus_copy_button";
}

static struct kset_uevent_ops copy_button_uevent_ops = {
	.filter = copy_button_hotplug_filter,
	.name   = copy_button_hotplug_name,
	.uevent = NULL,
};

static int copy_button_prep_sysfs(void)
{
	int err = 0;

	/* prep the sysfs interface for use */
	kobject_set_name(&copy_button_driver.kset.kobj, "copy-button");
	copy_button_driver.kset.ktype = &ktype_copy_button;

	err = subsystem_register(&copy_button_driver.kset);
	if (err)
		return err;

	/* setup hotplugging */
	copy_button_driver.kset.uevent_ops = &copy_button_uevent_ops;

	/* setup the heirarchy, the name will be set on detection */
	kobject_init(&copy_button_driver.copy_button);
	copy_button_driver.copy_button.kset = kset_get(&copy_button_driver.kset);
	copy_button_driver.copy_button.parent = &copy_button_driver.kset.kobj;

	return 0;
}

static int copy_button_build_sysfs(void) {
	kobject_set_name(&copy_button_driver.copy_button, "copy-button-1");
	return kobject_add(&copy_button_driver.copy_button);
}

static int __init copy_button_init(void)
{
	int err = 0;
	unsigned long flags;

	err = copy_button_prep_sysfs();
	if (err)
		return -EINVAL;

	err = copy_button_build_sysfs();
	if (err)
		return -EINVAL;

	/* Setup the timer that will time how long the user holds down the copy
	button */
	init_timer(&timer);
	timer.data = 0;
	timer.function = timer_handler;

	/* Install a shared interrupt handler on the appropriate GPIO bank's
	interrupt line */
	if (request_irq(IRQ_NUM, int_handler, IRQF_SHARED, "Copy Button", &copy_button_driver)) {
		printk(KERN_ERR "Copy Button: cannot register IRQ %d\n", IRQ_NUM);
		del_timer_sync(&timer);
		return -EIO;
	}

	spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
	/* Disable primary, secondary and teriary GPIO functions on switch lines */
	writel(readl(SWITCH_PRISEL_REG) & ~SWITCH_MASK, SWITCH_PRISEL_REG);
	writel(readl(SWITCH_SECSEL_REG) & ~SWITCH_MASK, SWITCH_SECSEL_REG);
	writel(readl(SWITCH_TERSEL_REG) & ~SWITCH_MASK, SWITCH_TERSEL_REG);

	/* Enable GPIO input on switch line */
	writel(SWITCH_MASK, SWITCH_CLR_OE_REG);

	/* Set up the copy button GPIO line for active high, debounced interrupt */
	writel(readl(DEBOUNCE_REG)    | SWITCH_MASK, DEBOUNCE_REG);
	writel(readl(LEVEL_INT_REG)   | SWITCH_MASK, LEVEL_INT_REG);
	writel(readl(RISING_INT_REG) | SWITCH_MASK, RISING_INT_REG);
	spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

	printk(KERN_INFO "Thecus Copy button driver registered\n");
	return 0;
}

static void __exit copy_button_exit(void)
{
	unsigned long flags;

	kobject_del(&copy_button_driver.copy_button);
	subsystem_unregister(&copy_button_driver.kset);

	/* Deactive the timer */
	del_timer_sync(&timer);

	/* Disable interrupt generation by the copy button GPIO line */
	spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
	writel(readl(RISING_INT_REG) & ~SWITCH_MASK, RISING_INT_REG);
	spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

	/* Remove the handler for the shared interrupt line */
	free_irq(IRQ_NUM, &copy_button_driver);
}

/**
 * macros to register intiialisation and exit functions with kernal
 */
module_init(copy_button_init);
module_exit(copy_button_exit);
