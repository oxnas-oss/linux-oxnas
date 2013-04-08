/*
 *  Copyright (C) 2008 Thecus Technology Corp.
 *
 *      Maintainer: citizen <citizen_lee@thecus.com>
 *
 *      Driver for io (include led, jumper ...) on Thecus N0204
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/rtc.h>		/* get the user-level API */
#include <linux/init.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/notifier.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/arch/thecus_io.h>

static DEFINE_MUTEX(thecus_io_mutex);
char MODELNAME[16] = "N0204";
u32 led_sata_fail_1 = 0;	// 0: off, 1: on, 2: blink
u32 led_sata_fail_2 = 0;	// 0: off, 1: on, 2: blink
u32 led_usb = 2;		// 0: off, 1: on, 2: blink
u32 led_usb_fail = 0;		// 0: off, 1: on, 2: blink
u32 led_blink_value = 0;

//#define DEBUG 1
#ifdef DEBUG
# define _DBG(x, fmt, args...) do{ if (DEBUG>=x) printk("%s: " fmt "\n", __FUNCTION__, ##args); } while(0);
#else
# define _DBG(x, fmt, args...) do { } while(0);
#endif

MODULE_AUTHOR("citizen Lee <citizen_lee@thecus.com>");
MODULE_DESCRIPTION("Thecus N0204 MB Driver for board depend io operation");
MODULE_LICENSE("GPL");
static int debug;;
module_param(debug, int, S_IRUGO | S_IWUSR);

#define LED_OFF 	0x0
#define LED_ON 		0x1
#define LED_BLINK 	0x2


static int sys_notify_reboot(struct notifier_block *nb,
			     unsigned long event, void *p)
{
    switch (event) {
    case SYS_RESTART:
	break;
    case SYS_HALT:
    case SYS_POWER_OFF:
	printk("Shutdown by N0204_POWER_OFF\n");
	ox810_gpio_write_bit(GPIO_N0204_POWER_OFF, 1);
	ox810_gpio_set_output(GPIO_N0204_POWER_OFF);
	msleep(800);
	ox810_gpio_write_bit(GPIO_N0204_POWER_OFF, 0);
	break;
    }
    return NOTIFY_DONE;
}

struct notifier_block sys_notifier_reboot = {
    .notifier_call = sys_notify_reboot,
    .next = NULL,
    .priority = 0
};


static ssize_t proc_thecus_io_write(struct file *file,
				    const char __user * buf, size_t length,
				    loff_t * ppos)
{
    char *buffer;
    int i, err, v1, v2;

    if (!buf || length > PAGE_SIZE)
	return -EINVAL;

    buffer = (char *) __get_free_page(GFP_KERNEL);
    if (!buffer)
	return -ENOMEM;

    err = -EFAULT;
    if (copy_from_user(buffer, buf, length))
	goto out;

    err = -EINVAL;
    if (length < PAGE_SIZE) {
	buffer[length] = '\0';
#define LF	0xA
	if (length > 0 && buffer[length - 1] == LF)
	    buffer[length - 1] = '\0';
    } else if (buffer[PAGE_SIZE - 1])
	goto out;

    // mutex for i2c command
    mutex_lock(&thecus_io_mutex);

    /*
     * Usage: echo "S_LED 1 0|1|2" >/proc/thecus_io //2:Blink   * LED SATA 1 Fail
     * Usage: echo "S_LED 2 0|1|2" >/proc/thecus_io //2:Blink   * LED SATA 2 Fail
     * Usage: echo "U_LED 0|1|2" >/proc/thecus_io //2:do as 1   * LED USB Success
     * Usage: echo "UF_LED 0|1|2" >/proc/thecus_io //2:do as 1  * LED USB Fail
     * Usage: echo "Fail 0|1" >/proc/thecus_io                  * LED System/RAID Fail
     * Usage: echo "Busy 0|1" >/proc/thecus_io                  * LED System Busy
     * Usage: echo "Buzzer 0|1" >/proc/thecus_io                * Buzzer, not used
     * Usage: echo "USB_EN 0|1" >/proc/thecus_io                * enable/disable front usb port
     */

    if (!strncmp(buffer, "S_LED", strlen("S_LED"))) {
	i = sscanf(buffer + strlen("S_LED"), "%d %d\n", &v1, &v2);
	if (i == 2)		//two input
	{
	    _DBG(1, "S_LED %d %d\n", v1, v2);
	    if (v1 == 1) {
		led_sata_fail_1 = v2;
		ox810_gpio_write_bit(GPIO_N0204_DISK1_FAIL_LED, v2);
	    } else if (v1 == 2) {
		led_sata_fail_2 = v2;
		ox810_gpio_write_bit(GPIO_N0204_DISK2_FAIL_LED, v2);
	    }
	}
    } else if (!strncmp(buffer, "U_LED", strlen("U_LED"))) {
	i = sscanf(buffer + strlen("U_LED"), "%d\n", &v1);
	if (i == 1)		//Only one input
	{
	    _DBG(1, "U_LED %d\n", v1);
	    led_usb = v1;
	    ox810_gpio_write_bit(GPIO_N0204_USB_COPY_SUCCESS_LED, v1);
	}
    } else if (!strncmp(buffer, "UF_LED", strlen("UF_LED"))) {
	i = sscanf(buffer + strlen("UF_LED"), "%d\n", &v1);
	if (i == 1)		//Only one input
	{
	    _DBG(1, "UF_LED %d\n", v1);
	    led_usb_fail = v1;
	    ox810_gpio_write_bit(GPIO_N0204_USB_COPY_FAIL_LED, v1);
	}
    } else if (!strncmp(buffer, "Busy", strlen("Busy"))) {
	i = sscanf(buffer + strlen("Busy"), "%d\n", &v1);
	if (i == 1)		//only one input
	{
	    _DBG(1, "Busy %d\n", v1);
	}
    } else if (!strncmp(buffer, "Fail", strlen("Fail"))) {
	i = sscanf(buffer + strlen("Fail"), "%d\n", &v1);
	if (i == 1)		//only one input
	{
	    _DBG(1, "Fail %d\n", v1);
	}
    } else if (!strncmp(buffer, "USB_EN", strlen("USB_EN"))) {
	i = sscanf(buffer + strlen("USB_EN"), "%d\n", &v1);
	if (i == 1)		//Only one input
	{
	    _DBG(1, "USB_EN %d\n", v1);
	    ox810_gpio_write_bit(GPIO_N0204_USB_PWR_EN, v1);
	}
    } else if (!strncmp(buffer, "MODELNAME", strlen("MODELNAME"))) {
	memset(MODELNAME, 0, sizeof(MODELNAME));
	strncpy(MODELNAME, buffer + strlen("MODELNAME") + 1, 16);
    }

    err = length;
    mutex_unlock(&thecus_io_mutex);

  out:
    free_page((unsigned long) buffer);
    *ppos = 0;

    return err;
}


static int proc_thecus_io_show(struct seq_file *m, void *v)
{
    seq_printf(m, "MODELNAME: %s\n", MODELNAME);

    seq_printf(m, "FAC_MODE: %s\n",
	       ox810_gpio_read_bit(GPIO_N0204_FACTORY_JUMPER) ? "OFF" :
	       "ON");

    seq_printf(m, "Buzzer: OFF\n");

    seq_printf(m, "DRAM: %s\n",
	       ox810_gpio_read_bit(GPIO_N0204_DRAM_VENDER) ? "1" : "0");

    seq_printf(m, "S_LED 1: %s\n",
	       ox810_gpio_read_bit(GPIO_N0204_DISK1_FAIL_LED) ? "1" :
	       "0");
    seq_printf(m, "S_LED 2: %s\n",
	       ox810_gpio_read_bit(GPIO_N0204_DISK2_FAIL_LED) ? "1" :
	       "0");

    seq_printf(m, "U_LED: %s\n",
	       ox810_gpio_read_bit(GPIO_N0204_USB_COPY_SUCCESS_LED) ? "1"
	       : "0");
    seq_printf(m, "UF_LED: %s\n",
	       ox810_gpio_read_bit(GPIO_N0204_USB_COPY_FAIL_LED) ? "1" :
	       "0");

    seq_printf(m, "USB_EN: %s\n",
	       ox810_gpio_read_bit(GPIO_N0204_USB_PWR_EN) ? "1" :
	       "0");

    seq_printf(m, "MAX_TRAY: %d\n", 2);
    seq_printf(m, "eSATA_TRAY: %d\n", -1);
    seq_printf(m, "WOL_FN: %d\n", 0);
    seq_printf(m, "FAN_FN: %d\n", 0);
    seq_printf(m, "BEEP_FN: %d\n", 0);
    seq_printf(m, "eSATA_FN: %d\n", 0);
    seq_printf(m, "MBTYPE: %d\n", 400);

    return 0;
}

static int proc_thecus_io_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_thecus_io_show, NULL);
}

static struct file_operations proc_thecus_io_operations = {
    .open = proc_thecus_io_open,
    .read = seq_read,
    .write = proc_thecus_io_write,
    .llseek = seq_lseek,
    .release = single_release,
};

// ----------------------------------------------------------
DECLARE_WAIT_QUEUE_HEAD(thecus_event_queue);
#define MESSAGE_LENGTH 80
static char Message[MESSAGE_LENGTH];
#define MY_WORK_QUEUE_NAME "btn_sched"	// length must < 10
#define WORK_QUEUE_TIMER_1 100
static u32 dyn_work_queue_timer = WORK_QUEUE_TIMER_1;
static void intrpt_routine(struct work_struct *unused);
static int module_die = 0;	/* set this to 1 for shutdown */
static struct workqueue_struct *my_workqueue;
static struct delayed_work Task;
static DECLARE_DELAYED_WORK(Task, intrpt_routine);

static void intrpt_routine(struct work_struct *unused)
{
//    u8 val;
//    static u32 interval = 0;

    if (led_sata_fail_1 == 2) {
	ox810_gpio_write_bit(GPIO_N0204_DISK1_FAIL_LED,
			     led_blink_value % 2);
    }
    if (led_sata_fail_2 == 2) {
	ox810_gpio_write_bit(GPIO_N0204_DISK2_FAIL_LED,
			     led_blink_value % 2);
    }
    if (led_usb == 2) {
	ox810_gpio_write_bit(GPIO_N0204_USB_COPY_SUCCESS_LED,
			     led_blink_value % 2);
    }
    if (led_usb_fail == 2) {
	ox810_gpio_write_bit(GPIO_N0204_USB_COPY_FAIL_LED,
			     led_blink_value % 2);
    }
    led_blink_value++;
/*
    if (1 == cs5535_gpio_read_bit(THECUS_N3200PLUS_PIC_INT)) {
	if (0 == pic16f877_get_regs(THECUS_PIC16F877_INT_STATUS, &val, 1)) {
	    interval++;
	    switch (val) {
	    case THECUS_PIC16F877_INT_POWER:
		sprintf(Message, "PWR Button: ON\n");
		wake_up_interruptible(&thecus_event_queue);
		break;
	    case THECUS_PIC16F877_INT_USB:
		sprintf(Message, "Copy Button: ON\n");
		wake_up_interruptible(&thecus_event_queue);
		break;
	    default:
		printk("Unknown pic16f877 interrupt status\n");
	    }

	    if (interval >= 100) {
		interval = 0;
		dyn_work_queue_timer = WORK_QUEUE_TIMER_1;
	    }
	}
    }
*/

    // If cleanup wants us to die
    if (module_die == 0)
	queue_delayed_work(my_workqueue, &Task, dyn_work_queue_timer);

}


static ssize_t thecus_event_read(struct file *file, char __user * buffer,
				 size_t length, loff_t * ppos)
{
    static int finished = 0;
    int i;
    if (finished) {
	finished = 0;
	return 0;
    }
//      printk(KERN_DEBUG "process %i (%s) going to sleep\n",
//           current->pid, current->comm);
    interruptible_sleep_on(&thecus_event_queue);
//      printk(KERN_DEBUG "awoken %i (%s)\n", current->pid, current->comm);
    for (i = 0; i < length && Message[i]; i++)
	put_user(Message[i], buffer + i);

    finished = 1;
    return i;
}

static struct file_operations proc_thecus_event_operations = {
    .read = thecus_event_read,
};

extern int register_reboot_notifier(struct notifier_block *);
extern int unregister_reboot_notifier(struct notifier_block *);

static __init int thecus_io_init(void)
{
    struct proc_dir_entry *pde;
    pde = create_proc_entry("thecus_io", 0, NULL);
    if (!pde) {
	printk(KERN_ERR "thecus_io: cannot create /proc/thecus_io.\n");
	return -ENOENT;
    }
    pde->proc_fops = &proc_thecus_io_operations;

    pde = create_proc_entry("thecus_event", S_IRUSR, NULL);
    if (!pde) {
	printk(KERN_ERR "thecus_io: cannot create /proc/thecus_event.\n");
	return -ENOENT;
    }
    pde->proc_fops = &proc_thecus_event_operations;


    ox810_gpio_set_output(GPIO_N0204_USB_COPY_SUCCESS_LED);
    ox810_gpio_set_output(GPIO_N0204_USB_COPY_FAIL_LED);
    ox810_gpio_set_output(GPIO_N0204_DISK1_FAIL_LED);
    ox810_gpio_set_output(GPIO_N0204_DISK2_FAIL_LED);
    ox810_gpio_set_output(GPIO_N0204_USB_PWR_EN);

    ox810_gpio_set_input(GPIO_N0204_FACTORY_JUMPER);
    ox810_gpio_set_input(GPIO_N0204_DRAM_VENDER);

    register_reboot_notifier(&sys_notifier_reboot);


    my_workqueue = create_workqueue(MY_WORK_QUEUE_NAME);
    queue_delayed_work(my_workqueue, &Task, dyn_work_queue_timer);
    init_waitqueue_head(&thecus_event_queue);

    printk(KERN_INFO "Thecus io driver registered\n");

    return 0;
}

static __exit void thecus_io_exit(void)
{
    module_die = 1;		// keep intrp_routine from queueing itself
    remove_proc_entry("thecus_io", NULL);
    remove_proc_entry("thecus_event", NULL);
    unregister_reboot_notifier(&sys_notifier_reboot);

    cancel_delayed_work(&Task);	// no "new ones"
    flush_workqueue(my_workqueue);	// wait till all "old ones" finished
    destroy_workqueue(my_workqueue);
}

module_init(thecus_io_init);
module_exit(thecus_io_exit);
