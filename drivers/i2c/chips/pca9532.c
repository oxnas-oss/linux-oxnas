/*
 *  Copyright (C) 2009 Thecus Technology Corp.
 *
 *      Written by Y.T. Lee (yt_lee@thecus.com)
 *
 *      based on linux/drivers/i2c/chips/rtc8564.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Driver for pca9532 chip on Thecus N2200
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <linux/seq_file.h>

#define LED_PSC0 0x2
#define LED_PWM0 0x3
#define LED_PSC1 0x4
#define LED_PWM1 0x5
#define LED_LS0 0x6
#define LED_LS1 0x7
#define LED_LS2 0x8
#define LED_LS3 0x9

#define LED_ON 0x1
#define LED_OFF 0x0
#define LED_BLINK1 0x2
#define LED_BLINK2 0x3

#define I2C_DRIVERID_PCA9532 0xf001

#define DEBUG
#ifdef DEBUG
# define _DBG(x, fmt, args...) do{ if (debug>=x) printk(KERN_DEBUG"%s: " fmt "\n", __FUNCTION__, ##args); } while(0);
#else
# define _DBG(x, fmt, args...) do { } while(0);
#endif

MODULE_AUTHOR("Maintainer: Citizen Lee <citizen_lee@thecus.com>");
MODULE_DESCRIPTION("Thecus N2200 board (pca9532) Driver");
MODULE_LICENSE("GPL");
static int debug = 2;
module_param(debug, int, S_IRUGO | S_IWUSR);

struct pca9532_data {
    struct i2c_client client;
    u8 ctrl;
};

static struct i2c_client *pca9532;

#define PCA9532_I2C_ID 0x60

static struct i2c_driver pca9532_driver;
static struct i2c_adapter *pca9532_adaptor;

static unsigned short ignore[] = { I2C_CLIENT_END };
static unsigned short pca9532_addr[] = { PCA9532_I2C_ID, I2C_CLIENT_END };

static struct i2c_client_address_data pca9532_addr_data = {
    .normal_i2c = pca9532_addr,
    .probe = ignore,
    .ignore = ignore,
};

/* return 0 for no error */
static int
pca9532_rw(struct i2c_adapter *adap, u8 reg_num, u8 * val, int wr)
{
    int ret;
    unsigned char ad[1];
    unsigned char data[2];
    struct i2c_msg ctrl_wr[1] = {
	{PCA9532_I2C_ID, 0, 2, data}
    };
    struct i2c_msg ctrl_rd[2] = {
	{PCA9532_I2C_ID, 0, 1, ad},
	{PCA9532_I2C_ID, I2C_M_RD, 1, data}
    };

    if (NULL == adap) {
	printk(KERN_INFO "pca9532_rw: i2c_adapter is NULL\n");
	return 1;
    }

    //_DBG(1, "pca9532_rw reg: %d, value: %d, %s\n", reg_num, *val,
	// (wr > 0) ? "write" : "read");
    if (wr) {			//Write
	data[0] = reg_num;
	data[1] = *val;

	ret = i2c_transfer(adap, ctrl_wr, 1);
	if (ret != 1) {
	    printk(KERN_INFO "pca9532_rw: cant write PCA9532 Reg#%02X\n",
		   reg_num);
	    return ret;
	}
    } else {			//Read
	data[0] = 0;
	data[1] = 0;
	ad[0] = reg_num;
	ret = i2c_transfer(adap, ctrl_rd, 2);
	if (ret != 2) {
	    printk(KERN_INFO "pca9532_rw: cant read PCA9532 Reg#%02X\n",
		   reg_num);
	    return ret;
	}
	*val = data[0];
    }
    return 0;
}

static int pca9532_attach(struct i2c_adapter *adap, int addr, int kind)
{
    int ret;
    struct i2c_client *new_client;
    struct pca9532_data *d;
    unsigned char data[16];
    unsigned char ad[1] = { 0x03 };
    struct i2c_msg ctrl_rd[2] = {
	{addr, 0, 1, ad},
	{addr, I2C_M_RD, 1, data}
    };

    d = kzalloc(sizeof(struct pca9532_data), GFP_KERNEL);
    if (!d) {
	ret = -ENOMEM;
	goto done;
    }
    new_client = &d->client;

    strlcpy(new_client->name, "PCA9532", I2C_NAME_SIZE);
    i2c_set_clientdata(new_client, d);
    new_client->addr = addr;
    new_client->adapter = adap;
    new_client->driver = &pca9532_driver;

    _DBG(1, "client=%p", new_client);

    /* read back ctrl1 and ctrl2 */
    data[0] = 0;
    data[1] = 0;
    ad[0] = 0x03;
    ret = i2c_transfer(new_client->adapter, ctrl_rd, 2);

    if (ret != 2) {
	printk(KERN_INFO "pca9532_attach: cant read ctrl\n");
	ret = -ENODEV;
	goto done;
    }

    if (data[0] != 0x80) {	//PCA9532 not found
	pca9532_adaptor = 0;
	ret = -ENODEV;
	goto done;
    }
    d->ctrl = data[0];

    pca9532 = new_client;
    pca9532_adaptor = adap;
    ret = i2c_attach_client(new_client);
  done:
    if (ret) {
	kfree(d);
    }
    return ret;
}

static int pca9532_probe(struct i2c_adapter *adap)
{
    int ret;
    _DBG(1, "Probing pca9532\n");
    ret = i2c_probe(adap, &pca9532_addr_data, pca9532_attach);
    _DBG(1, "Probe result=%X\n", ret);
    if (ret != 0) {
	ret = i2c_probe(adap, &pca9532_addr_data, pca9532_attach);
	_DBG(1, "Probe result=%X\n", ret);
    }
    return ret;
}

static int pca9532_detach(struct i2c_client *client)
{
    i2c_detach_client(client);
    kfree(i2c_get_clientdata(client));
    return 0;
}

static int
pca9532_command(struct i2c_client *client, unsigned int cmd, void *arg)
{

    _DBG(1, "cmd=%d", cmd);

    switch (cmd) {
    default:
	return -EINVAL;
    }
}

// beep use pin LED15
void buzzer_control(u8 action)
{
    u8 val1, val2;
    u8 offset = 6; //(15-3*4)*2
    val1 = action;
    val1 = (val1 << offset) & 0xC0;
    pca9532_rw(pca9532_adaptor, LED_LS3, &val2, 0);
    val2 &= ~(0x3 << offset);
    val1 |= val2;
    pca9532_rw(pca9532_adaptor, LED_LS3, &val1, 1);
}

EXPORT_SYMBOL(buzzer_control);

u8 get_buzzer(void)
{
    u8 val1, val2 = 1;
    u8 offset = 6; //(15-3*4)*2
    if (!pca9532_rw(pca9532_adaptor, LED_LS3, &val1, 0)) {
	val2 = (val1 >> offset) & 0x3;
    }
    return val2;
}

EXPORT_SYMBOL(get_buzzer);

// Power LED use pin LED12
void set_power_led(u8 action)
{
    u8 val1=0, val2;
    if (action == 0)
	val1 = 1;
    pca9532_rw(pca9532_adaptor, LED_LS3, &val2, 0);
    val2 &= ~(0x3);
    val1 |= val2;
    pca9532_rw(pca9532_adaptor, LED_LS3, &val1, 1);
}

EXPORT_SYMBOL(set_power_led);

u8 get_power_led(void)
{
    u8 val1, val2 = 1;
    if (!pca9532_rw(pca9532_adaptor, LED_LS3, &val1, 0)) {
	val2 = val1 & 0x3;
    }
    return (val2+1)%2;
}

EXPORT_SYMBOL(get_power_led);

// sys_restart use pin LED9
void sys_restart(void)
{
    u8 val1, val2;
    u8 offset = 2; //(9-2*4)*2
    pca9532_rw(pca9532_adaptor, LED_LS2, &val2, 0);
    val2 &= ~(0x3 << offset);
    val1 = (1 << offset);
    val1 |= val2;
    pca9532_rw(pca9532_adaptor, LED_LS2, &val1, 1);
    msleep(800);
    pca9532_rw(pca9532_adaptor, LED_LS2, &val2, 1);
}

EXPORT_SYMBOL(sys_restart);
/*
u8 get_lcm_light(void)
{
    u8 val1, val2 = 0;
    if (!pca9532_rw(pca9532_adaptor, LED_LS2, &val1, 0)) {
	val2 = val1 & 0x3;
    }
    return val2;
}

EXPORT_SYMBOL(get_lcm_light);
*/

static ssize_t
proc_pca9532_write(struct file *file, const char __user * buf,
		   size_t length, loff_t * ppos)
{
    char *buffer;
    int i, err, val, v1, v2;
    u8 val1, val2;

    if (!buf || length > PAGE_SIZE)
	return -EINVAL;

    buffer = (char *) __get_free_page(GFP_KERNEL);
    if (!buffer)
	return -ENOMEM;

    err = -EFAULT;
    if (copy_from_user(buffer, buf, length))
	goto out;

    err = -EINVAL;
    if (length < PAGE_SIZE)
	buffer[length] = '\0';
    else if (buffer[PAGE_SIZE - 1])
	goto out;

    /*
     * Usage: echo "buzzer 0|1"     > /proc/pca9532
     * Usage: echo "lcm_light 0|1"  > /proc/pca9532
     * Usage: echo "freq 1-2 1-152" > /proc/pca9532
     * Usage: echo "duty 1-2 1-256" > /proc/pca9532
     *
     */
    if (!strncmp(buffer, "S_LED", strlen("S_LED"))) {
	i = sscanf(buffer + strlen("S_LED"), "%d %d\n", &v1, &v2);
	if (i == 2)		//two input
	{
	    if (v2 == 0)	//input 0: want to turn off
		val = LED_OFF;
	    else if (v2 == 1)	//turn on
		val = LED_ON;
	    else
		val = LED_BLINK1;

	    val2 = (val << ((v1 - 1) * 2));
	    pca9532_rw(pca9532_adaptor, LED_LS0, &val1, 0);
	    val1 &= ~(0x3 << ((v1 - 1) * 2));
	    val2 |= val1;
	    pca9532_rw(pca9532_adaptor, LED_LS0, &val2, 1);
	}
    } else if (!strncmp(buffer, "lcm_light", strlen("lcm_light"))) {
	i = sscanf(buffer + strlen("lcm_light"), "%d\n", &val);
	if (i == 1)		//only one input
	{
	    if (val == 1)
		val1 = LED_ON;
	    else
		val1 = LED_OFF;

	    val1 = val1 & 0x03;
	    pca9532_rw(pca9532_adaptor, LED_LS2, &val2, 0);
	    val2 &= ~(0x03);
	    val1 |= val2;
	    pca9532_rw(pca9532_adaptor, LED_LS2, &val1, 1);
	}
    } else if (!strncmp(buffer, "buzzer", strlen("buzzer"))) {
	i = sscanf(buffer + strlen("buzzer"), "%d\n", &val);
	if (i == 1)		//only one input
	{
	    if (val == 1)	//input 0: want to turn off
		val1 = LED_ON;
	    else
		val1 = LED_OFF;

	    val1 = (val1 << 4) & 0x30;
	    pca9532_rw(pca9532_adaptor, LED_LS3, &val2, 0);
	    val2 &= ~(0x3 << 4);
	    val1 |= val2;
	    pca9532_rw(pca9532_adaptor, LED_LS3, &val1, 1);
	}
    } else if (!strncmp(buffer, "freq", strlen("freq"))) {
	i = sscanf(buffer + strlen("freq"), "%d %d\n", &v1, &v2);
	if (i == 2)		//two input
	{
	    if (v1 == 1)	//input 1: PSC0
		val = LED_PSC0;
	    else
		val = LED_PSC1;

	    val2 = (152 / v2) - 1;
	    _DBG(1, "port=0x%02X,val=0x%02X\n", val, val2);
	    pca9532_rw(pca9532_adaptor, val, &val2, 1);
	}
    } else if (!strncmp(buffer, "duty", strlen("duty"))) {
	i = sscanf(buffer + strlen("duty"), "%d %d\n", &v1, &v2);
	if (i == 2)		//two input
	{
	    if (v1 == 1)	//input 1: PWM0
		val1 = LED_PWM0;
	    else
		val1 = LED_PWM1;

	    val2 = (v2 * 256) / 100;
	    pca9532_rw(pca9532_adaptor, val1, &val2, 1);
	}
    } else;

    err = length;
  out:
    free_page((unsigned long) buffer);
    *ppos = 0;

    return err;
}

static int proc_pca9532_show(struct seq_file *m, void *v)
{
    int i;
    u8 val1, val2;
    char LED_STATUS[4][8];

    sprintf(LED_STATUS[LED_ON], "ON");
    sprintf(LED_STATUS[LED_OFF], "OFF");
    sprintf(LED_STATUS[LED_BLINK1], "BLINK");
    sprintf(LED_STATUS[LED_BLINK2], "-");

    if (pca9532_adaptor) {

	if (!pca9532_rw(pca9532_adaptor, LED_LS0, &val1, 0)) {
	    for (i = 0; i < 4; i++) {
		val2 = (val1 >> (i * 2)) & 0x3;
		seq_printf(m, "S_LED#%d: %s\n", i + 1, LED_STATUS[val2]);
	    }
	}
	if (!pca9532_rw(pca9532_adaptor, LED_LS2, &val1, 0)) {
	    val2 = val1 & 0x3;
	    seq_printf(m, "lcm_light: %s\n", LED_STATUS[val2]);
	}
	if (!pca9532_rw(pca9532_adaptor, LED_LS3, &val1, 0)) {
	    val2 = (val1 >> 4) & 0x3;
	    seq_printf(m, "buzzer: %s\n", val2 ? "ON" : "OFF");
	}
    }
    return 0;
}

static int proc_pca9532_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_pca9532_show, NULL);
}

static struct file_operations proc_pca9532_operations = {
    .open = proc_pca9532_open,
    .read = seq_read,
    .write = proc_pca9532_write,
    .llseek = seq_lseek,
    .release = single_release,
};

int pca9532_init_procfs(void)
{
    struct proc_dir_entry *pde;

    pde = create_proc_entry("pca9532", 0, NULL);
    if (!pde)
	return -ENOMEM;
    pde->proc_fops = &proc_pca9532_operations;

    return 0;
}

void pca9532_exit_procfs(void)
{
    remove_proc_entry("pca9532", NULL);
}

static struct i2c_driver pca9532_driver = {
    .driver = {
	       .name = "PCA9532",
	       },
    .id = I2C_DRIVERID_PCA9532,
    .attach_adapter = pca9532_probe,
    .detach_client = pca9532_detach,
    .command = pca9532_command
};

/*
 *	The various file operations we support.
 */

static __init int pca9532_init(void)
{
    int ret;
    if (pca9532_init_procfs()) {
	printk(KERN_ERR "pca9532: cannot create /proc/pca9532.\n");
	return -ENOENT;
    }
    if (debug > 0) {
	printk("Debug level=%d\n", debug);
	_DBG(1, "Debug statement: %d", debug);
    }

    printk("pca9532_init\n");
    ret = i2c_add_driver(&pca9532_driver);
    if (ret)
	return ret;

    return 0;
}

static __exit void pca9532_exit(void)
{
    pca9532_exit_procfs();
    i2c_del_driver(&pca9532_driver);
}

module_init(pca9532_init);
module_exit(pca9532_exit);
