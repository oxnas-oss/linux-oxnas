/*
 * drivers/i2c/chips/m41t00.c
 *
 * I2C client/driver for the ST M41T00 Real-Time Clock chip.
 *
 * Author: Mark A. Greer <mgreer@mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
/*
 * This i2c client/driver wedges between the drivers/char/genrtc.c RTC
 * interface and the SMBus interface of the i2c subsystem.
 * It would be more efficient to use i2c msgs/i2c_transfer directly but, as
 * recommened in .../Documentation/i2c/writing-clients section
 * "Sending and receiving", using SMBus level communication is preferred.
 * 
 * Modified to support the ARM mechanism for interfacing between rtctime and  I2C bus.
 * JJL 8 June 2006
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/rtc.h>
#include <linux/bcd.h>

#include <asm/mach/time.h>
#include <asm/rtc.h>

//#define RTC_DEBUG

#define	M41T00_DRV_NAME		"m41t00"

static DECLARE_MUTEX(m41t00_mutex);

static struct i2c_driver m41t00_driver;
static struct i2c_client *save_client;

#if 0 /* CONFIG_ARCH_OXNAS -keep for multiple I2C devices */
	/** Chip select - get this out of here and into OXNAS specific sources */
	writel((0x00000001 << I2C_GPIO_SCS), RPS_GPIO_OUTPUT_SET);
	writel((0x00000001 << I2C_GPIO_SCS), RPS_GPIO_OUTPUT_ENABLE_SET);
#endif // CONFIG_ARCH_OXNAS

static unsigned short ignore[] = { I2C_CLIENT_END };
static unsigned short normal_addr[] = { 0x68, I2C_CLIENT_END };

static struct i2c_client_address_data addr_data = {
	.normal_i2c		= normal_addr,
	.probe			= ignore,
	.ignore			= ignore,
};


int m41t00_get_rtc_time(struct rtc_time *now)
{
	s32	sec, min, hour, day, mon, year;
	s32	sec1, min1, hour1, day1, mon1, year1;
	ulong	limit = 10;

	sec = min = hour = day = mon = year = 0;
	sec1 = min1 = hour1 = day1 = mon1 = year1 = 0;

	down(&m41t00_mutex);
	do {
		
		if (((sec = i2c_smbus_read_byte_data(save_client, 0)) >= 0)
			&& ((min = i2c_smbus_read_byte_data(save_client, 1))
				>= 0)
			&& ((hour = i2c_smbus_read_byte_data(save_client, 2))
				>= 0)
			&& ((day = i2c_smbus_read_byte_data(save_client, 4))
				>= 0)
			&& ((mon = i2c_smbus_read_byte_data(save_client, 5) - 1)
				>= 0)
			&& ((year = i2c_smbus_read_byte_data(save_client, 6))
				>= 0)
			&& ((sec == sec1) && (min == min1) && (hour == hour1)
				&& (day == day1) && (mon == mon1)
				&& (year == year1)))

				break;

		sec1 = sec;
		min1 = min;
		hour1 = hour;
		day1 = day;
		mon1 = mon;
		year1 = year;
	} while (--limit > 0);
	up(&m41t00_mutex);
#ifdef RTC_DEBUG
	printk(KERN_INFO "time: %x, %x, %x, \ndate: %x, %x, %x\n",
		sec, min, hour,
		day, mon, year);
#endif 
	if (limit == 0) {
		dev_warn(&save_client->dev,
			"m41t00: can't read rtc chip\n");
		sec = min = hour = day = mon = year = 0;
	}

	sec &= 0x7f;
	min &= 0x7f;
	hour &= 0x3f;
	day &= 0x3f;
	mon &= 0x1f;
	year &= 0xff;

	BCD_TO_BIN(sec);
	BCD_TO_BIN(min);
	BCD_TO_BIN(hour);
	BCD_TO_BIN(day);
	BCD_TO_BIN(mon);
	BCD_TO_BIN(year);

	if (year < 70) year+=100;

	now->tm_sec = sec;
	now->tm_min = min;
	now->tm_hour = hour;
	now->tm_mday = day;
	now->tm_mon = mon;
	now->tm_year = year;

#ifdef RTC_DEBUG
	printk(KERN_INFO "to system time: %x, %x, %x, \ndate: %x, %x, %x\n",
		sec, min, hour,
		day, mon, year);
#endif 

	return 0 ;
}


int m41t00_set_rtc_time(struct rtc_time *now)
{
	struct rtc_time	tm = *now;

#ifdef RTC_DEBUG
	printk(KERN_INFO "from system time: %x, %x, %x, \ndate: %x, %x, %x\n",
		tm.tm_sec, tm.tm_min, tm.tm_hour,
		tm.tm_mday, tm.tm_mon, tm.tm_year);
#endif 
	tm.tm_year %= 100;
	
	BIN_TO_BCD(tm.tm_sec);
	BIN_TO_BCD(tm.tm_min);
	BIN_TO_BCD(tm.tm_hour);
	BIN_TO_BCD(tm.tm_mon);
	BIN_TO_BCD(tm.tm_mday);
	BIN_TO_BCD(tm.tm_year);

#ifdef RTC_DEBUG
	printk(KERN_INFO "time: %x, %x, %x, \ndate: %x, %x, %x\n",
		tm.tm_sec, tm.tm_min, tm.tm_hour,
		tm.tm_mday, tm.tm_mon, tm.tm_year);
#endif 

	down(&m41t00_mutex);
	if ((i2c_smbus_write_byte_data(save_client, 0, tm.tm_sec & 0x7f) < 0)
		|| (i2c_smbus_write_byte_data(save_client, 1, tm.tm_min & 0x7f)
			< 0)
		|| (i2c_smbus_write_byte_data(save_client, 2, tm.tm_hour & 0x7f)
			< 0)
		|| (i2c_smbus_write_byte_data(save_client, 4, tm.tm_mday & 0x7f)
			< 0)
		|| (i2c_smbus_write_byte_data(save_client, 5, (tm.tm_mon + 1) & 0x7f)
			< 0)
		|| (i2c_smbus_write_byte_data(save_client, 6, tm.tm_year & 0x7f)
			< 0))

		dev_warn(&save_client->dev,"m41t00: can't write to rtc chip\n");

	up(&m41t00_mutex);
	

	return 0;
}

/*
 *****************************************************************************
 *
 *	Driver Interface
 *
 *****************************************************************************
 */
#if CONFIG_ARCH_OXNAS 
struct rtc_ops m41t00_ops = {
	.owner = THIS_MODULE,
	.read_time = m41t00_get_rtc_time,
	.set_time = m41t00_set_rtc_time,
};

#endif // CONFIG_ARCH_OXNAS




static int
m41t00_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *client;
	int rc;

	client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	strncpy(client->name, M41T00_DRV_NAME, I2C_NAME_SIZE);
	client->addr = addr;
	client->adapter = adap;
	client->driver = &m41t00_driver;

	if ((rc = i2c_attach_client(client)) != 0) {
		kfree(client);
		return rc;
	}

	register_rtc(&m41t00_ops);
	
	save_client = client;
	return 0;
}

static int
m41t00_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, m41t00_probe);
}

static int
m41t00_detach(struct i2c_client *client)
{
	int	rc;
	unregister_rtc(&m41t00_ops);

	if ((rc = i2c_detach_client(client)) == 0) {
		kfree(client);
	}
	return rc;
}

static struct i2c_driver m41t00_driver = {
	.id		= I2C_DRIVERID_STM41T00,
	.attach_adapter	= m41t00_attach,
	.detach_client	= m41t00_detach,
};

static int __init
m41t00_init(void)
{
	return i2c_add_driver(&m41t00_driver);
}

static void __exit
m41t00_exit(void)
{
	i2c_del_driver(&m41t00_driver);
	return;
}

module_init(m41t00_init);
module_exit(m41t00_exit);

MODULE_AUTHOR("Mark A. Greer <mgreer@mvista.com>");
MODULE_DESCRIPTION("ST Microelectronics M41T00 RTC I2C Client Driver");
MODULE_LICENSE("GPL");
