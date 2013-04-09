/*
 * Device driver for the i2c thermostat found on the iBook G4, Albook G4
 *
 * Copyright (C) 2003, 2004 Colin Leroy, Rasmus Rohde, Benjamin Herrenschmidt
 *
 * Documentation from
 * http://www.analog.com/UploadedFiles/Data_Sheets/115254175ADT7467_pra.pdf
 * http://www.analog.com/UploadedFiles/Data_Sheets/3686221171167ADT7460_b.pdf
 *
 */


#include <linux/rtc.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/errno.h>
//#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
//#include <linux/capability.h>
//#include <linux/slab.h>
//#include <linux/init.h>
//#include <linux/spinlock.h>
//#include <linux/smp_lock.h>
#include <linux/wait.h>
#include <linux/suspend.h>
#include <linux/kthread.h>
//#include <linux/moduleparam.h>
#include <linux/freezer.h>

#include <asm/io.h>
#include <asm/system.h>
#include <asm/sections.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>

#include <asm/arch/thecus_io.h>
#include "asm/arch-oxnas/taco.h"
#include "thermistorCalibration.h"

/******************************************************************************
 *                                                                            *
 *  delta t=dutyCycle.Period                                                *
 *                                                                            *
 *  PWM in  >----+    V'=  Vcc - Vt                                         *
 *               |           --------                                         *
 *              _|_             Vcc                                           *
 *            | | |                                                           *
 *             \| |   Rt=   delta t                                         *
 *              | |        -------------                                      *
 *              |\|         -C.ln(V')                                      *
 *              | |                                                           *
 *              | |\  delta t is discovered by the hardware which performs a  *
 *              !_! \ varies the PWM duty cycle until one is found that just  *
 *               |    trips the Vt threshold.                                 *
 *               |                                                            *
 *               |----------->  V in (Schmidt trigger @ Vt)                   *
 *               |                                                            *
 *            ___!___                                                         *
 *            _______  C(eg 100nF)                                        *
 *               |                                                            *
 *               |                                                            *
 *               |                                                            *
 *            ---+---                                                         *
 *                                                                            *
 *  Steinhart Thermistor approximation:                                       *
 *                                                                            *
 *    1/T=A + B.ln(Rt) + C.ln(Rt)^3                                         *
 *                                                                            *
 ******************************************************************************/

/* This is not absolute temperature but counter val - thermistorCalibration*/


MODULE_AUTHOR("Thecus Simon");

MODULE_DESCRIPTION("Driver for Temperature sense on N0204 mini NAS");

MODULE_LICENSE("GPL");

#define CONFIG_GPIO_THERMAL GPIO_N0204_THERMAL

struct thermostat {
    int temps;
};

static struct thermostat *thermostat = NULL;
static struct task_struct *thread_therm = NULL;


static int write_reg(int reg, u32 data)
{
    writel(data, reg);
    return 0;
}

static int read_reg(int reg)
{
    int data = 0;
    data = readl(reg);
    return data;
}

/** GetTemperatureCounter is an internal function to the module that reads the
 * temperature as a counter value and returns it to the caller. The counter
 * value is used in all the internal calculations avoiding the necessity to
 * convert to Kelvin for interpretation. For corresponding temperature values
 * in Kelvin look at thermistoCalilbration.h
 */
int GetTemperatureCounter(void)
{
    u32 res;
    if (!thermostat) {
	printk(KERN_INFO "T&F?::$RERROR - Temperature conv not started\n");
	return -1;
    }
    while (!
	   (read_reg(TACHO_THERMISTOR_CONTROL) &
	    (1 << TACHO_THERMISTOR_CONTROL_THERM_VALID))) {
	printk(KERN_INFO
	       "T&F?::$rWarning - Temperature reading not stabalised\n");
	msleep(100);
    }
    res =
	read_reg(TACHO_THERMISTOR_RC_COUNTER) &
	TACHO_THERMISTOR_RC_COUNTER_MASK;
    return res;
}

/** This function converts the unit of the temperature from counts to
 * temperature in Kelvin
 */
int Convert2C(int tempCount)
{
    u32 res, arrayIndex;
    /* Convert the Counter Value to Temperature in degree C */
    arrayIndex = tempCount / THERM_INTERPOLATION_STEP;
    res = TvsCnt[arrayIndex];
    if ((THERM_ENTRIES_IN_CALIB_TABLE - 2) > arrayIndex)
	res -=
	    (tempCount % THERM_INTERPOLATION_STEP) * (TvsCnt[arrayIndex] -
						      TvsCnt[arrayIndex +
							     1]) /
	    THERM_INTERPOLATION_STEP;
    else
	res -=
	    (tempCount % THERM_INTERPOLATION_STEP) *
	    (TvsCnt[THERM_ENTRIES_IN_CALIB_TABLE - 2] -
	     TvsCnt[THERM_ENTRIES_IN_CALIB_TABLE -
		    1]) / THERM_INTERPOLATION_STEP;
    res = res / 100 - 20;
    return res;
}

static void read_sensors(struct thermostat *th)
{
    u32 tempCount;
    if (!th) {
	printk(KERN_INFO
	       "thecus_therm::read_sensors $RTH NOT ESTABLISHED YET\n");
	return;
    }
    tempCount = GetTemperatureCounter();
    th->temps = Convert2C(tempCount);	//GetTemperatureCounter();
}

static int monitor_task(void *arg)
{
    struct thermostat *th = arg;
    while (!kthread_should_stop()) {
	if (unlikely(freezing(current)))
	    refrigerator();
	msleep_interruptible(2000);
	read_sensors(th);
    }
    return 0;
}

static int
oxsemi_therm_read(char *buf, char **start, off_t offset, int len, int *eof,
		  void *unused)
{
    u32 i = 0;
    u32 temperature = 0;

    while (i < 5) {
	read_sensors(thermostat);
	temperature += thermostat->temps;
	i++;
    }
    temperature /= 5;

    len = sprintf(buf, "Temp 1: %d\nFAN 1 RPM: 1000\n", temperature);
    *start = buf;
    return len;
}

static struct proc_dir_entry *proc_oxsemi_therm;

static int proc_thecus_hwm_show(struct seq_file *m, void *v)
{
    u32 i = 0;
    u32 temperature = 0;

    while (i < 5) {
	read_sensors(thermostat);
	temperature += thermostat->temps;
	i++;
    }
    temperature /= 5;

    seq_printf(m, "Temp 1: %d\n", temperature);
    seq_printf(m, "FAN 1 RPM: 1000\n");

    return 0;
}

static int proc_thecus_hwm_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_thecus_hwm_show, NULL);
}

static struct file_operations proc_thecus_hwm_operations = {
    .open = proc_thecus_hwm_open,
    .read = seq_read,
    .write = NULL,
    .llseek = seq_lseek,
    .release = single_release,
};

static int __init thecus_therm_init(void)
{
    struct thermostat *th;
    int rc;

    if (thermostat)
	return 0;

    read_reg(SYS_CTRL_RSTEN_CTRL);

    /* release fan/tacho from system reset */
    *((volatile unsigned long *) SYS_CTRL_RSTEN_CLR_CTRL) =
	(1UL << SYS_CTRL_RSTEN_MISC_BIT);

    /* Pull Down the GPIO 29 from the software */
    *((volatile unsigned long *) SYSCTRL_GPIO_PULLUP_CTRL_0) |=
	TEMP_TACHO_PULLUP_CTRL_VALUE;

    /* enable primary use */
    *((volatile unsigned long *) SYS_CTRL_GPIO_PRIMSEL_CTRL_0) |=
	(1UL << CONFIG_GPIO_THERMAL);

    /* disable secondary use */
    *((volatile unsigned long *) SYS_CTRL_GPIO_SECSEL_CTRL_0) &=
	~(1UL << CONFIG_GPIO_THERMAL);

    /* disable tertiary use */
    *((volatile unsigned long *) SYS_CTRL_GPIO_TERTSEL_CTRL_0) &=
	~(1UL << CONFIG_GPIO_THERMAL);

    read_reg(SYS_CTRL_RSTEN_CTRL);
    read_reg(SYS_CTRL_GPIO_PRIMSEL_CTRL_0);
    read_reg(SYS_CTRL_GPIO_SECSEL_CTRL_0);
    read_reg(SYS_CTRL_GPIO_TERTSEL_CTRL_0);

    th = (struct thermostat *) kmalloc(sizeof(struct thermostat),
				       GFP_KERNEL);

    if (!th)
	return -ENOMEM;

    memset(th, 0, sizeof(struct thermostat));

    rc = read_reg(TACHO_CLOCK_DIVIDER);
    if (rc < 0) {
	printk(KERN_ERR "thecus_therm: Thermostat failed to read config ");
	kfree(th);
	return -ENODEV;
    }

    /* Set the Tacho clock divider up */
    /*      printk(KERN_INFO "thecus_therm: Setting tacho core frequency divider to %d\n", TACHO_CORE_THERM_DIVIDER_VALUE);
     */

// ######################################################################################
    write_reg(TACHO_CLOCK_DIVIDER, TACHO_CORE_TACHO_DIVIDER_VALUE);

    // check tacho divider set correctly
    rc = read_reg(TACHO_CLOCK_DIVIDER);
    // Comparing a 10 bit value to a 32 bit return value
    if ((rc & TACHO_CORE_TACHO_DIVIDER_VALUE) !=
	TACHO_CORE_TACHO_DIVIDER_VALUE) {
	printk(KERN_ERR
	       "thermAndFan: Set Tacho Divider Value Failed readback:%d\n",
	       rc);
	kfree(th);
	return -ENODEV;
    }
// ######################################################################################


    write_reg(PWM_CLOCK_DIVIDER, PWM_CORE_CLK_DIVIDER_VALUE);
    printk(KERN_INFO "Thecus thermal driver registered\n");
    thermostat = th;
    // Start the thermister measuring
    write_reg(TACHO_THERMISTOR_CONTROL,
	      (1 << TACHO_THERMISTOR_CONTROL_THERM_ENABLE));

/*
    thread_therm = kthread_run(monitor_task, th, "kfand");
    if (thread_therm == ERR_PTR(-ENOMEM)) {
	printk(KERN_INFO "thecus_therm: Kthread creation failed\n");
	thread_therm = NULL;
	return -ENOMEM;
    }
*/

    proc_oxsemi_therm = create_proc_entry("hwm", 0, NULL);
    if (proc_oxsemi_therm) {
//      proc_oxsemi_therm->read_proc = oxsemi_therm_read;
	proc_oxsemi_therm->proc_fops = &proc_thecus_hwm_operations;
    } else {
	printk(KERN_ERR "thermal: unable to register /proc/hwm\n");
    }
    return 0;
}


static void __exit thecus_therm_exit(void)
{
/*
    if (thread_therm) {
	kthread_stop(thread_therm);
	thread_therm = NULL;
    }
*/
    remove_proc_entry("hwm", NULL);

    kfree(thermostat);
    thermostat = NULL;
    // return fan/tacho to system reset
    *((volatile unsigned long *) SYS_CTRL_RSTEN_SET_CTRL) |=
	(1UL << SYS_CTRL_RSTEN_MISC_BIT);
}


module_init(thecus_therm_init);
module_exit(thecus_therm_exit);


/* End of File */
