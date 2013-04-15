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
 

#include <linux/config.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/smp_lock.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/capability.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/smp_lock.h>
#include <linux/wait.h>
#include <linux/suspend.h>
#include <linux/kthread.h>
#include <linux/moduleparam.h>

#include <asm/io.h>
#include <asm/system.h>
#include <asm/sections.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>

#include "asm/arch-oxnas/taco.h"
#include "thermistorCalibration.h"

//#define DEBUG 1
#undef DEBUG

/******************************************************************************     
 *                                                                            *
 *  delta t = dutyCycle.Period                                                *
 *                                                                            *
 *  PWM in  >----+    V' =   Vcc - Vt                                         *
 *               |           --------                                         *
 *              _|_             Vcc                                           *
 *            | | |                                                           *
 *             \| |   Rt =    delta t                                         *
 *              | |        -------------                                      *
 *              |\|         -C.ln( V' )                                       *
 *              | |                                                           *
 *              | |\  delta t is discovered by the hardware which performs a  *   
 *              !_! \ varies the PWM duty cycle until one is found that just  *
 *               |    trips the Vt threshold.                                 *
 *               |                                                            *
 *               |----------->  V in (Schmidt trigger @ Vt)                   *
 *               |                                                            *
 *            ___!___                                                         *
 *            _______  C ( eg 100nF )                                         *
 *               |                                                            *
 *               |                                                            *
 *               |                                                            *
 *            ---+---                                                         *
 *                                                                            *
 *  Steinhart Thermistor approximation:                                       *
 *                                                                            *
 *    1/T = A + B.ln(Rt) + C.ln(Rt)^3                                         *
 *                                                                            *
 ******************************************************************************/           

#define OXSEMI_FAN_SPEED_HYSTERESIS     2
#define MAX_FAN_CHANGE              20
#define OXSEMI_FAN_SPEED_MIN		50
#define OXSEMI_FAN_SPEED_MAX		255
#define THERM_CHARGE_CAP_NF 		100
#define THERM_PWM_FREQ_HZ		128000
#define THERM_VCC_MV			3300
#define THERM_VSCH_MV			(THERM_VCC_MV / 2)
#define THERM_V_RATIO			((THERM_VCC_MV-THERM_VSCH_MV)/THERM_VCC_MV)

#define FAN_SPD_SET			(PWM_DATA_2)


enum COMMANDS
{
	SET_LIMIT_ADJUST,
	SET_ABS_LIMIT_C	
};

enum SETTINGS
{
	THERM_DEFAULT_LIMIT = 70,
};


// enum TERMISTOR_PART
// {
//     THERM_COEF_A  = 0,
//     THERM_COEF_B  = 1,
//     THERM_COEF_C  = 2,
//     
//     THERM_0_1K1A  = 0,
//     THERM_0_3K1A  = 1,
//     THERM_1K2A    = 2,
//     THERM_1K7A    = 3,
//     THERM_2K3A    = 4,
//     THERM_2_2K3A  = 5,
//     THERM_3K3A    = 6,
//     THERM_5K3A    = 7,
//     THERM_10K3A   = 8,
//     THERM_10K4A   = 9,
//     THERM_30K5A   = 10,
//     THERM_30K6A   = 11,
//     THERM_50K6A   = 12,
//     THERM_100K6A  = 13,
//     THERM_1M9A    = 14,
//     
//     THERM_MAX     = THERM_1M9A + 1
// };

// /* BetaTherm NTC thermistor Steinhart coefficients */
// static const double SteinhartCoef[THERM_MAX][3] = {
//     /*   Part No.          "A"            "B"            "C"        Temp reference points�C  */
//     /*   0.1K1A    */ { 1.942952e-3, 2.989769e-4, 3.504383e-7 }, /* -20�C  25�C  and 50�C    */
//     /*   0.3K1A    */ { 1.627660e-3, 2.933316e-4, 2.870016e-7 }, /* -20�C  25�C  and 50�C    */
//     /*   1K2A      */ { 1.373168e-3, 2.772261e-4, 1.997412e-7 }, /* -20�C  25�C  and 50�C    */
//     /*   1K7A      */ { 1.446059e-3, 2.683626e-4, 1.643561e-7 }, /* -20�C  25�C  and 50�C    */
//     /*   2K3A      */ { 1.498872e-3, 2.379047e-4, 1.066953e-7 }, /*   0�C  25�C  and 70�C    */
//     /*   2.2K3A    */ { 1.471388e-3, 2.376138e-4, 1.051058e-7 }, /*   0�C  25�C  and 70�C    */
//     /*   3K3A      */ { 1.405027e-3, 2.369386e-4, 1.012660e-7 }, /*   0�C  25�C  and 70�C    */
//     /*   5K3A      */ { 1.287450e-3, 2.357394e-4, 9.505200e-8 }, /*   0�C  25�C  and 70�C    */
//     /*   10K3A     */ { 1.129241e-3, 2.341077e-4, 8.775468e-8 }, /*   0�C  25�C  and 70�C    */
//     /*   10K4A     */ { 1.028444e-3, 2.392435e-4, 1.562216e-7 }, /*   0�C  25�C  and 70�C    */
//     /*   30K5A     */ { 9.331754e-4, 2.213978e-4, 1.263817e-7 }, /*   0�C  25�C  and 70�C    */
//     /*   30K6A     */ { 1.068981e-3, 2.120700e-4, 9.019537e-8 }, /*   0�C  25�C  and 70�C    */
//     /*   50K6A     */ { 9.657154e-4, 2.106840e-4, 8.585481e-8 }, /*   0�C  25�C  and 70�C    */
//     /*   100K6A    */ { 8.271111e-4, 2.088020e-4, 8.059200e-8 }, /*   0�C  25�C  and 70�C    */
//     /*   1M9A      */ { 7.402387e-4, 1.760865e-4, 6.865999e-8 }  /*   25�C 100�C and 150�C   */
// };


static int hot_limit = 50;
static int cold_limit = 200;

static int fan_speed                   	= 64; // -1; 
                                       	
// static enum TERMISTOR_PART thermType    = THERM_10K3A;
static const u32 default_limit          	= THERM_DEFAULT_LIMIT + DEGREES_C_0;


MODULE_AUTHOR(		"Chris Ford"					);
MODULE_DESCRIPTION(	"Driver for Fan and temp sense of ox800"		);
MODULE_LICENSE(		"GPL"						);

module_param(hot_limit, int, 0644);
MODULE_PARM_DESC(hot_limit, "Thermistor input for maximum fan drive");

module_param(cold_limit, int, 0644);
MODULE_PARM_DESC(cold_limit,"Thermistor input to start fan");

module_param(fan_speed, int, 0644);
MODULE_PARM_DESC(fan_speed,"Specify starting fan drive (0-255) (default 64)");

struct thermostat {
	int			temps;
	int			cached_temp;
	int			curr_speed;
	int			last_speed;
	int	        set_speed;
	int			last_var;
	struct semaphore 	sem;
//	struct cdev		cdev;
};

static struct thermostat* 	thermostat   = NULL;
static struct task_struct*	thread_therm = NULL;


static int write_reg( int reg, u32 data )
{
/*	printk(KERN_INFO "thermAndFan::write_reg [0x%08x] = 0x%08x\n", reg, data);
 * */
	__raw_writel( data, reg );
	return 0;
}

static int read_reg( int reg )
{
	int data = 0;
	data = __raw_readl( reg );
/*	printk(KERN_INFO "thermAndFan::read_reg [0x%08x] == 0x%08x\n", reg, data);
 */
	return data;
}


static void read_sensors(struct thermostat *th)
{
	static int state;
	if ( !th ) {
		printk(KERN_INFO "thermAndFan::read_sensors $RTH NOT ESTABLISHED YET\n");		
		return;
	}
	switch (state) {		
	case 0: /* Get the temperature */
		th->temps  = GetTemperature();
		write_reg( TACHO_CLOCK_DIVIDER, TACHO_CORE_TACHO_DIVIDER_VALUE );
		state = 1;
		break;
	case 1:
		/* Get the temperature */
		th->curr_speed  = GetFanRPM();
		/* Stop the thermister measuring */
		write_reg( TACHO_THERMISTOR_CONTROL, 0 );
		write_reg( TACHO_CLOCK_DIVIDER, TACHO_CORE_THERM_DIVIDER_VALUE );
		/* Start the thermister measuring */
		write_reg( TACHO_THERMISTOR_CONTROL, (1 << TACHO_THERMISTOR_CONTROL_THERM_ENABLE) );
		state = 0;
		break;
	default:
		write_reg( TACHO_THERMISTOR_CONTROL, 0 );
		write_reg( TACHO_CLOCK_DIVIDER, TACHO_CORE_THERM_DIVIDER_VALUE );
		/* Start the thermister measuring */
		write_reg( TACHO_THERMISTOR_CONTROL, (1 << TACHO_THERMISTOR_CONTROL_THERM_ENABLE) );
		state = 0;
		break;
	}
}


#if DEBUT
/**
 * DumpTachoRegisters is a debug function used to inspect hte tacho registers.
 */
void DumpTachoRegisters(void)
{

	printk(KERN_INFO \
		"\n<Taco Registers> ---------------------------------\n"
		"      TACHO_FAN_SPEED_COUNTER          == 0x%08x\n"
		"      TACHO_THERMISTOR_RC_COUNTER      == 0x%08x\n"
		"      TACHO_THERMISTOR_CONTROL         == 0x%08x\n"
		"      TACHO_CLOCK_DIVIDER              == 0x%08x\n"
		"      PWM_CORE_CLK_DIVIDER_VALUE       == 0x%08x\n"
		"      FAN_SPD_SET                      == 0x%08x\n"
		"<\\Taco Registers> --------------------------------\n\n",
		(u32) __raw_readl( TACHO_FAN_SPEED_COUNTER	 ),	
		(u32) __raw_readl( TACHO_THERMISTOR_RC_COUNTER   ),	
		(u32) __raw_readl( TACHO_THERMISTOR_CONTROL	 ),	
		(u32) __raw_readl( TACHO_CLOCK_DIVIDER	 	 ),	
		(u32) __raw_readl( PWM_CLOCK_DIVIDER       	 ),	
		(u32) __raw_readl( FAN_SPD_SET 			 ) );	

	read_reg(RPS_GPIO_OUTPUT_ENABLE);
	read_reg(RPS_GPIO_OUTPUT_VALUE);
	read_reg(RPS_GPIO_DATA);

}
#else
void DumpTachoRegisters(void) {}
#endif

/**
 * Count2Temp is a conversion routine that turns the count value into a temperature
 * in degrees Kelvin
 * @param iCount is a u32 that details the count of periods that the circuit has
 * charged for.
 * @return the temperature value in degrees K.
 */
int Count2Temp( u32 iCount )
{
	u32 temp;
	int rem = (iCount % THERM_INTERPOLATION_STEP) - (THERM_INTERPOLATION_STEP / 2);
	iCount /= THERM_INTERPOLATION_STEP;
	
	// trunc the table lookup
	if ( iCount > THERM_ENTRIES_IN_CALIB_TABLE )
	{
		iCount = THERM_ENTRIES_IN_CALIB_TABLE - 1;
	}
	
	temp = TvsCnt[iCount];
	
	// adjust icount for interpolation.
	if ( rem > 0  &&  iCount < THERM_ENTRIES_IN_CALIB_TABLE - 1 )
	{
		rem  *= TvsCnt[++iCount] - temp;
		rem  /= THERM_INTERPOLATION_STEP / 2;
		temp += rem;		
	}
	
	if ( rem < 0 )
	{
		rem  *= TvsCnt[--iCount] - temp;
		rem  /= THERM_INTERPOLATION_STEP / 2;
		temp -= rem;		
	}
	
	
	return temp;
};
 

/**
 * GetTemperature will read the thermistor register and convert the value to 
 * kelvin.
 * @return an int that represents the thermister temperature in Kelvin, or a
 * negative value in the case of error.
 */
int GetTemperature(void)
{
	u32 res;
	
/*	printk(KERN_INFO "T&F?::GetTemperature ----\n");
 */
	// Test to ensure we are ready.
	if ( !thermostat ) {
		printk(KERN_INFO "T&F?::$RERROR - Temperature conv not started\n");
		return -1;
	}
		
	while ( !(     read_reg( TACHO_THERMISTOR_CONTROL ) & 
		   (1 << TACHO_THERMISTOR_CONTROL_THERM_VALID) ) ) {
		printk(KERN_INFO "T&F?::$rWarning - Temperature reading not stabalised\n");
		msleep(100);
	}
	
	res = read_reg( TACHO_THERMISTOR_RC_COUNTER );
	
/*	printk(KERN_INFO "T&F?::GetTemperature raw count == %u\n", res);
 */
	
	// convert value TODO
	//res = Count2Temp( res );
	//printk(KERN_INFO "T&F?::GetTemperature converted count == %u\n", res);
	
	return res;
}


/**
 * GetFanRPM will read the fan tacho register and convert the value to 
 * RPM.
 * @return an int that represents the fan speed in RPM, or a
 * negative value in the case of error.
 */
int GetFanRPM(void)
{
	u32 iCounterValue = read_reg( TACHO_FAN_SPEED_COUNTER ); 
	u32 res;
		
	/* iCounterValue == 0xffffffff indicates stationary */
	++iCounterValue;

	
	/* Fan Speed (rpm) = 60 * 2000 / (counter value +1) */
	res  = 60 /*secs/min*/ * TACHO_TARGET_CORE_FREQ_HZ;
	res /= TACHO_FAN_SPEED_DIVIDER;
	res /= iCounterValue;
	
/*	printk(KERN_INFO "thermAndFan::GetFanRPM == %d\n", res);
 */
	return res;
}


static void write_fan_speed(struct thermostat *th, int speed)
{
/*	printk(KERN_INFO "thermAndFan::write_fan_speed %u\n", speed);
 */
	if (speed > 0xff) 
		speed = 0xff;
	else if (speed < -1) 
		speed = 0;
	
	if (th->last_speed == speed)
		return;
	
	write_reg( FAN_SPD_SET, speed );
		
	th->last_speed = speed;			
}

#ifdef DEBUG
static void display_stats(struct thermostat *th)
{
	if ( 1 || th->temps != th->cached_temp) {
		printk(KERN_INFO 
			 "thermAndFan:: Temperature infos:\n"
			 "  * thermostats: %d;\n"
			 "  * pwm: %d;\n"
			 "  * fan speed: %d RPM\n\n",
			 th->temps,
			 fan_speed,
			 GetFanRPM());
		th->cached_temp = th->temps;
	}
}
#endif
/* 
 * Use fuzzy logic type approach to creating the new fan speed. 
 * if count < cold_limit fan should be off.
 * if count > hot_limit fan should be full on.
 * if count between limits set proportionally to base speed + proportional element.
 */
static void update_fan_speed(struct thermostat *th)
{
	int var = th->temps;

/* remember that var = 1/T ie smaller var higher temperature and faster fan speed needed */
	if (abs(var - th->last_var) > 4) {
		int new_speed;

		if(var < cold_limit){
			
			if (var < hot_limit)
			{
				th->last_var = var;
				/* too hot for proportional control */
				new_speed = OXSEMI_FAN_SPEED_MAX;;
			}
			else
			{
				/* fan speed it the user selected starting value for the fan 
				 * so scale operatation from nominal at cold limit to max at hot limit. 
				 */
				new_speed = OXSEMI_FAN_SPEED_MAX - 
					(OXSEMI_FAN_SPEED_MAX - fan_speed) * (var - hot_limit)/(cold_limit - hot_limit);

				if (th->set_speed == 0 ) th->set_speed = fan_speed;

				if ((new_speed - th->set_speed) > MAX_FAN_CHANGE)
					new_speed = th->set_speed + MAX_FAN_CHANGE; 
				else if ((new_speed - th->set_speed) < -MAX_FAN_CHANGE)
					new_speed = th->set_speed - MAX_FAN_CHANGE;
				else
					th->last_var = var;
			}
		}
		else {
			th->last_var = var;
			/* var greater than low limit - too cold for fan. */
			new_speed = 0; 
		}

		write_fan_speed(th, new_speed);
		th->set_speed = new_speed;
	}
}

static int monitor_task(void *arg)
{
	struct thermostat* th = arg;

	while(!kthread_should_stop()) {
		if (current->flags & PF_FREEZE)
			refrigerator();

		msleep_interruptible(2000);
DumpTachoRegisters();

		read_sensors(th);


		update_fan_speed(th);

#ifdef DEBUG
		display_stats(th);
#endif

	}

	return 0;
}



static int
oxsemi_therm_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	//int i = (int __user *)arg;

	printk(KERN_INFO "thermAndFan:: therm_ioctl\n");
	switch(cmd) {
	case SET_LIMIT_ADJUST:
		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;

		if (get_user(cold_limit, &arg))
			return -EFAULT;
		break;
	// etc...
		
	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}


static int
oxsemi_therm_read(char *buf, char **start, off_t offset,
		  int len, int *eof, void *unused)
{
	len = sprintf(buf, 
		"Thermostat And Fan state ---------\n"
		"      temps            == %d\n"
		"      cached_temp      == %d\n"
		"      last_speed       == %d\n"
		"      current_speed    == %d\n"
		"      last_var         == %d\n\n",
		thermostat->temps,
		thermostat->cached_temp,
		thermostat->last_speed,
		thermostat->curr_speed,
		thermostat->last_var );
	//*start = buf; 
	return len;
}

/*
static ssize_t oxsemi_therm_proc_read(struct file *, char __user *, size_t, loff_t *)
{
	struct thermostat* pTherm = file->private_data;
	ssize_t 		   retval = 0;
	
	// if ( down_inerruptable(&dev->sem) ) {
	// 	return -ERSTATSY;
	// }
	
	if ( *f_pos >= 
}
*/

static struct proc_dir_entry *proc_oxsemi_therm;


static struct file_operations oxsemi_therm_fops = {
	.owner		= THIS_MODULE,
	.open		= nonseekable_open,
//	.read		= oxsemi_therm_proc_read,
	.ioctl		= oxsemi_therm_ioctl,
};

static struct miscdevice oxsemi_therm_miscdev = {
	TEMP_MINOR,
	"temp",
	&oxsemi_therm_fops
};



static int __init oxsemi_therm_init(void)
{
	struct thermostat* th;
	int rc, ret;

	if (thermostat)
		return 0;

	read_reg(SYS_CTRL_RSTEN_CTRL);

/* release fan/tacho from system reset */		
	*((volatile unsigned long *) SYS_CTRL_RSTEN_CLR_CTRL) = (1UL << SYS_CTRL_RSTEN_MISC_BIT);
		

/*	printk(KERN_INFO "thermAndFan: mux out therm and fan pins onto GPIO)\n" );
 */
	*((volatile unsigned long *) SYS_CTRL_GPIO_PRIMSEL_CTRL_0) &= ~(1UL << SECONDARY_FUNCTION_ENABLE_FAN_PWM2);
	*((volatile unsigned long *) SYS_CTRL_GPIO_PRIMSEL_CTRL_0) |= (1UL << PRIMARY_FUNCTION_ENABLE_FAN_TACHO);
	*((volatile unsigned long *) SYS_CTRL_GPIO_PRIMSEL_CTRL_0) |= (1UL << PRIMARY_FUNCTION_ENABLE_FAN_TEMP);

/* disable secondary use */
	*((volatile unsigned long *) SYS_CTRL_GPIO_SECSEL_CTRL_0) |= (1UL << SECONDARY_FUNCTION_ENABLE_FAN_PWM2);
	*((volatile unsigned long *) SYS_CTRL_GPIO_SECSEL_CTRL_0) &= ~(1UL << PRIMARY_FUNCTION_ENABLE_FAN_TACHO);
	*((volatile unsigned long *) SYS_CTRL_GPIO_SECSEL_CTRL_0) &= ~(1UL << PRIMARY_FUNCTION_ENABLE_FAN_TEMP);

/* disable tertiary use */
	*((volatile unsigned long *) SYS_CTRL_GPIO_TERTSEL_CTRL_0) &= ~(1UL << SECONDARY_FUNCTION_ENABLE_FAN_PWM2);
	*((volatile unsigned long *) SYS_CTRL_GPIO_TERTSEL_CTRL_0) &= ~(1UL << PRIMARY_FUNCTION_ENABLE_FAN_TACHO);
	*((volatile unsigned long *) SYS_CTRL_GPIO_TERTSEL_CTRL_0) &= ~(1UL << PRIMARY_FUNCTION_ENABLE_FAN_TEMP);
	
	
	read_reg(SYS_CTRL_RSTEN_CTRL);
	read_reg(SYS_CTRL_GPIO_PRIMSEL_CTRL_0);
	read_reg(SYS_CTRL_GPIO_SECSEL_CTRL_0);
	read_reg(SYS_CTRL_GPIO_TERTSEL_CTRL_0);

	th = (struct thermostat *)
		kmalloc(sizeof(struct thermostat), GFP_KERNEL);

	if (!th)
		return -ENOMEM;

	memset(th, 0, sizeof(*th));
	init_MUTEX( &th->sem );

	rc = read_reg(TACHO_CLOCK_DIVIDER);
	if (rc < 0) {
		printk(KERN_ERR "thermAndFan: Thermostat failed to read config ");
		kfree(th);
		return -ENODEV;
	}

	/* Set the Tacho clock divider up */
/*	printk(KERN_INFO "thermAndFan: Setting tacho core frequency divider to %d\n", TACHO_CORE_THERM_DIVIDER_VALUE );
 */
	write_reg( TACHO_CLOCK_DIVIDER, TACHO_CORE_THERM_DIVIDER_VALUE );
	
/* check tacho divider set correctly */	
	rc = read_reg(TACHO_CLOCK_DIVIDER);
	if (rc != TACHO_CORE_THERM_DIVIDER_VALUE) {
		printk(KERN_ERR "thermAndFan: Thermostat failed to set config tacho divider readback:%d\n", rc);
		kfree(th);
		return -ENODEV;
	}
	
/*	printk(KERN_INFO "thermAndFan: Setting PWM core frequency divider to %d\n", PWM_CORE_CLK_DIVIDER_VALUE );
 */
	write_reg( PWM_CLOCK_DIVIDER, PWM_CORE_CLK_DIVIDER_VALUE );
	
	/* force manual control to start the fan quieter */
	if (fan_speed == -1) {
		fan_speed = 64;
	}
	
	printk(KERN_INFO "thermAndFan: initializing\n");
	DumpTachoRegisters();
	
	thermostat = th;
	
	/* Satrt the thermister measuring */
	write_reg( TACHO_THERMISTOR_CONTROL, (1 << TACHO_THERMISTOR_CONTROL_THERM_ENABLE) );

	/* be sure to really write fan speed the first time */
	th->last_speed    = -2;
	th->last_var	  = -80;

	if (fan_speed != -1) {
		/* manual mode, stop fans */
		write_fan_speed(th, 0);
	}
	
//thread_therm = ERR_PTR(-ENOMEM); // FINDME	
thread_therm = kthread_run(monitor_task, th, "kfand");

	if (thread_therm == ERR_PTR(-ENOMEM)) {
		printk(KERN_INFO "thermAndFan: Kthread creation failed\n");
		thread_therm = NULL;
		return -ENOMEM;
	}
	
	
	ret = misc_register(&oxsemi_therm_miscdev);
	if (ret < 0)
		return ret;
	
	proc_oxsemi_therm = create_proc_entry("therm", 0, NULL);
	if (proc_oxsemi_therm) {
		proc_oxsemi_therm->read_proc = oxsemi_therm_read;
	} else {
		printk(KERN_ERR "therm: unable to register /proc/therm\n");
	}


	return 0;
}


static void __exit oxsemi_therm_exit(void)
{
	if ( thread_therm )
	{
		kthread_stop(thread_therm);
	}
	
	remove_proc_entry("therm", NULL);
	misc_deregister(&oxsemi_therm_miscdev);
	
	kfree(thermostat);
	thermostat = NULL;
/* return fan/tacho to system reset */		
	*((volatile unsigned long *) SYS_CTRL_RSTEN_SET_CTRL) |= (1UL << SYS_CTRL_RSTEN_MISC_BIT);
}





module_init(oxsemi_therm_init);
module_exit(oxsemi_therm_exit);






















