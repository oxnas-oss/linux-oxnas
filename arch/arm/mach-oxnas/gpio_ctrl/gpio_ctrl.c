#include <linux/module.h>
#include <linux/init.h>
#include <asm/irq.h>
#include <asm/delay.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/syscalls.h>
#include <linux/kmod.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <asm/fcntl.h>
#include <asm/hardirq.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <linux/spinlock.h>
#include <asm/hardware.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <asm/atomic.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>

MODULE_LICENSE("GPL v2");
#define MODULE_NAME	"gpio"


#undef  POWER_RESUME_DEBUG

#define BEEP_TOGGLE_DELAY_MS    (10)                    /* delay between toggle the buzzer bit, in mini-second */
#define QUICK_PRESS_TIME	1
#define	PRESS_TIME		5
#define BEEP_DURATION		1000

#define BTN_POLLING_PERIOD		(HZ)
#define BTNCPY_POLLING_PERIOD		(HZ/10)


#define BTNCPY_IOC_MAGIC 'g'
#define BTNCPY_IOC_SET_NUM 	_IO(BTNCPY_IOC_MAGIC, 1)

#define LED_SET_CTL_IOC_NUM 	_IO(BTNCPY_IOC_MAGIC, 2)
#define LED_GET_CTL_IOC_NUM 	_IO(BTNCPY_IOC_MAGIC, 3)

#define BUZ_SET_CTL_IOC_NUM	_IO(BTNCPY_IOC_MAGIC, 4)

#define SPR_HD0_IOC_NUM		_IO(BTNCPY_IOC_MAGIC, 5)
#define GPOUT_BLK_QUERY_IOC_NUM	_IO(BTNCPY_IOC_MAGIC, 6)
#define BTN_POWER_ENABLE	_IO(BTNCPY_IOC_MAGIC, 7)
#define BTN_RESET_ENABLE	_IO(BTNCPY_IOC_MAGIC, 8)
#define BUTTON_TEST_IN_IOC_NUM  _IO(BTNCPY_IOC_MAGIC, 9)
#define BUTTON_TEST_OUT_IOC_NUM _IO(BTNCPY_IOC_MAGIC, 10)


#define	GPIO_WRITE_IOC_NUM	_IO(BTNCPY_IOC_MAGIC, 30)
#define GPIO_READ_IOC_NUM	_IO(BTNCPY_IOC_MAGIC, 31)

#define POWER_RESUME_SET	_IO(BTNCPY_IOC_MAGIC, 32)
#define POWER_RESUME_CLR	_IO(BTNCPY_IOC_MAGIC, 33)

dev_t	gpio_dev  = 0;


#define RESET_GPIO_REG_OFFSET			1
#define POWER_GPIO_REG_OFFSET			4		// gpio pin for power button
#define COPY_GPIO_REG_OFFSET			10		// gpio pin for copy button

#define BZ_GPIO_REG_OFFSET			5		// gpio pin for buzzer

#define POWER_RESUME_DATA_GPIO_REG_OFFSET	25
#define POWER_RESUME_CLK_GPIO_REG_OFFSET	34

#define SYS_LED_GPIO_REG_OFFSET			9

#define HDD_LED_GREEN_GPIO_REG_OFFSET		22

#define HDD_LED_RED_GPIO_REG_OFFSET		27



#define COPY_LED_GREEN_GPIO_REG_OFFSET		33
#define COPY_LED_RED_GPIO_REG_OFFSET		24


#define ESATA_LED_GREEN_GPIO_REG_OFFSET		26
#define ESATA_LED_RED_GPIO_REG_OFFSET		28

#define USB_LED_GREEN_GPIO_REG_OFFSET		31

#ifdef CONFIG_ZYXEL_MODEL_NSA221
	#define SYS_LED_ORANGE_GPIO_REG_OFFSET	32
#else
	#define USB_LED_RED_GPIO_REG_OFFSET	32
#endif

#define POWER_OFF_GPIO_REG_OFFSET	23

#define HTP_GPIO_REG_PIN	20


#define REG_CLEAR_BITS(addr, mask)     	writel(readl(addr) & ~mask, addr)

#define CLEAR REG_CLEAR_BITS

#define REG_SET_BITS(addr, mask)	writel(readl(addr) | (mask), addr)

#define GPIO_MASK(gpio_num) (1UL << ((gpio_num & 32) ? (gpio_num - 32):(gpio_num)))


#define Button_Released(data_reg,gpio_num) \
	(readl(data_reg) & GPIO_MASK(gpio_num ))


void DumpGPIO_A( void );
void DumpGPIO_B(void);
extern spinlock_t oxnas_gpio_spinlock;

//-----------------					Query Register 			--------------------//
//----------------------------------------------------------------------------------------------------------//
#define GPIO_MAGIC_G	103
#define GPIO_MAGIC_P	112

#define GPIO_QUERY_ADDR(GPIO_NUM)		\
	((GPIO_MAGIC_G << 24) | (GPIO_MAGIC_P << 16)  | (GPIO_NUM))


#define GET_GPIO_QUERY_NUM(GPIO_QUERY_ADDR)	\
	(GPIO_QUERY_ADDR & 0xff)

#define GPIO_CHECK_MAGIC_NUM(GPIO_QUERY_ADDR)	\
	((GPIO_QUERY_ADDR >> 24) == GPIO_MAGIC_G) && (((GPIO_QUERY_ADDR & (0xff << 16)) >> 16) == GPIO_MAGIC_P)


//---  shutdown enable/disable
static atomic_t shutdown_enable = ATOMIC_INIT(1);

/* Button test */
#define BUTTON_NUM	3

enum BUTTON_NUMBER {
	RESET_BTN_NUM,
	COPY_BTN_NUM,
	POWER_BTN_NUM,
};

static atomic_t button_test_enable = ATOMIC_INIT(0);
static atomic_t button_test_num = ATOMIC_INIT(BUTTON_NUM);


struct gpio_write_data
{
	unsigned long gpio_query_addr;
	unsigned long value;
};

enum{
	GPIO_1_INTERRUPT_NUM = 0,
	 GPIO_A_INTERRUPT_STATUS_REGISTER_NUM,
	 SYS_CTRL_GPIO_PRIMSEL_CTRL_0_NUM,
	 SYS_CTRL_GPIO_SECSEL_CTRL_0_NUM,
	 SYS_CTRL_GPIO_TERTSEL_CTRL_0_NUM,
	 GPIO_A_INPUT_DEBOUNCE_ENABLE_NUM,
	 GPIO_A_LEVEL_INTERRUPT_ENABLE_NUM,
	 GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE_NUM,
	 GPIO_A_DATA_NUM,
	 GPIO_A_OUTPUT_SET_NUM,
	 GPIO_A_OUTPUT_CLEAR_NUM,
	 GPIO_A_OUTPUT_ENABLE_SET_NUM,
	 GPIO_A_OUTPUT_ENABLE_CLEAR_NUM,
	 GPIO_2_INTERRUPT_NUM,
	 GPIO_B_INTERRUPT_STATUS_REGISTER_NUM,
	 SYS_CTRL_GPIO_PRIMSEL_CTRL_1_NUM,
	 SYS_CTRL_GPIO_SECSEL_CTRL_1_NUM,
	 SYS_CTRL_GPIO_TERTSEL_CTRL_1_NUM,
	 GPIO_B_INPUT_DEBOUNCE_ENABLE_NUM,
	 GPIO_B_LEVEL_INTERRUPT_ENABLE_NUM,
	 GPIO_B_FALLING_EDGE_ACTIVE_LOW_ENABLE_NUM,
	 GPIO_B_DATA_NUM,
	 GPIO_B_OUTPUT_SET_NUM,
	 GPIO_B_OUTPUT_CLEAR_NUM,
	 GPIO_B_OUTPUT_ENABLE_SET_NUM,
	 GPIO_B_OUTPUT_ENABLE_CLEAR_NUM,
};


static unsigned long gpio_table[] ={
	 GPIO_1_INTERRUPT,
	 GPIO_A_INTERRUPT_STATUS_REGISTER,
	 SYS_CTRL_GPIO_PRIMSEL_CTRL_0,
	 SYS_CTRL_GPIO_SECSEL_CTRL_0,
	 SYS_CTRL_GPIO_TERTSEL_CTRL_0,
	 GPIO_A_INPUT_DEBOUNCE_ENABLE,
	 GPIO_A_LEVEL_INTERRUPT_ENABLE,
	 GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE,
	 GPIO_A_DATA,
	 GPIO_A_OUTPUT_SET,
	 GPIO_A_OUTPUT_CLEAR,
	 GPIO_A_OUTPUT_ENABLE_SET,
	 GPIO_A_OUTPUT_ENABLE_CLEAR,
	 GPIO_2_INTERRUPT,
	 GPIO_B_INTERRUPT_STATUS_REGISTER,
	 SYS_CTRL_GPIO_PRIMSEL_CTRL_1,
	 SYS_CTRL_GPIO_SECSEL_CTRL_1,
	 SYS_CTRL_GPIO_TERTSEL_CTRL_1,
	 GPIO_B_INPUT_DEBOUNCE_ENABLE,
	 GPIO_B_LEVEL_INTERRUPT_ENABLE,
	 GPIO_B_FALLING_EDGE_ACTIVE_LOW_ENABLE,
	 GPIO_B_DATA,
	 GPIO_B_OUTPUT_SET,
	 GPIO_B_OUTPUT_CLEAR,
	 GPIO_B_OUTPUT_ENABLE_SET,
	 GPIO_B_OUTPUT_ENABLE_CLEAR,
};
//---------------------------------------------------------------------------------------------------------------------//
struct gpio_config_registers
{
	unsigned long  IRQ_NUM;
	unsigned long  INT_STATUS_REG;
	unsigned long  SWITCH_PRISEL_REG;
	unsigned long  SWITCH_SECSEL_REG;
	unsigned long  SWITCH_TERSEL_REG;
	unsigned long  SWITCH_PWM_REG;
	unsigned long  SWITCH_CLR_OE_REG;
	unsigned long  DEBOUNCE_REG;
	unsigned long  LEVEL_INT_REG;
	unsigned long  FALLING_INT_REG;
	unsigned long  DATA_REG;
	unsigned long  OUTPUT_SET_REG;
	unsigned long  OUTPUT_CLR_REG;
	unsigned long  OUTPUT_ENABLE_REG;
	unsigned long  OUTPUT_DISABLE_REG;
};


struct led_blk_timer_data
{
	unsigned short led_gpio_regoffset[2];		// reg offset for led with two colors
	struct timer_list *my_timer;			// pointer to timer list
	unsigned short 	led_color;			// which color should light
	unsigned short	blk_rate;			// blinking rate
	unsigned short 	bit_value;			//
	unsigned short 	timer_state;			//
	spinlock_t 	lock;				// spin lock

};


#define GPIO_CONFIG_NUM(gpio_num)	(gpio_num >> 5)

struct gpio_config_registers gpio_config_regs[2] =
{
	{GPIO_1_INTERRUPT,
	 GPIO_A_INTERRUPT_STATUS_REGISTER,
	 SYS_CTRL_GPIO_PRIMSEL_CTRL_0,
	 SYS_CTRL_GPIO_SECSEL_CTRL_0,
	 SYS_CTRL_GPIO_TERTSEL_CTRL_0,
	 SYS_CTRL_GPIO_PWMSEL_CTRL_0,
	 GPIO_A_OUTPUT_ENABLE_CLEAR,
	 GPIO_A_INPUT_DEBOUNCE_ENABLE,
	 GPIO_A_LEVEL_INTERRUPT_ENABLE,
	 GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE,
	 GPIO_A_DATA,
	 GPIO_A_OUTPUT_SET,
	 GPIO_A_OUTPUT_CLEAR,
	 GPIO_A_OUTPUT_ENABLE_SET,
	 GPIO_A_OUTPUT_ENABLE_CLEAR},

	{GPIO_2_INTERRUPT,
	 GPIO_B_INTERRUPT_STATUS_REGISTER,
	 SYS_CTRL_GPIO_PRIMSEL_CTRL_1,
	 SYS_CTRL_GPIO_SECSEL_CTRL_1,
	 SYS_CTRL_GPIO_TERTSEL_CTRL_1,
	 SYS_CTRL_GPIO_PWMSEL_CTRL_1,
	 GPIO_B_OUTPUT_ENABLE_CLEAR,
	 GPIO_B_INPUT_DEBOUNCE_ENABLE,
	 GPIO_B_LEVEL_INTERRUPT_ENABLE,
	 GPIO_B_FALLING_EDGE_ACTIVE_LOW_ENABLE,
	 GPIO_B_DATA,
	 GPIO_B_OUTPUT_SET,
	 GPIO_B_OUTPUT_CLEAR,
	 GPIO_B_OUTPUT_ENABLE_SET,
	 GPIO_B_OUTPUT_ENABLE_CLEAR},
};



//***					led blk timer data					    ***//

#define SLOW_BLINKING_PERIOD	(HZ/2)
#define FAST_BLINKING_PERIOD	(HZ/10)
#define VERY_FAST_BLINKING_PERIOD (HZ/10)

#define RED		(1<<0)
#define GREEN		(2<<0)
#define ORANGE		(RED | GREEN)
#define NO_COLOR	0

#define	COLOR_MASK	ORANGE
#define LED_MASK 0xF

#define BLK_STATE_BASE	1
#define FAST_BLK	1
#define SLOW_BLK	0

#define TIMER_RUNNING	0x1
#define	TIMER_SLEEPING	0x0


// Define the status of LED
//	color : red, green, orange, no clolr
//	state: on/off, fast/slow blinking

enum LED_STATE {
	LED_OFF,
	LED_ON,
	LED_BLK_SLOW,
	LED_BLK_FAST,
};


#define LED_COLOR_BITS	0	// 0,1
#define LED_STATE_BITS	2	// 2,3
#define LED_NUM_BITS	4	// 4,5



#define LED_STATE_MAP_ADDR(led_num, led_state, led_color)	\
	(led_num << LED_NUM_BITS) | (led_state << LED_STATE_BITS) | (led_color << LED_COLOR_BITS)

#define GET_LED_INDEX(map_addr) ((map_addr >> LED_NUM_BITS) & 0xf)
#define GET_LED_COLOR(map_addr)	(map_addr & 0x3)
#define GET_LED_STATE(map_addr) ((map_addr >> LED_STATE_BITS) & 0x3)



static struct timer_list 	led0_blk_timer;
static struct timer_list 	led1_blk_timer;
static struct timer_list	led2_blk_timer;
static struct timer_list	led3_blk_timer;
static struct timer_list	led4_blk_timer;

struct led_blk_timer_data led0_blk_timer_data =
{
	{HDD_LED_RED_GPIO_REG_OFFSET,  HDD_LED_GREEN_GPIO_REG_OFFSET},			// HD red led gpio pin, HD green led pgio pin
	&led0_blk_timer,								// timer for HD led
	0,										// current color
	0,										// blink rate
	0,										// current on/off
	0,										// timer state
	SPIN_LOCK_UNLOCKED,								// lock
};


struct led_blk_timer_data led1_blk_timer_data =
{
	{COPY_LED_RED_GPIO_REG_OFFSET, COPY_LED_GREEN_GPIO_REG_OFFSET},
	&led1_blk_timer,
	0,
	0,
	0,
	0,
	SPIN_LOCK_UNLOCKED,
};



struct led_blk_timer_data led2_blk_timer_data =
{
	{ESATA_LED_RED_GPIO_REG_OFFSET,  ESATA_LED_GREEN_GPIO_REG_OFFSET},
	&led2_blk_timer,
	0,
	0,
	0,
	0,
	SPIN_LOCK_UNLOCKED,
};


struct led_blk_timer_data led3_blk_timer_data =
{
#ifdef  CONFIG_ZYXEL_MODEL_NSA221
	{0x0,  USB_LED_GREEN_GPIO_REG_OFFSET},
#else
	{USB_LED_RED_GPIO_REG_OFFSET,  USB_LED_GREEN_GPIO_REG_OFFSET},
#endif

	&led3_blk_timer,
	0,
	0,
	0,
	0,
	SPIN_LOCK_UNLOCKED,
};

struct led_blk_timer_data led4_blk_timer_data =
{
#ifdef CONFIG_ZYXEL_MODEL_NSA221
	{SYS_LED_ORANGE_GPIO_REG_OFFSET, SYS_LED_GPIO_REG_OFFSET},
#else
	{0x0, SYS_LED_GPIO_REG_OFFSET},
#endif
	&led4_blk_timer,
	0,
	0,
	0,
	0,
	SPIN_LOCK_UNLOCKED,
};


enum LED_NUM{
	LED0_INDEX_NUM,
	LED1_INDEX_NUM,
	LED2_INDEX_NUM,
	LED3_INDEX_NUM,
	LED4_INDEX_NUM,
};

#define LED_NUMS	5

unsigned long led_status[LED_NUMS] = {0x0, 0x0, 0x0, 0x0, 0x0};
spinlock_t led_status_lock = SPIN_LOCK_UNLOCKED;
static struct timer_list*		all_led_blk_timer_ptr[LED_NUMS];
static struct led_blk_timer_data*	all_led_blk_timer_data_ptr[LED_NUMS];
//#define GET_LED_INDEX_NUM
//#define GET_COLOR_NUM
//#define GET_BLK_STATE


void init_led_blk_timer(void)
{
	all_led_blk_timer_ptr[LED0_INDEX_NUM] = &led0_blk_timer;
	all_led_blk_timer_ptr[LED1_INDEX_NUM] = &led1_blk_timer;
	all_led_blk_timer_ptr[LED2_INDEX_NUM] = &led2_blk_timer;
	all_led_blk_timer_ptr[LED3_INDEX_NUM] = &led3_blk_timer;
	//all_led_blk_timer_ptr[LED4_INDEX_NUM] = &led4_blk_timer;

	init_timer(&led0_blk_timer);
	init_timer(&led1_blk_timer);
	init_timer(&led2_blk_timer);
	init_timer(&led3_blk_timer);
	//init_timer(&led4_blk_timer);
}

void init_led_blk_timer_data(void)
{
	all_led_blk_timer_data_ptr[LED0_INDEX_NUM] = &led0_blk_timer_data;
	all_led_blk_timer_data_ptr[LED1_INDEX_NUM] = &led1_blk_timer_data;
	all_led_blk_timer_data_ptr[LED2_INDEX_NUM] = &led2_blk_timer_data;
	all_led_blk_timer_data_ptr[LED3_INDEX_NUM] = &led3_blk_timer_data;
	//all_led_blk_timer_data_ptr[LED4_INDEX_NUM] = &led4_blk_timer_data;
}


void init_sys_led(void)
{
	all_led_blk_timer_ptr[LED4_INDEX_NUM] = &led4_blk_timer;
	init_timer(&led4_blk_timer);
	all_led_blk_timer_data_ptr[LED4_INDEX_NUM] = &led4_blk_timer_data;
}



#define ENABLE_LED_BLK(timer_data) 					\
	(timer_data)->timer_state = TIMER_RUNNING;			\
	(timer_data)->my_timer->function = led_blk_timer_run;			\
	(timer_data)->my_timer->data = (unsigned long)(timer_data);			\
	mod_timer((timer_data->my_timer), jiffies + (timer_data)->blk_rate);

#define DISABLE_LED_BLK(timer_data)					\
	(timer_data)->timer_state = TIMER_SLEEPING;			\
	del_timer(timer_data->my_timer);

void turn_off_led(unsigned long led_num)
{
	unsigned long red_led_reg, green_led_reg;
	unsigned long red_output_clr_reg, green_output_clr_reg;

	red_led_reg = all_led_blk_timer_data_ptr[led_num]->led_gpio_regoffset[RED-1];
	green_led_reg = all_led_blk_timer_data_ptr[led_num]->led_gpio_regoffset[GREEN-1];

	red_output_clr_reg = gpio_config_regs[GPIO_CONFIG_NUM(red_led_reg)].OUTPUT_CLR_REG;
	green_output_clr_reg = gpio_config_regs[GPIO_CONFIG_NUM(green_led_reg)].OUTPUT_CLR_REG;

	spin_lock(&oxnas_gpio_spinlock);

	//printk(KERN_ERR"/**********************************************************************/\n");
	//printk(KERN_ERR"OUTPUT_CLR_REG = %x\n", readl(red_output_clr_reg));
	//printk(KERN_ERR"OUTPUT_CLR_REG1 = %x\n", readl(green_output_clr_reg));

	//printk(KERN_ERR"write = %x\n", readl(red_output_clr_reg) | GPIO_MASK(red_led_reg));
	//printk(KERN_ERR"write1 = %x\n", readl(green_output_clr_reg) | GPIO_MASK(green_led_reg));
	if(red_led_reg > 0)
		writel(GPIO_MASK(red_led_reg), red_output_clr_reg);

	if(green_led_reg > 0)
		writel(GPIO_MASK(green_led_reg), green_output_clr_reg);

	//printk(KERN_ERR"OUTPUT_CLR_REG = %x\n", readl(red_output_clr_reg));
	//printk(KERN_ERR"OUTPUT_CLR_REG1 = %x\n", readl(green_output_clr_reg));

	//printk(KERN_ERR"/**********************************************************************/\n");

	//REG_SET_BITS(red_output_clr_reg, GPIO_MASK(red_led_reg));
	//REG_SET_BITS(green_output_clr_reg, GPIO_MASK(green_led_reg));
	spin_unlock(&oxnas_gpio_spinlock);

	//DumpGPIO_A();


}

void turn_on_led(unsigned long led_num, unsigned long led_color)
{
	unsigned long red_led_reg, green_led_reg;
	unsigned long red_output_set_reg, green_output_set_reg;
	unsigned long red_output_clr_reg, green_output_clr_reg;

	red_led_reg = all_led_blk_timer_data_ptr[led_num]->led_gpio_regoffset[RED-1];		// get red led gpio pin
	green_led_reg = all_led_blk_timer_data_ptr[led_num]->led_gpio_regoffset[GREEN-1];	// get green led gpio pin

	red_output_set_reg = gpio_config_regs[GPIO_CONFIG_NUM(red_led_reg)].OUTPUT_SET_REG;   	// Use red gpio pin to get output set
	green_output_set_reg = gpio_config_regs[GPIO_CONFIG_NUM(green_led_reg)].OUTPUT_SET_REG;	// Use green gpio pin to get output set

	red_output_clr_reg = gpio_config_regs[GPIO_CONFIG_NUM(red_led_reg)].OUTPUT_CLR_REG;	// Use red gpio pin to get output clear
	green_output_clr_reg = gpio_config_regs[GPIO_CONFIG_NUM(green_led_reg)].OUTPUT_CLR_REG; // Use green gpio pin to get output clear

	spin_lock(&oxnas_gpio_spinlock);
	if((led_color & RED) && red_led_reg > 0)
	{
		writel(GPIO_MASK(red_led_reg), red_output_set_reg);				// turn on red(set red pin in output set)
	}

	if((led_color & GREEN) && green_led_reg > 0)
	{
		writel(GPIO_MASK(green_led_reg), green_output_set_reg);				// turn on green
	}

	spin_unlock(&oxnas_gpio_spinlock);

	//DumpGPIO_A();

}


// Blk timer function
void led_blk_timer_run(unsigned long in_data)
{
	struct	led_blk_timer_data	*my_data;
	short	bit_on;
	unsigned long	red_led_reg, green_led_reg;


	spin_lock(&(my_data->lock));
	my_data = (struct led_blk_timer_data*) in_data;
	bit_on = my_data->bit_value;

	bit_on ^= 1;										// Period On/Off for blinking
	my_data->bit_value = bit_on;								// store current bit value

	if(!my_data->led_color)
	{
		printk(KERN_ERR"LED_BLK: BLK but without color\n");
		return ;
	}


	if(my_data->led_color & RED && my_data->led_color & GREEN)				// check blinking led color(orange)
	{
		unsigned long red_output_set_reg, green_output_set_reg;
		unsigned long red_output_clr_reg, green_output_clr_reg;

		red_led_reg = my_data->led_gpio_regoffset[RED - 1];				// get red led gpio pin
		green_led_reg = my_data->led_gpio_regoffset[GREEN - 1];				// get green led gpio pin


		red_output_set_reg = gpio_config_regs[GPIO_CONFIG_NUM(red_led_reg)].OUTPUT_SET_REG; // get output set reg for red led
		red_output_clr_reg = gpio_config_regs[GPIO_CONFIG_NUM(red_led_reg)].OUTPUT_CLR_REG; // get output clear reg for red led

		green_output_set_reg = gpio_config_regs[GPIO_CONFIG_NUM(green_led_reg)].OUTPUT_SET_REG;	//get output set reg for green led
		green_output_clr_reg = gpio_config_regs[GPIO_CONFIG_NUM(green_led_reg)].OUTPUT_CLR_REG;	//get output clear reg for green led


		if(!bit_on)										// led on
		{
			spin_lock(&oxnas_gpio_spinlock);
			if(red_led_reg > 0)
				writel(GPIO_MASK(red_led_reg), red_output_clr_reg);			// set red pin in output set
			if(green_led_reg > 0)
				writel(GPIO_MASK(green_led_reg), green_output_clr_reg);			// set green pin in output set
			spin_unlock(&oxnas_gpio_spinlock);
		}
		else											// led off
		{
			spin_lock(&oxnas_gpio_spinlock);

			if(red_led_reg > 0)
				writel(GPIO_MASK(red_led_reg), red_output_set_reg);			// set red pin in output clear
			if(green_led_reg > 0)
				writel(GPIO_MASK(green_led_reg),green_output_set_reg);			// set green pin in output clear
			spin_unlock(&oxnas_gpio_spinlock);
		}

	}
	else if(my_data->led_color & RED)
	{
		unsigned long red_output_set_reg;
		unsigned long red_output_clr_reg;

		red_led_reg = my_data->led_gpio_regoffset[RED - 1];
		red_output_set_reg = gpio_config_regs[GPIO_CONFIG_NUM(red_led_reg)].OUTPUT_SET_REG;
		red_output_clr_reg = gpio_config_regs[GPIO_CONFIG_NUM(red_led_reg)].OUTPUT_CLR_REG;

		spin_lock(&oxnas_gpio_spinlock);
		if(!bit_on && red_led_reg > 0)
			writel(GPIO_MASK(red_led_reg), red_output_clr_reg);
		else
		{
			if(red_led_reg > 0)
				writel(GPIO_MASK(red_led_reg), red_output_set_reg);
		}
		spin_unlock(&oxnas_gpio_spinlock);
	}
	else if(my_data->led_color & GREEN)
	{
		unsigned long green_output_set_reg, green_output_clr_reg;

		green_led_reg = my_data->led_gpio_regoffset[GREEN - 1];
		green_output_set_reg = gpio_config_regs[GPIO_CONFIG_NUM(green_led_reg)].OUTPUT_SET_REG;
		green_output_clr_reg = gpio_config_regs[GPIO_CONFIG_NUM(green_led_reg)].OUTPUT_CLR_REG;

		spin_lock(&oxnas_gpio_spinlock);
		if(!bit_on && green_led_reg > 0)
			writel(GPIO_MASK(green_led_reg), green_output_clr_reg);
		else
		{
			if(green_led_reg > 0)
				writel(GPIO_MASK(green_led_reg), green_output_set_reg);
		}
		spin_unlock(&oxnas_gpio_spinlock);

	}
	else;


	ENABLE_LED_BLK(my_data);
	spin_unlock(&(my_data->lock));

}


void init_all_led_config_regs(void)
{
	unsigned long red_led_reg, green_led_reg;
	unsigned long   red_output_set_reg, green_output_set_reg;
	int	i;

	for(i = 0 ; i < LED_NUMS ; i++)
	{
		red_led_reg = all_led_blk_timer_data_ptr[i]->led_gpio_regoffset[RED - 1];		// get red led pin
		green_led_reg = all_led_blk_timer_data_ptr[i]->led_gpio_regoffset[GREEN - 1];		// get green led pin


		red_output_set_reg = gpio_config_regs[GPIO_CONFIG_NUM(red_led_reg)].OUTPUT_SET_REG;	// output set reg for red led
		green_output_set_reg = gpio_config_regs[GPIO_CONFIG_NUM(green_led_reg)].OUTPUT_SET_REG;	// output set reg for green led

		spin_lock(&oxnas_gpio_spinlock);

		if(green_led_reg > 0)
		{
			//Disable other uses (primary)
			REG_CLEAR_BITS(gpio_config_regs[GPIO_CONFIG_NUM(green_led_reg)].SWITCH_PRISEL_REG, GPIO_MASK(green_led_reg));
			// disable secondary
			REG_CLEAR_BITS(gpio_config_regs[GPIO_CONFIG_NUM(green_led_reg)].SWITCH_SECSEL_REG, GPIO_MASK(green_led_reg));
			// disable 3rd use
			REG_CLEAR_BITS(gpio_config_regs[GPIO_CONFIG_NUM(green_led_reg)].SWITCH_TERSEL_REG, GPIO_MASK(green_led_reg));

			REG_CLEAR_BITS(gpio_config_regs[GPIO_CONFIG_NUM(green_led_reg)].SWITCH_PWM_REG, GPIO_MASK(green_led_reg));

			// enable output for this green gpio pin
			REG_SET_BITS((gpio_config_regs[GPIO_CONFIG_NUM(green_led_reg)].OUTPUT_ENABLE_REG),GPIO_MASK(green_led_reg));
		}

		if(red_led_reg > 0)	// led 4 without red led
		{
			// disable 1st use
			REG_CLEAR_BITS(gpio_config_regs[GPIO_CONFIG_NUM(red_led_reg)].SWITCH_PRISEL_REG, GPIO_MASK(red_led_reg));
			// disable 2nd use
			REG_CLEAR_BITS(gpio_config_regs[GPIO_CONFIG_NUM(red_led_reg)].SWITCH_SECSEL_REG, GPIO_MASK(red_led_reg));
			// disable 3rd use
			REG_CLEAR_BITS(gpio_config_regs[GPIO_CONFIG_NUM(red_led_reg)].SWITCH_TERSEL_REG, GPIO_MASK(red_led_reg));
			// enable output
			REG_SET_BITS((gpio_config_regs[GPIO_CONFIG_NUM(red_led_reg)].OUTPUT_ENABLE_REG), GPIO_MASK(red_led_reg));
		}

		spin_unlock(&oxnas_gpio_spinlock);

	}
}




void kernel_start_up_blk(void)
{
	struct led_blk_timer_data*      timer_data_tptr;
	struct timer_list *     timer_tptr;


	timer_data_tptr =  all_led_blk_timer_data_ptr[LED4_INDEX_NUM];
	timer_tptr = all_led_blk_timer_ptr[LED4_INDEX_NUM];
	spin_lock(&(timer_data_tptr->lock));

	if(timer_data_tptr->timer_state == TIMER_RUNNING)
		DISABLE_LED_BLK(timer_data_tptr);


	turn_off_led(LED4_INDEX_NUM);
	timer_data_tptr->led_color = GREEN;
	timer_data_tptr->blk_rate = VERY_FAST_BLINKING_PERIOD;
	timer_data_tptr->bit_value = 0;
	ENABLE_LED_BLK(timer_data_tptr);

}



static int set_led_config(unsigned long led_data)
{
	unsigned int led_index, color, state;
	struct led_blk_timer_data*      timer_data_tptr;
	struct timer_list *     timer_tptr;


	led_index = GET_LED_INDEX(led_data);
	color = GET_LED_COLOR(led_data);
	state = GET_LED_STATE(led_data);

	if(led_index >= LED_NUMS)
		return -ENOTTY;

	spin_lock(&led_status_lock);
	led_status[led_index] = led_data;
	spin_unlock(&led_status_lock);


	timer_data_tptr = all_led_blk_timer_data_ptr[led_index];
	timer_tptr = all_led_blk_timer_ptr[led_index];


	spin_lock(&(timer_data_tptr->lock));

	//Turn off all first
	if(timer_data_tptr->timer_state == TIMER_RUNNING)
		DISABLE_LED_BLK(timer_data_tptr);


	turn_off_led(led_index);

	if(state == LED_BLK_SLOW || state == LED_BLK_FAST)
	{
		timer_data_tptr->led_color = color;

		if(state == LED_BLK_SLOW)
			timer_data_tptr->blk_rate = SLOW_BLINKING_PERIOD;
		else
			timer_data_tptr->blk_rate = FAST_BLINKING_PERIOD;

		timer_data_tptr->bit_value = 0;

		ENABLE_LED_BLK(timer_data_tptr);

	}
	else if(state == LED_ON)
		turn_on_led(led_index, color);
	else;

	spin_unlock(&(timer_data_tptr->lock));

	return 0;

}



//------------------------------------------------------------------------------------------------------------//

//***					Start of buzzer control						***//
//-------------------------------------------------------------------------------------------------------- //


#ifdef CONFIG_ZYXEL_MODEL_NSA221
	#define PWM_PERIOD	810
#else
	#define PWM_PERIOD 	7810
#endif
#define MAX_PWMS     	16

static struct timer_list        bz_timer;
static short bz_time;
static short bz_timer_status = TIMER_SLEEPING;
spinlock_t bz_lock = SPIN_LOCK_UNLOCKED;

typedef enum{
	FREQ_C = 0,
	FREQ_D,
	FREQ_E,
	FREQ_F,
	FREQ_G,
	FREQ_A,
	FREQ_B,
	FREQ_C_PLUS,
}buz_freq;

typedef enum {
	BUZ_OFF = 0,            /* turn off buzzer */
	BUZ_ON,
	BUZ_KILL,               /* kill buzzer daemon, equaling to BUZ_OFF*/
	BUZ_FOREVER             /* keep buzzing */
} buz_cmd_t;


#define BZ_TIMER_PERIOD (HZ/2)
#define TIME_BITS       5
#define FREQ_BITS       4
#define STATE_BITS      2

#define TIME_MASK       (0x1F << (FREQ_BITS + STATE_BITS))
#define FREQ_MASK       (0xF << STATE_BITS)
#define STATE_MASK      0x3


#define GET_TIME(addr)  ((addr & TIME_MASK) >> (FREQ_BITS + STATE_BITS))
#define GET_FREQ(addr)  ((addr & FREQ_MASK) >> STATE_BITS)
#define GET_STATE(addr) (addr & STATE_MASK)

#define BUZ_GPIO_MAP_ADDR(time, freq, state)    \
	(time << (FREQ_BITS + STATE_BITS) | (freq << STATE_BITS) | state)



static void buzzer_timer_func(unsigned long in_data)
{
	spin_lock(&bz_lock);


	if(bz_time != 0)	// continue the timer
	{

#ifdef CONFIG_ZYXEL_MODEL_NSA221
		int i;
		unsigned short high_low = 0;

		for(i = 0 ; i < 1000 ; i++)
		{
			if(high_low)
				writel(GPIO_MASK(BZ_GPIO_REG_OFFSET),
					gpio_config_regs[GPIO_CONFIG_NUM(BZ_GPIO_REG_OFFSET)].OUTPUT_SET_REG);
			else
				writel(GPIO_MASK(BZ_GPIO_REG_OFFSET),
						gpio_config_regs[GPIO_CONFIG_NUM(BZ_GPIO_REG_OFFSET)].OUTPUT_CLR_REG);
			udelay(500);
			high_low ^= 1;
		}
#endif

		bz_timer.function = buzzer_timer_func;
		mod_timer(&bz_timer, jiffies + BZ_TIMER_PERIOD);

	}
	else
	{
		// turn off buzzer, output default val

#ifdef  CONFIG_ZYXEL_MODEL_NSA221
		REG_SET_BITS((gpio_config_regs[GPIO_CONFIG_NUM(BZ_GPIO_REG_OFFSET)].OUTPUT_CLR_REG),GPIO_MASK(BZ_GPIO_REG_OFFSET));
#endif

#ifdef  CONFIG_ZYXEL_MODEL_NSA210
		/* Turn off PWM*/
		writel(readl(SYS_CTRL_GPIO_PWMSEL_CTRL_0) &  ~(1 << BZ_GPIO_REG_OFFSET), SYS_CTRL_GPIO_PWMSEL_CTRL_0);
#endif

	}

	if(bz_time > 0) --bz_time;

	spin_unlock(&bz_lock);
}

void init_buzzer_control(void)
{
	spin_lock(&bz_lock);
	// Disable other system control use (primary, secondary, tertiary)
	CLEAR(SYS_CTRL_GPIO_PRIMSEL_CTRL_0, (1 << BZ_GPIO_REG_OFFSET));
	CLEAR(SYS_CTRL_GPIO_SECSEL_CTRL_0, (1 << BZ_GPIO_REG_OFFSET));
	CLEAR(SYS_CTRL_GPIO_TERTSEL_CTRL_0, (1 << BZ_GPIO_REG_OFFSET));
	CLEAR(SYS_CTRL_GPIO_PWMSEL_CTRL_0, (1 << BZ_GPIO_REG_OFFSET));


	/* Set buzzer bit as gpio and output low */
#ifdef	CONFIG_ZYXEL_MODEL_NSA221
	REG_SET_BITS((gpio_config_regs[GPIO_CONFIG_NUM(BZ_GPIO_REG_OFFSET)].OUTPUT_ENABLE_REG),GPIO_MASK(BZ_GPIO_REG_OFFSET));
	REG_SET_BITS((gpio_config_regs[GPIO_CONFIG_NUM(BZ_GPIO_REG_OFFSET)].OUTPUT_CLR_REG),GPIO_MASK(BZ_GPIO_REG_OFFSET));
#endif

#ifdef	CONFIG_ZYXEL_MODEL_NSA210

	writel((1<<SYS_CTRL_RSTEN_MISC_BIT), SYS_CTRL_RSTEN_CLR_CTRL);
	writel(PWM_PERIOD, PWM_CLOCK_REGISTER);
	writel(250, (PWM_DATA_REGISTER_BASE+4*(BZ_GPIO_REG_OFFSET % MAX_PWMS)));
#endif
	// init bz timer
	init_timer(&bz_timer);
	bz_timer.function = buzzer_timer_func;
	bz_timer_status = TIMER_SLEEPING;
	spin_unlock(&bz_lock);
}


int set_buzzer(unsigned long bz_data)
{
	short	time;
	unsigned short freq, status;

	time = GET_TIME(bz_data);
	freq = GET_FREQ(bz_data);
	status = GET_STATE(bz_data);

	printk(KERN_ERR"bz time = %x\n", time);
	printk(KERN_ERR"bz freq = %x\n", freq);
	printk(KERN_ERR"bz status = %x\n", status);

	spin_lock(&bz_lock);


	// Turn off bz first
	if(bz_timer_status == TIMER_RUNNING)
	{
		bz_timer_status = TIMER_SLEEPING;
		writel(readl(SYS_CTRL_GPIO_PWMSEL_CTRL_0) &  ~(1 << BZ_GPIO_REG_OFFSET), SYS_CTRL_GPIO_PWMSEL_CTRL_0);
		del_timer(&bz_timer);
	}

	if(status == BUZ_ON || status == BUZ_FOREVER)
	{
		// set bz time

		bz_timer_status = TIMER_RUNNING;

		if(time >= 32 || status == BUZ_FOREVER) time = -1;

		bz_time = time;

#ifdef	CONFIG_ZYXEL_MODEL_NSA210
		/* Enable buzzer */
		writel(readl(SYS_CTRL_GPIO_PWMSEL_CTRL_0) | (1 << BZ_GPIO_REG_OFFSET), SYS_CTRL_GPIO_PWMSEL_CTRL_0);
#endif

		bz_timer.function = buzzer_timer_func;
		mod_timer(&bz_timer, jiffies + HZ);
	}

	spin_unlock(&bz_lock);

	return 0;

}

//***					End of buzzer control						***//
//---------------------------------------------------------------------------------------------------------//





//***					Start of button	control						***//
//---------------------------------------------------------------------------------------------------------//
static struct timer_list	btnpow_timer;
static struct timer_list	btnreset_timer;
static struct timer_list	btncpy_timer;

static int    btncpy_pid = 0;
static int    btncpy_nr_devs = 1;
static int    btn_test_num = 0;

struct cdev *btncpy_cdev ;

// work to to do something about button
static DECLARE_WORK(halt_nsa, NULL);
static DECLARE_WORK(Reset_User_Info, NULL);
static DECLARE_WORK(Open_Backdoor, NULL);
static DECLARE_WORK(Reset_To_Default, NULL);

// trigger signal to button daemon in user space
static DECLARE_WORK(btncpy_signal10, NULL);
static DECLARE_WORK(btncpy_signal12, NULL);

// Add by George to enable/disable disk power saving function
extern unsigned short time_to_power_save;
extern int EnablePowerSaving(unsigned long PowerSavingTime );
extern void DisablePowerSaving(void);
struct workqueue_struct *btn_workqueue;





//***			GeorgeKang:  Help function for button control 		    ***//
//-------------------------------------------------------------------------------------//
#define BEEP_DURATION   1000
static void  Beep(void)
{
#ifdef  CONFIG_ZYXEL_MODEL_NSA221
	int i;
	unsigned short high_low = 0;

	for(i = 0 ; i < BEEP_DURATION ; i++)
	{
		if(high_low)
			writel(GPIO_MASK(BZ_GPIO_REG_OFFSET),
					gpio_config_regs[GPIO_CONFIG_NUM(BZ_GPIO_REG_OFFSET)].OUTPUT_SET_REG);
		else
			writel(GPIO_MASK(BZ_GPIO_REG_OFFSET),
					gpio_config_regs[GPIO_CONFIG_NUM(BZ_GPIO_REG_OFFSET)].OUTPUT_CLR_REG);
		udelay(500);
		high_low ^= 1;
	}

	writel(GPIO_MASK(BZ_GPIO_REG_OFFSET),gpio_config_regs[GPIO_CONFIG_NUM(BZ_GPIO_REG_OFFSET)].OUTPUT_CLR_REG);
#endif

#ifdef  CONFIG_ZYXEL_MODEL_NSA210
	spin_lock(&oxnas_gpio_spinlock);
	writel(readl(SYS_CTRL_GPIO_PWMSEL_CTRL_0) | (1 << BZ_GPIO_REG_OFFSET), SYS_CTRL_GPIO_PWMSEL_CTRL_0);
	spin_unlock(&oxnas_gpio_spinlock);

	mdelay(500);

	spin_lock(&oxnas_gpio_spinlock);
	writel(readl(SYS_CTRL_GPIO_PWMSEL_CTRL_0) &  ~(1 << BZ_GPIO_REG_OFFSET), SYS_CTRL_GPIO_PWMSEL_CTRL_0);
	spin_unlock(&oxnas_gpio_spinlock);

#endif




}

void power_off(void *no_used)
{
	unsigned output_set_reg;
	if(atomic_read(&shutdown_enable))
	{
		spin_lock(&oxnas_gpio_spinlock);

		output_set_reg = gpio_config_regs[GPIO_CONFIG_NUM(POWER_OFF_GPIO_REG_OFFSET)].OUTPUT_SET_REG;// OUTPUT_SET_REG;
		writel(GPIO_MASK(POWER_OFF_GPIO_REG_OFFSET), output_set_reg);

		spin_unlock(&oxnas_gpio_spinlock);
		//DumpGPIO_A();
	}
	return ;


}

void nsa_shutdown_func(struct work_struct *in)
{
	if(atomic_read(&shutdown_enable))
		sys_kill(1,10);
}


void Reset_UserInfo_func(struct work_struct *in)
{
	if(atomic_read(&shutdown_enable))
		call_usermodehelper("/usr/local/btn/reset_userinfo.sh", NULL, NULL, 0);
}

void Open_Backdoor_func(struct work_struct *in)
{
	if(atomic_read(&shutdown_enable))
		call_usermodehelper("/usr/local/btn/open_back_door.sh", NULL, NULL, 0);
}

void Reset_To_Defu_func(struct work_struct *in)
{
	if(atomic_read(&shutdown_enable))
	{
		Beep();
		ssleep(1);

		Beep();
		ssleep(1);

		Beep();
		ssleep(1);
		call_usermodehelper("/usr/local/btn/reset_and_reboot.sh", NULL, NULL, 0);
	}
}
void btncpy_signal_func10(struct work_struct *in)
{
	sys_kill(btncpy_pid, 10);
}
void btncpy_signal_func12(struct work_struct *in)
{
	sys_kill(btncpy_pid, 12);
}
//***				 End of help function for button 		***//
//---------------------------------------------------------------------------------//
//***			GeorgeKang: timer function to control the button	***//
static void btnpow_timer_func(unsigned long in_data)
{
	static int 	polling_times = 0;
	struct gpio_config_registers *gcr;

	gcr = &(gpio_config_regs[GPIO_CONFIG_NUM(POWER_GPIO_REG_OFFSET)]);

	if(Button_Released(GPIO_A_DATA, POWER_GPIO_REG_OFFSET))
	{
		if(atomic_read(&button_test_enable) &&
			(atomic_read(&button_test_num) == POWER_BTN_NUM))
		{
			atomic_set(&button_test_enable, 0);
			PREPARE_WORK(&btncpy_signal10, btncpy_signal_func10);
			queue_work(btn_workqueue, &btncpy_signal10);

		}
		else
		{
			if(polling_times >= QUICK_PRESS_TIME  && polling_times < PRESS_TIME)
			{
				PREPARE_WORK(&halt_nsa, nsa_shutdown_func);
				queue_work(btn_workqueue,&halt_nsa);
				//power_off(NULL);				// work around first
			} else if(polling_times >= PRESS_TIME) {
				//	del_timer(&btnpow_timer);
				power_off(NULL);

			}
			else;
		}

		polling_times = 0;

		spin_lock(&oxnas_gpio_spinlock);
		writel(readl(gcr->FALLING_INT_REG) | GPIO_MASK(POWER_GPIO_REG_OFFSET), gcr->FALLING_INT_REG);
		spin_unlock(&oxnas_gpio_spinlock);

	}
	else
	{
		++polling_times;
		printk(KERN_ERR"Power button press\n");
		if(polling_times == QUICK_PRESS_TIME || polling_times == PRESS_TIME) Beep();
		mod_timer(&btnpow_timer, jiffies + BTN_POLLING_PERIOD);
	}
}

static void btnreset_timer_func(unsigned long in_data)
{
	static int polling_times = 0;
	struct gpio_config_registers *gcr;

	gcr = &(gpio_config_regs[GPIO_CONFIG_NUM(RESET_GPIO_REG_OFFSET)]);

	if(Button_Released(GPIO_A_DATA, RESET_GPIO_REG_OFFSET))
	{
		if(atomic_read(&button_test_enable) &&
				(atomic_read(&button_test_num) == RESET_BTN_NUM))

		{
			atomic_set(&button_test_enable, 0);
			PREPARE_WORK(&btncpy_signal10, btncpy_signal_func10);
			queue_work(btn_workqueue, &btncpy_signal10);
		}
		else
		{
			if(polling_times >= 2 && polling_times <= 3)
			{
				printk(KERN_INFO"Reset admin password & ip setting ........\n");  // May move to Reset_UserInfo_func
				PREPARE_WORK(&Reset_User_Info, Reset_UserInfo_func);
				queue_work(btn_workqueue, &Reset_User_Info);
			}
			else if(polling_times >= 6 && polling_times <= 7)
			{
				printk(KERN_INFO"Open backdoor ... \n");			//...
				PREPARE_WORK(&Open_Backdoor, Open_Backdoor_func);
				queue_work(btn_workqueue, &Open_Backdoor);
			}
			else if(polling_times >= 10)
			{
				printk(KERN_INFO"remove configuration (etc/zyxel/config) and reboot\n");

				PREPARE_WORK(&Reset_To_Default, Reset_To_Defu_func);
				queue_work(btn_workqueue, &Reset_To_Default);
			}
			else ;
		}

		polling_times = 0;

		spin_lock(&oxnas_gpio_spinlock);
		writel(readl(gcr->FALLING_INT_REG) | GPIO_MASK(RESET_GPIO_REG_OFFSET), gcr->FALLING_INT_REG);
		spin_unlock(&oxnas_gpio_spinlock);

	}
	else
	{
		printk(KERN_ERR"Reset button press\n");
		++polling_times;
		if(polling_times == 6) Beep();
		else if(polling_times == 2) Beep();
		else;

		mod_timer(&btnreset_timer, jiffies + BTN_POLLING_PERIOD);
	}

}

static void btncpy_timer_func(unsigned long in_data)
{
#ifdef POWER_RESUME_DEBUG

	static int polling_times = 0;
	struct gpio_config_registers *gcr;
	static atomic_t power_resume = ATOMIC_INIT(0);

	unsigned long data_output_set_reg, clk_output_set_reg;
	unsigned long data_output_clr_reg, clk_output_clr_reg;
	unsigned short data_pin = POWER_RESUME_DATA_GPIO_REG_OFFSET, clk_pin = POWER_RESUME_CLK_GPIO_REG_OFFSET;

	gcr = &(gpio_config_regs[GPIO_CONFIG_NUM(COPY_GPIO_REG_OFFSET)]);
	if(Button_Released(GPIO_A_DATA, COPY_GPIO_REG_OFFSET))
	{
		data_output_set_reg = gpio_config_regs[GPIO_CONFIG_NUM(data_pin)].OUTPUT_SET_REG;
		clk_output_set_reg = gpio_config_regs[GPIO_CONFIG_NUM(clk_pin)].OUTPUT_SET_REG;

		data_output_clr_reg = gpio_config_regs[GPIO_CONFIG_NUM(data_pin)].OUTPUT_CLR_REG;
		clk_output_clr_reg = gpio_config_regs[GPIO_CONFIG_NUM(clk_pin)].OUTPUT_CLR_REG;

		spin_lock(&oxnas_gpio_spinlock);
		writel(GPIO_MASK(clk_pin), clk_output_clr_reg);		// pull down clock first
		if(atomic_read(&power_resume))
			writel(GPIO_MASK(data_pin), data_output_set_reg);	// pull up data
		else
			writel(GPIO_MASK(data_pin), data_output_clr_reg);	// pull up data
		spin_unlock(&oxnas_gpio_spinlock);

		udelay(1000);
		spin_lock(&oxnas_gpio_spinlock);
		writel(GPIO_MASK(clk_pin), clk_output_set_reg);		// pull up clock
		spin_unlock(&oxnas_gpio_spinlock);
		//udelay(1000);
		// pull high clk
		//spin_lock(&oxnas_gpio_spinlock);
		//writel(GPIO_MASK(clk_pin), clk_output_clr_reg);		// pull down clock
		//spin_unlock(&oxnas_gpio_spinlock);

		if(atomic_read(&power_resume))
			atomic_set(&power_resume, 0);
		else
			atomic_set(&power_resume, 1);

		polling_times = 0;

		spin_lock(&oxnas_gpio_spinlock);
		writel(readl(gcr->FALLING_INT_REG) | GPIO_MASK(COPY_GPIO_REG_OFFSET), gcr->FALLING_INT_REG);
		spin_unlock(&oxnas_gpio_spinlock);
	}
	else
	{
		printk(KERN_ERR"Copy_button press\n");
		++polling_times;
		if(polling_times == 30) Beep();
		else;

		mod_timer(&btncpy_timer, jiffies + BTNCPY_POLLING_PERIOD);
	}
#else
	static int polling_times = 0;
	struct gpio_config_registers *gcr;


	gcr = &(gpio_config_regs[GPIO_CONFIG_NUM(COPY_GPIO_REG_OFFSET)]);
	if(Button_Released(GPIO_A_DATA, COPY_GPIO_REG_OFFSET))
	{
		if(atomic_read(&button_test_enable) &&
				(atomic_read(&button_test_num) == COPY_BTN_NUM))

		{
			atomic_set(&button_test_enable, 0);
			PREPARE_WORK(&btncpy_signal10, btncpy_signal_func10);
			queue_work(btn_workqueue, &btncpy_signal10);
		}
		else
		{
			if(btncpy_pid)
			{
				if(polling_times >= 30 && polling_times < 300) {
					PREPARE_WORK(&btncpy_signal12, btncpy_signal_func12);
					queue_work(btn_workqueue, &btncpy_signal12);
				}
				else if(polling_times >= 300)
				{
					show_state();
					show_mem();
				}
				else
				{
					PREPARE_WORK(&btncpy_signal10, btncpy_signal_func10);
					queue_work(btn_workqueue, &btncpy_signal10);
				}
			}
		}
		polling_times = 0;

		printk(KERN_ERR"Copy button release\n");
		// Enable irq line
		spin_lock(&oxnas_gpio_spinlock);
		writel(readl(gcr->FALLING_INT_REG) | GPIO_MASK(COPY_GPIO_REG_OFFSET), gcr->FALLING_INT_REG);
		spin_unlock(&oxnas_gpio_spinlock);

	}
	else
	{
		printk(KERN_ERR"Copy_button press\n");
		++polling_times;
		if(polling_times == 30) Beep();
		else if(polling_times == 300) Beep();
		else;

		mod_timer(&btncpy_timer, jiffies + BTNCPY_POLLING_PERIOD);
	}
#endif
}
//***			End of timer function to control the button		***//
//---------------------------------------------------------------------------------//


irqreturn_t gpio_A_interrupt(int irq, void *dev_id)
{
	unsigned int	int_status;

	spin_lock(&oxnas_gpio_spinlock);
	int_status = readl((volatile unsigned long *)GPIO_A_INTERRUPT_STATUS_REGISTER);
	spin_unlock(&oxnas_gpio_spinlock);

	if(int_status & GPIO_MASK(RESET_GPIO_REG_OFFSET))
	{
		int	gpio_num;
		struct gpio_config_registers *gcr;

		printk(KERN_ERR"Reset button\n");

		gpio_num = RESET_GPIO_REG_OFFSET;
		gcr = &(gpio_config_regs[GPIO_CONFIG_NUM(gpio_num)]);

		//Disable reset button GPIO line interrupt
		spin_lock(&oxnas_gpio_spinlock);
		writel(readl(gcr->FALLING_INT_REG) & ~GPIO_MASK(gpio_num), gcr->FALLING_INT_REG);
		spin_unlock(&oxnas_gpio_spinlock);

		// Init timer for reset button
		mod_timer(&btnreset_timer, jiffies + BTN_POLLING_PERIOD);

	}
	else if(int_status & GPIO_MASK(POWER_GPIO_REG_OFFSET))
	{

		int	gpio_num;
		struct gpio_config_registers *gcr;

		printk(KERN_ERR"power button\n");

		gpio_num = POWER_GPIO_REG_OFFSET;
		gcr = &(gpio_config_regs[GPIO_CONFIG_NUM(gpio_num)]);

		//Disable power button GPIO line interrupt
		spin_lock(&oxnas_gpio_spinlock);
		writel(readl(gcr->FALLING_INT_REG) & ~GPIO_MASK(gpio_num), gcr->FALLING_INT_REG);
		spin_unlock(&oxnas_gpio_spinlock);

		// Init timer for power button
		mod_timer(&btnpow_timer, jiffies + BTN_POLLING_PERIOD);

	}
	else if(int_status &  GPIO_MASK(COPY_GPIO_REG_OFFSET))
	{

		int	gpio_num;
		struct gpio_config_registers *gcr;

		printk(KERN_ERR"Copy button\n");
		gpio_num = COPY_GPIO_REG_OFFSET;
		gcr = &(gpio_config_regs[GPIO_CONFIG_NUM(gpio_num)]);

		//Disable the copy button GPIO line interrupt
		spin_lock(&oxnas_gpio_spinlock);
		writel(readl(gcr->FALLING_INT_REG) & ~GPIO_MASK(gpio_num), gcr->FALLING_INT_REG);
		spin_unlock(&oxnas_gpio_spinlock);

		// Init timer for copy button
		mod_timer(&btncpy_timer, jiffies + BTN_POLLING_PERIOD);

	}
	else;

	return IRQ_HANDLED;
}

irqreturn_t gpio_B_interrupt(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

///  						Power Resume					///

void init_power_resume_config_regs(void)
{
	unsigned short data_pin = POWER_RESUME_DATA_GPIO_REG_OFFSET, clk_pin = POWER_RESUME_CLK_GPIO_REG_OFFSET;

	spin_lock(&oxnas_gpio_spinlock);

	// disable other use for data pin
	REG_CLEAR_BITS(gpio_config_regs[GPIO_CONFIG_NUM(data_pin)].SWITCH_PRISEL_REG, GPIO_MASK(data_pin));
	REG_CLEAR_BITS(gpio_config_regs[GPIO_CONFIG_NUM(data_pin)].SWITCH_SECSEL_REG, GPIO_MASK(data_pin));
	REG_CLEAR_BITS(gpio_config_regs[GPIO_CONFIG_NUM(data_pin)].SWITCH_TERSEL_REG, GPIO_MASK(data_pin));

	// disable other use for clk pin
	REG_CLEAR_BITS(gpio_config_regs[GPIO_CONFIG_NUM(clk_pin)].SWITCH_PRISEL_REG, GPIO_MASK(clk_pin));
	REG_CLEAR_BITS(gpio_config_regs[GPIO_CONFIG_NUM(clk_pin)].SWITCH_SECSEL_REG, GPIO_MASK(clk_pin));
	REG_CLEAR_BITS(gpio_config_regs[GPIO_CONFIG_NUM(clk_pin)].SWITCH_TERSEL_REG, GPIO_MASK(clk_pin));

	// enable output enable
	REG_SET_BITS((gpio_config_regs[GPIO_CONFIG_NUM(data_pin)].OUTPUT_ENABLE_REG),GPIO_MASK(data_pin));
	REG_SET_BITS((gpio_config_regs[GPIO_CONFIG_NUM(clk_pin)].OUTPUT_ENABLE_REG),GPIO_MASK(clk_pin));

	spin_unlock(&oxnas_gpio_spinlock);
}
/////////////////////////////////////////////////////////////////////////////////////////////////


void  init_power_off_reg()
{
	int power_off_pin = POWER_OFF_GPIO_REG_OFFSET;

	spin_lock(&oxnas_gpio_spinlock);

	REG_CLEAR_BITS(gpio_config_regs[GPIO_CONFIG_NUM(power_off_pin)].SWITCH_PRISEL_REG, GPIO_MASK(power_off_pin));
	REG_CLEAR_BITS(gpio_config_regs[GPIO_CONFIG_NUM(power_off_pin)].SWITCH_SECSEL_REG, GPIO_MASK(power_off_pin));
	REG_CLEAR_BITS(gpio_config_regs[GPIO_CONFIG_NUM(power_off_pin)].SWITCH_TERSEL_REG, GPIO_MASK(power_off_pin));

	writel(GPIO_MASK(POWER_OFF_GPIO_REG_OFFSET),  GPIO_A_OUTPUT_CLEAR);
	REG_SET_BITS((gpio_config_regs[GPIO_CONFIG_NUM(power_off_pin)].OUTPUT_ENABLE_REG),GPIO_MASK(power_off_pin));


	spin_unlock(&oxnas_gpio_spinlock);
}

//
static int btncpy_open(struct inode *inode , struct file* filp)
{
	return 0;
}

static int btncpy_release(struct inode *inode , struct file *filp)
{
	return 0;
}

static ssize_t btncpy_read(struct file *file, char *buf, size_t count, loff_t *ptr)
{
	printk(KERN_INFO "Read system call is no useful\n");
	return 0;
}

static ssize_t btncpy_write(struct file * file, const char *buf, size_t count, loff_t * ppos)
{
	printk(KERN_INFO "Write system call is no useful\n");
	return 0;
}

static int btncpy_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{

	unsigned long led_index, _led_value, ret = 0;
	struct gpio_write_data	w_data;

	switch(cmd)
	{
		case BTNCPY_IOC_SET_NUM:
		  if(!capable(CAP_SYS_ADMIN)) return -EPERM;
		  btncpy_pid = arg;
		  //printk(KERN_INFO"///---   Btncpy Deamon PID = %x  ---///\n",btncpy_pid);
		break;
		//case DISK_IOC_SUSPEND:
			//printk(KERN_ERR"///---   suspending time = %d\n   ---///\n", arg);
		//	if(arg == 0)
		//		DisablePowerSaving();
		//	else
		//	{   DisablePowerSaving();
		//		EnablePowerSaving(arg);
		//	}
		//break;
		case LED_SET_CTL_IOC_NUM:		// Just set leds, no check.
			ret = set_led_config(arg);
			if(ret < 0)
				return ret;
			break;
		case LED_GET_CTL_IOC_NUM:
			led_index =  arg;

			if(led_index >= LED_NUMS)
				return -ENOTTY;
			spin_lock(&led_status_lock);
			_led_value = led_status[led_index];

			spin_unlock(&led_status_lock);

			return _led_value;

			break;
		case BUZ_SET_CTL_IOC_NUM:
			set_buzzer(arg);
			break;
		case POWER_RESUME_CLR:
		{
			unsigned long data_output_set_reg, clk_output_set_reg;
			unsigned long data_output_clr_reg, clk_output_clr_reg;
			unsigned short data_pin = POWER_RESUME_DATA_GPIO_REG_OFFSET, clk_pin = POWER_RESUME_CLK_GPIO_REG_OFFSET;

			data_output_set_reg = gpio_config_regs[GPIO_CONFIG_NUM(data_pin)].OUTPUT_SET_REG;
			clk_output_set_reg = gpio_config_regs[GPIO_CONFIG_NUM(clk_pin)].OUTPUT_SET_REG;

			data_output_clr_reg = gpio_config_regs[GPIO_CONFIG_NUM(data_pin)].OUTPUT_CLR_REG;
			clk_output_clr_reg = gpio_config_regs[GPIO_CONFIG_NUM(clk_pin)].OUTPUT_CLR_REG;

			spin_lock(&oxnas_gpio_spinlock);
			writel(GPIO_MASK(clk_pin), clk_output_clr_reg);		// pull down clock first
			writel(GPIO_MASK(data_pin), data_output_clr_reg);	// pull down data
			spin_unlock(&oxnas_gpio_spinlock);

			udelay(1000);
			spin_lock(&oxnas_gpio_spinlock);
			writel(GPIO_MASK(clk_pin), clk_output_set_reg);		// pull up clock
			spin_unlock(&oxnas_gpio_spinlock);

			//udelay(1000);
			// pull high clk
			//spin_lock(&oxnas_gpio_spinlock);
			//writel(GPIO_MASK(clk_pin), clk_output_clr_reg);		// pull down clock
			//spin_unlock(&oxnas_gpio_spinlock);

			break;
		}
		case POWER_RESUME_SET:
		{
			unsigned long data_output_set_reg, clk_output_set_reg;
			unsigned long data_output_clr_reg, clk_output_clr_reg;
			unsigned short data_pin = POWER_RESUME_DATA_GPIO_REG_OFFSET, clk_pin = POWER_RESUME_CLK_GPIO_REG_OFFSET;

			data_output_set_reg = gpio_config_regs[GPIO_CONFIG_NUM(data_pin)].OUTPUT_SET_REG;
			clk_output_set_reg = gpio_config_regs[GPIO_CONFIG_NUM(clk_pin)].OUTPUT_SET_REG;

			data_output_clr_reg = gpio_config_regs[GPIO_CONFIG_NUM(data_pin)].OUTPUT_CLR_REG;
			clk_output_clr_reg = gpio_config_regs[GPIO_CONFIG_NUM(clk_pin)].OUTPUT_CLR_REG;

			spin_lock(&oxnas_gpio_spinlock);
			writel(GPIO_MASK(clk_pin), clk_output_clr_reg);		// pull down clock first
			writel(GPIO_MASK(data_pin), data_output_set_reg);	// pull up data
			spin_unlock(&oxnas_gpio_spinlock);
			udelay(1000);
			spin_lock(&oxnas_gpio_spinlock);
			writel(GPIO_MASK(clk_pin), clk_output_set_reg);		// pull up clock
			spin_unlock(&oxnas_gpio_spinlock);
			udelay(1000);
			// pull high clk
			//spin_lock(&oxnas_gpio_spinlock);
			//writel(GPIO_MASK(clk_pin), clk_output_clr_reg);		// pull down clock
			//spin_unlock(&oxnas_gpio_spinlock);

			break;
		}
		case GPIO_WRITE_IOC_NUM:
			if(copy_from_user(&w_data, (void __user*)arg, sizeof(struct gpio_write_data)) == 0)
			{
				if(GPIO_CHECK_MAGIC_NUM(w_data.gpio_query_addr))
				{
					unsigned gpio_query_index;

					gpio_query_index = GET_GPIO_QUERY_NUM(w_data.gpio_query_addr);

					writel(w_data.value, gpio_table[gpio_query_index]);

					return readl(gpio_table[gpio_query_index]);
				}
			}
			return -ENOTTY;
			break;
		case GPIO_READ_IOC_NUM:
			if(GPIO_CHECK_MAGIC_NUM(arg))
			{
				unsigned gpio_query_index;

				gpio_query_index = GET_GPIO_QUERY_NUM(arg);

				return readl(gpio_table[gpio_query_index]);
			}
			break;
		case BUTTON_TEST_IN_IOC_NUM:
			btncpy_pid = arg >> 3;
			atomic_set(&button_test_enable, 1);
			atomic_set(&button_test_num, arg & 0x7);
			break;

		case BUTTON_TEST_OUT_IOC_NUM:
			atomic_set(&button_test_enable, 0);
			atomic_set(&button_test_num, BUTTON_NUM);

		default :
			return -ENOTTY;
	}

	return 0;
}

struct file_operations btncpy_fops =
{
	owner: 		THIS_MODULE,
	read:  		btncpy_read,
	write: 		btncpy_write,
	ioctl:		btncpy_ioctl,
	open:		btncpy_open,
	release:	btncpy_release,
};

void btncpy_cleanup_module(void)
{
	unregister_chrdev_region(gpio_dev, btncpy_nr_devs);
	destroy_workqueue(btn_workqueue);
}


void DumpGPIO_A( void )
{
	printk( KERN_ERR "================= GPIO Dump\n"
	  "     GPIO_A_DATA(%x)                            0x%08x\n"
	  "     GPIO_A_OUTPUT_ENABLE(%x)                   0x%08x\n"
	  "     GPIO_A_INTERRUPT_ENABLE                0x%08x\n"
	  "     GPIO_A_INTERRUPT_EVENT                 0x%08x\n"
	  "     GPIO_A_OUTPUT_VALUE                    0x%08x\n"
	  "     GPIO_A_OUTPUT_SET                      0x%08x\n"
	  "     GPIO_A_OUTPUT_CLEAR                    0x%08x\n"
	  "     GPIO_A_OUTPUT_ENABLE_SET               0x%08x\n"
	  "     GPIO_A_OUTPUT_ENABLE_CLEAR             0x%08x\n"
	  "     GPIO_A_INPUT_DEBOUNCE_ENABLE           0x%08x\n"
	  "     GPIO_A_RISING_EDGE_ACTIVE_HIGH_ENABLE  0x%08x\n"
	  "     GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE  0x%08x\n"
	  "     GPIO_A_RISING_EDGE_DETECT              0x%08x\n"
	  "     GPIO_A_FALLING_EDGE_DETECT             0x%08x\n"
	  "     GPIO_A_LEVEL_INTERRUPT_ENABLE          0x%08x\n"
	  "     GPIO_A_INTERRUPT_STATUS_REGISTER       0x%08x\n",
	  GPIO_A_DATA,
        readl( GPIO_A_DATA                           ),
	 GPIO_A_OUTPUT_ENABLE,
        readl( GPIO_A_OUTPUT_ENABLE                  ),
        readl( GPIO_A_INTERRUPT_ENABLE               ),
        readl( GPIO_A_INTERRUPT_EVENT                ),
        readl( GPIO_A_OUTPUT_VALUE                   ),
        readl( GPIO_A_OUTPUT_SET                     ),
        readl( GPIO_A_OUTPUT_CLEAR                          ),
        readl( GPIO_A_OUTPUT_ENABLE_SET              ),
        readl( GPIO_A_OUTPUT_ENABLE_CLEAR            ),
        readl( GPIO_A_INPUT_DEBOUNCE_ENABLE          ),
        readl( GPIO_A_RISING_EDGE_ACTIVE_HIGH_ENABLE ),
        readl( GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE ),
        readl( GPIO_A_RISING_EDGE_DETECT             ),
        readl( GPIO_A_FALLING_EDGE_DETECT            ),
        readl( GPIO_A_LEVEL_INTERRUPT_ENABLE         ),
        readl( GPIO_A_INTERRUPT_STATUS_REGISTER      ) );

}


void DumpGPIO_B( void )
{
	printk( KERN_ERR "================= GPIO Dump\n"
	  "     GPIO_B_DATA                            0x%08x\n"
	  "     GPIO_B_OUTPUT_ENABLE                   0x%08x\n"
	  "     GPIO_B_INTERRUPT_ENABLE                0x%08x\n"
	  "     GPIO_B_INTERRUPT_EVENT                 0x%08x\n"
	  "     GPIO_B_OUTPUT_VALUE                    0x%08x\n"
	  "     GPIO_B_OUTPUT_SET                      0x%08x\n"
	  "     GPIO_B_OUTPUT_CLEAR                    0x%08x\n"
	  "     GPIO_B_OUTPUT_ENABLE_SET               0x%08x\n"
	  "     GPIO_B_OUTPUT_ENABLE_CLEAR             0x%08x\n"
	  "     GPIO_B_INPUT_DEBOUNCE_ENABLE           0x%08x\n"
	  "     GPIO_B_RISING_EDGE_ACTIVE_HIGH_ENABLE  0x%08x\n"
	  "     GPIO_B_FALLING_EDGE_ACTIVE_LOW_ENABLE  0x%08x\n"
	  "     GPIO_B_RISING_EDGE_DETECT              0x%08x\n"
	  "     GPIO_B_FALLING_EDGE_DETECT             0x%08x\n"
	  "     GPIO_B_LEVEL_INTERRUPT_ENABLE          0x%08x\n"
	  "     GPIO_B_INTERRUPT_STATUS_REGISTER       0x%08x\n",
        readl( GPIO_B_DATA                           ),
        readl( GPIO_B_OUTPUT_ENABLE                  ),
        readl( GPIO_B_INTERRUPT_ENABLE               ),
        readl( GPIO_B_INTERRUPT_EVENT                ),
        readl( GPIO_B_OUTPUT_VALUE                   ),
        readl( GPIO_B_OUTPUT_SET                     ),
        readl( GPIO_B_OUTPUT_CLEAR                          ),
        readl( GPIO_B_OUTPUT_ENABLE_SET              ),
        readl( GPIO_B_OUTPUT_ENABLE_CLEAR            ),
        readl( GPIO_B_INPUT_DEBOUNCE_ENABLE          ),
        readl( GPIO_B_RISING_EDGE_ACTIVE_HIGH_ENABLE ),
        readl( GPIO_B_FALLING_EDGE_ACTIVE_LOW_ENABLE ),
        readl( GPIO_B_RISING_EDGE_DETECT             ),
        readl( GPIO_B_FALLING_EDGE_DETECT            ),
        readl( GPIO_B_LEVEL_INTERRUPT_ENABLE         ),
        readl( GPIO_B_INTERRUPT_STATUS_REGISTER      ) );

}

#define BUTTON_INTERRUPT_CONFIG_REGISTER(reg_set, gpio_num_mask)						\
	writel(readl(reg_set.SWITCH_PRISEL_REG) & ~gpio_num_mask, reg_set.SWITCH_PRISEL_REG);			\
	writel(readl(reg_set.SWITCH_SECSEL_REG) & ~gpio_num_mask, reg_set.SWITCH_SECSEL_REG);			\
	writel(readl(reg_set.SWITCH_TERSEL_REG) & ~gpio_num_mask, reg_set.SWITCH_TERSEL_REG);			\
	writel(gpio_num_mask, reg_set.SWITCH_CLR_OE_REG);							\
	writel(readl(reg_set.DEBOUNCE_REG)    | gpio_num_mask, reg_set.DEBOUNCE_REG);				\
	writel(readl(reg_set.LEVEL_INT_REG)   | gpio_num_mask, reg_set.LEVEL_INT_REG);				\
	writel(readl(reg_set.FALLING_INT_REG) | gpio_num_mask, reg_set.FALLING_INT_REG)



/*  HTP test pin */

static void init_htp_pin()	/* Set as gpio input */
{
	struct gpio_config_registers *regs;
	unsigned gpio_num_mask, gpio_pin;

	gpio_pin = HTP_GPIO_REG_PIN;
	gpio_num_mask = GPIO_MASK(gpio_pin);

	regs = &(gpio_config_regs[GPIO_CONFIG_NUM(gpio_pin)]);

	writel(readl(regs->SWITCH_PRISEL_REG) & ~gpio_num_mask, regs->SWITCH_PRISEL_REG);
	writel(readl(regs->SWITCH_SECSEL_REG) & ~gpio_num_mask, regs->SWITCH_SECSEL_REG);
	writel(readl(regs->SWITCH_TERSEL_REG) & ~gpio_num_mask, regs->SWITCH_TERSEL_REG);
	writel(gpio_num_mask, regs->SWITCH_CLR_OE_REG);
}

static struct proc_dir_entry *htp_status;
static int htp_status_read_fn(char *buf, char **start, off_t offset,
                int count, int *eof, void *data)
{
	unsigned long len;
	if((readl(GPIO_A_DATA) & GPIO_MASK(HTP_GPIO_REG_PIN )))
		len = sprintf(buf, "1\n");
	else
		len = sprintf(buf, "0\n");

	*eof = 1;

	return len;

}

static int htp_status_write_fn(struct file *file, const char __user *buffer,
                unsigned long count, void *data)
{
	return 0;
}


///		proc fs to enable/disable poweroff, restart, ...		///
//
static struct proc_dir_entry  *shutdown_status;


static int shutdown_enable_status_read_fn(char *buf, char **start, off_t offset,
		int count, int *eof, void *data)
{
	int	len;

	len = sprintf(buf, "%d\n", atomic_read(&shutdown_enable));
	*eof = 1;

	return len;
}

static int shutdown_enable_status_write_fn(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
	char my_buf[10];

	if(count > 2)
	{
		printk(KERN_ERR"Fail to write to proc\n");
		return -EFAULT;
	}

	copy_from_user(my_buf, buffer, count);

	if(my_buf[0] == '0')
		atomic_set(&shutdown_enable, 0);
	else
		atomic_set(&shutdown_enable, 1);

	return count;

}

///////////////////////////////////////////////////////////////////
static struct proc_dir_entry *led_proc;

static int led_ctrl_read_fn(char *buf, char **start, off_t offset,
		int count, int *eof, void *data)
{
	int len;

	len = sprintf(buf, "0\n");
	*eof = 1;

	return len;
}

static int led_ctrl_write_fn(struct file* file, const char __user *buffer,
				unsigned long count, void *data)
{
	char		my_buf[50];
	unsigned long 	led_data;
	int	ret;

	if(count > 50)
	{
		printk(KERN_ERR"Exceed write buffer\n");
		return -EFAULT;
	}

	copy_from_user(my_buf, buffer, count);
	led_data = simple_strtoul(my_buf, NULL, 10);

	ret = set_led_config(led_data);

	if(ret < 0)
		return ret;
	else
		return count;

}

static struct proc_dir_entry* gpio_proc;

static int gpio_read_fn(char *buf, char **start, off_t offset,
                int count, int *eof, void *data)
{
	DumpGPIO_A();
	DumpGPIO_B();
	*eof = 1;
	return 0;
}

static int gpio_write_fn(struct file* file, const char __user *buffer,
                                unsigned long count, void *data)
{
	return 0;
}


//////////////////////////////////////////////////////////////////


static int __init gpio_init(void)
{
	int	result = 0;
	int	err = 0;
	unsigned long flags;

	//-------------------------------------------------------------------------------//
	//result = register_chrdev(btncpy_major, "btncpy", &btncpy_fops);

	err = alloc_chrdev_region(&gpio_dev, 0, btncpy_nr_devs, "gpio");
	if(err < 0)
	{
		printk(KERN_ERR"%s: failed to allocate char dev region\n", __FILE__);
		return -1;
	}


	btncpy_cdev = cdev_alloc();
	btncpy_cdev->ops = &btncpy_fops;
	btncpy_cdev->owner = THIS_MODULE;
	err = cdev_add(btncpy_cdev, gpio_dev, 1);

	if(err) printk(KERN_INFO "Error adding device\n");

	//-------------------------------------------------------------------------------//

	shutdown_status = create_proc_entry("shutdownStatus", 0644, NULL);
	if(shutdown_status != NULL)
	{
		shutdown_status->read_proc = shutdown_enable_status_read_fn;
		shutdown_status->write_proc = shutdown_enable_status_write_fn;
	}

	//-------------------------------------------------------------------------------//
	htp_status = create_proc_entry("htp", 0644, NULL);
	if(htp_status != NULL)
	{
		htp_status->read_proc = htp_status_read_fn;
		htp_status->write_proc = htp_status_write_fn;
	}


	/* ------------------------------------------------------------------------------*/


	led_proc = create_proc_entry("led", 0644, NULL);
	if(led_proc != NULL)
	{
		led_proc->read_proc = led_ctrl_read_fn;
		led_proc->write_proc = led_ctrl_write_fn;
	}

	gpio_proc = create_proc_entry("gpio", 0644, NULL);
	if(gpio_proc != NULL)
	{
		gpio_proc->read_proc = gpio_read_fn;
		gpio_proc->write_proc = gpio_write_fn;
	}

	//-------------------------------------------------------------------------------//


	//DumpGPIO_A();
	//DumpGPIO_B();

	init_htp_pin();

	init_power_off_reg();

        init_led_blk_timer();
        init_led_blk_timer_data();
	init_all_led_config_regs();

	init_power_resume_config_regs();

	init_buzzer_control();

	printk(KERN_ERR"Init led blk\n");


	//DumpGPIO_A();
	//DumpGPIO_B();

	init_timer(&btnpow_timer);
	btnpow_timer.function = btnpow_timer_func;
	btnpow_timer.data = 0;

	init_timer(&btnreset_timer);
	btnreset_timer.function = btnreset_timer_func;
	btnreset_timer.data = 0;

	init_timer(&btncpy_timer);
	btncpy_timer.function = btncpy_timer_func;
	btncpy_timer.data = 0;
	btn_workqueue = create_workqueue("button controller");


	/** init and enable interrupt for button **/
	//Disable interrupt for all gpio pin

	writel(0x0, GPIO_A_FALLING_EDGE_ACTIVE_LOW_ENABLE);
	writel(0x0, GPIO_A_LEVEL_INTERRUPT_ENABLE);

	spin_lock_irqsave(&oxnas_gpio_spinlock, flags);


	BUTTON_INTERRUPT_CONFIG_REGISTER(
		(gpio_config_regs[GPIO_CONFIG_NUM(RESET_GPIO_REG_OFFSET)]),
		(GPIO_MASK(RESET_GPIO_REG_OFFSET))
		);

	BUTTON_INTERRUPT_CONFIG_REGISTER(
		(gpio_config_regs[GPIO_CONFIG_NUM(POWER_GPIO_REG_OFFSET)]),
		(GPIO_MASK(POWER_GPIO_REG_OFFSET))
		);

	BUTTON_INTERRUPT_CONFIG_REGISTER(
		(gpio_config_regs[GPIO_CONFIG_NUM(COPY_GPIO_REG_OFFSET)]),
		(GPIO_MASK(COPY_GPIO_REG_OFFSET))
		);

	printk(KERN_ERR"Enable Interrupt\n");

	mdelay(10);

	result = request_irq(GPIO_1_INTERRUPT, gpio_A_interrupt, IRQF_DISABLED,"Button Event", NULL);
	if(result)
	{
		printk(KERN_ERR"Power Button : Can't get assigned irq %d\n",GPIO_1_INTERRUPT);
		goto IRQ_REQUEST_FAILE;
	}

	//DumpGPIO_A();
	//DumpGPIO_B();



	spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

	return 0;

IRQ_REQUEST_FAILE:
	btncpy_cleanup_module();

	return result;
}

static void __exit gpio_exit(void)
{
	free_irq(GPIO_1_INTERRUPT, NULL);

	if(timer_pending(&bz_timer))
		del_timer(&bz_timer);

	if(timer_pending(&led0_blk_timer))
		del_timer(&led0_blk_timer);

	if(timer_pending(&led1_blk_timer))
		del_timer(&led1_blk_timer);

	if(timer_pending(&led2_blk_timer))
		del_timer(&led2_blk_timer);

	if(timer_pending(&led3_blk_timer))
		del_timer(&led3_blk_timer);

	if(timer_pending(&led4_blk_timer))
		del_timer(&led4_blk_timer);

	 if(timer_pending(&btnpow_timer))
		del_timer(&btnpow_timer);
	 if(timer_pending(&btnreset_timer))
		del_timer(&btnreset_timer);

	if(timer_pending(&btncpy_timer))
		del_timer(&btncpy_timer);


	remove_proc_entry("htp", NULL);
	remove_proc_entry("shutdownStatus", NULL);
	remove_proc_entry("led", NULL);

	remove_proc_entry("gpio", NULL);

	// disable bz
	writel(readl(SYS_CTRL_GPIO_PWMSEL_CTRL_0) & ~(1 << BZ_GPIO_REG_OFFSET), SYS_CTRL_GPIO_PWMSEL_CTRL_0);
	btncpy_cleanup_module();
}

module_init(gpio_init);
module_exit(gpio_exit);
