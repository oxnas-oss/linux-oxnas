/*
 * linux/arch/arm/mach-oxnas/leds.c
 *
 * Copyright (C) 2005 Oxford Semiconductor Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/interrupt.h>

#include <linux/platform_device.h>
//#include <asm/arch/pxa-regs.h>


#include <linux/leds.h>
 
#include <asm/hardware.h>

#define PWM_RESOLUTION   16
#define LED_1_MASK     0
#define LED_2_MASK     (1 << (34-32))
#define TIMER_LED_MODE    TIMER_MODE_PERIODIC


// Setup timer 2 prescaler, free-running operation and start it
// This will not be used to generate interrupt, just as a hi-res source of
// timing information
#define PERIODIC_INTERRUPT        (TIMER_PRESCALE_1 << TIMER_PRESCALE_BIT)        | \
        						  (TIMER_LED_MODE          << TIMER_MODE_BIT)     | \
						          (TIMER_ENABLE_ENABLE     << TIMER_ENABLE_BIT);
/*
 * prescaler is set to 1 so 25Mhz clock will genereate 40us ticks to the 
 * counter. testing rate is 5ms interrupts to divider should be 5000/40 counts
 * 125.
 * LED frame rate is 5ms*16 or every 80ms ie 12.5 HZ  (slow enough to flicker).
 */
#define FAST_TIMER_INT         125

#define oxnas_leds_cpu   4
#define oxnas_leds_timer 8

static u32 frame_buffer [PWM_RESOLUTION];
static u32 * next_led;
 
#define writel(data,address) (*(volatile u32 *) address = data)
#define readl(address)       (*(volatile u32 *) address)
#define oxnas_set_gpio_value(a,b)
#define oxnas_get_gpio_value(a) 0
#define oxnas_set_gpio_output(a,b)

/* save timer status before use */
static u32 timer_load;
static u32 timer_control;


 /* fast interrupt handler coded in assembler */
 
static irqreturn_t  fast_interrupt_handler(int irq, void * dev_id, struct pt_regs *regs)
{
	/*
	 * register usage :
	 * 	r0 - pointer to next LED output state
	 *  r1 - current GPIO status
	 *  r2 - masked LED output state
	 *
asm (" 	; make registers for handler - r0, r1, r2
		; load r0 with pointer to LED status for output
		; load r1 with current status
		; mask r1 to clear current state
		; load r2 with next LED status
		; mask unused GPIOs from r2
		; OR r2 with r1 into r1
		; output r1
		; repeat with second GPIO register.
		; increment and mask pointer
		; save new pointer.
	");		
	*/
	
	if (next_led < frame_buffer ||
	    next_led >= frame_buffer+PWM_RESOLUTION) next_led=frame_buffer;
	writel( (readl(GPIO_A_OUTPUT_VALUE) & ~LED_1_MASK) | (*next_led | LED_1_MASK), GPIO_A_OUTPUT_VALUE);
	writel( (readl(GPIO_B_OUTPUT_VALUE) & ~LED_2_MASK) | (*next_led | LED_2_MASK), GPIO_B_OUTPUT_VALUE);
	++next_led;
	return IRQ_HANDLED;
}


static void ramp_power_on_leds(unsigned long data);

DEFINE_TIMER (power_ramp_timer, ramp_power_on_leds, 0, 0);

enum { POWER_ON,
	NUMBER_LEDS};
	
static u16 offset[NUMBER_LEDS] = {(34-32)};

static u16 led [NUMBER_LEDS];

#define MAX_BRIGHTNESS   15
static void set_led(u16 led, u16 value)
{
	u16 index;
	u32 *fb = frame_buffer;
	u32 led_bit = 1 << offset[led];
	
	for (index = 0; index < PWM_RESOLUTION; index ++)
	{
		*fb = (*fb | ~led_bit) &  (value < index ? led_bit : 0 );
	}
}

static void ramp_power_on_leds(unsigned long data)
{
	if (led[POWER_ON] < MAX_BRIGHTNESS) {
		 set_led(POWER_ON, ++led[POWER_ON]);
		 mod_timer(&power_ramp_timer, (power_ramp_timer.expires + msecs_to_jiffies(64)) );
	}
	else del_timer(&power_ramp_timer);
}

static void oxnasled_power_on_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	/* set group of 4 leds to brightness passed in */
	static enum led_brightness current_bright = 0;
	
	if (value != current_bright) 
	{
		if (value == 0) {
			current_bright = 0;
			led[POWER_ON]=0;
		    set_led(POWER_ON, 0);
			
		}
		else
		{
			power_ramp_timer.expires = jiffies + msecs_to_jiffies(64);
			add_timer(&power_ramp_timer);
		}
	}
}

static struct led_classdev oxnas_power_on_led = {
	.name			= "oxnas:power_on",
	.brightness_set		= oxnasled_power_on_set,
};


#ifdef CONFIG_PM
static int oxnasled_suspend(struct platform_device *dev, pm_message_t state)
{
#ifdef CONFIG_LEDS_TRIGGERS
	if (oxnas_amber_led.trigger && strcmp(oxnas_amber_led.trigger->name, "sharpsl-charge"))
#endif
		led_classdev_suspend(&oxnas_amber_led);
	led_classdev_suspend(&oxnas_green_led);
	return 0;
}

static int oxnasled_resume(struct platform_device *dev)
{
	led_classdev_resume(&oxnas_amber_led);
	led_classdev_resume(&oxnas_green_led);
	return 0;
}
#endif

static int oxnasled_probe(struct platform_device *pdev)
{
	int ret;
	
	/* save timer 2 state for restoring later. */
	
	timer_load = *(volatile u32*) TIMER2_LOAD;
	timer_control = *(volatile u32*) TIMER2_CONTROL;
    *((volatile unsigned long*)TIMER2_CONTROL) = 0;
	
	
	/* set up timer 2 for LED CONTROL. */

	

    ret = request_irq(TIMER_2_INTERRUPT,  fast_interrupt_handler, SA_INTERRUPT, "led_pwm", 0);
    if (ret < 0) 
    	return ret;
    	
	*(volatile u32*) TIMER2_LOAD = FAST_TIMER_INT;
	*(volatile u32*) TIMER2_CONTROL = PERIODIC_INTERRUPT;
    	
	ret = led_classdev_register(&pdev->dev, &oxnas_power_on_led);
	if (ret < 0) goto error_1;
	
	return ret;
	
error_1:		
	free_irq(TIMER_2_INTERRUPT, 0);
	return ret;
}

static int oxnasled_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&oxnas_power_on_led);
	
    *((volatile unsigned long*)TIMER2_CONTROL) = 0;

	free_irq(TIMER_2_INTERRUPT, 0);

    *(volatile u32*) TIMER2_LOAD = timer_load;
	*(volatile u32*) TIMER2_CONTROL = timer_control;
	
	return 0;
}

static struct platform_driver oxnasled_driver = {
	.probe		= oxnasled_probe,
	.remove		= oxnasled_remove,
#ifdef CONFIG_PM
	.suspend	= oxnasled_suspend,
	.resume		= oxnasled_resume,
#endif
	.driver		= {
		.name		= "oxnas-led",
	},
};

static int __init oxnasled_init(void)
{
	return platform_driver_register(&oxnasled_driver);
}

static void __exit oxnasled_exit(void)
{
 	platform_driver_unregister(&oxnasled_driver);
}

module_init(oxnasled_init);
module_exit(oxnasled_exit);

MODULE_AUTHOR("John Larkworthy <john.larkworthy@oxsem.com");
MODULE_DESCRIPTION("OXNAS front pannel LED driver");
MODULE_LICENSE("GPL");
