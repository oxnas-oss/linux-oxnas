/*
 * OX800 I2C interface.
 *
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/stddef.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/i2c-algo-oxnas.h>

#include <asm/io.h>
#include <asm/irq.h>


static void
oxnas_iic_init(struct i2c_algo_oxnas_data *data)
{
int i=100;
	
	printk(KERN_INFO "oxnas_iic_init: i2c OX800 driver init\n");
	data->iMaxAutoIncTransfer_ = 1;
	data->iTransferInProgress_ = 0;
	data->iError_		   = 0;
	data->iReadBufferLength_	   = 0;
	data->i2c	   	   = (volatile i2c_registers_oxnas_t  *) I2C_BASE;
	
	// dedicate the GPIO over to i2c.
	*( (volatile unsigned long * ) SYS_CTRL_GPIO_PRIMSEL_CTRL_0 ) &= ~(1UL <<  (SECONDARY_FUNCTION_ENABLE_I2C_SCL  & 0x0000001F) );
	*( (volatile unsigned long * ) SYS_CTRL_GPIO_PRIMSEL_CTRL_0 ) &= ~(1UL <<  (SECONDARY_FUNCTION_ENABLE_I2C_SDA  & 0x0000001F) );
	*( (volatile unsigned long * ) SYS_CTRL_GPIO_PRIMSEL_CTRL_0 ) &= ~(1UL <<  (SECONDARY_FUNCTION_ENABLE_I2C_SCS  & 0x0000001F) );

	*( (volatile unsigned long * ) SYS_CTRL_GPIO_SECSEL_CTRL0   ) |= (1UL <<    (SECONDARY_FUNCTION_ENABLE_I2C_SCL  & 0x0000001F) );	
	*( (volatile unsigned long * ) SYS_CTRL_GPIO_SECSEL_CTRL0   ) |= (1UL <<    (SECONDARY_FUNCTION_ENABLE_I2C_SDA  & 0x0000001F) );	
	*( (volatile unsigned long * ) SYS_CTRL_GPIO_SECSEL_CTRL0   ) |= (1UL <<    (SECONDARY_FUNCTION_ENABLE_I2C_SCS  & 0x0000001F) );	

}

static int oxnas_install_isr(int irq, void (*func)(void *, void *), void *data)
{
	int result;
	printk(KERN_INFO "oxnas_install_isr: i2c OX800 isr init\n");

	/* install interrupt handler */
	if ((result = request_irq(irq, func, 0, "i2c-oxnas", (struct i2c_adapter*) data)) < 0) {
		printk(KERN_ERR
		       "i2c-oxnas - failed to attach interrupt\n");
		return result;
	}

	return 0;
}

static void oxnas_remove_isr(int irq, void *data)
{
	int result;
	printk(KERN_INFO "oxnas_remove_isr: i2c OX800 isr remove\n");

	/* remove interrupt handler */
	free_irq(irq, (struct i2c_adapter*) data);
}

static struct i2c_algo_oxnas_data oxnas_data = {
	.setisr   = oxnas_install_isr,
	.clearisr = oxnas_remove_isr
};

static struct i2c_adapter oxnas_ops = {
	.owner		= THIS_MODULE,
	.name		= "oxnas i2c module",
	.id		= 0x01, //I2C_HW_OXNAS,
	.algo_data	= &oxnas_data,
};

int __init i2c_oxnas_init(void)
{
	printk(KERN_INFO "$Gi2c_oxnas_init: i2c oxnas bus driver\n");

	/* reset hardware to sane state */
	oxnas_iic_init(&oxnas_data);

	if (i2c_oxnas_algo_add_bus(&oxnas_ops) < 0) {
		printk(KERN_ERR "i2c-oxnas: Unable to register with I2C\n");
		return -ENODEV;
	}
	
	return 0;
}

void __exit i2c_oxnas_exit(void)
{
	i2c_oxnas_algo_del_bus(&oxnas_ops);
}

MODULE_AUTHOR("Chris Ford <...@oxsemi>");
MODULE_DESCRIPTION("I2C-Bus adapter routines for ox800 boards");

module_init(i2c_oxnas_init);
module_exit(i2c_oxnas_exit);
