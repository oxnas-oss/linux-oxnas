#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <linux/ioctl.h>
#include <asm/hardware.h>
#include <linux/proc_fs.h>

MODULE_LICENSE("GPL v2");

#define PRODUCT_INTERNAL_TEST_MAGIC 't'
#define SATA0_TEST	_IO(PRODUCT_INTERNAL_TEST_MAGIC, 1)
#define SATA1_TEST	_IO(PRODUCT_INTERNAL_TEST_MAGIC, 2)
#define USB0_TEST	_IO(PRODUCT_INTERNAL_TEST_MAGIC, 3)

static unsigned long sata0_test_regs[10] = {
	SATA0_REGS_BASE + 0x3c,				/* Version register */
	-1,
};

static unsigned long sata0_test_reg_vals[10] = {
	0x1f2,
	-1
};



static unsigned long sata1_test_regs[10] = {
	SATA1_REGS_BASE + 0x3c,				/* Version register */
	-1,
};

static unsigned long sata1_test_reg_vals[10] = {
	0x1f2,
	-1
};


#if 0
#define USB0_TEST_REG
#define USB0_TEST_REG_VAL
static unsigned long usb0_test_regs[10] = {
	USB0_TEST_REG0,
	-1,
};

static unsigned long usb0_test_regs[10] = {
	USB0_TEST_REG_VAL0,
	-1
};

#endif

static struct proc_dir_entry	*sata0_test;
static int sata0_read_fn(char *buf, char **start, off_t offset,
		int count, int *eof, void *data)
{
	int	t_num, is_ok, len;

	is_ok = 1;
	for(t_num = 0 ; sata0_test_regs[t_num] != -1 && sata0_test_reg_vals[t_num] != -1; t_num++){
		if(readl(sata0_test_regs[t_num]) != sata0_test_reg_vals[t_num]){
			is_ok = 0;
			break;
		}
	}

	if(is_ok)
		len = sprintf(buf, "OK");
	else
		len = sprintf(buf, "FAIL");

	*eof = 1;

	return len;



}
static int sata0_write_fn(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
	return 0;
}



static struct proc_dir_entry	*sata1_test;
static int sata1_read_fn(char *buf, char **start, off_t offset,
		int count, int *eof, void *data)
{
	int	t_num, is_ok, len;

	is_ok = 1;
	for(t_num = 0 ; sata1_test_regs[t_num] != -1 && sata1_test_reg_vals[t_num] != -1; t_num++){
		if(readl(sata1_test_regs[t_num]) != sata1_test_reg_vals[t_num]){
			is_ok = 0;
			break;
		}
	}

	if(is_ok)
		len = sprintf(buf, "OK");
	else
		len = sprintf(buf, "FAIL");

	*eof = 1;

	return len;



}
static int sata1_write_fn(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
	return 0;
}


static int __init  hwtest_init(void)
{
	int	err;

	sata0_test = create_proc_entry("sata0Test", 0644, NULL);
	if(sata0_test != NULL)
	{
		sata0_test->read_proc =  sata0_read_fn;
		sata0_test->write_proc = sata0_write_fn;
	}

	sata1_test = create_proc_entry("sata1Test", 0644, NULL);
	if(sata1_test != NULL)
	{
		sata1_test->read_proc = sata1_read_fn;
		sata1_test->write_proc = sata1_write_fn;
	}

	return err;

}

static void __exit hwtest_exit(void)
{
	remove_proc_entry("sata0Test", NULL);
	remove_proc_entry("sata1Test", NULL);
}

module_init(hwtest_init);
module_exit(hwtest_exit);
