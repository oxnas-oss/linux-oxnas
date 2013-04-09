/*
 * n2510_flash.c
 *
 * Handle mapping of the flash on N2510 board
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>


/* N2510 flash layout
 * 0 : 0x4100 0000 - 0x4101 FFFF : u-boot
 * 1 : 0x4102 0000 - 0x410D FFFF : user config
 * 2 : 0x410E 0000 - 0x4129 FFFF : kernel
 * 3 : 0x412A 0000 - 0x413D FFFF : ramdisk
 * 4 : 0x413E 0000 - 0x413F FFFF : u-boot config
 */

#define WINDOW_ADDR 0x41000000
#define WINDOW_SIZE 0x00400000


/* partition_info gives details on the logical partitions that the split the
 * single flash device into. If the size if zero we use up to the end of the
 * device. */
static struct mtd_partition partition_info[]={
// OX810
/*
	{
		.name		= "u-boot",
		.offset 	= 0,
		.size		= 0x020000,
//		.mask_flags	= MTD_WRITEABLE
	},
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x190000
	},
	{
		.name		= "ramdisk",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x230000
	},
	{
		.name		= "u-boot config",
		.offset		= MTDPART_OFS_APPEND,
	},
*/
// N2510
	{
		.name		= "u-boot",
		.offset 	= 0,
		.size		= 0x020000,
//		.mask_flags	= MTD_WRITEABLE
	},
	{
		.name		= "user config",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x0C0000
	},
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x1C0000
	},
	{
		.name		= "ramdisk",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x140000
	},
	{
		.name		= "u-boot config",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x020000
	},
/*,
	{
	    .name       = "U-Boot",
	    .offset     = MTDPART_OFS_APPEND,
	    .size       = 0x100000,
		.mask_flags	= MTD_WRITEABLE
    }
*/
};

#define PARTITION_NUM (sizeof(partition_info)/sizeof(struct mtd_partition))

static struct mtd_info *mymtd;


struct map_info n2510_flash_map = {
	.name		= "N2510 Flash Map Info",
	.size		= WINDOW_SIZE,
	.phys		= WINDOW_ADDR,
	.bankwidth	= 2,
};

int __init init_n2510_flash(void)
{
	printk(KERN_NOTICE "N2510 flash device: %x at %x Partition number %d\n",
			WINDOW_SIZE, WINDOW_ADDR, PARTITION_NUM);
	n2510_flash_map.virt = ioremap(WINDOW_ADDR, WINDOW_SIZE);

	if (!n2510_flash_map.virt) {
		printk("Failed to ioremap\n");
		return -EIO;
	}
	simple_map_init(&n2510_flash_map);

	mymtd = do_map_probe("cfi_probe", &n2510_flash_map);
	if (mymtd) {
		mymtd->owner = THIS_MODULE;
                add_mtd_partitions(mymtd, partition_info, PARTITION_NUM);
		printk(KERN_NOTICE "N2510 flash device initialized\n");
		return 0;
	}

	iounmap((void *)n2510_flash_map.virt);
	return -ENXIO;
}

static void __exit cleanup_n2510_flash(void)
{
	if (mymtd) {
		del_mtd_device(mymtd);
		map_destroy(mymtd);
	}
	if (n2510_flash_map.virt) {
		iounmap((void *)n2510_flash_map.virt);
		n2510_flash_map.virt = 0;
	}
}

module_init(init_n2510_flash);
module_exit(cleanup_n2510_flash);

MODULE_AUTHOR("citizen");
MODULE_DESCRIPTION("MTD map driver for Thecus N2510 board");
MODULE_LICENSE("GPL");
