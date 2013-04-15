/*
 * EHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 2005 John Larkworthy <john.larkworthy@oxsemi.com>
 * 
 * OXNAS Bus Glue
 *
 * Written by John Larkworthy 
 *
 * This file is licenced under the GPL.
 */
 

#undef CONFIG_PCI
#include <linux/config.h>

#ifndef CONFIG_ARCH_OXNAS
#error "This file is OXNAS bus glue. CONFIG_ARCH_OXNAS must be defined."
#endif

#include <linux/device.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/dmapool.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/usb.h>
#include <linux/moduleparam.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <asm/mach-types.h>
#include <asm/hardware.h>
#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>

/* need the ehci-hcd functions but no the PCI elements. */
#undef CONFIG_PCI 

#include "ehci-hcd.c"

extern spinlock_t oxnas_gpio_spinlock;

int usb_patch = 1;

module_param(usb_patch, int, 1);
MODULE_PARM_DESC (usb_patch, "use usb hw patch");

/* called during probe() after chip reset completes */
static int ehci_oxnas_setup(struct usb_hcd *hcd)
{
        struct ehci_hcd	*ehci = hcd_to_ehci(hcd);
	int		 temp;
        int              retval;

        ehci->caps = hcd->regs;
        ehci->regs = hcd->regs + HC_LENGTH(readl(&ehci->caps->hc_capbase));
        dbg_hcs_params(ehci, "reset");
        dbg_hcc_params(ehci, "reset");

        /* cache this readonly data; minimize chip reads */
        ehci->hcs_params = readl(&ehci->caps->hcs_params);

        retval = ehci_halt(ehci);
        if (retval)
                return retval;

        /* data structure init */
        retval = ehci_init(hcd);
        if (retval)
                return retval;

        if (ehci_is_TDI(ehci))
                ehci_reset(ehci);

        /* at least the Genesys GL880S needs fixup here */
        temp = HCS_N_CC(ehci->hcs_params) * HCS_N_PCC(ehci->hcs_params);
        temp &= 0x0f;
        if (temp && HCS_N_PORTS(ehci->hcs_params) > temp) {
                ehci_dbg(ehci, "bogus port configuration: "
                        "cc=%d x pcc=%d < ports=%d\n",
                        HCS_N_CC(ehci->hcs_params),
                        HCS_N_PCC(ehci->hcs_params),
                        HCS_N_PORTS(ehci->hcs_params));
        }

        ehci_port_power(ehci, 0);

        return retval;
}

static const struct hc_driver ehci_driver = {
	.description =		hcd_name,
	.product_desc =		"OXNAS EHCI Host Controller",
	.hcd_priv_size =	sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ehci_irq,
	.flags =		HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset =		ehci_oxnas_setup,
	.start =		ehci_run,
#ifdef	CONFIG_PM
	.suspend =		ehci_suspend,
	.resume =		ehci_resume,
#endif
	.stop =			ehci_stop,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ehci_urb_enqueue,
	.urb_dequeue =		ehci_urb_dequeue,
	.endpoint_disable =	ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ehci_hub_status_data,
	.hub_control =		ehci_hub_control,
	.bus_suspend =		ehci_bus_suspend,
	.bus_resume =		ehci_bus_resume,
};

/*
 * USB Host controller
 */

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */
/* release function for the low level device */
void usb_hcd_oxnas_release (struct device *dev)
{
	printk( DRIVER_INFO "\nehci-oxnas released\n");
}

/**
 * usb_hcd_oxnas_probe - initialize OXNAS-based HCD
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller.
 *
 */
static int usb_hcd_oxnas_probe(const struct hc_driver *driver, struct device *dev)
{
	int retval;
	unsigned long ehci_id;
	struct usb_hcd *hcd = 0;
	struct ehci_hcd *ehci;

	if (((ehci_id = readl(USB_BASE)) & 0x2f) != 0x05) {
		dbg("wrong chip ID found %lx", ehci_id);
		return -ENODEV;
	}

	hcd = usb_create_hcd(driver, dev, "usb");
	if (!hcd) {
		dbg("usb_create_hcd() failed");
		retval = -ENOMEM;
	}
	hcd->regs = (void *)(USB_BASE + 0x100); /* adjust to point at cap length register */

	printk(DRIVER_INFO "@%p Device ID register %lx\n", (void *)USB_BASE, *(unsigned long *)USB_BASE);

	/* OXNAS device has a transaction translator */
	ehci = hcd_to_ehci(hcd);
	ehci->is_tdi_rh_tt = 1;

	/* Finished initialisation and register */
	if ((retval = usb_add_hcd(hcd, USB_FS_INTERRUPT, SA_INTERRUPT))) {
		dbg("usb_add_hcd() failed");
		kfree(hcd);
		return retval;
	}
	return 0;
}
/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_oxnas_remove - shutdown processing for OXNAS-based HCD
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_oxnas_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static void usb_hcd_oxnas_remove(struct device *dev)
{
	struct usb_hcd *hcd = (struct usb_hcd*)dev_get_drvdata(dev);
	
	/* force hcd to idle */

	usb_remove_hcd(hcd);

	dev_set_drvdata(dev, NULL);

	kfree(hcd);
}

static int ehci_hcd_oxnas_drv_probe(struct device *dev)
{
 	int ret;

//      printk (DRIVER_INFO " probing  - (OX-800)\n");

	if (usb_disabled())
		return -ENODEV;

	ret = usb_hcd_oxnas_probe(&ehci_driver, dev);

	return ret;

}

static int ehci_hcd_oxnas_drv_remove(struct device *dev)
{
	usb_hcd_oxnas_remove(dev);

	return 0;
}

static struct device_driver ehci_hcd_oxnas_driver = {	
	.name	= "oxnas-ehci",
	.bus	= &platform_bus_type,
	.probe	= ehci_hcd_oxnas_drv_probe,
	.remove	= ehci_hcd_oxnas_drv_remove,
};

static u64 device_usb_dmamask = 0xffffffffUL;

static struct platform_device ehci_hcd_oxnas_device = {
	.name	= "oxnas-ehci",
	.id = -1,
	.dev = {
		.dma_mask = &device_usb_dmamask,
		.coherent_dma_mask = 0xffffffffUL,
		.release = usb_hcd_oxnas_release,
	},
};

static int __init init_oxnas_usb_ehci(void)
{
	int ret;
    unsigned long flags;
    unsigned long input_polarity = 0;
    unsigned long output_polarity = 0;
    unsigned long power_switch_mask = 0;
    unsigned long power_monitor_mask = 0;
    unsigned long power_lines_mask = 0;

	if (usb_disabled())
		return -ENODEV;

	pr_debug("%s: block sizes: qh %Zd qtd %Zd itd %Zd sitd %Zd\n",
		hcd_name,
		sizeof (struct ehci_qh), sizeof (struct ehci_qtd),
		sizeof (struct ehci_itd), sizeof (struct ehci_sitd));

#ifdef CONFIG_OXNAS_USB_PORTA_POWER_CONTROL
    power_switch_mask  |= (1UL << USBA_POWO_GPIO);
    power_monitor_mask |= (1UL << USBA_OVERI_GPIO);
#endif // CONFIG_OXNAS_USB_PORTA_POWER_CONTROL

#ifdef CONFIG_OXNAS_USB_PORTB_POWER_CONTROL
    power_switch_mask  |= (1UL << USBB_POWO_GPIO);
    power_monitor_mask |= (1UL << USBB_OVERI_GPIO);
#endif // CONFIG_OXNAS_USB_PORTB_POWER_CONTROL

#ifdef CONFIG_OXNAS_USB_PORTC_POWER_CONTROL
    power_switch_mask  |= (1UL << USBC_POWO_GPIO);
    power_monitor_mask |= (1UL << USBC_OVERI_GPIO);
#endif // CONFIG_OXNAS_USB_PORTC_POWER_CONTROL

    power_lines_mask = power_switch_mask | power_monitor_mask;

    // Configure USB power monitoring input and switch output GPIOs
#ifdef CONFIG_OXNAS_USB_OVERCURRENT_POLARITY_NEGATIVE
    input_polarity = ((1UL << SYS_CTRL_USBHSMPH_IP_POL_A_BIT) |
                      (1UL << SYS_CTRL_USBHSMPH_IP_POL_B_BIT) |
                      (1UL << SYS_CTRL_USBHSMPH_IP_POL_C_BIT));
#endif // CONFIG_OXNAS_USB_OVERCURRENT_POLARITY_NEGATIVE

#ifdef CONFIG_OXNAS_USB_POWER_SWITCH_POLARITY_NEGATIVE
    output_polarity = ((1UL << SYS_CTRL_USBHSMPH_OP_POL_A_BIT) |
                       (1UL << SYS_CTRL_USBHSMPH_OP_POL_B_BIT) |
                       (1UL << SYS_CTRL_USBHSMPH_OP_POL_C_BIT));
#endif // CONFIG_OXNAS_USB_POWER_SWITCH_POLARITY_NEGATIVE

    // Enable primary function on USB power monitor and switch lines
    spin_lock_irqsave(&oxnas_gpio_spinlock, flags);
    writel(readl(SYS_CTRL_GPIO_PRIMSEL_CTRL_0) |  power_lines_mask, SYS_CTRL_GPIO_PRIMSEL_CTRL_0);
    writel(readl(SYS_CTRL_GPIO_SECSEL_CTRL_0)  & ~power_lines_mask, SYS_CTRL_GPIO_SECSEL_CTRL_0);
    writel(readl(SYS_CTRL_GPIO_TERTSEL_CTRL_0) & ~power_lines_mask, SYS_CTRL_GPIO_TERTSEL_CTRL_0);
    spin_unlock_irqrestore(&oxnas_gpio_spinlock, flags);

    // Enable GPIO output on USB power switch output GPIOs
    writel(power_switch_mask, GPIO_A_OUTPUT_ENABLE_SET);

    // Enable GPIO input on USB power monitoring input GPIOs
    writel(power_monitor_mask, GPIO_A_OUTPUT_ENABLE_CLEAR);

    // Set the polarity of the USB power switch output and monitoring
    // inputs in system control
    if (usb_patch) {
        writel(input_polarity | output_polarity| (1<<6) , SYS_CTRL_USBHSMPH_CTRL);
    } 
    else {
            writel(input_polarity | output_polarity, SYS_CTRL_USBHSMPH_CTRL);
    }
    
    // Ensure the USB block is properly reset
    writel(1UL << SYS_CTRL_RSTEN_USBHS_BIT, SYS_CTRL_RSTEN_SET_CTRL);
    writel(1UL << SYS_CTRL_RSTEN_USBHS_BIT, SYS_CTRL_RSTEN_CLR_CTRL);
    writel(1UL << SYS_CTRL_RSTEN_USBHSPHY_BIT, SYS_CTRL_RSTEN_SET_CTRL);
    writel(1UL << SYS_CTRL_RSTEN_USBHSPHY_BIT, SYS_CTRL_RSTEN_CLR_CTRL);

    // Force the high speed clock to be generated all the time, via serial
    // programming of the USB HS PHY
    writel((2UL << SYS_CTRL_USBHSPHY_TEST_ADD) |
           (0xe0UL << SYS_CTRL_USBHSPHY_TEST_DIN), SYS_CTRL_USBHSPHY_CTRL);

    writel((1UL << SYS_CTRL_USBHSPHY_TEST_CLK) |
           (2UL << SYS_CTRL_USBHSPHY_TEST_ADD) |
           (0xe0UL << SYS_CTRL_USBHSPHY_TEST_DIN), SYS_CTRL_USBHSPHY_CTRL);

    writel((0xfUL << SYS_CTRL_USBHSPHY_TEST_ADD) | 
           (0xaaUL << SYS_CTRL_USBHSPHY_TEST_DIN), SYS_CTRL_USBHSPHY_CTRL);

    writel((1UL << SYS_CTRL_USBHSPHY_TEST_CLK) |
           (0xfUL << SYS_CTRL_USBHSPHY_TEST_ADD) |
           (0xaaUL << SYS_CTRL_USBHSPHY_TEST_DIN), SYS_CTRL_USBHSPHY_CTRL);

    // Enable the clock to the USB block
    writel(1UL << SYS_CTRL_CKEN_USBHS_BIT, SYS_CTRL_CKEN_SET_CTRL);

    // Ensure reset and clock operations are complete
    wmb();

	ret = driver_register(&ehci_hcd_oxnas_driver);

	if (!ret) {
		ret = platform_device_register(&ehci_hcd_oxnas_device);
		if (ret) {
			driver_unregister(&ehci_hcd_oxnas_driver);
		}
	}
	return ret;
}
module_init(init_oxnas_usb_ehci);

static void __exit cleanup_oxnas_usb_ehci(void)
{
	platform_device_unregister(&ehci_hcd_oxnas_device);
	
	driver_unregister(&ehci_hcd_oxnas_driver);

	// put usb core into reset
    writel(1UL << SYS_CTRL_RSTEN_USBHS_BIT, SYS_CTRL_RSTEN_SET_CTRL);

    // Disable the clock to the USB block
    writel(1UL << SYS_CTRL_CKEN_USBHS_BIT, SYS_CTRL_CKEN_CLR_CTRL);
}
module_exit(cleanup_oxnas_usb_ehci);
