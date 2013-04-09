
#include <linux/version.h>

#ifdef MODVERSIONS
#include <linux/modversions.h>
#endif
#include <linux/module.h>

#include <linux/pci.h>

#include "zd1205.h"
#include "zddebug.h"
#include "zdpci_hotplug.h"
#include "zdversion.h"


/******************************************************************************
        Global variable definition section
******************************************************************************/
MODULE_AUTHOR("Yarco Yang");
MODULE_DESCRIPTION("ZyDAS 802.11 Wireless LAN adapter");
MODULE_LICENSE("GPL");

extern struct net_device *g_dev;

static struct pci_device_id zd1205_id_table[]  =
        {
                {
                        0x167b, 0x2102, PCI_ANY_ID, PCI_ANY_ID, 0, 0, ZD_1202
                },
                {0x167b, 0x2100, PCI_ANY_ID, PCI_ANY_ID, 0, 0, ZD_1202},
                {0x167b, 0x2105, PCI_ANY_ID, PCI_ANY_ID, 0, 0, ZD_1205},
                {0,}			/* terminate list */
        };


// register the device with the Hotplug facilities of the kernel
MODULE_DEVICE_TABLE(pci, zd1205_id_table);


struct pci_driver zd1205_driver =
        {
                .name         = "zd1205",			// Driver name
                                .id_table     = zd1205_id_table,	// id table
                                                .probe        = zd1205_found1,		// probe function
                                                                .remove       = zd1205_remove1,		// remove function
                                                                        };

/******************************************************************************
    Module initialization functions
******************************************************************************/


int init_module( void )
{
        printk(KERN_NOTICE "%s - version %s\n",  DRIVER_NAME, VERSIONID);

        if (pci_register_driver(&zd1205_driver) <= 0) {
                printk(KERN_ERR "%s: No devices found, driver not "
                       "installed.\n", DRIVER_NAME);
                pci_unregister_driver(&zd1205_driver);
                return -ENODEV;
        }

        return 0;
}


void cleanup_module( void )
{
        pci_unregister_driver(&zd1205_driver);

        printk(KERN_NOTICE "Unloaded %s \n",  DRIVER_NAME);
}
