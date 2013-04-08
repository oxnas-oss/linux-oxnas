#include <linux/version.h>

#ifdef MODVERSIONS
#include <linux/modversions.h>
#endif

#include <linux/module.h>


#include <linux/pci.h>
#include <pcmcia/driver_ops.h>

#include "zd1205.h"
#include "zddebug.h"
#include "zdpci_pcmcia.h"
#include "zdversion.h"

/******************************************************************************
        Global variable definition section
******************************************************************************/
MODULE_AUTHOR("Yarco Yang");
MODULE_DESCRIPTION("ZyDAS 802.11 Wireless LAN adapter");
MODULE_LICENSE("GPL");

extern struct net_device *g_dev;

struct driver_operations zdpci_ops =
        {
                "zd1205", zdpci_attach, zdpci_suspend, zdpci_resume, zdpci_detach
        };


/******************************************************************************
    Module initialization functions
******************************************************************************/
dev_node_t *zdpci_attach(dev_locator_t * loc)
{
        u32 io;
        u16 dev_id;
        u8 bus, devfn, irq, latency_tmr;
        struct pci_dev *pci_device;
        dev_node_t *node;
        struct zd1205_private *macp;

        // perform some initial setting checks
        if (loc->bus != LOC_PCI)
                return NULL;

        bus = loc->b.pci.bus;
        devfn = loc->b.pci.devfn;

        // get some pci settings for verification
        pcibios_read_config_dword(bus, devfn, PCI_BASE_ADDRESS_0, &io);
        pcibios_read_config_byte(bus, devfn, PCI_INTERRUPT_LINE, &irq);
        pcibios_read_config_word(bus, devfn, PCI_DEVICE_ID, &dev_id);

        // check whether the latency timer is set correctly
        pcibios_read_config_byte(bus, devfn, PCI_LATENCY_TIMER, &latency_tmr);

        if (io &= ~3, io == 0 || irq == 0) {
                printk(KERN_ERR "The interface was not assigned\n");
                return NULL;
        }

        // get pci device information by loading the pci_dev structure
        if (pci_device = pci_find_slot(bus, devfn), pci_device == NULL) {
                // error reading the pci device structure
                printk(KERN_ERR "ERROR: %s could not get PCI device "
                       "information \n", DRIVER_NAME );
                return NULL;
        }

        zd1205_found1(pci_device, NULL);


        // finally setup the node structure with the device information
        node = kmalloc(sizeof(dev_node_t), GFP_KERNEL);
        strcpy(node->dev_name, macp->device->name);
        node->major = 0;
        node->minor = 0;
        node->next = NULL;
        MOD_INC_USE_COUNT;
        return node;
}

void zdpci_suspend(dev_node_t * node)
{
}

void zdpci_resume(dev_node_t * node)
{
}

void zdpci_detach(dev_node_t * node)
{
        struct zd1205_private *macp = g_dev->priv;

        unregister_netdev(g_dev);
        zd1205_remove_proc_subdir(macp);
        zd1205_clear_structs(g_dev);
        //--zd1205nics;

        // free the node
        kfree(node);
        MOD_DEC_USE_COUNT;
}


int init_module(void)
{
        printk(KERN_NOTICE "%s - version %s\n",  DRIVER_NAME, VERSIONID);
        //register_driver(&zdpci_ops);
        register_pcmcia_driver();

        return 0;
}


void cleanup_module(void)
{
        unregister_driver(&zdpci_ops);

        printk(KERN_NOTICE "Unloaded %s \n",  DRIVER_NAME);
}
