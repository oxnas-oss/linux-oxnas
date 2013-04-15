/*
 * arch/arm/mach-oxnas/ibw.c
 *
 * Copyright (C) 2006 Oxford Semiconductor Inc.
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
 *
 */
 
 
#include <linux/config.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/sysdev.h>
#include <linux/kobject.h>
#include <asm/semaphore.h>
#include <asm/arch/cipher.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/dma.h>

/****************************** DEBUG CONTROL *********************************/
#if 0
#define DPRINTK(fmt, args...) printk(KERN_ERR "%s: " fmt, __FUNCTION__, ## args)
#define VPRINTK(fmt, args...) printk(KERN_ERR "%s: " fmt, __FUNCTION__, ## args)
#else
#define DPRINTK(fmt, args...)
#define VPRINTK(fmt, args...)
#endif


/********************************** CODE **************************************/
#define DELAY_TIME ((HZ) / 4)


/**
 * Device driver object
 */
struct ox800ibw_driver_s {
    struct semaphore sem;
    struct timer_list *timer;
    uint button_present;
    
    /** The area the password applies to, if set to ~0, the password is not 
     * valid */ 
    uint password_page;
    char password[DS1991_PASSWORD_SIZE];
    
    /** sysfs dir tree root for ibw driver */
    struct subsystem sysfsroot;
    
    /* Its name is its serial number */
    struct kobject iButton;
    
    /* each o the areas in the iButton */
    struct kobject id[DS1991_IBUTTON_FIELDS];
} ox800ibw_driver;

/**
 * The subsystem and kobjects form a file system tree in the sys file system as
 * follows the behaviour of files in the tree is described by the attribute
 * structures.
 *
 *  sys
 *  |-- iButton
 *      `-- <serial number>
 *          |-- plaintext
 *          |   `-- data 
 *          |-- 0
 *          |   |-- data 
 *          |   |-- id 
 *          |   |-- newpassword
 *          |   `-- password
 *          |-- 1
 *          |   |-- data 
 *          |   |-- id 
 *          |   |-- newpassword
 *          |   `-- password
 *          `-- 2
 *              |-- data 
 *              |-- id 
 *              |-- newpassword
 *              `-- password
 */


/* hotplug work queue function and definition */
static void ox800ibw_hotplug_work( void* );
DECLARE_WORK(ox800ibw_hotplug_wk, ox800ibw_hotplug_work, 0);

/**
 * Waits until the previous communication has finished, or the iButton is 
 * removed
 */
void ox800_busy_wait( void ) {
    u32 reg ;
    do {
        reg = readl(OX800IBW_STATUS);
        rmb();
        //VPRINTK(" %02x\n",reg);
    } while( (reg & OX800IBW_STATUS_HOLDOFF) &&
             (reg & OX800IBW_STATUS_PRESENT) );
}

/**
 * Reset internal data pointer and data buffer and reset the
 * CRC state to 0x00.
 */
void ox800ibw_rx_init(void) {
    VPRINTK("\n");
    writel( OX800IBW_CTRL_ENABLE | OX800IBW_CTRL_RX_INIT, OX800IBW_CONTROL);
}

/**
 * Reset internal data pointer only, allowing buffered data to be
 * re-used in a subsequent write
 */
void ox800ibw_tx_init(void) {
    VPRINTK("\n");
    writel( OX800IBW_CTRL_ENABLE | OX800IBW_CTRL_TX_INIT, OX800IBW_CONTROL);
}



/**
 * write a byte
 * @param byte To be written...
 */
static void ox800ibw_write_byte(char byte) {
    VPRINTK(" %02x\n",byte);
    writel( OX800IBW_CTRL_ENABLE | OX800IBW_CTRL_WR | byte , OX800IBW_CONTROL);
    
    ox800_busy_wait();
}

/**
 * Resets the iButton bus
 */
static void ox800ibw_send_reset( void ) {
    VPRINTK("\n");
    writel( OX800IBW_CTRL_ENABLE | OX800IBW_CTRL_RESET , OX800IBW_CONTROL);
    
    ox800_busy_wait();
}

/**
 * Read a byte, use ox800ibw_rx_init to switch into read mode first 
 * @return the byte read
 */
static char ox800ibw_read_byte( void ) {
    
    char byte;
    
    /* Read a byte of data from the 1-wire iButton interface */
    writel( OX800IBW_CTRL_ENABLE | OX800IBW_CTRL_RD,  OX800IBW_CONTROL);
    
    ox800_busy_wait();

    byte = readl(OX800IBW_STATUS) >> 24;
    VPRINTK(" %02x\n",byte);
    return byte;
}


/**
 * Read Serial Number from the IBW core's registers.
 * @param buffer Space to put the serial number as an ASCII string (9 bytes
 *               required)
 */
static void ox800ibw_read_serial( char* buffer ) {
    u32 lo,hi;
    VPRINTK("\n");
    lo = readl(OX800IBW_SERIAL_LO);
    hi = readl(OX800IBW_SERIAL_HI);

    sprintf(buffer, "%08x%08x",hi,lo); 
}

/**
 * Will try to read encrypted data from a DS1991 iButton. If the passward isn't
 * valid, the data will be random, but the id should be correct.
 *
 * @param area 0..2 one of the three encrypted storaege areas on the DS1991 
 * @param id Space for the ID to be written (8 bytes required)
 * @param pass Password for data access (8 bytes)
 * @param data Space for the data to be written (48 bytes required)
 * 
 */
static void ox800ibw_read_encrypted(uint area, char* id, char* pass, char* data) {
    int i, addr;
    VPRINTK("\n");
    
    ox800ibw_send_reset();

    /* address is area shifted to MS bits */
    addr = (area << 6) | 0x10 ;

    ox800ibw_write_byte(DS1991_READ_SUBKEY);
    ox800ibw_write_byte(addr);
    ox800ibw_write_byte(~addr);

    ox800ibw_rx_init();
        
    /* read id */
    for ( i = 0; i < DS1991_ID_SIZE; ++i) {
        id[i] = ox800ibw_read_byte();
    }

    ox800ibw_tx_init();
    
    /* write password */
    for ( i = 0; i < DS1991_PASSWORD_SIZE; ++i) {
        ox800ibw_write_byte(pass[i]);
    }

    ox800ibw_rx_init();

    /* read data */ 
    for (i = 0; i < DS1991_DATA_SIZE; ++i) {
        data[i] = ox800ibw_read_byte();
    }

    ox800ibw_send_reset();
}

/**
 * Will try to write data to a DS1991 iButton. If the passward isn't
 * valid, the data will not be written. This function only reads the id field
 * of the iButton, to write use ox800ibw_new_password.
 *
 * @param area 0..2 one of the three encrypted storaege areas on the DS1991 
 * @param id Space for the ID to be written (8 bytes required)
 * @param pass Password for data access (8 bytes)
 * @param data Data to be written (48 bytes required)
 * 
 */
static void ox800ibw_write_encrypted(uint area, char* id, const char* pass, const char* data) {
    int i, addr;
    
    VPRINTK("\n");
    ox800ibw_send_reset();

    /* address is area shifted to MS bits */
    addr = (area << 6) | 0x10 ;

    ox800ibw_write_byte(DS1991_WRITE_SUBKEY);
    ox800ibw_write_byte(addr);
    ox800ibw_write_byte(~addr);

    ox800ibw_rx_init();
        
    /* read id */
    for ( i = 0; i < DS1991_ID_SIZE; ++i) {
        id[i] = ox800ibw_read_byte();
    }

    ox800ibw_tx_init();
    
    /* write password, this is used to verify permission to write rather than to
    change the password */
    for ( i = 0; i < DS1991_PASSWORD_SIZE; ++i) {
        ox800ibw_write_byte(pass[i]);
    }

    ox800ibw_tx_init();

    /* write data */    
    for (i = 0; i < DS1991_DATA_SIZE; ++i) {
        ox800ibw_write_byte(data[i]);
    }

    ox800ibw_send_reset();

}

/**
 * This can be used to change the password and id. In doing so, it clears any
 * data previously stored in that area.
 *
 * @param area 0..2 one of the three encrypted storaege areas on the DS1991 
 * @param id The new id text to write, if NULL, will re-use the old id 
 * @param pass The new password (8 bytes )
 */
static void ox800ibw_new_password(uint area, const char* id, const char* pass) {
    int i, addr;
    char old_id[DS1991_ID_SIZE];
    const char* id_to_write;
    
    VPRINTK("\n");
    ox800ibw_send_reset();

    /* address is area shifted to MS bits */
    addr = area << 6;

    ox800ibw_write_byte(DS1991_WRITE_PASSWORD);
    ox800ibw_write_byte(addr);
    ox800ibw_write_byte(~addr);

    ox800ibw_rx_init();
        
    /* read id */
    for ( i = 0; i < DS1991_ID_SIZE; ++i) {
        old_id[i] = ox800ibw_read_byte();
    }

    ox800ibw_tx_init();

    /* write the id back again to satisfy verification */
    for ( i = 0; i < DS1991_ID_SIZE; ++i) {
        ox800ibw_write_byte(old_id[i]);
    }

    if (id) 
        id_to_write = id;
    else
        id_to_write = old_id;
    
    /* write the new/old id */
    for ( i = 0; i < DS1991_ID_SIZE; ++i) {
        ox800ibw_write_byte(id_to_write[i]);
    }

    /* write new password */
    for ( i = 0; i < DS1991_PASSWORD_SIZE; ++i) {
        ox800ibw_write_byte(pass[i]);
    }

    ox800ibw_send_reset();
}

/**
 * Tells the IBW that we're done and it can go back to HW control 
 */
static void ox800ibw_complete( void ) {
    u32 reg = readl(OX800IBW_CONTROL);
    VPRINTK("\n");
    writel(reg | OX800IBW_CTRL_DONE , OX800IBW_CONTROL);
}

/**
 * Describes the "data" file in the sys file system
 */
static struct attribute ox800ibw_data_attr = {
    .name = "data",
    .owner =  THIS_MODULE,
    .mode = S_IRUSR | S_IWUSR /* root can read / write */
};
 
/**
 * Called when the user reads the data file in the plain text directory
 *
 * @param kobj Not Used
 * @param attr 
 * @param buffer Space to put the file contents
 * @return The number of bytes transferred (usually 64
 */
static ssize_t ox800ibw_show_plaintext(struct kobject* kobj,
    struct attribute* attr, char* buffer) 
{
    int i, addr;
    VPRINTK("\n");
    
    ox800ibw_send_reset();

    /* address is always 3 */
    addr = 3 << 6;

    ox800ibw_write_byte(DS1991_READ_SCRATCHPAD);
    ox800ibw_write_byte(addr);
    ox800ibw_write_byte(~addr);

    ox800ibw_rx_init();

    /* read data */ 
    for (i = 0; i < DS1991_PLAINTEXT_SIZE; ++i) {
        buffer[i] = ox800ibw_read_byte();
    }

    ox800ibw_send_reset();

    return i;
}

/**
 * Called when the user writes to the data file in the plain text directory
 *
 * @param kobj Not Used
 * @param attr 
 * @param buffer Data to store
 * @param size The amount of data in buffer to store on the iButton, only
 *             64 bytes of this will actually be stared, the rest will be junked
 * @return The number of bytes transferred (usually 64
 */
static ssize_t ox800ibw_store_plaintext(struct kobject* kobj, 
    struct attribute* attr, const char* buffer, size_t size) 
{
    int i, addr;
    VPRINTK("\n");
    
    ox800ibw_send_reset();

    /* address is always 3 */
    addr = 3 << 6;

    ox800ibw_write_byte(DS1991_WRITE_SCRATCHPAD);
    ox800ibw_write_byte(addr);
    ox800ibw_write_byte(~addr);

    ox800ibw_tx_init();

    /* store data, fill rest with 0 */
    if (size > DS1991_PLAINTEXT_SIZE)
        size = DS1991_PLAINTEXT_SIZE;
    for (i = 0; i < size; ++i) {
        ox800ibw_write_byte( buffer[i] );
    }
    for (i = size; i < DS1991_PLAINTEXT_SIZE; ++i) {
        ox800ibw_write_byte( 0 );
    }

    ox800ibw_send_reset();

    return size;
}

/**
 * Sets the functions and attributes that describe the plaintext data file and
 * handle reading and writing operations on it.
 */
static struct sysfs_ops ox800ibw_plaintext_ops = {
    .show  = ox800ibw_show_plaintext,
    .store = ox800ibw_store_plaintext,
};

static struct attribute* ox800ibw_plaintext_attr[] = {
    &ox800ibw_data_attr, 
    0
};

static struct kobj_type ox800ibw_plaintext_type = {
    .release = 0,
    .sysfs_ops = &ox800ibw_plaintext_ops,
    .default_attrs = (struct attribute** )&ox800ibw_plaintext_attr
};


/**
 * Sets the functions and attributes that describe the encrypted password, 
 * newpassword, id and data files and handle reading and writing operations on
 * them.
 */
static struct attribute ox800ibw_pass_attr = {
    .name = "password",
    .owner =  THIS_MODULE,
    .mode = S_IWUSR /* root can write */
};

static struct attribute ox800ibw_newpass_attr = {
    .name = "newpassword",
    .owner =  THIS_MODULE,
    .mode = S_IWUSR /* root can write */
};

static struct attribute ox800ibw_id_attr = {
    .name = "id",
    .owner =  THIS_MODULE,
    .mode = S_IRUSR | S_IWUSR /* root can read / write */
};

/**
 * Called when the user reads a file in one of the three directories associated
 * with encrypted storage (named 0, 1 and 2). The function decide which file was
 * accessed using tho attributes and then provide the data. In some cases, the
 * password will control if the data can be returned, if not a permission error 
 * will be returned. 
 *
 * After a read of either the data or id, the password will be cleared and set
 * to non-valid 
 *
 * @param kobj Not Used
 * @param attr Used to determine which file is being accessed
 * @param buffer Space to put the file contents
 * @return The number of bytes transferred or an error
 */
static ssize_t ox800ibw_show_ciphertext(struct kobject* kobj,
    struct attribute* attr, char* buffer) 
{
    uint i;
    char dummy[64];
    uint area = ~0;

    VPRINTK("\n");
    /* find the correct page for the iButton by comparing the kobj pointer with
    known values for the kobjects of the different fields */
    for (i = 0; i < DS1991_IBUTTON_FIELDS; ++i) {
        if (kobj == &ox800ibw_driver.id[i] )
            area = i;
    }
    
    /* exit if we can't find a match */
    if (area == ~0)
        return -EIO;

    /* if trying to read the data */    
    if (attr == &ox800ibw_data_attr) {
        /* need a password for this area */
        if (ox800ibw_driver.password_page != area)
            return -EPERM;
        
        ox800ibw_read_encrypted(area, dummy, ox800ibw_driver.password, buffer);

        /* clear stored password */
        ox800ibw_driver.password_page = ~0;
        for(i = 0; i < DS1991_PASSWORD_SIZE; ++i)
            ox800ibw_driver.password[i] = 0;
    
        return DS1991_DATA_SIZE; 
    }

    /* should NOT be able to read the password */
    if ((attr == &ox800ibw_pass_attr) ||
        (attr == &ox800ibw_newpass_attr)) {
        BUG();
        return -EPERM;
    }

    /* try to read the ID */
    if (attr == &ox800ibw_id_attr) {
        /* clear stored password */
        ox800ibw_driver.password_page = ~0;
        for(i = 0; i < DS1991_PASSWORD_SIZE; ++i)
            ox800ibw_driver.password[i] = 0;
        
        ox800ibw_read_encrypted(area, buffer, dummy, dummy);
            
        return DS1991_ID_SIZE;
    }

    /* if we get here, there's been an error */
    return -EIO;
}

/**
 * Called when the user writes to a file in one of the three directories 
 * associated with encrypted storage (named 0, 1 and 2). The function decide
 * which file was accessed using tho attributes and then provide the data. In 
 * some cases, the password will control if the data can be written,
 * if not a permission error will be returned. 
 *
 * After a read of write of the data, newpassword or id, the password will be
 * cleared and set to non-valid. 
 *
 * @param kobj Not Used
 * @param attr 
 * @param buffer Data to store
 * @param size The amount of data in buffer to store on the iButton, only
 *             64 bytes of this will actually be stared, the rest will be junked
 * @return The number of bytes transferred (usually 64
 */
static ssize_t ox800ibw_store_ciphertext(struct kobject* kobj,
    struct attribute* attr, const char* buffer, size_t size)
{
    uint i;
    char dummy[DS1991_ID_SIZE];
    uint area = ~0;

    VPRINTK("\n");
    /* find the correct page for the iButton by comparing the kobj pointer with
    known values for the kobjects of the different fields */
    for (i = 0; i < DS1991_IBUTTON_FIELDS; ++i) {
        if (kobj == &ox800ibw_driver.id[i] )
            area = i;
    }
    
    /* exit if we can't find a match */
    if (area == ~0)
        return -EIO;

    if (attr == &ox800ibw_data_attr) { 
        /* need a password for this area */
        if (ox800ibw_driver.password_page != area) 
            return -EPERM;

        ox800ibw_write_encrypted(area, dummy, ox800ibw_driver.password, buffer);

        /* clear stored password */
        ox800ibw_driver.password_page = ~0;
        for(i = 0; i < DS1991_PASSWORD_SIZE; ++i)
            ox800ibw_driver.password[i] = 0;

        return size;
    }
    
    /* writing the password will "unlock" data, id, new passward in this area 
    for one operation */
    if (attr == &ox800ibw_pass_attr) {
        
        /*password must be 8 characters */
        if (size < DS1991_PASSWORD_SIZE)
            return 0;
        
        /* copy password into memory */
        ox800ibw_driver.password_page = area;
        for(i = 0; i < DS1991_PASSWORD_SIZE; ++i)
            ox800ibw_driver.password[i] = buffer[i];
        
        return size;
    }
    
    /* write in the new password */
    if (attr == &ox800ibw_newpass_attr) {
        /* clear stored password */
        ox800ibw_driver.password_page = ~0;
        for(i = 0; i < DS1991_PASSWORD_SIZE; ++i)
            ox800ibw_driver.password[i] = 0;
        
        /*new password must be 8 characters */
        if (size < DS1991_PASSWORD_SIZE) {
            ox800ibw_driver.password_page = ~0;
            for(i = 0; i < DS1991_PASSWORD_SIZE; ++i)
                ox800ibw_driver.password[i] = 0;
            return -EINVAL;
        }
        
        /* write new password with old id */
        ox800ibw_new_password(area, 0, buffer);
        
        return size;
    }
    
    /* use the new-password command to re-write the password with a new id */
    if (attr == &ox800ibw_id_attr) {
        /* need a password for this area */
        if (ox800ibw_driver.password_page != area) 
            return -EPERM;

        /* write new id with old password */
        ox800ibw_new_password(area, buffer, ox800ibw_driver.password);

        /* clear stored password */
        ox800ibw_driver.password_page = ~0;
        for(i = 0; i < DS1991_PASSWORD_SIZE; ++i)
            ox800ibw_driver.password[i] = 0;

        return size;
    }

    /* if we get here, there's been an error */
    return -EIO;
}

/**
 * Sets the functions and attributes that describe the files in the three cipher
 * text directories (named 0, 1 and 2) and the functions that handle reads and 
 * writes.
 */
static struct attribute* ox800ibw_ciphertext_attr[] = {
    &ox800ibw_data_attr,
    &ox800ibw_pass_attr,
    &ox800ibw_newpass_attr,
    &ox800ibw_id_attr,
    0
};

static struct sysfs_ops ox800ibw_ciphertext_ops = {
    .show  = ox800ibw_show_ciphertext,
    .store = ox800ibw_store_ciphertext,
};

static struct kobj_type ox800ibw_ciphertext_type = {
    .release = 0,
    .sysfs_ops = &ox800ibw_ciphertext_ops,
    .default_attrs = (struct attribute** )&ox800ibw_ciphertext_attr
};

/**
 * Builds sysfs directory structure described above. It interrogates the
 * iButton for its serial number. This is called when the iButton is connected.
 */
static void ox800ibw_build_sysfs( void ) {
    uint i;
    char buffer[65];

    VPRINTK("\n");
    /* read serial number and add a directory */    
    ox800ibw_read_serial(buffer);
    kobject_set_name(&ox800ibw_driver.iButton, "%s", buffer );
    kobject_add(&ox800ibw_driver.iButton);
    
    /* show the encrypted areas */
    for( i = 0; i < DS1991_IBUTTON_FIELDS - 1; ++i) {
        kobject_add(&ox800ibw_driver.id[i]);
    }
    
    /* show scratch area */
    kobject_add(&ox800ibw_driver.id[3]);
}

/**
 * Removes sysfs iButton directory structure. This should be called when the 
 * iButton is unplugged.
 */
static void ox800ibw_clean_sysfs( void ) {
    VPRINTK("\n");
    kobject_del(&ox800ibw_driver.iButton);
}

/**
 * A ktype structure to allow the iButton hotplugging to work
 */
static struct kobj_type ktype_ibutton = {
    .release = 0,
    .sysfs_ops = 0,
    .default_attrs = 0,
};

/**
 * hotplug filter, returns non-zero for the serial number / iButton kobject
 *
 * @param kset The set that has been hotplugged
 * @param kobj The object that the hotplugging event was reported on
 * @return non-zero if the hotplugging event shauld be reported
 */
static int ox800ibw_hotplug_filter(struct kset* kset, struct kobject* kobj) {
    struct kobj_type* ktype = get_ktype( kobj );
    if (ktype == &ktype_ibutton) {
        DPRINTK("yes\n");
        return 1;
    } else {
        DPRINTK("no\n");
        return 0;
    }
}

/**
 * Returns the name "iButton" for an iButton hotplugging event
 *
 * @param kset The set that has been hotplugged
 * @param kobj The object that the hotplugging event was reported on
 * @return the name of the hot plug event to pass to the user(s)
 */
static const char* ox800ibw_hotplug_name(struct kset* kset, struct kobject* kobj) {
    DPRINTK("\n");
    return "iButton";
}

/**
 * Sets the hot plugging functions
 */
static struct kset_uevent_ops ox800ibw_uevent_ops = {
    .filter = ox800ibw_hotplug_filter,
    .name = ox800ibw_hotplug_name,
    .uevent = NULL,
};

/**
 * Once only initialisation for iButton objects in the sys file-system. This
 * establishes their existance and behaviour ox800ibw_build_sysfs will make 
 * them visable
 */
static void ox800ibw_prep_sysfs( void )
{
    int i;
    
    VPRINTK("\n");
    
    /* prep the sysfs interface for use */
    kobject_set_name(&ox800ibw_driver.sysfsroot.kset.kobj, "iButton");
    ox800ibw_driver.sysfsroot.kset.ktype = &ktype_ibutton;
    subsystem_register(&ox800ibw_driver.sysfsroot);
    
    /* setup hotplugging */
    ox800ibw_driver.sysfsroot.kset.uevent_ops = &ox800ibw_uevent_ops;
    
    /* setup the heirarchy, the name will be set on detection */
    kobject_init(&ox800ibw_driver.iButton);
    ox800ibw_driver.iButton.kset = kset_get(&ox800ibw_driver.sysfsroot.kset);
    ox800ibw_driver.iButton.parent = &ox800ibw_driver.sysfsroot.kset.kobj;

    for(i = 0; i < DS1991_IBUTTON_FIELDS; ++i) {
        kobject_init(&ox800ibw_driver.id[i]  );
        /* setup the heirarchy */
        ox800ibw_driver.id[i].parent = &ox800ibw_driver.iButton;
        
        if (i == 3) {
            ox800ibw_driver.id[i].ktype = &ox800ibw_plaintext_type;
        } else {
            ox800ibw_driver.id[i].ktype = &ox800ibw_ciphertext_type;
        }
    }
    
    /* set the area names */
    for( i = 0; i < DS1991_IBUTTON_FIELDS - 1; ++i) {
        kobject_set_name(&ox800ibw_driver.id[i], "%d", i );
    }
    kobject_set_name(&ox800ibw_driver.id[i], "plaintext" );
}
 

/**
 * Polling for connect/disconnect. This function is called by the system timer
 * on completion it will re-schedule itself to be run again. It is called in
 * atomic context, it mustn't sleep. Hotplug framework calls tend to sleep so if
 * it needs to use them' it queues a work item on the system work queue. On
 * detecting a change on the bus it will add or remove the iButton bits of the
 * sys file system and start the process of informing the hotplug scripts.
 *
 * @param unused 
 */
static void ox800ibw_timer( unsigned long unused )
{
    u32 reg = readl(OX800IBW_STATUS);
    //printk("ox800ibw_timer %08x\n",reg);
    
    /* check for disconnection */
    if ((ox800ibw_driver.button_present) && (reg & OX800IBW_STATUS_DEPARTURE)) {
        DPRINTK("departure -> bus reset\n");
        ox800ibw_driver.button_present = 0;
        
        /* reset the bus */
        ox800ibw_send_reset();
        
        /* work item to hotlpug out and remove sysfs */
        schedule_work(&ox800ibw_hotplug_wk);
    }
    
    /* check for connection */
    if ((!ox800ibw_driver.button_present) && 
        (reg & (OX800IBW_STATUS_ARRIVAL | OX800IBW_STATUS_PRESENT)) && 
        ((readl(OX800IBW_SERIAL_LO) & 0x000000ff) == 0x02) ) {
        DPRINTK("arrival\n");
        ox800ibw_driver.button_present = 1;

        /* explore and build directory structure */
        ox800ibw_build_sysfs();
        ox800ibw_complete();

        /* work item to hotlpug in */
        schedule_work(&ox800ibw_hotplug_wk);
    }   

    /* resubmit the timer again */
    ox800ibw_driver.timer->expires = jiffies + DELAY_TIME;
    add_timer(ox800ibw_driver.timer);
    
}

/**
 * A work item for the system work queue, it performs the (sleepy) hotplug calls
 * that the timer function can't do because it's atomic. 
 */
static void ox800ibw_hotplug_work( void* not_used ) {
    if (ox800ibw_driver.button_present) {
        /* ooo look a file system */        
        kobject_uevent(&ox800ibw_driver.iButton, KOBJ_ADD);
    } else {
        /* tell things the file system sub-tree is going to dissappear */
        kobject_uevent(&ox800ibw_driver.iButton, KOBJ_REMOVE);

        /* pull down directory structure */
        ox800ibw_clean_sysfs();
    }
}


/**
 * Initialisation
 *
 * Turns on and un-resets the dpe/ibw core. It can't reset the core as the dpe
 * part of it may be in use. It sets up the GPIO controls to route the iButton
 * signals to a pin and sets up the timer to check for iButton
 * insertion / removal. 
 */
static int __init ox800ibw_init( void )
{
    u32 reg;
    
    printk("ox800ibw_init\n");
    init_MUTEX(&ox800ibw_driver.sem);

    ox800ibw_driver.timer = NULL;

    /* Enable the clock to the DPE block */
    writel(1UL << SYS_CTRL_CKEN_DPE_BIT, SYS_CTRL_CKEN_SET_CTRL);

    /* Bring out of reset (don't try to reset the core as any encryption 
    systems may be using the core) */
    writel(1UL << SYS_CTRL_RSTEN_DPE_BIT, SYS_CTRL_RSTEN_CLR_CTRL);
    
    /* route signal through GPIO maze */
    reg = readl(SYS_CTRL_GPIO_PRIMSEL_CTRL_1);
    writel(reg | (1UL << IBW_GPIO_DATA), SYS_CTRL_GPIO_PRIMSEL_CTRL_1);

    /* take ibw bus out of forced idle */
    writel(OX800IBW_CTRL_ENABLE , OX800IBW_CONTROL );

    down(&ox800ibw_driver.sem);
    
    ox800ibw_prep_sysfs();
    
    ox800ibw_driver.password_page = ~0;
    
    /* create our timer and submit it */
    if (!ox800ibw_driver.timer) {
        ox800ibw_driver.timer = kmalloc(sizeof(*ox800ibw_driver.timer), GFP_KERNEL);
        if (!ox800ibw_driver.timer) {
            printk("Couldn't setup timer\n");
            up(&ox800ibw_driver.sem);
            return -ENOMEM;
        }
    }
    ox800ibw_driver.timer->data = 0;
    ox800ibw_driver.timer->expires = jiffies + DELAY_TIME;
    ox800ibw_driver.timer->function = ox800ibw_timer;
    init_timer(ox800ibw_driver.timer);
    add_timer(ox800ibw_driver.timer);

    up(&ox800ibw_driver.sem);
    return 0;
}

/**
 * Cleanup
 */
static void __exit ox800ibw_exit( void )
{
    /* turn/cancel off timer */
    del_timer(ox800ibw_driver.timer);
    
    /* can't turn off hardware, as encrypton may be using it */
}

/** 
 * macros to register intiialisation and exit functions with kernal
 */
module_init(ox800ibw_init);
module_exit(ox800ibw_exit);
