/* src/zd1205_proc.c
*
*
*
* Copyright (C) 2004 ZyDAS Inc.  All Rights Reserved.
* --------------------------------------------------------------------
*
*
*
*   The contents of this file are subject to the Mozilla Public
*   License Version 1.1 (the "License"); you may not use this file
*   except in compliance with the License. You may obtain a copy of
*   the License at http://www.mozilla.org/MPL/
*
*   Software distributed under the License is distributed on an "AS
*   IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
*   implied. See the License for the specific language governing
*   rights and limitations under the License.
*
*   Alternatively, the contents of this file may be used under the
*   terms of the GNU Public License version 2 (the "GPL"), in which
*   case the provisions of the GPL are applicable instead of the
*   above.  If you wish to allow the use of your version of this file
*   only under the terms of the GPL and not to allow others to use
*   your version of this file under the MPL, indicate your decision
*   by deleting the provisions above and replace them with the notice
*   and other provisions required by the GPL.  If you do not delete
*   the provisions above, a recipient may use your version of this
*   file under either the MPL or the GPL.
*
* -------------------------------------------------------------------- */

#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))
#include <linux/autoconf.h>
#else
#include <linux/config.h>
#endif

#ifdef CONFIG_PROC_FS
#include "zd1205.h"

/***************************************************************************/
/*       /proc File System Interaface Support Functions                    */
/***************************************************************************/

static struct proc_dir_entry *adapters_proc_dir = 0;

/* externs from zd1205.c */
extern char zd1205_short_driver_name[];
extern char zd1205_driver_version[];
extern struct net_device_stats *zd1205_get_stats(struct net_device *dev);


static void zd1205_proc_cleanup(void);
static unsigned char zd1205_init_proc_dir(void);

#define ADAPTERS_PROC_DIR   "zd1205"
#define WRITE_BUF_MAX_LEN   20
#define READ_BUF_MAX_LEN    256
#define ZD1205_PE_LEN       25

#define bdp_drv_off(off) (unsigned long)(offsetof(struct zd1205_private, drv_stats.off))
#define bdp_prm_off(off) (unsigned long)(offsetof(struct zd1205_private, params.off))

typedef struct _zd1205_proc_entry
{
        char *name;
        read_proc_t *read_proc;
        write_proc_t *write_proc;

        unsigned long offset;	/* offset into bdp. ~0 means no value, pass NULL. */
}
zd1205_proc_entry;


static int
generic_read(char *page, char **start, off_t off, int count, int *eof, int len)
{
        if (len <= off + count)
                *eof = 1;

        *start = page + off;
        len -= off;
        if (len > count)
                len = count;

        if (len < 0)
                len = 0;

        return len;
}

#if 0
static int
read_ulong(char *page, char **start, off_t off,
           int count, int *eof, unsigned long l)
{
        int len;

        len = sprintf(page, "%lu\n", l);

        return generic_read(page, start, off, count, eof, len);
}

static int
read_gen_ulong(char *page, char **start, off_t off,

               int count, int *eof, void *data)
{
        unsigned long val = 0;

        if (data)
                val = *((unsigned long *) data);

        return read_ulong(page, start, off, count, eof, val);
}
#endif

static int
read_hwaddr(char *page, char **start, off_t off,
            int count, int *eof, unsigned char *hwaddr)
{
        int len;

        len = sprintf(page, "%02X:%02X:%02X:%02X:%02X:%02X\n",
                      hwaddr[0], hwaddr[1], hwaddr[2],
                      hwaddr[3], hwaddr[4], hwaddr[5]);

        return generic_read(page, start, off, count, eof, len);
}


static int
read_permanent_hwaddr(char *page, char **start, off_t off,
                      int count, int *eof, void *data)
{
        struct zd1205_private *macp = data;
        unsigned char *hwaddr = macp->macAdr;

        return read_hwaddr(page, start, off, count, eof, hwaddr);
}


static zd1205_proc_entry zd1205_proc_list[] = {
                        {"Permanent_HWaddr",      read_permanent_hwaddr, 0, 0},
                        {"\n",},
                        {"", 0, 0, 0}
                };


static int
read_info(char *page, char **start, off_t off, int count, int *eof, void *data)
{

        struct zd1205_private *macp = data;
        zd1205_proc_entry *pe;
        int tmp;
        void *val;
        int len = 0;

        for (pe = zd1205_proc_list; pe->name[0]; pe++) {
                if (pe->name[0] == '\n') {
                        len += sprintf(page + len, "\n");
                        continue;
                }

                if (pe->read_proc) {
                        if ((len + READ_BUF_MAX_LEN + ZD1205_PE_LEN + 1) >=
                                        PAGE_SIZE)
                                break;

                        if (pe->offset != ~0)
                                val = ((char *) macp) + pe->offset;
                        else
                                val = NULL;

                        len += sprintf(page + len, "%-"
                                       __MODULE_STRING(ZD1205_PE_LEN)
                                       "s ", pe->name);
                        len += pe->read_proc(page + len, start, 0,
                                             READ_BUF_MAX_LEN + 1, &tmp, val);
                }
        }

        return generic_read(page, start, off, count, eof, len);
}


static struct proc_dir_entry *
                        create_proc_rw(char *name, void *data, struct proc_dir_entry *parent,
                                       read_proc_t * read_proc, write_proc_t * write_proc)
{
        struct proc_dir_entry *pdep;
        mode_t mode = S_IFREG;

        if (write_proc)
        {
                mode |= S_IWUSR;
                if (read_proc) {
                        mode |= S_IRUSR;
                }

        } else if (read_proc)
        {
                mode |= S_IRUGO;
        }

        if (!(pdep = create_proc_entry(name, mode, parent)))
                return NULL;

        pdep->read_proc = read_proc;
        pdep->write_proc = write_proc;
        pdep->data = data;
        return pdep;
}


void
zd1205_remove_proc_subdir(struct zd1205_private *macp, char *name)
{
        zd1205_proc_entry *pe;
        char info[256];
        int len;

        /* If our root /proc dir was not created, there is nothing to remove */
        if (adapters_proc_dir == NULL)
        {
                return;
        }

        len = strlen(macp->ifname);
        strncpy(info, macp->ifname, sizeof (info));
        strncat(info + len, ".info", sizeof (info) - len);

        if (macp->proc_parent)
        {
                for (pe = zd1205_proc_list; pe->name[0]; pe++) {
                        if (pe->name[0] == '\n')
                                continue;

                        remove_proc_entry(pe->name, macp->proc_parent);
                }

                remove_proc_entry(macp->ifname, adapters_proc_dir);
                macp->proc_parent = NULL;
        }

        remove_proc_entry(info, adapters_proc_dir);

        /* try to remove the main /proc dir, if it's empty */
        zd1205_proc_cleanup();
}


int
zd1205_create_proc_subdir(struct zd1205_private *macp)
{
        struct proc_dir_entry *dev_dir;
        zd1205_proc_entry *pe;
        char info[256];
        int len;
        void *data;

        /* create the main /proc dir if needed */
        if (!adapters_proc_dir)
        {
                if (!zd1205_init_proc_dir())
                        return -ENOMEM;
        }

        strncpy(info, macp->ifname, sizeof (info));
        len = strlen(info);
        strncat(info + len, ".info", sizeof (info) - len);

        /* info */
        if (!(create_proc_rw(info, macp, adapters_proc_dir, read_info, 0)))
        {
                zd1205_proc_cleanup();
                return -ENOMEM;
        }

        dev_dir = create_proc_entry(macp->ifname, S_IFDIR,
                                    adapters_proc_dir);
        macp->proc_parent = dev_dir;

        if (!dev_dir)
        {
                zd1205_remove_proc_subdir(macp, macp->ifname);
                return -ENOMEM;
        }

        for (pe = zd1205_proc_list; pe->name[0]; pe++)
        {
                if (pe->name[0] == '\n')
                        continue;

                if (pe->offset != ~0)
                        data = ((char *) macp) + pe->offset;
                else
                        data = NULL;

                if (!(create_proc_rw(pe->name, data, dev_dir,
                                     pe->read_proc, pe->write_proc))) {
                        zd1205_remove_proc_subdir(macp, macp->ifname);
                        return -ENOMEM;
                }
        }

        return 0;
}

/****************************************************************************
 * Name:          zd1205_init_proc_dir
 *
 * Description:   This routine creates the top-level /proc directory for the
 *                driver in /proc/net
 *
 * Arguments:     none
 *
 * Returns:       true on success, false on fail
 *
 ***************************************************************************/
static unsigned char
zd1205_init_proc_dir(void)
{
        int len;

        /* first check if adapters_proc_dir already exists */
        len = strlen(ADAPTERS_PROC_DIR);
        for (adapters_proc_dir = init_net.proc_net->subdir;
                        adapters_proc_dir; adapters_proc_dir = adapters_proc_dir->next) {

                if ((adapters_proc_dir->namelen == len) &&
                                (!memcmp(adapters_proc_dir->name, ADAPTERS_PROC_DIR, len)))
                        break;
        }

        if (!adapters_proc_dir)
                adapters_proc_dir =
                        create_proc_entry(ADAPTERS_PROC_DIR, S_IFDIR, init_net.proc_net);

        if (!adapters_proc_dir)
                return false;

        return true;
}


/****************************************************************************
 * Name:          zd1205_proc_cleanup
 *
 * Description:   This routine clears the top-level /proc directory, if empty.
 *
 * Arguments:     none
 *
 * Returns:       none
 *
 ***************************************************************************/
static void
zd1205_proc_cleanup(void)
{
        struct proc_dir_entry *de;

        if (adapters_proc_dir == NULL) {
                return;
        }

        /* check if subdir list is empty before removing adapters_proc_dir */
        for (de = adapters_proc_dir->subdir; de; de = de->next) {
                /* ignore . and .. */
                if (*(de->name) != '.')
                        break;
        }

        if (de)
                return;

        remove_proc_entry(ADAPTERS_PROC_DIR, init_net.proc_net);
        adapters_proc_dir = NULL;
}

#endif /* CONFIG_PROC_FS */
