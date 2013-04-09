#ifndef _ZD_DEBUG_
#define _ZD_DEBUG_

#include <linux/string.h>
#include <stdarg.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include "zd1205.h"

#define ZD_DEBUG
//#define WPA_DEBUG

//#define DEBUG_DUMP_ASSOC_REQ

#ifdef ZD_DEBUG
#define ZD1211DEBUG(n, args...) do { if (macp->dbg_flag>(n)) printk(KERN_DEBUG "zd1211:" args); } while(0)
#define ZENTER(n) do { if (macp->dbg_flag>(n))	printk(KERN_DEBUG "%s: (enter) %s, %s line %i\n", "zd1205", __FUNCTION__,__FILE__,__LINE__); } while(0)
#define ZEXIT(n) do { if (macp->dbg_flag>(n))printk(KERN_DEBUG "%s: (exit) %s, %s line %i\n", "zd1205", __FUNCTION__,__FILE__,__LINE__); } while(0)
#else
#define ZD1211DEBUG(n, args...) do { } while (0)
#define ZENTER(n) //do {} while (0)
#define ZEXIT(n) //do {} while (0)
#endif

#ifdef WPA_DEBUG
#define WPADEBUG(args...) do { printk(KERN_ERR args); } while(0)
#else
#define WPADEBUG(args...) do { } while (0)
#endif

#ifdef WPADATA_DEBUG
#define WPADATADEBUG(bData, args...) do { if (bData) printk(KERN_DEBUG args); } while (0)
#else
#define WPADATADEBUG(bData, args...) do { } while (0)
#endif

extern  U32 mDebugFlag;
int zd1205_zd_dbg_ioctl(struct zd1205_private *macp, struct zdap_ioctl *zdreq);
void zd1205_set_sniffer_mode(struct zd1205_private *macp);
void zd1205_dump_regs(struct zd1205_private *macp);
void zd1205_dump_cnters(struct zd1205_private *macp);
void zd1205_dump_rfds(struct zd1205_private *macp);
#endif
