#ifndef _ZD_DEBUG2_
#define _ZD_DEBUG2_

#include <linux/string.h>
#include <stdarg.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include "zd1205.h"
#include "zdsorts.h"
#include "zdutils.h"



int zd1205_zd_dbg2_ioctl(struct zd1205_private *macp, struct zdap_ioctl *zdreq,u32 *ret);


#endif
