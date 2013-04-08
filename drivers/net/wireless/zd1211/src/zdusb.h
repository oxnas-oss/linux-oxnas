#ifndef _ZDUSB_H
#define _ZDUSB_H

#ifndef DRIVER_NAME
	#ifdef ZD1211
		#define DRIVER_NAME             "zd1211"
	#elif defined(ZD1211B)
		#define DRIVER_NAME             "ZD1211B"
	#endif
#endif

/* Define these values to match your device */
#define VENDOR_ZYDAS	0x0ACE  //ZyDAS
#define PRODUCT_1211	0x1211
#define PRODUCT_A211    0xa211
#define VENDOR_ZYXEL	0x0586  //ZyXEL
#define PRODUCT_G220	0x3401
#define VENDOR_3COM     0x6891
#define PRODUCT_A727    0xA727
#define PRODUCT_G220F	0x3402

#if !defined(SET_MODULE_OWNER)
#define SET_MODULE_OWNER(dev) do { } while (0)
#endif

#endif
