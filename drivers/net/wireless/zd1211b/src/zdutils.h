#ifndef __ZDUTILS_H__
#define __ZDUTILS_H__
#include "zdsorts.h"


#define isGroup(pMac)			(((U8*)pMac)[0] & 0x01)
#define eLen(elm)				((elm)->buf[1])
#define body(f, n) 				((f)->body[n])


#define addr1(f)				((MacAddr_t*)&((f)->header[4]))
#define addr2(f)				((MacAddr_t*)&((f)->header[10]))
#define addr3(f)				((MacAddr_t*)&((f)->header[16]))
#define addr4(f)				((MacAddr_t*)&((f)->header[24]))
#define setAddr1(f, addr)		(memcpy((char*)&((f)->header[4]),  (char*)addr, 6))
#define setAddr2(f, addr)		(memcpy((char*)&((f)->header[10]), (char*)addr, 6))
#define setAddr3(f, addr)		(memcpy((char*)&((f)->header[16]), (char*)addr, 6))
#define setAddr4(f, addr)		(memcpy((char*)&((f)->header[24]), (char*)addr, 6))
#define setFrameType(f, ft)		do {\
										f->header[0] = ft;\
										f->header[1] = 0;\
								} while (0)
#define baseType(f)				((f)->header[0] & 0x0C)
#define frmType(f)				((f)->header[0] & 0xFC)

#define wepBit(f) 				(((f)->header[1] & WEP_BIT) ? 1 : 0)
#define orderBit(f) 			(((f)->header[1] & ORDER_BIT) ? 1 : 0)
#define durId(f)				(((f)->header[2]) + ((f)->header[3]*256))
#define setAid(f, aid)			do {\
										f->header[2] = (U8)aid;\
										f->header[3] = ((U8)(aid >> 8) | 0xc0);\
								} while (0)

#define status(f)  				(body(f, 2) + (body(f, 3) * 256))
#define authType(f) 			(body(f, 0) + (body(f, 1) * 256))
#define authSeqNum(f)			(body(f, 2) + (body(f, 3) * 256))
#define authStatus(f) 			(body(f, 4) + (body(f, 5) * 256))
#define reason(f)				(body(f, 0) + (body(f, 1) * 256))
#define listenInt(f)			(body(f, 2) + (body(f, 3) * 256))
#define cap(f)					(body(f, 0) + (body(f, 1) * 256))
#define setTs(f, loTm, hiTm)  	do {\
									body(f, 0) = (U8)loTm;\
									body(f, 1) = (U8)(loTm >> 8);\
									body(f, 2) = (U8)(loTm >> 16);\
									body(f, 3) = (U8)(loTm >> 24);\
									body(f, 4) = (U8)hiTm;\
									body(f, 5) = (U8)(hiTm >> 8);\
									body(f, 6) = (U8)(hiTm >> 16);\
									body(f, 7) = (U8)(hiTm >> 24);\
								} while (0)
#define trafficMap(trafficmap, aid)  (((trafficmap)->t[(aid/8)] & (1<<(7-(aid%8))) ) == 0 ? 0 : 1)


#define beaconInt(f)			(body(f, 8) + (body(f, 9) * 256))
#define cap1(f)					(body(f, 10) + (body(f, 11) * 256))
#define aid(f)					(body(f, 4) + (body(f, 5) * 256))
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2STR(a) (a)[0],(a)[1],(a)[2],(a)[3],(a)[4],(a)[5]

#endif
