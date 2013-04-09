#ifndef __MENU_DRV_MACRO_H
#define __MENU_DRV_MACRO_H

#define	RDPhy		1
#define WRPhy		2
#define	RDMAC		3
#define WRMAC		4
#define RDCNT		5
#define BUF_LEN		2048

#define	T_bcnCnt		0
#define T_dtimCnt		1
#define T_txCmpCnt		2
#define T_rxCnt			3
#define T_retryFailCnt	4
#define T_txCnt			5
#define T_txIdleCnt		6
#define T_rxIdleCnt		7
#define T_rxDupCnt		8

#define ZD_MENU_DBG		ZDAPIOCTL+4
#endif
