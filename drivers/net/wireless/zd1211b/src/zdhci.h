#ifndef _ZDHCI_H_
#define _ZDHCI_H_
#include "zdapi.h"

extern zd_80211Obj_t *pdot11Obj;
void zd_SendClass3ErrorFrame(MacAddr_t *sta, U8 vapId);
U8 zd_CheckTotalQueCnt(void);
void zd_EnableProtection(U8 protect);
//BOOLEAN zd_ChooseAP(void);
BOOLEAN zd_ChooseAP(BOOLEAN bUseBssid);
extern void zd1205_dump_data(char *info, u8 *data, u32 data_len);
BOOLEAN zd_CmdDeauth(MacAddr_t *sta, U8 rCode);
BOOLEAN zd_CmdDisasoc(MacAddr_t *sta, U8 rCode);
#endif
