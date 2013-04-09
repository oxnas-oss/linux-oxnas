#ifndef _ZD_DEBUG_C_
#define _ZD_DEBUG_C_

#include "zddebug.h"
#include "zdhw.h"
#include "zdsm.h"
#include "zdpmfilter.h"
#include "zdhci.h"
#include "zdpsmon.h"
#include "zdutils.h"
#ifdef HOST_IF_USB
    #include "zd1211.h"
#endif

extern zd_80211Obj_t dot11Obj;

//for debug message show
extern u32 freeSignalCount;
extern u32 freeFdescCount;
extern void zd_ShowQInfo(void);
extern void zd_ShowState(void);
extern BOOLEAN mPrivacyInvoked;
extern U16 mCap;
extern U8 mWpaBcKeyLen;
extern U8 mGkInstalled;
extern U8 mDynKeyMode;
extern U8 mKeyFormat;
int zd1205_cont_tx(struct zd1205_private *macp, u8 rate);
extern int zd_SetKeyContext(U8 *addr, U8 encryMode, U8 keyLength, U8 KeyId, U8 *pKeyContent);
extern void ConfigBcnFIFO(void);
extern BOOLEAN zd_CmdDeauth(MacAddr_t *sta, U8 rCode);
extern BOOLEAN zd_CmdDisasoc(MacAddr_t *sta, U8 rCode);
extern void update_beacon_interval(struct zd1205_private *macp, int val);
void zd1205_set_sniffer_mode(struct zd1205_private *macp)
{
        struct net_device *dev = macp->device;

        dev->type = ARPHRD_IEEE80211;
        dev->hard_header_len = ETH_HLEN;
        dev->addr_len = ETH_ALEN;

        if (netif_running(dev))
                netif_stop_queue(dev);

        zd_writel(0x01, SnifferOn);
        zd_writel(0xffffff, Rx_Filter);
        zd_writel(0x08, EncryptionType);
        macp->intrMask = RX_COMPLETE_EN;
}

void zd1205_dump_regs(struct zd1205_private *macp)
{
#ifndef HOST_IF_USB
        spin_lock_irqsave(&macp->q_lock, flags);
#endif

        printk(KERN_DEBUG "*******************************************************\n");
        printk(KERN_DEBUG "MACAddr_P1         = %08x  MACAddr_P2    = %08x\n",
               zd_readl(MACAddr_P1), zd_readl(MACAddr_P2));
        printk(KERN_DEBUG "BCNInterval        = %08x. BCNPLCPCfg    = %08x\n",
               zd_readl(BCNInterval), zd_readl(BCNPLCPCfg));
        printk(KERN_DEBUG "TSF_LowPart        = %08x, TSF_HighPart  = %08x\n",
               zd_readl(TSF_LowPart), zd_readl(TSF_HighPart));
        printk(KERN_DEBUG "DeviceState        = %08x, NAV_CCA       = %08x\n",
               zd_readl(DeviceState), zd_readl(NAV_CCA));
        printk(KERN_DEBUG "CRC32Cnt           = %08x, CRC16Cnt      = %08x\n",
               zd_readl(CRC32Cnt), zd_readl(CRC16Cnt));
        printk(KERN_DEBUG "TotalRxFrm         = %08x, TotalTxFrm    = %08x\n",
               zd_readl(TotalRxFrm), zd_readl(TotalTxFrm));
        printk(KERN_DEBUG "RxFIFOOverrun      = %08x, UnderrunCnt   = %08x\n",
               zd_readl(RxFIFOOverrun), zd_readl(UnderrunCnt));
        printk(KERN_DEBUG "BSSID_P1           = %08x, BSSID_P2      = %08x\n",
               zd_readl(BSSID_P1), zd_readl(BSSID_P2));
        printk(KERN_DEBUG "Pre_TBTT           = %08x, ATIMWndPeriod = %08x\n",
               zd_readl(Pre_TBTT), zd_readl(ATIMWndPeriod));
        printk(KERN_DEBUG "RetryCnt           = %08x, IFS_Value     = %08x\n",
               zd_readl(RetryCnt), zd_readl(IFS_Value));
        printk(KERN_DEBUG "NAV_CNT            = %08x, CWmin_CWmax   = %08x\n",
               zd_readl(NAV_CNT), zd_readl(CWmin_CWmax));
        //printk(KERN_DEBUG "GroupHash_P1       = %08x, GroupHash_P2  = %08x\n",
        //    zd_readl(GroupHash_P1), zd_readl(GroupHash_P2));
        printk(KERN_DEBUG "DecrypErr_UNI      = %08x, DecrypErr_Mul = %08x\n",
               zd_readl(DecrypErr_UNI), zd_readl(DecrypErr_Mul));

#ifndef HOST_IF_USB

        printk(KERN_DEBUG "InterruptCtrl      = %08x, Rx_Filter     = %08x\n",
               zd_readl(InterruptCtrl), zd_readl(Rx_Filter));
        printk(KERN_DEBUG "ReadTcbAddress     = %08x, ReadRfdAddress= %08x\n",
               zd_readl(ReadTcbAddress), zd_readl(ReadRfdAddress));
        printk(KERN_DEBUG "BCN_FIFO_Semaphore = %08x, CtlReg1       = %08x\n",
               zd_readl(BCN_FIFO_Semaphore),  zd_readl(CtlReg1));
        printk(KERN_DEBUG "RX_OFFSET_BYTE     = %08x, RX_TIME_OUT   = %08x\n",
               zd_readl(RX_OFFSET_BYTE), zd_readl(RX_TIME_OUT));
        printk(KERN_DEBUG "CAM_DEBUG          = %08x, CAM_STATUS    = %08x\n",
               zd_readl(CAM_DEBUG), zd_readl(CAM_STATUS));
        printk(KERN_DEBUG "CAM_ROLL_TB_LOW    = %08x, CAM_ROLL_TB_HIGH = %08x\n",
               zd_readl(CAM_ROLL_TB_LOW), zd_readl(CAM_ROLL_TB_HIGH));
        printk(KERN_DEBUG "CAM_MODE           = %08x\n", zd_readl(CAM_MODE));
#endif

#ifndef HOST_IF_USB

        spin_unlock_irqrestore(&macp->q_lock, flags);
#endif
}

void zd1205_dump_cnters(struct zd1205_private *macp)
{
        zd1205_lock(macp);
        printk(KERN_DEBUG "*************************************************\n");
        printk(KERN_DEBUG "freeTxQ         = %08d, activeTxQ      = %08d\n", macp->freeTxQ->count, macp->activeTxQ->count);
        printk(KERN_DEBUG "freeSignalCount = %08d, freeFdescCount = %08d\n", freeSignalCount, freeFdescCount);
        //printk(KERN_DEBUG "hwTotalRxFrm    = %08d, hwTotalTxFrm   = %08d\n", macp->hwTotalRxFrm, macp->hwTotalTxFrm);
        //printk(KERN_DEBUG "hwRxFIFOOverrun = %08d, hwUnderrunCnt  = %08d\n", macp->hwRxFIFOOverrun, macp->hwUnderrunCnt);
        //printk(KERN_DEBUG "hwCRC32Cnt      = %08d, hwCRC16Cnt     = %08d\n", macp->hwCRC32Cnt, macp->hwCRC16Cnt);

        //printk(KERN_DEBUG "ErrLongFrmCnt   = %08d, ErrShortFrmCnt = %08d\n", macp->ErrLongFrmCnt, macp->ErrShortFrmCnt);
        //printk(KERN_DEBUG "ErrToHostFrmCnt = %08d, ErrZeroLenFrmCnt= %08d\n", macp->ErrToHostFrmCnt, macp->ErrZeroLenFrmCnt);
        printk(KERN_DEBUG "rxOFDMDataFrame = %08d, rx11bDataFrame = %08d\n", macp->rxOFDMDataFrame, macp->rx11bDataFrame);
        printk(KERN_DEBUG "rxSignalQuality = %08d, rxSignalStrength= %08d\n", macp->rxSignalQuality, macp->rxSignalStrength);
        printk(KERN_DEBUG "rxRate          = %08d, txRate         = %08dx\n", macp->rxInfo.rate, macp->cardSetting.CurrTxRate);

        //printk(KERN_DEBUG "rxNeedFragCnt   = %08d, rxCompFragCnt  = %08d\n", macp->rxNeedFragCnt, macp->rxCompFragCnt);
        //printk(KERN_DEBUG "ArFreeFailCnt   = %08d, ArAgedCnt      = %08d\n", macp->ArFreeFailCnt, macp->ArAgedCnt);
        //printk(KERN_DEBUG "ArSearchFailCnt = %08d, DropFirstFragCnt= %08d\n", macp->ArSearchFailCnt, macp->DropFirstFragCnt);
        //printk(KERN_DEBUG "skb_req         = %08d, AllocSkbFailCnt= %08d\n", macp->skb_req, macp->AllocSkbFailCnt);
        printk(KERN_DEBUG "txQueToUpCnt    = %08d, txQueSetCnt    = %08d\n", macp->txQueToUpCnt, macp->txQueSetCnt);
        printk(KERN_DEBUG "sleepCnt        = %08d, wakeupCnt      = %08d\n", macp->sleepCnt, macp->wakeupCnt);
        printk(KERN_DEBUG "WaitLenInfoCnt  = %08d, CompLenInfoCnt = %08d\n", macp->WaitLenInfoCnt, macp->CompLenInfoCnt);
        printk(KERN_DEBUG "Continue2Rx     = %08d, NoMergedRxCnt  = %08d\n", macp->Continue2Rx, macp->NoMergedRxCnt);
        printk(KERN_DEBUG "bcnCnt          = %08d, dtimCnt        = %08d\n", macp->bcnCnt, macp->dtimCnt);
        printk(KERN_DEBUG "txCnt           = %08d, txCmpCnt       = %08d\n", macp->txCnt, macp->txCmpCnt);
        printk(KERN_DEBUG "retryFailCnt    = %08d, rxCnt          = %08d\n", macp->retryFailCnt, macp->rxCnt);
        printk(KERN_DEBUG "usbTxCnt        = %08d, usbTxCompCnt   = %08d\n", macp->usbTxCnt, macp->usbTxCompCnt);

        printk(KERN_DEBUG "regWaitRCompCnt = %08d, regWaitWCompCnt= %08d\n", macp->regWaitRCompCnt, macp->regWaitWCompCnt);
        printk(KERN_DEBUG "regRWCompCnt    = %08d, regUnCompCnt   = %08d\n", macp->regRWCompCnt, macp->regUnCompCnt);
        printk(KERN_DEBUG "regWaitRspCnt   = %08d, regRspCompCnt  = %08d\n", macp->regWaitRspCnt, macp->regRspCompCnt);
        printk(KERN_DEBUG "regRdSleepCnt   = %08d, regRspCompCnt  = %08d\n", macp->regRdSleepCnt, macp->regRspCompCnt);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0))
        printk(KERN_DEBUG "macp.flags = %08x\n", macp->flags);
#else
        printk(KERN_DEBUG "macp.flags = %08lx\n", macp->flags);
#endif

        zd_ShowQInfo();
        zd_ShowState();
        macp->bcnCnt = 0;
        macp->dtimCnt = 0;
        macp->rxCnt = 0;
        macp->txCmpCnt = 0;
        macp->txCnt = 0;
        macp->retryFailCnt = 0;
        macp->txIdleCnt = 0;
        macp->rxIdleCnt = 0;
        macp->hwTotalRxFrm = 0;
        macp->hwTotalTxFrm = 0;

        macp->hwRxFIFOOverrun = 0;
        macp->hwUnderrunCnt = 0;
        macp->hwCRC32Cnt =0;
        macp->hwCRC16Cnt =0;

        macp->ErrLongFrmCnt = 0;
        macp->ErrShortFrmCnt = 0;
        macp->ErrToHostFrmCnt = 0;
        macp->ErrZeroLenFrmCnt = 0;
        macp->rxOFDMDataFrame = 0;
        macp->rx11bDataFrame = 0;

        macp->ArFreeFailCnt = 0;
        macp->ArAgedCnt = 0;
        macp->ArSearchFailCnt = 0;
        macp->DropFirstFragCnt = 0;
        macp->rxNeedFragCnt = 0;
        macp->rxCompFragCnt = 0;
        macp->txQueToUpCnt = 0;
        macp->txQueSetCnt = 0;
        //macp->sleepCnt = 0;
        //macp->wakeupCnt = 0;
        macp->Continue2Rx = 0;
        macp->NoMergedRxCnt = 0;
#ifdef WPA_DEBUG

        printk("cardSet.WPAIeLen=%d\n",macp->cardSetting.WPAIeLen);
        printk("mDynKeyMode:%d,mKeyFormat:%d,mPrivacyInvoked:%d,mCap:0x%X,mWpaBcKenLen:%d\n",mDynKeyMode,mKeyFormat,mPrivacyInvoked,mCap,mWpaBcKeyLen);
#endif

        zd1205_unlock(macp);
}

void zd1205_update_brate(struct zd1205_private *macp, u32 value)
{
        u8 ii;
        u8 nRate;
        u8 *pRate;
        card_Setting_t *pSetting = &macp->cardSetting;
        u8 rate_list[4] = { 0x02, 0x04, 0x0B, 0x16 };

        /* Get the number of rates we support */
        nRate = pSetting->Info_SupportedRates[1];

        pRate = &(pSetting->Info_SupportedRates[2]);

        for(ii = 0; ii < nRate; ii++)
        {
                /* If the Rate is less than the basic Rate, mask 0x80 with the value. */
                if((*pRate & 0x7f) <= rate_list[value])
                        *pRate |= 0x80;
                else
                        *pRate &= 0x7f;

                pRate++;
        }
}

void acquire_ctrl_of_phy_req(void *regp)
{
        u32 tmpValue;

        tmpValue = zd_readl(CtlReg1);
        tmpValue &= ~0x80;
        zd_writel(tmpValue, CtlReg1);
}


void release_ctrl_of_phy_req(void *regp)
{
        u32 tmpValue;

        tmpValue = zd_readl(CtlReg1);
        tmpValue |= 0x80;
        zd_writel(tmpValue, CtlReg1);
}

void zd1205_dump_phy(struct zd1205_private *macp)
{
        void *regp = macp->regp;

        u32 regValue[4];
        int i;

        acquire_ctrl_of_phy_req(regp);
        for (i=0; i<256; i+=4)
        {
                //acquire_ctrl_of_phy_req(regp);
                if (i==4)//The offset of CR4 to CR8 are not multiplied by 4 directly.
                {
                        regValue[0]=zd_readl(ZD_CR4);
                        regValue[1]=zd_readl(ZD_CR5);
                        regValue[2]=zd_readl(ZD_CR6);
                        regValue[3]=zd_readl(ZD_CR7);
                } else if (i==8) {
                        regValue[0]=zd_readl(ZD_CR8);
                        regValue[1]=zd_readl(ZD_CR9);
                        regValue[2]=zd_readl(ZD_CR10);
                        regValue[3]=zd_readl(ZD_CR11);
                } else {
                        regValue[0] = zd_readl(4*i);
                        regValue[1] = zd_readl(4*(i+1));
                        regValue[2] = zd_readl(4*(i+2));
                        regValue[3] = zd_readl(4*(i+3));
                }

                printk(KERN_DEBUG "CR%03d = %02x  CR%03d = %02x  CR%03d = %02x  CR%03d = %02x\n",
                       i, (u8)regValue[0],  i+1, (u8)regValue[1], i+2, (u8)regValue[2], i+3, (u8)regValue[3]);
                //release_ctrl_of_phy_req(regp);
        }
        release_ctrl_of_phy_req(regp);
}
/*
void zd1205_dump_eeprom(struct zd1205_private *macp)
{
	u32 e2pValue, e2pValue1;
	int i;

	for (i=0; i<20; i+=2) {
		e2pValue = zd_readl(E2P_SUBID+4*i);
		e2pValue1 = zd_readl(E2P_SUBID+4*(i+1));
		printk(KERN_DEBUG "0x%x = %08x,     0x%x = %08x\n", E2P_SUBID+4*i, e2pValue,  E2P_SUBID+4*(i+1), e2pValue1);
	}
}
*/
void zd1205_dump_eeprom(struct zd1205_private *macp)
{
        u32 V1,V2,V3,V4 ;
        int i;

        for (i=0; i<0x30; i+=4)
        {
                V1 = zd_readl(E2P_SUBID+4*i);
                V2 = zd_readl(E2P_SUBID+4*(i+1));
                V3 = zd_readl(E2P_SUBID+4*(i+2));
                V4 = zd_readl(E2P_SUBID+4*(i+3));

                printk(KERN_DEBUG "0x%x = %08x %08x %08x %08x \n", E2P_SUBID+4*i, V1,V2,V3,V4);
        }
}

extern void zd_ShowHashInfo(u8 aid);
void zd1205_show_hash(struct zd1205_private *macp, u32 value)
{
        if (value < 33)
                zd_ShowHashInfo(value);
}

void zd1205_show_card_setting(struct zd1205_private *macp)
{
        card_Setting_t *pSetting = &macp->cardSetting;

        printk(KERN_DEBUG "RTSThreshold   = %04x   FragThreshold  = %04x\n",
               pSetting->RTSThreshold, pSetting->FragThreshold);
        printk(KERN_DEBUG "DtimPeriod     = %04x   BeaconInterval = %04x\n",
               pSetting->DtimPeriod, pSetting->BeaconInterval);
        printk(KERN_DEBUG "EncryMode      = %04x   EncryOnOff     = %04x\n",
               pSetting->EncryMode, pSetting->EncryOnOff);
        printk(KERN_DEBUG "EncryKeyId     = %04x   WepKeyLen      = %04x\n",
               pSetting->EncryKeyId, pSetting->WepKeyLen);
        printk(KERN_DEBUG "PreambleType   = %04x   AuthMode       = %04x\n",
               pSetting->PreambleType, pSetting->AuthMode);
        printk(KERN_DEBUG "Channel        = %04x   BssType        = %04x\n",
               pSetting->Channel, pSetting->BssType);
        printk(KERN_DEBUG "SuggestionMode = %04x   PwrState       = %04x\n",
               macp->SuggestionMode, macp->PwrState);
        printk(KERN_DEBUG "bPSMSupported  = %04x   bAssoc         = %04x\n",
               macp->bPSMSupported, macp->bAssoc);
        printk(KERN_DEBUG "bAnyActivity   = %04x   BSS_Members    = %04x\n",
               macp->bAnyActivity, macp->BSS_Members);
}
#if 0
void zd1205_dump_cam(struct zd1205_private *macp)
{
        int ii;

        for(ii = 0; ii < 445; ii++)
        {
                u32 data = HW_CAM_Read(&dot11Obj, ii);

                if((ii % 4) == 0)
                        printk(KERN_ERR "\nAddr=0x%04x ", ii);

                printk(KERN_ERR "0x%08x ", data);
        }

        printk(KERN_ERR "\n");
}
#endif
void zd1205_dump_cam(struct zd1205_private *macp,u32 beginAddr, u32 length)
{
        u8  valid_uid[40];
        u8  valid_uid_cnt;
        u32 data;
        int ii,jj;
        //u32 RollCallTblLow;
        //u32 RollCallTblHigh;
        u32 MACINCAM[6];
        u32 tmpRollCallTblLow;
        u32 tmpRollCallTblHigh;
        //u32 bDisplay;
        int UserIdBase;
        char *EncTypeStr[]={"","WEP64","TKIP","","AES","WEP128","WEP256",""};
        char *CamModeStr[]={"IBSS","AP","STA","WDS","Client","VAP","",""};

        for(ii = 0; ii < length; ii++)
        {
                data = HW_CAM_Read(&dot11Obj, beginAddr+ii);
                printk(KERN_ERR "\nAddr(%03u)=0x%08x", ii+beginAddr, data);
        }
        printk(KERN_ERR "\n");
        data = zd_readl(0x700); // Cam mode: MACREG(0x700)[2:0]
        printk(KERN_ERR "CAM Mode: %s\n", CamModeStr[data&7]);
        //tmpRollCallTblLow=RollCallTblLow=zd_readl(0x704);
        //tmpRollCallTblHigh=RollCallTblHigh=zd_readl(0x708)&0xf;
        tmpRollCallTblLow=zd_readl(0x704);
        tmpRollCallTblHigh=zd_readl(0x708)&0xf;
        //Scan user ID of CAM
        valid_uid_cnt=0; //Reset number of user ID
        for (ii=0; ii<40; ii++)
        {
                valid_uid[ii]=0;// Reset to invalid

                if (ii<32) {// For user 0 - 31
                        if (tmpRollCallTblLow & 1) {
                                valid_uid[ii]=1;// set to valid
                                valid_uid_cnt++;
                        }
                        tmpRollCallTblLow = tmpRollCallTblLow >> 1;
                } else {
                        if (tmpRollCallTblHigh & 1) {
                                valid_uid[ii]=1; // set to valid
                                valid_uid_cnt++;
                        }
                        tmpRollCallTblHigh = tmpRollCallTblHigh >> 1;
                }
        }
        // Dump MAC address
        UserIdBase=0;
        for(ii = 0; ii < 60; ii+=6)
        {
                //UserIdBase = UserIdBase+4;
                MACINCAM[0]=HW_CAM_Read(&dot11Obj, ii);
                MACINCAM[1]=HW_CAM_Read(&dot11Obj, ii+1);
                MACINCAM[2]=HW_CAM_Read(&dot11Obj, ii+2);
                MACINCAM[3]=HW_CAM_Read(&dot11Obj, ii+3);
                MACINCAM[4]=HW_CAM_Read(&dot11Obj, ii+4);
                MACINCAM[5]=HW_CAM_Read(&dot11Obj, ii+5);
                for (jj=0; jj<4; jj++) {
                        if (valid_uid[UserIdBase+jj]) {
                                printk(KERN_ERR "UID:%d Mac:%02x:%02x:%02x:%02x:%02x:%02x\n",UserIdBase+jj, MACINCAM[0]&255, MACINCAM[1]&255, MACINCAM[2]&255, MACINCAM[3]&255,MACINCAM[4]&255,MACINCAM[5]&255);
                        }
                        MACINCAM[0]=MACINCAM[0]>>8;
                        MACINCAM[1]=MACINCAM[1]>>8;
                        MACINCAM[2]=MACINCAM[2]>>8;
                        MACINCAM[3]=MACINCAM[3]>>8;
                        MACINCAM[4]=MACINCAM[4]>>8;
                        MACINCAM[5]=MACINCAM[5]>>8;
                }
                UserIdBase = UserIdBase+4;
        }
        // Dump Encryption type: CAM location: 60-65
        //tmpRollCallTblLow=RollCallTblLow;
        //tmpRollCallTblHigh=RollCallTblHigh;
        for(ii=60; ii<66; ii++)
        {
                data = HW_CAM_Read(&dot11Obj, ii);
                UserIdBase=(ii-60)*8; //One location for 8 users.
                if (UserIdBase >= 40) {
                        {//location 65:For default key
                                printk(KERN_ERR "DefaultKeySet:%s\n",EncTypeStr[data&7]);
                        }
                } else {
                        for(jj=0; jj<8; jj++) {
                                if (valid_uid[UserIdBase+jj]) {
                                        printk(KERN_ERR "UID:%02d:%s\n",UserIdBase+jj,EncTypeStr[data&7]);
                                        valid_uid[UserIdBase+jj] |= ((data & 7)<<1);
                                }
                                data = data >> 4; // Next user.
                        }
                }
        }
        printk(KERN_ERR "KeyContents:\n");
        for (ii=0; ii<40; ii++)
        {
                u32 keylen;
                u32 keytype;

                if (valid_uid[ii]) {
                        keytype=valid_uid[ii]>>1;
                        switch(keytype) {
                        case 2://TKIP
                                keylen=32;
                                break;
                        case 4://AES
                                keylen=16;
                                break;
                        case 1://WEP64
                                keylen=8;
                                break;
                        case 5://WEP128
                                keylen=16;
                                break;
                        default:
                                keylen=0;
                                break;
                        }
                        keylen = keylen >> 2;
                        printk(KERN_ERR "UID:%02d\n", ii);
                        for (jj=0; jj<keylen; jj++) {
                                data = HW_CAM_Read(&dot11Obj, (66+(8*ii))+jj);
                                printk(KERN_ERR "%08x\n", data);
                        }
                }
        }
        printk(KERN_ERR "\n");
}
void zd1205_cam_read(struct zd1205_private *macp, u32 addr)
{
        u32 value = HW_CAM_Read(&dot11Obj, addr);
        printk(KERN_ERR "Addr: 0x%08x, value = 0x%08x\n", addr, value);
}

void zd1205_cam_write(struct zd1205_private *macp, u32 addr, u32 value)
{
        HW_CAM_Write(&dot11Obj, addr, value);
        printk(KERN_ERR "Write value: 0x%08x to CAM address: 0x%08x\n", value, addr);
}

void zd1205_cam_rest(struct zd1205_private *macp, int mode)
{
}

int zd1205_zd_dbg_ioctl(struct zd1205_private *macp, struct zdap_ioctl *zdreq)
{
        void *regp = macp->regp;
        u16 zd_cmd;
        u32 tmp_value;
        u32 tmp_addr;
        u32 CRn;

        zd_cmd = zdreq->cmd;

        switch(zd_cmd)
        {
        case ZD_IOCTL_DEBUG_FLAG:
                macp->debugflag = zdreq->addr;
                mDebugFlag = zdreq->value;
                break;
        case ZD_IOCTL_REG_READ:
                acquire_ctrl_of_phy_req(regp);
                tmp_value = zd_readl(zdreq->addr);
                release_ctrl_of_phy_req(regp);
                zdreq->value = tmp_value;

                printk(KERN_DEBUG "zd1211 read register:  reg = 0x%04x, value = 0x%08x\n",
                       zdreq->addr, zdreq->value);
                //if (copy_to_user(ifr->ifr_data, &zdreq, sizeof (zdreq)))
                //return -EFAULT;
                break;

        case ZD_IOCTL_REG_WRITE:
                acquire_ctrl_of_phy_req(regp);
                zd_writel(zdreq->value, zdreq->addr);
                release_ctrl_of_phy_req(regp);

                if (zdreq->addr == RX_OFFSET_BYTE)
                        macp->rxOffset = zdreq->value;
                break;

        case ZD_IOCTL_MEM_DUMP:
                zd1205_dump_data("mem", (u8 *)zdreq->addr, zdreq->value);
                //memcpy(&zdreq->data[0], (u8 *)zdreq->addr, zdreq->value);
                //if (copy_to_user(ifr->ifr_data, &zdreq, sizeof (zdreq)))
                //return -EFAULT;
                break;

        case ZD_IOCTL_RATE:
                /* Check for the validation of vale */
                if(zdreq->value > 3 || zdreq->value < 0) {
                        printk(KERN_DEBUG "zd1205: Basic Rate %x doesn't support\n", zdreq->value);
                        break;
                }

                printk(KERN_DEBUG "zd1205: Basic Rate = %x\n", zdreq->value);
                zd1205_update_brate(macp, zdreq->value);
                break;

        case ZD_IOCTL_SNIFFER:
                macp->sniffer_on = zdreq->value;
                printk(KERN_DEBUG "zd1205: sniffer_on = %x\n", macp->sniffer_on);
                zd1205_set_sniffer_mode(macp);
                break;

        case ZD_IOCTL_CAM_DUMP://Arg1: Location, Arg2: Length
                {
                        u32 startAddr, length;
                        startAddr=((zdreq->addr & 0xF00)>>8)*100+
                                  ((zdreq->addr & 0xF0)>>4)*10+
                                  (zdreq->addr & 0xF);
                        length=((zdreq->value & 0xF00)>>8)*100+
                               ((zdreq->value & 0xF0)>>4)*10+
                               (zdreq->value & 0xF);
                        printk(KERN_DEBUG "zd1205: dump cam\n");
                        zd1205_dump_cam(macp,startAddr,length);
                        break;
                }
        case ZD_IOCTL_DUMP_PHY:
                printk(KERN_DEBUG "zd1205: dump phy\n");
                zd1205_dump_phy(macp);
                break;
        case ZD_IOCTL_READ_PHY:
        case ZD_IOCTL_WRITE_PHY:
                acquire_ctrl_of_phy_req(regp);
                tmp_addr = zdreq->addr;
                CRn=    ((tmp_addr & 0xF00)>>8)*100+
                        ((tmp_addr & 0xF0)>>4)*10+
                        (tmp_addr & 0xF);
                if (CRn >= 4 && CRn <= 8)//Special handling for CR4 to CR8
                {
                        u8 cnvtbl1[]={0x20, 0x10, 0x14, 0x18, 0x1c};
                        tmp_addr = cnvtbl1[CRn-4];
                } else {
                        tmp_addr = CRn*4;
                }
                if (zd_cmd == ZD_IOCTL_READ_PHY) {
                        zdreq->value = zd_readl(tmp_addr);
                        printk(KERN_DEBUG "CR%d=0x%x\n",CRn, zdreq->value);
                } else {// ZD_IOCTL_WRITE_PHY
                        zd_writel(zdreq->value, tmp_addr);
                        printk(KERN_DEBUG "set CR%d=0x%x\n",CRn, zdreq->value);
                }
                release_ctrl_of_phy_req(regp);
                break;

        case ZD_IOCTL_CARD_SETTING:
                printk(KERN_DEBUG "zd1205: card setting\n");
                zd1205_show_card_setting(macp);
                break;

        case ZD_IOCTL_HASH_DUMP:
                printk(KERN_DEBUG "zd1205: aid = %x\n", zdreq->value);
                zd1205_show_hash(macp, zdreq->value);
                break;

        case ZD_IOCTL_RFD_DUMP:
                printk(KERN_DEBUG "===== zd1205 rfd dump =====\n");
                zd1205_dump_rfds(macp);
                break;

        case ZD_IOCTL_MEM_READ: {
                        u32 *p;

                        p = (u32 *) bus_to_virt(zdreq->addr);
                        printk(KERN_DEBUG "zd1205: read memory addr: 0x%08x value: 0x%08x\n", zdreq->addr, *p);
                        break;
                }

        case ZD_IOCTL_MEM_WRITE: {
                        u32 *p;

                        p = (u32 *) bus_to_virt(zdreq->addr);
                        *p = zdreq->value;
                        printk(KERN_DEBUG "zd1205: write value: 0x%08x to memory addr: 0x%08x\n", zdreq->value, zdreq->addr);
                        break;
                }

        case ZD_IOCTL_TX_RATE:
                printk(KERN_DEBUG "zd1205: set tx rate = %d\n", zdreq->value);

                if (zdreq->value < 0x0c) {
                        macp->cardSetting.FixedRate = zdreq->value;
                        macp->bFixedRate = 1;
                } else
                        macp->bFixedRate = 0;
                break;

        case ZD_IOCTL_EEPROM:
                printk(KERN_DEBUG "zd1205: dump eeprom\n");
                zd1205_dump_eeprom(macp);
                break;

                /* Generate the beacon */
        case ZD_IOCTL_BCN:
                dot11Obj.dbg_cmd |= DBG_CMD_BEACON;
                printk(KERN_DEBUG "zd1205: configuration beacon\n");
                ConfigBcnFIFO();
                break;

        case ZD_IOCTL_REG_READ16:
                tmp_value = zd1211_readl(zdreq->addr, false);
                zdreq->value = tmp_value & 0xffff;
                printk(KERN_DEBUG "zd1205 read register:  reg = %4x, value = %4x\n",
                       zdreq->addr, zdreq->value);
                break;

        case ZD_IOCTL_REG_WRITE16:
                tmp_value = zdreq->value & 0xffff;
                zd1211_writel(zdreq->addr, tmp_value, false);
                printk(KERN_DEBUG "zd1205 write register: reg = %4x, value = %4x\n",
                       zdreq->addr, zdreq->value);
                break;

        case ZD_IOCTL_CAM_READ:
                printk(KERN_ERR "zd1205: cam read, addr: 0x%08x\n", zdreq->addr);
                zd1205_cam_read(macp, zdreq->addr);
                break;

        case ZD_IOCTL_CAM_WRITE:
                printk(KERN_ERR "zd1205: cam write, addr: 0x%08x value: 0x%08x\n", zdreq->addr, zdreq->value);
                zd1205_cam_write(macp, zdreq->addr, zdreq->value);
                break;

        case ZD_IOCTL_CAM_RESET:
                printk(KERN_ERR "zd1205: reset cam\n");
                zd1205_cam_rest(macp, zdreq->value);
                break;

        case ZD_IOCTL_CONT_TX:
                zd1205_cont_tx(macp, zdreq->value);
                break;
        case ZD_IOCTL_SET_MIC_CNT_ENABLE:
                dot11Obj.MIC_CNT = zdreq->value>0?1:0;
                printk("WPA MIC Counter Measure Feature : %s\n",
                       dot11Obj.MIC_CNT ? "Enable":"Disalbe");

                break;
        case ZD_IOCTL_GET_MIC_CNT_ENABLE:
                printk("WPA MIC Counter Measure Feature : %s\n",
                       dot11Obj.MIC_CNT ? "Enable":"Disalbe");

                break;
        default :
                printk(KERN_ERR "zd_dbg_ioctl: error command = %x\n", zd_cmd);
                break;
        }

        return 0;
}


int zd1205_cont_tx(struct zd1205_private *macp, u8 rate)
{

        void *reg = dot11Obj.reg;
        static u8 LastCont_TX_Rate=0;
        printk(KERN_ERR "ZDContinuousTx,Rate=%d\n",rate);

        //if (le32_to_cpu(pZDRD->ZDRdLength) < sizeof(ZD_RD_STRUCT))
        //{
        //   pZDRD->ZDRdLength = cpu_to_le32(sizeof(ZD_RD_STRUCT));
        //    *BytesNeeded = sizeof(ZD_RD_STRUCT);
        //    *BytesRead = 0;
        //    Status = NDIS_STATUS_BUFFER_TOO_SHORT;
        //    break;
        //}

        //macp->bContinueTxMode = le32_to_cpu(pZDRD->Buffer[0]);
        //macp->bContinueTx = le32_to_cpu(pZDRD->Buffer[1]);

        /* Use the Fixed Rate instead of LastSentTxRate */
        //macp->LastZDContinuousTxRate = macp->cardSetting.LastSentTxRate;
        //macp->LastZDContinuousTxRate = macp->cardSetting.FixedRate;

        // Roger 2004-11-10 , Set for Dr.Wang request , set 0x0001c4 when CCK mode with AL2230
        if (dot11Obj.rfMode == AL2230_RF)
        {
                //if (macp->cardSetting.LastSentTxRate > 3) {
                if (LastCont_TX_Rate > 3) {
                        HW_Set_IF_Synthesizer(&dot11Obj, 0x0005a4);
                }
                //else if (macp->cardSetting.LastSentTxRate <= 3) {
                else if (LastCont_TX_Rate <= 3) {
                        HW_Set_IF_Synthesizer(&dot11Obj, 0x0001c4);
                }
        }
        LastCont_TX_Rate = rate;

        if(rate <= RATE_54M)
        {  // Start
                u8	tmpChr = 0;
                u32	RateTmp= 0;
                u32	tmpvalue;
                u32	nLoop;
                LockPhyReg(&dot11Obj);
                dot11Obj.SetReg(reg, ZD1205_CR2, 0x3F);
                dot11Obj.SetReg(reg, ZD1205_CR138, 0x28);
                dot11Obj.SetReg(reg, ZD1205_CR33, 0x20);
                // Query CR60 until change to 0x04
                nLoop = 200;
                while(nLoop--) {
                        dot11Obj.DelayUs(10*1000); // sleep 10ms
                        tmpvalue = dot11Obj.GetReg(reg, ZD1205_CR60);

                        if(tmpvalue == 0x04)
                                break;
                }

                UnLockPhyReg(&dot11Obj);

                printk(KERN_DEBUG "Start ContTx_Normal\n");

                dot11Obj.bContinueTx = 1;
                LockPhyReg(&dot11Obj);
                /* In order to avoid the uninitial length problem,
                   force to set length to 0x20.
                 */
                dot11Obj.SetReg(reg, ZD1205_CR134, 0x20);
                UnLockPhyReg(&dot11Obj);

                switch (rate) {
                case RATE_6M: //6M
                        RateTmp = 0xB;
                        break;
                case RATE_9M: //9M
                        RateTmp = 0xF;
                        break;
                case RATE_12M: //12M
                        RateTmp = 0xA;
                        break;
                case RATE_18M: //18M
                        RateTmp = 0xE;
                        break;
                case RATE_24M: //24M
                        RateTmp = 0x9;
                        break;
                case RATE_36M: //36M
                        RateTmp = 0xD;
                        break;

                case RATE_48M:   //48M
                        RateTmp = 0x8;
                        break;

                case RATE_54M:   //54M
                        RateTmp = 0xC;
                        break;

                default:
                        RateTmp = 0;
                        break;
                }

                printk(KERN_DEBUG "RateTmp=0x%08x\n", RateTmp);

                if (RateTmp) {
                        LockPhyReg(&dot11Obj);
                        dot11Obj.SetReg(reg, ZD1205_CR132, RateTmp);

                        //AcquireCtrOfPhyReg(Adapter);
                        tmpvalue = dot11Obj.GetReg(reg, ZD1205_CR159);
                        tmpvalue &= ~(BIT_0 + BIT_1 );
                        tmpvalue |= BIT_2;
                        dot11Obj.SetReg(reg, ZD1205_CR159, tmpvalue);


                        dot11Obj.SetReg(reg, 0x644, 7);

                        tmpvalue = dot11Obj.GetReg(reg, 0x648);
                        tmpvalue &= ~BIT_0;
                        dot11Obj.SetReg(reg, 0x648, tmpvalue);
                        UnLockPhyReg(&dot11Obj);

                }

                tmpChr = LastCont_TX_Rate;
                printk(KERN_DEBUG "tmpChr=0x%x\n", tmpChr);

#if 0

                if (macp->preambleMode == 1)
                        macp->cardSetting.PreambleType = 0x00;
                else if (macp->preambleMode == 2)
                        macp->cardSetting.PreambleType = 0x20;
#endif

                if (macp->cardSetting.PreambleType == SHORT_PREAMBLE) {
                        // short premable
                        tmpChr |= BIT_5;
                } else {
                        // long premable
                        tmpChr &= ~BIT_5;
                }

                if (macp->RegionCode == 0x10)
                        tmpChr &= ~BIT_6;   //USA
                if (macp->RegionCode == 0x40)
                        tmpChr |= BIT_6;    //japan

                LockPhyReg(&dot11Obj);
                dot11Obj.SetReg(reg, ZD1205_CR5, tmpChr);
                dot11Obj.SetReg(reg, 0x644, 3);
                UnLockPhyReg(&dot11Obj);
        } else
        {
                u32	tmpvalue;

                // Roger 2004-11-10 , Set for Dr.Wang request , set 0x0001c4 when CCK mode with AL2230
                if (dot11Obj.rfMode == AL2230_RF) {
                        HW_Set_IF_Synthesizer(&dot11Obj, 0x0005a4);
                }

                LockPhyReg(&dot11Obj);
                dot11Obj.SetReg(reg, ZD1205_CR2, 0x26);
                dot11Obj.SetReg(reg, ZD1205_CR138, 0xA8);
                dot11Obj.SetReg(reg, ZD1205_CR33, 0x08);
                UnLockPhyReg(&dot11Obj);
                printk(KERN_DEBUG "Stop Normal Continuous Transmit\n");

                dot11Obj.bContinueTx = 0;
                LockPhyReg(&dot11Obj);
                macp->PHYTestTimer = 30;
                //                      ZD1205_WRITE_REGISTER(Adapter,CR122, 0x0);
                UnLockPhyReg(&dot11Obj);

                if (LastCont_TX_Rate >= 4) {
                        LockPhyReg(&dot11Obj);
                        tmpvalue = dot11Obj.GetReg(reg, ZD1205_CR159);
                        tmpvalue &= ~(BIT_0 + BIT_1 + BIT_2 );
                        dot11Obj.SetReg(reg, ZD1205_CR159, tmpvalue);

                        dot11Obj.SetReg(reg, 0x644, 0);

                        tmpvalue = dot11Obj.GetReg(reg, 0x648);
                        tmpvalue |= BIT_0;
                        dot11Obj.SetReg(reg, 0x648, tmpvalue);
                        UnLockPhyReg(&dot11Obj);

                } else {
                        LockPhyReg(&dot11Obj);
                        dot11Obj.SetReg(reg, 0x644, 0);

                        tmpvalue = dot11Obj.GetReg(reg, 0x648);
                        tmpvalue |= BIT_0;
                        dot11Obj.SetReg(reg, 0x648, tmpvalue);
                        UnLockPhyReg(&dot11Obj);
                }

                //dot11Obj.SetReg(reg, ZD_PS_Ctrl, 0x1);
                //Roger 2004-11-16 SoftwareReset here to solve RX fail after TxContinue problem
                {
                        //zd1205_device_reset(macp);
                        //Card resetting , copying from driver of ZyNOS
                        //It's a little different from ours
                        u32  tmp_value;
                        /* Update the value of Beacon Interval and Pre TBTT */
                        update_beacon_interval(macp, 0x2);
                        zd_writel(0x01, Pre_TBTT);

                        LockPhyReg(&dot11Obj);
                        dot11Obj.SetReg(dot11Obj.reg, ZD1205_PHY_END, 0x8);
                        tmp_value = zd_readl(PS_Ctrl);
                        zd_writel(tmp_value | BIT_5, PS_Ctrl);
                        dot11Obj.SetReg(dot11Obj.reg, ZD1205_PHY_END, 0x0);
                        UnLockPhyReg(&dot11Obj);

                        dot11Obj.bDeviceInSleep = 1;
                        dot11Obj.DelayUs(5000);
                }
        }

        return 0;
}

#endif
