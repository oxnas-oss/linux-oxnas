#ifndef __ZDMMRX_C__
#define __ZDMMRX_C__

#include "zd80211.h"


void RxPsPoll(Frame_t *rdu)
{
        U16 aid;
        U8 vapId = 0;
        Hash_t *pHash;
        MacAddr_t *sta;

        //PSDEBUG("RxPsPoll");

        aid = durId(rdu) & 0x3FFF;
        sta = addr2(rdu);
        if (isGroup(sta))
                return;

        if ((aid > MAX_AID) || (aid == 0)) {
                zd_SendClass3ErrorFrame(sta, vapId);
                return;
        } else {
                pHash = sstByAid[aid];
                if (pHash->asoc == STATION_STATE_ASOC)
                        PsPolled(sta, aid);
                else
                        zd_SendClass3ErrorFrame(sta, vapId);
        }
}

BOOLEAN RxMgtMpdu(Signal_t *signal)
{
        StationState sas, sau;
        FrmDesc_t *pfrmDesc;
        Frame_t *rdu;
        U8 FrmType;
        U16 authSeq = 0;

        ZDEBUG("MgtRxMpdu");
        pfrmDesc = signal->frmInfo.frmDesc;
        rdu = pfrmDesc->mpdu;
        FrmType = frmType(rdu);
        ZDEBUG_V("FrmType", FrmType);

        switch (FrmType) {
        case ST_PROBE_REQ:
                signal->id = SIG_PROBE;
                signal->block = BLOCK_SYNCH;
                goto rx_indicate;

        case ST_DEAUTH:
                if (isGroup(addr1(rdu)))
                        goto rx_release;
                else {
                        if ((memcmp(&mBssId.mac[0],addr2(rdu),6)==0 && mBssType==INFRASTRUCTURE_BSS)) {
#ifdef WPA_DEBUG
                                printk(KERN_ERR "Rx Deauth,Reason:%u\n",reason(rdu));
#endif

                                signal->id = SIG_DEAUTH;
                                signal->block = BLOCK_AUTH_RSP;
                                goto rx_indicate;
                        } else if ((memcmp(&mBssId.mac[0],addr1(rdu),6)==0 && mBssType==AP_BSS)) {
#ifdef WPA_DEBUG
                                u8 *pCh=(u8*)addr2(rdu);
                                u8 i;
                                printk(KERN_ERR "Rx Deauth from station:");
                                for (i=0; i<6; i++) {
                                        printk("%02X ",  *pCh++);
                                }
                                printk("\n");
#endif

                                signal->id = SIG_DEAUTH;
                                signal->block = BLOCK_AUTH_RSP;
                                goto rx_indicate;
                        } else {
#ifdef WPA_DEBUG
                                u8 *pCh=(u8*)addr2(rdu);
                                u8 i;
                                printk(KERN_ERR "Rx Deauth from unknown station:");
                                for (i=0; i<6; i++) {
                                        printk("%02X ",  *pCh++);
                                }
                                printk("\n");
#endif

                                goto rx_release;
                        }

                }


        case ST_AUTH:
                if (isGroup(addr1(rdu)))
                        goto rx_release;
                else {
                        authSeq = authSeqNum(rdu);
                        if ((authSeq == 1) || ((authSeq == 3) && wepBit(rdu))) { //Seqence 1 or 3
                                signal->id = SIG_AUTH_ODD;
                                signal->block = BLOCK_AUTH_RSP;
                                goto rx_indicate;
                        } else if ((authSeq == 2) || (authSeq == 4)) { //Seqence 2 or 4
                                signal->id = SIG_AUTH_EVEN;
                                signal->block = BLOCK_AUTH_REQ;
                                goto rx_indicate;
                        } else {
                                goto rx_release;
                        }
                }

        case ST_DISASOC:
                SsInquiry(addr2(rdu), &sas, &sau);
                if ((memcmp(&mBssId.mac[0],addr2(rdu),6)==0 && mBssType==INFRASTRUCTURE_BSS)) {
#ifdef WPA_DEBUG
                        printk(KERN_ERR "Rx DeAssoc,Reason:%u\n",reason(rdu));
#endif

                }
                if(sau == STATION_STATE_NOT_AUTH) {
                        goto class2_err;
                } else {
                        signal->id = SIG_DISASSOC;
                        signal->block = BLOCK_ASOC;
                        goto rx_indicate;
                }

        case ST_ASOC_REQ:
                SsInquiry(addr2(rdu), &sas, &sau);
                if (sau == STATION_STATE_NOT_AUTH) {
                        goto class2_err;
                } else {
                        signal->id = SIG_ASSOC;
                        signal->block = BLOCK_ASOC;
                        goto rx_indicate;
                }

        case ST_REASOC_REQ:
                SsInquiry(addr2(rdu), &sas, &sau);
                if (sau == STATION_STATE_NOT_AUTH) {
                        goto class2_err;
                } else {
                        signal->id = SIG_REASSOC;
                        signal->block = BLOCK_ASOC;
                        goto rx_indicate;
                }

        case ST_BEACON:
        case ST_PROBE_RSP:
                if (mBssType == AP_BSS)
                        goto rx_release;

                signal->id = SIQ_PROBE_RSP_BEACON;
                signal->block = BLOCK_SYNCH;
                goto rx_indicate;

        case ST_ASOC_RSP:
                if (mBssType == AP_BSS)
                        goto rx_release;

                signal->id = SIG_ASSOC_RSP;
                signal->block = BLOCK_ASOC;
                goto rx_indicate;

        case ST_REASOC_RSP:
                if (mBssType == AP_BSS)
                        goto rx_release;

                signal->id = SIG_REASSOC_RSP;
                signal->block = BLOCK_ASOC;
                goto rx_indicate;

        default:
                goto rx_release;
        }

rx_release:
        ZDEBUG("goto rx_release");
        return FALSE;

rx_indicate:
        signal->frmInfo.frmDesc = pfrmDesc;
        sigEnque(pMgtQ, (signal));
        return TRUE;

class2_err:
        ZDEBUG("goto class2_err");
        signal->id =  SIG_DEAUTH_REQ;
        signal->block = BLOCK_AUTH_REQ;
        memcpy((U8 *)&signal->frmInfo.Sta, (U8 *)(addr2(rdu)), 6);
        signal->frmInfo.rCode = RC_CLASS2_ERROR;
        freeFdesc(pfrmDesc);
        sigEnque(pMgtQ, (signal));
        return TRUE;
}

#endif
