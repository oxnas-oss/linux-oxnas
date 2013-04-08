#ifndef _ZD_INLINEF_H
#define _ZD_INLINEF_H
#include "zd1205.h"
#include "zdglobal.h"
#include "zddebug.h"

#define write_str(a,b)
extern zd_80211Obj_t dot11Obj;

#if defined(OFDM)

extern u8 OfdmRateTbl_11A[12];

extern u8 OfdmRateTbl[12];
#endif


__inline void
Cal_Us_Service(
        u8 TxRate,
        u16 Len,
        u16	*pLenInUs,
        u8 *pService
)
{
        u32		Remainder;
        u32		Delta;

#if defined(OFDM)

        *(pService) = 0x00;
#endif

        switch(TxRate) {
        case RATE_1M:		/* 1M bps */
                *(pLenInUs) = Len << 3;
                break;

        case RATE_2M:		/* 2M bps */
                *(pLenInUs) = Len << 2;
                break;

        case RATE_5M:		/* 5.5M bps */
                *(pLenInUs) = (u16)(((u32)Len << 4)/11);
                Remainder = (((u32)Len << 4) % 11);
                if ( Remainder ) {
                        *(pLenInUs) += 1;
                }
                break;

        case RATE_11M:		/* 11M bps */
                *(pLenInUs) = (u16)(((u32)Len << 3)/11);
                Remainder = (((u32)Len << 3) % 11);
                Delta = 11 - Remainder;
                if ( Remainder ) {
                        *(pLenInUs) += 1;
                        if ( Delta >= 8 ) {
                                *(pService) |= BIT_7;
                        }
                }
                break;

#if !(defined(GCCK) && defined(OFDM))

        case RATE_16M:		// 16.5M bps
                *(pLenInUs) = (u16)(((u32)Len << 4)/33);
                Remainder = (((u32)Len << 4) % 33);
                Delta = 33 - Remainder;
                if ( Remainder ) {
                        *(pLenInUs) += 1;
                        if ( Delta < 16 ) {}
                        else if ( (Delta >= 16) && (Delta < 32) ) {
                                *(pService) |= BIT_7;
                        } else if ( Delta >= 32 ) {
                                *(pService) |= BIT_6;
                        }
                }
                break;

        case RATE_22M:		// 22M bps
                *(pLenInUs) = (u16)(((u32)Len << 2)/11);
                Remainder = (((u32)Len << 2) % 11);
                Delta = 11 - Remainder;
                if ( Remainder ) {
                        *(pLenInUs) += 1;
                        if ( Delta < 4 ) {}
                        else if ( (Delta >= 4) && (Delta < 8) ) {
                                *(pService) |= BIT_7;
                        } else if ( Delta >= 8 ) {
                                *(pService) |= BIT_6;
                        }
                }
                break;



        case RATE_27M:		// 27.5 bps
                *(pLenInUs) = (u16)(((u32)Len << 4)/55);
                Remainder = (((u32)Len << 4) % 55);
                Delta = 55 - Remainder;
                if ( Remainder ) {
                        *(pLenInUs) += 1;
                        if ( Delta < 16 ) {}
                        else if ( (Delta >= 16) && (Delta < 32) ) {
                                *(pService) |= BIT_7;

                        }
                        else if ( (Delta >= 32) && (Delta < 48) ) {
                                *(pService) |= BIT_6;
                        } else if ( Delta >= 48 ) {
                                *(pService) |= (BIT_6 | BIT_7);
                        }
                }
                break;

        case RATE_33M:		// 33M bps
                *(pLenInUs) = (u16)(((u32)Len << 3)/33);
                Remainder = (((u32)Len << 3) % 33);
                Delta = 33 - Remainder;
                if ( Remainder ) {
                        *(pLenInUs) += 1;
                        if ( Delta < 8 ) {}
                        else if ( (Delta >= 8) && (Delta < 16) ) {
                                *(pService) |= BIT_7;
                        } else if ( (Delta >= 16) && (Delta < 24) ) {
                                *(pService) |= BIT_6;
                        } else if ( (Delta >= 24) && (Delta < 32) ) {
                                *(pService) |= (BIT_6 | BIT_7);
                        } else if ( Delta >= 32 ) {
                                *(pService) |= BIT_5;
                        }
                }
                break;

        case RATE_38M:		// 38.5M bps
                *(pLenInUs) = (u16)(((u32)Len << 4)/77);
                Remainder = (((u32)Len << 4) % 77);
                Delta = 77 - Remainder;
                if ( Remainder ) {
                        *(pLenInUs) += 1;
                        if ( Delta < 16 ) {}
                        else if ( (Delta >= 16) && (Delta < 32) ) {
                                *(pService) |= BIT_7;


                        } else if ( (Delta >= 32) && (Delta < 48) ) {
                                *(pService) |= BIT_6;
                        } else if ( (Delta >= 48) && (Delta < 64) ) {
                                *(pService) |= (BIT_6 | BIT_7);
                        } else if ( Delta >= 64) {
                                *(pService) |= BIT_5;
                        }
                }
                break;

        case RATE_44M:		// 44M bps
                *(pLenInUs) = (u16)(((u32)Len << 1)/11);
                Remainder = (((u32)Len << 1) % 11);
                Delta = 11 - Remainder;
                if ( Remainder ) {
                        *(pLenInUs) += 1;
                        if ( Delta < 2 ) {}
                        else if ( (Delta >= 2) && (Delta < 4) ) {
                                *(pService) |= BIT_7;
                        } else if ( (Delta >= 4) && (Delta < 6) ) {
                                *(pService) |= BIT_6;
                        } else if ( (Delta >= 6) && (Delta < 8) ) {
                                *(pService) |= (BIT_6 | BIT_7);
                        } else if ( (Delta >= 8) && (Delta < 10) ) {
                                *(pService) |= BIT_5;
                        } else if ( Delta >= 10 ) {
                                *(pService) |= (BIT_5 | BIT_7);
                        }
                }
                break;

        case RATE_49M:		// 49.5M bps
                *(pLenInUs) = (u16)(((u32)Len << 4)/99);
                Remainder = (((u32)Len << 4) % 99);
                Delta = 99 - Remainder;
                if ( Remainder ) {
                        *(pLenInUs) += 1;
                        if ( Delta < 16 ) {}
                        else if ( (Delta >= 16) && (Delta < 32) ) {
                                *(pService) |= BIT_7;
                        } else if ( (Delta >= 32) && (Delta < 48) ) {
                                *(pService) |= BIT_6;
                        } else if ( (Delta >= 48) && (Delta < 64) ) {
                                *(pService) |= (BIT_6 | BIT_7);
                        }

                        else if ( (Delta >= 64) && (Delta < 80) ) {
                                *(pService) |= BIT_5;
                        } else if ( (Delta >= 80) && (Delta < 96) ) {
                                *(pService) |= (BIT_5 | BIT_7);
                        } else if ( Delta >= 96 ) {
                                *(pService) |= (BIT_5 | BIT_6);
                        }
                }
                break;

        case RATE_55M:		// 55M bps
                *(pLenInUs) = (u16)(((u32)Len << 3)/55);
                Remainder = (((u32)Len << 3) % 55);
                Delta = 55 - Remainder;
                if ( Remainder ) {
                        *(pLenInUs) += 1;
                        if ( Delta < 8 ) {}
                        else if ( (Delta >= 8) && (Delta < 16) ) {
                                *(pService) |= BIT_7;
                        } else if ( (Delta >= 16) && (Delta < 24) ) {
                                *(pService) |= BIT_6;
                        } else if ( (Delta >= 24) && (Delta < 32) ) {
                                *(pService) |= (BIT_6 | BIT_7);
                        } else if ( (Delta >= 32) && (Delta < 40) ) {
                                *(pService) |= BIT_5;
                        } else if ( (Delta >= 40) && (Delta < 48) ) {
                                *(pService) |= (BIT_5 | BIT_7);
                        } else if ( Delta >= 48 ) {
                                *(pService) |= (BIT_5 | BIT_6);
                        }
                }
                break;

        case RATE_60M:		// 60.5M bps
                *(pLenInUs) = (u16)(((u32)Len << 4)/121);
                Remainder = (((u32)Len << 4) % 121);
                Delta = 121 - Remainder;
                if ( Remainder ) {
                        *(pLenInUs) += 1;
                        if ( Delta < 16 ) {}
                        else if ( (Delta >= 16) && (Delta < 32) ) {
                                *(pService) |= BIT_7;
                        } else if ( (Delta >= 32) && (Delta < 48) ) {
                                *(pService) |= BIT_6;
                        } else if ( (Delta >= 48) && (Delta < 64) ) {
                                *(pService) |= (BIT_6 | BIT_7);
                        } else if ( (Delta >= 64) && (Delta < 80) ) {
                                *(pService) |= BIT_5;
                        } else if ( (Delta >= 80) && (Delta < 96) ) {
                                *(pService) |= (BIT_5 | BIT_7);
                        } else if ( (Delta >= 96) && (Delta < 112) ) {
                                *(pService) |= (BIT_5 | BIT_6);
                        } else if ( Delta >= 112 ) {
                                *(pService) |= (BIT_5 | BIT_6 | BIT_7);
                        }
                }
                break;

        case 13:		// 8.25M bps
                *(pLenInUs) = (u16)(((u32)Len * 32)/33);
                Remainder = (((u32)Len * 32) % 33);
                Delta = 33 - Remainder;
                if ( Remainder ) {
                        *(pLenInUs) += 1;
                        if (Delta < 32) {}
                        else {
                                *(pService) |= BIT_7;
                        }
                }

                break;
#else

        case RATE_6M:	// 6M
                *(pLenInUs) = (u16)(((u32)Len << 3)/6);
                break;

        case RATE_9M:	// 9M
                *(pLenInUs) = (u16)(((u32)Len << 3)/9);
                break;

        case RATE_12M:	// 12M
                *(pLenInUs) = (u16)(((u32)Len << 3)/12);
                break;

        case RATE_18M:	// 18M
                *(pLenInUs) = (u16)(((u32)Len << 3)/18);
                break;

        case RATE_24M:	// 24M
                *(pLenInUs) = (u16)(((u32)Len << 3)/24);
                break;

        case RATE_36M:	// 36M
                *(pLenInUs) = (u16)(((u32)Len << 3)/36);

                break;

        case RATE_48M:	// 48M
                *(pLenInUs) = (u16)(((u32)Len << 3)/48);
                break;

        case RATE_54M:	// 54M
                *(pLenInUs) = (u16)(((u32)Len << 3)/54);
                break;
#endif

        default:
                printk(KERN_ERR "zd1205: Invalid RF module parameter\n");

        }
}

char DbgStr100[]="TKIP:";
char DbgStr101[]="CtrlSetting";
char DbgStr102[]="EncryType";
#define CTRL_SIZE	25
__inline unsigned long
Cfg_CtrlSetting(
        struct zd1205_private *macp,
        zd1205_SwTcb_t		*pSwTcb,
        wla_Header_t	*pWlaHdr,
        ctrl_Set_parm_t *pSetParms
)
{
        zd1205_Ctrl_Set_t	*pCtrlSet = pSwTcb->pHwCtrlPtr;
        u8			tmp;
        u16			Len = 0;
        u16			NextLen = 0;
        u16			LenInUs = 0;
        u16			NextLenInUs = 0;
        u8			Service;
        u8			TxRate;
        u8			Rate = pSetParms->Rate;
        u8			Preamble = pSetParms->Preamble;
        u32			CurrFragLen = pSetParms->CurrFragLen;
        u32			NextFragLen = pSetParms->NextFragLen;
        u8			encryType = pSetParms->encryType;
        //u8			vapId = pSetParms->vapId;
        u8			bMgtFrame = 0;
        u8			bGroupAddr = 0;
        u8			EnCipher = ((pWlaHdr->FrameCtrl[1] & ENCRY_BIT) ? 1 : 0);
        u16			FragNum = (pWlaHdr->SeqCtrl[0] & 0x0F);
        card_Setting_t		*pCardSettting = &macp->cardSetting;
        u16			FrameType = pWlaHdr->FrameCtrl[0];
        u8			bBusrt = 0;
#ifdef ZD1211B

        u8			LengthDiff=0;
#endif

        memset(pCtrlSet,0,sizeof(zd1205_Ctrl_Set_t));

        if (Rate > macp->AdapterMaxRate)
                Rate = macp->AdapterMaxRate;

        if ((FrameType & 0x0c) == MANAGEMENT)
        {
                bMgtFrame = 1;
                Rate = dot11Obj.BasicRate;
        }
        if ((FrameType & 0x0c) == DATA)
        {
                write_str(DbgStr101, EnCipher);
        }
        if (bMgtFrame)
        {
                if ((FrameType == PROBE_RSP) || (FrameType == PROBE_REQ)) {
                        // Ensure Site-Survey smooth
                        if(PURE_A_MODE != pCardSettting->MacMode) {
                                Rate = RATE_1M;
                                Preamble = 0;
                        } else if(PURE_A_MODE == pCardSettting->MacMode) {
                                Rate = RATE_6M;
                                Preamble =1;
                        }

                }
        }

        if (FrameType == PS_POLL)
        {
                // For compatibility with D-Link AP
                if(PURE_A_MODE != pCardSettting->MacMode) {
                        Rate = RATE_1M;
                        Preamble = 0;
                } else if(PURE_A_MODE == pCardSettting->MacMode) {
                        Rate = RATE_6M;
                        Preamble = 1;
                }

        }

        if ((Rate == RATE_1M) && (Preamble == 1))
        { //1M && short preamble
                Rate = RATE_2M;
        }

        if (macp->bFixedRate)
        {
                if (!bMgtFrame)
                        Rate = pCardSettting->FixedRate;
        }

        pSwTcb->Rate = Rate;
        //FPRINT_V("zdinline Rate", Rate);

#if !defined(OFDM)

        pCtrlSet->CtrlSetting[0] = (Rate | (Preamble << 5));
#else

        if (Rate < RATE_6M)
        { //CCK frame
                pCtrlSet->CtrlSetting[0] = (Rate | (Preamble << 5));
                //``JWEI 2003/12/22
#if fTX_PWR_CTRL

                if ((macp->TxOFDMCnt > 0) && (macp->TxOFDMCnt < cTX_SENT_LEN))
                        macp->TxOFDMCnt --;
                macp->TxPwrCCK ++;
#endif

        } else
        {
#if fTX_PWR_CTRL
                macp->TxPwrOFDM ++;
                macp->TxOFDMCnt ++;
                if (Rate == RATE_48M) {
                        if (macp->TxOFDMType != cTX_48M)
                                macp->TxOFDMCnt = 0;
                        macp->TxOFDMType = cTX_48M;
                } else if (Rate == RATE_54M) {
                        if (macp->TxOFDMType != cTX_54M)
                                macp->TxOFDMCnt = 0;
                        macp->TxOFDMType = cTX_54M;
                } else {
                        if (macp->TxOFDMType != cTX_OFDM)
                                macp->TxOFDMCnt = 0;
                        macp->TxOFDMType = cTX_OFDM;
                }
#endif

                if(PURE_A_MODE != pCardSettting->MacMode)
                        pCtrlSet->CtrlSetting[0] = OfdmRateTbl[Rate];
                else if(PURE_A_MODE == pCardSettting->MacMode) {
                        pCtrlSet->CtrlSetting[0] = OfdmRateTbl_11A[Rate];
                }

        }
#endif

        TxRate = Rate;

        //keep current Tx rate
        pCardSettting->CurrTxRate = TxRate;

        /* Length in byte */
        if (EnCipher)
        {
                if (!pCardSettting->SwCipher) {
                        write_str(DbgStr102, encryType);
                        switch(encryType) {
                        case WEP64:
                        case WEP128:
                        case WEP256:
                                Len = CurrFragLen + 36; 	/* Header(24) + CRC32(4) + IV(4) + ICV(4) */
                                NextLen = NextFragLen + 36;
#ifdef ZD1211B

                                LengthDiff=17;
#endif

                                break;

                        case TKIP:
                                write_str(DbgStr100, CurrFragLen);
                                Len = CurrFragLen + 40; 	/* Header(24) + CRC32(4) + IV(4) + EIV(4) + ICV(4) */
                                NextLen = NextFragLen + 40;
#ifdef ZD1211B

                                LengthDiff=17;
#endif

                                break;

                        case AES:
                                Len = CurrFragLen + 44; 	/* Header(24) + CRC32(4) + IV(4) +  ExtendedIV(4) + MIC(8) */
                                NextLen = NextFragLen + 44;
#ifdef ZD1211B

                                LengthDiff=13;
#endif
                                //FPRINT_V("Len", Len);
                                break;

                        default:
                                printk(KERN_DEBUG "error encryType = %x\n", encryType);
                                break;
                        }
                } else { //use software encryption
                        if (pCardSettting->DynKeyMode == DYN_KEY_TKIP) {
                                if ((pWlaHdr->DA[0] & BIT_0) && (pCardSettting->WpaBcKeyLen != 32)) { //multicast
                                        Len = CurrFragLen + 32; // Header(24) + CRC32(4) + IV(4), ICV was packed under payload

                                        NextLen = NextFragLen + 32;
                                } else {
                                        Len = CurrFragLen + 36; // Header(24) + CRC32(4) + IV(4) + ExtendIV(4), ICV was packed under payload
                                        NextLen = NextFragLen + 36;
                                }
                        } else {
                                Len = CurrFragLen + 32; // Header(24) + CRC32(4) + IV(4), ICV was packed under payload
                                NextLen = NextFragLen + 32;
                        }
                }
        } else
        { // no cipher
                Len = CurrFragLen + 28; 	/* Header(24) + CRC32(4) */
                NextLen = NextFragLen + 28;
#ifdef ZD1211B

                LengthDiff=21;
#endif

        }

        /* Corret some exceptions */
        if (FrameType == PS_POLL)
        {
                Len = CurrFragLen + 20; // Header(16) + CRC32(4)
        }

        /* Corret some exceptions */
        if (NextFragLen == 0)
                NextLen = 0;

        pCtrlSet->CtrlSetting[1] = (u8)Len; 			/* low byte */
        pCtrlSet->CtrlSetting[2] = (u8)(Len >> 8);   /* high byte */

        /* TCB physical address */
        pCtrlSet->CtrlSetting[3] = (u8)(pSwTcb->TcbPhys);
        pCtrlSet->CtrlSetting[4] = (u8)(pSwTcb->TcbPhys >> 8);
        pCtrlSet->CtrlSetting[5] = (u8)(pSwTcb->TcbPhys >> 16);
        pCtrlSet->CtrlSetting[6] = (u8)(pSwTcb->TcbPhys >> 24);

        pCtrlSet->CtrlSetting[7] = 0x00;
        pCtrlSet->CtrlSetting[8] = 0x00;
        pCtrlSet->CtrlSetting[9] = 0x00;
        pCtrlSet->CtrlSetting[10] = 0x00;

        /* Misc */
        tmp = 0;
        if (!FragNum)
        {
                tmp |= BIT_0;

                if (macp->bTxBurstEnable) {
                        if (macp->activeTxQ->count > 0) {
                                // AT LEAST one packet in ActiveChainList
                                macp->TxBurstCount++;
                                if (macp->TxBurstCount == 3)	// only 3 packets
                                        macp->TxBurstCount = 0;
                        } else {
                                // recount again, next packet will back off
                                macp->TxBurstCount = 0;
                        }

                        // burst mode
                        if (macp->TxBurstCount == 0) {
                                tmp |= BIT_0;	//need back off
                        } else {
                                tmp &= ~BIT_0;	//burst, no need back off
                                bBusrt = 1;

                                pCtrlSet->CtrlSetting[0] |= BIT_7;
                        }
                }
        }

        if (pWlaHdr->DA[0] & BIT_0)
        {		/* Multicast */
                bGroupAddr = 1;
                tmp |= BIT_1;
        }

        //if (bMgtFrame){
        //	tmp |= BIT_3;
        //}

        if (FrameType == PS_POLL) //AP don't send PS_POLL
                tmp |= BIT_2;


#ifndef HOST_IF_USB

        if ((pCardSettting->BssType == INDEPENDENT_BSS) && (!bMgtFrame))
        {
                if (zd1205_DestPowerSave(macp, &pWlaHdr->DA[0])) {
                        tmp |= BIT_4;
                }
        }
#endif

        if (Len > pCardSettting->RTSThreshold)
        {
                if ((!bMgtFrame) && (!bGroupAddr)) {
                        if(PURE_A_MODE != macp->cardSetting.MacMode)
                                tmp |= BIT_5;
                        else if(PURE_A_MODE == macp->cardSetting.MacMode)
                                tmp |= BIT_7;
                }
        }

#if (defined(GCCK) && defined(OFDM))

        if (TxRate > RATE_11M && PURE_A_MODE != macp->cardSetting.MacMode)
        {

                if (tmp & BIT_5) {
                        // need to send RTS or CTS, in OFDM only send CTS, in CCK send RTS
                        tmp &= ~BIT_5;
                        tmp |= BIT_7;
                }
                //else if ((dot11Obj.ConfigFlag & ENABLE_PROTECTION_SET) && dot11Obj.bDisProtection==0)
                else if ((dot11Obj.ConfigFlag & ENABLE_PROTECTION_SET)) {
                        // id SelfCTS on, force send CTS when OFDM
                        tmp |= BIT_7;
                }
        }
#endif

        if ((EnCipher) && (!pCardSettting->SwCipher))
        {
                tmp |= BIT_6;
        }


        if ((macp->bTxBurstEnable))
        {
                if (!bBusrt)
                        // bBusrt off, this is the first one, force send CTS
                        tmp |= BIT_7;
                else
                        // bBusrt on, this is the burst one, no need CTS
                        tmp &= ~BIT_7;
        }

        pCtrlSet->CtrlSetting[11] = tmp;

        /* Address1 */
        pCtrlSet->CtrlSetting[12] = pWlaHdr->DA[0];
        pCtrlSet->CtrlSetting[13] = pWlaHdr->DA[1];
        pCtrlSet->CtrlSetting[14] = pWlaHdr->DA[2];
        pCtrlSet->CtrlSetting[15] = pWlaHdr->DA[3];
        pCtrlSet->CtrlSetting[16] = pWlaHdr->DA[4];
        pCtrlSet->CtrlSetting[17] = pWlaHdr->DA[5];

        if (FrameType == DATA)
        {
                macp->TotalTxDataFrmBytes += Len;
                if (pCtrlSet->CtrlSetting[12] & BIT_0) {
                        macp->txMulticastFrm ++;
                        macp->txMulticastOctets += CurrFragLen;
                } else {
                        macp->txUnicastFrm ++;
                        macp->txUnicastOctets += CurrFragLen;
                        macp->txDataPerSec += CurrFragLen;
                }

        }

        /* NextLen */
#ifdef ZD1211
        if (NextLen)
        {
                pCtrlSet->CtrlSetting[18] = (u8)NextLen;
                pCtrlSet->CtrlSetting[19] = (u8)(NextLen >> 8);
        }
#elif defined(ZD1211B)
        pSwTcb->LengthDiff = LengthDiff;
#endif

        /* LenInUs */
        Cal_Us_Service(TxRate, Len, &LenInUs, &Service);

        if (macp->bTxBurstEnable)
                if (!bBusrt)
                        LenInUs = LenInUs * 4;

        pCtrlSet->CtrlSetting[20] = (u8)LenInUs;
        pCtrlSet->CtrlSetting[21] = (u8)(LenInUs >> 8);

        /* Service */
#if !(defined(GCCK) && defined(OFDM))

        pCtrlSet->CtrlSetting[22] = Service;
#else

        if (Rate < RATE_6M)
        {
                pCtrlSet->CtrlSetting[22] = Service;
        } else
        {
                pCtrlSet->CtrlSetting[22] = 0;
        }
#endif

        if (NextLen == 0)
        {
                NextLenInUs = 0;
        } else
        {
                Cal_Us_Service(TxRate, NextLen, &NextLenInUs, &Service);
                pCtrlSet->CtrlSetting[23] = (u8)NextLenInUs;
                pCtrlSet->CtrlSetting[24] = (u8)(NextLenInUs >> 8);

#ifdef OFDM
                // NextLen
                // backup NextLen, because OFDM use these 2 bytes as total length,
                // so we backup here for Retry fail use.
                pCtrlSet->CtrlSetting[26] = (u8)NextLen;
                pCtrlSet->CtrlSetting[27] = (u8)(NextLen >> 8);
#endif

        }

        return(CTRL_SIZE);
}


#define LOW8(v16)	((u8)( (v16) & 0xFF))
#define HIGH8(v16)  ((u8)(((v16)>>8) & 0xFF))
__inline u8
ABS(u8 N1, u8 N2)
{
        s16 Result;

        Result = N1-N2;
        if(Result < 0 )
                Result *= -1;
        return (u8)Result;

}

__inline u32
Cfg_MacHeader(
        struct zd1205_private	*macp,
        zd1205_SwTcb_t	 		*pSwTcb,
        wla_Header_t 			*pWlaHdr,
        u8						hdrLen
)
{
        zd1205_Header_t	*pHdr;
        u8	HdrLen;

        pHdr = pSwTcb->pHwHeaderPtr;

        HdrLen = hdrLen;
        memcpy(&pHdr->MacHeader[0], (u8 *)pWlaHdr, hdrLen);

        return(HdrLen);
}

#endif
