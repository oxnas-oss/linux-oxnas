#ifndef __ZDSHARED_C__
#define __ZDSHARED_C__

#include "zd80211.h"
#include "zddebug.h"
#include "zd1205.h"
extern struct net_device *g_dev;

void mkFragment(Signal_t *signal, FrmDesc_t *pfrmDesc, U8 *pEthHdr)
{
        struct zd1205_private *macp=g_dev->priv;
        Frame_t *mpdu, *curMpdu;
        FrmInfo_t *pfrmInfo;
        BOOLEAN bWep;
        U16 pdusize;
        U8 *pBody;
        U16 len;
        U8 fn;
        U8 *pByte;
        int i;
        Hash_t *pHash;
        MICvar *pTxMicKey = NULL;
        U8 KeyInstalled = 0;
        U8 vapId = 0;
        U8 Num;
        U8 bDataFrm = signal->bDataFrm;
        //U8 bDataFrm = pfrmDesc->bDataFrm;
        U16 HdrLen;

        ZDEBUG("mkFragment");
        pfrmDesc->CalMIC[MIC_LNG]=FALSE;
        pfrmInfo = &signal->frmInfo;
        pfrmInfo->frmDesc = pfrmDesc; //make connection for signal and frmDesc
        //PSDEBUG_V("mkFrag pfrmDesc", (U32)pfrmInfo->frmDesc);
        mpdu = pfrmDesc->mpdu;
        vapId = signal->vapId;
#if 0

        if (mDynKeyMode == DYN_KEY_TKIP || mDynKeyMode == DYN_KEY_AES) {
                if (bDataFrm) {
                        if (isGroup(addr1(mpdu))) {
                                KeyInstalled = mGkInstalled;

                                if (mDynKeyMode == DYN_KEY_TKIP) {
                                        pTxMicKey = &mBcMicKey;
                                        if (mWpaBcKeyLen != 32) // Not TKIP, don't make MIC
                                                KeyInstalled = 0;
                                }
                        } else { //unicast
                                pHash = HashSearch(addr1(mpdu));
                                if (!pHash) {
                                        FPRINT("HashSearch2 failed !!!");
                                        zd1205_dump_data("addr1 = ", (U8 *)addr1(mpdu), 6);
                                        KeyInstalled = 0;
                                } else {
                                        if (mDynKeyMode == DYN_KEY_TKIP)
                                                pTxMicKey = &pHash->TxMicKey;
                                        KeyInstalled = pHash->pkInstalled;
                                }
                        }

                        if ((KeyInstalled) && (mDynKeyMode == DYN_KEY_TKIP)) {
#endif
                                // The following section is used for TKIP-MIC append.
                                if (bDataFrm) {
                                        if (macp->cardSetting.WPAIeLen) // WPA is supported for now.
                                        {
                                                if (isGroup(addr1(mpdu)))
                                                {// Prepare to send the BC/MC packet.
                                                        KeyInstalled=mGkInstalled;
                                                        if (KeyInstalled) {
                                                                if(mWpaBcKeyLen == 32)
                                                                        pTxMicKey = &mBcMicKey;
                                                        } else
                                                                FPRINT("MkFrag: No Group key installed\n");
                                                } else
                                                {// Prepare to send the UC packet.
                                                        pHash = HashSearch(addr1(mpdu));
                                                        KeyInstalled=pHash->pkInstalled;
                                                        if (!pHash) {
                                                                FPRINT("HashSearch2 failed !!!");
                                                                zd1205_dump_data("addr1 = ", (U8 *)addr1(mpdu), 6);
                                                        } else if(KeyInstalled) {
                                                                //if (pHash->keyLength==32)
                                                                if (pHash->encryMode == TKIP)
                                                                        pTxMicKey = &pHash->TxMicKey;
                                                        } else {
                                                                FPRINT("MkFrag: Can't find Pairwise key\n");
                                                        }
                                                }

                                                if (pTxMicKey != NULL)
                                                { // This section is used for TKIP-MIC append.

                                                        U16 len = mpdu->bodyLen;

                                                        // calculate and append MIC to payload before fragmentation
                                                        MICclear(pTxMicKey);


                                                        if(mBssType == AP_BSS || mBssType==INDEPENDENT_BSS)
                                                                pByte = &mpdu->header[4]; //DA=Addr1
                                                        else //if (mBssType == INFRASTRUCTURE_BSS)
                                                                pByte = &mpdu->header[16];//DA=Addr3
                                                        for(i=0; i<6; i++) { //for DA
                                                                MICappendByte(*pByte++, pTxMicKey);
                                                        }

                                                        if(mBssType == INFRASTRUCTURE_BSS || mBssType==INDEPENDENT_BSS)
                                                                pByte = &mpdu->header[10]; //SA=Addr2
                                                        else //if (mBssType == AP_BSS)
                                                                pByte = &mpdu->header[16];
                                                        for(i=0; i<6; i++) { //for SA
                                                                MICappendByte(*pByte++, pTxMicKey);
                                                        }

                                                        MICappendByte(0, pTxMicKey);
                                                        MICappendByte(0, pTxMicKey);
                                                        MICappendByte(0, pTxMicKey);
                                                        MICappendByte(0, pTxMicKey);

                                                        pByte = mpdu->body;
                                                        for (i=0; i<len; i++) {
                                                                MICappendByte(*pByte++, pTxMicKey);
                                                        }
                                                        MICgetMIC(pfrmDesc->CalMIC, pTxMicKey);
                                                        pfrmDesc->CalMIC[MIC_LNG]=TRUE;
                                                        //zd1205_dump_data("add sw mic:",(u8*)pfrmDesc->CalMIC, 8);
                                                        mpdu->bodyLen += MIC_LNG;
                                                }
                                        }
                                }

                                bWep = mPrivacyInvoked;
                                if ((!bDataFrm) && (!(pfrmDesc->ConfigSet & FORCE_WEP_SET))) {
                                        bWep = FALSE;
                                } else {
                                        if (pfrmDesc->ConfigSet & EAPOL_FRAME_SET)
                                                bWep = FALSE;
                                }

                                if (KeyInstalled) // After pairwise key installed, even Eapol frame need to be encrypted
                                {
                                        bWep = TRUE;
                                } else {
                                        if (bDataFrm && !mKeyFormat && bWep) {
                                                printk(KERN_DEBUG "chkpnt 001\n");
                                                bWep = FALSE;
                                        }
                                }

                                pfrmInfo->eol = 0;

                                pdusize = mFragThreshold;
                                if ((!isGroup(addr1(mpdu))) && (mpdu->HdrLen + mpdu->bodyLen + CRC_LNG > pdusize)) { //Need fragment
                                        pdusize -= mpdu->HdrLen + CRC_LNG;
                                        pfrmInfo->fTot = (mpdu->bodyLen + (pdusize-1)) / pdusize;
                                        if (pfrmInfo->fTot == 0)
                                                pfrmInfo->fTot = 1;
                                } else {
                                        pdusize = mpdu->bodyLen;
                                        pfrmInfo->fTot = 1;
                                }

                                curMpdu = mpdu;
                                pBody = mpdu->body;
                                len = mpdu->bodyLen;
                                Num = pfrmInfo->fTot;
                                HdrLen = mpdu->HdrLen;

                                for (fn=0; fn<Num; fn++) {
                                        if (fn) {
                                                curMpdu = &pfrmDesc->mpdu[fn];
                                                memcpy(&curMpdu->header[0], &mpdu->header[0], HdrLen); //make header
                                                curMpdu->HdrLen = HdrLen;
                                                curMpdu->body = pBody;
                                        }
                                        curMpdu->header[22] = ((curMpdu->header[22] & 0xF0) | fn);

                                        if (fn == (Num - 1)) {
                                                curMpdu->bodyLen = len;
                                                curMpdu->header[1] &= ~MORE_FRAG_BIT;
                                        } else {
                                                curMpdu->bodyLen = pdusize;
                                                pBody += pdusize;
                                                len -= pdusize;
                                                curMpdu->header[1] |= MORE_FRAG_BIT;
                                        }

                                        if (bWep)
                                                curMpdu->header[1] |= WEP_BIT;
                                }
                        }


                        BOOLEAN sendMgtFrame(Signal_t *signal, FrmDesc_t *pfrmDesc) {
                                //	ZDEBUG("sendMgtFrame");
                                pfrmDesc->ConfigSet &= ~INTRA_BSS_SET;
                                pfrmDesc->ConfigSet &= ~EAPOL_FRAME_SET;
                                pfrmDesc->pHash = NULL;
                                pdot11Obj->ReleaseBuffer(signal->buf);
                                signal->buf = NULL;
                                signal->bDataFrm = 0;
                                //pfrmDesc->bDataFrm = 0;
                                mkFragment(signal, pfrmDesc, NULL);
                                return SendPkt(signal, pfrmDesc, TRUE);
                        }
#if 0
                        BOOLEAN	getElem(Frame_t	*frame, ElementID  eleID, Element  *elem) {
                                U8 k = 0; 	//offset bytes to first element
                                U8 n = 0; 	//num. of element


                                U8 pos;		//current position
                                U8 len;
                                U8 max_len=34;
                                switch (frmType(frame)) {
                                case ST_PROBE_REQ:
                                        k = 0;
                                        n = 4;
                                        if ((mDynKeyMode == DYN_KEY_TKIP) || (mDynKeyMode == DYN_KEY_AES))
                                                n++;
                                        break;

                                case ST_ASOC_REQ:

                                        k = 4;
                                        n = 4;
                                        if ((mDynKeyMode == DYN_KEY_TKIP) || (mDynKeyMode == DYN_KEY_AES))
                                                n++;
                                        break;

                                case ST_REASOC_REQ:
                                        k = 10;
                                        n = 4;
                                        if ((mDynKeyMode == DYN_KEY_TKIP) || (mDynKeyMode == DYN_KEY_AES))
                                                n++;
                                        break;

                                case ST_AUTH:
                                        k = 6;
                                        n = 1;
                                        max_len=130;
                                        break;

                                case ST_BEACON:
                                case ST_PROBE_RSP:
                                        k = 12;
                                        n = 6;

                                        if (mBssType == INDEPENDENT_BSS)
                                                n++;
                                        if ((mDynKeyMode == DYN_KEY_TKIP) || (mDynKeyMode == DYN_KEY_AES))
                                                n++;
                                        n++; //for country info
                                        break;

                                case ST_ASOC_RSP:
                                case ST_REASOC_RSP:
                                        k = 6;
                                        n = 2;

                                        if ((mDynKeyMode == DYN_KEY_TKIP) || (mDynKeyMode == DYN_KEY_AES))
                                                n++;
                                        break;

                                default:
                                        elem->buf[1] = 0;
                                        return FALSE;
                                }

                                //while(n--){
                                while(k < frame->bodyLen) {
                                        pos = frame->body[k];
                                        len = frame->body[k+1] + 2;

                                        if ((pos == eleID) && (len <= max_len)) {	//match
                                                if (eleID == EID_WPA) //Valid WPA IE
                                                {
                                                        if (len > 20)
                                                        {
                                                                memcpy((U8 *)elem, &frame->body[k], len);
                                                                return TRUE;
                                                        } else
                                                                k += len;
                                                } else {
                                                        memcpy((U8 *)elem, &frame->body[k], len);
                                                        return TRUE;
                                                }
                                        } else {
                                                k += len;
                                        }
                                }

                                elem->buf[1] = 0; //set element length to zero
                                return FALSE;
                        }
#endif
                        BOOLEAN	getElem(Frame_t	*frame, ElementID  eleID, Element  *elem) {
                                U8 k = 0; 	//offset bytes to first element
                                U8 pos;		//current position
                                U8 len;
                                U8 max_len = 130;

                                switch (frmType(frame)) {
                                case ST_PROBE_REQ:
                                        k = 0;
                                        break;

                                case ST_ASOC_REQ:
                                        k = 4;
                                        break;

                                case ST_REASOC_REQ:
                                        k = 10;
                                        break;

                                case ST_AUTH:
                                        k = 6;
                                        break;

                                case ST_BEACON:
                                case ST_PROBE_RSP:
                                        k = 12;
                                        break;

                                case ST_ASOC_RSP:
                                case ST_REASOC_RSP:
                                        k = 6;
                                        break;

                                default:
                                        elem->buf[1] = 0;
                                        return FALSE;
                                }

                                //jxiao
                                while (k < frame->bodyLen) {
                                        pos=frame->body[k];
                                        len=frame->body[k+1]+2;
                                        if ((pos==eleID) && (len <= max_len)) {
                                                if (eleID == EID_WPA)//The WMM and WPA use the same Element ID.
                                                {
                                                        if (len > 20)
                                                        {
                                                                u8 WPA_OUI[4]={0x00, 0x50, 0xF2, 0x01};
                                                                //u8 WMM_OUI[4]={0x00, 0x50, 0xF2, 0x02};
                                                                if (memcmp (WPA_OUI, (u8*)&frame->body[k+2], 4)==0) {
                                                                        memcpy((U8 *)elem, &frame->body[k], len);
                                                                        return TRUE;
                                                                }
                                                                //else if (memcmp (WMM_OUI, (u8*)&frame->body[k+2], 4)==0)
                                                                //{
                                                                //   memcpy((U8 *)&elem->buf[0]+128, &frame->body[k], len);
                                                                //return TRUE;
                                                                //}
                                                                else
                                                                        k += len;
                                                        } else
                                                                k += len;
                                                } else {
                                                        memcpy((U8 *)elem, &frame->body[k], len);
                                                        return TRUE;
                                                }
                                        } else {
                                                k += len;
                                        }
                                }
                                elem->buf[1]=0;
                                return FALSE;
                        }

                        void mkAuthFrm(FrmDesc_t* pfrmDesc, MacAddr_t *addr1, U16 Alg, U16 Seq,
                                       U16 Status, U8 *pChalng, U8 vapId) {
                                U8 *body;
                                U16 len;
                                Frame_t *pf = pfrmDesc->mpdu;

                                setFrameType(pf, ST_AUTH);
                                pf->body = pfrmDesc->buffer;
                                body = pf->body;
                                setAddr1(pf, addr1);
                                setAddr2(pf, &dot11MacAddress);
                                setAddr3(pf, &mBssId);
                                pf->HdrLen = MAC_HDR_LNG;

                                body[0] = Alg & 0xff;			//AuthAlg
                                body[1] = (Alg & 0xff00) >> 8;
                                body[2] = Seq & 0xff;			//AuthSeq
                                body[3] = (Seq & 0xff00) >> 8;
                                body[4] = Status & 0xff;		//Status
                                body[5] = (Status & 0xff00) >> 8;
                                len = 6;

                                if ((Alg == SHARE_KEY) && ((Seq == 2)|| (Seq == 3)) && (pChalng)) {
                                        body[len] = EID_CTEXT;
                                        body[len+1] = CHAL_TEXT_LEN;
                                        memcpy(&body[len+2], pChalng, CHAL_TEXT_LEN);
                                        len += (2+CHAL_TEXT_LEN);
                                }

                                pf->bodyLen = len;
                        }

                        void mkRe_AsocRspFrm(FrmDesc_t* pfrmDesc, TypeSubtype subType, MacAddr_t *addr1,
                                             U16 Cap, U16 Status, U16 Aid, Element *pSupRates, Element *pExtRates, U8 vapId) {
                                U8 *body;
                                U8 elemLen;
                                U16 len;
                                Frame_t *pf = pfrmDesc->mpdu;

                                setFrameType(pf, subType);
                                pf->body = pfrmDesc->buffer;
                                body = pf->body;
                                setAddr1(pf, addr1);
                                setAddr2(pf, &dot11MacAddress);
                                setAddr3(pf, &mBssId);
                                pf->HdrLen = MAC_HDR_LNG;

                                body[0] = Cap & 0xff;			//Cap
                                body[1] = (Cap & 0xff00) >> 8;
                                body[2] = Status & 0xff;		//Status
                                body[3] = (Status & 0xff00) >> 8;
                                body[4] = Aid & 0xff;			//AID
                                body[5] = (Aid & 0xff00) >> 8;
                                len = 6;

                                elemLen = pSupRates->buf[1]+2;
                                memcpy(&body[len], (U8 *)pSupRates, elemLen); //Support Rates
                                len += elemLen;

                                if ((mMacMode != PURE_B_MODE) && (PURE_A_MODE != mMacMode) && (pExtRates)) {
                                        elemLen = pExtRates->buf[1]+2;
                                        memcpy(&body[len], (U8 *)pExtRates, elemLen); //Extended rates
                                        len += elemLen;
                                }

                                pf->bodyLen = len;

                        }


                        void mkProbeRspFrm(FrmDesc_t* pfrmDesc, MacAddr_t *addr1, U16 BcnInterval,
                                           U16 Cap, Element *pSsid, Element *pSupRates, Element *pDsParms,
                                           Element *pExtRates, Element *pWpa, U8 vapId)
                        {
                                struct zd1205_private *macp=g_dev->priv;

                                U8 *body;
                                U8 elemLen;
                                U16 len;
                                Frame_t *pf = pfrmDesc->mpdu;

                                setFrameType(pf, ST_PROBE_RSP);
                                pf->body = pfrmDesc->buffer;
                                body = pf->body;
                                setAddr1(pf, addr1);
                                setAddr2(pf, &dot11MacAddress);
                                setAddr3(pf, &mBssId);
                                pf->HdrLen = MAC_HDR_LNG;

                                body[8] = BcnInterval & 0xff;	//BcnPeriod
                                body[9] = (BcnInterval & 0xff00) >> 8;
                                body[10] = Cap & 0xff;			//Cap
                                body[11] = (Cap & 0xff00) >> 8;

                                len = 12;
                                elemLen = pSsid->buf[1]+2;
                                memcpy(&body[len], (U8 *)pSsid, elemLen); //SSID
                                len += elemLen;

                                elemLen = pSupRates->buf[1]+2;

                                memcpy(&body[len], (U8 *)pSupRates, elemLen); //Suported rates
                                len += elemLen;

                                elemLen = pDsParms->buf[1]+2;
                                memcpy(&body[len], (U8 *)pDsParms, elemLen); //Extended rates
                                len += elemLen;

                                if ((mMacMode != PURE_B_MODE)&& (mMacMode != PURE_A_MODE) && (pExtRates)) {
                                        elemLen = pExtRates->buf[1]+2;
                                        memcpy(&body[len], (U8 *)pExtRates, elemLen); //Extended rates
                                        len += elemLen;
                                }
#if 0
                                if (((mDynKeyMode == DYN_KEY_TKIP) || (mDynKeyMode == DYN_KEY_AES))  && (pWpa)) {
                                        elemLen = pWpa->buf[1]+2;
                                        memcpy(&body[len], (U8 *)pWpa, elemLen); //WPA IE
                                        len += elemLen;
                                }
#endif
                                //if (((mDynKeyMode == DYN_KEY_TKIP) || (mDynKeyMode == DYN_KEY_AES))  && (pWpa)){
                                if (macp->cardSetting.WPAIeLen) {
                                        //elemLen = pWpa->buf[1]+2;
                                        //memcpy(&body[len], (U8 *)pWpa, elemLen); //WPA IE
                                        memcpy(&body[len], macp->cardSetting.WPAIe, macp->cardSetting.WPAIeLen);
                                        len += macp->cardSetting.WPAIeLen;
                                }

                                pf->bodyLen = len;
                        }


                        void mkDisAssoc_DeAuthFrm(FrmDesc_t* pfrmDesc, TypeSubtype subType, MacAddr_t *addr1,
                                                  U16 Reason, U8 vapId) {
                                U8 *body;
                                Frame_t *pf = pfrmDesc->mpdu;

                                setFrameType(pf, subType);
                                pf->body = pfrmDesc->buffer;
                                body = pf->body;
                                setAddr1(pf, addr1);
                                setAddr2(pf, &dot11MacAddress);
                                setAddr3(pf, &mBssId);
                                pf->HdrLen = MAC_HDR_LNG;

                                body[0] = Reason & 0xff;	//Reason Code
                                body[1] = (Reason & 0xff00) >> 8;

                                pf->bodyLen = 2;
                        }


                        void sendProbeRspFrm(MacAddr_t *addr1, U16 BcnInterval, U16 Cap,
                                             Element *pSsid, Element *pSupRates, Element *pDsParms,
                                             Element *pExtRates, Element *pWpa, U8 vapId) {
                                Signal_t *signal;
                                FrmDesc_t *pfrmDesc;

                                if ((signal = allocSignal()) == NULL)
                                        return;

                                if ((pfrmDesc = allocFdesc()) == NULL) {
                                        freeSignal(signal);
                                        return;
                                }

                                mkProbeRspFrm(pfrmDesc, addr1, BcnInterval, Cap, pSsid, pSupRates, pDsParms,
                                              pExtRates, pWpa, vapId);
                                sendMgtFrame(signal, pfrmDesc);
                        }


                        void mkProbeReqFrm(FrmDesc_t* pfrmDesc, MacAddr_t *addr1, Element *pSsid, Element *pSupRates,
                                           Element *pExtRates, Element *pWpa, U8 vapId) {
                                U8 *body;
                                U8 elemLen;
                                U16 len;
                                Frame_t *pf = pfrmDesc->mpdu;

                                setFrameType(pf, ST_PROBE_REQ);
                                pf->body = pfrmDesc->buffer;
                                body = pf->body;
                                setAddr1(pf, addr1);
                                setAddr2(pf, &dot11MacAddress);
                                setAddr3(pf, &dot11BCAddress);
                                pf->HdrLen = MAC_HDR_LNG;

                                len = 0;
                                if (pSsid->buf[1] > 0) {
                                        elemLen = pSsid->buf[1]+2;

                                        memcpy(&body[len], (U8 *)pSsid, elemLen); //Extended rates
                                        len += elemLen;
                                } else {
                                        body[0] = pSsid->buf[0];
                                        body[1] = 0; //broadcast SSID
                                        len += 2;
                                }

                                elemLen = pSupRates->buf[1]+2;
                                memcpy(&body[len], (U8 *)pSupRates, elemLen); //Extended rates
                                len += elemLen;

                                if ((mMacMode != PURE_A_MODE) && (mMacMode != PURE_B_MODE) && (pExtRates)) {
                                        elemLen = pExtRates->buf[1]+2;
                                        memcpy(&body[len], (U8 *)pExtRates, elemLen); //Extended rates
                                        len += elemLen;
                                }

                                if ((mDynKeyMode == DYN_KEY_TKIP)  && (pWpa)) {
                                        elemLen = pWpa->buf[1]+2;
                                        memcpy(&body[len], (U8 *)pWpa, elemLen); //WPA IE
                                        len += elemLen;
                                }

                                pf->bodyLen = len;
                        }


                        void mkRe_AsocReqFrm(FrmDesc_t* pfrmDesc, TypeSubtype subType, MacAddr_t *addr1,
                                             U16 Cap, U16 LisInterval, MacAddr_t *oldAP, Element *pSsid, Element *pSupRates,
                                             Element *pExtRates, Element *pWpa, U8 vapId)
                        {
                                U8 *body;
                                U8 elemLen;
                                U16 len;
                                Frame_t *pf = pfrmDesc->mpdu;

                                setFrameType(pf, subType);
                                pf->body = pfrmDesc->buffer;
                                body = pf->body;
                                setAddr1(pf, addr1);
                                setAddr2(pf, &dot11MacAddress);
                                setAddr3(pf, &mBssId);
                                pf->HdrLen = MAC_HDR_LNG;

                                body[0] = Cap & 0xff;			//Cap
                                body[1] = (Cap & 0xff00) >> 8;
                                body[2] = LisInterval & 0xff;	//LisInterval
                                body[3] = (LisInterval & 0xff00) >> 8;
                                len = 4;

                                if (subType == ST_REASOC_REQ) {
                                        memcpy(&body[4], oldAP, 6);
                                        len = 10;
                                }

                                elemLen = pSsid->buf[1]+2;
                                memcpy(&body[len], (U8 *)pSsid, elemLen); 	//SSID
                                len += elemLen;

#ifdef DEBUG_DUMP_ASSOC_REQ

                                zd1205_dump_data("SSID element:", (U8*) &body[len-elemLen], elemLen);
#endif

                                elemLen = pSupRates->buf[1]+2;
                                memcpy(&body[len], (U8 *)pSupRates, elemLen); //Support Rates
                                len += elemLen;
#ifdef DEBUG_DUMP_ASSOC_REQ

                                zd1205_dump_data("SupportedRate element:", (U8*) &body[len-elemLen], elemLen);
#endif

                                if ((mMacMode != PURE_B_MODE) && (mMacMode != PURE_A_MODE)&& (pExtRates)) {
                                        elemLen = pExtRates->buf[1]+2;
                                        memcpy(&body[len], (U8 *)pExtRates, elemLen); //Extended rates
                                        len += elemLen;
                                }

                                // WPA IE
                                if (pWpa->buf[1] != 0) {
                                        elemLen = pWpa->buf[1]+2;
                                        memcpy(&body[len], (U8 *)pWpa, elemLen); // WPA IE
                                        len += elemLen;
                                }

                                pf->bodyLen = len;
                        }

                        BOOLEAN sendPsPollFrame(Signal_t *signal, FrmDesc_t *pfrmDesc, MacAddr_t *addr1, U16 aid) {
                                FrmInfo_t *pfrmInfo;
                                Frame_t *pf = pfrmDesc->mpdu;

                                setFrameType(pf, ST_PS_POLL);
                                setAid(pf, aid);
                                setAddr1(pf, addr1);
                                setAddr2(pf, &dot11MacAddress);
                                pf->HdrLen = 16;
                                pf->bodyLen = 0;

                                pfrmDesc->ConfigSet |= PS_POLL_SET;
                                pfrmDesc->ConfigSet &= ~INTRA_BSS_SET;
                                pfrmDesc->ConfigSet &= ~EAPOL_FRAME_SET;
                                pfrmDesc->pHash = NULL;
                                signal->buf = NULL;
                                signal->bDataFrm = 0;
                                //pfrmDesc->bDataFrm = 0;

                                pfrmInfo = &signal->frmInfo;
                                pfrmInfo->frmDesc = pfrmDesc; //make connection for signal and frmDesc
                                return SendPkt(signal, pfrmDesc, TRUE);
                        }


                        BOOLEAN sendNullDataFrame(Signal_t *signal, FrmDesc_t *pfrmDesc, MacAddr_t *addr1) {
                                Frame_t *pf = pfrmDesc->mpdu;

                                setFrameType(pf, ST_NULL_FRAME);
                                pf->header[1] = TO_DS_BIT;

                                if (mPwrState) {
                                        pf->header[1] |= PW_SAVE_BIT;
                                } else
                                        pf->header[1] &= ~PW_SAVE_BIT;

                                setAddr1(pf, addr1);
                                setAddr2(pf, &dot11MacAddress);
                                setAddr3(pf, &mBssId);
                                pf->HdrLen = MAC_HDR_LNG;
                                pf->bodyLen = 0;

                                pfrmDesc->ConfigSet &= ~INTRA_BSS_SET;
                                pfrmDesc->ConfigSet &= ~EAPOL_FRAME_SET;
                                pfrmDesc->pHash = NULL;
                                signal->buf = NULL;
                                signal->bDataFrm = 1;

                                //pfrmDesc->bDataFrm = 0;
                                mkFragment(signal, pfrmDesc, NULL);
                                return SendPkt(signal, pfrmDesc, TRUE);
                        }


                        U8 RateConvert(U8 rate) {
#if defined(OFDM)
                                switch (rate) {
                                case 2  :
                                        return 0;  // 1M
                                case 4  :
                                        return 1;  // 2M
                                case 11 :
                                        return 2;  // 5.5M
                                case 22 :
                                        return 3;  // 11M
                                case 12 :
                                        return 4;  // 6M
                                case 18 :
                                        return 5;  // 9M
                                case 24 :
                                        return 6;  // 12M
                                case 36 :
                                        return 7;	// 18M
                                case 48 :
                                        return 8;	// 24M
                                case 72 :
                                        return 9;	// 36M
                                case 96 :
                                        return 0xa;	// 48M
                                case 108:
                                        return 0xb;	// 54M
                                }
#else
                                switch (rate) {
                                case 2  :
                                        return 0;  // 1M
                                case 4  :
                                        return 1;  // 2M
                                case 11 :
                                        return 2;  // 5.5M
                                case 22 :
                                        return 3;  // 11M
                                case 33 :
                                        return 4;  // 16.5M
                                case 44 :
                                        return 5;  // 22M
                                case 55 :
                                        return 6;  // 27.5M
#if defined(ECCK_60_5)

                                case 66 :
                                        return 7;  // 33M
                                case 77 :
                                        return 8;  // 38.5M
                                case 88 :
                                        return 9;  // 44M
                                case 99 :
                                        return 10; // 49.5M
                                case 110:
                                        return 11; // 55M
                                case 121:
                                        return 12; // 60.5M
#endif

                                }
#endif

                                return 3;	// 11M
                        }
#endif
