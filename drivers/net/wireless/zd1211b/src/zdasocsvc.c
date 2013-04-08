#ifndef __ZDASOCSVC_C__
#define __ZDASOCSVC_C__

#include "zd80211.h"
#include "zd1205.h"

U8 AsocState = STE_ASOC_IDLE;
static MacAddr_t AsSta;
extern struct net_device *g_dev;


BOOLEAN Disasoc(Signal_t *signal)
{
        FrmDesc_t *pfrmDesc;
        Frame_t *rdu;
        MacAddr_t Sta;
        ReasonCode Rsn;
        U8 vapId = 0;

        //ZDEBUG("Disasoc");
        //FPRINT("Disasoc");
        pfrmDesc = signal->frmInfo.frmDesc;
        rdu = pfrmDesc->mpdu;
        memcpy((U8 *)&Sta, (U8 *)(addr2(rdu)), 6); // Get the address of the transmitter.
        Rsn = (ReasonCode)(reason(rdu));

        if (mBssType == INFRASTRUCTURE_BSS) {// This frame should be come from the associated AP.
                if ((!mAssoc) || (memcmp(&Sta, (U8*)&mBssId, 6) != 0)) { //Not for this BSSID
                        //discard this packet
                        freeFdesc(pfrmDesc);
                        return TRUE;
                } else {
                        freeFdesc(pfrmDesc);
                        UpdateStaStatus(&Sta, STATION_STATE_DIS_ASOC, vapId);
                        pdot11Obj->StatusNotify(STA_DISASSOCIATED, (U8 *)&Sta);
                        mAssoc = FALSE;
                        memset((U8 *)&mBssId, 0, 6);
                        printk(KERN_ERR "Rx Disasoc from AP(Rsn:%d),set DIS_CONNECT_SET\n",Rsn);
                        mRequestFlag |= DIS_CONNECT_SET;
                        return TRUE;
                }
        } else if (mBssType == AP_BSS) {
                if (memcmp(addr1(rdu), (U8*)&mBssId, 6)) { //Not for this BSSID
                        freeFdesc(pfrmDesc);
                        return TRUE;
                }
                UpdateStaStatus(&Sta, STATION_STATE_DIS_ASOC, vapId);
                freeFdesc(pfrmDesc);
                //here to handle disassoc ind.
                pdot11Obj->StatusNotify(STA_DISASSOCIATED, (U8 *)&Sta);
                return TRUE;
        } else {
                freeFdesc(pfrmDesc);
                return TRUE;
        }
}


BOOLEAN DisasocReq(Signal_t *signal)
{
        FrmDesc_t *pfrmDesc;
        MacAddr_t Sta;
        ReasonCode Rsn = RC_UNSPEC_REASON;
        U8 vapId = 0;

        //ZDEBUG("DisasocReq");
        //FPRINT("DisasocReq");

        memcpy((U8 *)&Sta, (U8 *)&signal->frmInfo.Sta, 6);
        Rsn = signal->frmInfo.rCode;
        pdot11Obj->StatusNotify(STA_DISASSOCIATED, (U8 *)&Sta);

        vapId = signal->vapId;
        UpdateStaStatus(&Sta, STATION_STATE_DIS_ASOC, vapId);

        pfrmDesc = allocFdesc();
        if(!pfrmDesc) {
                sigEnque(pMgtQ, (signal));
                return FALSE;
        }

        mkDisAssoc_DeAuthFrm(pfrmDesc, ST_DISASOC, &Sta, Rsn, vapId);
        sendMgtFrame(signal, pfrmDesc);

        if (mBssType == INFRASTRUCTURE_BSS) {
                if (memcmp((U8 *)&mBssId, (U8 *)&Sta, 6) == 0) { // my AP
                        mAssoc = FALSE;
                        memset((U8 *)&mBssId, 0, 6);
                }
        }

        return FALSE;
}


BOOLEAN Re_Asociate(Signal_t *signal)
{
        struct zd1205_private *macp=g_dev->priv;
        Hash_t	*pHash;
        FrmDesc_t *pfrmDesc;
        Frame_t *rdu;
        MacAddr_t Sta;
        U16 aid = 0;
        StatusCode asStatus;
        U8 lsInterval;
        Element WPA;
        Element asSsid;
        Element asRates;
        Element extRates;
        U16 cap;
        U8 ZydasMode = 0;
        int i;
        U8 tmpMaxRate = 0x02;
        U8 MaxRate;
        U16 notifyStatus = STA_ASOC_REQ;
        U16 notifyStatus1 = STA_ASSOCIATED;
        TypeSubtype type = ST_ASOC_RSP;
        U8	Preamble = 0;
        U8	HigestBasicRate = 0;
        U8	vapId = 0;
        U8	Len;
        Element *pExtRate = NULL;

        BOOLEAN bErpSta = TRUE;
        pExtRate = &mExtRates;

        ZDEBUG("Re_Asociate");
        pfrmDesc = signal->frmInfo.frmDesc;

        if (mBssType == INFRASTRUCTURE_BSS) {
                freeFdesc(pfrmDesc);
                return TRUE;
        } else if (mBssType == AP_BSS) {
                rdu = pfrmDesc->mpdu;
                lsInterval = listenInt(pfrmDesc->mpdu);
                //FPRINT_V("lsInterval", lsInterval);
                cap = cap(rdu);
                memcpy((U8 *)&Sta, (U8 *)addr2(rdu), 6);

                if ((isGroup(addr2(rdu))) ||  (!getElem(rdu, EID_SSID, &asSsid))
                                || (!getElem(rdu, EID_SUPRATES, &asRates))) {
                        freeFdesc(pfrmDesc);
                        return TRUE;
                }

                if ((eLen(&asSsid) != eLen(&dot11DesiredSsid) ||
                                memcmp(&asSsid, &dot11DesiredSsid, eLen(&dot11DesiredSsid)+2) != 0)) {
                        freeFdesc(pfrmDesc);
                        return TRUE;
                }

                //check capability
                if (cap & CAP_SHORT_PREAMBLE) {
                        if (mPreambleType == LONG_PREAMBLE) { //we are long preamble, and STA is short preamble capability
                                freeFdesc(pfrmDesc);
                                return TRUE;
                        } else
                                Preamble = 1;
                } else {
                        Preamble = 0;
                }


                // Privacy not match
                if (cap & CAP_PRIVACY) {
                        if (!mPrivacyInvoked && macp->cardSetting.WPAIeLen==0) {
                                //if (!mPrivacyInvoked){
                                freeFdesc(pfrmDesc);
                                return TRUE;
                        }
                } else {
                        if (mPrivacyInvoked) {
                                freeFdesc(pfrmDesc);
                                return TRUE;
                        }
                }

                //check short slot time
#if 0//defined(OFDM)
                if (mMacMode != PURE_B_MODE)
                {
                        if (cap & CAP_SHORT_SLOT_TIME) {
                                if (!(mCap & CAP_SHORT_SLOT_TIME)) {
                                        FPRINT("CAP_SHORT_SLOT_TIME not match!!");
                                        freeFdesc(pfrmDesc);
                                        return TRUE;
                                }
                        } else {
                                if (mCap & CAP_SHORT_SLOT_TIME) {
                                        asStatus = (StatusCode)SC_UNSUP_SHORT_SLOT_TIME;
                                        goto check_failed;
                                }
                        }
                }

                FPRINT_V("cap", cap);
#endif


                //check supported rates
                Len = eLen(&asRates);
                for (i=0; i<Len; i++) {
                        if ( (asRates.buf[2+i] & 0x7f) > tmpMaxRate ) {
                                tmpMaxRate = (asRates.buf[2+i] & 0x7f);
                                if (asRates.buf[2+i] & 0x80)
                                        HigestBasicRate = asRates.buf[2+i];
                        }

                        if (((asRates.buf[2+i] & 0x7f) == 0x21) && (!(cap & CAP_PBCC_ENABLE))) { //Zydas 16.5M
                                ZydasMode = 1;
                                mZyDasModeClient = TRUE;
                                //FPRINT("ZydasMode");
                        }
                }

                if (!getElem(rdu, EID_EXT_RATES, &extRates)) {
                        void *reg = pdot11Obj->reg;
                        // 11b STA
                        //FPRINT("11b STA");
                        if (mMacMode == PURE_G_MODE) { // don't support b only sta
                                MaxRate = RateConvert((tmpMaxRate & 0x7f));
                                //MaxBasicRate = RateConvert((HigestBasicRate & 0x7f));
                                if (MaxRate < pdot11Obj->BasicRate) {
                                        //FPRINT_V("MaxRate", MaxRate);
                                        //FPRINT_V("pdot11Obj->BasicRate", pdot11Obj->BasicRate);
                                        asStatus = SC_UNSUP_RATES;
                                        goto check_failed;
                                }
                        }
                        bErpSta = FALSE;
                        pdot11Obj->ConfigFlag |= NON_ERP_PRESENT_SET;
                        pdot11Obj->ConfigFlag &= ~SHORT_SLOT_TIME_SET;
                        mCap &= ~CAP_SHORT_SLOT_TIME;
                        pdot11Obj->SetReg(reg, ZD_CWmin_CWmax, CW_NORMAL_SLOT);
                        if(PURE_A_MODE == mMacMode) {
                                pdot11Obj->ConfigFlag &= ~NON_ERP_PRESENT_SET;
                                pdot11Obj->ConfigFlag |= SHORT_SLOT_TIME_SET;
                                mCap |= CAP_SHORT_SLOT_TIME;
                                pdot11Obj->SetReg(reg, ZD_CWmin_CWmax, CW_SHORT_SLOT);
                        }

                } else { //11g STA
                        if (mMacMode != PURE_B_MODE && mMacMode != PURE_A_MODE) {

                                Len = eLen(&extRates);
                                for (i=0; i<Len; i++) {
                                        if ( (extRates.buf[2+i] & 0x7f) > tmpMaxRate ) {
                                                tmpMaxRate = (extRates.buf[2+i] & 0x7f);
                                        }
                                }
                                bErpSta = TRUE;
                                //FPRINT("11g STA");
                        } else {
                                FPRINT("Pure B mode don't support G sta");
                                asStatus = SC_UNSUP_RATES;
                                goto check_failed;
                        }
                }

                MaxRate = RateConvert((tmpMaxRate & 0x7f));

                if (MaxRate > mMaxTxRate)
                        MaxRate = mMaxTxRate;

                if (signal->id == SIG_REASSOC)
                        notifyStatus = STA_REASOC_REQ;

                if (!pdot11Obj->StatusNotify(notifyStatus, (U8 *)&Sta)) { //Accept it
                        //if ((mDynKeyMode == DYN_KEY_TKIP) || (mDynKeyMode == DYN_KEY_AES)){
                        if (macp->cardSetting.WPAIeLen) {
                                if (getElem(rdu, EID_WPA, &WPA)||getElem(rdu,EID_RSN,&WPA)) {
                                        //zd1205_OctetDump("AssocRequest = ", asRdu->body, asRdu->bodyLen);
                                        //zd1205_OctetDump("AssocRequest WPA_IE = ", &WPA.buf[2], WPA.buf[1]);
                                        if (pdot11Obj->AssocRequest) {
                                                if (pdot11Obj->AssocRequest((U8 *)&Sta, rdu->body, rdu->bodyLen)) { //reject
                                                        asStatus = SC_UNSPEC_FAILURE;
                                                        goto check_failed;
                                                        //we need reason code here
                                                }
                                        }
                                } else {
                                        asStatus = SC_UNSPEC_FAILURE;
                                        goto wpa_check_failed;
                                }
                        }

                        //wpa_check_ok:
                        if (!UpdateStaStatus(&Sta, STATION_STATE_ASOC, vapId)) {
                                asStatus = SC_AP_FULL;
                        } else {
                                AssocInfoUpdate(&Sta, MaxRate, lsInterval, ZydasMode, Preamble, bErpSta, vapId);
                                aid = AIdLookup(&Sta);
                                printk(KERN_DEBUG "Re_Asoc: aid=%d\n", aid);
                                pHash = HashSearch(&Sta);
                                if (pHash != NULL) {
                                        if (!pHash->AlreadyIn) {
                                                pHash->AlreadyIn=1;
                                                mCurrConnUser++;
                                                if (bErpSta == FALSE) {
                                                        printk(KERN_DEBUG "Increment mNumBOnlySta\n");
                                                        mNumBOnlySta++;
                                                }
                                        }
                                }

                                asStatus = SC_SUCCESSFUL;
                                if (signal->id == SIG_REASSOC)
                                        notifyStatus1 = STA_REASSOCIATED;
                                pdot11Obj->StatusNotify(notifyStatus1, (U8 *)&Sta);

                                if (mMacMode != PURE_B_MODE && mMacMode != PURE_A_MODE) {
                                        if (pdot11Obj->ConfigFlag & NON_ERP_PRESENT_SET) {
                                                U32 tmpValue;
                                                void *reg = pdot11Obj->reg;

                                                // force enabled protection mode for debug
                                                mErp.buf[2] |= (NON_ERP_PRESENT | USE_PROTECTION);
                                                tmpValue = pdot11Obj->GetReg(reg, ZD_RTS_CTS_Rate);
                                                tmpValue &= ~CTS_MOD_TYPE_OFDM;
                                                tmpValue |= CTS_RATE_11M;
                                                pdot11Obj->SetReg(reg, ZD_RTS_CTS_Rate, tmpValue);

                                                if (Preamble == 0) { //long preamble
                                                        mErp.buf[2] |= BARKER_PREAMBLE;
                                                        tmpValue = pdot11Obj->GetReg(reg, ZD_RTS_CTS_Rate);
                                                        tmpValue &= ~NON_BARKER_PMB_SET;
                                                        tmpValue |= CTS_RATE_11M;
                                                        pdot11Obj->SetReg(reg, ZD_RTS_CTS_Rate, tmpValue);
                                                        //FPRINT("Enable Barker Preamble");
                                                }
                                                pdot11Obj->ConfigFlag |= ENABLE_PROTECTION_SET;
                                                //FPRINT("Enable Protection Mode");
                                        }
                                }

                        }
                } else {
                        //wpa_check_failed:
                        asStatus = SC_UNSPEC_FAILURE;
                }
wpa_check_failed:

                aid |= 0xC000;
                if (aid != 0xC000) {
                        FPRINT_V("Aid", aid);
                        FPRINT_V("MaxRate", MaxRate);
                }

check_failed:
                if (signal->id == SIG_REASSOC)
                        type = ST_REASOC_RSP;
                mkRe_AsocRspFrm(pfrmDesc, type, &Sta, mCap, asStatus, aid, &mBrates, pExtRate, vapId);
                sendMgtFrame(signal, pfrmDesc);

                return FALSE;
        } else {
                freeFdesc(pfrmDesc);
                return TRUE;
        }
}

BOOLEAN Re_AsocReq(Signal_t *signal)
{
        FrmDesc_t *pfrmDesc;
        U8 vapId = 0;
        TypeSubtype subType = ST_ASOC_REQ;
        Element *pExtRate = NULL;

        FPRINT("Re_AsocReq");
        if (memcmp(zeroMacAddress, (u8*)&mBssId, 6)==0) {
                sigEnque(pMgtQ, (signal));
                return FALSE;
        }

        pfrmDesc = allocFdesc();
        if(!pfrmDesc) {
                sigEnque(pMgtQ, (signal));
                return FALSE;
        }


        if (signal->id == SIG_REASSOC_REQ) {
                subType = ST_REASOC_REQ;
        }

        pExtRate = &mExtRates;

        memcpy((U8 *)&AsSta, (U8 *)&mBssId, 6);

        mkRe_AsocReqFrm(pfrmDesc, subType, &mBssId, mCap, mListenInterval,
                        &mOldAP, &mSsid, &mBrates, pExtRate, &mWPAIe, vapId);

        pdot11Obj->StartTimer(ASOC_TIMEOUT, DO_ASOC);
        AsocState = STE_WAIT_ASOC_RSP;
        return sendMgtFrame(signal, pfrmDesc);
}

BOOLEAN Re_AsocRsp(Signal_t *signal)
{
        Hash_t	*pHash;
        FrmDesc_t *pfrmDesc;
        Frame_t *rdu;
        U8 vapId = 0;
        MacAddr_t Sta;
        U16 status;

        //ZDEBUG("Re_AsocRsp");
        FPRINT("Re_AsocRsp");

        pfrmDesc = signal->frmInfo.frmDesc;
        rdu = pfrmDesc->mpdu;

        memcpy((U8 *)&Sta, (U8 *)addr2(rdu), 6);
        if (memcmp(&AsSta, &Sta, 6) != 0) {
                //FPRINT("Not for my Assoc AP");
                goto asoc_release;
        }

        pdot11Obj->StopTimer(DO_ASOC);
        AsocState = STE_ASOC_IDLE;


        status = status(rdu);
        if (status == SC_SUCCESSFUL) {
                U8	Len;
                U8	tmpMaxRate = 0x02;
                U8	ZydasMode = 0;
                U8	Preamble = 0;
                U8	MaxRate;
                BOOLEAN bErpSta = FALSE;
                int i;
                U8	HigestBasicRate = 0;

                UpdateStaStatus(&Sta, STATION_STATE_ASOC, vapId);
                pHash = HashSearch(&Sta);
                if (pHash != NULL) {
                        if (!pHash->AlreadyIn) {
                                pHash->AlreadyIn=1;
                                mCurrConnUser++;
                        }
                }

                mAPCap = cap(rdu);

                //check capability
                if (mAPCap & CAP_SHORT_PREAMBLE) {
                        Preamble = 1;
                } else {
                        Preamble = 0;
                }

                mAid = (aid(rdu) & 0x3FFF);
                getElem(rdu, EID_SUPRATES, &mAPBrates);
                //zd1205_dump_data("mAPBrates", &mAPBrates.buf[2], mAPBrates.buf[1]);

                if (getElem(rdu, EID_EXT_RATES, &mAPErates)) {
                        //zd1205_dump_data("mAPErates", &mAPErates.buf[2], mAPErates.buf[1]);
                }

                memcpy((U8 *)&mBssId, (U8 *)addr2(rdu), 6);
                mAssoc = TRUE;

                //update supported rates
                HW_SetSupportedRate(pdot11Obj, (U8 *)&mAPBrates);
                HW_SetSupportedRate(pdot11Obj, (U8 *)&mAPErates);

                Len = eLen(&mAPBrates);
                for (i=0; i<Len; i++) {
                        if ((mAPBrates.buf[2+i] & 0x7f) > tmpMaxRate ) {
                                tmpMaxRate = (mAPBrates.buf[2+i] & 0x7f);
                                if (mAPBrates.buf[2+i] & 0x80)
                                        HigestBasicRate = mAPBrates.buf[2+i];
                        }

                        if (((mAPBrates.buf[2+i] & 0x7f) == 0x21) && (!(mAPCap & CAP_PBCC_ENABLE))) { //Zydas 16.5M
                                ZydasMode = 1;
                        }
                }


                Len = eLen(&mAPErates);
                for (i=0; i<Len; i++) {
                        if ((mAPErates.buf[2+i] & 0x7f) > tmpMaxRate ) {
                                tmpMaxRate = (mAPErates.buf[2+i] & 0x7f);
                                if (mAPErates.buf[2+i] & 0x80)
                                        HigestBasicRate = mAPErates.buf[2+i];
                        }
                }

                //FPRINT_V("tmpMaxRate", tmpMaxRate);
                //FPRINT_V("mMaxTxRate", mMaxTxRate);
                MaxRate = RateConvert((tmpMaxRate & 0x7f));
                //FPRINT_V("MaxRate", MaxRate);
                if (MaxRate > mMaxTxRate)
                        MaxRate = mMaxTxRate;
                memset(mAPBrates.buf, 0, sizeof(Element));
                memset(mAPErates.buf, 0, sizeof(Element));
                AssocInfoUpdate(&mBssId, MaxRate, mAid, ZydasMode, Preamble, bErpSta, vapId);

                pdot11Obj->StatusNotify(STA_ASSOCIATED, (U8 *)&mBssId);
                pdot11Obj->Aid = mAid;
                mConnRetryCnt = 0;

                FPRINT_V("Aid", mAid);
                FPRINT_V("MaxRate", MaxRate);
        } else {
                FPRINT("Asoc Failed!!!");
                FPRINT_V("Status Code", status);
        }

asoc_release:
        freeFdesc(pfrmDesc);
        return TRUE;
}


BOOLEAN AsocTimeOut(Signal_t *signal)
{
        U8 vapId = 0;
        FPRINT("AsocTimeOut");

        if (AsocState == STE_WAIT_ASOC_RSP) {
                AsocState = STE_ASOC_IDLE;
                UpdateStaStatus(&mBssId, STATION_STATE_DIS_ASOC, vapId);
        }

        mRequestFlag |= CONNECT_TOUT_SET;
        return FALSE;
}


BOOLEAN AsocEntry(Signal_t *signal)
{
        FrmDesc_t *pfrmDesc;

        if (AsocState == STE_ASOC_IDLE) {
                switch(signal->id) {
                case SIG_DIASSOC_REQ:
                        return DisasocReq(signal);

                case SIG_DISASSOC:
                        return Disasoc(signal);

                case SIG_ASSOC:
                case SIG_REASSOC:
                        return Re_Asociate(signal);

                case SIG_ASSOC_REQ:
                case SIG_REASSOC_REQ:
                        return Re_AsocReq(signal);

                default:
                        goto asoc_discard;
                }
        } else if (AsocState == STE_WAIT_ASOC_RSP) {
                switch(signal->id) {
                case SIG_DIASSOC_REQ:
                        return DisasocReq(signal);

                case SIG_DISASSOC:
                        return Disasoc(signal);

                case SIG_ASSOC:
                case SIG_REASSOC:
                        return Re_Asociate(signal);

                case SIG_ASSOC_REQ:
                case SIG_REASSOC_REQ:
                        return Re_AsocReq(signal);

                case SIG_ASSOC_RSP:
                case SIG_REASSOC_RSP:
                        return Re_AsocRsp(signal);

                case SIG_TO_ASOC:

                        return AsocTimeOut(signal);

                default:
                        goto asoc_discard;
                }
        } else
                goto asoc_discard;

asoc_discard:
        pfrmDesc = signal->frmInfo.frmDesc;
        freeFdesc(pfrmDesc);
        return TRUE;
}

#endif
