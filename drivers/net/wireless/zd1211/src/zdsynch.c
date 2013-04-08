#ifndef __ZDSYNCH_C__
#define __ZDSYNCH_C__

#include "zd80211.h"
extern u8 LastMacMode;
extern u8 mMacMode;
//extern CurrScanCH;
extern const U16 dot11A_Channel[];
extern u8 LastSetChannel;

void zd_makeRateInfoMAP(U8 *pRates, U16 *basicRateMap, U16 *supRateMap)
{
        int j;
        U8 rate;
        U8 eleLen = *(pRates+1);

        for (j=0; j<eleLen; j++) {
                rate =  *(pRates+2+j);
                switch (rate & 0x7f) {
                case SR_1M:
                        *supRateMap |= BIT_0;
                        if (rate & 0x80)
                                *basicRateMap |= BIT_0;
                        break;

                case SR_2M:
                        *supRateMap |= BIT_1;
                        if (rate & 0x80)
                                *basicRateMap |= BIT_1;
                        break;

                case SR_5_5M:
                        *supRateMap |= BIT_2;
                        if (rate & 0x80)
                                *basicRateMap |= BIT_2;
                        break;

                case SR_11M:
                        *supRateMap |= BIT_3;
                        if (rate & 0x80)
                                *basicRateMap |= BIT_3;
                        break;

                case SR_6M:
                        *supRateMap |= BIT_4;
                        if (rate & 0x80)
                                *basicRateMap |= BIT_4;
                        break;

                case SR_9M:
                        *supRateMap |= BIT_5;
                        if (rate & 0x80)
                                *basicRateMap |= BIT_5;
                        break;

                case SR_12M:
                        *supRateMap |= BIT_6;
                        if (rate & 0x80)
                                *basicRateMap |= BIT_6;
                        break;

                case SR_18M:
                        *supRateMap |= BIT_7;
                        if (rate & 0x80)
                                *basicRateMap |= BIT_7;
                        break;

                case SR_24M:
                        *supRateMap |= BIT_8;
                        if (rate & 0x80)
                                *basicRateMap |= BIT_8;
                        break;

                case SR_36M:
                        *supRateMap |= BIT_9;
                        if (rate & 0x80)
                                *basicRateMap |= BIT_9;
                        break;

                case SR_48M:
                        *supRateMap |= BIT_10;
                        if (rate & 0x80)
                                *basicRateMap |= BIT_10;
                        break;

                case SR_54M:
                        *supRateMap |= BIT_11;
                        if (rate & 0x80)
                                *basicRateMap |= BIT_11;
                        break;

#if 0

                case SR_16_5M:
                        *supRateMap |= BIT_12;
                        if (rate & 0x80)
                                *basicRateMap |= BIT_12;
                        break;

                case SR_27_5M:
                        *supRateMap |= BIT_13;
                        if (rate & 0x80)
                                *basicRateMap |= BIT_13;
                        break;
#endif

                default:

                        break;
                }
        }
}

BOOLEAN Probe(Signal_t *signal)
{
        FrmDesc_t *pfrmDesc;
        Frame_t *rdu;
        MacAddr_t sta;
        Element rSsid;
        Element *pWPA = NULL;
        U8 vapId = 0;
        Element *pExtRate = NULL;

        ZDEBUG("Probe");

        pfrmDesc = signal->frmInfo.frmDesc;
        rdu = pfrmDesc->mpdu;

        if (mBssType == INFRASTRUCTURE_BSS) {
                goto release;
        }

        if (!getElem(rdu, EID_SSID, &rSsid))
                goto release;

        if (mHiddenSSID) { //discard broadcast ssid
                if (eLen(&rSsid) == 0) {
                        goto release;
                }
        }

        memcpy((U8*)&sta, (U8*)addr2(rdu), 6);
        pExtRate = &mExtRates;

        if (eLen(&rSsid) == 0) {
                //WPA
                if (mDynKeyMode == DYN_KEY_TKIP || mDynKeyMode==DYN_KEY_AES)
                        pWPA = &mWPAIe;

                mkProbeRspFrm(pfrmDesc, &sta, mBeaconPeriod, mCap, &dot11DesiredSsid, &mBrates, &mPhpm, pExtRate, (Element *)pWPA, vapId);
                return sendMgtFrame(signal, pfrmDesc);
        } else {
                if (memcmp(&rSsid, &dot11DesiredSsid, eLen(&dot11DesiredSsid)+2) == 0) {
                        //WPA
                        if ((mDynKeyMode == DYN_KEY_TKIP) || (mDynKeyMode == DYN_KEY_AES))
                                pWPA = &mWPAIe;

                        mkProbeRspFrm(pfrmDesc, &sta, mBeaconPeriod, mCap, &dot11DesiredSsid, &mBrates, &mPhpm, pExtRate, (Element *)pWPA, vapId);
                        return sendMgtFrame(signal, pfrmDesc);
                }
        }

release:
        ZDEBUG("goto release");
        freeFdesc(pfrmDesc);
        return TRUE;
}

BOOLEAN ProbeReq(Signal_t *signal)
{
        FrmDesc_t *pfrmDesc;
        Element	BCSsid;
        U8 vapId = 0;
        Element *pSsid = NULL;
        Element *pExtRate = NULL;

        ZDEBUG("ProbeReq");
        //FPRINT("ProbeReq");

        pfrmDesc = allocFdesc();
        if(!pfrmDesc) {
                sigEnque(pMgtQ, (signal));
                return FALSE;
        }

        BCSsid.buf[0] = EID_SSID;
        BCSsid.buf[1] = 0;

        if (mProbeWithSsid) {
                //pSsid = &mSsid;
                pSsid = &dot11DesiredSsid;
        } else {
                pSsid = &BCSsid;
        }

        pExtRate = &mExtRates;

        mkProbeReqFrm(pfrmDesc, &dot11BCAddress, pSsid, &mBrates, pExtRate, NULL, vapId);
        if (signal->vapId == 0)
                pdot11Obj->StartTimer(SCAN_TIMEOUT, DO_SCAN);
        //mProbeWithSsid = FALSE;
        return sendMgtFrame(signal, pfrmDesc);
}

BOOLEAN ProbeRsp_Beacon(Signal_t *signal)
{
        FrmDesc_t *pfrmDesc;
        Frame_t *rdu;
        MacAddr_t *pBssid = NULL;
        //Element *pWPA = NULL;
        U16 Cap = 0;
        U16 BcnInterval = 0;
        int i;
        U8 FrmType;
        BOOLEAN bUpdateInfo = FALSE;
        U8 index;

        BssInfo_t *pCurrBssInfo;

        ZDEBUG("ProbeRsp_Beacon");
        pfrmDesc = signal->frmInfo.frmDesc;
        rdu = pfrmDesc->mpdu;

        FrmType = frmType(rdu);

        if (pdot11Obj->ConfigFlag & ACTIVE_CHANNEL_SCAN_SET) {
                Cap = cap1(rdu);
                //FPRINT_V("Cap", Cap);
                BcnInterval = beaconInt(rdu);

                if (Cap & CAP_IBSS) {
                        pBssid = addr3(rdu);
                } else
                        pBssid = addr2(rdu);

                index = mBssNum;
                //The following if-statements is used to filter existing AP Info.
                //The rule is:
                //1. The bssid is seen before.
                //2. The old Info.apMode equals to LastMacMode
                //The meaning is if the incoming ap's bssid == old's bssid and
                // ap's apMode(judged by LastMacMode) == old's apMode then Drop It.
                for (i=0; i<mBssNum; i++) {
                        if ((memcmp((U8 *)&mBssInfo[i].bssid, (U8 *)pBssid, 6) == 0) &&
                                        ((PURE_A_AP==mBssInfo[i].apMode&& PURE_A_MODE==LastMacMode) ||
                                         (PURE_A_AP!=mBssInfo[i].apMode&& PURE_A_MODE!=LastMacMode)
                                        )
                           ) {
                                if (FrmType == ST_BEACON) {
                                        goto release;
                                } else {
                                        bUpdateInfo = TRUE;
                                        index = i;
                                }
                        }
                }

                pCurrBssInfo =  &mBssInfo[index];
                pCurrBssInfo->basicRateMap = 0;
                pCurrBssInfo->supRateMap = 0;

                /* Reset supRates, extRates */
                memset(&pCurrBssInfo->supRates, 0, NUM_SUPPORTED_RATE);
                memset(&pCurrBssInfo->extRates, 0, NUM_SUPPORTED_RATE);

                //get bssid
                memcpy((U8 *)&pCurrBssInfo->bssid, (U8 *)pBssid, 6);

                //get beacon interval
                pCurrBssInfo->bcnInterval = BcnInterval;

                //get capability
                pCurrBssInfo->cap = Cap;

                if (!getElem(rdu, EID_SSID, &pCurrBssInfo->ssid)) {
                        goto release;
                }

                if (!getElem(rdu, EID_SUPRATES, &pCurrBssInfo->supRates)) {
                        goto release;
                }

                if (!getElem(rdu, EID_DSPARMS, &pCurrBssInfo->Phpm)) {
                        pCurrBssInfo->Phpm.buf[0]=0x3;// DS Parameter Set
                        pCurrBssInfo->Phpm.buf[1]=1;
                        pCurrBssInfo->Phpm.buf[2]=LastSetChannel;

                        //goto release;
                }
                //This is used to filter non-allowed channel beacons
                if (!((1 << (pCurrBssInfo->Phpm.buf[2]-1)) & pdot11Obj->AllowedChannel)) {
                        if(PURE_A_MODE != mMacMode)
                                goto release;
                }

                if (Cap & CAP_IBSS) {
                        if (!getElem(rdu, EID_IBPARMS, &pCurrBssInfo->IbssParms)) {
                                goto release;
                        }
                }

                if (getElem(rdu, EID_EXT_RATES, &pCurrBssInfo->extRates)) {
                        //zd1205_dump_data("Ext Rates", &pCurrBssInfo->extRates.buf[2], pCurrBssInfo->extRates.buf[1]);
                }

                if (getElem(rdu, EID_ERP, &pCurrBssInfo->erp)) {
                        //zd1205_dump_data("ERP Info", &pCurrBssInfo->erp.buf[2], pCurrBssInfo->erp.buf[1]);
                }
                getElem(rdu, EID_COUNTRY, &pCurrBssInfo->country);

                zd_makeRateInfoMAP((U8 *)&pCurrBssInfo->supRates, &pCurrBssInfo->basicRateMap, &pCurrBssInfo->supRateMap);
                zd_makeRateInfoMAP((U8 *)&pCurrBssInfo->extRates, &pCurrBssInfo->basicRateMap, &pCurrBssInfo->supRateMap);
                //FPRINT_V("basicRateMap", pCurrBssInfo->basicRateMap);
                //FPRINT_V("supRateMap", pCurrBssInfo->supRateMap);

                if (LastMacMode != PURE_A_MODE && pCurrBssInfo->supRateMap > 0x0f) {  //support rates include OFDM rates
                        if (pCurrBssInfo->basicRateMap & ~0xf) { // basic rates include OFDM rates
                                pCurrBssInfo->apMode = PURE_G_AP;
                                //FPRINT("PURE_G_AP");
                        } else {
                                pCurrBssInfo->apMode = MIXED_AP;
                                //FPRINT("MIXED_AP");
                        }
                } else if(LastMacMode == PURE_A_MODE) {
                        pCurrBssInfo->apMode = PURE_A_AP;
                } else {
                        pCurrBssInfo->apMode = PURE_B_AP;
                        //FPRINT("PURE_B_AP");
                }

                /* Get WPA IE Information */
                //getElem(rdu, EID_WPA, (Element *)&pCurrBssInfo->WPAIe);

                memset(&pCurrBssInfo->WPAIe,0x00, sizeof(pCurrBssInfo->WPAIe));
                getElem(rdu, EID_WPA, (Element*)&pCurrBssInfo->WPAIe);
                memset(&pCurrBssInfo->RSNIe,0x00, sizeof(pCurrBssInfo->RSNIe));
                getElem(rdu, EID_RSN, (Element*)&pCurrBssInfo->RSNIe);


#if 0
                /* Dump WPA IE */
                if(pCurrBssInfo->WPAIe[1] != 0) {
                        int ii;
                        u8 SSID[34+1];

                        memcpy(SSID, (u8 *)(&pCurrBssInfo->ssid.buf[2]), pCurrBssInfo->ssid.buf[1]);
                        SSID[pCurrBssInfo->ssid.buf[1]] = '\0';

                        printk(KERN_ERR "WPA IE found in site survey, SSID: %s\n", SSID);

                        for(ii = 0; ii < pCurrBssInfo->WPAIe[1]+2; ) {
                                printk(KERN_ERR "0x%02x ", pCurrBssInfo->WPAIe[ii]);
                                ii++;
                        }

                        printk(KERN_ERR "\n");
                }
#endif

                pCurrBssInfo->signalStrength = pfrmDesc->signalStrength;
                pCurrBssInfo->signalQuality = pfrmDesc->signalQuality;

                if (mBssNum < 63) {
                        if (!bUpdateInfo)
                                mBssNum++;
                }
        }

release:
        ZDEBUG("goto release");
        freeFdesc(pfrmDesc);
        return TRUE;
}


BOOLEAN SynchEntry(Signal_t *signal)
{
        FrmDesc_t *pfrmDesc;

        switch(signal->id) {
        case SIG_PROBE:
                return Probe(signal);

        case SIG_PROBE_REQ:
                return ProbeReq(signal);

        case SIQ_PROBE_RSP_BEACON:
                return ProbeRsp_Beacon(signal);

        default:
                goto sync_discard;
        }

sync_discard:
        pfrmDesc = signal->frmInfo.frmDesc;
        freeFdesc(pfrmDesc);
        return TRUE;
}
#endif
