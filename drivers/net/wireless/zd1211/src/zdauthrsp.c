#ifndef __ZDAUTHRSP_C__
#define __ZDAUTHRSP_C__

#include "zd80211.h"
#include "zd1205.h"

U8 AuthRspState = STE_AUTH_RSP_IDLE;
static U8 arChalng[CHAL_TEXT_LEN]; 	//Challenge text
static MacAddr_t Sta2;
static MacAddr_t Sta;
extern struct net_device *g_dev;

BOOLEAN CheckAlg(U8 alg)
{
        int i = 0;

        for (i=0; i<3; i++) {
                if (i == 2)
                        return FALSE;

                if (mAuthAlogrithms[i] == alg)
                        return TRUE;
        }
        return FALSE;
}


BOOLEAN AuthOdd_Idle(Signal_t *signal)
{
        struct zd1205_private *macp=g_dev->priv;
        FrmDesc_t *pfrmDesc;
        Frame_t *rdu;
        U16 arAlg;
        U16	arSeq;
        U16	arSC;
        U8 vapId = 0;

        ZDEBUG("AuthOdd_Idle");
        pfrmDesc = signal->frmInfo.frmDesc;
        rdu = pfrmDesc->mpdu;
        arSeq = authSeqNum(rdu);
        arAlg = authType(rdu);
        memcpy((U8 *)&Sta, (U8 *)addr2(rdu), 6);

        if (arSeq != 1) {
                arSC = SC_AUTH_OUT_OF_SEQ;
                UpdateStaStatus(&Sta, STATION_STATE_NOT_AUTH, vapId);
                mkAuthFrm(pfrmDesc, &Sta, arAlg, arSeq+1, arSC, NULL, vapId);
                return sendMgtFrame(signal, pfrmDesc);
        }

        if (!CheckAlg(arAlg)) {
                arSC = SC_UNSUPT_ALG;
                UpdateStaStatus(&Sta, STATION_STATE_NOT_AUTH, vapId);
                mkAuthFrm(pfrmDesc, &Sta, arAlg, arSeq+1, arSC, NULL, vapId);
                return sendMgtFrame(signal, pfrmDesc);
        }

        if (pdot11Obj->StatusNotify(STA_AUTH_REQ, (U8 *)&Sta)) { //Reject it
                arSC = SC_UNSPEC_FAILURE;
                UpdateStaStatus(&Sta, STATION_STATE_NOT_AUTH, vapId);
                mkAuthFrm(pfrmDesc, &Sta, arAlg, arSeq+1, arSC, NULL, vapId);
                return sendMgtFrame(signal, pfrmDesc);
        }

        if (arAlg == OPEN_SYSTEM) {
                if (UpdateStaStatus(&Sta, STATION_STATE_AUTH_OPEN, vapId))
                        arSC = SC_SUCCESSFUL;
                else
                        arSC = SC_AP_FULL;

                mkAuthFrm(pfrmDesc, &Sta, arAlg, arSeq+1, arSC, NULL, vapId);
                return sendMgtFrame(signal, pfrmDesc);
        }

        if (arAlg == SHARE_KEY) {
                //WPA
                if (macp->cardSetting.WPAIeLen) {
                        //if ((mDynKeyMode == DYN_KEY_TKIP) || (mDynKeyMode == DYN_KEY_AES)){
                        arSC = SC_UNSUPT_ALG;
                        UpdateStaStatus(&Sta, STATION_STATE_NOT_AUTH, vapId);
                        mkAuthFrm(pfrmDesc, &Sta, arAlg, arSeq+1, arSC, NULL, vapId);

                        return sendMgtFrame(signal, pfrmDesc);
                } else {
                        if ((mCurrConnUser + 1) > mLimitedUser)
                                arSC = SC_AP_FULL;
                        else
                                arSC = SC_SUCCESSFUL;

                        mkAuthFrm(pfrmDesc, &Sta, arAlg, arSeq+1, arSC, arChalng, vapId);
                        if (arSC == SC_SUCCESSFUL) {
                                pdot11Obj->StartTimer(512, DO_CHAL);
                                AuthRspState = STE_AUTH_RSP_WAIT_CRSP;
                        }
                        return sendMgtFrame(signal, pfrmDesc);
                }
        }

        freeFdesc(pfrmDesc);
        return TRUE;
}


BOOLEAN AuthOdd_WaitChalRsp(Signal_t *signal)

{
        FrmDesc_t *pfrmDesc;
        Frame_t *rdu;
        U8 arAlg2;
        U16	arSeq2;
        U16	arSC;
        U8 vapId = 0;
        U8 ChalText[130];

        ZDEBUG("AuthOdd_WaitChalRsp");
        pfrmDesc = signal->frmInfo.frmDesc;
        rdu = pfrmDesc->mpdu;

        arSeq2	= authSeqNum(rdu);
        arAlg2	= authType(rdu);
        memcpy((U8 *)&Sta2, (U8 *)addr2(rdu), 6);

        if(arSeq2 == 1) {
                if (memcmp(&Sta, &Sta2, 6) == 0) { // open system request from a different station
                        arSC = SC_UNSPEC_FAILURE;
                        goto fail_sta2;
                }

                if (arAlg2 == OPEN_SYSTEM) {
                        arSC = SC_SUCCESSFUL;
                        UpdateStaStatus(&Sta, STATION_STATE_AUTH_OPEN, vapId);
                        mkAuthFrm(pfrmDesc, &Sta2, arAlg2, arSeq2+1, arSC, NULL, vapId);

                        return sendMgtFrame(signal, pfrmDesc);
                } else {
                        arSC = SC_UNSPEC_FAILURE;
                        goto fail_sta2;
                }
        } else if(arSeq2 == 3) {
                if (memcmp(&Sta, &Sta2, 6) == 0) {
                        pdot11Obj->StopTimer(DO_CHAL);
                        if (wepBit(rdu)) {
                                if (!getElem(rdu, EID_CTEXT, (Element *)&ChalText[0])) {
                                        goto chal_failed;
                                } else {
                                        //zd1205_OctetDump("ChalText = ", &ChalText[0], CHAL_TEXT_LEN+2);
                                        if (memcmp(&ChalText[2], arChalng, CHAL_TEXT_LEN) != 0) {
                                                goto chal_failed;
                                        }
                                }

                                if (UpdateStaStatus(&Sta, STATION_STATE_AUTH_KEY, vapId))
                                        arSC = SC_SUCCESSFUL;
                                else
                                        arSC = SC_AP_FULL;
                        } else {
chal_failed:
                                arSC = SC_CHAL_FAIL;
                                UpdateStaStatus(&Sta2, STATION_STATE_NOT_AUTH, vapId);
                        }
                } else {
                        arSC = SC_UNSPEC_FAILURE;
                        UpdateStaStatus(&Sta2, STATION_STATE_NOT_AUTH, vapId);
                }
                AuthRspState = STE_AUTH_RSP_IDLE;
                mkAuthFrm(pfrmDesc, &Sta2, arAlg2, arSeq2+1, arSC, NULL, vapId);
                return sendMgtFrame(signal, pfrmDesc);
        } else {
                arSC = SC_UNSPEC_FAILURE;
                goto fail_sta2;
        }

fail_sta2:
        UpdateStaStatus(&Sta2, STATION_STATE_NOT_AUTH, vapId);
        mkAuthFrm(pfrmDesc, &Sta2, arAlg2, arSeq2+1, arSC, NULL, vapId);
        return sendMgtFrame(signal, pfrmDesc);
}


BOOLEAN Tchal_WaitChalRsp(Signal_t *signal)
{
        U8 vapId = 0;

        ZDEBUG("Tchal_WaitChalRsp");
        if 	(AuthRspState == STE_AUTH_RSP_WAIT_CRSP) {
                UpdateStaStatus(&Sta, STATION_STATE_NOT_AUTH, vapId);
                AuthRspState = STE_AUTH_RSP_IDLE;
        }
        return FALSE;
}


BOOLEAN Deauth(Signal_t *signal)
{
        FrmDesc_t *pfrmDesc;
        Frame_t *rdu;
        U8 vapId = 0;

        ZDEBUG("Deauth");
        pfrmDesc = signal->frmInfo.frmDesc;
        rdu = pfrmDesc->mpdu;
        UpdateStaStatus(addr2(rdu), STATION_STATE_NOT_AUTH, vapId);
        if (memcmp(&mBssId, addr2(rdu), 6) == 0)
                pdot11Obj->StatusNotify(STA_DEAUTHED, (U8 *)addr2(rdu));

        if (mBssType == INFRASTRUCTURE_BSS) {
                //if (memcmp(&mBssId, addr2(rdu), 6) == 0){
                FPRINT("Deauth");
                mAssoc = FALSE;
                memset((U8 *)&mBssId, 0, 6);
                mRequestFlag |= DIS_CONNECT_SET;
                //}
        }

        //here to handle deauth ind.
        freeFdesc(pfrmDesc);
        return TRUE;
}


BOOLEAN AuthRspEntry(Signal_t *signal)
{
        if (AuthRspState == STE_AUTH_RSP_IDLE) {
                switch(signal->id) {
                case SIG_AUTH_ODD:
                        return AuthOdd_Idle(signal);

                case SIG_DEAUTH:
                        return Deauth(signal);

                default:
                        return TRUE;
                }
        }
        else if (AuthRspState == STE_AUTH_RSP_WAIT_CRSP) {
                switch(signal->id) {
                case SIG_AUTH_ODD:
                        return AuthOdd_WaitChalRsp(signal);

                case SIG_DEAUTH:
                        return Deauth(signal);

                case SIG_TO_CHAL:
                        return Tchal_WaitChalRsp(signal);

                default:
                        return TRUE;
                }
        } else
                return TRUE;
}

#endif
