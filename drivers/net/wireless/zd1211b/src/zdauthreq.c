#ifndef __ZDAUTHREQ_C__
#define __ZDAUTHREQ_C__

#include "zd80211.h"

U8 AuthReqState = STE_AUTH_REQ_IDLE;
static MacAddr_t AuSta;


BOOLEAN DeauthReq(Signal_t *signal)
{
        FrmDesc_t *pfrmDesc;
        MacAddr_t Sta;
        ReasonCode Rsn = RC_UNSPEC_REASON;
        U8	vapId = 0;

        ZDEBUG("DeauthReq");

        memcpy((U8 *)&Sta, (U8 *)&signal->frmInfo.Sta, 6);
        Rsn = signal->frmInfo.rCode;
        pdot11Obj->StatusNotify(STA_DEAUTHED, (u8 *)&Sta);

        vapId = signal->vapId;
        UpdateStaStatus(&Sta, STATION_STATE_NOT_AUTH, vapId);

        pfrmDesc = allocFdesc();
        if(!pfrmDesc) {
                sigEnque(pMgtQ, (signal));
                return FALSE;
        }

        mkDisAssoc_DeAuthFrm(pfrmDesc, ST_DEAUTH, &Sta, Rsn, vapId);
        sendMgtFrame(signal, pfrmDesc);

        return FALSE;
}


BOOLEAN AuthReq(Signal_t *signal)
{
        FrmDesc_t *pfrmDesc;
        U8 vapId = 0;
        U16 auSC = SC_SUCCESSFUL;

        //ZDEBUG("AuthReq");
        FPRINT("AuthReq");
        pfrmDesc = allocFdesc();
        if(!pfrmDesc) {
                FPRINT("allocFdesc failed");
                sigEnque(pMgtQ, (signal));
                return FALSE;
        }

        memcpy((U8 *)&AuSta, (U8 *)&signal->frmInfo.Sta, 6);
        //memcpy((U8 *)&AuSta, (U8 *)&mBssId, 6);
        mkAuthFrm(pfrmDesc, &AuSta, mAuthAlg, 1, auSC, NULL, vapId);
        pdot11Obj->StartTimer(AUTH_TIMEOUT, DO_AUTH);
        AuthReqState = STE_WAIT_AUTH_SEQ2;
        return sendMgtFrame(signal, pfrmDesc);
}


BOOLEAN AuthEven(Signal_t *signal)
{
        FrmDesc_t *pfrmDesc;
        Frame_t *rdu;
        U16 auAlg;
        U16	auSeq;
        U16 auSC;
        U8 vapId = 0;
        MacAddr_t Sta;
        U8 ChalText[130];

        //ZDEBUG("AuthEven");
        FPRINT("AuthEven");
        pfrmDesc = signal->frmInfo.frmDesc;
        rdu = pfrmDesc->mpdu;
        auSeq = authSeqNum(rdu);
        auAlg = authType(rdu);
        memcpy((U8 *)&Sta, (U8 *)addr2(rdu), 6);
        if (memcmp(&AuSta, &Sta, 6) != 0) {
                //FPRINT("Not for my Auth AP");
                goto auth_release;
        }

        if ((AuthReqState == STE_WAIT_AUTH_SEQ2) && (auSeq != 2)) {
                FPRINT("Seq!= 2");
                goto auth_release;
        }

        if ((AuthReqState == STE_WAIT_AUTH_SEQ4) && (auSeq != 4)) {
                FPRINT("Seq!= 4");
                goto auth_release;
        }

        auSC = authStatus(rdu);
        if (auSC != SC_SUCCESSFUL) {
                FPRINT("Auth Failed!!!");
                FPRINT_V("Status Code", auSC);
                goto auth_release;
        }

        pdot11Obj->StopTimer(DO_AUTH);
        //FPRINT_V("auAlg", auAlg);

        if (AuthReqState == STE_WAIT_AUTH_SEQ4) {
                UpdateStaStatus(&Sta, STATION_STATE_AUTH_KEY, vapId);
                AuthReqState = STE_AUTH_REQ_IDLE;

                //do Assoicate
                signal->id = SIG_ASSOC_REQ;
                signal->block = BLOCK_ASOC;
                sigEnque(pMgtQ, (signal));
                freeFdesc(pfrmDesc);
                return FALSE;

        } else if (AuthReqState == STE_WAIT_AUTH_SEQ2) {
                if (auAlg == OPEN_SYSTEM) {
                        FPRINT("Auth Success !!!");
                        UpdateStaStatus(&Sta, STATION_STATE_AUTH_OPEN, vapId);
                        AuthReqState = STE_AUTH_REQ_IDLE;

                        //do Assoicate
                        signal->id = SIG_ASSOC_REQ;
                        signal->block = BLOCK_ASOC;
                        sigEnque(pMgtQ, (signal));
                        freeFdesc(pfrmDesc);
                        return FALSE;
                }

                if (auAlg == SHARE_KEY) {
                        if (!getElem(rdu, EID_CTEXT, (Element *)&ChalText[0])) {
                                FPRINT("Get ChalText failed");
                                goto auth_release;
                        }

                        FPRINT("Send Auth Seq 3 frame");
                        auSC = SC_SUCCESSFUL;
                        mkAuthFrm(pfrmDesc, &mBssId, mAuthAlg, 3, auSC, &ChalText[2], vapId);
                        pdot11Obj->StartTimer(AUTH_TIMEOUT, DO_AUTH);
                        AuthReqState = STE_WAIT_AUTH_SEQ4;
                        pfrmDesc->ConfigSet |= FORCE_WEP_SET;
                        return sendMgtFrame(signal, pfrmDesc);
                }
        }

auth_release:
        FPRINT("auth_release");
        freeFdesc(pfrmDesc);
        return TRUE;
}


BOOLEAN AuthTimeOut(Signal_t *signal)
{
        U8 vapId = 0;
        FPRINT("AuthTimeOut");

        if 	((AuthReqState == STE_WAIT_AUTH_SEQ2) ||


                        (AuthReqState == STE_WAIT_AUTH_SEQ4)) {
                UpdateStaStatus(&mBssId, STATION_STATE_NOT_AUTH, vapId);
                AuthReqState = STE_AUTH_REQ_IDLE;
        }

        mRequestFlag |= CONNECT_TOUT_SET;
        return FALSE;
}


BOOLEAN AuthReqEntry(Signal_t *signal)
{
        FrmDesc_t *pfrmDesc;

        if (AuthReqState == STE_AUTH_REQ_IDLE) {
                switch(signal->id) {
                case SIG_DEAUTH_REQ:
                        return DeauthReq(signal);

                case SIG_AUTH_REQ:
                        return AuthReq(signal);

                default:
                        goto auth_discard;
                }
        } else if ((AuthReqState == STE_WAIT_AUTH_SEQ2) || (AuthReqState == STE_WAIT_AUTH_SEQ4)) {
                switch(signal->id) {
                case SIG_DEAUTH_REQ:
                        return TRUE;

                case SIG_AUTH_REQ:
                        return TRUE;

                case SIG_AUTH_EVEN:
                        return AuthEven(signal);

                case SIG_TO_AUTH:
                        return AuthTimeOut(signal);

                default:
                        goto auth_discard;
                }
        } else
                goto auth_discard;

auth_discard:
        pfrmDesc = signal->frmInfo.frmDesc;
        freeFdesc(pfrmDesc);
        return TRUE;
}
#endif
