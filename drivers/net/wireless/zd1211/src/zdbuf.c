#ifndef __ZDBUF_C__
#define __ZDBUF_C__

#include "zd80211.h"
#include <linux/vmalloc.h>


#define MAX_SIGNAL_NUM		64

SignalQ_t mgtQ, txQ, awakeQ, psQ[MAX_RECORD];
SignalQ_t *pMgtQ = &mgtQ, *pTxQ, *pAwakeQ, *pPsQ[MAX_RECORD];

Signal_t *FreeSignalList;
Signal_t *SignalBuf[MAX_SIGNAL_NUM];
U32 freeSignalCount;

FrmDesc_t *FreeFdescList;
FrmDesc_t *FdescBuf[MAX_SIGNAL_NUM];
U32 freeFdescCount = MAX_SIGNAL_NUM;

U32 allocCnt = 0;
U32 freeCnt = 0;

void initSigQue(SignalQ_t *Q)
{
        U32 flags;

        flags = pdot11Obj->EnterCS();
        Q->first = NULL;
        Q->last = NULL;
        Q->cnt = 0;
        pdot11Obj->ExitCS(flags);
}

void releaseSignalBuf(void)
{
        int i;

       /* ath_desc: workaround for detecting device multiple times */
        for (i=0; i<MAX_SIGNAL_NUM; i++) {
                vfree((void *)SignalBuf[i]);
                SignalBuf[i] = NULL;
       }

        /*  Point the the FreeSignalList to NULL */
        FreeSignalList = NULL;
}

void initSignalBuf(void)
{
        int i;
        U32 flags;

        initSigQue(pMgtQ);
        FreeSignalList = NULL;
        freeSignalCount = MAX_SIGNAL_NUM;

        for (i=0; i<MAX_SIGNAL_NUM; i++) {
                SignalBuf[i] = (Signal_t *)vmalloc(sizeof(Signal_t));  //can't use for DMA operation

                if (!SignalBuf[i]) {
                        FPRINT("80211: initSignalBuf failed");
                        //pdot11Obj->ExitCS(flags);
                        return;
                }
                flags = pdot11Obj->EnterCS();
                SignalBuf[i]->pNext = FreeSignalList;
                FreeSignalList = SignalBuf[i];
                pdot11Obj->ExitCS(flags);
        }
}


Signal_t *allocSignal(void)
{
        U32 flags;
        Signal_t *signal = NULL;

        flags = pdot11Obj->EnterCS();
        if (FreeSignalList != NULL) {
                signal = FreeSignalList;
                FreeSignalList = FreeSignalList->pNext;
                signal->pNext = NULL;
                signal->buf = NULL;
                signal->frmInfo.frmDesc = NULL;
                freeSignalCount-- ;
        }

        pdot11Obj->ExitCS(flags);
        return signal;
}

void freeSignal(Signal_t *signal)
{
        U32 flags;

        if (!signal) {
                FPRINT("Free NULL signal");
                return;
        }

        flags = pdot11Obj->EnterCS();
        signal->buf = NULL;
        signal->vapId=0;

        signal->frmInfo.frmDesc = NULL;
        signal->pNext = FreeSignalList;
        FreeSignalList = signal;
        freeSignalCount++;
        pdot11Obj->ExitCS(flags);
}

void re_initFdescBuf(void)
{
        int i;
        for (i=0; i<MAX_SIGNAL_NUM; i++)
                freeFdesc(FdescBuf[i]);
        freeFdescCount = MAX_SIGNAL_NUM;

}

void initFdescBuf(void)
{
        int i;
        U32 flags;
        //FrmDesc_t *pFrmDesc;

        flags = pdot11Obj->EnterCS();
        FreeFdescList = NULL;

        for (i=0; i<MAX_SIGNAL_NUM; i++) {
                FdescBuf[i] = (FrmDesc_t *) kmalloc(sizeof (FrmDesc_t), GFP_ATOMIC);  //may use for DMA operation

                if (!FdescBuf[i]) {
                        FPRINT("80211: initFdescBuf failed");
                        pdot11Obj->ExitCS(flags);
                        return;
                }

                FdescBuf[i]->pNext = FreeFdescList;
                FreeFdescList = FdescBuf[i];
        }

#if 0
        FPRINT_V("FreeFdescList", FreeFdescList);
        pFrmDesc =  FreeFdescList;
        for (i=0; i<MAX_SIGNAL_NUM; i++) {
                FPRINT_V("pFrmDesc", pFrmDesc);
                FPRINT_V("pFrmDesc->pNext", pFrmDesc->pNext);
                pFrmDesc = pFrmDesc->pNext;
        }
#endif

        pdot11Obj->ExitCS(flags);
}


void releaseFdescBuf(void)
{
        int i;

        for (i=0; i<MAX_SIGNAL_NUM; i++)
                kfree((void *)FdescBuf[i]);

        /*  Point the the FreeFdescList to NULL */
        FreeFdescList = NULL;
}


FrmDesc_t *allocFdesc(void)
{
        U32 flags;
        FrmDesc_t *pfrmDesc = NULL;

        flags = pdot11Obj->EnterCS();

        if (FreeFdescList != NULL) {
                pfrmDesc = FreeFdescList;
                FreeFdescList = FreeFdescList->pNext;
                //memset(pfrmDesc,0,sizeof(FrmDesc_t));
                //memset(pfrmDesc->mpdu,0,sizeof(Frame_t)*MAX_FRAG_NUM);
                pfrmDesc->pNext = NULL;
                pfrmDesc->ConfigSet = 0;
                //pfrmDesc->bDataFrm = 0;
                pfrmDesc->pHash = NULL;
                pfrmDesc->bValid = TRUE;
                freeFdescCount--;
                allocCnt++;

                if (FreeFdescList == NULL) {
                        FPRINT("FreeFdescList == NULL");
                        FPRINT_V("freeFdescCount", (U32)freeFdescCount);
                        FPRINT_V("Cnt of MgtQ", pMgtQ->cnt);
                }
        }

        pdot11Obj->ExitCS(flags);
        //FPRINT_V("alloc pfrmDesc", (U32)pfrmDesc);
        //FPRINT_V("FreeFdescList", (U32)FreeFdescList);

        return pfrmDesc;
}


void freeFdesc(FrmDesc_t *pfrmDesc)
{
        U32 flags;
        FrmDesc_t *pOldFdesc;

        if (!pfrmDesc) {
                FPRINT("Free NULL pfrmDesc");
                return;
        }

        flags = pdot11Obj->EnterCS();
#if 0

        if (!pfrmDesc->bValid) {
                FPRINT_V("pfrmDesc->bValid", pfrmDesc->bValid);
                pdot11Obj->ExitCS(flags);
                return;
        }
#endif

        //FPRINT_V("FreeFdescList", (U32)FreeFdescList);
        pOldFdesc =  FreeFdescList;
        pfrmDesc->ConfigSet = 0;
        //pfrmDesc->bDataFrm = 0;
        pfrmDesc->pHash = NULL;
        pfrmDesc->pNext = FreeFdescList;
        FreeFdescList = pfrmDesc;
        pfrmDesc->bValid = FALSE;
        freeFdescCount++;
        freeCnt++;
        //FPRINT_V("free pfrmDesc", (U32)pfrmDesc);

#if 0

        if (FreeFdescList == 0) {
                FPRINT("xxxxxxxxxx");
                FPRINT_V("free pfrmDesc", (U32)pfrmDesc);
                FPRINT_V("freeFdescCount", (U32)freeFdescCount);
        }

        if (pfrmDesc == pfrmDesc->pNext) {
                FPRINT("ooooooooooo");
                FPRINT_V("pOldFdesc", (U32)pOldFdesc);
                FPRINT_V("free pfrmDesc", (U32)pfrmDesc);
                FPRINT_V("freeFdescCount", (U32)freeFdescCount);
        }
#endif
        pdot11Obj->ExitCS(flags);
}


Signal_t *sigDeque(SignalQ_t *Q)
{
        U32 flags;
        Signal_t *signal = NULL;

        flags = pdot11Obj->EnterCS();
        if (Q->first != NULL) {
                Q->cnt--;
                signal = Q->first;
                Q->first = (Q->first)->pNext;
                if (Q->first == NULL)
                        Q->last = NULL;
        }
        pdot11Obj->ExitCS(flags);
        return signal;
}


void sigEnque(SignalQ_t *Q, Signal_t *signal)
{
        U32 flags;

        flags = pdot11Obj->EnterCS();
        signal->pNext = NULL;
        if (Q->last == NULL) {
                Q->first = signal;
                Q->last = signal;
        } else {
                Q->last->pNext = signal;
                Q->last = signal;
        }
        Q->cnt++;

        if (Q == pMgtQ)
                pdot11Obj->QueueFlag |= MGT_QUEUE_SET;
        else if (Q == pTxQ)
                pdot11Obj->QueueFlag |= TX_QUEUE_SET;
        else if (Q == pAwakeQ)
                pdot11Obj->QueueFlag |= AWAKE_QUEUE_SET;

        pdot11Obj->ExitCS(flags);
}


void sigEnqueFirst(SignalQ_t *Q, Signal_t *signal)
{
        int i_state;

        FPRINT("sigEnqueFirst");

        i_state = pdot11Obj->EnterCS();
        signal->pNext = Q->first;

        if (Q->last == NULL) {
                Q->last = signal;
        }

        Q->first = signal;
        Q->cnt++;

        pdot11Obj->ExitCS(i_state);
}

#endif
