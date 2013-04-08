#ifndef __ZDBUF_H__
#define __ZDBUF_H__


extern SignalQ_t mgtQ, *pMgtQ;
extern SignalQ_t txQ, *pTxQ;
extern SignalQ_t awakeQ, *pAwakeQ;
extern SignalQ_t psQ[MAX_RECORD], *pPsQ[MAX_RECORD];
extern U32 freeSignalCount;
extern U32 freeFdescCount;


void initSigQue(SignalQ_t *Q);
void sigEnque(SignalQ_t *Q, Signal_t *signal);
void sigEnqueFirst(SignalQ_t *Q, Signal_t *signal);
Signal_t *sigDeque(SignalQ_t* Q);
void initSignalBuf(void);
void initFdescBuf(void);
void freeFdesc(FrmDesc_t *pfrmDesc);
FrmDesc_t *allocFdesc(void);
extern Signal_t* allocSignal(void);
extern void freeSignal(Signal_t *signal);
void releaseSignalBuf(void);
void releaseFdescBuf(void);
void re_initFdescBuf(void);

#endif
