#ifndef _ZDPMFILTER_H
#define _ZDPMFILTER_H

extern Signal_t *txRequest[BURST_NUM];

void TxCompleted(U32 result, U8 retID, U16 aid);
void StaWakeup(MacAddr_t *sta);
void ConfigBcnFIFO(void);
void SendMcPkt(void);
void ResetPMFilter(void);
void PsPolled(MacAddr_t *sta, U16 aid);
void InitPMFilterQ(void);
void FlushQ(SignalQ_t *Q);
BOOLEAN CleanupAwakeQ(void);
BOOLEAN CleanupTxQ(void);
BOOLEAN SendPkt(Signal_t* signal, FrmDesc_t *pfrmDesc, BOOLEAN bImmediate);
#endif
