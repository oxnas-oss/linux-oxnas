#ifndef _ZDPSMON_H_
#define _ZDPSMON_H_


void ResetPSMonitor(void);
void SsInquiry(MacAddr_t *sta, StationState *sst, StationState *asst);
void AssocInfoUpdate(MacAddr_t *sta, U8 MaxRate, U8 lsInterval, U8 ZydasMode, U8 Preamble, BOOLEAN bErpSta, U8 vapId);
U16 AIdLookup(MacAddr_t *sta);
BOOLEAN AgeHashTbl(void);
BOOLEAN UpdateStaStatus(MacAddr_t *sta, StationState staSte,  U8 vapId);
int zd_SetKeyInfo(U8 *addr, U8 encryMode, U8 keyLength, U8 KeyId, U8 *pKeyContent);
int zd_SetKeyContext(U8 *addr, U8 encryMode, U8 keyLength, U8 KeyId, U8 *pKeyContent);
Hash_t *HashSearch(MacAddr_t *pMac);
Hash_t *RxInfoIndicate(MacAddr_t *sta, PsMode psm, U8 rate);
void RxInfoUpdate(Hash_t *pHash, PsMode psm, U8 rate);
extern Hash_t *HashTbl[MAX_RECORD];
extern Hash_t *sstByAid[MAX_RECORD];
void initHashBuf(void);
void InitHashTbl(void);
void CleanupHash(Hash_t *hash);
#endif
