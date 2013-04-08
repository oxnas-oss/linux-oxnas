#ifndef __ZDPSMON_C__
#define __ZDPSMON_C__

#include "zd80211.h"
#include "zddebug.h"

#define GetEntry(pMac)		(((pMac->mac[3]) ^ (pMac->mac[4]) ^ (pMac->mac[5])) & (MAX_AID-1))

Hash_t *FreeHashList;
Hash_t HashBuf[MAX_RECORD];
Hash_t *HashTbl[MAX_RECORD];
Hash_t *sstByAid[MAX_RECORD];
U32 freeHashCount;

extern void zd1205_config_dyn_key(u8 DynKeyMode, u8 *pkey, int idx);
Hash_t *HashInsert(MacAddr_t *pMac);

void CleanupHash(Hash_t *hash)
{
        memset(hash->mac, 0, 6);
        hash->asoc = STATION_STATE_DIS_ASOC;
        hash->auth = STATION_STATE_NOT_AUTH;
        hash->psm = PSMODE_STA_ACTIVE;
        hash->encryMode = WEP_NOT_USED;
        hash->ZydasMode = 0;
        hash->pkInstalled = 0;
        hash->AlreadyIn = 0;
        hash->ContSuccFrames = 0;
        hash->ttl = 0;
        hash->bValid = FALSE;
        hash->Preamble = 0;
        hash->keyLength = 0;
        hash->KeyId = 0;
        memset(hash->wepIv, 0, 4);
        memset(&hash->TxSeed, 0, sizeof(Seedvar));
        memset(&hash->RxSeed, 0, sizeof(Seedvar));
        memset(&hash->TxMicKey, 0, sizeof(MICvar));
        memset(&hash->RxMicKey, 0, sizeof(MICvar));
        hash->SuccessFrames = 0;
        hash->FailedFrames = 0;
        hash->bJustRiseRate = FALSE;
        hash->RiseConditionCount = 0;
        hash->DownConditionCount = 0;
        hash->vapId = 0;
#if defined(OFDM)

        hash->bErpSta = TRUE;
#else

        hash->bErpSta = FALSE;
#endif
}


void CleanupKeyInfo(Hash_t *hash)
{
        hash->encryMode = WEP_NOT_USED;
        hash->pkInstalled = 0;
        hash->keyLength = 0;
        hash->KeyId = 0;
        memset(hash->wepIv, 0, 4);
        memset(&hash->TxSeed, 0, sizeof(Seedvar));
        memset(&hash->RxSeed, 0, sizeof(Seedvar));
        memset(&hash->TxMicKey, 0, sizeof(MICvar));
        memset(&hash->RxMicKey, 0, sizeof(MICvar));
}


void initHashBuf(void)
{
        int i;

        freeHashCount = MAX_RECORD;

        for (i=0; i<MAX_AID; i++) { //from 0 to 31
                HashBuf[i].pNext = &HashBuf[i+1];
                sstByAid[i] = &HashBuf[i];
                HashBuf[i].aid = i;
                CleanupHash(&HashBuf[i]);
        }

        //aid 32 is here
        HashBuf[MAX_AID].pNext = NULL;
        sstByAid[MAX_AID] = &HashBuf[MAX_AID];
        HashBuf[MAX_AID].aid = MAX_AID;
        CleanupHash(&HashBuf[MAX_AID]);

        FreeHashList = &HashBuf[1]; //by pass aid = 0

        //deal with aid = 0
        HashBuf[0].pNext = NULL;
}


Hash_t *allocHashBuf(void)
{
        Hash_t *hash = NULL;
        /* ath_desc: AMD64 support */
        unsigned long flags;

        //HSDEBUG("*****allocHashBuf*****");
        flags = pdot11Obj->EnterCS();
        if (FreeHashList != NULL) {
                hash = FreeHashList;
                FreeHashList = FreeHashList->pNext;
                hash->pNext = NULL;
                freeHashCount--;
        }
        pdot11Obj->ExitCS(flags);
        return hash;
}



void freeHashBuf(Hash_t *hash)
{
        unsigned long flags;

        //HSDEBUG("*****freeHashBuf*****");
        flags = pdot11Obj->EnterCS();
        if (hash->AlreadyIn) {
                if (mCurrConnUser > 0)
                        mCurrConnUser--;
                if (hash->bErpSta == FALSE && mNumBOnlySta > 0) {
                        mNumBOnlySta--;
                        if (mNumBOnlySta==0) {
                                pdot11Obj->ConfigFlag &= ~NON_ERP_PRESENT_SET;
                                mErp.buf[2] &= ~NON_ERP_PRESENT;
                        }
                }
        }

        if (hash->psm == PSMODE_POWER_SAVE) {
                if (mPsStaCnt > 0)
                        mPsStaCnt--;
        }

#if defined(AMAC)
        HW_CAM_ClearRollTbl(pdot11Obj, hash->aid);
#endif

        CleanupHash(hash);
        hash->pNext = FreeHashList;
        FreeHashList = hash;
        freeHashCount++;
        pdot11Obj->ExitCS(flags);
}


void InitHashTbl(void)
{
        int i;

        for (i=0; i<MAX_RECORD; i++) {
                HashTbl[i] = NULL;
        }
}


Hash_t *HashSearch(MacAddr_t *pMac)
{
        U8 entry;
        Hash_t *hash = NULL;
        unsigned long flags;

        if (mBssType == INFRASTRUCTURE_BSS) {
                if (memcmp(&mBssId, pMac, 6) != 0) {
                        return NULL;
                } else
                        return sstByAid[0];
        }

        //HSDEBUG("HashSearch");
        entry = GetEntry(pMac);
        flags = pdot11Obj->EnterCS();
        if (HashTbl[entry] == NULL) {
                goto exit;
        } else {
                hash = HashTbl[entry];
                do {
                        if (memcmp(hash->mac, (U8 *)pMac, 6) == 0) {
                                //HSDEBUG("Search got one");
                                goto exit;
                        } else
                                hash = hash->pNext;

                } while(hash != NULL);
        }

exit:
        pdot11Obj->ExitCS(flags);
        if (hash) {
#if 0
                printf("macaddr = %02x:%02x:%02x:%02x:%02x:%02x\n",
                       hash->mac[0],  hash->mac[1], hash->mac[2],
                       hash->mac[3], hash->mac[4], hash->mac[5]);
                printf("asoc = %x\n", hash->asoc);
                printf("auth = %x\n", hash->auth);
                printf("psm = %x\n", hash->psm);
                printf("aid = %x\n", hash->aid);
                printf("lsInterval = %x\n", hash->lsInterval);
#endif

        } else
                ;//HSDEBUG("Search no one");

        return hash;
}






Hash_t *HashInsert(MacAddr_t *pMac)
{
        U8 entry;
        Hash_t *hash;
        unsigned long flags;

        HSDEBUG("HashInsert");

        if (mBssType == INFRASTRUCTURE_BSS) {
                hash = sstByAid[0];
                memcpy(hash->mac, (U8 *)pMac, 6);
                hash->ttl = HW_GetNow(pdot11Obj);
                hash->bValid = TRUE;
                return hash;
        }

        hash = allocHashBuf();
        if (!hash) {
                HSDEBUG("No free one");
                //Age Hash table
                AgeHashTbl();
                return NULL; // no free one
        } else {
                entry = GetEntry(pMac);
                HSDEBUG_V("entry", entry);

                if (HashTbl[entry] == NULL) { //entry is null
                        HashTbl[entry] = hash;
                        HSDEBUG("Entry is null");
                } else { //insert list head
                        flags = pdot11Obj->EnterCS();
                        hash->pNext = HashTbl[entry];
                        HashTbl[entry] = hash;
                        pdot11Obj->ExitCS(flags);
                        HSDEBUG("Insert to list head");
                }

                memcpy(hash->mac, (U8 *)pMac, 6);
                hash->ttl = HW_GetNow(pdot11Obj);
                hash->bValid = TRUE;
                return hash;
        }
}


BOOLEAN AgeHashTbl(void)
{
        U32 now, ttl, idleTime;
        U8 entry, firstLayer;

        int i;
        MacAddr_t *pMac;
        Hash_t *hash, *preHash = NULL;
        BOOLEAN ret = FALSE;

        HSDEBUG("*****AgeHashTbl*****");
        now = HW_GetNow(pdot11Obj);

        for (i=1; i<(MAX_AID+1); i++) {
                ttl = sstByAid[i]->ttl;
                if (now > ttl)
                        idleTime = now - ttl;
                else
                        idleTime = 	(0xffffffff - ttl) + now;


                if (sstByAid[i]->bValid) {
                        if (idleTime > IDLE_TIMEOUT ) {
                                HSDEBUG("*****Age one*****");
                                HSDEBUG_V("aid", i);
                                HSDEBUG_V("now", now);
                                HSDEBUG_V("ttl", ttl);
                                HSDEBUG_V("idleTime", idleTime);

                                pMac = (MacAddr_t *)&sstByAid[i]->mac[0];
                                entry = GetEntry(pMac);
                                HSDEBUG_V("entry", entry);
                                hash = HashTbl[entry];
                                firstLayer = 1;
                                do {
                                        if (hash == sstByAid[i]) {
                                                if (firstLayer == 1) {
                                                        HSDEBUG("*****firstLayer*****");
                                                        if (hash->pNext != NULL)
                                                                HashTbl[entry] = hash->pNext;
                                                        else
                                                                HashTbl[entry] = NULL;
                                                } else {
                                                        HSDEBUG("*****Not firstLayer*****");
                                                        preHash->pNext = hash->pNext;
                                                }
                                                zd_CmdProcess(CMD_DISASOC, &hash->mac[0], ZD_INACTIVITY);
                                                freeHashBuf(hash);
                                                break;
                                        } else {
                                                preHash = hash;
                                                hash = hash->pNext;
                                                firstLayer = 0;
                                        }
                                } while(hash != NULL);
                                ret = TRUE;
                        } else {
                                if (sstByAid[i]->ZydasMode == 1)
                                        mZyDasModeClient = TRUE;

                                if (sstByAid[i]->bErpSta == FALSE && mMacMode != PURE_A_MODE) {
                                        pdot11Obj->ConfigFlag |= NON_ERP_PRESENT_SET;
                                        pdot11Obj->ConfigFlag |= ENABLE_PROTECTION_SET;
                                        if (sstByAid[i]->Preamble == 0) { //long preamble
                                                pdot11Obj->ConfigFlag |= BARKER_PREAMBLE_SET;
                                        }
                                }
                        }
                }

        }

        //HSDEBUG_V("ret", ret);
        return ret;
}


void ResetPSMonitor(void)
{
        ZDEBUG("ResetPSMonitor");
        initHashBuf();
        InitHashTbl();
        mPsStaCnt = 0;
}


Hash_t *RxInfoIndicate(MacAddr_t *sta, PsMode psm, U8 rate)
{
        Hash_t *pHash;

        ZDEBUG("RxInfoIndicate");

        //if (isGroup(sta))
        //return NULL;

        pHash = HashSearch(sta);
        if (!pHash) {
                if (mBssType == PSEUDO_IBSS) {
                        pHash = HashInsert(sta);
                        if (!pHash)
                                return NULL;
                        else {
                                pHash->asoc = STATION_STATE_ASOC;
                                zd1205_dump_data(" HashInsert macAddr = ", (U8 *)&pHash->mac[0], 6);
                                goto updateInfo;
                        }
                } else
                        return NULL;
        } else {
updateInfo:
                if (rate > pHash->MaxRate)
                        pHash->MaxRate = rate;

                pHash->RxRate = rate;
                pHash->ttl = HW_GetNow(pdot11Obj);

                if (mBssType == AP_BSS) {
                        PsMode oldPsm = pHash->psm;
                        StationState asoc = pHash->asoc;

                        if (psm == PSMODE_STA_ACTIVE) {
                                if (oldPsm == PSMODE_POWER_SAVE) {
                                        StaWakeup(sta);
                                        if (asoc == STATION_STATE_ASOC) {
                                                if (mPsStaCnt >0) {
                                                        mPsStaCnt--;
                                                }
                                        }
                                }
                        } else {
                                if (oldPsm == PSMODE_STA_ACTIVE) {
                                        if (asoc == STATION_STATE_ASOC) {
                                                if (mPsStaCnt < MAX_AID) {
                                                        mPsStaCnt++;
                                                }
                                        }
                                } else if (oldPsm == PSMODE_POWER_SAVE) {
                                        if (asoc == STATION_STATE_ASOC) {
                                                if (mPsStaCnt == 0)
                                                        mPsStaCnt++;
                                        }
                                }
                        }
                }

                pHash->psm = psm;
        }

        return pHash;
}


void RxInfoUpdate(Hash_t *pHash, PsMode psm, U8 rate)
{
        PsMode oldPsm = pHash->psm;
        StationState asoc = pHash->asoc;

        if (rate > pHash->MaxRate)
                pHash->MaxRate = rate;

        pHash->RxRate = rate;
        pHash->ttl = HW_GetNow(pdot11Obj);

        if (psm == PSMODE_STA_ACTIVE) {
                if (oldPsm == PSMODE_POWER_SAVE) {
                        StaWakeup((MacAddr_t *)pHash->mac);
                        if (asoc == STATION_STATE_ASOC) {
                                if (mPsStaCnt >0) {
                                        mPsStaCnt--;
                                }
                        }
                }
        } else {
                if (oldPsm == PSMODE_STA_ACTIVE) {
                        if (asoc == STATION_STATE_ASOC) {
                                if (mPsStaCnt < MAX_AID) {
                                        mPsStaCnt++;
                                }
                        }
                } else if (oldPsm == PSMODE_POWER_SAVE) {
                        if (asoc == STATION_STATE_ASOC) {
                                if (mPsStaCnt == 0)
                                        mPsStaCnt++;
                        }

                }
        }


        pHash->psm = psm;
}


BOOLEAN UpdateStaStatus(MacAddr_t *sta, StationState staSte, U8 vapId)
{
        Hash_t *pHash;

        ZDEBUG("UpdateStaStatus");

        if (mBssType == AP_BSS) {
                pHash = HashSearch(sta);
                if (pHash)
                        goto UpdateStatus;
                else {
                        if ((STATION_STATE_AUTH_OPEN == staSte) || (STATION_STATE_AUTH_KEY == staSte)) {
                                if ((mCurrConnUser + 1) > mLimitedUser) {
                                        //AgeHashTbl();
                                        return FALSE;
                                } else {
                                        pHash = HashInsert(sta);
                                        if (!pHash)
                                                return FALSE;
                                }
                        } else
                                return FALSE;
                }
        } else if (mBssType == INFRASTRUCTURE_BSS) {
                if ((STATION_STATE_AUTH_OPEN == staSte) || (STATION_STATE_AUTH_KEY == staSte)) {
                        CleanupHash(sstByAid[0]);
                        pHash = HashInsert(sta);
                } else {
                        pHash = sstByAid[0]; //use aid = 0 to store AP's info
                }
        } else if (mBssType == INDEPENDENT_BSS) {
                pHash = HashSearch(sta);
                if (pHash)
                        goto UpdateStatus;
                else {
                        pHash = HashInsert(sta);
                        if (!pHash)
                                return FALSE;
                        else
                                zd1205_dump_data(" HashInsert macAddr = ", (U8 *)&pHash->mac[0], 6);
                }
        } else
                return FALSE;

UpdateStatus:
        switch(staSte) {
        case STATION_STATE_AUTH_OPEN:
        case STATION_STATE_AUTH_KEY:
                pHash->auth = staSte;
                break;

        case STATION_STATE_ASOC:
                if (mBssType == AP_BSS) {
                        if (((mCurrConnUser + 1) > mLimitedUser) && (!pHash->AlreadyIn)) {
                                return FALSE;
                        }

                        if (pHash->psm == PSMODE_POWER_SAVE) {
                                if (mPsStaCnt > 0) {
                                        mPsStaCnt--;
                                }

                        }

                        pHash->asoc = STATION_STATE_ASOC;
                        /*if (!pHash->AlreadyIn){
				pHash->AlreadyIn = 1;
				mCurrConnUser++;
                        }*/
                } else {
                        pHash->asoc = STATION_STATE_ASOC;
                }

                if (mBssType != INDEPENDENT_BSS)
                        CleanupKeyInfo(pHash);

                memcpy(&pdot11Obj->CurrSsid[0], (U8 *)&mSsid, mSsid.buf[1]+2);
                break;

        case STATION_STATE_NOT_AUTH:
        case STATION_STATE_DIS_ASOC:
                if (mBssType == AP_BSS) {
                        if (pHash->asoc == STATION_STATE_ASOC) {
                                if (pHash->psm == PSMODE_POWER_SAVE) {
                                        FlushQ(pPsQ[pHash->aid]);
                                        if (mPsStaCnt > 0) {
                                                mPsStaCnt--;
                                                if (mPsStaCnt == 0) {
                                                        FlushQ(pAwakeQ);
                                                        FlushQ(pPsQ[0]);
                                                }
                                        }
                                }
                                /*if (pHash->AlreadyIn){
					pHash->AlreadyIn = 0;
					mCurrConnUser--;
                                }*/
                        }
                }

                pHash->auth = STATION_STATE_NOT_AUTH;
                pHash->asoc = STATION_STATE_DIS_ASOC;
                CleanupKeyInfo(pHash);
                //for Rx-Retry filter
                HW_CAM_ClearRollTbl(pdot11Obj, pHash->aid);
                {
                        MacAddr_t	*pMac;
                        Hash_t 		*sta_info;
                        U8 			entry;
                        pMac = (MacAddr_t *) pHash->mac;
                        entry = GetEntry(pMac);
                        sta_info=HashTbl[entry];
                        if (sta_info) {
                                if (memcmp(sta_info->mac, pHash->mac, 6)==0) {
                                        HashTbl[entry]=sta_info->pNext;
                                        freeHashBuf(pHash);
                                } else {
                                        while (sta_info->pNext != NULL && memcmp(sta_info->pNext->mac, pHash->mac, 6) != 0)
                                                sta_info = sta_info->pNext;
                                        if (sta_info->pNext != NULL) {
                                                Hash_t	*sta_info1;
                                                sta_info1 = sta_info->pNext;
                                                sta_info->pNext =  sta_info->pNext->pNext;
                                                freeHashBuf(sta_info1);
                                        } else {
                                                printk(KERN_DEBUG "Could not remove STA:" MACSTR "\n", MAC2STR(pHash->mac));
                                        }
                                }
                        }
                }

                break;

        }

        return TRUE;
}


void SsInquiry(MacAddr_t *sta, StationState *sst, StationState *asst)
{
        ZDEBUG("SsInquiry");
        if (isGroup(sta)) {
                *asst = STATION_STATE_NOT_AUTH;
                *sst = STATION_STATE_DIS_ASOC;
        } else {
                Hash_t *pHash;
                pHash = HashSearch(sta);

                if (!pHash) {
                        *asst = STATION_STATE_NOT_AUTH;
                        *sst = STATION_STATE_DIS_ASOC;
                } else {
                        *asst = pHash->auth;
                        if ((*asst == STATION_STATE_AUTH_OPEN) || (*asst == STATION_STATE_AUTH_KEY))
                                *sst = pHash->asoc;
                        else
                                *sst = STATION_STATE_DIS_ASOC;
                }
        }

}


U16 AIdLookup(MacAddr_t *sta)
{
        Hash_t *pHash;

        ZDEBUG("AIdLookup");
        pHash = HashSearch(sta);
        if (!pHash)
                return (U16)0;
        else
                return pHash->aid;
}


void AssocInfoUpdate(MacAddr_t *sta, U8 MaxRate, U8 lsInterval, U8 ZydasMode, U8 Preamble, BOOLEAN bErpSta, U8 vapId)
{
        Hash_t *pHash;

        ZDEBUG("AssocInfoUpdate");
        if (isGroup(sta))
                return;

        pHash = HashSearch(sta);
        if (!pHash)
                return;
        else {
                pHash->MaxRate = MaxRate;
                pHash->CurrTxRate = MaxRate;
                pHash->lsInterval = lsInterval;
                pHash->ZydasMode = ZydasMode;
                pHash->Preamble = Preamble;
                pHash->bErpSta = bErpSta;
                pHash->vapId = vapId;
        }
}

int zd_SetKeyInfo(U8 *addr, U8 encryMode, U8 keyLength, U8 KeyId, U8 *pKeyContent)
{
        Hash_t *pHash;
        MacAddr_t *sta = (MacAddr_t *)addr;
        U8 bcAddr[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
        U8 ZeroAddr[6]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        U16 aid;
        U8 change_enc = 0;

        if (isGroup(sta)) {
                change_enc = 1;
                if (keyLength == 0) {
                        WPADEBUG("Clear Group key RollTbl (aid0)\n");
                        HW_CAM_ClearRollTbl(pdot11Obj, 0);//Clear group key.(aid0)
                } else if (encryMode == WEP64 || encryMode == WEP128) {

                        if (mWpaBcKeyLen == keyLength && mGkInstalled == 1)
                                change_enc = 0; // Nonfirst time group key update.
                        mWpaBcKeyLen = keyLength;
                        mBcKeyId = KeyId;// We tell KeyLength to hdwr by encryMode, while, the key ID is set by the driver when transmit packet.

                        if (mOperationMode != CAM_AP_VAP) {
                                HW_ConfigDynaKey(pdot11Obj, 32, &bcAddr[0], pKeyContent, keyLength, encryMode, change_enc);
                                // Also set default key for Multicast case to avoid Tx-underrun.
                                HW_CAM_Write(pdot11Obj, DEFAULT_ENCRY_TYPE, encryMode);
                                HW_ConfigStatKey(pdot11Obj, pKeyContent, keyLength, STA_KEY_START_ADDR+(KeyId * 8));
                        } else
                                HW_ConfigDynaKey(pdot11Obj, CAM_VAP_START_AID, (U8 *)&dot11MacAddress, pKeyContent, keyLength, encryMode,change_enc);

                        mGkInstalled = 1;
                        return 0;
                } else if (encryMode == TKIP) {
                        //only vaild for Tx
                        if (mWpaBcKeyLen == keyLength && mGkInstalled==1)//Non-First time group key update.
                                change_enc = 0;

                        mWpaBcKeyLen = keyLength;
                        mWpaBcKeyId = KeyId;
                        if (mWpaBcKeyLen == 32) {
                                if (mOperationMode != CAM_AP_VAP) {
                                        //Tmep key(16), Tx Mic Key(8), Rx Mic Key(8)
                                        HW_ConfigDynaKey(pdot11Obj, 32, &bcAddr[0], pKeyContent, keyLength, encryMode, change_enc);
                                        // Also set default key for Multicast case to avoid Tx-underrun.
                                        //if ((mDebugFlag & BIT_1)==0)
                                        {
                                                HW_CAM_Write(pdot11Obj, DEFAULT_ENCRY_TYPE, encryMode);
                                                HW_ConfigStatKey(pdot11Obj, pKeyContent, keyLength, STA_KEY_START_ADDR+(KeyId * 8));
                                        }
                                } else {
                                        HW_ConfigDynaKey(pdot11Obj, CAM_VAP_START_AID, (U8 *)&dot11MacAddress, pKeyContent, keyLength, encryMode, change_enc);
                                }
                                if (mBssType == INFRASTRUCTURE_BSS)
                                        MICsetKey(&pKeyContent[24], &mBcMicKey); //Tx Mic key
                                else
                                        MICsetKey(&pKeyContent[16], &mBcMicKey);// For Infra-STA mode.
                        }
                        mGkInstalled = 1;
                        return 0;
                } else if (encryMode == AES) {
                        if (mWpaBcKeyLen == keyLength && mGkInstalled == 1)//Non-First time group key update.
                                change_enc = 0;
                        mWpaBcKeyLen = keyLength;
                        mWpaBcKeyId = KeyId;
                        if (mWpaBcKeyLen == 16) {
                                if (mOperationMode != CAM_AP_VAP) {
                                        HW_ConfigDynaKey(pdot11Obj, 32, &bcAddr[0], pKeyContent, keyLength, encryMode, change_enc);
                                        // Also set default key for Multicast case to avoid Tx-underrun.
                                        HW_CAM_Write(pdot11Obj, DEFAULT_ENCRY_TYPE, encryMode);
                                        HW_ConfigStatKey(pdot11Obj, pKeyContent, keyLength, STA_KEY_START_ADDR+(KeyId * 8));
                                } else {
                                        HW_ConfigDynaKey(pdot11Obj, CAM_VAP_START_AID, (U8 *)&dot11MacAddress, pKeyContent, keyLength, encryMode, change_enc);
                                }
                        }
                        mGkInstalled = 1;
                        return 0;
                } else {
                        return -1;
                }
        }// End of Group key setting.

        // Start of Pairwise key setting.
        pHash = HashSearch(sta);
        if (!pHash) {
                if (!memcmp(&sta->mac[0], ZeroAddr, 6)) {
                        int i;
                        HW_CAM_ResetRollTbl(pdot11Obj);
                        if (mGkInstalled) {
                                HW_CAM_UpdateRollTbl(pdot11Obj,0);//ReEnable group key.
                        }
                        if (mBssType != INFRASTRUCTURE_BSS) {//AP mode.
                                WPADEBUG("clear all tx key\n");
                                for (i=0; i<MAX_RECORD; i++)
                                        HashBuf[i].pkInstalled=0;
                        } else {// STA mode.
                                WPADEBUG("clear key of aid %d\n",sstByAid[0]->aid);
                                sstByAid[0]->pkInstalled=0;
                        }
                }
                return -1;
        } else {
                pHash->keyLength = keyLength;
                if (pHash->encryMode != encryMode)
                        change_enc = 1;
                pHash->encryMode = encryMode;
                aid = pHash->aid;

                if (encryMode != NO_WEP)
                        WPADEBUG("********* Set key%s for aid:%d\n",DbgStrEncryType[encryMode & 7],aid);
                else
                        WPADEBUG("********* Clear key for aid:%d\n",aid);
                if (encryMode == NO_WEP) {// Clear pairwise key
                        pHash->pkInstalled = 0;
                        if (mBssType == INFRASTRUCTURE_BSS)
                                HW_CAM_ClearRollTbl(pdot11Obj, 8);
                        else
                                HW_CAM_ClearRollTbl(pdot11Obj, aid);
                } else if (encryMode == TKIP) {
                        if (mBssType == INFRASTRUCTURE_BSS) {
                                //				zd1205_dump_data("key:", (u8*)pKeyContent, 32);
                                HW_ConfigDynaKey(pdot11Obj, 8, addr, pKeyContent, 32, encryMode, change_enc);
                        } else
                                HW_ConfigDynaKey(pdot11Obj, aid, addr, pKeyContent, 32, encryMode, change_enc);

                        MICsetKey(&pKeyContent[16], &pHash->TxMicKey);
                        MICsetKey(&pKeyContent[24], &pHash->RxMicKey);
                        pHash->KeyId = KeyId;
                        pHash->pkInstalled = 1;
                } else //if (encryMode == AES)
                {
                        if (mBssType == INFRASTRUCTURE_BSS) {
                                WPADEBUG("********* setAESkey\n");
                                HW_ConfigDynaKey(pdot11Obj, 8, addr, pKeyContent, keyLength, encryMode, change_enc);
                        } else
                                HW_ConfigDynaKey(pdot11Obj, aid, addr, pKeyContent, keyLength, encryMode, change_enc);
                        pHash->KeyId = KeyId;
                        pHash->pkInstalled = 1;
                }
                return 0;
        }
}

BOOLEAN zd_GetKeyInfo(U8 *addr, U8 *encryMode, U8 *keyLength, U8 *pKeyContent)
{
        Hash_t *pHash;
        MacAddr_t *sta = (MacAddr_t *)addr;

        ZDEBUG("zd_GetKeyInfo");
        if (isGroup(sta)) {
                return FALSE;
        }

        pHash = HashSearch(sta);
        if (!pHash) {
                *encryMode = 0;
                *keyLength = 0;
                return FALSE;
        } else {
                *encryMode = pHash->encryMode;
                *keyLength = pHash->keyLength;
                memcpy(pKeyContent, &pHash->keyContent[0], pHash->keyLength);
                return TRUE;
        }
}

/**
 * zd_SetKeyContext - Set Key context to CAM (used for WPA/WPA2)
 * @addr: MAC address of AP we associated with
 * @encryMode: Encryption mode
 * @keyLength: Length of key context
 * @keyId: Key index
 * @pKeyContent: Context of key
 */
#if 0

int zd_SetKeyContext(U8 *addr, U8 encryMode, U8 keyLength, U8 KeyId, U8 *pKeyContent)
{
        Hash_t *pHash;

        if (isGroup(addr)) {
                mWpaBcKeyLen = keyLength;
                mWpaBcKeyId = KeyId;

                if (encryMode == DYN_KEY_TKIP) {
                        if (keyLength == 32) {
                                zd1205_config_dyn_key(encryMode, pKeyContent, KeyId);
                                MICsetKey(&pKeyContent[24], &mBcMicKey);
                        }

                        mGkInstalled = 1;
                        return 0;
                } else if (encryMode == DYN_KEY_AES) {
                        printk(KERN_ERR "***** set group key ID: %d\n",KeyId);
                        zd1205_config_dyn_key(encryMode, pKeyContent, KeyId);
                        mGkInstalled = 1;
                        return 0;
                } else {
                        WPADEBUG("zd_SetKeyContext: encryMode: %d not support\n", encryMode);
                        return -1;
                }

        }

        pHash = HashSearch((MacAddr_t*)addr);

        if(!pHash) {
                WPADEBUG("Can't find AP's MAC address in the hash table\n");
                return -1;
        } else {
                pHash->encryMode = encryMode;

                if (encryMode == DYN_KEY_TKIP) {
                        zd1205_config_dyn_key(encryMode, pKeyContent, KeyId);

                        MICsetKey(&pKeyContent[16], &pHash->TxMicKey);
                        MICsetKey(&pKeyContent[24], &pHash->RxMicKey);
                        pHash->KeyId = KeyId;
                        pHash->pkInstalled = 1;
                } else if (encryMode == DYN_KEY_AES) {
                        zd1205_config_dyn_key(encryMode, pKeyContent, KeyId);
                        pHash->KeyId = KeyId;
                        pHash->pkInstalled = 1;
                }
                else {
                        WPADEBUG("zd_SetKeyContext: encryMode: %d not support\n", encryMode);
                }
        }

        return 0;
}
#endif

#if defined(PHY_1202)
int zd_GetKeyInfo_ext(U8 *addr, U8 *encryMode, U8 *keyLength, U8 *pKeyContent, U16 iv16, U32 iv32)
{
        Hash_t *pHash;
        MacAddr_t *sta = (MacAddr_t *)addr;

        ZDEBUG("zd_GetKeyInfo_ext");
        if (isGroup(sta)) {
                return -1;
        }

        if (mDynKeyMode != DYN_KEY_TKIP)
                return -1;

        pHash = HashSearch(sta);
        if (!pHash) {
                *encryMode = 0;
                *keyLength = 0;
                return -1;
        } else {
                if (pHash->pkInstalled == 0)
                        return -2;

                if ((iv16 == pHash->RxSeed.IV16) && (iv32 == pHash->RxSeed.IV32)) {
                        // iv out of sequence
                        //FPRINT_V("iv16", iv16);
                        //FPRINT_V("iv32", iv32);
                        //return -3;
                }

                *encryMode = pHash->encryMode;
                *keyLength = pHash->keyLength;
                //do key mixing
                Tkip_phase1_key_mix(iv32, &pHash->RxSeed);
                Tkip_phase2_key_mix(iv16, &pHash->RxSeed);
                Tkip_getseeds(iv16, pKeyContent, &pHash->RxSeed);
                pHash->RxSeed.IV16 = iv16;
                pHash->RxSeed.IV32 = iv32;
                return pHash->aid;
        }
}


int zd_SetTsc(U8 *addr, U8 KeyId, U8 direction, U32 tscHigh, U16 tscLow)
{
        Hash_t *pHash;
        MacAddr_t *sta = (MacAddr_t *)addr;

        ZDEBUG("zd_SetTsc");
        if (isGroup(sta)) {
                return -1;
        }

        pHash = HashSearch(sta);
        if (!pHash)
                return -1;
        else {
                pHash->KeyId = KeyId;
                if (direction == 0) { //Tx
                        pHash->TxSeed.IV16 = tscLow;
                        pHash->TxSeed.IV32 = tscHigh;
                } else if (direction == 1) { //Rx
                        pHash->RxSeed.IV16 = tscLow;
                        pHash->RxSeed.IV32 = tscHigh;
                }
                return 0;
        }
}


int zd_GetTsc(U8 *addr, U8 KeyId, U8 direction, U32 *tscHigh, U16 *tscLow)
{
        Hash_t *pHash;
        MacAddr_t *sta = (MacAddr_t *)addr;

        ZDEBUG("zd_GetTsc");
        if (isGroup(sta)) {
                return -1;
        }

        pHash = HashSearch(sta);
        if (!pHash)
                return -1;
        else {
                if (direction == 0) { //Tx
                        *tscLow = pHash->TxSeed.IV16;
                        *tscHigh = pHash->TxSeed.IV32;
                } else if (direction == 1) { //Rx
                        *tscLow = pHash->RxSeed.IV16;
                        *tscHigh = pHash->RxSeed.IV32;
                }
                return 0;
        }
}
#endif


BOOLEAN zd_CheckIvSeq(U8 aid, U16 iv16, U32 iv32)
{
        Hash_t *pHash = NULL;
        U16 oldIv16;
        U32 oldIv32;


        ZDEBUG("zd_CheckIvSeq");

        if (mDynKeyMode != DYN_KEY_TKIP) {
                FPRINT("Not in DYN_KEY_TKIP mode");
                return FALSE;
        }

        pHash = sstByAid[aid];
        if (!pHash) {
                FPRINT("zd_CheckIvSeq failed");
                return FALSE;
        } else {
                if (pHash->pkInstalled == 0) {
                        FPRINT("pkInstalled == 0");
                        return FALSE;
                }

                oldIv16 = pHash->RxSeed.IV16;
                oldIv32 = pHash->RxSeed.IV32;

#if 1

                if ((oldIv16 == iv16) && (oldIv32 == iv32)) {
                        // iv out of sequence
                        FPRINT("iv out of sequence");
                        FPRINT_V("iv16", iv16);
                        FPRINT_V("iv32", iv32);
                        return FALSE;
                }

#else //If fifo overrun, this will failed
                if (iv32 == oldIv32) {
                        if (iv16 != oldIv16+1) {
                                // iv out of sequence
                                FPRINT("iv out of sequence");
                                FPRINT_V("iv16", iv16);
                                FPRINT_V("iv32", iv32);
                                return FALSE;
                        }
                } else {
                        if ((iv16 != 0) || (oldIv16 != 0xffff)) {
                                // iv out of sequence
                                FPRINT("iv out of sequence");
                                FPRINT_V("iv16", iv16);
                                FPRINT_V("iv32", iv32);
                                return FALSE;
                        }
                }
#endif

                pHash->RxSeed.IV16 = iv16;
                pHash->RxSeed.IV32 = iv32;
                return TRUE;
        }
}


#endif
