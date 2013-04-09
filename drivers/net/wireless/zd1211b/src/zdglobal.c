#ifndef __ZDGLOBAL_C__
#define __ZDGLOBAL_C__

#include "zd80211.h"
#include "zddebug.h"


U8		mPreambleType = LONG_PREAMBLE;
MacAddr_t	dot11MacAddress = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
Element		dot11DesiredSsid;
U8 		dot11DesiredBssid[6];  // When macp->ap_scan=1, use this to associate with an AP.
U32		mDebugFlag = 0;
U8		mAuthAlogrithms[2] = {OPEN_SYSTEM, SHARE_KEY};
U16		mRfChannel = 0;
U16		mBeaconPeriod = 100;
U16		mDtimPeriod = 1;
U16		mFragThreshold = 2432;
U16 		mRtsThreshold = 2432;
U16		mTmRetryConnect=0;
// For debugging purpose
//#ifdef WPADATA_DEBUG
u8 		*DbgStrEncryType[]={"NOWEP","WEP64","TKIP","NA3","AES","WEP128","WEP256", "NA7"};
u8 		*DbgStrDynKeyMode[]={"NOWEP","WEP64","WEP128","NA3","TKIP","AES","NA6","NA7"};
//#endif
// ------------------------------------------------------------------------

//WPA
Element		mWPAIe;
U8		mCounterMeasureState;
//WEP
U8		mKeyId = 0;
U8		mKeyFormat = WEP64_USED;
BOOLEAN 	mPrivacyInvoked = FALSE;
Element		mSsid;
Element		mBrates;
Element 	mPhpm;
MacAddr_t	mBssId;
U16 		mCap = CAP_ESS;
U16 		mDtimCount;

U8	 	mPsStaCnt = 0;	//Station count for associated and in power save mode
U8		mHiddenSSID = 0;
U8		mLimitedUser = 0;
U8		mCurrConnUser = 0;
U8		mNumBOnlySta=0;

U8		mBlockBSS = 0;
U8		mRadioOn = 1;
U8		mSwCipher = 0;
U8		mKeyVector[4][16];
U8		mBcKeyVector[16];
U8 		mWepIv[4];
U8 		mBcIv[4];
U8		mWepKeyLen;
U8		mBcKeyLen;
U8		mBcKeyId;
U8		mDynKeyMode = 0;
BOOLEAN		mZyDasModeClient = FALSE;
Seedvar		mBcSeed;
MICvar		mBcMicKey;
U8		mWpaBcKeyLen = 0;
U8		mWpaBcKeyId = 1;
U8		mGkInstalled = 0;
U16		mIv16 = 0;
U32		mIv32 = 0;
const	U8  zeroMacAddress[6] =
        {
                0,0,0,0,0,0
        };
MacAddr_t	dot11BCAddress = {{0xff, 0xff, 0xff, 0xff, 0xff, 0xff}};
BssInfo_t	mBssInfo[64];
U8		mBssNum = 0;
U8		mBssCnt = 0;
U16		mAuthAlg = OPEN_SYSTEM;
U16		mListenInterval = 1;
U16		mAid;
BOOLEAN		mAssoc = FALSE;
MacAddr_t	mOldAP;
U8		mBssType = INFRASTRUCTURE_BSS;
U16		mAPCap;
Element		mAPBrates;
U8		mBssIndex = 0;
U16		mRequestFlag = 0;
U8		mPwrState = 0;
BOOLEAN		mAPAlive = FALSE;
BOOLEAN		mProbeWithSsid = FALSE;
Element		mIbssParms;
U16		mATIMWindow;
U8		mConnRetryCnt = 0;
U8		mMaxTxRate = 3;

// for G mode
Element		mErp = {{EID_ERP, 1, 0x00}};
Element		mExtRates;
U8		mMacMode = MIXED_MODE;
U8		mOperationMode;
U8		mBurstMode;
Element		mAPErates;
BOOLEAN		mIfaceOpened = FALSE;

Element		mBrates11A; //Basic Rate for 11A

U8		mAuthMode;
#endif
