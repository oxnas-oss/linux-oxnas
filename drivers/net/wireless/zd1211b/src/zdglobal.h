#ifndef __ZDGLOBAL_H__
#define __ZDGLOBAL_H__

extern U16		mRfChannel;
extern U16		mDtimPeriod;
extern U16		mBeaconPeriod;
extern Element		dot11DesiredSsid;
extern U8		mAuthAlogrithms[2];
extern U8		mPreambleType;

extern U32		mDebugFlag;

extern BOOLEAN 		mPrivacyInvoked;
extern U8 		mKeyId;
extern U8 		mBcKeyId;
extern U8		mKeyFormat;
extern MacAddr_t	dot11MacAddress;
extern U16 		mRtsThreshold;
extern U16		mFragThreshold;
extern const	U8  zeroMacAddress[6];
extern U8 		dot11DesiredBssid[6];  // When macp->ap_scan=1, use this to associate with an AP.
// for debugging purpose
extern u8 *DbgStrEncryType[];
extern u8 *DbgStrDynKeyMode[];


//WPA
extern Element		mWPAIe;
extern U8	mCounterMeasureState;
extern U8			mNumBOnlySta;

extern U16 		mCap;
extern U16 		mDtimCount;
extern Element		mSsid;
extern Element		mBrates;
extern Element 		mPhpm;
extern MacAddr_t 	mBssId;

//feature
extern U8 		mPsStaCnt;
extern U8		mHiddenSSID;
extern U8		mLimitedUser;
extern U8		mCurrConnUser;
extern U8		mBlockBSS;
extern U8		mRadioOn;
extern U8		mSwCipher;
extern U8		mKeyVector[4][16];
extern U8		mBcKeyVector[16];
extern U8 		mWepIv[4];
extern U8 		mBcIv[4];
extern U8		mWepKeyLen;
extern U8		mBcKeyLen;
extern U8		mDynKeyMode;
extern BOOLEAN		mZyDasModeClient;
extern Seedvar		mBcSeed;
extern MICvar		mBcMicKey;
extern U8		mWpaBcKeyLen;
extern U8		mWpaBcKeyId;
extern U8		mGkInstalled;
extern U16		mIv16;
extern U32		mIv32;
extern MacAddr_t	dot11BCAddress;
extern BssInfo_t	mBssInfo[64];
extern U8		mBssNum;
extern U8		mBssCnt;
extern U16		mAuthAlg;
extern U16		mListenInterval;
extern U16		mAid;
extern BOOLEAN		mAssoc;
extern MacAddr_t	mOldAP;
extern U8		mBssType;
extern U16		mAPCap;
extern Element		mAPBrates;
extern U8		mBssIndex;
extern U16		mRequestFlag;
extern U8		mPwrState;

extern BOOLEAN		mAPAlive;
extern BOOLEAN		mProbeWithSsid;
extern Element		mIbssParms;
extern U16		mATIMWindow;
extern U8		mConnRetryCnt;
extern U8		mMaxTxRate;

extern Element		mErp;
extern Element		mExtRates;
extern U8		mMacMode;
extern U8		mOperationMode;
extern U8		mBurstMode;
extern Element		mAPErates;
extern BOOLEAN		mIfaceOpened;

extern U8		mAuthMode;
#endif
