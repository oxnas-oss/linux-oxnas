#ifndef __ZDSORTS_H__
#define __ZDSORTS_H__

#include "zdtkipseed.h"
#include "zdmic.h"

#define MAX_MSDU_LNG		1600// only payload
#define MAC_HDR_LNG			24	// 802.11, not include A4
#define WEP_ADD_LNG			8	// 4 for IV, 4 for ICV
#define WDS_ADD_LNG			6	// for Address 4
#define IV_LNG				4
#define EIV_LNG				4
#define ICV_LNG				4
#define CRC_LNG				4
#define MIC_LNG				8

#define MIN_FRAG_LNG		256
#define MAX_FRAG_NUM		(MAX_MSDU_LNG / (MIN_FRAG_LNG - MAC_HDR_LNG - CRC_LNG))


#define MAX_AID 			32
#define MAX_RECORD			(MAX_AID + 1)
#define BURST_NUM	        64//32
#define TXQ_THRESHOLD		48
#define	MCQ_THRESHOLD		15
#define CHAL_TEXT_LEN		128
#ifndef HOST_IF_USB
    #define IDLE_TIMEOUT		(10*60*1000*1000) //10 min
#else
    #define IDLE_TIMEOUT		(10*60*100) //10 min
#endif

#define RISE_RATE_THRESHOLD	4
#define HIGH_RISE_RATE_THRESHOLD	0xff
#define AGE_HASH_PERIOD		(10*60) //10 min

#define LONG_PREAMBLE		0
#define SHORT_PREAMBLE		1

#define OPEN_SYSTEM			0
#define SHARE_KEY			1
#define NULL_AUTH			2


#define WEP_NOT_USED		0
#define AES_USED			4
#define WEP64_USED			1
#define WEP128_USED			5
#define TKIP_USED			2
#define WEP256_USED			6


#define	TO_DS_BIT			0x01
#define FROM_DS_BIT			0x02
#define MORE_FRAG_BIT		0x04

#define PW_SAVE_BIT			0x10
#define MORE_DATA_BIT		0x20
#define WEP_BIT				0x40
#define ORDER_BIT			0x80

#define EIV_BIT				0x20
#define KEYID_MASK              0xC0
#define NON_ERP_PRESENT		0x01
#define USE_PROTECTION		0x02
#define BARKER_PREAMBLE		0x04
//register 0xD40
#define MIC_BUSY			0x01


typedef enum
{
        PSMODE_STA_ACTIVE,
        PSMODE_POWER_SAVE
} PsMode;


typedef enum
{
        STATION_STATE_NOT_AUTH,
        STATION_STATE_AUTH_OPEN,
        STATION_STATE_AUTH_KEY,
        STATION_STATE_ASOC,
        STATION_STATE_DIS_ASOC
} StationState;


typedef struct
{
        U8 mac[6];
}
MacAddr_t;


typedef enum
{
        EID_SSID 	= 0,
        EID_SUPRATES,
        EID_FHPARMS,
        EID_DSPARMS,
        EID_CFPARMS,
        EID_TIM,
        EID_IBPARMS,
        EID_COUNTRY,
        EID_CTEXT	= 0x10,
        EID_ERP		= 0x2A,
        EID_RSN		= 0x30,
        EID_EXT_RATES	= 0x32,
        EID_WPA		= 0xDD,
} ElementID;

typedef struct
{
        U8	buf[34+1];	//Max SSID Length = 32
}
Element;


typedef enum
{
        CAP_ESS 	= 0x01,
        CAP_IBSS	= 0x02,
        CAP_POLLABLE = 0x04,
        CAP_POLLREQ = 0x08,
        CAP_PRIVACY = 0x10,
        CAP_SHORT_PREAMBLE = 0x20,
        CAP_PBCC_ENABLE	= 0x40,
        CAP_SHORT_SLOT_TIME = 0x0400,
        CAP_DSSS_OFDM_BIT = 0x2000
} Capability;


typedef enum
{
        RC_UNSPEC_REASON = 1,
        RC_AUTH_NOT_VALID,
        RC_DEAUTH_LEAVE_BSS,
        RC_INACTIVITY,
        RC_AP_OVERLOAD,
        RC_CLASS2_ERROR,
        RC_CLASS3_ERROR,
        RC_DISAS_LEAVE_CSS,
        RC_ASOC_NOT_AUTH,
        RC_INVALID_IE = 13,
        RC_MIC_FAIL,
        RC_4WAY_SHAKE_TIMEOUT,
        RC_GKEY_UPDATE_TIMEOUT,
        RC_IE_IMCOMPABILITY,
        RC_MC_CIPHER_INVALID,
        RC_UNI_CIPHER_INVALID,
        RC_AKMP_INVALID,
        RC_UNSUP_RSNE_VERSION,
        RC_INVALID_RSNE_CAP,
        RC_8021X_AUTH_FAIL
} ReasonCode;


typedef enum
{
        SC_SUCCESSFUL = 0,
        SC_UNSPEC_FAILURE,
        SC_UNSUP_CAP = 10,
        SC_REASOC_NO_ASOC,
        SC_FAIL_OTHER,
        SC_UNSUPT_ALG,
        SC_AUTH_OUT_OF_SEQ,
        SC_CHAL_FAIL,
        SC_AUTH_TIMEOUT,
        SC_AP_FULL,
        SC_UNSUP_RATES,
        SC_UNSUP_SHORT_SLOT_TIME = 25,
        SC_UNSUP_ER_PBCC = 26,
        SC_UNSUP_DSSS_OFDM = 27
} StatusCode;


typedef enum
{
        ST_ASOC_REQ 	= 0x00,
        ST_ASOC_RSP 	= 0x10,
        ST_REASOC_REQ 	= 0x20,
        ST_REASOC_RSP 	= 0x30,
        ST_PROBE_REQ 	= 0x40,
        ST_PROBE_RSP 	= 0x50,
        ST_BEACON 	= 0x80,
        ST_ATIM 	= 0x90,
        ST_DISASOC 	= 0xA0,
        ST_AUTH 	= 0xB0,
        ST_DEAUTH 	= 0xC0,
        ST_PS_POLL 	= 0xA4,
        ST_RTS 		= 0xB4,
        ST_CTS 		= 0xC4,
        ST_ACK 		= 0xD4,
        ST_CFEND 	= 0xE4,
        ST_CFEND_ACK 	= 0xF4,
        ST_DATA 	= 0x08,
        ST_DATA_ACK 	= 0x18,
        ST_DATA_POLL 	= 0x28,
        ST_DATA_POLL_ACK= 0x38,
        ST_NULL_FRAME 	= 0x48
} TypeSubtype;


typedef struct TrafficMap_s
{
        U8	t[(MAX_AID/8)+1];
}
TrafficMap_t;


#define RATEARRAY_NUM		16
typedef struct Hash_s
{
        struct Hash_s *pNext;
        U8	mac[6];
        StationState asoc;
        StationState auth;
        PsMode	psm;
        U16  aid;
        BOOLEAN	bValid;
        BOOLEAN	bErpSta;
        BOOLEAN bJustRiseRate;
        U8	lsInterval;
        U8	RxRate;//entry;
        U8	encryMode;
        U8	keyLength;
        U8	pkInstalled;
        U8	ZydasMode;
        U8	AlreadyIn;
        U8	MaxRate;
        U8	CurrTxRate;
        U8	ContSuccFrames;
        U8	Preamble;
        U8	KeyId;
        U16 iv16;
        U32 iv32;
        U32 ttl;
        U32 SuccessFrames;
        U32 FailedFrames;
        U8	RateArray[RATEARRAY_NUM];// this array is the rate adaption table
        U8	RateArrayCount;
        U8	SupportRateArray[RATEARRAY_NUM];
        U8	SupportRateArrayCount;
        U8	RiseConditionCount;
        U8	DownConditionCount;
        U8	keyContent[16];
        U8	wepIv[4];
        U8	vapId;
        Seedvar	TxSeed;
        Seedvar	RxSeed;
        MICvar	TxMicKey;
        MICvar	RxMicKey;
#ifdef HOSTAPD_SUPPORT

        Element WPAIE;
#endif
}
Hash_t;


typedef struct Frame_s
{
        U16		HdrLen;
        U16		bodyLen;
        U8		header[32];	//include IV, eIV
        U8		*body;
        void	*fragBuf;
}
Frame_t;

typedef struct FrmDesc_s
{
        struct	FrmDesc_s *pNext;
        U8		buffer[160]; 			//use mbuf to send boradcast
        Frame_t mpdu[MAX_FRAG_NUM];		//for fragment
        BOOLEAN bValid;
        U8		ConfigSet;
        U8		signalStrength;
        U8		signalQuality;
        U8 		CalMIC[MIC_LNG+1];				//1~8 for MIC, 9==TRUE ,if used.
        Hash_t	*pHash;
        //U8		HwMicPhys[12];			// MIC valuse(8). MIC status(4)
}
FrmDesc_t;

#define MAX_COUNTRY_INFO_SIZE  50
typedef struct BssInfo_s
{
        MacAddr_t	bssid;
        U16		bcnInterval;
        U16		cap;
        Element		ssid;
        Element		supRates;
        Element		Phpm;
        Element		IbssParms;
        Element		erp;
        Element		extRates;
        Element		country;
        U8		signalStrength;
        U8		signalQuality;
        U8  		apMode;
        U16		basicRateMap;
        U16		supRateMap;
        U8		WPAIe[128];
        U8		RSNIe[128];

}
BssInfo_t;

#endif
