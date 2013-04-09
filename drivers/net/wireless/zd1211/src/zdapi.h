#ifndef _ZDAPI_H_
#define _ZDAPI_H_

#include "zdtypes.h"
#include "zdsorts.h"
//#define HMAC_DEBUG

#ifdef HMAC_DEBUG
#define FPRINT(string)			printk(KERN_DEBUG "%s\n", string)
#define FPRINT_V(string, i)		printk(KERN_DEBUG "%s = %x\n", string, i)
#else
#define FPRINT(string)
#define FPRINT_V(string, i)
#define VerAssert(x)	printk("Maybe Unsupport mode in %s(%d)",__FILE__,__LINE__);
#define LongPrint(x,y)	{int i;for(i=0;i<100*y;i++) printk("%s",x);}
#endif


//#define ZD_DEBUG 	//debug protocol stack

#ifdef ZD_DEBUG
#define ZDEBUG(string)			FPRINT(string)
#define ZDEBUG_V(string, i)		FPRINT_V(string, i)
#else
#define ZDEBUG(string) 			//do {} while (0)
#define ZDEBUG_V(string, i)		//do {} while (0)
#endif

//#define PS_DEBUG		//debug power save function


#ifdef PS_DEBUG
#define PSDEBUG(string)			FPRINT(string)
#define PSDEBUG_V(string, i)	FPRINT_V(string, i)
#else
#define PSDEBUG(string) 		//do {} while (0)
#define PSDEBUG_V(string, i)	//do {} while (0)
#endif


//#define HASH_DEBUG	//debug hash function

#ifdef HASH_DEBUG
#define HSDEBUG(string)			FPRINT(string)
#define HSDEBUG_V(string, i)	FPRINT_V(string, i)
#else
#define HSDEBUG(string) 		//do {} while (0)
#define HSDEBUG_V(string, i)	//do {} while (0)
#endif

//#define RATE_DEBUG	//debug rate adaption function

#ifdef RATE_DEBUG
#define RATEDEBUG(string)		FPRINT(string)
#define RATEDEBUG_V(string, i)	FPRINT_V(string, i)
#else
#define RATEDEBUG(string) 		//do {} while (0)
#define RATEDEBUG_V(string, i)	//do {} while (0)
#endif


//#define DEFRAG_DEBUG	//debug defrag function

#ifdef DEFRAG_DEBUG
#define DFDEBUG(string)			FPRINT(string)
#define DFDEBUG_V(string, i)	FPRINT_V(string, i)
#else
#define DFDEBUG(string) 		//do {} while (0)
#define DFDEBUG_V(string, i)	//do {} while (0)
#endif


/* ath_desc: bigendian support */
/* ath: use cpu_to_le32 instead of zd_cpu_to_le32 */
/* ath: deleted zd_cpu_to_le32 definition */

#define CMD_RESET_80211			0x0001	//parm1: zd_80211Obj_t *
#define CMD_ENABLE				0x0002	//parm1: None
#define CMD_DISASOC				0x0003	//parm1: U8 *MacAddress, parm2: reasonCode
#define CMD_DEAUTH				0x0004	//parm1: U8 *MacAddress, parm2: reasonCode
#define CMD_PS_POLL				0x0005	//parm1: U8 *MacHeader
#define CMD_PASSIVE_SCAN		0x0006	//parm1: None
#define CMD_DISASOC_ALL			0x0007	//parm1: U8 *MacAddress, parm2: reasonCode
#define CMD_CONNECT				0x0008	//parm1: None, parm2: BssIndex
#define CMD_PROBE_REQ			0x0009	//parm1: None, parm2: WithSSID
#define CMD_DIS_CONNECT			0x000A
#define	CMD_FLUSH_QUEUE			0x000B
#define	CMD_ROAMING             0x000C


//Event Notify

#define EVENT_TBCN				0x0010
#define EVENT_DTIM_NOTIFY		0x0011
#define EVENT_TX_COMPLETE		0x0012 //parm1: tx status, parm2: msgId, parm3: aid
#define EVENT_TCHAL_TIMEOUT		0x0013
#define EVENT_SCAN_TIMEOUT		0x0014
#define EVENT_UPDATE_TX_RATE	0x0015 //parm1: rate, parm2: aid
#define EVENT_SW_RESET          0x0016
#define EVENT_BUF_RELEASE       0x0017
#define EVENT_ENABLE_PROTECTION	0x0018
#define EVENT_AUTH_TIMEOUT		0x0019
#define EVENT_ASOC_TIMEOUT		0x001A
#define EVENT_PS_CHANGE			0x001B //parm1: PwrState
#define EVENT_MORE_DATA			0x001C
#define EVENT_ENABLE_BARKER     0x001D
#define EVENT_SHORT_SLOT        0x001E


#define DO_CHAL					0x0001
#define DO_SCAN					0x0002
#define DO_AUTH					0x0003
#define DO_ASOC					0x0004

#define SCAN_TIMEOUT			50   //ms
#define HOUSE_KEEPING_PERIOD	100	 //ms
#define AUTH_TIMEOUT			512  //3000 //ms
#define ASOC_TIMEOUT			512  //2000 //ms


//reason code
#define ZD_UNSPEC_REASON 		1
#define ZD_AUTH_NOT_VALID		2
#define ZD_DEAUTH_LEAVE_BSS		3

#define ZD_INACTIVITY			4
#define ZD_AP_OVERLOAD			5
#define ZD_CLASS2_ERROR			6
#define ZD_CLASS3_ERROR			7
#define ZD_DISAS_LEAVE_CSS		8
#define ZD_ASOC_NOT_AUTH		9
#define ZD_INVALID_IE			13
#define ZD_MIC_FAIL				14
#define ZD_4WAY_SHAKE_TIMEOUT	15
#define ZD_GKEY_UPDATE_TIMEOUT	16
#define ZD_IE_IMCOMPABILITY		17
#define ZD_MC_CIPHER_INVALID	18
#define ZD_UNI_CIPHER_INVALID	19
#define ZD_AKMP_INVALID			20
#define ZD_UNSUP_RSNE_VERSION	21
#define ZD_INVALID_RSNE_CAP		22
#define ZD_8021X_AUTH_FAIL		23


/* association_status_notify() <- status */
#define STA_ASOC_REQ			0x0001
#define STA_REASOC_REQ			0x0002
#define STA_ASSOCIATED			0x0003
#define STA_REASSOCIATED		0x0004
#define STA_DISASSOCIATED		0x0005
#define STA_AUTH_REQ			0x0006
#define STA_DEAUTHED			0x0007

//Tx complete event
#define ZD_TX_CONFIRM			0x0001
#define ZD_RETRY_FAILED			0x0002


//for Dymanic Key
#define DYN_KEY_WEP64			1
#define DYN_KEY_WEP128			2
#define DYN_KEY_TKIP			4
#define DYN_KEY_AES			    5


//Rate Defintion
#define RATE_1M		            0
#define	RATE_2M		            1
#define	RATE_5M		            2
#define	RATE_11M	            3
#define	RATE_16M	            4

#define	RATE_22M	            5
#define	RATE_27M	            6
#define RATE_33M		        7
#define	RATE_38M		        8
#define	RATE_44M		        9
#define	RATE_49M	            10
#define	RATE_55M	            11
#define	RATE_60M	            12

#define	RATE_6M	                0x04
#define	RATE_9M	                0x05
#define	RATE_12M	            0x06
#define	RATE_18M	            0x07
#define	RATE_24M	            0x08
#define	RATE_36M	            0x09
#define	RATE_48M	            0x0a
#define	RATE_54M	            0x0b


#define NO_WEP                  0x0
#define AES                     0x4
#define WEP64                   0x1
#define WEP128                  0x5
#define WEP256                  0x6
#define TKIP                    0x2



#define	NUM_SUPPORTED_RATE	    32

// pfrmDesc->ConfigSet
#define INTRA_BSS_SET			0x01
#define EAPOL_FRAME_SET			0x02
#define FORCE_WEP_SET			0x04

//BssType
#define INDEPENDENT_BSS			0x0
#define INFRASTRUCTURE_BSS		0x1
#define PSEUDO_IBSS				0x3
#define	AP_BSS					0x4

//RxFilter
#define AP_RX_FILTER			0x0400feff
#define STA_RX_FILTER			0x0000ffff

//pSetting->MacMode
#define MIXED_MODE		        0x01
#define PURE_G_MODE		        0x02
#define PURE_B_MODE 	        0x03
#define PURE_A_MODE				0x04



#define CW_SHORT_SLOT		    0x7f043f
#define CW_NORMAL_SLOT		    0xff043f
#define CW_LONG_SLOT            0x7f047f



//for Rate Adaption
#define RISE_CONDITION_THRESHOLD	5
#define DOWN_CONDITION_THRESHOLD	3
#define	LINE1				100
#define	LINE2				10
#define	LINE3				5


//for CAM
#define CAM_VAP_START_AID	33
#define CAM_VAP_END_AID		39


#define HW_MIC_FINISH		0x55555555
#define CAM_ADDR_NOT_MATCH	40

#define PURE_B_AP       0
#define MIXED_AP        1
#define PURE_G_AP       2
#define PURE_A_AP			3


enum Operation_Mode {
        CAM_IBSS = 0,
        CAM_AP,
        CAM_STA,
        CAM_AP_WDS,
        CAM_AP_CLIENT,
        CAM_AP_VAP
};


typedef struct card_Setting_s
{
        U8		EncryOnOff;		//0: encryption off, 1: encryption on
        U8		OperationMode;	//0: IBSS, 1: AP, 2: STA, 3: WDS, 4: AP Client, 5: Virtual AP
        U8		PreambleType;	//0: long preamble, 1: short preamble
        U8		TxRate;			//0: 1M, 1: 2M, 2: 5.5M, 3: 11M, 4: 16.5M
        U8		FixedRate;		// fixed Tx Rate
        U8		CurrTxRate;		//
        U8		AuthMode;		//0: open system only, 1: shared key only, 2: auto
        U8		HiddenSSID;		//0: disable, 1:enable
        U8		LimitedUser;	//limited client number max to 32 user
        U8		RadioOn;		//0: radio off, 1: radio on
        U8		BlockBSS;		//0: don't block intra-bss traffic, 1: block
        U8		TxPowerLevel;	//0: 17dbm, 1: 14dbm, 2: 11dbm
        U8		BasicRate;		//
        U8		EncryMode;		//0: no wep, 2: wep63, 3:wep128
        U8		EncryKeyId;		//encryption key id
        U8		BcKeyId;		//broadcast key id for dynamic key
        U8		SwCipher;		//
        U8		WepKeyLen;		//WEP key length
        U8		BcKeyLen;		//Broadcast key length
        U8		DynKeyMode;		//Dynamic key mode, 1: WEP64, 2: WEP128, 4:TKIP
        U16		Channel;		//channel number
        U16		FragThreshold;	//fragment threshold, from 256~2432
        U16		RTSThreshold;	//RTS threshold, from 256~2432

        U16		BeaconInterval;	//default 100 ms
        U16		DtimPeriod;		//default 1
        U8		MacAddr[8];
        // ElementID(1), Len(1), SSID
        U8		Info_SSID[36];	//include element ID, element Length, and element content
        // ElementID(1), Len(1), SupportedRates(1-8)
        U8		Info_SupportedRates[NUM_SUPPORTED_RATE];	//include element ID, element Length, and element content
        U8		keyVector[4][32];
        U8		BcKeyVector[16];
        U8		WPAIe[128];
        U8		WPAIeLen;
        U8		WPASupport;
        U8		Rate275;
        U8		WpaBcKeyLen;
        U8		BssType;
        U16		ATIMWindow;

        //added for G
        U8		Ext_SupportedRates[NUM_SUPPORTED_RATE];
        U8		MacMode;
        U8		ShortSlotTime;
        U8		BarkerPreamble;
        // for UART support
        //U8		UartEnable;
        //U8		BaudRate;

        U8		LastSentTxRate;
        U8		ap_scan;
#ifdef OFDM

        U8		HighestTxRate;
#endif

}
card_Setting_t;


#define ZD_MAX_FRAG_NUM		8

typedef struct fragInfo_s
{
        U8	*macHdr[ZD_MAX_FRAG_NUM];
        U8	*macBody[ZD_MAX_FRAG_NUM];
        U32	bodyLen[ZD_MAX_FRAG_NUM];
        U32	nextBodyLen[ZD_MAX_FRAG_NUM];
        U8	hdrLen;
        U8	totalFrag;
        U8	bIntraBss;
        U8	msgID;
        U8	rate;
        U8	preamble;
        U8	encryType;
        U8	burst;
        U16 	vapId;
        U16 	aid;
        U8 	CalSwMic[MIC_LNG+1];
        //U8	keyInstalled;
        //U8	bWaitingMIC;
        //U8	bSwCalcMIC;
        //U32	HwMicPhys;


        void *buf;

}
fragInfo_t;


typedef struct rxInfo_s
{
        U8	rate;
        U8	bDataFrm;
        U8	SaIndex;
        U8	signalStrength;
        U8	signalQuality;
        U8	bSwCheckMIC;
}
rxInfo_t;


typedef struct bss_info_s
{
        U8	bssid[6];
        U16	beaconInterval;
        U16	cap;
        U16	atimWindow;
        U8	ssid[36];
        U8	supRates[NUM_SUPPORTED_RATE];
        U8	extRates[NUM_SUPPORTED_RATE];
        U8	WPAIe[128];
        U8	RSNIe[128];
        U8	channel;
        U8	signalStrength;
        U8 	signalQuality;
        U8	apMode;
}
bss_info_t;


//for pdot11Obj->QueueFlag
#define TX_QUEUE_SET				0x01
#define MGT_QUEUE_SET				0x02
#define	AWAKE_QUEUE_SET				0x04


//for mRequestFlag
#define CONNECT_TOUT_SET			0x0001
#define DIS_CONNECT_SET				0x0002
#define BSS_CONNECT_SET             0x0004
#define CHANNEL_SCAN_SET            0x0008
#define PS_CHANGE_SET				0x0010
#define PS_POLL_SET					0x0020
#define IBSS_CONNECT_SET			0x0040
#define ROAMING_SET                 0x0080


//for pdot11Obj->ConfigFlag
#define ENABLE_PROTECTION_SET 		0x0001
#define BARKER_PREAMBLE_SET 		0x0002
#define SHORT_SLOT_TIME_SET         0x0004
#define NON_ERP_PRESENT_SET         0x0008
#define PASSIVE_CHANNEL_SCAN_SET	0x0010
#define ACTIVE_CHANNEL_SCAN_SET		0x0020
#define IBSS_CHANNEL_SCAN_SET		0x0040
#define SCAN_AND_CONNECT_SET        0x0080
#define JUST_CHANNEL_SCAN           0x1000

// Feature Bit Map
#define	FBM_ANTTENA_DIVERSITY		0x00000001
#define	FBM_802_11D					0x00000002
#define	FBM_27_5_MBPS				0x00000004
#define	FBM_SINGLE_LED				0x00000008

// Define debug command
#define DBG_CMD_BEACON				0x0001

//driver to provide callback functions for 802.11 protocol stack
typedef	struct zd_80211Obj_s
{
        void		*reg;			//Input
        U8		QueueFlag;		//Output
        U16		ConfigFlag; 		//Output
        U8		BasicRate;
        U8		bDeviceInSleep;
        U8		BssType;
        U8		bOverWritePhyRegFromE2P;
        U8		bIsNormalSize;
        U16		BeaconInterval;
        U16		Channel;
        U16		Aid;
        U32		rfMode;
        U32     HWFeature;
        U32		RegionCode;
        U32		S_bit_cnt;
        U32		AllowedChannel;
        U32		dbg_cmd;
        U8		TxGainSetting;
#if fTX_GAIN_OFDM

        U8      	TxGainSetting2;
#endif

        U8     	CR31Flag;
        U8     	CR122Flag;
        U8     	CR203Flag;
        U8	PhyTest;
        U8	IsUSB2_0;
        U8      bContinueTx;
        U8      bChScanning;
        U16    	IntValue[14];
        U8      CurrSsid[34+1];
#ifdef ZD1211B

        U8		LengthDiff;
#endif

        BOOLEAN MIC_CNT;

        void	(* ReleaseBuffer)(void *buf);							// release rx buffer
        void	(* StartTimer)(U32 timeout, U32 event);					// start a chanllege timer (shared key authentication)
        void	(* StopTimer)(U32 TimerId);								// stop the challenge timer
        void	(* RxInd)(U8 *pData, U32 length, void *buf);			// rx indication
        void	(* TxCompleted)(void);									// tx completed
        BOOLEAN	(* SetupNextSend)(fragInfo_t *pFragInfo);				// send to HMAC
        void	(* SetReg)(void *reg, U32 offset, U32 value);			// set HMAC register
        U32	(* GetReg)(void *reg, U32 offset);						// get HMAC register
        U16 	(* StatusNotify)(U16 status, U8 *StaAddr);				// association notify for bridge management
        void 	(* ExitCS)(U32 flags);								// enable interrupt
        U32	(* EnterCS)(void);								// disable interrupt
        U32 	(* Vir2PhyAddr)(U32 virtAddr);							// translate virtual address to physical address
        BOOLEAN	(* CheckTCBAvail)(U8 NumOfFrag);						// check TCB available
        void	(* DelayUs)(U16 ustime);								// delay function
        void *	(* AllocBuffer)(U16 dataSize, U8 **pData);				// allocate wireless forwarding buffer

        int	(* Rand)(U32 seed);
        void    (* AcquireDoNotSleep)(void);
        void    (* ReleaseDoNotSleep)(void);

        // wpa support
        void	(* MicFailure)(unsigned char *addr);
        int	(* AssocRequest)(U8 *addr, U8* data, U16 size);
        int 	(* WpaIe)(U8 *buffer, int length);
}
zd_80211Obj_t;


//802.11 export functions for driver use
extern void zd_SigProcess(void);									// protocol statck entry point
extern BOOLEAN zd_SendPkt(U8 *pEthHdr, U8 *pBody, U32 bodyLen, void *buf, U8 bEapol, void *pHash);	// tx request
extern void zd_ReceivePkt(U8 *pHdr, U32 hdrLen, U8 *pBody, U32 bodyLen, void *buf, U8 *pEthHdr, rxInfo_t *pRxInfo); // rx indication
extern BOOLEAN zd_CmdProcess(U16 CmdId, void *parm1, U32 parm2);	//command process
extern void zd_EventNotify(U16 EventId, U32 parm1, U32 parm2, U32 parm3);		//event notify
extern void zd_UpdateCardSetting(card_Setting_t *pSetting);
extern BOOLEAN zd_CleanupTxQ(void);
extern BOOLEAN zd_CleanupAwakeQ(void);
extern int zd_SetKeyInfo(U8 *addr, U8 encryMode, U8 keyLength, U8 KeyId, U8 *pKeyContent);
extern void zd_PerSecTimer(void);
extern BOOLEAN zd_CheckIvSeq(U8 aid, U16 iv16, U32 iv32);
extern void zd_RateMoniter(void);
extern BOOLEAN zd_QueryStaTable(U8 *sta, void **ppHash);
extern void zd_ConnectMon(void);
extern U8 zd_GetBssList(bss_info_t *pBssList);
extern U16 zd_AidLookUp(U8 *addr);
extern void zd_makeRateInfoMAP(U8 *pRates, U16 *basicRateMap, U16 *supRateMap);
extern void zd_UpdateIbssInfo(U8 *addr, U8 maxRate, U8 preamble, U8 erpSta);
#endif
