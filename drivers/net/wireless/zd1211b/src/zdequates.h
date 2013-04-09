/****************************************************************************
** COPYRIGHT (C) 2002 ZYDAS CORPORATION                                    **
** HTTP://WWW.ZYDAS.COM.TW/                                                **
****************************************************************************/

#ifndef _ZDEQUATES_H
#define _ZDEQUATES_H


//-------------------------------------------------------------------------
// Ethernet Frame_t Sizes
//-------------------------------------------------------------------------
#define ETHERNET_ADDRESS_LENGTH         6
#define ETHERNET_HEADER_SIZE            14
#define MINIMUM_ETHERNET_PACKET_SIZE    60
#define MAXIMUM_ETHERNET_PACKET_SIZE    1514
#if 1//fMERGE_RX_FRAME
    #define	MAX_WLAN_SIZE				4800
    #define ZD_MAX_WLAN_SIZE			4800
#else
    #define	MAX_WLAN_SIZE               2432
    #define ZD_MAX_WLAN_SIZE			1600
#endif

#define WLAN_HEADER						24
#define CRC32_LEN						4
#define IV_SIZE							4
#define ICV_SIZE						4

#if defined(HOST_IF_USB)
#define PLCP_HEADER						5
#define EXTRA_INFO_LEN					5 	//8 for ZD1212
#else


#define PLCP_HEADER						4
#define EXTRA_INFO_LEN					4
#endif

#define EXTEND_IV_LEN					4
#define MIC_LENGTH						8
#define WDS_ADD_HEADER					6
#define EAPOL_TYPE						0x888e

#define BCN_INTERVAL_OFFSET				8
#define CAP_OFFSET						10
#define SSID_OFFSET						12
#define	NUM_SUPPORTED_RATE				32

#define BSS_INFO_NUM					64

#define TUPLE_CACHE_SIZE				16
#define MAX_DEFRAG_NUM					8


#if defined(HOST_IF_USB)
    #define MAX_RX_TIMEOUT				(100)
#else
    #define MAX_RX_TIMEOUT				(512*10*1000)
#endif

#define MAXIM2_MAX_TX_PWR_SET       	0x7F
#define MAXIM2_MIN_TX_PWR_SET       	0x0
#define RFMD_MAX_TX_PWR_SET         	0xF0
#define RFMD_MIN_TX_PWR_SET        		0x60

#define	GCT_MAX_TX_PWR_SET				0x3f
#define	GCT_MIN_TX_PWR_SET				0x0

#define	AL2210_MAX_TX_PWR_SET			0xff
#define	AL2210_MIN_TX_PWR_SET			0x80

#define MAX_TX_PWR_READING              0xf0
#define MIN_TX_PWR_READING              0x30

#define AL2230_MAX_TX_PWR_SET       	(0x7F-1)
#define AL2230_MIN_TX_PWR_SET       	(0x00+1)


#define	TRACKING_NUM					20//10


//for USB
#define cTX_CCK                     	1
#define cTX_OFDM                    	2       // 6M - 36M
#define cTX_48M                     	3
#define cTX_54M                     	4
#define cPWR_CTRL_GUARD             	4       // CR57: 4 -> 0.5 dB
#define cPWR_INT_VALUE_GUARD        	8       // CR31: 4 -> 1 dB; 8 -> 2 dB
#define cPWR_STRONG_SIG_DROP        	(0x18 - cPWR_INT_VALUE_GUARD)


#define cLBTEST_COUNT               	1000
#define cLBTEST_PATN                	0x55


#define cMAX_MULTI_WRITE_REG_NUM    	15
#define cMIN_MULTI_WRITE_REG_NUM    	0
#define cMAX_MULTI_RF_REG_NUM       	28
#define cMAX_MULTI_READ_REG_NUM     	15


/* Firmware Interface */
#define cTX_QUEUE_LEN               	4
// make sure already Tx by HMAC (for UMAC System)
// 1.Host->UMAC, 2.In UMAC Queue, 3.HMAC Sent
#define cTX_SENT_LEN                	(cTX_QUEUE_LEN + 2)
#define cFIRMWARE_OLD_ADDR          	0xEC00
#define cFIRMWARE_START_ADDR        	0xEE00
#define cFIRMWARE_EXT_CODE          	0x1000
#define cADDR_ENTRY_TABLE           	(cFIRMWARE_START_ADDR + 0x1D)
#define cBOOTCODE_START_ADDR        	0xF800
#define cINT_VECT_ADDR              	0xFFF5
#define cEEPROM_SIZE                	0x800   // 2k word (4k byte)


// in word (16 bit width)
#define cLOAD_CODE_LEN              	0xE
#define cLOAD_VECT_LEN              	(0x10000 - 0xFFF7)
#define cEPDATA_OFFSET              	(cLOAD_CODE_LEN + cLOAD_VECT_LEN)
#define USB_BASE_ADDR_EEPROM        	0x9900
#ifdef ZD1211B
	#define USB_BASE_ADDR_HOST         	0x9F00
#elif defined(ZD1211)
	#define USB_BASE_ADDR_HOST			0x9B00
#else
	#error	"***** You Need To Specified ZD1211 or ZD1211B *****"
#endif
#define BASE_ADDR_MASK_HOST         	(~0x00FF)
#define cFIRMWARE_EEPROM_OFFSET     	(cBOOTCODE_START_ADDR + cEPDATA_OFFSET)
//end of USB


// For Rate Adaption
#define FALL_RATE						0x0
#define	RISE_RATE						0x1

#define PS_CAM							0x0
#define	PS_PSM							0x1

#define	ACC_1							0x0
#define	ACC_2							0x1

// MAC_PA_STATE
#define	MAC_INI						0x0
#define	MAC_OPERATION					0x1
#define MAC_PS_OPERATION                                0x1
#define MAC_PS_SLEEP					0x2



// RF TYPE
#define UW2451_RF					0x2
#define uChip_RF					0x3
#define	AL2230_RF					0x4
#define	AL2210MPVB_RF				0x4
#define AL7230B_RF					0x5 //a,b,g RF
#define	THETA_RF					0x6
#define	AL2210_RF					0x7
#define	MAXIM_NEW_RF				0x8
#define	GCT_RF						0x9
#define	AL2230S_RF                  0xA
#define	RALINK_RF					0xB
#define	INTERSIL_RF					0xC
#define	RFMD_RF						0xD


#define	MAXIM_NEW_RF2				0xE
#define	PHILIPS_RF					0xF


#define ELEID_SSID					0


#define	ELEID_SUPRATES				1
#define ELEID_DSPARMS				3
#define ELEID_TIM					5
#define ELEID_ERP_INFO				42
#define ELEID_EXT_RATES				50
//-------------------------------------------------------------------------
//- Miscellaneous Equates
//-------------------------------------------------------------------------
#ifndef FALSE
#define FALSE       0
#define TRUE        1
#endif

#define DRIVER_NULL ((u32)0xffffffff)

//-------------------------------------------------------------------------
// Bit Mask definitions
//-------------------------------------------------------------------------
#define BIT_0       		0x0001
#define BIT_1       		0x0002
#define BIT_2       		0x0004
#define BIT_3       		0x0008
#define BIT_4       		0x0010
#define BIT_5       		0x0020
#define BIT_6       		0x0040
#define BIT_7       		0x0080
#define BIT_8       		0x0100
#define BIT_9       		0x0200
#define BIT_10      		0x0400
#define BIT_11      		0x0800
#define BIT_12      		0x1000
#define BIT_13      		0x2000
#define BIT_14      		0x4000
#define BIT_15      		0x8000
#define BIT_16      		0x00010000
#define BIT_17      		0x00020000
#define BIT_18      		0x00040000
#define BIT_19      		0x00080000
#define BIT_20      		0x00100000
#define BIT_21				0x00200000
#define BIT_22				0x00400000
#define BIT_23				0x00800000
#define BIT_24      		0x01000000
#define BIT_25				0x02000000
#define BIT_26				0x04000000
#define BIT_27				0x08000000
#define BIT_28      		0x10000000
#define BIT_29      		0x20000000
#define BIT_30      		0x40000000
#define BIT_31      		0x80000000

#define BIT_0_1				0x0003
#define BIT_0_2     		0x0007
#define BIT_0_3     		0x000F
#define BIT_0_4    			0x001F
#define BIT_0_5     		0x003F
#define BIT_0_6     		0x007F
#define BIT_0_7     		0x00FF
#define BIT_0_8    			0x01FF
#define BIT_0_13    		0x3FFF
#define BIT_0_15    		0xFFFF
#define BIT_1_2    			0x0006
#define BIT_1_3     		0x000E
#define BIT_2_5     		0x003C
#define BIT_3_4     		0x0018
#define BIT_4_5     		0x0030
#define BIT_4_6     		0x0070
#define BIT_4_7     		0x00F0
#define BIT_5_7     		0x00E0
#define BIT_5_9     		0x03E0
#define BIT_5_12    		0x1FE0
#define BIT_5_15    		0xFFE0
#define BIT_6_7     		0x00c0
#define BIT_7_11    		0x0F80
#define BIT_8_10    		0x0700
#define BIT_9_13    		0x3E00
#define BIT_12_15   		0xF000

#define BIT_16_20   		0x001F0000
#define BIT_21_25   		0x03E00000
#define BIT_26_27   		0x0C000000

#define RANDOM				0x0
#define INCREMENT			0x1



// Device Bus-Master (Tx) state
#define TX_IDLE					0x00
#define TX_READ_TBD				0x01
#define TX_READ_DATA0			0x02
#define TX_READ_DATA1			0x03
#define TX_CHK_TBD_CNT			0x04
#define TX_READ_TCB				0x05
#define TX_CHK_TCB				0x06
#define TX_WAIT_DATA			0x07
#define TX_RETRYFAILURE			0x08
#define TX_REDOWNLOAD			0x09

// Device Bus-Master (Rx) state
#define RX_IDLE					0x00
#define RX_WAIT_DATA			0x10
#define RX_DATA0				0x20
#define RX_DATA1				0x30
#define RX_READ_RCB				0x50

#define RX_CHK_RCB				0x60
#define RX_WAIT_STS				0x70
#define RX_FRM_ERR				0x80
#define RX_CHK_DATA				0x90


#define MAX_SSID_LEN			32

#define HOST_PEND				BIT_31
#define CAM_WRITE				BIT_31
#define MAC_LENGTH				6
#define RX_MIC_ERROR_IND	    BIT_4 // Bit4 of ExtraInfo[6], its Bit3-Bit0 indicates the encryption type.
#define RX_HW_MIC_ENABLE	    BIT_25 // The subfield of ZD_SnifferOn

#define MIC_FINISH				BIT_0

#define RX_MIC_ERROR_IND		BIT_4
#define HW_MIC_ENABLE			BIT_25


enum Frame_Control_Bit {
        TO_DS = BIT_0,
        FROM_DS = BIT_1,
        MORE_FRAG = BIT_2,
        RETRY_BIT = BIT_3,
        PWR_BIT = BIT_4,
        MORE_DATA = BIT_5,
        ENCRY_BIT = BIT_6,
        ODER_BIT = BIT_7

};



#define MAX_USER				40
#define MAX_KEY_LENGTH			16
#define ENCRY_TYPE_START_ADDR	60
#define DEFAULT_ENCRY_TYPE		65
#define KEY_START_ADDR			66
#define STA_KEY_START_ADDR		386
#define COUNTER_START_ADDR		418
#define STA_COUNTER_START_ADDR	423

#define EXTENDED_IV				BIT_5
#define QoS_DATA				BIT_7
#define TO_DS_FROM_DS			BIT_0_1

#define AP_MODE					BIT_24
#define IBSS_MODE				BIT_25
#define POWER_MNT				BIT_26
#define STA_PS					BIT_27

#define NON_ERP_PRESENT_BIT		BIT_0

#define USE_PROTECTION_BIT		BIT_1
#define BARKER_PREAMBLE_BIT		BIT_2

#define HOST_BIG_ENDIAN			BIT_0

#define MEMBER_ZD1202			BIT_0
#define MEMBER_OTHERS			BIT_1
#define	BEACON_TIME				1
#define REG_MAX_WAIT			500

//for UART support

/*
 * These are the definitions for the Line Control Register
 */
#define UART_LCR_SBC			BIT_6	/* Set break control */
#define UART_LCR_DLAB			BIT_7	/* Divisor latch access bit */

/*
 * These are the definitions for the Line Status Register
 */
#define UART_LSR_DR				BIT_0	/* Receiver data ready */
#define UART_LSR_OE				BIT_1	/* Overrun error indicator */
#define UART_LSR_BI				BIT_4	/* Break interrupt indicator */
#define UART_LSR_THRE			BIT_5	/* Transmit-hold-register empty */
#define UART_LSR_TEMT			BIT_6	/* Transmitter empty */

/*
 * These are the definitions for the Interrupt Identification Register
 */
#define UART_IIR_ID_MASK		0x0E	/* Mask for the interrupt ID */
#define UART_IIR_MSI			0x00	/* Modem status interrupt */
#define UART_IIR_NO_INT			BIT_0	/* No interrupts pending */
#define UART_IIR_THRI			BIT_1	/* Transmitter holding register empty */
#define UART_IIR_RDI			BIT_2	/* Receiver data interrupt */
#define UART_IIR_RLSI			0x06	/* Receiver line status interrupt */
#define UART_IIR_RX_TIMEOUT		0x0C	/* Rx timeout interrupt */

/*


 * These are the definitions for the Interrupt Enable Register
 */
#define UART_IER_RDI			BIT_0	/* Enable receiver data interrupt */
#define UART_IER_THRI			BIT_1	/* Enable Transmitter holding register int. */
#define UART_IER_RLSI			BIT_2	/* Enable receiver line status interrupt */
#define UART_IER_MSI			BIT_3	/* Enable Modem status interrupt */

/*
 * These are the definitions for the Modem Control Register
 */
#define UART_MCR_DTR			BIT_0	/* DTR complement */
#define UART_MCR_RTS			BIT_1	/* RTS complement */
#define UART_MCR_TAFC			BIT_5	/* Tx auto flow control */
#define UART_MCR_RAFC			BIT_6	/* Rx auot flow control */

/*
 * These are the definitions for the Modem Status Register
 */
#define UART_MSR_DCTS			BIT_0	/* Delta CTS */
#define UART_MSR_DDCD			BIT_3	/* Delta DCD */
#define UART_MSR_CTS			BIT_4	/* Clear to Send */
#define UART_MSR_DCD			BIT_7	/* Data Carrier Detect */
#define UART_MSR_ANY_DELTA 		0x09	/* Any of the delta bits! */

/*
 * These are the definitions for the FIFO Control Register
 *
 */
#define UART_FCR_CLEAR_RCVR		BIT_1 	/* Clear the RCVR FIFO */
#define UART_FCR_CLEAR_XMIT		BIT_2 	/* Clear the XMIT FIFO */
#define UART_FCR_TX_TRIGGER_1	0x00 /* Mask for tx trigger set at 1 */
#define UART_FCR_TX_TRIGGER_2	0x10 /* Mask for tx trigger set at 2 */
#define UART_FCR_TX_TRIGGER_4	0x20 /* Mask for tx trigger set at 4 */
#define UART_FCR_TX_TRIGGER_8	0x30 /* Mask for tx trigger set at 8 */
#define UART_FCR_RX_TRIGGER_1	0x00 /* Mask for rx trigger set at 1 */
#define UART_FCR_RX_TRIGGER_4	0x40 /* Mask for rx trigger set at 4 */
#define UART_FCR_RX_TRIGGER_8	0x80 /* Mask for rx trigger set at 8 */
#define UART_FCR_RX_TRIGGER_14	0xC0 /* Mask for rx trigger set at 14 */


//Extra Control register
#define UART_ECR_BREAK_LEN_11_BAUD	0x00 	/* Break length 11 baud clocks */
#define UART_ECR_BREAK_LEN_12_BAUD	0x01 	/* Break length 12 baud clocks */
#define UART_ECR_BREAK_LEN_23_BAUD	0x02 	/* Break length 23 baud clocks */
#define UART_ECR_BREAK_LEN_25_BAUD	0x03 	/* Break length 25 baud clocks */
#define UART_ECR_BREAK_DETECTION 	BIT_2	/* Break detection */
#define UART_ECR_BREAK_ENABLE 		BIT_3	/* Break enable */
#define UART_ECR_RX_TOUT_10_BAUD	0x00	/* Rx fifo timeout 10 baud cloacks */
#define UART_ECR_RX_TOUT_20_BAUD	0x10	/* Rx fifo timeout 20 baud cloacks */
#define UART_ECR_RX_TOUT_40_BAUD	0x20	/* Rx fifo timeout 40 baud cloacks */
#define UART_ECR_RX_TOUT_80_BAUD	0x30	/* Rx fifo timeout 80 baud cloacks */
#define UART_ECR_ENABLE 			BIT_7	/* UART enable */

//Baud rate definition
#define	BAUD_RATE_2400			0xBF
#define BAUD_RATE_4800 			0x5F
#define BAUD_RATE_9600 			0x2F
#define BAUD_RATE_19200 		0x17
#define BAUD_RATE_38400			0x0B
#define BAUD_RATE_57600 		0x07
#define BAUD_RATE_115200		0x03
#define BAUD_RATE_230400		0x01
#define BAUD_RATE_460800		0x00

// for EEPROM support
#define EEPROM_BASE_ADDRESS		0x900
#define EEPROM_WRITE_ACCESS		BIT_15
#define EEPROM_BUSY_FLAG		BIT_15
#define EEPROM_ACCESS_WRITE		0x01
#define EEPROM_ACCESS_READ		0x00


#define mBIT(b)                 (1 << (b))
#define mMASK(w)                (mBIT(w) - 1)
#define mSET_MASK(a, b)         ((a) | (b))
#define mCLR_MASK(a, b)         ((a) & (~(b)))
#define mSET_BIT(a, b)          mSET_MASK(a, mBIT(b))
#define mCLR_BIT(a, b)          mCLR_MASK(a, mBIT(b))
#define mCHK_BIT1(a, b)         ((a) & mBIT(b))
#define mTEST_BIT(a, b)         mCHK_BIT1(a, b)


#endif      // _EQUATES_H
