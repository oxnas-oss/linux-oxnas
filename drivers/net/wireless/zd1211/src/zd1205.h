#ifndef _ZD1205_H_
#define _ZD1205_H_

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/pci.h>

#ifdef HOST_IF_USB
	#include <linux/usb.h>
#endif

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/reboot.h>
#include <asm/io.h>
#include <asm/unaligned.h>
#include <asm/processor.h>
#include <linux/ethtool.h>
#include <linux/inetdevice.h>
#include <linux/bitops.h>
#include <linux/if.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/ip.h>
#include <linux/wireless.h>
#include <linux/if_arp.h>
#include <linux/unistd.h>
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0))
	#include <linux/workqueue.h>
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0))
    #include <asm/div64.h>
#endif

#include "zdcompat.h"
#include "zdequates.h"
#include "zdapi.h"
#include "zydas_common.h"

#include "zd1211_wext.h"

#ifdef HOST_IF_USB
#define fANT_DIVERSITY          0
#define fTX_PWR_CTRL            1
#define fZD1211_LOOPBACK        1
#define fDUMP_LOOPBACK_DATA     (0 && fZD1211_LOOPBACK)
#define fLOAD_BOOTCODE          1
#define fPROG_FLASH             (1 && fWRITE_WORD_REG)
#define fPROG_FLASH_BY_FW       (1 && fPROG_FLASH)
#define fDRV_WRITE_RF_REG       (1 && fWRITE_WORD_REG)
#define fMERGE_RX_FRAME         (ENHANCE_RX && fDRV_WRITE_RF_REG)
#define fDRV_UPDATE_EEP         (1 && fWRITE_WORD_REG)
//#define fQuickPhySet			0 //(fREAD_MUL_REG && fWRITE_WORD_REG)
#endif
enum zd1205_device_type {
        ZD_1202 = 1,
        ZD_1205,
};


#define	ASOC_RSP				0x10
#define REASOC_RSP 				0x30
#define PROBE_RSP 				0x50
#define DISASOC 				0xA0
#define AUTH 					0xB0
#define DEAUTH 					0xC0
#define DATA 					0x08
#define PS_POLL					0xA4
#define MANAGEMENT				0x00
#define PROBE_REQ				0x40
#define BEACON					0x80
#define ACK					0xD4
#define CONTROL					0x04
#define NULL_FUNCTION				0x48
#define LB_DATA					0x88

#define VLAN_SIZE   	4
#define CHKSUM_SIZE 	2

#define false		(0)
#define true		(1)



/**************************************************************************
**		Register Offset Definitions
***************************************************************************
*/

#define		ZD1205_CR0			0x0000
#define		ZD1205_CR1			0x0004
#define		ZD1205_CR2			0x0008
#define		ZD1205_CR3			0x000C
#define		ZD1205_CR5			0x0010
#define		ZD1205_CR6			0x0014
#define		ZD1205_CR7			0x0018
#define		ZD1205_CR8			0x001C
#define		ZD1205_CR4			0x0020
#define		ZD1205_CR9			0x0024
#define		ZD1205_CR10			0x0028
#define		ZD1205_CR11			0x002C
#define		ZD1205_CR12			0x0030
#define		ZD1205_CR13			0x0034
#define		ZD1205_CR14			0x0038
#define		ZD1205_CR15			0x003C
#define		ZD1205_CR16			0x0040
#define		ZD1205_CR17			0x0044
#define		ZD1205_CR18			0x0048
#define		ZD1205_CR19			0x004C
#define		ZD1205_CR20			0x0050
#define		ZD1205_CR21			0x0054
#define		ZD1205_CR22			0x0058
#define		ZD1205_CR23			0x005C



#define		ZD1205_CR24			0x0060
#define		ZD1205_CR25			0x0064
#define		ZD1205_CR26			0x0068
#define		ZD1205_CR27			0x006C
#define		ZD1205_CR28			0x0070
#define		ZD1205_CR29			0x0074
#define		ZD1205_CR30			0x0078
#define		ZD1205_CR31			0x007C
#define		ZD1205_CR32			0x0080
#define		ZD1205_CR33			0x0084
#define		ZD1205_CR34			0x0088
#define		ZD1205_CR35			0x008C
#define		ZD1205_CR36			0x0090
#define		ZD1205_CR37			0x0094
#define		ZD1205_CR38			0x0098
#define		ZD1205_CR39			0x009C
#define		ZD1205_CR40			0x00A0
#define		ZD1205_CR41			0x00A4
#define		ZD1205_CR42			0x00A8
#define		ZD1205_CR43			0x00AC
#define		ZD1205_CR44			0x00B0
#define		ZD1205_CR45			0x00B4
#define		ZD1205_CR46			0x00B8
#define		ZD1205_CR47			0x00BC
#define		ZD1205_CR48			0x00C0
#define		ZD1205_CR49			0x00C4

#define		ZD1205_CR50			0x00C8
#define		ZD1205_CR51			0x00CC
#define		ZD1205_CR52			0x00D0
#define		ZD1205_CR53			0x00D4
#define		ZD1205_CR54			0x00D8
#define		ZD1205_CR55			0x00DC
#define		ZD1205_CR56			0x00E0
#define		ZD1205_CR57			0x00E4
#define		ZD1205_CR58			0x00E8
#define		ZD1205_CR59			0x00EC
#define		ZD1205_CR60			0x00F0
#define		ZD1205_CR61			0x00F4
#define		ZD1205_CR62			0x00F8
#define		ZD1205_CR63			0x00FC
#define		ZD1205_CR64			0x0100
#define		ZD1205_CR65			0x0104
#define		ZD1205_CR66			0x0108
#define		ZD1205_CR67			0x010C
#define		ZD1205_CR68			0x0110
#define		ZD1205_CR69			0x0114
#define		ZD1205_CR70			0x0118
#define		ZD1205_CR71			0x011C
#define		ZD1205_CR72			0x0120
#define		ZD1205_CR73			0x0124
#define		ZD1205_CR74			0x0128
#define		ZD1205_CR75			0x012C
#define		ZD1205_CR76			0x0130
#define		ZD1205_CR77			0x0134
#define		ZD1205_CR78			0x0138
#define		ZD1205_CR79			0x013C
#define		ZD1205_CR80			0x0140
#define		ZD1205_CR81			0x0144
#define		ZD1205_CR82			0x0148
#define		ZD1205_CR83			0x014C
#define		ZD1205_CR84			0x0150
#define		ZD1205_CR85			0x0154
#define		ZD1205_CR86			0x0158
#define		ZD1205_CR87			0x015C
#define		ZD1205_CR88			0x0160
#define		ZD1205_CR89			0x0164

#define		ZD1205_CR90			0x0168
#define		ZD1205_CR91			0x016C
#define		ZD1205_CR92			0x0170
#define		ZD1205_CR93			0x0174
#define		ZD1205_CR94			0x0178
#define		ZD1205_CR95			0x017C
#define		ZD1205_CR96			0x0180
#define		ZD1205_CR97			0x0184
#define		ZD1205_CR98			0x0188
#define		ZD1205_CR99			0x018C
#define		ZD1205_CR100			0x0190
#define		ZD1205_CR101			0x0194
#define		ZD1205_CR102			0x0198
#define		ZD1205_CR103			0x019C
#define		ZD1205_CR104			0x01A0
#define		ZD1205_CR105			0x01A4
#define		ZD1205_CR106			0x01A8
#define		ZD1205_CR107			0x01AC
#define		ZD1205_CR108			0x01B0
#define		ZD1205_CR109			0x01B4
#define		ZD1205_CR110			0x01B8
#define		ZD1205_CR111			0x01BC
#define		ZD1205_CR112			0x01C0
#define		ZD1205_CR113			0x01C4
#define		ZD1205_CR114			0x01C8
#define		ZD1205_CR115			0x01CC
#define		ZD1205_CR116			0x01D0
#define		ZD1205_CR117			0x01D4
#define		ZD1205_CR118			0x01D8
#define		ZD1205_CR119			0x01EC
#define		ZD1205_CR120			0x01E0
#define		ZD1205_CR121			0x01E4
#define		ZD1205_CR122			0x01E8
#define		ZD1205_CR123			0x01EC
#define		ZD1205_CR124			0x01F0
#define		ZD1205_CR125			0x01F4
#define		ZD1205_CR126			0x01F8
#define		ZD1205_CR127			0x01FC
#define		ZD1205_CR128			0x0200
#define		ZD1205_CR129			0x0204
#define		ZD1205_CR130			0x0208
#define		ZD1205_CR131			0x020C
#define		ZD1205_CR132			0x0210
#define		ZD1205_CR133			0x0214
#define		ZD1205_CR134			0x0218
#define		ZD1205_CR135			0x021C
#define		ZD1205_CR136			0x0220
#define		ZD1205_CR137			0x0224
#define		ZD1205_CR138			0x0228
#define		ZD1205_CR139			0x022C
#define		ZD1205_CR140			0x0230
#define		ZD1205_CR141			0x0234
#define		ZD1205_CR142			0x0238
#define		ZD1205_CR143			0x023C
#define		ZD1205_CR144			0x0240
#define		ZD1205_CR145			0x0244
#define		ZD1205_CR146			0x0248
#define		ZD1205_CR147			0x024C
#define		ZD1205_CR148			0x0250
#define		ZD1205_CR149			0x0254
#define		ZD1205_CR150			0x0258
#define		ZD1205_CR151			0x025C
#define		ZD1205_CR152			0x0260
#define		ZD1205_CR153			0x0264
#define		ZD1205_CR154			0x0268
#define		ZD1205_CR155			0x026C
#define		ZD1205_CR156			0x0270
#define		ZD1205_CR157			0x0274
#define		ZD1205_CR158			0x0278
#define		ZD1205_CR159			0x027C
#define		ZD1205_CR160			0x0280
#define		ZD1205_CR161			0x0284
#define		ZD1205_CR162			0x0288
#define		ZD1205_CR163			0x028C
#define		ZD1205_CR164			0x0290
#define		ZD1205_CR165			0x0294



#define		ZD1205_CR166			0x0298
#define		ZD1205_CR167			0x029C
#define		ZD1205_CR168			0x02A0
#define		ZD1205_CR169			0x02A4
#define		ZD1205_CR170			0x02A8
#define		ZD1205_CR171			0x02AC
#define		ZD1205_CR172			0x02B0
#define		ZD1205_CR173			0x02B4
#define		ZD1205_CR174			0x02B8
#define		ZD1205_CR175			0x02BC
#define		ZD1205_CR176			0x02C0
#define		ZD1205_CR177			0x02C4
#define		ZD1205_CR178			0x02C8
#define		ZD1205_CR179			0x02CC
#define		ZD1205_CR180			0x02D0
#define		ZD1205_CR181			0x02D4
#define		ZD1205_CR182			0x02D8
#define		ZD1205_CR183			0x02DC
#define		ZD1205_CR184			0x02E0
#define		ZD1205_CR185			0x02E4
#define		ZD1205_CR186			0x02E8
#define		ZD1205_CR187			0x02EC
#define		ZD1205_CR188			0x02F0
#define		ZD1205_CR189			0x02F4
#define		ZD1205_CR190			0x02F8
#define		ZD1205_CR191			0x02FC
#define		ZD1205_CR192			0x0300
#define		ZD1205_CR193			0x0304
#define		ZD1205_CR194			0x0308
#define		ZD1205_CR195			0x030C
#define		ZD1205_CR196			0x0310
#define		ZD1205_CR197			0x0314
#define		ZD1205_CR198			0x0318
#define		ZD1205_CR199			0x031C
#define		ZD1205_CR200			0x0320
#define		ZD1205_CR201			0x0324
#define		ZD1205_CR202			0x0328
#define		ZD1205_CR203			0x032C
#define		ZD1205_CR204			0x0330
#define		ZD1205_CR205			0x0334
#define		ZD1205_CR206			0x0338
#define		ZD1205_CR207			0x033C
#define		ZD1205_CR208			0x0340
#define		ZD1205_CR209			0x0344
#define		ZD1205_CR210			0x0348
#define		ZD1205_CR211			0x034C
#define		ZD1205_CR212			0x0350
#define		ZD1205_CR213			0x0354
#define		ZD1205_CR214			0x0358
#define		ZD1205_CR215			0x035C
#define		ZD1205_CR216			0x0360
#define		ZD1205_CR217			0x0364
#define		ZD1205_CR218			0x0368
#define		ZD1205_CR219			0x036C
#define		ZD1205_CR220			0x0370
#define		ZD1205_CR221			0x0374

#define		ZD1205_CR222			0x0378
#define		ZD1205_CR223			0x037C
#define		ZD1205_CR224			0x0380
#define		ZD1205_CR225			0x0384
#define		ZD1205_CR226			0x0388
#define		ZD1205_CR227			0x038C
#define		ZD1205_CR228			0x0390
#define		ZD1205_CR229			0x0394
#define		ZD1205_CR230			0x0398
#define		ZD1205_CR231			0x039C

#define		ZD1205_CR232			0x03A0
#define		ZD1205_CR233			0x03A4
#define		ZD1205_CR234			0x03A8
#define		ZD1205_CR235			0x03AC
#define		ZD1205_CR236			0x03B0

#define		ZD1205_CR240			0x03C0
#define		ZD1205_CR241			0x03C4
#define		ZD1205_CR242			0x03C8
#define		ZD1205_CR243			0x03CC
#define		ZD1205_CR244			0x03D0
#define		ZD1205_CR245			0x03D4

#define		ZD1205_CR251			0x03EC
#define		ZD1205_CR252			0x03F0
#define		ZD1205_CR253			0x03F4
#define		ZD1205_CR254			0x03F8
#define		ZD1205_CR255			0x03FC


#define		ZD1205_PHY_END		0x03fc
#define		RF_IF_CLK		0x0400
#define		RF_IF_DATA		0x0404
#define		PE1_PE2			0x0408
#define		PE2_DLY			0x040C
#define		LE1			0x0410
#define		LE2			0x0414
#define		GPI_EN			0x0418
#define		RADIO_PD		0x042C
#define		RF2948_PD		0x042C

#ifndef HOST_IF_USB
	#define		LED1		0x0430
	#define		LED2		0x0434
#else
	#define     rLED_CTRL           0x0644
	#define     LED2                BIT_8   // Note: this is really LED1
	#define     LED1                BIT_9   // Note: this is really LED2
#endif


#define		EnablePSManualAGC	0x043C	// 1: enable
#define		CONFIGPhilips		0x0440
#define		SA2400_SER_AP		0x0444
#define		I2C_WRITE		0x0444	// Same as SA2400_SER_AP (for compatible with ZD1201)
#define		SA2400_SER_RP		0x0448

#define		RADIO_PE		0x0458
#define		RstBusMaster		0x045C

#define		RFCFG			0x0464

#define		HSTSCHG			0x046C

#define		PHY_ON			0x0474
#define		RX_DELAY		0x0478
#define		RX_PE_DELAY		0x047C


#define		GPIO_1			0x0490
#define		GPIO_2			0x0494


#define		EncryBufMux		0x04A8


#define		PS_Ctrl			0x0500

#define		ADDA_MBIAS_WarmTime	0x0508

#define		InterruptCtrl		0x0510
#define		TSF_LowPart		0x0514
#define		TSF_HighPart		0x0518
#define		ATIMWndPeriod		0x051C
#define		BCNInterval		0x0520
#define		Pre_TBTT		0x0524	//In unit of TU(1024us)

#define		PCI_TxAddr_p1		0x0600
#define		PCI_TxAddr_p2		0x0604
#define		PCI_RxAddr_p1		0x0608
#define		PCI_RxAddr_p2		0x060C
#define		MACAddr_P1		0x0610
#define		MACAddr_P2		0x0614
#define		BSSID_P1		0x0618
#define		BSSID_P2		0x061C
#define		BCNPLCPCfg		0x0620
#define		GroupHash_P1		0x0624
#define		GroupHash_P2		0x0628
#define		WEPTxIV			0x062C

#define		BasicRateTbl		0x0630
#define		MandatoryRateTbl	0x0634
#define		RTS_CTS_Rate		0x0638

#define		Wep_Protect		0x063C
#define		RX_THRESHOLD		0x0640
#define		TX_PE_CTRL		0x0644

#if defined(AMAC)
	#define	AfterPNP		0x0648
#endif

#if defined(OFDM)
	#define	AckTime80211		0x0658
#endif

#define		Rx_OFFSET		0x065c


#define		PHYDelay		0x066C
#define		BCNFIFO			0x0670
#define		SnifferOn		0x0674
#define		EncryptionType		0x0678
#define		RetryMAX		0x067C
#define		CtlReg1			0x0680	//Bit0:		IBSS mode
//Bit1:		PwrMgt mode
//Bit2-4 :	Highest basic Rate
//Bit5:		Lock bit
//Bit6:		PLCP weight select
//Bit7:		PLCP switch
#define		DeviceState		0x0684
#define		UnderrunCnt		0x0688
#define		Rx_Filter		0x068c
#define		Ack_Timeout_Ext		0x0690
#define		BCN_FIFO_Semaphore	0x0694
#define		IFS_Value		0x0698
#define		RX_TIME_OUT		0x069C
#define		TotalRxFrm		0x06A0
#define		CRC32Cnt		0x06A4
#define		CRC16Cnt		0x06A8
#define		DecrypErr_UNI		0x06AC
#define		RxFIFOOverrun		0x06B0

#define		DecrypErr_Mul		0x06BC

#define		NAV_CNT			0x06C4
#define		NAV_CCA			0x06C8
#define		RetryCnt		0x06CC

#define		ReadTcbAddress		0x06E8

#define		ReadRfdAddress		0x06EC
#define		CWmin_CWmax		0x06F0
#define		TotalTxFrm		0x06F4
#define		RX_OFFSET_BYTE		0x06F8

#define		CAM_MODE		0x0700
#define		CAM_ROLL_TB_LOW		0x0704
#define		CAM_ROLL_TB_HIGH	0x0708
#define		CAM_ADDRESS		0x070C
#define		CAM_DATA		0x0710
#define 	DECRY_ERR_FLG_LOW	0x0714
#define 	DECRY_ERR_FLG_HIGH	0x0718
#define		WEPKey0			0x0720
#define		WEPKey1			0x0724
#define		WEPKey2			0x0728
#define		WEPKey3			0x072C
#define		CAM_DEBUG		0x0728
#define		CAM_STATUS		0x072c
#define		WEPKey4				0x0730
#define		WEPKey5				0x0734
#define		WEPKey6				0x0738
#define		WEPKey7				0x073C
#define		WEPKey8				0x0740
#define		WEPKey9				0x0744
#define		WEPKey10			0x0748
#define		WEPKey11			0x074C
#define		WEPKey12			0x0750
#define		WEPKey13			0x0754
#define		WEPKey14			0x0758
#define		WEPKey15			0x075c
#define		TKIP_MODE			0x0760

#define		Dbg_FIFO_Rd			0x0800
#define		Dbg_Select			0x0804
#define		FIFO_Length			0x0808


//#define		RF_Mode					0x080C

#define		RSSI_MGC			0x0810

#define		PON					0x0818
#define		Rx_ON				0x081C
#define		Tx_ON				0x0820
#define		CHIP_EN				0x0824
#define		LO_SW				0x0828
#define		TxRx_SW				0x082C
#define		S_MD				0x0830

#define		USB_DEBUG_PORT		0x0888

// EEPROM Memmory Map Region
#define		E2P_SUBID			0x0900
#define		E2P_POD				0x0904
#define		E2P_MACADDR_P1		0x0908
#define		E2P_MACADDR_P2		0x090C

#ifndef HOST_IF_USB
#define		E2P_PWR_CAL_VALUE	0x0910

#define		E2P_PWR_INT_VALUE	0x0920

#define		E2P_ALLOWED_CHANNEL	0x0930
#define		E2P_PHY_REG			0x0934

#define		E2P_REGION_CODE		0x0960
#define		E2P_FEATURE_BITMAP	0x0964
#endif

//-------------------------------------------------------------------------
// Command Block (CB) Field Definitions
//-------------------------------------------------------------------------
//- RFD Command Bits
#define RFD_EL_BIT              BIT_0	        // RFD EL Bit

//- CB Command Word
#define CB_S_BIT                0x1          // CB Suspend Bit

//- CB Status Word
#define CB_STATUS_COMPLETE      0x1234          // CB Complete Bit

#define RFD_STATUS_COMPLETE     0x1234   //0x34120000       // RFD Complete Bit


/**************************************************************************
**		MAC Register Bit Definitions
***************************************************************************
*/
// Interrupt STATUS
#define	TX_COMPLETE		BIT_0
#define	RX_COMPLETE		BIT_1
#define	RETRY_FAIL		BIT_2
#define WAKE_UP			BIT_3
#define	DTIM_NOTIFY		BIT_5
#define	CFG_NEXT_BCN		BIT_6
#define BUS_ABORT		BIT_7
#define TX_FIFO_READY		BIT_8
#define UART_INT		BIT_9

#define	TX_COMPLETE_EN		BIT_16
#define	RX_COMPLETE_EN		BIT_17
#define	RETRY_FAIL_EN		BIT_18
#define WAKE_UP_EN		BIT_19
#define	DTIM_NOTIFY_EN		BIT_21
#define	CFG_NEXT_BCN_EN		BIT_22
#define BUS_ABORT_EN		BIT_23
#define TX_FIFO_READY_EN	BIT_24
#define UART_INT_EN		BIT_25

#define	FILTER_BEACON		0xFEFF //mask bit 8
#define	UN_FILTER_PS_POLL	0x0400

#define RX_LEN_THRESHOLD	0x640	//1600

#define	DBG_MSG_SHOW		0x1
#define	DBG_MSG_HIDE		0x0


#define	RFD_POINTER(skb, macp)      ((zd1205_RFD_t *) (((unsigned char *)((skb)->data))-((macp)->rfd_size)))
#define	SKB_RFD_STATUS(skb, macp)   ((RFD_POINTER((skb),(macp)))->CbStatus)


/**************************************************************************
**		Descriptor Data Structure
***************************************************************************/
struct driver_stats
{
        struct net_device_stats net_stats;
/* ath_desc: added iw_get_stats */
#ifdef CONFIG_NET_WIRELESS
        struct iw_statistics iw_stats;
#endif
        unsigned long tx_late_col;
        unsigned long tx_ok_defrd;
        unsigned long tx_one_retry;
        unsigned long tx_mt_one_retry;
        unsigned long rcv_cdt_frames;
        unsigned long xmt_fc_pkts;
        unsigned long rcv_fc_pkts;
        unsigned long rcv_fc_unsupported;
        unsigned long xmt_tco_pkts;
        unsigned long rcv_tco_pkts;

        unsigned long rx_intr_pkts;
};

//-------------------------------------------------------------------------
// Transmit Command Block (TxCB)
//-------------------------------------------------------------------------
typedef struct zd1205_HwTCB_s
{
        u32	CbStatus;					// Bolck status
        u32	CbCommand;					// Block command
/* ath_desc: AMD64 support */
#ifndef __LP64__
        u32	NextCbPhyAddrLowPart; 		// Next TCB address(low part)
        u32	NextCbPhyAddrHighPart;		// Next TCB address(high part)
        u32 TxCbFirstTbdAddrLowPart; 	// First TBD address(low part)
        u32 TxCbFirstTbdAddrHighPart;	// First TBD address(high part)
#else
        u64 NextCbPhyAddr;		// Next TCB address
        u64 TxCbFirstTbdAddr;		// First TBD address
#endif
        u32 TxCbTbdNumber;				// Number of TBDs for this TCB
}
zd1205_HwTCB_t;

//-------------------------------------------------------------------------
// Transmit Buffer Descriptor (TBD)
//-------------------------------------------------------------------------
typedef struct zd1205_TBD_s
{
#ifndef __LP64__
        u32 TbdBufferAddrLowPart;		// Physical Transmit Buffer Address
        u32 TbdBufferAddrHighPart;		// Physical Transmit Buffer Address
#else
        u64 TbdBufferAddr;
#endif
        u32 TbdCount;		// Data length
#ifdef HOST_IF_USB

        u32 PrvFragLen;
#endif
}
zd1205_TBD_t;

//-------------------------------------------------------------------------
// Receive Frame Descriptor (RFD)
//-------------------------------------------------------------------------
struct zd1205_RFD_s
{
        u32	CbStatus;				// Bolck status
        u32	ActualCount;			// Rx buffer length
        u32	CbCommand;				// Block command
        u32	MaxSize;				//
#ifndef __LP64__
        u32	NextCbPhyAddrLowPart;		// Next RFD address(low part)
        u32	NextCbPhyAddrHighPart;		// Next RFD address(high part)
#else
        u64	NextCbPhyAddr;		// Next RFD address
#endif
        u8	RxBuffer[MAX_WLAN_SIZE];	// Rx buffer
        u32	Pad[2];			// Pad to 16 bytes alignment - easy view for debug
} __attribute__ ((__packed__));
typedef struct zd1205_RFD_s zd1205_RFD_t;


typedef struct zd1205_Ctrl_Set_s
{
        u8	CtrlSetting[40];
}
zd1205_Ctrl_Set_t;

typedef struct zd1205_Header_s
{
        u8	MacHeader[32];

}
zd1205_Header_t;

//-------------------------------------------------------------------------
// ZD1205SwTcb -- Software Transmit Control Block.  This structure contains
// all of the variables that are related to a specific Transmit Control
// block (TCB)
//-------------------------------------------------------------------------
typedef struct zd1205_SwTcb_s
{
        // Link to the next SwTcb in the list
        struct zd1205_SwTcb_s *next;
        struct sk_buff *skb;

        // physical and virtual pointers to the hardware TCB
        zd1205_HwTCB_t *pTcb;
        unsigned long TcbPhys;

        // Physical and virtual pointers to the TBD array for this TCB
        zd1205_TBD_t *pFirstTbd;
        unsigned long FirstTbdPhys;

        zd1205_Ctrl_Set_t *pHwCtrlPtr;
        unsigned long HwCtrlPhys;

        zd1205_Header_t *pHwHeaderPtr;
        unsigned long HwHeaderPhys;
        u32 TcbCount;
        u8 LastFrag;
        u8 MsgID;
        u8 FrameType;
        u8 Rate;
        u16 aid;
        u8 bIntraBss;
        //u8 encryType;

#ifdef HOST_IF_USB

        u8 bHasCompleteBeforeSend;
        u8 bHasBeenDelayedBefore;
#endif

        u8 CalMIC[MIC_LNG+1];
        u8 MIC_Start;
        u8 MIC_Len;
        u32 LengthDiff;
}
zd1205_SwTcb_t;


typedef struct SwTcbQ_s
{
        zd1205_SwTcb_t 	*first;		/* first zd1205_SwTcb_t in Q */
        zd1205_SwTcb_t 	*last;		/* last zd1205_SwTcb_t in Q */
        u16			count;		/* number of zd1205_SwTcb_t in Q */
}
zd1205_SwTcbQ_t;

//- Wireless 24-byte Header
typedef struct wla_Header_s
{
        u8		FrameCtrl[2];
        u8		Duration[2];
        u8		DA[6];
        u8		BSSID[6];
        u8		SA[6];
        u8		SeqCtrl[2];
}
wla_Header_t;
struct hostap_ieee80211_hdr
{
        u16 frame_control;
        u16 duration_id;
        u8 addr1[6];
        u8 addr2[6];
        u8 addr3[6];
        u16 seq_ctrl;
        u8 addr4[6];
}
__attribute__ ((packed));

//from station
typedef struct plcp_wla_Header_s
{
        u8		PlcpHdr[PLCP_HEADER];    //Oh! shit!!!
        u8		FrameCtrl[2];
        u8		Duration[2];
        u8		Address1[6];
        u8		Address2[6];
        u8		Address3[6];
        u8		SeqCtrl[2];
}
plcp_wla_Header_t;

typedef struct ctrl_Set_parm_s
{
        u8			Rate;
        u8			Preamble;
        u8			encryType;
        u8			vapId;
        //u8			bHwAppendMic;
        u32			CurrFragLen;
        u32			NextFragLen;
}
ctrl_Set_parm_t;

typedef struct tuple_s
{
        u8		ta[6]; //TA (Address 2)
        u16		sn;
        u8		fn;
        u8		full;
}
tuple_t;

typedef struct tuple_Cache_s
{
        tuple_t cache[TUPLE_CACHE_SIZE];
        u8 freeTpi;
}
tuple_Cache_t;

typedef struct defrag_Mpdu_s
{
        u8	ta[6];
        u8	inUse;
        u8	fn;
        u32	eol;
        u16	sn;
        void 	*buf;
        void	*dataStart;
}
defrag_Mpdu_t;


typedef struct defrag_Array_s
{
        defrag_Mpdu_t mpdu[MAX_DEFRAG_NUM];
}
defrag_Array_t;

/*Rx skb holding structure*/
struct rx_list_elem
{
        struct list_head list_elem;
        dma_addr_t dma_addr;
        struct sk_buff *skb;
#ifdef HOST_IF_USB

        u32 UnFinishFrmLen;
#endif
}
__attribute__ ((__packed__));


#define ZD1211_MAX_MTU		2400
#define MAX_EPINT_BUFFER	64
#define NUM_TCB				64//32
#define NUM_TBD_PER_TCB		(2+MAX_SKB_FRAGS)	//3
#define NUM_TBD				(NUM_TCB * NUM_TBD_PER_TCB)
#define NUM_RFD				32
#ifdef HOST_IF_USB
//    #define ZD1205_INT_MASK		CFG_NEXT_BCN_EN | DTIM_NOTIFY_EN | WAKE_UP_EN
#define ZD1205_INT_MASK		0x4F0000
#else
    #define ZD1205_INT_MASK		TX_COMPLETE_EN | RX_COMPLETE_EN | RETRY_FAIL_EN | CFG_NEXT_BCN_EN | DTIM_NOTIFY_EN | WAKE_UP_EN | BUS_ABORT_EN
#endif
#define TX_RING_BYTES		(NUM_TCB * (sizeof(zd1205_HwTCB_t) + sizeof(zd1205_Ctrl_Set_t) + sizeof(zd1205_Header_t)))+ (NUM_TBD * sizeof(zd1205_TBD_t))
#define ZD1205_REGS_SIZE	4096
#define ZD_RX_OFFSET        0x03

struct zdap_ioctl
{
        u16 cmd;                /* Command to run */
        u32 addr;                /* Length of the data buffer */
        u32 value;              /* Pointer to the data buffer */
        u8  data[0x100];
};

struct zd1205_private
{
        //linux used
        struct net_device 	*device;

#ifdef HOST_IF_USB

        struct usb_device	*usb;
        int			dev_index;
        struct urb		*ctrl_urb, *rx_urb, *tx_urb, *intr_urb, *reg_urb;
        struct urb		*read_urb, *write_urb; /* tmp urb pointer for rx_tasklet, tx_tasklet */
        wait_queue_head_t	regSet_wait;
        wait_queue_head_t	iorwRsp_wait;
        wait_queue_head_t	term_wait;
        wait_queue_head_t	msdelay;
        struct semaphore	ps_sem;
        struct semaphore	bcn_sem;
        struct semaphore	reg_sem;
        struct semaphore	config_sem;
        struct usb_interface	*interface;		/* the interface for this device */
        spinlock_t 		intr_lock;
        spinlock_t		rx_pool_lock;
        u8          		tx_buff[ZD1211_MAX_MTU];
#if 1//(LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0))

        u8			IntEPBuffer[MAX_EPINT_BUFFER];
#else

        u8			*IntEPBuffer;
        dma_addr_t		IntBufferHandle;
#endif

        u8			IntEPBuffer2[MAX_EPINT_BUFFER];
        u8			IntEPBuffer3[MAX_EPINT_BUFFER];
        u8			num_interrupt_in;
        u8			num_interrupt_out;
        u8			num_bulk_in;
        u8			num_bulk_out;
        u8			in_interval;
        u8			out_interval;
        u8			ep4isIntOut;
        u8			cmd_end;
        u8			read_end;
        u16			wMaxPacketSize;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0))

        unsigned int		flags;
        unsigned int		kevent_flags;
#else

        unsigned long		flags;
        unsigned long		kevent_flags;
#endif

        int 			release;			/* release number */
        int			total_rx_cnt;
        int			end;
        u32			usbTxCnt;
        u32			usbTxCompCnt;
        struct rx_list_elem **rx_struct_array;
        struct tasklet_struct zd1211_rx_tasklet;
        struct tasklet_struct zd1211_tx_tasklet;
        struct tasklet_struct rx_buff_tasklet;
        struct work_struct      scan_tout_event;

        struct work_struct	kevent;
        struct zdap_ioctl	zdreq;
        /* ath_desc: support for unpatched wpasupplicant */
        struct iw_mlme		mlme_req;
        struct ifreq		ifreq;
        struct zydas_wlan_param	zd_wpa_req;
        int			ifcmd;

        //debug counter
        u32			regWaitRCompCnt;
        u32			regWaitWCompCnt;
        u32			regRWCompCnt;
        u32			regWaitRspCnt;
        u32			regRspCompCnt;
        u32			regUnCompCnt;
        u32			regRdSleepCnt;
#else

        struct pci_dev 		*pdev;
#endif

        struct driver_stats	drv_stats;
#if ZDCONF_WE_STAT_SUPPORT == 1

        struct iw_statistics iwstats;
#elif !defined(ZDCONF_WE_STAT_SUPPORT)
	#error "Undefine ZDCONF_WE_STAT_SUPPORT"
#endif

        struct timer_list 	watchdog_timer;		/* watchdog timer id */
        struct timer_list 	tm_chal_id;
        struct timer_list	tm_scan_id;
        struct timer_list	tm_auth_id;
        struct timer_list	tm_asoc_id;
        struct timer_list	tm_hking_id;
        struct timer_list	tm_mgt_id;
        char			ifname[IFNAMSIZ];
        spinlock_t 		bd_lock;		/* board lock */
        spinlock_t 		bd_non_tx_lock;		/* Non transmit command lock  */
        spinlock_t		q_lock;
        spinlock_t		cs_lock;
        spinlock_t		conf_lock;
        int	                using_dac;
        struct tasklet_struct zd1205_tasklet;
        struct tasklet_struct zd1205_ps_tasklet;
        struct tasklet_struct zd1205_tx_tasklet;
        struct proc_dir_entry *proc_parent;
        struct list_head 	active_rx_list;		/* list of rx buffers */
        struct list_head 	rx_struct_pool;		/* pool of rx buffer struct headers */
        u16 			rfd_size;
        u8			rev_id;			/* adapter PCI revision ID */
        u8	                sniffer_on;
        int			skb_req;		/* number of skbs neede by the adapter */

        rwlock_t 		isolate_lock;

        int			driver_isolated;
        char			*cable_status;
        void			*regp;
        u8			macAdr[8];
        u8			mcastAdr[8];
        u32			intrMask;
        zd1205_SwTcbQ_t 	*freeTxQ;
        zd1205_SwTcbQ_t 	*activeTxQ;
        u32  		        txCachedSize;
        u8			*txCached;
        u16			dtimCount;
        u8			numTcb;
        u16			numTbd;
        u8			numRfd;
        u8			numTbdPerTcb;
        u32			rxOffset;
        u32 			debugflag;
        card_Setting_t		cardSetting;
        u8			BSSID[8];
        u32			dbg_flag;

        //debug counter
        u32			bcnCnt;
        u32			dtimCnt;
        u32			txCmpCnt;
        u32			rxCnt;
        u32			retryFailCnt;
        u32			txCnt;
        u32			txIdleCnt;
        u32			rxIdleCnt;
        u32 			rxDupCnt;

        u32			DriverDiscardedFrmCauseByMulticastList;
        u32			DriverDiscardedFrmCauseByFrmCtrl;
        u32			DriverReceivedFrm;
        u32			DriverRxMgtFrmCnt;
        u32			ErrLongFrmCnt;
        u32			ErrShortFrmCnt;

        u32			ErrToHostFrmCnt;
        u32			ErrZeroLenFrmCnt;
        u32			ArFreeFailCnt;
        u32			ArSearchFailCnt;
        u32			ArAgedCnt;

        u32			DropFirstFragCnt;
        u32			rxNeedFragCnt;
        u32			rxCompFragCnt;
        u32			AllocSkbFailCnt;
        u32			txQueToUpCnt;
        u32			txQueSetCnt;
        u32			sleepCnt;
        u32			wakeupCnt;

        //HMAC counter
        u32			hwTotalRxFrm;
        u32			hwCRC32Cnt;
        u32			hwCRC16Cnt;
        u32			hwDecrypErr_UNI;
        u32			hwDecrypErr_Mul;
        u32			hwRxFIFOOverrun;
        u32			hwTotalTxFrm;
        u32			hwUnderrunCnt;

        u32			hwRetryCnt;
        u32			TxStartTime;
        u32			HMAC_TxTimeout;

        u8			bTraceSetPoint;
        u8			bEnableTxPwrCtrl;
        u8			TxOFDMCnt;
        u8			TxOFDMType;
        u8			TxPwrCCK;
        u8			TxPwrOFDM;          // 6M - 36M
        u8			bFixedRate;
        u8			bDataTrafficLight;
        u8			NoBcnDetectedCnt;
        u8			LinkTimer;
        u32			LinkLEDn;
        u32			LinkLED_OnDur;
        u32			LinkLED_OffDur;
        u32			DataLED;

        u16			AddrEntryTable;
        u8			bAllowAccessRegister;
        U8			FlashType;
        u16			ReadRegCount;
        u16			SetPoint;
        u8			dtim_notify_en;
        u8			config_next_bcn_en;

        u32			invalid_frame_good_crc;
        u8			bGkInstalled;
        u8			rxSignalQuality;

        u8			rxSignalStrength;
        u8			rxSignalQualityIndB;
        u8			rxSignalQuality1;
        u8			rxSignalQuality2;
        u16			EepSetPoint[14];
        u16              	SetPointOFDM[3][14];//JWEI 2003/12/31
        u32			RegionCode;
        u32			RF_Mode;
        u8			PA_Type;
        u8			MaxTxPwrSet;
        u8			MinTxPwrSet;
        u8			bss_index;
        bss_info_t 		BSSInfo[BSS_INFO_NUM];
        tuple_Cache_t 		cache;

        defrag_Array_t 		defragArray;
        rxInfo_t 		rxInfo;

        //added for STA
        atomic_t		DoNotSleep;
        u8			bSurpriseRemoved;
        u8			bAssoc;
        u8			PwrState;
        u8			SuggestionMode;
        u8			bPSMSupported;
        u8			bAPAlive;
        u8			BSS_Members;
        u8			Notification;
        u8			WorseSQThr;
        u8			bIBSS_Wakeup_Dest;
        u8			bFrmRxed1;
        u8			bAnyActivity;
        u8			NiceSQThr;
        u8               	NiceSQThr_OFDM;
        u8			bEnableSwAntennaDiv;
        u8			Ant_MonitorDur1;
        u8			Ant_MonitorDur2;
        u8			CR138Flag;
        u8   			MulticastAddr[194]; // the first byte is the number of multicast addresses
        u32			TotalTxDataFrmBytes;

        u32			TotalRxDataFrmBytes;
        u32			txMulticastFrm;
        u32			txMulticastOctets;
        u32			txUnicastFrm;
        u32			txUnicastOctets;
        u32			NormalBackoff;
        u32			UrgentBackoff;
        u32			LooseBackoff;
        u32			Acc_Num_OFDM;
        u32			Acc_SQ_OFDM;
        u32			Bcn_Acc_Num;
        u32			Bcn_Acc_SQ;
        u32			CheckForHangLoop;
        u16			SequenceNum;
        u16			iv16;
        u32			iv32;
        u32			Acc_Num;
        u32			Acc_SQ;
        u32			GroupHashP1;
        u32			GroupHashP2;
        u32			PSThreshhold;
        u8			rxDecryType;
        u32  			rx11bDataFrame;
        u32  			rxOFDMDataFrame;

#ifdef OFDM

        u8			bTxBurstEnable;
        int			TxBurstCount;
#endif

#ifdef HOST_IF_USB

        u32			USBCSRAddress;
        u8			bUSBDeveiceAttached;
        u8			bUSBDeveiceResetting;
        u8			bHandleNonRxTxRunning;
        u32			REG_6e4_Add;
        u32 			Continue2Rx;
        u8			LastZDContinuousTxRate;
        u32			WaitLenInfoCnt;
        u32			CompLenInfoCnt;
        u32			NoMergedRxCnt;
        u8			bFlashable;
#endif

        u8			bDisableTx;
        u8			PHYSettingFlag;
        u8			PHYTestIndex;
        u8			PHYTestTimer;
        u8			PHYTestTimerCount;
        u8			PHYTestRssiBound;
        u8			PHYTestRssi;
        u8			PHYLowPower;
        u8			IPCFlag;
        u8			AdapterMaxRate;

        u8			PHYFreOFDMframe;
        u8			EnableTxPwrCtrl;

        u32			PHYTestTotal;
        u32			TrafficBound;
        u32			DriverRxFrmCnt;
        u64			rxDataPerSec;
        u64			txDataPerSec;

        u32			txUnCachedSize;
        dma_addr_t		txUnCachedPhys;
        void			*txUnCached;
        //Modified for Supplicant
        int			bDefaultIbssMacMode;
        u32	bOLBC;
        u32	nOLBC_CounterInSec;

};

typedef struct zd1205_private	zd1205_private_t;


struct usb_eth_dev
{
        char	*name;
        __u16	vendor;

        __u16	device;
        __u32	private; /* LSB is gpio reset value */
};

#define ZD_IOCTL_REG_READ			0x01
#define ZD_IOCTL_REG_WRITE			0x02
#define ZD_IOCTL_MEM_DUMP			0x03
#define ZD_IOCTL_RATE       			0x04
#define ZD_IOCTL_SNIFFER    			0x05
#define ZD_IOCTL_CAM_DUMP   			0x06
#define ZD_IOCTL_DUMP_PHY   			0x07
#define ZD_IOCTL_CARD_SETTING 			0x08
#define ZD_IOCTL_HASH_DUMP			0x09
#define ZD_IOCTL_RFD_DUMP			0x0A
#define ZD_IOCTL_MEM_READ			0x0B
#define ZD_IOCTL_MEM_WRITE			0x0C

//for STA
#define ZD_IOCTL_TX_RATE			0x0D
#define ZD_IOCTL_EEPROM				0x0E

//for debug purposes
#define ZD_IOCTL_BCN				0x10
#define ZD_IOCTL_REG_READ16			0x11
#define ZD_IOCTL_REG_WRITE16			0x12

//for CAM Test
#define	ZD_IOCTL_CAM_READ			0x13
#define ZD_IOCTL_CAM_WRITE			0x14
#define ZD_IOCTL_CAM_RESET			0x15
#define ZD_IOCTL_READ_PHY			0x16
#define ZD_IOCTL_WRITE_PHY			0x17
#define ZD_IOCTL_CONT_TX			0x18
#define ZD_IOCTL_SET_MIC_CNT_ENABLE 0x19
#define ZD_IOCTL_GET_MIC_CNT_ENABLE 0x1A
#define ZD_IOCTL_DEBUG_FLAG	0x21

#define	ZDAPIOCTL	SIOCDEVPRIVATE


/**************************************************************************
**		Function Declarations
***************************************************************************
*/
void zd1205_SetRatesInfo(struct zd1205_private *macp);
void zd1205_sleep_reset(struct zd1205_private *macp);
void update_beacon_interval(struct zd1205_private *macp, int val);
void zd1205_sw_reset(struct zd1205_private *macp);
void zd1205_watchdog_cb(struct net_device *);
void zd1205_dump_data(char *info, u8 *data, u32 data_len);
void zd1205_init_card_setting(struct zd1205_private *macp);
/* ath: loading/saving files from kernel is a hack */
//void zd1205_load_card_setting(struct zd1205_private *macp, u8 bInit);
//void zd1205_save_card_setting(struct zd1205_private *macp);
zd1205_SwTcb_t * zd1205_first_txq(struct zd1205_private *macp, zd1205_SwTcbQ_t *Q);
/* ath: gcc4 needs inline function bodies in the declaration */
u32 zd_readl(u32 offset);
void zd_writel(u32 value, u32 offset);
void zd1205_disable_int(void);
void zd1205_enable_int(void);
void zd1205_lock(struct zd1205_private *macp);
void zd1205_unlock(struct zd1205_private *macp);
void zd1205_device_reset(struct zd1205_private *macp);
void zd1205_config_wep_keys(struct zd1205_private *macp);
void zd1205_config_dyn_key(u8 DynKeyMode, u8 *pkey, int idx);
struct sk_buff* zd1205_prepare_tx_data(struct zd1205_private *macp, u16 bodyLen);
void zd1205_tx_test(struct zd1205_private *macp, u16 size);
void zd1205_qlast_txq(struct zd1205_private *macp, zd1205_SwTcbQ_t *Q, zd1205_SwTcb_t *signal);
int zd1205_DestPowerSave(struct zd1205_private *macp, u8 *pDestAddr);
int zd1205_found1(struct pci_dev *pcid, const struct pci_device_id *ent);
void zd1205_remove1(struct pci_dev *pcid);
BOOLEAN a_OSC_get_cal_int( u8 ch, u32 rate, u8 *intValue, u8 *calValue);
void zd1205_tx_isr(struct zd1205_private *);
u32 zd1205_rx_isr(struct zd1205_private *macp);
void  zd1205_clear_structs(struct net_device *dev);
unsigned char zd1205_init(struct zd1205_private *);
int zd1205_open(struct net_device *);
int zd1205_close(struct net_device *);
int zd1205_change_mtu(struct net_device *, int);
int zd1205_set_mac(struct net_device *, void *);
void zd1205_set_multi(struct net_device *);
void zd1205_IncreaseTxPower(struct zd1205_private *macp, u8 TxPwrType);
void zd1205_DecreaseTxPower(struct zd1205_private *macp, u8 TxPwrType);
void iLED_ON(struct zd1205_private *macp, u32 LEDn);
void iLED_OFF(struct zd1205_private *macp, u32 LEDn);
void iLED_SWITCH(struct zd1205_private *macp, u32 LEDn);
void HKeepingCB(struct net_device *dev);
void zd1205_mgt_mon_cb(struct net_device *dev);
int zd1205_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd);
int zd1205_xmit_frame(struct sk_buff *skb, struct net_device *dev);
struct net_device_stats *zd1205_get_stats(struct net_device *);
void hostap_michael_mic_failure(struct zd1205_private *macp,
                                struct hostap_ieee80211_hdr *hdr, int keyidx);
#ifndef HOST_IF_USB

void zd1205_start_ru(struct zd1205_private *);
#else

struct rx_list_elem *zd1205_start_ru(struct zd1205_private *);
#endif
void zd1205_process_wakeup(struct zd1205_private *macp);
void zd1205_connect_mon(struct zd1205_private *macp);

void zd1205_watchdog(struct zd1205_private *macp);
void zd1205_house_keeping(struct zd1205_private *macp);
void zd1211_set_multicast(struct zd1205_private *macp);
int zd1205_dis_connect(struct zd1205_private *macp);
void ChangeMacMode(u8 MAC_Mode, u8 Channel);
void zd1205_alloc_skbs(struct zd1205_private *macp);
int zd1211_mlme(struct zd1205_private *macp);
#if ZDCONF_WE_STAT_SUPPORT == 1

struct iw_statistics * zd1205_iw_getstats(struct net_device *dev);
#elif !defined(ZDCONF_WE_STAT_SUPPORT)
	#error "Undefine ZDCONF_WE_STAT_SUPPORT"
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22))
static inline void eth_copy_and_sum (struct sk_buff *dest,
                                     const unsigned char *src,
                                     int len, int base)
{
        memcpy (dest->data, src, len);
}
#endif

#endif	/* _ZD1205_H_ */
