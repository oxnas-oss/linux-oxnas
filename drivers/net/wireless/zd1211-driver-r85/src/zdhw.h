#ifndef __ZDHW_H__
#define __ZDHW_H__

#include "zdequates.h"

#define		RTS_RATE_1M				0x00
#define		RTS_RATE_11M			0x03
#define		RTS_RATE_24M			0x09
#define		RTS_MOD_TYPE_OFDM		0x100
#define		RTS_PMB_TYPE_SHORT		0x200
#define		CTS_RATE_11M			0x30000
#define		CTS_RATE_24M			0x90000
#define		CTS_MOD_TYPE_OFDM		0x1000000
#define		CTS_PMB_TYPE_SHORT		0x2000000
#define		NON_BARKER_PMB_SET		(RTS_MOD_TYPE_OFDM | RTS_PMB_TYPE_SHORT | CTS_MOD_TYPE_OFDM | CTS_PMB_TYPE_SHORT)
#define		NON_PROTECT_SET			(RTS_MOD_TYPE_OFDM | CTS_MOD_TYPE_OFDM)

#define		ZD_CR0			0x0000
#define		ZD_CR1			0x0004
#define		ZD_CR2			0x0008
#define		ZD_CR3			0x000C
#define		ZD_CR5			0x0010
#define		ZD_CR6			0x0014
#define		ZD_CR7			0x0018
#define		ZD_CR8			0x001C
#define		ZD_CR4			0x0020
#define		ZD_CR9			0x0024
#define		ZD_CR10			0x0028
#define		ZD_CR11			0x002C
#define		ZD_CR12			0x0030
#define		ZD_CR13			0x0034
#define		ZD_CR14			0x0038
#define		ZD_CR15			0x003C
#define		ZD_CR16			0x0040
#define		ZD_CR17			0x0044
#define		ZD_CR18			0x0048
#define		ZD_CR19			0x004C
#define		ZD_CR20			0x0050
#define		ZD_CR21			0x0054
#define		ZD_CR22			0x0058
#define		ZD_CR23			0x005C
#define		ZD_CR24			0x0060
#define		ZD_CR25			0x0064
#define		ZD_CR26			0x0068
#define		ZD_CR27			0x006C
#define		ZD_CR28			0x0070
#define		ZD_CR29			0x0074
#define		ZD_CR30			0x0078
#define		ZD_CR31			0x007C
#define		ZD_CR32			0x0080
#define		ZD_CR33			0x0084
#define		ZD_CR34			0x0088
#define		ZD_CR35			0x008C
#define		ZD_CR36			0x0090
#define		ZD_CR37			0x0094
#define		ZD_CR38			0x0098
#define		ZD_CR39			0x009C
#define		ZD_CR40			0x00A0
#define		ZD_CR41			0x00A4
#define		ZD_CR42			0x00A8
#define		ZD_CR43			0x00AC
#define		ZD_CR44			0x00B0
#define		ZD_CR45			0x00B4
#define		ZD_CR46			0x00B8
#define		ZD_CR47			0x00BC
#define		ZD_CR48			0x00C0
#define		ZD_CR49			0x00C4
#define		ZD_CR50			0x00C8
#define		ZD_CR51			0x00CC
#define		ZD_CR52			0x00D0
#define		ZD_CR53			0x00D4
#define		ZD_CR54			0x00D8
#define		ZD_CR55			0x00DC
#define		ZD_CR56			0x00E0
#define		ZD_CR57			0x00E4
#define		ZD_CR58			0x00E8
#define		ZD_CR59			0x00EC
#define		ZD_CR60			0x00F0
#define		ZD_CR61			0x00F4
#define		ZD_CR62			0x00F8
#define		ZD_CR63			0x00FC
#define		ZD_CR64			0x0100
#define		ZD_CR65			0x0104
#define		ZD_CR66			0x0108
#define		ZD_CR67			0x010C
#define		ZD_CR68			0x0110
#define		ZD_CR69			0x0114
#define		ZD_CR70			0x0118
#define		ZD_CR71			0x011C
#define		ZD_CR72			0x0120
#define		ZD_CR73			0x0124
#define		ZD_CR74			0x0128
#define		ZD_CR75			0x012C
#define		ZD_CR76			0x0130
#define		ZD_CR77			0x0134
#define		ZD_CR78			0x0138
#define		ZD_CR79			0x013C
#define		ZD_CR80			0x0140
#define		ZD_CR81			0x0144
#define		ZD_CR82			0x0148
#define		ZD_CR83			0x014C
#define		ZD_CR84			0x0150
#define		ZD_CR85			0x0154
#define		ZD_CR86			0x0158
#define		ZD_CR87			0x015C
#define		ZD_CR88			0x0160
#define		ZD_CR89			0x0164
#define		ZD_CR90			0x0168
#define		ZD_CR91			0x016C
#define		ZD_CR92			0x0170
#define		ZD_CR93			0x0174
#define		ZD_CR94			0x0178
#define		ZD_CR95			0x017C
#define		ZD_CR96			0x0180
#define		ZD_CR97			0x0184
#define		ZD_CR98			0x0188
#define		ZD_CR99			0x018C
#define		ZD_CR100		0x0190
#define		ZD_CR101		0x0194
#define		ZD_CR102		0x0198
#define		ZD_CR103		0x019C
#define		ZD_CR104		0x01A0
#define		ZD_CR105		0x01A4
#define		ZD_CR106		0x01A8
#define		ZD_CR107		0x01AC
#define		ZD_CR108		0x01B0
#define		ZD_CR109		0x01B4
#define		ZD_CR110		0x01B8
#define		ZD_CR111		0x01BC
#define		ZD_CR112		0x01C0
#define		ZD_CR113		0x01C4
#define		ZD_CR114		0x01C8
#define		ZD_CR115		0x01CC
#define		ZD_CR116		0x01D0
#define		ZD_CR117		0x01D4
#define		ZD_CR118		0x01D8
#define		ZD_CR119		0x01DC
#define		ZD_CR120		0x01E0
#define		ZD_CR121		0x01E4
#define		ZD_CR122		0x01E8
#define		ZD_CR123		0x01EC
#define		ZD_CR124		0x01F0
#define		ZD_CR125		0x01F4
#define		ZD_CR126		0x01F8
#define		ZD_CR127		0x01FC
#define		ZD_CR128		0x0200
#define		ZD_CR129		0x0204
#define		ZD_CR130		0x0208
#define		ZD_CR131		0x020C
#define		ZD_CR132		0x0210
#define		ZD_CR133		0x0214
#define		ZD_CR134		0x0218
#define		ZD_CR135		0x021C
#define		ZD_CR136		0x0220
#define		ZD_CR137		0x0224
#define		ZD_CR138		0x0228
#define		ZD_CR139		0x022C
#define		ZD_CR140		0x0230
#define		ZD_CR141		0x0234
#define		ZD_CR142		0x0238
#define		ZD_CR143		0x023C
#define		ZD_CR144		0x0240
#define		ZD_CR145		0x0244
#define		ZD_CR146		0x0248
#define		ZD_CR147		0x024C
#define		ZD_CR148		0x0250
#define		ZD_CR149		0x0254
#define		ZD_CR150		0x0258
#define		ZD_CR151		0x025C
#define		ZD_CR152		0x0260
#define		ZD_CR153		0x0264
#define		ZD_CR154		0x0268
#define		ZD_CR155		0x026C
#define		ZD_CR156		0x0270
#define		ZD_CR157		0x0274
#define		ZD_CR158		0x0278
#define		ZD_CR159		0x027C
#define		ZD_CR160		0x0280
#define		ZD_CR161		0x0284
#define		ZD_CR162		0x0288
#define		ZD_CR163		0x028C
#define		ZD_CR164		0x0290
#define		ZD_CR165		0x0294
#define		ZD_CR166		0x0298
#define		ZD_CR167		0x029C
#define		ZD_CR168		0x02A0
#define		ZD_CR169		0x02A4
#define		ZD_CR170		0x02A8
#define		ZD_CR171		0x02AC
#define		ZD_CR172		0x02B0
#define		ZD_CR173		0x02B4
#define		ZD_CR174		0x02B8
#define		ZD_CR175		0x02BC
#define		ZD_CR176		0x02C0
#define		ZD_CR177		0x02C4
#define		ZD_CR178		0x02C8
#define		ZD_CR179		0x02CC
#define		ZD_CR180		0x02D0
#define		ZD_CR181		0x02D4
#define		ZD_CR182		0x02D8
#define		ZD_CR183		0x02DC
#define		ZD_CR184		0x02E0
#define		ZD_CR185		0x02E4
#define		ZD_CR186		0x02E8
#define		ZD_CR187		0x02EC
#define		ZD_CR188		0x02F0
#define		ZD_CR189		0x02F4
#define		ZD_CR190		0x02F8
#define		ZD_CR191		0x02FC
#define		ZD_CR192		0x0300
#define		ZD_CR193		0x0304
#define		ZD_CR194		0x0308
#define		ZD_CR195		0x030C
#define		ZD_CR196		0x0310
#define		ZD_CR197		0x0314
#define		ZD_CR198		0x0318
#define		ZD_CR199		0x031C
#define		ZD_CR200		0x0320
#define		ZD_CR201		0x0324
#define		ZD_CR202		0x0328
#define		ZD_CR203		0x032C
#define		ZD_CR204		0x0330
#define		ZD_CR205		0x0334
#define		ZD_CR206		0x0338
#define		ZD_CR207		0x033C
#define		ZD_CR208		0x0340
#define		ZD_CR209		0x0344
#define		ZD_CR210		0x0348
#define		ZD_CR211		0x034C
#define		ZD_CR212		0x0350
#define		ZD_CR213		0x0354
#define		ZD_CR214		0x0358
#define		ZD_CR215		0x035C
#define		ZD_CR216		0x0360
#define		ZD_CR217		0x0364
#define		ZD_CR218		0x0368
#define		ZD_CR219		0x036C
#define		ZD_CR220		0x0370
#define		ZD_CR221		0x0374
#define		ZD_CR222		0x0378
#define		ZD_CR223		0x037C
#define		ZD_CR224		0x0380
#define		ZD_CR225		0x0384
#define		ZD_CR226		0x0388
#define		ZD_CR227		0x038C
#define		ZD_CR228		0x0390
#define		ZD_CR229		0x0394
#define		ZD_CR230		0x0398
#define		ZD_CR231		0x039C
#define		ZD_CR232		0x03A0
#define		ZD_CR233		0x03A4
#define		ZD_CR234		0x03A8
#define		ZD_CR235		0x03AC
#define		ZD_CR236		0x03B0

#define		ZD_CR240		0x03C0
#define		ZD_CR241		0x03C4
#define		ZD_CR242		0x03C8
#define		ZD_CR243		0x03CC
#define		ZD_CR244		0x03D0
#define		ZD_CR245		0x03D4

#define		ZD_CR251		0x03EC
#define		ZD_CR252		0x03F0
#define		ZD_CR253		0x03F4
#define		ZD_CR254		0x03F8
#define		ZD_CR255		0x03FC

#define		ZD_RF_IF_CLK			0x0400
#define		ZD_RF_IF_DATA			0x0404
#define		ZD_PE1_PE2				0x0408
#define		ZD_PE2_DLY				0x040C
#define		ZD_LE1					0x0410
#define		ZD_LE2					0x0414
#define		ZD_GPI_EN				0x0418
#define		ZD_RADIO_PD				0x042C
#define		ZD_RF2948_PD			0x042C

#ifndef HOST_IF_USB
	#define		ZD_LED1				0x0430
	#define		ZD_LED2				0x0434
#endif


#define		ZD_EnablePSManualAGC	0x043C	// 1: enable
#define		ZD_CONFIGPhilips		0x0440
#define		ZD_SA2400_SER_AP		0x0444
#define		ZD_I2C_WRITE			0x0444	// Same as SA2400_SER_AP (for compatible with ZD1201)
#define		ZD_SA2400_SER_RP		0x0448


#define		ZD_RADIO_PE				0x0458
#define		ZD_RstBusMaster			0x045C

#define		ZD_RFCFG				0x0464

#define		ZD_HSTSCHG				0x046C

#define		ZD_PHY_ON				0x0474
#define		ZD_RX_DELAY				0x0478
#define		ZD_RX_PE_DELAY			0x047C


#define		ZD_GPIO_1				0x0490
#define		ZD_GPIO_2				0x0494


#define		ZD_EncryBufMux			0x04A8


#define		ZD_PS_Ctrl				0x0500
#define		ZD_ADDA_PwrDwn_Ctrl		0x0504
#define		ZD_ADDA_MBIAS_WarmTime	0x0508
#define		ZD_MAC_PS_STATE			0x050C
#define		ZD_InterruptCtrl		0x0510
#define		ZD_TSF_LowPart			0x0514
#define		ZD_TSF_HighPart			0x0518
#define		ZD_ATIMWndPeriod		0x051C
#define		ZD_BCNInterval			0x0520
#define		ZD_Pre_TBTT				0x0524	//In unit of TU(1024us)

//for UART support
#define		ZD_UART_RBR_THR_DLL		0x0540
#define		ZD_UART_DLM_IER			0x0544
#define		ZD_UART_IIR_FCR			0x0548
#define		ZD_UART_LCR				0x054c
#define		ZD_UART_MCR				0x0550
#define		ZD_UART_LSR				0x0554
#define		ZD_UART_MSR				0x0558
#define		ZD_UART_ECR				0x055c
#define		ZD_UART_STATUS			0x0560

#define		ZD_PCI_TxAddr_p1		0x0600
#define		ZD_PCI_TxAddr_p2		0x0604
#define		ZD_PCI_RxAddr_p1		0x0608
#define		ZD_PCI_RxAddr_p2		0x060C
#define		ZD_MACAddr_P1			0x0610
#define		ZD_MACAddr_P2			0x0614
#define		ZD_BSSID_P1				0x0618
#define		ZD_BSSID_P2				0x061C
#define		ZD_BCNPLCPCfg			0x0620
#define		ZD_GroupHash_P1			0x0624
#define		ZD_GroupHash_P2			0x0628
#define		ZD_RX_TIMEOUT			0x062C

#define		ZD_BasicRateTbl			0x0630
#define		ZD_MandatoryRateTbl		0x0634
#define		ZD_RTS_CTS_Rate			0x0638

//				Wep protect
//				_
//	if set to 0x114
//				 __
//				0x14 * slot time(if is 2us) = 40us = wep init time.
//
// So if we change slot time, change 0x14 to let wep init time close to 40us.
#define		ZD_Wep_Protect			0x063C
#define		ZD_RX_THRESHOLD			0x0640

#ifdef HOST_IF_USB
	#define		RFCFG1					0x0644
#else
	#define		ZD_TX_PE_CTRL			0x0644
#endif

#if defined(AMAC)
	#define		ZD_AfterPNP			0x0648
#endif

#if defined(OFDM)
	#define		ZD_AckTime80211		0x0658
#endif

#define		ZD_Rx_OFFSET			0x065c

#ifdef ZD1211B
	#define		ZD_BCNLENGTH			0x0664
#endif
#define		ZD_PHYDelay				0x066C
#define		ZD_BCNFIFO				0x0670
#define		ZD_SnifferOn			0x0674
#define		ZD_EncryType			0x0678
#ifdef ZD1211
	#define		ZD_RetryMAX				0x067C
#endif
#define		ZD_CtlReg1				0x0680	//Bit0:		IBSS mode
//Bit1:		PwrMgt mode
//Bit2-4 :	Highest basic rate
//Bit5:		Lock bit
//Bit6:		PLCP weight select
//Bit7:		PLCP switch
#define		ZD_DeviceState			0x0684
#define		ZD_UnderrunCnt			0x0688
#define		ZD_Rx_Filter			0x068c
#define		ZD_Ack_Timeout_Ext		0x0690
#define		ZD_BCN_FIFO_Semaphore	0x0694
#define		ZD_IFS_Value			0x0698
#define		ZD_RX_TIME_OUT			0x069C
#define		ZD_TotalRxFrm			0x06A0
#define		ZD_CRC32Cnt				0x06A4
#define		ZD_CRC16Cnt				0x06A8
#define		ZD_DecrypErr_UNI		0x06AC
#define		ZD_RxFIFOOverrun		0x06B0

#define		ZD_DecrypErr_Mul		0x06BC

#define		ZD_NAV_CNT				0x06C4
#define		ZD_NAV_CCA				0x06C8
#define		ZD_RetryCnt				0x06CC

#define		ZD_ReadTcbAddress		0x06E8
#define		ZD_ReadRfdAddress		0x06EC
#define		ZD_CWmin_CWmax			0x06F0
#define		ZD_TotalTxFrm			0x06F4

#define		ZD_CAM_MODE				0x0700
#define		ZD_CAM_ROLL_TB_LOW		0x0704
#define		ZD_CAM_ROLL_TB_HIGH		0x0708
#define		ZD_CAM_ADDRESS			0x070C
#define		ZD_CAM_DATA				0x0710

#define		ZD_ROMDIR				0x0714

#define		ZD_DECRY_ERR_FLG_LOW	0x0714
#define		ZD_DECRY_ERR_FLG_HIGH	0x0718

#define		ZD_WEPKey0				0x0720
#define		ZD_WEPKey1				0x0724
#define		ZD_WEPKey2				0x0728
#define		ZD_WEPKey3				0x072C
#define		ZD_WEPKey4				0x0730
#define		ZD_WEPKey5				0x0734
#define		ZD_WEPKey6				0x0738
#define		ZD_WEPKey7				0x073C
#define		ZD_WEPKey8				0x0740
#define		ZD_WEPKey9				0x0744
#define		ZD_WEPKey10				0x0748
#define		ZD_WEPKey11				0x074C
#define		ZD_WEPKey12				0x0750
#define		ZD_WEPKey13				0x0754
#define		ZD_WEPKey14				0x0758
#define		ZD_WEPKey15				0x075c
#define		ZD_TKIP_MODE			0x0760

#define		ZD_TX_CCM_MODE			0x0B80
#define		ZD_TX_CCM_MASK			0x0B84
#define		ZD_RX_CCM_MODE			0x0C88
#define		ZD_RX_CCM_MASK			0x0C8C


#define		ZD_EEPROM_PROTECT0		0x0758
#define		ZD_EEPROM_PROTECT1		0x075C

#define		ZD_Dbg_FIFO_Rd			0x0800
#define		ZD_Dbg_Select			0x0804
#define		ZD_FIFO_Length			0x0808


//#define		RF_Mode					0x080C
#define		ZD_RSSI_MGC				0x0810

#define		ZD_PON					0x0818
#define		ZD_Rx_ON				0x081C
#define		ZD_Tx_ON				0x0820
#define		ZD_CHIP_EN				0x0824
#define		ZD_LO_SW				0x0828
#define		ZD_TxRx_SW				0x082C
#define		ZD_S_MD					0x0830

#define		ZD_USB_DEBUG_PORT		0x0888

// EEPROM Memmory Map Region
#define		ZD_EEPROM_START_ADDRESS	0x0900
#define		ZD_E2P_SUBID			0x0900
#define		ZD_E2P_POD				0x0904
#define		ZD_E2P_MACADDR_P1		0x0908
#define		ZD_E2P_MACADDR_P2		0x090C
#define     ZD_E2P_PWR_CAL_VALUE1   0x0910
#define     ZD_E2P_PWR_CAL_VALUE2   0x0914
#define     ZD_E2P_PWR_CAL_VALUE3   0x0918
#define     ZD_E2P_PWR_CAL_VALUE4   0x091c
#define		ZD_E2P_PWR_INT_VALUE1	0x0920
#define		ZD_E2P_PWR_INT_VALUE2	0x0924
#define		ZD_E2P_PWR_INT_VALUE3	0x0928
#define		ZD_E2P_PWR_INT_VALUE4	0x092c
#define		ZD_E2P_ALLOWED_CHANNEL	0x0930


#ifdef HOST_IF_USB
	#define     E2P_DEVICE_VER          0x0940
	#define     E2P_PHY_REG             0x094A
	#define     E2P_36M_CAL_VALUE       0x0950
#define		ZD_E2P_36M_CAL_VALUE2			0x0954
#define         ZD_E2P_36M_CAL_VALUE3                   0x0958
#define         ZD_E2P_36M_CAL_VALUE4                   0x095C

#define		ZD_E2P_11A_INT_VALUE1			0x0960 //802.11a IntValue1 for All Rate
#define         ZD_E2P_11A_INT_VALUE2                   0x0964 //802.11a IntValue2 for All Rate
#define         ZD_E2P_11A_INT_VALUE3                   0x0968 //802.11a IntValue3 for All Rate
#define         ZD_E2P_48M_CAL_VALUE2                   0x0974
#define         ZD_E2P_48M_CAL_VALUE3                   0x0978
#define         ZD_E2P_48M_CAL_VALUE4                   0x097C

#define		ZD_E2P_A36M_CAL_VALUE			0x0980 //802.11a SetPoint for 36~6M
#define         ZD_E2P_A36M_CAL_VALUE2                  0x0984
#define         ZD_E2P_A36M_CAL_VALUE3                  0x0988
#define         ZD_E2P_A36M_CAL_VALUE4                  0x098C
#define         ZD_E2P_54M_CAL_VALUE2                   0x0994
#define         ZD_E2P_54M_CAL_VALUE3                   0x0998
#define         ZD_E2P_54M_CAL_VALUE4                   0x099C


#define		ZD_E2P_A54M_CAL_VALUE			0x09A0 //802.11a SetPoint for 54/48M
#define         ZD_E2P_A54M_CAL_VALUE2                  0x09A4
#define         ZD_E2P_A54M_CAL_VALUE3                  0x09A8
#define         ZD_E2P_A54M_CAL_VALUE4                  0x09AC
#define         ZD_E2P_11A_INT_VALUE4                   0x096C //802.11a IntValue4 for All Rate


#define     E2P_36M_INT_VALUE       0x0960
	#define     E2P_48M_CAL_VALUE       0x0970
	#define     E2P_48M_INT_VALUE       0x0980
	#define     E2P_54M_CAL_VALUE       0x0990
	#define     E2P_54M_INT_VALUE       0x09A0


#ifndef fNEW_CODE_MAP
		#define     E2P_END             0x09F0
    #else
		#define     E2P_END             0x09FC
    #endif
#else
	#define		ZD_E2P_PHY_REG			0x0934
	#define		ZD_E2P_REGION_CODE		0x0960
	#define		ZD_E2P_FEATURE_BITMAP	0x0964
	#define     ZD_E2P_END              0x0968
#endif

#ifdef ZD1211B
	#define 	ZD_RetryMAX				0x0B28
#endif

#ifdef HOST_IF_USB
	#ifdef ZD1211
		#define     FW_FIRMWARE_VER         0x0B00
		#define     FW_USB_SPEED            0x0B01
		#define     FW_FIX_TX_RATE          0x0B02
		#define     FW_LINK_STATUS          0x0B03
		#define     FW_SOFT_RESET           0x0B04
		#define     FW_FLASH_CHK            0x0B05
	#elif defined(ZD1211B)
        #define     FW_FIRMWARE_VER         0x0F00
        #define     FW_USB_SPEED            0x0F01
        #define     FW_FIX_TX_RATE          0x0F02
        #define     FW_LINK_STATUS          0x0F03
        #define     FW_SOFT_RESET           0x0F04
        #define     FW_FLASH_CHK            0x0F05
	#endif

#define     ZD1211_CLOCK_CTRL       0x8540
	#define     bZD1211_CLOCK_EEPROM    0
	#define     bmZD1211_CLOCK_EEPROM   mBIT(bZD1211_CLOCK_EEPROM)
	#define     UMAC_EPROM_ROM_ADDR     0x862A
	#define     UMAC_EPROM_RAM_ADDR     0x862B
	#define     UMAC_EPROM_DMA_LEN_DIR  0x862C
	#define     bmEPROM_XFER_DIR        BIT_15
	#define     UMAC_WAIT_STATE         0x8801
#endif


#define		ZD_TX_PWR_CTRL_1		0x0B00
#define		ZD_TX_PWR_CTRL_2		0x0B04
#define		ZD_TX_PWR_CTRL_3		0x0B08
#define		ZD_TX_PWR_CTRL_4		0x0B0C

#define		ZD_EPP_ROM_ADDRESS		0x0A04
#define		ZD_EPP_SRAM_ADDRESS		0x0A08
#define		ZD_EPP_LENG_DIR			0x0A0C
#define		ZD_EPP_CLOCK_DIV		0x0A10
#define		ZD_EPP_KEY_PROT			0x0A14

//HW MIC Engine
#define		ZD_MIC_START_ADDR0		0x0D00


#define		ZD_MIC_BLOCK0_LEN		0x0D04
#define		ZD_MIC_START_ADDR1		0x0D08
#define		ZD_MIC_BLOCK1_LEN		0x0D0C
#define		ZD_MIC_START_ADDR2		0x0D10
#define		ZD_MIC_BLOCK2_LEN		0x0D14
#define		ZD_MIC_START_ADDR3		0x0D18
#define		ZD_MIC_BLOCK3_LEN		0x0D1C
#define		ZD_MIC_START_ADDR4		0x0D20
#define		ZD_MIC_BLOCK4_LEN		0x0D24
#define		ZD_MIC_START_ADDR5		0x0D28
#define		ZD_MIC_BLOCK5_LEN		0x0D2C
#define		ZD_MIC_KEY_LOW			0x0D30
#define		ZD_MIC_KEY_HIGH			0x0D34
#define		ZD_MIC_VALUE_LOW		0x0D38
#define		ZD_MIC_VALUE_HIGH		0x0D3C
#define		ZD_MIC_STATUS			0x0D40
#define		ZD_MIC_TOTAL_BLOCK_NUM	0x0D44
#define		ZD_MIC_WRITE_BACK_ADDRS	0x0D48
#define		ZD_HOST_ENDIAN			0x0D4C


#define	SR_1M		0x02
#define	SR_2M		0x04
#define	SR_5_5M		0x0b
#define	SR_11M		0x16
#define	SR_16_5M	0x21
#define	SR_27_5M	0x37
#define	SR_6M		0x0c
#define	SR_9M		0x12
#define	SR_12M		0x18
#define	SR_18M		0x24
#define	SR_24M		0x30
#define	SR_36M		0x48
#define	SR_48M		0x60
#define	SR_54M		0x6c

void HW_Set_Intersil_Chips(zd_80211Obj_t *pObj, U32 ChannelNo, U8 InitChOnly);
void HW_Set_GCT_Chips(zd_80211Obj_t *pObj, U32 ChannelNo, U8 InitChOnly);
void HW_Set_AL7230B_RF_Chips(zd_80211Obj_t *pObj, U32 ChannelNo, U8 InitChOnly,U8 MAC_Mode);

void LockPhyReg(zd_80211Obj_t *pObj);
void UnLockPhyReg(zd_80211Obj_t *pObj);
void HW_SetSupportedRate(zd_80211Obj_t *pObj, U8 *prates);
void HW_EnableBeacon(zd_80211Obj_t *pObj, U16 BecaonInterval, U16 DtimPeriod, U8 BssType);
U32 HW_GetNow(zd_80211Obj_t *pObj);
void HW_GetTsf(zd_80211Obj_t *pObj, U32 *loTsf, U32 *hiTsf);
void HW_SetBeaconFIFO(zd_80211Obj_t *pObj, U8 *pBeacon, U16 index);
void HW_SwitchChannel(zd_80211Obj_t *pObj, U16 channel, U8 InitChOnly, const U8 MAC_Mode);
void HW_SetRfChannel(zd_80211Obj_t *pObj, U16 channel, U8 InitChOnly, const U8 MAC_Mode);

int HW_HTP(zd_80211Obj_t *pObj);
void HW_RadioOnOff(zd_80211Obj_t *pObj, U8 on);
void HW_Set_IF_Synthesizer(zd_80211Obj_t *pObj, U32 InputValue);
void HW_ResetPhy(zd_80211Obj_t *pObj);
void HW_InitHMAC(zd_80211Obj_t *pObj);
void HW_UpdateIntegrationValue(zd_80211Obj_t *pObj, U32 ChannelNo,const U8 MAC_Mode);

void HW_WritePhyReg(zd_80211Obj_t *pObj, U8 PhyIdx, U8 PhyValue);
void HW_OverWritePhyRegFromE2P(zd_80211Obj_t *pObj);
void HW_E2P_AutoPatch(zd_80211Obj_t *pObj);
void HW_SetSTA_PS(zd_80211Obj_t *pObj, U8 op);
void HW_Write_TxGain(zd_80211Obj_t *pObj, U32 txgain);
void HW_Set_FilterBand(zd_80211Obj_t *pObj, U32	region_code);
void HW_UpdateBcnInterval(zd_80211Obj_t *pObj, U16 BcnInterval);
void HW_UpdateATIMWindow(zd_80211Obj_t *pObj, U16 AtimWnd);
void HW_UpdatePreTBTT(zd_80211Obj_t *pObj, U32 pretbtt);


// for AMAC CAM operation
void HW_CAM_Avail(zd_80211Obj_t *pObj);
void HW_CAM_Write(zd_80211Obj_t *pObj, U32 address, U32 data);
U32 HW_CAM_Read(zd_80211Obj_t *pObj, U32 address);
void HW_CAM_SetMAC(zd_80211Obj_t *pObj, U16 aid, U8 *pMAC);
void HW_CAM_GetMAC(zd_80211Obj_t *pObj, U16 aid, U8 *pMac);
void HW_CAM_SetEncryType(zd_80211Obj_t *pObj, U16 aid, U8 encryType);
U8 HW_CAM_GetEncryType(zd_80211Obj_t *pObj, U16 aid);
void HW_CAM_SetKey(zd_80211Obj_t *pObj, U16 aid, U8 keyLength, U8 *pKey);
void HW_CAM_GetKey(zd_80211Obj_t *pObj, U16 aid, U8 keyLength, U8 *pKey);
void HW_CAM_UpdateRollTbl(zd_80211Obj_t *pObj, U16 aid);
void HW_CAM_ResetRollTbl(zd_80211Obj_t *pObj);
void HW_CAM_ClearRollTbl(zd_80211Obj_t *pObj, U16 aid);
//void HW_ConfigDynaKey(zd_80211Obj_t *pObj, U16 aid, U8 *pMac, U8 *pKey, U8 KeyLength, U8 encryType);
void HW_ConfigDynaKey(zd_80211Obj_t *pObj, U16 aid, U8 *pMac, U8 *pKey, U8 KeyLength, U8 encryType, U8 change_enc);
void HW_ConfigStatKey(zd_80211Obj_t *pObj, U8 *pKey, U8 keyLen, U32 startAddr);
void HW_GetStatKey(zd_80211Obj_t *pObj);
void HW_EEPROM_ACCESS(zd_80211Obj_t *pObj, U8 RamAddr, U32 RomAddr, U32 length, U8 bWrite);


//
void HW_Write_TxGain0(zd_80211Obj_t *pObj, U8 *pTxGain, U8 TxPwrType);
void HW_Write_TxGain1(zd_80211Obj_t *pObj, U8 txgain, U8 TxPwrType);
void HW_Write_TxGain2(zd_80211Obj_t *pObj, U8 TxPwrType);
#endif
