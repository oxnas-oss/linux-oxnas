#ifndef _ZD1211_H_
#define _ZD1211_H_

#include "zd1205.h"


#define MAX_NUM_PORTS		8	/* The maximum number of ports one device can grab at once */

#define IOWAITTIME			2

#define EP_DATA_OUT			0x01
#define EP_DATA_IN			0x02
#define EP_INT_IN			0x03
#define EP_REG_OUT			0x04

// EP0 [request, ID] setting.
#define REGID_WRITE			0x21
#define REGID_READ			0x22
#define REGID_RFOFDMSET		0x23
#define REGID_PROG_FLSH 	0x24
#define EEPROM_START		0x128
#define EEPROM_MID			0x28
#define EEPROM_END			0x228

#define FIRMWARE_DOWNLOAD	0x30
#define FIRMWARE_CONFIRM	0x31
#define FIRMWARE_READ_DATA  0x32

#define EPINT_IORDRsp			0x90
#define EPINT_RetryFial_Event	0xa0

// for macp->flags
#define	ZD1211_UNPLUG		1
#define	ZD1211_REQ_COMP		2
#define ZD1211_RUNNING          3
#define ZD1211_TX_BUSY          4
#define ZD1211_CMD_FINISH       5
#define ZD1211_SCAN_REQUEST     6
#define ZD1211_SCAN_COMPLETE    7
#define CTX_FLAG_ESSID_WAS_SET  8

#define KEVENT_SCAN_TIMEOUT     1
#define KEVENT_MGT_MON_TIMEOUT  2
#define KEVENT_HOUSE_KEEPING    3
#define KEVENT_WATCH_DOG        4
#define KEVENT_AUTH_TIMEOUT     5
#define KEVENT_ASOC_TIMEOUT     6
#define KEVENT_TCHAL_TIMEOUT    7
#define KEVENT_NON_TX_RX_INT    8
#define KEVENT_ZD_IOCTL         9
#define KEVENT_EN_PROTECTION	10
#define KEVENT_DIS_PROTECTION	11
#define KEVENT_UPDATE_SETTING	12
#define KEVENT_SET_MULTICAST    13
#define KEVENT_PROCESS_SIGNAL   14
#define KEVENT_EN_BARKER        15
#define KEVENT_DIS_BARKER   	16
#define KEVENT_EN_SHORT_SLOT    17
#define KEVENT_DIS_SHORT_SLOT   18
#define KEVENT_DIS_CONNECT      19
#define KEVENT_STD_IOCTL        20
#define KEVENT_REGISTER_NET     21
#define KEVENT_ZD_MLME_IOCTL    22
/* ath_desc: Defer tx kill_urb on Linux 2.6.8+ */
#define KEVENT_KILL_TX_URB      23


#define mFILL_WRITE_REGISTER(addr0, value0) \
{                                           \
    WriteAddr[WriteIndex] = addr0;          \
    WriteData[WriteIndex ++] = value0;      \
}


// write 32_bit register ==> write high word first
#define mFILL_WRITE_REGISTER32(addr0, value32)               \
{                                                            \
    WriteAddr[WriteIndex] = (addr0) + 2;                     \
    WriteData[WriteIndex ++] = (u16) ((value32) >> 16);      \
    WriteAddr[WriteIndex] = addr0;                           \
    WriteData[WriteIndex ++] = (u16) ((value32) & 0xFFFF);   \
}

#define mFILL_READ_REGISTER(addr0)	(ReadAddr[ReadIndex++] = addr0)
#define mFILL_RF_REGISTER(value0)	(WriteData[WriteIndex++] = value0)

/* Flash interface */
#define bmFLASH_A0                  1

#define mFLASH_SET_EVEN_ADDR(orgCR203)                              \
    mFILL_WRITE_REGISTER(ZD1205_CR203 + (u16) (macp->USBCSRAddress), \
                         mCLR_BIT((u16) (orgCR203), bmFLASH_A0))

#define mFLASH_SET_ODD_ADDR(orgCR203)                               \
    mFILL_WRITE_REGISTER(ZD1205_CR203 + (u16) (macp->USBCSRAddress), \
                         mSET_BIT((u16) (orgCR203), bmFLASH_A0))

#define mFLASH_WRITE_EVEN_ADDR(addr0, value0, orgCR203) \
{                                                       \
    mFLASH_SET_EVEN_ADDR(orgCR203);                     \
    mFILL_WRITE_REGISTER(((addr0) >> 1), value0);       \
}

#define mFLASH_WRITE_ODD_ADDR(addr0, value0, orgCR203)  \
{                                                       \
    mFLASH_SET_ODD_ADDR(orgCR203);                      \
    mFILL_WRITE_REGISTER(((addr0) >> 1), value0);       \
}


//-------------------------------------------------------------------------
#if !fDRV_UPDATE_EEP
	#define WRITE_WORD_TO_EEPROM_PER_TIME   8

#else
	#define WRITE_WORD_TO_EEPROM_PER_TIME   16
#endif


typedef struct _USB_EEPROM_DATA
{
        u16  RequestID;
        u16  Data[WRITE_WORD_TO_EEPROM_PER_TIME];
}
USB_EEPROM_DATA, * PUSB_EEPROM_DATA;


typedef struct _USB_REG_PACKAGE
{
        u16  Address;
        u16  Data;
}
USB_REG_PACKAGE, *PUSB_REG_PACKAGE;


typedef struct _USB_READ_REG
{
        u16 RequestID;
        USB_REG_PACKAGE Package[cMAX_MULTI_READ_REG_NUM];
}
USB_READ_REG, *PUSB_READ_REG;



typedef struct _USB_READ_REG_REQ
{
        u16  RequestID;
        u16  Address[cMAX_MULTI_READ_REG_NUM];
}
USB_READ_REG_REQ, *PUSB_READ_REG_REQ;


typedef struct _USB_WRITE_REG_PACKAGE
{
        u16	Address;

        u16	WriteData_low;
}
USB_WRITE_REG_PACKAGE, * PUSB_WRITE_REG_PACKAGE;


/* ath_desc: bigendian support */
struct _USB_WRITE_REG
{
        u16  RequestID;
        USB_WRITE_REG_PACKAGE WritePackage[cMAX_MULTI_WRITE_REG_NUM];
} __attribute__((__packed__));
typedef struct _USB_WRITE_REG USB_WRITE_REG, * PUSB_WRITE_REG;


typedef struct _USB_SET_RF
{
        u16  RequestID;
        u16  Value;
        u16  Index;
        u16  Data[cMAX_MULTI_RF_REG_NUM];
}
USB_SET_RF, * PUSB_SET_RF;


void zd1211_StrongSignalDect(struct zd1205_private *macp);
void zd1211_TxCalibration(struct zd1205_private *macp);
void zd1211_CheckWithIPC(struct zd1205_private *macp);
void zd1211_unlink_all_urbs(struct zd1205_private *macp);
u16 zd1211_SetHighAddr(struct zd1205_private *macp, u16 high_addr);
u16 zd1211_SetAbsAddr(struct zd1205_private *macp, u32 abs_addr, u16 *get_cr203);
void zd1211_InitHighAddr(struct zd1205_private *macp);
void zd1211_FlashCmdWrite(struct zd1205_private *macp, u8 Cmd);
void zd1211_FlashSecErase(struct zd1205_private *macp, u16 Sec0);
void zd1211_FlashProgram(struct zd1205_private *macp, u16 addr0, u16 val0);
void zd1211_EraseFlash(struct zd1205_private *macp);
int zd1211_ProgFlash(struct zd1205_private *macp, u32 StartAddr, u32 BufLenInBytes, u8 *pDownloadBuffer);
u8 zd1211_InitSetup(struct net_device *dev,	struct zd1205_private *macp);
void zd1211_SwitchAntenna(struct zd1205_private *macp);
int zd1211_alloc_all_urbs(struct zd1205_private *macp);
void zd1211_free_all_urbs(struct zd1205_private *macp);
int zd1211_writel(u32 Address, u32 Value, u8 bAddUSBCSRAddress);
int zd1211_USB_PACKAGE_WRITE_REGISTER(u16 *Address, u16 *Value, u16 RegCount, u8 bAddUSBCSRAddress);
u32 zd1211_readl(u32 Address, u8 bAddUSBCSRAddress);
int zd1211_USB_PACKAGE_READ_REGISTER(u16 *Address, u16 *pValue, u16 RegCount, u8 bAddUSBCSRAddress);
void zd1211_handle_non_tx_rx(struct zd1205_private *macp);
void ZD1211_WRITE_MULTI_REG(u16  *pAddress, u16  *pValue, u16 *pRegCount);
int zd1211_USB_SET_RF_REG(u16 *InputValue, int bIs3683A);
int zd1211_submit_rx_urb(struct zd1205_private *macp);
int zd1211_submit_tx_urb(struct zd1205_private *macp,BOOLEAN LastFrag);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)||LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
void zd1211_tx_comp_cb(struct urb *urb);
void zd1211_rx_comp_cb(struct urb *urb);
void zd1211_intr_cb(struct urb *urb);
void zd1211_reg_cb(struct urb *urb);
#else
void zd1211_tx_comp_cb(struct urb *urb, struct pt_regs *regs);
void zd1211_rx_comp_cb(struct urb *urb, struct pt_regs *regs);
void zd1211_intr_cb(struct urb *urb, struct pt_regs *regs);
void zd1211_reg_cb(struct urb *urb, struct pt_regs *regs);
#endif
void zd1211_disable_net_traffic(struct zd1205_private *macp);
int zd1211_USB_ProgramFlash(struct zd1205_private *macp, u16 *Value, u16 RegCount);
int zd1211_LoadUSBSpecCode(struct zd1205_private *macp, u8 *pBuffer, u32 uImgLength, u16 uCodeOfst, u8 bReboot);
int zd1211_Download_IncludeFile(struct zd1205_private *macp);
int zd1211_GetUSBSpecData(struct zd1205_private *macp, u8 *pBuffer, u32 uImgLength, u16 uCodeOfst);
int zd1211_DownLoadUSBCode(struct zd1205_private *macp, u8* FileName, void *ptr, u16 uCodeOfst);
int zd1211_WriteMultiRegister(u16 *Address, u16 *Value, u16 RegCount, u8 bAddUSBCSRAddress);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
void kevent(void *data);
#endif
void defer_kevent(struct zd1205_private *macp, int flag);
void zd1211_rx_isr(unsigned long parm);
void zd1211_tx_isr(unsigned long parm);
void zd1211_alloc_rx(unsigned long parm);

#endif
