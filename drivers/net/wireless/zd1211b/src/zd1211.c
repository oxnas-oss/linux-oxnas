/* src/zd1211.c
*
*
*
* Copyright (C) 2004 ZyDAS Inc.  All Rights Reserved.
* --------------------------------------------------------------------
*
*
*
*   The contents of this file are subject to the Mozilla Public
*   License Version 1.1 (the "License"); you may not use this file
*   except in compliance with the License. You may obtain a copy of
*   the License at http://www.mozilla.org/MPL/
*
*   Software distributed under the License is distributed on an "AS
*   IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
*   implied. See the License for the specific language governing
*   rights and limitations under the License.
*
*   Alternatively, the contents of this file may be used under the
*   terms of the GNU Public License version 2 (the "GPL"), in which
*   case the provisions of the GPL are applicable instead of the
*   above.  If you wish to allow the use of your version of this file
*   only under the terms of the GPL and not to allow others to use
*   your version of this file under the MPL, indicate your decision
*   by deleting the provisions above and replace them with the notice
*   and other provisions required by the GPL.  If you do not delete
*   the provisions above, a recipient may use your version of this
*   file under either the MPL or the GPL.
*
* -------------------------------------------------------------------- */

#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))
#include <linux/autoconf.h>
#else
#include <linux/config.h>
#endif
#include <net/checksum.h>
#include <linux/tcp.h>
#include <linux/udp.h>


#include "zddebug.h"
#include "zdhw.h"
#include "zd1211.h"
#include "zdcompat.h"

u8 WS11UPh[]
#if fMERGE_RX_FRAME
	#include "WS11UPhR.h"
u8 WS11UPhm[]
#include "WS11UPhm.h"
#else
    #include "WS11UPhm.h"
#endif

u8 WS11Ur[]
#include "WS11Ur.h"

u8 WS11Ub[]
#include "WS11Ub.h"


u8 WS11Ur2[(0xEE00 - 0xEC00) * 2] = { 0x0F, 0x9F, 0x00, 0xEE };  // JMP 0xEE00


extern zd_80211Obj_t dot11Obj;
extern struct net_device *g_dev;

/* ath_desc: use usb_kill_urb() instead of usb_unlink_urb() in 2.6.8+ */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,8)
#  define ZD_USB_KILL_URB(u)       usb_kill_urb(u)
#else
#  define ZD_USB_KILL_URB(u)       usb_unlink_urb(u)
#endif

/* ath_desc: URB_ASYNC_UNLINK has been removed in Linux 2.6.14 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,14)
#  define URB_ASYNC_UNLINK 0
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0) /* tune me! */
#  define SUBMIT_URB(u,f)       usb_submit_urb(u,f)
#  define USB_ALLOC_URB(u,f)    usb_alloc_urb(u,f)
#else
#  define SUBMIT_URB(u,f)       usb_submit_urb(u)
#  define USB_ALLOC_URB(u,f)    usb_alloc_urb(u)
#endif


inline void zd1211_DumpErrorCode(struct zd1205_private *macp, int err)
{
        switch (err)
        {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0))
        case USB_ST_CRC:
                ZD1211DEBUG(0, "USB_ST_CRC\n");
                break;

                //case USB_ST_BITSTUFF:
                //ZD1211DEBUG(0, "USB_ST_BITSTUFF\n");
                //break;

                //case USB_ST_NORESPONSE:
                //ZD1211DEBUG(0, "USB_ST_NORESPONSE\n");
                //break;

        case USB_ST_DATAOVERRUN:
                ZD1211DEBUG(0, "USB_ST_DATAOVERRUN\n");
                break;

        case USB_ST_DATAUNDERRUN:
                ZD1211DEBUG(0, "USB_ST_DATAUNDERRUN\n");
                break;

        case USB_ST_BUFFEROVERRUN:
                ZD1211DEBUG(0, "USB_ST_BUFFEROVERRUN\n");
                break;

        case USB_ST_BUFFERUNDERRUN:
                ZD1211DEBUG(0, "USB_ST_BUFFERUNDERRUN\n");
                break;

        case USB_ST_INTERNALERROR:
                ZD1211DEBUG(0, "USB_ST_INTERNALERROR\n");
                break;

                //case USB_ST_SHORT_PACKET:
                //ZD1211DEBUG(0, "USB_ST_SHORT_PACKET\n");
                //break;

        case USB_ST_PARTIAL_ERROR:
                ZD1211DEBUG(0, "USB_ST_PARTIAL_ERROR\n");
                break;

        case USB_ST_URB_KILLED:
                ZD1211DEBUG(0, "USB_ST_URB_KILLED\n");
                break;

        case USB_ST_URB_PENDING:
                ZD1211DEBUG(0, "USB_ST_URB_PENDING\n");
                break;

        case USB_ST_REMOVED:
                ZD1211DEBUG(0, "USB_ST_REMOVED\n");
                break;

        case USB_ST_TIMEOUT:
                ZD1211DEBUG(0, "USB_ST_TIMEOUT\n");
                break;

        case USB_ST_NOTSUPPORTED:
                ZD1211DEBUG(0, "USB_ST_NOTSUPPORTED\n");
                break;





        case USB_ST_BANDWIDTH_ERROR:
                ZD1211DEBUG(0, "USB_ST_BANDWIDTH_ERROR\n");
                break;

        case USB_ST_URB_INVALID_ERROR:
                ZD1211DEBUG(0, "USB_ST_URB_INVALID_ERROR\n");
                break;

        case USB_ST_URB_REQUEST_ERROR:
                ZD1211DEBUG(0, "USB_ST_URB_REQUEST_ERROR\n");
                break;

        case USB_ST_STALL:
                ZD1211DEBUG(0, "USB_ST_STALL\n");
                break;

        case -ENOMEM:
                ZD1211DEBUG(0, "ENOMEM\n");
                break;
#endif

        default:
                ZD1211DEBUG(0, "USB ST Code = %d\n",err);

                break;
        }
        /* ath_desc: reenable iwpriv dbg_flag */
        /* ath_old: macp->dbg_flag=0; */
        /* do not reset dbg_flag on USB error */

}


void zd1211_DumpReadMultipleReg(struct zd1205_private *macp, u16 adr0)
{
        u16  ReadAddr[cMAX_MULTI_READ_REG_NUM];
        u16  ReadData[cMAX_MULTI_READ_REG_NUM];
        u16  ReadIndex = 0;

        FPRINT_V("adr0", adr0);

        for (ReadIndex = 0; ReadIndex < cMAX_MULTI_READ_REG_NUM;)



                mFILL_READ_REGISTER(adr0++);

        zd1211_USB_PACKAGE_READ_REGISTER(ReadAddr, ReadData, ReadIndex, false);

        for (ReadIndex = 0; ReadIndex < 8; ReadIndex ++)
                printk("%04X, ", ReadData[ReadIndex]);

        printk("\n");
        printk("      ");

        for (; ReadIndex < cMAX_MULTI_READ_REG_NUM; ReadIndex ++)
                printk("%04X, ", ReadData[ReadIndex]);
        printk("\n");
}



// len0: in word, adr: word offset
void zd1211_WriteEEPROM(struct zd1205_private *macp, u16 rom_adr, u16 ram_adr, u16 len0)
{
        u32  tmpvalue;
        u16  WriteAddr[cMAX_MULTI_WRITE_REG_NUM];
        u16  WriteData[cMAX_MULTI_WRITE_REG_NUM];
        u16  WriteIndex = 0;

        tmpvalue = zd1211_readl(ZD1211_CLOCK_CTRL, false);
        mFILL_WRITE_REGISTER(ZD1211_CLOCK_CTRL, mSET_BIT((u16) tmpvalue, bZD1211_CLOCK_EEPROM));
        mFILL_WRITE_REGISTER(UMAC_EPROM_ROM_ADDR, rom_adr);
        mFILL_WRITE_REGISTER(UMAC_EPROM_RAM_ADDR, ram_adr);
        mFILL_WRITE_REGISTER(UMAC_EPROM_DMA_LEN_DIR, bmEPROM_XFER_DIR | len0);
        mFILL_WRITE_REGISTER(ZD1211_CLOCK_CTRL, mCLR_BIT((u16) tmpvalue, bZD1211_CLOCK_EEPROM));
        zd1211_WriteMultiRegister(WriteAddr, WriteData, WriteIndex, false);
}


#if fMERGE_RX_FRAME
int zd1211_ChangeToFlashAble(struct zd1205_private *macp)
{
        u32 tmpvalue;

        if (!macp->bFlashable)
        {
                int	LoadRet;

                zd_writel(0x01, FW_SOFT_RESET);

                macp->bDisableTx = 1;
                //USB_StopTxEP(macp);

                macp->bAllowAccessRegister = 0;

                LoadRet = zd1211_LoadUSBSpecCode(macp, WS11UPhm, sizeof(WS11UPhm),
                                                 cFIRMWARE_START_ADDR, true);
                if (LoadRet) {
                        FPRINT("Load WS11UPhm fail");
                        return 1;
                }

                ZD1211DEBUG(0, "Load WS11UPhm Done\n");

                //macp->bAllowAccessRegister = 1;
                macp->bFlashable = 1;

#if fWRITE_WORD_REG || fREAD_MUL_REG
                // Must get this information before any register write
                tmpvalue = zd1211_readl(cADDR_ENTRY_TABLE, FALSE);
                macp->AddrEntryTable = (u16) tmpvalue;
#endif

        }

        return 0;
}
#endif


int zd1211_UpdateBootCode(struct zd1205_private *macp, u16 *pCheckSum, u16 *pEEPROMData,
                          u32 EEPROMLen)
{
        u32 i;
        //int ret;
        u16  WriteAddr[cMAX_MULTI_WRITE_REG_NUM];
        u16  WriteData[cMAX_MULTI_WRITE_REG_NUM];
        u16  WriteIndex = 0;
        u16  ROMBufAdr = cBOOTCODE_START_ADDR;

        ZD1211DEBUG(0, "UpdateBootCode\n");

        for (i=0; i<EEPROMLen; i+=(2*WRITE_WORD_TO_EEPROM_PER_TIME))
        {
                for (WriteIndex=0; WriteIndex<WRITE_WORD_TO_EEPROM_PER_TIME/2; ) {
                        u16 WriteValue;
                        if (ROMBufAdr >= cINT_VECT_ADDR) {
                                FPRINT("Exceed max address");
                                break;
                        }
                        /* ath: fixed ambiguous statement */
                        WriteValue = pEEPROMData[WriteIndex * 2] | (pEEPROMData[WriteIndex * 2 + 1] << 8);
                        mFILL_WRITE_REGISTER(ROMBufAdr ++, WriteValue);
                }

                if (WriteIndex)
                        zd1211_WriteMultiRegister(WriteAddr, WriteData, WriteIndex, false);

                if (ROMBufAdr >= cINT_VECT_ADDR) {
                        FPRINT("Exceed max address1");
                        return 0;
                }

                pEEPROMData += WRITE_WORD_TO_EEPROM_PER_TIME;
        }

        if (EEPROMLen % (2*WRITE_WORD_TO_EEPROM_PER_TIME))
        {
                for (WriteIndex = 0; WriteIndex < (EEPROMLen % (2 * WRITE_WORD_TO_EEPROM_PER_TIME)) / 2;) {
                        u16 WriteValue;
                        if (ROMBufAdr >= cINT_VECT_ADDR) {
                                FPRINT("Exceed max address2");
                                break;
                        }
                        /* ath: fixed ambiguous statement */
                        WriteValue = pEEPROMData[WriteIndex * 2] | (pEEPROMData[WriteIndex * 2 + 1] << 8);
                        mFILL_WRITE_REGISTER(ROMBufAdr ++, WriteValue);

                }

                if (WriteIndex)
                        zd1211_WriteMultiRegister(WriteAddr, WriteData, WriteIndex, false);
        }

        return 0;
}




int zd1211_USB_Write_EEPROM(struct zd1205_private *macp, u16 *pEEPROMData, u32 EEPROMLen) //in bytes
{
        u16 CheckSum = 0;
        int ret;

        ZD1211DEBUG(0, "USB_Write_EEPROM\n");

        macp->bDisableTx = 1;

        //USB_StopTxEP(macp);

        ret = zd1211_UpdateBootCode(macp, &CheckSum, pEEPROMData, EEPROMLen);
        if (ret != 0)
                return ret;

        zd1211_WriteEEPROM(macp, 0, cBOOTCODE_START_ADDR, cEEPROM_SIZE - cLOAD_VECT_LEN);
        //macp->bDisableTx = 0;

        return 0;

}



int zd1211_USB_WRITE_EEPROM_DATA(struct zd1205_private *macp, PUSB_EEPROM_DATA	pData, int DataLen)
{
        int ret;
        u8 *pBuffer;
        //int memflags = GFP_KERNEL;

        ZD1211DEBUG(0, "USB_WRITE_EEPROM_DATA\n");

        if (!macp->bUSBDeveiceAttached)
        {
                return 1;
        }

        down(&macp->reg_sem);

        pBuffer = kmalloc(DataLen, GFP_KERNEL);

        if (!pBuffer)
        {
                up(&macp->reg_sem);
                return -ENOMEM;
        } else
                memcpy(pBuffer, (u8 *)pData, DataLen);

        if (macp->ep4isIntOut)
                usb_fill_int_urb(macp->reg_urb, macp->usb,
                                 usb_sndintpipe(macp->usb, EP_REG_OUT),
                                 pBuffer, DataLen,
                                 zd1211_reg_cb, macp, 1);
        else
                usb_fill_bulk_urb(macp->reg_urb, macp->usb,
                                  usb_sndbulkpipe(macp->usb, EP_REG_OUT),
                                  pBuffer, DataLen,
                                  zd1211_reg_cb, macp);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14))

        macp->reg_urb->transfer_flags |= URB_ASYNC_UNLINK;
#endif

        if ((ret = SUBMIT_URB(macp->reg_urb, GFP_ATOMIC)))
        {
                printk(KERN_ERR "zd1211: failed reg_urb\n");
                zd1211_DumpErrorCode(macp, ret);
                goto out;
        }

        wait_event(macp->regSet_wait, test_bit(ZD1211_CMD_FINISH, &macp->flags));
        clear_bit(ZD1211_CMD_FINISH, &macp->flags);

out:
        kfree(pBuffer);
        up(&macp->reg_sem);
        return ret;
}

#if fPROG_FLASH_BY_FW
int zd1211_USB_ProgramFlash(struct zd1205_private *macp, u16 *Value, u16 RegCount)
{
        u8 *pRegBuffer = NULL;

        int ret;
        u16 size = sizeof(USB_WRITE_REG);

        u16 bufSize;
        int ii;

        ZD1211DEBUG(0, "USB_ProgramFlash\n");

        if ((RegCount == 0) || (!macp->bUSBDeveiceAttached))
                return 0;

        down(&macp->reg_sem);
        pRegBuffer = kmalloc(size, GFP_KERNEL);

        if (!pRegBuffer)
        {
                up(&macp->reg_sem);
                return -ENOMEM;
        } else
                memset(pRegBuffer, 0x0, size);

        /* ath_desc: bigendian support */
        ((PUSB_WRITE_REG)pRegBuffer)->RequestID = cpu_to_le16(REGID_PROG_FLSH);
        ((PUSB_SET_RF) pRegBuffer)->Value       = cpu_to_le16(Value[0]);
        ((PUSB_SET_RF) pRegBuffer)->Index       = cpu_to_le16(Value[1]);

        for (ii = 2; ii < RegCount; ii ++)
                ((PUSB_SET_RF)pRegBuffer)->Data[ii - 2] = cpu_to_le16(Value[ii]);

        bufSize = sizeof(u16) * (1+RegCount);

        if (macp->ep4isIntOut)
                usb_fill_int_urb(macp->reg_urb, macp->usb,
                                 usb_sndintpipe(macp->usb, EP_REG_OUT),
                                 pRegBuffer, bufSize,
                                 zd1211_reg_cb, macp, 1);
        else
                usb_fill_bulk_urb(macp->reg_urb, macp->usb,
                                  usb_sndbulkpipe(macp->usb, EP_REG_OUT),
                                  pRegBuffer, bufSize,
                                  zd1211_reg_cb, macp);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14))

        macp->reg_urb->transfer_flags |= URB_ASYNC_UNLINK;
#endif

        if ((ret = SUBMIT_URB(macp->reg_urb, GFP_KERNEL)))
        {
                printk(KERN_ERR "zd1211: failed reg_urb\n");
                zd1211_DumpErrorCode(macp, ret);
                goto out;
        }

        wait_event(macp->regSet_wait, test_bit(ZD1211_CMD_FINISH, &macp->flags));
        clear_bit(ZD1211_CMD_FINISH, &macp->flags);

out:
        kfree(pRegBuffer);
        up(&macp->reg_sem);
        return ret;
}
#endif


// return 0: success
int zd1211_USB_PACKAGE_READ_REGISTER(u16 *Address, u16 *pValue, u16 RegCount, u8 bAddUSBCSRAddress)
{
        struct zd1205_private *macp = g_dev->priv;
        u8 *pRegBuffer = NULL;
        int ret = 0;
        u16 size = sizeof(USB_READ_REG_REQ);
        u16 bufSize;
        int ii;
        //int memflags = GFP_KERNEL;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0))

        if (in_interrupt()) {
#else

        if (in_atomic()) {
#endif
                printk(KERN_ERR "********zd1211_USB_PACKAGE_READ_REGISTER in_interrupt*********\n");
                return 0;
        }
        down(&macp->reg_sem);

        if ((RegCount == 0) || (!macp->bUSBDeveiceAttached) || (!test_bit(ZD1211_RUNNING, &macp->flags))) {
                up(&macp->reg_sem);
                return 0;
        }

        pRegBuffer = kmalloc(size, GFP_KERNEL);

        if (!pRegBuffer) {
                up(&macp->reg_sem);
                return -ENOMEM;
        } else
                memset(pRegBuffer, 0x0, size);

        ((PUSB_READ_REG_REQ)pRegBuffer)->RequestID  = cpu_to_le16(REGID_READ);

        for (ii = 0; ii < RegCount; ii ++) {
                if ((Address[ii] & BASE_ADDR_MASK_HOST) == USB_BASE_ADDR_HOST)
                        Address[ii] = Address[ii] - USB_BASE_ADDR_HOST + macp->AddrEntryTable;
                else if ((Address[ii] & BASE_ADDR_MASK_HOST) == USB_BASE_ADDR_EEPROM)
                        Address[ii] = ((Address[ii] - USB_BASE_ADDR_EEPROM) / 2) + cFIRMWARE_EEPROM_OFFSET;
                //0x9900                     //0xF817
                ((PUSB_READ_REG_REQ) pRegBuffer)->Address[ii] = cpu_to_le16(Address[ii]);
        }

        bufSize = sizeof(u16) * (1+RegCount);

        if (macp->ep4isIntOut)
                usb_fill_int_urb(macp->reg_urb, macp->usb,
                                 usb_sndintpipe(macp->usb, EP_REG_OUT),
                                 pRegBuffer, bufSize,
                                 zd1211_reg_cb, macp, 1);
        else
                usb_fill_bulk_urb(macp->reg_urb, macp->usb,
                                  usb_sndbulkpipe(macp->usb, EP_REG_OUT),
                                  pRegBuffer, bufSize,
                                  zd1211_reg_cb, macp);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14))

        macp->reg_urb->transfer_flags |= URB_ASYNC_UNLINK;
#endif

        if ((ret = SUBMIT_URB(macp->reg_urb, GFP_ATOMIC))) {
                printk(KERN_ERR "zd1211: failed reg_urb\n");
                zd1211_DumpErrorCode(macp, ret);
                up(&macp->reg_sem);
                kfree(pRegBuffer);
                return ret;
        }

        //wait command complete
        macp->regWaitRCompCnt++;
        //printk(KERN_ERR "before wait 4\n");
        wait_event(macp->regSet_wait, test_bit(ZD1211_CMD_FINISH, &macp->flags));
        //printk(KERN_ERR "after wait 4\n");
        macp->regRWCompCnt++;

        clear_bit(ZD1211_CMD_FINISH, &macp->flags);
        kfree(pRegBuffer);

        if (ret != 0)
                goto out;

        //wait response complete
        macp->regWaitRspCnt++;

#if 0//(LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0))

        if (wait_event_interruptible_timeout(macp->iorwRsp_wait, test_bit(ZD1211_REQ_COMP, &macp->flags), HZ/2)) { //use it, we may can't wake up

                //interrupt by a signal
                memset(macp->IntEPBuffer, 0, MAX_EPINT_BUFFER);
                macp->regUnCompCnt++;
                ret = -ERESTARTSYS;
                goto out;
        } else
                macp->regRspCompCnt++;
#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0))

        interruptible_sleep_on_timeout(&macp->iorwRsp_wait, 1); //magic delay
#else

        interruptible_sleep_on_timeout(&macp->iorwRsp_wait, 1); //magic delay
#endif

        //interruptible_sleep_on_timeout(&macp->iorwRsp_wait, HZ/40); //magic delay
        if (!test_bit(ZD1211_REQ_COMP, &macp->flags)) {
                //check if Rsp has completed, race condition may happen,
                macp->regRdSleepCnt++;
                //we waste time_out time
                //printk(KERN_ERR "before wait 2\n");
                interruptible_sleep_on_timeout(&macp->iorwRsp_wait, HZ/10);
                //wake up, check if timeout or ompleted

        }
        if (test_bit(ZD1211_REQ_COMP, &macp->flags))
                macp->regRspCompCnt++;
        else {
                memset(macp->IntEPBuffer, 0x0, MAX_EPINT_BUFFER);
                macp->regUnCompCnt++;
                ret = -1;
                goto out;
        }
#endif

        // Get data
        if ((macp->ReadRegCount == 0) || (macp->ReadRegCount > MAX_EPINT_BUFFER)) {
                ret = 1;
        } else {
                for (ii = 0; ii < (macp->ReadRegCount-2) / 4; ii++) {
                        pValue[ii] = cpu_to_le16(*((u16 *) macp->IntEPBuffer2 + 1 + ii * 2 + 1));
                }
                ret = 0;
        }

out:
        clear_bit(ZD1211_REQ_COMP, &macp->flags);
        up(&macp->reg_sem);
        return ret;
}

u32 zd1211_readl(u32 Address, u8 bAddUSBCSRAddress) {
        struct zd1205_private *macp = g_dev->priv;

        u16  ReadAddr[2];
        u16  ReadData[2];
        int bRet = 1;
        u32 value;
        int count = 0;

        if (bAddUSBCSRAddress) {
                Address += macp->USBCSRAddress;

                if ((Address & BASE_ADDR_MASK_HOST) == USB_BASE_ADDR_HOST)
                        ReadAddr[1] = (u16) Address + 1;
                else
                        ReadAddr[1] = (u16) Address + 2;
        } else
                ReadAddr[1] = (u16) Address + 1;

        ReadAddr[0] = (u16) Address;    // Read Low Word first

        while (bRet != 0) {
                bRet = zd1211_USB_PACKAGE_READ_REGISTER(ReadAddr, ReadData, 2, false);
                count++;

                if (count > 5) {
                        printk(KERN_ERR "1211_readl failed for 5 attempts...Very Serious");
                        break;
                }
        }

        value = (((u32) ReadData[1]) << 16) + ReadData[0];
        return value;
}

//return 0: success
int zd1211_USB_PACKAGE_WRITE_REGISTER(u16 *Address, u16 *Value, u16 RegCount, u8 bAddUSBCSRAddress) {
        struct zd1205_private *macp = g_dev->priv;
        u8 *pRegBuffer = NULL;
        int ret;
        u16 size = sizeof(USB_WRITE_REG);
        u16 bufSize;
        int i;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0))

        if (in_interrupt()) {
#else
        if (in_atomic()) {
#endif
                FPRINT("********zd1211_USB_PACKAGE_WRITE_REGISTER in_interrupt*********");
                return 0;
        }

        down(&macp->reg_sem);

        if ((RegCount == 0) || (!macp->bUSBDeveiceAttached) || !test_bit(ZD1211_RUNNING, &macp->flags)) {
                up(&macp->reg_sem);
                return 0;
        }

        pRegBuffer = kmalloc(size, GFP_KERNEL);
        if (!pRegBuffer) {
                up(&macp->reg_sem);
                return -ENOMEM;
        } else
                memset(pRegBuffer, 0x0, size);

        ((PUSB_WRITE_REG)pRegBuffer)->RequestID	= cpu_to_le16(REGID_WRITE);

        if (RegCount > cMIN_MULTI_WRITE_REG_NUM) {
                for (i=cMIN_MULTI_WRITE_REG_NUM; i<RegCount; i++) {
                        if (bAddUSBCSRAddress)
                                Address[i] += macp->USBCSRAddress;

                        if ((Address[i] & BASE_ADDR_MASK_HOST) == USB_BASE_ADDR_HOST)
                                Address[i] = Address[i] - USB_BASE_ADDR_HOST + macp->AddrEntryTable;
                        else if ((Address[i] & BASE_ADDR_MASK_HOST) == USB_BASE_ADDR_EEPROM)
                                Address[i] = ((Address[i] - USB_BASE_ADDR_EEPROM) / 2) + cFIRMWARE_EEPROM_OFFSET;

                        ((PUSB_WRITE_REG)pRegBuffer)->WritePackage[i].Address = cpu_to_le16(Address[i]);
                        ((PUSB_WRITE_REG)pRegBuffer)->WritePackage[i].WriteData_low = cpu_to_le16(Value[i]);
                }
        }

        bufSize = sizeof(u16) * (1+RegCount*2);

        if (macp->ep4isIntOut)
                usb_fill_int_urb(macp->reg_urb, macp->usb,
                                 usb_sndintpipe(macp->usb, EP_REG_OUT),
                                 pRegBuffer, bufSize,
                                 zd1211_reg_cb, macp, 1);
        else
                usb_fill_bulk_urb(macp->reg_urb, macp->usb,
                                  usb_sndbulkpipe(macp->usb, EP_REG_OUT),
                                  pRegBuffer, bufSize,
                                  zd1211_reg_cb, macp);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14))

        macp->reg_urb->transfer_flags |= URB_ASYNC_UNLINK;
#endif

        if ((ret = SUBMIT_URB(macp->reg_urb, GFP_ATOMIC))) {
                printk(KERN_ERR "zd1211: failed reg_urb\n");
                zd1211_DumpErrorCode(macp, ret);
                goto out;
        }

        macp->regWaitWCompCnt++;
        wait_event(macp->regSet_wait, test_bit(ZD1211_CMD_FINISH, &macp->flags));

        macp->regRWCompCnt++;
        clear_bit(ZD1211_CMD_FINISH, &macp->flags);

out:
        kfree(pRegBuffer);
        up(&macp->reg_sem);
        return ret;
}

int zd1211_WriteMultiRegister(u16 *Address, u16 *Value, u16 RegCount, u8 bAddUSBCSRAddress) {
        int ret = 1;
        int count = 0;

        while (ret != 0) {
                ret = zd1211_USB_PACKAGE_WRITE_REGISTER(Address, Value, RegCount, bAddUSBCSRAddress);

                count++;
                if (count > 5) {
                        FPRINT("zd1211_WriteMultiRegister failed");
                        break;
                }
        }

        return ret;
}


//return 0: success
int zd1211_writel(u32 Address, u32 Value, u8 bAddUSBCSRAddress) {
        struct zd1205_private *macp = g_dev->priv;
#ifdef fQuickPhySet

        u8	bIsPhyReg = 0;
#endif


        u16  WriteAddr[6];
        u16  WriteData[6];
        int ret = 1;
        int count = 0;

#ifdef fQuickPhySet

        if (bAddUSBCSRAddress && (Address <= ZD1205_PHY_END))

                bIsPhyReg = 1;
#endif


#ifdef fQuickPhySet

        if (bIsPhyReg) {
                u32	tmpvalue;

                tmpvalue = zd_readl(CtlReg1);
                tmpvalue &= ~0x80;
                {
                        if (((macp->USBCSRAddress+CtlReg1) & BASE_ADDR_MASK_HOST) == USB_BASE_ADDR_HOST)
                                WriteAddr[0] = (u16) (macp->USBCSRAddress+CtlReg1) + 1;
                        else

                                // must write High word first
                                WriteAddr[0] = (u16) (macp->USBCSRAddress+CtlReg1) + 2;
                }
                WriteData[0] = (u16) (tmpvalue >> 16);

                WriteAddr[1] = (u16) (macp->USBCSRAddress+CtlReg1);

                WriteData[1] = (u16) (tmpvalue & 0xFFFF);

                if (bAddUSBCSRAddress) {
                        Address += (u16) (macp->USBCSRAddress);
                        if ((Address & BASE_ADDR_MASK_HOST) == USB_BASE_ADDR_HOST)
                                WriteAddr[2] = (u16) Address + 1;
                        else
                                // must write High word first
                                WriteAddr[2] = (u16) Address + 2;
                } else
                        WriteAddr[2] = (u16) Address + 1;

                WriteData[2] = (u16) (Value >> 16);
                WriteAddr[3] = (u16) Address;

                WriteData[3] = (u16) (Value & 0xFFFF);


                tmpvalue |= 0x80;
                {

                        if (((macp->USBCSRAddress+CtlReg1) & BASE_ADDR_MASK_HOST) == USB_BASE_ADDR_HOST)

                                WriteAddr[4] = (u16) (macp->USBCSRAddress+CtlReg1) + 1;
                        else
                                // must write High word first
                                WriteAddr[4] = (u16) (macp->USBCSRAddress+CtlReg1) + 2;
                }
                WriteData[4] = (u16) (tmpvalue >> 16);
                WriteAddr[5] = (u16) (macp->USBCSRAddress+CtlReg1);
                WriteData[5] = (u16) (tmpvalue & 0xFFFF);

                return zd1211_USB_PACKAGE_WRITE_REGISTER(WriteAddr, WriteData, 6, false);
        } else {
#endif
                if (bAddUSBCSRAddress) {
                        Address += macp->USBCSRAddress;
                        if ((Address & BASE_ADDR_MASK_HOST) == USB_BASE_ADDR_HOST)
                                WriteAddr[0] = (u16) Address + 1;
                        else
                                // must write High word first
                                WriteAddr[0] = (u16) Address + 2;
                } else
                        WriteAddr[0] = (u16) Address + 1;

                WriteAddr[1] = (u16) Address;
                WriteData[0] = (u16) (Value >> 16);
                WriteData[1] = (u16) (Value & 0xFFFF);


                while (ret != 0) {
                        ret = zd1211_USB_PACKAGE_WRITE_REGISTER(WriteAddr, WriteData, 2, false);
                        count++;
                        if (count > 5) {

                                printk(KERN_ERR "zd1211_writel failed for 5 attempts\n");
                                break;
                        }
                }

#ifdef fQuickPhySet

        }
#endif

        return ret;
}



void zd1211_StrongSignalDect(struct zd1205_private *macp) {
        u32 tmpvalue;

        if ( (macp->PHYTestTimerCount >= macp->PHYTestTimer)&&
                        (macp->PHYTestTimer) && (macp->PHYLowPower & BIT_0)	&&
                        (!dot11Obj.bContinueTx) &&

                        (macp->bAssoc))
        {
                macp->PHYTestTimerCount = 0;
                if (macp->RF_Mode == RFMD_RF) {
                        if ( (macp->PHYTestRssi >= macp->PHYTestRssiBound) &&

                                        ((!dot11Obj.CR122Flag) || (dot11Obj.CR122Flag == 2))) {
                                LockPhyReg(&dot11Obj);
                                zd_writel(0xff, ZD1205_CR122);

                                if ( (macp->PHYLowPower & BIT_1)&&
                                                ((!dot11Obj.CR31Flag) || (dot11Obj.CR31Flag == 2)) ) {


                                        macp->bTraceSetPoint = 0;

                                        tmpvalue = dot11Obj.IntValue[dot11Obj.Channel-1] - cPWR_STRONG_SIG_DROP;

                                        zd_writel(tmpvalue, ZD1205_CR31);

                                        dot11Obj.CR31Flag = 1;
                                }


                                UnLockPhyReg(&dot11Obj);

                                dot11Obj.CR122Flag = 1;

                        } else if ( (macp->PHYTestRssi < macp->PHYTestRssiBound) &&
                                        ((dot11Obj.CR122Flag) || (dot11Obj.CR122Flag == 2)) ) {
                                LockPhyReg(&dot11Obj);
                                zd_writel(0x00, ZD1205_CR122);
                                UnLockPhyReg(&dot11Obj);

                                if ( (macp->PHYLowPower & BIT_1) &&

                                                ((dot11Obj.CR31Flag) || (dot11Obj.CR31Flag == 2)) ) {
                                        macp->bTraceSetPoint = 1;

                                        HW_UpdateIntegrationValue(&dot11Obj, dot11Obj.Channel, macp->cardSetting.MacMode);

                                        dot11Obj.CR31Flag = 0;

                                }


                                dot11Obj.CR122Flag = 0;
                        }

                } else if((macp->RF_Mode == AL2230_RF) || (macp->RF_Mode == AL2230S_RF)) {
                        if ( (macp->PHYTestRssi >= macp->PHYTestRssiBound)&&
                                        ((!dot11Obj.CR203Flag) || (dot11Obj.CR203Flag == 2)) ) {

                                LockPhyReg(&dot11Obj);

                                zd_writel(0x0a, ZD1205_CR203);
                                if ( (macp->PHYLowPower & BIT_1)&&
                                                ((!dot11Obj.CR31Flag) || (dot11Obj.CR31Flag == 2)) ) {



                                        macp->bTraceSetPoint = 0;

                                        tmpvalue = dot11Obj.IntValue[dot11Obj.Channel-1] - cPWR_STRONG_SIG_DROP;
                                        zd_writel(tmpvalue, ZD1205_CR31);
                                        dot11Obj.CR31Flag = 1;
                                }

                                UnLockPhyReg(&dot11Obj);
                                dot11Obj.CR203Flag = 1;
                        } else if ( (macp->PHYTestRssi < macp->PHYTestRssiBound)&&
                                        ((dot11Obj.CR203Flag) || (dot11Obj.CR203Flag == 2)) ) {
                                LockPhyReg(&dot11Obj);
                                zd_writel(0x06, ZD1205_CR203);
                                UnLockPhyReg(&dot11Obj);




                                if ( (macp->PHYLowPower & BIT_1)&&
                                                ((dot11Obj.CR31Flag) || (dot11Obj.CR31Flag == 2)) ) {

                                        macp->bTraceSetPoint = 1;


                                        HW_UpdateIntegrationValue(&dot11Obj, dot11Obj.Channel, macp->cardSetting.MacMode);
                                        dot11Obj.CR31Flag = 0;
                                }


                                dot11Obj.CR203Flag = 0;
                        }
                }
        } else
        {
                macp->PHYTestTimerCount++;

        }
}


//================================================================
// Housekeeping Every 0.5 s
//================================================================


void zd1211_TxCalibration(struct zd1205_private *macp) {
        static u32 loop = 0;

        static u16 TrackingLoop = 0;
        static u32 accumulate  = 0;
        u8 setpoint;
        u16 channel;
        u32	average = 0;
        u32	tmpvalue;
        static u16 TrackingCnt = 0;
        static u32 accumulate_OFDM = 0;
        static u16 TrackingCnt_OFDM = 0;
        u8  PreTxOFDMType = cTX_CCK;

        loop++;


#if fTX_PWR_CTRL

        if ((loop % 64) == 0)
        {
                if (macp->bTraceSetPoint) {


                        LockPhyReg(&dot11Obj);
                        if (TrackingLoop == TRACKING_NUM) {
                                TrackingLoop = 0;


                                if (TrackingCnt && PURE_A_MODE != macp->cardSetting.MacMode ) {
                                        average = (u32) (accumulate / TrackingCnt);
                                        channel = dot11Obj.Channel;
                                        setpoint = macp->EepSetPoint[channel-1];
                                        if (macp->EnableTxPwrCtrl) {
                                                if (average < (u32) (setpoint - cPWR_CTRL_GUARD))
                                                        zd1205_IncreaseTxPower(macp, cTX_CCK);
                                                else if (average > setpoint)

                                                        zd1205_DecreaseTxPower(macp, cTX_CCK);
                                        }
                                        accumulate = 0;
                                        TrackingCnt = 0;
                                }

                                if (TrackingCnt_OFDM) {
                                        average = (u32) (accumulate_OFDM / TrackingCnt_OFDM);
                                        channel = dot11Obj.Channel;
                                        if(PURE_A_MODE != macp->cardSetting.MacMode) {
                                                setpoint = macp->SetPointOFDM[macp->TxOFDMType - cTX_OFDM][channel - 1];
                                        } else if (PURE_A_MODE == macp->cardSetting.MacMode) {
                                                u8 UselessInt;//Only for store return Integration value that we don't need
                                                int ret;
                                                ret = a_OSC_get_cal_int(
                                                              channel,
                                                              macp->cardSetting.LastSentTxRate ,
                                                              &UselessInt, &setpoint);
                                                if(0 != ret)
                                                        printk("a_OSC_get_cal_int can't found the channel\n");
                                        }
                                        //printk("Enter TrackingCnt_OFDM(CH:%d)(SET:%d)(avg:%d)\n",channel,setpoint,average);
                                        if (macp->EnableTxPwrCtrl) {
                                                if (average < (u32) (setpoint - cPWR_CTRL_GUARD))
                                                        zd1205_IncreaseTxPower(macp, cTX_OFDM);
                                                else if (average > setpoint)
                                                        zd1205_DecreaseTxPower(macp, cTX_OFDM);
                                        }
                                        accumulate_OFDM = 0;
                                        TrackingCnt_OFDM = 0;
                                }
                        } else {

                                TrackingLoop ++;
                                tmpvalue = zd_readl(rLED_CTRL);
                                if (tmpvalue & BIT_0) {   // Continuous Tx
                                        if (tmpvalue & BIT_2) {   // Tx OFDM

                                                macp->TxPwrOFDM ++;
                                                macp->TxOFDMCnt = cTX_SENT_LEN + 1;
                                                tmpvalue = zd_readl(ZD1205_CR132);
                                                tmpvalue &= 0xFF;
                                                macp->TxOFDMType = cTX_OFDM;
                                                if (tmpvalue == 0xC)
                                                        macp->TxOFDMType = cTX_54M;

                                                else if (tmpvalue == 0x8)



                                                        macp->TxOFDMType = cTX_48M;
                                        } else
                                                macp->TxPwrCCK ++;   // Tx CCK
                                }

                                if (macp->TxPwrCCK) {  // New sent after last read
                                        tmpvalue = zd_readl(ZD1205_CR58);
                                        tmpvalue &= 0xFF;
                                        accumulate += tmpvalue;
                                        TrackingCnt ++;

                                        macp->TxPwrCCK = 0;

                                }


                                if (macp->TxPwrOFDM) {
                                        if (macp->TxOFDMCnt > cTX_SENT_LEN) { // make sure Tx by HMAC (for UMAC)
                                                tmpvalue = zd_readl(ZD1205_CR57);
                                                tmpvalue &= 0xFF;
                                                accumulate_OFDM += tmpvalue;
                                                TrackingCnt_OFDM ++;
                                                PreTxOFDMType = macp->TxOFDMType;
                                        } else {
                                                if (PreTxOFDMType != macp->TxOFDMType) {

                                                        accumulate_OFDM = 0;
                                                        TrackingCnt_OFDM = 0;
                                                }

                                        }
                                        macp->TxPwrOFDM = 0;
                                }
                        }
                        UnLockPhyReg(&dot11Obj);
                }
        }


#endif

}


//================================================================
// Housekeeping Every 1s


//================================================================
void zd1211_CheckWithIPC(struct zd1205_private *macp) {
        static u32 loop = 0;
        u8 BssType = macp->cardSetting.BssType;

        loop++;

        if ((loop % 10) == 0)
        {
                // bypass the weak signal in BSS and AP mode
                if ( (macp->bAssoc) &&
                                (macp->PHYTestRssi <= 0x18) &&
                                ((BssType == INDEPENDENT_BSS) ||
                                 (BssType == PSEUDO_IBSS)) ) {
                        if (!macp->CR138Flag) {

                                LockPhyReg(&dot11Obj);
                                zd_writel(0xa8, ZD1205_CR138);


                                UnLockPhyReg(&dot11Obj);
                                macp->CR138Flag = 1;
                        }
                }
                else if (macp->CR138Flag) {
                        LockPhyReg(&dot11Obj);
                        zd_writel(0x28, ZD1205_CR138);
                        UnLockPhyReg(&dot11Obj);
                        macp->CR138Flag = 0;
                }

#if 0
                // solve the throughput problem when communicate with the IPC card
                if ( ((macp->rxDataPerSec + macp->txDataPerSec) > 50000) &&

                                (macp->RF_Mode == RFMD_RF) &&
                                (BssType != PSEUDO_IBSS) &&
                                (macp->IPCFlag != 4) ) {


                        if ( (macp->rxDataPerSec > 3*macp->txDataPerSec) &&
                                        (macp->PHYTestRssi <= 0x24) ) {
                                if ((!macp->IPCFlag) || (macp->IPCFlag!=1)) {
                                        LockPhyReg(&dot11Obj);
                                        zd_writel(0x0a, ZD1205_CR87);
                                        zd_writel(0x04, ZD1205_CR89);
                                        UnLockPhyReg(&dot11Obj);
                                        macp->AdapterMaxRate = 8;  // MAX = 24M
                                        macp->IPCFlag = 1;
                                }
                        } else if ( 3*macp->rxDataPerSec < macp->txDataPerSec ) {
                                if ((!macp->IPCFlag) || (macp->IPCFlag != 3)) {
                                        LockPhyReg(&dot11Obj);
                                        zd_writel(0x2A, ZD1205_CR87);
                                        zd_writel(0x24, ZD1205_CR89);
                                        UnLockPhyReg(&dot11Obj);
                                        macp->AdapterMaxRate = 0x0B;  // MAX = 54M

                                        macp->IPCFlag = 3;
                                }
                        } else {
                                if ((!macp->IPCFlag) || (macp->IPCFlag != 2)) {
                                        LockPhyReg(&dot11Obj);
                                        zd_writel(0x10, ZD1205_CR87);
                                        zd_writel(0x0C, ZD1205_CR89);
                                        UnLockPhyReg(&dot11Obj);
                                        macp->AdapterMaxRate = 9;  // MAX = 36M
                                        macp->IPCFlag = 2;

                                }
                        }

                } else if ((macp->RF_Mode == RFMD_RF) &&

                                (BssType == PSEUDO_IBSS) &&
                                (macp->IPCFlag != 4)) {
                        if ((!macp->IPCFlag) || (macp->IPCFlag != 3)) {
                                LockPhyReg(&dot11Obj);
                                zd_writel(0x2A, ZD1205_CR87);
                                zd_writel(0x24, ZD1205_CR89);
                                UnLockPhyReg(&dot11Obj);
                                macp->AdapterMaxRate = 0x0B;  // MAX = 54M
                                macp->IPCFlag = 3;
                        }
                }

                else if ((macp->RF_Mode == RFMD_RF) &&
                                (macp->IPCFlag != 4) ) {
                        if ( (!macp->IPCFlag) || (macp->IPCFlag != 2)) {
                                LockPhyReg(&dot11Obj);
                                zd_writel(0x10, ZD1205_CR87);
                                zd_writel(0x0C, ZD1205_CR89);

                                UnLockPhyReg(&dot11Obj);
                                macp->AdapterMaxRate = 9;  // MAX = 36M
                                macp->IPCFlag = 2;
                        }
                }

                macp->rxDataPerSec = 0;
                macp->txDataPerSec = 0;
#endif

                if (macp->LinkLEDn == LED2)
                        iLED_OFF(macp, LED1);

                if (!macp->bAssoc) {
                        macp->LinkTimer ++;

                        if ((macp->LinkTimer == 1) && (macp->LinkLED_OnDur != 0)) {
                                iLED_ON(macp, macp->LinkLEDn);

                        }

                        if (macp->LinkTimer == (macp->LinkLED_OnDur + 1)) {
                                iLED_OFF(macp, macp->LinkLEDn);
                        }


                        if (macp->LinkTimer >= (macp->LinkLED_OnDur + macp->LinkLED_OffDur))
                                macp->LinkTimer = 0;
                }

#if 0
                if (dot11Obj.PhyTest & BIT_8) {

                        u32 tmpvalue;
                        LockPhyReg(&dot11Obj);
                        tmpvalue = zd_readl(ZD1205_CR122);
                        if ((tmpvalue & 0xFF) != 0xFF)
                                zd_writel(0xFF, ZD1205_CR122);

                        else
                                zd_writel(0x00, ZD1205_CR122);
                        UnLockPhyReg(&dot11Obj);
                }
#endif

        }// end of (loop % 10)
}


// Switch to another antenna
void zd1211_SwitchAntenna(struct zd1205_private *macp) {
        u32   tmpvalue;

        LockPhyReg(&dot11Obj);

        tmpvalue = zd_readl(ZD1205_CR10);
        tmpvalue ^= BIT_1;
        zd_writel(tmpvalue, ZD1205_CR10);

        tmpvalue = zd_readl(ZD1205_CR9);
        tmpvalue ^= BIT_2;
        zd_writel(tmpvalue, ZD1205_CR9);

        UnLockPhyReg(&dot11Obj);
}

//-----------------------------------------------------------------------------
#if fPROG_FLASH
// 1:Intel Flash;   0: MXIC, Winbond, AMD, Atmel...
#define cFLASH_MXIC             0
#define cFLASH_INTEL            1

u16 zd1211_SetHighAddr(struct zd1205_private *macp, u16 high_addr) {
        u16  tmp_cr203;
        u16  WriteAddr[cMAX_MULTI_WRITE_REG_NUM];
        u16  WriteData[cMAX_MULTI_WRITE_REG_NUM];
        u16  WriteIndex = 0;

        tmp_cr203 = ((high_addr << 1) & ~mMASK(2)) + (high_addr & mBIT(0));

        if (macp->FlashType == cFLASH_INTEL)
        {
                mFILL_WRITE_REGISTER(rLED_CTRL, 0);
                if (mTEST_BIT(high_addr, 7))
                        mFILL_WRITE_REGISTER(rLED_CTRL, LED2);
        }


        mFILL_WRITE_REGISTER(ZD1205_CR203, tmp_cr203);
        zd1211_WriteMultiRegister(WriteAddr, WriteData, WriteIndex, true);
        return tmp_cr203;
}


/* abs_addr: in word */
u16 zd1211_SetAbsAddr(struct zd1205_private *macp, u32 abs_addr, u16 *get_cr203)
{
        static u16 pre_high_addr = 0, pre_cr203 = 0;

        u16 high_addr;

        high_addr = (u16) (abs_addr >> 14);
        if (pre_high_addr != high_addr)
        {
                pre_cr203 = zd1211_SetHighAddr(macp, high_addr);
                pre_high_addr = high_addr;
        }


        if (get_cr203 != NULL)
                *get_cr203 = pre_cr203;

        return ((u16) (abs_addr & mMASK(14)));
}

void zd1211_FlashCmdWrite(struct zd1205_private *macp, u8 Cmd) {
        u32   tmpvalue;
        u16  WriteAddr[cMAX_MULTI_WRITE_REG_NUM];
        u16  WriteData[cMAX_MULTI_WRITE_REG_NUM];
        u16  WriteIndex = 0;

        tmpvalue = zd1211_readl(ZD1205_CR203, true);
        if (macp->FlashType == cFLASH_MXIC)
        {
                mFLASH_WRITE_EVEN_ADDR(0xAAA, 0xAA, tmpvalue);
                mFLASH_WRITE_ODD_ADDR(0x555, 0x55, tmpvalue);
                mFLASH_WRITE_EVEN_ADDR(0xAAA, Cmd, tmpvalue);
        } else

                mFLASH_WRITE_EVEN_ADDR(0, Cmd, tmpvalue);
        zd1211_WriteMultiRegister(WriteAddr, WriteData, WriteIndex, false);
}

void zd1211_FlashResetCmd(struct zd1205_private *macp) {
        if (macp->FlashType == cFLASH_MXIC)
                zd1211_writel(0, 0xF0, false);
        else
                zd1211_FlashCmdWrite(macp, 0xFF);
}



void zd1211_InitHighAddr(struct zd1205_private *macp) {
        u16  WriteAddr[cMAX_MULTI_WRITE_REG_NUM];

        u16  WriteData[cMAX_MULTI_WRITE_REG_NUM];
        u16  WriteIndex = 0;

        mFILL_WRITE_REGISTER(UMAC_WAIT_STATE, 0x022);   // 25ns * 2
        mFILL_WRITE_REGISTER(ZD1205_CR11 + (u16) (macp->USBCSRAddress), 0x15);
        // Use AntSel to control VPEN for Intel Flash
        mFILL_WRITE_REGISTER(ZD1205_CR10 + (u16) (macp->USBCSRAddress), 0x82);

        mFILL_WRITE_REGISTER(ZD1205_CR9 + (u16) (macp->USBCSRAddress), 0x24);
        mFILL_WRITE_REGISTER(ZD1205_CR204 + (u16) (macp->USBCSRAddress), 0x7C);
        mFILL_WRITE_REGISTER(ZD1205_CR203 + (u16) (macp->USBCSRAddress), 0);
        zd1211_WriteMultiRegister(WriteAddr, WriteData, WriteIndex, false);
        if (macp->FlashType == 0xFF)
        {
                u32  tmpvalue;


                macp->FlashType = cFLASH_INTEL;
                zd1211_FlashCmdWrite(macp, 0x90);       // Read Chip ID

                tmpvalue = zd1211_readl(0, false);
                if ((tmpvalue & 0xFFFF) != 0x8989)  // Intel Manufacture Code
                        macp->FlashType = cFLASH_MXIC;
        }
        zd1211_SetAbsAddr(macp, 0, NULL);
        zd1211_FlashResetCmd(macp);


}


/* Top: 8k byte / sector ==> 0 - 0x1000 (word address) */
/*   ==> sec0 address = 0; sec1 address = 0x1000 ... */
void zd1211_FlashSecErase(struct zd1205_private *macp, u16 Sec0) {
        u32  tmpvalue;
        u16  WriteAddr[cMAX_MULTI_WRITE_REG_NUM];
        u16  WriteData[cMAX_MULTI_WRITE_REG_NUM];
        u16  WriteIndex = 0;

        LockPhyReg(&dot11Obj);
        tmpvalue = zd1211_readl(ZD1205_CR203, true);
        if (macp->FlashType == cFLASH_MXIC)
        {
                zd1211_FlashCmdWrite(macp, 0x80);
                mFLASH_WRITE_EVEN_ADDR(0xAAA, 0xAA, tmpvalue);
                mFLASH_WRITE_ODD_ADDR(0x555, 0x55, tmpvalue);
                mFLASH_WRITE_EVEN_ADDR(Sec0 << 1, 0x30, tmpvalue);
        } else
        {
                mFLASH_SET_EVEN_ADDR(tmpvalue);
                mFILL_WRITE_REGISTER(Sec0, 0x20);

                mFILL_WRITE_REGISTER(Sec0, 0xD0);
        }

        zd1211_WriteMultiRegister(WriteAddr, WriteData, WriteIndex, false);
        UnLockPhyReg(&dot11Obj);
}





void zd1211_EraseFlash(struct zd1205_private *macp) {
        u32  ii;
        u32  tmpvalue;
        u16  low_addr, jj;

        macp->bDisableTx = 1;
        //USB_StopTxEP(Adapter);
        LockPhyReg(&dot11Obj);
        macp->bAllowAccessRegister = 0;


        zd1211_InitHighAddr(macp);

        if (macp->FlashType == cFLASH_MXIC)

        {
                zd1211_FlashCmdWrite(macp, 0x80);
                zd1211_FlashCmdWrite(macp, 0x10);
        } else
        {
                for (ii = 0; ii < 0x400000L; ii += 0x10000L) {
                        low_addr = zd1211_SetAbsAddr(macp, ii, NULL);
                        zd1211_FlashSecErase(macp, low_addr);
                        for (jj = 0; jj < 100; jj ++) {
                                tmpvalue = zd1211_readl(0, false);
                                if (tmpvalue & 0x8000)
                                        break;

                                mdelay(10);            // Sleep 10ms
                        }
                }

        }


        //macp->bAllowAccessRegister = 1;



        UnLockPhyReg(&dot11Obj);

        //macp->bDisableTx = 0;
}


#if !fPROG_FLASH_BY_FW
void FlashProgram(struct zd1205_private *macp, u16 addr0, u8 *pbuf, u16 tmpvalue) {
        u16  WriteAddr[cMAX_MULTI_WRITE_REG_NUM];
        u16  WriteData[cMAX_MULTI_WRITE_REG_NUM];
        u16  WriteIndex = 0;
        u16  jj;

        if (macp->FlashType == cFLASH_MXIC)
        {
                for (jj = 0; jj < 16; jj ++) {
                        WriteIndex = 0;
                        mFLASH_SET_EVEN_ADDR(tmpvalue);
                        zd1211_FlashCmdWrite(macp, 0xA0);
                        mFILL_WRITE_REGISTER(addr0 + jj, pbuf[jj * 2]);
                        zd1211_WriteMultiRegister(WriteAddr, WriteData, WriteIndex, false;
                                          }

                                          for (jj = 0; jj < 16; jj ++) {
                                                  WriteIndex = 0;
                                                  mFLASH_SET_ODD_ADDR(tmpvalue);
                                                  zd1211_FlashCmdWrite(macp, 0xA0);
                                                  mFILL_WRITE_REGISTER(addr0 + jj, pbuf[jj * 2 + 1]);

                                                  if (jj == 15) {
                                                          // Read Word Addr

                                                          mFLASH_SET_EVEN_ADDR(tmpvalue);
                                                  }

                                                  zd1211_WriteMultiRegister(WriteAddr, WriteData, WriteIndex, false);
                                          }
                                  } else
                                  {
                                          mFLASH_SET_EVEN_ADDR(tmpvalue);
                                          mFILL_WRITE_REGISTER(addr0, 0xE8);
                                          mFILL_WRITE_REGISTER(addr0, 0x0F);
                                          for (jj = 0; jj < 8; jj ++)
                                                  mFILL_WRITE_REGISTER(addr0 + jj, pbuf[jj * 2]);
                                          zd1211_WriteMultiRegister(WriteAddr, WriteData, WriteIndex, false);


                                          WriteIndex = 0;
                                          for (jj = 8; jj < 16; jj ++)

                                                  mFILL_WRITE_REGISTER(addr0 + jj, pbuf[jj * 2]);

                                          mFILL_WRITE_REGISTER(0, 0xD0);
                                          zd1211_WriteMultiRegister(WriteAddr, WriteData, WriteIndex, false);

                                          WriteIndex = 0;
                                          mFLASH_SET_ODD_ADDR(tmpvalue);
                                          mFILL_WRITE_REGISTER(addr0, 0xE8);
                                          mFILL_WRITE_REGISTER(addr0, 0x0F);
                                          for (jj = 0; jj < 8; jj ++)
                                                  mFILL_WRITE_REGISTER(addr0 + jj, pbuf[jj * 2 + 1]);
                                          zd1211_WriteMultiRegister(WriteAddr, WriteData, WriteIndex, false);

                                          WriteIndex = 0;
                                          for (jj = 8; jj < 16; jj ++)
                                                  mFILL_WRITE_REGISTER(addr0 + jj, pbuf[jj * 2 + 1]);
                                          mFILL_WRITE_REGISTER(0, 0xD0);
                                          zd1211_WriteMultiRegister(WriteAddr, WriteData, WriteIndex, false);
                                  }

                          }
#endif



                          int zd1211_ProgFlash(struct zd1205_private *macp, u32 StartAddr,
                                               u32 BufLenInBytes, u8 *pDownloadBuffer) {
                                  int i;
                                  u32   ii;
                                  u16  low_addr, jj;
                                  int chk_flg = 1;
                                  u16  tmpvalue;
#if fPROG_FLASH_BY_FW

                                  u16  WriteData[cMAX_MULTI_RF_REG_NUM];
                                  u16  WriteIndex = 0;
#endif

#ifdef fVERIFY_FLASH

                                  u16  ReadAddr[cMAX_MULTI_READ_REG_NUM];
                                  u16  ReadData[cMAX_MULTI_READ_REG_NUM];
                                  u16  ReadIndex = 0;
                                  u16  chk_cnt = 0;
#endif

                                  macp->bDisableTx = 1;
                                  //USB_StopTxEP(Adapter);

                                  LockPhyReg(&dot11Obj);
                                  macp->bAllowAccessRegister = 0;

                                  zd1211_InitHighAddr(macp);
                                  StartAddr /= 2;                         // Convert Byte Addr to Word Addr

                                  for (ii = 0; ii < BufLenInBytes / 2; ii += 16)
                                  {
                                          if (macp->FlashType == cFLASH_MXIC) {
                                                  if ((ii + StartAddr) >= 0x200000)   // 2M Word = 4M Byte
                                                          break;
                                          } else {
                                                  if ((ii + StartAddr) >= 0x400000)   // 4M Word = 8M Byte
                                                          break;
                                          }

                                          low_addr = zd1211_SetAbsAddr(macp, ii + StartAddr, &tmpvalue);

#if fPROG_FLASH_BY_FW

                                          WriteIndex = 0;
                                          tmpvalue &= 0x00FF;
                                          tmpvalue |= ((macp->FlashType) << 8);
                                          mFILL_RF_REGISTER(mCLR_BIT((tmpvalue), bmFLASH_A0));
                                          mFILL_RF_REGISTER(low_addr);

                                          for (jj = 0; jj < 16; jj ++)
                                                  mFILL_RF_REGISTER((u16) pDownloadBuffer[(ii + jj) * 2]);

                                          zd1211_USB_ProgramFlash(macp, WriteData, WriteIndex);
                                          WriteIndex = 0;
                                          mFILL_RF_REGISTER(mSET_BIT((tmpvalue), bmFLASH_A0));
                                          mFILL_RF_REGISTER(low_addr);

                                          for (jj = 0; jj < 16; jj ++)
                                                  mFILL_RF_REGISTER((u16) pDownloadBuffer[(ii + jj) * 2 + 1]);

                                          for (i=0; i<5; i++) {
                                                  if (zd1211_USB_ProgramFlash(macp, WriteData, WriteIndex))
                                                          break;
                                          }
#else
                                          zd1211_FlashProgram(macp, low_addr, pDownloadBuffer + 2 * ii, tmpvalue);
#endif

                                  }

                                  //macp->bAllowAccessRegister = 1;
                                  UnLockPhyReg(&dot11Obj);
                                  //macp->bDisableTx = 0;

                                  return chk_flg;
                          }

#endif

                          int zd1211_USB_SET_RF_REG(u16 *InputValue, int bIs3683A) {
                                  struct zd1205_private *macp = g_dev->priv;

                                  u8 *pRegBuffer = NULL;
                                  int ret;
                                  u16 size = sizeof(USB_SET_RF);
                                  u16 bufSize;
                                  u32	S_bit_cnt = dot11Obj.S_bit_cnt;
                                  u16 i;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0))

                                  if (in_interrupt()) {
#else
                                  if (in_atomic()) {
#endif
                                          FPRINT("********zd1211_USB_SET_RF_REG in_interrupt*********");
                                          return 0;
                                  }

                                  down(&macp->reg_sem);

                                  if (!(macp->bUSBDeveiceAttached)) {
                                          up(&macp->reg_sem);
                                          return 0;
                                  }

                                  pRegBuffer = kmalloc(size, GFP_KERNEL);

                                  if (!pRegBuffer) {
                                          up(&macp->reg_sem);
                                          return -ENOMEM;
                                  } else
                                          memset(pRegBuffer, 0x0, size);

                                  ((PUSB_SET_RF)pRegBuffer)->RequestID = cpu_to_le16(REGID_RFOFDMSET);

                                  if (bIs3683A)
                                          ((PUSB_SET_RF)pRegBuffer)->Value = cpu_to_le16(1);
                                  else
                                          ((PUSB_SET_RF)pRegBuffer)->Value = cpu_to_le16(2);

                                  ((PUSB_SET_RF)pRegBuffer)->Index = cpu_to_le16((u16)S_bit_cnt);

                                  for (i = 0; i < S_bit_cnt; i ++)
                                          ((PUSB_SET_RF)pRegBuffer)->Data[i] = cpu_to_le16(InputValue[i]);

                                  bufSize = sizeof(u16) * (3+S_bit_cnt);

                                  if (macp->ep4isIntOut)
                                          usb_fill_int_urb(macp->reg_urb, macp->usb,
                                                           usb_sndintpipe(macp->usb, EP_REG_OUT),
                                                           pRegBuffer, bufSize,
                                                           zd1211_reg_cb, macp, 1);
                                  else
                                          usb_fill_bulk_urb(macp->reg_urb, macp->usb,
                                                            usb_sndbulkpipe(macp->usb, EP_REG_OUT),
                                                            pRegBuffer, bufSize,
                                                            zd1211_reg_cb, macp);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14))

                                  macp->reg_urb->transfer_flags |= URB_ASYNC_UNLINK;
#endif

                                  if ((ret = SUBMIT_URB(macp->reg_urb, GFP_ATOMIC))) {
                                          printk(KERN_ERR "zd1211: failed rf reg_urb\n");
                                          zd1211_DumpErrorCode(macp, ret);
                                          goto out;
                                  }

                                  macp->regWaitWCompCnt++;
                                  wait_event(macp->regSet_wait, test_bit(ZD1211_CMD_FINISH, &macp->flags));
                                  macp->regRWCompCnt++;
                                  clear_bit(ZD1211_CMD_FINISH, &macp->flags);

                          out:
                                  kfree(pRegBuffer);
                                  up(&macp->reg_sem);
                                  return ret;
                          }

#define bmZD_IF_LE              1
#define bmZD_RF_CLK             2
#define bmZD_RF_DATA            3
                          void
                          HW_Set_IF_Synthesizer(zd_80211Obj_t *pObj, U32 InputValue) {
                                  u32	S_bit_cnt;
                                  u32	tmpvalue;
                                  u16 WriteData[cMAX_MULTI_RF_REG_NUM];
                                  u16 WriteIndex = 0;

                                  S_bit_cnt = pObj->S_bit_cnt;
                                  InputValue = InputValue << (31 - S_bit_cnt);

                                  //to avoid un-necessary register read/write
                                  //LockPhyReg(pObj);
                                  tmpvalue = zd_readl(ZD_CR203);
                                  tmpvalue = mCLR_BIT(tmpvalue, bmZD_IF_LE);

                                  // Configure RF by Software
                                  tmpvalue = mCLR_BIT(tmpvalue, bmZD_RF_CLK);

                                  while (S_bit_cnt) {
                                          InputValue = InputValue << 1;

                                          if (InputValue & 0x80000000) {
                                                  tmpvalue = mSET_BIT(tmpvalue, bmZD_RF_DATA);
                                                  mFILL_RF_REGISTER((u16) tmpvalue);
                                          } else {
                                                  tmpvalue = mCLR_BIT(tmpvalue, bmZD_RF_DATA);
                                                  mFILL_RF_REGISTER((u16) tmpvalue);
                                          }

                                          if (WriteIndex >= cMAX_MULTI_RF_REG_NUM) {
                                                  FPRINT_V("S_bit_cnt over range! ", (u32)pObj->S_bit_cnt);
                                                  break;
                                          }

                                          S_bit_cnt --;
                                  }

                                  zd1211_USB_SET_RF_REG(WriteData, 0);
                                  //UnLockPhyReg(pObj);
                          }

                          /* ath_desc: Defer tx kill_urb on Linux 2.6.8+ */
                          static void zd1211_kill_tx_urb(void *_macp) {
                                  struct zd1205_private *macp = (struct zd1205_private *)_macp;
                                  ZENTER(1);
                                  macp->tx_urb->transfer_flags |= URB_ASYNC_UNLINK;
                                  ZD_USB_KILL_URB(macp->tx_urb);
                          }

                          static void zd1211_tx_timeout(struct net_device *dev) {
                                  struct zd1205_private *macp = dev->priv;

                                  if (!macp)
                                          return;

                                  /* ath_desc: added Tx timeout reset */
                                  printk("%s: Tx timed out, trying reset\n", dev->name);
                                  macp->drv_stats.net_stats.tx_carrier_errors++;
                                  dev->trans_start = jiffies;
                                  defer_kevent(macp, KEVENT_UPDATE_SETTING);
                                  /* ath: On Linux 2.6.8+ usb_kill_urb() cannot be called from an interrupt context, so defer it */
                                  defer_kevent(macp, KEVENT_KILL_TX_URB);
                          }

                          int zd1211_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd) {
                                  struct zd1205_private *macp = dev->priv;

                                  memcpy(&macp->ifreq, ifr, sizeof(struct ifreq));
                                  macp->ifcmd = cmd;
                                  defer_kevent(macp, KEVENT_STD_IOCTL);
                                  return 0;
                          }
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
			  void process_event(struct zd1205_private *macp);
                          void scan_tout_event(struct work_struct * work)
                          {
                                  struct zd1205_private *macp =
                                          container_of(work, struct zd1205_private, scan_tout_event);
                                  process_event(macp);
                          }
			void kevent(struct work_struct * work)
			{
				struct zd1205_private *macp =
					container_of(work, struct zd1205_private, kevent);
				process_event(macp);
			}
#endif

#define	ZD1211_TX_TIMEOUT		(HZ*10)
#define	ZD1211_MTU			1500
                          extern struct iw_handler_def p80211wext_handler_def;

/* ath_desc: added iw_get_stats */
#ifdef CONFIG_NET_WIRELESS
extern struct iw_statistics *zd1205wext_iw_get_stats(struct net_device *);
#endif


u8 zd1211_InitSetup(struct net_device *dev, struct zd1205_private *macp) {
        int res;

        //return true; //for debug
        ZENTER(1);

        init_MUTEX(&macp->ps_sem);
        init_MUTEX(&macp->reg_sem);
        init_MUTEX(&macp->bcn_sem);
        init_MUTEX(&macp->config_sem);

        spin_lock_init(&(macp->intr_lock));
        spin_lock_init(&(macp->q_lock));
        spin_lock_init(&(macp->cs_lock));

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
        INIT_WORK(&macp->kevent, kevent);
        INIT_WORK(&macp->scan_tout_event, scan_tout_event);
#else
        INIT_WORK(&macp->kevent, kevent, macp);
        INIT_WORK(&macp->scan_tout_event, kevent, macp);
#endif

        macp->numTcb = NUM_TCB;
        macp->numTbd = NUM_TBD;
        macp->numRfd = NUM_RFD;
        macp->numTbdPerTcb = NUM_TBD_PER_TCB;
        macp->rxOffset  = ZD_RX_OFFSET;
        macp->rfd_size = 24; // form CbStatus to NextCbPhyAddrHighPart

        init_timer(&macp->watchdog_timer);
        macp->watchdog_timer.data = (unsigned long) dev;

        macp->watchdog_timer.function = (void *) &zd1205_watchdog_cb;

        init_timer(&macp->tm_hking_id);
        macp->tm_hking_id.data = (unsigned long) dev;
        macp->tm_hking_id.function = (void *) &HKeepingCB;

        init_timer(&macp->tm_mgt_id);
        macp->tm_mgt_id.data = (unsigned long) dev;
        macp->tm_mgt_id.function = (void *) &zd1205_mgt_mon_cb;

        /* ath_desc: AMD64 support */
        macp->USBCSRAddress = 0x9000;
        dot11Obj.reg = macp->regp=(void *)0x9000;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0))

        macp->in_interval = 10;
#else

        macp->in_interval = 10;
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0))

        usb_fill_int_urb(macp->intr_urb, macp->usb,
                         usb_rcvintpipe(macp->usb, EP_INT_IN),
                         macp->IntEPBuffer, MAX_EPINT_BUFFER,
                         zd1211_intr_cb, macp, macp->in_interval);
#else  //fake it

        usb_fill_bulk_urb(macp->intr_urb, macp->usb,
                          usb_rcvbulkpipe(macp->usb, EP_INT_IN),
                          macp->IntEPBuffer, MAX_EPINT_BUFFER,
                          zd1211_intr_cb, macp);
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14))

        macp->intr_urb->transfer_flags |= URB_ASYNC_UNLINK;
#endif
#if 0

        macp->intr_urb->transfer_dma = macp->IntBufferHandle;
        macp->intr_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
#endif

        if ((res = SUBMIT_URB(macp->intr_urb, GFP_KERNEL)))
        {
                printk(KERN_ERR "zd1211: failed intr_urb\n");
                zd1211_DumpErrorCode(macp, res);
                return false;
        }
        zd1205_init(macp);

        dev->open = zd1205_open;
        dev->stop = zd1205_close;
        dev->watchdog_timeo = ZD1211_TX_TIMEOUT;
        dev->do_ioctl = zd1205_ioctl;

#if WIRELESS_EXT > 12

        dev->wireless_handlers = (struct iw_handler_def *)&p80211wext_handler_def;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
        /* ath_desc: added iw_get_stats */
        /* ath: using p80211wext_handler_def.get_wireless_stats in 2.6.10+ */
        dev->get_wireless_stats = &zd1205wext_iw_get_stats;
#endif
#endif

        dev->hard_start_xmit = zd1205_xmit_frame;
        dev->set_multicast_list = zd1205_set_multi;
        dev->get_stats = zd1205_get_stats;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
#if ZDCONF_WE_STAT_SUPPORT == 1

        dev->get_wireless_stats = zd1205_iw_getstats;
#elif !defined(ZDCONF_WE_STAT_SUPPORT)
	#error "Undefine ZDCONF_WE_STAT_SUPPORT"
#endif
#endif

        dev->mtu = ZD1211_MTU;
        dev->set_mac_address = zd1205_set_mac;
        dev->tx_timeout = &zd1211_tx_timeout;

        //dev->features |= NETIF_F_SG | NETIF_F_HW_CSUM;
        dev->flags |= IFF_MULTICAST;

        //memcpy(macp->ifname, dev->name, IFNAMSIZ);
        //macp->ifname[IFNAMSIZ-1] = 0;

        ZEXIT(1);
        return true;
}

int zd1211_alloc_all_urbs(struct zd1205_private *macp) {
        struct usb_interface *interface = macp->interface;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0))

        struct usb_interface_descriptor *iface_desc = &interface->altsetting[0];
#else

        struct usb_host_interface *iface_desc = &interface->altsetting[0];
#endif

        struct usb_endpoint_descriptor *endpoint;
        struct usb_endpoint_descriptor *interrupt_in_endpoint[MAX_NUM_PORTS];
        struct usb_endpoint_descriptor *interrupt_out_endpoint[MAX_NUM_PORTS];
        struct usb_endpoint_descriptor *bulk_in_endpoint[MAX_NUM_PORTS];

        struct usb_endpoint_descriptor *bulk_out_endpoint[MAX_NUM_PORTS];

        u8 num_bulk_in = 0;
        u8 num_bulk_out = 0;

        u8 num_interrupt_in = 0;
        u8 num_interrupt_out = 0;

        int i;


        /* descriptor matches, let's find the endpoints needed */
        /* check out the endpoints */
        //ZD1211DEBUG(2, "bNumEndpoints = %d\n", iface_desc->bNumEndpoints);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0))

        for (i = 0; i < iface_desc->bNumEndpoints; ++i)
        {

                endpoint = &iface_desc->endpoint[i];
#else

        for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
                endpoint = &iface_desc->endpoint[i].desc;
#endif

                if ((endpoint->bEndpointAddress & 0x80) &&


                                ((endpoint->bmAttributes & 3) == 0x02)) {

                        /* we found a bulk in endpoint */
                        bulk_in_endpoint[num_bulk_in] = endpoint;
                        ++num_bulk_in;


/* ath: USB config fields are le16 on kernels >= 2.6.11 only */
/* ath: see Greg Kroah-Hartman, http://kernel.org/pub/linux/kernel/v2.6/testing/ChangeLog-2.6.11-rc1 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11))
                        macp->wMaxPacketSize = le16_to_cpu(endpoint->wMaxPacketSize);
                        ZD1211DEBUG(0, "bulk in: wMaxPacketSize = %x\n", le16_to_cpu(endpoint->wMaxPacketSize));
#else
                        macp->wMaxPacketSize = endpoint->wMaxPacketSize;
                        ZD1211DEBUG(0, "bulk in: wMaxPacketSize = %x\n", endpoint->wMaxPacketSize);
#endif
                }



                if (((endpoint->bEndpointAddress & 0x80) == 0x00) &&
                                ((endpoint->bmAttributes & 3) == 0x02)) {
                        /* we found a bulk out endpoint */
                        bulk_out_endpoint[num_bulk_out] = endpoint;
                        ++num_bulk_out;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11))
                        ZD1211DEBUG(0, "bulk out: wMaxPacketSize = %x\n", le16_to_cpu(endpoint->wMaxPacketSize));
#else
                        ZD1211DEBUG(0, "bulk out: wMaxPacketSize = %x\n", endpoint->wMaxPacketSize);
#endif

                }

                if ((endpoint->bEndpointAddress & 0x80) &&


                                ((endpoint->bmAttributes & 3) == 0x03)) {
                        /* we found a interrupt in endpoint */
                        interrupt_in_endpoint[num_interrupt_in] = endpoint;
                        ++num_interrupt_in;
                        macp->in_interval = endpoint->bInterval;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11))
                        ZD1211DEBUG(0, "interrupt in: wMaxPacketSize = %x\n", le16_to_cpu(endpoint->wMaxPacketSize));
#else
                        ZD1211DEBUG(0, "interrupt in: wMaxPacketSize = %x\n", endpoint->wMaxPacketSize);
#endif

                        ZD1211DEBUG(0, "interrupt in: int_interval = %d\n", endpoint->bInterval);

                }

                if (((endpoint->bEndpointAddress & 0x80) == 0x00) &&
                                ((endpoint->bmAttributes & 3) == 0x03)) {
                        /* we found a interrupt out endpoint */
                        interrupt_out_endpoint[num_interrupt_out] = endpoint;
                        ++num_interrupt_out;
                        macp->ep4isIntOut = 0;//1;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11))
                        ZD1211DEBUG(0, "interrupt out: wMaxPacketSize = %x\n", le16_to_cpu(endpoint->wMaxPacketSize));
#else
                        ZD1211DEBUG(0, "interrupt out: wMaxPacketSize = %x\n", endpoint->wMaxPacketSize);
#endif
                        macp->out_interval = endpoint->bInterval;
                }
        }


        macp->num_bulk_in = num_bulk_in;
        macp->num_bulk_out = num_bulk_out;
        macp->num_interrupt_in = num_interrupt_in;

        macp->num_interrupt_out = num_interrupt_out;


        macp->rx_urb = USB_ALLOC_URB(0, GFP_KERNEL);
        if (!macp->rx_urb)
                return 0;

        macp->tx_urb = USB_ALLOC_URB(0, GFP_KERNEL);
        if (!macp->tx_urb) {
                usb_free_urb(macp->rx_urb);
                return 0;
        }


        macp->intr_urb = USB_ALLOC_URB(0, GFP_KERNEL);
        if (!macp->intr_urb) {

                usb_free_urb(macp->rx_urb);
                usb_free_urb(macp->tx_urb);
                return 0;
        }

        macp->ctrl_urb = USB_ALLOC_URB(0, GFP_KERNEL);
        if (!macp->ctrl_urb) {
                usb_free_urb(macp->rx_urb);
                usb_free_urb(macp->tx_urb);
                usb_free_urb(macp->intr_urb);
                return 0;


        }


        macp->reg_urb = USB_ALLOC_URB(0, GFP_KERNEL);
        if (!macp->reg_urb) {
                usb_free_urb(macp->rx_urb);
                usb_free_urb(macp->tx_urb);
                usb_free_urb(macp->intr_urb);
                usb_free_urb(macp->ctrl_urb);
                return 0;
        }

#if 0//(LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0))
    #if 1
        macp->IntEPBuffer = kmalloc(MAX_EPINT_BUFFER, GFP_KERNEL);

#else //always failed? why???

        macp->IntEPBuffer = usb_buffer_alloc(macp->device,
                                             MAX_EPINT_BUFFER,
                                             GFP_KERNEL,
                                             &macp->IntBufferHandle);
#endif

        if (!macp->IntEPBuffer) {
                FPRINT("usb_buffer_alloc failed");
                usb_free_urb(macp->rx_urb);
                usb_free_urb(macp->tx_urb);
                usb_free_urb(macp->intr_urb);
                usb_free_urb(macp->ctrl_urb);

                usb_free_urb(macp->reg_urb);
                return 0;
        }
#endif

        return 1;
}




void zd1211_free_all_urbs(struct zd1205_private *macp)

{


        if (macp->rx_urb)
                usb_free_urb(macp->rx_urb);
        if (macp->tx_urb)

                usb_free_urb(macp->tx_urb);
        if (macp->intr_urb)

                usb_free_urb(macp->intr_urb);
        if 	(macp->ctrl_urb)
                usb_free_urb(macp->ctrl_urb);
        if 	(macp->reg_urb)
                usb_free_urb(macp->reg_urb);

#if 0//(LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0))

        if (macp->IntEPBuffer)
#if 1

                kfree(macp->IntEPBuffer);
#else

                usb_buffer_free(macp->device,
                                MAX_EPINT_BUFFER,
                                (void *)macp->IntEPBuffer,

                                macp->IntBufferHandle);
#endif
#endif
}


void zd1211_unlink_all_urbs(struct zd1205_private *macp)
{
        if (macp->rx_urb) {
                macp->rx_urb->transfer_flags &= ~URB_ASYNC_UNLINK;
                ZD_USB_KILL_URB(macp->rx_urb);
        }
        if (macp->tx_urb) {
                macp->tx_urb->transfer_flags &= ~URB_ASYNC_UNLINK;
                ZD_USB_KILL_URB(macp->tx_urb);
        }
        if (macp->ctrl_urb) {
                macp->ctrl_urb->transfer_flags &= ~URB_ASYNC_UNLINK;
                ZD_USB_KILL_URB(macp->ctrl_urb);
        }
        if (macp->reg_urb) {
                macp->reg_urb->transfer_flags &= ~URB_ASYNC_UNLINK;
                ZD_USB_KILL_URB(macp->reg_urb);
        }
        if (test_bit(ZD1211_UNPLUG, &macp->flags) && macp->intr_urb) {
                macp->intr_urb->transfer_flags &= ~URB_ASYNC_UNLINK;
                ZD_USB_KILL_URB(macp->intr_urb);
        }
}

#define MAX_RX_MERGE_PACKET_NUM		3
void zd1211_rx_isr(unsigned long parm)
{
        struct zd1205_private *macp = (struct zd1205_private *)parm;
        struct urb *urb = macp->read_urb;
        struct rx_list_elem *rx_struct;
#if fMERGE_RX_FRAME

        struct rx_list_elem *rx_struct_array[MAX_RX_MERGE_PACKET_NUM];
        int total_rx_struct = 1, rx_array_cnt = 0;
        int i;
        u32 tmpLen = 0;
        u16 last_pkt_len;
#endif

        u32 TotalLength = urb->actual_length;
        u8 *pRxBuffer;
        struct sk_buff *skb;
        zd1205_RFD_t *rfd = NULL;

        spin_lock(&macp->intr_lock);

        ZD1211DEBUG(4, "actual_length = %x\n", urb->actual_length);
        rx_struct = list_entry(macp->active_rx_list.next,
                               struct rx_list_elem, list_elem);

        skb = rx_struct->skb;
        rfd = RFD_POINTER(skb, macp);
        pRxBuffer = &rfd->RxBuffer[macp->rxOffset];

#if 0
        //for debug only
        zd1205_dump_data("pRxBuffer", (u8 *)pRxBuffer, TotalLength);
        //zd1211_submit_rx_urb(macp);
        //return;
#endif

#if fMERGE_RX_FRAME

        if (rx_struct->UnFinishFrmLen) {
                TotalLength += rx_struct->UnFinishFrmLen;
                rx_struct->UnFinishFrmLen = 0;
                macp->CompLenInfoCnt++;
                //ZD1211DEBUG(0, "Got Rx Frames Length Info!!\n");
        }

        last_pkt_len = TotalLength & (macp->wMaxPacketSize - 1);

        if (last_pkt_len <= (macp->wMaxPacketSize - 4)) {
                if (le16_to_cpu(((u16 *) pRxBuffer)[TotalLength / sizeof(u16) - 1]) == 0x697E) {
                        total_rx_struct = 3;
                        //ZD1211DEBUG(0, "Got merged Rx Frames!!\n");
                        //zd1205_dump_data("pRxBuffer", (u8 *)pRxBuffer, TotalLength);
                        macp->Continue2Rx++;
                } else
                        macp->NoMergedRxCnt++;

                //ZD1211DEBUG(3, "last_pkt_len = %x\n", last_pkt_len);
                //zd1205_dump_data("pRxBuffer", (u8 *)pRxBuffer, TotalLength);

                for (i=0; i<total_rx_struct; i++) {
                        int CurFrmLen;

                        if (total_rx_struct> 1) {
                                CurFrmLen = cpu_to_le16(((u16 *) pRxBuffer)[TotalLength / sizeof(u16) + i - 4]);
                                //ZD1211DEBUG(2, "CurFrmLen = %x\n", CurFrmLen);
                        } else
                                CurFrmLen = TotalLength;

                        if (CurFrmLen == 0) {
                                break;
                        }

                        rx_struct_array[i] = list_entry(macp->active_rx_list.next,
                                                        struct rx_list_elem, list_elem);
                        list_del(&(rx_struct_array[i]->list_elem));
                        rx_array_cnt++;

                        ZD1211DEBUG(2, "CurFrmLen = %x\n", CurFrmLen);

                        skb = rx_struct_array[i]->skb;
                        rfd = RFD_POINTER(skb, macp);

                        rfd->CbStatus = cpu_to_le32(RFD_STATUS_COMPLETE);
                        rfd->ActualCount = cpu_to_le32(CurFrmLen);

                        if (i > 0) {
                                memcpy(&rfd->RxBuffer[macp->rxOffset],
                                       pRxBuffer + tmpLen,
                                       CurFrmLen);
                        }

                        tmpLen += (CurFrmLen & ~0x03);

                        if (CurFrmLen & 0x03)
                                tmpLen += 4;
                        rfd->ActualCount = cpu_to_le32(CurFrmLen + macp->rxOffset);
                }
        } else {
                // last_pkt_len = 509, 510, 511
                // wait next Rx
                //ZD1211DEBUG(0, "Wait Rx Frames Length Info!!\n");
                //ZD1211DEBUG(2, "last_pkt_len = %x\n", last_pkt_len);
                macp->WaitLenInfoCnt++;
                rx_struct->UnFinishFrmLen = ((TotalLength / macp->wMaxPacketSize) + 1)
                                            * (macp->wMaxPacketSize);
                //zd1205_dump_data("pRxBuffer", (u8 *)pRxBuffer, TotalLength);
        }


        zd1211_submit_rx_urb(macp);

        if (!rx_struct->UnFinishFrmLen) {
                macp->total_rx_cnt = rx_array_cnt;
                macp->rx_struct_array = rx_struct_array;
                zd1205_rx_isr(macp);
        }


#else
        rfd->CbStatus = cpu_to_le32(RFD_STATUS_COMPLETE);
        rfd->ActualCount = cpu_to_le32(TotalLength + macp->rxOffset);
        zd1205_rx_isr(macp);
#endif

        if (dot11Obj.QueueFlag & MGT_QUEUE_SET)
                defer_kevent(macp, KEVENT_PROCESS_SIGNAL);
        spin_unlock(&macp->intr_lock);
}





#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)||LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
void zd1211_rx_comp_cb(struct urb *urb)
#else
void zd1211_rx_comp_cb(struct urb *urb, struct pt_regs *regs)
#endif
{
        struct zd1205_private *macp = urb->context;



        if ((!macp) || !test_bit(ZD1211_RUNNING, &macp->flags))

                return;


        if (!netif_device_present(macp->device))
                return;

        if (urb->status != 0)
        {
                zd1211_DumpErrorCode(macp, urb->status);
                if ((urb->status != -ENOENT) &&

                                (urb->status != -ECONNRESET) &&

                                (urb->status != -ESHUTDOWN)) {
                        //printk("nonzero read bulk status received: %d\n", urb->status);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0))
                        if (urb->status == USB_ST_INTERNALERROR)

                                return;

#else

                        if (urb->status == -EPIPE) {
                                printk("nonzero read bulk status received: -EPIPE\n");
                                return;
                        }

                        if (urb->status == -EPROTO) {
                                printk("nonzero read bulk status received: -EPROTO\n");
                                return;
                        }



#endif


                        zd1211_submit_rx_urb(macp);
                        return;
                }
                return;

        }


        if (urb->actual_length == 0)
        {
                FPRINT("Got Length = 0");

                zd1211_submit_rx_urb(macp);
                return;
        }


        macp->read_urb = urb;
        zd1211_rx_isr((unsigned long) macp);

}





//callback function for interrupt or response
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)||LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
void zd1211_intr_cb(struct urb *urb)
#else
void zd1211_intr_cb(struct urb *urb, struct pt_regs *regs)
#endif
{
        struct zd1205_private *macp = urb->context;
        u16 intNum;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0))
        int status;
#endif
        u32 actual_length = urb->actual_length;

        if (!macp)
                return;

        spin_lock(&macp->intr_lock);

        if (urb->status != 0)
        {
                zd1211_DumpErrorCode(macp, urb->status);

                if (urb->status == -ENODEV) { //device was removed
                        FPRINT("Device was removed!!!");
                        macp->bUSBDeveiceAttached = 0;

                        wake_up(&macp->regSet_wait);
                        wake_up_interruptible(&macp->iorwRsp_wait);

                        spin_unlock(&macp->intr_lock);
                        return;
                }

                switch (urb->status) {
                case -ECONNRESET:
                case -ENOENT:
                case -ESHUTDOWN:
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0))

                case -EPROTO:
#endif

                        macp->bUSBDeveiceAttached = 0;
                        FPRINT("Device was down!!!");
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0))

                        spin_unlock(&macp->intr_lock);
                        return;
#endif

                        break;

                default:
                        //printk("nonzero intr status received: %d\n", urb->status);
                        break;
                }

        } else
        {
                if (macp->IntEPBuffer[0] != 0x01)
                        FPRINT("Got unknown packet");

                switch (macp->IntEPBuffer[1]) {
                case EPINT_IORDRsp:

                        // this is USB_READ_REGISTER response
                        macp->ReadRegCount = (u16)actual_length;

                        intNum = le16_to_cpu(*(u16 *)(macp->IntEPBuffer+2));
                        //intNum = macp->IntEPBuffer[2] + (((u16)macp->IntEPBuffer[3]) << 8);

                        if (intNum == (InterruptCtrl | macp->USBCSRAddress)) {
                                // Handle non-RxTx interrupt
                                if (macp->bHandleNonRxTxRunning) {
                                        printk("Impossible, interrupt happen!!!!! %x\n", intNum);
                                        break;
                                }

                                // disable non-RxTx interrupt
                                // No needed to diable interrupt, firmware will do it.
                                macp->bHandleNonRxTxRunning = 1;
                                memcpy(macp->IntEPBuffer3, macp->IntEPBuffer, MAX_EPINT_BUFFER);
                                defer_kevent(macp, KEVENT_NON_TX_RX_INT);

                        } else {
                                // handle read register
                                memcpy(macp->IntEPBuffer2, macp->IntEPBuffer, MAX_EPINT_BUFFER);
                                set_bit(ZD1211_REQ_COMP, &macp->flags);
                                wake_up_interruptible(&macp->iorwRsp_wait);
                        }
                        break;

                case EPINT_RetryFial_Event: {
                                u8 *pMacAddr = macp->IntEPBuffer + 4;
                                /* ath: not endian safe! u8 NewRate = (u8)(*(u16 *)(macp->IntEPBuffer + 2)); */
                                //u8 NewRate = macp->IntEPBuffer[2];
                                u16 aid;

#if fTX_PWR_CTRL

                                if (macp->TxOFDMType >= cTX_48M)
                                        macp->TxOFDMCnt = 0;
#endif

                                if ( macp->IntEPBuffer[10] & BIT_0 )

                                        macp->bIBSS_Wakeup_Dest = 1;


                                //ZD1211DEBUG(2, "Retry Failed!!!\n");
                                //ZD1211DEBUG(2, "NewRate = %x\n", NewRate);
                                aid = zd_AidLookUp(pMacAddr);
                                zd_EventNotify(EVENT_TX_COMPLETE, ZD_RETRY_FAILED, 0xff, (U32)aid);
                                macp->retryFailCnt +=  le16_to_cpu(*(u16 *)(macp->IntEPBuffer + 10));
                                break;

                        }


                default:
                        FPRINT("Got Unknown interrupt!!!");
                        break;
                }

        }

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0))
#if 1
        //memset(macp->IntEPBuffer, 0x0, MAX_EPINT_BUFFER);

        //use bulk instead of interrupt in
        usb_fill_bulk_urb(macp->intr_urb, macp->usb,
                          usb_rcvbulkpipe(macp->usb, EP_INT_IN),
                          macp->IntEPBuffer, MAX_EPINT_BUFFER,
                          zd1211_intr_cb, macp);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14))

        macp->intr_urb->transfer_flags |= URB_ASYNC_UNLINK;
#endif

        status = SUBMIT_URB(macp->intr_urb, GFP_ATOMIC);
#else

        status = SUBMIT_URB(urb, GFP_ATOMIC);
#endif

        if (status)
                FPRINT("Can't resubmit interrupt urb!!!");
#endif

        spin_unlock(&macp->intr_lock);

        return;
}


//callback function for register get/set
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)||LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
void zd1211_reg_cb(struct urb *urb)
#else
void zd1211_reg_cb(struct urb *urb, struct pt_regs *regs)
#endif
{
        struct zd1205_private *macp = urb->context;
        set_bit(ZD1211_CMD_FINISH, &macp->flags);
        wake_up(&macp->regSet_wait);
}

void zd1211_handle_non_tx_rx(struct zd1205_private *macp)
{
        u32	intr_status;
        // in current design, no need to use spinlock

        intr_status = le16_to_cpu(*(u16 *)(macp->IntEPBuffer3+4));
        //ZD1211DEBUG(2, "intr_status = %x\n", intr_status);

        if (!intr_status)
                return;

        if (intr_status & WAKE_UP)
        {
                ZD1211DEBUG(1, "WAKE_UP\n");
                //printk(KERN_ERR "befor1 : %lu\n", jiffies);
                down(&macp->ps_sem);
                //printk(KERN_ERR "before2: %lu\n", jiffies);
                if (dot11Obj.bDeviceInSleep) {
                        zd1205_process_wakeup(macp);
                }

                //printk(KERN_ERR "after2: %lu\n", jiffies);
                up(&macp->ps_sem);
                //printk(KERN_ERR "after1: %lu\n", jiffies);

        }

        if (intr_status & CFG_NEXT_BCN)
        {
                //ZD1211DEBUG(2, "CFG_NEXT_BCN\n");
                if (macp->config_next_bcn_en) {
                        if (macp->cardSetting.BssType == AP_BSS)
                                goto done;

                        macp->bcnCnt++;
                        down(&macp->bcn_sem);
                        zd_EventNotify(EVENT_TBCN, 0, 0, 0);
                        up(&macp->bcn_sem);

                        if (macp->cardSetting.BssType == INDEPENDENT_BSS) {
                                macp->bFrmRxed1 = 0;
                        }
#if 0
                        else if (macp->cardSetting.BssType == AP_BSS) {
                                if (macp->dtimCount == 0)
                                        macp->dtimCount = macp->cardSetting.DtimPeriod;
                                macp->dtimCount--;
                        }
#endif

                }
        }

        if (intr_status & DTIM_NOTIFY)
        {
                ZD1211DEBUG(2, "DTIM_NOTIFY\n");
                if (macp->dtim_notify_en) {
                        macp->dtimCnt++;
                        zd_EventNotify(EVENT_DTIM_NOTIFY, 0, 0, 0);
                }
        }

done:
        macp->bHandleNonRxTxRunning = 0;

        // enable non-RxTx interrupt
        zd1205_enable_int();
        return;
}

int zd1211_submit_rx_urb(struct zd1205_private *macp)
{
        struct rx_list_elem *rx_struct = NULL;
        u8 *rx_buff = NULL;
        u32 bufLen = MAX_WLAN_SIZE - macp->rxOffset;
        int res;
        //int memflags = GFP_KERNEL;



        rx_struct = zd1205_start_ru(macp);

        if (!rx_struct)
                return 1;

#if fMERGE_RX_FRAME

        if (rx_struct->UnFinishFrmLen)
        {
                rx_buff = (u8 *)(rx_struct->dma_addr)+ macp->rxOffset + (rx_struct->UnFinishFrmLen);
                bufLen -= (rx_struct->UnFinishFrmLen);
        } else
                rx_buff = (u8 *)(rx_struct->dma_addr) + macp->rxOffset;


#else

        rx_buff = (u8 *)(rx_struct->dma_addr) + macp->rxOffset;
#endif


        usb_fill_bulk_urb(macp->rx_urb, macp->usb,
                          usb_rcvbulkpipe(macp->usb, EP_DATA_IN),
                          rx_buff, bufLen,

                          zd1211_rx_comp_cb, macp);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14))

        macp->rx_urb->transfer_flags |= URB_ASYNC_UNLINK;
#endif

        if ((res = SUBMIT_URB(macp->rx_urb, GFP_ATOMIC)))
        {
                printk(KERN_ERR "zd1211: failed rx_urb\n");
                zd1211_DumpErrorCode(macp, res);
        }



        return 0;
}



void zd1211_tx_isr(unsigned long parm)
{
        struct zd1205_private *macp = (struct zd1205_private *)parm;
        zd1205_SwTcb_t *sw_tcb;

        ZENTER(3);

        spin_lock(&macp->intr_lock);


        sw_tcb = macp->activeTxQ->first;
        if (sw_tcb)
                sw_tcb->pTcb->CbStatus = cpu_to_le32(CB_STATUS_COMPLETE);

        zd1205_tx_isr(macp);

#if 0

        if (dot11Obj.QueueFlag & TX_QUEUE_SET) {
                macp->txQueSetCnt++;

                tasklet_schedule(&macp->zd1205_tx_tasklet);
        }
#endif


        spin_unlock(&macp->intr_lock);

        ZEXIT(3);

}



#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)||LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
void zd1211_tx_comp_cb(struct urb *urb)
#else
void zd1211_tx_comp_cb(struct urb *urb, struct pt_regs *regs)

#endif
{
        struct zd1205_private *macp = urb->context;


        if ((!macp) || !test_bit(ZD1211_RUNNING, &macp->flags))
        {
                printk(KERN_ERR "macp is NULL or macp.flags.ZD1211_RUNNING\n");
                return;
        }

        if (!netif_device_present(macp->device))
        {
                printk(KERN_ERR "netif_device_present return NULL\n");
                return;
        }

        if (urb->status)
        {
                /* ath_desc: fix printks */
                printk(KERN_ERR "Tx status: %d\n", urb->status);
                zd1211_DumpErrorCode(macp, urb->status);
        }

        macp->usbTxCompCnt++;
        clear_bit(ZD1211_TX_BUSY, &macp->flags);

        macp->TxStartTime = 0;

        macp->write_urb = urb;

        zd1211_tx_isr((unsigned long) macp);

        return;
}






// return send_length
int zd1211_submit_tx_urb(struct zd1205_private *macp,BOOLEAN LastFrag)
{
        u8 *dst = macp->tx_buff;


        u32 TotalLength = 0;

        zd1205_Ctrl_Set_t *pCtrlSet;
        zd1205_TBD_t *Tbd;
        zd1205_SwTcb_t *sw_tcb;
        int res;
        //int memflags = GFP_KERNEL;



        if (!macp->activeTxQ->count)
        {
                return 0;

        }

        if (test_bit(ZD1211_TX_BUSY, &macp->flags))
                return 0;

        ZENTER(3);

        sw_tcb = macp->activeTxQ->first;
        pCtrlSet = sw_tcb->pHwCtrlPtr;
        Tbd = sw_tcb->pFirstTbd;

        // We skip TCB address, Address 1, NextLength = 16 bytes, add 2 bytes for total length
        //pCtrlSet->CtrlSetting[23]=0;
        //pCtrlSet->CtrlSetting[24]=0;

        memcpy(dst, (u8 *)(pCtrlSet), 3);
        dst += 3;

        TotalLength += 3;

        memcpy(dst,	(u8 *)(pCtrlSet)+(3+8),	// 8 mean skip TCB address

               1);					// misc

        dst += 1;


        TotalLength += 1;


        dst += 2;							// reserver 2 bytes for total length

        TotalLength += 2;



        memcpy(dst,	(u8 *)(pCtrlSet)+(3+8+1+6+2),	// 6:skip address 1, 2:skip next length
               5);

        dst += 5;

        TotalLength += 5;

        ZD1211DEBUG(2, "Tx Ctrl Length = %x\n", TotalLength);

        Tbd++;

        /* ath_desc: removed "Non-Zero Duration" debug output */
        #if 0
        //Mac Header
        if(*(u8 *)(le32_to_cpu(Tbd->TbdBufferAddrLowPart)+2)!=0)
        {
                if(macp->bPSMSupported != 1)
                        *(u16 *)(le32_to_cpu(Tbd->TbdBufferAddrLowPart)+2)=0;
                //printk("Non-Zero Duration,%d\n",*(u16 *)(le32_to_cpu(Tbd->TbdBufferAddrLowPart)+2));
                //printk("\n OK\n");
        } else if(*(u8 *)(le32_to_cpu(Tbd->TbdBufferAddrLowPart)+3)!=0)
        {
                if(macp->bPSMSupported != 1)
                        *(u16 *)(le32_to_cpu(Tbd->TbdBufferAddrLowPart)+2)=0;
                //printk("Non-Zero Duration,%d\n",*(u16 *)(le32_to_cpu(Tbd->TbdBufferAddrLowPart)+2));
                //printk("\n OK\n");
        }
        #endif


#ifndef __LP64__
        memcpy(dst, (u8 *)le32_to_cpu(Tbd->TbdBufferAddrLowPart), le32_to_cpu(Tbd->TbdCount));

#else
        memcpy(dst, (u8 *)le64_to_cpu(Tbd->TbdBufferAddr), le32_to_cpu(Tbd->TbdCount));
#endif
        ZD1211DEBUG(2, "MAC Header Length = %x\n", le32_to_cpu(Tbd->TbdCount));
        dst += le32_to_cpu(Tbd->TbdCount);

        TotalLength += le32_to_cpu(Tbd->TbdCount);

        Tbd++;

        //MAC Body
#ifndef __LP64__
        memcpy(dst, (u8 *)le32_to_cpu(Tbd->TbdBufferAddrLowPart), le32_to_cpu(Tbd->TbdCount));

#else
        memcpy(dst, (u8 *)le64_to_cpu(Tbd->TbdBufferAddr), le32_to_cpu(Tbd->TbdCount));
#endif
        ZD1211DEBUG(2, "Tx DATA Length = %x\n", le32_to_cpu(Tbd->TbdCount));
        dst += le32_to_cpu(Tbd->TbdCount);


        TotalLength += le32_to_cpu(Tbd->TbdCount);
        Tbd++;

        ZD1211DEBUG(2, "TotalLength = %x\n", TotalLength);

        if(LastFrag == TRUE)
        {
                memcpy(dst-sw_tcb->MIC_Len,sw_tcb->CalMIC+sw_tcb->MIC_Start,sw_tcb->MIC_Len);

        }



        // write down total length
#ifdef ZD1211
        *((u16 *)(macp->tx_buff+4)) = cpu_to_le16((u16)TotalLength + 14);
#elif defined(ZD1211B)

        *((u16 *)(macp->tx_buff+4)) = cpu_to_le16(sw_tcb->LengthDiff);
#endif

        ZD1211DEBUG(2, "macp->tx_buff+4 = %x\n", le16_to_cpu(*((u16 *)(macp->tx_buff+4))));


        usb_fill_bulk_urb(macp->tx_urb, macp->usb,
                          usb_sndbulkpipe(macp->usb, EP_DATA_OUT),
                          macp->tx_buff, TotalLength,
                          zd1211_tx_comp_cb, macp);



        macp->tx_urb->transfer_buffer_length = TotalLength;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14))

        macp->tx_urb->transfer_flags |= URB_ASYNC_UNLINK;
#endif

        res = SUBMIT_URB(macp->tx_urb, GFP_ATOMIC);
        if (res)
        {

                printk("error in tx submit urb: %d", res);
                zd1211_DumpErrorCode(macp, res);
                goto err;
        }

        macp->usbTxCnt++;

        set_bit(ZD1211_TX_BUSY, &macp->flags);




        ZEXIT(3);

        return TotalLength;



err:
        return 0;
}



void zd1211_disable_net_traffic(struct zd1205_private *macp)
{

        // When suspend, call this
        if ( (macp->RF_Mode == AL2230_RF) || (macp->RF_Mode == AL2230S_RF) )
                HW_Set_IF_Synthesizer(&dot11Obj, 0x71687);

        LockPhyReg(&dot11Obj);
        zd_writel(0xFF, ZD_CR252);
        zd_writel(0xc, ZD_CR253);



        zd_writel(0x3, ZD_CR255);
        UnLockPhyReg(&dot11Obj);


        zd_writel(0x0, BCNInterval);
        // Turn off LEDs

        zd_writel(0x0, rLED_CTRL);

        // turn of MAC
        zd_writel(0x3, PS_Ctrl);




        dot11Obj.PhyTest = 0;
        macp->LinkTimer = 0;


}


#if 0
int zd1211_DownLoadUSBCode(struct zd1205_private *macp, u8* filename, void *ptr, u16 uCodeOfst)
{
        int ifp;
        long bcount;
        mm_segment_t fs;

        struct stat file_info;

        size_t* file_length;

        u8 *buffer;

        //FPRINT("DownLoadUSBCode");

        // Open the code file

        // for file opening temporarily tell the kernel I am not a user for
        // memory management segment access

        fs = get_fs();
        set_fs(KERNEL_DS);


        // open the file with the firmware for uploading

        if (ifp = open(filename, O_RDONLY, 0 ), ifp < 0)
        {
                // error opening the file
                FPRINT("ERROR: File opening did not success");
                set_fs(fs);
                return -1;
        }




        /* Get information about the file. */
        fstat (ifp, &file_info);
        file_length = file_info.st_size;


        buffer = kmalloc(file_length, GFP_ATOMIC);

        /* Read the file into the buffer. */
        bcount = read(ifp, buffer, file_length);

        if (bcount != file_length)
                FPRINT("read failed");



        // close the file



        close(ifp);

        // switch back the segment setting

        set_fs(fs);



        ret = zd1211_LoadUSBSpecCode(macp, buffer, file_length, uCodeOfst, true);



        kfree(buffer);




        return ret;


}
#endif
void ZD1211_WriteMultiRegister(u16 *Address, u16 *Value, u16 RegCount, BOOLEAN AddUSBCSRAddress)
{
        int count = 0;
        u8 ret = 0xff;

        while (ret != 0) {
                ret = zd1211_USB_PACKAGE_WRITE_REGISTER(Address, Value, RegCount, AddUSBCSRAddress);
                count++;

                if (count > 5) {
                        printk("ZD1211_WriteMultiRegister failed!!\n");
                        break;
                }
        }
}
void ZD1211_WRITE_MULTI_REG(u16  *pAddress, u16  *pValue, u16  *pRegCount)
{
        u16 TotalRegCount = *pRegCount;
        u16* pWriteAddr = pAddress;
        u16* pWriteData = pValue;

        while (TotalRegCount > cMAX_MULTI_WRITE_REG_NUM) {
                ZD1211_WriteMultiRegister(pWriteAddr, pWriteData, cMAX_MULTI_WRITE_REG_NUM, true);

                TotalRegCount -= cMAX_MULTI_WRITE_REG_NUM;
                pWriteAddr += cMAX_MULTI_WRITE_REG_NUM;
                pWriteData += cMAX_MULTI_WRITE_REG_NUM;
        }

        if (TotalRegCount)
                ZD1211_WriteMultiRegister(pWriteAddr, pWriteData, TotalRegCount, true);

        *pRegCount = 0;


}
int zd1211_GetUSBSpecData(struct zd1205_private *macp, u8 *pBuffer,
                          u32 uImgLength, u16 uCodeOfst)

{
        u32 uCurLength;
        int result = 0;
        u8 *image, *ptr;
        u32 uploadLength = uImgLength;


        image = kmalloc(uImgLength, GFP_KERNEL);
        ptr = image;

        while (uImgLength > 0)
        {
                uCurLength = uImgLength;
                if (uCurLength > 60)
                        uCurLength = 60;

                // Get data from device
                result = usb_control_msg(macp->usb, usb_rcvctrlpipe(macp->usb, 0),
                                         FIRMWARE_READ_DATA, USB_DIR_IN | 0x40, uCodeOfst, 0,



                                         image, uCurLength, 1000 * HZ);

                //ZD1211DEBUG(3, "result = %d\n", result);
                if (result < 0) {
                        printk(KERN_ERR "zd1211: usb_rcvctrlpipe 1 fail: %02X\n", result);
                        goto exit;
                }




                /* ath_desc: fix update of uImgLength */
                uImgLength -= result;
                image += result;

                uCodeOfst += (u16) (result / 2); // in Word (16 bit)
                result = 0;

        }

        image -= uploadLength; //move to buffer head

        memcpy(pBuffer, image, uploadLength);


exit:
        //kfree(image);
        kfree(ptr);

        return result ;
}








//return 0: success, others: fail
int zd1211_LoadUSBSpecCode(struct zd1205_private *macp, u8 *pBuffer, u32 uImgLength,
                           u16 uCodeOfst, u8 bReboot)
{
        u8 ret;
        int result = 0;
        u8 *image, *ptr;


        ZD1211DEBUG(1, "uImgLength = %x\n", uImgLength);
        image = kmalloc(uImgLength, GFP_KERNEL);
        ptr = image;
        memcpy(image, pBuffer, uImgLength);


        while (uImgLength > 0)
        {
                int translen = (uImgLength > 4096) ? 4096 : uImgLength;

                ZD1211DEBUG(1, "translen = %x\n", translen);
                ZD1211DEBUG(1, "uCodeOfst = %x\n", uCodeOfst);


                result = usb_control_msg(macp->usb, usb_sndctrlpipe(macp->usb, 0),
                                         FIRMWARE_DOWNLOAD, USB_DIR_OUT | 0x40, uCodeOfst, 0,

                                         image, translen, HZ);


                ZD1211DEBUG(1, "result = %x\n", result);


                if (result < 0) {
                        printk(KERN_ERR "zd1211: usb_control_msg 1 fail: %02X\n", result);
                        goto exit;
                }


                uImgLength -= translen;
                image += translen;


                uCodeOfst += (u16) (translen / 2); // in Word (16 bit)
                result = 0;
        }



        if (bReboot)
        {
                result = usb_control_msg(macp->usb, usb_rcvctrlpipe(macp->usb, 0),
                                         FIRMWARE_CONFIRM, USB_DIR_IN | 0x40, 0, 0,
                                         &ret, sizeof(ret), 1000 * HZ);
                if (result < 0) {


                        printk(KERN_ERR "zd1211: usb_control_msg 2 fail: %02X\n", result);



                        goto exit;


                }

                //ZD1211DEBUG(2, "result = %x\n", result);

                ZD1211DEBUG(1, "FIRMWARE_CONFIRM = %x\n", ret);



                if (ret & 0x80) {
                        FPRINT("USB Download Boot code error");

                        goto exit;
                } else {
                        ZD1211DEBUG(0, "USB Download Boot code success\n");
                }

                result = 0;

        }





exit:
        //kfree(image);
        kfree(ptr);
        return result;

}





int zd1211_Download_IncludeFile(struct zd1205_private *macp)

{
        int ret;
        u16 EEPVer;

        //return 0; //for debug

        EEPVer = WS11Ub[cEPDATA_OFFSET * 2] + (WS11Ub[cEPDATA_OFFSET * 2 + 1] << 8);
        printk(KERN_NOTICE "EEPORM Ver = %x\n", EEPVer);

/* ath_desc: removed superfluous firmware downloading routine */
/* ath_msgsubj: [zd1211-devs] FW download wierdness (zd1211.c) */
/* ath_msgdate: 2005-03-20 20:25 */
/* ath: superfluous routine removed. */

        //for single RX
        ret = zd1211_LoadUSBSpecCode(macp, WS11UPh, sizeof(WS11UPh), cFIRMWARE_START_ADDR, true);

        if (ret != 0)
        {

                FPRINT("Load WS11UPh fail\n");
                return ret;
        }

        return 0;
}

// tasklet (work deferred from completions, in_irq) or timer
void defer_kevent(struct zd1205_private *macp, int flag)
{

        ZENTER(4);

        if (!macp->bUSBDeveiceAttached)
        {
                return;
        }
        //   if (macp->kevent_flags != 0)
        //     printk("macp->kevent_flags=%08x\n",macp->kevent_flags);
        set_bit(flag, &macp->kevent_flags);
        if (flag == KEVENT_SCAN_TIMEOUT)
        {
                if(!schedule_work(&macp->scan_tout_event)) {
                        ZD1211DEBUG(4, "schedule_task failed, flag = %x\n", flag);
                        if(!schedule_work(&macp->scan_tout_event)) {
                                dot11Obj.bChScanning = FALSE;
                        }
                        printk("******* Schedule task fail *********\n");
                }
                ZEXIT(4);
                return;
        }



        if (!schedule_work(&macp->kevent))
        {
                ZD1211DEBUG(4, "schedule_task failed, flag = %x\n", flag);
        }

        ZEXIT(4);

}

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
void process_event(struct zd1205_private *macp)
{
#else
void kevent(void *data)
{
        struct zd1205_private *macp = (struct zd1205_private *) data;
#endif
        if (!macp->bUSBDeveiceAttached) {
                return;
        }

        //non tx rx interrupt
        if (test_bit(KEVENT_NON_TX_RX_INT, &macp->kevent_flags)) {
                zd1211_handle_non_tx_rx(macp);
                clear_bit(KEVENT_NON_TX_RX_INT, &macp->kevent_flags);
        }

        //scan timeout
        if (test_bit(KEVENT_SCAN_TIMEOUT, &macp->kevent_flags)) {
                //FPRINT("scan");
                zd_EventNotify(EVENT_SCAN_TIMEOUT, 0, 0, 0);
                clear_bit(KEVENT_SCAN_TIMEOUT, &macp->kevent_flags);
        }

        //mgt_mon timeout
        if (test_bit(KEVENT_MGT_MON_TIMEOUT, &macp->kevent_flags)) {
                //FPRINT("connect_mon");
                zd1205_connect_mon(macp);
                clear_bit(KEVENT_MGT_MON_TIMEOUT, &macp->kevent_flags);
        }

        //house keeping timeout
        if (test_bit(KEVENT_HOUSE_KEEPING, &macp->kevent_flags)) {
                zd1205_house_keeping(macp);
                clear_bit(KEVENT_HOUSE_KEEPING, &macp->kevent_flags);
        }

        //watchdog timeout
        if (test_bit(KEVENT_WATCH_DOG, &macp->kevent_flags)) {
                zd1205_watchdog(macp);
                clear_bit(KEVENT_WATCH_DOG, &macp->kevent_flags);
        }

        //auth timeout
        if (test_bit(KEVENT_AUTH_TIMEOUT, &macp->kevent_flags)) {
                zd_EventNotify(EVENT_AUTH_TIMEOUT, 0, 0, 0);
                clear_bit(KEVENT_AUTH_TIMEOUT, &macp->kevent_flags);
        }

        //associate timeout
        if (test_bit(KEVENT_ASOC_TIMEOUT, &macp->kevent_flags)) {
                zd_EventNotify(EVENT_ASOC_TIMEOUT, 0, 0, 0);
                clear_bit(KEVENT_ASOC_TIMEOUT, &macp->kevent_flags);
        }

        //challenge timeout
        if (test_bit(KEVENT_TCHAL_TIMEOUT, &macp->kevent_flags)) {
                zd_EventNotify(EVENT_TCHAL_TIMEOUT, 0, 0, 0);
                clear_bit(KEVENT_TCHAL_TIMEOUT, &macp->kevent_flags);
        }

        //zd_ioctl
        if (test_bit(KEVENT_ZD_IOCTL, &macp->kevent_flags)) {
                //FPRINT("ioctl");
                zd1205_zd_dbg_ioctl(macp, &macp->zdreq);
                clear_bit(KEVENT_ZD_IOCTL, &macp->kevent_flags);
        }
/* ath_desc: support for unpatched wpasupplicant */
        //MLME ioctl handling
        if (test_bit(KEVENT_ZD_MLME_IOCTL, &macp->kevent_flags)) {
                zd1211_mlme(macp);
                clear_bit(KEVENT_ZD_MLME_IOCTL, &macp->kevent_flags);
        }

        //use protection
        if (test_bit(KEVENT_EN_PROTECTION, &macp->kevent_flags)) {
                zd_EventNotify(EVENT_ENABLE_PROTECTION, 1, 0, 0);
                clear_bit(KEVENT_EN_PROTECTION, &macp->kevent_flags);
        }

        //disable protection
        if (test_bit(KEVENT_DIS_PROTECTION, &macp->kevent_flags)) {
                zd_EventNotify(EVENT_ENABLE_PROTECTION, 0, 0, 0);
                clear_bit(KEVENT_DIS_PROTECTION, &macp->kevent_flags);
        }

        //update card setting
        if (test_bit(KEVENT_UPDATE_SETTING, &macp->kevent_flags)) {
                zd_UpdateCardSetting(&macp->cardSetting);
                clear_bit(KEVENT_UPDATE_SETTING, &macp->kevent_flags);
        }

        //set multicast
        if (test_bit(KEVENT_SET_MULTICAST, &macp->kevent_flags)) {
                zd1211_set_multicast(macp);
                clear_bit(KEVENT_SET_MULTICAST, &macp->kevent_flags);
        }

        //process signal
        if (test_bit(KEVENT_PROCESS_SIGNAL, &macp->kevent_flags)) {
                zd_SigProcess();
                clear_bit(KEVENT_PROCESS_SIGNAL, &macp->kevent_flags);
        }

        //enable barker preamble
        if (test_bit(KEVENT_EN_BARKER, &macp->kevent_flags)) {
                zd_EventNotify(EVENT_ENABLE_BARKER, 1, 0, 0);
                clear_bit(KEVENT_EN_BARKER, &macp->kevent_flags);
        }

        //disable barker preamble
        if (test_bit(KEVENT_DIS_BARKER, &macp->kevent_flags)) {
                zd_EventNotify(EVENT_ENABLE_BARKER, 0, 0, 0);
                clear_bit(KEVENT_DIS_BARKER, &macp->kevent_flags);
        }

        //enable short slot
        if (test_bit(KEVENT_EN_SHORT_SLOT, &macp->kevent_flags)) {
                zd_EventNotify(EVENT_SHORT_SLOT, 1, 0, 0);
                clear_bit(KEVENT_EN_SHORT_SLOT, &macp->kevent_flags);
        }

        //disable short slot
        if (test_bit(KEVENT_DIS_SHORT_SLOT, &macp->kevent_flags)) {
                zd_EventNotify(EVENT_SHORT_SLOT, 0, 0, 0);
                clear_bit(KEVENT_DIS_SHORT_SLOT, &macp->kevent_flags);
        }

        //disable short slot
        if (test_bit(KEVENT_DIS_CONNECT, &macp->kevent_flags)) {
                if (!zd1205_dis_connect(macp)) {
                        zd_CmdProcess(CMD_DIS_CONNECT, 0, 0);
                        macp->NoBcnDetectedCnt = 0;
                }
                clear_bit(KEVENT_DIS_CONNECT, &macp->kevent_flags);
        }

        //std_ioctl
        if (test_bit(KEVENT_STD_IOCTL, &macp->kevent_flags)) {
                //FPRINT("ioctl");
                zd1205_ioctl(macp->device, &macp->ifreq, macp->ifcmd);
                clear_bit(KEVENT_STD_IOCTL, &macp->kevent_flags);
        }

        // kill tx urb (usb_kill_urb cannot be called from interrupt)
        if (test_bit(KEVENT_KILL_TX_URB, &macp->kevent_flags)) {
                zd1211_kill_tx_urb(macp);
                clear_bit(KEVENT_KILL_TX_URB, &macp->kevent_flags);
        }

        if (test_bit(KEVENT_REGISTER_NET, &macp->kevent_flags)) {
                register_netdev(macp->device);
                clear_bit(KEVENT_REGISTER_NET, &macp->kevent_flags);
        }
}

void zd1211_alloc_rx(unsigned long parm)
{
        struct zd1205_private *macp = (struct zd1205_private *) parm;
        unsigned long flags;

        spin_lock_irqsave(&macp->rx_pool_lock, flags);
        zd1205_alloc_skbs(macp);
        spin_unlock_irqrestore(&macp->rx_pool_lock, flags);
}
