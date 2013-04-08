#ifndef _ZD_DEBUG2_C_
#define _ZD_DEBUG2_C_

#ifdef ZDCONF_MENUDBG
#include "zddebug2.h"
#include "menu_drv_macro.h"

extern zd_80211Obj_t dot11Obj;
//for debug message show
extern U8 mDynKeyMode;
extern U8 mKeyFormat;
struct zd1205_private *lmacp;
extern  struct net_device *g_dev;

#define CNT_MAX	64
static u32 CNT_TBL[CNT_MAX];


static void copy_cnt_tally(void)
{
        struct zd1205_private *macp = (struct zd1205_private *)g_dev->priv;

        memset(CNT_TBL,0,sizeof(CNT_TBL));

        CNT_TBL[T_bcnCnt]=macp->bcnCnt;
        CNT_TBL[T_txCmpCnt]=macp->txCmpCnt;
        CNT_TBL[T_dtimCnt]=macp->dtimCnt;
        CNT_TBL[T_rxCnt]=macp->rxCnt;
        CNT_TBL[T_retryFailCnt]=macp->retryFailCnt;
        CNT_TBL[T_txCnt]=macp->txCnt;

}
static void acquire_ctrl_of_phy_req(void *regp)
{
        u32 tmpValue;

        tmpValue = zd_readl(CtlReg1);
        tmpValue &= ~0x80;
        zd_writel(tmpValue, CtlReg1);
}


static void release_ctrl_of_phy_req(void *regp)
{
        u32 tmpValue;

        tmpValue = zd_readl(CtlReg1);
        tmpValue |= 0x80;
        zd_writel(tmpValue, CtlReg1);
}


int zd1205_zd_dbg2_ioctl(struct zd1205_private *macp, struct zdap_ioctl *zdreq, u32 *ret)
{
        void *regp = macp->regp;
        u16 zd_cmd;
        u32 tmp_value;
        u32 tmp_addr;
        u32 CRn;

        *ret=0;
        zd_cmd = zdreq->cmd;
        switch(zd_cmd)
        {
        case RDCNT:
                copy_cnt_tally();
                if(zdreq->addr < CNT_MAX-1)
                        *ret = CNT_TBL[zdreq->addr];
                break;
        case RDMAC:
                acquire_ctrl_of_phy_req(regp);
                tmp_value = zd_readl(zdreq->addr);
                release_ctrl_of_phy_req(regp);
                zdreq->value = tmp_value;

                *ret =  tmp_value;
                //if (copy_to_user(ifr->ifr_data, &zdreq, sizeof (zdreq)))
                //return -EFAULT;
                break;

        case WRMAC:
                acquire_ctrl_of_phy_req(regp);
                zd_writel(zdreq->value, zdreq->addr);
                release_ctrl_of_phy_req(regp);

                if (zdreq->addr == RX_OFFSET_BYTE)
                        macp->rxOffset = zdreq->value;
                break;

        case RDPhy:
        case WRPhy:
                acquire_ctrl_of_phy_req(regp);
                tmp_addr = zdreq->addr;
                CRn=    ((tmp_addr & 0xF00)>>8)*100+
                        ((tmp_addr & 0xF0)>>4)*10+
                        (tmp_addr & 0xF);
                if (CRn >= 4 && CRn <= 8)//Special handling for CR4 to CR8
                {
                        u8 cnvtbl1[]={0x20, 0x10, 0x14, 0x18, 0x1c};
                        tmp_addr = cnvtbl1[CRn-4];
                } else {
                        tmp_addr = CRn*4;
                }
                if (zd_cmd == RDPhy) {
                        zdreq->value = zd_readl(tmp_addr);
                        *ret =  zdreq->value;
                } else {// ZD_IOCTL_WRITE_PHY
                        zd_writel(zdreq->value, tmp_addr);
                }
                release_ctrl_of_phy_req(regp);
                break;

        default :
                printk(KERN_ERR "zd1205: error command = %x\n", zd_cmd);
                break;
        }

        return 0;
}
#endif
#endif
