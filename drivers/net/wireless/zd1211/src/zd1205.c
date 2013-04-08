/* src/zd1205.c
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
#define __KERNEL_SYSCALLS__

#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))
#include <linux/autoconf.h>
#else
#include <linux/config.h>
#endif
#include <net/checksum.h>
#include <linux/tcp.h>
#include <linux/udp.h>

#include <linux/fs.h>
#include <linux/stat.h>

#include "zd1205.h"
#include "zdinlinef.h"
#include "zddebug.h"
#include "zddebug2.h"
#include "menu_drv_macro.h"
#include "zdhw.h"
#include "zdsorts.h"
#include "zdglobal.h"
#include "zdutils.h"
#include "zdpsmon.h"
#include "zdsm.h"
#ifdef HOST_IF_USB
	#include "zd1211.h"
#endif
#include "zdhci.h"


#if WIRELESS_EXT > 12
    #include <net/iw_handler.h>
#endif
extern U16 mTmRetryConnect;
extern BOOLEAN mProbeWithSsid;
extern u8 mMacMode;
extern U8 mBssType;
extern Element mSsid;
extern Element dot11DesiredSsid;
int errno;
extern u8 mCurrConnUser;
extern U8 mNumBOnlySta;
/* ath_desc: reset auth/asoc states on init */
extern U8 AuthReqState;
extern U8 AsocState;

extern u8 mBssNum;
extern U8 mKeyFormat; //Init value: WEP64_USED(1)
extern BOOLEAN mPrivacyInvoked; // Init value: FALSE
extern U8 mKeyVector[4][16]; // Store WEP key
extern U8 mWepKeyLen;
extern U8 mKeyId;  // Init value: 0
extern U16 mCap;   // Init value: CAP_ESS(1);
extern u16 CurrScanCH;
extern MacAddr_t dot11MacAddress;

extern BOOLEAN zd_CmdProbeReq(U8 bWithSSID);
extern Hash_t *HashSearch(MacAddr_t *pMac);
extern void re_initFdescBuf(void);
/******************************************************************************
*						   C O N S T A N T S
*******************************************************************************
*/


static u8	ZD_SNAP_HEADER[6] = {0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00};
static u8	ZD_SNAP_BRIDGE_TUNNEL[6] = {0xAA, 0xAA, 0x03, 0x00, 0x00, 0xF8};
static u8  zd_Snap_Apple_Type[] = {0xAA,0xAA,0x03,0x08,0x00,0x07,0x80,0x9b};

static u16 IPX=0x8137;
//static u16 NOVELL=0xe0e0; // currently not used
static u16 APPLE_TALK=0x80f3;
static u16 EAPOL=0x888e;


#define MAX_MULTICAST_ADDRS     32
#define NUM_WEPKEYS     4

#define bGroup(pWlanHdr)			(pWlanHdr->Address1[0] & BIT_0)
#define getSeq(pWlanHdr)			(((u16)pWlanHdr->SeqCtrl[1] << 4) + (u16)((pWlanHdr->SeqCtrl[0] & 0xF0) >> 4))
#define getFrag(pWlanHdr)			(pWlanHdr->SeqCtrl[0] & 0x0F)
#define	getTA(pWlanHdr)				(&pWlanHdr->Address2[0])
#define isWDS(pWlanHdr)				(((pWlanHdr->FrameCtrl[1] & TO_DS_FROM_DS) == TO_DS_FROM_DS) ? 1 : 0)
#define bRetryBit(pWlanHdr)			(pWlanHdr->FrameCtrl[1] & RETRY_BIT)
#define bWepBit(pWlanHdr)			(pWlanHdr->FrameCtrl[1] & ENCRY_BIT)
#define bMoreFrag(pWlanHdr)			(pWlanHdr->FrameCtrl[1] & MORE_FRAG)
#define bMoreData(pWlanHdr)			(pWlanHdr->FrameCtrl[1] & MORE_DATA)
#define BaseFrameType(pWlanHdr)		(pWlanHdr->FrameCtrl[0] & 0x0C)
#define SubFrameType(pWlanHdr)		(pWlanHdr->FrameCtrl[0])
#define bDataMgtFrame(pWlanHdr)		(((pWlanHdr->FrameCtrl[0] & 0x04) == 0))
#ifndef HOST_IF_USB
    #define nowT()					(zd_readl(TSF_LowPart))  //us unit
#else
    #define nowT()					(jiffies) //tick (10ms) unit
#endif
/******************************************************************************
*			   F U N C T I O N	 D E C L A R A T I O N S
*******************************************************************************
*/
#ifdef CONFIG_PROC_FS

extern int zd1205_create_proc_subdir(struct zd1205_private *);
extern void zd1205_remove_proc_subdir(struct zd1205_private *);
#else
    #define zd1205_create_proc_subdir(X) 0
    #define zd1205_remove_proc_subdir(X) do {} while(0)
#endif
static u32 channel_11A_to_Freq(const u32 channel);
//static u32 Freq_11A_to_channel(const u32 freq);


static unsigned char zd1205_alloc_space(struct zd1205_private *);
unsigned char zd1205_init(struct zd1205_private *);
static void zd1205_setup_tcb_pool(struct zd1205_private *macp);
static void zd1205_config(struct zd1205_private *macp);
static void zd1205_rd_eaddr(struct zd1205_private *);
int zd1205_open(struct net_device *);
int zd1205_close(struct net_device *);
int zd1205_change_mtu(struct net_device *, int);
int zd1205_set_mac(struct net_device *, void *);
void zd1205_set_multi(struct net_device *);
struct net_device_stats *zd1205_get_stats(struct net_device *);
static int zd1205_alloc_tcb_pool(struct zd1205_private *);
static void zd1205_free_tcb_pool(struct zd1205_private *);
static int zd1205_alloc_rfd_pool(struct zd1205_private *);
static void zd1205_free_rfd_pool(struct zd1205_private *);
static void zd1205_clear_pools(struct zd1205_private *macp);
zd1205_SwTcb_t * zd1205_first_txq(struct zd1205_private *macp, zd1205_SwTcbQ_t *Q);
void zd1205_qlast_txq(struct zd1205_private *macp, zd1205_SwTcbQ_t *Q, zd1205_SwTcb_t *signal);
static void zd1205_init_txq(struct zd1205_private *macp, zd1205_SwTcbQ_t *Q);

#ifndef HOST_IF_USB

static u8 zd1205_pci_setup(struct pci_dev *, struct zd1205_private *);
static void zd1205_intr(int, void *, struct pt_regs *);
static void zd1205_retry_failed(struct zd1205_private *);
static void zd1205_dtim_notify(struct zd1205_private *);
void zd1205_start_ru(struct zd1205_private *);
u8 zd1205_RateAdaption(u16 aid, u8 CurrentRate, u8 gear);
#else

struct rx_list_elem *zd1205_start_ru(struct zd1205_private *);
#endif

u32 zd1205_rx_isr(struct zd1205_private *macp);
void zd1205_tx_isr(struct zd1205_private *);
static void zd1205_transmit_cleanup(struct zd1205_private *, zd1205_SwTcb_t *sw_tcb);
static int zd1205_validate_frame(struct zd1205_private *macp, zd1205_RFD_t *rfd);
int zd1205_xmit_frame(struct sk_buff *, struct net_device *);
static void zd1205_dealloc_space(struct zd1205_private *macp);
void zd1205_disable_int(void);
void zd1205_enable_int(void);
void zd1205_config_wep_keys(struct zd1205_private *macp);
void HKeepingCB(struct net_device *dev);
void zd1205_mgt_mon_cb(struct net_device *dev);
void zd1205_process_wakeup(struct zd1205_private *macp);
void zd1205_device_reset(struct zd1205_private *macp);
int zd1205_DestPowerSave(struct zd1205_private *macp, u8 *pDestAddr);

void zd1205_recycle_rx(struct zd1205_private *macp);

//for 1211
u8 CalculateStrength(struct zd1205_private *macp, zd1205_RFD_t *rfd);
u8 CalculateQuality(struct zd1205_private *macp, zd1205_RFD_t *rfd, u8 *pQualityIndB);
void zd1205_initCAM(struct zd1205_private *macp);
int zd1205_CheckOverlapBss(struct zd1205_private *macp, plcp_wla_Header_t *pWlanHdr, u8 *pMacBody, u32 bodyLen);
void zd1205_HandleQosRequest(struct zd1205_private *macp);
// void zd1205_SetRatesInfo(struct zd1205_private *macp);

u8 X_To_dB(u32 X, u8 rate);
u16 ZDLog10multiply100(int data);
void zd1205_connect_mon(struct zd1205_private *macp);

//wireless extension helper functions
void zd1205_lock(struct zd1205_private *macp);
void zd1205_unlock(struct zd1205_private *macp);
static int zd1205_ioctl_setiwencode(struct net_device *dev, struct iw_point *erq, char* key);
static int zd1205_ioctl_getiwencode(struct net_device *dev, struct iw_point *erq, char* key);
static int zd1205_ioctl_setessid(struct net_device *dev, struct iw_point *erq);
static int zd1205_ioctl_setbssid(struct net_device *dev, struct iwreq *wrq);

static int zd1205_ioctl_getessid(struct net_device *dev, struct iw_point *erq);
static int zd1205_ioctl_setfreq(struct net_device *dev, struct iw_freq *frq);
//static int zd1205_ioctl_setsens(struct net_device *dev, struct iw_param *srq);
static int zd1205_ioctl_setrts(struct net_device *dev, struct iw_param *rrq);
static int zd1205_ioctl_setfrag(struct net_device *dev, struct iw_param *frq);

static int zd1205_ioctl_getfrag(struct net_device *dev, struct iw_param *frq);
static int zd1205_ioctl_setrate(struct net_device *dev, struct iw_param *frq);
static int zd1205_ioctl_getrate(struct net_device *dev, struct iw_param *frq);
//static int zd1205_ioctl_settxpower(struct net_device *dev, struct iw_param *prq);
//static int zd1205_ioctl_gettxpower(struct net_device *dev, struct iw_param *prq);
static int zd1205_ioctl_setpower(struct net_device *dev, struct iw_param *prq);
static int zd1205_ioctl_getpower(struct net_device *dev, struct iw_param *prq);
static int zd1205_ioctl_setmode(struct net_device *dev, __u32 *mode);

/* Wireless Extension Handler functions */
static int zd1205wext_giwfreq(struct net_device *dev, struct iw_request_info *info, struct iw_freq *freq, char *extra);
static int zd1205wext_siwmode(struct net_device *dev, struct iw_request_info *info, __u32 *mode, char *extra);
static int zd1205wext_giwmode(struct net_device *dev, struct iw_request_info *info, __u32 *mode, char *extra);
static int zd1205wext_giwrate(struct net_device *dev, struct iw_request_info *info, struct iw_param *rrq, char *extra);
static int zd1205wext_giwrts(struct net_device *dev, struct iw_request_info *info, struct iw_param *rts, char *extra);
static int zd1205wext_giwfrag(struct net_device *dev, struct iw_request_info *info, struct iw_param *frag, char *extra);
//static int zd1205wext_giwtxpow(struct net_device *dev, struct iw_request_info *info, struct iw_param *rrq, char *extra);
//static int zd1205wext_siwtxpow(struct net_device *dev, struct iw_request_info *info, struct iw_param *rrq, char *extra);
static int zd1205wext_giwrange(struct net_device *dev, struct iw_request_info *info, struct iw_point *data, char *extra);
/* ath_desc: use new get_wireless_stats in 2.6.10+ */
struct iw_statistics *zd1205wext_iw_get_stats(struct net_device *dev);

/* ath_desc: add iwconfig commit support */
#if WIRELESS_EXT > 12
static int zd1205wext_commit(struct net_device *dev, struct iw_request_info *info, void *wrqu,  char *extra);
#endif

#if WIRELESS_EXT > 13
static int zd1205wext_siwscan(struct net_device *dev, struct iw_request_info *info, struct iw_point *data, char *extra);
static int zd1205wext_giwscan(struct net_device *dev, struct iw_request_info *info, struct iw_point *data, char *extra);
#endif

/* functions to support 802.11 protocol stack */
void zdcb_rx_ind(U8 *pData, U32 length, void *buf);
void zdcb_release_buffer(void *buf);
void zdcb_tx_completed(void);
void zdcb_start_timer(U32 timeout, U32 event);
void zdcb_stop_timer(U32 TimerId);
void zd1205_set_zd_cbs(zd_80211Obj_t *pObj);
void zdcb_set_reg(void *reg, U32 offset, U32 value);
void chal_tout_cb(unsigned long ptr);
U32 zdcb_dis_intr(void);
void zdcb_set_intr_mask(U32 flags);
BOOLEAN zdcb_check_tcb_avail(U8	num_of_frag);
BOOLEAN zdcb_setup_next_send(fragInfo_t *frag_info);


void zd_CmdScanReq(u16 channel);
void zd_ScanBegin(void);
void zd_ScanEnd(void);

U16 zdcb_status_notify(U16 status, U8 *StaAddr);
U32 zdcb_vir_to_phy_addr(U32 virtAddr);
U32 zdcb_get_reg(void *reg, U32 offset);
void zdcb_delay_us(U16 ustime);
int zdcb_Rand(U32 seed);

/* For WPA supported functions */
void zd1205_notify_join_event(struct zd1205_private *macp);
void zd1205_notify_disjoin_event(struct zd1205_private *macp);
void zd1205_notify_scan_done(struct zd1205_private *macp);
BOOLEAN zd_CmdProbeReq(U8 ProbeWithSsid);


/******************************************************************************

*						 P U B L I C   D A T A
*******************************************************************************
*/
/* Global Data structures and variables */
#ifndef HOST_IF_USB
char zd1205_copyright[] __devinitdata = "Copyright (c) 2002 Zydas Corporation";
char zd1205_driver_version[]="0.0.1";
const char *zd1205_full_driver_name = "Zydas ZD1205 Network Driver";
char zd1205_short_driver_name[] = "zd1205";
#endif

const char config_filename[] = "/etc/zd1211.conf";
static BOOLEAN CustomMACSet = FALSE;
static u8 CustomMAC[ETH_ALEN];
static BOOLEAN AsocTimerStat = FALSE; //If the Asoc Timer is enabled

zd1205_SwTcbQ_t free_txq_buf, active_txq_buf;
struct net_device *g_dev;
u8 *mTxOFDMType;
zd_80211Obj_t dot11Obj = {0};
/* ath_desc: fix for unaligned writes to skbs on bigendian platform */
/* ath_old: #define RX_COPY_BREAK       0//1518 //we do bridge, don't care IP header alignment */
#define RX_COPY_BREAK       MAX_WLAN_SIZE
#define BEFORE_BEACON       25
/* Definition of Wireless Extension */

/*
 * Structures to export the Wireless Handlers

 */
typedef enum
{
        ZD_REGION_Default   = 0x00,//All channel
        ZD_REGION_USA    = 0x10,//G channel->ch1-11;
        ZD_REGION_Canada   = 0x20,//G channel->ch1-11;
        ZD_REGION_Argentina         = 0x21,//G channel->ch1-11;
        ZD_REGION_Brazil            = 0x22,//G channel->ch1-11;
        ZD_REGION_Europe         = 0x30,//G channel->ETSI ch1-13;
        ZD_REGION_Spain    = 0x31,//G channel->ETSI ch1-13;
        ZD_REGION_France   = 0x32,//G channel->ch10-13;
        ZD_REGION_Ukraine           = 0x33,//G channel->ch1-11;
        ZD_REGION_AustriaBelgium    = 0x34,//Austria and Belgium G channel->ch1-13;;
        ZD_REGION_Switzerland       = 0x35,//G channel->ch1-13;
        ZD_REGION_Japan    = 0x40,//G channel->ch1-14;
        ZD_REGION_Australia         = 0x42,//G channel->ch1-13;
        ZD_REGION_China             = 0x43,//G channel->ch1-11;
        ZD_REGION_HongKong          = 0x44,//G channel->ch1-11;
        ZD_REGION_Korea             = 0x45,//G channel->ch1-11;
        ZD_REGION_NewZealand        = 0x46,//G channel->ch1-11;
        ZD_REGION_Singapore         = 0x47,//G channel->ch10-13;
        ZD_REGION_Taiwan            = 0x48,//G channel->ch1-13;
        ZD_REGION_Israel   = 0x50,//G channel->ch3-9;
        ZD_REGION_Mexico   = 0x51 //G channel->ch10,11;
} ZD_REGION;


struct iw_priv_args zd1205_private_args[] =
        {
                {
                        SIOCIWFIRSTPRIV + 0x0, 0, 0, "list_bss"
                },
                { SIOCIWFIRSTPRIV + 0x1, 0, 0, "card_reset" },
                { SIOCIWFIRSTPRIV + 0x2, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_auth" },  /* 0 - open, 1 - shared key */
                { SIOCIWFIRSTPRIV + 0x3, 0, IW_PRIV_TYPE_CHAR | 12, "get_auth" },
                { SIOCIWFIRSTPRIV + 0x4, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_preamble" },  /* 0 - long, 1 - short */
                { SIOCIWFIRSTPRIV + 0x5, 0, IW_PRIV_TYPE_CHAR | 6, "get_preamble" },
                { SIOCIWFIRSTPRIV + 0x6, 0, 0, "cnt" },
                { SIOCIWFIRSTPRIV + 0x7, 0, 0, "regs" },
                { SIOCIWFIRSTPRIV + 0x8, 0, 0, "probe" },
                /* ath_desc: reenable iwpriv dbg_flag */
                { SIOCIWFIRSTPRIV + 0x10, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "dbg_flag" },
                { SIOCIWFIRSTPRIV + 0xA, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "connect" },
                { SIOCIWFIRSTPRIV + 0xB, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_mac_mode" },
                { SIOCIWFIRSTPRIV + 0xC, 0, IW_PRIV_TYPE_CHAR | 12, "get_mac_mode" },
                { SIOCIWFIRSTPRIV + 0xD, 0, 0, "save_conf" /* has been removed */ },
                { SIOCIWFIRSTPRIV + 0xE, 0, 0, "load_conf" /* has been removed */ },
                { SIOCIWFIRSTPRIV + 0xF, 0, IW_PRIV_TYPE_CHAR | 14, "get_Region" },
                { SIOCIWFIRSTPRIV + 0x9,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_Region" },
        };


#if WIRELESS_EXT > 12
static iw_handler zd1205wext_handler[] = {
                        (iw_handler) zd1205wext_commit,                 /* SIOCSIWCOMMIT */

                        (iw_handler) NULL,                              /* SIOCGIWNAME */
                        (iw_handler) NULL,                              /* SIOCSIWNWID */
                        (iw_handler) NULL,                              /* SIOCGIWNWID */
                        (iw_handler) NULL,                              /* SIOCSIWFREQ */
                        (iw_handler) zd1205wext_giwfreq,                /* SIOCGIWFREQ */


                        (iw_handler) NULL,                              /* SIOCSIWMODE */
                        (iw_handler) zd1205wext_giwmode,                /* SIOCGIWMODE */

                        (iw_handler) NULL,                              /* SIOCSIWSENS */
                        (iw_handler) NULL,                              /* SIOCGIWSENS */
                        (iw_handler) NULL, /* not used */               /* SIOCSIWRANGE */
                        (iw_handler) zd1205wext_giwrange,               /* SIOCGIWRANGE */
                        (iw_handler) NULL, /* not used */               /* SIOCSIWPRIV */
                        (iw_handler) NULL, /* kernel code */            /* SIOCGIWPRIV */
                        (iw_handler) NULL, /* not used */               /* SIOCSIWSTATS */
                        (iw_handler) NULL, /* kernel code */            /* SIOCGIWSTATS */

                        (iw_handler) NULL,                              /* SIOCSIWSPY */
                        (iw_handler) NULL,                              /* SIOCGIWSPY */
                        (iw_handler) NULL,                              /* -- hole -- */
                        (iw_handler) NULL,                              /* -- hole -- */
                        (iw_handler) NULL,                              /* SIOCSIWAP */
                        (iw_handler) NULL,                              /* SIOCGIWAP */
                        (iw_handler) NULL,				                /* -- hole -- */
                        (iw_handler) NULL,                              /* SIOCGIWAPLIST */
#if WIRELESS_EXT > 13
                        (iw_handler) zd1205wext_siwscan,                /* SIOCSIWSCAN */
                        (iw_handler) zd1205wext_giwscan,                /* SIOCGIWSCAN */
#else /* WIRELESS_EXT > 13 */
                        (iw_handler) NULL,      /* null */              /* SIOCSIWSCAN */
                        (iw_handler) NULL,      /* null */              /* SIOCGIWSCAN */
#endif /* WIRELESS_EXT > 13 */
                        (iw_handler) NULL,                              /* SIOCSIWESSID */
                        (iw_handler) NULL,                              /* SIOCGIWESSID */
                        (iw_handler) NULL,                              /* SIOCSIWNICKN */
                        (iw_handler) NULL,                              /* SIOCGIWNICKN */
                        (iw_handler) NULL,                              /* -- hole -- */
                        (iw_handler) NULL,                              /* -- hole -- */
                        (iw_handler) NULL,                              /* SIOCSIWRATE */
                        (iw_handler) zd1205wext_giwrate,                /* SIOCGIWRATE */
                        (iw_handler) NULL,                              /* SIOCSIWRTS */

                        (iw_handler) zd1205wext_giwrts,                 /* SIOCGIWRTS */
                        (iw_handler) NULL,                              /* SIOCSIWFRAG */
                        (iw_handler) zd1205wext_giwfrag,                /* SIOCGIWFRAG */
                        (iw_handler) NULL,                              /* SIOCSIWTXPOW */
                        (iw_handler) NULL,                              /* SIOCGIWTXPOW */
                        (iw_handler) NULL,                              /* SIOCSIWRETRY */
                        (iw_handler) NULL,                              /* SIOCGIWRETRY */
                        (iw_handler) NULL,                              /* SIOCSIWENCODE */
                        (iw_handler) NULL,                              /* SIOCGIWENCODE */
                        (iw_handler) NULL,                              /* SIOCSIWPOWER */
                        (iw_handler) NULL,                              /* SIOCGIWPOWER */
                };

static const iw_handler		zd1205_private_handler[] =
        {
                NULL,				/* SIOCIWFIRSTPRIV */
        };


struct iw_handler_def p80211wext_handler_def =
        {
/* ath_desc: depend on wireless version, not kernel version */
#if WIRELESS_EXT > 16
		.get_wireless_stats = &zd1205wext_iw_get_stats,
#endif
        num_standard:
                sizeof(zd1205wext_handler) / sizeof(iw_handler),
        num_private:
                sizeof(zd1205_private_handler)/sizeof(iw_handler),

        num_private_args:
                sizeof(zd1205_private_args)/sizeof(struct iw_priv_args),
        standard: zd1205wext_handler,
        private: (iw_handler *) zd1205_private_handler,
        private_args: (struct iw_priv_args *) zd1205_private_args
        };
#endif

#if 0//(LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0))
#define do_div(n,base) ({ \
	unsigned long __upper, __low, __high, __mod; \
	asm("":"=a" (__low), "=d" (__high):"A" (n)); \
	__upper = __high; \
	if (__high) { \
		__upper = __high % (base); \
		__high = __high / (base); \
	} \
	asm("divl %2":"=a" (__low), "=d" (__mod):"rm" (base), "0" (__low), "1" (__upper)); \
	asm("":"=A" (n):"a" (__low),"d" (__high)); \
	__mod; \
})
#endif
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,6))
static void wait_ms(unsigned int ms)
{
        if(!in_interrupt()) {
                current->state = TASK_UNINTERRUPTIBLE;
                schedule_timeout(1 + ms * HZ / 1000);
        } else
                mdelay(ms);
}
#endif
const U16 dot11A_Channel[]=
        {
                36,40,44,48,52,56,60,64,100,104,108,112,116,120,124,128,132,136,140,184,188,192,196,8,12,16,34,38,42,46,149,153,157,161,165
        };
const U16 dot11A_Channel_Amount=sizeof(dot11A_Channel)/sizeof(U16);




#if defined(OFDM)
u8 OfdmRateTbl[12] = {
                             0x00,  //1M
                             0x01,  //2M
                             0x02,  //5.5M
                             0x03,  //11M
                             0x1b,  //6M
                             0x1f,  //9M
                             0x1a,  //12M
                             0x1e,  //18M
                             0x19,  //24M
                             0x1d,  //36M
                             0x18,  //48M
                             0x1c   //54M
                     };
//Rate > 6M, the bit Long/Short preamble is 1 for 802.11a
//While it's 0 for 802.11g
u8 OfdmRateTbl_11A[12]= {
                                0x00,  //Useless
                                0x01,  //Useless
                                0x02,  //Useless
                                0x03,  //Useless
                                0x3b,  //6M
                                0x3f,  //9M
                                0x3a,  //12M
                                0x3e,  //18M
                                0x39,  //24M
                                0x3d,  //36M
                                0x38,  //48M
                                0x3c   //54M

                        };
#endif
//The Order is meaningful, Do not Change
static	u8 a_ChannelMap[] = {
                                   184,196,16,
                                   36,36,
                                   40,48,
                                   52,64,
                                   100,112,
                                   128,140,
                                   149,165};

typedef struct _a_InterpolationStruc
{
        u8	a_Channel;
        u8	Left_Most_Channel;
        u8	Right_Most_Channel;
        u8	TotalDivisionNum;
        u8	Position;	// starting from left.
}
a_InterpolationStruc;

static	a_InterpolationStruc a_InterpolationTbl[] = {
                        //a_Channel,  Left_Most_Channel, Right_Most_Channel,TotalDivisionNum,Position
                        {188,            184,                196,                 3,                  1},
                        {192,            184,                196,                 3,                  2},
                        {  8,             16,                 16,                 3,                  3},
                        { 12,             16,                 16,                 3,                  3},
                        { 34,             36,                 36,                 3,                  3},
                        { 38,             36,                 36,                 3,                  3},
                        { 42,             40,                 48,                 4,                  1},
                        { 44,             40,                 48,                 4,                  2},
                        { 46,             40,                 48,                 4,                  3},
                        { 56,             52,                 64,                 3,                  1},
                        { 60,             52,                 64,                 3,                  2},
                        {104,            100,                112,                 3,                  1},
                        {108,            100,                112,                 3,                  2},
                        {116,            112,                128,                 4,                  1},
                        {120,            112,                128,                 4,                  2},
                        {124,            112,                128,                 4,                  3},
                        {132,            128,                140,                 3,                  1},
                        {136,            128,                140,                 3,                  2},
                        {153,            149,                165,                 4,                  1},
                        {157,            149,                165,                 4,                  2},
                        {161,            149,                165,                 4,                  3}

                };
#define	a_CALIBRATED_CH_NUM		sizeof(a_ChannelMap)
#define	a_INTERPOLATION_CH_NUM	(sizeof(a_InterpolationTbl) / sizeof(a_InterpolationStruc))
#define a_MAX_CALIBRATION_CH_NUM 16
#define a_MAX_INTERPOLATION_CH_NUM 32
#define cPWR_INT_VALUE_GUARD 8
u8      a_Calibration_Data[4][a_MAX_CALIBRATION_CH_NUM];
u8      a_Interpolation_Data[4][a_MAX_INTERPOLATION_CH_NUM];
//	[0][]:CH,
//	[1][]:Integration Value
//	[2][]:SetPoint_36M
//	[3][]:SetPoint_48M_54M


//Find the array index of certain Channel
u8 a_get_cal_int_val( u8 index)
{
        u32	tmpvalue;

        if (index < a_CALIBRATED_CH_NUM) {
                tmpvalue=zd_readl(ZD_E2P_11A_INT_VALUE1+((index>>2)<<2));
                return ((u8)(tmpvalue >> (index%4*8)));
        } else {
                printk("Error in a_get_cal_int_val\n");
                return FALSE;
        }
}

u8	a_get_cal_36M_setpoint_val( u8 index)
{
        u32	tmpvalue;

        if (index < 16) {
                tmpvalue=zd_readl(ZD_E2P_A36M_CAL_VALUE+((index>>2)<<2));
                return ((u8)(tmpvalue >> (index%4*8)));
        } else {
                printk("Error in a_get_cal_36M_setpoint_val\n");
                return FALSE;
        }
}

u8 a_get_cal_48M_54M_setpoint_val( u8 index)
{
        u32 tmpvalue;

        if (index < a_CALIBRATED_CH_NUM ) {
                tmpvalue=zd_readl(ZD_E2P_A54M_CAL_VALUE+((index>>2)<<2));
                return ((u8)(tmpvalue >> (index%4*8)));
        } else {
                printk("Error in a_get_cal_54M_setpoint_val\n");
                return FALSE;
        }
}
u8 a_find_index_in_a_Calibration_Data( u8 ch)
{
        u8 i;
        for (i=0;i<a_CALIBRATED_CH_NUM;i++) {
                if (ch == a_Calibration_Data[0][i]) {
                        return i;
                }
        }
        return (u8)0xff;
}
u8 a_find_index_in_a_Interpolation_Data( u8 ch)
{
        u8 i;
        for (i=0;i<a_INTERPOLATION_CH_NUM;i++) {
                if (ch == a_Interpolation_Data[0][i]) {
                        return i;
                }
        }
        return (u8)0xff;
}

BOOLEAN a_get_interpolation_value(
        u8 index,
        u8 *pIntVal,
        u8 *pSetPoint_36M,
        u8 *pSetPoint_48M_54M)
{
        u8 lm_index; //Left Most Index
        u8 rm_index; //Right Most Index
        // Find Left-Most CH in a_ChannelMap
        lm_index = a_find_index_in_a_Calibration_Data( a_InterpolationTbl[index].Left_Most_Channel);
        if (lm_index == 0xff) {
                printk("Get lm_index error in a_get_inter..\n");
                return FALSE;
        }
        // Find Right-Most CH in a_ChannelMap
        rm_index = a_find_index_in_a_Calibration_Data( a_InterpolationTbl[index].Right_Most_Channel);
        if (rm_index == 0xff) {
                printk("Get rm_index error in a_get_inter..\n");
                return FALSE;
        }

        *pIntVal = (u8)(a_Calibration_Data[1][lm_index] +
                        (u32)ABS(a_Calibration_Data[1][rm_index],a_Calibration_Data[1][lm_index])*
                        a_InterpolationTbl[index].Position / a_InterpolationTbl[index].TotalDivisionNum);

        *pSetPoint_36M = (u8)(a_Calibration_Data[2][lm_index] +
                              (u32)ABS(a_Calibration_Data[2][rm_index],a_Calibration_Data[2][lm_index])*
                              a_InterpolationTbl[index].Position / a_InterpolationTbl[index].TotalDivisionNum);
        *pSetPoint_48M_54M = (u8)(a_Calibration_Data[3][lm_index] +
                                  (u32)ABS(a_Calibration_Data[3][rm_index],a_Calibration_Data[3][lm_index])*
                                  a_InterpolationTbl[index].Position / a_InterpolationTbl[index].TotalDivisionNum);

        return TRUE;
}
//Channel A, One Stop Call to get cal & int
u8 a_OSC_get_cal_int( u8 ch, u32 rate, u8 *intValue, u8 *calValue)
{

        u8 idx;
        idx = a_find_index_in_a_Calibration_Data(ch);
        if(0xff == idx) {//Error code, we can't found the channel in calibration data
                idx = a_find_index_in_a_Interpolation_Data(ch);
                if(0xff != idx) {
                        *intValue = a_Interpolation_Data[1][idx];
                        if(rate <= RATE_36M)
                                *calValue = a_Interpolation_Data[2][idx];
                        else  //rate >=48
                                *calValue = a_Interpolation_Data[3][idx];
                } else
                        return 0xff;
        } else {//Channel is in Calibration Data
                *intValue = a_Calibration_Data[1][idx];
                if(rate <= RATE_36M)
                        *calValue = a_Calibration_Data[2][idx];
                else  //rate >=48
                        *calValue = a_Calibration_Data[3][idx];
        }

        return 0;

}


/* ath: gcc4 needs inline function bodies in the declaration */
inline void zd_writel(u32 value, u32 offset)
{
	struct zd1205_private *macp = g_dev->priv;
	void *regp = macp->regp;
	u32 RegWait = 0;

#ifdef HOST_IF_USB
#if fPROG_FLASH

	if (!macp->bAllowAccessRegister)
		return;

#endif

	zd1211_writel(offset, value, true);


	return;
#endif


	atomic_inc(&macp->DoNotSleep);
	if (dot11Obj.bDeviceInSleep) {
		while((readl(regp+ZD_MAC_PS_STATE) & 0x7) != MAC_OPERATION) {
			udelay(1000);
			RegWait++;
			if ((RegWait > REG_MAX_WAIT) || macp->bSurpriseRemoved) {
				dot11Obj.bDeviceInSleep = 0;
				ZD1211DEBUG(0, "zd_writel Sleep to die!!!");
				break;
			}

		}
	}

	writel(value, regp+offset);
	atomic_dec(&macp->DoNotSleep);
}


u32 zd_readl(u32 offset)
{
	struct zd1205_private *macp = g_dev->priv;
	void *regp = macp->regp;
	u32	value;
	u32	RegWait = 0;

#ifdef HOST_IF_USB
#if fPROG_FLASH

	if (!macp->bAllowAccessRegister)

		return 0xffffffff;
#endif

	value = zd1211_readl(offset, true);
	return value;
#endif

	atomic_inc(&macp->DoNotSleep);
	if (dot11Obj.bDeviceInSleep) {
		while((readl(regp+ZD_MAC_PS_STATE) & 0x7) != MAC_OPERATION) {
			udelay(1000);
			RegWait++;
			if ((RegWait > REG_MAX_WAIT) || macp->bSurpriseRemoved) {

				dot11Obj.bDeviceInSleep = 0;
				ZD1211DEBUG(0, "zd_readl Sleep to die!!!");
				break;
			}
		}
	}

	value = readl(regp+offset);
	atomic_dec(&macp->DoNotSleep);

	return value;
}


#ifdef HOST_IF_USB
#define fDISABLE_LED            0
void iLED_ON(struct zd1205_private *macp, u32 LEDn)
{
#if !fDISABLE_LED

        u32   tmpvalue;

        tmpvalue = zd_readl(rLED_CTRL);
        tmpvalue |= LEDn;
        zd_writel(tmpvalue, rLED_CTRL);

#ifdef ROBIN_KAO

        tmpvalue = zd_readl(FW_LINK_STATUS);
        tmpvalue |= 0x1;
        zd_writel(tmpvalue, FW_LINK_STATUS);
#endif

#endif
}


void iLED_OFF(struct zd1205_private *macp, u32 LEDn)
{
#if !fDISABLE_LED

        u32   tmpvalue;


        tmpvalue = zd_readl(rLED_CTRL);
        tmpvalue &= ~LEDn;
        zd_writel(tmpvalue, rLED_CTRL);


#ifdef ROBIN_KAO

        zd_writel(0x0, FW_LINK_STATUS);
#endif

#endif
}


void iLED_SWITCH(struct zd1205_private *macp, u32 LEDn)
{
#if !fDISABLE_LED
        u32   tmpvalue;

        tmpvalue = zd_readl(rLED_CTRL);
        tmpvalue ^= LEDn;
        zd_writel(tmpvalue, rLED_CTRL);
#endif
}

#else
void iLED_ON(struct zd1205_private	*macp, u32 LEDn)
{
        zd_writel(0x1, LEDn);
}





void iLED_OFF(struct zd1205_private *macp, u32 LEDn)
{
        zd_writel(0x0, LEDn);
}


void iLED_SWITCH(struct zd1205_private	*macp, u32 LEDn)
{
        u32   tmpvalue;

        tmpvalue = zd_readl(LEDn);
        tmpvalue ^= 0x1;
        zd_writel(tmpvalue, LEDn);

}
#endif


void zd1205_disable_int(void)
{

        /* Disable interrupts on our PCI board by setting the mask bit */
        zd_writel(0, InterruptCtrl);
}




void zd1205_enable_int(void)
{
        struct zd1205_private *macp = g_dev->priv;

        zd_writel(macp->intrMask, InterruptCtrl);
}


void zd1205_start_download(u32 phyAddr)
{
#ifdef HOST_IF_USB
        return;
#endif

        if (!dot11Obj.bDeviceInSleep)
                zd_writel(phyAddr, ZD_PCI_TxAddr_p1);
}


void zd1205_start_upload(u32 phyAddr)
{
#ifdef HOST_IF_USB

        return;
#endif


        if (!dot11Obj.bDeviceInSleep) {
                zd_writel(phyAddr, ZD_PCI_RxAddr_p1);


                zd_writel(0, ZD_PCI_RxAddr_p2);

        }
}


int zd1205_DestPowerSave(struct zd1205_private *macp, u8 *pDestAddr)

{
        u32	tmpvalue;

        if ((macp->cardSetting.ATIMWindow != 0) && (macp->bAssoc))
        {
                // We must make sure that Device has been in IBSS mode and PwrMgt mode.
                tmpvalue = zd_readl(ZD_BCNInterval);
                if ((tmpvalue & IBSS_MODE) && (tmpvalue & POWER_MNT)) {


                        // ATIM could be sent only when BCNController is active.
                        if ((*(pDestAddr) & BIT_0) || (macp->bIBSS_Wakeup_Dest)) { // We should issue ATIM for multicast frame.
                                macp->bIBSS_Wakeup_Dest = 0;
                                return 1;
                        }
                }
        }


        return 0;
}



static void zd1205_action(unsigned long parm)
{
        zd_SigProcess();  //process management frame queue in mgtQ
}


static void zd1205_ps_action(unsigned long parm)
{
        zd_CleanupAwakeQ();
}


static void zd1205_tx_action(unsigned long parm)
{
        zd_CleanupTxQ();

}

#ifndef HOST_IF_USB
u8 zd1205_RateAdaption(u16 aid, u8 CurrentRate, u8 gear)
{

        u8	NewRate;


        RATEDEBUG("***** zd1205_RateAdaption");
        RATEDEBUG_V("aid", aid);


        RATEDEBUG_V("CurrentRate", CurrentRate);

        if (gear == FALL_RATE) {


                if (CurrentRate >= RATE_2M) {
                        NewRate = CurrentRate - 1;
                        zd_EventNotify(EVENT_UPDATE_TX_RATE, (U32)NewRate, (U32)aid, 0);
                } else {
                        NewRate = CurrentRate;
                }
                return (NewRate);
        }

        return 0;
}
#endif

void zd1205_ClearTupleCache(struct zd1205_private *macp)
{
        int i;
        tuple_Cache_t *pCache = &macp->cache;

        pCache->freeTpi = 0;
        for (i=0; i<TUPLE_CACHE_SIZE; i++)
        {
                pCache->cache[i].full = 0;
        }
}


u8 zd1205_SearchTupleCache(struct zd1205_private *macp, u8 *pAddr, u16 seq, u8 frag)
{
        int k;
        tuple_Cache_t *pCache = &macp->cache;

        for (k=0; k<TUPLE_CACHE_SIZE; k++)
        {
                if ((memcmp((char *)&pCache->cache[k].ta[0], (char *)pAddr, 6) == 0)
                                && (pCache->cache[k].sn == seq) && (pCache->cache[k].fn == frag)
                                && (pCache->cache[k].full))
                        return 1;
        }

        return 0;
}


void zd1205_UpdateTupleCache(struct zd1205_private *macp, u8 *pAddr, u16 seq ,u8 frag)
{
        int k;
        tuple_Cache_t *pCache = &macp->cache;

        for (k=0; k<TUPLE_CACHE_SIZE; k++)
        {
                if (pCache->cache[k].full) {
                        if ((memcmp((char *)&pCache->cache[k].ta[0], (char *)pAddr, 6) == 0)
                                        && (pCache->cache[k].sn == seq) ) {
                                pCache->cache[k].fn = frag;

                                return;
                        }
                }
        }

        pCache->freeTpi &= (TUPLE_CACHE_SIZE-1);
        memcpy(&pCache->cache[pCache->freeTpi].ta[0], (char *)pAddr, 6);
        pCache->cache[pCache->freeTpi].sn = seq;
        pCache->cache[pCache->freeTpi].fn = frag;
        pCache->cache[pCache->freeTpi].full = 1;

        pCache->freeTpi++;

}


void zd1205_ArReset(struct zd1205_private *macp)
{
        u8 i;
        defrag_Array_t *pArray = &macp->defragArray;

        for (i=0; i<MAX_DEFRAG_NUM; i++)
                pArray->mpdu[i].inUse = 0;
}


void zd1205_ArAge(struct zd1205_private *macp, u32 age)
{
        u8 i;
        defrag_Array_t *pArray = &macp->defragArray;

        for (i=0; i<MAX_DEFRAG_NUM; i++)
        {
                if (pArray->mpdu[i].inUse) {
                        if ((age - pArray->mpdu[i].eol) > MAX_RX_TIMEOUT) {
                                DFDEBUG("***** zd1205_ArAged");
                                macp->ArAgedCnt++;




                                dot11Obj.ReleaseBuffer(pArray->mpdu[i].buf);
                                pArray->mpdu[i].inUse = 0;
                        }
                }
        }
}




int	zd1205_ArFree(struct zd1205_private *macp)
{
        u8 i;
        defrag_Array_t *pArray = &macp->defragArray;

        for (i=0; i<MAX_DEFRAG_NUM; i++)
        {
                if (!pArray->mpdu[i].inUse)

                        return i;
        }

        macp->ArFreeFailCnt++;
        return -1;
}


int	zd1205_ArSearch(struct zd1205_private *macp, u8 *pAddr, u16 seq, u8 frag)
{
        u8 i;
        defrag_Array_t *pArray = &macp->defragArray;
        defrag_Mpdu_t *pDeMpdu;


        for (i=0; i<MAX_DEFRAG_NUM; i++)
        {
                pDeMpdu = &pArray->mpdu[i];
                if (pDeMpdu->inUse) {
                        if ((memcmp((char *)&pDeMpdu->ta[0], pAddr, 6) == 0)
                                        && (pDeMpdu->sn == seq)) {
                                if (pDeMpdu->fn == (frag-1)) {
                                        return i;
                                } else {
                                        dot11Obj.ReleaseBuffer(pDeMpdu->buf);
                                        pDeMpdu->inUse = 0;
                                        return -1;
                                }
                        }
                }
        }

        return -1;
}



void zd1205_ArUpdate(struct zd1205_private *macp, u8 *pAddr, u16 seq, u8 frag, int i)
{

        defrag_Array_t *pArray = &macp->defragArray;


        pArray->mpdu[i].inUse = 1;
        memcpy(&pArray->mpdu[i].ta[0], (char*)pAddr, 6);
        pArray->mpdu[i].sn = seq;
        pArray->mpdu[i].fn = frag;
        pArray->mpdu[i].eol = nowT();
}


void zd1205_IncreaseTxPower(struct zd1205_private *macp, u8 TxPwrType)
{
        u8   *pTxGain;

#if fTX_GAIN_OFDM

        if (TxPwrType != cTX_OFDM)
                pTxGain = &(dot11Obj.TxGainSetting);

        else
                pTxGain = &(dot11Obj.TxGainSetting2);
#else

        pTxGain = &(dot11Obj.TxGainSetting);
#endif

        switch(macp->RF_Mode)
        {
        case MAXIM_NEW_RF:
                if (*pTxGain < MAXIM2_MAX_TX_PWR_SET)

                        (*pTxGain)++;
                break;

        case RFMD_RF:
                if (*pTxGain < RFMD_MAX_TX_PWR_SET)
                        (*pTxGain) ++;
                break;

        case AL2230_RF:
        case AL2230S_RF:
        case AL7230B_RF:
                if (*pTxGain < AL2230_MAX_TX_PWR_SET)
                        (*pTxGain) += 2;
                break;

        default:
                break;
        }

        HW_Write_TxGain2(&dot11Obj, TxPwrType);
}


void zd1205_DecreaseTxPower(struct zd1205_private *macp, u8 TxPwrType)
{
        u8   *pTxGain;

#if fTX_GAIN_OFDM

        if (TxPwrType != cTX_OFDM)
                pTxGain = &(dot11Obj.TxGainSetting);
        else
                pTxGain = &(dot11Obj.TxGainSetting2);
#else

        pTxGain = &(dot11Obj.TxGainSetting);

#endif


        switch(macp->RF_Mode)
        {
        case MAXIM_NEW_RF:
                if (*pTxGain > MAXIM2_MIN_TX_PWR_SET)

                        (*pTxGain)--;
                break;

        case RFMD_RF:
                if (*pTxGain > RFMD_MIN_TX_PWR_SET)

                        (*pTxGain) --;

                break;

        case AL2230_RF:
        case AL2230S_RF:
        case AL7230B_RF:
                if (*pTxGain > AL2230_MIN_TX_PWR_SET)
                        (*pTxGain) -= 2;
                break;

        default:
                break;
        }

        HW_Write_TxGain2(&dot11Obj, TxPwrType);

}



int
zd1205_AnyActivity(struct zd1205_private *macp)
{
        unsigned long flags;

        // Any frame wait for transmission.
        spin_lock_irqsave(&macp->q_lock, flags);
        if (macp->activeTxQ->count)
        {
                spin_unlock_irqrestore(&macp->q_lock, flags);



                return 1;
        }
        spin_unlock_irqrestore(&macp->q_lock, flags);


        if ((dot11Obj.QueueFlag & MGT_QUEUE_SET) || (dot11Obj.QueueFlag & TX_QUEUE_SET))
                return 1;

        if (macp->bAnyActivity)
                return 1;

        // No any activity.
        return 0;
}


void zd1205_connect_mon(struct zd1205_private *macp)
{
        static u16 IdleLoop_Under_Seq1 = 0;

        zd_ConnectMon();
        //    if (dot11Obj.bDeviceInSleep)
        // 	printk(KERN_ERR "mon\n");
        if ((macp->cardSetting.BssType == INFRASTRUCTURE_BSS) && (macp->bPSMSupported))
        {
                if ((!dot11Obj.bChScanning) && (macp->PwrState) && (!dot11Obj.bDeviceInSleep) && (macp->bAssoc)) {
                        // Solve Sequence number duplication problem after wakeup.
                        if (!zd1205_AnyActivity(macp)) {
                                if ((macp->SequenceNum != 1) || (IdleLoop_Under_Seq1 > 20)) {
                                        //zd1205_sleep_reset(macp);
                                        IdleLoop_Under_Seq1 = 0;
                                        // Avoid accessing Registers to save computation power.
                                } else {
                                        IdleLoop_Under_Seq1++;
                                }
                                //ZD1211DEBUG(2, "IdleLoop_Under_Seq1= %d\n", IdleLoop_Under_Seq1);
                        }
                }
        }


}


void zd1205_mgt_mon_cb(struct net_device *dev)
{
        struct zd1205_private *macp = dev->priv;


#ifdef HOST_IF_USB

        defer_kevent(macp, KEVENT_MGT_MON_TIMEOUT);
        mod_timer(&(macp->tm_mgt_id), jiffies+ (1*HZ)/50); //20ms


#else

        zd1205_connect_mon(macp);
        mod_timer(&(macp->tm_mgt_id), jiffies+ (1*HZ)/50); //20ms
#endif

}


void zd1205_SwAntennaDiv(struct zd1205_private *macp)
{
#if fANT_DIVERSITY
        static u32 loop = 0;
        loop++;

        // Software Antenna Diversity Mechanism
        if (macp->bEnableSwAntennaDiv)
        {
                switch(AccState) {
                case ACC_1:
                        if ((loop % macp->Ant_MonitorDur1) == 0) {
                                if (macp->Acc_Num_OFDM)
                                        Avg1_SQ_OFDM = macp->Acc_SQ_OFDM / macp->Acc_Num_OFDM;
                                else {
                                        Avg1_SQ_OFDM = 0;

                                        if (macp->Acc_Num)
                                                Avg1_SQ = macp->Acc_SQ / macp->Acc_Num;
                                        else
                                                Avg1_SQ = 0;
                                }

                                // Higher SQ is better
                                if (((Avg1_SQ_OFDM < macp->NiceSQThr_OFDM) && (Avg1_SQ_OFDM > 0))
                                                || ((Avg1_SQ_OFDM == 0)
                                                    && ((Avg1_SQ < macp->NiceSQThr) && (Avg1_SQ > 0)))
                                                || (!macp->bAssoc)) { // disconnected
                                        SwitchAntenna(macp);
                                        AccState = ACC_2;
                                }

                                macp->Acc_SQ = 0;


                                macp->Acc_Num = 0;

                                macp->Acc_SQ_OFDM = 0;
                                macp->Acc_Num_OFDM = 0;
                        }
                        break;

                case ACC_2:
                        if ((loop % macp->Ant_MonitorDur2) == 0)
                        {
                                if (macp->Acc_Num_OFDM)
                                        Avg2_SQ_OFDM = macp->Acc_SQ_OFDM / macp->Acc_Num_OFDM;
                                else {
                                        Avg2_SQ_OFDM = 0;
                                        if (macp->Acc_Num)
                                                Avg2_SQ = macp->Acc_SQ / macp->Acc_Num;
                                        else
                                                Avg2_SQ = 0;
                                }

                                // Higher SQ is better
                                if ((Avg2_SQ_OFDM < Avg1_SQ_OFDM)


                                                || (((Avg2_SQ_OFDM == 0) && (Avg1_SQ_OFDM == 0))
                                                    && (Avg2_SQ < Avg1_SQ))
                                                || (!macp->bAssoc)) { // disconnected
                                        SwitchAntenna(macp);


                                }

                                AccState = ACC_1;
                                macp->Acc_SQ = 0;
                                macp->Acc_Num = 0;
                                macp->Acc_SQ_OFDM = 0;
                                macp->Acc_Num_OFDM = 0;
                        }

                        break;

                default:
                        break;
                }
        }
#endif
}


void zd1205_CollectHwTally(struct zd1205_private *macp)
{
        macp->hwTotalRxFrm += zd_readl(TotalRxFrm);
        macp->hwCRC32Cnt += zd_readl(CRC32Cnt);

        macp->hwCRC16Cnt += zd_readl(CRC16Cnt);
        //macp->hwDecrypErr_UNI += zd_readl(DecrypErr_UNI);
        //macp->hwDecrypErr_Mul += zd_readl(DecrypErr_Mul);
        macp->hwRxFIFOOverrun += zd_readl(RxFIFOOverrun);
        macp->hwTotalTxFrm += zd_readl(TotalTxFrm);


        macp->hwUnderrunCnt += zd_readl(UnderrunCnt);
        macp->hwRetryCnt += zd_readl(RetryCnt);
}


#define	TOLERANCE		2

int zd1205_IbssPsCheck(struct zd1205_private *macp)
{
        u32 ul_BcnItvl, ul_atimwnd;
        u64 TSFTimer;
        u32 tmpvalue;


        // Make sure that we have passed (ATIM-Window+TOLERANCE)
        ul_BcnItvl = zd_readl(ZD_BCNInterval);
        ul_BcnItvl &= 0xffff;

        ul_atimwnd = zd_readl(ZD_ATIMWndPeriod);
        tmpvalue = zd_readl(ZD_TSF_LowPart);
        TSFTimer = tmpvalue;

        tmpvalue = zd_readl(ZD_TSF_HighPart);
        TSFTimer += (((u64)tmpvalue) << 32);
        TSFTimer = TSFTimer >> 10; // in unit of TU

        //printk("TSF(TU) %d \n", TSFTimer);
        //printk("BeaconInterval = %d\n", ul_BcnItvl);
        //printk("TSF mod BeaconInterval = %d\n", (TSFTimer % ul_BcnItvl));

        if ((do_div(TSFTimer, ul_BcnItvl)) > (ul_atimwnd + TOLERANCE))
        {
                // Make sure no traffic before (ATIMWnd+TOLERANCE)
                if ((!macp->bFrmRxed1) && (macp->SuggestionMode == PS_PSM)) {
                        // Any frame wait for transmission.
                        if (!macp->activeTxQ->count) {

                                //zd1205_sleep_reset(macp);
                                return 1;
                        }
                }
        }

        return 0;
}



void zd1205_InfraPsCheck(struct zd1205_private *macp)
{
        u32 tmpvalue;

        // Now, we assure that no any power-save related operation performing.
        // That's because all power-save related operations are either
        // Mutexed by Adapter->Lock or Notified by Adapter->Notification.
        if ((macp->SuggestionMode == PS_PSM) && (macp->PwrState == PS_CAM))
        {
                down(&macp->bcn_sem);
                tmpvalue = zd_readl(ZD_BCNInterval);

                tmpvalue |= POWER_MNT;


                zd_writel(tmpvalue, ZD_BCNInterval);
                up(&macp->bcn_sem);

                macp->PwrState = PS_PSM;
                zd_EventNotify(EVENT_PS_CHANGE, (U8)macp->PwrState, 0, 0);
                ZD1211DEBUG(0, "=====CAM --> PSM\n");
        } else if ((macp->SuggestionMode == PS_CAM) && (macp->PwrState == PS_PSM) &&
                        (!dot11Obj.bDeviceInSleep))
        {
                down(&macp->bcn_sem);

                tmpvalue = zd_readl(ZD_BCNInterval);
                tmpvalue &= ~POWER_MNT;
                zd_writel(tmpvalue, ZD_BCNInterval);
                up(&macp->bcn_sem);

                macp->PwrState = PS_CAM;
                zd_EventNotify(EVENT_PS_CHANGE, (U8)macp->PwrState, 0, 0);
                ZD1211DEBUG(0, "=====PSM --> CAM\n");
        }

        return;
}

//Normally, house keeping routine is run every 100ms.
void zd1205_house_keeping(struct zd1205_private *macp)
{

        static u32 loop = 0;
        card_Setting_t *pSetting = &macp->cardSetting;
        u8 BssType = pSetting->BssType;
        u8 bAssoc = macp->bAssoc;

#if 0

        if (dot11Obj.QueueFlag & TX_QUEUE_SET)
        {
                macp->txQueSetCnt++;
                //tasklet_schedule(&macp->zd1205_tx_tasklet);
                zd_CleanupTxQ();
        }
#endif
        loop++;

#ifndef HOST_IF_USB

        while (dot11Obj.bDeviceInSleep)
        {
                // If device is in sleep, do not access device register often to
                // prevent host from slowing down.
                wait_ms(10);
        }
#else
        if (dot11Obj.bDeviceInSleep)
                return;
#endif

        // Software Antenna Diversity Mechanism
        if (macp->bEnableSwAntennaDiv)
        {
                zd1205_SwAntennaDiv(macp);
        }

        // IBSS power-save monitor
        if ((BssType == INDEPENDENT_BSS) && (bAssoc))
        {
                if ((!dot11Obj.bChScanning) && macp->bPSMSupported) {
                        if (zd1205_IbssPsCheck(macp))
                                return;
                }
        }

#if 1
        // Infrasture AP mode beacon generation
        if (BssType == AP_BSS)
        {
                down(&macp->bcn_sem);
                zd_EventNotify(EVENT_TBCN, 0, 0, 0);
                up(&macp->bcn_sem);

                if (macp->dtimCount == 0)
                        macp->dtimCount = macp->cardSetting.DtimPeriod;
                macp->dtimCount--;
        }
#endif


        //++ Recovery mechanism for ZD1202 ASIC Phy-Bus arbitration fault.
        //   We combined tx-power-tracking/Sw Antenna diversity code here to
        //   reduce the frequence of
        //   calling ReleaseCtrOfPhyReg. It's harmful to throughput.

        if ((loop % 1) == 0)
        { //every 100 ms
                //Collect HW Tally
                //zd1205_CollectHwTally(macp);  //This will make us lose CfgNextBcn interrupt

#ifdef HOST_IF_USB

                //tmpvalue = zd_readl(0x6e4);
                //macp->REG_6e4_Add += tmpvalue;
                //printk(KERN_ERR "Detect Strong Signal:%lu\n",jiffies);
                zd1211_StrongSignalDect(macp);
#endif

                // Infrastructure Power-State momitor

                if ((!dot11Obj.bChScanning) && (BssType == INFRASTRUCTURE_BSS) && (bAssoc) &&  (macp->bPSMSupported)) {
                        zd1205_InfraPsCheck(macp);
                }
        }

#ifdef HOST_IF_USB
	#ifndef ZD1211B
        zd1211_TxCalibration(macp);
#endif

        zd1211_CheckWithIPC(macp);
#endif

}

void HKeepingCB(struct net_device *dev)
{
        struct zd1205_private *macp = dev->priv;

#ifdef HOST_IF_USB

        defer_kevent(macp, KEVENT_HOUSE_KEEPING);
        mod_timer(&(macp->tm_hking_id), jiffies+ (1*HZ)/10);
#else

        zd1205_house_keeping(macp);
        mod_timer(&(macp->tm_hking_id), jiffies+ (1*HZ)/10);
#endif

}

void zd1205_CollectBssInfo(struct zd1205_private *macp, plcp_wla_Header_t *pWlanHdr, u8 *pMacBody, u32 bodyLen)
{
        u8 bssidmatched = 0;
        u8 i, j;
        u8 *pBssid;
        u8 *pByte;
        u32 currPos = 0;
        u8 elemId, elemLen;

        if ((*(pMacBody+CAP_OFFSET)) & BIT_1) //IBSS
                pBssid = pWlanHdr->Address3;
        else
                pBssid = pWlanHdr->Address2;

        for (i=0; i<macp->bss_index; i++)
        {
                for (j=0; j<6; j++) {
                        if (macp->BSSInfo[i].bssid[j] != pBssid[j]) {
                                break;
                        }
                }

                if (j==6) {
                        bssidmatched = 1;
                        break;
                }
        }

        if (bssidmatched)
                return;

        //get bssid
        for (i=0; i<6; i++)
        {
                macp->BSSInfo[macp->bss_index].bssid[i] = pBssid[i];
        }


        //get beacon interval
        pByte = pMacBody+BCN_INTERVAL_OFFSET;
        macp->BSSInfo[macp->bss_index].beaconInterval = ((*pByte) + ((u16)(*(pByte+1))<<8));

        //get capability
        pByte = pMacBody+CAP_OFFSET;
        macp->BSSInfo[macp->bss_index].cap = ((*pByte) + ((u16)(*(pByte+1))<<8) );

        //get element
        pByte = pMacBody+SSID_OFFSET;
        currPos = SSID_OFFSET;

        while(currPos < bodyLen)
        {
                elemId = *pByte;
                elemLen = *(pByte+1);

                switch(elemId) {
                case ELEID_SSID: //ssid
                        for (i=0; i<elemLen+2; i++) {
                                macp->BSSInfo[macp->bss_index].ssid[i] = *pByte;
                                pByte++;
                        }
                        break;

                case ELEID_SUPRATES: //supported rateS

                        for (i=0; i<elemLen+2; i++) {
                                macp->BSSInfo[macp->bss_index].supRates[i] = *pByte;
                                pByte++;
                        }
                        break;


                case ELEID_DSPARMS: //ds parameter
                        macp->BSSInfo[macp->bss_index].channel = *(pByte+2);
                        pByte += (elemLen+2);
                        break;

                case ELEID_EXT_RATES:

                        pByte += (elemLen+2);
                        break;

                default:

                        pByte += (elemLen+2);
                        break;
                }

                currPos += elemLen+2;
        }

        macp->BSSInfo[macp->bss_index].signalStrength = macp->rxSignalStrength;
        macp->BSSInfo[macp->bss_index].signalQuality = macp->rxSignalQuality;

        if (macp->bss_index < (BSS_INFO_NUM-1))
        {
                macp->bss_index ++;
        }

        return;
}

void zd1205_dump_rfds(struct zd1205_private *macp)
{
        struct rx_list_elem *rx_struct = NULL;
        struct list_head *entry_ptr = NULL;
        zd1205_RFD_t *rfd = 0;
        struct sk_buff *skb;

        int i = 0;

        list_for_each(entry_ptr, &(macp->active_rx_list))
        {

                rx_struct = list_entry(entry_ptr, struct rx_list_elem, list_elem);
                if (!rx_struct)
                        return;
#ifndef HOST_IF_USB

                pci_dma_sync_single(macp->pdev, rx_struct->dma_addr,
                                    macp->rfd_size, PCI_DMA_FROMDEVICE);
#endif

                skb = rx_struct->skb;
                rfd = RFD_POINTER(skb, macp);	/* locate RFD within skb */
#if 0

                printk(KERN_DEBUG "zd1205: i = %x\n", i);
                printk(KERN_DEBUG "zd1205: rx_struct = %x\n", (u32)rx_struct);

                printk(KERN_DEBUG "zd1205: rx_struct->dma_addr = %x\n", (u32)rx_struct->dma_addr);
                printk(KERN_DEBUG "zd1205: rx_struct->skb = %x\n", (u32)rx_struct->skb);
                printk(KERN_DEBUG "zd1205: rfd = %x\n", (u32)rfd);
                printk(KERN_DEBUG "zd1205: CbStatus = %x\n", le32_to_cpu(rfd->CbStatus));

                printk(KERN_DEBUG "zd1205: CbCommand = %x\n", le32_to_cpu(rfd->CbCommand));
                printk(KERN_DEBUG "zd1205: NextCbPhyAddrLowPart = %x\n", le32_to_cpu(rfd->NextCbPhyAddrLowPart));
                printk(KERN_DEBUG "zd1205: NextCbPhyAddrHighPart = %x\n", le32_to_cpu(rfd->NextCbPhyAddrHighPart));
#endif

                zd1205_dump_data("rfd", (u8 *)rfd, 24);
                i++;

        }

}

void zd1205_dump_data(char *info, u8 *data, u32 data_len)
{
        int i;
        printk(KERN_DEBUG "%s data [%d]: \n", info, data_len);

        /* ath_desc: fix printks */
        // printk parses KERN_* macros only after newlines
        printk(KERN_DEBUG "");
        for (i=0; i<data_len; i++) {
                printk("%02x ", data[i]);
                if ((i>0) && ((i+1)%16 == 0))
                        printk("\n" KERN_DEBUG);
        }

        printk("\n");
}



/**
 * zd1205_get_rx_struct - retrieve cell to hold skb buff from the pool
 * @macp: atapter's private data struct

 *
 * Returns the new cell to hold sk_buff or %NULL.
 */

static struct rx_list_elem *
                        zd1205_get_rx_struct(struct zd1205_private *macp)
{
        struct rx_list_elem *rx_struct = NULL;

        if (!list_empty(&(macp->rx_struct_pool)))
        {

                rx_struct = list_entry(macp->rx_struct_pool.next,
                                       struct rx_list_elem, list_elem);
                list_del(&(rx_struct->list_elem));
        }

        return rx_struct;
}


/**
 * zd1205_alloc_skb - allocate an skb for the adapter
 * @macp: atapter's private data struct
 *
 * Allocates skb with enough room for rfd, and data, and reserve non-data space.
 * Returns the new cell with sk_buff or %NULL.
 */

static struct rx_list_elem *
                        zd1205_alloc_skb(struct zd1205_private *macp)
{
        struct sk_buff *new_skb;

        u32 skb_size = sizeof (zd1205_RFD_t);
        struct rx_list_elem *rx_struct;

        ZENTER(4);

        new_skb = (struct sk_buff *) dev_alloc_skb(skb_size);
        if (new_skb)
        {
                /* The IP data should be
                   DWORD aligned. since the ethernet header is 14 bytes long,
                   we need to reserve 2 extra bytes so that the TCP/IP headers

                   will be DWORD aligned. */
                //skb_reserve(new_skb, 2); //for zd1202, rx dma must be 4-bytes aligmnebt
                if ((rx_struct = zd1205_get_rx_struct(macp)) == NULL)
                        goto err;

                /* ath_desc: AMD64 support */
                ZD1211DEBUG(4, "zd1211: rx_struct = %p\n", (void *)rx_struct);


                rx_struct->skb = new_skb;

                //Rx DMA address  must be 4 bytes alignment
#ifndef HOST_IF_USB

                rx_struct->dma_addr = pci_map_single(macp->pdev, new_skb->data, sizeof (zd1205_RFD_t), PCI_DMA_FROMDEVICE);

#endif

                ZD1211DEBUG(4, "zd1211: rx_struct->dma_addr = %p\n", (void *)rx_struct->dma_addr);

#ifndef HOST_IF_USB

                if (!rx_struct->dma_addr)
                        goto err;
#endif


                skb_reserve(new_skb, macp->rfd_size); //now skb->data point to RxBuffer

#ifdef HOST_IF_USB

                rx_struct->dma_addr = (dma_addr_t)new_skb->data;
                rx_struct->UnFinishFrmLen = 0;
#endif

                ZEXIT(4);
                return rx_struct;
        } else
        {
                macp->AllocSkbFailCnt++;
                printk(KERN_DEBUG "zd1205: dev_alloc_skb fail\n");
                return NULL;
        }

err:
        printk(KERN_DEBUG "zd1205: ****** err\n");
        dev_kfree_skb_irq(new_skb);

        return NULL;
}


/**
 * zd1205_add_skb_to_end - add an skb to the end of our rfd list

 * @macp: atapter's private data struct
 * @rx_struct: rx_list_elem with the new skb
 *
 * Adds a newly allocated skb to the end of our rfd list.
 */



void
zd1205_add_skb_to_end(struct zd1205_private *macp, struct rx_list_elem *rx_struct)
{
        zd1205_RFD_t *rfdn;	/* The new rfd */

        zd1205_RFD_t *rfd;		/* The old rfd */
        struct rx_list_elem *rx_struct_last;

        ZENTER(4);


        (rx_struct->skb)->dev = macp->device;
        rfdn = RFD_POINTER(rx_struct->skb, macp);

        rfdn->CbCommand = __constant_cpu_to_le32(RFD_EL_BIT);
        wmb();
        rfdn->CbStatus = 0xffffffff;

        rfdn->ActualCount = 0;
        rfdn->MaxSize = __constant_cpu_to_le32(MAX_WLAN_SIZE);

#ifndef __LP64__
        rfdn->NextCbPhyAddrHighPart = 0;

        rfdn->NextCbPhyAddrLowPart = 0;

#else
        rfdn->NextCbPhyAddr = 0;
#endif

#ifndef HOST_IF_USB

        wmb();

        pci_dma_sync_single(macp->pdev, rx_struct->dma_addr, macp->rfd_size,
                            PCI_DMA_TODEVICE);
#endif

        if (!list_empty(&(macp->active_rx_list)))
        {
                rx_struct_last = list_entry(macp->active_rx_list.prev,

                                            struct rx_list_elem, list_elem);


                rfd = RFD_POINTER(rx_struct_last->skb, macp);
                ZD1211DEBUG(4, "zd1211: rfd = %p\n", (unsigned long *)rfd);

#ifndef HOST_IF_USB

                pci_dma_sync_single(macp->pdev, rx_struct_last->dma_addr,

                                    4, PCI_DMA_FROMDEVICE);
#endif

#ifndef __LP64__
                put_unaligned(cpu_to_le32(rx_struct->dma_addr),
                              ((u32 *) (&(rfd->NextCbPhyAddrLowPart))));
#else
                put_unaligned(cpu_to_le64(rx_struct->dma_addr),&(rfd->NextCbPhyAddr));
#endif
#ifndef HOST_IF_USB

                wmb();
                pci_dma_sync_single(macp->pdev, rx_struct_last->dma_addr,
                                    8, PCI_DMA_TODEVICE);

#endif

                rfd->CbCommand = 0;


#ifndef HOST_IF_USB

                wmb();
                pci_dma_sync_single(macp->pdev, rx_struct_last->dma_addr,
                                    4, PCI_DMA_TODEVICE);

#endif

        }

        list_add_tail(&(rx_struct->list_elem), &(macp->active_rx_list)); //add elem to active_rx_list
        ZEXIT(4);
}


void zd1205_alloc_skbs(struct zd1205_private *macp)
{
        for (; macp->skb_req > 0; macp->skb_req--)
        {
                struct rx_list_elem *rx_struct;

                if ((rx_struct = zd1205_alloc_skb(macp)) == NULL) {
                        printk(KERN_DEBUG "zd1205: zd1205_alloc_skb fail\n");
                        return;

                }
                zd1205_add_skb_to_end(macp, rx_struct);
        }

}


void zd1205_transmit_cleanup(struct zd1205_private *macp, zd1205_SwTcb_t *sw_tcb)
{
        zd1205_HwTCB_t *hw_tcb;
        u32 tbd_cnt;
        zd1205_TBD_t *tbd_arr = sw_tcb->pFirstTbd;

        ZENTER(2);

        hw_tcb = sw_tcb->pTcb;
        tbd_cnt = le32_to_cpu(hw_tcb->TxCbTbdNumber);
        tbd_arr += 2; //CtrlSetting and MacHeader

        ZD1211DEBUG(2, "zd1211: umap tbd cnt = %x\n", tbd_cnt-2);

#ifndef HOST_IF_USB

        for (i=0; i<tbd_cnt-2; i++, tbd_arr++)
        {
                ZD1211DEBUG(2, "zd1211: umap body_dma = %x\n", le32_to_cpu(tbd_arr->TbdBufferAddrLowPart));
                pci_unmap_single(macp->pdev,
                                 le32_to_cpu(tbd_arr->TbdBufferAddrLowPart),
                                 le32_to_cpu(tbd_arr->TbdCount),
                                 PCI_DMA_TODEVICE);
        }
#endif

        ZD1211DEBUG(2, "zd1211: Free TcbPhys = %x\n", (u32)sw_tcb->TcbPhys);
        zd1205_qlast_txq(macp, macp->freeTxQ, sw_tcb);
        ZD1211DEBUG(2, "zd1211: Cnt of freeTxQ = %x\n", macp->freeTxQ->count);

        //sw_tcb->HangDur = 0;
        hw_tcb->CbStatus = 0xffffffff;
        hw_tcb->TxCbTbdNumber = cpu_to_le32(0xaaaaaaaa);	/* for debug */
        hw_tcb->CbCommand = cpu_to_le32(CB_S_BIT);

        if ((netif_running(macp->device)) && (macp->bAssoc))
        {
                netif_carrier_on(macp->device);
                netif_wake_queue(macp->device);   //resume tx
        }

        ZEXIT(2);
        return;
}

void zd1205_tx_isr(struct zd1205_private *macp)
{
        zd1205_SwTcb_t *sw_tcb = NULL;

#ifndef HOST_IF_USB

        zd1250_SwTcb_t *next_sw_tcb;
#endif

        u16 aid;

#ifdef HOST_IF_USB

        int bRunOnce = false;
#endif

        ZD1211DEBUG(2, "***** zd1205_tx_isr enter *****\n");

        if (!macp->activeTxQ->count)
        {
                printk(KERN_DEBUG "No element in activeQ\n");
		clear_bit(ZD1211_TX_BUSY, &macp->flags);
                return;
        }

        /* Look at the TCB at the head of the queue.  If it has been completed
           then pop it off and place it at the tail of the completed list.
           Repeat this process until all the completed TCBs have been moved to the
           completed list */
        while (macp->activeTxQ->count)
        {
                sw_tcb = macp->activeTxQ->first;

#ifdef HOST_IF_USB
                // in USB modem, only run once
                if (bRunOnce)
                        break;
                bRunOnce = true;
#endif
                // check to see if the TCB has been DMA'd

                // Workaround for hardware problem that seems leap over a TCB
                // and then fill completion token in the next TCB.
                ZD1211DEBUG(2, "zd1211: hw_tcb = %p\n", (unsigned long *)sw_tcb->pTcb);
                ZD1211DEBUG(2, "zd1211: CbStatus = %x\n", (u16)le32_to_cpu(sw_tcb->pTcb->CbStatus));

#ifndef HOST_IF_USB

                rmb();

                if ((u16)le32_to_cpu(sw_tcb->pTcb->CbStatus) != CB_STATUS_COMPLETE) {
                        next_sw_tcb = sw_tcb;

                        while(1) {
                                next_sw_tcb = next_sw_tcb->next;
                                if (!next_sw_tcb)
                                        break;

                                if ((u16)le32_to_cpu(next_sw_tcb->pTcb->CbStatus) == CB_STATUS_COMPLETE)
                                        break;
                        }

                        if (!next_sw_tcb)
                                break;
                }
#endif

                /* Remove the TCB from the active queue. */
                sw_tcb = zd1205_first_txq(macp, macp->activeTxQ);
                ZD1211DEBUG(2, "zd1211: Cnt of activeQ = %x\n", macp->activeTxQ->count);

                aid = sw_tcb->aid;
                zd1205_transmit_cleanup(macp, sw_tcb);
                macp->txCmpCnt++;

                if (!sw_tcb->LastFrag)
                        continue;

                zd_EventNotify(EVENT_TX_COMPLETE, ZD_TX_CONFIRM, (U32)sw_tcb->MsgID, (U32)aid);
                macp->SequenceNum++;
                macp->bDataTrafficLight = 1;
        }

#ifdef HOST_IF_USB
        if(sw_tcb->CalMIC[MIC_LNG] == TRUE)
                zd1211_submit_tx_urb(macp,TRUE);
        else
                zd1211_submit_tx_urb(macp,FALSE);
#endif

        ZD1211DEBUG(2, "***** zd1205_tx_isr exit *****\n");
        return;
}


#ifndef HOST_IF_USB
static void zd1205_retry_failed(struct zd1205_private *macp)
{
        zd1205_SwTcb_t *sw_tcb;
        zd1205_SwTcb_t *next_sw_tcb = NULL;

        zd1205_HwTCB_t *hw_tcb;
        zd1205_Ctrl_Set_t *ctrl_set;

        u8 CurrentRate, NewRate;
        u8 ShortPreambleFg;
        u16 Len;
        u16 NextLen;
        u16 LenInUs;
        u16 NextLenInUs;
        u8 Service;
        u16 aid;

        ZD1211DEBUG(2, "+++++ zd1205_retry_failed enter +++++\n");

        if (!macp->activeTxQ->count)
        {
                ZD1211DEBUG(1, "**********empty activeTxQ, got retry failed");
                sw_tcb = macp->freeTxQ->first;
                zd1205_start_download(sw_tcb->TcbPhys | BIT_0);
                return;
        }


        // Feature: Rate Adaption
        // - During the procedure of processing a transmitting frame, we must keep
        //   the TaRate consistent.
        // - When to fall OppositeContext.CurrentTxRate:
        //   Whenever RetryFail occurs, change OppositeContext.CurrentTxRate by a value
        //   ((Rate of this TCB) minus a degree) and modify this TCB's control-setting
        //   with the OppositeContext.CurrentTxRate and then Restart this TCB.
        //	 (Set RetryMAX = 2).
        //   Once the TxRate is 1M and still RetryFail, abandon this frame.
        // - When to rise TxRate:
        //   If there are 10 frames transmitted successfully
        //   (OppositeContext.ConsecutiveSuccessFrames >= 10), change
        //   OppositeContext.CurrentTxRate by a value
        //   ((Rate of this TCB) plus a degree).


        // - Adjust OppositeContext.CurrentTxRate manually. (by application tool)
        sw_tcb = macp->activeTxQ->first;
        aid = sw_tcb->aid;
        ctrl_set = sw_tcb->pHwCtrlPtr;

        if (ctrl_set->CtrlSetting[11] & BIT_3)
        { //management frame
                goto no_rate_adaption;
        }

        //CurrentRate = (ctrl_set->CtrlSetting[0] & 0x1f);

        CurrentRate = sw_tcb->Rate;

        ShortPreambleFg = (ctrl_set->CtrlSetting[0] & 0x20);

        if (((!ShortPreambleFg) && (CurrentRate > RATE_1M)) ||
                        ((ShortPreambleFg) && (CurrentRate > RATE_2M)))
        {
                // Fall TxRate a degree

                NewRate = zd1205_RateAdaption(aid, CurrentRate, FALL_RATE);
                sw_tcb->Rate = NewRate;

                // Modify Control-setting
                ctrl_set->CtrlSetting[0] = (ShortPreambleFg | NewRate);
                ctrl_set->CtrlSetting[11] |= BIT_0; // Set need backoff

                // LenInUs, Service
                Len = (ctrl_set->CtrlSetting[1] + ((u16)ctrl_set->CtrlSetting[2] << 8));
                Cal_Us_Service(NewRate, Len, &LenInUs, &Service);
                ctrl_set->CtrlSetting[20] = (u8)LenInUs;
                ctrl_set->CtrlSetting[21] = (u8)(LenInUs >> 8);
                ctrl_set->CtrlSetting[22] = Service;


                // NextLenInUs
#if defined(OFDM)

                NextLen = (ctrl_set->CtrlSetting[25+1] + ((u16)ctrl_set->CtrlSetting[25+2] << 8));
#else

                NextLen = (ctrl_set->CtrlSetting[18] + ((u16)ctrl_set->CtrlSetting[19] << 8));
#endif





                Cal_Us_Service(NewRate, NextLen, &NextLenInUs, &Service);
                ctrl_set->CtrlSetting[23] = (u8)NextLenInUs;
                ctrl_set->CtrlSetting[24] = (u8)(NextLenInUs >> 8);

#if defined(OFDM)

                if (NewRate > RATE_11M) {
                        NewRate = OfdmRateTbl[NewRate];
                }

                macp->retryFailCnt++;

                ctrl_set->CtrlSetting[0] = (ShortPreambleFg | NewRate);
                ctrl_set->CtrlSetting[11] |= BIT_0; // Set need backoff

#endif
                // Re-Start Tx-Bus master with a lower Rate

                zd1205_start_download(sw_tcb->TcbPhys | BIT_0);
                return;


        }

        /* Look at the TCB at the head of the queue.  If it has been completed

            then pop it off and place it at the tail of the completed list.
            Repeat this process until all the completed TCBs have been moved to the
            completed list */

no_rate_adaption:
        while (macp->activeTxQ->count)
        {
                //ZD1211DEBUG(1, "zd1211: sw_tcb = %x\n", (u32)sw_tcb);
                ZD1211DEBUG(2, "zd1211: hw_tcb = %x\n", (u32)sw_tcb->pTcb);


                /* Remove the TCB from the active queue. */
                sw_tcb = zd1205_first_txq(macp, macp->activeTxQ);

                ZD1211DEBUG(2, "zd1211: Cnt of activeQ = %x\n", macp->activeTxQ->count);

                zd1205_transmit_cleanup(macp, sw_tcb);
                macp->retryFailCnt++;
                if (!sw_tcb->LastFrag)
                        continue;


                zd_EventNotify(EVENT_TX_COMPLETE, ZD_RETRY_FAILED, (U32)sw_tcb->MsgID, aid);

                if (!macp->activeTxQ->count) {
                        // Re-Start Tx-Bus master with an suspend TCB
                        hw_tcb = (zd1205_HwTCB_t *)sw_tcb->pTcb;
                        // Set BIT_0 to escape from Retry-Fail-Wait State.
                        zd1205_start_download((cpu_to_le32(hw_tcb->NextCbPhyAddrLowPart) | BIT_0));
                } else {
                        next_sw_tcb = macp->activeTxQ->first;
                        // Re-Start Tx bus master
                        // Set BIT_0 to escape from Retry-Fail-Wait state.

                        zd1205_start_download(next_sw_tcb->TcbPhys | BIT_0);
                }
                break;
        }

        macp->bIBSS_Wakeup_Dest = 1;



        ZD1211DEBUG(2, "+++++ zd1205_retry_failed exit +++++\n");

        return;
}
#endif


static void zd1205_config(struct zd1205_private *macp)
{
        u32 tmpValue;
        int i, jj;

        ZENTER(1);


        // Retrieve Feature BitMap
        zd_writel(macp->cardSetting.EncryMode, EncryptionType);
        macp->dtimCount = 0;


        /* Setup Physical Address */
        zd_writel(cpu_to_le32(*(u32 *)&macp->macAdr[0]), MACAddr_P1);
        zd_writel(cpu_to_le32(*(u32 *)&macp->macAdr[4]), MACAddr_P2);
        if (macp->cardSetting.BssType == AP_BSS)
        {
                /* Set bssid = MacAddress */
                macp->BSSID[0] = macp->macAdr[0];
                macp->BSSID[1] = macp->macAdr[1];
                macp->BSSID[2] = macp->macAdr[2];
                macp->BSSID[3] = macp->macAdr[3];
                macp->BSSID[4] = macp->macAdr[4];
                macp->BSSID[5] = macp->macAdr[5];
                zd_writel(cpu_to_le32(*(u32 *)&macp->macAdr[0]), BSSID_P1);
                zd_writel(cpu_to_le32(*(u32 *)&macp->macAdr[4]), BSSID_P2);
        } else
        {
                zd_writel(STA_RX_FILTER, ZD_Rx_Filter);
        }


        macp->intrMask = ZD1205_INT_MASK;

        if (macp->intrMask & DTIM_NOTIFY_EN)
                macp->dtim_notify_en = 1;
        else
                macp->dtim_notify_en = 0;


        if (macp->intrMask & CFG_NEXT_BCN_EN)

                macp->config_next_bcn_en = 1;
        else
                macp->config_next_bcn_en = 0;



        zd1205_ClearTupleCache(macp);
        zd1205_ArReset(macp);

        macp->bTraceSetPoint = 1;
        macp->bFixedRate = 0;
        dot11Obj.bDeviceInSleep = 0;

        macp->bGkInstalled = 0;
        macp->PwrState = PS_CAM;

        // Get Allowed Channel and Default Channel
        dot11Obj.AllowedChannel = zd_readl(ZD_E2P_ALLOWED_CHANNEL);

        dot11Obj.AllowedChannel = 0x7FF;

        ZD1211DEBUG(0, "AllowedChannel = %08x\n", (u32)dot11Obj.AllowedChannel);

        if (!(dot11Obj.AllowedChannel & 0xFFFF0000))
        {
                dot11Obj.AllowedChannel |= 0x10000;
        }


#ifdef HOST_IF_USB
        tmpValue = zd_readl(E2P_SUBID);
        macp->RegionCode = (u16)(tmpValue >> 16);

        dot11Obj.RegionCode = macp->RegionCode;
        macp->LinkLEDn = LED1;
        if (macp->RF_Mode & BIT_4)
        {


                macp->LinkLEDn = LED2;
                ZD1211DEBUG(0, "LED2\n");
        }
        ZD1211DEBUG(0, "LinkLEDn = %x\n", macp->LinkLEDn);

        if (macp->RF_Mode & BIT_8)
        {

                dot11Obj.bOverWritePhyRegFromE2P = 1;
                ZD1211DEBUG(0, "OverWritePhyRegFromE2P\n");


        }


        if (macp->RF_Mode & BIT_9)
        {
                dot11Obj.bIsNormalSize = 1;
                ZD1211DEBUG(0, "NormalSize\n");
        }

        macp->LinkLED_OnDur = 2;
        macp->LinkLED_OffDur = 1;
        macp->DataLED = 0;
        if (macp->RF_Mode & BIT_24)
        {

                macp->LinkLED_OnDur = ((macp->RF_Mode) >> 25) & 0x3;
                macp->LinkLED_OffDur = ((macp->RF_Mode) >> 27) & 0x3;
                if (macp->RF_Mode & BIT_29)
                        macp->DataLED = 1;
        }
        ZD1211DEBUG(1, "LinkLED_OnDur = %d\n", macp->LinkLED_OnDur);

        ZD1211DEBUG(1, "LinkLED_OffDur = %d\n", macp->LinkLED_OffDur);

        if (!(macp->RF_Mode & BIT_10))
        { // The IPC protection: the default is disablesd
                macp->IPCFlag = 4;
        }

        macp->RF_Mode &= 0x0f;

        tmpValue = zd_readl(FW_USB_SPEED);
        dot11Obj.IsUSB2_0 = (u8) tmpValue;
#else

        dot11Obj.bIsNormalSize = 1;
        dot11Obj.IsUSB2_0 = 1;
#endif

        printk("AllowedChannel = %08x\n", (u32)dot11Obj.AllowedChannel);
        printk("Region:%u\n",(unsigned)dot11Obj.RegionCode);


        ZD1211DEBUG(1, "IsUSB2_0 = %d\n", dot11Obj.IsUSB2_0);
        // read Set Point from EEPROM

        tmpValue = zd_readl(ZD_E2P_PWR_INT_VALUE1);
        tmpValue -= cPWR_INT_VALUE_GUARD;
        dot11Obj.IntValue[0] = (u8)tmpValue;
        dot11Obj.IntValue[1] = (u8)(tmpValue >> 8);

        dot11Obj.IntValue[2] = (u8)(tmpValue >> 16);
        dot11Obj.IntValue[3] = (u8)(tmpValue >> 24);

        tmpValue = zd_readl(ZD_E2P_PWR_INT_VALUE2);
        tmpValue -= cPWR_INT_VALUE_GUARD;
        dot11Obj.IntValue[4] = (u8)tmpValue;
        dot11Obj.IntValue[5] = (u8)(tmpValue >> 8);
        dot11Obj.IntValue[6] = (u8)(tmpValue >> 16);
        dot11Obj.IntValue[7] = (u8)(tmpValue >> 24);

        tmpValue = zd_readl(ZD_E2P_PWR_INT_VALUE3);
        tmpValue -= cPWR_INT_VALUE_GUARD;
        dot11Obj.IntValue[8] = (u8)tmpValue;
        dot11Obj.IntValue[9] = (u8)(tmpValue >> 8);
        dot11Obj.IntValue[10] = (u8)(tmpValue >> 16);
        dot11Obj.IntValue[11] = (u8)(tmpValue >> 24);

        tmpValue = zd_readl(ZD_E2P_PWR_INT_VALUE4);
        tmpValue -= cPWR_INT_VALUE_GUARD;
        dot11Obj.IntValue[12] = (u8)tmpValue;
        dot11Obj.IntValue[13] = (u8)(tmpValue >> 8);

        //Initiate a_Calibration_Data CH field
        for (i=0;i<a_CALIBRATED_CH_NUM;i++)
        {
                if((i == 0) || ((i != 0) && (a_ChannelMap[i-1] != a_ChannelMap[i]))) {
                        a_Calibration_Data[0][i] = a_ChannelMap[i];
                } else {
                        a_Calibration_Data[0][i] = 0xff;
                }
        }
        //Initiate a_Interpolation_Data CH field
        for (i=0;i<a_INTERPOLATION_CH_NUM;i++)
                a_Interpolation_Data[0][i] = a_InterpolationTbl[i].a_Channel;

        for (i=0;i<a_CALIBRATED_CH_NUM;i++)
        {
                //Adapter->a_Calibration_Data[0][i] = a_ChannelMap[i];
                if(a_Calibration_Data[0][i] != 0xff) {
                        a_Calibration_Data[1][i] = a_get_cal_int_val((u8)i) - cPWR_INT_VALUE_GUARD;
                        a_Calibration_Data[2][i] = a_get_cal_36M_setpoint_val((u8)i);
                        a_Calibration_Data[3][i] = a_get_cal_48M_54M_setpoint_val((u8)i);
                } else {
                        a_Calibration_Data[1][i] = 0xff;
                        a_Calibration_Data[2][i] = 0xff;
                        a_Calibration_Data[3][i] = 0xff;

                }
        }

        //Calculate Interpolation SetPoints(For 802.11a)
        for (i=0;i<a_INTERPOLATION_CH_NUM;i++)
        {
                //Adapter->a_Interpolation_Data[0][i] = a_InterpolationTbl[i].a_Channel;
                if(a_InterpolationTbl[i].Left_Most_Channel == a_InterpolationTbl[i].Right_Most_Channel) {
                        for (jj=0;jj<a_CALIBRATED_CH_NUM;jj++) {
                                if(a_Calibration_Data[0][jj] == a_InterpolationTbl[i].Left_Most_Channel) {
                                        a_Interpolation_Data[1][i] = a_Calibration_Data[1][jj];
                                        a_Interpolation_Data[2][i] = a_Calibration_Data[2][jj];
                                        a_Interpolation_Data[3][i] = a_Calibration_Data[3][jj];
                                }//Directly used certain calibrated channel values
                        }
                } else {
                        if(0xff == a_get_interpolation_value((u8)i, &a_Interpolation_Data[1][i],
                                                             &a_Interpolation_Data[2][i], &a_Interpolation_Data[3][i]))
                                printk("Get Int/Cal wrong in Interpolation Tbl(%d)\n",i);
                }
        }
        //End Calculate Interpolation SetPoints(For 802.11a)



#if fTX_PWR_CTRL
        for (jj = 0; jj < 3; jj ++)
        {

                for (i = 0; i < 4; i++) {
                        tmpValue = zd_readl(E2P_36M_CAL_VALUE + jj*0x20 + i*4);
                        macp->SetPointOFDM[jj][i*4] = (u8) tmpValue;
                        macp->SetPointOFDM[jj][i*4+1] = (u8) (tmpValue >> 8);
                        if (i != 3) {
                                macp->SetPointOFDM[jj][i*4+2] = (u8) (tmpValue >> 16);
                                macp->SetPointOFDM[jj][i*4+3] = (u8) (tmpValue >> 24);
                        }
                }
        }
#endif


        zd_writel(0x00000064,ZD_BCNInterval);
        HW_UpdateBcnInterval(&dot11Obj, 0x00000064);

        // read Set Point from EEPROM
        tmpValue = zd_readl(ZD_E2P_PWR_CAL_VALUE1);
        macp->EepSetPoint[0] = (u8)tmpValue;
        macp->EepSetPoint[1] = (u8)(tmpValue >> 8);
        macp->EepSetPoint[2] = (u8)(tmpValue >> 16);
        macp->EepSetPoint[3] = (u8)(tmpValue >> 24);

        tmpValue = zd_readl(ZD_E2P_PWR_CAL_VALUE2);

        macp->EepSetPoint[4] = (u8)tmpValue;

        macp->EepSetPoint[5] = (u8)(tmpValue >> 8);
        macp->EepSetPoint[6] = (u8)(tmpValue >> 16);
        macp->EepSetPoint[7] = (u8)(tmpValue >> 24);


        tmpValue = zd_readl(ZD_E2P_PWR_CAL_VALUE3);
        macp->EepSetPoint[8] = (u8)tmpValue;

        macp->EepSetPoint[9] = (u8)(tmpValue >> 8);
        macp->EepSetPoint[10] = (u8)(tmpValue >> 16);
        macp->EepSetPoint[11] = (u8)(tmpValue >> 24);

        tmpValue = zd_readl(ZD_E2P_PWR_CAL_VALUE4);
        macp->EepSetPoint[12] = (u8)tmpValue;
        macp->EepSetPoint[13] = (u8)(tmpValue >> 8);

        HW_SetRfChannel(&dot11Obj, (dot11Obj.AllowedChannel >> 16), 0, MIXED_MODE);
        // For Antenna Diversity Parameters
        macp->bEnableSwAntennaDiv = 0;
        macp->Ant_MonitorDur1 = 10;//100;
        macp->Ant_MonitorDur2 = 1;
        macp->NiceSQThr = 48;

        macp->rxOffset = ZD_RX_OFFSET;

        macp->bPSMSupported = 0;
        macp->NormalBackoff = 0x7f047f;
        macp->UrgentBackoff = 0x7f0407;
        macp->LooseBackoff = 0x7f107f;
        macp->WorseSQThr = 0x48;
        macp->MulticastAddr[0] = 0;
        macp->iv16 = 0;
        macp->iv32 = 0;
        macp->EnableTxPwrCtrl = 1;
        macp->PSThreshhold = 10000;

#if fANT_DIVERSITY
        //    macp->NiceSQThr_OFDM = 12 * 4;    // 12 dB --> 48 %
        macp->NiceSQThr_OFDM = 48;       // 48 %
        macp->bEnableSwAntennaDiv = 1;
#endif

#if fWRITE_WORD_REG

        macp->FlashType = 0xFF;

#endif


        macp->PHYTestIndex = 5;
        macp->PHYTestRssiBound = 0x3a;
        macp->PHYTestTimer = 30;
        macp->TrafficBound = 200;

        macp->PHYLowPower = 3;  // Tx/Rx enable
        dot11Obj.CR122Flag = 2; // initial value

        dot11Obj.CR31Flag = 2; // initial value
        dot11Obj.CR203Flag = 2; // initial value
        dot11Obj.PhyTest = 4;
        macp->AdapterMaxRate = 0x0B;  // initail max rate = 54M
        ZEXIT(0);
}


int zd1205_dis_connect(struct zd1205_private *macp)
{
        u32 tmpvalue;

        ZD1211DEBUG(0, "zd1205_dis_connect\n");

        netif_carrier_off(macp->device);
        netif_stop_queue(macp->device);

        // Note: The following sequence is important, do not change it arbitrarily.
        macp->bAssoc = 0;
        macp->PwrState = PS_CAM;

        // PwrMgt = 0
        down(&macp->bcn_sem);
        tmpvalue = zd_readl(ZD_BCNInterval);
        tmpvalue &= ~POWER_MNT;
        zd_writel(tmpvalue, ZD_BCNInterval);
        up(&macp->bcn_sem);


#ifndef HOST_IF_USB

        while (dot11Obj.bDeviceInSleep)
        {
                wait_ms(1);
                //if (macp->Notification & RQ_TERMINATION){
                //	break;
                //}
        }
#else


        if (dot11Obj.bDeviceInSleep)
        {
                ZD1211DEBUG(1, "Device in sleep\n");

                return 1;

        }
#endif

        // After all pending packets are served, then notify NDIS with
        // DISCONNECT.
        // Otherwise, it might cause IP Address = 0.0.0.0

#ifndef HOST_IF_USB
        while (macp->activeTxQ->count)
        {
                // After this step, we can make sure that no ATIM pended.
                wait_ms(1);
                //if (macp->Notification & RQ_TERMINATION){
                //	break;
                //}
        }
#else
        //if (macp->activeTxQ->count){
        //    ZD1211DEBUG(1, "activeTxQ not empty\n");

        //    return 1;
        //}

#endif

        // Notice PS_Change
        zd_EventNotify(EVENT_PS_CHANGE, (u8)macp->PwrState, 0, 0);

        // IBSS = 0, i.e, stop sending beacon.
        down(&macp->bcn_sem);
        tmpvalue = zd_readl(ZD_BCNInterval);
        tmpvalue &= ~IBSS_MODE;
        zd_writel(tmpvalue, ZD_BCNInterval);
        up(&macp->bcn_sem);
        zd1205_notify_disjoin_event(macp);


        // We may need issue disassociate frame.
        // Issue notification

        return 0;
}



#ifndef HOST_IF_USB

static void zd1205_dtim_notify(
        struct zd1205_private *macp

)

{
        zd1205_SwTcb_t *sw_tcb;
        u32 tmp_value;

        zd_EventNotify(EVENT_DTIM_NOTIFY, 0, 0, 0);
        if (!macp->activeTxQ->count)
                sw_tcb = macp->freeTxQ->first;
        else
                sw_tcb = macp->activeTxQ->first;


        tmp_value = zd_readl(DeviceState);
        tmp_value &= 0xf;
        zd1205_start_download(sw_tcb->TcbPhys | BIT_0);

}
#endif

void zd1205_config_wep_keys(struct zd1205_private *macp)
{
        card_Setting_t *pSetting = &macp->cardSetting;

        u8 encryMode = pSetting->EncryMode;
        u8 i, j;
        u8 DynKeyMode = pSetting->DynKeyMode;
        u8 keyLength;

        if ((encryMode == 0) || (DynKeyMode != 0))
        {
                HW_CAM_Write(&dot11Obj, DEFAULT_ENCRY_TYPE, NO_WEP);
                return;
        }

        if (pSetting->OperationMode != CAM_AP_VAP)
        {
                HW_CAM_ResetRollTbl(&dot11Obj); //force CAM to use default encry type

                switch(encryMode) {
                case WEP64:
                        ZD1211DEBUG(0, "WEP64 Mode\n");
                        keyLength = 5;
                        break;

                case WEP128:
                        ZD1211DEBUG(0, "WEP128 Mode\n");
                        keyLength = 13;
                        break;

                case WEP256:
                        ZD1211DEBUG(0, "WEP256 Mode\n");
                        keyLength = 29;
                        break;

                default:
                        ZD1211DEBUG(0, "Not supported Mode\n");
                        ZD1211DEBUG(0, "encryMode = %d\n", encryMode);

                        return;
                }

                HW_CAM_Write(&dot11Obj, DEFAULT_ENCRY_TYPE, encryMode);

                for (i=0, j=0; i<4; i++, j+=8) { //one key occupy 32 bytes space
                        HW_ConfigStatKey(&dot11Obj, &pSetting->keyVector[i][0], keyLength, STA_KEY_START_ADDR+j);
                }
        }
        return;
}
#if 0
void zd1205_config_dyn_key(u8 DynKeyMode, u8 *pkey, int idx)
{
        u8 keyLength;
        u8 encryMode;
        u32 offset;

        HW_CAM_ResetRollTbl(&dot11Obj); //force CAM to use default encry type

        switch(DynKeyMode) {
        case DYN_KEY_TKIP:
                WPADEBUG("Dynamic key TKIP mode\n");
                keyLength = 32;
                encryMode = TKIP;
                break;

        case DYN_KEY_AES:
                WPADEBUG("Dynamic key AES mode\n");
                keyLength = 16;
                encryMode = AES;
                break;

        default:
                WPADEBUG("Do not support the Dynamic key mode = %d\n", DynKeyMode);
                return;
        }

        /* The size for each key is 256 bits (32 bytes) */
        offset = idx * 8;

        HW_CAM_Write(&dot11Obj, DEFAULT_ENCRY_TYPE, encryMode);
        HW_ConfigStatKey(&dot11Obj, pkey, keyLength, STA_KEY_START_ADDR+offset);

        return;
}
#endif
static int zd1205_validate_frame(
        struct zd1205_private *macp,
        zd1205_RFD_t *rfd
)
{
        plcp_wla_Header_t *wla_hdr;
        u32	min_length;
        u32 frame_len;

        u32 len, tot_len;
        u8	bOfdmFrm = 0;
        u8	PlcpRate;
        //u8	rx_offset = macp->rxOffset;

        u8	FrameEndInd;
#if ZDCONF_WE_STAT_SUPPORT == 1

        U32 Tmp=0;
        static U8  Qual[10]={0,0,0,0,0,0,0,0,0,0};
        static U8   Str[10]={0,0,0,0,0,0,0,0,0,0};
        static U32  Qidx=0;
#elif !defined(ZDCONF_WE_STAT_SUPPORT)
	#error "Undefine ZDCONF_WE_STAT_SUPPORT"
#endif

        u32 idx;


        // Extension Info
        /*********************************************************************/
        /* Signal Quality | Signal Strength | Signal Quality2 | Noise Report */
        /*********************************************************************/

        /*****************************************************/
        /* DA Index | SA Index | Rx Decrypt Type | Rx Status */
        /*****************************************************/

        // Accept Data/Management frame only.
        wla_hdr = (plcp_wla_Header_t *)&rfd->RxBuffer[macp->rxOffset];

        tot_len=(le32_to_cpu(rfd->ActualCount) & 0x3fff);
        len = tot_len - EXTRA_INFO_LEN;
        frame_len = tot_len - macp->rxOffset;

        PlcpRate = wla_hdr->PlcpHdr[0];

        if (frame_len == 0)
        {
                macp->ErrZeroLenFrmCnt++;
                return false;
        }

        //pSwRfd->pRfd->RxBuffer[frameLen+rxOffset-1]
        //bit7:	error frame
        //bit6:	crc16 error
        //bit5:	address not match
        //bit4: crc32 error
        //bit3:	decrypt error

        //bit2: overrun

        //bit1: Rx Timeout
        //bit0:	OFDM modulation

        macp->rxDecryType = rfd->RxBuffer[tot_len-2];
        FrameEndInd = rfd->RxBuffer[tot_len-1];
        if (FrameEndInd & BIT_7)
        {
                macp->ErrToHostFrmCnt++;
                return FALSE;
        }

        if (bWepBit(wla_hdr))
        {
                //if (macp->cardSetting.EncryMode == ENCRY_TKIP)
                //    min_length = 48;
                //else
                min_length = 44;
                if (frame_len < min_length) {
                        //printk(KERN_DEBUG "frame_len = %x\n", frame_len);
                        macp->ErrShortFrmCnt++;
                        return false;
                }
        } else
        {
                // Minimum Length = PLCP(5)+MACHeader(24)+EXTINFO(5)+CRC(4)
                if (frame_len < 36) {
                        //printk(KERN_DEBUG "frame_len = %x\n", frame_len);
                        macp->ErrShortFrmCnt++;
                        return false;
                }
        }

        // Check if frame_len > MAX_WLAN_SIZE.
        if (frame_len > ZD_MAX_WLAN_SIZE)
        {
                // Do not worry about the corruption of HwRfd.
                // If the frame_len > 2410, the Rx-Bus-Master skip the bytes that exceed
                // 2410 to protect the structure of HwRfd.
                // However, the Rx-Bus-Master still reports this frame to host if the frame
                // is recognized as good by the FA(Frame Analyzer).
                macp->ErrLongFrmCnt++;
                return false;
        }

        // Check if the SwRfd->frame_len matched the length derived from PLCP.
        bOfdmFrm = (FrameEndInd & BIT_0);

        if (bOfdmFrm)
        {
                // it's OFDM
                macp->rxOFDMDataFrame++;
#ifdef HOST_IF_USB

                macp->PHYFreOFDMframe = 1;
#endif

                switch(PlcpRate & 0xF) {
                case 0x0B:	//6M

                        macp->rxInfo.rate = RATE_6M;
                        break;

                case 0x0F:	//9M
                        macp->rxInfo.rate = RATE_9M;
                        break;


                case 0x0A:	//12M
                        macp->rxInfo.rate = RATE_12M;

                        break;

                case 0x0E:	//18M
                        macp->rxInfo.rate = RATE_18M;
                        break;


                case 0x09:	//24M
                        macp->rxInfo.rate = RATE_24M;
                        break;

                case 0x0D:	//36M
                        macp->rxInfo.rate = RATE_36M;
                        break;

                case 0x08:	//48M
                        macp->rxInfo.rate = RATE_48M;
                        break;

                case 0x0C:	//54M
                        macp->rxInfo.rate = RATE_54M;
                        break;

                default:
                        break;
                }

        } else
        {
                // it's CCK
                macp->rx11bDataFrame++;
                // the value from PHY is in scale from Max is 0 and Min is 0xb5
                switch(PlcpRate) {
                case 0x0A:

                        macp->rxInfo.rate = RATE_1M;
                        break;

                case 0x14:
                        macp->rxInfo.rate = RATE_2M;
                        break;

                case 0x37:
                        macp->rxInfo.rate = RATE_5M;
                        break;

                case 0x6E:
                        macp->rxInfo.rate = RATE_11M;
                        break;

                default:
                        break;
                }
        }


        macp->rxSignalQuality = rfd->RxBuffer[len];
        macp->rxSignalQuality1 = macp->rxSignalQuality;
        macp->rxSignalStrength = rfd->RxBuffer[len+1];
        macp->rxSignalQuality2 = rfd->RxBuffer[len+2];
        //macp->rxNoiseReport = rfd->RxBuffer[len+3]; //3d31

#ifdef HOST_IF_USB

        macp->rxSignalQuality = CalculateQuality(macp, rfd, &macp->rxSignalQualityIndB);
        macp->rxSignalStrength = CalculateStrength(macp, rfd);
#endif
#if ZDCONF_WE_STAT_SUPPORT == 1

        Qual[Qidx++ % 10] = macp->rxSignalQuality;
        Str[Qidx % 10] = macp->rxSignalStrength;

        if(Qidx % 100 == 0)
        {
                Tmp = 0;
                for(idx=0;idx<10;idx++)
                        Tmp += Qual[idx];
                macp->iwstats.qual.qual=Tmp/10;

                Tmp = 0;
                for(idx=0;idx<10;idx++)
                        Tmp += Str[idx];
                Tmp/=10;

                Tmp = -(100 - Tmp);
                Tmp = Tmp >  -40 ?  -40: Tmp;
                Tmp = Tmp < -105 ? -105: Tmp;
                Tmp = (Tmp + 105)*100/65;
                macp->iwstats.qual.level= Tmp;

                Tmp = -(100 - rfd->RxBuffer[len+3]);
                Tmp = Tmp >  -40 ?  -40: Tmp;
                Tmp = Tmp < -105 ? -105: Tmp;
                Tmp = (Tmp + 105)*100/65;
                macp->iwstats.qual.noise= Tmp;

        }
#elif !defined(ZDCONF_WE_STAT_SUPPORT)
	#error "Undefine ZDCONF_WE_STAT_SUPPORT"
#endif

        macp->rxInfo.signalQuality = macp->rxSignalQuality;
        macp->rxInfo.signalStrength = macp->rxSignalStrength;

        return true;
}

/**
 * zd1205_alloc_tcb_pool - allocate TCB circular list
 * @macp: atapter's private data struct
 *
 * This routine allocates memory for the circular list of transmit descriptors.
 *
 * Returns:
 *       0: if allocation has failed.
 *       1: Otherwise.
 */

int
zd1205_alloc_tcb_pool(struct zd1205_private *macp)
{
        /* deal with Tx uncached memory */
        /* Allocate memory for the shared transmit resources with enough extra mem
         * to paragraph align (4-byte alignment) everything  */

        macp->txUnCachedSize = (macp->numTcb *
                                (sizeof(zd1205_HwTCB_t)+ sizeof(zd1205_Ctrl_Set_t)+sizeof(zd1205_Header_t)))
                               + (macp->numTbd * sizeof(zd1205_TBD_t));

#ifndef HOST_IF_USB

        if (!(macp->txUnCached = pci_alloc_consistent(macp->pdev,
                                 macp->txUnCachedSize, &(macp->txUnCachedPhys))))
        {
                return 0;
        }
#else
        macp->txUnCached = kmalloc(macp->txUnCachedSize, GFP_ATOMIC);
        if (!macp->txUnCached)
        {
                printk(KERN_ERR "zd1205: kmalloc txCached failed\n");
                return 0;
        }
#endif

        memset(macp->txUnCached, 0x00, macp->txUnCachedSize);
        return 1;
}

void
zd1205_free_tcb_pool(struct zd1205_private *macp)
{
#ifndef HOST_IF_USB

        pci_free_consistent(macp->pdev, macp->txUnCachedSize,

                            macp->txUnCached, macp->txUnCachedPhys);
#else

        if (macp->txUnCached)
                kfree(macp->txUnCached);
#endif

        macp->txUnCachedPhys = 0;

}



static void
zd1205_free_rfd_pool(struct zd1205_private *macp)
{
        struct rx_list_elem *rx_struct;

        while (!list_empty(&(macp->active_rx_list)))
        {
                rx_struct = list_entry(macp->active_rx_list.next,
                                       struct rx_list_elem, list_elem);
                list_del(&(rx_struct->list_elem));


#ifndef HOST_IF_USB

                pci_unmap_single(macp->pdev, rx_struct->dma_addr,

                                 sizeof (zd1205_RFD_t), PCI_DMA_TODEVICE);
                dev_kfree_skb(rx_struct->skb);
#else

                dev_kfree_skb(rx_struct->skb);
#endif

                kfree(rx_struct);
        }


        while (!list_empty(&(macp->rx_struct_pool)))
        {
                rx_struct = list_entry(macp->rx_struct_pool.next,
                                       struct rx_list_elem, list_elem);
                list_del(&(rx_struct->list_elem));
                kfree(rx_struct);
        }

}



/**
 * zd1205_alloc_rfd_pool - allocate RFDs
 * @macp: atapter's private data struct

 *
 * Allocates initial pool of skb which holds both rfd and data,
 * and return a pointer to the head of the list
 */



static int
zd1205_alloc_rfd_pool(struct zd1205_private *macp)
{
        struct rx_list_elem *rx_struct;
        int i;



        INIT_LIST_HEAD(&(macp->active_rx_list));
        INIT_LIST_HEAD(&(macp->rx_struct_pool));
        macp->skb_req = macp->numRfd;


        for (i = 0; i < macp->skb_req; i++)
        {
                rx_struct = kmalloc(sizeof (struct rx_list_elem), GFP_ATOMIC);
                list_add(&(rx_struct->list_elem), &(macp->rx_struct_pool));
        }

        zd1205_alloc_skbs(macp);
        return !list_empty(&(macp->active_rx_list));
}


void

zd1205_clear_pools(struct zd1205_private *macp)
{
        zd1205_dealloc_space(macp);
        zd1205_free_rfd_pool(macp);

        zd1205_free_tcb_pool(macp);
}


/**
 * zd1205_start_ru - start the RU if needed
 * @macp: atapter's private data struct
 *
 * This routine checks the status of the receive unit(RU),
 * and starts the RU if it was not already active.  However,
 * before restarting the RU, the driver gives the RU the buffers
 * it freed up during the servicing of the ISR. If there are
 * no free buffers to give to the RU, (i.e. we have reached a
 * no resource condition) the RU will not be started till the
  * next ISR.
 */
#ifndef HOST_IF_USB
void zd1205_start_ru(struct zd1205_private *macp)
#else
struct rx_list_elem *zd1205_start_ru(struct zd1205_private *macp)
#endif
{
#ifndef HOST_IF_USB
        u32 tmp_value;
        u32 loopCnt = 0;
#endif

        struct rx_list_elem *rx_struct = NULL;
        struct list_head *entry_ptr = NULL;

        zd1205_RFD_t *rfd = 0;
        int buffer_found = 0;
        struct sk_buff *skb;

        ZENTER(4);

        list_for_each(entry_ptr, &(macp->active_rx_list))
        {
                rx_struct = list_entry(entry_ptr, struct rx_list_elem, list_elem);

                if (!rx_struct)
#ifndef HOST_IF_USB

                        return;
#else

                        return NULL;
#endif

#ifndef HOST_IF_USB

                pci_dma_sync_single(macp->pdev, rx_struct->dma_addr,

                                    macp->rfd_size, PCI_DMA_FROMDEVICE);
#endif

                skb = rx_struct->skb;
                rfd = RFD_POINTER(skb, macp);	/* locate RFD within skb */

                if (SKB_RFD_STATUS(rx_struct->skb, macp) !=  __constant_cpu_to_le32(RFD_STATUS_COMPLETE)) {
                        buffer_found = 1;
                        break;
                }

        }


        /* No available buffers */
        if (!buffer_found)
        {
                printk(KERN_ERR "zd1205: No available buffers\n");
#ifndef HOST_IF_USB

                return;
#else

                return NULL;
#endif

        }


#ifndef HOST_IF_USB
        while(1)
        {
                tmp_value = zd_readl(DeviceState);
                tmp_value &= 0xf0;
                if ((tmp_value == RX_READ_RCB) || (tmp_value == RX_CHK_RCB)) {
                        /* Device is now checking suspend or not.
                         Keep watching until it finished check. */
                        loopCnt++;
                        if (loopCnt > 10000000)
                                break;

                        udelay(1);
                        continue;
                } else {
                        break;
                }




        }
        if (loopCnt > 10000000)
                ZD1211DEBUG(0, "I am in zd1205_start_ru loop\n");


        if (tmp_value == RX_IDLE)
        {
                /* Rx bus master is in idle state. */
                if ((u16)le32_to_cpu(rfd->CbStatus) != RFD_STATUS_COMPLETE) {
                        zd1205_start_upload(rx_struct->dma_addr);

                }
        }

        ZEXIT(4);

        return;
#else


        return rx_struct;


#endif
}

void zd1205_recycle_rx(struct zd1205_private *macp)
{
        struct rx_list_elem *rx_struct = NULL;
        struct list_head *entry_ptr = NULL;
        zd1205_RFD_t *rfd = 0;
        struct sk_buff *skb;
        ZENTER(4);

        list_for_each(entry_ptr, &(macp->active_rx_list))
        {
                rx_struct = list_entry(entry_ptr, struct rx_list_elem, list_elem);
                if (!rx_struct)
                        return;

                rx_struct->UnFinishFrmLen = 0;
                skb = rx_struct->skb;
                rfd = RFD_POINTER(skb, macp);	/* locate RFD within skb */
                rfd->CbStatus = 0xffffffff;
        }
}


void zd1205_CheckBeaconInfo(struct zd1205_private *macp, plcp_wla_Header_t *pWlanHdr, u8 *pMacBody, u32 bodyLen)
{
        u8 *pBssid;
        u8 *pByte;
        u32 currPos = 0;
        u8 elemId, elemLen;
        u8 Zd1202Detected = 0;
        u8 BitmapCtrl;
        u16 N1;
        u16 N2;
        u16 quotient;
        u16 remainder;
        u8 BssType = macp->cardSetting.BssType;
        u8 erp;
        u16 cap;
        u8 preamble = 0;
        u16 basicRateMap = 0;
        u16 supRateMap = 0;
        u8 bErpSta = 0;
        int i;
        u8 tmpMaxRate = 0x02;
        u8 rate;
        //u16 sequence_number;

        cap =  (*(pMacBody + CAP_OFFSET)) + ((*(pMacBody + CAP_OFFSET +1)) << 8);

        if (cap & BIT_1) //IBSS
                pBssid = pWlanHdr->Address3;
        else
        {
                pBssid = pWlanHdr->Address2;
                if (BssType == INDEPENDENT_BSS)
                        return;
        }
        //sequence_number = (*(u16*)(&pWlanHdr->SeqCtrl[0])) >> 4;

        if (cap & BIT_5)
                preamble = 1;
        else
                preamble = 0;

        //get element
        pByte = pMacBody + SSID_OFFSET;

        currPos = SSID_OFFSET;

        while(currPos < bodyLen)
        {
                elemId = *pByte;
                elemLen = *(pByte+1);

                switch (elemId) {
                case ELEID_DSPARMS:
                        if (BssType==INFRASTRUCTURE_BSS && macp->bAssoc && dot11Obj.Channel != *(pByte+2)) {
                                macp->bAssoc=0;
                                printk("Channel changed:%u to %u\n",dot11Obj.Channel,*(pByte+2));
                                zd_CmdProcess(CMD_ROAMING,0,0);
                        }
                        pByte += (elemLen+2);
                        break;
                case ELEID_SSID:
                        //printk("Get ssid\n");
                        if (BssType == INFRASTRUCTURE_BSS && macp->bAssoc &&(dot11DesiredSsid.buf[1]==0)) // Any ssid
                        {
                                //printk("Rx ssidlen=%d\n",elemLen);

                                if (memcmp((pByte+2),&dot11Obj.CurrSsid[2],elemLen)==0 && dot11Obj.CurrSsid[1]==elemLen)
                                {
                                        //   printk("SSID Same\n");
                                } else
                                {
                                        macp->bAssoc=0;
                                        printk("SSID Changed\n");
                                        zd_CmdProcess(CMD_ROAMING,0,0);
                                }
                        }
                        pByte += (elemLen+2);
                        break;


                case 0xfe:	//ZyDAS Extended Supported Rate (16.5M)
                case 0xff:	//ZyDAS Extended Supported Rate (27.5M)
                        Zd1202Detected = 1;

                        if (elemLen != 0) { //For possible future compatibility issue,
                                //we adopt "length = 0", which will not
                                //disturb others.
                        }
                        pByte += (elemLen+2);
                        break;

                case ELEID_TIM:
                        if ((BssType == INFRASTRUCTURE_BSS) && (macp->bAssoc)) {
                                if (elemLen >= 3) {
                                        BitmapCtrl = *(pByte+4);
                                        N1 = (BitmapCtrl & ~BIT_0);
                                        N2 = (elemLen + N1 - 4);
                                        quotient = (dot11Obj.Aid >> 3);
                                        remainder = (dot11Obj.Aid & 0x7);

                                        if ((quotient < N1) || (quotient > N2)) {
                                                macp->bAnyActivity = 0;
                                                pByte += (elemLen+2);
                                                break;
                                        }

                                        if ((*(pByte+5+quotient-N1) >> remainder) & BIT_0) {
                                                //ZD1211DEBUG(1, "TIM: More data at SequNum=%u\n",sequence_number);
                                                zd_EventNotify(EVENT_MORE_DATA, 0, 0, 0);
                                                macp->bAnyActivity = 1;
                                                //ZD1211DEBUG(2, "EVENT_MORE_DATA!\n");
                                                //zd1205_dump_data("TIM", pByte, elemLen+2);
                                                //zd1205_dump_data("pMacBody", (u8 *)pMacBody, bodyLen);
                                        } else {
                                                macp->bAnyActivity = 0;
                                        }

                                        // Multicast frames queued in AP
                                        if (BitmapCtrl & BIT_0) {
                                                ZD1211DEBUG(1, "Got multicast framed queued information!\n");
                                                macp->bAnyActivity = 1;
                                        }
                                }
                        }
                        pByte += (elemLen+2);
                        break;


                case ELEID_ERP_INFO:
                        if (macp->bAssoc) {
                                erp = *(pByte+2);

                                if (erp & USE_PROTECTION_BIT) {
                                        if (!(dot11Obj.ConfigFlag & ENABLE_PROTECTION_SET))
                                                defer_kevent(macp, KEVENT_EN_PROTECTION);
                                } else {
                                        if (dot11Obj.ConfigFlag & ENABLE_PROTECTION_SET)
                                                defer_kevent(macp, KEVENT_DIS_PROTECTION);
                                }

                                // check Barker_Preamble_Mode
                                if (erp & BARKER_PREAMBLE_BIT) {
                                        if (!(dot11Obj.ConfigFlag & BARKER_PREAMBLE_SET))
                                                defer_kevent(macp, KEVENT_EN_BARKER);
                                } else {
                                        if (dot11Obj.ConfigFlag & BARKER_PREAMBLE_SET)
                                                defer_kevent(macp, KEVENT_DIS_BARKER);
                                }

                                // check B-STA
                                if (erp & NON_ERP_PRESENT_BIT) {
                                        if (dot11Obj.ConfigFlag & SHORT_SLOT_TIME_SET) {
                                                defer_kevent(macp, KEVENT_DIS_SHORT_SLOT);
                                        }
                                } else {
                                        if (!(dot11Obj.ConfigFlag & SHORT_SLOT_TIME_SET)) {
                                                defer_kevent(macp, KEVENT_EN_SHORT_SLOT);
                                        }
                                }
                        }

                        pByte += (elemLen+2);
                        break;

                case ELEID_SUPRATES:
                        if ((BssType == INDEPENDENT_BSS) && (macp->bAssoc)) {
                                zd_makeRateInfoMAP(pByte, &basicRateMap, &supRateMap);
                                for (i=0; i<elemLen; i++) {
                                        rate = *(pByte+2+i);
                                        if ((rate & 0x7f) > tmpMaxRate)
                                                tmpMaxRate = (rate & 0x7f);
                                }
                        }
                        pByte += (elemLen+2);
                        break;

                case ELEID_EXT_RATES:
                        if ((BssType == INDEPENDENT_BSS) && (macp->bAssoc)) {
                                zd_makeRateInfoMAP(pByte, &basicRateMap, &supRateMap);

                                for (i=0; i<elemLen; i++) {
                                        rate = *(pByte+2+i);
                                        if ((rate & 0x7f) > tmpMaxRate)
                                                tmpMaxRate = (rate & 0x7f);
                                }
                        }

                        pByte += (elemLen+2);
                        break;

                default:
                        pByte += (elemLen+2);
                        break;

                }

                currPos += (elemLen+2);
        }


        if (Zd1202Detected)
        {
                macp->BSS_Members |= MEMBER_ZD1202;
        } else
        {
                macp->BSS_Members |= MEMBER_OTHERS;
        }

        if ((BssType == INDEPENDENT_BSS) && (macp->bAssoc))
        {
                if (supRateMap > 0x0f) {  //support rates include OFDM rates
                        if (basicRateMap & ~0xf) // basic rates include OFDM rates
                                bErpSta = 1;
                        else
                                bErpSta = 1;
                } else
                        bErpSta = 0;

                zd_UpdateIbssInfo(pWlanHdr->Address2, tmpMaxRate, preamble, bErpSta);
                if ((macp->cardSetting.MacMode != PURE_A_MODE) &&(macp->cardSetting.MacMode != PURE_B_MODE) &&  (!bErpSta)) {
                        if (!(dot11Obj.ConfigFlag & ENABLE_PROTECTION_SET)) {
#ifdef HOST_IF_USB
                                defer_kevent(macp, KEVENT_EN_PROTECTION);
                                ZD1211DEBUG(2, "KEVENT_EN_PROTECTION\n");
#else

                                zd_EventNotify(EVENT_ENABLE_PROTECTION, 1, 0, 0);
#endif

                        }
                }
        }

        macp->Bcn_Acc_Num++;
        macp->Bcn_Acc_SQ += macp->rxInfo.signalQuality;
        return;
}



#define ETH_P_80211_RAW     (ETH_P_ECONET + 1)

/**
 * zd1205_rx_isr - service RX queue
 * @macp: atapter's private data struct
 * @max_number_of_rfds: max number of RFDs to process
 * @rx_congestion: flag pointer, to inform the calling function of congestion.
 *
 * This routine processes the RX interrupt & services the RX queues.
 * For each successful RFD, it allocates a new msg block, links that

 * into the RFD list, and sends the old msg upstream.
 * The new RFD is then put at the end of the free list of RFD's.
 * It returns the number of serviced RFDs.
 */

u32 zd1205_rx_isr(struct zd1205_private *macp)
{
        zd1205_RFD_t *rfd;		/* new rfd, received rfd */
        int i;
        u32 rfd_status;
        struct sk_buff *skb;
        struct net_device *dev;
        u32 data_sz;
        struct rx_list_elem *rx_struct;
        u32 rfd_cnt = 0;
        plcp_wla_Header_t *wla_hdr;
        u8 *pHdr;
        u8 *pIv;
        u8 *pBody = NULL;
        u32 bodyLen = 0;
        u32 hdrLen = WLAN_HEADER;
        u16 seq = 0;
        u8 frag = 0;
        u8 *pTa = NULL;
        defrag_Array_t *pDefArray = &macp->defragArray;
        u8 EthHdr[12];
        card_Setting_t *pSetting = &macp->cardSetting;
        u8 bDataFrm = 0;
        u8 BaseFrmType = 0;
        int SaIndex = 0;
        u8 BssType=pSetting->BssType;
        u8 bSwCheckMIC=0;
        u8 rxDecryType = 0;

#ifdef HOST_IF_USB
    #if fMERGE_RX_FRAME

        int rx_cnt;
        struct rx_list_elem **rx_struct_array = macp->rx_struct_array;
        int total_rx_cnt = macp->total_rx_cnt;
#else

        u8 RunOnce = 0;
#endif
#endif

        ZENTER(4);
        dev = macp->device;
        /* current design of rx is as following:
         * 1. socket buffer (skb) used to pass network packet to upper layer
         * 2. all HW host memory structures (like RFDs, RBDs and data buffers)
         *    are placed in a skb's data room
         * 3. when rx process is complete, we change skb internal pointers to exclude
         *    from data area all unrelated things (RFD, RDB) and to leave
         *    just rx'ed packet netto
         * 4. for each skb passed to upper layer, new one is allocated instead.
         * 5. if no skb left, in 2 sec another atempt to allocate skbs will be made
         *    (watchdog trigger SWI intr and isr should allocate new skbs)
         */


#if fMERGE_RX_FRAME //USB

        for (rx_cnt=0; rx_cnt<total_rx_cnt; rx_cnt++)
        {
#else //PCI
        for (i=0; i<macp->numRfd; i++) {
#endif

#if fMERGE_RX_FRAME
                rx_struct = rx_struct_array[rx_cnt];
                skb = rx_struct->skb;
                rfd = RFD_POINTER(skb, macp);
                rfd_status = SKB_RFD_STATUS(rx_struct->skb, macp);

                if (rfd_status != __constant_cpu_to_le32(RFD_STATUS_COMPLETE))	/* does not contains data yet - exit */
                        break;
                macp->DriverRxFrmCnt ++;
#else   //end of fMERGE_RX_FRAME

                if (list_empty(&(macp->active_rx_list))) {
                        printk(KERN_ERR "zd1205: list_empty\n");
                        break;
                }

                rmb();

                rx_struct = list_entry(macp->active_rx_list.next,
                                       struct rx_list_elem, list_elem);

                ZD1211DEBUG(4, "zd1211: rx_struct = %x\n", (u32)rx_struct);
                skb = rx_struct->skb;
                rfd = RFD_POINTER(skb, macp);	/* locate RFD within skb */

                // sync only the RFD header
#ifndef HOST_IF_USB

                pci_dma_sync_single(macp->pdev, rx_struct->dma_addr,
                                    macp->rfd_size+PLCP_HEADER+WLAN_HEADER, PCI_DMA_FROMDEVICE);
#endif

                rfd_status = SKB_RFD_STATUS(rx_struct->skb, macp);	/* get RFD's status */

                ZD1211DEBUG(4, "zd1211: rfd_status = %x\n", rfd_status);
                if (rfd_status != __constant_cpu_to_le32(RFD_STATUS_COMPLETE))	/* does not contains data yet - exit */
                        break;

#ifdef HOST_IF_USB

                if (RunOnce)
                        break;
                RunOnce = 1;
#endif

                macp->DriverRxFrmCnt ++;
                /* to allow manipulation with current skb we need to unlink it */
                list_del(&(rx_struct->list_elem));

#ifdef HOST_IF_USB

                zd1211_submit_rx_urb(macp);
#endif

#endif //end of !fMERGE_RX_FRAME

                bSwCheckMIC=0;

                data_sz = (u16)(le32_to_cpu(rfd->ActualCount) & 0x3fff);
                data_sz -= macp->rxOffset;
                ZD1211DEBUG(4, "zd1211: data_sz = %x\n", data_sz);
#if 0
                //for debug only
                if (macp->bPSMSupported)
                        zd1205_dump_data("RxBuffer", (u8 *)rfd->RxBuffer, data_sz);

                //zd1205_add_skb_to_end(macp, rx_struct);
                //continue;
#endif

                wla_hdr = (plcp_wla_Header_t *)&rfd->RxBuffer[macp->rxOffset];
                pHdr = (u8 *)wla_hdr + PLCP_HEADER;

                /*
				u8 *mb = hdrLen+ pHdr;
				if(mb[0] == 0xf0 && mb[1] == 0xf0)
					printk("Get NETBIOS\n");
                */

                if (SubFrameType(wla_hdr) != BEACON) {
                        macp->bFrmRxed1 = 1;
                }

                        /* ath_desc: add monitor mode */
                        BaseFrmType = BaseFrameType(wla_hdr);

                        if ((BaseFrmType == DATA) || (BaseFrmType == MANAGEMENT)) { //Data or Management Frames
                                /* do not free & unmap badly recieved packet.
                                 * move it to the end of skb list for reuse */

#ifndef HOST_IF_USB
                                //sync for access correctly
                                pci_dma_sync_single(macp->pdev, rx_struct->dma_addr,
                                                    data_sz + macp->rfd_size, PCI_DMA_FROMDEVICE);
#endif

                                if (zd1205_validate_frame(macp, rfd) == false) {
                                        //int i;
                                        //int frame_len=(le32_to_cpu(rfd->ActualCount)&0x3fff)-macp->rxOffset;
                                        ZD1211DEBUG(4, "zd1211: invalid frame\n");
                                        //plcp_wla_Header_t *wla_hdrb;
                                        //printk("zd1211: invalid frame\n");
                                        //printk("\nAddr1: ");
                                        //for(i=0;i<6;i++)
                                        //	printk("%02x ",	wla_hdr->Address1[i]);
                                        //printk("\nAddr2: ");
                                        //for(i=0;i<6;i++)
                                        //    printk("%02x ", wla_hdr->Address2[i]);
                                        //printk("\nAddr3: ");
                                        //for(i=0;i<6;i++)
                                        //    printk("%02x ", wla_hdr->Address3[i]);
                                        //printk("\nDuration:%d",*(u16 *)wla_hdr->Duration);
                                        //printk("\nFrmCtl:%d",*(u16 *)wla_hdr->FrameCtrl);
                                        //printk("\nLength:%d\n",frame_len);


                                        macp->invalid_frame_good_crc ++;
                                        zd1205_add_skb_to_end(macp, rx_struct);
                                        continue;
                                }
                        }

                        if (!macp->sniffer_on && (BssType != IW_MODE_MONITOR)) {
                            if ((BaseFrmType == DATA) || (BaseFrmType == MANAGEMENT)) {
                                seq = getSeq(wla_hdr);
                                frag = getFrag(wla_hdr);
                                pTa = getTA(wla_hdr);

                                if (!bGroup(wla_hdr)) { //unicast
                                        if (memcmp(&wla_hdr->Address1[0], &macp->macAdr[0], 6) != 0) {
                                                zd1205_add_skb_to_end(macp, rx_struct);
                                                continue;
                                        } else { //check dupicated frame
                                                //if (BaseFrmType == DATA)
                                                //zd1205_dump_data("RxBuffer", (u8 *)rfd->RxBuffer, data_sz);

                                                if ((bRetryBit(wla_hdr))
                                                                && (zd1205_SearchTupleCache(macp, pTa, seq, frag))) { //dupicated
                                                        zd1205_UpdateTupleCache(macp, pTa, seq, frag);
                                                        zd1205_add_skb_to_end(macp, rx_struct);
                                                        macp->rxDupCnt ++;
                                                        continue;
                                                }
                                                zd1205_UpdateTupleCache(macp, pTa, seq, frag);
                                        }
                                } else { //group address
                                        // check if the address1 of the multicast frame is in the multicast list
                                        if (wla_hdr->Address1[0] != 0xff) {
                                                int tmpvalue = -1;
                                                //zd1205_dump_data("address1", (u8 *)wla_hdr->Address1, 6);

                                                for(i=0; i<macp->MulticastAddr[0]; i++) {
                                                        tmpvalue = memcmp(&macp->MulticastAddr[6*i+1], wla_hdr->Address1, 6);

                                                        if (tmpvalue == 0)
                                                                break;
                                                }

                                                //If the address is not registerd multicast addr.
                                                //and not if promisc || All multicast mode.Drop It.
                                                if ((tmpvalue != 0) &&
                                                                !(dev->flags & IFF_PROMISC) &&
                                                                !(dev->flags & IFF_ALLMULTI)) {

                                                        zd1205_add_skb_to_end(macp, rx_struct);
                                                        ZD1211DEBUG(1, " - address1 is not in the multicast list\n");
                                                        continue;
                                                }
                                        }

                                        if (BaseFrameType(wla_hdr) == DATA) {
                                                if (BssType == INFRASTRUCTURE_BSS) {
                                                        if (memcmp(&macp->BSSID[0], &wla_hdr->Address2[0], 6) != 0) {
                                                                //BSSID filter
                                                                zd1205_add_skb_to_end(macp, rx_struct);
                                                                continue;
                                                        }
                                                } else if ((BssType == INDEPENDENT_BSS) || (BssType == PSEUDO_IBSS)) {
                                                        if (memcmp(&macp->BSSID[0], &wla_hdr->Address3[0], 6) != 0) {
                                                                //BSSID filter
                                                                zd1205_add_skb_to_end(macp, rx_struct);
                                                                continue;
                                                        }
                                                } else {
                                                        zd1205_add_skb_to_end(macp, rx_struct);
                                                        continue;
                                                }
                                        } else {
                                                ;//ZD1211DEBUG(3, "Group Mgt Frame\n");
                                        }
                                }

                                hdrLen = WLAN_HEADER;
                                pBody = (u8 *)pHdr + WLAN_HEADER;
                                bodyLen = data_sz - PLCP_HEADER - WLAN_HEADER - EXTRA_INFO_LEN - CRC32_LEN;

                                //frame with WEP
                                if (bWepBit(wla_hdr)) {
                                        u16 RxIv16 = 0;
                                        u32 RxIv32 = 0;
                                        ZD1211DEBUG(4, "zd1205: wep frame\n");

                                        pIv =  pHdr +  hdrLen;
                                        pBody += IV_SIZE;
                                        bodyLen =  bodyLen - IV_SIZE - ICV_SIZE;
                                        hdrLen += IV_SIZE;
                                        rxDecryType = (macp->rxDecryType & 0x0f);

                                        switch(rxDecryType) {
                                        case WEP64:
                                        case WEP128:
                                        case WEP256:
                                                break;

                                        case TKIP:
                                                bSwCheckMIC=1;// zd1211 always use sw-mic regardless of fragmentation.
                                                bodyLen -= EXTEND_IV_LEN;
                                                pBody += EXTEND_IV_LEN;
                                                hdrLen += EXTEND_IV_LEN;
#if 0 // zd1211 does not have hw-mic feature.

                                                if (macp->rxDecryType & RX_MIC_ERROR_IND) {
                                                        printk("hw mic error!\n");
                                                        if (dot11Obj.MicFailure) {//For hostapd
                                                                //dot11Obj.MicFailure(&wla_hdr->Address2[0]);
                                                        }
                                                        //SDMichaelFailureDetected((UCHAR*) &wla_hdr->Address2[0]);// For hostapd
                                                        //		        hostap_michael_mic_failure((zd1205_private_t *)g_dev->priv, (struct hostap_ieee80211_hdr *)pHdr, (int)pIv[3] & KEYID_MASK); // For Xsupplicant.

                                                }

#endif
                                                RxIv16 = ((u16)pIv[0] << 8) + pIv[2];
                                                RxIv32 = pIv[4] + ((u32)pIv[5] << 8) + ((u32)pIv[6] << 16) + ((u32)pIv[7] << 24);
                                                // check iv sequence
                                                break;

                                        case AES:
                                                ZD1211DEBUG(2, "Got AES frame !!!\n");
                                                bodyLen -= (MIC_LENGTH);
                                                pBody += EXTEND_IV_LEN;
                                                hdrLen += EXTEND_IV_LEN;
                                                break;
                                        default:
                                                break;
                                        }
                                }//end of wep bit
                                if (BssType == AP_BSS) {
                                        memcpy(EthHdr, &wla_hdr->Address3[0], 6); 		// copy DA
                                        memcpy(&EthHdr[6], &wla_hdr->Address2[0], 6);	// copy SA
                                } else if (BssType == INFRASTRUCTURE_BSS) {
                                        memcpy(EthHdr, &wla_hdr->Address1[0], 6); 		// copy DA
                                        memcpy(&EthHdr[6], &wla_hdr->Address3[0], 6);	// copy SA
                                        //If we got a broadcast frame with Add3 is ourself. Drop it.
                                        // This kind of packet is from our broadcast reqest to AP . 					// And AP rebroadcast it .
                                        if( bGroup(wla_hdr) &&
                                                        memcmp(macp->macAdr,wla_hdr->Address3, 6) == 0)  {
                                                zd1205_add_skb_to_end(macp, rx_struct);
                                                continue;

                                        }

                                } else if ((BssType == INDEPENDENT_BSS) || (BssType == PSEUDO_IBSS)) {
                                        memcpy(EthHdr, &wla_hdr->Address1[0], 6); 		// copy DA
                                        memcpy(&EthHdr[6], &wla_hdr->Address2[0], 6);	// copy SA
                                }

                                /* ath_desc: send probe replies in AP mode */
                                if ((BaseFrmType == DATA) || (BaseFrmType == MANAGEMENT && (SubFrameType(wla_hdr) == PROBE_REQ) && (BssType == AP_BSS))) {
                                        bDataFrm = 1;

                                        if (isWDS(wla_hdr)) {
                                                //ZD1211DEBUG(3, "***** WDS or group\n");
                                                zd1205_add_skb_to_end(macp, rx_struct);
                                                continue;
                                        }
                                        if (frag == 0) { //No fragment or first fragment
                                                if (!bMoreFrag(wla_hdr)) { //No more fragment
                                                        //ZD1211DEBUG(2, "***** No Frag\n");
                                                        //if (rxDecryType == TKIP)
                                                        //	    bodyLen -= MIC_LENGTH;
                                                        goto defrag_ind;
                                                } else {	//First fragment
                                                        DFDEBUG("***** First Frag");
                                                        macp->rxNeedFragCnt++;
                                                        i = zd1205_ArFree(macp); //Get a free one

                                                        if (i < 0) {
                                                                zd1205_ArAge(macp, nowT());
                                                                i = zd1205_ArFree(macp);
                                                                if (i < 0) {
                                                                        DFDEBUG("***** ArFree fail");
                                                                        macp->DropFirstFragCnt++;
                                                                        zd1205_add_skb_to_end(macp, rx_struct);
                                                                        continue;
                                                                }
                                                        }

                                                        zd1205_ArUpdate(macp, pTa, seq, frag, i);
                                                        pDefArray->mpdu[i].dataStart = pBody;
                                                        skb->len = bodyLen;
                                                        pDefArray->mpdu[i].buf = (void *)skb; //save skb

#ifndef HOST_IF_USB

                                                        pci_unmap_single(macp->pdev, rx_struct->dma_addr,
                                                                         sizeof (zd1205_RFD_t), PCI_DMA_FROMDEVICE);
#endif

                                                        list_add(&(rx_struct->list_elem), &(macp->rx_struct_pool));
                                                        macp->skb_req++;	/* incr number of requested skbs */
#ifndef HOST_IF_USB

                                                        zd1205_alloc_skbs(macp);	/* and get them */
#else

                                                        tasklet_schedule(&macp->rx_buff_tasklet);
#endif

                                                        rfd_cnt++;
                                                        zd1205_ArAge(macp, nowT());
                                                        continue;
                                                }
                                        }//end of farg == 0
                                        else { //more frag
                                                struct sk_buff *defrag_skb;
                                                u8 *pStart;

                                                i = zd1205_ArSearch(macp, pTa, seq, frag); //Get exist one
                                                if (i < 0) {
                                                        DFDEBUG("***** ArSearch fail");
                                                        macp->ArSearchFailCnt++;
                                                        zd1205_ArAge(macp, nowT());
                                                        zd1205_add_skb_to_end(macp, rx_struct); //discard this one
                                                        continue;
                                                }

                                                defrag_skb = (struct sk_buff *)pDefArray->mpdu[i].buf;
                                                pStart = (u8 *)pDefArray->mpdu[i].dataStart;
                                                pDefArray->mpdu[i].fn = frag;

                                                memcpy((pStart+defrag_skb->len), pBody, bodyLen); //copy to reassamble buffer
                                                defrag_skb->len += bodyLen;

                                                if (!bMoreFrag(wla_hdr)) { //Last fragment
                                                        DFDEBUG("***** Last Frag");
                                                        zd1205_add_skb_to_end(macp, rx_struct);
                                                        pDefArray->mpdu[i].inUse = 0;
                                                        skb = defrag_skb;
                                                        skb->data = (u8 *)pDefArray->mpdu[i].dataStart; //point mac body
                                                        pBody = skb->data;
                                                        bodyLen = skb->len;
                                                        macp->rxCompFragCnt++;
                                                        //if (rxDecryType == TKIP)
                                                        //	    bSwCheckMIC = 1;

                                                        //goto defrag_ind; //bug
                                                        goto defrag_comp;
                                                } else {
                                                        DFDEBUG("***** More Frag");
                                                        zd1205_ArAge(macp, nowT());
                                                        zd1205_add_skb_to_end(macp, rx_struct);
                                                        continue;
                                                }
                                        }
                                }//end of data frame
                                else if (BaseFrameType(wla_hdr) == MANAGEMENT) {
                                        if (SubFrameType(wla_hdr) == BEACON) {
                                                if (BssType == AP_BSS) {
                                                        if (dot11Obj.ConfigFlag & PASSIVE_CHANNEL_SCAN_SET) {
                                                                zd1205_CollectBssInfo(macp, wla_hdr, pBody, bodyLen);
                                                        }
#if defined(OFDM)
                                                        if (pSetting->MacMode != PURE_B_MODE &&	pSetting->MacMode != PURE_A_MODE) {
                                                                if (!dot11Obj.ConfigFlag & ENABLE_PROTECTION_SET) {
                                                                        if (zd1205_CheckOverlapBss(macp, wla_hdr, pBody, bodyLen)) {
                                                                                // ebable protection mode
                                                                                macp->bOLBC++;
                                                                                if (!(dot11Obj.ConfigFlag & ENABLE_PROTECTION_SET) && (macp->bOLBC >= 3)) {
#ifdef HOST_IF_USB
                                                                                        defer_kevent(macp, KEVENT_EN_PROTECTION);
#else

                                                                                        zd_EventNotify(EVENT_ENABLE_PROTECTION, 1, 0, 0);
#endif
                                                                                }
                                                                        }
                                                                }
                                                        }
#endif
                                                        zd1205_add_skb_to_end(macp, rx_struct);
                                                        continue;
                                                } else { //STA mode
                                                        if (dot11Obj.ConfigFlag & ACTIVE_CHANNEL_SCAN_SET) //path through
                                                                goto defrag_ind;
                                                        else {
                                                                if (memcmp(&macp->BSSID[0], &wla_hdr->Address3[0], 6) == 0) { //BSSID filter
                                                                        macp->bcnCnt++;
                                                                        zd1205_CheckBeaconInfo(macp, wla_hdr, pBody, bodyLen);
                                                                        if (macp->bPSMSupported) {
                                                                                ZD1211DEBUG(0, "AP is alive due to Bcn Rcvd\n");
                                                                        }
                                                                        macp->bAPAlive = 1;
                                                                }
#if 0
                                                                else {
                                                                        if ((pSetting->MacMode != PURE_B_MODE) && (BssType == INDEPENDENT_BSS)) {
                                                                                if (!dot11Obj.ConfigFlag & ENABLE_PROTECTION_SET) {
                                                                                        if (zd1205_CheckOverlapBss(macp, wla_hdr, pBody, bodyLen)) {
                                                                                                // ebable protection mode
#ifdef HOST_IF_USB

                                                                                                defer_kevent(macp, KEVENT_EN_PROTECTION);
#else

                                                                                                zd_EventNotify(EVENT_ENABLE_PROTECTION, 1, 0, 0);
#endif

                                                                                        }
                                                                                }
                                                                        }
                                                                }

#endif
                                                                //discard Beacon
                                                                zd1205_add_skb_to_end(macp, rx_struct); /* discard Beacon frames */
                                                                continue;
                                                        }
                                                }
                                        } else {
                                                if (bGroup(wla_hdr)) {
                                                        if ((BssType != AP_BSS) || (BssType != INDEPENDENT_BSS)) {
                                                                zd1205_add_skb_to_end(macp, rx_struct);
                                                                continue;
                                                        }
                                                }
                                        }

                                        goto defrag_ind;
                                } //end of management frame
                        } else if (SubFrameType(wla_hdr) == PS_POLL) {
                                if (BssType==AP_BSS && memcmp(&wla_hdr->Address1[0], &macp->macAdr[0], 6) == 0) //Ps-Poll for me
                                        zd_CmdProcess(CMD_PS_POLL, (void *)pHdr, 0);
                                zd1205_add_skb_to_end(macp, rx_struct);
                                continue;
                        } else {
                                zd1205_add_skb_to_end(macp, rx_struct);
                                continue;
                        }
                }//end of not sniffer_on and not monitor mode


defrag_ind:
                macp->rxCnt++;
#ifndef HOST_IF_USB

                pci_unmap_single(macp->pdev, rx_struct->dma_addr,
                                 sizeof (zd1205_RFD_t), PCI_DMA_FROMDEVICE);

#endif

                list_add(&(rx_struct->list_elem), &(macp->rx_struct_pool));
                /* end of dma access to rfd */
                macp->skb_req++;	/* incr number of requested skbs */
#ifndef HOST_IF_USB

                zd1205_alloc_skbs(macp);	/* and get them */

#else

                tasklet_schedule(&macp->rx_buff_tasklet);
#endif

defrag_comp:
                macp->rxInfo.bSwCheckMIC = bSwCheckMIC;

                rfd_cnt++;
                if ((!macp->sniffer_on) && (BssType != IW_MODE_MONITOR)) {
                        if (BaseFrmType == DATA)
                                macp->TotalRxDataFrmBytes += (hdrLen+bodyLen);

                        macp->rxInfo.bDataFrm = BaseFrmType;
                        macp->rxInfo.SaIndex = SaIndex;

                        if ((BssType == INFRASTRUCTURE_BSS) && (macp->bAssoc)) {
                                if (memcmp(&macp->BSSID[0], &wla_hdr->Address2[0], 6) == 0) {

                                        macp->bAPAlive = 1;

                                        if ((macp->bPSMSupported) && (macp->PwrState) && (!dot11Obj.bDeviceInSleep)) {
                                                if (bMoreData(wla_hdr)) {
                                                        // More date in AP
                                                        zd_EventNotify(EVENT_MORE_DATA, 0, 0, 0);
                                                }
                                        }
                                }
                        }

                        zd_ReceivePkt(pHdr, hdrLen, pBody, bodyLen, (void *)skb, EthHdr, &macp->rxInfo);
                        macp->bDataTrafficLight = 1;
                } else {
                        skb->tail = skb->data = pHdr;
                        /* ath_desc: fix monitor mode frame length */
                        skb_put(skb, data_sz - PLCP_HEADER - EXTRA_INFO_LEN - CRC32_LEN);
                        skb_reset_mac_header(skb);
                        skb->pkt_type = PACKET_OTHERHOST;
                        skb->protocol = __constant_htons(ETH_P_802_2);
                        skb->dev = dev;
                        skb->ip_summed = CHECKSUM_NONE;
                        memset(skb->cb, 0, sizeof(skb->cb));
                        netif_rx(skb);
                }
        }/* end of rfd loop */

#ifdef HOST_IF_USB
    #if !fMERGE_RX_FRAME
        if (!RunOnce)
                zd1211_submit_rx_urb(macp);
#endif

        if (dot11Obj.QueueFlag & TX_QUEUE_SET) {
                macp->txQueSetCnt++;
                //tasklet_schedule(&macp->zd1205_tx_tasklet);
                zd_CleanupTxQ();
        }

#else
        /* restart the RU if it has stopped */
        zd1205_start_ru(macp);
#endif

        ZEXIT(4);
        return rfd_cnt;
}

/**
 * zd1205_intr - interrupt handler
 * @irq: the IRQ number
 * @dev_inst: the net_device struct
 * @regs: registers (unused)
 *
 * This routine is the ISR for the zd1205 board. It services
 * the RX & TX queues & starts the RU if it has stopped due
 * to no resources.
 */

#ifndef HOST_IF_USB
void
zd1205_intr(int irq, void *dev_inst, struct pt_regs *regs)
{
        struct net_device *dev;
        struct zd1205_private *macp;
        void *regp;
        u32 intr_status;
        dev = dev_inst;
        macp = dev->priv;
        regp = macp->regp;

        intr_status = zd_readl(InterruptCtrl);

        if (!intr_status)

                return;


        /* disable intr before we ack & after identifying the intr as ours */
        zd1205_disable_int();


        /* the device is closed, don't continue or else bad things may happen. */

        if (!netif_running(dev))
        {
                zd1205_enable_int();
                return;

        }

        if (macp->driver_isolated)
        {
                goto exit;

        }



        {
                /* Then, do Rx as soon as possible */
                if (intr_status & RX_COMPLETE) {
                        zd_writel((intr_status | RX_COMPLETE), InterruptCtrl);
                        macp->drv_stats.rx_intr_pkts += zd1205_rx_isr(macp);
                }

                /* Then, recycle Tx chain/descriptors */
                if (intr_status & TX_COMPLETE) {
                        zd_writel((intr_status | TX_COMPLETE), InterruptCtrl);
                        zd1205_tx_isr(macp);
                        macp->TxStartTime = 0;
                }


                if (intr_status & RETRY_FAIL) {
                        zd_writel((intr_status | RETRY_FAIL), InterruptCtrl);
                        zd1205_retry_failed(macp);
                        macp->TxStartTime = 0;
                }


                if (intr_status & CFG_NEXT_BCN) {
                        zd_writel((intr_status | CFG_NEXT_BCN), InterruptCtrl);

                        if (macp->config_next_bcn_en) {
                                macp->bcnCnt++;
                                zd_EventNotify(EVENT_TBCN, 0, 0, 0);

                                if (macp->cardSetting.BssType == INDEPENDENT_BSS) {
                                        macp->bFrmRxed1 = 0;
                                } else if (macp->cardSetting.BssType == AP_BSS) {
                                        if (macp->dtimCount == 0)

                                                macp->dtimCount = macp->cardSetting.DtimPeriod;
                                        macp->dtimCount--;
                                }
                        }

                }


                if (intr_status & DTIM_NOTIFY) {
                        zd_writel((intr_status | DTIM_NOTIFY), InterruptCtrl);

                        if (macp->dtim_notify_en) {
                                macp->dtimCnt++;
                                zd1205_dtim_notify(macp);
                        }
                }


                if (intr_status & BUS_ABORT) {
                        if (!dot11Obj.bDeviceInSleep)
                                ZD1211DEBUG(0, "******Bus Abort!!!\n");
                        zd_writel(0xff, InterruptCtrl);
                        //zd1205_sleep_reset(macp);
                }


                if (intr_status & WAKE_UP) {
                        //ZD1211DEBUG(1, "******WAKE_UP!!!\n");
                        zd_writel((intr_status | WAKE_UP), InterruptCtrl);

                        if (dot11Obj.bDeviceInSleep) {
                                //++ After wake up, we should ignore all interrupt except for Wakeup Interrupt.
                                //	 This is very important!
                                //	 If, we do not obey this, the following bug might occurs:
                                //
                                //	-------------------------------------------------------------------> time
                                //	^		^			^			^			^			^
                                //	|		|			|			|			|			|
                                //	Sleep	|			Process		Process		GetReturn	|
                                //			Wakeup		WakeupInt	RxComplete	Packet		RxComplet

                                //			&			(ReStart	(NotifyNdis (Rfd1)		(due to
                                //			RxComplete	 Rx(Rfd1))	 Rfd1)		Chain Rfd1	 ReStartRx
                                //												at last of	 in Process
                                //												RfdList		 WakeupInt)
                                //	This problem cause Rx-master stays in Idle
                                //	state, and we did not restart it again!
                                zd_writel(0xff, InterruptCtrl);
                                zd1205_process_wakeup(macp);

                        }
                }
        }

        if (macp->dtimCount == macp->cardSetting.DtimPeriod - 1)
        {
                if (dot11Obj.QueueFlag & AWAKE_QUEUE_SET)
                        tasklet_schedule(&macp->zd1205_ps_tasklet);
        }

        if (dot11Obj.QueueFlag & MGT_QUEUE_SET)
                tasklet_schedule(&macp->zd1205_tasklet);


        if (dot11Obj.QueueFlag & TX_QUEUE_SET)

                tasklet_schedule(&macp->zd1205_tx_tasklet);

exit:


        zd1205_enable_int();
}
#endif

int
zd1205_open(struct net_device *dev)
{
        struct zd1205_private *macp = dev->priv;
        int rc = 0;

        ZENTER(0);

        //This is used before previous up stat only,So we clear it.
        clear_bit(CTX_FLAG_ESSID_WAS_SET, (void*)&macp->flags);
        //read_lock(&(macp->isolate_lock));
        if (macp->driver_isolated)
        {
                rc = -EBUSY;
                goto exit;
        }

        //macp->bUSBDeveiceAttached = 1;

        if ((rc = zd1205_alloc_space(macp)) != 0)
        {
                rc = -ENOMEM;
                goto exit;
        }

        /* setup the tcb pool */
        if (!zd1205_alloc_tcb_pool(macp))
        {
                printk(KERN_ERR "zd1205: failed to zd1205_alloc_tcb_pool\n");
                rc = -ENOMEM;
                goto err_exit;
        }

        zd1205_setup_tcb_pool(macp);

        if (!zd1205_alloc_rfd_pool(macp))
        {
                printk(KERN_ERR "zd1205: failed to zd1205_alloc_rfd_pool\n");
                rc = -ENOMEM;
                goto err_exit;
        }


        mod_timer(&(macp->watchdog_timer), jiffies + (1*HZ));  //1 sec
        mod_timer(&(macp->tm_hking_id), jiffies + (1*HZ)/10);  //100 ms
        mod_timer(&(macp->tm_mgt_id), jiffies + (1*HZ)/50);   //20 ms

#ifndef HOST_IF_USB

        if ((rc = request_irq(dev->irq, &zd1205_intr, SA_SHIRQ, dev->name, dev)) != 0)
        {
                printk(KERN_ERR "zd1205: failed to request_irq\n");
                del_timer_sync(&macp->watchdog_timer);
                del_timer_sync(&macp->tm_hking_id);
                del_timer_sync(&macp->tm_mgt_id);
                goto err_exit;
        }

        zd1205_start_ru(macp);
#endif

        if (macp->cardSetting.BssType == AP_BSS)
        {
                netif_start_queue(dev);
                zd_writel(0x1, LED1);
        }

        zd_UpdateCardSetting(&macp->cardSetting);
        zd_CmdProcess(CMD_ENABLE, &dot11Obj, 0);  //AP start send beacon , STA start scan
        zd1205_enable_int();

        if (zd1211_submit_rx_urb(macp))
                goto err_exit;

        set_bit(ZD1211_RUNNING, &macp->flags);

        if(CustomMACSet)
        {
                zd_writel(cpu_to_le32(*(u32 *)macp->macAdr), MACAddr_P1);
                zd_writel(cpu_to_le32(*(u32 *)(macp->macAdr+4)), MACAddr_P2);
        }

        goto exit;

err_exit:
        zd1205_clear_pools(macp);

exit:
        //read_unlock(&(macp->isolate_lock));
        ZEXIT(0);

        return rc;
}
void zd1205_monitor_mode_enable(struct zd1205_private *macp)
{
        struct net_device *dev = macp->device;

        if (netif_running(dev))
                netif_stop_queue(dev);

        dev->type = ARPHRD_IEEE80211;
        zd_writel(0x01, SnifferOn);
        zd_writel(0xffffffff, Rx_Filter);
        zd_writel(0x08, EncryptionType);
        macp->intrMask = RX_COMPLETE_EN;
}

void zd1205_monitor_mode_disable(struct zd1205_private *macp)
{
        struct net_device *dev = macp->device;

        dev->type = ARPHRD_ETHER;
        zd_writel(0x00, SnifferOn);
        //zd_writel(0x00000000, Rx_Filter); /* will be set for the new mode */
        zd_writel(NO_WEP, EncryptionType);
        macp->intrMask = ZD1205_INT_MASK;

        if (!netif_running(dev))
                netif_start_queue(dev);
}

void zd1205_init_txq(struct zd1205_private *macp, zd1205_SwTcbQ_t *Q)
{
        unsigned long flags;

        spin_lock_irqsave(&macp->q_lock, flags);
        Q->first = NULL;
        Q->last = NULL;



        Q->count = 0;
        spin_unlock_irqrestore(&macp->q_lock, flags);

}



void zd1205_qlast_txq(struct zd1205_private *macp, zd1205_SwTcbQ_t *Q, zd1205_SwTcb_t *signal)
{
        unsigned long flags;
        spin_lock_irqsave(&macp->q_lock, flags);

        signal->next = NULL;
        if (Q->last == NULL)
        {
                Q->first = signal;
                Q->last = signal;
        } else
        {
                Q->last->next = signal;



                Q->last = signal;
        }

        Q->count++;
        spin_unlock_irqrestore(&macp->q_lock, flags);
}





zd1205_SwTcb_t * zd1205_first_txq(struct zd1205_private *macp, zd1205_SwTcbQ_t *Q)
{
        zd1205_SwTcb_t *p = NULL;

        unsigned long flags;

        spin_lock_irqsave(&macp->q_lock, flags);

        if (Q->first != NULL)
        {
                Q->count--;
                p = Q->first;
                Q->first = (Q->first)->next;
                if (Q->first == NULL)
                        Q->last = NULL;

        }



        spin_unlock_irqrestore(&macp->q_lock, flags);

        return p;

}


static void
zd1205_setup_tcb_pool(struct zd1205_private *macp)
{

        /* TCB local variables */
        zd1205_SwTcb_t  *sw_tcb;         /* cached TCB list logical pointers */

        zd1205_HwTCB_t	*hw_tcb;         /* uncached TCB list logical pointers */
        unsigned long	HwTcbPhys;       /* uncached TCB list physical pointer */
        u32      	    TcbCount;

        /* TBD local variables */
        zd1205_TBD_t  		*pHwTbd;         /* uncached TBD list pointers */
        unsigned long		HwTbdPhys;       /* uncached TBD list physical pointer */
        zd1205_Ctrl_Set_t	*pHwCtrlPtr;
        unsigned long		HwCtrlPhys;
        zd1205_Header_t	    *pHwHeaderPtr;
        unsigned long		HwHeaderPhys;

        macp->freeTxQ = &free_txq_buf;
        macp->activeTxQ = &active_txq_buf;
        zd1205_init_txq(macp, macp->freeTxQ);
        zd1205_init_txq(macp, macp->activeTxQ);

#if 0
        /* print some basic sizing debug info */
        printk(KERN_DEBUG "sizeof(SwTcb) = %04x\n", sizeof(zd1205_SwTcb_t));
        printk(KERN_DEBUG "sizeof(HwTcb) = %04x\n", sizeof(zd1205_HwTCB_t));

        printk(KERN_DEBUG "sizeof(HwTbd)= %04x\n", sizeof(zd1205_TBD_t));
        printk(KERN_DEBUG "sizeof(CTRL_STRUC) = %04x\n", sizeof(zd1205_Ctrl_Set_t));
        printk(KERN_DEBUG "sizeof(HEADER_STRUC) = %04x\n", sizeof(zd1205_Header_t));
        printk(KERN_DEBUG "macp->numTcb = %04x\n", macp->numTcb);
        printk(KERN_DEBUG "macp->numTbdPerTcb = %04x\n", macp->numTbdPerTcb);
        printk(KERN_DEBUG "macp->numTbd = %04x\n", macp->numTbd);
#endif


        /* Setup the initial pointers to the HW and SW TCB data space */
        sw_tcb = (zd1205_SwTcb_t *) macp->txCached;
        hw_tcb = (zd1205_HwTCB_t *) macp->txUnCached;



#ifndef HOST_IF_USB

        HwTcbPhys = macp->txUnCachedPhys;
#else

        HwTcbPhys = (unsigned long)macp->txUnCached;
#endif

        /* Setup the initial pointers to the TBD data space.
          TBDs are located immediately following the TCBs */
        pHwTbd = (zd1205_TBD_t *) &hw_tcb[macp->numTcb];
        HwTbdPhys = HwTcbPhys + (sizeof(zd1205_HwTCB_t) * macp->numTcb);

        /* Setup yhe initial pointers to the Control Setting space
        CTRLs are located immediately following the TBDs */
        pHwCtrlPtr = (zd1205_Ctrl_Set_t *) &pHwTbd[macp->numTbd];
        HwCtrlPhys = HwTbdPhys + (sizeof(zd1205_TBD_t) * macp->numTbd);

        /* Setup the initial pointers to the Mac Header space
         MACHEADERs are located immediately following the CTRLs */
        pHwHeaderPtr = (zd1205_Header_t *) &pHwCtrlPtr[macp->numTcb];
        HwHeaderPhys = HwCtrlPhys + (sizeof(zd1205_Ctrl_Set_t) * macp->numTcb);



        /* Go through and set up each TCB */
        for (TcbCount = 0; TcbCount < macp->numTcb;


                        TcbCount++, sw_tcb++, hw_tcb++, HwTcbPhys += sizeof(zd1205_HwTCB_t),
                        pHwTbd = (zd1205_TBD_t *) (((u8 *) pHwTbd) + ((sizeof(zd1205_TBD_t) * macp->numTbdPerTcb))),
                        HwTbdPhys += (sizeof(zd1205_TBD_t) * macp->numTbdPerTcb),
                        pHwCtrlPtr++, HwCtrlPhys += sizeof(zd1205_Ctrl_Set_t),
                        pHwHeaderPtr++, HwHeaderPhys += sizeof(zd1205_Header_t))
        {
                /* point the cached TCB to the logical address of the uncached one */
                sw_tcb->TcbCount = TcbCount;

                sw_tcb->skb = 0;
                sw_tcb->pTcb = hw_tcb;
                sw_tcb->TcbPhys = HwTcbPhys;
                sw_tcb->pFirstTbd = pHwTbd;
                sw_tcb->FirstTbdPhys = HwTbdPhys;
                sw_tcb->pHwCtrlPtr = pHwCtrlPtr;
                sw_tcb->HwCtrlPhys = HwCtrlPhys;

#if 0

                // Pre-init control setting
                {
                        zd1205_Ctrl_Set_t	*ctrl_set = sw_tcb->pHwCtrlPtr;


                        ctrl_set->CtrlSetting[3] = (u8)(sw_tcb->TcbPhys);

                        ctrl_set->CtrlSetting[4] = (u8)(sw_tcb->TcbPhys >> 8);
                        ctrl_set->CtrlSetting[5] = (u8)(sw_tcb->TcbPhys >> 16);
                        ctrl_set->CtrlSetting[6] = (u8)(sw_tcb->TcbPhys >> 24);
                        ctrl_set->CtrlSetting[18] = 0; //default for fragment
                        ctrl_set->CtrlSetting[19] = 0;
                        ctrl_set->CtrlSetting[23] = 0; //default for fragment

                        ctrl_set->CtrlSetting[24] = 0;
                        ctrl_set->CtrlSetting[26] = 0;
                        ctrl_set->CtrlSetting[27] = 0;
                }
#endif

                sw_tcb->pHwHeaderPtr = pHwHeaderPtr;
                sw_tcb->HwHeaderPhys = HwHeaderPhys;



                /* initialize the uncached TCB contents -- status is zeroed */
                hw_tcb->CbStatus = 0xffffffff;
                hw_tcb->CbCommand = cpu_to_le32(CB_S_BIT);
#ifndef __LP64__
                hw_tcb->TxCbFirstTbdAddrLowPart = cpu_to_le32(HwTbdPhys);
                hw_tcb->TxCbFirstTbdAddrHighPart = 0;
#else
                hw_tcb->TxCbFirstTbdAddr = cpu_to_le64(HwTbdPhys);

#endif
                hw_tcb->TxCbTbdNumber = 0;
                if (TcbCount == (macp->numTcb -1)) {
                        /* Turn around TBD */

#ifndef __LP64__
#ifndef HOST_IF_USB
                        hw_tcb->NextCbPhyAddrLowPart =  cpu_to_le32(macp->txUnCachedPhys);
#else


                        hw_tcb->NextCbPhyAddrLowPart =	cpu_to_le32((u32)macp->txUnCached);
#endif

                        hw_tcb->NextCbPhyAddrHighPart = 0;
#else
#ifndef HOST_IF_USB
                        hw_tcb->NextCbPhyAddr = cpu_to_le64(macp->txUnCachedPhys);
#else

                        hw_tcb->NextCbPhyAddr = cpu_to_le64(macp->txUnCached);
#endif

#endif
                } else {
#ifndef __LP64__
                        hw_tcb->NextCbPhyAddrLowPart = cpu_to_le32(HwTcbPhys + sizeof(zd1205_HwTCB_t));
                        hw_tcb->NextCbPhyAddrHighPart = 0;
#else
                        hw_tcb->NextCbPhyAddr = cpu_to_le64(HwTcbPhys + sizeof(zd1205_HwTCB_t));
#endif
                }



                /* add this TCB to the free list */
                zd1205_qlast_txq(macp, macp->freeTxQ, sw_tcb);
        }

        return;
}




/**
 * zd1205_get_stats - get driver statistics

 * @dev: adapter's net_device struct
 *
 * This routine is called when the OS wants the adapter's stats returned.
 * It returns the address of the net_device_stats stucture for the device.
 * If the statistics are currently being updated, then they might be incorrect
 * for a short while. However, since this cannot actually cause damage, no
 * locking is used.
 */

struct net_device_stats *
                        zd1205_get_stats(struct net_device *dev)
{
        struct zd1205_private *macp = dev->priv;

        macp->drv_stats.net_stats.tx_errors =
                macp->drv_stats.net_stats.tx_carrier_errors +
                macp->drv_stats.net_stats.tx_aborted_errors;

	/* ath_desc: more precise statistics */
	macp->drv_stats.net_stats.rx_length_errors =
		macp->ErrZeroLenFrmCnt +
		macp->ErrLongFrmCnt +
		macp->ErrShortFrmCnt;

	macp->drv_stats.net_stats.rx_frame_errors =
		macp->ErrToHostFrmCnt;

	macp->drv_stats.net_stats.rx_crc_errors =
		macp->hwCRC32Cnt +
		macp->hwCRC16Cnt;

	macp->drv_stats.net_stats.rx_over_errors =
		macp->hwRxFIFOOverrun;

	macp->drv_stats.net_stats.rx_fifo_errors =
		macp->hwUnderrunCnt;

        macp->drv_stats.net_stats.rx_errors =
                macp->drv_stats.net_stats.rx_crc_errors +
                macp->drv_stats.net_stats.rx_frame_errors +
		macp->drv_stats.net_stats.rx_over_errors +
		macp->drv_stats.net_stats.rx_fifo_errors +
                macp->drv_stats.net_stats.rx_length_errors +
                macp->drv_stats.rcv_cdt_frames;


        return &(macp->drv_stats.net_stats);
}

/* ath_desc: added iw_get_stats */
/**
 * zd1205wext_iw_get_stats - get driver wireless statistics
 * @dev: adapter's net_device struct
 *
 * This routine is called when the OS wants the adapter's wireless
 * stats returned. It returns the address of the iw_statistics
 * stucture for the device. If the statistics are currently being
 * updated, then they might be incorrect for a short while. However,
 * since this cannot actually cause damage, no locking is used.
 */
#if WIRELESS_EXT > 12
struct iw_statistics *zd1205wext_iw_get_stats(struct net_device *dev)
{
	struct zd1205_private *macp = dev->priv;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
	struct iw_statistics *iw_stats = &macp->drv_stats.iw_stats;
#else
	struct iw_statistics *iw_stats = &macp->device->stats;
#endif

	iw_stats->qual.noise = 0;
	iw_stats->qual.qual = macp->rxSignalQuality;
	iw_stats->qual.level = (macp->rxSignalStrength*100)/65;
	iw_stats->qual.updated = 7;

	iw_stats->discard.nwid = 0;
	iw_stats->discard.code = macp->hwDecrypErr_UNI + macp->hwDecrypErr_Mul;
	iw_stats->discard.fragment = macp->ErrToHostFrmCnt;
	iw_stats->discard.retries = macp->retryFailCnt;
	iw_stats->discard.misc = macp->DropFirstFragCnt + macp->ErrLongFrmCnt +
				 macp->ErrShortFrmCnt + macp->ErrZeroLenFrmCnt;

	iw_stats->miss.beacon = 0;

	return iw_stats;
}
#endif

/**
 * zd1205_set_mac - set the MAC address
 * @dev: adapter's net_device struct

 * @addr: the new address
 *
 * This routine sets the ethernet address of the board


 * Returns:
 * 0  - if successful
 * -1 - otherwise
 */
int

zd1205_set_mac(struct net_device *dev, void *addr)
{
        struct zd1205_private *macp;

        int rc = -1;
        struct sockaddr *p_sockaddr = (struct sockaddr *) addr;


        macp = dev->priv;
        read_lock(&(macp->isolate_lock));

        if (macp->driver_isolated)
        {
                goto exit;
        }

        {
                memcpy(&(dev->dev_addr[0]), p_sockaddr->sa_data, ETH_ALEN);
                zd_writel(cpu_to_le32(*(u32 *)p_sockaddr->sa_data), MACAddr_P1);
                zd_writel(cpu_to_le32(*(u32 *)(p_sockaddr->sa_data+4)), MACAddr_P2);
                memcpy(macp->macAdr,p_sockaddr->sa_data,ETH_ALEN);
                memcpy(CustomMAC,p_sockaddr->sa_data,ETH_ALEN);
                memcpy(macp->cardSetting.MacAddr,macp->macAdr,ETH_ALEN);

                if (macp->cardSetting.BssType == AP_BSS) {
                        // Set bssid = MacAddress
                        memcpy(macp->BSSID,macp->macAdr,ETH_ALEN);

                        zd_writel(cpu_to_le32(*(u32 *)&macp->macAdr[0]), BSSID_P1);
                        zd_writel(cpu_to_le32(*(u32 *)&macp->macAdr[4]), BSSID_P2);
                } else {
                        zd_writel(STA_RX_FILTER, ZD_Rx_Filter);
                }
                CustomMACSet = TRUE;

                rc = 0;
        }

exit:
        read_unlock(&(macp->isolate_lock));
        return rc;

}



void
zd1205_isolate_driver(struct zd1205_private *macp)
{
        write_lock_irq(&(macp->isolate_lock));
        macp->driver_isolated = true;
        write_unlock_irq(&(macp->isolate_lock));
        del_timer_sync(&macp->watchdog_timer);
        del_timer_sync(&macp->tm_hking_id);
        del_timer_sync(&macp->tm_mgt_id);
        del_timer_sync(&macp->tm_scan_id);


        if (netif_running(macp->device))
        {
                netif_carrier_off(macp->device);
                netif_stop_queue(macp->device);
        }
}


int
zd1205_change_mtu(struct net_device *dev, int new_mtu)

{
        if ((new_mtu < 68) || (new_mtu > (ETH_DATA_LEN + VLAN_SIZE)))
                return -EINVAL;

        dev->mtu = new_mtu;

        return 0;
}


int
zd1205_close(struct net_device *dev)
{
        struct zd1205_private *macp = dev->priv;

        ZENTER(0);

        netif_carrier_off(macp->device);


#if 0

        while (dot11Obj.bDeviceInSleep)
        {
                wait_ms(10);
        }
#endif

        zd1205_isolate_driver(macp);

        macp->intrMask = 0;
        macp->bAssoc = 0;
        mCounterMeasureState = 0;
        macp->bGkInstalled = 0;
        mGkInstalled=0;
        macp->cardSetting.DynKeyMode=0;
        mPrivacyInvoked=0;
        dot11Obj.MIC_CNT=FALSE;

        //zd_writel(0x01, Pre_TBTT);


#ifndef HOST_IF_USB

        zd_writel(0x0, BCNInterval);
        zd_writel(0x0, LED1);
        zd_writel(0x0, LED2);
        free_irq(dev->irq, dev);
        zd1205_device_reset(macp);
#else

        if (!test_bit(ZD1211_UNPLUG, &macp->flags))
        {
                iLED_OFF(macp, macp->LinkLEDn);
                zd_writel(0x0, FW_LINK_STATUS);
                //zd1211_disable_net_traffic(macp);
                //zd1205_device_reset(macp);
        }

        clear_bit(ZD1211_RUNNING, &macp->flags);

        //tasklet_kill(&macp->zd1211_rx_tasklet);
        //tasklet_kill(&macp->zd1211_tx_tasklet);
        //tasklet_kill(&macp->rx_buff_tasklet);


        zd1211_unlink_all_urbs(macp);
#endif



        zd1205_clear_pools(macp);
        macp->bPSMSupported = 0;
        dot11Obj.bDeviceInSleep = 0;


        //set FdescBuf unused
        re_initFdescBuf();

        /* set the isolate flag to false, so zd1205_open can be called */
        macp->driver_isolated = false;
        ZEXIT(0);
        return 0;
}




u8 CalNumOfFrag(struct zd1205_private *macp, u32 length)

{

        u8 FragNum = 1;
        u32 pdusize;



        pdusize = macp->cardSetting.FragThreshold;

        if ((length + CRC32_LEN) > pdusize)
        { //Need fragment
                pdusize -= WLAN_HEADER + CRC32_LEN;

                FragNum = ((length - WLAN_HEADER)+ (pdusize-1)) / pdusize;
                if (FragNum == 0)
                        FragNum = 1;
        }

        return FragNum;
}

int
zd1205_xmit_frame(struct sk_buff *skb, struct net_device *dev)
{
        int rc = 0;
        int notify_stop = false;
        struct zd1205_private *macp = dev->priv;

        u16 TypeLen;
        u8 *pHdr = skb->data;

        u32 bodyLen;
        u32 TotalLen;
        u8 *pBody;
        u8 NumOfFrag = 1;
        u8 EtherHdr[12];
        u8 bEapol = 0;
        u8 *pMac = NULL;
        Hash_t *pHash = NULL;
        u8 bGroupAddr = 0;
        card_Setting_t *pSetting = &macp->cardSetting;
        u8 bEthType2 = 0;
        u8 *pSkbData = skb->data;
        u32 SkbLength = skb->len;
        BOOLEAN bHashSearchResult;
        ZENTER(2);

        //zd1205_dump_data("tx packet", (u8 *)skb->data, skb->len);
        if (pHdr[0] & BIT_0)
                bGroupAddr = 1;

        read_lock(&(macp->isolate_lock));
        if (macp->driver_isolated)
        {
                rc = -EBUSY;
                goto exit2;
        }

        if (!spin_trylock(&macp->bd_non_tx_lock))
        {
                notify_stop = true;
                rc = 1;
                goto exit2;
        }

        TypeLen = (((u16) pHdr[12]) << 8) + (pHdr[13]);
        if ((pSetting->BssType == INFRASTRUCTURE_BSS) || (pSetting->BssType == INDEPENDENT_BSS))
        {
                if (dot11Obj.bDeviceInSleep) {
                        //queue to upper layer
                        notify_stop = true;
                        rc = 1;
                        //dev_kfree_skb_irq(skb);
                        //rc = 0;
                        goto exit1;
                }

                if (pSetting->BssType == INFRASTRUCTURE_BSS)
                        pMac = macp->BSSID;
                else if (pSetting->BssType == INDEPENDENT_BSS)
                        pMac = pHdr;

                bHashSearchResult = zd_QueryStaTable(pMac, (void**)&pHash);
                if ((!macp->bAssoc) || ((!bHashSearchResult) && (!bGroupAddr))) {
                        //Not Associated to AP
                        //printk(KERN_ERR "*** Not associated to AP\n");
                        dev_kfree_skb_irq(skb);
                        rc = 0;
                        goto exit1;
                }
                if (macp->cardSetting.WPASupport==1 && pSetting->BssType==INFRASTRUCTURE_BSS) {
                        if (TypeLen != 0x888e && pHash->pkInstalled==0) {
                                printk(KERN_DEBUG "*** Block Non-EAPol packet before key installed:%04x\n", TypeLen);
                                dev_kfree_skb_irq(skb);
                                rc=0;
                                goto exit1;
                        }
                }

        } else if	(pSetting->BssType == AP_BSS)
        {
                if (!bGroupAddr) { //da is unicast
                        if (!zd_QueryStaTable(pHdr,(void**)&pHash)) {
                                dev_kfree_skb_irq(skb);
                                rc = 0;
                                goto exit1;
                        }
                } else {
                        if (mCurrConnUser==0 || ((pSetting->DynKeyMode) && (mGkInstalled == 0))) {
                                dev_kfree_skb_irq(skb);
                                rc = 0;
                                goto exit1;
                        }
                }
        }

        //TypeLen = (((u16) pHdr[12]) << 8) + (pHdr[13]);

        //WPADEBUG("TypeLen: 0x%04x\n", TypeLen);

        if (TypeLen > 1500)
        {	/* Ethernet 2 frame */
                bEthType2 = 1;
                bodyLen = skb->len - 6;
        } else
        {
                bEthType2 = 0;
                bodyLen = TypeLen;
        }

        TotalLen = bodyLen + WLAN_HEADER; //Mac Header(24)
        NumOfFrag = CalNumOfFrag(macp, TotalLen);

        if (macp->freeTxQ->count < (NumOfFrag+1))
        {
                //printk(KERN_DEBUG "********Queue to upper layer************\n");
                macp->txQueToUpCnt++;
                notify_stop = true;
                rc = 1;
                goto exit1;
        }

        memcpy(&EtherHdr[0], pHdr, 12); //save ethernet header


        if (bEthType2)
        {	/* Ethernet 2 frame */
                /* DA(6) SA(6) Type(2) Data....(reserved array) */
                if (TypeLen == IPX) {
                        memcpy(pHdr+6, ZD_SNAP_BRIDGE_TUNNEL, sizeof(ZD_SNAP_BRIDGE_TUNNEL));
                } else if(TypeLen == APPLE_TALK) {
                        memcpy(pHdr+6, zd_Snap_Apple_Type, sizeof(zd_Snap_Apple_Type));
                }
                else
                        memcpy(pHdr+6, (void *)ZD_SNAP_HEADER, sizeof(ZD_SNAP_HEADER));

                if (TypeLen == EAPOL ) {
                        WPADEBUG("Transmit EAPOL packet\n");
                        bEapol = 1;
                }

                skb->len -= 6;  /* Minus DA, SA; Plus 802.2LLC Header */
                bodyLen = skb->len;
                skb->data += 6;
        } else
        {	/* 802.3 frame */
                /* DA(6) SA(6) Len(2) 802.2_LLC(3) 802.2_SNAP(3+2) Data.... */
                skb->len -= 14;
                bodyLen = TypeLen;
                skb->data += 14;
        }

        pBody = skb->data;

#if 0
        //force release for debug only
        dev_kfree_skb_irq(skb);
        rc = 0;
        goto exit1;
#endif

        if (!zd_SendPkt(EtherHdr, pBody, bodyLen, (void *)skb, bEapol, pHash))
        {
                notify_stop = true;
                rc = 1;
                //restore skb data structure
                skb->data = pSkbData;
                skb->len = SkbLength;
                goto exit1;
        }

        macp->drv_stats.net_stats.tx_bytes += skb->len;
        macp->drv_stats.net_stats.tx_packets++;

exit1:
        spin_unlock(&macp->bd_non_tx_lock);

exit2:
        read_unlock(&(macp->isolate_lock));

        if (notify_stop)
        {
                //netif_carrier_off(dev);
                netif_stop_queue(dev);
        }

        ZEXIT(2);
        return rc;
}

void zd1205_sw_release(void)
{
        zd_EventNotify(EVENT_BUF_RELEASE, 0, 0, 0);
}

void zd1205_sleep_reset(struct zd1205_private *macp)
{
        u32 tmpvalue;
        u32 ul_pretbtt;
        u32 ul_BcnItvl;
        u64 TSFTimer;
        u32     ul_Rem;


#ifndef HOST_IF_USB

        unsigned long flags;
#endif

        //return; //for debug only, test SW

        ZD1211DEBUG(1, "Prepare to enter sleep mode\n");
        netif_stop_queue(macp->device);
        netif_carrier_off(macp->device);
        //HW_RadioOnOff(&dot11Obj, 0);

#if 1

        ul_BcnItvl = zd_readl(ZD_BCNInterval) & 0xFFFF;
        HW_UpdatePreTBTT(&dot11Obj, ul_BcnItvl-BEFORE_BEACON);
        ul_pretbtt = zd_readl(ZD_Pre_TBTT);
#else

        ul_BcnItvl = dot11Obj.BeaconInterval;
        if (ul_BcnItvl < (BEFORE_BEACON))
                ul_BcnItvl=100;
        ul_pretbtt = ul_BcnItvl-BEFORE_BEACON;//Adjust Pre-TBTT occur before TBTT */
#endif
        //ZD1211DEBUG(1, "Pre_TBTT = %u\n", ul_pretbtt);


        while(1)
        {
                // Make sure that the time issued sleep-command is not too close to Pre_TBTT.
                // Also make sure that sleep-command is out of Beacon-Tx duration.
                tmpvalue = zd_readl(ZD_TSF_LowPart);
                TSFTimer = tmpvalue;
                tmpvalue = zd_readl(ZD_TSF_HighPart);
                TSFTimer += (((u64)tmpvalue) << 32);
                TSFTimer = TSFTimer >> 10; // in unit of TU
                //printk("TSF(TU) %d \n", TSFTimer);
                //printk("BeaconInterval = %d\n", ul_BcnItvl);
                //printk("TSF mod BeaconInterval = %d\n", (TSFTimer % ul_BcnItvl));

                ul_Rem=do_div(TSFTimer, ul_BcnItvl);
                if ((ul_pretbtt > ul_Rem) || (macp->bSurpriseRemoved)) {
                        //++ Ensure the following is an atomic operation.
                        ZD1211DEBUG(1, "Rem=%u\n",ul_Rem);
                        if ( (((ul_pretbtt - ul_Rem) >= 3) && (ul_Rem > BEACON_TIME) && (!atomic_read(&macp->DoNotSleep))) || (macp->bSurpriseRemoved)) {
                                down(&macp->ps_sem); // for zd1211
#if 0

                                tmpvalue = zd_readl(ZD_PS_Ctrl); //Because readout value is always zero in zd1211, no need perform read operation before modifying.
                                zd_writel((tmpvalue | BIT_0), ZD_PS_Ctrl);
#else

                                zd_writel(BIT_28 | BIT_0, ZD_PS_Ctrl); // Keep Power on of 44MHz Osc while in sleep.
#endif

                                dot11Obj.bDeviceInSleep = 1;
                                up(&macp->ps_sem);
                                macp->sleepCnt++;
                                break;
                        }
                }
                mdelay(1);
        }

        macp->TxStartTime = 0;

}


void update_beacon_interval(struct zd1205_private *macp, int val)
{
        int BcnInterval;
        int ul_PreTBTT;
        int tmpvalue;

        BcnInterval = val;


        /* One thing must be sure that BcnInterval > Pre_TBTT > ATIMWnd >= 0 */
        if(BcnInterval < 5)
        {
                BcnInterval = 5;
        }


        ul_PreTBTT = zd_readl(Pre_TBTT);

        if(ul_PreTBTT < 4)
        {
                ul_PreTBTT = 4;
        }


        if(ul_PreTBTT >= BcnInterval)
        {
                ul_PreTBTT = BcnInterval - 1;
        }

        zd_writel(ul_PreTBTT, Pre_TBTT);

        tmpvalue = zd_readl(BCNInterval);
        tmpvalue &= ~0xffffffff;
        tmpvalue |= BcnInterval;
        zd_writel(tmpvalue, BCNInterval);
}



void zd1205_device_reset(struct zd1205_private *macp)
{
        u32  tmp_value;


        /* Update the value of Beacon Interval and Pre TBTT */
        update_beacon_interval(macp, 0x2);
        zd_writel(0x01, Pre_TBTT);

        tmp_value = zd_readl(PS_Ctrl);
        zd_writel(tmp_value | BIT_0, PS_Ctrl);
        dot11Obj.bDeviceInSleep = 1;
        /* Sleep for 5 msec */
        wait_ms(5);

}



void zd1205_recycle_tx(struct zd1205_private *macp)

{
        zd1205_SwTcb_t *sw_tcb;


#if 0

        while (macp->activeTxQ->count)
        {
                sw_tcb = zd1205_first_txq(macp, macp->activeTxQ);
                zd1205_transmit_cleanup(macp, sw_tcb);
                macp->txCmpCnt++;
                if (!sw_tcb->LastFrag)
                        continue;

                zd_EventNotify(EVENT_TX_COMPLETE, ZD_RETRY_FAILED, (U32)sw_tcb->MsgID, sw_tcb->aid);
        }
#else
        if (macp->activeTxQ->count)
        {
                sw_tcb = macp->activeTxQ->first;
                zd1205_start_download(sw_tcb->TcbPhys);
        }
#endif
}

void zd1205_process_wakeup(struct zd1205_private *macp)
{
        card_Setting_t *pSetting = &macp->cardSetting;

#ifndef HOST_IF_USB

        u32 tmpvalue;
        u64 TSFTimer;
#endif

        ZENTER(1);

        if (pSetting->BssType == AP_BSS)
        {
                HW_EnableBeacon(&dot11Obj, pSetting->BeaconInterval, pSetting->DtimPeriod, AP_BSS);
                HW_SetRfChannel(&dot11Obj, pSetting->Channel, 0,macp->cardSetting.MacMode);
        }
#if 0
        else if (pSetting->BssType == INFRASTRUCTURE_BSS)
        {
                //HW_SetRfChannel(&dot11Obj, dot11Obj.Channel, 0);

                if ((netif_running(macp->device)) && (macp->bAssoc)) {
                        netif_wake_queue(macp->device);   //resume tx
                }
        }
#endif
        printk(KERN_ERR "Bf RF ON: %lu\n",jiffies);
        //HW_RadioOnOff(&dot11Obj, 1);
        printk(KERN_ERR "After RF ON: %lu\n", jiffies);

#ifndef HOST_IF_USB

        tmpvalue = zd_readl(ZD_TSF_LowPart);
        TSFTimer = tmpvalue;
        tmpvalue = zd_readl(ZD_TSF_HighPart);
        TSFTimer += (((u64)tmpvalue) << 32);
        TSFTimer = TSFTimer >> 10; // in unit of TU

        //printk("TSF(TU) %d \n", TSFTimer);
        //printk("BeaconInterval = %d\n", dot11Obj.BeaconInterval);
        //printk("TSF mod BeaconInterval = %d\n", (TSFTimer % dot11Obj.BeaconInterval));
        //printk("Now, Device had been waken up\n");

        tmpvalue = zd_readl(ZD_DeviceState);
        //printk("DeviceState == %x\n", tmpvalue);

        // In IBSS mode, BCNATIM is now operating, therefore, the Tx-State will not
        // stay in idle state. So, we change form 0xffff to 0xff, ie, we just make

        // sure that bus-masters, both Tx and Rx, are in idle-state.
#endif


        dot11Obj.bDeviceInSleep = 0;

        // Solve Sequence number duplication problem after wakeup.
        macp->SequenceNum = 0;

#ifndef HOST_IF_USB

        zd1205_recycle_tx(macp);
#endif

#ifndef HOST_IF_USB

        zd1205_start_ru(macp);
#else
        //zd1205_recycle_rx(macp);
#endif

        macp->wakeupCnt++;

        if ((netif_running(macp->device)) && (macp->bAssoc))
        {
                netif_carrier_on(macp->device);
                netif_wake_queue(macp->device);
        }



}



void zd1205_sw_reset(struct zd1205_private *macp)
{
        zd1205_disable_int();
        zd1205_tx_isr(macp);
        memset(macp->txUnCached, 0x00, macp->txUnCachedSize);

        zd1205_setup_tcb_pool(macp);
        zd1205_sleep_reset(macp);
        zd1205_start_ru(macp);
        zd_EventNotify(EVENT_SW_RESET, 0, 0, 0);
        zd1205_enable_int();

        if(netif_running(macp->device))
        {
                netif_carrier_on(macp->device);
                netif_wake_queue(macp->device);
        }
}



/**
 * zd1205_sw_init - initialize software structs
 * @macp: atapter's private data struct
 *
 * This routine initializes all software structures. Sets up the
 * circular structures for the RFD's & TCB's. Allocates the per board
 * structure for storing adapter information. The CSR is also memory
  * mapped in this routine.
 *
 * Returns :
 *      true: if S/W was successfully initialized

 *      false: otherwise
 */


static unsigned char

zd1205_sw_init(struct zd1205_private *macp)
{

        //ZENTER(0);
        zd1205_init_card_setting(macp);
        zd1205_set_zd_cbs(&dot11Obj);
        zd_CmdProcess(CMD_RESET_80211, &dot11Obj, 0);

        /* Initialize our spinlocks */
        spin_lock_init(&(macp->bd_lock));
        spin_lock_init(&(macp->bd_non_tx_lock));
        //spin_lock_init(&(macp->q_lock));
        spin_lock_init(&(macp->conf_lock));

        tasklet_init(&macp->zd1205_tasklet, zd1205_action, 0);
        tasklet_init(&macp->zd1205_ps_tasklet, zd1205_ps_action, 0);
        tasklet_init(&macp->zd1205_tx_tasklet, zd1205_tx_action, 0);

#ifdef HOST_IF_USB
        //spin_lock_init(&(macp->intr_lock));
        spin_lock_init(&(macp->rx_pool_lock));
        tasklet_init(&macp->zd1211_rx_tasklet, zd1211_rx_isr, (unsigned long)macp);
        tasklet_init(&macp->zd1211_tx_tasklet, zd1211_tx_isr, (unsigned long)macp);
        tasklet_init(&macp->rx_buff_tasklet, zd1211_alloc_rx, (unsigned long)macp);
#endif


        macp->isolate_lock = RW_LOCK_UNLOCKED;

        macp->driver_isolated = false;

        AuthReqState = STE_AUTH_REQ_IDLE;
        AsocState = STE_ASOC_IDLE;

        //ZEXIT(0);
        return 1;

}


/**
 * zd1205_hw_init - initialized tthe hardware
 * @macp: atapter's private data struct


 * @reset_cmd: s/w reset or selective reset
 *

 * This routine performs a reset on the adapter, and configures the adapter.
 * This includes configuring the 82557 LAN controller, validating and setting


 * the node address, detecting and configuring the Phy chip on the adapter,
 * and initializing all of the on chip counters.
 *
 * Returns:
 *      true - If the adapter was initialized
 *      false - If the adapter failed initialization
 */
unsigned char
zd1205_hw_init(struct zd1205_private *macp)

{

        //ZENTER(0);
        HW_ResetPhy(&dot11Obj);
        HW_InitHMAC(&dot11Obj);
        zd1205_config(macp);

        //ZEXIT(0);
        return true;
}



void zd1211_set_multicast(struct zd1205_private *macp)
{
        struct net_device *dev = macp->device;
        struct dev_mc_list *mc_list;
        unsigned int i;

        u8 *pKey;
        u32 tmpValue;
        u8  mcBuffer[192];
        u16 mcLen;


        if (!(dev->flags & IFF_UP))
                return;

        if (macp->cardSetting.BssType == AP_BSS)
                return;


        zd_writel(0, GroupHash_P1);
        zd_writel(0x80000000, GroupHash_P2);
        macp->MulticastAddr[0] = dev->mc_count;
        mcLen = dev->mc_count*ETH_ALEN ;


        for (i = 0, mc_list = dev->mc_list;
                        (i < dev->mc_count) && (i < MAX_MULTICAST_ADDRS);

                        i++, mc_list = mc_list->next)
        {


                //zd1205_dump_data("mc addr", (u8 *)&(mc_list->dmi_addr), ETH_ALEN);
                memcpy(&macp->MulticastAddr[1+i * ETH_ALEN], (u8 *) &(mc_list->dmi_addr), ETH_ALEN);
        }
        macp->MulticastAddr[mcLen +1] = 0;
        //zd1205_dump_data("MulticastAddr", (u8 *)macp->MulticastAddr, mcLen +2);

        memcpy(mcBuffer, &macp->MulticastAddr[1], mcLen);

        //zd1205_dump_data("mcBuffer", (u8 *)mcBuffer, mcLen);
        pKey = mcBuffer;

        for (i=0; i<mcLen; i++)
        {
                if ((i%6) == 5) {
                        *(pKey+i) = (*(pKey+i)) >> 2;
                        if (*(pKey+i) >= 32) {
                                tmpValue = zd_readl(GroupHash_P2);

                                tmpValue |= (0x01 << (*(pKey+i)-32));
                                zd_writel(tmpValue, GroupHash_P2);
                        }

                        else {
                                tmpValue = zd_readl(GroupHash_P1);
                                tmpValue |= (0x01 << (*(pKey+i)));
                                zd_writel(tmpValue, GroupHash_P1);
                        }
                }
        }

        if(dev->flags & IFF_PROMISC)
                printk("Promiscuous mode enabled.\n");

        if(dev->flags & IFF_PROMISC)
        {
                zd_writel(0xffffffff,GroupHash_P1);
                zd_writel(0xffffffff,GroupHash_P2);
        } else if( dev->flags & IFF_ALLMULTI)
        {
                zd_writel(0xffffffff,GroupHash_P1);
                zd_writel(0xffffffff,GroupHash_P2);
        }



        macp->GroupHashP1 = zd_readl(GroupHash_P1);


        macp->GroupHashP2 = zd_readl(GroupHash_P2);

        ZD1211DEBUG(1, "GroupHashP1 = %x\n", macp->GroupHashP1);
        ZD1211DEBUG(1, "GroupHashP2 = %x\n", macp->GroupHashP2);

        //for debug only
        //zd_writel(0xffffffff, GroupHash_P1);

        //zd_writel(0xffffffff, GroupHash_P2);
}


void zd1205_set_multi(struct net_device *dev)
{
        struct zd1205_private *macp = dev->priv;

#ifdef HOST_IF_USB

        defer_kevent(macp, KEVENT_SET_MULTICAST);
#else

        zd1211_set_multicast(macp);
#endif
}



#ifdef HOST_IF_USB
    #define  TX_TIMEOUT     (4*100) //4sec
#else
    #define  TX_TIMEOUT     (4*1000*1000) //4sec
#endif

/**
 * zd1205_watchdog

 * @dev: adapter's net_device struct
 *
 * This routine runs every 1 seconds and updates our statitics and link state,
 * and refreshs txthld value.
 */
void
zd1205_watchdog(struct zd1205_private *macp)
{


        card_Setting_t *pSetting = &macp->cardSetting;
        u32 TxBytes, RxBytes;

#ifndef HOST_IF_USB

        u32 diffTime;
        u32 tmpvalue;
#endif

        //read_lock(&(macp->isolate_lock));

        if (macp->driver_isolated)
        {
                //goto pexit;
                return;
        }

        if (!netif_running(macp->device))
        {

                //goto pexit;
                return;
        }

        rmb();

        macp->CheckForHangLoop++;

        zd_PerSecTimer();

        TxBytes = macp->TotalTxDataFrmBytes;
        RxBytes = macp->TotalRxDataFrmBytes;

        // Check if AP(Access Point) still alive in the current channel

        if (pSetting->BssType == INFRASTRUCTURE_BSS)
        {
                if(macp->bAssoc) { // We thought the Station is still associated with AP.
                        // dump dot11DesiredSsid
                        //U8 cbTemp;
                        //U8 ssidLenToDump=dot11DesiredSsid.buf[1];
                        //for (cbTemp=0; cbTemp<ssidLenToDump; cbTemp++)
                        //   printk("%c", dot11DesiredSsid.buf[2+cbTemp]);
                        //printk("\n");
                        mTmRetryConnect=0;
                        if (!macp->bAPAlive) {  // The AP-exist flag is not set by any received Mgt or Data frame yet, so we increase the lost-Beacon counter.
                                macp->NoBcnDetectedCnt++;
                                if (dot11Obj.bChScanning)
                                        macp->NoBcnDetectedCnt = 0;

                                if (macp->activeTxQ->count > 12)
                                        macp->NoBcnDetectedCnt = 0;

                                if (macp->NoBcnDetectedCnt > 5) {
                                        printk(KERN_ERR "******We Lose AP for 5 seconds\n");
                                        if(!mCounterMeasureState) {
                                                zd1205_dis_connect(macp);
                                                //zd_CmdProcess(CMD_DIS_CONNECT, 0, 0);
                                                if (macp->cardSetting.ap_scan != 1)
                                                        zd_CmdProcess(CMD_ROAMING, 0, 0);
                                                else {
                                                        zdcb_status_notify(STA_DISASSOCIATED, &macp->BSSID[0]);
                                                }
                                        }
                                        macp->NoBcnDetectedCnt = 0;
                                        //defer_kevent(macp, KEVENT_DIS_CONNECT);
                                }
                        } else { // We have received at least one Mgt or Data frame from AP, so we reset the lost-Beacon counter.
                                macp->NoBcnDetectedCnt = 0;
                        }

                        macp->bAPAlive = 0; // Clear AP-exist flag, it will be Set when a Mgt or Data frame is received.
                } else { // macp->bAssoc ==0, we are in disconnected state.

                        //printk(KERN_ERR "***** We are disconnected\n");
                        mTmRetryConnect++;
                        if (mTmRetryConnect >= 6) {
                                if (macp->cardSetting.ap_scan != 1) { //when wpa_supplicant takes care of scanning and AP selection, it is not necessary for driver attempt to reconnect.
                                        if (zd_CmdProcess(CMD_ROAMING,0,0)) {
                                                mTmRetryConnect=0;
                                        }
                                } else {// ap_scan=1 in wpa_supplicant.conf
                                        ZD1211DEBUG(0, "wpa_supplicant takes care of scanning and AP selection, no need to roam driver itself\n");
                                        mTmRetryConnect = 0;
                                }

                        }
                }
        } else if (pSetting->BssType == AP_BSS)
        {
                if((pSetting->MacMode==MIXED_MODE || pSetting->MacMode==PURE_G_MODE)) {// Try to disable protection mechanism if OLBC not exist any more.
                        if (dot11Obj.ConfigFlag & ENABLE_PROTECTION_SET) {
                                if (macp->bOLBC==0) // The bOLBC will be incremented after checking OLBC by calling zd1205_CheckOverlapBss.
                                {
                                        if(++macp->nOLBC_CounterInSec > 2)
                                        {
                                                if (mNumBOnlySta==0) {
                                                        zd_EventNotify(EVENT_ENABLE_PROTECTION, 0,0,0);//Disable protection mode.
                                                }
                                                macp->nOLBC_CounterInSec=0;
                                        }
                                }
                                else {// OLBC condition exist.
                                        macp->nOLBC_CounterInSec=0;
                                }
                        }
                        macp->bOLBC=0;
                }
        }// End of AP_BSS condition

        if ((macp->bPSMSupported) && (macp->bAssoc))
        {

                // Check if we need to enter the PSM (power-save mode), CAM mode or no-change

                if ((TxBytes < macp->PSThreshhold) && (RxBytes < macp->PSThreshhold)) {
                        macp->SuggestionMode = PS_PSM;
                }
                else
                        macp->SuggestionMode = PS_CAM;
        }

        macp->TotalTxDataFrmBytes = 0;
        macp->TotalRxDataFrmBytes = 0;


}


void
zd1205_watchdog_cb(struct net_device *dev)
{
        struct zd1205_private *macp = dev->priv;

#ifdef HOST_IF_USB

        defer_kevent(macp, KEVENT_WATCH_DOG);
        mod_timer(&(macp->watchdog_timer), jiffies+(1*HZ));
#else

        zd1205_watchdog(macp);
        mod_timer(&(macp->watchdog_timer), jiffies+(1*HZ));
#endif
}


/**
 * zd1205_pci_setup - setup the adapter's PCI information

 * @pcid: adapter's pci_dev struct
 * @macp: atapter's private data struct

 *
 * This routine sets up all PCI information for the adapter. It enables the bus
 * master bit (some BIOS don't do this), requests memory ans I/O regions, and
 * calls ioremap() on the adapter's memory region.

 *
 * Returns:

 *      true: if successfull
 *      false: otherwise
 */
#ifndef HOST_IF_USB
static unsigned char
zd1205_pci_setup(struct pci_dev *pcid, struct zd1205_private *macp)

{
        struct net_device *dev = macp->device;


        int rc = 0;

        ZENTER(0);
        if ((rc = pci_enable_device(pcid)) != 0)
        {
                goto err;
        }

        if (!pci_set_dma_mask(pcid, 0xffffffffffffffff))
        {
                macp->using_dac = 1;
                printk(KERN_DEBUG "zd1205: support 64-bit DMA.\n");
        }



        else if (!pci_set_dma_mask(pcid, 0xffffffff))
        {

                macp->using_dac = 0;
                printk(KERN_DEBUG "zd1205: support 32-bit DMA.\n");

        } else
        {


                printk(KERN_ERR "zd1205: No suitable DMA available.\n");
                goto err;
        }


        /* dev and ven ID have already been checked so it is our device */
        pci_read_config_byte(pcid, PCI_REVISION_ID, (u8 *) &(macp->rev_id));

        /* address #0 is a memory region */

        dev->mem_start = pci_resource_start(pcid, 0);
        dev->mem_end = dev->mem_start + ZD1205_REGS_SIZE;

        /* address #1 is a IO region */
        dev->base_addr = pci_resource_start(pcid, 1);
        if ((rc = pci_request_regions(pcid, zd1205_short_driver_name)) != 0)
        {
                goto err_disable;
        }

        pci_enable_wake(pcid, 0, 0);

        /* if Bus Mastering is off, turn it on! */
        pci_set_master(pcid);

        /* address #0 is a memory mapping */
        macp->regp = (void *)ioremap_nocache(dev->mem_start, ZD1205_REGS_SIZE);
        dot11Obj.reg = macp->regp;
        //printk(KERN_DEBUG "zd1205: dot11Obj.reg = %x\n", (u32)dot11Obj.reg);


        if (!macp->regp)
        {
                printk(KERN_ERR "zd1205: %s: Failed to map PCI address 0x%lX\n",
                       dev->name, pci_resource_start(pcid, 0));
                rc = -ENOMEM;
                goto err_region;
        } else
                printk(KERN_DEBUG "zd1205: mapping base addr = %x\n", (u32)macp->regp);

        ZEXIT(0);
        return 0;

err_region:
        pci_release_regions(pcid);


err_disable:
        pci_disable_device(pcid);

err:

        return rc;
}
#endif




/**
 * zd1205_alloc_space - allocate private driver data
 * @macp: atapter's private data struct
 *

 * This routine allocates memory for the driver. Memory allocated is for the
 * selftest and statistics structures.
 *
 * Returns:
 *      0: if the operation was successful
 *      %-ENOMEM: if memory allocation failed
 */




unsigned char
zd1205_alloc_space(struct zd1205_private *macp)
{
        /* deal with Tx cached memory */
        macp->txCachedSize = (macp->numTcb * sizeof(zd1205_SwTcb_t));
        macp->txCached = kmalloc(macp->txCachedSize, GFP_ATOMIC);

        if (!macp->txCached)
        {
                printk(KERN_ERR "zd1205: kmalloc txCached failed\n");
                return 1;


        }
        else
        {
                memset(macp->txCached, 0, macp->txCachedSize);
                return 0;
        }

}



static void
zd1205_dealloc_space(struct zd1205_private *macp)
{
        if (macp->txCached)
                kfree(macp->txCached);


}


/* Read the permanent ethernet address from the eprom. */
void
zd1205_rd_eaddr(struct zd1205_private *macp)
{
        u32 tmpValue;


        //ZENTER(0);
        //The MAC is set by User. We don't load it from EEPROM.

        tmpValue = zd_readl(E2P_MACADDR_P1);
        ZD1211DEBUG(1, "E2P_MACADDR_P1 = %08x\n", tmpValue);




        macp->device->dev_addr[0] =	macp->macAdr[0] = (u8)tmpValue;//0x00;
        macp->device->dev_addr[1] =	macp->macAdr[1] = (u8)(tmpValue >> 8);//0xA0;
        macp->device->dev_addr[2] =	macp->macAdr[2] = (u8)(tmpValue >> 16);//0xC5;
        macp->device->dev_addr[3] =	macp->macAdr[3] = (u8)(tmpValue >> 24);//0x11;
        tmpValue = zd_readl(E2P_MACADDR_P2);
        ZD1211DEBUG(1, "E2P_MACADDR_P2 = %08x\n", tmpValue);
        macp->device->dev_addr[4] =	macp->macAdr[4] = (u8)tmpValue;//0x22;
        macp->device->dev_addr[5] =	macp->macAdr[5] = (u8)(tmpValue >> 8);//0x33;

        ZD1211DEBUG(0, "MAC address = %02x:%02x:%02x:%02x:%02x:%02x\n",
                    macp->device->dev_addr[0], macp->device->dev_addr[1], macp->device->dev_addr[2],
                    macp->device->dev_addr[3], macp->device->dev_addr[4], macp->device->dev_addr[5]);


        macp->cardSetting.MacAddr[0] = macp->macAdr[0];

        macp->cardSetting.MacAddr[1] = macp->macAdr[1];

        macp->cardSetting.MacAddr[2] = macp->macAdr[2];
        macp->cardSetting.MacAddr[3] = macp->macAdr[3];

        macp->cardSetting.MacAddr[4] = macp->macAdr[4];
        macp->cardSetting.MacAddr[5] = macp->macAdr[5];
        //ZEXIT(0);
}

void
zd1205_lock(struct zd1205_private *macp)
{
#ifndef HOST_IF_USB
        spin_lock_bh(&macp->conf_lock);
#else

        spin_lock(&macp->conf_lock);
#endif
}

void
zd1205_unlock(struct zd1205_private *macp)
{
#ifndef HOST_IF_USB
        spin_unlock_bh(&macp->conf_lock);
#else

        spin_unlock(&macp->conf_lock);
#endif
}

//wireless extension helper functions
/* taken from orinoco.c ;-) */
const u32 channel_frequency[] =
        {
                2412, 2417, 2422, 2427, 2432, 2437, 2442,
                2447, 2452, 2457, 2462, 2467, 2472, 2484

        };
const u32 channel_frequency_11A[] =
        {
                //Even element for Channel Number, Odd for Frequency
                36,5180,
                40,5200,
                44,5220,
                48,5240,
                52,5260,
                56,5280,
                60,5300,
                64,5320,
                100,5500,
                104,5520,
                108,5540,
                112,5560,
                116,5580,
                120,5600,
                124,5620,
                128,5640,
                132,5660,
                136,5680,
                140,5700,
                //
                184,4920,
                188,4940,
                192,4960,
                196,4980,
                8,5040,
                12,5060,
                16,5080,
                34,5170,
                38,5190,
                42,5210,
                46,5230,
                //
                149,5745,
                153,5765,
                157,5785,
                161,5805,
                165,5825
                //
        };




#define NUM_CHANNELS ( sizeof(channel_frequency) / sizeof(channel_frequency[0]) )
#define NUM_CHANNELS_11A ( (sizeof(channel_frequency_11A)/2) / sizeof(u32))

#define MAX_KEY_SIZE    13
//Find the Channel Frequency in channel_frequency_11A
static u32 channel_11A_to_Freq(const u32 channel)
{
        u32 i;

        for(i=0;i<NUM_CHANNELS_11A;i++) {
                if(channel == channel_frequency_11A[i*2])
                        return channel_frequency_11A[i*2 + 1];
        }
        printk("\n\nWarnning channel_11A_to_Freq fail(CH:%d)\n\n",channel);
        printk("\n\nZero Return\n\n");
        return 0;
}
#if 0
static u32 Freq_11A_to_channel(const u32 freq)
{
        u32 i;

        for(i=0;i<NUM_CHANNELS_11A;i++) {
                if(freq == channel_frequency_11A[i*2 + 1])
                        return channel_frequency_11A[i*2];
        }
        printk("\n\nWarnning Freq_11A_to_channel fail(CH:%d)\n\n",freq);
        printk("\n\nZero Return\n\n");
        return 0;

}
#endif
#define MIN_KEY_SIZE    5

static int
zd1205_ioctl_setiwencode(struct net_device *dev, struct iw_point *erq, char *key)
{
        //BOOLEAN bReconnect=FALSE;
        struct zd1205_private *macp = dev->priv;
        card_Setting_t *pSetting = &macp->cardSetting;

        if (erq->length > 0)
        {
                int index = (erq->flags & IW_ENCODE_INDEX) - 1;
                int current_index =  pSetting->EncryKeyId;

                //	ZD1211DEBUG(1, "index = %d\n", index);
                //	ZD1211DEBUG(1, "erq->length = %d\n", erq->length);

                if (erq->length > MAX_KEY_SIZE)
                        return -EINVAL;

                if ((index < 0) || (index >= 4))
                        index = current_index;

                /* Set the length */
                if (erq->length > MIN_KEY_SIZE) {
                        pSetting->WepKeyLen = MAX_KEY_SIZE;
                        pSetting->EncryMode = WEP128;
                } else {
                        //	if (erq->length > 0){
                        pSetting->WepKeyLen = MIN_KEY_SIZE;
                        pSetting->EncryMode = WEP64;
                        //	}
                        //	else {
                        //		pSetting->WepKeyLen = 0;   /* Disable the key */
                        //		pSetting->EncryMode = NO_WEP;
                        //	}
                }

                /* Check if the key is not marked as invalid */
                if (!(erq->flags & IW_ENCODE_NOKEY)) {  // for command: key xxxxxxxxxx [n]
                        //		ZD1211DEBUG(0, "Set contents of key %d\n", index+1);
                        pSetting->EncryKeyId = index;
                        memcpy(&pSetting->keyVector[index][0], key, pSetting->WepKeyLen);
                        zd1205_config_wep_keys(macp);
                } else { // For command: key on
                        //			ZD1211DEBUG(0, "key %d is enabled\n", index+1);
                }

                /* WE specify that if a valid key is set, encryption
                 * should be enabled (user may turn it off later)
                 * This is also how "iwconfig ethX key on" works */
                /*if ((index == current_index) && (pSetting->WepKeyLen > 0) &&
			(pSetting->EncryOnOff == 0)) {
			pSetting->EncryOnOff = 1;
                } */
                pSetting->EncryOnOff=1;
        } else if(erq->flags & IW_ENCODE_DISABLED)
        {       // for command: key off
                //	ZD1211DEBUG(0, "Disable Encryption\n");
                pSetting->EncryOnOff=0;
        } else
        {
                /* Do we want to just set the transmit key index ? */
                // For command: (erq->length==0)
                //              key on (If no key ever set)
                //              key [n] , change current active key
                int index = (erq->flags & IW_ENCODE_INDEX) - 1;
                //ZD1211DEBUG(0, "change key %d as active key\n", index+1);
                if ((index >= 0) && (index < 4)) {
                        //			ZD1211DEBUG(0, "Active key id=%d\n", index+1);
                        pSetting->EncryKeyId = index; // Because pSetting->WepKeyLen has been set, it is not necessary to set it again!
                        pSetting->EncryOnOff = 1;
                } else	/* Don't complain if only change the mode */
                {
                        if(!(erq->flags & IW_ENCODE_MODE)) {
                                //				ZD1211DEBUG(0, "change mode for invalid key id:%d\n",index+1);
                                return -EINVAL;
                        }
                }
        }
        if(erq->flags & IW_ENCODE_RESTRICTED)
        {
                pSetting->EncryOnOff = 1;
        }

        if(erq->flags & IW_ENCODE_OPEN)
        {
                pSetting->EncryOnOff = 1;	// Only Wep
        }

        //	ZD1211DEBUG(0,"pSetting->EncryOnOff: %d\n", pSetting->EncryOnOff);
        if (mPrivacyInvoked == pSetting->EncryOnOff)
        { // Privacy setting is the same as before one, No need do reconnect, just update some global parameters.

                mKeyFormat = pSetting->EncryMode;
                mKeyId = pSetting->EncryKeyId;
                mPrivacyInvoked = pSetting->EncryOnOff;
                if (mPrivacyInvoked)
                        mCap |= CAP_PRIVACY;
                else
                        mCap &= ~CAP_PRIVACY;
                memcpy(&mKeyVector[0][0], &pSetting->keyVector[0][0],sizeof(mKeyVector));
                mWepKeyLen = pSetting->WepKeyLen;
                printk(KERN_DEBUG "Just Update WEP key\n");
                return 0;
        }
        printk(KERN_DEBUG "Update CardSetting\n");


#ifdef HOST_IF_USB

        defer_kevent(macp, KEVENT_UPDATE_SETTING);
#else

        zd_UpdateCardSetting(pSetting);
#endif

        return 0;
}



static int
zd1205_ioctl_getiwencode(struct net_device *dev, struct iw_point *erq, char *key)


{
        struct zd1205_private *macp = dev->priv;
        card_Setting_t *pSetting = &macp->cardSetting;



        int index = (erq->flags & IW_ENCODE_INDEX) - 1;

        zd1205_lock(macp);
        if (pSetting->EncryOnOff)
        {
                erq->flags = IW_ENCODE_OPEN;
        }
        else
        {
                erq->flags = IW_ENCODE_DISABLED;
        }

        /* We can't return the key, so set the proper flag and return zero */
        erq->flags |= IW_ENCODE_NOKEY;
        memset(key, 0, 16);

        /* Which key do we want ? -1 -> tx index */

        if((index < 0) || (index >= 4))
                index = pSetting->EncryKeyId;


        erq->flags |= index + 1;
        /* Copy the key to the user buffer */

        erq->length = pSetting->WepKeyLen;
        if (erq->length > 16)
        {
                erq->length = 0;


        }
        zd1205_unlock(macp);


        return 0;
}

static int
zd1205_ioctl_setessid(struct net_device *dev, struct iw_point *erq)
{
        struct zd1205_private *macp = dev->priv;
        char essidbuf[IW_ESSID_MAX_SIZE+1];


        memset(&essidbuf, 0, sizeof(essidbuf));


        if (erq->flags)
        {
                if (erq->length > (IW_ESSID_MAX_SIZE+1))
                        return -E2BIG;

                if (copy_from_user(&essidbuf, erq->pointer, erq->length))
                        return -EFAULT;
        }

        zd1205_lock(macp);

        //essidbuf[erq->length] = '\0';
        memcpy(&macp->cardSetting.Info_SSID[2], essidbuf, erq->length);
        macp->cardSetting.Info_SSID[1] = strlen(essidbuf);

        //memcpy(&macp->cardSetting.Info_SSID[2], essidbuf, erq->length-1);
        //macp->cardSetting.Info_SSID[1] = erq->length-1;
        zd1205_unlock(macp);

        return 0;
}
static int
zd1205_ioctl_setbssid(struct net_device *dev, struct iwreq *wrq)
{
        //struct zd1205_private *macp = dev->priv;
        memcpy(dot11DesiredBssid, &wrq->u.ap_addr.sa_data, ETH_ALEN);
        //ZD1211DEBUG(0,"set AP BSSID=" MACSTR "\n",MAC2STR(dot11DesiredBssid));
        return 0;

}

static int
zd1205_ioctl_getessid(struct net_device *dev, struct iw_point *erq)
{
        struct zd1205_private *macp = dev->priv;
        char essidbuf[IW_ESSID_MAX_SIZE+1];
        u8 len;

        zd1205_lock(macp);

        if (macp->bAssoc)
        {
                len = dot11Obj.CurrSsid[1];
                memcpy(essidbuf, &dot11Obj.CurrSsid[2], len);
        } else
        {
                len = macp->cardSetting.Info_SSID[1];
                memcpy(essidbuf, &macp->cardSetting.Info_SSID[2], len);
        }

        essidbuf[len] = 0;
        zd1205_unlock(macp);

        erq->flags = 1;
        erq->length = strlen(essidbuf);

        WPADEBUG("zd1205_ioctl_getessid: %s\n", essidbuf);

        //erq->length = strlen(essidbuf) + 1;
        //zd1205_dump_data("essidbuf", (u8 *)essidbuf, erq->length);

        if (erq->pointer)
                if ( copy_to_user(erq->pointer, essidbuf, erq->length) )
                        return -EFAULT;
        return 0;
}

static int
zd1205_ioctl_setfreq(struct net_device *dev, struct iw_freq *frq)
{
        struct zd1205_private *macp = dev->priv;
        int chan = -1;
        int fflag=0; //Found Flag

        if (macp->cardSetting.BssType == INFRASTRUCTURE_BSS)
                return -EINVAL;

        if ( (frq->e == 0) && (frq->m <= 1000) )
        {
                /* Setting by channel number */
                chan = frq->m;
                fflag=1;
        } else
        {
                /* Setting by frequency - search the table */
                int mult = 1;
                int i;

                for (i = 0; i < (6 - frq->e); i++)
                        mult *= 10;

                if(PURE_A_MODE != mMacMode ) {
                        for (i = 0; i < NUM_CHANNELS; i++)
                                if (frq->m == (channel_frequency[i] * mult)) {
                                        chan = i+1;
                                        fflag=1;
                                        break;
                                }
                } else {
                        for (i = 0; i < NUM_CHANNELS_11A; i++)
                                if (frq->m == (channel_frequency_11A[i*2+1] * mult)) {
                                        chan = channel_frequency_11A[i*2];
                                        fflag=1;
                                        break;
                                }
                }

        }

        if(PURE_A_MODE != mMacMode)
        {
                if ( (chan < 1) || (14 < chan) ) {
                        printk("We Can't Found Required Channel in ioctl_setfreq(2.4G)\n");
                        return -EINVAL;
                }
        } else
        {
                if ( (chan < 1) || (0 == fflag) ) {
                        printk("We Can't Found Required Channel in ioctl_setfreq(5G)\n");
                        return -EINVAL;
                }
                if( 0 == channel_11A_to_Freq(chan) ) {
                        printk("The channel isn't exist(%d)\n",chan);
                        return -EINVAL;
                }

        }

        zd1205_lock(macp);
        macp->cardSetting.Channel = chan;
        zd1205_unlock(macp);

        return 0;
}

static int
zd1205_ioctl_setrts(struct net_device *dev, struct iw_param *rrq)
{
        struct zd1205_private *macp = dev->priv;
        int val = rrq->value;

        if (rrq->disabled)
                val = 2347;

        if ( (val < 0) || (val > 2347) )
                return -EINVAL;

        zd1205_lock(macp);

        macp->cardSetting.RTSThreshold = val;
        zd1205_unlock(macp);

        return 0;

}

static int
zd1205_ioctl_setfrag(struct net_device *dev, struct iw_param *frq)
{
        struct zd1205_private *macp = dev->priv;

        int err = 0;

        zd1205_lock(macp);

        if (frq->disabled)
        {
                macp->cardSetting.FragThreshold = 2346;
        } else
        {
                if ( (frq->value < 256) || (frq->value > 2346) ) {
                        err = -EINVAL;
                } else {
                        macp->cardSetting.FragThreshold= frq->value & ~0x1; /* must be even */
                }
        }

        zd1205_unlock(macp);
        return err;
}

static int
zd1205_ioctl_getfrag(struct net_device *dev, struct iw_param *frq)
{
        struct zd1205_private *macp = dev->priv;

        u16 val;

        zd1205_lock(macp);
        val = macp->cardSetting.FragThreshold;
        frq->value = val;
        frq->disabled = (val >= 2346);
        frq->fixed = 1;
        zd1205_unlock(macp)
        ;

        return 0;
}

static int
zd1205_ioctl_setrate(struct net_device *dev, struct iw_param *frq)
{
        return 0;
}

static int
zd1205_ioctl_getrate(struct net_device *dev, struct iw_param *frq)
{
        struct zd1205_private *macp = dev->priv;


        frq->fixed = 0;
        frq->disabled = 0;
        frq->value = 0;

        switch(macp->cardSetting.CurrTxRate)
        {
        case RATE_1M:
                frq->value = 1000000;
                break;

        case RATE_2M:

                frq->value = 2000000;
                break;

        case RATE_5M:
                frq->value = 5500000;
                break;

        case RATE_11M:
                frq->value = 11000000;
                break;

        case RATE_6M:
                frq->value = 6000000;
                break;

        case RATE_9M:
                frq->value = 9000000;
                break;

        case RATE_12M:
                frq->value = 12000000;
                break;

        case RATE_18M:
                frq->value = 18000000;
                break;

        case RATE_24M:
                frq->value = 24000000;
                break;

        case RATE_36M:
                frq->value = 36000000;
                break;

        case RATE_48M:
                frq->value = 48000000;
                break;

        case RATE_54M:
                frq->value = 54000000;
                break;

        default:
                return -EINVAL;
        }

        return 0;
}
#if 0 // not used
static int
zd1205_ioctl_settxpower(struct net_device *dev, struct iw_param *prq)
{
        struct zd1205_private *macp = dev->priv;
        int ret = 0;

#define TX_17dbm        0x00
#define TX_14dbm        0x01
#define TX_11dbm        0x02

        if(prq->value >= TX_17dbm && prq->value <= TX_11dbm)
                macp->cardSetting.TxPowerLevel = prq->value;
        else
                ret = -EINVAL;

        return ret;
}

static int
zd1205_ioctl_gettxpower(struct net_device *dev, struct iw_param *prq)
{
        struct zd1205_private *macp = dev->priv;


#define TX_17dbm        0x00
#define TX_14dbm        0x01
#define TX_11dbm        0x02

        prq->flags = 0;
        prq->disabled = 0;
        prq->fixed = 0;

        switch(macp->cardSetting.TxPowerLevel)
        {
        case TX_17dbm:
                prq->value = 17;
                break;

        case TX_14dbm:
                prq->value = 14;
                break;

        case TX_11dbm:
                prq->value = 11;
                break;

        default:
                return -EINVAL;
        }

        return 0;
}
#endif

static int
zd1205_ioctl_setpower(struct net_device *dev, struct iw_param *frq)
{
        struct zd1205_private *macp = dev->priv;

        int err = 0;

        zd1205_lock(macp);

        if (frq->disabled)
        {
                printk(KERN_ERR "power save disabed\n");
                macp->cardSetting.ATIMWindow = 0x0;
                macp->bPSMSupported = 0;
                macp->PwrState = PS_CAM;
                zd_EventNotify(EVENT_PS_CHANGE, (U8)macp->PwrState, 0, 0);
        } else
        {
                printk(KERN_ERR "power save enabled\n");
                macp->cardSetting.ATIMWindow = 0x5;
                macp->bPSMSupported = 1;
        }

        zd1205_unlock(macp);
        HW_UpdatePreTBTT(&dot11Obj, dot11Obj.BeaconInterval-BEFORE_BEACON);

        return err;
}

static int
zd1205_ioctl_getpower(struct net_device *dev, struct iw_param *frq)
{
        struct zd1205_private *macp = dev->priv;

        zd1205_lock(macp);
        if (macp->bPSMSupported)
                frq->disabled = 0;
        else
                frq->disabled = 1;
        zd1205_unlock(macp);

        return 0;
}

static long
zd1205_hw_get_freq(struct zd1205_private *macp)
{
        u32 freq;
        zd1205_lock(macp);
        if(PURE_A_MODE != mMacMode)
                freq = channel_frequency[dot11Obj.Channel-1] * 100000;
        else if(PURE_A_MODE == mMacMode)
                //for PURE_A_MODE the Channel Number is not required to sub one.
                //Because the channel is get from setting not the order in array
                freq =  channel_11A_to_Freq(dot11Obj.Channel) * 100000;
        zd1205_unlock(macp);
        return freq;

}

static int zd1205_ioctl_setmode(struct net_device *dev, __u32 *mode)
{
        struct zd1205_private *macp = dev->priv;

        zd1205_lock(macp);

        if (macp->cardSetting.BssType == IW_MODE_MONITOR)
                  zd1205_monitor_mode_disable(macp);

        switch(*mode)
        {
        case IW_MODE_ADHOC:
                ZD1211DEBUG(0, "Switch to Ad-Hoc mode\n");
                macp->cardSetting.BssType = INDEPENDENT_BSS;

                if (macp->bDefaultIbssMacMode==0)
                        macp->cardSetting.MacMode=PURE_B_MODE;

                zd_writel(STA_RX_FILTER, Rx_Filter);
                break;

        case IW_MODE_INFRA:
                ZD1211DEBUG(0, "Switch to Infra mode\n");
                macp->cardSetting.BssType = INFRASTRUCTURE_BSS;
                macp->cardSetting.AuthMode = 0;

                if (macp->bDefaultIbssMacMode==0) {
                        macp->cardSetting.MacMode=MIXED_MODE;
                }

                zd_writel(STA_RX_FILTER, Rx_Filter);
                break;


        case IW_MODE_MASTER:
                ZD1211DEBUG(0, "Switch to AP mode\n");
                macp->cardSetting.BssType = AP_BSS;

                /* Set bssid = MacAddress */

                macp->BSSID[0] = macp->macAdr[0];
                macp->BSSID[1] = macp->macAdr[1];
                macp->BSSID[2] = macp->macAdr[2];

                macp->BSSID[3] = macp->macAdr[3];
                macp->BSSID[4] = macp->macAdr[4];
                macp->BSSID[5] = macp->macAdr[5];

                zd_writel(cpu_to_le32(*(u32 *)&macp->macAdr[0]), BSSID_P1);
                zd_writel(cpu_to_le32(*(u32 *)&macp->macAdr[4]), BSSID_P2);
                macp->cardSetting.AuthMode = 2; 	//auto auth
                zd_writel(AP_RX_FILTER, Rx_Filter);
                netif_start_queue(dev);
                break;

        case IW_MODE_MONITOR:
                ZD1211DEBUG(0, "Switch to Monitor mode\n");
                macp->cardSetting.BssType = IW_MODE_MONITOR;
                zd1205_monitor_mode_enable(macp);
                break;

        default:
                ZD1211DEBUG(0, "Switch to PSEUDO_IBSS mode\n");
                macp->cardSetting.BssType = PSEUDO_IBSS;
                zd_writel(STA_RX_FILTER, Rx_Filter);
                break;

        }

        macp->bAssoc = 0;
        if (macp->usb->speed != USB_SPEED_HIGH)
                macp->cardSetting.MacMode = PURE_B_MODE;
        else
        {
#if 0
                if (macp->cardSetting.BssType == INDEPENDENT_BSS)
                        macp->cardSetting.MacMode = PURE_B_MODE;
                else
                        macp->cardSetting.MacMode = MIXED_MODE;
#endif

        }

        zd1205_SetRatesInfo(macp);
        zd1205_unlock(macp);
        return 0;
}


/////////////////////////

static int
zd1205_ioctl_getretry(struct net_device *dev, struct iw_param *prq)
{
        return 0;
}

/* For WIRELESS_EXT > 12 */
static int zd1205wext_giwfreq(struct net_device *dev, struct iw_request_info *info, struct iw_freq *freq, char *extra)
{
        struct zd1205_private *macp;
        if(!netif_running(dev))
                return -EINVAL;

        macp = dev->priv;
        freq->m = zd1205_hw_get_freq(macp);
        freq->e = 1;
        return 0;
}

static int zd1205wext_siwmode(struct net_device *dev, struct iw_request_info *info, __u32 *mode, char *extra)
{
        int err;
        err = zd1205_ioctl_setmode(dev, mode);
        return err;
}


static int zd1205wext_giwmode(struct net_device *dev, struct iw_request_info *info, __u32 *mode, char *extra)
{
        struct zd1205_private *macp = dev->priv;
        u8 BssType = macp->cardSetting.BssType;

        if(!netif_running(dev))
                return -EINVAL;

        zd1205_lock(macp);

        switch(BssType)
        {
        case AP_BSS:
                *mode = IW_MODE_MASTER;
                break;

        case INFRASTRUCTURE_BSS:
                *mode = IW_MODE_INFRA;
                break;

        case INDEPENDENT_BSS:
                *mode = IW_MODE_ADHOC;
                break;

        case IW_MODE_MONITOR:
                *mode = IW_MODE_MONITOR;
                break;

        default:
                *mode = IW_MODE_ADHOC;
                break;
        }

        zd1205_unlock(macp);
        return 0;
}

static int zd1205wext_giwrate(struct net_device *dev, struct iw_request_info *info, struct iw_param *rrq, char *extra)
{
        if(!netif_running(dev))
                return -EINVAL;

        return zd1205_ioctl_getrate(dev, rrq);
}

static int zd1205wext_giwrts(struct net_device *dev, struct iw_request_info *info, struct iw_param *rts, char *extra)
{
        struct zd1205_private *macp;
        macp = dev->priv;

        if(!netif_running(dev))
                return -EINVAL;

        rts->value = macp->cardSetting.RTSThreshold;
        rts->disabled = (rts->value == 2347);
        rts->fixed = 1;

        return 0;
}

static int zd1205wext_giwfrag(struct net_device *dev, struct iw_request_info *info, struct iw_param *frag, char *extra)
{
        if(!netif_running(dev))
                return -EINVAL;

        return zd1205_ioctl_getfrag(dev, frag);
}
#if 0 // not used
static int zd1205wext_giwtxpow(struct net_device *dev, struct iw_request_info *info, struct iw_param *rrq, char *extra)
{
        if(!netif_running(dev))
                return -EINVAL;

        return zd1205_ioctl_gettxpower(dev, rrq);
}

static int zd1205wext_siwtxpow(struct net_device *dev, struct iw_request_info *info, struct iw_param *rrq, char *extra)
{
        return zd1205_ioctl_settxpower(dev, rrq);
}
#endif

static int zd1205wext_giwrange(struct net_device *dev, struct iw_request_info *info, struct iw_point *data, char *extra)
{
        struct iw_range *range = (struct iw_range *) extra;
        int i, val;
        if(!netif_running(dev))
                return -EINVAL;


#if WIRELESS_EXT > 9

        range->txpower_capa = IW_TXPOW_DBM;
        // XXX what about min/max_pmp, min/max_pmt, etc.
#endif

#if WIRELESS_EXT > 10

        range->we_version_compiled = WIRELESS_EXT;
        range->we_version_source = 13;
        range->retry_capa = IW_RETRY_LIMIT;
        range->retry_flags = IW_RETRY_LIMIT;
        range->min_retry = 0;
        range->max_retry = 255;

#endif /* WIRELESS_EXT > 10 */



        /* XXX need to filter against the regulatory domain &| active set */
        val = 0;
        if(PURE_A_MODE != mMacMode )
        {
                for (i = 0; i < NUM_CHANNELS ; i++) {
                        range->freq[val].i = i + 1;
                        range->freq[val].m = channel_frequency[i] * 100000;
                        range->freq[val].e = 1;
                        val++;
                }
        } else if(PURE_A_MODE == mMacMode)
        {
                for (i = 0; i < NUM_CHANNELS_11A && i < 32; i++) {
                        range->freq[val].i = channel_frequency_11A[i*2];
                        ;
                        range->freq[val].m = channel_frequency_11A[i*2+1] * 100000;
                        range->freq[val].e = 1;
                        val++;
                        //For 802.11a, there are too more frequency. We can't return them all
                }

        }


        range->num_frequency = val;

        /* Max of /proc/net/wireless */
        range->max_qual.qual = 100;
        range->max_qual.level = 100;

        range->max_qual.noise = 100;
        range->sensitivity = 3;

        // XXX these need to be nsd-specific!
        range->min_rts = 256;
        range->max_rts = 2346;

        range->min_frag = 256;
        range->max_frag = 2346;
        range->max_encoding_tokens = NUM_WEPKEYS;
        range->num_encoding_sizes = 2;
        range->encoding_size[0] = 5;
        range->encoding_size[1] = 13;

        // XXX what about num_bitrates/throughput?
        range->num_bitrates = 0;

        /* estimated max throughput */
        // XXX need to cap it if we're running at ~2Mbps..
        range->throughput = 5500000;

        return 0;
}

#if WIRELESS_EXT > 12
static int zd1205wext_commit(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	struct zd1205_private *macp;

	macp = dev->priv;
	defer_kevent(macp, KEVENT_UPDATE_SETTING);
	return 0;
}
#endif

#if WIRELESS_EXT > 13
static int zd1205wext_siwscan(struct net_device *dev, struct iw_request_info *info, struct iw_point *data, char *extra)
{
        //    u8 i;
        //  u8 oldMacMode;
        //u32 ul_mac_ps_state;
        //u16 channel;
        //BOOLEAN ProbeWithSsid_bak;
        struct zd1205_private *macp = g_dev->priv;
        u32 wait_cnt = 0;

        if(!netif_running(dev))
                return -EINVAL;

        goto scanning_done;
#if 0
        // If the device is scanning when user issue site survey request, we use the result of it directly.
        if (dot11Obj.bChScanning)
        { // Use the result of driver-driven scan.
                while (dot11Obj.bChScanning) {
                        int cnt=0;
                        cnt++;
                        if(cnt>500) {
                                printk("Locked in waitting bChScanning for 5s. Exit!\n");
                                dot11Obj.bChScanning=FALSE;
                                CurrScanCH=1;
                                return 0;
                        }
                        wait_ms(10);
                }
                goto scanning_done;
        } else
        {
                // Set Scanning flag firstly to prevent device from entering sleeping state again before complete of site survey.
                dot11Obj.bChScanning=1;
                while (dot11Obj.bDeviceInSleep) { // busy wait until the device is awaken.
                        wait_ms(1);
                }
        }

        //***********************************************************************
        // Execute site survey request only bChScanning flag is FALSE.
        { // Execute user's site survey request.
                ProbeWithSsid_bak=mProbeWithSsid;
                mProbeWithSsid=0;  // Send Probe request with broadcast ssid.
                zd_ScanBegin();
                for (channel=1; channel <= 14; channel++)
                {
                        zd_CmdScanReq(channel);//Set RF channel then send ProbeRequest
                        wait_ms(100);
                }
                zd_ScanEnd();
                dot11Obj.bChScanning=0;
                mProbeWithSsid=ProbeWithSsid_bak;
        }
#endif
scanning_done:
        if(!dot11Obj.bChScanning)
        {
                if(1 || mAssoc) {
                        dot11Obj.ConfigFlag |= JUST_CHANNEL_SCAN;
                        zd_CmdProbeReq(0);
                } else
                        ;// This mean there is one just done scanning.
        }
        while(dot11Obj.bChScanning)
        {
                if(wait_cnt++ > 100) {
                        int i;
                        for(i=0;i<10;i++)
                                printk(KERN_ERR "UnStoppable Scanning\n");
                        dot11Obj.bChScanning=0;
                        break;
                }
                wait_ms(50);
        }

        zd1205_notify_scan_done(macp);
        set_bit(CTX_FLAG_ESSID_WAS_SET, (void *)&macp->flags);


        return 0;

}

#if WIRELESS_EXT > 14
/*
 * Encode a WPA or RSN information element as a custom
 * element using the hostap format.
 */
static u_int
encode_ie(void *buf, size_t bufsize,
          const u_int8_t *ie, size_t ielen,
          const char *leader, size_t leader_len)
{
        u8 *p;
        int i;

        if (bufsize < leader_len)
                return 0;
        p = buf;
        memcpy(p, leader, leader_len);
        bufsize -= leader_len;
        p += leader_len;
        for (i = 0; i < ielen && bufsize > 2; i++)
                p += sprintf(p, "%02x", ie[i]);
        return (i == ielen ? p - (u8 *)buf : 0);
}
#endif /* WIRELESS_EXT > 14 */

/*------------------------------------------------------------------*/
/*
 * Translate scan data returned from the card to a card independent
 * format that the Wireless Tools will understand
 */
static char *zd1205_translate_scan(struct net_device *dev,
                                   char *current_ev,
                                   char *end_buf,
                                   bss_info_t *list)
{
        struct iw_event	iwe;		/* Temporary buffer */
        u16	capabilities;

#if WIRELESS_EXT > 14

        char buf[64*2 + 30];
#endif

        char *current_val;	/* For rates */
        U32 Tmp;
        int	i;

        /* First entry *MUST* be the AP MAC address */
        iwe.cmd = SIOCGIWAP;
        iwe.u.ap_addr.sa_family = ARPHRD_ETHER;
        memcpy(iwe.u.ap_addr.sa_data, list->bssid, ETH_ALEN);
        current_ev = iwe_stream_add_event(current_ev, end_buf, &iwe, IW_EV_ADDR_LEN);

        /* Other entries will be displayed in the order we give them */

        /* Add the ESSID */
        iwe.u.data.length = list->ssid[1];
        if(iwe.u.data.length > 32)
                iwe.u.data.length = 32;
        iwe.cmd = SIOCGIWESSID;
        iwe.u.data.flags = 1;
        current_ev = iwe_stream_add_point(current_ev, end_buf, &iwe, &list->ssid[2]);

        /* Add mode */
        iwe.cmd = SIOCGIWMODE;
        capabilities = list->cap;
        if(capabilities & (0x01 | 0x02))
        {
                if(capabilities & 0x01)
                        iwe.u.mode = IW_MODE_MASTER;

                else
                        iwe.u.mode = IW_MODE_ADHOC;
                current_ev = iwe_stream_add_event(current_ev, end_buf, &iwe, IW_EV_UINT_LEN);
        }

        /* Add frequency */
        iwe.cmd = SIOCGIWFREQ;
        iwe.u.freq.m = list->channel;
        if(list->apMode != PURE_A_AP)
                iwe.u.freq.m = channel_frequency[iwe.u.freq.m-1] * 100000;
        else
        {
                iwe.u.freq.m = channel_11A_to_Freq(iwe.u.freq.m) * 100000;
        }
        iwe.u.freq.e = 1;
        current_ev = iwe_stream_add_event(current_ev, end_buf, &iwe, IW_EV_FREQ_LEN);

#if WIRELESS_EXT < 15
        /* Add quality statistics */
        iwe.cmd = IWEVQUAL;

        Tmp = -(100 - list->signalStrength);
        Tmp = Tmp >  -40 ?  -40: Tmp;
        Tmp = Tmp < -105 ? -105: Tmp;
        Tmp = (Tmp + 105)*100/65;
        iwe.u.qual.level = Tmp;
        iwe.u.qual.noise = 0;
        iwe.u.qual.qual = list->signalQuality;
        current_ev = iwe_stream_add_event(current_ev, end_buf, &iwe, IW_EV_QUAL_LEN);
#else
        // Transform Signal quality from level to percentage
        memset(&iwe, 0, sizeof(iwe));
        iwe.cmd = IWEVCUSTOM;
        Tmp = -(100 - list->signalStrength);
        Tmp = Tmp >  -40 ?  -40: Tmp;
        Tmp = Tmp < -105 ? -105: Tmp;
        Tmp = (Tmp + 105)*100/65;

        snprintf(buf, sizeof(buf), "SignalStrength=%lu%%,LinkQuality:%d%%", Tmp,list->signalQuality);
        iwe.u.data.length = strlen(buf);
        current_ev = iwe_stream_add_point(current_ev, end_buf, &iwe, buf);
#endif
        //
        /* Add encryption capability */

        iwe.cmd = SIOCGIWENCODE;
        if(capabilities & 0x10)
                iwe.u.data.flags = IW_ENCODE_ENABLED | IW_ENCODE_NOKEY;
        else
                iwe.u.data.flags = IW_ENCODE_DISABLED;
        iwe.u.data.length = 0;
        current_ev = iwe_stream_add_point(current_ev, end_buf, &iwe, list->ssid);

        /* Rate : stuffing multiple values in a single event require a bit
         * more of magic */
        current_val = current_ev + IW_EV_LCP_LEN;


        iwe.cmd = SIOCGIWRATE;


        /* Those two flags are ignored... */
        iwe.u.bitrate.fixed = iwe.u.bitrate.disabled = 0;

        for(i = 0 ; i < list->supRates[1] ; i++)
        {
                /* Bit rate given in 500 kb/s units (+ 0x80) */
                iwe.u.bitrate.value = ((list->supRates[i+2] & 0x7f) * 500000);
                /* Add new value to event */
                current_val = iwe_stream_add_value(current_ev, current_val, end_buf, &iwe, IW_EV_PARAM_LEN);
        }

        if (list->apMode != PURE_B_AP)
        {
                for (i = 0 ; i < list->extRates[1] ; i++) {
                        /* Bit rate given in 500 kb/s units (+ 0x80) */
                        iwe.u.bitrate.value = ((list->extRates[i+2] & 0x7f) * 500000);
                        /* Add new value to event */
                        current_val = iwe_stream_add_value(current_ev, current_val, end_buf, &iwe, IW_EV_PARAM_LEN);
                }
        }

        /* Check if we added any event */
        if((current_val - current_ev) > IW_EV_LCP_LEN)
                current_ev = current_val;

#if WIRELESS_EXT > 14

#define	IEEE80211_ELEMID_RSN	0x30

        memset(&iwe, 0, sizeof(iwe));
        iwe.cmd = IWEVCUSTOM;
        snprintf(buf, sizeof(buf), "bcn_int=%d", list->beaconInterval);
        iwe.u.data.length = strlen(buf);
        current_ev = iwe_stream_add_point(current_ev, end_buf, &iwe, buf);

        if (list->WPAIe[1] != 0)
        {
                static const char rsn_leader[] = "rsn_ie=";
                static const char wpa_leader[] = "wpa_ie=";

                memset(&iwe, 0, sizeof(iwe));
                iwe.cmd = IWEVCUSTOM;
                if (list->WPAIe[0] == IEEE80211_ELEMID_RSN)
                        iwe.u.data.length = encode_ie(buf, sizeof(buf),
                                                      list->WPAIe, list->WPAIe[1]+2,
                                                      rsn_leader, sizeof(rsn_leader)-1);
                else
                        iwe.u.data.length = encode_ie(buf, sizeof(buf),
                                                      list->WPAIe, list->WPAIe[1]+2,
                                                      wpa_leader, sizeof(wpa_leader)-1);
                if (iwe.u.data.length != 0)
                        current_ev = iwe_stream_add_point(current_ev, end_buf,
                                                          &iwe, buf);
        }
        if (list->RSNIe[1] != 0)
        {
                static const char rsn_leader[] = "rsn_ie=";
                memset(&iwe, 0, sizeof(iwe));
                iwe.cmd = IWEVCUSTOM;
                if (list->RSNIe[0] == IEEE80211_ELEMID_RSN) {
                        iwe.u.data.length = encode_ie(buf, sizeof(buf),
                                                      list->RSNIe, list->RSNIe[1]+2,
                                                      rsn_leader, sizeof(rsn_leader)-1);
                        if (iwe.u.data.length != 0)
                                current_ev = iwe_stream_add_point(current_ev, end_buf,	&iwe, buf);
                }
        }

#endif /* WIRELESS_EXT > 14 */

        /* The other data in the scan result are not really
         * interesting, so for now drop it */
        return current_ev;
}


static int zd1205wext_giwscan(struct net_device *dev, struct iw_request_info *info, struct iw_point *data, char *extra)
{
        struct zd1205_private *macp = dev->priv;
        char *current_ev = extra;
        int i;

        if(!netif_running(dev))
                return -EINVAL;


        //ZENTER(0);

        macp->bss_index = zd_GetBssList(&macp->BSSInfo[0]);
        //ZD1211DEBUG(0, "macp->bss_index = %x\n", macp->bss_index);

        /* Read and parse all entries */
        for (i=0; i<macp->bss_index; i++)
        {
                /* Translate to WE format this entry */
                current_ev = zd1205_translate_scan(dev, current_ev,
                                                   extra + IW_SCAN_MAX_DATA,
                                                   &macp->BSSInfo[i]);

        }

        /* Length of data */
        data->length = (current_ev - extra);
        data->flags = 0;	/* todo */

        return 0;
}

#endif
#ifdef ZDCONF_APDBG
void zd1205_dumpEEPROM(struct zd1205_private *macp)
{
        int i,ret;
        u8 int54,int36,cal54,cal36;

        if(AL7230B_RF == dot11Obj.rfMode)
        {
                printk("802.11a Integration & SetPoint Values:\n");
                printk("-----------------------------------------\n");
        }

        for(i=0;i<dot11A_Channel_Amount && AL7230B_RF == dot11Obj.rfMode ;i++)
        {
                ret = a_OSC_get_cal_int(dot11A_Channel[i], RATE_36M, &int36, &cal36);
                if( 0xff == ret	) {
                        printk("Channel %d doesn't exist in zd1205_dumpEEPROM\n",i);
                }
                ret = a_OSC_get_cal_int(dot11A_Channel[i], RATE_54M, &int54, &cal54);

                printk("Channel:%3d Int:%x Set36:%x Set54:%x\n",
                       dot11A_Channel[i],int36,cal36,cal54);

        }
        printk("\n802.11b/g Integration & SetPoint Values:\n");
        printk("-----------------------------------------\n");

        for(i=1;i<=14;i++)
        {
                printk("Channel:%3d Int:%x Set11:%x Set36:%x Set48:%x Set54:%x\n",
                       i,
                       dot11Obj.IntValue[i-1],
                       macp->EepSetPoint[i-1],
                       macp->SetPointOFDM[0][i-1],
                       macp->SetPointOFDM[1][i-1],macp->SetPointOFDM[2][i-1]);
        }

}
#endif

void zd1205_list_bss(struct zd1205_private *macp)
{
        int i, j;
        U32 Tmp;

        u16 cap;
        bss_info_t *pBssInfo;

        printk("\nSSID          BSSID            CH  Signal  Mode     Basic-Rates  Ext-Rates    b/g AP");
        printk("\n------------------------------------------------------------------------------------");

        for (i=0; i<macp->bss_index; i++)
        {
                pBssInfo = &macp->BSSInfo[i];
                printk("\n");

                for (j=0; j<pBssInfo->ssid[1]; j++) {
                        printk("%c", pBssInfo->ssid[2+j]);
                }

                for (j=pBssInfo->ssid[1]; j<12; j++) {
                        printk(" ");
                }

                printk("%02x:%02x:%02x:%02x:%02x:%02x",
                       pBssInfo->bssid[0], pBssInfo->bssid[1], pBssInfo->bssid[2],
                       pBssInfo->bssid[3], pBssInfo->bssid[4], pBssInfo->bssid[5]);
                printk("  %2d", pBssInfo->channel);
                Tmp = -(100 - pBssInfo->signalStrength);
                Tmp = Tmp >  -40 ?  -40: Tmp;
                Tmp = Tmp < -105 ? -105: Tmp;
                Tmp = (Tmp + 105)*100/65;

                printk("   %2lu %%", Tmp);

                cap = pBssInfo->cap;
                cap &= (0x10 | 0x02 | 0x01);

                switch(cap) {
                case 0x01:

                        printk("   Infra   ");
                        break;

                case 0x02:
                        printk("   Ad_Hoc  ");
                        break;

                case 0x11:
                        printk("   Infra, W");
                        break;

                case 0x12:
                        printk("   Ad_Hoc,W");
                        break;

                default :
                        break;
                }


                printk("  ");

                for (j=0; j<pBssInfo->supRates[1]; j++) {
                        printk(" %x", pBssInfo->supRates[2+j]);
                }

                printk("  ");
                for (j=0; j<pBssInfo->extRates[1]; j++) {
                        printk(" %x", pBssInfo->extRates[2+j]);

                }

                if (pBssInfo->apMode == PURE_B_AP)
                        printk("   B-AP");
                else if (pBssInfo->apMode == PURE_G_AP)
                        printk("   G-AP");
                else if  (pBssInfo->apMode == MIXED_AP)
                        printk("   M-AP");
                else if (pBssInfo->apMode == PURE_A_AP)
                        printk(" A-AP");
                else {
                        VerAssert();
                }
        }
        printk("\n");
}


/* ath_desc: support for unpatched wpasupplicant */
static void zd1211_print_auth_alg_desc(const u32 auth_alg) {
	if (auth_alg & IW_AUTH_ALG_OPEN_SYSTEM)
		printk("  Open system");
	if (auth_alg & IW_AUTH_ALG_SHARED_KEY)
		printk("  Shared key");
	if (auth_alg & IW_AUTH_ALG_LEAP)
		printk("  LEAP");
	printk("\n");
}

static const char * zd1211_print_enabled_disabled(const u32 value) {
	return (value ? "enabled" : "disabled");
}

static void zd1211_print_cipher(const u32 cipher) {
	switch (cipher) {
	case IW_AUTH_CIPHER_NONE:
		printk(" NONE");
		break;
	case IW_AUTH_CIPHER_WEP40:
		printk(" WEP40");
		break;
	case IW_AUTH_CIPHER_TKIP:
		printk(" TKIP");
		break;
	case IW_AUTH_CIPHER_CCMP:
		printk(" CCMP");
		break;
	case IW_AUTH_CIPHER_WEP104:
		printk(" WEP104");
		break;
	default:
		printk(" unsupported cipher '0x%0X8'", cipher);
	}
	printk("\n");
}

static void zd1211_print_key_mgmt(const u32 key_mgmt) {
	switch (key_mgmt) {
	case IW_AUTH_KEY_MGMT_802_1X:
		printk(" IEEE802.1X");
		break;
	case IW_AUTH_KEY_MGMT_PSK:
		printk(" PSK");
		break;
	default:
		printk(" unsupported key mgmt '0x%0X8'", key_mgmt);
	}
	printk("\n");
}

static void zd1211_print_roaming(const u32 roaming) {
	switch (roaming) {
	case IW_AUTH_ROAMING_ENABLE:
		printk(" driver/firmware based");
		break;
	case IW_AUTH_ROAMING_DISABLE:
		printk(" done in user space");
		break;
	default:
		printk(" unsupported roaming strategy '0x%0X8'", roaming);
	}
	printk("\n");
}

static void zd1211_print_wpa_version(const u32 wpa_version) {
	switch (wpa_version) {
	case IW_AUTH_WPA_VERSION_DISABLED:
		printk(" WPA disabled");
		break;
	case IW_AUTH_WPA_VERSION_WPA:
		printk(" WPA");
		break;
	case IW_AUTH_WPA_VERSION_WPA2:
		printk(" WPA2");
		break;
	default:
		printk(" unsupported version '0x%0X8'", wpa_version);
	}
	printk("\n");
}

static int zd1211_set_auth_param(struct net_device *net_dev, const int idx, const u32 value) {
	int result;
	struct zd1205_private *macp;

	macp = net_dev->priv;
	result = -EINVAL;
	switch(idx) {
	case IW_AUTH_WPA_VERSION:
		printk(KERN_INFO "%s: WPA version: ", net_dev->name);
		zd1211_print_wpa_version(value);
		result = 0;
		break;
	case IW_AUTH_CIPHER_PAIRWISE:
		printk(KERN_INFO "%s: pairwise cipher: ", net_dev->name);
		zd1211_print_cipher(value);
		result = 0;
		break;
	case IW_AUTH_CIPHER_GROUP:
		printk(KERN_INFO "%s: group cipher: ", net_dev->name);
		zd1211_print_cipher(value);
		result = 0;
		break;
	case IW_AUTH_KEY_MGMT:
		printk(KERN_INFO "%s: key mgmt: ", net_dev->name);
		zd1211_print_key_mgmt(value);
		result = 0;
		break;
	case IW_AUTH_80211_AUTH_ALG:
		printk(KERN_INFO "%s: Configured authentication algorithms:", net_dev->name);
		zd1211_print_auth_alg_desc(value);
		result = 0;
		break;
	case IW_AUTH_DROP_UNENCRYPTED:
		printk(KERN_INFO "%s: Drop encrypted: %s\n", net_dev->name, zd1211_print_enabled_disabled(value) );
		result = 0;
		break;
	case IW_AUTH_TKIP_COUNTERMEASURES:
		if (value) {
			mCounterMeasureState = dot11Obj.MIC_CNT ? 1 : 0;
			printk(KERN_INFO "%s: TKIP counter measures: %s\n", net_dev->name, zd1211_print_enabled_disabled(mCounterMeasureState) );
		}
		result = 0;
		break;
	case IW_AUTH_WPA_ENABLED:
		if (value) {
			macp->cardSetting.WPASupport = 1;
		} else {
			/* Reset the WPA related variables */
			macp->cardSetting.WPASupport = 0;
			/* Now we only set the length in the WPA IE
			 * field to zero.                         */
			macp->cardSetting.WPAIe[1] = 0;
		}
		printk(KERN_INFO "%s: WPA: %s\n", net_dev->name, zd1211_print_enabled_disabled(value) );
		result = 0;
		break;
	case IW_AUTH_RX_UNENCRYPTED_EAPOL:
		printk(KERN_INFO "%s: Allow Rx unencrypted EAPOL: %s\n", net_dev->name, zd1211_print_enabled_disabled(value) );
		result = 0;
		break;
	case IW_AUTH_ROAMING_CONTROL:
		printk(KERN_INFO "%s: roaming: ", net_dev->name);
		macp->cardSetting.ap_scan=(U8)value;
		zd1211_print_roaming(value);
		result = 0;
		break;
	case IW_AUTH_PRIVACY_INVOKED:
		/* Turn on/off the privacy invoke flag */
		macp->cardSetting.EncryOnOff = value ? 1 : 0;
		printk(KERN_INFO "%s: privacy: %s\n", net_dev->name, zd1211_print_enabled_disabled(value) );
		result = 0;
		break;
	default:
		printk(KERN_INFO "%s(): unsupported function/value pair: %d %u\n", __FUNCTION__, idx, value);
		break;
	}
	return result;
}

static U8 zero_mac[]={0,0,0,0,0,0};
// SIOCSIWENCODEEXT
static int zd1211_set_encode_ext(struct zd1205_private *macp, struct iwreq *wrq) {
	int i_key;
	u8 *key;
	size_t key_len;
	struct iw_encode_ext *ext;
        card_Setting_t *pSetting = &macp->cardSetting;
        u8 CamEncryType=0;
        u8 tmpDynKeyMode;

	ext = wrq->u.encoding.pointer;
	key = ext->key;
	i_key = wrq->u.encoding.flags - 1;
	key_len = ext->key_len;
	switch (ext->alg) {
	case IW_ENCODE_ALG_CCMP:
		CamEncryType = AES;
		//if (idx == 0)
		{// Pairwise key
			mKeyFormat = CamEncryType;
			mDynKeyMode = pSetting->DynKeyMode = DYN_KEY_AES;
		}
		break;
	case IW_ENCODE_ALG_TKIP:
		CamEncryType = TKIP;
		//if (idx == 0)
		{// Pairwise key
			mKeyFormat = CamEncryType;
			mDynKeyMode = pSetting->DynKeyMode = DYN_KEY_TKIP;
		}
		break;
	case IW_ENCODE_ALG_WEP:
		if (key_len == 5) { // WEP 64
			CamEncryType = WEP64;
			tmpDynKeyMode=DYN_KEY_WEP64;
		} else {//keylen=13, WEP 128
			CamEncryType = WEP128;
			tmpDynKeyMode=DYN_KEY_WEP128;
		}
		// For Dynamic WEP key (Non-WPA Radius), the key ID range: 0-3
		// In WPA/RSN mode, the key ID range: 1-3, usually, a broadcast key.
		// For WEP key setting: we set mDynKeyMode and mKeyFormat in following case:
		//   1. For 802.1x dynamically generated WEP key method.
		//   2. For WPA/RSN mode, but key id == 0. (But this is an impossible case)
		// So, only check case 1.
		if (pSetting->WPAIeLen==0) {
			mKeyFormat = CamEncryType;
			mDynKeyMode = pSetting->DynKeyMode = tmpDynKeyMode;
			mPrivacyInvoked=TRUE;
			mCap |= CAP_PRIVACY;
			pSetting->EncryOnOff=1;
		}
		break;
	case IW_ENCODE_ALG_NONE:
	default:
		CamEncryType = NO_WEP;
		//	pSetting->DynKeyMode = 0;
		//      pSetting->EncryMode=0;
		//    mKeyFormat=0;

		zd_SetKeyInfo(ext->addr.sa_data, CamEncryType, key_len, i_key, key);
		if (ext->addr.sa_data[0] & 1)//del group key
		{
			if (pSetting->WPAIeLen==0)
			{//802.1x dynamic WEP
				mDynKeyMode = 0;
				mKeyFormat = 0;
				mPrivacyInvoked=FALSE;
				mCap &= ~CAP_PRIVACY;
				pSetting->EncryOnOff=0;
			}
			mWpaBcKeyLen = mGkInstalled = 0;
		} else {
			if (memcmp(zero_mac, ext->addr.sa_data, 6)==0) {
				mDynKeyMode=0;
				mKeyFormat=0;
				pSetting->DynKeyMode=0;
				pSetting->EncryMode=0;
				mDynKeyMode=0;
			}
		}
		goto exit_zd1211_set_encode_ext;
		break;
	}
	zd_SetKeyInfo(ext->addr.sa_data, CamEncryType, key_len, i_key, key);
exit_zd1211_set_encode_ext:
	return 0;
}

// SIOCSIWGENIE
static int zd1211_set_wpa_ie(struct zd1205_private *macp, struct iwreq *wrq) {
        card_Setting_t *pSetting = &macp->cardSetting;

	/* Copy the WPA IE */
	pSetting->WPAIeLen = wrq->u.data.length;
	memcpy(&pSetting->WPAIe, wrq->u.data.pointer, pSetting->WPAIeLen);
#ifdef HOSTAPD_SUPPORT
	if (pSetting->BssType == AP_BSS) {// Update Beacon FIFO in the next TBTT.
		memcpy(&mWPAIe, pSetting->WPAIe, pSetting->WPAIeLen);
		WPADEBUG("Copy WPA IE into mWPAIe\n");
	}
#endif
	return 0;
}


int zd1211_mlme(struct zd1205_private *macp) {
	int result;
#ifdef HOSTAPD_SUPPORT
	u8 mac_addr[80];
#endif

	result = -ENOTSUPP;
	/* Translate STA's address */
	sprintf(mac_addr, "%02x:%02x:%02x:%02x:%02x:%02x",
		(u8)macp->mlme_req.addr.sa_data[0], (u8)macp->mlme_req.addr.sa_data[1],
		(u8)macp->mlme_req.addr.sa_data[2], (u8)macp->mlme_req.addr.sa_data[3],
		(u8)macp->mlme_req.addr.sa_data[4], (u8)macp->mlme_req.addr.sa_data[5]);
	switch(macp->mlme_req.cmd) {
	case MLME_STA_DEAUTH:
		if (zd_CmdDeauth((MacAddr_t *) macp->mlme_req.addr.sa_data, macp->mlme_req.reason_code) == FALSE) {
			printk(KERN_ERR "Can't deauthencate STA: %s\n", mac_addr);
			result = -EIO;
		} else {
			printk(KERN_INFO "Deauthenticate STA: %s with reason code: %d\n", mac_addr, macp->mlme_req.reason_code);
			result = 0;
		}
		break;
	case MLME_STA_DISASSOC:
		if (zd_CmdDisasoc((MacAddr_t *) macp->mlme_req.addr.sa_data, macp->mlme_req.reason_code) == FALSE) {
			printk(KERN_ERR "Can't disassociate STA: %s\n", mac_addr);
			result = -EIO;
		} else {
			printk(KERN_INFO "Disassociate STA: %s with reason code: %d\n", mac_addr, macp->mlme_req.reason_code);
			result = 0;
		}
		break;
	default:
		printk(KERN_ERR "MLME command: 0x%04x not support\n", macp->mlme_req.cmd);
		break;
	}
	return result;
}


/////////////////////////////////////////
int
zd1205_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
        struct zd1205_private *macp;
        void *regp;
        struct zdap_ioctl zdreq;
        struct iwreq *wrq = (struct iwreq *)ifr;
        int err = 0;
        int changed = 0;

        macp = dev->priv;
        regp = macp->regp;

        /* ath_desc: ifconfig up fix - another try */
        /* ath_old: if(!netif_running(dev)) return -EINVAL; */

        switch (cmd)
        {
	case SIOCSIWMLME:
		//ZD1211DEBUG(0, "%s: SIOCSIWMLME\n", dev->name);
		if (!netif_running(dev))
			return -EINVAL;
		if (copy_from_user(&macp->mlme_req, wrq->u.data.pointer, sizeof(macp->mlme_req))) {
			err = -EFAULT;
		} else {
			zd1205_lock(macp);
#ifdef HOST_IF_USB
			defer_kevent(macp, KEVENT_ZD_MLME_IOCTL);
			err = 0;
#else
			err = zd1211_mlme(macp);
#endif
			zd1205_unlock(macp);
		}
		break;

	case SIOCSIWAUTH:
		//ZD1211DEBUG(0, "%s: SIOCSIWAUTH\n", dev->name);
		if (!netif_running(dev))
			return -EINVAL;
		err = zd1211_set_auth_param(dev, wrq->u.param.flags, wrq->u.param.value);
		break;

	case SIOCSIWENCODEEXT:
		//ZD1211DEBUG(0, "%s: SIOCSIWENCODEEXT\n", dev->name);
		if (!netif_running(dev))
			return -EINVAL;
		err = zd1211_set_encode_ext(macp, wrq);
		break;

	case SIOCSIWGENIE:
		//ZD1211DEBUG(0, "%s: SIOCSIWGENIE\n", dev->name);
		if (!netif_running(dev))
			return -EINVAL;
		err = zd1211_set_wpa_ie(macp, wrq);
		break;

        case SIOCGIWNAME:
                //ZD1211DEBUG(1, "%s: SIOCGIWNAME\n", dev->name);
                //strcpy(wrq->u.name, "IEEE 802.11-DS");
                strcpy(wrq->u.name, "802.11b/g NIC");
                break;

        case SIOCGIWAP:
                ZD1211DEBUG(1, "%s: SIOCGIWAP\n", dev->name);
                wrq->u.ap_addr.sa_family = ARPHRD_ETHER;

                if (macp->cardSetting.BssType == AP_BSS)
                        memcpy(wrq->u.ap_addr.sa_data, macp->macAdr, 6);
                else {
                        if(macp->bAssoc)
                                memcpy(wrq->u.ap_addr.sa_data, macp->BSSID, 6);
                        else
                                memset(wrq->u.ap_addr.sa_data, 0, 6);
                }
                break;

        case SIOCGIWRANGE:
                ZD1211DEBUG(1, "%s: SIOCGIWRANGE\n", dev->name);
                if ( wrq->u.data.pointer != NULL) {
                        struct iw_range range;
                        err = zd1205wext_giwrange(dev, NULL, &wrq->u.data, (char *) &range);

                        /* Push that up to the caller */
                        if (copy_to_user(wrq->u.data.pointer, &range, sizeof(range)))
                                err = -EFAULT;
                }
                break;

        case SIOCSIWMODE:
                ZD1211DEBUG(1, "%s: SIOCSIWMODE\n", dev->name);
                err = zd1205wext_siwmode(dev, NULL, &wrq->u.mode, NULL);

                if (!err)
                        changed = 1;
                break;

        case SIOCGIWMODE:
                ZD1211DEBUG(1, "%s: SIOCGIWMODE\n", dev->name);
                err = zd1205wext_giwmode(dev, NULL, &wrq->u.mode, NULL);
                break;

        case SIOCSIWENCODE: {
                        char keybuf[MAX_KEY_SIZE];
                        ZD1211DEBUG(1, "%s: SIOCSIWENCODE\n", dev->name);

                        if (wrq->u.encoding.pointer) {
                                if (wrq->u.encoding.length > MAX_KEY_SIZE) {
                                        err = -E2BIG;
                                        break;
                                }

                                if (copy_from_user(keybuf, wrq->u.encoding.pointer, wrq->u.encoding.length)) {
                                        err = -EFAULT;
                                        break;
                                }
                        }

                        zd1205_dump_data("keybuf", keybuf, wrq->u.encoding.length);
                        err = zd1205_ioctl_setiwencode(dev, &wrq->u.encoding, keybuf);

                        if (!err)
                                changed = 0;
                }
                break;

        case SIOCGIWENCODE: {
                        char keybuf[MAX_KEY_SIZE];

                        ZD1211DEBUG(1, "%s: SIOCGIWENCODE\n", dev->name);
                        err = zd1205_ioctl_getiwencode(dev, &wrq->u.encoding, keybuf);

                        if (wrq->u.encoding.pointer) {
                                if (copy_to_user(wrq->u.encoding.pointer, keybuf, wrq->u.encoding.length))
                                        err = -EFAULT;
                        }
                }
                break;

        case SIOCSIWESSID:
                ZD1211DEBUG(1, "%s: SIOCSIWESSID\n", dev->name);
                err = zd1205_ioctl_setessid(dev, &wrq->u.essid);
                if (!err && macp->cardSetting.ap_scan != 1)
                        changed = 1;
                break;
        case SIOCSIWAP:
                ZD1211DEBUG(1, "%s: SIOCSIWAP\n", dev->name);
                err = zd1205_ioctl_setbssid(dev, wrq);
                if (!err && macp->cardSetting.ap_scan == 1) {
                        //set_bit(CTX_FLAG_ESSID_WAS_SET,(void*)&macp->flags);
                        changed = 1;
                }

                break;

        case SIOCGIWESSID:
                ZD1211DEBUG(1, "%s: SIOCGIWESSID\n", dev->name);
                err = zd1205_ioctl_getessid(dev, &wrq->u.essid);
                break;

        case SIOCGIWFREQ:
                ZD1211DEBUG(1, "%s: SIOCGIWFREQ\n", dev->name);
                wrq->u.freq.m = zd1205_hw_get_freq(macp);
                wrq->u.freq.e = 1;
                break;

        case SIOCSIWFREQ:
                ZD1211DEBUG(1, "%s: SIOCSIWFREQ\n", dev->name);

                err = zd1205_ioctl_setfreq(dev, &wrq->u.freq);
                if (!err)
                        changed = 1;
                break;

        case SIOCGIWRTS:
                ZD1211DEBUG(1, "%s: SIOCGIWRTS\n", dev->name);
                zd1205wext_giwrts(dev, NULL, &wrq->u.rts, NULL);
                break;

        case SIOCSIWRTS:
                ZD1211DEBUG(1, "%s: SIOCSIWRTS\n", dev->name);


                err = zd1205_ioctl_setrts(dev, &wrq->u.rts);
                if (! err)
                        changed = 1;
                break;

        case SIOCSIWFRAG:
                ZD1211DEBUG(1, "%s: SIOCSIWFRAG\n", dev->name);

                err = zd1205_ioctl_setfrag(dev, &wrq->u.frag);
                if (! err)
                        changed = 1;
                break;

        case SIOCGIWFRAG:
                ZD1211DEBUG(1, "%s: SIOCGIWFRAG\n", dev->name);
                err = zd1205_ioctl_getfrag(dev, &wrq->u.frag);
                break;

        case SIOCSIWRATE:
                ZD1211DEBUG(1, "%s: SIOCSIWRATE\n", dev->name);

                err = zd1205_ioctl_setrate(dev, &wrq->u.bitrate);
                if (! err)
                        changed = 1;

                break;

        case SIOCGIWRATE:
                ZD1211DEBUG(1, "%s: SIOCGIWRATE\n", dev->name);
                err = zd1205_ioctl_getrate(dev, &wrq->u.bitrate);
                break;

        case SIOCSIWPOWER:
                ZD1211DEBUG(1, "%s: SIOCSIWPOWER\n", dev->name);

                err = zd1205_ioctl_setpower(dev, &wrq->u.power);
                if (!err)
                        changed = 0;
                break;


        case SIOCGIWPOWER:
                ZD1211DEBUG(1, "%s: SIOCGIWPOWER\n", dev->name);
                err = zd1205_ioctl_getpower(dev, &wrq->u.power);
                break;

#if WIRELESS_EXT > 10

        case SIOCSIWRETRY:
                ZD1211DEBUG(1, "%s: SIOCSIWRETRY\n", dev->name);
                err = -EOPNOTSUPP;
                break;


        case SIOCGIWRETRY:
                ZD1211DEBUG(1, "%s: SIOCGIWRETRY\n", dev->name);
                err = zd1205_ioctl_getretry(dev, &wrq->u.retry);
                break;
#endif /* WIRELESS_EXT > 10 */

#if WIRELESS_EXT > 12
	case SIOCSIWCOMMIT:
		ZD1211DEBUG(1, "%s: SIOCSIWCOMMIT\n", dev->name);
		err = zd1205wext_commit(dev, NULL, NULL, NULL);
	break;
#endif

        case SIOCGIWPRIV:
                if (wrq->u.data.pointer) {
                        struct iw_priv_args privtab[] = {
                                                                {
                                                                        SIOCIWFIRSTPRIV + 0x0, 0, 0, "list_bss"
                                                                },
                                                                { SIOCIWFIRSTPRIV + 0x1, 0, 0, "card_reset" },
                                                                { SIOCIWFIRSTPRIV + 0x2, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_auth" },  /* 0 - open, 1 - shared key */
                                                                { SIOCIWFIRSTPRIV + 0x3, 0, IW_PRIV_TYPE_CHAR | 12, "get_auth" },
                                                                { SIOCIWFIRSTPRIV + 0x4, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_preamble" },  /* 0 - long, 1 - short */
                                                                { SIOCIWFIRSTPRIV + 0x5, 0, IW_PRIV_TYPE_CHAR | 6, "get_preamble" },
                                                                { SIOCIWFIRSTPRIV + 0x6, 0, 0, "cnt" },
                                                                { SIOCIWFIRSTPRIV + 0x7, 0, 0, "regs" },
                                                                { SIOCIWFIRSTPRIV + 0x8, 0, 0, "probe" },
                                                                /* ath_desc: reenable iwpriv dbg_flag */
                                                                { SIOCIWFIRSTPRIV + 0x10, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "dbg_flag" },
                                                                { SIOCIWFIRSTPRIV + 0xA, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "connect" },
                                                                { SIOCIWFIRSTPRIV + 0xB, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_mac_mode" },
                                                                { SIOCIWFIRSTPRIV + 0xC, 0, IW_PRIV_TYPE_CHAR | 12, "get_mac_mode" },
                                                                { SIOCIWFIRSTPRIV + 0xD, 0, 0, "save_conf" },
                                                                { SIOCIWFIRSTPRIV + 0xF, 0, IW_PRIV_TYPE_CHAR | 14, "get_Region" },
                                                                { SIOCIWFIRSTPRIV + 0x9,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_Region" },
                                                        };

                        err = access_ok(VERIFY_WRITE, wrq->u.data.pointer, sizeof(privtab));
                        if (err)
                                break;

                        wrq->u.data.length = sizeof(privtab) / sizeof(privtab[0]);
                        if (copy_to_user(wrq->u.data.pointer, privtab, sizeof(privtab)))
                                err = -EFAULT;
                }
                break;

        case SIOCIWFIRSTPRIV + 0x0: /* list_bss */
                ZD1211DEBUG(1, "%s: SIOCIWFIRSTPRIV + 0x0 (list_bss)\n", dev->name);
                macp->bss_index = zd_GetBssList(&macp->BSSInfo[0]);
                zd1205_list_bss(macp);
                break;

        case SIOCIWFIRSTPRIV + 0x1: /* card_reset */
                ZD1211DEBUG(1, "%s: SIOCIWFIRSTPRIV + 0x1 (card_reset)\n", dev->name);
                if (! capable(CAP_NET_ADMIN)) {
                        err = -EPERM;
                        break;
                }

                printk(KERN_DEBUG "%s: Force scheduling reset!\n", dev->name);
                zd1205_lock(macp);
                zd1205_device_reset(macp);
                zd1205_unlock(macp);
                err = 0;
                break;

        case SIOCIWFIRSTPRIV + 0x2: /* set_auth */
                ZD1211DEBUG(1, "%s: SIOCIWFIRSTPRIV + 0x2 (set_auth)\n", dev->name);
                if (! capable(CAP_NET_ADMIN)) {
                        err = -EPERM;
                        break;
                }
                {
                        int val = *( (int *) wrq->u.name );
                        if ((val < 0) || (val > 1)) {
                                err = -EINVAL;
                                break;
                        } else {
                                zd1205_lock(macp);
                                macp->cardSetting.AuthMode = val;
                                zd1205_unlock(macp);
                                err = 0;
                                changed = 1;
                        }
                }
                break;

        case SIOCIWFIRSTPRIV + 0x3: /* get_auth */
                ZD1211DEBUG(1, "%s: SIOCIWFIRSTPRIV + 0x3 (get_auth)\n", dev->name);

                if (wrq->u.data.pointer) {
                        wrq->u.data.flags = 1;

                        if (macp->cardSetting.AuthMode == 0) {
                                wrq->u.data.length = 12;

                                if (copy_to_user(wrq->u.data.pointer, "open system", 12)) {
                                        return -EFAULT;
                                }
                        } else if (macp->cardSetting.AuthMode == 1) {
                                wrq->u.data.length = 11;

                                if (copy_to_user(wrq->u.data.pointer, "shared key", 11)) {
                                        return -EFAULT;
                                }
                        } else if (macp->cardSetting.AuthMode == 2) {
                                wrq->u.data.length = 10;

                                if (copy_to_user(wrq->u.data.pointer, "auto mode", 10)) {
                                        return -EFAULT;
                                }
                        } else {
                                return -EFAULT;
                        }
                }
                break;

        case SIOCIWFIRSTPRIV + 0x4: /* set_preamble */
                ZD1211DEBUG(1, "%s: SIOCIWFIRSTPRIV + 0x4 (set_preamble)\n", dev->name);

                if (! capable(CAP_NET_ADMIN)) {
                        err = -EPERM;
                        break;
                }
                {
                        int val = *( (int *) wrq->u.name );

                        if ((val < 0) || (val > 1)) {
                                err = -EINVAL;
                                break;
                        } else {
                                zd1205_lock(macp);

                                if (val)
                                        macp->cardSetting.PreambleType = 1;
                                else
                                        macp->cardSetting.PreambleType = 0;

                                zd1205_unlock(macp);
                                err = 0;
                                changed = 1;
                        }
                }
                break;


        case SIOCIWFIRSTPRIV + 0x5: /* get_preamble */
                ZD1211DEBUG(1, "%s: SIOCIWFIRSTPRIV + 0x5 (get_preamble)\n", dev->name);

                if (wrq->u.data.pointer) {
                        wrq->u.data.flags = 1;

                        if (macp->cardSetting.PreambleType) {
                                wrq->u.data.length = 6;

                                if (copy_to_user(wrq->u.data.pointer, "short", 6)) {
                                        return -EFAULT;
                                }
                        } else {
                                wrq->u.data.length = 5;

                                if (copy_to_user(wrq->u.data.pointer, "long", 5)) {
                                        return -EFAULT;
                                }
                        }
                }
                break;

        case SIOCIWFIRSTPRIV + 0x6: /* dump_cnt */
                ZD1211DEBUG(1, "%s: SIOCIWFIRSTPRIV + 0x6 (dump_cnt)\n", dev->name);
                zd1205_dump_cnters(macp);
                break;

        case SIOCIWFIRSTPRIV + 0x7: /* dump_reg */
                ZD1211DEBUG(1, "%s: SIOCIWFIRSTPRIV + 0x7 (dump_cnt)\n", dev->name);
                zd1205_dump_regs(macp);
                break;

        case SIOCIWFIRSTPRIV + 0x8: /* probe */
                ZD1211DEBUG(1, "%s: SIOCIWFIRSTPRIV + 0x8 (probe)\n", dev->name);
                zd_CmdProcess(CMD_PROBE_REQ, 0, 0);
                break;

        /* ath_desc: reenable iwpriv dbg_flag */
        case SIOCIWFIRSTPRIV + 0x10: /* set_dbgflag */
                ZD1211DEBUG(1, "%s: SIOCIWFIRSTPRIV + 0x10 (set_dbgflag)\n", dev->name);

                if (! capable(CAP_NET_ADMIN)) {
                        err = -EPERM;
                        break;
                }
                {
                        int val = *( (int *) wrq->u.name );

                        if ((val < 0) || (val > 5)){
                                err = -EINVAL;
                                break;
                        }
                        else {
                                zd1205_lock(macp);
                                macp->dbg_flag = val;
                                zd1205_unlock(macp);
                                err = 0;
                        }
                }
                break;

        case SIOCIWFIRSTPRIV + 0xA: /* connect */
                ZD1211DEBUG(1, "%s: SIOCIWFIRSTPRIV + 0xA (connect)\n", dev->name);

                if (! capable(CAP_NET_ADMIN)) {
                        err = -EPERM;
                        break;
                }
                {
                        int val = *( (int *) wrq->u.name );

                        if ((val < 1) || (val >macp->bss_index)) {
                                err = -EINVAL;
                                break;
                        } else {
                                U8  bssTypeToConnect;
                                U16 capabilities;
                                u8 ChangeToMacMode=MIXED_MODE;
                                capabilities = macp->BSSInfo[val-1].cap;
                                //If you connect to non-A AP while in 5G Band, or
                                //you connect to A AP while in 2.4G, you need to
                                //do mac_mode change first
                                if((PURE_A_AP == macp->BSSInfo[val-1].apMode  &&
                                                PURE_A_MODE != mMacMode) ||
                                                (PURE_A_AP != macp->BSSInfo[val-1].apMode  &&
                                                 PURE_A_MODE == mMacMode) ) {
                                        if(PURE_A_AP == macp->BSSInfo[val-1].apMode)
                                                ChangeToMacMode = PURE_A_MODE;

                                        printk("Changed macmode in connect\n");
                                        macp->cardSetting.Channel = 8;//Default Channel to 8
                                        macp->cardSetting.MacMode = ChangeToMacMode;
                                        macp->bDefaultIbssMacMode=1;
                                        //set_mac_mode command has been issued by the user.
                                        zd1205_SetRatesInfo(macp);
                                        err = 0;
                                        zd_UpdateCardSetting(&macp->cardSetting);
                                }
                                if (capabilities & (CAP_IBSS | CAP_ESS)) {
                                        zd1205_lock(macp);
                                        memcpy((U8*)&mSsid,(U8*)macp->BSSInfo[val-1].ssid,34+1);
                                        memcpy((U8*)&dot11DesiredSsid, &mSsid, 34+1);
                                        macp->BSSInfo[val-1].ssid[mSsid.buf[1]+2]=0;
                                        mProbeWithSsid=TRUE;
                                        if (capabilities & CAP_IBSS) {
                                                if (macp->bDefaultIbssMacMode==0)
                                                        mMacMode=macp->cardSetting.MacMode=PURE_B_MODE;
                                                bssTypeToConnect=INDEPENDENT_BSS;
                                        } else {
                                                if (macp->bDefaultIbssMacMode==0)
                                                        mMacMode=macp->cardSetting.MacMode=MIXED_MODE;
                                                bssTypeToConnect=INFRASTRUCTURE_BSS;
                                        }
                                        mBssType=macp->cardSetting.BssType=bssTypeToConnect;
                                        zd_CmdProcess(CMD_CONNECT, &bssTypeToConnect, val);
                                        zd1205_unlock(macp);
                                }
                                err = 0;
                        }
                }
                break;


        case SIOCIWFIRSTPRIV + 0xB: /* set_mac_mode */
                ZD1211DEBUG(1, "%s: SIOCIWFIRSTPRIV + 0xB (set_mac_mode)\n", dev->name);

                if (! capable(CAP_NET_ADMIN)) {
                        err = -EPERM;
                        break;
                }
                {
                        int val = *( (int *) wrq->u.name );
                        int mac_mode_limit;

                        if(AL7230B_RF == dot11Obj.rfMode)
                                mac_mode_limit = 4; //4 = A,B,G
                        else if (AL2230_RF == dot11Obj.rfMode)
                                mac_mode_limit = 3; //3 = B,G
                        else if (AL2230S_RF == dot11Obj.rfMode)
                                mac_mode_limit = 3;
                        else {
                                printk("Unknown RF Module. You are not allowed to set mac mode\n");
                                mac_mode_limit = 0;
                        }
                        if ((val < 1) || (val > mac_mode_limit)) {
                                err = -EINVAL;
                                break;
                        } else {
                                //If Band changed from 2.4G <-> 5G, we need
                                //to set default channel
                                if( (macp->cardSetting.MacMode != PURE_A_MODE &&
                                                val == PURE_A_MODE)) {
                                        macp->cardSetting.Channel = 36;
                                } else if(macp->cardSetting.MacMode == PURE_A_MODE &&
                                                val != PURE_A_MODE) {
                                        macp->cardSetting.Channel = 1;
                                }

                                macp->cardSetting.MacMode = val;
                                macp->bDefaultIbssMacMode=1;// Indicates that the set_mac_mode command has been issued by the user.
                                zd1205_SetRatesInfo(macp);
                                err = 0;
                                changed = 1;
                        }
                }
                break;

        case SIOCIWFIRSTPRIV + 0xC: /* get_mac_mode */
                ZD1211DEBUG(1, "%s: SIOCIWFIRSTPRIV + 0xC (get_mac_mode)\n", dev->name);


                if (wrq->u.data.pointer) {
                        wrq->u.data.flags = 1;

                        if (macp->cardSetting.MacMode == MIXED_MODE) {
                                wrq->u.data.length = 11;
                                if (copy_to_user(wrq->u.data.pointer, "Mixed Mode", 11)) {
                                        return -EFAULT;
                                }
                        } else if (macp->cardSetting.MacMode == PURE_G_MODE) {
                                wrq->u.data.length = 12;
                                if (copy_to_user(wrq->u.data.pointer, "Pure G Mode", 12)) {
                                        return -EFAULT;
                                }
                        } else if (macp->cardSetting.MacMode == PURE_B_MODE) {
                                wrq->u.data.length = 12;
                                if (copy_to_user(wrq->u.data.pointer, "Pure B Mode", 12)) {
                                        return -EFAULT;
                                }
                        } else if (macp->cardSetting.MacMode == PURE_A_MODE) {
                                wrq->u.data.length = 12;
                                if (copy_to_user(wrq->u.data.pointer, "Pure A Mode", 12)) {
                                        return -EFAULT;
                                }
                        } else
                                return -EFAULT;
                }
                break;

        case SIOCIWFIRSTPRIV + 0xF: //get_Region
                //zd1205_dumpEEPROM(macp);
                if (wrq->u.data.pointer) {
                        wrq->u.data.flags = 1;

                        if (ZD_REGION_USA == dot11Obj.RegionCode) {
                                wrq->u.data.length = 3;
                                if (copy_to_user(wrq->u.data.pointer, "USA", 4))
                                        return -EFAULT;
                        } else if (ZD_REGION_Europe == dot11Obj.RegionCode) {
                                wrq->u.data.length = 13;
                                if (copy_to_user(wrq->u.data.pointer, "Taiwan/Europe", 14))
                                        return -EFAULT;
                        } else if (ZD_REGION_France == dot11Obj.RegionCode) {
                                wrq->u.data.length = 6;
                                if (copy_to_user(wrq->u.data.pointer, "France", 7))
                                        return -EFAULT;
                        } else if (ZD_REGION_Japan == dot11Obj.RegionCode) {
                                wrq->u.data.length = 5;

                                if (copy_to_user(wrq->u.data.pointer, "Japan", 6))
                                        return -EFAULT;
                        } else if (ZD_REGION_Israel == dot11Obj.RegionCode) {
                                wrq->u.data.length = 6;
                                if (copy_to_user(wrq->u.data.pointer, "Israel", 7))
                                        return -EFAULT;
                        } else if (ZD_REGION_Mexico == dot11Obj.RegionCode) {
                                wrq->u.data.length = 6;
                                if (copy_to_user(wrq->u.data.pointer, "Mexico", 7))
                                        return -EFAULT;
                        } else
                                return -EFAULT;
                }

                break;
        case SIOCIWFIRSTPRIV + 0x9 : //set_Region
                {
                        int val = *( (int *) wrq->u.name );

                        if ((val < 1) || (val > 6))
                        {
                                err = -EINVAL;
                                break;
                        } else
                        {
                                switch(val) {
                                case 1 :
                                        macp->RegionCode = ZD_REGION_USA;
                                        break;
                                case 2 :
                                        macp->RegionCode = ZD_REGION_Europe;
                                        break;
                                case 3 :
                                        macp->RegionCode = ZD_REGION_France;
                                        break;
                                case 4 :
                                        macp->RegionCode = ZD_REGION_Japan;
                                        break;
                                case 5 :
                                        macp->RegionCode = ZD_REGION_Israel;
                                        break;
                                case 6 :
                                        macp->RegionCode = ZD_REGION_Mexico;
                                        break;
                                }
                        }
                        dot11Obj.RegionCode = macp->RegionCode;
                        switch(val)
                        {
                        case 1 :
                                dot11Obj.AllowedChannel = 0x107ff;
                                break;//1-11
                        case 2 :
                                dot11Obj.AllowedChannel = 0x11fff;
                                break;//1-13
                        case 3 :
                                dot11Obj.AllowedChannel = 0xa1e00;
                                break;//10-13
                        case 4 :
                                dot11Obj.AllowedChannel = 0x13fff;
                                break;//1-14
                        case 5 :
                                dot11Obj.AllowedChannel = 0x301fc;
                                break;//3-9
                        case 6 :
                                dot11Obj.AllowedChannel = 0xa0600;
                                break;//10,11
                        }

                        break;
                }
                ////////////////////////////
#ifdef ZDCONF_MENUDBG
        case ZD_MENU_DBG:
                {
                        u32 in=0,ret=0;
                        if (copy_from_user(&zdreq, ifr->ifr_data, sizeof(zdreq))) {
                                return -EFAULT;
                        }
                        zd1205_lock(macp);
                        zd1205_zd_dbg2_ioctl(macp, &zdreq,&ret);
                        if (copy_from_user(&in,((struct zdap_ioctl *)(ifr->ifr_data))->data,4)) {
				zd1205_unlock(macp);
				return -EFAULT;
                        }
			if (copy_to_user(((struct zdap_ioctl *)(ifr->ifr_data))->data, &ret, sizeof(ret))) {
				zd1205_unlock(macp);
				return -EFAULT;
			}
                        zd1205_unlock(macp);
                        err = 0;
                }
                break;
#endif

        case ZDAPIOCTL:    //ZD1202 debug command
                if (copy_from_user(&zdreq, ifr->ifr_data, sizeof (zdreq))) {
                        printk(KERN_ERR "ZDAPIOCTL: copy_from_user error\n");
                        return -EFAULT;
                }

                printk(KERN_DEBUG "zd1211: cmd = %2x, reg = 0x%04x, value = 0x%08x\n", zdreq.cmd, zdreq.addr, zdreq.value);

                zd1205_lock(macp);
#ifdef HOST_IF_USB

                memcpy(&macp->zdreq, &zdreq, sizeof(zdreq));
                defer_kevent(macp, KEVENT_ZD_IOCTL);
#else

                zd1205_zd_dbg_ioctl(macp, &zdreq);
#endif

                zd1205_unlock(macp);
                err = 0;
                break;

        default:
                ZD1211DEBUG(2, "zd1205_ioctl: unsupported cmd = %2x\n", cmd);
                err = -EOPNOTSUPP;
                break;
        }

        if ((!err) && changed)
        {
#ifdef HOST_IF_USB
                defer_kevent(macp, KEVENT_UPDATE_SETTING);
#else

                zd_UpdateCardSetting(&macp->cardSetting);
#endif

        }

        return err;
}





/**
 * zd1205init - initialize the adapter
 * @macp: atapter's private data struct
 *
 * This routine is called when this driver is loaded. This is the initialization
 * routine which allocates memory, configures the adapter and determines the
 * system resources.
 *
 * Returns:


 *      true: if successful
 *      false: otherwise
 */

unsigned char
zd1205_init(struct zd1205_private *macp)
{
        u32 tmpValue;

        //ZENTER(0);
#if fPROG_FLASH

        macp->bAllowAccessRegister = 1;
#endif
        /* read the MAC address from the eprom */
        mTxOFDMType = &(((struct zd1205_private *)g_dev->priv)->TxOFDMType);
        zd1205_rd_eaddr(macp);

        zd_writel(0x01, AfterPNP);

#if fWRITE_WORD_REG || fREAD_MUL_REG

        // Must get this information before any register write


        tmpValue = zd1211_readl(cADDR_ENTRY_TABLE, false);
        macp->AddrEntryTable = (u16) tmpValue;
        ZD1211DEBUG(0, "AddrEntryTable = %04x\n", macp->AddrEntryTable);
#endif

        macp->RF_Mode = zd_readl(E2P_POD);
        ZD1211DEBUG(0, "RF_Mode = %08x\n", macp->RF_Mode);
        macp->PA_Type = ((macp->RF_Mode) >> 16) & 0xF;//hardware type 2, bit0-3
        printk(KERN_ERR "PA type: %01x\n", macp->PA_Type);

        dot11Obj.HWFeature = macp->RF_Mode & 0xfff0;

        if (((macp->RF_Mode & 0xf) == AL2230_RF) && (dot11Obj.HWFeature & BIT_7) )
                macp->RF_Mode = AL2230S_RF;
        else
                macp->RF_Mode &= 0x0f;

        dot11Obj.rfMode = (macp->RF_Mode & 0x0f);


        if ((dot11Obj.rfMode == 0x04) || (dot11Obj.rfMode == 0x07))
                printk("AiroHa AL2230RF\n");
        else if (dot11Obj.rfMode == 0x07)
                printk("Airoha AL7230B_RF\n");
        else if (dot11Obj.rfMode == 0x0a)
                printk("Airoha AL2230S_RF\n");
        else if (dot11Obj.rfMode == 0x09)
                printk("GCT RF\n");
        else if (dot11Obj.rfMode == 0x0d)
                printk("RFMD RF\n");
        else if (dot11Obj.rfMode == 0x05)
                printk("AIroHa 7230B_RF\n");
        else
                printk("RF_Mode = %x\n", (u8)dot11Obj.rfMode);



        zd_writel(0x00, GPI_EN);

        zd1205_sw_init(macp);
        zd1205_hw_init(macp);
        zd1205_disable_int();

        ZEXIT(0);
        return true;
}



void zd1205_init_card_setting(struct zd1205_private *macp)
{
        card_Setting_t *pSetting = &macp->cardSetting;

        pSetting->BssType = INFRASTRUCTURE_BSS;
        //pSetting->BssType = AP_BSS;
        //pSetting->BssType = INDEPENDENT_BSS;
        //pSetting->BssType = PSEUDO_IBSS;
        pSetting->HiddenSSID = 0; 	//disable hidden essid
        pSetting->LimitedUser = 32;
        pSetting->RadioOn = 1;

        pSetting->BlockBSS = 0;
        pSetting->EncryOnOff = 0;
        //pSetting->PreambleType = 0; //long preamble
        pSetting->PreambleType = 1; //short preamble
        pSetting->Channel = 6;
        pSetting->EncryMode = NO_WEP;
        pSetting->EncryKeyId = 0;
        pSetting->TxPowerLevel = 0;

        if (pSetting->BssType == AP_BSS)
        {
                pSetting->AuthMode = 2; 	//auto auth
                pSetting->Info_SSID[0] = 0;
                pSetting->Info_SSID[1] = 0x08;
                pSetting->Info_SSID[2] = 'Z';
                pSetting->Info_SSID[3] = 'D';
                pSetting->Info_SSID[4] = '1';
                pSetting->Info_SSID[5] = '2';
                pSetting->Info_SSID[6] = '1';
                pSetting->Info_SSID[7] = '1';
                pSetting->Info_SSID[8] = 'A';
                pSetting->Info_SSID[9] = 'P';
        } else if (pSetting->BssType == INFRASTRUCTURE_BSS)
        {
                pSetting->AuthMode = 0; 	//open syatem

                pSetting->Info_SSID[0] = 0;
                //pSetting->Info_SSID[1] = 0x05;
                pSetting->Info_SSID[1] = 0x00;
                pSetting->Info_SSID[2] = 'G';
                pSetting->Info_SSID[3] = '1';
                pSetting->Info_SSID[4] = '0';
                pSetting->Info_SSID[5] = '0';
                pSetting->Info_SSID[6] = '0';
                //pSetting->Info_SSID[7] = 'A';
                //pSetting->Info_SSID[8] = 'B';
        } else if (pSetting->BssType == INDEPENDENT_BSS)
        {
                pSetting->AuthMode = 0; 	//open syatem
                pSetting->Info_SSID[0] = 0;
                pSetting->Info_SSID[1] = 0x09;
                pSetting->Info_SSID[2] = '1';
                pSetting->Info_SSID[3] = '2';

                pSetting->Info_SSID[4] = '1';
                pSetting->Info_SSID[5] = '1';
                pSetting->Info_SSID[6] = 'A';
                pSetting->Info_SSID[7] = 'd';
                pSetting->Info_SSID[8] = 'H';
                pSetting->Info_SSID[9] = 'o';
                pSetting->Info_SSID[10] = 'c';
        }

#if	!(defined(GCCK) && defined(OFDM))
        pSetting->Info_SupportedRates[0] = 0x01;
        pSetting->Info_SupportedRates[1] = 0x05;
        pSetting->Info_SupportedRates[2] = 0x82;
        pSetting->Info_SupportedRates[3] = 0x84;
        pSetting->Info_SupportedRates[4] = 0x8B;
        pSetting->Info_SupportedRates[5] = 0x96;
        pSetting->Info_SupportedRates[6] = 0x21;



        if ((dot11Obj.rfMode == AL2210MPVB_RF) || (dot11Obj.rfMode == AL2210_RF))
        {
                pSetting->Rate275 = 1;
                pSetting->Info_SupportedRates[7] = 0x2C;//22
                pSetting->Info_SupportedRates[8] = 0x37;//27.5
                pSetting->Info_SupportedRates[1] = 0x07;
        } else
                pSetting->Rate275 = 0;
#else

        if (macp->usb->speed != USB_SPEED_HIGH)
                pSetting->MacMode = PURE_B_MODE;
        else
        {

                //if (pSetting->BssType == INDEPENDENT_BSS)
                //pSetting->MacMode = PURE_B_MODE;
                //else
                pSetting->MacMode = MIXED_MODE;
        }
        zd1205_SetRatesInfo(macp);
        //pCardSetting->UartEnable = 1;
        //pCardSetting->BaudRate = BAUD_RATE_115200;


#endif


        pSetting->FragThreshold = 0x980;
        pSetting->RTSThreshold = 0x980;

        pSetting->BeaconInterval = 100;
        pSetting->DtimPeriod = 3;
        pSetting->SwCipher = 0;


        pSetting->DynKeyMode = 0;
        pSetting->WpaBcKeyLen = 32; // Tmp key(16) + Tx Mic key(8) + Rx Mic key(8)


        //dot11Obj.MicFailure = NULL;
        //dot11Obj.AssocRequest = NULL;
        //dot11Obj.WpaIe =  NULL;
}



#ifndef HOST_IF_USB
int
zd1205_found1(struct pci_dev *pcid, const struct pci_device_id *ent)
{
        static int first_time = true;
        struct net_device *dev = NULL;
        struct zd1205_private *macp = NULL;

        int rc = 0;


        ZENTER(0);

        dev = alloc_etherdev(sizeof (struct zd1205_private));


        if (dev == NULL)
        {
                printk(KERN_ERR "zd1205: Not able to alloc etherdev struct\n");
                rc = -ENODEV;
                goto out;

        }



        g_dev = dev;  //save this for CBs use
        SET_MODULE_OWNER(dev);

        if (first_time)
        {
                first_time = false;

                printk(KERN_NOTICE "%s - version %s\n",
                       zd1205_full_driver_name, zd1205_driver_version);

                printk(KERN_NOTICE "%s\n", zd1205_copyright);
                printk(KERN_NOTICE "\n");
        }

        macp = dev->priv;
        macp->pdev = pcid;
        macp->device = dev;

        pci_set_drvdata(pcid, dev);
        macp->numTcb = NUM_TCB;

        macp->numTbd = NUM_TBD;
        macp->numRfd = NUM_RFD;

        macp->numTbdPerTcb = NUM_TBD_PER_TCB;
        macp->regp = 0;
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


        if ((rc = zd1205_pci_setup(pcid, macp)) != 0)
        {
                goto err_dev;
        }



        if (!zd1205_init(macp))
        {
                printk(KERN_ERR "zd1025: Failed to initialize, instance \n");

                rc = -ENODEV;

                goto err_pci;
        }


        dev->irq = pcid->irq;
        dev->open = &zd1205_open;
        dev->hard_start_xmit = &zd1205_xmit_frame;

        dev->stop = &zd1205_close;
        dev->change_mtu = &zd1205_change_mtu;
        dev->get_stats = &zd1205_get_stats;
        dev->set_multicast_list = &zd1205_set_multi;

        dev->set_mac_address = &zd1205_set_mac;

        dev->do_ioctl = &zd1205_ioctl;
        dev->features |= NETIF_F_SG | NETIF_F_HW_CSUM;

        if ((rc = register_netdev(dev)) != 0)
        {
                goto err_pci;
        }




        memcpy(macp->ifname, dev->name, IFNAMSIZ);
        macp->ifname[IFNAMSIZ-1] = 0;



        if (netif_carrier_ok(macp->device))
                macp->cable_status = "Cable OK";
        else
                macp->cable_status = "Not Available";

        if (zd1205_create_proc_subdir(macp) < 0)
        {
                printk(KERN_ERR "zd1205: Failed to create proc dir for %s\n",
                       macp->device->name);

        }


        printk(KERN_NOTICE "\n");
        goto out;

err_pci:

        iounmap(macp->regp);
        pci_release_regions(pcid);
        pci_disable_device(pcid);


err_dev:

        pci_set_drvdata(pcid, NULL);


        kfree(dev);

out:
        ZEXIT(0);
        return rc;
}
#endif



/**
 * zd1205_clear_structs - free resources
  * @dev: adapter's net_device struct

 *
 * Free all device specific structs, unmap i/o address, etc.
 */

void

zd1205_clear_structs(struct net_device *dev)
{
#ifndef HOST_IF_USB
        struct zd1205_private *macp = dev->priv;
#endif

        zd1205_sw_release();
#ifndef HOST_IF_USB

        iounmap(macp->regp);
        pci_release_regions(macp->pdev);
        pci_disable_device(macp->pdev);
        pci_set_drvdata(macp->pdev, NULL);
#endif

        //kfree(dev);
        /* ath_desc: fix module deregistering */
        /* ath_old: free_netdev(dev); */
}

#ifndef HOST_IF_USB
void
zd1205_remove1(struct pci_dev *pcid)
{
        struct net_device *dev;
        struct zd1205_private *macp;


        ZENTER(0);

        if (!(dev = (struct net_device *) pci_get_drvdata(pcid)))

                return;

        macp = dev->priv;
        unregister_netdev(dev);
        zd1205_remove_proc_subdir(macp);
        zd1205_clear_structs(dev);


        ZEXIT(0);
}
#endif


#if 0  //move codes to zdpci_hotplug.c
static struct pci_device_id zd1205_id_table[] __devinitdata =
        {


                {
                        0x167b, 0x2102, PCI_ANY_ID, PCI_ANY_ID, 0, 0, ZD_1202
                },
                {0x167b, 0x2100, PCI_ANY_ID, PCI_ANY_ID, 0, 0, ZD_1202},
                {0x167b, 0x2105, PCI_ANY_ID, PCI_ANY_ID, 0, 0, ZD_1205},

                {0,}
        };



MODULE_DEVICE_TABLE(pci, zd1205_id_table);



static struct pci_driver zd1205_driver =
        {
                .name         = "zd1205",
                                .id_table     = zd1205_id_table,
                                                .probe        = zd1205_found1,

                                                                .remove       = __devexit_p(zd1205_remove1),
                                                                        };


static int __init
zd1205_init_module(void)
{

        int ret;

        ret = pci_module_init(&zd1205_driver);
        return ret;
}


static void __exit
zd1205_cleanup_module(void)
{
        pci_unregister_driver(&zd1205_driver);
}

module_init(zd1205_init_module);

module_exit(zd1205_cleanup_module);
#endif

/*************************************************************************/
BOOLEAN zdcb_setup_next_send(fragInfo_t *frag_info)
{
        struct zd1205_private *macp = g_dev->priv;
        struct sk_buff *skb = (struct sk_buff *)frag_info->buf;
        U8 bIntraBss =  frag_info->bIntraBss;
        U8 MsgID = frag_info->msgID;
        U8 numOfFrag = frag_info->totalFrag;

        U16 aid = frag_info->aid;
        U8 hdrLen = frag_info->hdrLen;
        zd1205_SwTcb_t 	*sw_tcb;
        zd1205_HwTCB_t	*hw_tcb;
        zd1205_TBD_t	*pTbd;
        U8		*hdr, *pBody;
        U32		bodyLen, length;
        U32 		tcb_tbd_num = 0;
        int 		i;
        U16 		pdu_size = 0;

        void 		*addr;
        wla_Header_t	*wla_hdr;
        U32		CurrFragLen;
        U32		NextFragLen;

        skb_frag_t *frag = NULL;
        ctrl_Set_parm_t ctrl_setting_parms;
        U32 TotalLength = 0;

#ifndef HOST_IF_USB

        zd1205_SwTcb_t 	*next_sw_tcb;
        U32 		tmp_value, tmp_value3;
        unsigned long lock_flag;
        U32 loopCnt = 0;
#endif

#ifdef HOST_IF_USB

        U32 PrvFragLen = 0;

        if (!test_bit(ZD1211_RUNNING, &macp->flags))
                return FALSE;
#endif

        ZD1211DEBUG(2, "===== zdcb_setup_next_send enter =====\n");
        ZD1211DEBUG(2, "zd1211: bIntraBss = %x\n", bIntraBss);
        ZD1211DEBUG(2, "zd1211: numOfFrag = %x\n", numOfFrag);
        ZD1211DEBUG(2, "zd1211: skb = %p\n", (void *)skb);
        ZD1211DEBUG(2, "zd1211: aid = %x\n", aid);

#ifndef HOST_IF_USB

        spin_lock_irqsave(&macp->bd_non_tx_lock, lock_flag);
#endif


        if ((skb) && (!bIntraBss)) {   //data frame from upper layer
                if (skb_shinfo(skb)->nr_frags) {   //got frag buffer
                        frag = &skb_shinfo(skb)->frags[0];

                        if (skb->len > macp->cardSetting.FragThreshold) {  //need fragment
                                pdu_size = macp->cardSetting.FragThreshold - 24 - 4; //mac header and crc32 length
                                numOfFrag = (skb->len + (pdu_size-1) ) / pdu_size;

                                if (numOfFrag == 0)
                                        numOfFrag = 1;
                                ZD1211DEBUG(2, "zd1211: numOfFrag = %x\n", numOfFrag);
                        }
                }
        }

        if (macp->freeTxQ->count -1 < numOfFrag) {
                printk(KERN_ERR "zd1205: Not enough freeTxQ\n");
                //printk(KERN_ERR "zd1205: Cnt of freeTxQ = %x\n", macp->freeTxQ->count);
                zd_EventNotify(EVENT_TX_COMPLETE, ZD_RETRY_FAILED, (U32)MsgID, aid);
#ifndef HOST_IF_USB

                spin_unlock_irqrestore(&macp->bd_non_tx_lock, lock_flag);
#endif

                return FALSE;
        }

        ctrl_setting_parms.Rate = frag_info->rate;
        ctrl_setting_parms.Preamble = frag_info->preamble;
        ctrl_setting_parms.encryType = frag_info->encryType;
        //ctrl_setting_parms.vapId = frag_info->vapId;

        for (i=0; i<numOfFrag; i++) {
                ZD1211DEBUG(2, "zd1211: Cnt of freeTxQ = %x\n", macp->freeTxQ->count);
                ZD1211DEBUG(2, "zd1211: Frag Num = %x\n", i);

                sw_tcb = zd1205_first_txq(macp, macp->freeTxQ);

#ifdef HOST_IF_USB
                //sw_tcb->bHasCompleteBeforeSend = 0;
                //sw_tcb->bHasBeenDelayedBefore = 0;
#endif

                hdr = frag_info->macHdr[i];

                if (macp->dbg_flag > 4)
                        zd1205_dump_data("header part", (U8 *)hdr, 24);


                if (skb) {
                        if ((bIntraBss) || (!frag)) {  //wireless forwarding or tx data from upper layer and no linux frag
                                ZD1211DEBUG(2, "zd1211: Wireless forwarding or no linux frag\n");
                                pBody = frag_info->macBody[i];
                                bodyLen = frag_info->bodyLen[i];
                                CurrFragLen = bodyLen;
                                NextFragLen = frag_info->nextBodyLen[i];
                                if (i == (numOfFrag - 1))
                                        sw_tcb->LastFrag = 1;
                                else
                                        sw_tcb->LastFrag = 0;
                        } else { //tx data from upper layer with linux frag
                                ZD1211DEBUG(2, "zd1211: tx data from upper layer with linux frag\n");
                                pBody = skb->data;
                                bodyLen = skb->len;

                                if (i == (numOfFrag - 1)) {
                                        CurrFragLen = bodyLen - i*pdu_size;
                                        NextFragLen = 0;
                                        sw_tcb->LastFrag = 1;
                                } else {
                                        CurrFragLen = pdu_size;
                                        sw_tcb->LastFrag = 0;

                                        if (i == (numOfFrag-2))
                                                NextFragLen = bodyLen - (i+1)*pdu_size;
                                        else
                                                NextFragLen = pdu_size;

                                }
                        }
                } else {  //mgt frame
                        //ZD1211DEBUG(2, "zd1211: mgt frame\n");

                        pBody = frag_info->macBody[i];
                        bodyLen = frag_info->bodyLen[i];
                        CurrFragLen = bodyLen;
                        NextFragLen = frag_info->nextBodyLen[i];
                        sw_tcb->LastFrag = 1;
                }

                wla_hdr = (wla_Header_t *)hdr;
                hw_tcb = sw_tcb->pTcb;
                pTbd = sw_tcb->pFirstTbd;
                tcb_tbd_num = 0;
                hw_tcb->TxCbTbdNumber = 0;

                sw_tcb->FrameType = hdr[0];
                sw_tcb->MsgID = MsgID;
                sw_tcb->aid = aid;
                sw_tcb->skb = skb;
                sw_tcb->bIntraBss = bIntraBss;
                sw_tcb->Rate = frag_info->rate;

                //ZD1211DEBUG(2, "zd1211: sw_tcb = %x\n", sw_tcb);
                ZD1211DEBUG(2, "zd1211: hw_tcb = %p\n", (void *)hw_tcb);
                ZD1211DEBUG(2, "zd1211: first tbd = %p\n", (void *)pTbd);

                ctrl_setting_parms.CurrFragLen = CurrFragLen;
                ctrl_setting_parms.NextFragLen = NextFragLen;

                /* Control Setting */
                length = Cfg_CtrlSetting(macp, sw_tcb, wla_hdr, &ctrl_setting_parms);
                TotalLength = length;

#ifndef __LP64__
                pTbd->TbdBufferAddrHighPart = 0;
                pTbd->TbdBufferAddrLowPart = cpu_to_le32(sw_tcb->HwCtrlPhys);
#else
                pTbd->TbdBufferAddr = cpu_to_le64(sw_tcb->HwCtrlPhys);
#endif
                pTbd->TbdCount = cpu_to_le32(length);
                pTbd++;
                tcb_tbd_num++;

                /* MAC Header */
                if (ctrl_setting_parms.encryType == TKIP) {
                        length = Cfg_MacHeader(macp, sw_tcb, wla_hdr, hdrLen);
#ifndef __LP64__
                        pTbd->TbdBufferAddrLowPart = cpu_to_le32(sw_tcb->HwHeaderPhys);
#else
                        pTbd->TbdBufferAddr = cpu_to_le64(sw_tcb->HwHeaderPhys);
#endif
                } else { //WPA will failed, why??
                        length = hdrLen;
#ifndef __LP64__
                        pTbd->TbdBufferAddrLowPart = cpu_to_le32(hdr);
#else
                        pTbd->TbdBufferAddr = cpu_to_le64(hdr);
#endif
                }

                TotalLength += length;
                pTbd->TbdCount = cpu_to_le32(length);
                pTbd++;
                tcb_tbd_num++;

#if defined(AMAC)

                TotalLength += CurrFragLen;
#ifdef ZD1211

                sw_tcb->pHwCtrlPtr->CtrlSetting[18] = (u8)TotalLength;
                sw_tcb->pHwCtrlPtr->CtrlSetting[19] = (u8)(TotalLength >> 8);
#endif
#endif

                /* Frame Body */
                if ((!skb) || ((skb) && (!frag)) ) {
                        U8 * body_dma;

                        ZD1211DEBUG(2, "zd1211: Management frame body or No linux frag\n");

                        if (macp->dbg_flag > 4)
                                zd1205_dump_data("data part", (U8 *)pBody, 14);

#ifndef __LP64__
                        pTbd->TbdBufferAddrHighPart = 0;
#endif
#ifndef HOST_IF_USB

                        body_dma =  pci_map_single(macp->pdev, pBody, bodyLen, PCI_DMA_TODEVICE);
#else

                        body_dma = pBody;
#endif

                        ZD1211DEBUG(2, "zd1211: body_dma = %p\n", body_dma);

#ifndef __LP64__
                        pTbd->TbdBufferAddrLowPart =  cpu_to_le32(body_dma);
#else
                        pTbd->TbdBufferAddr =  cpu_to_le64(body_dma);
#endif
                        pTbd->TbdCount = cpu_to_le32(CurrFragLen);
                        pBody += CurrFragLen;

#ifdef HOST_IF_USB

                        pTbd->PrvFragLen = PrvFragLen;
                        PrvFragLen += CurrFragLen;
#endif

                        pTbd++;
                        tcb_tbd_num++;
                } else {
                        while(CurrFragLen ) {
                                void * body_dma;


                                if (CurrFragLen >= frag->size ) {
                                        printk(KERN_DEBUG "zd1205: linux more frag skb\n");
                                        addr = ((void *) page_address(frag->page) + frag->page_offset);
#ifndef __LP64__
                                        pTbd->TbdBufferAddrHighPart = 0;
#endif
#ifndef HOST_IF_USB

                                        body_dma = pci_map_single(macp->pdev, addr, frag->size, PCI_DMA_TODEVICE);
#else

                                        body_dma = addr;
#endif

#ifndef __LP64__
                                        pTbd->TbdBufferAddrLowPart =  cpu_to_le32(body_dma);
#else
                                        pTbd->TbdBufferAddr =  cpu_to_le64(body_dma);
#endif
                                        pTbd->TbdCount = cpu_to_le32(frag->size);
                                        tcb_tbd_num++;
#ifdef HOST_IF_USB

                                        pTbd->PrvFragLen = PrvFragLen;
                                        PrvFragLen += CurrFragLen;
#endif

                                        CurrFragLen -= frag->size;
                                        frag++;
                                } else {
                                        printk(KERN_DEBUG "zd1205: linux last frag skb\n");
                                        addr = ((void *) page_address(frag->page) + frag->page_offset);
#ifndef __LP64__
                                        pTbd->TbdBufferAddrHighPart = 0;
#endif
#ifndef HOST_IF_USB

                                        body_dma = cpu_to_le32(pci_map_single(macp->pdev, addr, pdu_size, PCI_DMA_TODEVICE));
#else

                                        body_dma =  addr;
#endif

#ifndef __LP64__
                                        pTbd->TbdBufferAddrLowPart =  cpu_to_le32(body_dma);
#else
                                        pTbd->TbdBufferAddr =  cpu_to_le64(body_dma);
#endif
                                        frag->page_offset += CurrFragLen;
                                        frag->size -= CurrFragLen;
#ifdef HOST_IF_USB

                                        pTbd->PrvFragLen = PrvFragLen;
                                        PrvFragLen += CurrFragLen;
#endif

                                        CurrFragLen = 0;
                                }

                                printk(KERN_DEBUG "zd1205: page_address = %p\n", addr);
                                printk(KERN_DEBUG "zd1205: body_dma = %p\n", body_dma);
                                pTbd++;
                                tcb_tbd_num++;
                        }
                }

                hw_tcb->TxCbTbdNumber = cpu_to_le32(tcb_tbd_num);
                macp->txCnt++;

#ifndef HOST_IF_USB

                hw_tcb->CbCommand = 0; /* set this TCB belong to bus master */
                wmb();

                while(1) {
                        tmp_value = zd_readl(DeviceState);
                        tmp_value &= 0xf;

                        if ((tmp_value == TX_READ_TCB) || (tmp_value == TX_CHK_TCB)) {
                                /* Device is now checking suspend or not.
                                 Keep watching until it finished check. */
                                loopCnt++;

                                if (loopCnt > 1000000)
                                        break;

                                udelay(1);
                                continue;
                        } else
                                break;
                }

                if (loopCnt > 1000000)
                        ZD1211DEBUG(0, "I am in zdcb_setup_next_send loop\n") ;

                ZD1211DEBUG(1, "zd1211: Device State = %x\n", (u32)tmp_value);

                if (tmp_value == TX_IDLE) { /* bus master get suspended TCB */
                        macp->txIdleCnt++;

                        /* Tx bus master is in idle state. */
                        //tmpValue1 = zd_readl(InterruptCtrl);
                        /* No retry fail happened */
                        tmp_value3 = zd_readl(ReadTcbAddress);
                        next_sw_tcb = macp->freeTxQ->first;

                        if (tmp_value3 != le32_to_cpu(next_sw_tcb->pTcb->NextCbPhyAddrLowPart)) {
                                /* Restart Tx again */
                                zd1205_start_download(sw_tcb->TcbPhys);
                                ZD1211DEBUG(1, "zd1211: Write  PCI_TxAddr_p1 = %x\n", sw_tcb->TcbPhys);
                        }
                }
                else if (tmp_value == 0xA) { //Dtim Notify Int happened
                        zd1205_start_download(sw_tcb->TcbPhys | BIT_0);
                }

                ZD1211DEBUG(2, "zd1211: NAV_CCA = %x\n", zd_readl(NAV_CCA));
                ZD1211DEBUG(2, "zd1211: NAC_CNT = %x\n", zd_readl(NAV_CNT));

#endif

                zd1205_qlast_txq(macp, macp->activeTxQ, sw_tcb);

#ifdef HOST_IF_USB
                //The following code is to handle cross fragment MIC
                memcpy(sw_tcb->CalMIC,frag_info->CalSwMic, MIC_LNG+1);
                sw_tcb->MIC_Start=0;
                sw_tcb->MIC_Len=0;
                if(i==numOfFrag-1 && sw_tcb->CalMIC[MIC_LNG]==TRUE) {
                        if(frag_info->bodyLen[i] < MIC_LNG) {
                                sw_tcb->MIC_Start=MIC_LNG-frag_info->bodyLen[i];
                                sw_tcb->MIC_Len=frag_info->bodyLen[i];
                        } else {
                                sw_tcb->MIC_Start=0;
                                sw_tcb->MIC_Len=MIC_LNG;
                        }
                        zd1211_submit_tx_urb(macp,TRUE);
                } else if(sw_tcb->CalMIC[MIC_LNG] == TRUE && (i == numOfFrag-2)) {
                        if(frag_info->bodyLen[i+1] < MIC_LNG) {
                                sw_tcb->MIC_Start=0;
                                sw_tcb->MIC_Len=MIC_LNG-frag_info->bodyLen[i+1];
                        } else {
                                sw_tcb->MIC_Start=0;
                                sw_tcb->MIC_Len=0;
                        }
                        zd1211_submit_tx_urb(macp,TRUE);
                } else
                        zd1211_submit_tx_urb(macp,FALSE);
                //zd1205_tx_isr(macp); //for debug only
#endif

                g_dev->trans_start = jiffies;
                macp->TxStartTime = nowT();
                ZD1211DEBUG(2, "zd1211: Cnt of activeQ = %x\n", macp->activeTxQ->count);
        }

#ifndef HOST_IF_USB
        spin_unlock_irqrestore(&macp->bd_non_tx_lock, lock_flag);
#endif

        ZD1211DEBUG(2, "===== zdcb_setup_next_send exit =====\n");
        return TRUE;
}

void zdcb_release_buffer(void *buf)
{
        struct sk_buff *skb = (struct sk_buff *)buf;

        if (skb)
                dev_kfree_skb_any(skb);
        //dev_kfree_skb_irq(skb);
}

void zdcb_rx_ind(U8 *pData, U32 length, void *buf)
{
        struct zd1205_private *macp = g_dev->priv;
        struct sk_buff *skb = (struct sk_buff *)buf;

        ZENTER(3);

        //copy packet for IP header is located on 4-bytes alignment
        if (length < RX_COPY_BREAK) {
                dev_kfree_skb_irq(skb);
                skb = dev_alloc_skb(length+2);

                if (skb) {
                        skb->dev = g_dev;
                        skb_reserve(skb, 2);
                        eth_copy_and_sum(skb, pData, length, 0);

                        skb_put(skb, length);
                }
        } else {
                skb->tail = skb->data = pData;
                skb_put(skb, length);
        }

        //zd1205_dump_data("rx_ind", (U8 *)skb->data, skb->len);

        ZD1211DEBUG(2, "zd1211: rx_ind length = %x\n", (u32)length);

        /* set the protocol */
        skb->protocol = eth_type_trans(skb, g_dev);
        skb->ip_summed = CHECKSUM_NONE;	//TBD
        g_dev->last_rx = jiffies;

        switch(netif_rx(skb)) {
        case NET_RX_BAD:
        case NET_RX_DROP:
        case NET_RX_CN_MOD:
        case NET_RX_CN_HIGH:
                break;

        default:
                macp->drv_stats.net_stats.rx_packets++;
                macp->drv_stats.net_stats.rx_bytes += skb->len;
                break;
        }
        ZEXIT(3);
}

U16 zdcb_status_notify(U16 status, U8 *StaAddr)
{
        struct zd1205_private *macp = g_dev->priv;
        U16 result = 0;
        int newassoc = 0;

        switch (status) {
        case STA_AUTH_REQ:
                break;

        case STA_ASOC_REQ:
                break;

        case STA_REASOC_REQ:
                break;

        case STA_ASSOCIATED:
        case STA_REASSOCIATED:
                macp->bAssoc = 1;
                mTmRetryConnect=0;
                iLED_ON(macp, macp->LinkLEDn);
#ifdef HOST_IF_USB

                macp-> LinkTimer = 0;

                if (macp->DataLED == 0) {
#ifdef ROBIN_KAO
                        zd_writel(0x03, FW_LINK_STATUS);
#else

                        zd_writel(0x01, FW_LINK_STATUS);
#endif

                } else
                        zd_writel(0x00, FW_LINK_STATUS);
#endif

                memcpy(&macp->BSSID[0], StaAddr, 6);

                //if (macp->cardSetting.BssType == INFRASTRUCTURE_BSS)
                if (macp->cardSetting.BssType != AP_BSS) {
                        netif_wake_queue(macp->device);
                        netif_carrier_on(macp->device);
                }

                if (status == STA_ASSOCIATED) {
                        ZD1211DEBUG(0, "STA_ASSOCIATED with " MACSTR "\n", MAC2STR(StaAddr));
                        newassoc = 1;
                } else {
                        ZD1211DEBUG(0, "STA_REASSOCIATED with " MACSTR "\n", MAC2STR(StaAddr));
                }
                /* Generate a wireless event to the upper layer */
                if(test_and_clear_bit(CTX_FLAG_ESSID_WAS_SET, (void*)&macp->flags)) {
                        zd1205_notify_join_event(macp);
                }

                break;

        case STA_DISASSOCIATED:
        case STA_DEAUTHED:
#ifndef HOST_IF_USB

                iLED_OFF(macp, LED1);
#else

                zd_writel(0x0, FW_LINK_STATUS);
#endif

                macp->bAssoc = 0;

                if (macp->cardSetting.BssType == INFRASTRUCTURE_BSS) {
                        union iwreq_data wreq;
                        memset(&wreq, 0, sizeof(wreq));
                        wreq.addr.sa_family=ARPHRD_ETHER;
                        wireless_send_event(macp->device, SIOCGIWAP,&wreq, NULL);
                        memset(&macp->BSSID[0], 0, 6);
                        netif_stop_queue(macp->device);
                        //zd1205_dis_connect(macp);
                        netif_carrier_off(macp->device);
                }
                if (status == STA_DISASSOCIATED)
                        ZD1211DEBUG(0, "STA_DISASSOCIATED:" MACSTR "\n",MAC2STR(StaAddr));
                else
                        ZD1211DEBUG(0, "STA_DEAUTHED:" MACSTR "\n",MAC2STR(StaAddr));

                break;

        default:
                break;
        }

        return result;
}


void zdcb_tx_completed(void)
{
}

void chal_tout_cb(unsigned long ptr)
{
#ifdef HOST_IF_USB
        struct zd1205_private *macp = g_dev->priv;
        defer_kevent(macp, KEVENT_TCHAL_TIMEOUT);
#else

        zd_EventNotify(EVENT_TCHAL_TIMEOUT, 0, 0, 0);
#endif
}

void scan_tout_cb(unsigned long ptr)
{
#ifdef HOST_IF_USB
        struct zd1205_private *macp = g_dev->priv;
        defer_kevent(macp, KEVENT_SCAN_TIMEOUT);
#else

        zd_EventNotify(EVENT_SCAN_TIMEOUT, 0, 0, 0);
#endif
}

void asoc_tout_cb(unsigned long ptr)
{
#ifdef HOST_IF_USB
        struct zd1205_private *macp = g_dev->priv;
        defer_kevent(macp, KEVENT_AUTH_TIMEOUT);
#else

        zd_EventNotify(EVENT_ASOC_TIMEOUT, 0, 0, 0);
#endif
}

void auth_tout_cb(unsigned long ptr)
{
#ifdef HOST_IF_USB
        struct zd1205_private *macp = g_dev->priv;
        defer_kevent(macp, KEVENT_AUTH_TIMEOUT);
#else

        zd_EventNotify(EVENT_AUTH_TIMEOUT, 0, 0, 0);
#endif
}

void zdcb_start_timer(U32 timeout, U32 event)
{
        struct zd1205_private *macp = g_dev->priv;
        u32	timeout_in_jiffies;
        if (!macp->bUSBDeveiceAttached)
                return;
        timeout_in_jiffies= (timeout*HZ)/1000; // Conver ms to jiffies

        switch (event) {
        case DO_CHAL:
                /* ath_desc: fix timer-related race conditions */
                macp->tm_chal_id.data = (unsigned long) g_dev;
                macp->tm_chal_id.function = chal_tout_cb;
                mod_timer(&macp->tm_chal_id, jiffies + timeout_in_jiffies);
                break;

        case DO_SCAN:
                macp->tm_scan_id.data = (unsigned long) g_dev;
                macp->tm_scan_id.function = scan_tout_cb;
                mod_timer(&macp->tm_scan_id, jiffies + timeout_in_jiffies);
                break;


        case DO_AUTH:
                macp->tm_auth_id.data = (unsigned long) g_dev;
                macp->tm_auth_id.function = auth_tout_cb;
                mod_timer(&macp->tm_auth_id, jiffies + timeout_in_jiffies);
                break;

        case DO_ASOC:
                if(AsocTimerStat) {
                        del_timer_sync(&macp->tm_asoc_id);
                        AsocTimerStat = FALSE;
                }
                init_timer(&macp->tm_asoc_id);
                macp->tm_asoc_id.data = (unsigned long) g_dev;
                macp->tm_asoc_id.function = asoc_tout_cb;
                mod_timer(&macp->tm_asoc_id, jiffies + timeout_in_jiffies);
                AsocTimerStat = TRUE;
                break;

        default:
                break;
        }
}


void zdcb_stop_timer(U32 TimerId)
{
        struct zd1205_private *macp = g_dev->priv;

        switch (TimerId) {
        case DO_CHAL:
                del_timer_sync(&macp->tm_chal_id);
                break;

        case DO_AUTH:
                del_timer_sync(&macp->tm_auth_id);
                break;


        case DO_ASOC:
                del_timer_sync(&macp->tm_asoc_id);
                AsocTimerStat = FALSE;
                break;

        default:
                break;

        }
}

U32
zdcb_dis_intr(void)
{
        struct zd1205_private *macp = g_dev->priv;
        U32 flags = 0;

#if 1//ndef HOST_IF_USB

        spin_lock_irqsave(&macp->cs_lock, flags);
#else

        spin_lock(&macp->cs_lock);
#endif

        return flags;
}

void
zdcb_set_intr_mask(U32 flags)
{
        struct zd1205_private *macp = g_dev->priv;

#if 1//ndef HOST_IF_USB

        spin_unlock_irqrestore(&macp->cs_lock, flags);
#else

        spin_unlock(&macp->cs_lock);
#endif
}

U32 zdcb_vir_to_phy_addr(U32 virtAddr) //TBD
{
        return virtAddr;
}

void zdcb_set_reg(void *reg, U32 offset, U32 value)
{
        zd_writel(value, offset);
}

U32 zdcb_get_reg(void *reg, U32 offset)
{
        return zd_readl(offset);
}

BOOLEAN
zdcb_check_tcb_avail(U8	num_of_frag)
{
        struct zd1205_private *macp = g_dev->priv;
        BOOLEAN ret;

        U32 flags;

        spin_lock_irqsave(&macp->q_lock, flags);

        /* ath_desc: fix oops when performing scan without invoking ifconfig up first */
        if (!macp->freeTxQ) {
                printk(KERN_WARNING "zd1211: FIXME: frames cannot be sent when the iface is down\n");
                ret = FALSE;
        } else if (macp->freeTxQ->count < (num_of_frag+1))
                ret = FALSE;
        else
                ret = TRUE;

        spin_unlock_irqrestore(&macp->q_lock, flags);
        return ret;
}


void zdcb_delay_us(U16 ustime)
{
        struct zd1205_private *macp=g_dev->priv;
        interruptible_sleep_on_timeout(&macp->msdelay, ustime/1000);
        //udelay(ustime);
}

void * zdcb_AllocBuffer(U16 dataSize, U8 **pData)
{
        struct sk_buff *new_skb = NULL;

        new_skb = (struct sk_buff *) dev_alloc_skb(dataSize);
        if (new_skb) {
                *pData = new_skb->data;
        }

        return (void *)new_skb;
}

unsigned long int next = 1;

int zdcb_Rand(U32 seed)
{
        static int first = 1;

        if (first) {
                next = seed;
                first = 0;
        }

        next = next * 1103515245 + 12345;
        return ((unsigned int)(next/65535)%32768);
}

void zdcb_AcquireDoNotSleep(void)
{
        struct zd1205_private *macp = g_dev->priv;
        atomic_inc(&macp->DoNotSleep);
}

void zdcb_ReleaseDoNotSleep(void)
{
        struct zd1205_private *macp = g_dev->priv;
        atomic_dec(&macp->DoNotSleep);
}
#ifdef HOSTAPD_SUPPORT
/*
   In this callback function, we just copy the WPA IE into the
   field of each STA's hash_t entry. The behavior is for the hostapd.
 */

int zdcb_AssocRequest(U8 *addr, U8 *WPAIE, U16 size)
{
        Hash_t *pHash;
        (void)size; /* unused callback param */

        if (mCounterMeasureState) {
#ifdef WPA_DEBUG
                printk(KERN_DEBUG "---------In CounterMeasure state, fail AssocReqest\n");
#endif

                return 1;
        }
        pHash = HashSearch((MacAddr_t *)addr);

        /* Can't search the MAC address in the hash table. */
        if (pHash == NULL)
                return 1;
        else {
                memcpy(&pHash->WPAIE, WPAIE, sizeof(pHash->WPAIE));
        }
        return 0;
}

void zdcb_MicFailure(unsigned char *addr)
{
        union iwreq_data wrqu;
        char buf[128];

        /* TODO: needed parameters: count, keyid, key type, src address, TSC */
#ifdef WPA_DEBUG

        printk(KERN_ERR "MLME-MICHAELMICFAILURE.indication addr=" MACSTR "\n", MAC2STR(addr));
#endif

        memset(&wrqu, 0, sizeof(wrqu));
        wrqu.data.length = strlen(buf);
        wireless_send_event(g_dev, IWEVCUSTOM, &wrqu, buf);
}

#endif

//setup callback functions for protocol stack
void zd1205_set_zd_cbs(zd_80211Obj_t *pObj)
{
        pObj->QueueFlag = 0;
        pObj->ConfigFlag = 0;
        pObj->SetReg = zdcb_set_reg;
        pObj->GetReg = zdcb_get_reg;

        pObj->ReleaseBuffer = zdcb_release_buffer;
        pObj->RxInd = zdcb_rx_ind;
        pObj->TxCompleted = zdcb_tx_completed;
        pObj->StartTimer = zdcb_start_timer;
        pObj->StopTimer = zdcb_stop_timer;
        pObj->SetupNextSend = zdcb_setup_next_send;

        pObj->StatusNotify = zdcb_status_notify;
        pObj->ExitCS = zdcb_set_intr_mask;
        pObj->EnterCS = zdcb_dis_intr;
        pObj->Vir2PhyAddr = zdcb_vir_to_phy_addr;
        pObj->CheckTCBAvail = zdcb_check_tcb_avail;
        pObj->DelayUs = zdcb_delay_us;
        pObj->AllocBuffer = zdcb_AllocBuffer;
        pObj->Rand = zdcb_Rand;
        pObj->AcquireDoNotSleep = zdcb_AcquireDoNotSleep;
        pObj->ReleaseDoNotSleep = zdcb_ReleaseDoNotSleep;
        pObj->bChScanning=0;

        // wpa support
#ifdef HOSTAPD_SUPPORT

        pObj->MicFailure = zdcb_MicFailure;
        pObj->AssocRequest = zdcb_AssocRequest;
        pObj->WpaIe = NULL;
#else
        // wpa support
        pObj->MicFailure = NULL;
        pObj->AssocRequest = NULL;
        pObj->WpaIe = NULL;
#endif

}

void zd1205_SetRatesInfo(struct zd1205_private *macp)
{

        u8 bRatesSet = 1;
        card_Setting_t *pCardSetting;

        pCardSetting = &macp->cardSetting;

        if (pCardSetting->MacMode == MIXED_MODE)
        {
                ZD1211DEBUG(0, "Mixed Mode\n");
                zd_writel(CW_SHORT_SLOT, CWmin_CWmax);
                pCardSetting->ShortSlotTime = 1;
                pCardSetting->PreambleType = 1; //short preamble
                bRatesSet = 1;
        } else if (pCardSetting->MacMode == PURE_G_MODE)
        {
                ZD1211DEBUG(0, "Pure G-Mode\n");
                zd_writel(CW_SHORT_SLOT, CWmin_CWmax);
                pCardSetting->ShortSlotTime = 1;
                pCardSetting->PreambleType = 1; //short preamble
                bRatesSet = 2;
        } else if (pCardSetting->MacMode == PURE_A_MODE)
        {
                ZD1211DEBUG(0, "Pure A-Mode\n");
                zd_writel(CW_SHORT_SLOT, CWmin_CWmax);
                pCardSetting->ShortSlotTime = 1;
                pCardSetting->PreambleType = 1; //short preamble
                bRatesSet = 4;

        } else if (pCardSetting->MacMode == PURE_B_MODE)
        { // pure B mode
                ZD1211DEBUG(0, "Pure B-Mode\n");
                zd_writel(CW_NORMAL_SLOT, CWmin_CWmax);
                pCardSetting->ShortSlotTime = 0;
                pCardSetting->PreambleType = 1; //short preamble
                bRatesSet = 3;
        } else
                VerAssert();

        if (bRatesSet == 1)
        { //wi-fi set1
                // supported rates
                pCardSetting->Info_SupportedRates[0] = 0x01;
                pCardSetting->Info_SupportedRates[1] = 0x04;
                pCardSetting->Info_SupportedRates[2] = 0x82; //basic rate
                pCardSetting->Info_SupportedRates[3] = 0x84; //basic rate
                pCardSetting->Info_SupportedRates[4] = 0x8B; //basic rate
                pCardSetting->Info_SupportedRates[5] = 0x96; //basic rate

                //Extended supported rates
                pCardSetting->Ext_SupportedRates[0] = 0x32;
                pCardSetting->Ext_SupportedRates[1] = 0x08;
                pCardSetting->Ext_SupportedRates[2] = 0x0c;
                pCardSetting->Ext_SupportedRates[3] = 0x12;
                pCardSetting->Ext_SupportedRates[4] = 0x18;
                pCardSetting->Ext_SupportedRates[6] = 0x24;
                pCardSetting->Ext_SupportedRates[7] = 0x30;
                pCardSetting->Ext_SupportedRates[8] = 0x48;
                pCardSetting->Ext_SupportedRates[5] = 0x60;
                pCardSetting->Ext_SupportedRates[9] = 0x6c;
                zd_writel(0x150f, MandatoryRateTbl); //1,2,5.5,11,6,12,24
        } else if (bRatesSet == 2)
        { //wi-fi set2
                // supported rates
                pCardSetting->Info_SupportedRates[0] = 0x01;
                pCardSetting->Info_SupportedRates[1] = 0x04;
                pCardSetting->Info_SupportedRates[2] = 0x82; //basic rate
                pCardSetting->Info_SupportedRates[3] = 0x84; //basic rate
                pCardSetting->Info_SupportedRates[4] = 0x8B; //basic rate
                pCardSetting->Info_SupportedRates[5] = 0x96; //basic rate

                //Extended supported rates
                pCardSetting->Ext_SupportedRates[0] = 0x32;
                pCardSetting->Ext_SupportedRates[1] = 0x08;
                pCardSetting->Ext_SupportedRates[2] = 0x8c; //basic rate
                pCardSetting->Ext_SupportedRates[3] = 0x12;
                pCardSetting->Ext_SupportedRates[4] = 0x98; //basic rate
                pCardSetting->Ext_SupportedRates[6] = 0x24;
                pCardSetting->Ext_SupportedRates[7] = 0xb0; //basic rate
                pCardSetting->Ext_SupportedRates[8] = 0x48;
                pCardSetting->Ext_SupportedRates[5] = 0x60;
                pCardSetting->Ext_SupportedRates[9] = 0x6c;

                zd_writel(0x150f, MandatoryRateTbl); //1,2,5.5,11,6,12,24
        } else if (bRatesSet == 3)
        { //pure b mode
                // supported rates
                pCardSetting->Info_SupportedRates[0] = 0x01;
                pCardSetting->Info_SupportedRates[1] = 0x04;
                pCardSetting->Info_SupportedRates[2] = 0x82; //basic rate
                pCardSetting->Info_SupportedRates[3] = 0x84; //basic rate
                pCardSetting->Info_SupportedRates[4] = 0x8B; //basic rate
                pCardSetting->Info_SupportedRates[5] = 0x96; //basic rate
                zd_writel(0x0f, MandatoryRateTbl); //1,2,5.5,11
        } else if (bRatesSet == 4)
        {//Pure A
                pCardSetting->Info_SupportedRates[0] = 0x01; //Element ID
                pCardSetting->Info_SupportedRates[1] = 0x08; //Rates Amount
                pCardSetting->Info_SupportedRates[2] = 0x80+12 ; //RateByte = Mandatory Bit + 500k x 12
                pCardSetting->Info_SupportedRates[3] = 0x00+18; //Supported Rate
                pCardSetting->Info_SupportedRates[4] = 0x80+24; //basic rate
                pCardSetting->Info_SupportedRates[5] = 0x00+36;
                pCardSetting->Info_SupportedRates[6] = 0x80+48; //basic rate
                pCardSetting->Info_SupportedRates[7] = 0x00+72;
                pCardSetting->Info_SupportedRates[8] = 0x00+96;
                pCardSetting->Info_SupportedRates[9] = 0x00+108;

                zd_writel(0x0f, MandatoryRateTbl); //6,9,12,18,24,36,48,54

        }

}


u16 ZDLOGTEN[] = {0,
                  0   ,  30 ,  47 ,  60 ,  69 ,  77 ,  84 ,  90 ,  95 , 100 ,
                  104 , 107 , 111 , 114 , 117 , 120 , 123 , 125 , 127 , 130 ,
                  132 , 134 , 136 , 138 , 139 , 141 , 143 , 144 , 146 , 147 ,
                  149 , 150 , 151 , 153 , 154 , 155 , 156 , 157 , 159 , 160 ,
                  161 , 162 , 163 , 164 , 165 , 166 , 167 , 168 , 169 , 169 ,
                  170 , 171 , 172 , 173 , 174 , 174 , 175 , 176 , 177 , 177 ,
                  178 , 179 , 179 , 180 , 181 , 181 , 182 , 183 , 183 , 184 ,
                  185 , 185 , 186 , 186 , 187 , 188 , 188 , 189 , 189 , 190 ,
                  190 , 191 , 191 , 192 , 192 , 193 , 193 , 194 , 194 , 195 ,
                  195 , 196 , 196 , 197 , 197 , 198 , 198 , 199 , 199 , 200 ,
                  200 , 200 , 210 , 210 , 220 , 220 , 220 , 230 , 230 , 240 ,
                  240 , 240 , 250 , 250 , 260 , 260 , 260 , 270 , 270 , 270 ,
                  280 , 280 , 280 , 290 , 290 , 210 , 210 , 210 , 211 , 211 ,
                  211 , 212 , 212 , 212 , 213 , 213 , 213 , 213 , 214 , 214 ,
                  214 , 215 , 215 , 215 , 216 , 216 , 216 , 217 , 217 , 217 ,
                  217 , 218 , 218 , 218 , 219 , 219 , 219 , 219 , 220 , 220 ,
                  220 , 220 , 221 , 221 , 221 , 222 , 222 , 222 , 222 , 223 ,
                  223 , 223 , 223 , 224 , 224 , 224 , 224 , 225 , 225 , 225 ,
                  225
                 };

u16
ZDLog10multiply100(int data)
{
        if ((data >= 0) && (data <= 0xb5))
                return ZDLOGTEN[data];
        else
                return 225;
}



u32 X_Constant[] = {
                           715, 655, 585, 540, 470, 410, 360, 315,
                           270, 235, 205, 175, 150, 125, 105, 85,
                           65, 50, 40, 25, 15

                   };


u8 X_To_dB(u32 X, u8 rate)
{
        u8 ret = 0;
        int i;

        int SizeOfX_Con = sizeof(X_Constant);

        switch (rate) {
        case 0x0B:  // 6M
        case 0x0A:  // 12M
        case 0x09:  // 24M
                X /= 2;
                break;
        case 0x0F:  // 9M
        case 0x0E:  // 18M
        case 0x0D:  // 36M
        case 0x0C:  // 54M
                X *= 3;
                X /= 4;
                break;
        case 0x08:  // 48M
                X *= 2;
                X /= 3;
                break;
        default:
                break;
        }
        for (i=0; i<SizeOfX_Con; i++) {
                if (X > X_Constant[i])
                        break;
        }

        switch (rate) {
        case 0x0B:  // 6M
        case 0x0F:  // 9M
                ret = i + 3;
                break;
        case 0x0A:  // 12M
        case 0x0E:  // 18M
                ret = i + 5;
                break;
        case 0x09:  // 24M
        case 0x0D:  // 36M
                ret = i + 9;
                break;
        case 0x08:  // 48M
        case 0x0C:  // 54M
                ret = i + 15;
                break;
        default:
                break;
        }
        return ret;
}

u8 CalculateQuality(struct zd1205_private *macp, zd1205_RFD_t *rfd, u8 *pQualityIndB)
{
        u8 CorrectQuality = 0;
        plcp_wla_Header_t *wla_hdr;
        u32 frame_len,tot_len;
        u8 SignalQuality2 = macp->rxSignalQuality2;
        u32 X;
        u16	tmpf;
        //u8 rxOffset = macp->rxOffset;

        wla_hdr = (plcp_wla_Header_t *)&rfd->RxBuffer[macp->rxOffset];
        tot_len = (le32_to_cpu(rfd->ActualCount) & 0x3fff);
        frame_len = tot_len - EXTRA_INFO_LEN;
        SignalQuality2 = rfd->RxBuffer[frame_len+2];



        if (rfd->RxBuffer[tot_len-1] & 0x01)
        {
                // it's OFDM
                macp->rxOFDMDataFrame++;

                X = 10000 * SignalQuality2 / (frame_len - macp->rxOffset);
                CorrectQuality = X_To_dB(X, wla_hdr->PlcpHdr[0] & 0xF);

                if (pQualityIndB)
                        *pQualityIndB = CorrectQuality;

                CorrectQuality = (CorrectQuality +0)* 4;
                if (CorrectQuality > 100)
                        CorrectQuality = 100;
        } else
        {
                // it's CCK
                macp->rx11bDataFrame++;

                // the value from PHY is in scale from Max is 0 and Min is 0xb5
                switch(wla_hdr->PlcpHdr[0]) {
                case 0x0A: //1M
                case 0x14: //2M
                case 0x37: //5.5M
                case 0x6E: //11M
                        tmpf = 0;

                        if (macp->rxSignalQuality1 > 0)
                                tmpf = (u16)(ZDLog10multiply100(macp->rxSignalQuality1) * 20 /100);
                        CorrectQuality = 45 - (u8)(tmpf);

                        if (pQualityIndB)
                                *pQualityIndB = CorrectQuality;

                        CorrectQuality = (CorrectQuality+5) * 4;

                        if (CorrectQuality > 100)
                                CorrectQuality = 100;
                        break;

                default:
                        break;


                }
        }

        return CorrectQuality;
}

u8 CalculateStrength(struct zd1205_private *macp, zd1205_RFD_t *rfd)
{
        // return in ? , the Value-105 = dB
        // the value from PHY is in ?
        u32 frame_len;
        u32 tot_len;
        u8 i, rssi, tmp;
        u32 tmpvalue = 2;
        plcp_wla_Header_t *wla_hdr;
        //u8 rxOffset = macp->rxOffset;

        wla_hdr = (plcp_wla_Header_t *)&rfd->RxBuffer[macp->rxOffset];
        tot_len = (le32_to_cpu(rfd->ActualCount) & 0x3fff);
        frame_len = tot_len - EXTRA_INFO_LEN;
        rssi = rfd->RxBuffer[frame_len+1];

        if ( (((macp->cardSetting.BssType == INFRASTRUCTURE_BSS)&&
                        (!memcmp(wla_hdr->Address2, macp->BSSID, 6))) ||
                        ((macp->cardSetting.BssType == INDEPENDENT_BSS)&&
                         (!memcmp(wla_hdr->Address3, macp->BSSID, 6))) ||
                        (macp->cardSetting.BssType == PSEUDO_IBSS)) &&
                        (macp->bAssoc) )
        {
                for(i=0; i<macp->PHYTestIndex-1; i++)
                        tmpvalue *= 2;

                //if ( (dot11Obj.CR122Flag == 1)||(dot11Obj.CR203Flag == 1) )
                //	rssi += 22;
                tmp = macp->PHYTestRssi;
                macp->PHYTestTotal = macp->PHYTestTotal
                                     - (macp->PHYTestTotal/tmpvalue)
                                     + rssi;
                macp->PHYTestRssi = (u8) (macp->PHYTestTotal/tmpvalue);
        }

        return rssi;
}

void zd1205_initCAM(struct zd1205_private *macp)
{
        int i;

        zd_writel(0, CAM_ROLL_TB_LOW);
        zd_writel(0, CAM_ROLL_TB_HIGH);

        for (i=0; i<445; i++)
        {
                HW_CAM_Write(&dot11Obj, i, 0);
        }
}

int zd1205_CheckOverlapBss(struct zd1205_private *macp, plcp_wla_Header_t *pWlanHdr, u8 *pMacBody, u32 bodyLen)
{
        u8 *pByte;
        u32 currPos = 0;
        u8 elemId, elemLen;
        u8 gAP = 0;

        u8 ErpInfo = 0;

        //get element
        pByte = pMacBody+SSID_OFFSET;
        currPos = SSID_OFFSET;

        while(currPos < bodyLen)
        {
                elemId = *pByte;
                elemLen = *(pByte+1);

                switch(elemId) {
                case ELEID_ERP_INFO: //ERP info
                        gAP = 1;
                        ErpInfo = *(pByte+2);
                        pByte += (elemLen+2);
                        break;

                default:
                        pByte += (elemLen+2);
                        break;

                }

                currPos += elemLen+2;
        }

        if (gAP)
        {
                if (ErpInfo & NON_ERP_PRESENT_BIT) { //with B sta associated
                        return 1;
                } else
                        return 0;
        } else // B AP exist, enable protection mode
                return 1;
}

void zd1205_HandleQosRequest(struct zd1205_private *macp)
{
        zd1205_SwTcb_t *sw_tcb;

        if (!macp->activeTxQ->count)
                sw_tcb = macp->freeTxQ->first;
        else
                sw_tcb = macp->activeTxQ->first;
        zd1205_start_download(sw_tcb->TcbPhys | BIT_0);
}

/**
 * zd1205_notify_join_event - Notify wireless join event to the upper layer
 * @macp: atapter's private data struct
 * @newassoc: new associate or not
 *
 */

void zd1205_notify_join_event(struct zd1205_private *macp)
{
        union iwreq_data wreq;

        memset(&wreq, 0, sizeof(wreq));
        memcpy(wreq.addr.sa_data, &macp->BSSID[0], 6);
        wreq.addr.sa_family = ARPHRD_ETHER;

        {
                ZD1211DEBUG(0, "Notify_join_event:" MACSTR "\n",MAC2STR(macp->BSSID));
                /*	int ii;

			WPADEBUG("zd1205_notfiy_join_event: MAC= ");
			for(ii = 0; ii < 6; ii++)
				WPADEBUG("%02x ", macp->BSSID[ii] & 0xff);
			WPADEBUG("\n");*/
        }

        if(macp->cardSetting.BssType == INFRASTRUCTURE_BSS)
        {
                wireless_send_event(macp->device, SIOCGIWAP, &wreq, NULL);
        }
#if WIRELESS_EXT >= 15
        else if(macp->cardSetting.BssType == AP_BSS)
        {
                wireless_send_event(macp->device, IWEVREGISTERED, &wreq, NULL);
        }
#endif
}
void zd1205_notify_disjoin_event(struct zd1205_private *macp)
{
        union iwreq_data wreq;

        memset(&wreq, 0, sizeof(wreq));
        //memcpy(wreq.addr.sa_data, &macp->BSSID[0], 6);
        wreq.addr.sa_family = ARPHRD_ETHER;
        printk(KERN_DEBUG "zd1205_notify_disjoin_event\n");
        /*{
		int ii;

		WPADEBUG("zd1205_notfiy_join_event: MAC= ");
		for(ii = 0; ii < 6; ii++)
			WPADEBUG("%02x ", macp->BSSID[ii] & 0xff);
		WPADEBUG("\n");
        }*/

        if(macp->cardSetting.BssType == INFRASTRUCTURE_BSS)
        {
                //wireless_send_event(macp->device, SIOCGIWSCAN, &wreq, NULL);
                wireless_send_event(macp->device, SIOCGIWAP, &wreq, NULL);
        }
        /*#if WIRELESS_EXT >= 15
		else if(macp->cardSetting.BssType == AP_BSS) {
			wireless_send_event(macp->device, IWEVREGISTERED, &wreq, NULL);
		}
        #endif*/
}
void zd1205_notify_scan_done(struct zd1205_private *macp)
{
        union iwreq_data wreq;
        wreq.data.length = 0;
        wreq.data.flags = 0;
        wireless_send_event(macp->device, SIOCGIWSCAN, &wreq, NULL);
}
#if WIRELESS_EXT >= 18
void hostap_michael_mic_failure(struct zd1205_private *macp,
                                struct hostap_ieee80211_hdr *hdr,
                                int keyidx)
{
        union iwreq_data wrqu;
        struct iw_michaelmicfailure ev;

        /* TODO: needed parameters: count, keyid, key type, TSC */
        memset(&ev, 0, sizeof(ev));
        ev.flags = keyidx & IW_MICFAILURE_KEY_ID;
        if (hdr->addr1[0] & 0x01)
                ev.flags |= IW_MICFAILURE_GROUP;
        else
                ev.flags |= IW_MICFAILURE_PAIRWISE;
        ev.src_addr.sa_family = ARPHRD_ETHER;
        memcpy(ev.src_addr.sa_data, hdr->addr2, ETH_ALEN);
        memset(&wrqu, 0, sizeof(wrqu));
        wrqu.data.length = sizeof(ev);
        wireless_send_event(g_dev, IWEVMICHAELMICFAILURE, &wrqu, (char *) &ev);
}
#elif WIRELESS_EXT >= 15
// For kernel 2.6.5(FC2), WIRELESS_EXT is 16
void hostap_michael_mic_failure(struct zd1205_private *macp,
                                struct hostap_ieee80211_hdr *hdr,
                                int keyidx)
{
        union iwreq_data wrqu;
        char buf[128];

        /* TODO: needed parameters: count, keyid, key type, TSC */
        sprintf(buf, "MLME-MICHAELMICFAILURE.indication(keyid=%d %scast addr="
                MACSTR ")", keyidx, hdr->addr1[0] & 0x01 ? "broad" : "uni",
                MAC2STR(hdr->addr2));
        memset(&wrqu, 0, sizeof(wrqu));
        wrqu.data.length = strlen(buf);
        printk("MLME-MICHAELMICFAILURE.indication(keyid=%d %scast addr="
               MACSTR ")", keyidx, hdr->addr1[0] & 0x01 ? "broad" : "uni",
               MAC2STR(hdr->addr2));
        wireless_send_event(g_dev, IWEVCUSTOM, &wrqu, buf);
}
#else /* WIRELESS_EXT >= 15 */
void hostap_michael_mic_failure(struct zd1205_private *macp,
                                struct hostap_ieee80211_hdr *hdr,
                                int keyidx)
{}
#endif /* WIRELESS_EXT >= 15 */
void ChangeMacMode(u8 MAC_Mode, u8 Channel)
{
        struct zd1205_private *macp;

        if(NULL != g_dev && NULL != g_dev->priv)
                macp = (struct zd1205_private *)g_dev->priv;
        else {
                LongPrint("NULL macp in ChnageMacMode\n",1);
                return;
        }


        zd1205_lock(macp);
        macp->cardSetting.Channel = Channel; //Default Channel to 8
        dot11Obj.Channel = Channel;
        macp->cardSetting.MacMode = MAC_Mode ;
        macp->bDefaultIbssMacMode=1;
        zd1205_unlock(macp);

        //set_mac_mode command has been issued by the user.
        zd1205_SetRatesInfo(macp);
        //zd_UpdateCardSetting(&(macp->cardSetting));
}
#if ZDCONF_WE_STAT_SUPPORT == 1
struct iw_statistics *
                        zd1205_iw_getstats(struct net_device *dev)
{
        struct zd1205_private *macp = (struct zd1205_private *)dev->priv;

        macp->iwstats.discard.fragment = macp->ArAgedCnt
                                         + macp->ArFreeFailCnt;

        macp->iwstats.discard.retries = macp->retryFailCnt;
        macp->iwstats.discard.misc = macp->invalid_frame_good_crc
                                     + macp->rxDupCnt;

        return &macp->iwstats;

}
#elif !defined(ZDCONF_WE_STAT_SUPPORT)
	#error "Undefine ZDCONF_WE_STAT_SUPPORT"
#endif
