/*
 * linux/arch/arm/mach-oxnas/gmac.h
 *
 * Copyright (C) 2005 Oxford Semiconductor Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#if !defined(__GMAC_H__)
#define __GMAC_H__

#include <asm/semaphore.h>
#include <asm/types.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/list.h>
#include <linux/ethtool.h>
#include <asm/arch/desc_alloc.h>
#include <asm/arch/dma.h>
#ifdef CONFIG_LEON_COPRO
#include <asm/arch/leon.h>
#include "gmac_offload.h"
#endif // CONFIG_LEON_COPRO

#ifdef GMAC_DEBUG
#define DBG(n, args...)\
    do {\
        if ((n) <= priv->msg_level)\
            printk(args);\
    } while (0)
#else
#define DBG(n, args...)   do { } while(0)
#endif

#define MS_TO_JIFFIES(x) (((x) < (1000/(HZ))) ? 1 : (x) * (HZ) / 1000)

//#define ENABLE_TX_OFFLOAD
//#define ENABLE_TX_TSO_OFFLOAD
//#define ENABLE_RX_OFFLOAD
#ifdef ENABLE_RX_OFFLOAD
#define USE_RX_NAPI
#endif // ENABLE_RX_OFFLOAD

//#define ARMULATING

typedef struct gmac_desc_list_info {
    volatile gmac_dma_desc_t *base_ptr;
    int                       num_descriptors;
    int                       empty_count;
    int                       full_count;
    int                       r_index;
    int                       w_index;
} gmac_desc_list_info_t;

typedef struct gmac_sram_buffer_info {
    unsigned                      number_;
    unsigned char                *v_buffer_;
    dma_addr_t                    p_buffer_;
    unsigned long                 buffer_size_;
    unsigned long                 packet_length_;
    struct gmac_sram_buffer_info *next_;
} gmac_sram_buffer_info_t;

#ifdef ENABLE_TX_OFFLOAD
typedef struct tx_offload_job {
    struct sk_buff   *skb_;
    unsigned long     number_;
    struct list_head  list_;
} tx_offload_job_t;
#endif // ENABLE_TX_OFFLOAD

#ifdef ENABLE_RX_OFFLOAD
typedef struct rx_offload_job {
    unsigned                 sram_info_entry_count_;
    u32                      packet_length_;
    int                      desc_status_;
    unsigned long            number_;
    struct list_head         list_;
    gmac_sram_buffer_info_t *sram_info_[1];
} rx_offload_job_t;
#endif //ENABLE_RX_OFFLOAD

#ifdef CONFIG_LEON_COPRO
typedef struct copro_params {
    u32 cmd_que_head_;
    u32 cmd_que_tail_;
    u32 fwd_intrs_mailbox_;
    u32 tx_que_head_;
    u32 tx_que_tail_;
    u32 free_start_;
    u32 mtu_;
    u32 rx_mitigation_;
    u32 rx_mitigation_frames_;
    u32 rx_mitigation_usec_;
} __attribute ((packed)) copro_params_t;
#endif // CONFIG_LEON_COPRO

// Private data structure for the GMAC driver
typedef struct gmac_priv {
    /** Base address of GMAC MAC registers */
    u32                       macBase;
    /** Base address of GMAC DMA registers */
    u32                       dmaBase;

    struct net_device*        netdev;

    struct net_device_stats   stats;

    u32                       msg_level;

    /** Whether we own an IRQ */
    int                       have_irq;

    /** Pointer to outstanding tx packet that has not yet been queued due to
     *  lack of descriptors */
    struct sk_buff           *tx_pending_skb;
    gmac_sram_buffer_info_t **tx_pending_sram_info;
    /** DMA consistent physical address of outstanding tx packet */
    dma_addr_t                tx_pending_dma_addr;
    unsigned long             tx_pending_length;

    /** To synchronise ISR and thread TX activities' access to private data */
    spinlock_t                tx_spinlock_;

    /** To synchronise access to the PHY */
    spinlock_t                phy_lock;

    /** The timer for NAPI polling when out of memory when trying to fill RX
     *  descriptor ring */

    /** PHY related info */
    struct mii_if_info        mii;
    struct ethtool_cmd        ethtool_cmd;
    u32                       phy_addr;
    u32                       phy_type;
    int                       gmii_csr_clk_range;

    /** Periodic timer to check link status etc */
    struct timer_list         watchdog_timer;
    volatile int              watchdog_timer_shutdown;

    /** Flags controlling h/w acceleration features */
    int                       rx_csum;
    int                       wake_on_lan;

    /** The number of descriptors in the gmac_dma_desc_t array holding both the TX and
     *  RX descriptors. The TX descriptors reside at the start of the array */
    unsigned                  total_num_descriptors;
    /** The CPU accessible virtual address of the start of the descriptor array */
    gmac_dma_desc_t*          desc_vaddr;
    /** The hardware accessible physical address of the start of the descriptor array */
    dma_addr_t                desc_dma_addr;

    /** Descriptor list management */
    gmac_desc_list_info_t     tx_gmac_desc_list_info;
    gmac_desc_list_info_t     rx_gmac_desc_list_info;

    /** Record of disabling RX overflow interrupts */
    unsigned                  rx_overflow_ints_disabled;

#if defined(ENABLE_TX_OFFLOAD) || defined(ENABLE_RX_OFFLOAD)
    /** SRAM network buffers allocation management */
    gmac_sram_buffer_info_t  *sram_buffer_infos_;
    gmac_sram_buffer_info_t  *sram_buffer_free_list_;
    unsigned                  sram_buffer_free_count_;
    spinlock_t                sram_buffer_spinlock_;
    struct semaphore          sram_buffer_sem_;
#endif // defined(ENABLE_TX_OFFLOAD) || defined(ENABLE_RX_OFFLOAD)

#ifdef ENABLE_TX_OFFLOAD
    /** TX offload management */
    struct list_head             tx_offload_list_;
    spinlock_t                   tx_offload_lock_;
    struct workqueue_struct     *tx_work_queue_;
    struct work_struct           tx_offload_work_;
    struct semaphore             tx_offload_semaphore_;
    unsigned                     tx_job_number_;
    volatile int                 tx_work_queue_stop_flag_;
    struct semaphore             tx_work_queue_stop_semaphore_;
    oxnas_dma_callback_status_t  tx_dma_status_;
#ifdef ENABLE_TX_TSO_OFFLOAD
    struct semaphore             tso_netif_resumed_semaphore_;
    u32                          tso_csum_fixed_dst_;
#endif // ENABLE_TX_TSO_OFFLOAD
#endif // ENABLE_TX_OFFLOAD

#ifdef ENABLE_RX_OFFLOAD
    /** RX offload management */
    struct list_head             rx_offload_list_;
    spinlock_t                   rx_offload_lock_;
    struct workqueue_struct     *rx_work_queue_;
    struct work_struct           rx_offload_work_;
    struct semaphore             rx_offload_semaphore_;
    unsigned                     rx_job_number_;
    volatile int                 rx_work_queue_stop_flag_;
    struct semaphore             rx_work_queue_stop_semaphore_;
    oxnas_dma_callback_status_t  rx_dma_status_;
#endif // ENABLE_RX_OFFLOAD

    /** The result of the last H/W DMA generated checksum operation */
    u16                       tx_csum_result_;

    /** Whether we deal in jumbo frames */
    int                       jumbo_;

    volatile int              mii_init_media;
    volatile int              phy_force_negotiation;

    /** To synchronise rx descriptor list accesses */
    spinlock_t                rx_spinlock_;

#ifdef CONFIG_LEON_COPRO
    /** DMA coherent memory for CoPro's parameter storage */
    copro_params_t  *shared_copro_params_;
    dma_addr_t       shared_copro_params_pa_;

    /** ARM's local CoPro parameter storage */
    copro_params_t   copro_params_;

    /** Queue for commands/acks to/from CoPro */
    int              copro_a_irq_;
    int              copro_a_irq_alloced_;
    int              copro_b_irq_;
    int              copro_b_irq_alloced_;
    cmd_que_t        cmd_queue_;
    tx_que_t         tx_queue_;
    int              copro_cmd_que_num_entries_;
    int              copro_tx_que_num_entries_;
	struct semaphore copro_stop_complete_semaphore_;
	struct list_head copro_tx_skb_list_;
	int				 copro_tx_skb_list_count_;
#endif // CONFIG_LEON_COPRO

    spinlock_t       cmd_que_lock_;
} gmac_priv_t;

#endif        //  #if !defined(__GMAC_H__)
