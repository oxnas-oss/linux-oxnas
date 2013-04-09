/*
 * linux/arch/arm/mach-oxnas/gmac.c
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

#include <linux/config.h>
#include <linux/crc32.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/in.h>
#include <net/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/irqs.h>

#ifdef CONFIG_LEON_COPRO
#include <asm/arch/leon-program.h>
#endif // CONFIG_LEON_COPRO

//#define GMAC_DEBUG
#undef GMAC_DEBUG

#include "gmac.h"
#include "gmac_ethtool.h"
#include "gmac_phy.h"
#include "gmac_desc.h"
#include "gmac_reg.h"

#define CORRECT_FRAG_IP_CSUM

#define MAX_GMAC_UNITS 1
#define ALLOW_PROBING
//#define INCLUDE_DUMP_REGS
//#define DUMP_REGS_ON_PROBE

#define ALLOW_AUTONEG
#define FORCE_HALF_DUPLEX   /* Only effective if auto-negotiation disabled */
#define ALLOW_1000_SUPPORT
//#define ALLOW_FULL_DUPLEX_AT_1000

//#define USE_RX_CSUM
#define USE_RX_NAPI

//#define TEST_COPRO
#define COPRO_RX_MITIGATION 0   /* No Rx mitigation in CoPro */
#define COPRO_RX_MITIGATION_FRAMES 5
#define COPRO_RX_MITIGATION_USECS 500

#define COPRO_TX_QUEUE_NUM_ENTRIES  4
#define COPRO_CMD_QUEUE_NUM_ENTRIES 6
#define COPRO_MAX_QUEUED_TX_SKBS    16

//#define FORCE_TSO_SEG_SIZE 504

#ifdef CONFIG_LEON_OFFLOAD_TX
#define NUM_RX_DMA_DESCRIPTORS  NUM_GMAC_DMA_DESCRIPTORS
#define NUM_TX_DMA_DESCRIPTORS  0
#else
#define NUM_RX_DMA_DESCRIPTORS (NUM_GMAC_DMA_DESCRIPTORS / 2)
#define NUM_TX_DMA_DESCRIPTORS (NUM_GMAC_DMA_DESCRIPTORS - NUM_RX_DMA_DESCRIPTORS)
#endif // CONFIG_LEON_OFFLOAD_TX

#if (((NUM_RX_DMA_DESCRIPTORS) + (NUM_TX_DMA_DESCRIPTORS)) > (NUM_GMAC_DMA_DESCRIPTORS))
#error "GMAC TX+RX descriptors exceed allocation"
#endif

// Default as the first allocated Oxsemi MAC addr
static const u8 DEFAULT_MAC_ADDRESS[] = { 0x00, 0x30, 0xe0, 0x00, 0x00, 0x00 };

static const u32 MAC_BASE_OFFSET = 0x0000;
static const u32 DMA_BASE_OFFSET = 0x1000;

static const int MIN_PACKET_SIZE = 68;
static const int NORMAL_PACKET_SIZE = 1500;
static const int MAX_JUMBO = 9000;

// 256 byte SRAM buffers -> can fit 96 into 28KB available after 8KB for DMA descs
#define SRAM_NET_BUFFER_SIZE_POW2 8
#define SRAM_BUFFER_SIZE         (1 << (SRAM_NET_BUFFER_SIZE_POW2))
#define SRAM_BUFFER_COUNT        ((SRAM_NET_BUFFER_AVAIL_MEM) >> (SRAM_NET_BUFFER_SIZE_POW2))

// Share SRAM buffers between Tx and Rx
#if defined(ENABLE_TX_OFFLOAD) || defined(ENABLE_TX_TSO_OFFLOAD)
#define RX_NUM_SRAM_BUFFERS_LEAVE_FREE (SRAM_BUFFER_COUNT >> 1)
#else // defined(ENABLE_TX_OFFLOAD) || defined(ENABLE_TX_TSO_OFFLOAD)
#define RX_NUM_SRAM_BUFFERS_LEAVE_FREE 0
#endif // defined(ENABLE_TX_OFFLOAD) || defined(ENABLE_TX_TSO_OFFLOAD)

static const int EXTRA_RX_SKB_SPACE = 32;   // Otherwise GMAC spans over >1 skb

// Max length when received data is copied to new skb
static const int ETHERNET_PACKET_COPY = 0;//250;

static const u32 AUTO_NEGOTIATION_WAIT_TIMEOUT_MS = 5000;

static const u32 NAPI_POLL_WEIGHT = 64;
static const u32 NAPI_OOM_POLL_INTERVAL_MS = 50;

static const int WATCHDOG_TIMER_INTERVAL = 500*HZ/1000;

#define AUTO_NEG_MS_WAIT 500
static const int AUTO_NEG_INTERVAL = (AUTO_NEG_MS_WAIT)*HZ/1000;
static const int START_RESET_INTERVAL = 50*HZ/1000;
static const int RESET_INTERVAL = 10*HZ/1000;

static const int GMAC_RESET_TIMEOUT_MS = 1000;

static const char *version_string = "Synopsys GMAC v1.0 Brian H. Clarke";

// Holders for the networking parameters settable from the kernel cmdline
static int irq;
static u32 base;
static u32 mac_hi;
static u32 mac_lo;

static int debug = 0;

static int __initdata gmac_found_count = 0;

/* Parse netdev kernel cmdline options */
static int __init do_setup(char *str)
{
    int i;
    int ints[5];    // Hold arg count and four args

    get_options(str, sizeof(ints)/sizeof(int), ints);
    for (i=1; i<=ints[0]; i++) {
        switch (i) {
            case 1:
                irq = ints[i];
                break;
            case 2:
                base = ints[i];
                break;
            case 3:
                mac_hi = ints[i];
                break;
            case 4:
                mac_lo = ints[i];
                break;
        }
    }
    return 0;
}
__setup("netdev=",do_setup);

#ifdef INCLUDE_DUMP_REGS
static void dump_mac_regs(u32 macBase, u32 dmaBase)
{
    int n = 0;

    for (n=0; n<0x60; n+=4) {
        DBG(1, KERN_INFO "MAC Register    %08x (%08x) = %08x\n", n, macBase+n, readl(macBase+n));
    }

    for (n=0; n<0x60; n+=4) {
        DBG(1, KERN_INFO "DMA Register    %08x (%08x) = %08x\n", n, dmaBase+n, readl(dmaBase+n));
    }
}
#endif // INCLUDE_DUMP_REGS

#ifdef ENABLE_TX_OFFLOAD
/**
 * Blocking acquisition of a SRAM buffer
 */
static gmac_sram_buffer_info_t* alloc_sram_buffer(gmac_priv_t* priv)
{
    gmac_sram_buffer_info_t* entry = 0;
    while (!entry) {
        unsigned long irq_flags;

        // Wait for a buffer to be available
        while (down_interruptible(&priv->sram_buffer_sem_));

        // Serialise while manipulating free list
        spin_lock_irqsave(&priv->sram_buffer_spinlock_, irq_flags);

        // It's an error if there isn't a buffer available at this point
        BUG_ON(!priv->sram_buffer_free_list_);

        // Unlink the head entry on the free list and return it to caller
        entry = priv->sram_buffer_free_list_;
        priv->sram_buffer_free_list_ = priv->sram_buffer_free_list_->next_;
        --priv->sram_buffer_free_count_;

        // Finished manipulating free list
        spin_unlock_irqrestore(&priv->sram_buffer_spinlock_, irq_flags);
    }
    return entry;
}
#endif // ENABLE_TX_OFFLOAD

#ifdef ENABLE_RX_OFFLOAD
/**
 * Non-blocking acquisition of a SRAM buffer
 */
static gmac_sram_buffer_info_t* try_alloc_sram_buffer(
    gmac_priv_t* priv,
    unsigned num_buffers_to_leave_free)
{
    gmac_sram_buffer_info_t *entry = 0;
    unsigned long            irq_flags;

    // Serialise while manipulating free list
    spin_lock_irqsave(&priv->sram_buffer_spinlock_, irq_flags);

    // Are there sufficient buffers available to leave the requested number free?
    if (priv->sram_buffer_free_list_ &&
        (priv->sram_buffer_free_count_ > num_buffers_to_leave_free)) {
        // Yes, so unlink the head entry on the free list and return it to caller
        entry = priv->sram_buffer_free_list_;
        priv->sram_buffer_free_list_ = priv->sram_buffer_free_list_->next_;
        --priv->sram_buffer_free_count_;
    }

    // Finished manipulating free list
    spin_unlock_irqrestore(&priv->sram_buffer_spinlock_, irq_flags);

    return entry;
}
#endif // ENABLE_RX_OFFLOAD

#if defined(ENABLE_TX_OFFLOAD) || defined(ENABLE_RX_OFFLOAD)
static void free_sram_buffer(
    gmac_priv_t* priv,
    gmac_sram_buffer_info_t* sram_info)
{
    // Serialise while manipulating free list
    unsigned long irq_flags;
    spin_lock_irqsave(&priv->sram_buffer_spinlock_, irq_flags);

    // Insert the freed buffer at the head of the free list
    sram_info->next_ = priv->sram_buffer_free_list_;
    priv->sram_buffer_free_list_ = sram_info;
    ++priv->sram_buffer_free_count_;

    // Finished manipulating free list
    spin_unlock_irqrestore(&priv->sram_buffer_spinlock_, irq_flags);

    // Make freed buffer available for allocation
    up(&priv->sram_buffer_sem_);
}

/**
 * Allocate fixed size SRAM network buffers for use by both Tx and Rx
 */
static int alloc_sram_network_buffers(struct net_device *dev)
{
    int status = 0;
    gmac_priv_t *priv = (gmac_priv_t*)netdev_priv(dev);

    // Allocate space for an array of SRAM net buffer information structures
    DBG(1, KERN_INFO "$GAllocating %lu SRAM buffers of %u bytes\n", SRAM_BUFFER_COUNT, SRAM_BUFFER_SIZE);
    priv->sram_buffer_infos_ = kmalloc(sizeof(gmac_sram_buffer_info_t) * SRAM_BUFFER_COUNT, GFP_KERNEL);
    if (!priv->sram_buffer_infos_)
    {
        DBG(1, KERN_ERR "alloc_sram_network_buffers() %s: Failed to allocate memory for SRAM buffer info structures\n", dev->name);
        status = -ENOMEM;
    } else {
        unsigned char *v_base = (unsigned char*)SRAM_NET_BUFFER_BASE_VA;
        dma_addr_t     p_base = SRAM_NET_BUFFER_BASE_PA;

        // Initialise SRAM buffer info structures and place on free list
        int i=0;
        for (; i < SRAM_BUFFER_COUNT; ++i) {
            gmac_sram_buffer_info_t *info = &priv->sram_buffer_infos_[i];

            info->number_        = i;
            info->v_buffer_      = v_base;
            info->p_buffer_      = p_base;
            info->buffer_size_   = SRAM_BUFFER_SIZE;
            info->packet_length_ = 0;
            free_sram_buffer(priv, info);

            v_base += SRAM_BUFFER_SIZE;
            p_base += SRAM_BUFFER_SIZE;
        }
    }

    return status;
}

/**
 * Should have ensured by other means that no SRAM network buffers are in use
 */
static void free_sram_network_buffers(struct net_device *dev)
{
    gmac_priv_t *priv = (gmac_priv_t*)netdev_priv(dev);

    // Zeroise the free list
    priv->sram_buffer_free_list_ = 0;
    priv->sram_buffer_free_count_ = 0;
    sema_init(&priv->sram_buffer_sem_, 0);

    // Free the buffer info structures
    if (priv->sram_buffer_infos_) {
        kfree(priv->sram_buffer_infos_);
        priv->sram_buffer_infos_ = 0;
    }
}
#endif // defined(ENABLE_TX_OFFLOAD) || defined(ENABLE_RX_OFFLOAD)

static void gmac_int_en_set(
    gmac_priv_t *priv,
    u32          mask)
{
    unsigned long irq_flags;

#ifdef CONFIG_LEON_COPRO
    spin_lock_irqsave(&priv->cmd_que_lock_, irq_flags);
    cmd_que_queue_cmd(&priv->cmd_queue_, GMAC_CMD_INT_EN_SET, mask, 0);
    spin_unlock_irqrestore(&priv->cmd_que_lock_, irq_flags);

    // Interrupt the CoPro so it sees the new command
    writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);
#else
    spin_lock_irqsave(&priv->cmd_que_lock_, irq_flags);
    dma_reg_set_mask(priv, DMA_INT_ENABLE_REG, mask);
    spin_unlock_irqrestore(&priv->cmd_que_lock_, irq_flags);
#endif // CONFIG_LEON_COPRO
}

static void gmac_int_en_set_isr(
    gmac_priv_t *priv,
    u32          mask)
{
#ifdef CONFIG_LEON_COPRO
    spin_lock(&priv->cmd_que_lock_);
    cmd_que_queue_cmd(&priv->cmd_queue_, GMAC_CMD_INT_EN_SET, mask, 0);
    spin_unlock(&priv->cmd_que_lock_);

    // Interrupt the CoPro so it sees the new command
    writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);
#else
    spin_lock(&priv->cmd_que_lock_);
    dma_reg_set_mask(priv, DMA_INT_ENABLE_REG, mask);
    spin_unlock(&priv->cmd_que_lock_);
#endif // CONFIG_LEON_COPRO
}

#ifdef CONFIG_LEON_COPRO
static struct semaphore copro_int_clr_semaphore;
static unsigned long    copro_int_clr_return;

static void copro_int_clr_callback(volatile gmac_cmd_que_ent_t* entry)
{
    copro_int_clr_return = entry->operand_;
    up(&copro_int_clr_semaphore);
}
#endif // CONFIG_LEON_COPRO

static void gmac_int_en_clr(
    gmac_priv_t *priv,
    u32          mask,
    u32         *new_value)
{
#ifdef CONFIG_LEON_COPRO
    unsigned long irq_flags=0;

    spin_lock_irqsave(&priv->cmd_que_lock_, irq_flags);
    cmd_que_queue_cmd(&priv->cmd_queue_, GMAC_CMD_INT_EN_CLR, mask, copro_int_clr_callback);
    spin_unlock_irqrestore(&priv->cmd_que_lock_, irq_flags);

    // Interrupt the CoPro so it sees the new command
    writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);

    if (new_value) {
        // Wait until the CoPro acknowledges that it has stopped
        down_interruptible(&copro_int_clr_semaphore);
        *new_value = copro_int_clr_return;
    }
#else
    unsigned long temp;
    unsigned long irq_flags=0;

    spin_lock_irqsave(&priv->cmd_que_lock_, irq_flags);
    temp = dma_reg_clear_mask(priv, DMA_INT_ENABLE_REG, mask);
    spin_unlock_irqrestore(&priv->cmd_que_lock_, irq_flags);

    if (new_value) {
        *new_value = temp;
    }
#endif
}

#ifndef CONFIG_LEON_COPRO
static void gmac_int_en_clr_isr(
    gmac_priv_t *priv,
    u32          mask)
{
    spin_lock(&priv->cmd_que_lock_);
    dma_reg_clear_mask(priv, DMA_INT_ENABLE_REG, mask);
    spin_unlock(&priv->cmd_que_lock_);
}
#endif // !CONFIG_LEON_COPRO

#ifdef USE_RX_NAPI
/**
 * @param skb A struct sk_buff* which if non-zero refers to an existing socket
 *            buffer that can be reused for filling the RX descriptor ring
 * @return An int which is non-zero if the rx descriptor ring is now full
 */
static int refill_rx_ring(
    struct net_device *dev,
    struct sk_buff    *skb,
    int                reenable_ru_intrs)
{
    gmac_priv_t *priv = (gmac_priv_t*)netdev_priv(dev);
    int ring_full;

DBG(1, KERN_INFO "$Crefill_rx_ring() %s: Entered, skb = 0x%08x\n", dev->name, (u32)skb);
    // While there are empty RX descriptor ring slots
    while (1) {
        int available;
        int desc;
#ifdef ENABLE_RX_OFFLOAD
        gmac_sram_buffer_info_t* sram_info;
#else // ENABLE_RX_OFFLOAD
        u32 dma_length;
        dma_addr_t dma_address;
#endif // ENABLE_RX_OFFLOAD

        spin_lock_bh(&priv->rx_spinlock_);
        available = available_for_write(&priv->rx_gmac_desc_list_info);
        spin_unlock_bh(&priv->rx_spinlock_);
        if (!available) {
            break;
        }

#ifdef ENABLE_RX_OFFLOAD
        // Refill the Rx descriptor ring using fixed-size SRAM buffers
        sram_info = try_alloc_sram_buffer(priv, RX_NUM_SRAM_BUFFERS_LEAVE_FREE);
        if (!sram_info) {
DBG(1, KERN_INFO "refill_rx_ring() %s: No more SRAM buffers available\n", dev->name);
            break;
        }
        spin_lock_bh(&priv->rx_spinlock_);
        desc = set_rx_descriptor(priv, 0, 0, 0, sram_info);
        spin_unlock_bh(&priv->rx_spinlock_);
        if (desc < 0) {
DBG(2, KERN_INFO "refill_rx_ring() %s: Cannot set rx descriptor for sram buffer, rx ring full\n", dev->name);
            free_sram_buffer(priv, sram_info);
            break;
        }
#else // ENABLE_RX_OFFLOAD
        if (!skb) {
            // Allocate a new skb for the descriptor ring which is large
            // enough for any packet received from the link
            DBG(10, "$B[RF A]$n");
            skb = dev_alloc_skb(dev->mtu + NET_IP_ALIGN + EXTRA_RX_SKB_SPACE);
            if (!skb) {
                DBG(1, KERN_WARNING "refill_rx_ring() %s: No memory for replacement skb\n", dev->name);
                // Can't refill any more RX descriptor ring entries
                break;
            } else {
DBG(1, KERN_INFO "$Crefill_rx_ring() %s: Allocated skb\n", dev->name);
                // Despite what the comments in the original code from
                // Synopsys claimed, the GMAC DMA can cope with non-quad
                // aligned buffers - it will always perform quad transfers
                // but zero/ignore the unwanted bytes.
                skb_reserve(skb, NET_IP_ALIGN);
            }
        } else {
            DBG(2, KERN_INFO "$Crefill_rx_ring() %s: Re-using skb\n", dev->name);
        }

        // Get a consistent DMA mapping for the memory to be DMAed to - causing
        // an invalidation of any entries in the CPU's cache covering the memory
        // region
        dma_length = skb_tailroom(skb);
        dma_address = dma_map_single(0, skb->tail, dma_length, DMA_FROM_DEVICE);
        BUG_ON(dma_mapping_error(dma_address));

        // Associate the skb with the descriptor
        spin_lock_bh(&priv->rx_spinlock_);
        desc = set_rx_descriptor(priv, dma_address, dma_length, skb, 0);
        spin_unlock_bh(&priv->rx_spinlock_);
        if (desc >= 0) {
DBG(2, KERN_INFO "$Crefill_rx_ring() %s: Set rx descriptor %d for skb 0x%08x, len = %u, skb->tail = 0x%08x\n", dev->name, desc, (u32)skb, dma_length, (u32)skb->tail);
            // Socket buffer now attached to receive descriptor, so don't want
            // to free it in this routine
            skb = 0;
        } else {
DBG(2, KERN_INFO "refill_rx_ring() %s: Cannot set rx descriptor for skb 0x%08x, rx ring full\n", dev->name, (u32)skb);
            // No, so release the DMA mapping for the socket buffer
            dma_unmap_single(0, dma_address, dma_length, DMA_FROM_DEVICE);

            // Free the socket buffer
            DBG(10, "$B[RF F]$n");
            dev_kfree_skb(skb);
            skb = 0;

            // No more RX descriptor ring entries to refill
            break;
        }
#endif // ENABLE_RX_OFFLOAD
    }

    // Ensure that if we were given an skb and didn't use it, we free it now
    if (skb) {
        DBG(10, "$B[RF F]$n");
        dev_kfree_skb(skb);
    }

    // Return ring fill status
    spin_lock_bh(&priv->rx_spinlock_);
    ring_full = !available_for_write(&priv->rx_gmac_desc_list_info);
    spin_unlock_bh(&priv->rx_spinlock_);
    if (ring_full && reenable_ru_intrs) {
        // Enable RX overflow reporting, as there are now RX descriptors queued
DBG(3, "$Yrefill_rx_ring() %s: Enabling RX unavailable interrupts\n", dev->name);
        gmac_int_en_set(priv, (1UL << DMA_INT_ENABLE_RU_BIT));
    }

    return ring_full;
}
#else // USE_RX_NAPI
/**
 * @param skb A struct sk_buff* which if non-zero refers to an existing socket
 *            buffer that can be reused for filling the RX descriptor ring
 * @return An int which is non-zero if the rx descriptor ring is now full
 */
static int refill_rx_ring(
    struct net_device *dev,
    struct sk_buff    *skb)
{
    gmac_priv_t *priv = (gmac_priv_t*)netdev_priv(dev);
    int available;

    // While there are empty RX descriptor ring slots
    while (1) {
        int desc;
        u32 dma_length;
        dma_addr_t dma_address;

        spin_lock(&priv->rx_spinlock_);
        available = available_for_write(&priv->rx_gmac_desc_list_info);
        spin_unlock(&priv->rx_spinlock_);
        if (!available) {
            break;
        }

        if (!skb) {
            // Allocate a new skb for the descriptor ring which is large enough
            // for any packet received from the link
            skb = dev_alloc_skb(dev->mtu + NET_IP_ALIGN + EXTRA_RX_SKB_SPACE);
            if (!skb) {
                // Can't refill any more RX descriptor ring entries
                break;
            } else {
                // Despite what the comments in the original code from Synopsys
                // claimed, the GMAC DMA can cope with non-quad aligned buffers
                // - it will always perform quad transfers but zero/ignore the
                // unwanted bytes.
                skb_reserve(skb, NET_IP_ALIGN);
            }
        }

        // Get a consistent DMA mapping for the memory to be DMAed to - causing
        // an invalidation of any entries in the CPU's cache covering the memory
        // region
        dma_length = skb_tailroom(skb);
        dma_address = dma_map_single(0, skb->tail, dma_length, DMA_FROM_DEVICE);
        BUG_ON(dma_mapping_error(dma_address));

        // Associate the skb with the descriptor
        spin_lock(&priv->rx_spinlock_);
        desc = set_rx_descriptor(priv, dma_address, dma_length, skb, 0);
        spin_unlock(&priv->rx_spinlock_);
        if (desc >= 0) {
            // Socket buffer now attached to receive descriptor, so don't want
            // to free it in this routine
            skb = 0;
        } else {
            // No, so release the DMA mapping for the socket buffer
            dma_unmap_single(0, dma_address, dma_length, DMA_FROM_DEVICE);

            // Free the socket buffer
            dev_kfree_skb(skb);
            skb = 0;

            // No more RX descriptor ring entries to refill
            break;
        }
    }

    // Ensure that if we were given an skb and didn't use it, we free it now
    if (skb) {
        dev_kfree_skb(skb);
    }

    spin_lock(&priv->rx_spinlock_);
    available = available_for_write(&priv->rx_gmac_desc_list_info);
    spin_unlock(&priv->rx_spinlock_);

    return !available;
}
#endif // USE_RX_NAPI

static void start_watchdog_timer(gmac_priv_t* priv)
{
    priv->watchdog_timer.expires = jiffies + WATCHDOG_TIMER_INTERVAL;
    priv->watchdog_timer_shutdown = 0;
    mod_timer(&priv->watchdog_timer, priv->watchdog_timer.expires);
}

static void delete_watchdog_timer(gmac_priv_t* priv)
{
    // Ensure link/PHY state watchdog timer won't be invoked again
    priv->watchdog_timer_shutdown = 1;
    del_timer_sync(&priv->watchdog_timer);
}

static inline int is_auto_negotiation_in_progress(gmac_priv_t* priv)
{
    return !(phy_read(priv->netdev, priv->phy_addr, MII_BMSR) & BMSR_ANEGCOMPLETE);
}

static void watchdog_timer_action(unsigned long arg)
{
    typedef enum watchdog_state {
        WDS_IDLE,
        WDS_RESETTING,
        WDS_NEGOTIATING
    } watchdog_state_t;

    static int state = WDS_IDLE;

    gmac_priv_t* priv = (gmac_priv_t*)arg;
    unsigned long new_timeout = jiffies + WATCHDOG_TIMER_INTERVAL;
#ifndef ARMULATING
    int duplex_changed;
    int gigabit_changed;
    int not_ready;

    // Interpret the PHY/link state.
//printk("watchdog_timer_action() mii_init_media = %d, state = %d\n", priv->mii_init_media, state);
    duplex_changed = mii_check_media_ex(&priv->mii, 1, priv->mii_init_media, &gigabit_changed);
    priv->mii_init_media = 0;
//printk("watchdog_timer_action() duplex_changed = %d, gigabit_changed = %d,\n", duplex_changed, gigabit_changed);

    not_ready = priv->phy_force_negotiation ||
                !netif_carrier_ok(priv->netdev) ||
                (state == WDS_RESETTING);

    if (not_ready) {
//printk("watchdog_timer_action() Not ready: phy_force_negotiation = %d carrier %s, state = %d\n", priv->phy_force_negotiation, netif_carrier_ok(priv->netdev) ? "up" : "down", state);

        if (priv->phy_force_negotiation) {
            if (netif_carrier_ok(priv->netdev)) {
//printk("watchdog_timer_action() Forcing to WDS_RESETTING\n");
                state = WDS_RESETTING;
            } else {
//printk("watchdog_timer_action() Forcing to WDS_IDLE\n");
                state = WDS_IDLE;
            }

            priv->phy_force_negotiation = 0;
        }

        // May be a good idea to restart everything here, in an attempt to clear
        // out any fault conditions
        if ((state == WDS_NEGOTIATING) && is_auto_negotiation_in_progress(priv)) {
//printk("watchdog_timer_action() Extending negotiation interval\n");
            new_timeout = jiffies + AUTO_NEG_INTERVAL;
        } else {
            switch (state) {
                case WDS_IDLE:
//printk("watchdog_timer_action() IDLE -> reset PHY\n");
                    // Reset the PHY to get it into a known state
                    start_phy_reset(priv);
                    new_timeout = jiffies + START_RESET_INTERVAL;
                    state = WDS_RESETTING;
                    break;
                case WDS_RESETTING:
                    if (!is_phy_reset_complete(priv)) {
//printk("watchdog_timer_action() RESETTING not complete\n");
                        new_timeout = jiffies + RESET_INTERVAL;
                    } else {
//printk("watchdog_timer_action() RESETTING complete, start negotiation\n");
                        // Force or auto-negotiate PHY mode
                        set_phy_mode(priv->netdev);

                        state = WDS_NEGOTIATING;
                        new_timeout = jiffies + AUTO_NEG_INTERVAL;
                    }
                    break;
                default:
                    DBG(1, KERN_ERR "watchdog_timer_action() %s: Unexpected state\n", priv->netdev->name);
                    state = WDS_IDLE;
                    break;
            }
        }
    } else {
//printk("watchdog_timer_action() carrier up\n");
        state = WDS_IDLE;
        if (duplex_changed) {
DBG(15, KERN_INFO "watchdog_timer_action() Changing duplex to %s\n", priv->mii.full_duplex ? "full" : "half");
            priv->mii.full_duplex ? mac_reg_set_mask(priv,   MAC_CONFIG_REG, (1UL << MAC_CONFIG_DM_BIT)) :
                                    mac_reg_clear_mask(priv, MAC_CONFIG_REG, (1UL << MAC_CONFIG_DM_BIT));
        }

        if (gigabit_changed) {
            // Mask to extract the transmit status field from the status register
            u32 ts_mask = ((1UL << DMA_STATUS_TS_NUM_BITS) - 1) << DMA_STATUS_TS_BIT;

            // Must stop transmission in order to change store&forward mode
            dma_reg_clear_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_ST_BIT));

            // Transmission only stops after current Tx frame has completed
            // transmission, so wait for the Tx state machine to enter the
            // stopped state
            while ((dma_reg_read(priv, DMA_STATUS_REG) & ts_mask) != DMA_STATUS_TS_STOPPED);

            if (priv->mii.using_1000) {
DBG(15, KERN_INFO "watchdog_timer_action() Clearing MAC PS bit and setting DMA SF bit\n");
                mac_reg_clear_mask(priv, MAC_CONFIG_REG, (1UL << MAC_CONFIG_PS_BIT));
                dma_reg_set_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_SF_BIT));
            } else {
DBG(15, KERN_INFO "watchdog_timer_action() Setting MAC PS bit and clearing DMA SF bit\n");
                mac_reg_set_mask(priv, MAC_CONFIG_REG, (1UL << MAC_CONFIG_PS_BIT));
                dma_reg_clear_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_SF_BIT));
            }

            // Re-start transmission after store&forward change applied
            dma_reg_set_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_ST_BIT));
        }
    }
#endif // ARMULATING

    // Re-trigger the timer, unless some other thread has requested it be stopped
    if (!priv->watchdog_timer_shutdown) {
        // Restart the timer
        mod_timer(&priv->watchdog_timer, new_timeout);
    }
}

#ifdef ENABLE_RX_OFFLOAD
static void rx_offload_dma_callback(
    oxnas_dma_channel_t         *channel,
    oxnas_callback_arg_t         arg,
    oxnas_dma_callback_status_t  status,
    u16                          checksum,
    int                          interrupt_count)
{
    gmac_priv_t *priv = (gmac_priv_t*)arg;

    DBG(1, KERN_INFO "rx_offload_dma_callback() Called, channel = %u, status = %d\n", channel->channel_number_, status);
    priv->tx_dma_status_ = status;
    up(&priv->rx_offload_semaphore_);
}
#endif // ENABLE_RX_OFFLOAD

static struct sk_buff* process_rx_packet(
    gmac_priv_t    *priv,
    int             packet_len,
    u32             desc_status,
    struct sk_buff *skb)
{
    // Is the packet entirely contained within the desciptors and
    // without errors?
    int valid = is_rx_valid(desc_status);

    // Are we offloading RX checksuming?
    if (valid && priv->rx_csum) {
        // Determine whether Ethernet frame contains an IP packet -
        // only bother with Ethernet II frames, but do cope with
        // 802.1Q VLAN tag presence
        int vlan_offset = 0;
        unsigned short eth_protocol = ntohs(((struct ethhdr*)skb->data)->h_proto);
        int is_ip_packet = (eth_protocol == ETH_P_IP);
DBG(3, KERN_INFO "RX csum offload: Is packet IP?\n");
        if (!is_ip_packet) {
DBG(3, KERN_INFO "RX csum offload: Is packet VLAN?\n");
            // Check for VLAN tag
            if (eth_protocol == ETH_P_8021Q) {
                // Extract the contained protocol type from after
                // the VLAN tag
DBG(3, KERN_INFO "RX csum offload:   packet is VLAN\n");
                eth_protocol = ntohs(*(unsigned short*)(skb->data + ETH_HLEN));
                is_ip_packet = (eth_protocol == ETH_P_IP);

                // Adjustment required to skip the VLAN stuff and
                // get to the IP header
                vlan_offset = 4;
            }
        }

        // Only offload checksum calculation for IP packets
        if (is_ip_packet) {
            u16 payload_length = 0;
            struct iphdr* ip_header = (struct iphdr*)(skb->data + ETH_HLEN + vlan_offset);

            // Because we don't have all the fragments, we cannot compute the
            // checksum here, let the network stack reassemble the fragments
            // then calculate the checksum
            if (ip_header->frag_off & htons(IP_MF|IP_OFFSET))
                goto good_receive;

            switch (ip_header->protocol) {
                case IPPROTO_TCP:
                    // Compute TCP pseudo-header checksum
                    payload_length = ntohs(ip_header->tot_len) - (ip_header->ihl*4);
                    break;
                case IPPROTO_UDP:
                    {
                        struct udphdr* udp_header = (struct udphdr*)((u8*)ip_header + (ip_header->ihl*4));
                        payload_length = ntohs(udp_header->len);
                    }
                    break;
                default:
                    // Not supporting any other than TCP/UDP
                    break;
            }

            if (payload_length) {
                // Get the hardware generated payload checksum from
                // the end of the received packet, reverse the 1's
                // complement operation that the h/w applies and add
                // to the pseudo-header checksum, in network order
                u16 hw_csum = ~(*(u16*)(skb->data + packet_len - 2));

                // Calculate checksum of pseudo header and payload
                if (csum_tcpudp_magic(
                        ip_header->saddr,
                        ip_header->daddr,
                        payload_length,
                        ip_header->protocol,
                        hw_csum)) {
                    // Bad checksum, so indicate in descriptor status
                    desc_status |= (1UL << RDES0_IPC_BIT);
                    valid = 0;
DBG(20, KERN_INFO "RX csum offload: Bad checksum\n");
                } else {
                    skb->ip_summed = CHECKSUM_UNNECESSARY;
DBG(20, KERN_INFO "RX csum offload: Good checksum\n");
                }
            }
        }
    }

good_receive:
    // Process good packets only
    if (valid) {
        struct sk_buff *newskb = 0;
        int skb_too_small = 0;

        // Check for oversized packet, which should never occur but
        // have seen in long soak testing so need help working out
        // where it's coming from
        if (unlikely(packet_len > (skb->end - skb->tail))) {
            printk(KERN_WARNING "process_rx_packet() %s: Oversized packet of length = %u, buffer len = %u\n", priv->netdev->name, packet_len, skb->end - skb->tail);
            skb_too_small = 1;
        }

        if (skb_too_small || (packet_len <= ETHERNET_PACKET_COPY)) {
            // Allocate a new skb for the small or oversized packet
            DBG(10, "$B[PL A]$n");
            newskb = dev_alloc_skb(packet_len + NET_IP_ALIGN + EXTRA_RX_SKB_SPACE);
            if (!newskb) {
                // Failed to allocate socket buffer, so just
                // continue to use the original socket buffer - this
                // will result in a kernel panic if the allocation
                // was due to the packet being oversized
                DBG(1, KERN_WARNING "process_rx_packet() %s: No memory for newskb, using original one\n", priv->netdev->name);
            } else {
                unsigned char *data;

                // Despite what the comments in the original
                // code from Synopsys claimed, the GMAC DMA can
                // cope with non-quad aligned buffers - it will
                // always perform quad transfers but zero/ignore
                // the unwanted bytes.
                skb_reserve(newskb, NET_IP_ALIGN);

                // Record the space in the new, small, skb that
                // will be occupied by the RX packet
                data = skb_put(newskb, packet_len);

                // Copy the small packet into the new skb
                memcpy(data, skb->data, packet_len);

                // Set the device for the new small skb
                newskb->dev = priv->netdev;

                // Set packet protocol
                newskb->protocol = eth_type_trans(newskb, priv->netdev);
            }
        }

        if (!newskb) {
            // Will send the original, large, skb up the receive
            // stack
            newskb = skb;

            // Can't reuse the skb for filling the RX descriptor
            // ring
            skb = 0;

            // Increase the skb's data pointer to account for
            // the RX packet that has been already DMAed into it
            skb_put(newskb, packet_len);

            // Set the device for the skb
            newskb->dev = priv->netdev;

            // Set packet protocol
            newskb->protocol = eth_type_trans(newskb, priv->netdev);
        }

        DBG(20, "$G[0x%08x is %u bytes]$n", (u32)newskb->head, packet_len);
#ifdef ENABLE_RX_OFFLOAD
        // Protect against lock recursion with softirq networking stuff, as this
        // will be called from worker thread when prototype offload enabled, due
        // to having to sleep until DMA complete
        local_bh_disable();
#endif // ENABLE_RX_OFFLOAD

        // Send the packet up protocol stack, cope with ISR and NAPI cases
        in_irq() ? netif_rx(newskb) : netif_receive_skb(newskb);

#ifdef ENABLE_RX_OFFLOAD
        local_bh_enable();
#endif // ENABLE_RX_OFFLOAD

        // Update receive statistics
        priv->netdev->last_rx = jiffies;
        ++priv->stats.rx_packets;
        priv->stats.rx_bytes += packet_len;
    } else {
        DBG(2, KERN_WARNING "$Rprocess_rx_packet() %s: Received packet has bad desc_status = 0x%08x\n", priv->netdev->name, desc_status);
        ++priv->stats.rx_errors;

        // Update receive statistics from the descriptor status
        if (is_rx_collision_error(desc_status)) {
            DBG(20, KERN_INFO "$Yprocess_rx_packet() %s: Collision error (0x%08x:%u bytes)\n", priv->netdev->name, desc_status, packet_len);
            ++priv->stats.collisions;
        }
        if (is_rx_crc_error(desc_status)) {
            DBG(20, KERN_INFO "$Yprocess_rx_packet() %s: CRC error (0x%08x:%u bytes)\n", priv->netdev->name, desc_status, packet_len);
            ++priv->stats.rx_crc_errors;
        }
        if (is_rx_frame_error(desc_status)) {
            DBG(20, KERN_INFO "$Yprocess_rx_packet() %s: frame error (0x%08x:%u bytes)\n", priv->netdev->name, desc_status, packet_len);
            ++priv->stats.rx_frame_errors;
        }
        if (is_rx_length_error(desc_status)) {
            DBG(20, KERN_INFO "$Yprocess_rx_packet() %s: Length error (0x%08x:%u bytes)\n", priv->netdev->name, desc_status, packet_len);
            ++priv->stats.rx_length_errors;
        }
        if (is_rx_csum_error(desc_status)) {
            DBG(20, KERN_INFO "$Yprocess_rx_packet() %s: Checksum error (0x%08x:%u bytes)\n", priv->netdev->name, desc_status, packet_len);
            ++priv->stats.rx_frame_errors;
        }
    }

    return skb;
}

#ifdef USE_RX_NAPI
/*
 * NAPI receive polling method
 */
static int poll(struct net_device *dev, int* budget)
{
    gmac_priv_t *priv = (gmac_priv_t*)netdev_priv(dev);
    int rx_work_limit = dev->quota;
    int received = 0;
    int continue_polling;
    int finished;
    int available;

DBG(2, KERN_INFO "$Gpoll() %s: quota = %u, budget = %u\n", dev->name, dev->quota, *budget);
    finished = 0;
    do {
        u32 status;

        // While there are receive polling jobs to be done
#ifdef ENABLE_RX_OFFLOAD
        while (rx_work_limit) {
            dma_addr_t dma_address;
            u32 dma_length;
            u32 desc_status;
            int desc;
            rx_offload_job_t *rx_offload_job = 0;

            // Do explicit check for available Rx descriptor, as we want to
            // allocate Rx job, along with contained sram_info ptr array, before
            // call the get the Rx descriptor
            spin_lock(&priv->rx_spinlock_);
            available = rx_available_for_read(&priv->rx_gmac_desc_list_info, 1);
            spin_unlock(&priv->rx_spinlock_);

            if (!available) {
                break;
            }

            // How large a sram_info ptr array we want
            dma_length = SRAM_BUFFER_COUNT;

            // Allocate a Rx job containing sufficient space for the required
            // size of sram_info ptr array.
            rx_offload_job = (rx_offload_job_t*)kmalloc(
                sizeof(rx_offload_job_t) + (sizeof(gmac_sram_buffer_info_t*) * (dma_length-1)), GFP_ATOMIC);

            BUG_ON(!rx_offload_job);

            // Should now be guarenteed there's a Rx descriptor available, as we
            // tested the list above
            spin_lock(&priv->rx_spinlock_);
            desc = get_rx_descriptor(priv, &desc_status, &dma_address, &dma_length, 0, rx_offload_job->sram_info_);
            spin_unlock(&priv->rx_spinlock_);

            if (desc < 0) {
                printk(KERN_WARNING "$Ypoll() No Rx descriptor obtained, although Rx descriptor list has entries available\n");
                kfree(rx_offload_job);
                break;
            } else if (!dma_address) {
                panic("poll() %s: Found RX descriptor with zero length descriptor\n", dev->name);
            } else {
                // Initialise the job description structure with the details of
                // the SRAM buffers associated with the RX descriptor
                rx_offload_job->sram_info_entry_count_ = dma_length;
                rx_offload_job->packet_length_         = dma_address;
                rx_offload_job->desc_status_           = desc_status;

DBG(28, KERN_INFO "$Bpoll() %s: Have sram_info, length = %d, desc = %d\n", dev->name, dma_address, desc);
//printk("$B%d:%u:%u:0x%08x\n", desc, dma_length, dma_address, desc_status);
                INIT_LIST_HEAD(&rx_offload_job->list_);

                // Add RX offload description to queue of offload jobs
                spin_lock(&priv->rx_offload_lock_);
                // Maintain number for debugging
                rx_offload_job->number_ = priv->rx_job_number_++;
                // Insert the new entry in FIFO queue
                list_add_tail(&rx_offload_job->list_, &priv->rx_offload_list_);
                spin_unlock(&priv->rx_offload_lock_);

DBG(1, KERN_INFO "$GQueued job %lu\n", rx_offload_job->number_);
                // Schedule the rx offload job handler for execution on a kernel worker thread
                queue_work(priv->rx_work_queue_, &priv->rx_offload_work_);
            }
#else // ENABLE_RX_OFFLOAD
        while (rx_work_limit) {
            dma_addr_t dma_address;
            u32 dma_length;
            u32 desc_status;
            int desc;
            struct sk_buff* skb = 0;

            spin_lock(&priv->rx_spinlock_);
            desc = get_rx_descriptor(priv, &desc_status, &dma_address, &dma_length, &skb, 0);
            spin_unlock(&priv->rx_spinlock_);

            if (desc < 0) {
                break;
            } else if (!skb) {
                panic("poll() %s: Found RX descriptor without attached skb\n", dev->name);
            } else {
                int packet_len = get_rx_length(desc_status);

DBG(28, KERN_INFO "$Bpoll() %s: Got rx descriptor %d for skb 0x%08x, desc_status = 0x%08x, length = %d\n", dev->name, desc, (u32)skb, desc_status, packet_len);
                // Release the DMA mapping for the received data
                dma_unmap_single(0, dma_address, dma_length, DMA_FROM_DEVICE);

                // Do all receive processing for the packet
                skb = process_rx_packet(priv, packet_len, desc_status, skb);

                // If we have a socket buffer available and there is room to
                // queue a new RX descriptor
                if (skb) {
                    spin_lock(&priv->rx_spinlock_);
                    available = available_for_write(&priv->rx_gmac_desc_list_info);
                    spin_unlock(&priv->rx_spinlock_);

                    if (available) {
                        // Make use of the available socket buffer by
                        // attempting to fill all available slots in the RX
                        // descriptor ring
DBG(2, KERN_INFO "poll() %s: Have skb and desc not empty, so calling refill_rx_ring()\n", dev->name);
                        refill_rx_ring(dev, skb, 1);
                    } else {
                        // Free the socket buffer, as we couldn't make use of it
                        // to refill the RX descriptor ring
                        DBG(10, "$B[PL F]$n");
                        dev_kfree_skb(skb);
                    }
                }
            }
#endif // ENABLE_RX_OFFLOAD

            // Increment count of processed packets
            ++received;

            // Decrement our remaining quota
            if (rx_work_limit > 0) {
                --rx_work_limit;
            }
        }

        if (rx_work_limit) {
            // We have unused quota remaining, but apparently no Rx packets to
            // process
            available = 0;

            // Clear any RI status so we don't immediately get reinterrupted
            // when we leave polling, due to either a new RI event, or a left
            // over interrupt from one of the RX descriptors we've already
            // processed
            status = dma_reg_read(priv, DMA_STATUS_REG);
            if (status & (1UL << DMA_STATUS_RI_BIT)) {
                // Ack the RI, including the normal summary sticky bit
                dma_reg_write(priv, DMA_STATUS_REG, ((1UL << DMA_STATUS_RI_BIT)  |
                                                     (1UL << DMA_STATUS_NIS_BIT)));

                // Must check again for available RX descriptors, in case the RI
                // status came from a new RX descriptor
                spin_lock(&priv->rx_spinlock_);
                available = rx_available_for_read(&priv->rx_gmac_desc_list_info, 1);
                spin_unlock(&priv->rx_spinlock_);
            }

            if (!available) {
                // We have quota left but no Rx packets to process so stop
                // polling
                continue_polling = 0;
                finished = 1;
            }
        } else {
            spin_lock(&priv->rx_spinlock_);
            // If still have Rx packets to process we should indicte that we
            // wish to continue polling once more quota is available to us
            continue_polling = rx_available_for_read(&priv->rx_gmac_desc_list_info, 1);
            spin_unlock(&priv->rx_spinlock_);

            // Must leave poll() routine as no quota left
            finished = 1;
        }
    } while (!finished);

#ifndef ENABLE_RX_OFFLOAD
    // Attempt to fill all available slots in the RX descriptor ring
    spin_lock(&priv->rx_spinlock_);
    available = available_for_write(&priv->rx_gmac_desc_list_info);
    spin_unlock(&priv->rx_spinlock_);

    if (available) {
DBG(1, KERN_INFO "poll() %s: Not empty, so calling refill_rx_ring()\n", dev->name);
        refill_rx_ring(dev, 0, 1);
    }
#endif // !ENABLE_RX_OFFLOAD

    // Decrement the quota and budget even if we didn't process any packets
    if (!received) {
        DBG(1, KERN_WARNING "poll() %s: received = 0\n", dev->name);
        received = 1;
    }

    // Update record of quota consumed
    dev->quota -= received;
    *budget    -= received;

    if (!continue_polling) {
DBG(1, KERN_INFO "$Wpoll() %s: No outstanding packets and ring full, reenabling intrs\n", dev->name);
            // No more received packets to process so return to interrupt mode
            netif_rx_complete(dev);

            // Enable interrupts caused by received packets
            gmac_int_en_set(priv, (1UL << DMA_INT_ENABLE_RI_BIT));
    }

DBG(1, KERN_INFO "$Gpoll() %s: Leaving with continue_polling = %d\n", dev->name, continue_polling);
    return continue_polling;
}
#else // USE_RX_NAPI
/*
 * Normal interrupt driver receive
 */
static void receive(
    gmac_priv_t *priv,
    int         *was_rx_performed)
{
    int available;

    // While there are receive packets to process
    while (1) {
        dma_addr_t dma_address;
        u32 dma_length;
        u32 desc_status;
        int desc;
        struct sk_buff* skb = 0;

        spin_lock(&priv->rx_spinlock_);
        desc = get_rx_descriptor(priv, &desc_status, &dma_address, &dma_length, &skb, 0);
        spin_unlock(&priv->rx_spinlock_);
        if (desc < 0) {
            break;
        } else if (!skb) {
            panic("receive() %s: Found RX descriptor without attached skb\n", priv->netdev->name);
        } else {
            int packet_len = get_rx_length(desc_status);

            // Remember that at least one Rx descriptor has been freed
            *was_rx_performed = 1;

            // Release the DMA mapping for the received data
            dma_unmap_single(0, dma_address, dma_length, DMA_FROM_DEVICE);

            // Do all receive processing for the packet
            skb = process_rx_packet(priv, packet_len, desc_status, skb);

            // If we have a socket buffer available and there is room to
            // queue a new RX descriptor
            if (skb) {
                spin_lock(&priv->rx_spinlock_);
                available = available_for_write(&priv->rx_gmac_desc_list_info);
                spin_unlock(&priv->rx_spinlock_);
                if (available) {
                    // Make use of the available socket buffer by
                    // attempting to fill all available slots in the RX
                    // descriptor ring. Don't reenable RU interrupts
                    refill_rx_ring(priv->netdev, skb);
                } else {
                    // Free the socket buffer, as we couldn't make use
                    // of it to refill the RX descriptor ring
                    dev_kfree_skb(skb);
                }
            }
        }
    }

    // Are there are any Rx descriptors without associated buffers?
    spin_lock(&priv->rx_spinlock_);
    available = available_for_write(&priv->rx_gmac_desc_list_info);
    spin_unlock(&priv->rx_spinlock_);
    if (available) {
        // Yes, so attempt to fill all available slots in the RX descriptor ring.
        // Don't reenable RU interrupts
        refill_rx_ring(priv->netdev, 0);
    }
}
#endif // USE_RX_NAPI

#if defined(CONFIG_LEON_COPRO) && defined(CONFIG_LEON_OFFLOAD_TX)
static void copro_fill_tx_job(
    volatile gmac_tx_que_ent_t *job,
    struct sk_buff             *skb)
{
    int i;
    int nr_frags = skb_shinfo(skb)->nr_frags;
    unsigned short flags = 0;

    // Get a DMA mapping of the packet's data
    dma_addr_t hdr_dma_address = dma_map_single(0, skb->data, skb_headlen(skb), DMA_TO_DEVICE);
    BUG_ON(dma_mapping_error(hdr_dma_address));

    // Get a DMA mapping for as many fragments as will fit into the first level
    // fragment info. storage within the job structure
    for (i=0; (i < nr_frags) && (i < COPRO_NUM_TX_FRAGS_DIRECT); ++i) {
        struct skb_frag_struct *frag = &skb_shinfo(skb)->frags[i];
        job->frag_ptr_[i] = dma_map_page(0, frag->page, frag->page_offset, frag->size, DMA_TO_DEVICE);
        job->frag_len_[i] = frag->size;
    }

    // Allocate storage for remainder of fragments and create DMA mappings
    /** @todo Could change so that if too many fragments call sbk_linearize()
              and take the CPU memory copies hit */
    if (i < nr_frags) {
        panic("Fill: Insufficient fragment storage");
    }

    // Is h/w checksumming and possibly TSO required
    if (likely((skb->ip_summed == CHECKSUM_HW) &&
               (ntohs(skb->protocol) == ETH_P_IP))) {
        flags |= (1UL << TX_JOB_FLAGS_ACCELERATE_BIT);
    }

    // Fill the job description with information about the packet
    job->skb_         = (u32)skb;
    job->len_         = skb->len;
    job->data_len_    = skb->data_len;
    job->ethhdr_      = hdr_dma_address;
    job->iphdr_       = hdr_dma_address + ((void*)skb->nh.iph - (void*)skb->data);
    job->iphdr_csum_  = skb->nh.iph->check;
    job->tso_segs_    = skb_shinfo(skb)->tso_segs;
    job->tso_size_    = skb_shinfo(skb)->tso_size;
    job->flags_       = flags;
    job->statistics_  = 0;
}

static void copro_free_tx_resources(volatile gmac_tx_que_ent_t* job)
{
    int i;
    struct sk_buff* skb = (struct sk_buff*)job->skb_;
    int nr_frags = skb_shinfo(skb)->nr_frags;

    // Release the DMA mapping for the data directly referenced by the SKB
    dma_unmap_single(0, job->ethhdr_, skb_headlen(skb), DMA_TO_DEVICE);

    // Release the DMA mapping for any fragments in the first level fragment
    // info. storage within the job structure
    for (i=0; (i < nr_frags) && (i < COPRO_NUM_TX_FRAGS_DIRECT); ++i) {
        dma_unmap_page(0, job->frag_ptr_[i], job->frag_len_[i], DMA_TO_DEVICE);
    }

    // Release DMA mapping for remainder of fragments and release storage
    /** @todo */
    if (i < nr_frags) {
        panic("Free: Insufficient fragment storage");
    }

    // Inform the network stack that we've finished with the packet
    dev_kfree_skb_irq(skb);
}

static void copro_process_pending_tx_skbs(
    struct net_device          *dev,
    volatile gmac_tx_que_ent_t *job)
{
    gmac_priv_t *priv = (gmac_priv_t*)netdev_priv(dev);

    // Process pending SKBs, oldest first
    do {
        // Get the oldest pending SKB
        struct sk_buff *skb;
        struct list_head *entry = priv->copro_tx_skb_list_.next;
        BUG_ON(!entry);
        list_del(entry);

        skb = list_entry(entry, struct sk_buff, cb);
        BUG_ON(!skb);

        // Keep track of how many entries are in the pending SKB list
        --priv->copro_tx_skb_list_count_;

        // Fill the Tx offload job with the network packet's details
//printk("D");
        copro_fill_tx_job(job, skb);

        // Enqueue the new Tx offload job with the CoPro
        tx_que_new_job(dev, job);

        if (list_empty(&priv->copro_tx_skb_list_)) {
            // No more pending SKBs
            break;
        }
    } while ((job = tx_que_get_idle_job(dev)));
}

static void finish_xmit(struct net_device *dev)
{
    gmac_priv_t                *priv = (gmac_priv_t*)netdev_priv(dev);
    volatile gmac_tx_que_ent_t *job;

    // SMP-safe protection against concurrent operations in ISR and hard_start_xmit()
    spin_lock(&priv->tx_spinlock_);

//printk("?");
    // Process all available completed jobs
    while ((job = tx_que_get_finished_job(dev))) {
        int aborted;
        int carrier;
        int collisions;
        u32 statistics = job->statistics_;
//printk("F");

        copro_free_tx_resources(job);

        // Accumulate TX statistics returned by CoPro in the job structure
        priv->stats.tx_bytes   += (statistics & TX_JOB_STATS_BYTES_MASK)   >> TX_JOB_STATS_BYTES_BIT;
        priv->stats.tx_packets += (statistics & TX_JOB_STATS_PACKETS_MASK) >> TX_JOB_STATS_PACKETS_BIT;
        aborted    = (statistics & TX_JOB_STATS_ABORT_MASK)     >> TX_JOB_STATS_ABORT_BIT;
        carrier    = (statistics & TX_JOB_STATS_CARRIER_MASK)   >> TX_JOB_STATS_CARRIER_BIT;
        collisions = (statistics & TX_JOB_STATS_COLLISION_MASK) >> TX_JOB_STATS_COLLISION_BIT;
        priv->stats.tx_aborted_errors += aborted;
        priv->stats.tx_carrier_errors += carrier;
        priv->stats.collisions        += collisions;
        priv->stats.tx_errors += (aborted + carrier + collisions);
    }

    // Process any queued pending SKBs for which resources are available
    if (priv->copro_tx_skb_list_count_ && (job = tx_que_get_idle_job(dev))) {
        copro_process_pending_tx_skbs(dev, job);

        // Record start of transmission, so timeouts will work once they're
        // implemented
        dev->trans_start = jiffies;

        // Interrupt the CoPro to cause it to examine the Tx offload queue
        wmb();
        writel(1UL << COPRO_SEM_INT_TX, SYS_CTRL_SEMA_SET_CTRL);
    }

    // If the network stack's Tx queue was stopped and we now have resources
    // to process more Tx offload jobs
    if (netif_queue_stopped(dev) &&
        !tx_que_is_full(&priv->tx_queue_) &&
        !priv->copro_tx_skb_list_count_) {
        // Restart the network stack's TX queue
        netif_wake_queue(dev);
    }

    // SMP-safe protection against concurrent operations in ISR and hard_start_xmit()
    spin_unlock(&priv->tx_spinlock_);
}
#else
#ifdef ENABLE_TX_OFFLOAD
/**
 * Finish packet transmision started by hard_start_xmit()
 */
static void finish_xmit(struct net_device *dev)
{
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);
    unsigned descriptors_freed = 0;
    int desc;

DBG(1, KERN_INFO "$Yfinish_xmit() %s\n", dev->name);

    // SMP-safe protection against concurrent operations in ISR and hard_start_xmit()
    spin_lock(&priv->tx_spinlock_);

    // Handle transmit descriptors for the completed packet transmission
    do {
        u32 desc_status;
        dma_addr_t dma_address;
        u32 len;
        struct sk_buff* skb;
        gmac_sram_buffer_info_t **sram_info;

        // Get tx descriptor content
        desc = get_tx_descriptor(priv, &desc_status, &dma_address, &len, &skb, &sram_info);
        if (desc >= 0) {
            BUG_ON(!skb && !sram_info);

DBG(28, KERN_INFO "$Mfinish_xmit() %s: Got tx descriptor %d for skb 0x%08x, desc_status = %08x, len = %u\n", dev->name, desc, (u32)skb, desc_status, len);
            if (skb) {
DBG(10, "$W[TX SKB len %u]$n", len);
                // Release the DMA mapping for the socket buffer
                dma_unmap_single(0, dma_address, len, DMA_TO_DEVICE);

                // Inform the network stack that packet transmission has finished
                DBG(10, "$W[TX F]$n");
                dev_kfree_skb_irq(skb);
            }

            if (sram_info) {
#ifdef ENABLE_TX_OFFLOAD
                // Now have a null-terminated array of info struct pointers, all
                // of which should be freed then the array itself deallocated
                gmac_sram_buffer_info_t **info = sram_info;
                int i = 0;
DBG(10, "$W[TX FSRAM %u bufs]$n", dma_address);
                while (info[i]) {
                    free_sram_buffer(priv, info[i++]);
DBG(10, "$W  .$n");
                }
                kfree(sram_info);
#else // ENABLE_TX_OFFLOAD
                panic("finish_xmit() No TX offload support\n");
#endif // ENABLE_TX_OFFLOAD
            }

            if (skb || sram_info) {
                // Check the status of the transmission
                if (is_tx_valid(desc_status)) {
                    priv->stats.tx_bytes += len;
                    priv->stats.tx_packets++;
                } else {
                    DBG(1, KERN_WARNING "finish_xmit() %s: DMA desc_status indicated error (0x%08x)\n", dev->name, desc_status);
                    priv->stats.tx_errors++;
                    if (is_tx_aborted(desc_status)) {
DBG(1, KERN_INFO "$Rfinish_xmit() %s: Aborted error\n", dev->name);
                        ++priv->stats.tx_aborted_errors;
                    }
                    if (is_tx_carrier_error(desc_status)) {
DBG(1, KERN_INFO "$Rfinish_xmit() %s: Carrier error\n", dev->name);
                        ++priv->stats.tx_carrier_errors;
                    }
                }

                if (is_tx_collision_error(desc_status)) {
DBG(1, KERN_INFO "$Rfinish_xmit() %s: Collision error\n", dev->name);
                    ++priv->stats.collisions;
                }
            }

            // Track how many descriptors we make available, so we know
            // if we need to re-start of network stack's TX queue processing
            ++descriptors_freed;
        }
    } while (desc >= 0);   // While we have transmitted descriptors

    // If the TX queue is stopped, there may be a pending TX packet waiting to
    // be transmitted
    if (netif_queue_stopped(dev)) {
        // If any TX descriptors have been freed and there is an outstanding TX
        // packet waiting to be queued due to there not having been a TX descriptor
        // available when hard_start_xmit() was presented with an skb by the network
        // stack
        if ((priv->tx_pending_skb || priv->tx_pending_sram_info) &&
             available_for_write(&priv->tx_gmac_desc_list_info)) {
            // Construct the GMAC specific DMA descriptor
            desc = set_tx_descriptor(priv, priv->tx_pending_dma_addr, priv->tx_pending_length, priv->tx_pending_skb, priv->tx_pending_sram_info);
            if (desc < 0) {
                panic("$Rfinish_xmit() %s: Failed to set desc after finding avail for write\n", dev->name);
            } else {
                // Successfully queued descriptor for pending TX packet
DBG(1, KERN_INFO "finish_xmit() %s: Set TX descriptor %d for $Woutstanding$n skb = 0x%08x, sram_info = 0x%08x\n", dev->name, desc, (u32)priv->tx_pending_skb, (u32)priv->tx_pending_sram_info);

                // No TX packets now outstanding
                priv->tx_pending_dma_addr = 0;
                priv->tx_pending_length = 0;
                priv->tx_pending_skb = 0;
                priv->tx_pending_sram_info = 0;

                // We have used one of the TX descriptors freed by transmission
                // completion processing having occured above
                --descriptors_freed;

                // Issue a TX poll demand to restart TX descriptor processing, as we
                // have just added one, in case it had found there were no more
                // pending transmission
                dma_reg_write(priv, DMA_TX_POLL_REG, 0);
            }
        }

        // If there are TX descriptors available we should restart the TX queue
        if (descriptors_freed) {
            // The TX queue had been stopped by hard_start_xmit() due to lack of
            // TX descriptors, so restart it now that we've freed at least one
printk("$YWaking TX queue\n");
            netif_wake_queue(dev);
#ifdef ENABLE_TX_TSO_OFFLOAD
            up(&priv->tso_netif_resumed_semaphore_);
#endif // ENABLE_TX_TSO_OFFLOAD
        }
    }

    // SMP-safe protection against concurrent operations in ISR and hard_start_xmit()
    spin_unlock(&priv->tx_spinlock_);
}
#else
static void finish_xmit(struct net_device *dev)
{
    gmac_priv_t *priv = (gmac_priv_t*)netdev_priv(dev);
    unsigned     descriptors_freed = 0;

    // SMP-safe protection against concurrent operations in ISR and hard_start_xmit()
    spin_lock(&priv->tx_spinlock_);

    // Handle transmit descriptors for the completed packet transmission
    while (1) {
        u32             desc_status;
        dma_addr_t      dma_address;
        struct sk_buff *skb;
        u32             len;

        // Get tx descriptor content
        if (get_tx_descriptor(priv, &desc_status, &dma_address, &len, &skb, 0) < 0) {
            // No more descriptors so finish
            break;
        }

        // Must have a SKB associated with each descriptor
        BUG_ON(!skb);

        // Release the DMA mapping for the socket buffer
        dma_unmap_single(0, dma_address, len, DMA_TO_DEVICE);

        // Inform the network stack that packet transmission has finished
        dev_kfree_skb_irq(skb);

        // Check the status of the transmission
        if (is_tx_valid(desc_status)) {
            priv->stats.tx_bytes += len;
            priv->stats.tx_packets++;
        } else {
//printk(KERN_WARNING "Tx packet invalid, status = 0x%08x\n", desc_status);
            priv->stats.tx_errors++;
            if (is_tx_aborted(desc_status)) {
                ++priv->stats.tx_aborted_errors;
            }
            if (is_tx_carrier_error(desc_status)) {
                ++priv->stats.tx_carrier_errors;
            }
        }

        if (is_tx_collision_error(desc_status)) {
            ++priv->stats.collisions;
        }

        // Track how many descriptors we make available, so we know
        // if we need to re-start of network stack's TX queue processing
        ++descriptors_freed;
    }

    // If the TX queue is stopped, there may be a pending TX packet waiting to
    // be transmitted
    if (netif_queue_stopped(dev)) {
        // If any TX descriptors have been freed and there is an outstanding TX
        // packet waiting to be queued due to there not having been a TX
        // descriptor available when hard_start_xmit() was presented with an skb
        // by the network stack
        if (priv->tx_pending_skb && available_for_write(&priv->tx_gmac_desc_list_info)) {
            // Construct the GMAC specific DMA descriptor
            if (set_tx_descriptor(priv, priv->tx_pending_dma_addr, priv->tx_pending_length, priv->tx_pending_skb, 0) < 0) {
                panic("finish_xmit() %s: Failed to set desc after finding avail for write\n", dev->name);
            } else {
                // Successfully queued descriptor for pending TX packet
//printk(KERN_WARNING "Queued pending packet\n");

                // No TX packets now outstanding
                priv->tx_pending_dma_addr = 0;
                priv->tx_pending_length = 0;
                priv->tx_pending_skb = 0;

                // We have used one of the TX descriptors freed by transmission
                // completion processing having occured above
                --descriptors_freed;

                // Issue a TX poll demand to restart TX descriptor processing, as we
                // have just added one, in case it had found there were no more
                // pending transmission
                dma_reg_write(priv, DMA_TX_POLL_REG, 0);
            }
        }

        // If there are TX descriptors available we should restart the TX queue
        if (descriptors_freed) {
            // The TX queue had been stopped by hard_start_xmit() due to lack of
            // TX descriptors, so restart it now that we've freed at least one
//printk(KERN_WARNING "Waking queue\n");
            netif_wake_queue(dev);
        }
    }

    // SMP-safe protection against concurrent operations in ISR and hard_start_xmit()
    spin_unlock(&priv->tx_spinlock_);
}
#endif // ENABLE_TX_OFFLOAD
#endif // CONFIG_LEON_COPRO && CONFIG_LEON_OFFLOAD_TX

#ifndef CONFIG_LEON_COPRO
static void process_non_dma_ints(u32 raw_status)
{
    printk(KERN_ERR "$RFound GPI/GMI/GLI interrupt\n");
}
#endif // !CONFIG_LEON_COPRO

#ifdef CONFIG_LEON_COPRO
static void copro_fwd_intrs_handler(
    void *dev_id,
    u32   status,
    int  *was_rx_performed)
{
    struct net_device *dev = (struct net_device *)dev_id;
    gmac_priv_t       *priv = (gmac_priv_t*)netdev_priv(dev);
    int                restart_watchdog = 0;
    int                restart_tx = 0;
    int                poll_tx = 0;

    // Test for normal receive interrupt
    if (status & (1UL << DMA_STATUS_RI_BIT)) {
#ifdef USE_RX_NAPI
        if (netif_rx_schedule_prep(dev)) {
            // Tell system we have work to be done
            __netif_rx_schedule(dev);
        } else {
            printk(KERN_ERR "copro_fwd_intrs_handler() %s: RX interrupt while in poll\n", dev->name);
        }
#else
        receive(priv, was_rx_performed);
#endif // USE_RX_NAPI
    }

    // Test for unavailable RX buffers
    if (status & (1UL << DMA_STATUS_RU_BIT)) {
        DBG(30, KERN_INFO "$Rint_handler() %s: RX buffer unavailable\n", dev->name);
        // Accumulate receive statistics
        ++priv->stats.rx_over_errors;
        ++priv->stats.rx_errors;
    }

    // Test for normal TX interrupt
    if (status & ((1UL << DMA_STATUS_TI_BIT) |
                  (1UL << DMA_STATUS_ETI_BIT))) {
        DBG(1, KERN_INFO "$Mint_handler() %s: TX\n", dev->name);

#ifndef CONFIG_LEON_OFFLOAD_TX
        // Finish packet transmision started by start_xmit
        finish_xmit(dev);
#endif // !CONFIG_LEON_OFFLOAD_TX
    }

    // Test for abnormal transmitter interrupt where there may be completed
    // packets waiting to be processed
    if (status & ((1UL << DMA_STATUS_TJT_BIT) |
                  (1UL << DMA_STATUS_UNF_BIT))) {
#ifndef CONFIG_LEON_OFFLOAD_TX
        // Complete processing of any TX packets closed by the DMA
        finish_xmit(dev);
#endif // !CONFIG_LEON_OFFLOAD_TX

        if (status & (1UL << DMA_STATUS_TJT_BIT)) {
            // A transmit jabber timeout causes the transmitter to enter the
            // stopped state
            printk(/*DBG(50, KERN_INFO */"$Rint_handler() %s: TX jabber timeout\n", dev->name);
            restart_tx = 1;
        } else {
            printk(/*DBG(51, KERN_INFO */"$Rint_handler() %s: TX underflow\n", dev->name);
        }

        // Issue a TX poll demand in an attempt to restart TX descriptor
        // processing
        poll_tx = 1;
    }

    // Test for any of the error states which we deal with directly within
    // this interrupt service routine.
    if (status & ((1UL << DMA_STATUS_ERI_BIT) |
                  (1UL << DMA_STATUS_OVF_BIT) |
                  (1UL << DMA_STATUS_RWT_BIT) |
                  (1UL << DMA_STATUS_RPS_BIT) |
                  (1UL << DMA_STATUS_TPS_BIT) |
                  (1UL << DMA_STATUS_FBE_BIT))) {
        // Test for early RX interrupt
        if (status & (1UL << DMA_STATUS_ERI_BIT)) {
            // Don't expect to see this, as never enable it
            DBG(30, KERN_WARNING "$Mint_handler() %s: Early RX \n", dev->name);
        }

        if (status & (1UL << DMA_STATUS_OVF_BIT)) {
            DBG(52, KERN_INFO "$Rint_handler() %s: Overflow\n", dev->name);
            // Accumulate receive statistics
            ++priv->stats.rx_fifo_errors;
            ++priv->stats.rx_errors;
        }

        if (status & (1UL << DMA_STATUS_RWT_BIT)) {
            DBG(30, KERN_INFO "$Rint_handler() %s: RX watchdog timeout\n", dev->name);
            // Accumulate receive statistics
            ++priv->stats.rx_frame_errors;
            ++priv->stats.rx_errors;
            restart_watchdog = 1;
        }

        if (status & (1UL << DMA_STATUS_RPS_BIT)) {
            DBG(30, KERN_INFO "$Rint_handler() %s: RX process stopped\n", dev->name);
            ++priv->stats.rx_errors;
            restart_watchdog = 1;

            // Restart the receiver
            DBG(35, KERN_INFO "$Mint_handler() %s: Restarting receiver\n", dev->name);
            dma_reg_set_mask(priv, DMA_OP_MODE_REG, (1 << DMA_OP_MODE_SR_BIT));
        }

        if (status & (1UL << DMA_STATUS_TPS_BIT)) {
            printk(/*DBG(30, KERN_INFO */"$Rint_handler() %s: TX process stopped\n", dev->name);
            ++priv->stats.tx_errors;
            restart_watchdog = 1;
            restart_tx = 1;
        }

        // Test for pure error interrupts
        if (status & (1UL << DMA_STATUS_FBE_BIT)) {
            printk(/*DBG(30, KERN_INFO */"$Rint_handler() %s: Bus error\n", dev->name);
            restart_watchdog = 1;
        }

        if (restart_watchdog) {
            // Restart the link/PHY state watchdog immediately, which will
            // attempt to restart the system
            mod_timer(&priv->watchdog_timer, jiffies);
            restart_watchdog = 0;
        }
    }

    if (restart_tx) {
        // Restart the transmitter, causes am implicit Tx descriptor list poll
        printk(/*DBG(35, KERN_INFO */"$Mint_handler() %s: Restarting transmitter\n", dev->name);
#ifndef CONFIG_LEON_OFFLOAD_TX
        dma_reg_set_mask(priv, DMA_OP_MODE_REG, (1 << DMA_OP_MODE_ST_BIT));
#endif // !CONFIG_LEON_OFFLOAD_TX
        poll_tx = 0;
    }

    if (poll_tx) {
        // Issue a TX poll demand in an attempt to restart TX descriptor
        // processing
        printk(/*DBG(33, KERN_INFO */"$Mint_handler() %s: Issuing Tx poll demand\n", dev->name);
#ifndef CONFIG_LEON_OFFLOAD_TX
        dma_reg_write(priv, DMA_TX_POLL_REG, 0);
#endif // !CONFIG_LEON_OFFLOAD_TX
    }
}
#else // CONFIG_LEON_COPRO
static irqreturn_t int_handler(int int_num, void* dev_id, struct pt_regs* regs)
{
    struct net_device *dev = (struct net_device *)dev_id;
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);
    u32 int_enable;
    int rx_polling;
    u32 raw_status;
    u32 status;
#ifndef USE_RX_NAPI
    int rx_was_performed = 0;
#endif // !USE_RX_NAPI

    /** Read the interrupt enable register to determine if we're in rx poll mode
     *  Id like to get rid of this read, if a more efficient way of determining
     *  whether we are polling is available */
    spin_lock(&priv->cmd_que_lock_);
    int_enable = dma_reg_read(priv, DMA_INT_ENABLE_REG);
    spin_unlock(&priv->cmd_que_lock_);

    rx_polling = !(int_enable & (1UL << DMA_INT_ENABLE_RI_BIT));

    // Get interrupt status
    raw_status = dma_reg_read(priv, DMA_STATUS_REG);

    // MMC, PMT and GLI interrupts are not masked by the interrupt enable
    // register, so must deal with them on the raw status
    if (raw_status & ((1UL << DMA_STATUS_GPI_BIT) |
                      (1UL << DMA_STATUS_GMI_BIT) |
                      (1UL << DMA_STATUS_GLI_BIT))) {
        process_non_dma_ints(raw_status);
    }

    // Get status of enabled interrupt sources
    status = raw_status & int_enable;

DBG(1, KERN_INFO "int_handler() %s: Entered, rx_polling = %d, raw_status = 0x%08x, status = 0x%08x, int_enable = 0x%08x\n", dev->name, rx_polling, raw_status, status, int_enable);

    while (status) {
        // Whether the link/PHY watchdog timer should be restarted
        int restart_watchdog = 0;
        int restart_tx       = 0;
        int poll_tx          = 0;
        u32 int_disable_mask = 0;

#ifdef USE_RX_NAPI
        // Test for RX interrupt resulting from sucessful reception of a packet-
        // must do this before ack'ing, else otherwise can get into trouble with
        // the sticky summary bits when we try to disable further RI interrupts
        if (status & (1UL << DMA_STATUS_RI_BIT)) {
            // Disable interrupts caused by received packets as henceforth
            // we shall poll for packet reception
            int_disable_mask |= (1UL << DMA_INT_ENABLE_RI_BIT);

            // Do NAPI compatible receive processing for RI interrupts
            if (netif_rx_schedule_prep(dev)) {
DBG(1, KERN_INFO "$Cint_handler() %s: Register for polling\n", dev->name);
                // Remember that we are polling, so we ignore RX events for the
                // remainder of the ISR
                rx_polling = 1;

                // Tell system we have work to be done
                __netif_rx_schedule(dev);
            } else {
                DBG(1, KERN_ERR "int_handler() %s: RX interrupt while in poll\n", dev->name);
            }
        }
#endif // USE_RX_NAPI

        // Test for unavailable RX buffers - must do this before ack'ing, else
        // otherwise can get into trouble with the sticky summary bits
        if (status & (1UL << DMA_STATUS_RU_BIT)) {
            DBG(30, KERN_INFO "$Rint_handler() %s: RX buffer unavailable\n", dev->name);
            // Accumulate receive statistics
            ++priv->stats.rx_over_errors;
            ++priv->stats.rx_errors;

DBG(3, "$Yint_handler() %s: Disabling RX unavailable interrupts\n", dev->name);
            // Disable RX overflow reporting, so we don't get swamped
            int_disable_mask |= (1UL << DMA_INT_ENABLE_RU_BIT);
        }

        // Do any interrupt disabling with a single register write
        if (int_disable_mask) {
DBG(1, "$Wint_handler() %s: Disabing intr with mask = 0x%08x\n", dev->name, int_disable_mask);
            gmac_int_en_clr_isr(priv, int_disable_mask);

            // Update our record of the current interrupt enable status
            int_enable &= ~int_disable_mask;
        }

        // The broken GMAC interrupt mechanism with its sticky summary bits
        // means that we have to ack all asserted interrupts here; we can't not
        // ack the RI interrupt source as we might like to (in order that the
        // poll() routine could examine the status) because if it was asserted
        // prior to being masked above, then the summary bit(s) would remain
        // asserted and cause an immediate re-interrupt.
        dma_reg_write(priv, DMA_STATUS_REG, status | ((1UL << DMA_STATUS_NIS_BIT) |
                                                      (1UL << DMA_STATUS_AIS_BIT)));

#ifndef USE_RX_NAPI
        // Test for normal receive interrupt
        if (status & (1UL << DMA_STATUS_RI_BIT)) {
            DBG(1, KERN_INFO "$Mint_handler() %s: RX\n", dev->name);
            receive(priv, &rx_was_performed);
        }
#endif // !USE_RX_NAPI

        // Test for normal TX interrupt
        if (status & ((1UL << DMA_STATUS_TI_BIT) |
                      (1UL << DMA_STATUS_ETI_BIT))) {
            DBG(1, KERN_INFO "$Mint_handler() %s: TX\n", dev->name);

            // Finish packet transmision started by start_xmit
            finish_xmit(dev);
        }

        // Test for abnormal transmitter interrupt where there may be completed
        // packets waiting to be processed
        if (status & ((1UL << DMA_STATUS_TJT_BIT) |
                      (1UL << DMA_STATUS_UNF_BIT))) {
            // Complete processing of any TX packets closed by the DMA
            finish_xmit(dev);

            if (status & (1UL << DMA_STATUS_TJT_BIT)) {
                // A transmit jabber timeout causes the transmitter to enter the
                // stopped state
                DBG(50, KERN_INFO "$Rint_handler() %s: TX jabber timeout\n", dev->name);
                restart_tx = 1;
            } else {
                DBG(51, KERN_INFO "$Rint_handler() %s: TX underflow\n", dev->name);
            }

            // Issue a TX poll demand in an attempt to restart TX descriptor
            // processing
            poll_tx = 1;
        }

        // Test for any of the error states which we deal with directly within
        // this interrupt service routine.
        if (status & ((1UL << DMA_STATUS_ERI_BIT) |
                      (1UL << DMA_STATUS_OVF_BIT) |
                      (1UL << DMA_STATUS_RWT_BIT) |
                      (1UL << DMA_STATUS_RPS_BIT) |
                      (1UL << DMA_STATUS_TPS_BIT) |
                      (1UL << DMA_STATUS_FBE_BIT))) {
            // Test for early RX interrupt
            if (status & (1UL << DMA_STATUS_ERI_BIT)) {
                // Don't expect to see this, as never enable it
                DBG(30, KERN_WARNING "$Mint_handler() %s: Early RX \n", dev->name);
            }

            if (status & (1UL << DMA_STATUS_OVF_BIT)) {
                DBG(52, KERN_INFO "$Rint_handler() %s: Overflow\n", dev->name);
                // Accumulate receive statistics
                ++priv->stats.rx_fifo_errors;
                ++priv->stats.rx_errors;
            }

            if (status & (1UL << DMA_STATUS_RWT_BIT)) {
                DBG(30, KERN_INFO "$Rint_handler() %s: RX watchdog timeout\n", dev->name);
                // Accumulate receive statistics
                ++priv->stats.rx_frame_errors;
                ++priv->stats.rx_errors;
                restart_watchdog = 1;
            }

            if (status & (1UL << DMA_STATUS_RPS_BIT)) {
                DBG(30, KERN_INFO "$Rint_handler() %s: RX process stopped\n", dev->name);
                ++priv->stats.rx_errors;
                restart_watchdog = 1;

                // Restart the receiver
                DBG(35, KERN_INFO "$Mint_handler() %s: Restarting receiver\n", dev->name);
                dma_reg_set_mask(priv, DMA_OP_MODE_REG, (1 << DMA_OP_MODE_SR_BIT));
            }

            if (status & (1UL << DMA_STATUS_TPS_BIT)) {
                DBG(30, KERN_INFO "$Rint_handler() %s: TX process stopped\n", dev->name);
                ++priv->stats.tx_errors;
                restart_watchdog = 1;
                restart_tx = 1;
            }

            // Test for pure error interrupts
            if (status & (1UL << DMA_STATUS_FBE_BIT)) {
                DBG(30, KERN_INFO "$Rint_handler() %s: Bus error\n", dev->name);
                restart_watchdog = 1;
            }

            if (restart_watchdog) {
                // Restart the link/PHY state watchdog immediately, which will
                // attempt to restart the system
                mod_timer(&priv->watchdog_timer, jiffies);
                restart_watchdog = 0;
            }
        }

        if (restart_tx) {
            // Restart the transmitter
            DBG(35, KERN_INFO "$Mint_handler() %s: Restarting transmitter\n", dev->name);
            dma_reg_set_mask(priv, DMA_OP_MODE_REG, (1 << DMA_OP_MODE_ST_BIT));
        }

        if (poll_tx) {
            // Issue a TX poll demand in an attempt to restart TX descriptor
            // processing
            DBG(33, KERN_INFO "$Mint_handler() %s: Issuing Tx poll demand\n", dev->name);
            dma_reg_write(priv, DMA_TX_POLL_REG, 0);
        }

        // Read the record of current interrupt requests again, in case some
        // more arrived while we were processing
        raw_status = dma_reg_read(priv, DMA_STATUS_REG);

        // MMC, PMT and GLI interrupts are not masked by the interrupt enable
        // register, so must deal with them on the raw status
        if (raw_status & ((1UL << DMA_STATUS_GPI_BIT) |
                          (1UL << DMA_STATUS_GMI_BIT) |
                          (1UL << DMA_STATUS_GLI_BIT))) {
            process_non_dma_ints(raw_status);
        }

        // Get status of enabled interrupt sources.
        status = raw_status & int_enable;
DBG(1, KERN_INFO "int_handler() %s: raw_status = 0x%08x, status = 0x%08x, int_enable = 0x%08x\n", dev->name, raw_status, status, int_enable);
    }

#ifndef USE_RX_NAPI
    // If we processed any Rx packets and thus freed up Rx descriptors
    if (rx_was_performed) {
        // Enable interrupts caused by receive underruns
        gmac_int_en_set_isr(priv, (1UL << DMA_INT_ENABLE_RU_BIT));
    }
#endif // USE_RX_NAPI

DBG(1, KERN_INFO "int_handler() %s: Leaving\n", dev->name);

    return IRQ_HANDLED;
}
#endif // CONFIG_LEON_COPRO

#ifdef CONFIG_LEON_COPRO
static struct semaphore copro_stop_semaphore;

static void copro_stop_callback(volatile gmac_cmd_que_ent_t* entry)
{
    up(&copro_stop_semaphore);
}
#endif // CONFIG_LEON_COPRO

static void gmac_down(struct net_device *dev)
{
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);
    int desc;
    u32 int_enable;
#ifdef ENABLE_RX_OFFLOAD
    gmac_sram_buffer_info_t **rx_sram_infos;
#endif // ENABLE_RX_OFFLOAD
#ifdef CONFIG_LEON_COPRO
    unsigned long irq_flags;
    tx_que_t *tx_queue = &priv->tx_queue_;
#endif // CONFIG_LEON_COPRO

DBG(1, KERN_INFO "gmac_down() %s\n", dev->name);

    // Stop further TX packets being delivered to hard_start_xmit();
    netif_stop_queue(dev);
    netif_carrier_off(dev);

    // Disable all GMAC interrupts and wait for change to be acknowledged
    gmac_int_en_clr(priv, ~0UL, &int_enable);

#ifdef CONFIG_LEON_COPRO
    // Tell the CoPro to stop network offload operations
    spin_lock(&priv->cmd_que_lock_);
    cmd_que_queue_cmd(&priv->cmd_queue_, GMAC_CMD_STOP, 0, copro_stop_callback);
    spin_unlock(&priv->cmd_que_lock_);

    // Interrupt the CoPro so it sees the new command
    writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);

    // Wait until the CoPro acknowledges the STOP command
    down_interruptible(&copro_stop_semaphore);

    // Wait until the CoPro acknowledges that it has completed stopping
    down_interruptible(&priv->copro_stop_complete_semaphore_);

    // Protect CoPro Tx offload queue manipulations
    spin_lock_irqsave(&priv->tx_spinlock_, irq_flags);

    // Clear out the Tx offload job queue, deallocating associated resources
    while (tx_que_not_empty(tx_queue)) {
        // Free any dynamic fragment ptr/len storage
        /** @todo */
        tx_que_inc_r_ptr(tx_queue);
    }

    // Reinitialise the Tx offload queue metadata
    tx_que_init(
        &priv->tx_queue_,
        (gmac_tx_que_ent_t*)descriptors_phys_to_virt(priv->copro_params_.tx_que_head_),
        priv->copro_tx_que_num_entries_);

    // Empty the pending SKB queue
    while (!list_empty(&priv->copro_tx_skb_list_)) {
        struct sk_buff *skb;

        // Remove the first entry on the list
        struct list_head *entry = priv->copro_tx_skb_list_.next;
        BUG_ON(!entry);
        list_del(entry);

        // Get pointer to SKB from it's list_head member
        skb = list_entry(entry, struct sk_buff, cb);
        BUG_ON(!skb);

        // Inform the network stack that we've finished with the packet
        dev_kfree_skb(skb);
    }
    priv->copro_tx_skb_list_count_ =  0;

    // Finished manipulating CoPro Tx offload queue
    spin_unlock_irqrestore(&priv->tx_spinlock_, irq_flags);
#endif // CONFIG_LEON_COPRO

#ifndef CONFIG_LEON_OFFLOAD_TX
    // Stop transmitter, take ownership of all tx descriptors
    dma_reg_clear_mask(priv, DMA_OP_MODE_REG, DMA_OP_MODE_ST_BIT);
    if (priv->desc_vaddr) {
        tx_take_ownership(&priv->tx_gmac_desc_list_info);
    }
#endif // !CONFIG_LEON_OFFLOAD_TX

    // Stop receiver, take ownership of all rx descriptors
    dma_reg_clear_mask(priv, DMA_OP_MODE_REG, DMA_OP_MODE_SR_BIT);
    if (priv->desc_vaddr) {
        rx_take_ownership(&priv->rx_gmac_desc_list_info);
    }

    // Stop all timers
    delete_watchdog_timer(priv);

#ifdef ENABLE_RX_OFFLOAD
    // Drain Rx work queue
    spin_lock_bh(&priv->rx_offload_lock_);
    while (!list_empty(&priv->rx_offload_list_)) {
        rx_offload_job_t *rx_offload_job;

        // Remove the first entry on the list
        struct list_head *entry = priv->rx_offload_list_.next;
        BUG_ON(!entry);
        list_del(entry);

        // Get pointer to job from it's list_head member
        rx_offload_job = list_entry(entry, rx_offload_job_t, list_);
        BUG_ON(!rx_offload_job);

        // Return all Rx SRAM buffers for the packet to the free pool
        while (rx_offload_job->sram_info_entry_count_) {
            free_sram_buffer(priv, rx_offload_job->sram_info_[--rx_offload_job->sram_info_entry_count_]);
        }

        kfree(rx_offload_job);
    }
    spin_unlock_bh(&priv->rx_offload_lock_);

    // Ensure the Rx offload work queue processing has stopped
    priv->rx_work_queue_stop_flag_ = 1;
    queue_work(priv->rx_work_queue_, &priv->rx_offload_work_);
    while (down_interruptible(&priv->rx_work_queue_stop_semaphore_));
#endif // ENABLE_RX_OFFLOAD

#ifdef ENABLE_TX_OFFLOAD
    // Drain Tx work queue
    spin_lock(&priv->tx_offload_lock_);
    while (!list_empty(&priv->tx_offload_list_)) {
        tx_offload_job_t *tx_offload_job;
        struct sk_buff *skb;

        // Remove the first entry on the list
        struct list_head *entry = priv->tx_offload_list_.next;
        BUG_ON(!entry);
        list_del(entry);

        // Get pointer to job from it's list_head member
        tx_offload_job = list_entry(entry, tx_offload_job_t, list_);
        BUG_ON(!tx_offload_job);

        // Discard the SKB and its associated resources
        skb = tx_offload_job->skb_;
        BUG_ON(!skb);
        dev_kfree_skb_irq(skb);

        kfree(tx_offload_job);
    }
    spin_unlock(&priv->tx_offload_lock_);

    // Request that TX offload worker thread should stop
    priv->tx_work_queue_stop_flag_ = 1;

#ifdef ENABLE_TX_TSO_OFFLOAD
    // Ensure TSO offload worker thread is not blocked waiting for netif queue
    // to be resumed
    up(&priv->tso_netif_resumed_semaphore_);
#endif // ENABLE_TX_TSO_OFFLOAD

    // Ensure the Tx offload work queue processing has stopped
    queue_work(priv->tx_work_queue_, &priv->tx_offload_work_);
    while (down_interruptible(&priv->tx_work_queue_stop_semaphore_));
#endif // ENABLE_TX_OFFLOAD

#ifdef ENABLE_RX_OFFLOAD
    rx_sram_infos = (gmac_sram_buffer_info_t**)kmalloc(sizeof(gmac_sram_buffer_info_t*) * SRAM_BUFFER_COUNT, GFP_KERNEL);
#endif // ENABLE_RX_OFFLOAD

    if (priv->desc_vaddr) {
        // Free receive descriptors
        do {
            dma_addr_t dma_address;
            u32 dma_length;
            struct sk_buff* skb = 0;
#ifdef ENABLE_RX_OFFLOAD
            dma_address = 0;
            dma_length = SRAM_BUFFER_COUNT;
            desc = get_rx_descriptor(priv, NULL, &dma_address, &dma_length, 0, rx_sram_infos);
#else // ENABLE_RX_OFFLOAD
            desc = get_rx_descriptor(priv, NULL, &dma_address, &dma_length, &skb, 0);
#endif // ENABLE_RX_OFFLOAD
            if ((desc >= 0) && (skb || dma_length)) {
DBG(1, KERN_INFO "stop() %s: Got rx descriptor %d for skb 0x%08x\n", dev->name, desc, (u32)skb);
                if (skb) {
                    // Release the DMA mapping for the socket buffer
                    dma_unmap_single(0, dma_address, dma_length, DMA_FROM_DEVICE);

                    // Free the socket buffer
                    DBG(10, "$R[DN F]$n");
                    dev_kfree_skb(skb);
                } else {
#ifdef ENABLE_RX_OFFLOAD
                    // Return any Rx SRAM buffers to the free pool
                    while (dma_length) {
                        free_sram_buffer(priv, rx_sram_infos[--dma_length]);
                    }
#endif // ENABLE_RX_OFFLOAD
                }
            }

        } while (desc >= 0);

        // Free transmit descriptors
        do {
            dma_addr_t                dma_address;
            u32                       dma_length;
            struct sk_buff           *skb;
            gmac_sram_buffer_info_t **sram_info;
            desc = get_tx_descriptor(priv, NULL, &dma_address, &dma_length, &skb, &sram_info);
            if ((desc >= 0)) {
DBG(1, KERN_INFO "stop() %s: Got tx descriptor %d\n", dev->name, desc);

                if (skb) {
DBG(1, KERN_INFO "  for skb 0x%08x\n", (u32)skb);
                    // Release the DMA mapping for the socket buffer
                    dma_unmap_single(0, dma_address, dma_length, DMA_FROM_DEVICE);

                    // Free the socket buffer
                    DBG(10, "$R[DN F]$n");
                    dev_kfree_skb(skb);
                }

                if (sram_info) {
#ifdef ENABLE_TX_OFFLOAD
                    // Now have a null-terminated array of info struct pointers,
                    // all of should be freed then the array itself deallocated
                    gmac_sram_buffer_info_t **info = sram_info;
                    int i = 0;
DBG(1, KERN_INFO "  for sram_info 0x%08x\n", (u32)sram_info);
DBG(10, "$R[DN FSRAM]$n");
                    while (info[i]) {
                        free_sram_buffer(priv, info[i++]);
DBG(10, "$R  .$n");
                    }
                    kfree(sram_info);
#else // ENABLE_TX_OFFLOAD
                    panic("gmac_down() No Tx offload support\n");
#endif // ENABLE_TX_OFFLOAD
                }
            }
        } while (desc >= 0);

        // Free any resources associated with a pending TX packet using a SKB
        if (priv->tx_pending_skb) {
            // Release the DMA mapping for the socket buffer
            dma_unmap_single(0, priv->tx_pending_dma_addr, priv->tx_pending_length, DMA_FROM_DEVICE);

            // Free the socket buffer
            DBG(10, "$R[DNP F]$n");
            dev_kfree_skb(priv->tx_pending_skb);
        }

        // Free any resources associated with a pending TX packet using TSO/SRAM
        if (priv->tx_pending_sram_info) {
#ifdef ENABLE_TX_OFFLOAD
            // Now have a null-terminated array of info struct pointers,
            // all of should be freed then the array itself deallocated
            gmac_sram_buffer_info_t **info = priv->tx_pending_sram_info;
            int i = 0;
DBG(1, KERN_INFO "  for pending sram_info 0x%08x\n", (u32)priv->tx_pending_sram_info);
DBG(10, "$R[DNP FSRAM]$n");
            while (info[i]) {
                free_sram_buffer(priv, info[i++]);
DBG(10, "$R  .$n");
            }
            kfree(priv->tx_pending_sram_info);
#else // ENABLE_TX_OFFLOAD
            panic("gmac_down() Free pending - no Tx offload support\n");
#endif // ENABLE_TX_OFFLOAD
        }
    }

#if defined(ENABLE_TX_OFFLOAD) || defined(ENABLE_RX_OFFLOAD)
    // Deallocate the MTU-sized SRAM buffers for network offload prototyping
    free_sram_network_buffers(dev);
#endif // defined(ENABLE_TX_OFFLOAD) || defined(ENABLE_RX_OFFLOAD)

#ifdef ENABLE_RX_OFFLOAD
    // Free any storage allocated for the Rx SRAM info ptr array
    if (rx_sram_infos) {
        kfree(rx_sram_infos);
    }
#endif // ENABLE_RX_OFFLOAD

    // Power down the PHY - possibly should take Wake-On-LAN into account
    phy_powerdown(dev);
}

static int stop(struct net_device *dev)
{
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);

DBG(1, KERN_INFO "stop() %s\n", dev->name);
    gmac_down(dev);

#ifdef CONFIG_LEON_COPRO
    shutdown_copro();

    if (priv->shared_copro_params_) {
        // Free the DMA coherent parameter space
        dma_free_coherent(0, sizeof(copro_params_t), priv->shared_copro_params_, priv->shared_copro_params_pa_);
        priv->shared_copro_params_ = 0;
    }

    // Disable semaphore register from causing ARM interrupts
    *((volatile unsigned long*)SYS_CTRL_SEMA_MASKA_CTRL) = 0;
    *((volatile unsigned long*)SYS_CTRL_SEMA_MASKB_CTRL) = 0;

    // Release interrupts lines used by semaphore register interrupts
    if (priv->copro_a_irq_alloced_) {
        free_irq(priv->copro_a_irq_, dev);
        priv->copro_a_irq_alloced_ = 0;
    }
    if (priv->copro_b_irq_alloced_) {
        free_irq(priv->copro_b_irq_, dev);
        priv->copro_b_irq_alloced_ = 0;
    }
#endif // CONFIG_LEON_COPRO

    // Release the IRQ
    if (priv->have_irq) {
        free_irq(dev->irq, dev);
        priv->have_irq = 0;
    }

    if (priv->desc_vaddr) {
        // Free DMA descriptors' consistent memory
#ifndef CONFIG_DESCRIPTORS_IN_SRAM
        dma_free_coherent(
            0,
            sizeof(gmac_dma_desc_t) * priv->total_num_descriptors,
            priv->desc_vaddr,
            priv->desc_dma_addr);
#endif // CONFIG_DESCRIPTORS_IN_SRAM

        // Remember that we've freed the descriptors memory
        priv->desc_vaddr = 0;
    }

    // Disable the clock to the MAC block
    writel(1UL << SYS_CTRL_CKEN_MAC_BIT, SYS_CTRL_CKEN_CLR_CTRL);

    return 0;
}

static void hw_set_mac_address(struct net_device *dev, unsigned char* addr)
{
    u32 mac_lo;
    u32 mac_hi;

    mac_lo  =  (u32)addr[0];
    mac_lo |= ((u32)addr[1] << 8);
    mac_lo |= ((u32)addr[2] << 16);
    mac_lo |= ((u32)addr[3] << 24);

    mac_hi  =  (u32)addr[4];
    mac_hi |= ((u32)addr[5] << 8);
//printk("hw_set_mac_address() 0x%08x:0x%08x\n", mac_lo, mac_hi);

    mac_reg_write(netdev_priv(dev), MAC_ADR0_LOW_REG, mac_lo);
    mac_reg_write(netdev_priv(dev), MAC_ADR0_HIGH_REG, mac_hi);
}

static int set_mac_address(struct net_device *dev, void *p)
{
    struct sockaddr *addr = p;

//printk("set_mac_address() %02x:%02x:%02x:%02x:%02x:%02x\n", (int)*(u8*)p, (int)*(u8*)p+1, (int)*(u8*)p+2, (int)*(u8*)p+3, (int)*(u8*)p+4, (int)*(u8*)p+5);
    if (!is_valid_ether_addr(addr->sa_data)) {
        return -EADDRNOTAVAIL;
    }

    memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);
    hw_set_mac_address(dev, addr->sa_data);

    return 0;
}

static void multicast_hash(struct dev_mc_list *dmi, u32 *hash_lo, u32 *hash_hi)
{
    u32 crc = ether_crc_le(dmi->dmi_addrlen, dmi->dmi_addr);
    u32 mask = 1 << ((crc >> 26) & 0x1F);

    if (crc >> 31) {
        *hash_hi |= mask;
    } else {
        *hash_lo |= mask;
    }
}

static void set_multicast_list(struct net_device *dev)
{
    gmac_priv_t* priv = netdev_priv(dev);
    u32 hash_lo=0;
    u32 hash_hi=0;
    u32 mode = 0;
    int i;

//printk("set_multicast_list() IFF_PROMISC = %d, IFF_ALLMULTI = %d, mc_count = %d\n", dev->flags & IFF_PROMISC ? 1 : 0, dev->flags & IFF_ALLMULTI ? 1 : 0, dev->mc_count);
    // Disable promiscuous mode and uni/multi-cast matching
    mac_reg_write(priv, MAC_FRAME_FILTER_REG, mode);

    // Disable all perfect match registers
    for (i=0; i < NUM_PERFECT_MATCH_REGISTERS; ++i) {
        mac_adrhi_reg_write(priv, i, 0);
    }

    // Promiscuous mode overrides all-multi which overrides other filtering
    if (dev->flags & IFF_PROMISC) {
//printk("\tPromiscuous mode\n");
        mode |= (1 << MAC_FRAME_FILTER_PR_BIT);
    } else if (dev->flags & IFF_ALLMULTI) {
//printk("\tAll multicast mode\n");
        mode |= (1 << MAC_FRAME_FILTER_PM_BIT);
    } else {
        struct dev_mc_list *dmi;

        if (dev->mc_count <= NUM_PERFECT_MATCH_REGISTERS) {
//printk("\tPerfect matching %d addresses\n", dev->mc_count);
            // Use perfect matching registers
            for (i=0, dmi = dev->mc_list; dmi; dmi = dmi->next, ++i) {
                u32 addr;
//printk("\tAdr %d len=%d 0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x\n", i, dmi->dmi_addrlen,
//    dmi->dmi_addr[0], dmi->dmi_addr[1], dmi->dmi_addr[2], dmi->dmi_addr[3], dmi->dmi_addr[4], dmi->dmi_addr[5]);

                addr  =      dmi->dmi_addr[0];
                addr |= (u32)dmi->dmi_addr[1] << 8;
                addr |= (u32)dmi->dmi_addr[2] << 16;
                addr |= (u32)dmi->dmi_addr[3] << 24;
                mac_adrlo_reg_write(priv, i, addr);

                addr  =      dmi->dmi_addr[4];
                addr |= (u32)dmi->dmi_addr[5] << 8;
                addr |= (1 << MAC_ADR1_HIGH_AE_BIT);
                mac_adrhi_reg_write(priv, i, addr);
            }
        } else {
            // Use hashing
            mode |= (1 << MAC_FRAME_FILTER_HUC_BIT);
            mode |= (1 << MAC_FRAME_FILTER_HMC_BIT);
//printk("\tHashing %d addresses\n", dev->mc_count);

            for (dmi = dev->mc_list; dmi; dmi = dmi->next) {
                multicast_hash(dmi, &hash_lo, &hash_hi);
            }
        }
    }

    // Update the filtering rules
    mac_reg_write(priv, MAC_FRAME_FILTER_REG, mode);

    // Update the filtering hash table
    mac_reg_write(priv, MAC_HASH_LOW_REG,  hash_lo);
    mac_reg_write(priv, MAC_HASH_HIGH_REG, hash_hi);
}

#ifdef CONFIG_LEON_COPRO
static struct semaphore copro_start_semaphore;

static void copro_start_callback(volatile gmac_cmd_que_ent_t* entry)
{
    up(&copro_start_semaphore);
}
#endif // CONFIG_LEON_COPRO

static int gmac_up(struct net_device *dev)
{
    int status = 0;
    gmac_priv_t *priv = (gmac_priv_t*)netdev_priv(dev);
    u32 reg_contents;
    unsigned long end;

DBG(1, KERN_INFO "gmac_up() %s\n", dev->name);

    // Reset the entire GMAC
    dma_reg_write(priv, DMA_BUS_MODE_REG, 1UL << DMA_BUS_MODE_SWR_BIT);

    // Ensure reset is performed before testing for completion
    wmb();

    // Wait for the reset operation to complete
    status = -EIO;
    end = jiffies + MS_TO_JIFFIES(GMAC_RESET_TIMEOUT_MS);
    while (time_before(jiffies, end)) {
        if (!(dma_reg_read(priv, DMA_BUS_MODE_REG) & (1UL << DMA_BUS_MODE_SWR_BIT))) {
            status = 0;
            break;
        }
    }

    // Did the GMAC reset operation fail?
    if (status) {
        printk(KERN_ERR "open() %s: GMAC reset failed\n", dev->name);
        goto gmac_up_err_out;
    }

    // Form the MAC config register contents
    reg_contents = 0;
    if (!priv->mii.using_1000) {
        DBG(1, KERN_INFO "open() %s: PHY in 10/100Mb mode\n", dev->name);
        reg_contents |= (1UL << MAC_CONFIG_PS_BIT);
    } else {
        DBG(1, KERN_INFO "open() %s: PHY in 1000Mb mode\n", dev->name);
    }
    if (priv->mii.full_duplex) {
        reg_contents |= (1UL << MAC_CONFIG_DM_BIT);
    }
    if (priv->rx_csum) {
        reg_contents |= (1UL << MAC_CONFIG_IPC_BIT);
    }

    reg_contents |= ((1UL << MAC_CONFIG_TE_BIT) |
                     (1UL << MAC_CONFIG_RE_BIT));
    mac_reg_write(priv, MAC_CONFIG_REG, reg_contents);

    // Form the MAC flow control register contents
    reg_contents = 0;
    reg_contents |= ((1UL << MAC_FLOW_CNTL_RFE_BIT) |
                     (1UL << MAC_FLOW_CNTL_TFE_BIT));
    mac_reg_write(priv, MAC_FLOW_CNTL_REG, reg_contents);

    // Form the MAC VLAN tag register contents
    reg_contents = 0;
    mac_reg_write(priv, MAC_VLAN_TAG_REG, reg_contents);

    // Initialise the hardware's record of our primary MAC address
    hw_set_mac_address(dev, dev->dev_addr);

    // Initialise multicast and promiscuous modes
    set_multicast_list(dev);

    // Disable all MMC interrupt sources
    mac_reg_write(priv, MMC_RX_MASK_REG, ~0UL);
    mac_reg_write(priv, MMC_TX_MASK_REG, ~0UL);

    // Remember how large the unified descriptor array is to be
    priv->total_num_descriptors = NUM_TX_DMA_DESCRIPTORS + NUM_RX_DMA_DESCRIPTORS;

    // Initialise the structures managing the TX descriptor list
    init_tx_desc_list(&priv->tx_gmac_desc_list_info,
                      priv->desc_vaddr,
                      NUM_TX_DMA_DESCRIPTORS);

    // Initialise the structures managing the RX descriptor list
    init_rx_desc_list(&priv->rx_gmac_desc_list_info,
                      priv->desc_vaddr + NUM_TX_DMA_DESCRIPTORS,
                      NUM_RX_DMA_DESCRIPTORS);

#ifndef CONFIG_LEON_OFFLOAD_TX
    // Write the physical DMA consistent address of the start of the tx descriptor array
    dma_reg_write(priv, DMA_TX_DESC_ADR_REG, priv->desc_dma_addr);
#endif // !CONFIG_LEON_OFFLOAD_TX

    // Write the physical DMA consistent address of the start of the rx descriptor array
    dma_reg_write(priv, DMA_RX_DESC_ADR_REG, priv->desc_dma_addr +
                        (priv->tx_gmac_desc_list_info.num_descriptors * sizeof(gmac_dma_desc_t)));

    // Initialise the GMAC DMA bus mode register
    dma_reg_write(priv, DMA_BUS_MODE_REG, ((0UL  << DMA_BUS_MODE_FB_BIT)   |
                                           (0UL  << DMA_BUS_MODE_PR_BIT)   |
                                           (32UL << DMA_BUS_MODE_PBL_BIT)  | // AHB burst size
                                           (2UL  << DMA_BUS_MODE_DSL_BIT)  | // Skip the skb and sram_info
                                           (1UL  << DMA_BUS_MODE_DA_BIT)));  // Give Rx priority over Tx

#if defined(ENABLE_TX_OFFLOAD) || defined(ENABLE_RX_OFFLOAD)
    // Allocate the MTU-sized SRAM buffers for network offload prototyping
    alloc_sram_network_buffers(dev);
#endif // defined(ENABLE_TX_OFFLOAD) || defined(ENABLE_RX_OFFLOAD)

    // Prepare receive descriptors
#ifdef USE_RX_NAPI
    refill_rx_ring(dev, 0, 0);
#else // USE_RX_NAPI
    refill_rx_ring(dev, 0);
DBG(1, KERN_INFO "gmac_up() Rx ring refilled\n");
#endif // USE_RX_NAPI

    // Clear any pending interrupt requests
    dma_reg_write(priv, DMA_STATUS_REG, dma_reg_read(priv, DMA_STATUS_REG));

    // Initialise the GMAC DMA operation mode register. Set Tx/Rx FIFO thresholds
    // to make best use of our limited SDRAM bandwidth when operating in gigabit
    dma_reg_write(priv, DMA_OP_MODE_REG, ((DMA_OP_MODE_TTC_256 << DMA_OP_MODE_TTC_BIT) |    // Tx threshold
                                          (1UL << DMA_OP_MODE_FUF_BIT) |    // Forward Undersized good Frames
                                          (DMA_OP_MODE_RTC_32 << DMA_OP_MODE_RTC_BIT) |    // Rx threshold
                                          (1UL << DMA_OP_MODE_OSF_BIT)));   // Operate on 2nd frame

    // Use store&forward when operating in gigabit mode, as system does not have
    // sufficient SDRAM bandwidth to support gigabit Tx without it
    if (priv->mii.using_1000) {
        dma_reg_set_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_SF_BIT));
    } else {
        dma_reg_clear_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_SF_BIT));
    }

    // Ensure setup is complete, before enabling TX and RX
    wmb();

#ifdef CONFIG_LEON_COPRO
    // Update the CoPro's parameters with the current MTU
    priv->copro_params_.mtu_ = dev->mtu;

    // Only attempt to write to uncached/unbuffered shared parameter storage if
    // CoPro is started and thus storage has been allocated
    if (priv->shared_copro_params_) {
        // Fill the CoPro parameter block
        memcpy(priv->shared_copro_params_, &priv->copro_params_, sizeof(copro_params_t));
    }

    // Make sure the CoPro parameter block updates have made it to memory (which
    // is uncached/unbuffered, so just compiler issues to overcome)
    wmb();

    // Tell the CoPro to re-read parameters
    spin_lock(&priv->cmd_que_lock_);
    cmd_que_queue_cmd(&priv->cmd_queue_, GMAC_CMD_UPDATE_PARAMS, 0, 0);
    spin_unlock(&priv->cmd_que_lock_);

    // Interrupt the CoPro so it sees the new command
    writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);

    // Tell the CoPro to begin network offload operations
    spin_lock(&priv->cmd_que_lock_);
    cmd_que_queue_cmd(&priv->cmd_queue_, GMAC_CMD_START, 0, copro_start_callback);
    spin_unlock(&priv->cmd_que_lock_);

    // Interrupt the CoPro so it sees the new command
    writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);

    // Wait until the CoPro acknowledges that it has started
    down_interruptible(&copro_start_semaphore);
#endif // CONFIG_LEON_COPRO

    // Start the transmitter and receiver
#ifndef CONFIG_LEON_OFFLOAD_TX
    dma_reg_set_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_ST_BIT));
#endif // !LEON_OFFLOAD_TX
DBG(1, KERN_INFO "gmac_up() Starting receiver\n");
    dma_reg_set_mask(priv, DMA_OP_MODE_REG, (1UL << DMA_OP_MODE_SR_BIT));

    // Enable interesting GMAC interrupts
DBG(1, KERN_INFO "gmac_up() Enabling interrupts\n");
    gmac_int_en_set(priv, ((1UL << DMA_INT_ENABLE_NI_BIT)  |
                           (1UL << DMA_INT_ENABLE_AI_BIT)  |
                           (1UL << DMA_INT_ENABLE_FBE_BIT) |
                           (1UL << DMA_INT_ENABLE_RI_BIT)  |
                           (1UL << DMA_INT_ENABLE_RU_BIT)  |
                           (1UL << DMA_INT_ENABLE_OV_BIT)  |
                           (1UL << DMA_INT_ENABLE_RW_BIT)  |
                           (1UL << DMA_INT_ENABLE_RS_BIT)  |
                           (1UL << DMA_INT_ENABLE_TI_BIT)  |
                           (1UL << DMA_INT_ENABLE_UN_BIT)  |
                           (1UL << DMA_INT_ENABLE_TJ_BIT)  |
                           (1UL << DMA_INT_ENABLE_TS_BIT)));

    // (Re)start the link/PHY state monitoring timer
    start_watchdog_timer(priv);

    // Allow the network stack to call hard_start_xmit()
    netif_start_queue(dev);

    return status;

gmac_up_err_out:
    stop(dev);

    return status;
}

static int change_mtu(struct net_device *dev, int new_mtu)
{
    int status = 0;
    gmac_priv_t *priv = (gmac_priv_t*)netdev_priv(dev);
    int original_mtu = dev->mtu;

    // Check that new MTU is within supported range
    if ((new_mtu < MIN_PACKET_SIZE) || (new_mtu > MAX_JUMBO)) {
        DBG(1, KERN_WARNING "change_mtu() %s: Invalid MTU %d\n", dev->name, new_mtu);
        status = -EINVAL;
    } else {
        // Put MAC/PHY into quiesent state, causing all current buffers to be
        // deallocated and the PHY to powerdown
        gmac_down(dev);

        // Record the new MTU, so bringing the MAC back up will allocate
        // resources to suit the new MTU
        dev->mtu = new_mtu;

        // Reset the PHY to get it into a known state and ensure we have TX/RX
        // clocks to allow the GMAC reset to complete
        if (phy_reset(priv->netdev)) {
            DBG(1, KERN_ERR "change_mtu() %s: Failed to reset PHY\n", dev->name);
            status = -EIO;
        } else {
            u32 mask;

            // Record whether jumbo frames are enabled
            priv->jumbo_ = (dev->mtu > NORMAL_PACKET_SIZE);

            // Allow GMAC to transfer jumbo frames if jumbo frames are enabled
            // Must disable jabber timeout if jumbo frames are to pass through
            // the transmitter and must disable watchdog if jumbo frames are to
            // pass through the receiver
            mask = ((1UL << MAC_CONFIG_JD_BIT) | (1UL << MAC_CONFIG_JE_BIT) | (1UL << MAC_CONFIG_WD_BIT));
DBG(15, KERN_INFO "change_mtu() Changing to %s frames\n", priv->jumbo_ ? "jumbo" : "normal");
            priv->jumbo_ ? mac_reg_set_mask(priv,   MAC_CONFIG_REG, mask) :
                           mac_reg_clear_mask(priv, MAC_CONFIG_REG, mask);

            // Force or auto-negotiate PHY mode
            priv->phy_force_negotiation = 1;
//            set_phy_mode(dev);
//            if (priv->ethtool_cmd.autoneg == AUTONEG_ENABLE) {
//                // Wait for auto-negotiation to finish
//                msleep(AUTO_NEG_MS_WAIT);
//            }

            // Reallocate buffers with new MTU
            gmac_up(dev);
DBG(1, KERN_INFO "change_mtu() GMAC back up\n");
        }
    }

    // If there was a failure
    if (status) {
DBG(1, KERN_INFO "change_mtu() Failed, returning MTU to original value\n");
        // Return the MTU to its original value
        dev->mtu = original_mtu;
    }

    return status;
}

#ifdef TEST_COPRO
DECLARE_MUTEX_LOCKED(start_sem);
DECLARE_MUTEX_LOCKED(heartbeat_sem);

void start_callback(volatile gmac_cmd_que_ent_t* entry)
{
    printk("START callback, operand = 0x%08x\n", entry->operand_);
    up(&start_sem);
}

void heartbeat_callback(volatile gmac_cmd_que_ent_t* entry)
{
    printk("Heartbeat callback, operand = 0x%08x\n", entry->operand_);
    up(&heartbeat_sem);
}

static void test_copro(gmac_priv_t* priv)
{
    unsigned long irq_flags;

    spin_lock(&priv->cmd_que_lock_);
    cmd_que_queue_cmd(&priv->cmd_queue_, GMAC_CMD_STOP, 0, 0);
    spin_unlock(&priv->cmd_que_lock_);
    writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);
    mdelay(500);

    spin_lock(&priv->cmd_que_lock_);
    cmd_que_queue_cmd(&priv->cmd_queue_, GMAC_CMD_START, 0, start_callback);
    spin_unlock(&priv->cmd_que_lock_);
    writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);
    mdelay(500);

    spin_lock(&priv->cmd_que_lock_);
    cmd_que_queue_cmd(&priv->cmd_queue_, GMAC_CMD_HEARTBEAT, 0, heartbeat_callback);
    spin_unlock(&priv->cmd_que_lock_);
    writel(1UL << COPRO_SEM_INT_CMD, SYS_CTRL_SEMA_SET_CTRL);
    mdelay(500);

    printk("Waiting for start ack...\n");
    down_interruptible(&start_sem);
    printk("Start ack received\n");

    printk("Waiting for heartbeat ack...\n");
    down_interruptible(&heartbeat_sem);
    printk("Heartbeat ack received\n");
}
#endif // TEST_COPRO

#ifdef CONFIG_LEON_COPRO
#define SEM_INT_FWD      8
#define SEM_INT_ACK      16
#define SEM_INT_TX       17
#define SEM_INT_STOP_ACK 18

#define SEM_INTA_MASK  (1UL << SEM_INT_FWD)
#define SEM_INTB_MASK ((1UL << SEM_INT_ACK) | (1UL << SEM_INT_TX) | (1UL << SEM_INT_STOP_ACK))

static irqreturn_t copro_sema_intr(int irq, void *dev_id, struct pt_regs *regs)
{
    struct net_device *dev = (struct net_device *)dev_id;
    gmac_priv_t       *priv = (gmac_priv_t*)netdev_priv(dev);
    u32                asserted;
    u32                fwd_intrs_status = 0;
    int                is_fwd_intr;
    int                rx_was_performed = 0;

    // Read the contents of semaphore A register
    asserted = (*((volatile unsigned long*)SYS_CTRL_SEMA_STAT) & SEM_INTA_MASK);

    while (asserted) {
        // Extract any forwarded interrupts info
        is_fwd_intr = asserted & (1UL << SEM_INT_FWD);
        if (is_fwd_intr) {
            fwd_intrs_status = ((volatile gmac_fwd_intrs_t*)descriptors_phys_to_virt(priv->copro_params_.fwd_intrs_mailbox_))->status_;
        }

        // Clear any interrupts directed at the ARM
        *((volatile unsigned long*)SYS_CTRL_SEMA_CLR_CTRL) = asserted;

        if (is_fwd_intr) {
            // Process any forwarded GMAC interrupts
            copro_fwd_intrs_handler(dev_id, fwd_intrs_status, &rx_was_performed);
        }

        // Stay in interrupt routine if interrupt has been re-asserted
        asserted = (*((volatile unsigned long*)SYS_CTRL_SEMA_STAT) & SEM_INTA_MASK);
    }

    // If we processed any Rx packets and thus freed up Rx descriptors
    if (rx_was_performed) {
        // Enable interrupts caused by receive underruns, in case these have
        // been disabled at the CoPro
        gmac_int_en_set_isr(priv, (1UL << DMA_INT_ENABLE_RU_BIT));
    }

    return IRQ_HANDLED;
}

static irqreturn_t copro_semb_intr(int irq, void *dev_id, struct pt_regs *regs)
{
    struct net_device *dev = (struct net_device *)dev_id;
    gmac_priv_t       *priv = (gmac_priv_t*)netdev_priv(dev);
    u32                asserted;

    // Read the contents of semaphore B register
    asserted = (*((volatile unsigned long*)SYS_CTRL_SEMA_STAT) & SEM_INTB_MASK);

    while (asserted) {
        // Clear any interrupts directed at the ARM
        *((volatile unsigned long*)SYS_CTRL_SEMA_CLR_CTRL) = asserted;

        // Process any outstanding command acknowledgements
        if (asserted & (1UL << SEM_INT_ACK)) {
            while (!cmd_que_dequeue_ack(&priv->cmd_queue_));
        }

        // Process STOP completion signal
        if (asserted & (1UL << SEM_INT_STOP_ACK)) {
            up(&priv->copro_stop_complete_semaphore_);
        }

#ifdef CONFIG_LEON_OFFLOAD_TX
        // Process any completed TX offload jobs
        if (asserted & (1UL << SEM_INT_TX)) {
            finish_xmit(dev);
        }
#endif // CONFIG_LEON_OFFLOAD_TX

        // Stay in interrupt routine if interrupt has been re-asserted
        asserted = (*((volatile unsigned long*)SYS_CTRL_SEMA_STAT) & SEM_INTB_MASK);
    }

    return IRQ_HANDLED;
}
#endif // CONFIG_LEON_COPRO

static int open(struct net_device *dev)
{
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);
    int status;

//DBG(1, KERN_INFO "open() %s:\n", dev->name );

    // Ensure the MAC block is properly reset
    writel(1UL << SYS_CTRL_RSTEN_MAC_BIT, SYS_CTRL_RSTEN_SET_CTRL);
    writel(1UL << SYS_CTRL_RSTEN_MAC_BIT, SYS_CTRL_RSTEN_CLR_CTRL);

    // Enable the clock to the MAC block
    writel(1UL << SYS_CTRL_CKEN_MAC_BIT, SYS_CTRL_CKEN_SET_CTRL);

    // Ensure reset and clock operations are complete
    wmb();

    // Reset the PHY to get it into a known state and ensure we have TX/RX clocks
    // to allow the GMAC reset to complete
    if (phy_reset(priv->netdev)) {
        DBG(1, KERN_ERR "open() %s: Failed to reset PHY\n", dev->name);
        status = -EIO;
        goto open_err_out;
    }

    // Check that the MAC address is valid.  If it's not, refuse to bring the
    // device up
    if (!is_valid_ether_addr(dev->dev_addr)) {
        DBG(1, KERN_ERR "open() %s: MAC address invalid\n", dev->name);
        status = -EINVAL;
        goto open_err_out;
    }

#ifdef CONFIG_LEON_COPRO
    // Register ISRs for the semaphore register interrupt sources, which will
    // originate from the CoPro
    if (request_irq(priv->copro_a_irq_, &copro_sema_intr, 0, "SEMA", dev)) {
        panic("open: Failed to allocate semaphore A %u\n", priv->copro_a_irq_);
        status = -ENODEV;
        goto open_err_out;
    }
    priv->copro_a_irq_alloced_ = 1;

    if (request_irq(priv->copro_b_irq_, &copro_semb_intr, 0, "SEMB", dev)) {
        panic("open: Failed to allocate semaphore B %u\n", priv->copro_b_irq_);
        status = -ENODEV;
        goto open_err_out;
    }
    priv->copro_b_irq_alloced_ = 1;
#else // CONFIG_LEON_COPRO
    // Allocate the IRQ
    if (request_irq(dev->irq, &int_handler, 0, dev->name, dev)) {
        DBG(1, KERN_ERR "open() %s: Failed to allocate irq %d\n", dev->name, dev->irq);
        status = -ENODEV;
        goto open_err_out;
    }
    priv->have_irq = 1;
#endif // CONFIG_LEON_COPRO

    // Need a consistent DMA mapping covering all the memory occupied by DMA
    // unified descriptor array, as both CPU and DMA engine will be reading and
    // writing descriptor fields.
#ifdef CONFIG_DESCRIPTORS_IN_SRAM
    priv->desc_vaddr    = (gmac_dma_desc_t*)GMAC_DESC_ALLOC_START;
    priv->desc_dma_addr = GMAC_DESC_ALLOC_START_PA;
#else // CONFIG_DESCRIPTORS_IN_SRAM
    priv->desc_vaddr = dma_alloc_coherent(
        0,
        sizeof(gmac_dma_desc_t) * priv->total_num_descriptors,
        &priv->desc_dma_addr,
        GFP_KERNEL | GFP_DMA);
#endif // CONFIG_DESCRIPTORS_IN_SRAM

    if (!priv->desc_vaddr) {
        DBG(1, KERN_ERR "open() %s: Failed to allocate consistent memory for DMA descriptors\n", dev->name);
        status = -ENOMEM;
        goto open_err_out;
    }
    DBG(1, KERN_INFO "open() TX desc base = 0x%08x\n", priv->desc_dma_addr);

#ifdef CONFIG_LEON_COPRO
    // Allocate SRAM for the command queue entries
    priv->copro_params_.cmd_que_head_ = DESCRIPTORS_BASE_PA + DESCRIPTORS_SIZE;

    priv->copro_params_.cmd_que_tail_ =
        (u32)((gmac_cmd_que_ent_t*)(priv->copro_params_.cmd_que_head_) + priv->copro_cmd_que_num_entries_);
    priv->copro_params_.fwd_intrs_mailbox_ = priv->copro_params_.cmd_que_tail_;
    priv->copro_params_.tx_que_head_ = priv->copro_params_.fwd_intrs_mailbox_ + sizeof(gmac_fwd_intrs_t);
    priv->copro_params_.tx_que_tail_ =
        (u32)((gmac_tx_que_ent_t*)(priv->copro_params_.tx_que_head_) + priv->copro_tx_que_num_entries_);
    priv->copro_params_.free_start_ = priv->copro_params_.tx_que_tail_;

    // Set RX interrupt mitigation behaviour
    priv->copro_params_.rx_mitigation_        = COPRO_RX_MITIGATION;
    priv->copro_params_.rx_mitigation_frames_ = COPRO_RX_MITIGATION_FRAMES;
    priv->copro_params_.rx_mitigation_usec_   = COPRO_RX_MITIGATION_USECS;

    // Initialise command queue metadata
    cmd_que_init(
        &priv->cmd_queue_,
        (gmac_cmd_que_ent_t*)descriptors_phys_to_virt(priv->copro_params_.cmd_que_head_),
        priv->copro_cmd_que_num_entries_);

    // Initialise tx offload queue metadata
    tx_que_init(
        &priv->tx_queue_,
        (gmac_tx_que_ent_t*)descriptors_phys_to_virt(priv->copro_params_.tx_que_head_),
        priv->copro_tx_que_num_entries_);

    // Allocate DMA coherent space for the parameter block shared with the CoPro
    priv->shared_copro_params_ = dma_alloc_coherent(0, sizeof(copro_params_t), &priv->shared_copro_params_pa_, GFP_KERNEL);
    if (!priv->shared_copro_params_) {
        DBG(1, KERN_ERR "open() %s: Failed to allocate DMA coherent space for parameters\n");
        status = -ENOMEM;
        goto open_err_out;
    }

    // Fill the shared CoPro parameter block from the ARM's local copy
    memcpy(priv->shared_copro_params_, &priv->copro_params_, sizeof(copro_params_t));

    // Load CoPro program and start it running
    init_copro(leon_srec, priv->shared_copro_params_pa_);

    // Enable selected semaphore register bits to cause ARM interrupts
    *((volatile unsigned long*)SYS_CTRL_SEMA_MASKA_CTRL) = SEM_INTA_MASK;
    *((volatile unsigned long*)SYS_CTRL_SEMA_MASKB_CTRL) = SEM_INTB_MASK;

#ifdef TEST_COPRO
    // Send test commands to the CoPro
    test_copro(priv);
#endif // TEST_COPRO
#endif // CONFIG_LEON_COPRO

    // Do startup operations that are in common with gmac_down()/_up() processing
    priv->mii_init_media = 1;
    priv->phy_force_negotiation = 1;
    status = gmac_up(dev);
    if (status) {
        goto open_err_out;
    }

    return 0;

open_err_out:
    stop(dev);

    return status;
}

#ifdef ENABLE_TX_OFFLOAD
static void tx_offload_dma_callback(
    oxnas_dma_channel_t         *channel,
    oxnas_callback_arg_t         arg,
    oxnas_dma_callback_status_t  status,
    u16                          checksum,
    int                          interrupt_count)
{
    gmac_priv_t *priv = (gmac_priv_t*)arg;

    DBG(1, KERN_INFO "tx_offload_dma_callback() Called, channel = %u, checksum = 0x%04hx, status = %d\n", channel->channel_number_, checksum, status);
    priv->tx_csum_result_ = checksum;
    priv->tx_dma_status_ = status;
    up(&priv->tx_offload_semaphore_);
}

/**
 * The method employed here will vary greatly between the initial CPU driven
 * TX offload used to prove the algorithms and the final co-processor assisted
 * version.
 * The initial version will defer to a worker thread, so that the
 * blocking resource allocation and wait for the DMAs to complete can be
 * performed. This version will also handle all queuing of descriptors to the
 * GMAC, as well as passing the skb back to the network stack when processing is
 * complete.
 * The final version will probably pass a structure defining the offload job to
 * the co-processor, which will take care of resource allocation and waiting for
 * DMA completion. If the CPU is still responsible for GMAC descriptor
 * management, the co-processor will then signal the CPU, probably via the
 * shared semaphore register, causing the CPU to retrieve from the Co-Pro info.
 * about any buffers to be associated with descriptors and queued with the
 * GMAC. Once the descriptors are queued, the finish_xmit() method should deal
 * with cleanup, including returning the skb to the network stack, as usual.
 */
static void tx_offload(gmac_priv_t *priv, struct sk_buff *skb)
{
    // Allocate a description of the TX offload job
    tx_offload_job_t* tx_offload_job = (tx_offload_job_t*)kmalloc(sizeof(tx_offload_job_t), GFP_ATOMIC);
    BUG_ON(!tx_offload_job);
    INIT_LIST_HEAD(&tx_offload_job->list_);
    tx_offload_job->skb_ = skb;
DBG(1, KERN_INFO "$Gtx_offload() Called\n");

    // Add TX offload description to queue of offload jobs
    spin_lock(&priv->tx_offload_lock_);
    // Insert the new entry in FIFO queue
    list_add_tail(&tx_offload_job->list_, &priv->tx_offload_list_);
    // Maintain number for debugging
    tx_offload_job->number_ = priv->tx_job_number_++;
    spin_unlock(&priv->tx_offload_lock_);

DBG(1, KERN_INFO "$GQueued job %lu\n", tx_offload_job->number_);
    // Schedule the tx offload job handler for execution on a kernel worker thread
    queue_work(priv->tx_work_queue_, &priv->tx_offload_work_);
}

#ifdef ENABLE_TX_TSO_OFFLOAD
static int is_tx_offload_stop_request_pending(gmac_priv_t *priv)
{
    int stop_pending = 0;

    // Check for a request to stop Tx offload job processing
    if (priv->tx_work_queue_stop_flag_) {
        priv->tx_work_queue_stop_flag_ = 0;
        up(&priv->tx_work_queue_stop_semaphore_);
        stop_pending = 1;
    }

    return stop_pending;
}

static void tx_tso_offload_work(void* data)
{
    unsigned             max_dst_buffers;
    struct scatterlist  *dst_scatterlist;
    oxnas_dma_channel_t *channel;
    gmac_priv_t         *priv = (gmac_priv_t*)data;
    u16                  mtu = priv->netdev->mtu + ETH_HLEN;

#ifdef FORCE_TSO_SEG_SIZE
mtu = FORCE_TSO_SEG_SIZE;
#endif

    // Allocate sufficient destination scatterlist entries
    max_dst_buffers = (mtu >> (SRAM_NET_BUFFER_SIZE_POW2));
    if ((max_dst_buffers << (SRAM_NET_BUFFER_SIZE_POW2)) < mtu) {
        ++max_dst_buffers;
    }
    dst_scatterlist = (struct scatterlist*)kmalloc(sizeof(struct scatterlist) * max_dst_buffers, GFP_KERNEL);
    BUG_ON(!dst_scatterlist);

    // Allocate a DMA channel
    channel = oxnas_dma_request(1);
    BUG_ON(channel == OXNAS_DMA_CHANNEL_NUL);

    // Register the DMA notification callback
    oxnas_dma_set_callback(channel, tx_offload_dma_callback, priv);

    while (1) {
        struct scatterlist       *src_scatterlist;
        unsigned                  nr_frags;
        struct sk_buff           *skb;
        struct iphdr             *iphdr;
        u32                       pseudo_header_csum;
        u16                       eth_ip_hdrlen;
        u16                       hdr_len;
        unsigned                  num_output_packets;
        u16                       payload_csum_offset;
        int                       fin = 0;
        struct list_head         *entry = 0;
        tx_offload_job_t         *tx_offload_job = 0;
        unsigned                  tso_seg = 0;
        u16                       used_data = 0;
        u16                       previous_payload_length = 0;
        unsigned                  src_seg_num = 0;      // Which of skb and frags are we currently using for src data
        unsigned                  src_seg_offset = 0;   // Offset into current src segment
        u32                       first_non_urgent_seq = 0;
        int                       abandon_tso = 0;
//int i,j;

        // Check for a request to stop Tx offload job processing
        if (is_tx_offload_stop_request_pending(priv)) {
            break;
        }

        // Unlink the earliest queued TX offload job from queue of pending jobs
        spin_lock(&priv->tx_offload_lock_);
        if (!list_empty(&priv->tx_offload_list_)) {
            // Remove the first entry on the list
            entry = priv->tx_offload_list_.next;
            list_del(entry);
        }
        spin_unlock(&priv->tx_offload_lock_);

        if (!entry) {
            // No offload jobs pending
            break;
        }

        // Get pointer to job from it's list_head member
        tx_offload_job = list_entry(entry, tx_offload_job_t, list_);
        BUG_ON(!tx_offload_job);
//printk("$WProcessing job %lu\n", tx_offload_job->number_);

        skb = tx_offload_job->skb_;
        BUG_ON(!skb);
//printk("\nskb->len = %u\n", skb->len);

        // Extract a pointer to the IP header structure from the skb
        iphdr = skb->nh.iph;

//printk("TSO offload, IP hdr:\n");
//printk("  ihl=%u\n",        (unsigned)iphdr->ihl);
//printk("  tos=%hu\n",       iphdr->tos);
//printk("  tot_len=%hu\n",   ntohs(iphdr->tot_len));
//printk("  id=%hu\n",        ntohs(iphdr->id));
//printk("  frag_off=%hu\n",  ntohs(iphdr->frag_off) & 0x1fff);
//printk("  flags=%u\n",      (unsigned)((ntohs(iphdr->frag_off) & 0xe000) >> 13));
//printk("  ttl=%u\n",        (unsigned)iphdr->ttl);
//printk("  protocol=%hu\n",  iphdr->protocol);
//printk("  check=0x%08hx\n", ntohs(iphdr->check));
//printk("  saddr=0x%08x\n",  ntohl(iphdr->saddr));
//printk("  daddr=0x%08x\n",  ntohl(iphdr->daddr));
//
//if (skb_shinfo(skb)->nr_frags) {
//    printk("Fragments!\n");
//}
//
//if (skb_shinfo(skb)->tso_size) {
//    printk("TSO!\n");
//}
//
//if (skb_shinfo(skb)->frag_list) {
//    printk("Frag list - can't handle this!\n");
//}
//if (skb_shinfo(skb)->nr_frags || skb_shinfo(skb)->tso_size || skb_shinfo(skb)->frag_list) {
//    printk("Fragment count = %d, skb->len = %u, skb->data_len = %u, tso_segs = %u, head_len = %u, tso_size = %hu\n",
//        skb_shinfo(skb)->nr_frags, skb->len, skb->data_len, skb_shinfo(skb)->tso_segs, skb_headlen(skb), skb_shinfo(skb)->tso_size);
//    for (i=0; i < skb_shinfo(skb)->nr_frags; i++) {
//        u32 adr = (u32)(page_address(skb_shinfo(skb)->frags[i].page) + skb_shinfo(skb)->frags[i].page_offset);
//        printk("  frag[%d].size = %u\n", i, skb_shinfo(skb)->frags[i].size);
//        printk("  frag[%d].addr = 0x%08x\n", i, adr);
//        if (adr & 1) {
//            printk("*-*-*-*-*-*-*-*-*-*- BYTE ALIGNED FRAGMENT -*-*-*-*-*-*-*-*-*-*\n");
//        } else if (adr & 2) {
//            printk("*-*-*-*-*-*-*-*-*-*- WORD ALIGNED FRAGMENT -*-*-*-*-*-*-*-*-*-*\n");
//        }
//    }
//    if (skb_shinfo(skb)->frag_list) {
//        struct sk_buff *list = skb_shinfo(skb)->frag_list;
//        for (; list; list = list->next) {
//            printk("  frag_list[%d].len = %u\n", i, list->len);
//        }
//    }
//}

//printk("head_len %u\n", skb_headlen(skb));
//for (i=0; i < skb_headlen(skb);) {
//    printk("0x%02x ", (unsigned)skb->data[i++]);
//    if (!(i%16)) {
//        printk("\n");
//    }
//}
//printk("\n");

        // Create enough src SG entries for any possible arrangement of buffers,
        // i.e. one buffer for ETH+IP headers, one for TCP/UDP header, one for
        // the remainder of the data directly associated with the skb and one
        // for each fragment
        nr_frags = skb_shinfo(skb)->nr_frags;
//printk("nr_frags = %u\n", nr_frags);
        src_scatterlist = (struct scatterlist*)kmalloc(sizeof(struct scatterlist) * (nr_frags+3), GFP_KERNEL);
        BUG_ON(!src_scatterlist);

//for (j=0; j < nr_frags; j++) {
//    int frag_len = skb_shinfo(skb)->frags[j].size;
//    struct skb_frag_struct *frag = &skb_shinfo(skb)->frags[j];
//    unsigned char* frag_data = page_address(frag->page) + frag->page_offset;
//
//    printk("frag[%d] len %u\n", j, frag_len);
//    for (i=0; i < frag_len;) {
//        printk("0x%02x ", (unsigned)frag_data[i++]);
//        if (!(i%16)) {
//            printk("\n");
//        }
//    }
//    printk("\n");
//}

        // Zero IP header checksum field
        iphdr->check = 0;

        // Accumulate the total length of all headers and zero IP and TCP/UDP
        // checksum fields, in preparation for calculating new checksums
        pseudo_header_csum = 0;
        eth_ip_hdrlen = skb->h.raw - skb->data;
        payload_csum_offset = eth_ip_hdrlen;
        hdr_len = eth_ip_hdrlen;
        switch (iphdr->protocol) {
            case IPPROTO_TCP:
                // Add on length of TCP header
                hdr_len += ((struct tcphdr*)skb->h.raw)->doff * 4;

                // Calculate the offset of the TCP payload checksum from the
                // start of the packet
                payload_csum_offset += offsetof(struct tcphdr, check);
                break;
            case IPPROTO_UDP:
                // Add on length of UDP header
                hdr_len += 8;

                // Calculate the offset of the UDP payload checksum from the
                // start of the packet
                payload_csum_offset += offsetof(struct udphdr, check);
                break;
            default:
                panic("tx_tso_offload_work() IP encapsulated protocol %u is not TCP orUDP\n", (unsigned)iphdr->protocol);
        }
//printk("hdr_len = %u\n", hdr_len);

        // Zero TCP/UDP header checksum field
        *(u16*)(skb->data + payload_csum_offset) = 0;

        // Calculate the unchanging part of the pseudo header
        pseudo_header_csum = csum_tcpudp_nofold(
            iphdr->saddr,
            iphdr->daddr,
            0,
            iphdr->protocol,
            0);

        // If TSO is not involved there will be a single output packet,
        // processed using exactly the same logic as if TSO was present. This
        // will be somewhat wasteful in the non-TSO case, but eventually we'll
        // be handing this off to the CoPro anyway, so just need to prove the
        // logic here
#ifdef FORCE_TSO_SEG_SIZE
        // Calculate number of output packets based on the amount of data in the
        // input packet and knowledge of the MTU
        num_output_packets = skb->len / mtu;
        if ((num_output_packets * mtu) < skb->len) {
            ++num_output_packets;
        }
        // Make sure have enough space to store the additional header in each output
        // packet except for the first
        if ((num_output_packets * mtu) < (skb->len + (hdr_len * (num_output_packets-1)))) {
            ++num_output_packets;
        }
#else // FORCE_TSO_SEG_SIZE
        // Rely on the TSO support to know how many output packets are required
        num_output_packets = skb_shinfo(skb)->tso_segs;
        if (!num_output_packets) {
            num_output_packets = 1;
        }
#endif // FORCE_TSO_SEG_SIZE
//printk("num_output_packets = %u\n", num_output_packets);

        // Process each output packet
        for (tso_seg = 0; !abandon_tso && (tso_seg < num_output_packets); ++tso_seg) {
            int                       i;
            int                       first_packet;
            u16                       remaining_data;
            u16                       max_payload_length;
            int                       last_packet;
            u16                       payload_length;
            u16                       packet_length;
            unsigned                  num_dst_buffers;
            unsigned long             irq_flags;
            int                       desc;
            u16                       dst_remaining_length;
            unsigned                  num_src_buffers;
            dma_addr_t                dma_address;
            u16                       hw_csum;
            u16                       network_order_payload_length;
            gmac_sram_buffer_info_t **sram_info;
            int                       pad_required = 0;
            unsigned                  payload_remaining = 0;

//printk("Seg %u\n\n", tso_seg);
            // Calculate the payload length of this output packet
            first_packet = !used_data;
            remaining_data = skb->len - used_data;
            if (first_packet) {
                remaining_data -= hdr_len;
            }
            max_payload_length = mtu - hdr_len;
            last_packet  = (remaining_data <= max_payload_length);
            payload_length = min(max_payload_length, remaining_data);

            // Odd length payloads cannot be correctly checksummed by the h/w,
            // we must append a zero pad byte to the DMA transfer and include
            // this in the h/w checksumming operation
            pad_required = (payload_length & 1);

            // How many SRAM destination buffers are required for this output
            // packet?
            packet_length = payload_length + hdr_len;
            num_dst_buffers = (packet_length >> (SRAM_NET_BUFFER_SIZE_POW2));
            if ((num_dst_buffers << (SRAM_NET_BUFFER_SIZE_POW2)) < packet_length) {
                ++num_dst_buffers;
            }

            // Allocate sufficient storage for the sram info ptr array to
            // accomodate pointers for all the SRAM output buffers for this
            // output packet
            sram_info = (gmac_sram_buffer_info_t**)kmalloc(sizeof(gmac_sram_buffer_info_t*) * (num_dst_buffers + 1), GFP_KERNEL);
            BUG_ON(!sram_info);

//printk("first_packet %u\n", first_packet);
//printk("remaining_data %u\n", remaining_data);
//printk("payload_length %u\n", payload_length);
//printk("pad_required %u\n", pad_required);
//printk("packet_length %u\n", packet_length);
//printk("num_dst_buffers %u\n", num_dst_buffers);

            // Modify IP header total length field to match this output packet
            iphdr->tot_len = htons(packet_length - ETH_HLEN);

            // Is this the first, or only, packet
            if (first_packet) {
                // The first packet really consumes the MAC+IP+TCP/UDP headers
                used_data = hdr_len;

                if (iphdr->protocol == IPPROTO_TCP) {
                    struct tcphdr* tcphdr = (struct tcphdr*)skb->h.raw;

                    // Calculate the sequence number of the last urgent byte
                    if (tcphdr->urg) {
                        first_non_urgent_seq = ntohs(tcphdr->urg_ptr) + ntohs(tcphdr->seq) + 1;
                    }

                    // Cache the FIN flag, as will be destroyed in TCP header as
                    // it maybe modified to provide the basis for each output
                    // packet
                    fin = tcphdr->fin;

                    // Only the last packet should have FIN flag set
                    if (!last_packet && fin) {
                        // Clear the FIN flag
                        tcphdr->fin = 0;
                    }
                }
            }

            // Do TCP/UDP header adjustments common to all output packets
            if (iphdr->protocol == IPPROTO_TCP) {
                // Adjust the TCP header sequence number field
                struct tcphdr* tcphdr = (struct tcphdr*)skb->h.raw;

                // Adjust the sequence number to point to the first byte in the
                // output packet
                u32 sequence_number = ntohl(tcphdr->seq);
                if (previous_payload_length) {
                    sequence_number += previous_payload_length;
                    tcphdr->seq = htonl(sequence_number);
                }

                // Will there be any urgent data in the output packet?
                if (sequence_number < first_non_urgent_seq) {
                    // Yes, so set the urgent ptr to reflect the offset from
                    // the start of this output packet to the last urgent byte
                    tcphdr->urg_ptr = htons(first_non_urgent_seq - sequence_number - 1);
                } else if (first_non_urgent_seq) {
                    // This is the first output packet without urgent data, so
                    // clear the urgent flag
                    tcphdr->urg = 0;
                }

                // Only the last packet should have FIN flag set
                if (last_packet && fin) {
                    tcphdr->fin = 1;
                }
            } else if (iphdr->protocol == IPPROTO_UDP) {
                // Adjust the UDP header length field
                ((struct udphdr*)skb->h.raw)->len = htons(payload_length + 8);
            }

            // Remember the length of this packet, so can modify TCP sequence
            // length field of next packet
            previous_payload_length = payload_length;

            // Compute IP header checksum and insert into checksum field
            skb->nh.iph->check = 0;
            skb->nh.iph->check = ip_fast_csum(skb->nh.raw, skb->nh.iph->ihl);

            // Create a DMA mapping for the first (and possibly only) source
            // packet fragment, containing the ETH+IP+TCP/UDP header
            dma_address = dma_map_single(0, skb->data, hdr_len, DMA_TO_DEVICE);

            // Add entry for MAC+IP header to SG-DMA src descriptor list
            src_scatterlist[0].length = eth_ip_hdrlen;
            src_scatterlist[0].dma_address = dma_address;
            src_scatterlist[0].__address = skb->data;   // Virtual adr to aid debug

            // Add entry for TCP/UDP header, with checksuming, to SG-DMA src
            // descriptor list
            src_scatterlist[1].length = hdr_len - eth_ip_hdrlen;
            src_scatterlist[1].dma_address = dma_address + eth_ip_hdrlen;
            src_scatterlist[1].__address = skb->data + eth_ip_hdrlen;   // Virtual adr to aid debug

            // The DMAs to transfers the ETH+IP+TCP/UDP headers have been queued
            // now there's the payload left to create DMA entries for
            payload_remaining = payload_length;

            // For each src buffer contributing to the TCP/UDP payload for this
            // output packet, create streaming DMA mapping and add an entry,
            // with checksuming, to the SG-DMA src descriptor list
            num_src_buffers = 2;
            do {
                unsigned       src_desc;
                unsigned       src_seg_remaining;
                u16            src_length;
                struct page   *page = 0;
                u16            page_offset = 0;
                unsigned char *src_data = 0;
//printk("Loop\n");

                if (skb_is_nonlinear(skb)) {
                    if (src_seg_num) {
                        // Obtaining data from current fragment buffer
                        struct skb_frag_struct *skb_frag = &skb_shinfo(skb)->frags[src_seg_num-1];

                        src_seg_remaining = skb_frag->size - src_seg_offset;
                        page = skb_frag->page;
                        page_offset = skb_frag->page_offset + src_seg_offset;
                    } else {
                        // Is there any more data left in the directly associated
                        // skb buffer?
                        src_seg_remaining = skb_headlen(skb) - hdr_len;
                        src_data = skb->data + hdr_len;

                        if (!src_seg_remaining) {
                            struct skb_frag_struct *skb_frag;

                            // Obtaining data from first fragment buffer
                            src_seg_num = 1;
                            skb_frag = &skb_shinfo(skb)->frags[src_seg_num-1];

                            src_data = 0;
                            src_seg_offset = 0;
                            src_seg_remaining = skb_frag->size;
                            page = skb_frag->page;
                            page_offset = skb_frag->page_offset;
                        }
                    }
                } else {
                    // Obtaining data from buffer directly associated with the
                    // skb structure
                    src_seg_remaining = skb->len - hdr_len - src_seg_offset;
                    src_data = skb->data + hdr_len + src_seg_offset;
                }
//printk("src_seg_num = %u\n", src_seg_num);
//printk("src_seg_offset = %u\n", src_seg_offset);
//printk("src_seg_remaining = %u\n", src_seg_remaining);
//printk("payload_remaining = %u\n\n", payload_remaining);

                // Can all the data in the current src segment be accomodated
                // in the current output packet?
                if (payload_remaining >= src_seg_remaining) {
                    // Yes, so consume all src segment's data
                    payload_remaining -= src_seg_remaining;
                    src_length = src_seg_remaining;

                    // Move the the next src buffer
                    src_seg_offset = 0;
                    ++src_seg_num;
                } else {
                    // No, so fill remaining space in current output packet from
                    // the current src segment
                    src_length = payload_remaining;
                    src_seg_offset += src_length;
                    payload_remaining = 0;
                }
//printk("payload_remaining = %u\n", payload_remaining);
//printk("src_length = %u\n", src_length);
//printk("src_seg_num = %u\n", src_seg_num);
//printk("src_seg_offset = %u\n\n", src_seg_offset);

                // Calculate index of this fragment in the src SG-DMA scatterlist
                src_desc = num_src_buffers++;
//printk("src_desc = %u\n", src_desc);

                // Add the length of the src fragment to the src SG entry list
                src_scatterlist[src_desc].length = src_length;

                // Calculate the virtual address of the src fragment from either
                // the skb's integral buffer, or the current fragment
                src_scatterlist[src_desc].__address = src_data ? src_data : page_address(page) + page_offset;

                if (src_data) {
                    // Create a DMA mapping for the src buffer from the skb's
                    // integral buffer
                    dma_address = dma_map_single(0,
                                                 src_data,
                                                 src_scatterlist[src_desc].length,
                                                 DMA_TO_DEVICE);
                } else {
                    // Create a DMA mapping for the src buffer from the current
                    // fragment page description
                    dma_address = dma_map_page(0,
                                               page,
                                               page_offset,
                                               src_scatterlist[src_desc].length,
                                               DMA_TO_DEVICE);
                }

                // Start address fragment src SG list entry
                src_scatterlist[src_desc].dma_address = dma_address;
//printk("End-Loop\n");
            } while (payload_remaining);

            // Update record of data consumed for source buffers
            used_data += payload_length;

            // Split the destination data between the SRAM buffers and setup
            // dst scatterlist entries for each SRAM output buffer
//printk("Dealing with output buffers\n");
            dst_remaining_length = packet_length;
            for (i=0; i < num_dst_buffers; ++i) {
                // Allocate a SRAM buffer for the packet using blocking call
                sram_info[i] = alloc_sram_buffer(priv);
                BUG_ON(!sram_info[i]);

                // Fill as much of the packet as there is dst data remaining
                sram_info[i]->packet_length_ =
                    (sram_info[i]->buffer_size_ < dst_remaining_length) ?
                        sram_info[i]->buffer_size_ : dst_remaining_length;
//printk("Output packet of length %lu\n", sram_info[i]->packet_length_);

                dst_scatterlist[i].length      = sram_info[i]->packet_length_;  // Length of buffer
                dst_scatterlist[i].dma_address = sram_info[i]->p_buffer_;       // Physical address of buffer

                dst_remaining_length -= sram_info[i]->packet_length_;
            }
            // Null terminate SRAM buffer pointer array
            sram_info[i] = 0;

//printk("Performing SG-DMA\n");
//for (i=0; i < num_src_buffers; ++i) {
//    struct scatterlist* sg = src_scatterlist + i;
//    printk("0x%08x, %u\n", sg->dma_address, sg->length);
//}
//for (i=0; i < num_dst_buffers; ++i) {
//    struct scatterlist* sg = dst_scatterlist + i;
//    printk("0x%08x, %u\n", sg->dma_address, sg->length);
//}

            // Setup SG-DMA transfer without checksumming
            oxnas_dma_set_sg(
                channel,
                src_scatterlist,
                num_src_buffers,
                dst_scatterlist,
                num_dst_buffers,
                OXNAS_DMA_MODE_INC,
                OXNAS_DMA_MODE_INC,
                0);

            // Perform the SG DMA and wait for completion
            oxnas_dma_start(channel);
            while (down_interruptible(&priv->tx_offload_semaphore_));

            // Check for error from the SG DMA controller
            BUG_ON(priv->tx_dma_status_ != OXNAS_DMA_ERROR_CODE_NONE);

            // Release src DMA mapping for the ETH+IP+TCP/UDP header
            dma_unmap_single(0, src_scatterlist[0].dma_address, hdr_len, DMA_TO_DEVICE);

            // Release src mapping for all payload sections
            for (i=2; i < num_src_buffers; ++i) {
                dma_unmap_page(0, src_scatterlist[i].dma_address, src_scatterlist[i].length, DMA_TO_DEVICE);
            }

            // Adjust the DMA transfer info for the first SRAM buffer for the
            // packet to skip the ETH+IP header, as this should not contribute
            // to the TCP/UDP checksum
            dst_scatterlist[0].length      -= eth_ip_hdrlen;
            dst_scatterlist[0].dma_address += eth_ip_hdrlen;

            // Set the high order address bit, which indicates that checksumming
            // should be performed, for all the SRAM buffers used for the packet
            for (i=0; i < num_dst_buffers; ++i) {
                // Set the high order address bit which indicates to the DMA
                // controller that it should accumulate the checksum
                dst_scatterlist[i].dma_address |= (1UL << OXNAS_DMA_CSUM_ENABLE_ADR_BIT);
            }

            // If padding is required, increase the length of the last desti-
            // nation DMA transfer list entry by one to accomodate the pad byte
            // and set pad byte to zero
            if (pad_required) {
                int last_dst_buffer = num_dst_buffers - 1;
                *(u8*)(sram_info[last_dst_buffer]->v_buffer_ + sram_info[last_dst_buffer]->packet_length_) = 0;
                ++dst_scatterlist[num_dst_buffers - 1].length;
//printk("Padded last SRAM destination buffer with %d zero bytes\n", pad_required);
            }

            // Setup SG-DMA transfer with checksumming from the SRAM buffers
            // holding the TCP/UDP header and payload. Have to copy back to the
            // SRAM buffers, as SG-DMA appears not to cope with fixed source or
            // destination transfers
            oxnas_dma_set_sg(
                channel,
                dst_scatterlist,
                num_dst_buffers,
                dst_scatterlist,
                num_dst_buffers,
                OXNAS_DMA_MODE_INC,
                OXNAS_DMA_MODE_INC,
                1);

            // Perform the SG, CSUMing DMA and wait for completion
            oxnas_dma_start(channel);
            while (down_interruptible(&priv->tx_offload_semaphore_));

            // Check for error from the SG DMA controller
            if (priv->tx_dma_status_ != OXNAS_DMA_ERROR_CODE_NONE) {
                panic("SG-DMA error code 0x%08x\n", priv->tx_dma_status_);
            }

            // Extract the h/w calculated payload checksum from the DMA controller
            hw_csum = ~priv->tx_csum_result_;

            // Combine unchanging part of pseudo header with the TCP/UDP payload
            // length (which includes the length of the TCP/UDP header) for this
            // output packet and the h/w calculated payload csum and store in
            // TCP/UDP header in SRAM output packet
            network_order_payload_length = htons(packet_length - eth_ip_hdrlen);

            *(u16*)(sram_info[0]->v_buffer_ + payload_csum_offset) =
                csum_fold(csum_partial((u8*)&network_order_payload_length,
                                       sizeof(u16),
                                       csum_partial((u8*)&hw_csum,
                                                    sizeof(u16),
                                                    pseudo_header_csum)));

//printk("%u output buffers\n", num_dst_buffers);
//for (j=0; sram_info[j]; ++j) {
//    printk("Output buffer %d:\n", j);
//    for (i=0; i < sram_info[j]->packet_length_;) {
//        printk("0x%02x ", (unsigned)sram_info[j]->v_buffer_[i++]);
//        if (!(i%16)) {
//            printk("\n");
//        }
//    }
//    printk("\n");
//}

            // Add a descriptor to GMAC TX list for each SRAM buffer containing
            // the output packet
            spin_lock_irqsave(&priv->tx_spinlock_, irq_flags);
            desc = set_tx_descriptor(priv, 0, num_dst_buffers, 0, sram_info);
            if (desc < 0) {
                // Remember this pending packet, so that finish_xmit() can
                // queue it when the queue restarts, then sleep here until the
                // queue is restarted
printk("Failed to add Tx descriptor, recording pending packet and stopping TX queue\n");

                // Keep a record of pending TX packet
                priv->tx_pending_skb       = 0;
                priv->tx_pending_sram_info = sram_info;
                priv->tx_pending_dma_addr  = 0;
                priv->tx_pending_length    = num_dst_buffers;

                // Stop further calls to hard_start_xmit() until some descriptors
                // are freed up by already queued TX packets being completed
                netif_stop_queue(priv->netdev);

                spin_unlock_irqrestore(&priv->tx_spinlock_, irq_flags);

                // Wait for finish_xmit(), or stopping TX offload to wake us
                while (down_interruptible(&priv->tso_netif_resumed_semaphore_));

                // Must abandon processing any more TSO segments for current
                // packet if TX offload has been requested to stop while we
                // were waiting for netif to resume, so that we cannot block
                // again due to insufficient TX descriptors
                abandon_tso = is_tx_offload_stop_request_pending(priv);
            } else {
                // Record start of transmission, so timeouts will work once
                // they're implemented
                priv->netdev->trans_start = jiffies;

                // Poke the transmitter to look for available TX descriptors, as
                // we have just added one, in case it had previously found there
                // were no more pending transmission
                dma_reg_write(priv, DMA_TX_POLL_REG, 0);

                spin_unlock_irqrestore(&priv->tx_spinlock_, irq_flags);
            }
        }

        // Free the storage for the src scatterlist entries
        kfree(src_scatterlist);

        // Release the SDRAM socket buffer and resources back to the system
        dev_kfree_skb(tx_offload_job->skb_);
        kfree(tx_offload_job);
    }

    // Release the DMA channel
    oxnas_dma_set_callback(channel, 0, 0);
    oxnas_dma_free(channel);

    // Free the storage for the src scatterlist entries
    kfree(dst_scatterlist);
}

#else // ENABLE_TX_TSO_OFFLOAD

static void tx_offload_work(void* data)
{
    gmac_priv_t* priv = (gmac_priv_t*)data;
DBG(1, KERN_INFO "$Wtx_offload_work() Called\n");

    // Process all outstanding TX offload jobs
    while (1) {
        struct list_head        *entry = 0;
        tx_offload_job_t        *tx_offload_job = 0;
        unsigned long            pseudo_header_csum = 0;
        unsigned short           payload_csum_off = 0;
        unsigned short           payload_length = 0;
        unsigned char           *payload = 0;
        dma_addr_t               dma_address = 0;
        struct sk_buff          *skb;
        unsigned                 nr_frags;
        unsigned                 packet_length;
        int                      pad_required = 0;

        // Check for a request to stop Tx offload job processing
        if (priv->tx_work_queue_stop_flag_) {
            priv->tx_work_queue_stop_flag_ = 0;
            up(&priv->tx_work_queue_stop_semaphore_);
            break;
        }

        // Unlink the earliest queued TX offload job from queue of pending jobs
        spin_lock(&priv->tx_offload_lock_);
        if (!list_empty(&priv->tx_offload_list_)) {
            // Remove the first entry on the list
            entry = priv->tx_offload_list_.next;
            list_del(entry);
        }
        spin_unlock(&priv->tx_offload_lock_);

        if (!entry) {
            // No offload jobs pending
            break;
        }

        // Get pointer to job from it's list_head member
        tx_offload_job = list_entry(entry, tx_offload_job_t, list_);
        BUG_ON(!tx_offload_job);

DBG(1, KERN_INFO "$WProcessing job %lu\n", tx_offload_job->number_);
        // Get the network packet details from for the offload job
        skb = tx_offload_job->skb_;
        BUG_ON(!skb);

        nr_frags = skb_shinfo(skb)->nr_frags;
        packet_length = skb->len;

        // Is hardware checksumming required?
        if (likely(skb->ip_summed == CHECKSUM_HW)) {
DBG(1, KERN_INFO "TX CSUM required, Ethernet protocol = 0x%04hx, 0x%04hx\n", skb->protocol, ntohs(skb->protocol));
            if (ntohs(skb->protocol) == ETH_P_IP) {
                struct iphdr* iphdr     = skb->nh.iph;
                unsigned      iphdr_len = iphdr->ihl*4;

                payload = skb->h.raw;
                payload_length = ntohs(iphdr->tot_len) - iphdr_len;
DBG(39, KERN_INFO "$Ypayload_length = %u\n", payload_length);

                // Odd length payloads cannot be correctly checksummed by the
                // h/w, we must append a zero pad byte to the DMA transfer and
                // include this in the h/w checksumming operation
                pad_required = (payload_length & 1);
DBG(39, KERN_INFO "$Ypad_required = %u\n", pad_required);

                switch (iphdr->protocol) {
                    case IPPROTO_TCP:
                        DBG(1, KERN_INFO "  TCP\n");
                        payload_csum_off = (ETH_HLEN + iphdr_len + offsetof(struct tcphdr, check));
                        break;
                    case IPPROTO_UDP:
                        DBG(1, KERN_INFO "  UDP\n");
                        payload_csum_off = (ETH_HLEN + iphdr_len + offsetof(struct udphdr, check));
                        break;
                    default:
                        panic("  other IP encapsulated protocol %u\n", (unsigned)iphdr->protocol);
                }

                if (payload_csum_off) {
                    // Ensure the payload checksum field in src is zero'ed before
                    // calculating the checksum
DBG(1, KERN_INFO "$RZeroing checksum word at offset %u\n", payload_csum_off);
                    *(u16*)(skb->data + payload_csum_off) = 0;

                    // Network stack computes the IP header csum in any case, so
                    // just compute payload pseudo-header checksum
                    pseudo_header_csum = csum_tcpudp_nofold(
                        iphdr->saddr,
                        iphdr->daddr,
                        payload_length,
                        iphdr->protocol,
                        0);
DBG(1, KERN_INFO "$Rpseudo_header_csum = 0x%08lx\n", pseudo_header_csum);
                }
            }
        }

if ((skb_shinfo(skb)->nr_frags != 0) || (skb_shinfo(skb)->frag_list || skb_shinfo(skb)->tso_size)) {
    int i;
    printk("$CNon-zero fragment count = %d, skb->len = %u, skb->data_len = %u, tso_segs = %u, head_len = %u, tso_size = %hu\n",
        skb_shinfo(skb)->nr_frags, skb->len, skb->data_len, skb_shinfo(skb)->tso_segs, skb_headlen(skb), skb_shinfo(skb)->tso_size);
    for (i=0; i < skb_shinfo(skb)->nr_frags; i++) {
        printk("$C  frag[%d].size = %u\n", i, skb_shinfo(skb)->frags[i].size);
    }
    if (skb_shinfo(skb)->frag_list) {
        struct sk_buff *list = skb_shinfo(skb)->frag_list;
        for (; list; list = list->next) {
            printk("$C  frag_list[%d].len = %u\n", i, list->len);
        }
    }
}

        if (skb_shinfo(skb)->tso_size) {
            printk("$YTX PACKET REQUIRES TSO PROCESSING - DROPPING\n");
        } else {
            // If hardware csum or SG required, will DMA source data into SRAM
            // buffer, from which GMAC will extract packet for transmission
            struct scatterlist       *src_scatterlist;
            struct scatterlist       *dst_scatterlist;
            unsigned                  ip_hdr_len;
            unsigned                  frag;
            oxnas_dma_channel_t      *channel;
            gmac_sram_buffer_info_t **sram_info = 0;
            unsigned                  num_dst_buffers;
            unsigned                  num_src_buffers;
            unsigned long             dst_remaining_length;
            int                       i;

DBG(1, KERN_INFO "Require SRAM buffer for csummed/de-SG'ed packet\n");
            // Allocate SG descriptor array for source of DMA transfer, two
            // entries for the IP header and the remainder of the packet in the
            // fragment referenced by the skb directly, then one for each
            // fragment
            num_src_buffers = nr_frags + 2;
            src_scatterlist = (struct scatterlist*)kmalloc(sizeof(struct scatterlist) * num_src_buffers, GFP_KERNEL);
            BUG_ON(!src_scatterlist);

            // Create DMA SG entry to DMA the IP header from SDRAM packet to the
            // SRAM buffer - do not want this checksummed
            ip_hdr_len = skb->nh.iph->ihl*4;
            src_scatterlist[0].length = ETH_HLEN + ip_hdr_len;

//printk("src_scatterlist[0]: len = %u\n", src_scatterlist[0].length);
//for (i=0; i<src_scatterlist[0].length;) {
//    printk("0x%02x ", (unsigned)skb->data[i++]);
//    if (!(i%16)) {
//        printk("\n");
//    }
//}
//printk("\n");

            // Create DMA SG entry to DMA the first part of the possibly frag-
            // mented payload into SRAM buffer, calculating checksum if required
            src_scatterlist[1].length =
                skb_is_nonlinear(skb) ? skb_headlen(skb) - src_scatterlist[0].length :
                                        skb->len - src_scatterlist[0].length;

//printk("src_scatterlist[1]: len = %u, non-quad = %s\n", src_scatterlist[1].length, (src_scatterlist[1].length%4) ? "$RYES": "no");
//for (i=0; i<src_scatterlist[1].length;) {
//    printk("0x%02x ", (unsigned)(skb->data + src_scatterlist[0].length)[i++]);
//    if (!(i%16)) {
//        printk("\n");
//    }
//}
//printk("\n");

            // Create a DMA mapping for the first (and possibly only) source
            // packet fragment, containing the IP header and the remainder of
            // the packet in the fragment referenced by the skb directly. Must
            // include zero'ed pad byte if padding is required and this is the
            // only packet section
            if (pad_required && (num_src_buffers == 2)) {
                // Zero a trailing pad byte and include in DMA transfer length
                skb->data[src_scatterlist[0].length + src_scatterlist[1].length] = 0;
                ++src_scatterlist[1].length;
            }
            dma_address = dma_map_single(0,
                                         skb->data,
                                         src_scatterlist[0].length + src_scatterlist[1].length,
                                         DMA_TO_DEVICE);

            // Add DMA address of start of IP header src SG list entry
            src_scatterlist[0].dma_address = dma_address;

            // Record the virtual address for debugging purposes
            src_scatterlist[0].__address = skb->data;

            // Do we want to checksum the remainder of the fragment directly
            // referenced by the skb?
            if (payload_csum_off) {
                // Yes, so set the high order address bit which indicates to the
                // SG-DMA controller that it should accumulate the checksum
                dma_address |= (1UL << OXNAS_DMA_CSUM_ENABLE_ADR_BIT);
            }
            // Add DMA address of start of remainder of the fragment directly
            // referenced by the skb entry to SG src list
            src_scatterlist[1].dma_address = dma_address + src_scatterlist[0].length;

            // Record the virtual address for debugging purposes
            src_scatterlist[1].__address = skb->data + src_scatterlist[0].length;
DBG(39, KERN_INFO "skb->data_len = %u, skb->len = %u, iphdr @ 0x%08x, rest @ 0x%08x\n", skb->data_len, skb->len, src_scatterlist[0].dma_address, src_scatterlist[1].dma_address);

            // Create DMA SG entries to DMA the subsequent fragments of the
            // payload into SRAM buffer, calculating checksum if required
            for (frag=0; frag < nr_frags; ++frag) {
                struct skb_frag_struct* skb_frag = &skb_shinfo(skb)->frags[frag];
                // Skip the descriptor entries for the IP header and remainder
                // of the packet in the fragment referenced by the skb directly
                unsigned src_desc = frag+2;

                // Add length of fragment to the src SG list entry
                src_scatterlist[src_desc].length = skb_frag->size;

                // Calculate the virtual address of the fragment
                src_scatterlist[src_desc].__address = page_address(skb_frag->page) + skb_frag->page_offset;

                // If padding is required and this is the last fragment
                if (pad_required && (src_desc == (num_src_buffers-1))) {
                    // Zero a trailing pad byte and include in DMA transfer length
                    *(src_scatterlist[src_desc].__address + src_scatterlist[src_desc].length) = 0;
                    ++src_scatterlist[src_desc].length;
                }

//printk("src_scatterlist[src_desc]: len = %u, non-quad = %s\n", src_scatterlist[src_desc].length, (src_scatterlist[src_desc].length%4) ? "$RYES": "no");
//for (i=0; i<src_scatterlist[src_desc].length;) {
//    printk("0x%02x ", (unsigned)((unsigned char*)(page_address(skb_frag->page) + skb_frag->page_offset))[i++]);
//    if (!(i%16)) {
//        printk("\n");
//    }
//}
//printk("\n");

                // Create a DMA mapping of the virtual page
                dma_address = dma_map_page(0,
                                           skb_frag->page,
                                           skb_frag->page_offset,
                                           src_scatterlist[src_desc].length,
                                           DMA_TO_DEVICE);

                // Do we want to checksum the fragment?
                if (payload_csum_off) {
                    // Yes, so set the high order address bit which indicates to the
                    // SG-DMA controller that it should accumulate the checksum
                    dma_address |= (1UL << OXNAS_DMA_CSUM_ENABLE_ADR_BIT);
                }

                // Start address fragment src SG list entry
                src_scatterlist[src_desc].dma_address = dma_address;
DBG(39, KERN_INFO "src_scatterlist[%d].length = %u, adr = 0x%08x\n", src_desc, src_scatterlist[src_desc].length, src_scatterlist[src_desc].dma_address);
            }

            // Create entries for the SG destination list to describe enough
            // SRAM buffers to cover the entire packet
            num_dst_buffers = (packet_length >> (SRAM_NET_BUFFER_SIZE_POW2));
            if ((num_dst_buffers << (SRAM_NET_BUFFER_SIZE_POW2)) < packet_length) {
                ++num_dst_buffers;
            }
DBG(39, KERN_INFO "%u dst sg entries\n", num_dst_buffers);

            dst_scatterlist = (struct scatterlist*)kmalloc(sizeof(struct scatterlist) * num_dst_buffers, GFP_KERNEL);
            BUG_ON(!dst_scatterlist);

            // Allocate an array to hold the SRAM buffer pointers, with space
            // for a null-terminating entry
            sram_info = (gmac_sram_buffer_info_t**)kmalloc(sizeof(gmac_sram_buffer_info_t*) * (num_dst_buffers + 1), GFP_KERNEL);
            BUG_ON(!sram_info);

            // Split the destination data between the SRAM buffers
            dst_remaining_length = packet_length;
            for (i=0; (i < num_dst_buffers) && dst_remaining_length; ++i) {
                // Allocate a SRAM buffer for the packet using blocking call
                sram_info[i] = alloc_sram_buffer(priv);
                BUG_ON(!sram_info[i]);

                // Fill as much of the packet as there is destination data remaining
                sram_info[i]->packet_length_ =
                    (sram_info[i]->buffer_size_ < dst_remaining_length) ?
                        sram_info[i]->buffer_size_ : dst_remaining_length;

                dst_scatterlist[i].length      = sram_info[i]->packet_length_;  // Length of buffer
                dst_scatterlist[i].dma_address = sram_info[i]->p_buffer_;       // Physical address of buffer
DBG(39, KERN_INFO "dst_scatterlist[%d].length = %u\n", i, dst_scatterlist[i].length);

                dst_remaining_length -= sram_info[i]->packet_length_;
            }
            // Null terminate SRAM buffer pointer array
            sram_info[i] = 0;

            // If padding is required, increase the length of the last desti-
            // nation DMA transfer list entry by one to accomodate the pad byte
            if (pad_required) {
                ++dst_scatterlist[num_dst_buffers - 1].length;
            }

            // Allocate a DMA channel
            channel = oxnas_dma_request(1);

            // Setup the SG, possibly checksumming, transfer. This could block
            // while attempting to obtain the checksum engine
            oxnas_dma_set_sg(
                channel,
                src_scatterlist,
                nr_frags+2,
                dst_scatterlist,
                num_dst_buffers,
                OXNAS_DMA_MODE_INC,
                OXNAS_DMA_MODE_INC,
                payload_csum_off);

            // Perform the SG, CSUMing DMA and wait for completion
            oxnas_dma_set_callback(channel, tx_offload_dma_callback, priv);
            oxnas_dma_start(channel);
            while (down_interruptible(&priv->tx_offload_semaphore_));

//printk("Destination before csum insertion into payload header\n");
//i=0;
//while (sram_info[i]) {
//    int j;
//    printk("sram_info[%d] len = %lu\n", i, sram_info[i]->packet_length_);
//    for (j=0; j < sram_info[i]->packet_length_;) {
//        printk("0x%02x ", (unsigned)sram_info[i]->v_buffer_[j++]);
//        if (!(j%16)) {
//            printk("\n");
//        }
//    }
//    ++i;
//    printk("\n");
//}

            // If required, accumulate the full packet checksum from the pseudo
            // header and payload parts. Do this before releasing the DMA
            // channel to ensure exclusive use of the tx_csum_result_ field, as
            // there's only a single H/W DMA checksumming engine
            if (payload_csum_off) {
                u16 hw_csum = ~priv->tx_csum_result_;
//{
//u32 sw_csum = pseudo_header_csum;
//unsigned offset = src_scatterlist[0].length;
//i = 0;
//printk("$MInitial offset = %u\n", offset);
//do {
//    sw_csum = csum_partial(sram_info[i]->v_buffer_ + offset, sram_info[i]->packet_length_ - offset, sw_csum);
//    offset = 0;
//} while (sram_info[++i]);
//printk("$MS/W generated dst csum = 0x%04hx\n", csum_fold(sw_csum));
//
//sw_csum = csum_partial(src_scatterlist[1].__address, src_scatterlist[1].length, pseudo_header_csum);
//printk("$MS/W generated src csum = 0x%04hx\n", csum_fold(sw_csum));
//}

//printk("$Chw_csum            = 0x%04hx\n", hw_csum);
//printk("$Cpseudo_header_csum = 0x%08x\n", (u32)pseudo_header_csum);
//printk("$Cfolded csum        = 0x%04hx\n", csum_fold(csum_partial((u8*)&hw_csum, sizeof(u16), pseudo_header_csum)));
                *(u16*)(sram_info[0]->v_buffer_ + payload_csum_off) =
                    csum_fold(csum_partial((u8*)&hw_csum, sizeof(u16), pseudo_header_csum));
//printk("$CWritten to SRAM    = 0x%04hx\n", *(u16*)(sram_info[0]->v_buffer_ + payload_csum_off));
            }

            // Release the DMA channel
            oxnas_dma_set_callback(channel, 0, 0);
            oxnas_dma_free(channel);

            // Release the DMA mappings for the source packet fragment buffers
            // in SDRAM, accounting for the IP header and remainder of first
            // fragment being in the same mapped SDRAM region
            dma_unmap_single(0,
                             src_scatterlist[0].dma_address,
                             src_scatterlist[0].length + src_scatterlist[1].length,
                             DMA_TO_DEVICE);

            for (frag=0; frag < nr_frags; ++frag) {
                // Ensure the checksum enabling high order address bit is not
                // set as this would confuse the DMA mapping release function
                unsigned src_desc = frag+2;
                src_scatterlist[src_desc].dma_address &= ~(1UL << OXNAS_DMA_CSUM_ENABLE_ADR_BIT);

DBG(1, KERN_INFO "Freeing src sg frag %d of length = %u\n", src_desc, src_scatterlist[src_desc].length);
                dma_unmap_page(0,
                               src_scatterlist[src_desc].dma_address,
                               src_scatterlist[src_desc].length,
                               DMA_TO_DEVICE);
            }

//{
//int k=0;
//int l=0;
//unsigned char* dst = sram_info[k]->v_buffer_;
//printk("$Gpayload_csum_off = %u, src[0] len = %u\n", payload_csum_off, src_scatterlist[0].length);
//for (i=0; (i < (nr_frags+2)) && sram_info[k]; ++i) {
//    unsigned char* src = src_scatterlist[i].__address;
//    int j;
//    for (j=0; j < src_scatterlist[i].length;) {
//        if (*src != *dst) {
//            printk("$Rmismatch at src buf %u, offset %u and dst buf %u, offset %u, src = 0x%08x, dst = 0x%08x\n", i, j, k, l, (unsigned)*src, (unsigned)*dst);
//        }
//        ++src;
//        ++dst;
//        if (++l == sram_info[k]->packet_length_) {
//            if (!sram_info[++k]) {
//                break;
//            }
//            dst = sram_info[k]->v_buffer_;
//            l=0;
//        }
//        ++j;
//    }
//}
//}

            // Deallocate the SG descriptor arrays
            kfree(src_scatterlist);
            kfree(dst_scatterlist);

            // For each SRAM buffer, add a descriptor to GMAC TX list
            {
                unsigned long irq_flags;
                int desc;

                // SMP-safe protection against concurrent operations in ISR,
                // hard_start_xmit() and here
                spin_lock_irqsave(&priv->tx_spinlock_, irq_flags);

                // Construct the GMAC specific DMA descriptor chain for the
                // array to SRAM buffer info structures
                desc = set_tx_descriptor(priv, 0, num_dst_buffers, 0, sram_info);
                if (desc < 0) {
DBG(1, KERN_INFO "$Rtx_offload_work() %s: No more free TX descriptors\n", priv->netdev->name);
                    // Not keeping record of pending TX packet, as may be lots
                    // queued up for this worker-thread routine to send
                    priv->tx_pending_skb       = 0;
                    priv->tx_pending_sram_info = 0;
                    priv->tx_pending_dma_addr  = 0;
                    priv->tx_pending_length    = 0;

                    // Stop further calls to hard_start_xmit() until some
                    // desciptors are freed up by already queued TX packets
                    // being completed
printk("$YStopping TX queue due to no descriptors available\n");
                    netif_stop_queue(priv->netdev);
                } else {
DBG(39, KERN_INFO "$Rtx_offload_work() %s: Set tx descriptor %d, buf[0] = 0x%08x len[0] = %lu, sram_info = 0x%08x\n", priv->netdev->name, desc, sram_info[0]->p_buffer_, sram_info[0]->packet_length_, (u32)sram_info);
                    // Record start of transmission, so timeouts will work once they're implemented
                    priv->netdev->trans_start = jiffies;

                    // Poke the transmitter to look for available TX descriptors, as we have
                    // just added one, in case it had previously found there were no more
                    // pending transmission
                    dma_reg_write(priv, DMA_TX_POLL_REG, 0);
                }

                spin_unlock_irqrestore(&priv->tx_spinlock_, irq_flags);
            }
        }

        // Release the SDRAM socket buffer and resources back to the system
        dev_kfree_skb_irq(tx_offload_job->skb_);
        kfree(tx_offload_job);
    }
}
#endif // ENABLE_TX_TSO_OFFLOAD
#endif // ENABLE_TX_OFFLOAD

#ifdef ENABLE_RX_OFFLOAD
static void rx_offload_work(void* data)
{
    gmac_priv_t* priv = (gmac_priv_t*)data;
    int available;

    // Process all outstanding RX offload jobs
    while (1) {
        struct list_head *entry = 0;
        rx_offload_job_t *rx_offload_job = 0;
        struct sk_buff *skb = 0;

        // Check for a request to stop Rx offload job processing
        if (priv->rx_work_queue_stop_flag_) {
            priv->rx_work_queue_stop_flag_ = 0;
            up(&priv->rx_work_queue_stop_semaphore_);
            break;
        }

        // Unlink the earliest queued RX offload job from queue of pending jobs
        spin_lock_bh(&priv->rx_offload_lock_);
        if (!list_empty(&priv->rx_offload_list_)) {
            // Remove the first entry on the list
            entry = priv->rx_offload_list_.next;
            list_del(entry);
        }
        spin_unlock_bh(&priv->rx_offload_lock_);

        if (!entry) {
            // No offload jobs pending
            break;
        }

        // Get pointer to job from it's list_head member
        rx_offload_job = list_entry(entry, rx_offload_job_t, list_);
        BUG_ON(!rx_offload_job);

        // Allocate a network buffer to hold the entire Rx packet
        skb = dev_alloc_skb(rx_offload_job->packet_length_ + NET_IP_ALIGN + EXTRA_RX_SKB_SPACE);
        if (skb) {
            oxnas_dma_channel_t *channel;
            struct scatterlist   dst_scatterlist;
            unsigned             i;
            int                  dma_length;
            dma_addr_t           dma_address;

            // Create a source scatterlist with an entry for each
            // SRAM buffer
            struct scatterlist src_scatterlist[rx_offload_job->sram_info_entry_count_];
            for (i=0; i < rx_offload_job->sram_info_entry_count_; ++i) {
                src_scatterlist[i].dma_address = rx_offload_job->sram_info_[i]->p_buffer_;
                src_scatterlist[i].length      = rx_offload_job->sram_info_[i]->packet_length_;
            }

            // Align IP header to quad boundary
            skb_reserve(skb, NET_IP_ALIGN);

            // Create DMA mapping for DMA from SRAM to SDRAM, mapping only the
            // part of the SDRAM dst buffer that will be involved in the DMA
            // transfer
            dma_length = rx_offload_job->packet_length_;
            dma_address = dma_map_single(0, skb->tail, dma_length, DMA_FROM_DEVICE);
            BUG_ON(dma_mapping_error(dma_address));

            // Create a destination scatterlist entry for the SDRAM network
            // buffer to DMA only the valid data in the SRAM buffer into SDRAM
            dst_scatterlist.dma_address = dma_address;
            dst_scatterlist.length      = dma_length;

            // Allocate a DMA channel
            channel = oxnas_dma_request(1);

            // SG-DMA from multiple SRAM buffers to single network
            // buffer, including IP header alignment offset
            oxnas_dma_set_sg(
                channel,
                src_scatterlist,
                rx_offload_job->sram_info_entry_count_,
                &dst_scatterlist,
                1,
                OXNAS_DMA_MODE_INC,
                OXNAS_DMA_MODE_INC,
                0);

            // Perform the SG, CSUMing DMA and wait for completion
            oxnas_dma_set_callback(channel, rx_offload_dma_callback, priv);
            oxnas_dma_start(channel);
            while (down_interruptible(&priv->rx_offload_semaphore_));

            // Check for error from the SG DMA controller
            BUG_ON(priv->rx_dma_status_ != OXNAS_DMA_ERROR_CODE_NONE);

            // Release the DMA channel
            oxnas_dma_set_callback(channel, 0, 0);
            oxnas_dma_free(channel);

            // Release the DMA mapping
            dma_unmap_single(0, dma_address, dma_length, DMA_FROM_DEVICE);
        }

        // Return all Rx SRAM buffers for the packet to the free pool
        while (rx_offload_job->sram_info_entry_count_) {
            free_sram_buffer(priv, rx_offload_job->sram_info_[--rx_offload_job->sram_info_entry_count_]);
        }

        if (!skb) {
            DBG(1, KERN_WARNING "rx_offload_work() %s: Failed to allocate Rx network buffer\n", priv->netdev->name);
        } else {
            // Do all receive processing for the packet
            skb = process_rx_packet(priv, rx_offload_job->packet_length_, rx_offload_job->desc_status_, skb);

            // If we have a socket buffer available and there is room to
            // queue a new RX descriptor
            if (skb) {
                // Free the socket buffer, as we couldn't make use
                // of it to refill the RX descriptor ring
                DBG(10, "$B[PL F]$n");
                dev_kfree_skb(skb);
            }
        }

        kfree(rx_offload_job);
    }

    // Attempt to fill all available slots in the RX descriptor ring
    spin_lock_bh(&priv->rx_spinlock_);
    available = available_for_write(&priv->rx_gmac_desc_list_info);
    spin_unlock_bh(&priv->rx_spinlock_);
    if (available) {
        refill_rx_ring(priv->netdev, 0, 1);
    }
}
#endif // ENABLE_RX_OFFLOAD

static inline void fix_frag_ip_csum(struct sk_buff *skb)
{
#ifdef CORRECT_FRAG_IP_CSUM
    // Cope with apparently broken Linux network stack csum calc for fragmented
    // IP frames
    if (ntohs(((struct ethhdr*)skb->data)->h_proto) == ETH_P_IP) {
        // If more fragments or non-zero fragment offset, recalculate the csum
        u16 frag_info = ntohs(skb->nh.iph->frag_off);
        if ((frag_info & 0x1fff) || (frag_info & 0x2000)) {
            unsigned short fast_csum;
//            unsigned short orig_csum = skb->nh.iph->check;
            skb->nh.iph->check = 0;
            fast_csum = ip_fast_csum(skb->nh.raw, skb->nh.iph->ihl);
//            printk("IP: fast csum = 0x%04hx, original csum = 0x%04hx\n", fast_csum, orig_csum);
            skb->nh.iph->check = fast_csum;
        }
    }
#endif // CORRECT_FRAG_IP_CSUM
}

#if defined(CONFIG_LEON_COPRO) && defined(CONFIG_LEON_OFFLOAD_TX)
static int hard_start_xmit(
    struct sk_buff *skb,
    struct net_device *dev)
{
    gmac_priv_t                *priv = (gmac_priv_t*)netdev_priv(dev);
    volatile gmac_tx_que_ent_t *job;
    unsigned long               irq_flags;

    if (skb_shinfo(skb)->frag_list) {
        panic("Frag list - can't handle this!\n");
    }

    // Fix apparent failure of network stack to correctly checksum fragmented
    // IP packets
    fix_frag_ip_csum(skb);

    // Protection against concurrent operations in ISR and hard_start_xmit()
    if (!spin_trylock_irqsave(&priv->tx_spinlock_, irq_flags)) {
        return NETDEV_TX_LOCKED;
    }

    // NETIF_F_LLTX apparently introduces a potential for hard_start_xmit() to
    // be called when the queue has been stopped (although I think only in SMP)
    // so do a check here to make sure we should proceed
    if (netif_queue_stopped(dev)) {
        spin_unlock_irqrestore(&priv->tx_spinlock_, irq_flags);
        return NETDEV_TX_BUSY;
    }

    job = tx_que_get_idle_job(dev);
    if (!job) {
//printk("Q");
        // Tx offload queue is full, so add skb to pending skb list
        list_add_tail((struct list_head*)&skb->cb, &priv->copro_tx_skb_list_);

        // Keep track of how many entries are in the pending SKB list
        ++priv->copro_tx_skb_list_count_;

        // Have we queued the max allowed number of SKBs?
        if (priv->copro_tx_skb_list_count_ >= COPRO_MAX_QUEUED_TX_SKBS) {
            // Stop further calls to hard_start_xmit() until some descriptors
            // are freed up by already queued TX packets being completed
            netif_stop_queue(dev);
        }
    } else {
        if (priv->copro_tx_skb_list_count_) {
//printk("H");
            // Have queued pending SKBs, so add new SKB to tail of pending list
            list_add_tail((struct list_head*)&skb->cb, &priv->copro_tx_skb_list_);

            // Keep track of how many entries are in the pending SKB list
            ++priv->copro_tx_skb_list_count_;

            // Process pending SKBs, oldest first
            copro_process_pending_tx_skbs(dev, job);
        } else {
//printk("C");
            // Fill the Tx offload job with the network packet's details
            copro_fill_tx_job(job, skb);

            // Enqueue the new Tx offload job with the CoPro
            tx_que_new_job(dev, job);
        }

        // Record start of transmission, so timeouts will work once they're
        // implemented
        dev->trans_start = jiffies;

        // Interrupt the CoPro to cause it to examine the Tx offload queue
        wmb();
        writel(1UL << COPRO_SEM_INT_TX, SYS_CTRL_SEMA_SET_CTRL);

        // If the network stack's Tx queue was stopped and we now have resources
        // to process more Tx offload jobs
        if (netif_queue_stopped(dev) &&
            !tx_que_is_full(&priv->tx_queue_) &&
            !priv->copro_tx_skb_list_count_) {
            // Restart the network stack's TX queue
            netif_wake_queue(dev);
        }
    }

    spin_unlock_irqrestore(&priv->tx_spinlock_, irq_flags);

    return NETDEV_TX_OK;
}
#else
#ifdef ENABLE_TX_OFFLOAD
static int hard_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
    gmac_priv_t              *priv = (gmac_priv_t*)netdev_priv(dev);
    dma_addr_t                dma_address;
    int                       desc;
    unsigned long             irq_flags;
    unsigned                  packet_length = skb->len;

DBG(1, KERN_INFO "hard_start_xmit() %s\n", dev->name);

//DBG(1, KERN_INFO "Packet length = %u\n", packet_length);
    if (likely((skb->ip_summed == CHECKSUM_HW) ||
               skb_shinfo(skb)->tso_size ||
               skb_shinfo(skb)->nr_frags)) {
#ifdef ENABLE_TX_OFFLOAD
       tx_offload(priv, skb);
#else // ENABLE_TX_OFFLOAD
       panic("hard_start_xmit() Tx offload not supported (%d,%d,%d)", skb->ip_summed, skb_shinfo(skb)->tso_size, skb_shinfo(skb)->nr_frags);
#endif // ENABLE_TX_OFFLOAD
    } else {
        // Get a consistent DMA mapping for the SDRAM to be DMAed from by
        // the GMAC- causing a flush from the CPU's cache to the memory
        dma_address = dma_map_single(0, skb->data, packet_length, DMA_TO_DEVICE);
        BUG_ON(dma_mapping_error(dma_address));

        // SMP-safe protection against concurrent operations in ISR and hard_start_xmit()
        spin_lock_irqsave(&priv->tx_spinlock_, irq_flags);

        // Construct the GMAC specific DMA descriptor
        desc = set_tx_descriptor(priv, dma_address, packet_length, skb, 0);
        if (desc < 0) {
DBG(4, KERN_WARNING "hard_start_xmit() %s: No more free TX descriptors, stopping TX queue and remembering skb for later, skb = 0x%08x, dma_address = 0x%08x\n", dev->name, (u32)skb, dma_address);
            // Should keep a record of the skb that we haven't been able to queue
            // for transmission and queue it as soon as a descriptor becomes free
            priv->tx_pending_skb       = skb;
            priv->tx_pending_sram_info = 0;
            priv->tx_pending_dma_addr  = dma_address;
            priv->tx_pending_length    = packet_length;

            // Stop further calls to hard_start_xmit() until some descriptors
            // are freed up by already queued TX packets being completed
            netif_stop_queue(dev);
        } else {
DBG(1, KERN_INFO "$Mhard_start_xmit() %s: Set tx descriptor %d for skb 0x%08x, buf = 0x%08x, skb->data = 0x%08x, skb->len = %u\n", dev->name, desc, (u32)skb, dma_address, (u32)skb->data, skb->len);
            // Record start of transmission, so timeouts will work once they're implemented
            dev->trans_start = jiffies;

            // Poke the transmitter to look for available TX descriptors, as we have
            // just added one, in case it had previously found there were no more
            // pending transmission
            dma_reg_write(priv, DMA_TX_POLL_REG, 0);
        }

        spin_unlock_irqrestore(&priv->tx_spinlock_, irq_flags);
   }

    return 0;
}
#else // ENABLE_TX_OFFLOAD
static int hard_start_xmit(
    struct sk_buff    *skb,
    struct net_device *dev)
{
    gmac_priv_t   *priv = (gmac_priv_t*)netdev_priv(dev);
    dma_addr_t     dma_address;
    unsigned long  irq_flags;

    fix_frag_ip_csum(skb);

    // Get a consistent DMA mapping for the SDRAM to be DMAed from by the GMAC,
    // causing a flush from the CPU's cache to the memory.
    // Do the DMA mapping before acquiring the tx lock, even though it complicates
    // the later code, as this can be a long operation involving cache flushing
    dma_address = dma_map_single(0, skb->data, skb_headlen(skb), DMA_TO_DEVICE);
    BUG_ON(dma_mapping_error(dma_address));

    // Protection against concurrent operations in ISR and hard_start_xmit()
    if (!spin_trylock_irqsave(&priv->tx_spinlock_, irq_flags)) {
        dma_unmap_single(0, dma_address, skb_headlen(skb), DMA_TO_DEVICE);
//printk(KERN_WARNING "hard_start_xmit() Locked\n");
        return NETDEV_TX_LOCKED;
    }

    // NETIF_F_LLTX apparently introduces a potential for hard_start_xmit() to
    // be called when the queue has been stopped (although I think only in SMP)
    // so do a check here to make sure we should proceed
    if (netif_queue_stopped(dev)) {
        dma_unmap_single(0, dma_address, skb_headlen(skb), DMA_TO_DEVICE);
        spin_unlock_irqrestore(&priv->tx_spinlock_, irq_flags);
//printk(KERN_WARNING "hard_start_xmit() Busy\n");
        return NETDEV_TX_BUSY;
    }

    // Construct the GMAC specific DMA descriptor
    if (set_tx_descriptor(priv, dma_address, skb_headlen(skb), skb, 0) < 0) {
        // Shouldn't see a full ring without the queue having already been
        // stopped, and the queue should already have been stopped if we have
        // already queued a single pending packet
        if (priv->tx_pending_skb) {
            printk(KERN_WARNING "hard_start_xmit() Ring full and pending packet already queued\n");
            dma_unmap_single(0, dma_address, skb_headlen(skb), DMA_TO_DEVICE);
            spin_unlock_irqrestore(&priv->tx_spinlock_, irq_flags);
            return NETDEV_TX_BUSY;
        }

        // Should keep a record of the skb that we haven't been able to queue
        // for transmission and queue it as soon as a descriptor becomes free
        priv->tx_pending_skb      = skb;
        priv->tx_pending_dma_addr = dma_address;
        priv->tx_pending_length   = skb_headlen(skb);

        // Stop further calls to hard_start_xmit() until some descriptors are
        // freed up by already queued TX packets being completed
        netif_stop_queue(dev);
//printk(KERN_WARNING "Stopped queue\n");
    } else {
        // Record start of transmission, so timeouts will work once they're
        // implemented
        dev->trans_start = jiffies;

        // Poke the transmitter to look for available TX descriptors, as we have
        // just added one, in case it had previously found there were no more
        // pending transmission
        dma_reg_write(priv, DMA_TX_POLL_REG, 0);
    }

    spin_unlock_irqrestore(&priv->tx_spinlock_, irq_flags);

    return NETDEV_TX_OK;
}
#endif // ENABLE_TX_OFFLOAD
#endif // CONFIG_LEON_COPRO && CONFIG_LEON_OFFLOAD_TX

static struct net_device_stats *get_stats(struct net_device *dev)
{
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);

DBG(200, KERN_INFO "get_stats() %s\n", dev->name);

    return &priv->stats;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/**
 * Polling 'interrupt' - used by things like netconsole to send skbs without
 * having to re-enable interrupts. It's not called while the interrupt routine
 * is executing.
 */
static void netpoll(struct net_device *netdev)
{
printk("************** netpoll *************\n");
    disable_irq(netdev->irq);
    int_handler(netdev->irq, netdev, NULL);
    enable_irq(netdev->irq);
}
#endif // CONFIG_NET_POLL_CONTROLLER

static int probe(
    struct net_device *netdev,
    u32                vaddr,
    u32                irq,
    int                copro_a_irq,
    int                copro_b_irq)
{
    int err = 0;
    u32 version;
    int i;
    unsigned synopsis_version;
    unsigned vendor_version;
    gmac_priv_t* priv = netdev_priv(netdev);

    // Ensure the MAC block is properly reset
    writel(1UL << SYS_CTRL_RSTEN_MAC_BIT, SYS_CTRL_RSTEN_SET_CTRL);
    writel(1UL << SYS_CTRL_RSTEN_MAC_BIT, SYS_CTRL_RSTEN_CLR_CTRL);

    // Enable the clock to the MAC block
    writel(1UL << SYS_CTRL_CKEN_MAC_BIT, SYS_CTRL_CKEN_SET_CTRL);

    // Ensure reset and clock operations are complete
    wmb();

    // Ensure all of the device private data are zero, so we can clean up in
    // the event of a later failure to initialise all fields
    priv = (gmac_priv_t*)netdev_priv(netdev);
    memset(priv, 0, sizeof(gmac_priv_t));

    // No debug messages allowed
    priv->msg_level = 0UL;

    // Initialise the ISR/hard_start_xmit() lock
    spin_lock_init(&priv->tx_spinlock_);

    // Initialise the PHY access lock
    spin_lock_init(&priv->phy_lock);

#if defined(ENABLE_TX_OFFLOAD) || defined(ENABLE_RX_OFFLOAD)
    // Initialise SRAM buffer management resources
    priv->sram_buffer_free_list_ = 0;
    priv->sram_buffer_free_count_ = 0;
    sema_init(&priv->sram_buffer_sem_, 0);
    spin_lock_init(&priv->sram_buffer_spinlock_);
#endif // defined(ENABLE_TX_OFFLOAD) || defined(ENABLE_RX_OFFLOAD)

#ifdef ENABLE_TX_OFFLOAD
    // Initialise the Tx offload work queue stop synchronisation support
    priv->tx_work_queue_stop_flag_ = 0;
    sema_init(&priv->tx_work_queue_stop_semaphore_, 0);

    // Initialise TX offload management
    INIT_LIST_HEAD(&priv->tx_offload_list_);
    spin_lock_init(&priv->tx_offload_lock_);
    priv->tx_work_queue_ = create_singlethread_workqueue("GMAC TX");
    if (!priv->tx_work_queue_) {
        DBG(1, KERN_ERR "probe() %s: Failed to create TX offload workqueue\n", netdev->name);
        err = -ENOMEM;
        goto probe_err_out;
    }
#ifdef ENABLE_TX_TSO_OFFLOAD
    INIT_WORK(&priv->tx_offload_work_, tx_tso_offload_work, priv);
    sema_init(&priv->tso_netif_resumed_semaphore_, 0);

    // Use top quad of SRAM as a fixed destination buffer for TSO csum pass
    priv->tso_csum_fixed_dst_ = SRAM_END - sizeof(u32);
#else // ENABLE_TX_TSO_OFFLOAD
    INIT_WORK(&priv->tx_offload_work_, tx_offload_work, priv);
#endif // ENABLE_TX_TSO_OFFLOAD
    sema_init(&priv->tx_offload_semaphore_, 0);
    priv->tx_job_number_ = 0;
#endif // ENABLE_TX_OFFLOAD

#ifdef ENABLE_RX_OFFLOAD
    // Initialise the Rx offload work queue stop synchronisation support
    priv->rx_work_queue_stop_flag_ = 0;
    sema_init(&priv->rx_work_queue_stop_semaphore_, 0);

    // Initialise RX offload management
    INIT_LIST_HEAD(&priv->rx_offload_list_);
    spin_lock_init(&priv->rx_offload_lock_);
    priv->rx_work_queue_ = create_singlethread_workqueue("GMAC RX");
    if (!priv->rx_work_queue_) {
        DBG(1, KERN_ERR "probe() %s: Failed to create RX offload workqueue\n", netdev->name);
        err = -ENOMEM;
        goto probe_err_out;
    }
    INIT_WORK(&priv->rx_offload_work_, rx_offload_work, priv);
    sema_init(&priv->rx_offload_semaphore_, 0);
    priv->rx_job_number_ = 0;
#endif // ENABLE_RX_OFFLOAD

    // Initialise lock used to synchronise RX processing
    spin_lock_init(&priv->rx_spinlock_);

    // Set hardware device base addresses
    priv->macBase = vaddr + MAC_BASE_OFFSET;
    priv->dmaBase = vaddr + DMA_BASE_OFFSET;

    // Initialise IRQ ownership to not owned
    priv->have_irq = 0;

    // Lock protecting access to CoPro command queue functions or direct access
    // to the GMAC interrupt enable register if CoPro is not in use
    spin_lock_init(&priv->cmd_que_lock_);

#ifdef CONFIG_LEON_COPRO
    sema_init(&copro_stop_semaphore, 0);
    sema_init(&copro_start_semaphore, 0);
    sema_init(&copro_int_clr_semaphore, 0);
    priv->copro_a_irq_alloced_ = 0;
    priv->copro_b_irq_alloced_ = 0;
    sema_init(&priv->copro_stop_complete_semaphore_, 0);
    INIT_LIST_HEAD(&priv->copro_tx_skb_list_);
    priv->copro_tx_skb_list_count_ = 0;
#endif // CONFIG_LEON_COPRO

    init_timer(&priv->watchdog_timer);
    priv->watchdog_timer.function = &watchdog_timer_action;
    priv->watchdog_timer.data = (unsigned long)priv;

    // Set pointer to device in private data
    priv->netdev = netdev;

    /** Do something here to detect the present or otherwise of the MAC
     *  Read the version register as a first test */
    version = mac_reg_read(priv, MAC_VERSION_REG);
    synopsis_version = version & 0xff;
    vendor_version   = (version >> 8) & 0xff;

    /** Assume device is at the adr and irq specified until have probing working */
    netdev->base_addr  = vaddr;
    netdev->irq        = irq;
#ifdef CONFIG_LEON_COPRO
    priv->copro_a_irq_ = copro_a_irq;
    priv->copro_b_irq_ = copro_b_irq;
#endif // CONFIG_LEON_COPRO

#ifdef DUMP_REGS_ON_PROBE
    dump_mac_regs(priv->macBase, priv->dmaBase);
#endif // DUMP_REGS_ON_PROBE

#ifdef CONFIG_LEON_COPRO
    // Allocate the CoPro A IRQ
    err = request_irq(priv->copro_a_irq_, &copro_sema_intr, 0, "SEMA", netdev);
    if (err) {
        DBG(1, KERN_ERR "probe() %s: Failed to allocate CoPro irq A (%d)\n", netdev->name, priv->copro_a_irq_);
        goto probe_err_out;
    }
    // Release the CoPro A IRQ again, as open()/stop() should manage IRQ ownership
    free_irq(priv->copro_a_irq_, netdev);

    // Allocate the CoPro B IRQ
    err = request_irq(priv->copro_b_irq_, &copro_semb_intr, 0, "SEMB", netdev);
    if (err) {
        DBG(1, KERN_ERR "probe() %s: Failed to allocate CoPro irq B (%d)\n", netdev->name, priv->copro_b_irq_);
        goto probe_err_out;
    }
    // Release the CoPro B IRQ again, as open()/stop() should manage IRQ ownership
    free_irq(priv->copro_b_irq_, netdev);
#else // CONFIG_LEON_COPRO
    // Allocate the IRQ
    err = request_irq(netdev->irq, &int_handler, 0, netdev->name, netdev);
    if (err) {
        DBG(1, KERN_ERR "probe() %s: Failed to allocate irq %d\n", netdev->name, netdev->irq);
        goto probe_err_out;
    }

    // Release the IRQ again, as open()/stop() should manage IRQ ownership
    free_irq(netdev->irq, netdev);
#endif // CONFIG_LEON_COPRO

    // Initialise the ethernet device with std. contents
    ether_setup(netdev);

    // Tell the kernel of our MAC address
    if ((mac_hi==0)&&(mac_lo==0)) {
        memcpy(netdev->dev_addr, DEFAULT_MAC_ADDRESS, netdev->addr_len);
    } else {
        int i;
        for (i=0; i < netdev->addr_len; i++) {
            if (i < sizeof(u32)) {
                netdev->dev_addr[i] = ((mac_hi >> (((sizeof(u32)-1)-i)*8)) & 0xff);
            } else {
                netdev->dev_addr[i] = ((mac_lo >> (((sizeof(u32)+1)-i)*8)) & 0xff);
            }
        }
    }

    // Setup operations pointers
    netdev->open               = &open;
    netdev->hard_start_xmit    = &hard_start_xmit;
    netdev->stop               = &stop;
    netdev->get_stats          = &get_stats;
#ifdef USE_RX_NAPI
    netdev->poll               = &poll;
#endif // USE_RX_NAPI
    netdev->weight             = NAPI_POLL_WEIGHT;
    netdev->change_mtu         = &change_mtu;
#ifdef CONFIG_NET_POLL_CONTROLLER
    netdev->poll_controller    = &netpoll;
#endif // CONFIG_NET_POLL_CONTROLLER
    netdev->set_mac_address    = &set_mac_address;
    netdev->set_multicast_list = &set_multicast_list;
    set_ethtool_ops(netdev);

    if (debug) {
      netdev->flags |= IFF_DEBUG;
    }

#if defined(ENABLE_TX_OFFLOAD) || (defined(CONFIG_LEON_COPRO) && defined(CONFIG_LEON_OFFLOAD_TSO))
    // Do TX H/W checksum and SG list processing
    netdev->features |= NETIF_F_HW_CSUM;
    netdev->features |= NETIF_F_SG;
    netdev->features |= NETIF_F_FRAGLIST;

#if defined(ENABLE_TX_TSO_OFFLOAD) || (defined(CONFIG_LEON_COPRO) && defined(CONFIG_LEON_OFFLOAD_TSO))
    // Do hardware TCP/IP Segmentation Offload
    netdev->features |= NETIF_F_TSO;
#endif // ENABLE_TX_TSO_OFFLOAD
#endif // ENABLE_TX_OFFLOAD

    // We take care of our own TX locking
    netdev->features |= NETIF_F_LLTX;

    // Initialise PHY support
    priv->mii.phy_id_mask   = 0x1f;
    priv->mii.reg_num_mask  = 0x1f;
    priv->mii.force_media   = 0;
    priv->mii.full_duplex   = 1;
    priv->mii.supports_gmii = 0;
    priv->mii.using_100     = 0;
    priv->mii.using_1000    = 0;
    priv->mii.dev           = netdev;
    priv->mii.mdio_read     = phy_read;
    priv->mii.mdio_write    = phy_write;

    // Remember whether auto-negotiation is allowed
#ifdef ALLOW_AUTONEG
    priv->ethtool_cmd.autoneg = 1;
#else // ALLOW_AUTONEG
    priv->ethtool_cmd.autoneg = 0;
#endif // ALLOW_AUTONEG

    // Set up PHY mode for when auto-negotiation is not allowed
#ifdef ALLOW_1000_SUPPORT
    priv->ethtool_cmd.speed = SPEED_1000;
#else // ALLOW_1000_SUPPORT
    priv->ethtool_cmd.speed = SPEED_100;
#endif // ALLOW_1000_SUPPORT
#ifdef FORCE_HALF_DUPLEX
    priv->ethtool_cmd.duplex = DUPLEX_FULL;
#else // FORCE_HALF_DUPLEX
    priv->ethtool_cmd.duplex = DUPLEX_HALF;
#endif // FORCE_HALF_DUPLEX
    priv->ethtool_cmd.port = PORT_MII;
    priv->ethtool_cmd.transceiver = XCVR_INTERNAL;

#ifdef USE_RX_CSUM
    priv->rx_csum           = 1;
#endif // USE_RX_CSUM

    priv->gmii_csr_clk_range = 5;   // Slowest for now

    // Use simple mux for 25/125 Mhz clock switching
    *(u32*)SYS_CTRL_GMAC_CTRL |= (1UL << SYS_CTRL_GMAC_SIMPLE_MAX);

    // Enable GMII_GTXCLK to follow GMII_REFCLK - required for gigabit PHY
    *(u32*)SYS_CTRL_GMAC_CTRL |= (1UL << SYS_CTRL_GMAC_CKEN_GTX);

    // Attempt to locate the PHY
    phy_detect(netdev);
    priv->ethtool_cmd.phy_address = priv->mii.phy_id;

    // Did we find a PHY?
    switch (priv->phy_type) {
        case PHY_TYPE_NONE:
            printk(KERN_WARNING "%s: No PHY found\n", netdev->name);
            goto probe_err_out;
        case PHY_TYPE_VITESSE_VSC8201XVZ:
            // Is 1000
#ifdef ALLOW_1000_SUPPORT
            priv->mii.supports_gmii = 1;
#endif // ALLOW_1000_SUPPORT
            break;
        case PHY_TYPE_MICREL_KS8721BL:
            // 10/100 so fall through
        default:
            // Unknown, assume 10/100
            priv->mii.supports_gmii = 0;
            break;
    }

    // Initialise the set of features supported by the hardware
    priv->ethtool_cmd.supported = (SUPPORTED_10baseT_Half |
                                   SUPPORTED_10baseT_Full |
                                   SUPPORTED_100baseT_Half |
                                   SUPPORTED_100baseT_Full |
                                   SUPPORTED_Autoneg |
                                   SUPPORTED_MII);

    if (priv->mii.supports_gmii) {
        priv->ethtool_cmd.supported |= (SUPPORTED_1000baseT_Half |
                                        SUPPORTED_1000baseT_Full);
    }

    // Initialise the set of features we will advertise as being available for
    // negotiation
    priv->ethtool_cmd.advertising = (ADVERTISED_10baseT_Half |
                                     ADVERTISED_10baseT_Full |
                                     ADVERTISED_100baseT_Half |
                                     ADVERTISED_100baseT_Full |
                                     ADVERTISED_Autoneg |
                                     ADVERTISED_MII);

    if (priv->mii.supports_gmii) {
        priv->ethtool_cmd.advertising |= ADVERTISED_1000baseT_Half;
#ifdef ALLOW_FULL_DUPLEX_AT_1000
        priv->ethtool_cmd.advertising |= ADVERTISED_1000baseT_Full;
#endif // ALLOW_FULL_DUPLEX_AT_1000
    }

    // Register the device with the network intrastructure
    err = register_netdev(netdev);
    if (err) {
        DBG(1, KERN_ERR "probe() %s: Failed to register device\n", netdev->name);
        goto probe_err_out;
    }

    // Record details about the hardware we found
    printk(KERN_NOTICE "%s: %s\n", netdev->name, version_string);
    printk(KERN_NOTICE "%s: Synopsis ver = %u, vendor ver = %u at 0x%lx, IRQ %d\n", netdev->name, synopsis_version, vendor_version, netdev->base_addr, netdev->irq);
#ifndef ARMULATING
    printk(KERN_NOTICE "%s: Found PHY at address %u, type 0x%08x -> %s\n", priv->netdev->name, priv->phy_addr, priv->phy_type, priv->mii.supports_gmii ? "10/100/1000" : "10/100");
#endif // ARMULATING
    printk(KERN_NOTICE "%s: Ethernet addr: ", priv->netdev->name);
    for (i = 0; i < 5; i++) {
        printk("%02x:", netdev->dev_addr[i]);
    }
    printk("%02x\n", netdev->dev_addr[5]);

#ifdef CONFIG_LEON_COPRO
    // Define sizes of queues for communicating with the CoPro
    priv->copro_cmd_que_num_entries_ = COPRO_CMD_QUEUE_NUM_ENTRIES;
    priv->copro_tx_que_num_entries_ = COPRO_TX_QUEUE_NUM_ENTRIES;
#endif // CONFIG_LEON_COPRO

    return 0;

probe_err_out:
    if (netdev_priv(netdev)) {
#ifdef ENABLE_TX_OFFLOAD
        // Destroy the TX offload workqueue
        if (priv->tx_work_queue_) {
            destroy_workqueue(priv->tx_work_queue_);
        }
#endif // ENABLE_TX_OFFLOAD

#ifdef ENABLE_RX_OFFLOAD
        // Destroy the RX offload workqueue
        if (priv->rx_work_queue_) {
            destroy_workqueue(priv->rx_work_queue_);
        }
#endif // ENABLE_RX_OFFLOAD

#if defined(ENABLE_TX_OFFLOAD) || defined(ENABLE_RX_OFFLOAD)
        // Free storage for SRAM net buffer info entries
        if (priv->sram_buffer_infos_) {
            kfree(priv->sram_buffer_infos_);
        }
#endif // defined(ENABLE_TX_OFFLOAD) || defined(ENABLE_RX_OFFLOAD)
    }

#ifdef CONFIG_LEON_COPRO
    shutdown_copro();

    if (priv->shared_copro_params_) {
        // Free the DMA coherent parameter space
        dma_free_coherent(0, sizeof(copro_params_t), priv->shared_copro_params_, priv->shared_copro_params_pa_);
        priv->shared_copro_params_ = 0;
    }
#endif // CONFIG_LEON_COPRO

    // Disable the clock to the MAC block
    writel(1UL << SYS_CTRL_CKEN_MAC_BIT, SYS_CTRL_CKEN_CLR_CTRL);

    return err;
}

/**
 * External entry point to the driver, called from Space.c to detect a card
 */
struct net_device* __init synopsys_gmac_probe(int unit)
{
    int err = 0;
#ifdef ALLOW_PROBING
    struct net_device *netdev = alloc_etherdev(sizeof(gmac_priv_t));

    printk(KERN_NOTICE "Probing for Synopsis GMAC, unit %d\n", unit);

    // Will allocate private data later, as may want descriptors etc in special memory
    if (!netdev) {
        printk(KERN_WARNING "synopsys_gmac_probe() failed to alloc device\n");
        err = -ENODEV;
    } else {
        if (unit >= 0) {
            sprintf(netdev->name, "eth%d", unit);

            netdev_boot_setup_check(netdev);

            if (gmac_found_count >= MAX_GMAC_UNITS) {
                err = -ENODEV;
            } else {
                err = probe(netdev, MAC_BASE, MAC_INTERRUPT, SEM_A_INTERRUPT, SEM_B_INTERRUPT);
                if (err) {
                    printk(KERN_WARNING "synopsys_gmac_probe() Probing failed for %s\n", netdev->name);
                } else {
                    ++gmac_found_count;
                }
            }
        }

        if (err) {
            netdev->reg_state = NETREG_UNREGISTERED;
            free_netdev(netdev);
        }
    }
#else // ALLOW_PROBING
    printk(KERN_WARNING "synopsys_gmac_probe() Forcing no probe\n");
    err = -ENODEV;
#endif // ALLOW_PROBING

    return ERR_PTR(err);
}
