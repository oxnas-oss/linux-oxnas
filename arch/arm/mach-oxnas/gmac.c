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
#include <linux/version.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/irqs.h>

//#define GMAC_DEBUG 5
#undef GMAC_DEBUG

#include "gmac.h"
#include "gmac_ethtool.h"
#include "gmac_phy.h"
#include "gmac_desc.h"
#include "gmac_reg.h"

#define ALLOW_PROBING
#define ALLOW_AUTONEG
//#define INCLUDE_DUMP_REGS
//#define DUMP_REGS_ON_PROBE
//#define SUPPORTS_GIGABIT
//#define INITIALISE_HASH_TABLE
//#define USE_HASH_TABLE

#define NUM_RX_DMA_DESCRIPTORS 64
#define NUM_TX_DMA_DESCRIPTORS 64
#if (((NUM_RX_DMA_DESCRIPTORS) + (NUM_TX_DMA_DESCRIPTORS)) > (NUM_GMAC_DMA_DESCRIPTORS))
#error "GMAC TX+RX descriptors exceed allocation"
#endif

static const u8 DEFAULT_MAC_ADDRESS[] = { 0x00, 0xCF, 0x52, 0x49, 0xC3, 0x03 };

static const u32 MAC_BASE_OFFSET = 0x0000;
static const u32 DMA_BASE_OFFSET = 0x1000;

// The number of bytes wasted at the start of a received packet buffer in order
// to ensure the IP header will be aligned to a 32-bit boundary
static const int ETHER_FRAME_ALIGN_WASTAGE = 2;
static const int EXTRA_RX_SKB_SPACE = 32;   // Otherwise GMAC spans over >1 skb

// Max length when received data is copied to new skb
static const int ETHERNET_PACKET_COPY = 0;//250;

static const u32 AUTO_NEGOTIATION_WAIT_TIMEOUT_MS = 5000;

static const u32 NAPI_POLL_WEIGHT = 16;
static const u32 NAPI_OOM_POLL_INTERVAL_MS = 10;

static const int WATCHDOG_TIMER_INTERVAL = 500*HZ/1000;

static const int AUTO_NEG_INTERVAL = 500*HZ/1000;
static const int START_RESET_INTERVAL = 50*HZ/1000;
static const int RESET_INTERVAL = 10*HZ/1000;

static const int GMAC_RESET_TIMEOUT_MS = 1000;

// DMA status register bits for all interesting interrupt sources
static const u32 INTR_STATUS_MASK = ((1UL << DMA_STATUS_FBE_BIT) |
                                     (1UL << DMA_STATUS_RI_BIT) |
                                     (1UL << DMA_STATUS_RU_BIT) |
                                     (1UL << DMA_STATUS_OVF_BIT) |
                                     (1UL << DMA_STATUS_RWT_BIT) |
                                     (1UL << DMA_STATUS_RPS_BIT) |
                                     (1UL << DMA_STATUS_TI_BIT) |
                                     (1UL << DMA_STATUS_TU_BIT) |
                                     (1UL << DMA_STATUS_UNF_BIT) |
                                     (1UL << DMA_STATUS_TJT_BIT) |
                                     (1UL << DMA_STATUS_TPS_BIT));

// DMA status register bits for all but receive interesting interrupts
static const u32 INTR_STATUS_WITHOUT_RX_POLLING_MASK = ((1UL << DMA_STATUS_FBE_BIT) |
                                                        (1UL << DMA_STATUS_RU_BIT) |
                                                        (1UL << DMA_STATUS_OVF_BIT) |
                                                        (1UL << DMA_STATUS_RWT_BIT) |
                                                        (1UL << DMA_STATUS_RPS_BIT) |
                                                        (1UL << DMA_STATUS_TI_BIT) |
                                                        (1UL << DMA_STATUS_TU_BIT) |
                                                        (1UL << DMA_STATUS_UNF_BIT) |
                                                        (1UL << DMA_STATUS_TJT_BIT) |
                                                        (1UL << DMA_STATUS_TPS_BIT));

// DMA status register bits for interesting receive interrupt sources
static const u32 INTR_STATUS_ONLY_RX_POLLING_MASK = (1UL << DMA_STATUS_RI_BIT);

// DMA interrupt enable register bit for all interesting interrupts
static const u32 INTR_EN_MASK = ((1UL << DMA_INT_ENABLE_NI_BIT) |
                                 (1UL << DMA_INT_ENABLE_AI_BIT) |
                                 (1UL << DMA_INT_ENABLE_FBE_BIT) |
                                 (1UL << DMA_INT_ENABLE_RI_BIT) |
                                 (1UL << DMA_INT_ENABLE_RU_BIT) |
                                 (1UL << DMA_INT_ENABLE_OV_BIT) |
                                 (1UL << DMA_INT_ENABLE_RW_BIT) |
                                 (1UL << DMA_INT_ENABLE_RS_BIT) |
                                 (1UL << DMA_INT_ENABLE_TI_BIT) |
                                 (1UL << DMA_INT_ENABLE_TU_BIT) |
                                 (1UL << DMA_INT_ENABLE_UN_BIT) |
                                 (1UL << DMA_INT_ENABLE_TJ_BIT) |
                                 (1UL << DMA_INT_ENABLE_TS_BIT));

static const u32 INTR_EN_ONLY_RX_POLLING_MASK = (1UL << DMA_INT_ENABLE_RI_BIT);

static const char *version_string = "Synopsys GMAC v1.0 14/04/05 Brian H. Clarke";

static int debug = 0;

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

/**
 * @param skb A struct sk_buff* which if non-zero refers to an existing socket
 *            buffer that can be reused for filling the RX descriptor ring
 * @return An int which is non-zero if the rx descriptor ring is now full
 */
static int refill_rx_ring(struct net_device *dev, struct sk_buff* skb, u32* int_enable)
{
    gmac_priv_t *priv = (gmac_priv_t*)dev->priv;
    int ring_full;

DBG(1, KERN_INFO "$Crefill_rx_ring() %s: Entered\n", dev->name);
    // While there are empty RX descriptor ring slots
    while (available_for_write(&priv->rx_gmac_desc_list_info)) {
        u32 dma_length;
        dma_addr_t dma_address;
        int desc;

        if (!skb) {
            // Allocate a new skb for the descriptor ring which is large
            // enough for any packet received from the link
            skb = dev_alloc_skb(dev->mtu + ETHER_FRAME_ALIGN_WASTAGE + EXTRA_RX_SKB_SPACE);
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
                skb_reserve(skb, ETHER_FRAME_ALIGN_WASTAGE);
            }
        }

        // Get a consistent DMA mapping for the memory to be DMAed to - causing
        // an invalidation of any entries in the CPU's cache covering the memory
        // region
        dma_length = skb_tailroom(skb);
        dma_address = dma_map_single(0, skb->tail, dma_length, DMA_FROM_DEVICE);
        BUG_ON(dma_mapping_error(dma_address));

        // Associate the skb to the descriptor
        desc = set_rx_descriptor(priv, dma_address, dma_length, skb);
DBG(1, KERN_INFO "$Crefill_rx_ring() %s: Set rx descriptor %d for skb %p, len = %u, skb->tail = 0x%08x\n", dev->name, desc, skb, dma_length, (u32)skb->tail);

        // Socket buffer will either be comsumed by attaching to RX descriptor,
        // or if a descriptor could not be found, the socket buffer will be
        // deallocated
        skb = 0;

        // Was the socket buffer sucessfully associated with a descriptor?
        if (desc < 0) {
DBG(1, KERN_INFO "refill_rx_ring() %s: Cannot set rx descriptor for skb %p, rx ring full\n", dev->name, skb);
            // No, so release the DMA mapping for the socket buffer
            dma_unmap_single(0, dma_address, dma_length, DMA_TO_DEVICE);

            // Free the socket buffer
            dev_kfree_skb(skb);

            // No more RX descriptor ring entries to refill
            break;
        }
    }

    // Ensure that if we were given an skb and didn't use it, we free it now
    if (skb) {
        dev_kfree_skb(skb);
    }

    // Return ring fill status
    ring_full = !available_for_write(&priv->rx_gmac_desc_list_info);
    if (ring_full) {
        // Atomically test whether we should re-enable RX unavailable interrupts
        if (xchg(&priv->rx_overflow_ints_disabled, 0)) {
            // Enable RX overflow reporting, as there are now RX descriptors queued
DBG(5, "$YEnabling RX unavailable interrupts\n");
            *int_enable = dma_reg_set_mask(priv, DMA_INT_ENABLE_REG, (1UL << DMA_INT_ENABLE_RU_BIT));
        }
    }

    return ring_full;
}

static void start_watchdog_timer(gmac_priv_t* priv)
{
    priv->watchdog_timer.expires = jiffies + WATCHDOG_TIMER_INTERVAL;
    priv->watchdog_timer_shutdown = 0;
    add_timer(&priv->watchdog_timer);
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

static int wait_auto_negotiation_complete(gmac_priv_t* priv)
{
    int status = -ENODEV;

    if (!priv->allow_autonegotiation) {
        status = 0;
    } else {
        // Only wait if auto-negotiation is enabled
        u32 phy_data = phy_read(priv->netdev, priv->phy_addr, MII_BMCR);
        if (phy_data & BMCR_ANENABLE) {
            unsigned long end = jiffies + MS_TO_JIFFIES(AUTO_NEGOTIATION_WAIT_TIMEOUT_MS);
            while (time_before(jiffies, end)) {
                if (!is_auto_negotiation_in_progress(priv)) {
                    DBG(1, KERN_NOTICE "wait_auto_negotiation_complete() %s: Auto-negotiation complete\n", priv->netdev->name);
                    status = 0;
                    break;
                }
                msleep(1);
            }
        }
    }

    return status;
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

    // Interpret the PHY/link state.
    duplex_changed = mii_check_media(&priv->mii, 1, 0);

    // If the link is down, try to bring system back up
    if (!netif_carrier_ok(priv->netdev)) {
        // May be a good idea to restart everything here, in an attempt to clear
        // out any fault conditions
        if (is_auto_negotiation_in_progress(priv)) {
            state = WDS_NEGOTIATING;
            new_timeout = jiffies + AUTO_NEG_INTERVAL;
        } else {
            switch(state) {
                case WDS_IDLE:
                    // Reset the PHY to get it into a known state
                    start_phy_reset(priv);
                    new_timeout = jiffies + START_RESET_INTERVAL;
                    state = WDS_RESETTING;
                    break;
                case WDS_RESETTING:
                    if (!is_phy_reset_complete(priv)) {
                        new_timeout = jiffies + RESET_INTERVAL;
                    } else {
                        // Restart link auto negotiation
                        if (!mii_nway_restart(&priv->mii)) {
                            state = WDS_NEGOTIATING;
                            new_timeout = jiffies + AUTO_NEG_INTERVAL;
                        } else {
                            DBG(1, KERN_INFO "watchdog_timer_action() %s: Failed to start auto-negotiation\n", priv->netdev->name);
                            state = WDS_IDLE;
                        }
                    }
                    break;
                default:
                    DBG(1, KERN_ERR "watchdog_timer_action() %s: Unexpected state\n", priv->netdev->name);
                    state = WDS_IDLE;
                    break;
            }
        }
    } else {
        state = WDS_IDLE;
        if (duplex_changed) {
            priv->mii.full_duplex ? mac_reg_set_mask(priv,   MAC_CONFIG_REG, (1UL << MAC_CONFIG_DM_BIT)) :
                                    mac_reg_clear_mask(priv, MAC_CONFIG_REG, (1UL << MAC_CONFIG_DM_BIT));
        }
    }
#endif // ARMULATING

    // Re-trigger the timer, unless some other thread has requested it be stopped
    if (!priv->watchdog_timer_shutdown) {
        // Restart the timer
        mod_timer(&priv->watchdog_timer, new_timeout);
    }
}

/*
 * NAPI receive polling method
 */
static void receive(gmac_priv_t* priv, u32* int_enable)
{
    struct net_device* dev = priv->netdev;
    int desc;
    struct sk_buff* skb;
    dma_addr_t dma_address;
    u32 dma_length;
    u32 desc_status;

DBG(1, KERN_INFO "$receive() %s:, budget = %u\n", priv->dev->name, *budget);

    // Handle all pending receive descriptors
    while ((desc = get_rx_descriptor(priv, &desc_status, &dma_address, &dma_length, &skb)) > 0) {
        if (!skb) {
            DBG(1, KERN_ERR "receive() %s: Found RX descriptor without attached skb\n", dev->name);
        } else {
DBG(1, KERN_INFO "receive() %s: Get rx descriptor %d for skb %p, desc_status = 0x%08x\n", dev->name, desc, skb, desc_status);
            // Release the DMA mapping for the received data
            dma_unmap_single(0, dma_address, dma_length, DMA_FROM_DEVICE);

            // Process good packets only
            if (is_rx_valid(desc_status)) {
                struct sk_buff *newskb = 0;
                int len;

                // Account for the appended payload checksum if hardware
                // receive checksum calculation is enabled
                len = get_rx_length(desc_status);

DBG(5, KERN_INFO "$Breceive() %s: Length = %d\n", dev->name, len);
                if (len <= ETHERNET_PACKET_COPY) {
                    // Allocate a new skb for the small packet
                    newskb = dev_alloc_skb(len + ETHER_FRAME_ALIGN_WASTAGE + EXTRA_RX_SKB_SPACE);
                    if (!newskb) {
                        // Failed to allocate small packet skb, so just
                        // continue to use the original large skb
                        DBG(1, KERN_WARNING "receive() %s: No memory for newskb, using large one\n", dev->name);
                    } else {
                        unsigned char *data;

                        // Despite what the comments in the original
                        // code from Synopsys claimed, the GMAC DMA can
                        // cope with non-quad aligned buffers - it will
                        // always perform quad transfers but zero/ignore
                        // the unwanted bytes.
                        skb_reserve(newskb, ETHER_FRAME_ALIGN_WASTAGE);

                        // Record the space in the new, small, skb that
                        // will be occupied by the RX packet
                        data = skb_put(newskb, len);

                        // Copy the small packet into the new skb
                        memcpy(data, skb->data, len);

                        // Set the device for the new small skb
                        newskb->dev = dev;

                        // Set packet protocol
                        newskb->protocol = eth_type_trans(newskb, dev);
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
                    skb_put(newskb, len);

                    // Set the device for the skb
                    newskb->dev = dev;

                    // Set packet protocol
                    newskb->protocol = eth_type_trans(newskb, dev);
                }

                // Send the packet up protocol stack
                netif_rx(newskb);

                // Update receive statistics
                dev->last_rx = jiffies;
                ++priv->stats.rx_packets;
                priv->stats.rx_bytes += len;
            } else {
                printk(KERN_WARNING "$Rreceive() %s: Received packet has bad desc_status = 0x%08x\n", dev->name, desc_status);
                ++priv->stats.rx_errors;

                // Update receive statistics from the descriptor status
                if (is_rx_collision_error(desc_status)) {
                    printk(KERN_INFO "$Yreceive() %s: Collision error\n", dev->name);
                    ++priv->stats.collisions;
                }
                if (is_rx_crc_error(desc_status)) {
                    printk(KERN_INFO "$Yreceive() %s: CRC error\n", dev->name);
                    ++priv->stats.rx_crc_errors;
                }
                if (is_rx_frame_error(desc_status)) {
                    printk(KERN_INFO "$Yreceive() %s: frame error\n", dev->name);
                    ++priv->stats.rx_frame_errors;
                }
                if (is_rx_length_error(desc_status)) {
                    printk(KERN_INFO "$Yreceive() %s: Length error\n", dev->name);
                    ++priv->stats.rx_length_errors;
                }
            }

            // If we have a socket buffer available and there is room to
            // queue a new RX descriptor
            if (skb) {
                if (available_for_write(&priv->rx_gmac_desc_list_info)) {
                    // Make use of the available socket buffer by
                    // attempting to fill all available slots in the RX
                    // descriptor ring
DBG(1, KERN_INFO "receive() %s: Have skb and desc not empty, so calling refill_rx_ring()\n", dev->name);
                    refill_rx_ring(dev, skb, int_enable);
                } else {
                    // Free the socket buffer, as we couldn't make use
                    // of it to refill the RX descriptor ring
                    dev_kfree_skb(skb);
                }
            }
        }

        // Attempt to fill all available slots in the RX descriptor ring
        if (available_for_write(&priv->rx_gmac_desc_list_info)) {
DBG(1, KERN_INFO "receive() %s: Not empty, so calling refill_rx_ring()\n", dev->name);
            refill_rx_ring(dev, 0, int_enable);

            if (available_for_write(&priv->rx_gmac_desc_list_info)) {
DBG(5, KERN_INFO "$Rreceive() %s: OOM, SHOULD REALLY START POLLING FOR AVAILABLE MEMORY\n", dev->name);
            }
        }
    }

DBG(1, KERN_INFO "$Greceive() %s: Leaving\n", dev->name);
}

/**
 * Finish packet transmision started by hard_start_xmit()
 */
static void finish_xmit(struct net_device *dev)
{
    gmac_priv_t* priv = (gmac_priv_t*)dev->priv;
    unsigned descriptors_freed = 0;
    int desc;

//DBG(1, KERN_INFO "finish_xmit() %s\n", dev->name);

    // SMP-safe protection against concurrent operations in ISR and hard_start_xmit()
    spin_lock(priv->spinlock);

    // Handle transmit descriptors for the completed packet transmission
    do {
        u32 desc_status;
        dma_addr_t dma_address;
        u32 len;
        struct sk_buff* skb;

        // Get tx descriptor content
        desc = get_tx_descriptor(priv, &desc_status, &dma_address, &len, &skb);
        if (desc >= 0) {
DBG(1, KERN_INFO "$Mfinish_xmit() %s: Got tx descriptor %d for skb %p, desc_status = %08x, len = %u\n", dev->name, desc, skb, desc_status, len);

            if (skb) {
                // Release the DMA mapping for the socket buffer
                dma_unmap_single(0, dma_address, len, DMA_TO_DEVICE);

                // Inform the network stack that packet transmission has finished
                dev_kfree_skb_irq(skb);

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

            // Track how many descriptor we make available, so we know
            // if we need to re-start of network stack's TX queue processing
            ++descriptors_freed;
        }
    } while (desc >= 0);   // While we have transmitted descriptors

    // If any TX descriptors have been freed and there is an outstanding TX
    // packet waiting to be queued due to there not having been a TX descriptor
    // available when hard_start_xmit() was presented with an skb by the network
    // stack
    if (priv->tx_pending_skb && available_for_write(&priv->tx_gmac_desc_list_info)) {
        // Construct the GMAC specific DMA descriptor
        desc = set_tx_descriptor(priv, priv->tx_pending_dma_addr, priv->tx_pending_skb->len, priv->tx_pending_skb);
        if (desc < 0) {
            panic("$finish_xmit() %s: Failed to set desc after finding avail for write\n", dev->name);
        } else {
            // Successfully queued descriptor for pending TX packet
DBG(1, KERN_INFO "finish_xmit() %s: Set tx descriptor %d for $Woutstanding$n skb %p\n", dev->name, desc, priv->tx_pending_skb);

            // No TX packets now outstanding
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

    if (netif_queue_stopped(dev) && descriptors_freed) {
        // The TX queue had been stopped by hard_start_xmit() due to lack of TX
        // descriptors, so restart it now that we've freed at least one
        netif_wake_queue(dev);
    }

    // SMP-safe protection against concurrent operations in ISR and hard_start_xmit()
    spin_unlock(priv->spinlock);
}

static irqreturn_t int_handler(int int_num, void* dev_id, struct pt_regs* regs)
{
    struct net_device *dev = (struct net_device *)dev_id;
    gmac_priv_t* priv = (gmac_priv_t*)dev->priv;

    // Get the interrupt enable mask
    u32 int_enable = dma_reg_read(priv, DMA_INT_ENABLE_REG);

    // Get interrupt status
    u32 raw_status = dma_reg_read(priv, DMA_STATUS_REG);

    // Get status of enabled interrupt sources
    u32 status = raw_status & int_enable;
DBG(1, KERN_INFO "int_handler() %s: Entered, raw_status = 0x%08x, status = 0x%08x, int_enable = 0x%08x\n", dev->name, raw_status, status, int_enable);

    while (status) {
        // Whether the link/PHY watchdog timer should be restarted
        int restart_watchdog = 0;
        u32 int_disable_mask = 0;

        // Test for unavailable RX buffers - must do this before ack'ing, else
        // otherwise can get into trouble with the sticky summary bits
        if (status & (1UL << DMA_STATUS_RU_BIT)) {
            printk(KERN_INFO "$Rint_handler() %s: RX buffer unavailable\n", dev->name);
            // Accumulate receive statistics
            ++priv->stats.rx_over_errors;
            ++priv->stats.rx_errors;

            // Atomically test whether we should disable RX unavailable interrupts
            if (!xchg(&priv->rx_overflow_ints_disabled, 1)) {
DBG(5, "$YDisabling RX unavailable interrupts\n");
                // Disable RX overflow reporting, so we don't get swamped
                int_disable_mask |= (1UL << DMA_INT_ENABLE_RU_BIT);
            }
        }

        // Do any interrupt disabling with a single register write
        if (int_disable_mask) {
DBG(5, "$WDisabing intr with mask = 0x%08x\n", int_disable_mask);
            int_enable = dma_reg_clear_mask(priv, DMA_INT_ENABLE_REG, int_disable_mask);
        }

        // The broken GMAC interrupt mechanism with its sticky summary bits
        // means that I have to ack all asserted interrupts here; we can't not
        // ack the RX interrupt sources as we would like to (in order that the
        // poll() routine could examine the status) because if they were
        // asserted prior to being masked above, then the summary bit(s) would
        // remain asserted and cause an immediate re-interrupt. The poll()
        // routine will see the status of any RX events after those that caused
        // the change to the polling state, but currently will not ever see the
        // status for those that caused the change to the polling state
        dma_reg_write(priv, DMA_STATUS_REG, status | ((1UL << DMA_STATUS_NIS_BIT) |
                                                      (1UL << DMA_STATUS_AIS_BIT)));

        // Test for normal TX interrupt
        if (status & ((1UL << DMA_STATUS_TI_BIT) |
                      (1UL << DMA_STATUS_ETI_BIT))) {
            DBG(1, KERN_INFO "$Mint_handler() %s: TX\n", dev->name);

            // Finish packet transmision started by start_xmit
            finish_xmit(dev);
        }

        // Test for RX interrupt resulting in descriptors to be processed by CPU
        if (status & (1UL << DMA_STATUS_RI_BIT)) {
            // Process pending receive descriptors
            receive(priv, &int_enable);
        }

        // Test for abnormal transmitter interrupt where there may be completed
        // packets waiting to be processed
        if (status & ((1UL << DMA_STATUS_TJT_BIT) |
                      (1UL << DMA_STATUS_UNF_BIT))) {
            printk(KERN_INFO "$Rint_handler() %s: TX abnormal\n", dev->name);

            // Complete processing of any TX packets closed by the DMA
            finish_xmit(dev);

            // Issue a TX poll demand in an attempt to restart TX descriptor
            // processing; this will not work for jabber, as the TX state
            // machine will have entered the stopped state, in which case the
            // later test for TX stop interrupt assertion will cause an attempt
            // to restart
            dma_reg_write(priv, DMA_TX_POLL_REG, 0);
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
                printk(KERN_WARNING "$Mint_handler() %s: Early RX \n", dev->name);
            }

            if (status & (1UL << DMA_STATUS_OVF_BIT)) {
                printk(KERN_INFO "$Rint_handler() %s: Overflow\n", dev->name);
                // Accumulate receive statistics
                ++priv->stats.rx_fifo_errors;
                ++priv->stats.rx_errors;
            }

            if (status & (1UL << DMA_STATUS_RWT_BIT)) {
                printk(KERN_INFO "$Rint_handler() %s: RX watchdog timeout\n", dev->name);
                // Accumulate receive statistics
                ++priv->stats.rx_frame_errors;
                ++priv->stats.rx_errors;
                restart_watchdog = 1;
            }

            if (status & (1UL << DMA_STATUS_RPS_BIT)) {
                printk(KERN_INFO "$Rint_handler() %s: RX process stopped\n", dev->name);
                ++priv->stats.rx_errors;
                restart_watchdog = 1;
            }

            if (status & (1UL << DMA_STATUS_TPS_BIT)) {
                printk(KERN_INFO "$Rint_handler() %s: TX process stopped\n", dev->name);
                ++priv->stats.tx_errors;
                restart_watchdog = 1;
            }

            // Test for pure error interrupts
            if (status & (1UL << DMA_STATUS_FBE_BIT)) {
                printk(KERN_INFO "$Rint_handler() %s: Bus error\n", dev->name);
                restart_watchdog = 1;
            }

            if (restart_watchdog) {
                // Restart the link/PHY state watchdog immediately, which will
                // attempt to restart the system
                mod_timer(&priv->watchdog_timer, jiffies);
                restart_watchdog = 0;
            }
        }

        // Read the record of current interrupt requests again, in case some
        // more arrived while we were processing
        raw_status = dma_reg_read(priv, DMA_STATUS_REG);

        // Get status of enabled interrupt sources
        status = raw_status & int_enable;
DBG(1, KERN_INFO "int_handler() %s: raw_status = 0x%08x, status = 0x%08x, int_enable = 0x%08x\n", dev->name, raw_status, status, int_enable);
    }

DBG(1, KERN_INFO "int_handler() %s: Leaving\n", dev->name);

    return IRQ_HANDLED;
}

static void gmac_down(struct net_device *dev)
{
    gmac_priv_t* priv = (gmac_priv_t*)dev->priv;
    int desc;

DBG(1, KERN_INFO "gmac_down() %s\n", dev->name);

    // Stop further TX packets being delivered to hard_start_xmit();
    netif_stop_queue(dev);
    netif_carrier_off(dev);

    // Disable all GMAC interrupts
    dma_reg_write(priv, DMA_INT_ENABLE_REG, 0);

    // Release the IRQ
    if (priv->have_irq) {
        free_irq(dev->irq, dev);
        priv->have_irq = 0;
    }

    // Stop transmitter, take ownership of all tx descriptors
    dma_reg_clear_mask(priv, DMA_OP_MODE_REG, DMA_OP_MODE_ST_BIT);
    if (priv->desc_vaddr) {
        tx_take_ownership(&priv->tx_gmac_desc_list_info);
    }

    // Stop receiver, take ownership of all rx descriptors
    dma_reg_clear_mask(priv, DMA_OP_MODE_REG, DMA_OP_MODE_SR_BIT);
    if (priv->desc_vaddr) {
        rx_take_ownership(&priv->rx_gmac_desc_list_info);
    }

    // Stop all timers
    delete_watchdog_timer(priv);

    if (priv->desc_vaddr) {
        // Free receive descriptors
        do {
            dma_addr_t dma_address;
            u32 dma_length;
            struct sk_buff* skb;
            desc = get_rx_descriptor(priv, NULL, &dma_address, &dma_length, &skb);
            if ((desc >= 0) && skb) {
DBG(1, KERN_INFO "stop() %s: Got rx descriptor %d for skb %p\n", dev->name, desc, skb );

                // Release the DMA mapping for the socket buffer
                dma_unmap_single(0, dma_address, dma_length, DMA_FROM_DEVICE);

                // Free the socket buffer
                dev_kfree_skb(skb);
            }

        } while (desc >= 0);

        // Free transmit descriptors
        do {
            dma_addr_t dma_address;
            u32 dma_length;
            struct sk_buff* skb;
            desc = get_tx_descriptor(priv, NULL, &dma_address, &dma_length, &skb);
            if ((desc >= 0) && skb) {
DBG(1, KERN_INFO "stop() %s: Got tx descriptor %d for skb %p\n", dev->name, desc, skb );

                // Release the DMA mapping for the socket buffer
                dma_unmap_single(0, dma_address, dma_length, DMA_FROM_DEVICE);

                // Free the socket buffer
                dev_kfree_skb(skb);
            }
        } while (desc >= 0);

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

    // Power down the PHY - possibly should take Wake-On-LAN into account
    phy_powerdown(dev);
}

#ifdef USE_HASH_TABLE
/** returns hash bit number for given MAC address */
static int hash( u8 addr[6] )
{
  int i;
  u32 crc = 0xFFFFFFFF;
  u32 poly = 0xEDB88320;

  for( i=0; i<6; i++ )
  {
    int bit;
    u8 data = addr[i];
    for( bit=0; bit<8; bit++ )
    {
      int p = (crc^data) & 1;
      crc >>= 1;
      if( p != 0 ) crc ^= poly;
      data >>= 1;
    }
  }

  return (crc>>26) & 0x3F;      /* return upper 6 bits */
}
#endif // USE_HASH_TABLE

static int open(struct net_device *dev)
{
    gmac_priv_t* priv = (gmac_priv_t*)dev->priv;
    int desc;
    int status;
    u32 reg_contents;
    unsigned long end;

//DBG(1, KERN_INFO "open() %s:\n", dev->name );

    // Ensure the MAC block is properly reset
    writel(1UL << SYS_CTRL_RSTEN_MAC_BIT, SYS_CTRL_RSTEN_SET_CTRL);
    writel(1UL << SYS_CTRL_RSTEN_MAC_BIT, SYS_CTRL_RSTEN_CLR_CTRL);

    // Enable the clock to the MAC block
    writel(1UL << SYS_CTRL_CKEN_MAC_BIT, SYS_CTRL_CKEN_SET_CTRL);

    // Ensure reset and clock operations are complete
    wmb();

    // Disable all GMAC interrupts
    dma_reg_write(priv, DMA_INT_ENABLE_REG, 0);

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
        goto open_err_out;
    }

    // Check that the MAC address is valid.  If it's not, refuse to bring the
    // device up
    if (!is_valid_ether_addr(dev->dev_addr)) {
        DBG(1, KERN_ERR "open() %s: MAC address invalid\n", dev->name);
        status = -EINVAL;
        goto open_err_out;
    }

    // Allocate the IRQ
    if (request_irq(dev->irq, &int_handler, 0, dev->name, dev)) {
        DBG(1, KERN_ERR "open() %s: Failed to allocate irq %d\n", dev->name, dev->irq);
        status = -ENODEV;
        goto open_err_out;
    }
    priv->have_irq = 1;

    // Reset the PHY to get it into a known state
    if (phy_reset(priv->netdev)) {
        DBG(1, KERN_ERR "open() %s: Failed to reset PHY\n", dev->name);
        status = -EIO;
        goto open_err_out;
    }

#ifdef ARMULATING
    DBG(1, KERN_INFO "open() %s: Spoofing PHY\n", dev->name);
    netif_carrier_on(priv->mii.dev);
    priv->mii.full_duplex = 1;
#else // ARMULATING
    // Restart link auto negotiation
    if (mii_nway_restart(&priv->mii)) {
        DBG(1, KERN_ERR "start_auto_negotiation() %s: Failed to restart autonegotiation\n", dev->name);
        status = -EIO;
        goto open_err_out;
    }

    // Can't lock for negotiation, because will sleep - need to resolve this so
    // we're safe against e.g. ethtool caused PHY accesses
    if (wait_auto_negotiation_complete(priv)) {
        // End of serialised access to PHY/link
        DBG(1, KERN_ERR "open() %s: Failed to complete auto-negotiation\n", dev->name);
        status = -EIO;
        goto open_err_out;
    }

    // Interpret the PHY/link state., allowing printing of status
    mii_check_media(&priv->mii, 1, 1);
#endif // ARMULATING

    // Form the MAC config register contents
    reg_contents = 0;
    if (!priv->mii.supports_gmii) {
        reg_contents |= (1UL << MAC_CONFIG_PS_BIT);
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

    // Form the MAC frame filter register contents
    reg_contents = 0;
    mac_reg_write(priv, MAC_FRAME_FILTER_REG, reg_contents);

    // Form the hash table registers contents
    mac_reg_write(priv, MAC_HASH_HIGH_REG, 0);
    mac_reg_write(priv, MAC_HASH_LOW_REG, 0);

    // Form the MAC flow control register contents
    reg_contents = 0;
    reg_contents |= ((1UL << MAC_FLOW_CNTL_RFE_BIT) |
                     (1UL << MAC_FLOW_CNTL_TFE_BIT));
    mac_reg_write(priv, MAC_FLOW_CNTL_REG, reg_contents);

    // Form the MAC VLAN tag register contents
    reg_contents = 0;
    mac_reg_write(priv, MAC_VLAN_TAG_REG, reg_contents);

    // Form the MAC addr0 high and low registers contents
    reg_contents = (dev->dev_addr[5] << 8) | dev->dev_addr[4];
    mac_reg_write(priv, MAC_ADR0_HIGH_REG, reg_contents);
    reg_contents = (dev->dev_addr[3] << 24) |
                   (dev->dev_addr[2] << 16) |
                   (dev->dev_addr[1] << 8)  |
                   dev->dev_addr[0];
    mac_reg_write(priv, MAC_ADR0_LOW_REG, reg_contents);

    // Remember how large the unified descriptor array is to be
    priv->total_num_descriptors = NUM_TX_DMA_DESCRIPTORS + NUM_RX_DMA_DESCRIPTORS;

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
        DBG(1, KERN_ERR "probe() %s: Failed to allocate consistent memory for DMA descriptors\n", dev->name);
        status = -ENOMEM;
        goto open_err_out;
    }

    // Initialise the structures managing the TX descriptor list
    init_tx_desc_list(&priv->tx_gmac_desc_list_info,
                     priv->desc_vaddr,
                     NUM_TX_DMA_DESCRIPTORS);

    // Initialise the structures managing the RX descriptor list
    init_rx_desc_list(&priv->rx_gmac_desc_list_info,
                     priv->desc_vaddr + NUM_TX_DMA_DESCRIPTORS,
                     NUM_RX_DMA_DESCRIPTORS);

    // Prepare receive descriptors
    desc = 0;
    do {
        u32 dma_length;
        dma_addr_t dma_address;

        // Allocate a new skb for the descriptor ring which
        // is large enough for any packet received from the
        // link
        struct sk_buff *skb = dev_alloc_skb(dev->mtu + ETHER_FRAME_ALIGN_WASTAGE + EXTRA_RX_SKB_SPACE);
        if (!skb) {
            DBG(1, KERN_ERR "open() %s: No memory for socket buffer\n", dev->name);
            break;
        }

        // Despite what the comments in the original code
        // from Synopsys claimed, the GMAC DMA can cope with
        // non-quad aligned buffers - it will always perform
        // quad transfers but zero/ignore the unwanted bytes.
        skb_reserve(skb, ETHER_FRAME_ALIGN_WASTAGE);

        // Get a consistent DMA mapping for the memory to be
        // DMAed to - causing an invalidation of any entries
        // in the CPU's cache covering the memory region
        dma_length = skb_tailroom(skb);
        dma_address = dma_map_single(0, skb->tail, dma_length, DMA_FROM_DEVICE);
        BUG_ON(dma_mapping_error(dma_address));

        desc = set_rx_descriptor(priv, dma_address, dma_length, skb);
DBG(1, KERN_INFO "open() %s: Set rx descriptor %d for skb %p, len = %u, skb->tail = 0x%08x\n", dev->name, desc, skb, dma_length, (u32)skb->tail);

        if (desc < 0) {
            // Release the DMA mapping for the socket buffer
            dma_unmap_single(0, dma_address, dma_length, DMA_TO_DEVICE);
            dev_kfree_skb(skb);
        }
    } while (desc >= 0);

    // Initialise the GMAC DMA bus mode register
    dma_reg_write(priv, DMA_BUS_MODE_REG, ((0UL << DMA_BUS_MODE_FB_BIT)   |
                                           (0UL << DMA_BUS_MODE_PR_BIT)   |
                                           (32UL << DMA_BUS_MODE_PBL_BIT) | // AHB burst size
                                           (1UL << DMA_BUS_MODE_DSL_BIT)  |
                                           (0UL << DMA_BUS_MODE_DA_BIT)));

    // Write the physical DMA consistent address of the start of the tx descriptor array
DBG(1, KERN_INFO "$Wopen() TX desc base = 0x%08x\n", priv->desc_dma_addr);
    dma_reg_write(priv, DMA_TX_DESC_ADR_REG, priv->desc_dma_addr);

    // Write the physical DMA consistent address of the start of the rx descriptor array
    dma_reg_write(priv, DMA_RX_DESC_ADR_REG, priv->desc_dma_addr +
                        (priv->tx_gmac_desc_list_info.num_descriptors * sizeof(gmac_dma_desc_t)));

    // Clear any pending interrupt requests
    dma_reg_write(priv, DMA_STATUS_REG, dma_reg_read(priv, DMA_STATUS_REG));

    // Ensure setup is complete, before enabling TX and RX
    wmb();

    // Initialise the GMAC DMA operation mode register, starting both the
    // transmitter and receiver
    dma_reg_write(priv, DMA_OP_MODE_REG, ((1UL << DMA_OP_MODE_SF_BIT)  |    // Store and forward
                                          (0UL << DMA_OP_MODE_TTC_BIT) |    // Tx threshold
                                          (1UL << DMA_OP_MODE_ST_BIT)  |    // Enable transmitter
                                          (0UL << DMA_OP_MODE_RTC_BIT) |    // Rx threshold
                                          (1UL << DMA_OP_MODE_OSF_BIT) |    // Operate on 2nd frame
                                          (1UL << DMA_OP_MODE_SR_BIT)));    // Enable receiver

    // Enable interesting GMAC interrupts
    dma_reg_write(priv, DMA_INT_ENABLE_REG, INTR_EN_MASK);

    // Start the link/PHY state monitoring timer
    start_watchdog_timer(priv);

    // Allow the network stack to call hard_start_xmit()
    netif_start_queue(dev);

    return 0;

open_err_out:
    gmac_down(dev);

    // Disable the clock to the MAC block
    writel(1UL << SYS_CTRL_CKEN_MAC_BIT, SYS_CTRL_CKEN_CLR_CTRL);

    return status;
}

static int stop(struct net_device *dev)
{
DBG(1, KERN_INFO "stop() %s\n", dev->name);

    gmac_down(dev);

    // Disable the clock to the MAC block
    writel(1UL << SYS_CTRL_CKEN_MAC_BIT, SYS_CTRL_CKEN_CLR_CTRL);

    return 0;
}

static int hard_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
    gmac_priv_t* priv = (gmac_priv_t*)dev->priv;
    dma_addr_t dma_address;
    int desc;
    unsigned long irq_flags;

//DBG(1, KERN_INFO "hard_start_xmit() %s\n", dev->name );

    // Get a consistent DMA mapping for the memory to be DMAed from - causing a
    // flush from the CPU's cache to the memory
    dma_address = dma_map_single(0, skb->data, skb->len, DMA_TO_DEVICE);
    BUG_ON(dma_mapping_error(dma_address));

    // SMP-safe protection against concurrent operations in ISR and hard_start_xmit()
    spin_lock_irqsave(priv->spinlock, irq_flags);

    // Construct the GMAC specific DMA descriptor
    desc = set_tx_descriptor(priv, dma_address, skb->len, skb);
    if (desc < 0) {
        DBG(1, KERN_WARNING "hard_start_xmit() %s: No more free tx descriptors\n", dev->name);

        // Stop further calls to hard_start_xmit() until some desciptors are
        // freed up by already queued TX packets being completed
        netif_stop_queue(dev);

        // Should keep a record of the skb that we haven't been able to queue
        // for transmission and queue it as soon as a descriptor becomes free
        priv->tx_pending_skb = skb;
        priv->tx_pending_dma_addr = dma_address;
    } else {
DBG(1, KERN_INFO "$Mhard_start_xmit() %s: Set tx descriptor %d for skb %p, buf = 0x%08x, skb->data = 0x%08x\n", dev->name, desc, skb, dma_address, (u32)skb->data);
        // Record start of transmission, so timeouts will work once they're implemented
        dev->trans_start = jiffies;

        // Poke the transmitter to look for available TX descriptors, as we have
        // just added one, in case it had previously found there were no more
        // pending transmission
        dma_reg_write(priv, DMA_TX_POLL_REG, 0);
    }

    spin_unlock_irqrestore(priv->spinlock, irq_flags);

    return 0;
}

static struct net_device_stats *get_stats(struct net_device *dev)
{
    gmac_priv_t* priv = (gmac_priv_t*)dev->priv;

//    DBG(1, KERN_INFO "get_stats() %s\n", dev->name);

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
    disable_irq(netdev->irq);
    int_handler(netdev->irq, netdev, NULL);
    enable_irq(netdev->irq);
}
#endif // CONFIG_NET_POLL_CONTROLLER

static int probe(struct net_device *netdev, u32 vaddr, u32 irq)
{
    int err = 0;
    u32 version;
    int i;
    gmac_priv_t* priv;

//    DBG(1, KERN_INFO "probe() %s: Probing netdev=%p at addr=0x%x, irq=%u\n", netdev->name, netdev, vaddr, irq);

    // Ensure the MAC block is properly reset
    writel(1UL << SYS_CTRL_RSTEN_MAC_BIT, SYS_CTRL_RSTEN_SET_CTRL);
    writel(1UL << SYS_CTRL_RSTEN_MAC_BIT, SYS_CTRL_RSTEN_CLR_CTRL);

    // Enable the clock to the MAC block
    writel(1UL << SYS_CTRL_CKEN_MAC_BIT, SYS_CTRL_CKEN_SET_CTRL);

    // Ensure reset and clock operations are complete
    wmb();

    // Get memory for private device data
    netdev->priv = kmalloc(sizeof(gmac_priv_t), GFP_KERNEL | GFP_DMA);
    if (!netdev->priv) {
        DBG(1, KERN_ERR "probe() %s: Failed to allocate memory for device private data\n", netdev->name);
        err = -ENOMEM;
        goto probe_err_out;
    }

    // Ensure all of the device private data are zero, so we can clean up in
    // the event of a later failure to initialise all fields
    priv = (gmac_priv_t*)netdev->priv;
    memset(priv, 0, sizeof(gmac_priv_t));

    // Initialise the ISR/hard_start_xmit() lock
    spin_lock_init(&priv->spinlock);

    // Initialise the PHY access lock
    spin_lock_init(&priv->phy_lock);

    // Set hardware device base addresses
    priv->macBase = vaddr + MAC_BASE_OFFSET;
    priv->dmaBase = vaddr + DMA_BASE_OFFSET;

    // Initialise IRQ ownership to not owned
    priv->have_irq = 0;

    init_timer(&priv->watchdog_timer);
    priv->watchdog_timer.function = &watchdog_timer_action;
    priv->watchdog_timer.data = (unsigned long)priv;

    // Set pointer to device in private data
    priv->netdev = netdev;

    /** Do something here to detect the present or otherwise of the MAC
     *  Read the version register as a first test */
    version = mac_reg_read(priv, MAC_VERSION_REG);
    DBG(1, KERN_NOTICE "$WGMAC Synopsis version = 0x%x, vendor version = 0x%x\n", version & 0xff, (version >> 8) & 0xff);

    /** Assume device is at the adr and irq specified until have probing working */
    netdev->base_addr = vaddr;
    netdev->irq       = irq;

#ifdef DUMP_REGS_ON_PROBE
    dump_mac_regs(priv->macBase, priv->dmaBase);
#endif // DUMP_REGS_ON_PROBE

    // Allocate the IRQ
    err = request_irq(netdev->irq, &int_handler, 0, netdev->name, netdev);
    if (err) {
        DBG(1, KERN_ERR "probe() %s: Failed to allocate irq %d\n", netdev->name, netdev->irq);
        goto probe_err_out;
    }

    // Release the IRQ again, as open()/stop() should manage IRQ ownership
    free_irq(netdev->irq, netdev);

    // Initialise the ethernet device with std. contents
    ether_setup(netdev);

    // Set MAC address
    memcpy(netdev->dev_addr, DEFAULT_MAC_ADDRESS, netdev->addr_len);

    // Setup operations pointers
    netdev->open            = &open;
    netdev->hard_start_xmit = &hard_start_xmit;
    netdev->stop            = &stop;
    netdev->get_stats       = &get_stats;
#ifdef CONFIG_NET_POLL_CONTROLLER
    netdev->poll_controller = &netpoll;
#endif // CONFIG_NET_POLL_CONTROLLER
    set_ethtool_ops(netdev);

    if (debug) {
      netdev->flags |= IFF_DEBUG;
    }

    // hard_start_xmit() does its own locking
    netdev->features |= NETIF_F_LLTX;

    // Initialise PHY support
    priv->mii.phy_id_mask   = 0x1f;
    priv->mii.reg_num_mask  = 0x1f;
    priv->mii.force_media   = 0;
    priv->mii.full_duplex   = 0;
#ifdef SUPPORTS_GIGABIT
    priv->mii.supports_gmii = 1;
#endif // SUPPORTS_GIGABIT
    priv->mii.dev           = netdev;
    priv->mii.mdio_read     = phy_read;
    priv->mii.mdio_write    = phy_write;

    priv->gmii_csr_clk_range = 5;   // Slowest for now

#ifdef ALLOW_AUTONEG
    priv->allow_autonegotiation = 1;
#endif // ALLOW_AUTONEG

    // Attempt to locate the PHY
    phy_detect(netdev);

    // Did we find a PHY?
    if (!priv->phy_type) {
        DBG(1, KERN_WARNING "probe() %s No PHY found\n", netdev->name);
        goto probe_err_out;
    }

    // Register the device with the network intrastructure
    err = register_netdev(netdev);
    if (err) {
        DBG(1, KERN_ERR "probe() %s: Failed to register device\n", netdev->name);
        goto probe_err_out;
    }

    printk(KERN_NOTICE "$W%s: %s at 0x%lx, IRQ %d\n", netdev->name, version_string, netdev->base_addr, netdev->irq);
#ifndef ARMULATING
    printk(KERN_NOTICE "$W%s: Found PHY at address %u\n", priv->netdev->name, priv->phy_addr);
#endif // ARMULATING
    printk(KERN_NOTICE "$W%s: Ethernet addr: ", priv->netdev->name);
    for (i = 0; i < 5; i++) {
        printk("%02x:", netdev->dev_addr[i]);
    }
    printk("%02x\n", netdev->dev_addr[5]);

    return 0;

probe_err_out:
    if (netdev->priv) {
        kfree(netdev->priv);
    }

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
    static int probed;
    struct net_device *netdev = alloc_etherdev(0);
#endif // ALLOW_PROBING

//    DBG(1, KERN_INFO "synopsys_gmac_probe() Entered\n");

#ifdef ALLOW_PROBING
    // Will allocate private data later, as may want descriptors etc in special memory
    if (!netdev) {
        DBG(1, KERN_WARNING "synopsys_gmac_probe() failed to alloc device\n");
        err = -ENODEV;
    } else {
        if (unit >= 0) {
            sprintf(netdev->name, "eth%d", unit);

//DBG(1, KERN_INFO "synopsys_gmac_probe() Device name = '%s'\n", netdev->name);
            netdev_boot_setup_check(netdev);

//DBG(1, KERN_INFO "synopsys_gmac_probe(), unit=%d, netdev=%p\n", unit, netdev);
            if (probed) {
//DBG(1, KERN_INFO "synopsys_gmac_probe() Already probed\n");
                err = -ENODEV;
            } else {
                probed = 1;
                err = probe(netdev, MAC_BASE, MAC_INTERRUPT);
                if (err) {
                    DBG(1, KERN_WARNING "synopsys_gmac_probe() Probing failed\n");
                }
            }
        }

        if (err) {
//DBG(1, KERN_WARNING "synopsys_gmac_probe() Failure, freeing device\n");

            // Free any allocated memory resources
            if (netdev->priv) {
                // Free the private device data
                kfree(netdev->priv);
            }

            free_netdev(netdev);
            netdev->reg_state = NETREG_UNREGISTERED;
        }
    }
#else // ALLOW_PROBING
    DBG(1, KERN_WARNING "synopsys_gmac_probe() Forcing no probe\n");
    err = -ENODEV;
#endif // ALLOW_PROBING

//DBG(1, KERN_INFO "synopsys_gmac_probe() Leaving with err=%d\n", err);
    return ERR_PTR(err);
}
