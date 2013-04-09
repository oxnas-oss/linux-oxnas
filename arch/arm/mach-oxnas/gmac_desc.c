/*
 * linux/arch/arm/mach-oxnas/gmac_desc.c
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
#include <linux/delay.h>

//#define GMAC_DEBUG
#undef GMAC_DEBUG

#include "gmac.h"
#include "gmac_desc.h"

static const unsigned MAX_GMAC_BUFFER_POW2 = 11;

inline void init_rx_descriptor(
    volatile gmac_dma_desc_t *desc,
    int                       end_of_ring,
    int                       disable_ioc)
{
    desc->status = 0;
    desc->length = 0;
    if (disable_ioc) {
        desc->length |= (1UL << RDES1_DIC_BIT);
    }
    if (end_of_ring) {
        desc->length |= (1UL << RDES1_RER_BIT);
    }
    desc->buffer1 = 0;
    desc->buffer2 = 0;
    desc->skb = 0;
}

inline void init_tx_descriptor(
    volatile gmac_dma_desc_t *desc,
    int                       end_of_ring,
    int                       enable_ioc,
    int                       disable_crc,
    int                       disable_padding)
{
    desc->status = 0;
    desc->length = 0;
    if (enable_ioc) {
        desc->length |= (1UL << TDES1_IC_BIT);
    }
    if (disable_crc) {
        desc->length |= (1UL << TDES1_DC_BIT);
    }
    if (disable_padding) {
        desc->length |= (1UL << TDES1_DP_BIT);
    }
    if (end_of_ring) {
        desc->length |= (1UL << TDES1_TER_BIT);
    }
    desc->buffer1 = 0;
    desc->buffer2 = 0;
    desc->skb = 0;
}

void init_rx_desc_list(
    gmac_desc_list_info_t    *desc_list,
    volatile gmac_dma_desc_t *base_ptr,
    int                       num_descriptors)
{
    int i;

    desc_list->base_ptr = base_ptr;
    desc_list->num_descriptors = num_descriptors;
    desc_list->empty_count = num_descriptors;
    desc_list->full_count = 0;
    desc_list->r_index = 0;
    desc_list->w_index = 0;

    if (num_descriptors) {
        for (i=0; i < (num_descriptors - 1); ++i) {
            init_rx_descriptor(base_ptr + i, 0, 0);
        }
        init_rx_descriptor(base_ptr + i, 1, 0);
    }
}

void init_tx_desc_list(
    gmac_desc_list_info_t    *desc_list,
    volatile gmac_dma_desc_t *base_ptr,
    int                       num_descriptors)
{
    int i;

    desc_list->base_ptr = base_ptr;
    desc_list->num_descriptors = num_descriptors;
    desc_list->empty_count = num_descriptors;
    desc_list->full_count = 0;
    desc_list->r_index = 0;
    desc_list->w_index = 0;

    if (num_descriptors) {
        for (i=0; i < (num_descriptors - 1); ++i) {
            init_tx_descriptor(base_ptr + i, 0, 1, 0, 0);
        }
        init_tx_descriptor(base_ptr + i, 1, 1, 0, 0);
    }
}

void rx_take_ownership(gmac_desc_list_info_t* desc_list)
{
    int i;
    for (i=0; i < desc_list->num_descriptors; ++i) {
        (desc_list->base_ptr + i)->status &= ~(1UL << RDES0_OWN_BIT);
    }

    // Ensure all write to the descriptor shared with MAC have completed
    wmb();
}

void tx_take_ownership(gmac_desc_list_info_t* desc_list)
{
    int i;
    for (i=0; i < desc_list->num_descriptors; ++i) {
        (desc_list->base_ptr + i)->status &= ~(1UL << TDES0_OWN_BIT);
    }

    // Ensure all write to the descriptor shared with MAC have completed
    wmb();
}

int set_rx_descriptor(
    gmac_priv_t             *priv,
    dma_addr_t               dma_address,
    u32                      length,
    struct sk_buff          *skb,
    gmac_sram_buffer_info_t *sram_info)
{
    int index = -1;

    BUG_ON(skb && sram_info);

DBG(38, "$GSRX() In: ec=%d, fc=%d, ri=%d, wi=%d\n", priv->rx_gmac_desc_list_info.empty_count, priv->rx_gmac_desc_list_info.full_count, priv->rx_gmac_desc_list_info.r_index, priv->rx_gmac_desc_list_info.w_index);
    // Are sufficicent descriptors available for writing by the CPU?
    if (!available_for_write(&priv->rx_gmac_desc_list_info)) {
//printk(KERN_WARNING "$Rset_rx_descriptor() Insufficient descriptors available\n");
    } else {
        // Setup the descriptor required to describe the RX packet
        volatile gmac_dma_desc_t *descriptor;
        unsigned                  desc_length;

        // Get the index of the next RX descriptor available for writing by the CPU
        index = priv->rx_gmac_desc_list_info.w_index;

DBG(38, "$GSRX() index=%d, dma_adr=0x%08x, len=%u, skb=0x%08x, sram_info=0x%08x\n", index, dma_address, length, (u32)skb, (u32)sram_info);
        // Get a pointer to the next RX descriptor available for writing by the CPU
        descriptor = priv->rx_gmac_desc_list_info.base_ptr + index;

        // Set first buffer pointer to buffer from either skb or SRAM info
        descriptor->buffer1 = sram_info ? sram_info->p_buffer_ : dma_address;

        // Set both the first and last descriptor flags, so only a single buffer
        // is to be described
        descriptor->length |= ((1UL << RDES0_LS_BIT) | (1UL << RDES0_FS_BIT));

        // Remember the socket buffer or SRAM buffer info associated with the buffer
        descriptor->skb = (u32)skb;
        descriptor->sram_info = (u32)sram_info;

        // Set length of buffer from either skb or SRAM info
        desc_length = sram_info ? sram_info->buffer_size_ : length;

        // Fill in the first buffer length to be the length of the current one
        descriptor->length &= ~(((1UL << RX_DESC_LENGTH_RBS1_NUM_BITS) - 1) << RDES1_RBS1_BIT);
        descriptor->length |= (desc_length << RDES1_RBS1_BIT);

        // No second buffer or chained descriptors for receive
        descriptor->length &= ~(1UL << RDES1_RCH_BIT);
        descriptor->length &= ~(((1UL << RX_DESC_LENGTH_RBS2_NUM_BITS) - 1) << RDES1_RBS2_BIT);

        // Ensure all prior writes to the descriptor shared with MAC have
        // completed before setting the descriptor ownership flag to transfer
        // ownership to the GMAC
        wmb();

        // Set RX descriptor status to transfer ownership to the GMAC
        descriptor->status = (1UL << RDES0_OWN_BIT);

DBG(38, "$GSRX() s=0x%08x, l=%u, b1=0x%08x, b2=0x%08x, sk=0x%08x, sr=0x%08x\n", descriptor->status, descriptor->length, descriptor->buffer1, descriptor->buffer2, descriptor->skb, descriptor->sram_info);
        // Update the index of the next descriptor available for writing by the CPU
        priv->rx_gmac_desc_list_info.w_index =
            (descriptor->length & (1UL << RDES1_RER_BIT)) ? 0 : index + 1;

        // Account for the descriptor used to hold the new packet
        --priv->rx_gmac_desc_list_info.empty_count;
        ++priv->rx_gmac_desc_list_info.full_count;
    }
DBG(38, "$GSRX() Out: ec=%d, fc=%d, ri=%d, wi=%d\n", priv->rx_gmac_desc_list_info.empty_count, priv->rx_gmac_desc_list_info.full_count, priv->rx_gmac_desc_list_info.r_index, priv->rx_gmac_desc_list_info.w_index);

    return index;
}

int get_rx_descriptor(
    gmac_priv_t              *priv,
    u32                      *status,
    dma_addr_t               *dma_address,  // For SKB case, ptr to storage buffer, else for SRAM case the total Rx packet length
    u32                      *length,       // For SKB case, ptr to storage for packet length, else for SRAM case, ptr to length of sram_info array
    struct sk_buff          **skb,
    gmac_sram_buffer_info_t **sram_infos)   // Ptr to storage for sram info structure pointers
{
    int first_descriptor_index = -1;
    int num_descriptors_used   = 0;
    u32 compound_status        = 0;
    u32 packet_so_far          = 0;

    BUG_ON(skb && sram_infos);

DBG(37, "$MGRX() In: ec=%d, fc=%d, ri=%d, wi=%d\n", priv->rx_gmac_desc_list_info.empty_count, priv->rx_gmac_desc_list_info.full_count, priv->rx_gmac_desc_list_info.r_index, priv->rx_gmac_desc_list_info.w_index);
    // Process all descriptors used to describe the packet
    while (rx_available_for_read(&priv->rx_gmac_desc_list_info, first_descriptor_index == -1)) {
        int is_first_descriptor;

        // Get the index of the descriptor released the longest time ago by the GMAC DMA
        int index = priv->rx_gmac_desc_list_info.r_index;

        // Get a pointer to the descriptor released the longest time ago by the GMAC DMA
        volatile gmac_dma_desc_t *descriptor = priv->rx_gmac_desc_list_info.base_ptr + index;

        // Is this descriptor flagged by the GMAC as the first for the packet?
        is_first_descriptor = descriptor->status & (1UL << RDES0_FS_BIT);

        // Check that the first descriptor for the packet has the FS flag set
        if (!is_first_descriptor && (first_descriptor_index == -1)) {
            printk(KERN_WARNING "First descriptor for RX packet does not have FS flag set\n");
            break;
        }

        // Update the index of the next descriptor with which the GMAC DMA may have finished
        priv->rx_gmac_desc_list_info.r_index = (descriptor->length & (1UL << RDES1_RER_BIT)) ? 0 : index + 1;

        // Accumulate the status from all descriptors for the packet
        compound_status |= descriptor->status;

        // Account for the descriptor which is now no longer waiting to be processed by the CPU
        ++priv->rx_gmac_desc_list_info.empty_count;
        --priv->rx_gmac_desc_list_info.full_count;

        // Keep count of the number of descriptors describing the packet
        ++num_descriptors_used;

        if (is_first_descriptor) {
            // Check for getting first descriptor more than once for a single packet
            if (first_descriptor_index != -1) {
                // This will result in a descriptor status without the last
                // segment flag set and will thus be treated as invalid
                printk(KERN_WARNING "$Yget_rx_descriptor() Have first segment flag twice for single packet\n");
                break;
            }

            // Have the first descriptor describing the packet, so extract the
            // buffer start address and the pointer to any associated skb
            first_descriptor_index = index;

            // Extract the pointer to the socket buffer associated with the packet
            if (skb) {
                u32 desc_length = 0;

                *skb = (struct sk_buff*)(descriptor->skb);

                // Extract the pointer to the buffer containing the packet
                if (dma_address) {
                    *dma_address = descriptor->buffer1;
                }

                // When using a skb, can only have a single descriptor, so must
                // be finished
                if (!(descriptor->status & (1UL << RDES0_LS_BIT))) {
                    // Indicate problem by returning length as zero
                    printk(KERN_WARNING "$Yget_rx_descriptor() Not a single descriptor for skb case\n");
                } else {
                    desc_length = get_rx_length(descriptor->status);
                }

                // Store the length of the single packet for the skb
                if (length) {
                    *length = desc_length;
                }
                break;
            }
        }

        if (sram_infos) {
            gmac_sram_buffer_info_t *sram_info =
                (gmac_sram_buffer_info_t*)descriptor->sram_info;

            // Ensure the caller-supplied array of SRAM info structure pointers
            // is large enough
            if (num_descriptors_used > *length) {
                printk(KERN_WARNING "$Yget_rx_descriptor() sram_info ptr array too small\n");
                *length = num_descriptors_used-1;

                // Indicate problem by returning length as zero
                *dma_address = 0;
                break;
            }

            // Store the pointer to the SRAM info structure in the caller-
            // supplied array
            sram_infos[num_descriptors_used-1] = sram_info;

            if (descriptor->status & (1UL << RDES0_LS_BIT)) {
                // Determine the complete Rx packet length
                u32 packet_length = get_rx_length(descriptor->status);

                // Have last descriptor describing the packet. If using SRAM
                // buffers pass back the number of sram info structure ptr array
                // entries filled with valid pointers
                *length = num_descriptors_used;

                // Get the total length of the packet
                *dma_address = packet_length;

                // Calculate how much of the final descriptor's associated SRAM
                // buffer is filled with packet data
                sram_info->packet_length_ = packet_length - packet_so_far;
DBG(37, "$MGRX() Final buffer of Rx packet, packet_length = %u, *dma_address = %u, num_descriptors_used = %u, *length = %u, sram_info->packet_length_ = %lu\n", packet_length, *dma_address, num_descriptors_used, *length, sram_info->packet_length_);
                break;
            } else {
                // Record the size of valid data in the SRAM buffer. Until the
                // last descriptor, the entire SRAM buffer will contain packet
                // data
                sram_info->packet_length_ = sram_info->buffer_size_;

                // Remember length of packet received so far
                packet_so_far += sram_info->buffer_size_;
            }
        }
    }

    if (status) {
        *status = compound_status;
    }

DBG(6, "$MGRX() fi=%u, *stat=0x%08x, *dma_adr=0x%08x, *len=%u, skb=0x%08x\n", first_descriptor_index, status?*status:0, dma_address?*dma_address:0, length?*length:0, skb?(u32)*skb:0);

DBG(37, "$MGRX() Out: ec=%d, fc=%d, ri=%d, wi=%d\n", priv->rx_gmac_desc_list_info.empty_count, priv->rx_gmac_desc_list_info.full_count, priv->rx_gmac_desc_list_info.r_index, priv->rx_gmac_desc_list_info.w_index);
    return first_descriptor_index;
}

int set_tx_descriptor(
    gmac_priv_t              *priv,
    dma_addr_t                dma_address,
    u32                       length,
    struct sk_buff           *skb,
    gmac_sram_buffer_info_t **sram_info)
{
    int                       first_descriptor_index = -1;
    volatile gmac_dma_desc_t *current_descriptor;
    unsigned                  num_descriptors_used = 0;
    unsigned                  offset = 0;
    volatile gmac_dma_desc_t *previous_descriptor = 0;
    unsigned                  num_descriptors_required;

    // Calculate how many GMAC descriptors are required to describe the packet
    if (sram_info) {
        num_descriptors_required = length;
    } else {
        num_descriptors_required = length >> MAX_GMAC_BUFFER_POW2;
        if ((num_descriptors_required << MAX_GMAC_BUFFER_POW2) < length) {
            ++num_descriptors_required;
        }
    }

    // Are sufficicent descriptors available for writing by the CPU?
    if (available_for_write(&priv->tx_gmac_desc_list_info) < num_descriptors_required) {
//printk(KERN_WARNING "$Rset_tx_descriptor() Insufficient descriptors available\n");
        return -1;
    }

    // Setup all descriptors required to describe the TX packet
    while (num_descriptors_used < num_descriptors_required) {
        int index;
        unsigned desc_length;
        unsigned remainder;
DBG(35, "$CSTX() Desc. num = %u of %u\n", num_descriptors_used, num_descriptors_required);

        // Get the index of the next TX descriptor available for writing by the CPU
        index = priv->tx_gmac_desc_list_info.w_index;

        // Get a pointer to the next TX descriptor available for writing by the CPU
        current_descriptor = priv->tx_gmac_desc_list_info.base_ptr + index;

        // Initialise the first buffer pointer to the single passed buffer
        if (sram_info) {
            current_descriptor->buffer1 = sram_info[num_descriptors_used]->p_buffer_;
        } else {
            current_descriptor->buffer1 = dma_address + offset;
        }

        // Clear the first/last descriptor flags
        current_descriptor->length &= ~((1UL << TDES1_LS_BIT) | (1UL << TDES1_FS_BIT));

        // Is the current descriptor the first for the packet?
        if (!offset) {
DBG(35, "$CSTX() First descriptor\n");
            // Yes
            first_descriptor_index = index;

            // Remember the socket buffer associated with the single passed buffer
            current_descriptor->skb = (u32)skb;

            // Remember the SRAM buffer info associated with the single passed buffer
            current_descriptor->sram_info = (u32)sram_info;

            // Set flag indicating is first descriptor for packet
            current_descriptor->length |= (1UL << TDES1_FS_BIT);
        }

        // How much of the packet should be described by the current descriptor?
        if (sram_info) {
            desc_length = sram_info[num_descriptors_used]->packet_length_;
        } else {
            remainder = length - offset;
            desc_length = (remainder > (1UL << MAX_GMAC_BUFFER_POW2)) ?
                (1UL << MAX_GMAC_BUFFER_POW2) : remainder;
        }
DBG(35, "$CSTX() buffer_ = 0x%08x, length = %u\n", current_descriptor->buffer1, desc_length);

        // Fill in the first buffer length to be the length of the current one
        current_descriptor->length &= ~(((1UL << TDES1_TBS1_NUM_BITS) - 1) << TDES1_TBS1_BIT);
        current_descriptor->length |= (desc_length << TDES1_TBS1_BIT);

        // May have a second chained descriptor, but never a second buffer, so
        // ensure second buffer length field is zeroed and clear the flag indi-
        // cating whether there is a chained descriptor
        current_descriptor->length &= ~(1UL << TDES1_TCH_BIT);
        current_descriptor->length &= ~(((1UL << TDES1_TBS2_NUM_BITS) - 1) << TDES1_TBS2_BIT);

        // Was there a previous descriptor for this packet?
        if (previous_descriptor) {
            // Make the previous descriptor chain to the current one
            previous_descriptor->length |= (1UL << TDES1_TCH_BIT);
            previous_descriptor->buffer2 |= descriptors_virt_to_phys((u32)current_descriptor);
        }

        // Update record of how much of packet has been described
        offset += desc_length;

        // Update count of number of descriptors used
        ++num_descriptors_used;

        // Has current descriptor consumed the entire remainder of the packet?
        if (((sram_info) && (num_descriptors_used == num_descriptors_required)) ||
            (offset == length)) {
            // Yes, so set flag indicating it's the last descriptor for the packet
DBG(35, "$CSTX() Last descriptor\n");
            current_descriptor->length |= (1UL << TDES1_LS_BIT);
        } else {
            // Will require another descriptor to be chained after the current
            // one, so save the address of the current descriptor
DBG(35, "$CSTX() Chaining descriptor\n");
            previous_descriptor = current_descriptor;
        }

        // Ensure all prior writes to the descriptor shared with MAC have
        // completed before setting the descriptor ownership flag to transfer
        // ownership to the GMAC
        wmb();

        // Set RX descriptor status to transfer ownership to the GMAC
        current_descriptor->status = (1UL << TDES0_OWN_BIT);

        // Update the index of the next descriptor available for writing by the CPU
        priv->tx_gmac_desc_list_info.w_index =
            (current_descriptor->length & (1UL << TDES1_TER_BIT)) ? 0 : index + 1;
    }

    // Account for the number of descriptors used to hold the new packet
    priv->tx_gmac_desc_list_info.empty_count -= num_descriptors_used;
    priv->tx_gmac_desc_list_info.full_count  += num_descriptors_used;


DBG(35, "$CSTX() First descriptor used = %d, number used = %u\n", first_descriptor_index, num_descriptors_used);
    return first_descriptor_index;
}

int get_tx_descriptor(
    gmac_priv_t               *priv,
    u32                       *status,
    dma_addr_t                *dma_address,
    u32                       *length,
    struct sk_buff           **skb,
    gmac_sram_buffer_info_t ***sram_info)
{
    int num_descriptors_used = 0;
    u32 compound_status = 0;
    u32 total_length = 0;
    int first_descriptor_index = -1;

    // Process all descriptors used to describe the packet
    while (tx_available_for_read(&priv->tx_gmac_desc_list_info, first_descriptor_index == -1)) {
        int is_first_descriptor;

        // Get the index of the descriptor released the longest time ago by the GMAC DMA
        int index = priv->tx_gmac_desc_list_info.r_index;

        // Get a pointer to the descriptor released the longest time ago by the
        // GMAC DMA
        volatile gmac_dma_desc_t *current_descriptor = priv->tx_gmac_desc_list_info.base_ptr + index;

        // Is this descriptor flagged by the GMAC as the first for the packet?
        is_first_descriptor = current_descriptor->length & (1UL << TDES1_FS_BIT);

        // Check that the first descriptor for the packet has the FS flag set
        if (!is_first_descriptor && (first_descriptor_index == -1)) {
            printk(KERN_WARNING "First descriptor for TX packet does not have FS flag set\n");
            break;
        }

        // Update the index of the next descriptor with which the GMAC DMA may
        // have finished
        priv->tx_gmac_desc_list_info.r_index = (current_descriptor->length & (1UL << TDES1_TER_BIT)) ? 0 : index + 1;

        // Account for the descriptor which is now no longer waiting to be
        // processed by the CPU
        ++priv->tx_gmac_desc_list_info.empty_count;
        --priv->tx_gmac_desc_list_info.full_count;

        // Keep count of the number of descriptors describing the packet
        ++num_descriptors_used;

        // Accumulate the status from all descriptors for the packet
        compound_status |= current_descriptor->status;

        // Add this descriptor's length to the total so far for the packet
        total_length += ((current_descriptor->length >> TDES1_TBS1_BIT) &
                         ((1UL << TDES1_TBS1_NUM_BITS) - 1));

        if (is_first_descriptor) {
DBG(36, "$BGTD() 1st descriptor\n");
            // Check for getting first segment more than once
            if (first_descriptor_index != -1) {
                // This will result in a descriptor status without the last
                // segment flag set and will thus be treated as invalid
                printk(KERN_WARNING "$Yget_tx_descriptor() Have first segment flag twice for single packet\n");
                break;
            }

            // Have the first descriptor describing the packet, so extract the
            // buffer start address and the pointer to any associated skb
            first_descriptor_index = index;

            // Extract the pointer to the buffer containing the packet
            if (dma_address) {
                *dma_address = current_descriptor->buffer1;
            }

            // Extract the pointer to the socket buffer associated with the packet
            if (skb) {
                *skb = (struct sk_buff*)(current_descriptor->skb);
            }

            // Extract the pointer to the SRAM buffer info structure associated with the packet
            if (sram_info) {
                *sram_info = (gmac_sram_buffer_info_t**)(current_descriptor->sram_info);
            }
        }

        if (current_descriptor->length & (1UL << TDES1_LS_BIT)) {
            // Have last descriptor describing the packet
DBG(36, "$BGTX() Last descriptor\n");
            if (!is_tx_valid(compound_status)) {
                DBG(1, "BAD Tx packet:\n");
                DBG(1, "status    = 0x%08x\n", current_descriptor->status);
                DBG(1, "length    = 0x%08x\n", current_descriptor->length);
                DBG(1, "buffer1   = 0x%08x\n", current_descriptor->buffer1);
                DBG(1, "buffer2   = 0x%08x\n", current_descriptor->buffer2);
                DBG(1, "skb       = 0x%08x\n", current_descriptor->skb);
                DBG(1, "sram_info = 0x%08x\n", current_descriptor->sram_info);
            }
            break;
        }
    }

    if (status) {
        *status = compound_status;
    }
    if (length) {
        *length = total_length;
    }

DBG(36, "$BGTX() Returning index = %u, %u descriptors contributed\n", first_descriptor_index, num_descriptors_used);
    return first_descriptor_index;
}
