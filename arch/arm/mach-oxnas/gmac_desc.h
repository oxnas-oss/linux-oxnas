/*
 * linux/arch/arm/mach-oxnas/gmac_desc.h
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
#if !defined(__GMAC_DESC_H__)
#define __GMAC_DESC_H__

#include <asm/types.h>
#include "gmac.h"

typedef enum rdes0 {
    RDES0_OWN_BIT  = 31,
    RDES0_AFM_BIT  = 30,
    RDES0_FL_BIT   = 16,
    RDES0_ES_BIT   = 15,
    RDES0_DE_BIT   = 14,
    RDES0_SAF_BIT  = 13,
    RDES0_LE_BIT   = 12,
    RDES0_OE_BIT   = 11,
    RDES0_IPC_BIT  = 10,
    RDES0_FS_BIT   = 9,
    RDES0_LS_BIT   = 8,
    RDES0_VLAN_BIT = 7,
    RDES0_LC_BIT   = 6,
    RDES0_FT_BIT   = 5,
    RDES0_RWT_BIT  = 4,
    RDES0_RE_BIT   = 3,
    RDES0_DRE_BIT  = 2,
    RDES0_CE_BIT   = 1,
    RDES0_MAC_BIT  = 0
} rdes0_t;

#define RX_DESC_STATUS_FL_NUM_BITS 14

typedef enum rdes1 {
    RDES1_DIC_BIT  = 31,
    RDES1_RER_BIT  = 25,
    RDES1_RCH_BIT  = 24,
    RDES1_RBS2_BIT = 11,
    RDES1_RBS1_BIT = 0,
} rdes1_t;

#define RX_DESC_LENGTH_RBS2_NUM_BITS 11
#define RX_DESC_LENGTH_RBS1_NUM_BITS 11

typedef enum tdes0 {
    TDES0_OWN_BIT = 31,
    TDES0_ES_BIT  = 15,
    TDES0_JT_BIT  = 14,
    TDES0_FF_BIT  = 13,
    TDES0_LOC_BIT = 11,
    TDES0_NC_BIT  = 10,
    TDES0_LC_BIT  = 9,
    TDES0_EC_BIT  = 8,
    TDES0_VF_BIT  = 7,
    TDES0_CC_BIT  = 3,
    TDES0_ED_BIT  = 2,
    TDES0_UF_BIT  = 1,
    TDES0_DB_BIT  = 0
} tdes0_t;

#define TDES0_CC_NUM_BITS 4

typedef enum tdes1 {
    TDES1_IC_BIT   = 31,
    TDES1_LS_BIT   = 30,
    TDES1_FS_BIT   = 29,
    TDES1_DC_BIT   = 26,
    TDES1_TER_BIT  = 25,
    TDES1_TCH_BIT  = 24,
    TDES1_DP_BIT   = 23,
    TDES1_TBS2_BIT = 11,
    TDES1_TBS1_BIT = 0
} tdes1_t;

#define TDES1_TBS2_NUM_BITS 11
#define TDES1_TBS1_NUM_BITS 11

extern void init_rx_desc_list(
    gmac_desc_list_info_t    *desc_list,
    volatile gmac_dma_desc_t *base_ptr,
    int                       num_descriptors);

extern void init_tx_desc_list(
    gmac_desc_list_info_t    *desc_list,
    volatile gmac_dma_desc_t *base_ptr,
    int                       num_descriptors);

/** Force ownership of all descriptors in the specified list to being owned by
 *  the CPU
 */
extern void rx_take_ownership(gmac_desc_list_info_t* desc_list);

/** Force ownership of all descriptors in the specified list to being owned by
 *  the CPU
 */
extern void tx_take_ownership(gmac_desc_list_info_t* desc_list);

/** Return the number of descriptors available for the CPU to fill with new
 *  packet info */
static inline int available_for_write(gmac_desc_list_info_t* desc_list)
{
    return desc_list->empty_count;
}

/** Return non-zero if there is a descriptor available with a packet with which
 *  the GMAC DMA has finished */
static inline int tx_available_for_read(
    volatile gmac_desc_list_info_t *desc_list,
    int                             first_in_packet)
{
    volatile gmac_dma_desc_t *desc      = desc_list->base_ptr + desc_list->r_index;
    int                       available = desc_list->full_count && !(desc->status & (1UL << TDES0_OWN_BIT));

    if (available && first_in_packet && !(desc->length & (1UL << TDES1_FS_BIT))) {
        printk(KERN_WARNING "First TX descriptor available for read (r_index = %d) does"
               " not have FS flag set (status = 0x%08x, length = 0x%08x, full_count = %d)\n",
               desc_list->r_index, desc->status, desc->length, desc_list->full_count);
        available = 0;
    }

    return available;
}

/** Return non-zero if there is a descriptor available with a packet with which
 *  the GMAC DMA has finished */
static inline int rx_available_for_read(
    volatile gmac_desc_list_info_t *desc_list,
    int                             first_in_packet)
{
    u32 status     = (desc_list->base_ptr + desc_list->r_index)->status;
    int full_count = desc_list->full_count;
    int available  = full_count && !(status & (1UL << RDES0_OWN_BIT));

//    if (available && first_in_packet && !(status & (1UL << RDES0_FS_BIT))) {
    if (available && first_in_packet && (!(status & (1UL << RDES0_FS_BIT)) || !(status & (1UL << RDES0_LS_BIT)))) {
//        printk(KERN_WARNING "First RX descriptor available for read (r_index = %d)"
//               " does not have FS flag set (status = 0x%08x, full_count = %d)\n",
//               desc_list->r_index, status, full_count);
        available = 0;
    }

    return available;
}

/** Fill in a rx descriptor and pass ownership to DMA engine */
extern int set_rx_descriptor(
    gmac_priv_t             *priv,
    dma_addr_t               dma_address,
    u32                      length,
    struct sk_buff          *skb,
    gmac_sram_buffer_info_t *sram_info);

/** Extract data from the next available RX descriptor with which the GMAC DMA
 *  DMA controller has finished
 */
extern int get_rx_descriptor(
    gmac_priv_t              *priv,
    u32*                      status,
    dma_addr_t               *dma_address,
    u32*                      length,
    struct sk_buff          **skb,
    gmac_sram_buffer_info_t **sram_info);

/** Fill in a tx descriptor and pass ownership to DMA engine */
extern int set_tx_descriptor(
    gmac_priv_t              *priv,
    dma_addr_t                dma_address,
    u32                       length,
    struct sk_buff           *skb,
    gmac_sram_buffer_info_t **sram_info);

/** Extract data from the next available TX descriptor with which the GMAC DMA
 *  DMA controller has finished
 */
extern int get_tx_descriptor(
    gmac_priv_t              *priv,
    u32*                      status,
    dma_addr_t               *dma_address,
    u32*                      length,
    struct sk_buff           **skb,
    gmac_sram_buffer_info_t ***sram_info);

/**
 * @param A u32 containing the status from a received frame's DMA descriptor
 * @return An int which is non-zero if a valid received frame is fully contained
 *  within the descriptor from whence the status came
 */
static inline int is_rx_valid(u32 status)
{
    return !(status & (1UL << RDES0_ES_BIT))  &&
           (status & (1UL << RDES0_FS_BIT)) &&
           (status & (1UL << RDES0_LS_BIT));
}

static inline int is_rx_dribbling(u32 status)
{
    return status & (1UL << RDES0_DRE_BIT);
}

static inline u32 get_rx_length(u32 status)
{
    return (status >> RDES0_FL_BIT) & ((1UL << RX_DESC_STATUS_FL_NUM_BITS) - 1);
}

static inline int is_rx_collision_error(u32 status)
{
    return status & ((1UL << RDES0_OE_BIT) | (1UL << RDES0_LC_BIT));
}

static inline int is_rx_crc_error(u32 status)
{
    return status & (1UL << RDES0_CE_BIT);
}

static inline int is_rx_frame_error(u32 status)
{
    return status & (1UL << RDES0_DE_BIT);
}

static inline int is_rx_length_error(u32 status)
{
    return status & (1UL << RDES0_LE_BIT);
}

static inline int is_rx_csum_error(u32 status)
{
    return status & (1UL << RDES0_IPC_BIT);
}

static inline int is_rx_long_frame(u32 status)
{
    return status & (1UL << RDES0_VLAN_BIT);
}

static inline int is_tx_valid(u32 status)
{
    return !(status & (1UL << TDES0_ES_BIT));
}

static inline int is_tx_collision_error(u32 status)
{
    return (status & (((1UL << TDES0_CC_NUM_BITS) - 1) << TDES0_CC_BIT)) >> TDES0_CC_BIT;
}

static inline int is_tx_aborted(u32 status)
{
    return status & ((1UL << TDES0_LC_BIT) | (1UL << TDES0_EC_BIT));
}

static inline int is_tx_carrier_error(u32 status)
{
    return status & ((1UL << TDES0_LOC_BIT) | (1UL << TDES0_NC_BIT));
}
#endif  //  #if !defined(__GMAC_DESC_H__)
