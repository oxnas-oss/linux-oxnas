/*
 * linux/arch/arm/mach-oxnas/dma.c
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
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/arch/hardware.h>
#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#ifdef CONFIG_LEON_COPRO
#include <asm/arch/leon.h>
#endif // CONFIG_LEON_COPRO
#include <asm/arch/desc_alloc.h>

#include <asm/checksum.h>

//#define DMA_DEBUG
//#define DEBUG_PRD

#define OXNAS_DMA_NON_FILTERED_SG_INTS

#ifdef OXNAS_DMA_TEST
#define DMA_DEBUG
static void dma_test(unsigned long length);
#endif // OXNAS_DMA_TEST

#ifdef OXNAS_DMA_SG_TEST
#define DMA_DEBUG
static void dma_sg_test(void);
#endif // OXNAS_DMA_SG_TEST

#ifdef OXNAS_DMA_SG_TEST_2
#define DMA_DEBUG
static void dma_sg_test2(void);
#endif // OXNAS_DMA_SG_TEST_2

#ifdef DMA_DEBUG
#define DBG(args...) printk(args)
#else
#define DBG(args...) do { } while(0)
#endif

// Normal (non-SG) registers
#define DMA_REGS_PER_CHANNEL 8

#define DMA_CTRL_STATUS      0x0
#define DMA_BASE_SRC_ADR     0x4
#define DMA_BASE_DST_ADR     0x8
#define DMA_BYTE_CNT         0xC
#define DMA_CURRENT_SRC_ADR  0x10
#define DMA_CURRENT_DST_ADR  0x14
#define DMA_CURRENT_BYTE_CNT 0x18
#define DMA_INTR_ID          0x1C
#define DMA_INTR_CLEAR_REG   (DMA_CURRENT_SRC_ADR)

// 8 quad-sized registers per channel arranged contiguously
#define DMA_CALC_REG_ADR(channel, register) (DMA_BASE + ((channel) << 5) + (register))

#define DMA_CTRL_STATUS_FAIR_SHARE_ARB            (1 << 0)
#define DMA_CTRL_STATUS_IN_PROGRESS               (1 << 1)
#define DMA_CTRL_STATUS_SRC_DREQ_MASK             (0x0000003C)
#define DMA_CTRL_STATUS_SRC_DREQ_SHIFT            2
#define DMA_CTRL_STATUS_DEST_DREQ_MASK            (0x000003C0)
#define DMA_CTRL_STATUS_DEST_DREQ_SHIFT           6
#define DMA_CTRL_STATUS_INTR                      (1 << 10)
#define DMA_CTRL_STATUS_NXT_FREE                  (1 << 11)
#define DMA_CTRL_STATUS_RESET                     (1 << 12)
#define DMA_CTRL_STATUS_DIR_MASK                  (0x00006000)
#define DMA_CTRL_STATUS_DIR_SHIFT                 13
#define DMA_CTRL_STATUS_SRC_ADR_MODE              (1 << 15)
#define DMA_CTRL_STATUS_DEST_ADR_MODE             (1 << 16)
#define DMA_CTRL_STATUS_TRANSFER_MODE_A           (1 << 17)
#define DMA_CTRL_STATUS_TRANSFER_MODE_B           (1 << 18)
#define DMA_CTRL_STATUS_SRC_WIDTH_MASK            (0x00380000)
#define DMA_CTRL_STATUS_SRC_WIDTH_SHIFT           19
#define DMA_CTRL_STATUS_DEST_WIDTH_MASK           (0x01C00000)
#define DMA_CTRL_STATUS_DEST_WIDTH_SHIFT          22
#define DMA_CTRL_STATUS_PAUSE                     (1 << 25)
#define DMA_CTRL_STATUS_INTERRUPT_ENABLE          (1 << 26)
#define DMA_CTRL_STATUS_SOURCE_ADDRESS_FIXED      (1 << 27)
#define DMA_CTRL_STATUS_DESTINATION_ADDRESS_FIXED (1 << 28)
#define DMA_CTRL_STATUS_STARVE_LOW_PRIORITY       (1 << 29)
#define DMA_CTRL_STATUS_INTR_CLEAR_ENABLE         (1 << 30)

#define DMA_BYTE_CNT_MASK                         ((1 << 21) - 1)
#define DMA_BYTE_CNT_INC4_SET_MASK                (1 << 28)
#define DMA_BYTE_CNT_HPROT_MASK                   (1 << 29)
#define DMA_BYTE_CNT_WR_EOT_MASK                  (1 << 30)
#define DMA_BYTE_CNT_RD_EOT_MASK                  (1 << 31)

#define DMA_INTR_ID_GET_NUM_CHANNELS(reg_contents) (((reg_contents) >> 16) & 0xFF)
#define DMA_INTR_ID_GET_VERSION(reg_contents)      (((reg_contents) >> 24) & 0xFF)
#define DMA_INTR_ID_INT_BIT         0
#define DMA_INTR_ID_INT_NUM_BITS    (MAX_OXNAS_DMA_CHANNELS)
#define DMA_INTR_ID_INT_MASK        (((1 << DMA_INTR_ID_INT_NUM_BITS) - 1) << DMA_INTR_ID_INT_BIT)

#define DMA_HAS_V4_INTR_CLEAR(version) ((version) > 3)

// H/W scatter gather controller registers
#define OXNAS_DMA_NUM_SG_REGS 4

#define DMA_SG_CONTROL  0x0
#define DMA_SG_STATUS   0x04
#define DMA_SG_REQ_PTR  0x08
#define DMA_SG_RESETS   0x0C

#define DMA_SG_CALC_REG_ADR(channel, register) ((DMA_SG_BASE) + ((channel) * ((OXNAS_DMA_NUM_SG_REGS) * sizeof(unsigned long))) + (register))

// SG DMA controller control register field definitions
#define DMA_SG_CONTROL_START_BIT            0
#define DMA_SG_CONTROL_QUEUING_ENABLE_BIT   1
#define DMA_SG_CONTROL_HBURST_ENABLE_BIT    2

// SG DMA controller status register field definitions
#define DMA_SG_STATUS_ERROR_CODE_BIT        0
#define DMA_SG_STATUS_ERROR_CODE_NUM_BITS   6
#define DMA_SG_STATUS_BUSY_BIT              7

// SG DMA controller sub-block resets register field definitions
#define DMA_SG_RESETS_CONTROL_BIT	    0
#define DMA_SG_RESETS_ARBITER_BIT	    1
#define DMA_SG_RESETS_AHB_BIT	    	    2

// oxnas_dma_sg_info_t qualifier field definitions
#define OXNAS_DMA_SG_QUALIFIER_BIT      0
#define OXNAS_DMA_SG_QUALIFIER_NUM_BITS 16
#define OXNAS_DMA_SG_DST_EOT_BIT        16
#define OXNAS_DMA_SG_DST_EOT_NUM_BITS   2
#define OXNAS_DMA_SG_SRC_EOT_BIT        20
#define OXNAS_DMA_SG_SRC_EOT_NUM_BITS   2
#define OXNAS_DMA_SG_CHANNEL_BIT        24
#define OXNAS_DMA_SG_CHANNEL_NUM_BITS   8

// Address map aliases every 256MB
#define OXNAS_DMA_ADR_NUM_BITS  28

// Valid address bits mask
#define OXNAS_DMA_ADR_MASK       ((1UL << (OXNAS_DMA_ADR_NUM_BITS)) - 1)
#define OXNAS_DMA_CSUM_ADR_MASK  ((OXNAS_DMA_ADR_MASK) | (1UL << (OXNAS_DMA_CSUM_ENABLE_ADR_BIT)))

/* The available buses to which the DMA controller is attached */
typedef enum oxnas_dma_transfer_bus
{
    OXNAS_DMA_SIDE_A,
    OXNAS_DMA_SIDE_B
} oxnas_dma_transfer_bus_t;

/* Direction of data flow between the DMA controller's pair of interfaces */
typedef enum oxnas_dma_transfer_direction
{
    OXNAS_DMA_A_TO_A,
    OXNAS_DMA_B_TO_A,
    OXNAS_DMA_A_TO_B,
    OXNAS_DMA_B_TO_B
} oxnas_dma_transfer_direction_t;

/* The available data widths */
typedef enum oxnas_dma_transfer_width
{
    OXNAS_DMA_TRANSFER_WIDTH_8BITS,
    OXNAS_DMA_TRANSFER_WIDTH_16BITS,
    OXNAS_DMA_TRANSFER_WIDTH_32BITS
} oxnas_dma_transfer_width_t;

/* The mode of the DMA transfer */
typedef enum oxnas_dma_transfer_mode
{
    OXNAS_DMA_TRANSFER_MODE_SINGLE,
    OXNAS_DMA_TRANSFER_MODE_BURST
} oxnas_dma_transfer_mode_t;

/* The available transfer targets */
typedef enum oxnas_dma_dreq
{
    OXNAS_DMA_DREQ_PATA   = 0,
    OXNAS_DMA_DREQ_SATA   = 0,
    OXNAS_DMA_DREQ_DPE_RX = 1,
    OXNAS_DMA_DREQ_DPE_TX = 2,
    OXNAS_DMA_DREQ_MEMORY = 15
} oxnas_dma_dreq_t;

/* Pre-defined settings for known DMA devices */
oxnas_dma_device_settings_t oxnas_pata_dma_settings = {
    .address_              = 0,
    .fifo_size_            = 16,
    .dreq_                 = OXNAS_DMA_DREQ_PATA,
    .read_eot_policy_      = OXNAS_DMA_EOT_FINAL,
    .write_eot_policy_     = OXNAS_DMA_EOT_FINAL,
    .bus_                  = OXNAS_DMA_SIDE_A,
    .width_                = OXNAS_DMA_TRANSFER_WIDTH_32BITS,
    .transfer_mode_        = OXNAS_DMA_TRANSFER_MODE_BURST,
    .address_mode_         = OXNAS_DMA_MODE_FIXED,
    .address_really_fixed_ = 0
};

oxnas_dma_device_settings_t oxnas_sata_dma_settings = {
    .address_              = SATA_DATA_BASE_PA,
    .fifo_size_            = 16,
    .dreq_                 = OXNAS_DMA_DREQ_SATA,
    .read_eot_policy_      = OXNAS_DMA_EOT_FINAL,
    .write_eot_policy_     = OXNAS_DMA_EOT_FINAL,
    .bus_                  = OXNAS_DMA_SIDE_A,
    .width_                = OXNAS_DMA_TRANSFER_WIDTH_32BITS,
    .transfer_mode_        = OXNAS_DMA_TRANSFER_MODE_BURST,
    .address_mode_         = OXNAS_DMA_MODE_FIXED,
    .address_really_fixed_ = 0
};

oxnas_dma_device_settings_t oxnas_dpe_rx_dma_settings = {
    .address_              = DPE_BASE_PA,
    .fifo_size_            = 16,
    .dreq_                 = OXNAS_DMA_DREQ_DPE_RX,
    .read_eot_policy_      = OXNAS_DMA_EOT_FINAL,
    .write_eot_policy_     = OXNAS_DMA_EOT_FINAL,
    .bus_                  = OXNAS_DMA_SIDE_A,
    .width_                = OXNAS_DMA_TRANSFER_WIDTH_32BITS,
    .transfer_mode_        = OXNAS_DMA_TRANSFER_MODE_BURST,
    .address_mode_         = OXNAS_DMA_MODE_FIXED,
    .address_really_fixed_ = 0
};

oxnas_dma_device_settings_t oxnas_dpe_tx_dma_settings = {
    .address_              = DPE_BASE_PA,
    .fifo_size_            = 16,
    .dreq_                 = OXNAS_DMA_DREQ_DPE_TX,
    .read_eot_policy_      = OXNAS_DMA_EOT_FINAL,
    .write_eot_policy_     = OXNAS_DMA_EOT_FINAL,
    .bus_                  = OXNAS_DMA_SIDE_A,
    .width_                = OXNAS_DMA_TRANSFER_WIDTH_32BITS,
    .transfer_mode_        = OXNAS_DMA_TRANSFER_MODE_BURST,
    .address_mode_         = OXNAS_DMA_MODE_FIXED,
    .address_really_fixed_ = 0
};

/* For use with normal memory to memory transfers as the settings for the source
 * of the transfer */
oxnas_dma_device_settings_t oxnas_ram_only_src_dma_settings = {
    .address_              = 0,
    .fifo_size_            = 0,
    .dreq_                 = OXNAS_DMA_DREQ_MEMORY,
    .read_eot_policy_      = OXNAS_DMA_EOT_FINAL,   // Won't interfere with checksumming transfers, as csum only latched if high order address bit set
    .write_eot_policy_     = OXNAS_DMA_EOT_NONE,
    .bus_                  = OXNAS_DMA_SIDE_A,      // Maximise performance with src on side A while dst in on side B
    .width_                = OXNAS_DMA_TRANSFER_WIDTH_32BITS,
    .transfer_mode_        = OXNAS_DMA_TRANSFER_MODE_BURST,
    .address_mode_         = OXNAS_DMA_MODE_FIXED,
    .address_really_fixed_ = 1
};

/* For use with checksumming transfers as the settings for the source of the
 * transfer */
oxnas_dma_device_settings_t oxnas_ram_csum_src_dma_settings = {
    .address_              = 0,
    .fifo_size_            = 0,
    .dreq_                 = OXNAS_DMA_DREQ_MEMORY,
    .read_eot_policy_      = OXNAS_DMA_EOT_FINAL,   // To enable checksum accumulation
    .write_eot_policy_     = OXNAS_DMA_EOT_NONE,
    .bus_                  = OXNAS_DMA_SIDE_A,      // Checksumming happens on read from side A only
    .width_                = OXNAS_DMA_TRANSFER_WIDTH_32BITS,
    .transfer_mode_        = OXNAS_DMA_TRANSFER_MODE_BURST,
    .address_mode_         = OXNAS_DMA_MODE_FIXED,
    .address_really_fixed_ = 1
};

/* For use in all occasions not covered by oxnas_ram_only_src_dma_settings and
 * oxnas_ram_csum_src_dma_settings */
oxnas_dma_device_settings_t oxnas_ram_generic_dma_settings = {
    .address_              = 0,
    .fifo_size_            = 0,
    .dreq_                 = OXNAS_DMA_DREQ_MEMORY,
    .read_eot_policy_      = OXNAS_DMA_EOT_NONE,    // Don't interfere with any checksumming transfers
    .write_eot_policy_     = OXNAS_DMA_EOT_NONE,
    .bus_                  = OXNAS_DMA_SIDE_B,
    .width_                = OXNAS_DMA_TRANSFER_WIDTH_32BITS,
    .transfer_mode_        = OXNAS_DMA_TRANSFER_MODE_BURST,
    .address_mode_         = OXNAS_DMA_MODE_FIXED,
    .address_really_fixed_ = 1
};

static oxnas_dma_controller_t  dma_controller;

#ifdef CONFIG_DESCRIPTORS_IN_SRAM
/**
 * Acquisition of a SG DMA descriptor list entry
 * If called from non-atomic context the call could block.
 */
static oxnas_dma_sg_entry_t* alloc_sg_entry(void)
{
    oxnas_dma_sg_entry_t* entry = 0;
    if (in_atomic()) {
        if (down_trylock(&dma_controller.sg_entry_sem_)) {
            return (oxnas_dma_sg_entry_t*)0;
        }
    } else {
        // Wait for an entry to be available
        while (down_interruptible(&dma_controller.sg_entry_sem_));
    }

    // Serialise while manipulating free list
    spin_lock_bh(&dma_controller.alloc_spinlock_);

    // It's an error if there isn't a buffer available at this point
    BUG_ON(!dma_controller.sg_entry_head_);

    // Unlink the head entry on the free list and return it to caller
    entry = dma_controller.sg_entry_head_;
    dma_controller.sg_entry_head_ = dma_controller.sg_entry_head_->next_;
    --dma_controller.sg_entry_available_;

    // Finished manipulating free list
    spin_unlock_bh(&dma_controller.alloc_spinlock_);

    return entry;
}

static void free_sg_entry(oxnas_dma_sg_entry_t* entry)
{
    // Serialise while manipulating free list
    spin_lock(&dma_controller.alloc_spinlock_);

    // Insert the freed buffer at the head of the free list
    entry->next_ = dma_controller.sg_entry_head_;
    dma_controller.sg_entry_head_ = entry;
    ++dma_controller.sg_entry_available_;

    // Finished manipulating free list
    spin_unlock(&dma_controller.alloc_spinlock_);

    // Make freed buffer available for allocation
    up(&dma_controller.sg_entry_sem_);
}

/**
 * This implementation is not the most efficient, as it could result in alot
 * of alloc's only to decide to free them all as not sufficient available, but
 * in practice we would hope there will not be much contention for entries
 */
//static int alloc_sg_entries(
//    oxnas_dma_sg_entry_t** entries,
//    unsigned required)
//{
//    // Attempt to acquire all entries required
//    unsigned acquired = 0;
//    while (acquired < required) {
//        entries[acquired] = alloc_sg_entry();
//        if (!entries[acquired]) {
//            // Did not acquire the entry
//            break;
//        }
//        ++acquired;
//    }
//
//    // Did we acquire all the entries required
//    if (acquired < required) {
//        // No, so free the ones we did get
//        unsigned i;
//        for (i=0; i < acquired; ++i) {
//            free_sg_entry(entries[i]);
//            entries[i] = 0;
//        }
//        return 1;
//    }
//
//    return 0;
//}
//
//static void free_sg_entries(
//    oxnas_dma_sg_entry_t** entries,
//    unsigned count)
//{
//    unsigned i;
//    for (i=0; (i < count) && entries[i]; ++i) {
//        free_sg_entry(entries[i]);
//    }
//}
#endif // CONFIG_DESCRIPTORS_IN_SRAM

#ifdef OXNAS_DMA_HAVE_HW_SG
/**
 * Acquisition of a SG DMA controller instance. Very simple now that we have
 * one SG controller per DMA channel, with each SG controller hardwired to use
 * the associated DMA channel
 */
static oxnas_dma_sg_controller_t* alloc_sg_controller(oxnas_dma_channel_t* channel)
{
    return &dma_controller.sg_controllers_[channel->channel_number_];
}

static void free_sg_controller(oxnas_dma_sg_controller_t* sg_controller)
{
}
#endif // OXNAS_DMA_HAVE_HW_SG

/**
 * Blocking acquisition of the checksum engine
 */
static inline int alloc_csum_engine(void)
{
#ifdef CONFIG_LEON_RESERVE_DMA_CHANNEL
    // Checksum engine is allocated exclusively to the CoPro
    return 1;
#else
    if (in_atomic()) {
        return down_trylock(&dma_controller.csum_engine_sem_);
    } else {
        while (1) {
            if (!down_interruptible(&dma_controller.csum_engine_sem_)) {
                return 0;
            }
        }
    }
#endif // CONFIG_LEON_RESERVE_DMA_CHANNEL
}

static inline void free_csum_engine(void)
{
    up(&dma_controller.csum_engine_sem_);
}

/**
 * Optionally blocking acquisition of a DMA channel
 */
oxnas_dma_channel_t* oxnas_dma_request(int block)
{
    oxnas_dma_channel_t* channel = OXNAS_DMA_CHANNEL_NUL;
    while (channel == OXNAS_DMA_CHANNEL_NUL) {
        if (block) {
            // Wait for a channel to be available
            if (down_interruptible(&dma_controller.channel_sem_)) {
                // Awoken by signal
                continue;
            }
        } else {
            // Non-blocking test of whether a channel is available
            if (down_trylock(&dma_controller.channel_sem_)) {
                // No channel available so return to user immediately
                break;
            }
        }

        // Serialise while manipulating free list
        spin_lock_bh(&dma_controller.alloc_spinlock_);

        // It's an error if there isn't a channel available at this point
        BUG_ON(!dma_controller.channel_head_);

        // Unlink the head entry on the free list and return it to caller
        channel = dma_controller.channel_head_;
        dma_controller.channel_head_ = dma_controller.channel_head_->next_;

        // Finished manipulating free list
        spin_unlock_bh(&dma_controller.alloc_spinlock_);
    }
    return channel;
}

void oxnas_dma_free(oxnas_dma_channel_t* channel)
{
    if (oxnas_dma_is_active(channel)) {
        printk(KERN_WARNING "oxnas_dma_free() Freeing channel %u while active\n", channel->channel_number_);
    }

    // Serialise while manipulating free list
    spin_lock_bh(&dma_controller.alloc_spinlock_);

    // Insert the freed buffer at the head of the free list
    channel->next_ = dma_controller.channel_head_;
    dma_controller.channel_head_ = channel;

    // Finished manipulating free list
    spin_unlock_bh(&dma_controller.alloc_spinlock_);

    // Make freed buffer available for allocation
    up(&dma_controller.channel_sem_);
}

/**
 * Will be executed with interrupts disabled, so can share between all DMA
 * interrupt lines - so make it quick
 */
static irqreturn_t oxnas_dma_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
    unsigned i;
    u32 run_bh = 0;
#ifdef OXNAS_DMA_HAVE_HW_SG
    u32 channels_to_service = 0;
#endif // OXNAS_DMA_HAVE_HW_SG
#ifdef OXNAS_DMA_NON_FILTERED_SG_INTS
    oxnas_dma_channel_t *interrupting_channel = 0;
    unsigned             interrupting_channel_number = 0;
#else // OXNAS_DMA_NON_FILTERED_SG_INTS
    unsigned intId;
#endif // OXNAS_DMA_NON_FILTERED_SG_INTS

DBG("oxnas_dma_interrupt() from interrupt line %u\n", irq);

#ifdef OXNAS_DMA_NON_FILTERED_SG_INTS
    // Only acknowledge interrupts from the channel directly responsible for the
    // RPS interrupt line which caused the ISR to be entered, to get around the
    // problem that the SG-DMA controller can only filter DMA interrupts exter-
    // nally to the DMA controller, i.e. the DMA controller interrupt status
    // register always shows all active interrupts for all channels, regardless
    // of whether the SG-DMA controller is filtering them

    // Find the DMA channel that can generate interrupts on the RPS interrupt
    // line which caused the ISR to be invoked.
    for (i=0; i < dma_controller.numberOfChannels_; ++i) {
        // Is the RPS interrupt from this channel?
        if (dma_controller.channels_[i].rps_interrupt_ == irq) {
            interrupting_channel        = &dma_controller.channels_[i];
            interrupting_channel_number = interrupting_channel->channel_number_;
DBG("RPS interrupt %u from channel %u\n", irq, interrupting_channel_number);
            break;
        }
    }

    // Must find an associated DMA channel
    BUG_ON(!interrupting_channel);

    // Non-SG transfers have no completion status, so initialise
    // channel's error code to no-error. If transfer turns out to
    // have been SG, this status will be overwritten
    interrupting_channel->error_code_ = OXNAS_DMA_ERROR_CODE_NONE;

    // Mark this channel as requiring its bottom half to run
    run_bh |= (1UL << interrupting_channel_number);
DBG("Set bh service flag for channel %u -> run_bh = 0x%08x\n", interrupting_channel_number, run_bh);

    // Cope with the DMA controller's ability to have a pair of chained
    // transfers which have both completed, which causes the interrupt request
    // to stay active until both have been acknowledged, which is causing the SG
    // controller problems
    while (readl(DMA_CALC_REG_ADR(0, DMA_INTR_ID)) & (1 << interrupting_channel_number)) {
DBG("Ack'ing interrupt for channel %u\n", interrupting_channel_number);
        // Write to the interrupt clear register to clear interrupt
        writel(0, DMA_CALC_REG_ADR(interrupting_channel_number, DMA_INTR_CLEAR_REG));

        // Record how many interrupts are awaiting service
        atomic_inc(&interrupting_channel->interrupt_count_);
    }
DBG("Left int ack'ing loop\n");

#else // OXNAS_DMA_NON_FILTERED_SG_INTS

    // Acknowledge all interrupting channels, coping with the DMA controllers
    // ability to have a pair of chained transfers which have both completed,
    // which causes the interrupt request to stay active until both have been
    // acknowledged, which is causing the SG controller problems
    while ((intId = (readl(DMA_CALC_REG_ADR(0, DMA_INTR_ID)) & DMA_INTR_ID_INT_MASK))) {
DBG("intId = 0x%08x\n", intId);
        for (i=0; i < dma_controller.numberOfChannels_; i++, intId >>= 1) {
            if (intId & 0x1UL) {
                oxnas_dma_channel_t* channel = &dma_controller.channels_[i];

                // Non-SG transfers have no completion status, so initialise
                // channel's error code to no-error. If transfer turns out to
                // have been SG, this status will be overwritten
                channel->error_code_ = OXNAS_DMA_ERROR_CODE_NONE;

                // Write to the intr clear register to clear intr
                writel(0, DMA_CALC_REG_ADR(i, DMA_INTR_CLEAR_REG));

                // Mark this channel as requiring its bottom half to run
DBG("Setting bh service flag for channel %u\n", channel->channel_number_);
                run_bh |= (1UL << i);

                // Record how many interrupts are awaiting service
                atomic_inc(&channel->interrupt_count_);
            }
        }
    }
DBG("Left int clearing loop, run_bh=0x%08x\n", run_bh);
#endif // OXNAS_DMA_NON_FILTERED_SG_INTS

#ifdef OXNAS_DMA_HAVE_HW_SG
    // Deal with all SG channels that were found to be interrupting. When it's
    // working correctly, we should only ever see a single interrupt from a SG
    // channel no matter how many segments the SG transfer is composed of
    channels_to_service = run_bh;
    for (i=0; i < dma_controller.numberOfChannels_; i++, channels_to_service >>= 1) {
        if (channels_to_service & 0x1UL) {
            oxnas_dma_channel_t* channel = &dma_controller.channels_[i];

            // If was a SG transfer, record the completion status
            if (channel->v_sg_info_->v_srcEntries_) {
                u32 error_code;
                unsigned sg_channel_number = channel->sg_controller_->number_;

                // Thread-level code can poll SG status register at any time
                // for the busy status
                spin_lock(&dma_controller.sg_status_spinlock_);

                // Record the SG transfer completion status
                error_code = readl(DMA_SG_CALC_REG_ADR(sg_channel_number, DMA_SG_STATUS));
                channel->error_code_ =
                    ((error_code >> DMA_SG_STATUS_ERROR_CODE_BIT) &
                     ((1UL << DMA_SG_STATUS_ERROR_CODE_NUM_BITS) - 1));

DBG("Return SG controller to idle, error_code = 0x%08x\n", error_code);
                // Return the SG DMA controller to the IDLE state and clear
                // any SG controller error interrupt
                writel(1, DMA_SG_CALC_REG_ADR(sg_channel_number, DMA_SG_STATUS));

                // Disassociate the SG controller from the DMA channel
                channel->sg_controller_->channel_ = 0;

                spin_unlock(&dma_controller.sg_status_spinlock_);
            }
        }
    }
#endif // OXNAS_DMA_HAVE_HW_SG

#ifdef OXNAS_DMA_HAVE_HW_SG
    // If didn't find a source for the interrupt, assume it was a SG controller
    // doing something strange
    if (unlikely(!run_bh)) {
DBG("oxnas_dma_interrupt() Spurious interrupt %u, attempting to clear via SG controllers\n", irq);

        // Find the DMA channel / SG combination that can generate interrupts on
        // the RPS interrupt line which caused the ISR to be invoked.
        for (i=0; i < dma_controller.numberOfChannels_; ++i) {
            oxnas_dma_channel_t *channel = &dma_controller.channels_[i];

            // Is the RPS interrupt from this channel?
            if (channel->rps_interrupt_ == irq) {
                u32 error_code;

                // Find the SG-controller with the hardwired association to the
                // DMA channel
                oxnas_dma_sg_controller_t *sg_controller = &dma_controller.sg_controllers_[i];

                // Do we expect the SG controller to currently be using the DMA
                // channel?
                if (!sg_controller->channel_) {
DBG("oxnas_dma_interrupt() SG channel %d NOT associated with a DMA channel\n", sg_controller->number_);
                } else if (sg_controller->number_ != sg_controller->channel_->channel_number_) {
DBG("oxnas_dma_interrupt() SG channel %d associated with WRONG DMA channel %d\n", i, channel->channel_number_);
                } else {
DBG("oxnas_dma_interrupt() SG channel %d associated with DMA channel %d\n", i, channel->channel_number_);
                    // Disassociate the SG controller from the DMA channel
                    sg_controller->channel_ = 0;
                }

                // Thread-level code can poll SG status register at any time
                // for the busy status
                spin_lock(&dma_controller.sg_status_spinlock_);

                error_code = ((readl(DMA_SG_CALC_REG_ADR(i, DMA_SG_STATUS)) >>
                                    DMA_SG_STATUS_ERROR_CODE_BIT) &
                                ((1UL << DMA_SG_STATUS_ERROR_CODE_NUM_BITS) - 1));
DBG("oxnas_dma_interrupt() SG channel %d has error code  = 0x%08x\n", i, error_code);

                // Return the SG DMA controller to the IDLE state and clear any
                // SG controller error interrupt
                writel(1, DMA_SG_CALC_REG_ADR(i, DMA_SG_STATUS));

                spin_unlock(&dma_controller.sg_status_spinlock_);

                // Store the error code with the channel with the hardwired
                // association to the SG controller
                channel->error_code_ = error_code;

                // Mark this channel as requiring its bottom half to run
                run_bh |= (1UL << channel->channel_number_);

                // Have ack'ed the interrupt from the SG controller responsible
                // for the apparently spurious interrupt on the RPS interrupt
                // line which caused the ISR to be entered
                break;
            }
        }
    }
#endif // OXNAS_DMA_HAVE_HW_SG

    if (likely(run_bh)) {
DBG("Setting flag for channels which interrupted\n");
        // Set a flag for each channel which interrupted
        // Should not find active channel again until bottom halve has been serviced,
        // so any set bits in run_bh should not also be set in dma_controller.run_bh_,
        // thus add should be safe
        atomic_add(run_bh, &dma_controller.run_bh_);

DBG("Scheduling tasklet\n");
        // Signal the bottom half to perform the notifications
        tasklet_schedule(&dma_controller.tasklet_);
    }

DBG("Returning\n");
    return IRQ_HANDLED;
}

static void fake_interrupt(int channel)
{
    // Set a flag to cause the bottom half handler to be run for the channel
    set_bit(channel, (void*)&dma_controller.run_bh_);

    // Signal the bottom half to perform the notifications
    tasklet_schedule(&dma_controller.tasklet_);
}

static void dma_bh(unsigned long data)
{
    // Check for any bottom halves having become ready to run
    u32 run_bh = atomic_read(&dma_controller.run_bh_);
    while (run_bh) {
        unsigned i=0;
        for (; i < dma_controller.numberOfChannels_; i++) {
            if (run_bh & (1UL << i)) {
                oxnas_dma_channel_t* channel = &dma_controller.channels_[i];
                oxnas_dma_sg_entry_t* sg_entry = channel->v_sg_info_->v_srcEntries_;
DBG("Bottom halve for channel %u\n", channel->channel_number_);

                // If this channel computed a checksum
                if (channel->checksumming_) {
                    // Read the result of the checksum calculation, clearing the
                    // result in the process
                    channel->checksum_ = readl(DMA_CHECKSUM_BASE);
                    channel->checksumming_ = 0;

                    // Relinquish ownership of the checksum engine
                    free_csum_engine();
                }

#ifdef OXNAS_DMA_HAVE_HW_SG
                // Was the transfer a SG one?
                if (sg_entry) {
                    // Make the SG DMA controller instance available for reuse
                    free_sg_controller(channel->sg_controller_);
                }
#endif // OXNAS_DMA_HAVE_HW_SG

                // Free SG DMA source descriptor resources
                while (sg_entry) {
                    oxnas_dma_sg_entry_t* next = sg_entry->v_next_;
#ifdef CONFIG_DESCRIPTORS_IN_SRAM
                    free_sg_entry(sg_entry);
#else // CONFIG_DESCRIPTORS_IN_SRAM
                    dma_pool_free(dma_controller.desc_dma_pool, sg_entry, sg_entry->paddr_);
#endif // CONFIG_DESCRIPTORS_IN_SRAM
                    sg_entry = next;
                }
                channel->v_sg_info_->p_srcEntries_ = 0;
                channel->v_sg_info_->v_srcEntries_ = 0;

                // Free SG DMA destination descriptor resources
                sg_entry = channel->v_sg_info_->v_dstEntries_;
                while (sg_entry) {
                    oxnas_dma_sg_entry_t* next = sg_entry->v_next_;
#ifdef CONFIG_DESCRIPTORS_IN_SRAM
                    free_sg_entry(sg_entry);
#else // CONFIG_DESCRIPTORS_IN_SRAM
                    dma_pool_free(dma_controller.desc_dma_pool, sg_entry, sg_entry->paddr_);
#endif // CONFIG_DESCRIPTORS_IN_SRAM
                    sg_entry = next;
                }
                channel->v_sg_info_->p_dstEntries_ = 0;
                channel->v_sg_info_->v_dstEntries_ = 0;
            }
        }

        // Mark that we have serviced the bottom halves. None of the channels
        // we have just serviced can interrupt again until their active flags
        // are cleared below
        atomic_sub(run_bh, &dma_controller.run_bh_);

        // Notify all listeners of transfer completion
        for (i = 0; i < dma_controller.numberOfChannels_; i++) {
            if (run_bh & (1UL << i)) {
                int interrupt_count;
                oxnas_dma_channel_t* channel = &dma_controller.channels_[i];

                // Clear the count of received interrupts for the channel now
                // that we have serviced them all
                interrupt_count = atomic_read(&channel->interrupt_count_);
                atomic_sub(interrupt_count, &channel->interrupt_count_);

                // Decrement the count of active transfers, by the number of
                // interrupts we've seen. This must occur before we inform any
                // listeners who are awaiting completion notification. Should
                // only decrement if greater than zero, in case we see spurious
                // interrupt events - we can't be fully safe against this sort
                // of broken h/w, but we can at least stop the count underflowing
                // active_count_ is only shared with thread level code, so read
                // and decrement don't need to be atomic
                if (atomic_read(&channel->active_count_)) {
                    atomic_dec(&channel->active_count_);
                }

                // If there is a callback registered, notify the user that the
                // transfer is complete
                if (channel->notification_callback_ != OXNAS_DMA_CALLBACK_NUL) {
DBG("Notifying channel %u, %d outstanding interrupts\n", channel->channel_number_, interrupt_count);
                    (*channel->notification_callback_)(
                        &dma_controller.channels_[i],
                        channel->notification_arg_,
                        channel->error_code_,
                        channel->checksum_,
                        interrupt_count);
                }
            }
        }

        // Check for any more bottom halves having become ready to run
        run_bh = atomic_read(&dma_controller.run_bh_);
    }
}

void __init oxnas_dma_init()
{
    unsigned i;
    unsigned long intId;
    oxnas_dma_sg_info_t *v_info;
    dma_addr_t           p_info;
#ifdef OXNAS_DMA_HAVE_HW_SG
    oxnas_dma_sg_controller_t* sg_controller;
#endif // OXNAS_DMA_HAVE_HW_SG

    // Ensure the DMA block is properly reset
    writel(1UL << SYS_CTRL_RSTEN_DMA_BIT, SYS_CTRL_RSTEN_SET_CTRL);
    writel(1UL << SYS_CTRL_RSTEN_DMA_BIT, SYS_CTRL_RSTEN_CLR_CTRL);

    // Ensure the SG-DMA block is properly reset
    writel(1UL << SYS_CTRL_RSTEN_SGDMA_BIT, SYS_CTRL_RSTEN_SET_CTRL);
    writel(1UL << SYS_CTRL_RSTEN_SGDMA_BIT, SYS_CTRL_RSTEN_CLR_CTRL);

    // Enable the clock to the DMA block
    writel(1UL << SYS_CTRL_CKEN_DMA_BIT, SYS_CTRL_CKEN_SET_CTRL);

    // Initialise the DMA controller
    atomic_set(&dma_controller.run_bh_, 0);
    spin_lock_init(&dma_controller.spinlock_);
    spin_lock_init(&dma_controller.alloc_spinlock_);
    sema_init(&dma_controller.csum_engine_sem_, 1);

    // Initialise channel allocation management
    dma_controller.channel_head_ = 0;
    sema_init(&dma_controller.channel_sem_, 0);
#ifdef CONFIG_DESCRIPTORS_IN_SRAM
    // Initialise SRAM buffer management
    dma_controller.sg_entry_head_ = 0;
    sema_init(&dma_controller.sg_entry_sem_, 0);
    dma_controller.sg_entry_available_ = 0;
#endif // CONFIG_DESCRIPTORS_IN_SRAM
#ifdef OXNAS_DMA_HAVE_HW_SG
    spin_lock_init(&dma_controller.sg_status_spinlock_);

    // Add all available SG controller instances to the free list
    sg_controller = dma_controller.sg_controllers_;
    for (i=0; i < OXNAS_DMA_NUM_SG_CONTROLLERS; ++i, ++sg_controller) {
        sg_controller->number_ = i;
        sg_controller->channel_ = 0;
        free_sg_controller(sg_controller);
    }
#endif // OXNAS_DMA_HAVE_HW_SG

    tasklet_init(&dma_controller.tasklet_, dma_bh, 0);

    // Discover the number of channels available
    intId = readl(DMA_CALC_REG_ADR(0, DMA_INTR_ID));
    dma_controller.numberOfChannels_ = DMA_INTR_ID_GET_NUM_CHANNELS(intId);
    if (dma_controller.numberOfChannels_ > MAX_OXNAS_DMA_CHANNELS) {
        printk(KERN_WARNING "DMA: Too many DMA channels");
        dma_controller.numberOfChannels_ = MAX_OXNAS_DMA_CHANNELS;
    }

    dma_controller.version_ = DMA_INTR_ID_GET_VERSION(intId);
    printk(KERN_INFO "Number of DMA channels = %u, version = %u\n",
        dma_controller.numberOfChannels_, dma_controller.version_);

    if (!DMA_HAS_V4_INTR_CLEAR(dma_controller.version_)) {
        panic("DMA: Trying to use v4+ interrupt clearing on DMAC version without support\n");
    }

#ifdef CONFIG_LEON_RESERVE_DMA_CHANNEL
    // Reserve the last DMA channel for the CoPro's use
    --dma_controller.numberOfChannels_;
#endif // CONFIG_LEON_RESERVE_DMA_CHANNEL

    // Allocate coherent memory for array sg_info structs
#ifdef CONFIG_DESCRIPTORS_IN_SRAM
    dma_controller.v_sg_infos_ = (oxnas_dma_sg_info_t*)DMA_DESC_ALLOC_START;
    dma_controller.p_sg_infos_ = DMA_DESC_ALLOC_START_PA;
#else // CONFIG_DESCRIPTORS_IN_SRAM
    dma_controller.v_sg_infos_ = dma_alloc_coherent(0,
        sizeof(oxnas_dma_sg_info_t) * dma_controller.numberOfChannels_,
        &dma_controller.p_sg_infos_, GFP_KERNEL | GFP_DMA);
#endif // CONFIG_DESCRIPTORS_IN_SRAM

    if (!dma_controller.v_sg_infos_) {
        panic("DMA: Coherent alloc of SG info struct array");
    }

#ifdef CONFIG_DESCRIPTORS_IN_SRAM
    {
        unsigned long sg_info_alloc_size = (dma_controller.numberOfChannels_ * sizeof(oxnas_dma_sg_info_t));
        unsigned num_sg_entries = (DMA_DESC_ALLOC_SIZE - sg_info_alloc_size) / sizeof(oxnas_dma_sg_entry_t);
        oxnas_dma_sg_entry_t* entry_v = (oxnas_dma_sg_entry_t*)(DMA_DESC_ALLOC_START + sg_info_alloc_size);
        oxnas_dma_sg_entry_t* entry_p = (oxnas_dma_sg_entry_t*)(DMA_DESC_ALLOC_START_PA + sg_info_alloc_size);
printk("Allocating %u SRAM generic DMA descriptors\n", num_sg_entries);
        for (i=0; i < num_sg_entries; ++i, ++entry_v, ++entry_p) {
            entry_v->paddr_ = (dma_addr_t)entry_p;
            free_sg_entry(entry_v);
        }
    }
#else // CONFIG_DESCRIPTORS_IN_SRAM
    // Create a coherent pool for SG list entries
    dma_controller.desc_dma_pool = dma_pool_create("dma_descriptors", 0, sizeof(oxnas_dma_sg_entry_t), 0, 0);
    if (!dma_controller.desc_dma_pool) {
        panic("DMA: Cannot create descriptor DMA pool");
    }
#endif // CONFIG_DESCRIPTORS_IN_SRAM

    // Initialise all available DMA channels
    v_info = dma_controller.v_sg_infos_;
    p_info = dma_controller.p_sg_infos_;
    for (i=0; i < dma_controller.numberOfChannels_; i++) {
        oxnas_dma_channel_t *channel = &dma_controller.channels_[i];

        channel->channel_number_ = i;
        channel->notification_callback_ = OXNAS_DMA_CALLBACK_NUL;
        channel->notification_arg_ = OXNAS_DMA_CALLBACK_ARG_NUL;

        // Setup physical and virtual addresses of the SG info struct for this
        // channel
        channel->v_sg_info_ = v_info++;
        channel->p_sg_info_ = p_info;
        p_info += sizeof(oxnas_dma_sg_info_t);

        // Initialise heads of src and dst SG lists to null
        channel->v_sg_info_->p_srcEntries_ = 0;
        channel->v_sg_info_->p_dstEntries_ = 0;
        channel->v_sg_info_->v_srcEntries_ = 0;
        channel->v_sg_info_->v_dstEntries_ = 0;

        channel->error_code_ = 0;
        channel->checksumming_ = 0;
        channel->checksum_ = 0;
#ifdef OXNAS_DMA_HAVE_HW_SG
        channel->sg_controller_ = 0;
#endif // OXNAS_DMA_HAVE_HW_SG

        // Initialise the atomic variable that records the number of interrupts
        // for the channel that are awaiting service
        atomic_set(&channel->interrupt_count_, 0);

        // Initialise the atomic variable maintaining the count of in-progress
        // transfers for the channel. Currently can be a maximum of two, as
        // the hardware can only queue details for a pair of transfers
        atomic_set(&channel->active_count_, 0);

        // The binary semaphore for the default callback used when abort
        // requested without a user-registered callback being available
        sema_init(&channel->default_semaphore_, 0);

        // Add channel to free list
        oxnas_dma_free(channel);
    }

    // Connect the dma interrupt handler
    dma_controller.channels_[0].rps_interrupt_ = DMA_INTERRUPT_0;
    if (request_irq(DMA_INTERRUPT_0, &oxnas_dma_interrupt, SA_INTERRUPT, "DMA 0", 0)) {
        panic("DMA: Failed to allocate interrupt %u\n", DMA_INTERRUPT_0);
    }
    dma_controller.channels_[1].rps_interrupt_ = DMA_INTERRUPT_1;
    if (request_irq(DMA_INTERRUPT_1, &oxnas_dma_interrupt, SA_INTERRUPT, "DMA 1", 0)) {
        panic("DMA: Failed to allocate interrupt %u\n", DMA_INTERRUPT_1);
    }
    dma_controller.channels_[2].rps_interrupt_ = DMA_INTERRUPT_2;
    if (request_irq(DMA_INTERRUPT_2, &oxnas_dma_interrupt, SA_INTERRUPT, "DMA 2", 0)) {
        panic("DMA: Failed to allocate interrupt %u\n", DMA_INTERRUPT_2);
    }
    dma_controller.channels_[3].rps_interrupt_ = DMA_INTERRUPT_3;
    if (request_irq(DMA_INTERRUPT_3, &oxnas_dma_interrupt, SA_INTERRUPT, "DMA 3", 0)) {
        panic("DMA: Failed to allocate interrupt %u\n", DMA_INTERRUPT_3);
    }
#ifndef CONFIG_LEON_RESERVE_DMA_CHANNEL
    dma_controller.channels_[4].rps_interrupt_ = DMA_INTERRUPT_4;
    if (request_irq(DMA_INTERRUPT_4, &oxnas_dma_interrupt, SA_INTERRUPT, "DMA 4", 0)) {
        panic("DMA: Failed to allocate interrupt %u\n", DMA_INTERRUPT_4);
    }
#endif // CONFIG_LEON_RESERVE_DMA_CHANNEL

#ifdef OXNAS_DMA_OVERALL_TEST_LOOPS
    {
        int j;
        for (j=0; j < OXNAS_DMA_OVERALL_TEST_LOOPS; ++j) {
#ifdef OXNAS_DMA_TEST
            {
                int i;
                for (i=0; i < OXNAS_DMA_TEST_ITERATIONS; ++i) {
                    dma_test(512);
                }
            }
#endif // OXNAS_DMA_TEST
#ifdef OXNAS_DMA_SG_TEST
            {
                int i;
                for (i=0; i < OXNAS_DMA_SG_TEST_ITERATIONS; ++i) {
                    dma_sg_test();
                }
            }
#endif // OXNAS_DMA_SG_TEST
#ifdef OXNAS_DMA_SG_TEST_2
            {
                int i;
                for (i=0; i < OXNAS_DMA_SG_TEST_ITERATIONS; ++i) {
                    dma_sg_test2();
                }
            }
#endif // OXNAS_DMA_SG_TEST_2
#ifdef OXNAS_DMA_TEST
            {
                int i;
                for (i=0; i < OXNAS_DMA_TEST_AFTER_SG_ITERATIONS; ++i) {
                    dma_test(512);
                }
            }
#endif // OXNAS_DMA_TEST
        }
    }
#endif // OXNAS_DMA_OVERALL_TEST_LOOPS
}

void oxnas_dma_shutdown()
{
#ifdef CONFIG_DESCRIPTORS_IN_SRAM
    dma_controller.sg_entry_head_ = 0;
#else // CONFIG_DESCRIPTORS_IN_SRAM
    if (dma_controller.desc_dma_pool) {
        dma_pool_destroy(dma_controller.desc_dma_pool);
        dma_controller.desc_dma_pool = 0;
    }
#endif // CONFIG_DESCRIPTORS_IN_SRAM

    if (dma_controller.v_sg_infos_) {
#ifndef CONFIG_DESCRIPTORS_IN_SRAM
        dma_free_coherent(0,
                sizeof(oxnas_dma_sg_info_t) * dma_controller.numberOfChannels_,
                dma_controller.v_sg_infos_, dma_controller.p_sg_infos_);
#endif // CONFIG_DESCRIPTORS_IN_SRAM
        dma_controller.v_sg_infos_ = 0;
    }
}

int oxnas_dma_is_active(oxnas_dma_channel_t* channel)
{
    return atomic_read(&channel->active_count_);
}

/**
 * Get the transfer status directly from the hardware, so for instance the
 * end of a transfer can be polled for within interrupt context.
 *
 * NB If this function indicates the channel is inactive, it does NOT imply that
 * it can be reused. Reuse is only possible when oxnas_dma_is_active() returns
 * the inactive state
 */
int oxnas_dma_raw_isactive(oxnas_dma_channel_t* channel)
{
    unsigned long ctrl_status;

    spin_lock(&dma_controller.spinlock_);
    ctrl_status = readl(DMA_CALC_REG_ADR(channel->channel_number_, DMA_CTRL_STATUS));
    spin_unlock(&dma_controller.spinlock_);

    return ctrl_status & DMA_CTRL_STATUS_IN_PROGRESS;
}

/**
 * Get the SG transfer status directly from the hardware, so for instance the
 * end of a SG transfer can be polled for within interrupt context.
 *
 * NB If this function indicates the channel is inactive, it does NOT imply that
 * it can be reused. Reuse is only possible when oxnas_dma_is_active() returns
 * the inactive state
 */
int oxnas_dma_raw_sg_isactive(oxnas_dma_channel_t* channel)
{
#ifdef OXNAS_DMA_HAVE_HW_SG
    u32 status;
    unsigned long irq_saved_state;

    // ISR accesses SG status register to get error code and to return SG
    // controller to IDLE
    spin_lock_irqsave(&dma_controller.sg_status_spinlock_, irq_saved_state);

    // Record the SG channel status
    status = readl(DMA_SG_CALC_REG_ADR(channel->sg_controller_->number_, DMA_SG_STATUS));

    spin_unlock_irqrestore(&dma_controller.sg_status_spinlock_, irq_saved_state);

    return status & (1UL << DMA_SG_STATUS_BUSY_BIT);
#else // OXNAS_DMA_HAVE_HW_SG
    return 0;
#endif // OXNAS_DMA_HAVE_HW_SG
}

int oxnas_dma_get_raw_direction(oxnas_dma_channel_t* channel)
{
    unsigned long ctrl_status;

    spin_lock(&dma_controller.spinlock_);
    ctrl_status = readl(DMA_CALC_REG_ADR(channel->channel_number_, DMA_CTRL_STATUS));
    spin_unlock(&dma_controller.spinlock_);

    return (ctrl_status & DMA_CTRL_STATUS_DIR_MASK) >> DMA_CTRL_STATUS_DIR_SHIFT;
}

static unsigned long encode_control_status(
    oxnas_dma_device_settings_t *src_settings,
    oxnas_dma_device_settings_t *dst_settings,
    int                          paused)
{
    unsigned long ctrl_status;
    oxnas_dma_transfer_direction_t direction;

    ctrl_status  = paused ? DMA_CTRL_STATUS_PAUSE : 0;                          // Paused if requested
    ctrl_status |= DMA_CTRL_STATUS_INTERRUPT_ENABLE;                            // Interrupts enabled
    ctrl_status |= DMA_CTRL_STATUS_FAIR_SHARE_ARB;                              // High priority
    ctrl_status |= (src_settings->dreq_ << DMA_CTRL_STATUS_SRC_DREQ_SHIFT);     // Dreq
    ctrl_status |= (dst_settings->dreq_ << DMA_CTRL_STATUS_DEST_DREQ_SHIFT);    // Dreq
    ctrl_status &= ~DMA_CTRL_STATUS_RESET;                                      // !RESET

#if defined(OXNAS_DMA_TEST) || defined(OXNAS_DMA_SG_TEST)
printk("Using v4+ interrupt clearing register\n");
#endif //defined(OXNAS_DMA_TEST) || defined(OXNAS_DMA_SG_TEST)
    // Use new interrupt clearing register
    ctrl_status |= DMA_CTRL_STATUS_INTR_CLEAR_ENABLE;

    // Setup the transfer direction and burst/single mode for the two DMA busses
    if (src_settings->bus_ == OXNAS_DMA_SIDE_A) {
        // Set the burst/single mode for bus A based on src device's settings
        if (src_settings->transfer_mode_ == OXNAS_DMA_TRANSFER_MODE_BURST) {
            ctrl_status |= DMA_CTRL_STATUS_TRANSFER_MODE_A;
        } else {
            ctrl_status &= ~DMA_CTRL_STATUS_TRANSFER_MODE_A;
        }

        if (dst_settings->bus_ == OXNAS_DMA_SIDE_A) {
            direction = OXNAS_DMA_A_TO_A;
        } else {
            direction = OXNAS_DMA_A_TO_B;

            // Set the burst/single mode for bus B based on dst device's settings
            if (dst_settings->transfer_mode_ == OXNAS_DMA_TRANSFER_MODE_BURST) {
                ctrl_status |= DMA_CTRL_STATUS_TRANSFER_MODE_B;
            } else {
                ctrl_status &= ~DMA_CTRL_STATUS_TRANSFER_MODE_B;
            }
        }
    } else {
        // Set the burst/single mode for bus B based on src device's settings
        if (src_settings->transfer_mode_ == OXNAS_DMA_TRANSFER_MODE_BURST) {
            ctrl_status |= DMA_CTRL_STATUS_TRANSFER_MODE_B;
        } else {
            ctrl_status &= ~DMA_CTRL_STATUS_TRANSFER_MODE_B;
        }

        if (dst_settings->bus_ == OXNAS_DMA_SIDE_A) {
            direction = OXNAS_DMA_B_TO_A;

            // Set the burst/single mode for bus A based on dst device's settings
            if (dst_settings->transfer_mode_ == OXNAS_DMA_TRANSFER_MODE_BURST) {
                ctrl_status |= DMA_CTRL_STATUS_TRANSFER_MODE_A;
            } else {
                ctrl_status &= ~DMA_CTRL_STATUS_TRANSFER_MODE_A;
            }
        } else {
            direction = OXNAS_DMA_B_TO_B;
        }
    }
    ctrl_status |= (direction << DMA_CTRL_STATUS_DIR_SHIFT);

    // Setup source address mode fixed or increment
    if (src_settings->address_mode_ == OXNAS_DMA_MODE_FIXED) {
        // Fixed address
        ctrl_status &= ~(DMA_CTRL_STATUS_SRC_ADR_MODE);

        // Set up whether fixed address is _really_ fixed
        if (src_settings->address_really_fixed_) {
            ctrl_status |= DMA_CTRL_STATUS_SOURCE_ADDRESS_FIXED;
        } else {
            ctrl_status &= ~DMA_CTRL_STATUS_SOURCE_ADDRESS_FIXED;
        }
    } else {
        // Incrementing address
        ctrl_status |= DMA_CTRL_STATUS_SRC_ADR_MODE;
        ctrl_status &= ~DMA_CTRL_STATUS_SOURCE_ADDRESS_FIXED;
    }

    // Setup destination address mode fixed or increment
    if (dst_settings->address_mode_ == OXNAS_DMA_MODE_FIXED) {
        // Fixed address
        ctrl_status &= ~(DMA_CTRL_STATUS_DEST_ADR_MODE);

        // Set up whether fixed address is _really_ fixed
        if (dst_settings->address_really_fixed_) {
            ctrl_status |= DMA_CTRL_STATUS_DESTINATION_ADDRESS_FIXED;
        } else {
            ctrl_status &= ~DMA_CTRL_STATUS_DESTINATION_ADDRESS_FIXED;
        }
    } else {
        // Incrementing address
        ctrl_status |= DMA_CTRL_STATUS_DEST_ADR_MODE;
        ctrl_status &= ~DMA_CTRL_STATUS_DESTINATION_ADDRESS_FIXED;
    }

    // Set up the width of the transfers on the DMA buses
    ctrl_status |= (src_settings->width_ << DMA_CTRL_STATUS_SRC_WIDTH_SHIFT);
    ctrl_status |= (dst_settings->width_ << DMA_CTRL_STATUS_DEST_WIDTH_SHIFT);

    // Setup the priority arbitration scheme
    ctrl_status &= ~DMA_CTRL_STATUS_STARVE_LOW_PRIORITY;    // !Starve low priority

    return ctrl_status;
}

static unsigned long encode_eot(
    oxnas_dma_device_settings_t* src_settings,
    oxnas_dma_device_settings_t* dst_settings,
    unsigned long length,
    int isFinalTransfer)
{
    // Write the length, with EOT configuration and enable INC4 tranfers and
    // HPROT. HPROT will delay data reaching memory for a few clock cycles, but
    // most unlikely to cause a problem for the CPU.
    unsigned long encoded = length |
                            DMA_BYTE_CNT_INC4_SET_MASK |    // Always enable INC4 transfers
                            DMA_BYTE_CNT_HPROT_MASK;        // Always enable HPROT assertion

    // Encode the EOT setting for the src device based on its policy
    encoded &= ~DMA_BYTE_CNT_RD_EOT_MASK;
    switch (src_settings->read_eot_policy_) {
        case OXNAS_DMA_EOT_FINAL:
            if (!isFinalTransfer) {
                break;
            }
            // Fall through in case of final transfer and EOT required for final
            // transfer
        case OXNAS_DMA_EOT_ALL:
            encoded |= DMA_BYTE_CNT_RD_EOT_MASK;
            break;
        default:
            break;
    }

    // Encode the EOT setting for the dst device based on its policy
    encoded &= ~DMA_BYTE_CNT_WR_EOT_MASK;
    switch (dst_settings->write_eot_policy_) {
        case OXNAS_DMA_EOT_FINAL:
            if (!isFinalTransfer) {
                break;
            }
            // Fall through in case of final transfer and EOT required for final
            // transfer
        case OXNAS_DMA_EOT_ALL:
            encoded |= DMA_BYTE_CNT_WR_EOT_MASK;
            break;
        default:
            break;
    }

    return encoded;
}

static unsigned long encode_start(unsigned long ctrl_status)
{
    ctrl_status &= ~DMA_CTRL_STATUS_PAUSE;
    return ctrl_status;
}

static void oxnas_dma_set_common_lowlevel(
    oxnas_dma_channel_t *channel,
    unsigned long        ctrl_status,
    dma_addr_t           src_address,
    dma_addr_t           dst_address,
    unsigned long        lengthAndEOT)
{
    unsigned channel_number = channel->channel_number_;

    spin_lock(&dma_controller.spinlock_);

    // Write the control/status value to the DMAC
    writel(ctrl_status, DMA_CALC_REG_ADR(channel_number, DMA_CTRL_STATUS));

    // Ensure control/status word makes it to the DMAC before we write address/length info
    wmb();

    // Write the source addresses to the DMAC
    writel(src_address, DMA_CALC_REG_ADR(channel_number, DMA_BASE_SRC_ADR));

    // Write the destination addresses to the DMAC
    writel(dst_address, DMA_CALC_REG_ADR(channel_number, DMA_BASE_DST_ADR));

    // Write the length, with EOT configuration for the single transfer
    writel(lengthAndEOT, DMA_CALC_REG_ADR(channel_number, DMA_BYTE_CNT));

    // Ensure adr/len info makes it to DMAC before later modifications to
    // control/status register due to starting the transfer, which happens in
    // oxnas_dma_start()
    wmb();

    spin_unlock(&dma_controller.spinlock_);

    // Increase count of in-progress transfers on this channel
    atomic_inc(&channel->active_count_);
}

static int oxnas_dma_set_common(
    oxnas_dma_channel_t*         channel,
    unsigned long                length,
    oxnas_dma_device_settings_t *src_settings,
    oxnas_dma_device_settings_t *dst_settings,
    int                          isFinalTransfer,
    int                          paused)
{
    int status = 0;

    if (length > MAX_OXNAS_DMA_TRANSGER_LENGTH) {
        printk(KERN_WARNING "oxnas_dma_set_common() length exceeds hardware allowed maximum\n");
        status = 1;
    } else {
        oxnas_dma_set_common_lowlevel(
            channel,
            encode_control_status(src_settings, dst_settings, paused),
            (dma_addr_t)src_settings->address_,
            (dma_addr_t)dst_settings->address_,
            encode_eot(src_settings, dst_settings, length, isFinalTransfer));
    }
    return status;
}

int oxnas_dma_set(
    oxnas_dma_channel_t *channel,
    unsigned char       *src_adr,   // Physical address
    unsigned long        length,
    unsigned char       *dst_adr,   // Physical address
    oxnas_dma_mode_t     src_mode,
    oxnas_dma_mode_t     dst_mode,
    int                  do_checksum,
    int                  paused)
{
    if (oxnas_dma_is_active(channel)) {
        printk(KERN_WARNING "oxnas_dma_set() Trying to use channel %u while active\n", channel->channel_number_);
    }

    if (do_checksum) {
        // Arbitrate for ownership of the checksum engine
        if (alloc_csum_engine()) {
            // Did not obtain the csum engine, so return will failure status
            return 1;
        }
    }

    {
        // Assemble complete memory settings, accounting for csum generation if
        // required
        oxnas_dma_device_settings_t src_settings =
            do_checksum ? oxnas_ram_csum_src_dma_settings :
                          oxnas_ram_only_src_dma_settings;

        oxnas_dma_device_settings_t dst_settings = oxnas_ram_generic_dma_settings;

        // Assemble the source address
        src_settings.address_ = (unsigned long)src_adr;
        if (do_checksum) {
            // Record that we are checksumming, so that the result is read on
            // completion
            channel->checksumming_ = 1;

            // To checksum set the high order address bit to enable the engine
            src_settings.address_ |= (1UL << OXNAS_DMA_CSUM_ENABLE_ADR_BIT);
        }

        // Ensure only use the valid src address bits are used
        src_settings.address_ &= OXNAS_DMA_CSUM_ADR_MASK;
        src_settings.address_mode_ = src_mode;

        // Ensure only use the valid dst address bits are used
        dst_settings.address_ = ((unsigned long)dst_adr) & OXNAS_DMA_ADR_MASK;
        dst_settings.address_mode_ = dst_mode;

        return oxnas_dma_set_common(channel, length, &src_settings, &dst_settings, 1, paused);
    }
}

int oxnas_dma_device_set(
    oxnas_dma_channel_t         *channel,
    oxnas_dma_direction_t        direction,
    unsigned char               *mem_adr,   // Physical address
    unsigned long                length,
    oxnas_dma_device_settings_t *device_settings,
    oxnas_dma_mode_t             mem_mode,
    int                          paused)
{
    oxnas_dma_device_settings_t mem_settings;

    if (oxnas_dma_is_active(channel)) {
        printk(KERN_WARNING "oxnas_dma_device_set() Trying to use channel %u while active\n", channel->channel_number_);
    }

    // Assemble complete memory settings, ensuring addresses do not affect the
    // checksum enabling high order adr bit
    mem_settings = oxnas_ram_generic_dma_settings;
    mem_settings.address_ = ((unsigned long)mem_adr) & OXNAS_DMA_ADR_MASK;
    mem_settings.address_mode_ = mem_mode;

    device_settings->address_ &= OXNAS_DMA_ADR_MASK;

    return oxnas_dma_set_common(
        channel,
        length,
        (direction == OXNAS_DMA_TO_DEVICE)   ? &mem_settings : device_settings,
        (direction == OXNAS_DMA_FROM_DEVICE) ? &mem_settings : device_settings,
        1,
        paused);
}

int oxnas_dma_device_pair_set(
    oxnas_dma_channel_t*         channel,
    unsigned long                length,
    oxnas_dma_device_settings_t *src_device_settings,
    oxnas_dma_device_settings_t *dst_device_settings,
    int                          paused)
{
    if (oxnas_dma_is_active(channel)) {
        printk(KERN_WARNING "oxnas_dma_device_pair_set() Trying to use channel %u while active\n", channel->channel_number_);
    }

    // Ensure addresses do not affect the checksum enabling high order adr bit
    src_device_settings->address_ &= OXNAS_DMA_ADR_MASK;
    dst_device_settings->address_ &= OXNAS_DMA_ADR_MASK;
    return oxnas_dma_set_common(channel, length, src_device_settings, dst_device_settings, 1, paused);
}

static int oxnas_dma_set_sg_common(
    oxnas_dma_channel_t*         channel,
    struct scatterlist*          src_sg,
    unsigned                     src_sg_count,
    struct scatterlist*          dst_sg,
    unsigned                     dst_sg_count,
    oxnas_dma_device_settings_t* src_settings,
    oxnas_dma_device_settings_t* dst_settings)
{
    int i;
    int failed = 0;
    oxnas_dma_sg_entry_t *sg_entry;
    oxnas_dma_sg_entry_t *previous_entry;

    // Get reference to this channel's top level SG DMA descriptor structure
    oxnas_dma_sg_info_t *sg_info = channel->v_sg_info_;

    // Initialise list pointers to zero
    sg_info->v_srcEntries_ = 0;
    sg_info->p_srcEntries_ = 0;
    sg_info->v_dstEntries_ = 0;
    sg_info->p_dstEntries_ = 0;

    sg_entry = 0;
    previous_entry = 0;
    for (i=0; i < src_sg_count; i++) {
        // Is this entry contiguous with the previous one and would the combined
        // lengths not exceed the maximum that the hardware is capable of
        if (previous_entry &&
            ((previous_entry->addr_ + previous_entry->length_) == (src_sg[i].dma_address & OXNAS_DMA_CSUM_ADR_MASK)) &&
            ((previous_entry->length_ + src_sg[i].length) <= MAX_OXNAS_DMA_TRANSGER_LENGTH)) {
            // Yes, so coalesce the pair
#if defined(OXNAS_DMA_TEST) || defined(OXNAS_DMA_SG_TEST)
printk("Coalescing src SG entries\n");
#endif //defined(OXNAS_DMA_TEST) || defined(OXNAS_DMA_SG_TEST)
            previous_entry->length_ += src_sg[i].length;
        } else {
            // Allocate space for SG list entry from coherent DMA pool
#ifdef CONFIG_DESCRIPTORS_IN_SRAM
            oxnas_dma_sg_entry_t *new_sg_entry = alloc_sg_entry();
            if (!new_sg_entry) {
                failed = 1;
                break;
            }
            sg_entry = new_sg_entry;
#else // CONFIG_DESCRIPTORS_IN_SRAM
            dma_addr_t handle;
            sg_entry = (oxnas_dma_sg_entry_t*)dma_pool_alloc(dma_controller.desc_dma_pool, GFP_KERNEL | GFP_DMA, &handle);
            if (!sg_entry) {
                panic("Out of pool allocated DMA descriptors\n");
            }
            sg_entry->paddr_ = handle;
#endif // CONFIG_DESCRIPTORS_IN_SRAM

            if (previous_entry) {
                // Link the previous SG list entry forward to this one
                previous_entry->v_next_ = sg_entry;
                previous_entry->p_next_ = sg_entry->paddr_;
            } else {
                // Create a link from the SG info structure to the first SG list entry
                sg_info->v_srcEntries_ = sg_entry;
                sg_info->p_srcEntries_ = sg_entry->paddr_;
            }
            previous_entry = sg_entry;

            // Fill in the SG list entry with start address, ensuring only valid
            // address bits are used, preserving the checksum enabling flag
            sg_entry->addr_ = src_sg[i].dma_address & OXNAS_DMA_CSUM_ADR_MASK;

            // Fill in the length, checking that it does not exceed the hardware
            // allowed maximum
            sg_entry->length_ = (src_sg[i].length <= MAX_OXNAS_DMA_TRANSGER_LENGTH) ? src_sg[i].length : 0;
            if (!sg_entry->length_) {
                printk(KERN_WARNING "oxnas_dma_set_sg_common() Source entry too long, zeroing\n");
            }
        }
    }
    if (sg_entry) {
        // Mark the end of the source SG list with nulls
        sg_entry->p_next_ = 0;
        sg_entry->v_next_ = 0;
    }

    if (failed) {
        // Failed to allocate all SG src entries, so free those we did get
        oxnas_dma_sg_entry_t* sg_entry = sg_info->v_srcEntries_;
        while (sg_entry) {
            oxnas_dma_sg_entry_t* next = sg_entry->v_next_;
#ifdef CONFIG_DESCRIPTORS_IN_SRAM
            free_sg_entry(sg_entry);
#else // CONFIG_DESCRIPTORS_IN_SRAM
            dma_pool_free(dma_controller.desc_dma_pool, sg_entry, sg_entry->paddr_);
#endif // CONFIG_DESCRIPTORS_IN_SRAM
            sg_entry = next;
        }
        channel->v_sg_info_->p_srcEntries_ = 0;
        channel->v_sg_info_->v_srcEntries_ = 0;
        return 1;
    }

    // Assemble destination descriptors
    sg_entry = 0;
    previous_entry = 0;
    for (i=0; i < dst_sg_count; i++) {
        // Is this entry contiguous with the previous one?
        if (previous_entry &&
            ((previous_entry->addr_ + previous_entry->length_) == (dst_sg[i].dma_address & OXNAS_DMA_CSUM_ADR_MASK)) &&
            ((previous_entry->length_ + dst_sg[i].length) <= MAX_OXNAS_DMA_TRANSGER_LENGTH)) {
            // Yes, so coalesce the pair
#if defined(OXNAS_DMA_TEST) || defined(OXNAS_DMA_SG_TEST)
printk("Coalescing dst SG entries\n");
#endif //defined(OXNAS_DMA_TEST) || defined(OXNAS_DMA_SG_TEST)
            previous_entry->length_ += dst_sg[i].length;
        } else {
            // Allocate space for SG list entry from coherent DMA pool
#ifdef CONFIG_DESCRIPTORS_IN_SRAM
            oxnas_dma_sg_entry_t *new_sg_entry = alloc_sg_entry();
            if (!new_sg_entry) {
                failed = 1;
                break;
            }
            sg_entry = new_sg_entry;
#else // CONFIG_DESCRIPTORS_IN_SRAM
            dma_addr_t handle;
            sg_entry = (oxnas_dma_sg_entry_t*)dma_pool_alloc(dma_controller.desc_dma_pool, GFP_KERNEL | GFP_DMA, &handle);
            if (!sg_entry) {
                panic("Out of pool allocated DMA descriptors\n");
            }
            sg_entry->paddr_ = handle;
#endif // CONFIG_DESCRIPTORS_IN_SRAM

            if (previous_entry) {
                // Link the previous SG list entry forward to this one
                previous_entry->v_next_ = sg_entry;
                previous_entry->p_next_ = sg_entry->paddr_;
            } else {
                // Create a link from the SG info structure to the first SG list entry
                sg_info->v_dstEntries_ = sg_entry;
                sg_info->p_dstEntries_ = sg_entry->paddr_;
            }
            previous_entry = sg_entry;

            // Fill in the SG list entry with start address, ensuring address
            // does not affect the checksum enabling high order adr bit
            sg_entry->addr_   = dst_sg[i].dma_address & OXNAS_DMA_ADR_MASK;

            // Fill in the length, checking that it does not exceed the hardware
            // allowed maximum
            sg_entry->length_ = (dst_sg[i].length <= MAX_OXNAS_DMA_TRANSGER_LENGTH) ? dst_sg[i].length : 0;
            if (!sg_entry->length_) {
                printk(KERN_WARNING "oxnas_dma_set_sg_common() Destination entry too long, zeroing\n");
            }
        }
    }
    if (sg_entry) {
        // Mark the end of the destination SG list with nulls
        sg_entry->p_next_ = 0;
        sg_entry->v_next_ = 0;
    }

    if (failed) {
        // Failed to allocate all SG dst entries, so free those we did obtain
        oxnas_dma_sg_entry_t* sg_entry = sg_info->v_dstEntries_;
        while (sg_entry) {
            oxnas_dma_sg_entry_t* next = sg_entry->v_next_;
#ifdef CONFIG_DESCRIPTORS_IN_SRAM
            free_sg_entry(sg_entry);
#else // CONFIG_DESCRIPTORS_IN_SRAM
            dma_pool_free(dma_controller.desc_dma_pool, sg_entry, sg_entry->paddr_);
#endif // CONFIG_DESCRIPTORS_IN_SRAM
            sg_entry = next;
        }
        sg_info->p_dstEntries_ = 0;
        sg_info->v_dstEntries_ = 0;

        // Free all the SG src entries which we did sucessfully obtain
        sg_entry = sg_info->v_srcEntries_;
        while (sg_entry) {
            oxnas_dma_sg_entry_t* next = sg_entry->v_next_;
#ifdef CONFIG_DESCRIPTORS_IN_SRAM
            free_sg_entry(sg_entry);
#else // CONFIG_DESCRIPTORS_IN_SRAM
            dma_pool_free(dma_controller.desc_dma_pool, sg_entry, sg_entry->paddr_);
#endif // CONFIG_DESCRIPTORS_IN_SRAM
            sg_entry = next;
        }
        sg_info->p_srcEntries_ = 0;
        sg_info->v_srcEntries_ = 0;
        return 1;
    }

#ifdef OXNAS_USE_LEON_SG_DMA
    // Set the ID expected by the CoPro. Hide the channel number in the ms bits
    sg_info->qualifer_ = (1UL | (channel->channel_number_ << OXNAS_DMA_SG_CHANNEL_BIT));

    // Set the policy for asserting End-Of-Tranfer for both src and dst devices
    sg_info->srcEOT_ = src_settings->read_eot_policy_;
    sg_info->dstEOT_ = dst_settings->write_eot_policy_;
#endif // OXNAS_USE_LEON_SG_DMA

#ifdef OXNAS_DMA_HAVE_HW_SG
    // Make potentially blocking call to acquire a free SG DMA controller
    channel->sg_controller_ = alloc_sg_controller(channel);

    // Give the SG controller a reference to the DMA channel using it
    channel->sg_controller_->channel_ = channel;

    sg_info->qualifer_ = ((channel->channel_number_ << OXNAS_DMA_SG_CHANNEL_BIT) |
                          (src_settings->read_eot_policy_ << OXNAS_DMA_SG_SRC_EOT_BIT) |
                          (dst_settings->write_eot_policy_ << OXNAS_DMA_SG_DST_EOT_BIT) |
                          (1 << OXNAS_DMA_SG_QUALIFIER_BIT));
#endif // OXNAS_DMA_HAVE_HW_SG

    // Flags are the same for source and destination for each SG transfer component
    sg_info->control_ = encode_control_status(src_settings, dst_settings, 0);

    // Increase count of in-progress transfers on this channel
    atomic_inc(&channel->active_count_);

    return 0;
}

int oxnas_dma_set_sg(
    oxnas_dma_channel_t* channel,
    struct scatterlist*  src_sg,
    unsigned             src_sg_count,
    struct scatterlist*  dst_sg,
    unsigned             dst_sg_count,
    oxnas_dma_mode_t     src_mode,
    oxnas_dma_mode_t     dst_mode,
    int                  do_checksum)
{
    if (oxnas_dma_is_active(channel)) {
        printk(KERN_WARNING "oxnas_dma_set_sg() Trying to use channel %u while active\n", channel->channel_number_);
    }

    if (do_checksum) {
        // Arbitrate for ownership of the checksum engine
        if (alloc_csum_engine()) {
            // Failed to obtain csum engine, so return with failure status
            return 1;
        }
    }

    {
        // Assemble complete memory settings, accounting for csum generation if
        // required
        oxnas_dma_device_settings_t src_settings =
            do_checksum ? oxnas_ram_csum_src_dma_settings :
                          oxnas_ram_only_src_dma_settings;

        oxnas_dma_device_settings_t dst_settings = oxnas_ram_generic_dma_settings;

        // Normal adr bits not used for SG transfers
        src_settings.address_ = 0;
        src_settings.address_mode_ = src_mode;

        // Normal adr bits not used for SG transfers
        dst_settings.address_ = 0;
        dst_settings.address_mode_ = dst_mode;

        if (do_checksum) {
            // Record that we are checksumming, so that the result is read on
            // completion
            channel->checksumming_ = 1;

            // The high order address bit enabling the checksum engine will be
            // set by the caller in the passed scatterlist entries, for those
            // entries which are required to contribute to the checksum
            // calculation
        }

        return oxnas_dma_set_sg_common(
            channel,
            src_sg,
            src_sg_count,
            dst_sg,
            dst_sg_count,
            &src_settings,
            &dst_settings);
    }
}

int oxnas_dma_device_set_sg(
    oxnas_dma_channel_t*         channel,
    oxnas_dma_direction_t        direction,
    struct scatterlist*          mem_sg,
    unsigned                     mem_sg_count,
    oxnas_dma_device_settings_t* device_settings,
    oxnas_dma_mode_t             mem_mode)
{
    int i;
    struct scatterlist *sg;
    struct scatterlist  dev_sg;

    oxnas_dma_device_settings_t mem_settings;

    if (oxnas_dma_is_active(channel)) {
        printk(KERN_WARNING "oxnas_dma_device_set_sg() Trying to use channel %u while active\n", channel->channel_number_);
    }

    // Assemble complete memory settings
    mem_settings = oxnas_ram_generic_dma_settings;
    mem_settings.address_ = 0;  // Not used for SG transfers
    mem_settings.address_mode_ = mem_mode;

    // Need to total all memory transfer lengths and assign as device single transfer length
    dev_sg.dma_address = device_settings->address_;
    for (i=0, sg=mem_sg, dev_sg.length = 0; i < mem_sg_count; i++, sg++) {
        dev_sg.length += sg->length;
    }

    return oxnas_dma_set_sg_common(
        channel,
        (direction == OXNAS_DMA_TO_DEVICE)   ? mem_sg        : &dev_sg,
        (direction == OXNAS_DMA_TO_DEVICE)   ? mem_sg_count  : 1,
        (direction == OXNAS_DMA_FROM_DEVICE) ? mem_sg        : &dev_sg,
        (direction == OXNAS_DMA_FROM_DEVICE) ? mem_sg_count  : 1,
        (direction == OXNAS_DMA_TO_DEVICE)   ? &mem_settings : device_settings,
        (direction == OXNAS_DMA_FROM_DEVICE) ? &mem_settings : device_settings);
}

static int oxnas_dma_set_prd_common(
    oxnas_dma_channel_t         *channel,
    struct ata_prd              *src_prd,
    struct ata_prd              *dst_prd,
    oxnas_dma_device_settings_t *src_settings,
    oxnas_dma_device_settings_t *dst_settings)
{
    int i;
    int failed = 0;
    oxnas_dma_sg_entry_t *sg_entry;
    oxnas_dma_sg_entry_t *previous_entry;
    u32 eot;

    // Get reference to this channel's top level SG DMA descriptor structure
    oxnas_dma_sg_info_t *sg_info = channel->v_sg_info_;

    // Initialise list pointers to zero
    sg_info->v_srcEntries_ = 0;
    sg_info->p_srcEntries_ = 0;
    sg_info->v_dstEntries_ = 0;
    sg_info->p_dstEntries_ = 0;

    sg_entry = 0;
    previous_entry = 0;
    i=0;
    do {
        u32 addr;
        u32 length;
        u32 flags_len;

        addr = src_prd[i].addr;
        flags_len = le32_to_cpu(src_prd[i++].flags_len);
        length = flags_len & ~ATA_PRD_EOT;
        eot = flags_len & ATA_PRD_EOT;

        if (!length) {
            length = 0x10000;
        }

        // Is this entry contiguous with the previous one and would the combined
        // lengths not exceed the maximum that the hardware is capable of
        if (previous_entry &&
            ((previous_entry->addr_ + previous_entry->length_) == (addr & OXNAS_DMA_CSUM_ADR_MASK)) &&
            ((previous_entry->length_ + length) <= MAX_OXNAS_DMA_TRANSGER_LENGTH)) {
            // Yes, so coalesce the pair
#if defined(OXNAS_DMA_TEST) || defined(OXNAS_DMA_SG_TEST)
printk("Coalescing src SG entries\n");
#endif //defined(OXNAS_DMA_TEST) || defined(OXNAS_DMA_SG_TEST)
            previous_entry->length_ += length;
        } else {
            // Allocate space for SG list entry from coherent DMA pool
#ifdef CONFIG_DESCRIPTORS_IN_SRAM
            oxnas_dma_sg_entry_t *new_sg_entry = alloc_sg_entry();
            if (!new_sg_entry) {
                failed = 1;
                break;
            }
            sg_entry = new_sg_entry;
#else // CONFIG_DESCRIPTORS_IN_SRAM
            dma_addr_t handle;
            sg_entry = (oxnas_dma_sg_entry_t*)dma_pool_alloc(dma_controller.desc_dma_pool, GFP_KERNEL | GFP_DMA, &handle);
            if (!sg_entry) {
                panic("Out of pool allocated DMA descriptors\n");
            }
            sg_entry->paddr_ = handle;
#endif // CONFIG_DESCRIPTORS_IN_SRAM

            if (previous_entry) {
                // Link the previous SG list entry forward to this one
                previous_entry->v_next_ = sg_entry;
                previous_entry->p_next_ = sg_entry->paddr_;
            } else {
                // Create a link from the SG info structure to the first SG list entry
                sg_info->v_srcEntries_ = sg_entry;
                sg_info->p_srcEntries_ = sg_entry->paddr_;
            }
            previous_entry = sg_entry;

            // Fill in the SG list entry with start address, ensuring only valid
            // address bits are used, preserving the checksum enabling flag
            sg_entry->addr_ = addr & OXNAS_DMA_CSUM_ADR_MASK;

            // Fill in the length, checking that it does not exceed the hardware
            // allowed maximum
            if (length > MAX_OXNAS_DMA_TRANSGER_LENGTH) {
                printk(KERN_WARNING "oxnas_dma_set_prd_common() Source entry too long (0x%x), zeroing\n", length);
                sg_entry->length_ = 0;
            } else {
                sg_entry->length_ = length;
            }
        }
    } while (!eot);
    if (sg_entry) {
        // Mark the end of the source SG list with nulls
        sg_entry->p_next_ = 0;
        sg_entry->v_next_ = 0;
    }

    if (failed) {
        // Failed to allocate all SG src entries, so free those we did get
        oxnas_dma_sg_entry_t* sg_entry = sg_info->v_srcEntries_;
        while (sg_entry) {
            oxnas_dma_sg_entry_t* next = sg_entry->v_next_;
#ifdef CONFIG_DESCRIPTORS_IN_SRAM
            free_sg_entry(sg_entry);
#else // CONFIG_DESCRIPTORS_IN_SRAM
            dma_pool_free(dma_controller.desc_dma_pool, sg_entry, sg_entry->paddr_);
#endif // CONFIG_DESCRIPTORS_IN_SRAM
            sg_entry = next;
        }
        channel->v_sg_info_->p_srcEntries_ = 0;
        channel->v_sg_info_->v_srcEntries_ = 0;
        return 1;
    }

    // Assemble destination descriptors
    sg_entry = 0;
    previous_entry = 0;
    i=0;
    do {
        u32 addr;
        u32 length;
        u32 flags_len;

        addr = dst_prd[i].addr;
        flags_len = le32_to_cpu(dst_prd[i++].flags_len);
        length = flags_len & ~ATA_PRD_EOT;
        eot = flags_len & ATA_PRD_EOT;

        if (!length) {
            length = 0x10000;
        }

        // Is this entry contiguous with the previous one?
        if (previous_entry &&
            ((previous_entry->addr_ + previous_entry->length_) == (addr & OXNAS_DMA_CSUM_ADR_MASK)) &&
            ((previous_entry->length_ + length) <= MAX_OXNAS_DMA_TRANSGER_LENGTH)) {
            // Yes, so coalesce the pair
#if defined(OXNAS_DMA_TEST) || defined(OXNAS_DMA_SG_TEST)
printk("Coalescing dst SG entries\n");
#endif //defined(OXNAS_DMA_TEST) || defined(OXNAS_DMA_SG_TEST)
            previous_entry->length_ += length;
        } else {
            // Allocate space for SG list entry from coherent DMA pool
#ifdef CONFIG_DESCRIPTORS_IN_SRAM
            oxnas_dma_sg_entry_t *new_sg_entry = alloc_sg_entry();
            if (!new_sg_entry) {
                failed = 1;
                break;
            }
            sg_entry = new_sg_entry;
#else // CONFIG_DESCRIPTORS_IN_SRAM
            dma_addr_t handle;
            sg_entry = (oxnas_dma_sg_entry_t*)dma_pool_alloc(dma_controller.desc_dma_pool, GFP_KERNEL | GFP_DMA, &handle);
            if (!sg_entry) {
                panic("Out of pool allocated DMA descriptors\n");
            }
            sg_entry->paddr_ = handle;
#endif // CONFIG_DESCRIPTORS_IN_SRAM

            if (previous_entry) {
                // Link the previous SG list entry forward to this one
                previous_entry->v_next_ = sg_entry;
                previous_entry->p_next_ = sg_entry->paddr_;
            } else {
                // Create a link from the SG info structure to the first SG list entry
                sg_info->v_dstEntries_ = sg_entry;
                sg_info->p_dstEntries_ = sg_entry->paddr_;
            }
            previous_entry = sg_entry;

            // Fill in the SG list entry with start address, ensuring address
            // does not affect the checksum enabling high order adr bit
            sg_entry->addr_ = addr & OXNAS_DMA_ADR_MASK;

            // Fill in the length, checking that it does not exceed the hardware
            // allowed maximum
            if (length > MAX_OXNAS_DMA_TRANSGER_LENGTH) {
                printk(KERN_WARNING "oxnas_dma_set_prd_common() Destination entry too long (0x%x), zeroing\n", length);
                sg_entry->length_ = 0;
            } else {
                sg_entry->length_ = length;
            }
        }
    } while (!eot);
    if (sg_entry) {
        // Mark the end of the destination SG list with nulls
        sg_entry->p_next_ = 0;
        sg_entry->v_next_ = 0;
    }

    if (failed) {
        // Failed to allocate all SG dst entries, so free those we did obtain
        oxnas_dma_sg_entry_t* sg_entry = sg_info->v_dstEntries_;
        while (sg_entry) {
            oxnas_dma_sg_entry_t* next = sg_entry->v_next_;
#ifdef CONFIG_DESCRIPTORS_IN_SRAM
            free_sg_entry(sg_entry);
#else // CONFIG_DESCRIPTORS_IN_SRAM
            dma_pool_free(dma_controller.desc_dma_pool, sg_entry, sg_entry->paddr_);
#endif // CONFIG_DESCRIPTORS_IN_SRAM
            sg_entry = next;
        }
        sg_info->p_dstEntries_ = 0;
        sg_info->v_dstEntries_ = 0;

        // Free all the SG src entries which we did sucessfully obtain
        sg_entry = sg_info->v_srcEntries_;
        while (sg_entry) {
            oxnas_dma_sg_entry_t* next = sg_entry->v_next_;
#ifdef CONFIG_DESCRIPTORS_IN_SRAM
            free_sg_entry(sg_entry);
#else // CONFIG_DESCRIPTORS_IN_SRAM
            dma_pool_free(dma_controller.desc_dma_pool, sg_entry, sg_entry->paddr_);
#endif // CONFIG_DESCRIPTORS_IN_SRAM
            sg_entry = next;
        }
        sg_info->p_srcEntries_ = 0;
        sg_info->v_srcEntries_ = 0;
        return 1;
    }

    // Make potentially blocking call to acquire a free SG DMA controller
    channel->sg_controller_ = alloc_sg_controller(channel);

    // Give the SG controller a reference to the DMA channel using it
    channel->sg_controller_->channel_ = channel;

    sg_info->qualifer_ = ((channel->channel_number_ << OXNAS_DMA_SG_CHANNEL_BIT) |
                          (src_settings->read_eot_policy_ << OXNAS_DMA_SG_SRC_EOT_BIT) |
                          (dst_settings->write_eot_policy_ << OXNAS_DMA_SG_DST_EOT_BIT) |
                          (1 << OXNAS_DMA_SG_QUALIFIER_BIT));

    // Flags are the same for source and destination for each SG transfer component
    sg_info->control_ = encode_control_status(src_settings, dst_settings, 0);

    // Increase count of in-progress transfers on this channel
    atomic_inc(&channel->active_count_);

    return 0;
}

int oxnas_dma_device_set_prd(
    oxnas_dma_channel_t         *channel,
    oxnas_dma_direction_t        direction,
    struct ata_prd              *mem_prd,
    oxnas_dma_device_settings_t *device_settings,
    oxnas_dma_mode_t             mem_mode)
{
    int i;
    struct ata_prd dev_prd;
#ifdef DEBUG_PRD
u32 addr;
#endif // DEBUG_PRD
    u32 flags_len;
    u32 length;
    u32 eot;

    oxnas_dma_device_settings_t mem_settings;

    if (oxnas_dma_is_active(channel)) {
        printk(KERN_WARNING "oxnas_dma_device_set_prd() Trying to use channel %u while active\n", channel->channel_number_);
    }

    // Assemble complete memory settings
    mem_settings = oxnas_ram_generic_dma_settings;
    mem_settings.address_ = 0;  // Not used for SG transfers
    mem_settings.address_mode_ = mem_mode;

    // Need to total all memory transfer lengths and assign as device single transfer length
    dev_prd.addr = device_settings->address_;
    dev_prd.flags_len = ATA_PRD_EOT;
    i=0;
#ifdef DEBUG_PRD
printk("Is a %s\n", (direction == OXNAS_DMA_TO_DEVICE) ? "WRITE" : "READ");
#endif // DEBUG_PRD
    do {
#ifdef DEBUG_PRD
addr = mem_prd[i].addr;
#endif // DEBUG_PRD
        flags_len = le32_to_cpu(mem_prd[i++].flags_len);
        length = flags_len & ~ATA_PRD_EOT;
        if (!length) {
            length = 0x10000;
        }
        eot = flags_len & ATA_PRD_EOT;
        dev_prd.flags_len += length;
#ifdef DEBUG_PRD
{
int j;
printk("addr      = 0x%x\n", addr);
printk("flags_len = 0x%x\n", flags_len);
for (j=0; j < 16; j++)
    printk("0x%02x ", (int)*((unsigned char*)(phys_to_virt(addr))+j));
printk("\n");
}
#endif // DEBUG_PRD
    } while (!eot);
#ifdef DEBUG_PRD
printk("dev_prd.flags_len = 0x%x\n", dev_prd.flags_len);
#endif // DEBUG_PRD

    return oxnas_dma_set_prd_common(
        channel,
        (direction == OXNAS_DMA_TO_DEVICE)   ? mem_prd       : &dev_prd,
        (direction == OXNAS_DMA_FROM_DEVICE) ? mem_prd       : &dev_prd,
        (direction == OXNAS_DMA_TO_DEVICE)   ? &mem_settings : device_settings,
        (direction == OXNAS_DMA_FROM_DEVICE) ? &mem_settings : device_settings);
}

void oxnas_dma_set_callback(oxnas_dma_channel_t* channel, oxnas_dma_callback_t callback, oxnas_callback_arg_t arg)
{
#if defined(OXNAS_DMA_TEST) || defined(OXNAS_DMA_SG_TEST)
printk("Registering callback 0x%08x for channel %u\n", (unsigned)callback, channel->channel_number_);
#endif // defined(OXNAS_DMA_TEST) || defined(OXNAS_DMA_SG_TEST)
    channel->notification_callback_ = callback;
    channel->notification_arg_ = arg;
}

static void default_callback(
    oxnas_dma_channel_t*        channel,
    oxnas_callback_arg_t        arg,
    oxnas_dma_callback_status_t status,
    u16                         checksum,
    int                         interrupt_count)
{
    up(&channel->default_semaphore_);
}

void oxnas_dma_abort(oxnas_dma_channel_t* channel)
{
    u32 ctrl_status;
    unsigned channel_number = channel->channel_number_;
    int must_wait = 0;
    int callback_registered = 0;

    BUG_ON(in_interrupt());

    // Assert reset for the channel
    spin_lock(&dma_controller.spinlock_);
    ctrl_status = readl(DMA_CALC_REG_ADR(channel_number, DMA_CTRL_STATUS));
    ctrl_status |= DMA_CTRL_STATUS_RESET;
    writel(ctrl_status, DMA_CALC_REG_ADR(channel_number, DMA_CTRL_STATUS));
    spin_unlock(&dma_controller.spinlock_);

    // Wait for the channel to become idle - should be quick as should finish
    // after the next AHB single or burst transfer
    while (readl(DMA_CALC_REG_ADR(channel_number, DMA_CTRL_STATUS)) & DMA_CTRL_STATUS_IN_PROGRESS);

    // Deassert reset for the channel
    spin_lock(&dma_controller.spinlock_);
    ctrl_status = readl(DMA_CALC_REG_ADR(channel_number, DMA_CTRL_STATUS));
    ctrl_status &= ~DMA_CTRL_STATUS_RESET;
    writel(ctrl_status, DMA_CALC_REG_ADR(channel_number, DMA_CTRL_STATUS));
    spin_unlock(&dma_controller.spinlock_);

    // If no user callback is registered, we need to wait here for the DMA
    // channel to become inactive, i.e. for the ISR to be called and the
    // channel software returned to the idle state
    if (channel->notification_callback_ == OXNAS_DMA_CALLBACK_NUL) {
        must_wait = 1;
        if (!in_atomic()) {
            // If the callers is not calling us from atomic context we can
            // register our own callback and sleep until it is invoked
            oxnas_dma_set_callback(channel, default_callback, OXNAS_DMA_CALLBACK_ARG_NUL);
            callback_registered = 1;
        }
    }

    // Fake an interrupt to cause the channel to be cleaned up by running the
    // DMA bottom half tasklet
    fake_interrupt(channel_number);

    if (must_wait) {
        if (callback_registered) {
            // Sleep until the channel becomes inactive
            down_interruptible(&channel->default_semaphore_);

            // Deregister the callback
            oxnas_dma_set_callback(channel, OXNAS_DMA_CALLBACK_NUL, OXNAS_DMA_CALLBACK_ARG_NUL);
        } else {
            // If we reach here we are in an atomic context and thus must not do
            // anything that might cause us to sleep
            // NB. Possible problem here if we're atomic because someone has
            // called spin_lock_bh(); I'm concerned that calling do_softirq()
            // under these circumstances might cause issues, althought the net-
            // working code calls do_softirq() and doesn't appear to worry
            if (local_softirq_pending()) {
                // If an interrupt has not arrived and caused the tasklet to
                // have been run already, cause it to run now.
                do_softirq();
            }

            // The tasklet should have run by this point and cleaned up the channel
            BUG_ON(oxnas_dma_is_active(channel));
        }
    }
}

void oxnas_dma_start(oxnas_dma_channel_t* channel)
{
    // Are there SG lists setup for this channel?
    if (channel->v_sg_info_->v_srcEntries_) {
#ifdef OXNAS_DMA_SG_TEST_DUMP_DESCRIPTORS
        // Print the desciptor contents for debugging
        oxnas_dma_sg_entry_t* d = channel->v_sg_info_->v_srcEntries_;
#ifdef OXNAS_USE_LEON_SG_DMA
        printk("qualifer_ = 0x%08lx, control_ = 0x%lx, srcEOT_ = 0x%hx, dstEOT_ = 0x%hx\n", channel->v_sg_info_->qualifer_, channel->v_sg_info_->control_, channel->v_sg_info_->srcEOT_, channel->v_sg_info_->dstEOT_);
#else // OXNAS_USE_LEON_SG_DMA
        printk("qualifer_ = 0x%08lx, control_ = 0x%lx\n", channel->v_sg_info_->qualifer_, channel->v_sg_info_->control_);
#endif // OXNAS_USE_LEON_SG_DMA
        printk("Source Descriptors:\n");
        while (d) {
            printk("v_addr=0x%08x, p_addr=0x%08x, addr_=0x%08x, length_=0x%08lx, next=0x%08x\n", (u32)d, (u32)d->paddr_, d->addr_, d->length_, d->p_next_);
            d = d->v_next_;
        }
        printk("Destination Descriptors:\n");
        d = channel->v_sg_info_->v_dstEntries_;
        while (d) {
            printk("v_addr=0x%08x, p_addr=0x%08x, addr_=0x%08x, length_=0x%08lx, next=0x%08x\n", (u32)d, (u32)d->paddr_, d->addr_, d->length_, d->p_next_);
            d = d->v_next_;
        }
#endif // OXNAS_DMA_SG_TEST_DUMP_DESCRIPTORS

#if defined(CONFIG_LEON_COPRO) && defined(OXNAS_USE_LEON_SG_DMA)
        // Setup SG info ptr for h/w to channel->p_sg_info_ and CoPro active
        // type, then cause CoPro intr to queue command with CoPro
        leon_queue_cmd(
            0,                         // U32  cmd            - Only SG DMA commands supported currently
            (u32*)channel->p_sg_info_, // u32* payload        - Point to the SG transfer descriptor root
            0);                        // u32  payload_length - Implicit for SG DMA operations
#elif defined(OXNAS_DMA_HAVE_HW_SG)
	// Write to the SG-DMA channel's reset register to reset the control
	// in case the previous SG-DMA transfer failed in some way, thus
	// leaving the SG-DMA controller hung up part way through processing
	// its SG list. The reset bits are self-clearing
	writel(1UL << DMA_SG_RESETS_CONTROL_BIT, DMA_SG_CALC_REG_ADR(channel->sg_controller_->number_, DMA_SG_RESETS));

        // Write the pointer to the SG info struct into the Request Pointer reg.
        writel(channel->p_sg_info_, DMA_SG_CALC_REG_ADR(channel->sg_controller_->number_, DMA_SG_REQ_PTR));

#ifdef OXNAS_DMA_SG_TEST
printk("p_sg_info_ = 0x%08x written to 0x%08x\n", (u32)channel->p_sg_info_, DMA_SG_CALC_REG_ADR(channel->sg_controller_->number_, DMA_SG_REQ_PTR));
printk("*(DMA_SG_CONTROL) = 0x%08x\n", readl(DMA_SG_CALC_REG_ADR(channel->sg_controller_->number_, DMA_SG_CONTROL)));
printk("*(DMA_SG_STATUS)  = 0x%08x\n", readl(DMA_SG_CALC_REG_ADR(channel->sg_controller_->number_, DMA_SG_STATUS)));
printk("*(DMA_SG_REQ_PTR) = 0x%08x\n", readl(DMA_SG_CALC_REG_ADR(channel->sg_controller_->number_, DMA_SG_REQ_PTR)));
#endif // OXNAS_DMA_SG_TEST

        // Start the transfer
        writel((1UL << DMA_SG_CONTROL_START_BIT) |
               (1UL << DMA_SG_CONTROL_QUEUING_ENABLE_BIT) |
               (1UL << DMA_SG_CONTROL_HBURST_ENABLE_BIT),
               DMA_SG_CALC_REG_ADR(channel->sg_controller_->number_, DMA_SG_CONTROL));
#else
        panic("oxnas_dma_start(): No way to perform SG DMA transfers\n");
#endif // defined(CONFIG_LEON_COPRO) && defined(OXNAS_USE_LEON_SG_DMA)
    } else {
        // Single transfer mode, so unpause the DMA controller channel
        spin_lock(&dma_controller.spinlock_);
        writel(encode_start(readl(DMA_CALC_REG_ADR(channel->channel_number_, DMA_CTRL_STATUS))),
               DMA_CALC_REG_ADR(channel->channel_number_, DMA_CTRL_STATUS));
        spin_unlock(&dma_controller.spinlock_);
    }
}

void oxnas_dma_dump_registers()
{
    unsigned long* adr = (unsigned long*)DMA_CALC_REG_ADR(0, 0);
    unsigned long* end = (adr + DMA_REGS_PER_CHANNEL);
    int i;

    printk("oxnas_dma_dump_registers(), adr= 0x%08lx, end=0x%08lx\n", (unsigned long)adr, (unsigned long)(adr + (DMA_REGS_PER_CHANNEL * dma_controller.numberOfChannels_)));

    for (i=0; i < dma_controller.numberOfChannels_; i++) {
        for (; adr < end; adr++) {
            printk("0x%08lx\n", *adr);
        }
	printk("SG-Debug: 0x%08x\n", readl(DMA_SG_CALC_REG_ADR(i, DMA_SG_RESETS)));
        printk("-----------------------\n");
        end += DMA_REGS_PER_CHANNEL;
    }
    printk("oxnas_dma_dump_registers() - end\n");
}

void oxnas_dma_dump_registers_single(int channel_number)
{
    unsigned long* adr = (unsigned long*)DMA_CALC_REG_ADR(channel_number, 0);
    unsigned long* end = (adr + DMA_REGS_PER_CHANNEL);

    printk("DMA channel %d regs:\n", channel_number);
    for (; adr < end; adr++) {
        printk("0x%08lx\n", *adr);
    }
}

#if defined(OXNAS_DMA_TEST) || defined(OXNAS_DMA_SG_TEST)
static __DECLARE_SEMAPHORE_GENERIC(callback_semaphore, 0);   // Binary semaphore for testing

static void dma_callback(
    oxnas_dma_channel_t         *channel,
    oxnas_callback_arg_t         arg,
    oxnas_dma_callback_status_t  error_code,
    u16                          checksum,
    int                          interrupt_count)
{
    printk("dma_callback() for channel %u, arg = 0x%lx, status = 0x%04x, checksum = 0x%04hx, interrupt_count = %d\n", channel->channel_number_, (unsigned long)arg, error_code, checksum, interrupt_count);
    up(&callback_semaphore);
}

#include <linux/dma-mapping.h>
#include <linux/slab.h>

#ifdef OXNAS_DMA_TEST
static void dma_test(unsigned long length)
{
    void* memory1;
    void* memory2;
    unsigned long* ptr;
    unsigned long quads;
    int i;
    unsigned long* end;
    dma_addr_t dma_address1;
    dma_addr_t dma_address2;
    oxnas_dma_channel_t* channels[MAX_OXNAS_DMA_CHANNELS];

    printk("*************************************************************\n");
    printk("                                                               \n");
    printk("Simple DMA Test, length = %lu, number of channel = %u\n", length, MAX_OXNAS_DMA_CHANNELS);
    printk("                                                               \n");
    printk("*************************************************************\n");

    for (i=0; i < MAX_OXNAS_DMA_CHANNELS; ++i) {
        channels[i] = oxnas_dma_request(0);
        if (channels[i] == OXNAS_DMA_CHANNEL_NUL) {
            printk("No DMA channels[%d] obtained\n", i);
        } else {
            printk("Obtained DMA channels[%d] %u, isActive=%d\n", i, channels[i]->channel_number_, oxnas_dma_is_active(channels[i]));
        }
    }

    // Allocate some DMA coherent memory
    printk("Calling kmalloc()\n");
    memory1 = kmalloc(length, GFP_KERNEL | GFP_DMA);
    memory2 = kmalloc(length, GFP_KERNEL | GFP_DMA);

    // Test each available DMA channel
    for (i=0; i < MAX_OXNAS_DMA_CHANNELS; ++i) {
        int j;

        // Fill each memory area with a different pattern
        ptr = (unsigned long*)memory1;
        quads = length/sizeof(unsigned long);
        for (j=0; j < quads; j++) {
            *ptr++ = 0xdeadbeef;
        }
        ptr = (unsigned long*)memory2;
        for (j=0; j < quads; j++) {
            *ptr++ = 0xc001babe;
        }

        printk("Before:\n");
        ptr = (unsigned long*)memory1;
        end = (unsigned long*)(memory1 + length);
        while (ptr < end) {
            for (j=0; j < 8; j++) {
                printk("0x%08lx ", *ptr++);
            }
            printk("\n");
        }
        printk("---------------------------------------------------------\n");
        ptr = (unsigned long*)memory2;
        end = (unsigned long*)(memory2 + length);
        while (ptr < end) {
            for (j=0; j < 8; j++) {
                printk("0x%08lx ", *ptr++);
            }
            printk("\n");
        }

        // Get a consistent DMA mapping for the memory to be DMAed from - causing a
        // flush from the CPU's cache to the memory
        dma_address1 = dma_map_single(0, memory1, length, DMA_TO_DEVICE);
        if (dma_mapping_error(dma_address1)) {
            printk("Consistent DMA mapping 1 failed\n");
        }

        // Get a consistent DMA mapping for the memory to be DMAed to - causing a
        // flush and invalidation of any entries in the CPU's cache covering the
        // memory region
        dma_address2 = dma_map_single(0, memory2, length, DMA_BIDIRECTIONAL);
        if (dma_mapping_error(dma_address2)) {
            printk("Consistent DMA mapping 2 failed\n");
        }

        // Setup up DMA from first half to second half on memory, using physical addresses
        printk("Calling oxnas_dma_set(), memory1 = 0x%08lx, memory2 = 0x%08lx\n", (unsigned long)memory1, (unsigned long)memory2);
        oxnas_dma_set(
            channels[i],
            (unsigned char*)dma_address1,
            length,
            (unsigned char*)dma_address2,
            OXNAS_DMA_MODE_INC,
            OXNAS_DMA_MODE_INC,
            1,  // Calculate checksum over source data
            1); // Paused

        // Using notification callback
        oxnas_dma_set_callback(channels[i], dma_callback, OXNAS_DMA_CALLBACK_ARG_NUL);

//printk("Before starting status = 0x%08x, intId = 0x%08x\n", readl(DMA_CALC_REG_ADR(channels[i]->channel_number_, DMA_CTRL_STATUS)), readl(DMA_CALC_REG_ADR(0, DMA_INTR_ID)));
        // Start the transfer
        printk("oxnas_dma_start() for channel %u\n", channels[i]->channel_number_);
        oxnas_dma_start(channels[i]);

// Poll for transfer completion
//while (oxnas_dma_raw_isactive(channels[i])) {
//    printk(".");
//}
//printk("Found channel inactive, status = 0x%08x, intId = 0x%08x\n", readl(DMA_CALC_REG_ADR(channels[i]->channel_number_, DMA_CTRL_STATUS)), readl(DMA_CALC_REG_ADR(0, DMA_INTR_ID)));

        printk("Waiting for channel to be inactive\n");

        // Sleep until transfer completed
        while (down_interruptible(&callback_semaphore));
        oxnas_dma_set_callback(channels[i], OXNAS_DMA_CALLBACK_NUL, OXNAS_DMA_CALLBACK_ARG_NUL);

        // Release the consistent DMA mappings
        dma_unmap_single(0, dma_address1, length, DMA_TO_DEVICE);
        dma_unmap_single(0, dma_address2, length, DMA_BIDIRECTIONAL);

        printk("After:\n");
        ptr = (unsigned long*)memory1;
        end = (unsigned long*)(memory1 + length);
        while (ptr < end) {
            for (j=0; j < 8; j++) {
                printk("0x%08lx ", *ptr++);
            }
            printk("\n");
        }
        printk("---------------------------------------------------------\n");
        ptr = (unsigned long*)memory2;
        end = (unsigned long*)(memory2 + length);
        while (ptr < end) {
            for (j=0; j < 8; j++) {
                printk("0x%08lx ", *ptr++);
            }
            printk("\n");
        }
    }

    // Deallocate the memory
    printk("Calling kfree()\n");
    kfree(memory1);
    kfree(memory2);
    printk("Returned from kfree()\n");

    for (i=0; i < MAX_OXNAS_DMA_CHANNELS; ++i) {
        oxnas_dma_free(channels[i]);
    }

    for (i=0; i < MAX_OXNAS_DMA_CHANNELS; ++i) {
        channels[i] = oxnas_dma_request(0);
        if (channels[i] == OXNAS_DMA_CHANNEL_NUL) {
            printk("No DMA channels[%d] obtained\n", i);
        } else {
            printk("Obtained DMA channels[%d] %u, isActive=%d\n", i, channels[i]->channel_number_, oxnas_dma_is_active(channels[i]));
        }
    }

    for (i=0; i < MAX_OXNAS_DMA_CHANNELS; ++i) {
        oxnas_dma_free(channels[i]);
    }
}
#endif // OXNAS_DMA_TEST

#ifdef OXNAS_DMA_SG_TEST
static void dma_sg_test(void)
{
    int i;
    struct scatterlist* src_scatterlist = 0;
    struct scatterlist* dst_scatterlist = 0;
    const int num_src_buffers = 8;
    const int num_dst_buffers = 3;
    unsigned long src_fill_value = 0;
    unsigned long total_src_len = 0;
    int channel_number;
    oxnas_dma_channel_t* channels[MAX_OXNAS_DMA_CHANNELS];

    printk("*************************************************************\n");
    printk("                                                               \n");
    printk("Scatter-Gather DMA Test\n");
    printk("                                                               \n");
    printk("*************************************************************\n");

    for (i=0; i < MAX_OXNAS_DMA_CHANNELS; ++i) {
        channels[i] = oxnas_dma_request(0);
        if (channels[i] == OXNAS_DMA_CHANNEL_NUL) {
            printk("No DMA channels[%d] obtained\n", i);
        } else {
            printk("Obtained DMA channels[%d] %u, isActive=%d\n", i, channels[i]->channel_number_, oxnas_dma_is_active(channels[i]));
        }
    }

    for (channel_number=0; channel_number < MAX_OXNAS_DMA_CHANNELS; ++channel_number) {
        if (num_src_buffers) {
            printk("Allocating source SG list and entry buffers\n");
            // Allocate scatterlist and memory for source buffers - store virtual buffer
            // addresses in scatterlist.offset for convenience. Include some contiguous
            // entries to test coalescing
            src_scatterlist = (struct scatterlist*)kmalloc(sizeof(struct scatterlist) * num_src_buffers, GFP_KERNEL);
            src_scatterlist[0].offset = (unsigned int)kmalloc(8*1024,  GFP_KERNEL | GFP_DMA);
            src_scatterlist[0].__address = (char*)(8*1024);    // Real allocation length
            src_scatterlist[0].length = 8*1024;
            src_scatterlist[0].page = (struct page*)0xdeadbeef; // Fill value
            src_scatterlist[1].offset = (unsigned int)kmalloc(8,     GFP_KERNEL | GFP_DMA);
            src_scatterlist[1].__address = (char*)8;       // Real allocation length
            src_scatterlist[1].length = 8;
            src_scatterlist[1].page = (struct page*)0xc001babe; // Fill value
            src_scatterlist[2].offset = (unsigned int)kmalloc(48*1024, GFP_KERNEL | GFP_DMA);
            src_scatterlist[2].__address = (char*)(48*1024);   // Real allocation length
            src_scatterlist[2].length = 16*1024;
            src_scatterlist[2].page = (struct page*)0x22222222; // Fill value
            src_scatterlist[3].offset = src_scatterlist[2].offset + src_scatterlist[2].length;
            src_scatterlist[3].__address = (char*)0;         // No allocation
            src_scatterlist[3].length = 16*1024;
            src_scatterlist[3].page = (struct page*)0x33333333; // Fill value
            src_scatterlist[4].offset = src_scatterlist[3].offset + src_scatterlist[3].length;
            src_scatterlist[4].__address = (char*)0;         // No allocation
            src_scatterlist[4].length = 16*1024;
            src_scatterlist[4].page = (struct page*)0x44444444; // Fill value
            src_scatterlist[5].offset = (unsigned int)kmalloc(64,      GFP_KERNEL | GFP_DMA);
            src_scatterlist[5].__address = (char*)64;        // Real allocation length
            src_scatterlist[5].length = 64;
            src_scatterlist[5].page = (struct page*)0x55555555; // Fill value
            src_scatterlist[6].offset = (unsigned int)kmalloc(256,     GFP_KERNEL | GFP_DMA);
            src_scatterlist[6].__address = (char*)256;       // Real allocation length
            src_scatterlist[6].length = 128;
            src_scatterlist[6].page = (struct page*)0x66666666; // Fill value
            src_scatterlist[7].offset = src_scatterlist[6].offset + src_scatterlist[6].length;
            src_scatterlist[7].__address = (char*)0;         // No allocation
            src_scatterlist[7].length = 128;
            src_scatterlist[7].page = (struct page*)0x77777777; // Fill value
        }

        // Fill source memory buffers with stuff
        for (i=0; i < num_src_buffers; i++) {
            unsigned long* ptr = (unsigned long*)src_scatterlist[i].offset;
            int quads = src_scatterlist[i].length/sizeof(unsigned long);
            int j=0;
            printk("Filling source buffer %u\n", i);
            src_fill_value = (unsigned long)(src_scatterlist[i].page);
            for (; j < quads; j++) {
                *ptr++ = src_fill_value;
            }
        }

    #ifdef OXNAS_DMA_SG_TEST_DUMP_BUFFERS
        // Print before contents of source buffers
        printk("Source Before:\n");
        for (i=0; i < num_src_buffers; i++) {
            unsigned long* ptr = (unsigned long*)src_scatterlist[i].offset;
            unsigned long* end = (unsigned long*)(src_scatterlist[i].offset + src_scatterlist[i].length);
            printk("Buffer %d\n", i);
            while (ptr < end) {
                int j=0;
                for (; j < 8; j++) {
                    printk("0x%08lx ", *ptr++);
                }
                printk("\n");
            }
        }
    #endif // OXNAS_DMA_SG_TEST_DUMP_BUFFERS

        // Get a consistent DMA mapping for the memory to be DMAed from - causing a
        // flush from the CPU's cache to the memory
        for (i=0; i < num_src_buffers; i++) {
            printk("Creating DMA mappings for source entry buffer %u\n", i);
            src_scatterlist[i].dma_address = dma_map_single(0, (void*)src_scatterlist[i].offset, src_scatterlist[i].length, DMA_TO_DEVICE);
            if (dma_mapping_error(src_scatterlist[i].dma_address)) {
                printk("Consistent source DMA mapping %d failed\n", i);
            }

            // Set the checksum enabling high order address bit
            src_scatterlist[i].dma_address |= (1UL << OXNAS_DMA_CSUM_ENABLE_ADR_BIT);
        }

        // Allocate scatterlist and memory for destination buffers - store virtual
        // buffer addresses in scatterlist.offset for convenience
        if (num_dst_buffers) {
            unsigned long dst_length;
            unsigned long offset;

            printk("Allocating destination SG list and entry buffers\n");
            total_src_len = 0;
            for (i=0; i < num_src_buffers; i++) {
                total_src_len += src_scatterlist[i].length;
            }

            // Following will only work if no remainder due to divide
            dst_length = total_src_len / num_dst_buffers;
            dst_scatterlist = (struct scatterlist*)kmalloc(sizeof(struct scatterlist) * num_dst_buffers, GFP_KERNEL);

            // First destination segment owns the buffer
            dst_scatterlist[0].offset = (unsigned int)kmalloc(total_src_len,  GFP_KERNEL | GFP_DMA);
            dst_scatterlist[0].__address = (char*)total_src_len; // Real allocation length
            dst_scatterlist[0].length = dst_length;

            offset = dst_length;
            for (i=1; i < num_dst_buffers; i++) {
                dst_scatterlist[i].offset = dst_scatterlist[0].offset + offset;
                dst_scatterlist[i].__address = 0; // No allocation
                dst_scatterlist[i].length = dst_length;

                offset += dst_length;
            }
        }

        // Fill destination memory buffers with zero
        for (i=0; i < num_dst_buffers; i++) {
            unsigned long* ptr = (unsigned long*)dst_scatterlist[i].offset;
            int quads = dst_scatterlist[i].length/sizeof(unsigned long);
            int j=0;
            printk("Filling destination buffer %u\n", i);
            for (; j < quads; j++) {
                *ptr++ = 0x000000;
            }
        }

    //#ifdef OXNAS_DMA_SG_TEST_DUMP_BUFFERS
    //    // Print before contents of destination buffers
    //    printk("Destination Before:\n");
    //    for (i=0; i < num_dst_buffers; i++) {
    //        unsigned long* ptr = (unsigned long*)dst_scatterlist[i].offset;
    //        unsigned long* end = (unsigned long*)(dst_scatterlist[i].offset + dst_scatterlist[i].length);
    //        printk("Buffer %d\n", i);
    //        while (ptr < end) {
    //            int j=0;
    //            for (; j < 8; j++) {
    //                printk("0x%08lx ", *ptr++);
    //            }
    //            printk("\n");
    //        }
    //    }
    //#endif // OXNAS_DMA_SG_TEST_DUMP_BUFFERS

        // Get a consistent DMA mapping for the memory to be DMAed to - causing an
        // invalidate to the CPU's cache
        for (i=0; i < num_dst_buffers; i++) {
            printk("Creating DMA mappings for destination entry buffer %u\n", i);
            dst_scatterlist[i].dma_address = dma_map_single(0, (void*)dst_scatterlist[i].offset, dst_scatterlist[i].length, DMA_BIDIRECTIONAL);
            if (dma_mapping_error(dst_scatterlist[i].dma_address)) {
                printk("Consistent destination DMA mapping %d failed\n", i);
            }
        }

        // Setup up SG DMA transfer
        printk("Setting up transfer\n");
        oxnas_dma_set_sg(
            channels[channel_number],
            src_scatterlist,
            num_src_buffers,
            dst_scatterlist,
            num_dst_buffers,
            OXNAS_DMA_MODE_INC,
            OXNAS_DMA_MODE_INC,
            1); // Compute checksum

        // Using second DMA channel requested
        oxnas_dma_set_callback(channels[channel_number], dma_callback, OXNAS_DMA_CALLBACK_ARG_NUL);

        // Start the transfer
        printk("Starting the transfer\n");
        oxnas_dma_start(channels[channel_number]);

        // Sleep until transfer completed
        printk("Waiting for transfer to complete...\n");

        while (down_interruptible(&callback_semaphore));
        oxnas_dma_set_callback(channels[channel_number], OXNAS_DMA_CALLBACK_NUL, OXNAS_DMA_CALLBACK_ARG_NUL);

        // Release the consistent DMA mappings for the source buffers
        for (i=0; i < num_src_buffers; i++) {
            printk("Releasing DMA mappings for source entry buffer %u\n", i);
            // Ensure the checksum enabling high order address bit is not set, as
            // this would confuse the DMA mapping release function
            src_scatterlist[i].dma_address &= ~(1UL << OXNAS_DMA_CSUM_ENABLE_ADR_BIT);
            dma_unmap_single(0, src_scatterlist[i].dma_address, src_scatterlist[i].length, DMA_TO_DEVICE);
        }

        // Release the consistent DMA mappings for the destination buffers
        for (i=0; i < num_dst_buffers; i++) {
            printk("Releasing DMA mappings for destination entry buffer %u\n", i);
            dma_unmap_single(0, dst_scatterlist[i].dma_address, dst_scatterlist[i].length, DMA_BIDIRECTIONAL);
        }

    {
    u32 sw_csum = 0;
    for (i=0; i < num_src_buffers; i++) {
        sw_csum = csum_partial((u8*)src_scatterlist[i].offset, src_scatterlist[i].length, sw_csum);
    }
    printk("S/W generated src csum = 0x%04hx\n", csum_fold(sw_csum));

    sw_csum = 0;
    for (i=0; i < num_dst_buffers; i++) {
        sw_csum = csum_partial((u8*)dst_scatterlist[i].offset, dst_scatterlist[i].length, sw_csum);
    }
    printk("S/W generated dst csum = 0x%04hx\n", csum_fold(sw_csum));
    }

    #ifdef OXNAS_DMA_SG_TEST_DUMP_BUFFERS
    //    // Print after contents of source buffers
    //    printk("Source After:\n");
    //    for (i=0; i < num_src_buffers; i++) {
    //        unsigned long* ptr = (unsigned long*)src_scatterlist[i].offset;
    //        unsigned long* end = (unsigned long*)(src_scatterlist[i].offset + src_scatterlist[i].length);
    //        printk("Buffer %d\n", i);
    //        while (ptr < end) {
    //            int j=0;
    //            for (; j < 8; j++) {
    //                printk("0x%08lx ", *ptr++);
    //            }
    //            printk("\n");
    //        }
    //    }

        // Print after contents of destination buffers
        printk("Destination After:\n");
        for (i=0; i < num_dst_buffers; i++) {
            unsigned long* ptr = (unsigned long*)dst_scatterlist[i].offset;
            unsigned long* end = (unsigned long*)(dst_scatterlist[i].offset + dst_scatterlist[i].length);
            printk("Buffer %d\n", i);
            while (ptr < end) {
                int j=0;
                for (; j < 8; j++) {
                    printk("0x%08lx ", *ptr++);
                }
                printk("\n");
            }
        }
    #endif // OXNAS_DMA_SG_TEST_DUMP_BUFFERS

        // Free the memory for the source buffers
        for (i=0; i < num_src_buffers; i++) {
            // Check that unique allocation made for this entry
            if (src_scatterlist[i].__address) {
                printk("Freeing source SG entry buffer, adr = 0x%08x, len = 0x%08x\n", src_scatterlist[i].offset, (u32)src_scatterlist[i].__address);
                kfree((void*)src_scatterlist[i].offset);
            }
        }

        // Free the memory for the source scatterlist
        if (src_scatterlist) {
            printk("Freeing source SG scatter list structure\n");
            kfree(src_scatterlist);
        }

        // Free the memory for the destination buffers
        for (i=0; i < num_dst_buffers; i++) {
            if (dst_scatterlist[i].__address) {
                printk("Freeing destination SG entry, adr = 0x%08x, len = 0x%08x\n", dst_scatterlist[i].offset, (u32)dst_scatterlist[i].__address);
                kfree((void*)dst_scatterlist[i].offset);
            }
        }

        // Free the memory for the destination scatterlist
        if (dst_scatterlist) {
            printk("Freeing source SG scatter list structure\n");
            kfree(dst_scatterlist);
        }
    }

    for (i=0; i < MAX_OXNAS_DMA_CHANNELS; ++i) {
        oxnas_dma_free(channels[i]);
    }
}
#endif // OXNAS_DMA_SG_TEST

#ifdef OXNAS_DMA_SG_TEST_2
static void dma_sg_test2()
{
    /** Include initial 2 bytes of pad that real network buffers would contain
        in order to ensure that IP header and TCP/UDP header are quad aligned */
    static const unsigned char bad_src_data0[] = {
        0xff, 0xff, 0x00, 0xa0, 0xd2, 0x05, 0x06, 0xec, 0x00, 0xcf, 0x52, 0x49, 0xc3, 0x03, 0x08, 0x00,
        0x45, 0x00, 0x05, 0xb4, 0x99, 0x45, 0x40, 0x00, 0x40, 0x06, 0x42, 0xf5, 0xac, 0x1f, 0x00, 0x65,
        0xac, 0x1f, 0x00, 0x66
    };

    static const unsigned char bad_src_data1[] = {
        0x04, 0x00, 0x13, 0x89, 0x02, 0x8a, 0x5c, 0x83, 0x52, 0xde, 0xc7, 0x0c, 0x80, 0x19, 0x0b, 0x68,
        0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x08, 0x0a, 0xff, 0xff, 0xb3, 0x9d, 0x3f, 0x82, 0xf0, 0xff,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31,
        0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33,
        0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
        0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31
    };

    /** Include initial 2 bytes of pad that real network buffers would contain
        in order to ensure that IP header and TCP/UDP header are quad aligned */
    static const unsigned char good_src_data0[] = {
        0xff, 0xff, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5
    };

    static const unsigned char good_src_data1[] = {
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
        0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5
    };

    static const int src_offset = 2;    // To jump IP quad align padding
    static const int dst_buffer_size = 512;

    const unsigned char *src_data0 = bad_src_data0;
    const unsigned char *src_data1 = bad_src_data1;
    unsigned long src_data0_len = sizeof(bad_src_data0);
    unsigned long src_data1_len = sizeof(bad_src_data1);
    int channel_number;
    oxnas_dma_channel_t* channels[MAX_OXNAS_DMA_CHANNELS];
    int i;

//    const unsigned char *src_data0 = good_src_data0;
//    const unsigned char *src_data1 = good_src_data1;
//    unsigned long src_data0_len = sizeof(good_src_data0);
//    unsigned long src_data1_len = sizeof(good_src_data1);

    printk("*************************************************************\n");
    printk("                                                               \n");
    printk("Scatter-Gather DMA Test 2\n");
    printk("                                                               \n");
    printk("*************************************************************\n");

    printk("seg0 0x%08x, %lu\n", (u32)src_data0, src_data0_len);
    printk("seg1 0x%08x, %lu\n", (u32)src_data1, src_data1_len);

    for (i=0; i < MAX_OXNAS_DMA_CHANNELS; ++i) {
        channels[i] = oxnas_dma_request(0);
        if (channels[i] == OXNAS_DMA_CHANNEL_NUL) {
            printk("No DMA channels[%d] obtained\n", i);
        } else {
            printk("Obtained DMA channels[%d] %u, isActive=%d\n", i, channels[i]->channel_number_, oxnas_dma_is_active(channels[i]));
        }
    }

    // Test each available DMA channel
    for (channel_number=0; channel_number < MAX_OXNAS_DMA_CHANNELS; ++channel_number) {

        struct scatterlist* src_scatterlist = (struct scatterlist*)kmalloc(sizeof(struct scatterlist) * 2, GFP_KERNEL);

        unsigned long total_src_length = src_data0_len + src_data1_len;
        src_scatterlist[0].offset = (unsigned int)kmalloc(total_src_length,  GFP_KERNEL | GFP_DMA) + src_offset;
        src_scatterlist[0].length = src_data0_len - src_offset;
        memcpy((u8*)src_scatterlist[0].offset, src_data0, src_scatterlist[0].length);

        src_scatterlist[1].offset = src_scatterlist[0].offset + src_scatterlist[0].length;
        src_scatterlist[1].length = src_data1_len;
        memcpy((u8*)src_scatterlist[1].offset, src_data1, src_scatterlist[1].length);

        unsigned long total_dst_length = total_src_length - src_offset;  // Excludes initial IP quad alignment pad
        unsigned num_dst_buffers = total_dst_length / dst_buffer_size;
        if ((num_dst_buffers * dst_buffer_size) < total_dst_length) {
            ++num_dst_buffers;
        }
        printk("total_src_length = %lu, src_offset = %u, total_dst_length = %lu, dst_buffer_size = %u, num_dst_buffers = %u\n", total_src_length, src_offset, total_dst_length, dst_buffer_size, num_dst_buffers);
        struct scatterlist* dst_scatterlist = (struct scatterlist*)kmalloc(sizeof(struct scatterlist) * num_dst_buffers, GFP_KERNEL);

        int i;
        unsigned long remainder = total_dst_length;
        for (i=0; i < num_dst_buffers; ++i) {
            dst_scatterlist[i].offset = (unsigned int)kmalloc(dst_buffer_size,  GFP_KERNEL | GFP_DMA);
            dst_scatterlist[i].length = (remainder < dst_buffer_size) ? remainder : dst_buffer_size;
            remainder -= dst_scatterlist[i].length;
        }

        int j;
        for (j=0; j < OXNAS_DMA_SG_TEST2_ITERATIONS; ++j) {
            src_scatterlist[0].dma_address = dma_map_single(0, (void*)src_scatterlist[0].offset, src_scatterlist[0].length, DMA_TO_DEVICE);
            if (dma_mapping_error(src_scatterlist[0].dma_address)) {
                printk("Consistent source DMA mapping 0 failed\n");
            }
    // Set the checksum enabling high order address bit
    //src_scatterlist[0].dma_address |= (1UL << OXNAS_DMA_CSUM_ENABLE_ADR_BIT);

            src_scatterlist[1].dma_address = dma_map_single(0, (void*)src_scatterlist[1].offset, src_scatterlist[1].length, DMA_TO_DEVICE);
            if (dma_mapping_error(src_scatterlist[1].dma_address)) {
                printk("Consistent source DMA mapping 1 failed\n");
            }
            // Set the checksum enabling high order address bit
            src_scatterlist[1].dma_address |= (1UL << OXNAS_DMA_CSUM_ENABLE_ADR_BIT);

            printk("num_dst_buffers = %u\n", num_dst_buffers);
            for (i=0; i < num_dst_buffers; i++) {
                memset((void*)dst_scatterlist[i].offset, 0, dst_scatterlist[i].length);

                dst_scatterlist[i].dma_address = dma_map_single(0, (void*)dst_scatterlist[i].offset, dst_scatterlist[i].length, DMA_BIDIRECTIONAL);
                if (dma_mapping_error(dst_scatterlist[i].dma_address)) {
                    printk("Consistent destination DMA mapping %d failed\n", i);
                }
            }

            // Setup up SG DMA transfer
            printk("Setting up transfer\n");
            oxnas_dma_set_sg(
                channels[channel_number],
                src_scatterlist,
                2,
                dst_scatterlist,
                num_dst_buffers,
                OXNAS_DMA_MODE_INC,
                OXNAS_DMA_MODE_INC,
                1); // Compute checksum

            // Using second DMA channel requested
            oxnas_dma_set_callback(channels[channel_number], dma_callback, OXNAS_DMA_CALLBACK_ARG_NUL);

            // Start the transfer
            printk("Starting the transfer\n");
            oxnas_dma_start(channels[channel_number]);

            // Sleep until transfer completed
            printk("Waiting for transfer to complete...\n");
            while (down_interruptible(&callback_semaphore));
            oxnas_dma_set_callback(channels[channel_number], OXNAS_DMA_CALLBACK_NUL, OXNAS_DMA_CALLBACK_ARG_NUL);

            printk("Error code = %u\n", channels[channel_number]->error_code_);

            // Release the consistent DMA mappings for the source buffers
            for (i=0; i < 2; i++) {
                // Ensure the checksum enabling high order address bit is not set, as
                // this would confuse the DMA mapping release function
                src_scatterlist[i].dma_address &= ~(1UL << OXNAS_DMA_CSUM_ENABLE_ADR_BIT);
                dma_unmap_single(0, src_scatterlist[i].dma_address, src_scatterlist[i].length, DMA_TO_DEVICE);
            }

            // Release the consistent DMA mappings for the destination buffers
            for (i=0; i < num_dst_buffers; i++) {
                printk("Releasing DMA mappings for destination entry buffer %u\n", i);
                dma_unmap_single(0, dst_scatterlist[i].dma_address, dst_scatterlist[i].length, DMA_BIDIRECTIONAL);
            }

            u32 sw_csum = 0;
    //sw_csum = csum_partial((u8*)src_scatterlist[0].offset, src_scatterlist[0].length, 0);
            sw_csum = csum_partial((u8*)src_scatterlist[1].offset, src_scatterlist[1].length, sw_csum);
            printk("S/W generated src csum = 0x%04hx\n", csum_fold(sw_csum));

            sw_csum = 0;
            unsigned offset = src_scatterlist[0].length;
    //unsigned offset = 0;
            for (i=0; i < num_dst_buffers; i++) {
                sw_csum = csum_partial((u8*)dst_scatterlist[i].offset + offset, dst_scatterlist[i].length - offset, sw_csum);
                offset = 0;
            }
            printk("S/W generated dst csum = 0x%04hx\n", csum_fold(sw_csum));
        }

        for (i=0; i < num_dst_buffers; ++i) {
            kfree((void*)dst_scatterlist[i].offset);
        }
        kfree(dst_scatterlist);

        kfree((void*)(src_scatterlist[0].offset - src_offset));
        kfree(src_scatterlist);
    }

    for (i=0; i < MAX_OXNAS_DMA_CHANNELS; ++i) {
        oxnas_dma_free(channels[i]);
    }
}
#endif // OXNAS_DMA_SG_TEST_2
#endif // defined(OXNAS_DMA_TEST) || defined(OXNAS_DMA_SG_TEST)
