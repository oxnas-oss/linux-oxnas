/*
 * linux/arch/arm/mach-oxnas/gmac_phy.h
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
#if !defined(__GMAC_PHY_H__)
#define __GMAC_PHY_H__

#include <asm/types.h>
#include <linux/netdevice.h>
#include <linux/mii.h>
#include "gmac.h"

#define PHY_TYPE_NONE               0
#define PHY_TYPE_MICREL_KS8721BL    0x00221619
#define PHY_TYPE_VITESSE_VSC8201XVZ 0x000fc413

#define VSC8201_MII_ACSR    0x1c    // Vitesse VCS8201 gigabit PHY Auxillary Control and Status register
#define VSC8201_MII_ACSR_MDPPS_BIT 2    // Mode/Duplex Pin Priority Select

extern int phy_read(struct net_device *dev, int phyaddr, int phyreg);

extern void phy_write(struct net_device *dev, int phyaddr, int phyreg, int phydata);

extern void phy_detect(struct net_device *dev);

extern int phy_reset(struct net_device *dev);

extern void phy_powerdown(struct net_device *dev);

extern void start_phy_reset(gmac_priv_t* priv);

extern int is_phy_reset_complete(gmac_priv_t* priv);

extern void set_phy_mode(struct net_device *dev);
#endif        //  #if !defined(__GMAC_PHY_H__)
