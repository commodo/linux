/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ADIN1200/1300 compatibility layer for multiple kernel versions
 *
 * Copyright 2019 Analog Devices Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#ifndef __ADIN_COMPAT_H__
#define __ADIN_COMPAT_H__

#include <linux/version.h>
#include <linux/phy.h>
#include <uapi/linux/mdio.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 13, 0)

/**
 * phy_interface_mode_is_rgmii - Convenience function for testing if a
 * PHY interface mode is RGMII (all variants)
 * @mode: the phy_interface_t enum
 */
static inline bool phy_interface_mode_is_rgmii(phy_interface_t mode)
{
	return mode >= PHY_INTERFACE_MODE_RGMII &&
		mode <= PHY_INTERFACE_MODE_RGMII_TXID;
};

#endif /* kernel < 4.13.0 */


#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 16, 0)

#define __mdiobus_read	mdiobus_read
#define __mdiobus_write	mdiobus_write

/**
 * __phy_modify_changed() - Convenience function for modifying a PHY register
 * @phydev: a pointer to a &struct phy_device
 * @regnum: register number
 * @mask: bit mask of bits to clear
 * @set: bit mask of bits to set
 *
 * Unlocked helper function which allows a PHY register to be modified as
 * new register value = (old register value & ~mask) | set
 *
 * Returns negative errno, 0 if there was no change, and 1 in case of change
 */
static inline int phy_modify_changed(struct phy_device *phydev, u32 regnum, u16 mask,
			 u16 set)
{
	int new, ret;

	ret = phy_read(phydev, regnum);
	if (ret < 0)
		return ret;

	new = (ret & ~mask) | set;
	if (new == ret)
		return 0;

	ret = phy_write(phydev, regnum, new);

	return ret < 0 ? ret : 1;
}

/**
 * phy_modify - Convenience function for modifying a PHY register
 * @phydev: the phy_device struct
 * @regnum: register number to modify
 * @mask: bit mask of bits to clear
 * @set: new value of bits set in mask to write to @regnum
 *
 * NOTE: MUST NOT be called from interrupt context,
 * because the bus read/write functions may wait for an interrupt
 * to conclude the operation.
 */
static inline int phy_modify(struct phy_device *phydev, u32 regnum, u16 mask, u16 set)
{
	int ret;

	ret = phy_modify_changed(phydev, regnum, mask, set);

	return ret < 0 ? ret : 0;
}

/**
 * phy_set_bits - Convenience function for setting bits in a PHY register
 * @phydev: the phy_device struct
 * @regnum: register number to write
 * @val: bits to set
 */
static inline int phy_set_bits(struct phy_device *phydev, u32 regnum, u16 val)
{
	return phy_modify(phydev, regnum, 0, val);
}

/**
 * phy_clear_bits - Convenience function for clearing bits in a PHY register
 * @phydev: the phy_device struct
 * @regnum: register number to write
 * @val: bits to clear
 */
static inline int phy_clear_bits(struct phy_device *phydev, u32 regnum, u16 val)
{
	return phy_modify(phydev, regnum, val, 0);
}

#endif /* kernel version < 4.16.0 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 12, 0)

#define phy_read_mmd	adin_read_mmd
#define phy_write_mmd	adin_write_mmd

int adin_read_mmd(struct phy_device *phydev, int devad, u16 regnum);
int adin_write_mmd(struct phy_device *phydev, int devad, u16 regnum,
		   u16 val);

#endif /* kernel version < 4.12.0*/

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0)

/**
 * phy_modify_mmd_changed - Function for modifying a register on MMD
 * @phydev: the phy_device struct
 * @devad: the MMD containing register to modify
 * @regnum: register number to modify
 * @mask: bit mask of bits to clear
 * @set: new value of bits set in mask to write to @regnum
 *
 * Unlocked helper function which allows a MMD register to be modified as
 * new register value = (old register value & ~mask) | set
 *
 * Returns negative errno, 0 if there was no change, and 1 in case of change
 */
static inline int phy_modify_mmd_changed(struct phy_device *phydev, int devad, u32 regnum,
			     u16 mask, u16 set)
{
	int new, ret;

	ret = phy_read_mmd(phydev, devad, regnum);
	if (ret < 0)
		return ret;

	new = (ret & ~mask) | set;
	if (new == ret)
		return 0;

	ret = phy_write_mmd(phydev, devad, regnum, new);

	return ret < 0 ? ret : 1;
}

/**
 * phy_modify_mmd - Convenience function for modifying a register on MMD
 * @phydev: the phy_device struct
 * @devad: the MMD containing register to modify
 * @regnum: register number to modify
 * @mask: bit mask of bits to clear
 * @set: new value of bits set in mask to write to @regnum
 *
 * NOTE: MUST NOT be called from interrupt context,
 * because the bus read/write functions may wait for an interrupt
 * to conclude the operation.
 */
static inline int phy_modify_mmd(struct phy_device *phydev, int devad, u32 regnum,
		     u16 mask, u16 set)
{
	int ret;

	ret = phy_modify_mmd_changed(phydev, devad, regnum, mask, set);

	return ret < 0 ? ret : 0;
}

/**
 * phy_set_bits_mmd - Convenience function for setting bits in a register
 * on MMD
 * @phydev: the phy_device struct
 * @devad: the MMD containing register to modify
 * @regnum: register number to modify
 * @val: bits to set
 */
static inline int phy_set_bits_mmd(struct phy_device *phydev, int devad,
		u32 regnum, u16 val)
{
	return phy_modify_mmd(phydev, devad, regnum, 0, val);
}

/**
 * phy_clear_bits_mmd - Convenience function for clearing bits in a register
 * on MMD
 * @phydev: the phy_device struct
 * @devad: the MMD containing register to modify
 * @regnum: register number to modify
 * @val: bits to clear
 */
static inline int phy_clear_bits_mmd(struct phy_device *phydev, int devad,
		u32 regnum, u16 val)
{
	return phy_modify_mmd(phydev, devad, regnum, val, 0);
}

#endif /* kernel version < 5.0*/

#endif /* __ADIN_COMPAT_H__ */
