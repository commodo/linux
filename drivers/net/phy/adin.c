/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Driver for Analog Devices Industrial Ethernet PHYs
 *
 * Copyright 2019 Analog Devices Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/phy.h>

#define PHY_ID_ADIN1200				0x0283bc20
#define PHY_ID_ADIN1300				0x0283bc30

static int adin_config_init(struct phy_device *phydev)
{
	int rc;

	rc = genphy_config_init(phydev);
	if (rc < 0)
		return rc;

	return 0;
}

static struct phy_driver adin_driver[] = {
	{
		.phy_id		= PHY_ID_ADIN1200,
		.name		= "ADIN1200",
		.phy_id_mask	= 0xfffffff0,
		.features	= PHY_BASIC_FEATURES,
		.config_init	= adin_config_init,
		.config_aneg	= genphy_config_aneg,
		.read_status	= genphy_read_status,
		.resume		= genphy_resume,
		.suspend	= genphy_suspend,
	},
	{
		.phy_id		= PHY_ID_ADIN1300,
		.name		= "ADIN1300",
		.phy_id_mask	= 0xfffffff0,
		.features	= PHY_GBIT_FEATURES,
		.config_init	= adin_config_init,
		.config_aneg	= genphy_config_aneg,
		.read_status	= genphy_read_status,
		.resume		= genphy_resume,
		.suspend	= genphy_suspend,
	},
};

module_phy_driver(adin_driver);

static struct mdio_device_id __maybe_unused adin_tbl[] = {
	{ PHY_ID_ADIN1200, 0xfffffff0 },
	{ PHY_ID_ADIN1300, 0xfffffff0 },
	{ }
};

MODULE_DEVICE_TABLE(mdio, adin_tbl);
MODULE_DESCRIPTION("Analog Devices Industrial Ethernet PHY driver");
MODULE_LICENSE("GPL v2");

