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
#include <linux/bitfield.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/property.h>

#include <dt-bindings/net/adin.h>

#define PHY_ID_ADIN1200				0x0283bc20
#define PHY_ID_ADIN1300				0x0283bc30

#define ADIN1300_MII_EXT_REG_PTR		0x10
#define ADIN1300_MII_EXT_REG_DATA		0x11

#define ADIN1300_INT_MASK_REG			0x0018
#define   ADIN1300_INT_MDIO_SYNC_EN		BIT(9)
#define   ADIN1300_INT_ANEG_STAT_CHNG_EN	BIT(8)
#define   ADIN1300_INT_ANEG_PAGE_RX_EN		BIT(6)
#define   ADIN1300_INT_IDLE_ERR_CNT_EN		BIT(5)
#define   ADIN1300_INT_MAC_FIFO_OU_EN		BIT(4)
#define   ADIN1300_INT_RX_STAT_CHNG_EN		BIT(3)
#define   ADIN1300_INT_LINK_STAT_CHNG_EN	BIT(2)
#define   ADIN1300_INT_SPEED_CHNG_EN		BIT(1)
#define   ADIN1300_INT_HW_IRQ_EN		BIT(0)
#define ADIN1300_INT_MASK_EN	\
	(ADIN1300_INT_ANEG_STAT_CHNG_EN | ADIN1300_INT_ANEG_PAGE_RX_EN | \
	 ADIN1300_INT_LINK_STAT_CHNG_EN | ADIN1300_INT_SPEED_CHNG_EN | \
	 ADIN1300_INT_HW_IRQ_EN)
#define ADIN1300_INT_STATUS_REG			0x0019

#define ADIN1300_GE_RGMII_CFG_REG		0xff23
#define   ADIN1300_GE_RGMII_RX_MSK		GENMASK(8, 6)
#define   ADIN1300_GE_RGMII_RX_SEL(x)		FIELD_PREP(ADIN1300_GE_RGMII_RX_MSK, x)
#define   ADIN1300_GE_RGMII_GTX_MSK		GENMASK(5, 3)
#define   ADIN1300_GE_RGMII_GTX_SEL(x)		FIELD_PREP(ADIN1300_GE_RGMII_GTX_MSK, x)
#define   ADIN1300_GE_RGMII_RXID_EN		BIT(2)
#define   ADIN1300_GE_RGMII_TXID_EN		BIT(1)
#define   ADIN1300_GE_RGMII_EN			BIT(0)

#define ADIN1300_GE_RMII_CFG_REG		0xff24
#define   ADIN1300_GE_RMII_EN			BIT(0)

static int adin_get_phy_internal_mode(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	const char *pm;
	int i;

	if (device_property_read_string(dev, "phy-mode-internal", &pm))
		return phydev->interface;

	/**
	 * Getting here assumes that there is converter in-between the actual
	 * PHY, for example a GMII-to-RGMII converter. In this case the MAC
	 * talks GMII and PHY talks RGMII, so the PHY needs to be set in RGMII
	 * while the MAC can work in GMII mode.
	 */

	for (i = 0; i < PHY_INTERFACE_MODE_MAX; i++)
		if (!strcasecmp(pm, phy_modes(i)))
			return i;

	dev_err(dev, "Invalid value for 'phy-mode-internal': '%s'\n", pm);

	return -EINVAL;
}

static void adin_config_rgmii_rx_internal_delay(struct phy_device *phydev,
						int *reg)
{
	struct device *dev = &phydev->mdio.dev;
	u32 val;

	if (device_property_read_u32(dev, "adi,rx-internal-delay", &val))
		val = ADIN1300_RGMII_2_00_NS;

	*reg &= ADIN1300_GE_RGMII_RX_MSK;
	*reg |= ADIN1300_GE_RGMII_RX_SEL(val);
}

static void adin_config_rgmii_tx_internal_delay(struct phy_device *phydev,
						int *reg)
{
	struct device *dev = &phydev->mdio.dev;
	u32 val;

	if (device_property_read_u32(dev, "adi,tx-internal-delay", &val))
		val = ADIN1300_RGMII_2_00_NS;

	*reg &= ADIN1300_GE_RGMII_GTX_MSK;
	*reg |= ADIN1300_GE_RGMII_GTX_SEL(val);
}

static int adin_config_rgmii_mode(struct phy_device *phydev,
				  phy_interface_t intf)
{
	int reg;

	reg = phy_read_mmd(phydev, MDIO_MMD_VEND1, ADIN1300_GE_RGMII_CFG_REG);
	if (reg < 0)
		return reg;

	if (!phy_interface_mode_is_rgmii(intf)) {
		reg &= ~ADIN1300_GE_RGMII_EN;
		goto write;
	}

	reg |= ADIN1300_GE_RGMII_EN;

	if (intf == PHY_INTERFACE_MODE_RGMII_ID ||
	    intf == PHY_INTERFACE_MODE_RGMII_RXID) {
		reg |= ADIN1300_GE_RGMII_RXID_EN;
		adin_config_rgmii_rx_internal_delay(phydev, &reg);
	} else
		reg &= ~ADIN1300_GE_RGMII_RXID_EN;

	if (intf == PHY_INTERFACE_MODE_RGMII_ID ||
	    intf == PHY_INTERFACE_MODE_RGMII_TXID) {
		reg |= ADIN1300_GE_RGMII_TXID_EN;
		adin_config_rgmii_tx_internal_delay(phydev, &reg);
	} else
		reg &= ~ADIN1300_GE_RGMII_TXID_EN;

write:
	return phy_write_mmd(phydev, MDIO_MMD_VEND1,
			     ADIN1300_GE_RGMII_CFG_REG, reg);
}

static int adin_config_rmii_mode(struct phy_device *phydev,
				 phy_interface_t intf)
{
	int reg;

	reg = phy_read_mmd(phydev, MDIO_MMD_VEND1, ADIN1300_GE_RMII_CFG_REG);
	if (reg < 0)
		return reg;

	if (intf != PHY_INTERFACE_MODE_RMII) {
		reg &= ~ADIN1300_GE_RMII_EN;
		goto write;
	}

	reg |= ADIN1300_GE_RMII_EN;

write:
	return phy_write_mmd(phydev, MDIO_MMD_VEND1,
			     ADIN1300_GE_RMII_CFG_REG, reg);
}

static int adin_config_init(struct phy_device *phydev)
{
	phy_interface_t interface, rc;

	rc = genphy_config_init(phydev);
	if (rc < 0)
		return rc;

	interface = adin_get_phy_internal_mode(phydev);
	if (interface < 0)
		return interface;

	rc = adin_config_rgmii_mode(phydev, interface);
	if (rc < 0)
		return rc;

	rc = adin_config_rmii_mode(phydev, interface);
	if (rc < 0)
		return rc;

	if (phydev->interface == interface)
		dev_info(&phydev->mdio.dev, "PHY is using mode '%s'\n",
			 phy_modes(phydev->interface));
	else
		dev_info(&phydev->mdio.dev,
			 "PHY is using mode '%s', MAC is using mode '%s'\n",
			 phy_modes(interface), phy_modes(phydev->interface));

	return 0;
}

static int adin_phy_ack_intr(struct phy_device *phydev)
{
	int ret;

	/* Clear pending interrupts.  */
	ret = phy_read(phydev, ADIN1300_INT_STATUS_REG);
	if (ret < 0)
		return ret;

	return 0;
}

static int adin_phy_config_intr(struct phy_device *phydev)
{
	/**
	 * FIXME: test this; the macb driver is not very good for testing
	 *        with interrupts
	 */

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		return phy_set_bits(phydev, ADIN1300_INT_MASK_REG,
				    ADIN1300_INT_MASK_EN);

	return phy_clear_bits(phydev, ADIN1300_INT_MASK_REG,
			      ADIN1300_INT_MASK_EN);
}

static int adin_read_mmd(struct phy_device *phydev, int devad, u16 regnum)
{
	struct mii_bus *bus = phydev->mdio.bus;
	int phy_addr = phydev->mdio.addr;
	int err;

	if (phydev->is_c45) {
		u32 addr = MII_ADDR_C45 | (devad << 16) | (regnum & 0xffff);

		return __mdiobus_read(bus, phy_addr, addr);
	}

	err = __mdiobus_write(bus, phy_addr, ADIN1300_MII_EXT_REG_PTR, regnum);
	if (err)
		return err;

	return __mdiobus_read(bus, phy_addr, ADIN1300_MII_EXT_REG_DATA);
}

static int adin_write_mmd(struct phy_device *phydev, int devad, u16 regnum,
			  u16 val)
{
	struct mii_bus *bus = phydev->mdio.bus;
	int phy_addr = phydev->mdio.addr;
	int err;

	if (phydev->is_c45) {
		u32 addr = MII_ADDR_C45 | (devad << 16) | (regnum & 0xffff);

		return __mdiobus_write(bus, phy_addr, addr, val);
	}

	err = __mdiobus_write(bus, phy_addr, ADIN1300_MII_EXT_REG_PTR, regnum);
	if (err)
		return err;

	return __mdiobus_write(bus, phy_addr, ADIN1300_MII_EXT_REG_DATA, val);
}

static struct phy_driver adin_driver[] = {
	{
		.phy_id		= PHY_ID_ADIN1200,
		.name		= "ADIN1200",
		.phy_id_mask	= 0xfffffff0,
		.features	= PHY_BASIC_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.config_init	= adin_config_init,
		.config_aneg	= genphy_config_aneg,
		.read_status	= genphy_read_status,
		.ack_interrupt	= adin_phy_ack_intr,
		.config_intr	= adin_phy_config_intr,
		.resume		= genphy_resume,
		.suspend	= genphy_suspend,
		.read_mmd	= adin_read_mmd,
		.write_mmd	= adin_write_mmd,
	},
	{
		.phy_id		= PHY_ID_ADIN1300,
		.name		= "ADIN1300",
		.phy_id_mask	= 0xfffffff0,
		.features	= PHY_GBIT_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.config_init	= adin_config_init,
		.config_aneg	= genphy_config_aneg,
		.read_status	= genphy_read_status,
		.ack_interrupt	= adin_phy_ack_intr,
		.config_intr	= adin_phy_config_intr,
		.resume		= genphy_resume,
		.suspend	= genphy_suspend,
		.read_mmd	= adin_read_mmd,
		.write_mmd	= adin_write_mmd,
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

