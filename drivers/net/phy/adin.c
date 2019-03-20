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
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/property.h>
#include <linux/gpio/consumer.h>

#include <dt-bindings/net/adin.h>

#define PHY_ID_ADIN1200				0x0283bc20
#define PHY_ID_ADIN1300				0x0283bc30

#define ADIN1300_MII_EXT_REG_PTR		0x10
#define ADIN1300_MII_EXT_REG_DATA		0x11

#define ADIN1300_PHY_CTRL1			0x0012
#define   ADIN1300_AUTO_MDI_EN			BIT(10)
#define   ADIN1300_MAN_MDIX_EN			BIT(9)

#define ADIN1300_PHY_CTRL_STATUS2		0x0015
#define   ADIN1300_NRG_PD_EN			BIT(3)
#define   ADIN1300_NRG_PD_TX_EN			BIT(2)
#define   ADIN1300_NRG_PD_STATUS		BIT(1)

#define ADIN1300_PHY_CTRL2			0x0016
#define   ADIN1300_DOWNSPEED_AN_100_EN		BIT(11)
#define   ADIN1300_DOWNSPEED_AN_10_EN		BIT(10)
#define   ADIN1300_GROUP_MDIO_EN		BIT(6)
#define   ADIN1300_DOWNSPEEDS_EN	\
	(ADIN1300_DOWNSPEED_AN_100_EN | ADIN1300_DOWNSPEED_AN_10_EN)

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

#define ADIN1300_PHY_STATUS1			0x001a
#define   ADIN1300_PAIR_01_SWAP			BIT(11)

/* EEE register addresses, accessible via Clause 22 access using
 * ADIN1300_MII_EXT_REG_PTR & ADIN1300_MII_EXT_REG_DATA.
 * The bit-fields are the same as specified by IEEE, and can be
 * accessed via standard Clause 45 access.
 */
#define ADIN1300_EEE_CAP_REG			0x8000
#define ADIN1300_EEE_ADV_REG			0x8001
#define ADIN1300_EEE_LPABLE_REG			0x8002
#define ADIN1300_CLOCK_STOP_REG			0x9400
#define ADIN1300_LPI_WAKE_ERR_CNT_REG		0xa000

#define ADIN1300_GE_SOFT_RESET_REG		0xff0c
#define   ADIN1300_GE_SOFT_RESET		BIT(0)

#define ADIN1300_GE_RGMII_CFG_REG		0xff23
#define   ADIN1300_GE_RGMII_RX_MSK		GENMASK(8, 6)
#define   ADIN1300_GE_RGMII_RX_SEL(x)		FIELD_PREP(ADIN1300_GE_RGMII_RX_MSK, x)
#define   ADIN1300_GE_RGMII_GTX_MSK		GENMASK(5, 3)
#define   ADIN1300_GE_RGMII_GTX_SEL(x)		FIELD_PREP(ADIN1300_GE_RGMII_GTX_MSK, x)
#define   ADIN1300_GE_RGMII_RXID_EN		BIT(2)
#define   ADIN1300_GE_RGMII_TXID_EN		BIT(1)
#define   ADIN1300_GE_RGMII_EN			BIT(0)

#define ADIN1300_GE_RMII_CFG_REG		0xff24
#define   ADIN1300_GE_RMII_FIFO_DEPTH_MSK	GENMASK(6, 4)
#define   ADIN1300_GE_RMII_FIFO_DEPTH_SEL(x)	FIELD_PREP(ADIN1300_GE_RMII_FIFO_DEPTH_MSK, x)
#define   ADIN1300_GE_RMII_EN			BIT(0)

struct clause22_mmd_map {
	int devad;
	u16 cl22_regnum;
	u16 adin_regnum;
};

static struct clause22_mmd_map clause22_mmd_map[] = {
	{ MDIO_MMD_PCS,	MDIO_PCS_EEE_ABLE,	ADIN1300_EEE_CAP_REG },
	{ MDIO_MMD_AN,	MDIO_AN_EEE_LPABLE,	ADIN1300_EEE_LPABLE_REG },
	{ MDIO_MMD_AN,	MDIO_AN_EEE_ADV,	ADIN1300_EEE_ADV_REG },
	{ MDIO_MMD_PCS,	MDIO_CTRL1,		ADIN1300_CLOCK_STOP_REG },
	{ MDIO_MMD_PCS, MDIO_PCS_EEE_WK_ERR,	ADIN1300_LPI_WAKE_ERR_CNT_REG },
};

struct adin_hw_stat {
	const char *string;
	u16 reg1;
	u16 reg2;
	bool do_not_inc;
};

/* Named just like in the datasheet */
static struct adin_hw_stat adin_hw_stats[] = {
	{ "RxErrCnt",		0x0014,	},
	{ "MseA",		0x8402,	0,	true },
	{ "MseB",		0x8403,	0,	true },
	{ "MseC",		0x8404,	0,	true },
	{ "MseD",		0x8405,	0,	true },
	{ "FcFrmCnt",		0x940A, 0x940B }, /* FcFrmCntH + FcFrmCntL */
	{ "FcLenErrCnt",	0x940C },
	{ "FcAlgnErrCnt",	0x940D },
	{ "FcSymbErrCnt",	0x940E },
	{ "FcOszCnt",		0x940F },
	{ "FcUszCnt",		0x9410 },
	{ "FcOddCnt",		0x9411 },
	{ "FcOddPreCnt",	0x9412 },
	{ "FcDribbleBitsCnt",	0x9413 },
	{ "FcFalseCarrierCnt",	0x9414 },
};

/**
 * struct adin_priv - ADIN PHY driver private data
 * gpiod_reset		optional reset GPIO, to be used in soft_reset() cb
 * eee_modes		EEE modes to advertise after reset
 * edpd_enabled		true if Energy Detect Powerdown mode is enabled
 */
struct adin_priv {
	struct gpio_desc	*gpiod_reset;
	u8			eee_modes;
	bool			edpd_enabled;
	u64			stats[ARRAY_SIZE(adin_hw_stats)];
};

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
	struct device *dev = &phydev->mdio.dev;
	u32 val;
	int reg;

	reg = phy_read_mmd(phydev, MDIO_MMD_VEND1, ADIN1300_GE_RMII_CFG_REG);
	if (reg < 0)
		return reg;

	if (intf != PHY_INTERFACE_MODE_RMII) {
		reg &= ~ADIN1300_GE_RMII_EN;
		goto write;
	}

	reg |= ADIN1300_GE_RMII_EN;

	if (device_property_read_u32(dev, "adi,fifo-depth", &val))
		val = ADIN1300_RMII_8_BITS;

	reg &= ~ADIN1300_GE_RMII_FIFO_DEPTH_MSK;
	reg |= ADIN1300_GE_RMII_FIFO_DEPTH_SEL(val);

write:
	return phy_write_mmd(phydev, MDIO_MMD_VEND1,
			     ADIN1300_GE_RMII_CFG_REG, reg);
}

static int adin_config_init_eee(struct phy_device *phydev)
{
	struct adin_priv *priv = phydev->priv;
	int reg;

	reg = phy_read_mmd(phydev, MDIO_MMD_VEND1, ADIN1300_EEE_ADV_REG);
	if (reg < 0)
		return reg;

	if (priv->eee_modes)
		reg |= priv->eee_modes;
	else
		reg &= ~(MDIO_EEE_100TX | MDIO_EEE_1000T);

	return phy_write_mmd(phydev, MDIO_MMD_VEND1, ADIN1300_EEE_ADV_REG, reg);
}

static int adin_config_init_edpd(struct phy_device *phydev)
{
	struct adin_priv *priv = phydev->priv;

	if (priv->edpd_enabled)
		return phy_set_bits(phydev, ADIN1300_PHY_CTRL_STATUS2,
				(ADIN1300_NRG_PD_EN | ADIN1300_NRG_PD_TX_EN));

	return phy_clear_bits(phydev, ADIN1300_PHY_CTRL_STATUS2,
			(ADIN1300_NRG_PD_EN | ADIN1300_NRG_PD_TX_EN));
}

static int adin_config_init(struct phy_device *phydev)
{
	phy_interface_t interface, rc;

	phydev->mdix_ctrl = ETH_TP_MDI_AUTO;

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

	rc = adin_config_init_eee(phydev);
	if (rc < 0)
		return rc;

	rc = adin_config_init_edpd(phydev);
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

static int adin_cl22_to_adin_reg(int devad, u16 cl22_regnum)
{
	struct clause22_mmd_map *m;
	int i;

	if (devad == MDIO_MMD_VEND1)
		return cl22_regnum;

	for (i = 0; i < ARRAY_SIZE(clause22_mmd_map); i++) {
		m = &clause22_mmd_map[i];
		if (m->devad == devad && m->cl22_regnum == cl22_regnum)
			return m->adin_regnum;
	}

	pr_err("No translation available for devad: %d reg: %04x\n",
	       devad, cl22_regnum);

	return -EINVAL;
}

static int adin_read_mmd(struct phy_device *phydev, int devad, u16 regnum)
{
	struct mii_bus *bus = phydev->mdio.bus;
	int phy_addr = phydev->mdio.addr;
	int adin_regnum;
	int err;

	if (phydev->is_c45) {
		u32 addr = MII_ADDR_C45 | (devad << 16) | (regnum & 0xffff);

		return __mdiobus_read(bus, phy_addr, addr);
	}

	adin_regnum = adin_cl22_to_adin_reg(devad, regnum);
	if (adin_regnum < 0)
		return adin_regnum;

	err = __mdiobus_write(bus, phy_addr, ADIN1300_MII_EXT_REG_PTR,
			      adin_regnum);
	if (err)
		return err;

	return __mdiobus_read(bus, phy_addr, ADIN1300_MII_EXT_REG_DATA);
}

static int adin_write_mmd(struct phy_device *phydev, int devad, u16 regnum,
			  u16 val)
{
	struct mii_bus *bus = phydev->mdio.bus;
	int phy_addr = phydev->mdio.addr;
	int adin_regnum;
	int err;

	if (phydev->is_c45) {
		u32 addr = MII_ADDR_C45 | (devad << 16) | (regnum & 0xffff);

		return __mdiobus_write(bus, phy_addr, addr, val);
	}

	adin_regnum = adin_cl22_to_adin_reg(devad, regnum);
	if (adin_regnum < 0)
		return adin_regnum;

	err = __mdiobus_write(bus, phy_addr, ADIN1300_MII_EXT_REG_PTR,
			      adin_regnum);
	if (err)
		return err;

	return __mdiobus_write(bus, phy_addr, ADIN1300_MII_EXT_REG_DATA, val);
}

static int adin_config_mdix(struct phy_device *phydev)
{
	bool auto_en, mdix_en;
	int reg;

	mdix_en = false;
	auto_en = false;
	switch (phydev->mdix_ctrl) {
	case ETH_TP_MDI:
		break;
	case ETH_TP_MDI_X:
		mdix_en = true;
		break;
	case ETH_TP_MDI_AUTO:
		auto_en = true;
		break;
	default:
		return -EINVAL;
	}

	reg = phy_read(phydev, ADIN1300_PHY_CTRL1);
	if (reg < 0)
		return reg;

	if (mdix_en)
		reg |= ADIN1300_MAN_MDIX_EN;
	else
		reg &= ~ADIN1300_MAN_MDIX_EN;

	if (auto_en)
		reg |= ADIN1300_AUTO_MDI_EN;
	else
		reg &= ~ADIN1300_AUTO_MDI_EN;

	return phy_write(phydev, ADIN1300_PHY_CTRL1, reg);
}

static int adin_config_downspeeds(struct phy_device *phydev)
{
	int reg;

	reg = phy_read(phydev, ADIN1300_PHY_CTRL2);
	if (reg < 0)
		return reg;

	if ((reg & ADIN1300_DOWNSPEEDS_EN) == ADIN1300_DOWNSPEEDS_EN)
		return 0;

	reg |= ADIN1300_DOWNSPEEDS_EN;

	return phy_write(phydev, ADIN1300_PHY_CTRL2, reg);
}

static int adin_config_aneg(struct phy_device *phydev)
{
	int ret;

	ret = adin_config_mdix(phydev);
	if (ret)
		return ret;

	ret = adin_config_downspeeds(phydev);
	if (ret < 0)
		return ret;

	return genphy_config_aneg(phydev);
}

static int adin_mdix_update(struct phy_device *phydev)
{
	bool auto_en, mdix_en;
	bool swapped;
	int reg;

	reg = phy_read(phydev, ADIN1300_PHY_CTRL1);
	if (reg < 0)
		return reg;

	auto_en = !!(reg & ADIN1300_AUTO_MDI_EN);
	mdix_en = !!(reg & ADIN1300_MAN_MDIX_EN);

	/* If MDI/MDIX is forced, just read it from the control reg */
	if (!auto_en) {
		if (mdix_en)
			phydev->mdix = ETH_TP_MDI_X;
		else
			phydev->mdix = ETH_TP_MDI;
		return 0;
	}

	/**
	 * Otherwise, we need to deduce it from the PHY status2 reg.
	 * When Auto-MDI is enabled, the ADIN1300_MAN_MDIX_EN bit implies
	 * a preference for MDIX when it is set.
	 */
	reg = phy_read(phydev, ADIN1300_PHY_STATUS1);
	if (reg < 0)
		return reg;

	swapped = !!(reg & ADIN1300_PAIR_01_SWAP);

	if (mdix_en != swapped)
		phydev->mdix = ETH_TP_MDI_X;
	else
		phydev->mdix = ETH_TP_MDI;

	return 0;
}

static int adin_read_status(struct phy_device *phydev)
{
	int ret;

	ret = adin_mdix_update(phydev);
	if (ret < 0)
		return ret;

	return genphy_read_status(phydev);
}

static int adin_subsytem_soft_reset(struct phy_device *phydev)
{
	int reg, rc, i;

	reg = phy_read_mmd(phydev, MDIO_MMD_VEND1, ADIN1300_GE_SOFT_RESET_REG);
	if (reg < 0)
		return reg;

	reg |= ADIN1300_GE_SOFT_RESET;
	rc = phy_write_mmd(phydev, MDIO_MMD_VEND1, ADIN1300_GE_SOFT_RESET_REG,
			   reg);
	if (rc < 0)
		return rc;

	for (i = 0; i < 20; i++) {
		udelay(500);
		reg = phy_read_mmd(phydev, MDIO_MMD_VEND1,
				   ADIN1300_GE_SOFT_RESET_REG);
		if (reg < 0 || (reg & ADIN1300_GE_SOFT_RESET))
			continue;
		return 0;
	}

	return -ETIMEDOUT;
}

static int adin_reset(struct phy_device *phydev)
{
	struct adin_priv *priv = phydev->priv;
	int ret;

	/* Update EEE settings before resetting, in case ethtool changed them */
	ret = phy_read_mmd(phydev, MDIO_MMD_VEND1, ADIN1300_EEE_ADV_REG);
	priv->eee_modes = (ret & (MDIO_EEE_100TX | MDIO_EEE_1000T));

	if (priv->gpiod_reset) {
		/* GPIO reset requires min 10 uS low,
		 * 5 msecs max before we know that the interface is up again
		 */
		gpiod_set_value(priv->gpiod_reset, 0);
		udelay(12);
		gpiod_set_value(priv->gpiod_reset, 1);
		mdelay(6);

		return 0;
	}

	/* Reset PHY core regs & subsystem regs */
	return adin_subsytem_soft_reset(phydev);
}

static int adin_get_sset_count(struct phy_device *phydev)
{
	return ARRAY_SIZE(adin_hw_stats);
}

static void adin_get_strings(struct phy_device *phydev, u8 *data)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(adin_hw_stats); i++) {
		memcpy(data + i * ETH_GSTRING_LEN,
		       adin_hw_stats[i].string, ETH_GSTRING_LEN);
	}
}

static int adin_read_mmd_stat_regs(struct phy_device *phydev,
				   struct adin_hw_stat *stat,
				   u32 *val)
{
	int ret;

	ret = phy_read_mmd(phydev, MDIO_MMD_VEND1, stat->reg1);
	if (ret < 0)
		return ret;

	*val = (ret & 0xffff);

	if (stat->reg2 == 0)
		return 0;

	ret = phy_read_mmd(phydev, MDIO_MMD_VEND1, stat->reg2);
	if (ret < 0)
		return ret;

	*val <<= 16;
	*val |= (ret & 0xffff);

	return 0;
}

static u64 adin_get_stat(struct phy_device *phydev, int i)
{
	struct adin_hw_stat *stat = &adin_hw_stats[i];
	struct adin_priv *priv = phydev->priv;
	u32 val;
	int ret;

	if (stat->reg1 > 0x1f) {
		ret = adin_read_mmd_stat_regs(phydev, stat, &val);
		if (ret < 0)
			return (u64)(~0);
	} else {
		ret = phy_read(phydev, stat->reg1);
		if (ret < 0)
			return (u64)(~0);
		val = (ret & 0xffff);
	}

	if (stat->do_not_inc)
		priv->stats[i] = val;
	else
		priv->stats[i] += val;

	return priv->stats[i];
}

static void adin_get_stats(struct phy_device *phydev,
			   struct ethtool_stats *stats, u64 *data)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(adin_hw_stats); i++)
		data[i] = adin_get_stat(phydev, i);
}

static int adin_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct gpio_desc *gpiod_reset;
	struct adin_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	gpiod_reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(gpiod_reset))
		gpiod_reset = NULL;

	priv->gpiod_reset = gpiod_reset;
	if (device_property_read_bool(dev, "adi,eee-enabled"))
		priv->eee_modes = (MDIO_EEE_100TX | MDIO_EEE_1000T);
	if (device_property_read_bool(dev, "adi,disable-energy-detect"))
		priv->edpd_enabled = false;
	else
		priv->edpd_enabled = true;
	phydev->priv = priv;

	return adin_reset(phydev);
}

static struct phy_driver adin_driver[] = {
	{
		.phy_id		= PHY_ID_ADIN1200,
		.name		= "ADIN1200",
		.phy_id_mask	= 0xfffffff0,
		.features	= PHY_BASIC_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.config_init	= adin_config_init,
		.probe		= adin_probe,
		.config_aneg	= adin_config_aneg,
		.read_status	= adin_read_status,
		.ack_interrupt	= adin_phy_ack_intr,
		.config_intr	= adin_phy_config_intr,
		.get_sset_count	= adin_get_sset_count,
		.get_strings	= adin_get_strings,
		.get_stats	= adin_get_stats,
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
		.probe		= adin_probe,
		.config_aneg	= adin_config_aneg,
		.read_status	= adin_read_status,
		.ack_interrupt	= adin_phy_ack_intr,
		.config_intr	= adin_phy_config_intr,
		.get_sset_count	= adin_get_sset_count,
		.get_strings	= adin_get_strings,
		.get_stats	= adin_get_stats,
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

