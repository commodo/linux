// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD9737A/AD9739A SPI SPI DAC driver
 *
 * Copyright 2015-2020 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <linux/iio/dac/adi-axi-dac.h>

#define AD9739A_REG_MODE				0x00
#define   AD9739A_MODE_SDIO_DIR				BIT(7)
#define   AD9739A_MODE_LSB				BIT(6)
#define   AD9739A_MODE_RESET				BIT(5)

#define AD9739A_REG_POWER_DOWN				0x01
#define   AD9739A_POWER_DOWN_LVDS_DRVR_PD		BIT(5)
#define   AD9739A_POWER_DOWN_LVDS_RCVR_PD		BIT(4)
#define   AD9739A_POWER_DOWN_CLK_RCVR_PD		BIT(1)
#define   AD9739A_POWER_DOWN_DAC_BIAS_PD		BIT(0)

#define AD9739A_REG_CNT_CLK_DIS				0x02
#define   AD9739A_CNT_CLK_DIS_CLKGEN_PD			BIT(3)
#define   AD9739A_CNT_CLK_DIS_REC_CNT_CLK		BIT(1)
#define   AD9739A_CNT_CLK_DIS_MU_CNT_CLK		BIT(0)

#define AD9739A_REG_IRQ_EN				0x03
#define AD9739A_REG_IRQ_REQ				0x04
#define   AD9739A_IRQ_EN_MU_LST				BIT(3)
#define   AD9739A_IRQ_EN_MU_LCK				BIT(2)
#define   AD9739A_IRQ_EN_RCV_LST			BIT(1)
#define   AD9739A_IRQ_EN_RCV_LCK			BIT(0)

#define AD9739A_REG_FSC_1				0x06
#define   AD9739A_FSC_1_FSC_1_MSK			GENMASK(7, 0)
#define   AD9739A_FSC_1_FSC_1_SET(x)			\
		FIELD_PREP(AD9739A_FSC_1_FSC_1_MSK, x)

#define AD9739A_REG_FSC_2				0x07
#define   AD9739A_FSC_2_FSC_2_MSK			GENMASK(1, 0)
#define   AD9739A_FSC_2_FSC_2_SET(x)			\
		FIELD_PREP(AD9739A_FSC_2_FSC_2_MSK, x)
#define   AD9739A_FSC_2_SLEEP				BIT(7)

#define AD9739A_REG_DEC_CNT				0x08
#define   AD9739A_DEC_CNT_DAC_DEC_MSK			GENMASK(1, 0)

enum op_mode {
	OP_NORMAL_BASEBAND = 0,
	OP_MIX_MODE = 2,
};

#define AD9739A_REG_LVDS_STAT1				0x0C
#define   AD9739A_LVDS_STAT1_SUP_HLD_EDGE1		BIT(7)
#define   AD9739A_LVDS_STAT1_DCI_PHS3			BIT(5)
#define   AD9739A_LVDS_STAT1_DCI_PHS1			BIT(4)
#define   AD9739A_LVDS_STAT1_DCI_PRE_PH2		BIT(3)
#define   AD9739A_LVDS_STAT1_DCI_PRE_PH0		BIT(2)
#define   AD9739A_LVDS_STAT1_DCI_PST_PH2		BIT(1)
#define   AD9739A_LVDS_STAT1_DCI_PST_PH0		BIT(0)

#define AD9739A_REG_LVDS_REC_CNT1			0x10
#define   AD9739A_LVDS_REC_CNT1_RCVR_FLG_RST		BIT(2)
#define   AD9739A_LVDS_REC_CNT1_RCVR_LOOP_ON		BIT(1)
#define   AD9739A_LVDS_REC_CNT1_RCVR_CNT_ENA		BIT(0)

#define AD9739A_REG_LVDS_REC_CNT2			0x11
#define   AD9739A_LVDS_REC_CNT2_SMP_DEL_MSK		GENMASK(7, 6)

#define AD9739A_REG_LVDS_REC_CNT3			0x12
#define   AD9739A_LVDS_REC_CNT3_SMP_DEL_MSK		GENMASK(7, 0)

#define AD9739A_REG_LVDS_REC_CNT4			0x13
#define   AD9739A_LVDS_REC_CNT4_DCI_DEL_MSK		GENMASK(7, 4)
#define   AD9739A_LVDS_REC_CNT4_DCI_DEL_SET(x)		\
		FIELD_PREP(AD9739A_LVDS_REC_CNT4_DCI_DEL_MSK, x)
#define   AD9739A_LVDS_REC_CNT4_FINE_DEL_SKEW_MSK	GENMASK(3, 0)
#define   AD9739A_LVDS_REC_CNT4_FINE_DEL_SKEW_SET(x)	\
		FIELD_PREP(AD9739A_LVDS_REC_CNT4_FINE_DEL_SKEW_MSK, x)

#define AD9739A_REG_LVDS_REC_CNT5			0x14
#define   AD9739A_LVDS_REC_CNT5_DCI_DEL_MSK		GENMASK(5, 0)

#define AD9739A_REG_LVDS_REC_STAT1			0x19
#define   AD9739A_LVDS_REC_STAT1_SMP_DEL_MSK		GENMASK(7, 6)

#define AD9739A_REG_LVDS_REC_STAT2			0x1A
#define   AD9739A_LVDS_REC_STAT2_SMP_DEL_MSK		GENMASK(7, 0)

#define AD9739A_REG_LVDS_REC_STAT3			0x1B
#define   AD9739A_LVDS_REC_STAT3_DCI_DEL_MSK		GENMASK(7, 6)

#define AD9739A_REG_LVDS_REC_STAT4			0x1C
#define AD9739A_LVDS_REC_STAT4_DCI_DEL_MSK		GENMASK&(7, 0)

#define AD9739A_REG_LVDS_REC_STAT9			0x21
#define   AD9739A_LVDS_REC_STAT9_RCVR_TRK_ON		BIT(3)
#define   AD9739A_LVDS_REC_STAT9_RCVR_FE_ON		BIT(2)
#define   AD9739A_LVDS_REC_STAT9_RCVR_LST		BIT(1)
#define   AD9739A_LVDS_REC_STAT9_RCVR_LCK		BIT(0)

#define AD9739A_REG_CROSS_CNT1				0x22
#define   AD9739A_CROSS_CNT1_DIR_P			BIT(4)
#define   AD9739A_CROSS_CNT1_CLKP_OFFSET_MSK		GENMASK(3, 0)
#define   AD9739A_CROSS_CNT1_CLKP_OFFSET_SET(x)		\
		FIELD_PREP(AD9739A_CROSS_CNT1_CLKP_OFFSET_MSK, x)

#define AD9739A_REG_CROSS_CNT2				0x23
#define   AD9739A_CROSS_CNT2_DIR_N			BIT(4)
#define   AD9739A_CROSS_CNT2_CLKN_OFFSET_MSK		GENMASK(3, 0)
#define   AD9739A_CROSS_CNT2_CLKN_OFFSET_SET(x)		\
		FIELD_PREP(AD9739A_CROSS_CNT2_CLKN_OFFSET_MSK, x)

#define AD9739A_REG_PHS_DET				0x24
#define   AD9739A_PHS_DET_CMP_BST			BIT(5)
#define   AD9739A_PHS_DET_PHS_DET_AUTO_EN		BIT(4)

#define AD9739A_REG_MU_DUTY				0x25
#define   AD9739A_MU_DUTY_MU_DUTY_AUTO_EN		BIT(7)
#define   AD9739A_MU_DUTY_POS_NEG			BIT(6)
#define   AD9739A_MU_DUTY_ADJ_MSK			GENMASK(5, 4)

#define AD9739A_REG_MU_CNT1				0x26
#define   AD9739A_MU_CNT1_SLOPE				BIT(6)
#define   AD9739A_MU_CNT1_MODE_MSK			GENMASK(5, 4)
#define   AD9739A_MU_CNT1_READ				BIT(3)
#define   AD9739A_MU_CNT1_GAIN_MSK			GENMASK(2, 1)
#define   AD9739A_MU_CNT1_GAIN_SET(x)			\
		FIELD_PREP(AD9739A_MU_CNT1_GAIN_MSK, x)
#define   AD9739A_MU_CNT1_ENABLE			BIT(0)

#define AD9739A_REG_MU_CNT2				0x27
#define   AD9739A_MU_CNT2_MUDEL				BIT(7)
#define   AD9739A_MU_CNT2_SRCH_MODE_MSK			GENMASK(6, 5)
#define   AD9739A_MU_CNT2_SRCH_MODE_SET(x)		\
		FIELD_PREP(AD9739A_MU_CNT2_SRCH_MODE_MSK, x)
#define   AD9739A_MU_CNT2_SET_PHS_MSK			GENMASK(4, 0)
#define   AD9739A_MU_CNT2_SET_PHS_SET(x)		\
		FIELD_PREP(AD9739A_MU_CNT2_SET_PHS_MSK, x)

#define AD9739A_REG_MU_CNT3				0x28
#define   AD9739A_MU_CNT3_MUDEL_MSK			GENMASK(7, 0)

#define AD9739A_REG_MU_CNT4				0x29
#define   AD9739A_MU_CNT4_SEARCH_TOL			BIT(7)
#define   AD9739A_MU_CNT4_RETRY				BIT(6)
#define   AD9739A_MU_CNT4_CONTRST			BIT(5)
#define   AD9739A_MU_CNT4_GUARD_MSK			GENMASK(4, 0)
#define   AD9739A_MU_CNT4_GUARD_SET(x)			\
		FIELD_PREP(AD9739A_MU_CNT4_GUARD_MSK, x)

#define AD9739A_REG_MU_STAT1				0x2A
#define   AD9739A_MU_STAT1_MU_LST			BIT(1)
#define   AD9739A_MU_STAT1_MU_LKD			BIT(0)

#define AD9739A_REG_PART_ID				0x35
#define AD9739A_REG_PART_ID_MSK				GENMASK(7, 0)

#define AD9737A_CHIP_ID					0x2C
#define AD9739A_CHIP_ID					0x24
#define AD9739A_MIN_FSC					8580	/* 8.58 mA */
#define AD9739A_MAX_FSC					31700	/* 31.6998 mA */

enum {
	ID_AD9737A,
	ID_AD9739A,
};

struct ad9739a_state {
	struct spi_device		*spi;
	struct clk			*clk;

	bool				mix_mode_en;
	u32				fsc_ua;

	struct gpio_desc		*reset_gpio;
};

static int ad9739a_read(struct spi_device *spi, unsigned int reg)
{
	u8 tbuf[1], rbuf[1];
	int ret;

	tbuf[0] = 0x80 | (0x7F & reg);

	ret = spi_write_then_read(spi,
				  tbuf, ARRAY_SIZE(tbuf),
				  rbuf, ARRAY_SIZE(rbuf));

	if (ret < 0)
		return ret;

	return rbuf[0];
}

static int ad9739a_write(struct spi_device *spi,
			 unsigned int reg,
			 unsigned char val)
{
	u8 buf[2];

	buf[0] = reg & 0x7F;
	buf[1] = val;

	return spi_write(spi, buf, ARRAY_SIZE(buf));
}

static const struct adi_axi_dac_chip_info ad9739a_chip_tbl[] = {
	[ID_AD9737A] = {
		.id = AD9737A_CHIP_ID,
		.num_dac_channels = 2,
		.num_buf_channels = 1,
	},
	[ID_AD9739A] = {
		.id = AD9739A_CHIP_ID,
		.num_dac_channels = 2,
		.num_buf_channels = 1,
	},
};

static int ad9739a_set_fsc(struct ad9739a_state *st, unsigned int fsc_ua)
{
	struct spi_device *spi = st->spi;
	unsigned int reg_val;
	int ret;

	fsc_ua = clamp_t(u16, fsc_ua, AD9739A_MIN_FSC, AD9739A_MAX_FSC);
	reg_val = (fsc_ua - AD9739A_MIN_FSC) * 10 / 226;

	ret = ad9739a_write(spi, AD9739A_REG_FSC_1,
			    AD9739A_FSC_1_FSC_1_SET(reg_val));
	if (ret < 0)
		return ret;

	ret = ad9739a_write(spi, AD9739A_REG_FSC_2,
			    AD9739A_FSC_2_FSC_2_SET(reg_val >> 8));
	if (ret < 0)
		return ret;

	st->fsc_ua = fsc_ua;

	return 0;
}

#if 0
static int ad9739a_get_fsc(struct cf_axi_converter *conv, u16 *fsc_ua)
{
	struct ad9739a_phy *phy = conv_to_phy(conv);

	*fsc_ua = phy->pdata->fsc_ua;

	return 0;
}
#endif

static int ad9739a_set_op_mode(struct ad9739a_state *st, bool on)
{
	struct spi_device *spi = st->spi;
	int ret;

	if (on)
		ret = ad9739a_write(spi, AD9739A_REG_DEC_CNT, OP_MIX_MODE);
	else
		ret = ad9739a_write(spi, AD9739A_REG_DEC_CNT,
				    OP_NORMAL_BASEBAND);

	if (ret < 0)
		return ret;

	st->mix_mode_en = on;

	return 0;
}

#if 0
static int ad9739a_get_op_mode(struct cf_axi_converter *conv, enum operation_mode *op_mode)
{
	struct ad9739a_phy *phy = conv_to_phy(conv);

	*op_mode = phy->pdata->mix_mode_en ? MIX_MODE_OPERATION : NORMAL_BASEBAND_OPERATION;

	return 0;
}

static unsigned long long ad9739a_get_data_clk(struct cf_axi_converter *conv)
{
	return clk_get_rate(conv->clk[CLK_DAC]);
}

static ssize_t ad9739a_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	int ret = 0;
	u16 fsc_ua;
	enum operation_mode op_mode;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case AD9739A_FSC:
		ret = ad9739a_get_fsc(conv, &fsc_ua);
		ret |= sprintf(buf, "%u\n", fsc_ua);
		break;
	case AD9739A_OP_MODE_AVAIL:
		ret = sprintf(buf, "%s\n", "normal-baseband mix-mode");
		break;
	case AD9739A_OP_MODE:
		ret = ad9739a_get_op_mode(conv, &op_mode);
		ret |= sprintf(buf, "%s\n", ad9739a_op_modes[op_mode]);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static ssize_t ad9739a_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	int ret = 0;
	u16 fsc_ua;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case AD9739A_FSC:
		ret = kstrtou16(buf, 10, &fsc_ua);
		if (ret < 0)
			break;
		ret = ad9739a_set_fsc(conv, fsc_ua);
		break;
	case AD9739A_OP_MODE:
		if (sysfs_streq(buf, "mix-mode"))
			ad9739a_set_op_mode(conv, true);
		else
			ad9739a_set_op_mode(conv, false);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(full_scale_current, S_IRUGO | S_IWUSR,
					ad9739a_show,
					ad9739a_store,
					AD9739A_FSC);

static IIO_DEVICE_ATTR(operation_modes_available, S_IRUGO,
					ad9739a_show,
					NULL,
					AD9739A_OP_MODE_AVAIL);

static IIO_DEVICE_ATTR(operation_mode, S_IRUGO | S_IWUSR,
					ad9739a_show,
					ad9739a_store,
					AD9739A_OP_MODE);

static struct attribute *ad9739a_attributes[] = {
	&iio_dev_attr_full_scale_current.dev_attr.attr,
	&iio_dev_attr_operation_modes_available.dev_attr.attr,
	&iio_dev_attr_operation_mode.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9739a_attribute_group = {
	.attrs = ad9739a_attributes,
};

#endif

static int ad9739a_preenable_setup(struct adi_axi_dac_conv *conv)
{
	struct ad9739a_state *st = adi_axi_dac_conv_priv(conv);
	struct spi_device *spi = st->spi;
	int ret, mask;

	/* Set FINE_DEL_SKEW to 2. */
	ret = ad9739a_write(spi, AD9739A_REG_LVDS_REC_CNT4,
			    AD9739A_LVDS_REC_CNT4_DCI_DEL_SET(0x7) |
			    AD9739A_LVDS_REC_CNT4_FINE_DEL_SKEW_SET(0x2));
	if (ret < 0)
		return ret;

	/* Disable the data Rx controller before enabling it. */
	ret = ad9739a_write(spi, AD9739A_REG_LVDS_REC_CNT1, 0x00);
	if (ret < 0)
		return ret;

	/* Enable the data Rx controller for loop and IRQ. */
	ret = ad9739a_write(spi, AD9739A_REG_LVDS_REC_CNT1,
			    AD9739A_LVDS_REC_CNT1_RCVR_LOOP_ON);
	if (ret < 0)
		return ret;
		
	/* Enable the data Rx controller for search and track mode. */
	ret = ad9739a_write(spi, AD9739A_REG_LVDS_REC_CNT1,
			    AD9739A_LVDS_REC_CNT1_RCVR_LOOP_ON |
			    AD9739A_LVDS_REC_CNT1_RCVR_CNT_ENA);
	if (ret < 0)
		return ret;

	mdelay(10);
		
	ret = ad9739a_read(spi, AD9739A_REG_LVDS_REC_STAT9);
	if (ret < 0)
		return ret;
		
	mask = AD9739A_LVDS_REC_STAT9_RCVR_TRK_ON |
		AD9739A_LVDS_REC_STAT9_RCVR_LCK;
	
	if ((ret & mask) != mask) {
		dev_err(&spi->dev, "Rx data lock failure\n");
		return -EFAULT;
	}

	ret = ad9739a_set_fsc(st, st->fsc_ua);
	if (ret < 0)
		return ret;

	ret = ad9739a_set_op_mode(st, st->mix_mode_en);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad9739a_setup(struct ad9739a_state *st)
{
	struct spi_device *spi = st->spi;
	int ret;

	/* Configure for the 4-wire SPI mode with MSB. */
	ret = ad9739a_write(spi, AD9739A_REG_MODE, 0x00);
	if (ret < 0)
		return ret;

	/* Software reset to default SPI values. */
	ad9739a_write(spi, AD9739A_REG_MODE, AD9739A_MODE_RESET);
	if (ret < 0)
		return ret;

	/* Clear the reset bit. */
	ad9739a_write(spi, AD9739A_REG_MODE, 0x00);
	if (ret < 0)
		return ret;

	/* Set the common-mode voltage of DACCLK_P and DACCLK_N inputs */
	ad9739a_write(spi, AD9739A_REG_CROSS_CNT1,
		      AD9739A_CROSS_CNT1_CLKP_OFFSET_SET(0xF));
	if (ret < 0)
		return ret;
	ad9739a_write(spi, AD9739A_REG_CROSS_CNT2,
		      AD9739A_CROSS_CNT2_CLKN_OFFSET_SET(0xF));
	if (ret < 0)
		return ret;

	/* Configure the Mu controller. */
	ret = ad9739a_write(spi, AD9739A_REG_PHS_DET,
			    AD9739A_PHS_DET_CMP_BST |
			    AD9739A_PHS_DET_PHS_DET_AUTO_EN);
	if (ret < 0)
		return ret;
	ret = ad9739a_write(spi, AD9739A_REG_MU_DUTY,
			    AD9739A_MU_DUTY_MU_DUTY_AUTO_EN);
	if (ret < 0)
		return ret;
	ret = ad9739a_write(spi, AD9739A_REG_MU_CNT2,
			    AD9739A_MU_CNT2_SRCH_MODE_SET(2) |
			    AD9739A_MU_CNT2_SET_PHS_SET(4));
	if (ret < 0)
		return ret;
	ret = ad9739a_write(spi, AD9739A_REG_MU_CNT3, 0x6C);
	if (ret < 0)
		return ret;

	ret = ad9739a_write(spi, AD9739A_REG_MU_CNT4,
			    AD9739A_MU_CNT4_SEARCH_TOL |
			    AD9739A_MU_CNT4_RETRY |
			    AD9739A_MU_CNT4_GUARD_SET(0xB));
	if (ret < 0)
		return ret;
	ret = ad9739a_write(spi, AD9739A_REG_MU_CNT1,
			    AD9739A_MU_CNT1_GAIN_SET(1));
	if (ret < 0)
		return ret;
	/* Enable the Mu controller search and track mode. */
	ret = ad9739a_write(spi, AD9739A_REG_MU_CNT1,
			    AD9739A_MU_CNT1_GAIN_SET(1) |
			    AD9739A_MU_CNT1_ENABLE);
	if (ret < 0)
		return ret;

	mdelay(10);

	ret = ad9739a_read(spi, AD9739A_REG_MU_STAT1);
	if (ret < 0)
		return ret;

	if ((ret & AD9739A_MU_STAT1_MU_LKD) != AD9739A_MU_STAT1_MU_LKD) {
		dev_err(&spi->dev, "Mu lock failure\n");
		return -EFAULT;
	}

	return 0;
}

static void ad9739a_clk_disable(void *data)
{
	struct ad9739a_state *st = data;

	clk_disable_unprepare(st->clk);
}

static const struct of_device_id ad9739a_of_match[] = {
	{ .compatible = "adi,ad9737a", .data = &ad9739a_chip_tbl[ID_AD9737A], },
	{ .compatible = "adi,ad9739a", .data = &ad9739a_chip_tbl[ID_AD9739A], },
	{}
};
MODULE_DEVICE_TABLE(of, ad9739a_of_match);

static void ad9739a_parse_dt(struct ad9739a_state *st, struct device *dev)
{
	u32 tmp;

	st->mix_mode_en = device_property_read_bool(dev,
						    "adi,mix-mode-enable");

	if (device_property_read_u32(dev, "adi,full-scale-current-ua", &tmp))
		st->fsc_ua = 20000;
	else
		st->fsc_ua = tmp;
}

static int ad9739a_probe(struct spi_device *spi)
{
	const struct adi_axi_dac_chip_info *info;
	struct adi_axi_dac_conv *conv;
	struct ad9739a_state *st;
	unsigned int id;
	int ret;

	info = of_device_get_match_data(&spi->dev);
	if (!info)
		return -ENODEV;

	conv = devm_adi_axi_dac_conv_register(&spi->dev, sizeof(*st));
	if (IS_ERR(conv))
		return PTR_ERR(conv);

	st = adi_axi_dac_conv_priv(conv);
	st->spi = spi;

	st->clk = devm_clk_get(&spi->dev, "dac-clk");
	if (IS_ERR(st->clk))
		return PTR_ERR(st->clk);

	ret = clk_prepare_enable(st->clk);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad9739a_clk_disable, st);
	if (ret)
		return ret;

	st->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset",
						 GPIOD_OUT_LOW);
	if (IS_ERR(st->reset_gpio))
		return PTR_ERR(st->reset_gpio);

	if (st->reset_gpio) {
		udelay(1);
		ret = gpiod_direction_output(st->reset_gpio, 1);
		mdelay(10);
	}

	spi_set_drvdata(spi, st);

	conv->chip_info = info;

	id = ad9739a_read(spi, AD9739A_REG_PART_ID);
	if (id != conv->chip_info->id) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", id);
		return -ENODEV;
	}

	ad9739a_parse_dt(st, &spi->dev);

	conv->preenable_setup = ad9739a_preenable_setup;

	return ad9739a_setup(st);
}

static struct spi_driver ad9739a_driver = {
	.driver = {
		.name = "ad9739a",
		.of_match_table = ad9739a_of_match,
	},
	.probe = ad9739a_probe,
};
module_spi_driver(ad9739a_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9739 DAC driver");
MODULE_LICENSE("GPL v2");
