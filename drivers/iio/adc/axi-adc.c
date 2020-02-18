// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices Generic AXI ADC IP core
 * Link: https://wiki.analog.com/resources/fpga/docs/axi_adc_ip
 *
 * Copyright 2012-2020 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h> /* FIXME: remove this; for now it's needed */
#include <linux/iio/buffer-dmaengine.h>

#include <linux/fpga/adi-axi-common.h>
#include <linux/iio/adc/axi-adc.h>

/**
 * Register definitions:
 *   https://wiki.analog.com/resources/fpga/docs/axi_adc_ip#register_map
 */

#define AXI_ADC_UPPER16_MSK		GENMASK(31, 16)
#define AXI_ADC_UPPER16_SET(x)		FIELD_PREP(AXI_ADC_UPPER16_MSK, x)
#define AXI_ADC_UPPER16_GET(x)		FIELD_GET(AXI_ADC_UPPER16_MSK, x)

#define AXI_ADC_LOWER16_MSK		GENMASK(15, 0)
#define AXI_ADC_LOWER16_SET(x)		FIELD_PREP(AXI_ADC_UPPER16_MSK, x)
#define AXI_ADC_LOWER16_GET(x)		FIELD_GET(AXI_ADC_LOWER16_MSK, x)

/* ADC controls */

#define AXI_ADC_REG_RSTN			0x0040
#define   AXI_ADC_MMCM_RSTN			BIT(1)
#define   AXI_ADC_RSTN				BIT(0)

#define AXI_ADC_REG_CNTRL			0x0044
#define   AXI_ADC_R1_MODE			BIT(2)
#define   AXI_ADC_DDR_EDGESEL			BIT(1)
#define   AXI_ADC_PIN_MODE			BIT(0)

#define AXI_ADC_REG_CLK_FREQ			0x0054
#define AXI_ADC_REG_CLK_RATIO			0x0058

#define AXI_ADC_REG_STATUS			0x005C
#define   AXI_ADC_MUX_PN_ERR			BIT(3)
#define   AXI_ADC_MUX_PN_OOS			BIT(2)
#define   AXI_ADC_MUX_OVER_RANGE		BIT(1)
#define   AXI_ADC_STATUS			BIT(0)

#define AXI_ADC_REG_DRP_CNTRL			0x0070
#define   AXI_ADC_DRP_SEL			BIT(29)
#define   AXI_ADC_DRP_RWN			BIT(28)
#define   AXI_ADC_DRP_ADDRESS_MSK		GENMASK(27, 16)
#define   AXI_ADC_DRP_ADDRESS_SET(x)		\
		FIELD_PREP(AXI_ADC_DRP_ADDRESS_MSK, x)
#define   AXI_ADC_DRP_ADDRESS_GET(x)		\
		FIELD_GET(AXI_ADC_DRP_ADDRESS_MSK, x)
#define   AXI_ADC_DRP_WDATA_SET			AXI_ADC_LOWER16_SET
#define   AXI_ADC_DRP_WDATA_GET			AXI_ADC_LOWER16_GET

#define AXI_REG_DRP_STATUS			0x0074
#define   AXI_ADC_DRP_STATUS			BIT(16)
#define   AXI_ADC_DRP_RDATA_SET			AXI_ADC_LOWER16_SET
#define   AXI_ADC_DRP_RDATA_GET			AXI_ADC_LOWER16_GET

#define AXI_ADC_REG_DMA_STATUS			0x0088
#define   AXI_ADC_DMA_OVF			BIT(2)
#define   AXI_ADC_DMA_UNF			BIT(1)
#define   AXI_ADC_DMA_STATUS			BIT(0)

#define ADI_REG_DMA_BUSWIDTH			0x008C
#define AXI_ADC_REG_GP_CONTROL			0x00BC
#define AXI_ADC_REG_ADC_DP_DISABLE		0x00C0

/* ADC Channel controls */

#define AXI_ADC_REG_CHAN_CNTRL(c)		(0x0400 + (c) * 0x40)
#define   AXI_ADC_PN_SEL			BIT(10)
#define   AXI_ADC_IQCOR_ENB			BIT(9)
#define   AXI_ADC_DCFILT_ENB			BIT(8)
#define   AXI_ADC_FORMAT_SIGNEXT		BIT(6)
#define   AXI_ADC_FORMAT_TYPE			BIT(5)
#define   AXI_ADC_FORMAT_ENABLE			BIT(4)
#define   AXI_ADC_PN23_TYPE			BIT(1)
#define   AXI_ADC_ENABLE			BIT(0)

#define AXI_ADC_REG_CHAN_STATUS(c)		(0x0404 + (c) * 0x40)
#define   AXI_ADC_PN_ERR			BIT(2)
#define   AXI_ADC_PN_OOS			BIT(1)
#define   AXI_ADC_OVER_RANGE			BIT(0)

#define AXI_ADC_REG_CHAN_CNTRL_1(c)		(0x0410 + (c) * 0x40)
#define   AXI_ADC_DCFILT_OFFSET_MSK		AXI_ADC_UPPER16_MSK
#define   AXI_ADC_DCFILT_OFFSET_SET		AXI_ADC_UPPER16_SET
#define   AXI_ADC_DCFILT_OFFSET_GET		AXI_ADC_UPPER16_GET
#define   AXI_ADC_DCFILT_COEFF_MSK		AXI_ADC_LOWER16_MSK
#define   AXI_ADC_DCFILT_COEFF_SET		AXI_ADC_LOWER16_SET
#define   AXI_ADC_DCFILT_COEFF_GET		AXI_ADC_LOWER16_GET

#define AXI_ADC_REG_CHAN_CNTRL_2(c)		(0x0414 + (c) * 0x40)
#define   AXI_ADC_IQCOR_COEFF_1_MSK		AXI_ADC_UPPER16_MSK
#define   AXI_ADC_IQCOR_COEFF_1_SET		AXI_ADC_UPPER16_SET
#define   AXI_ADC_IQCOR_COEFF_1_GET		AXI_ADC_UPPER16_GET
#define   AXI_ADC_IQCOR_COEFF_2_MSK		AXI_ADC_LOWER16_MSK
#define   AXI_ADC_IQCOR_COEFF_2_SET		AXI_ADC_LOWER16_SET
#define   AXI_ADC_IQCOR_COEFF_2_GET		AXI_ADC_LOWER16_GET

/*  format is 1.1.14 (sign, integer and fractional bits) */
#define AXI_ADC_IQCOR_INT_1			0x4000UL
#define AXI_ADC_IQCOR_SIGN_BIT			BIT(15)
/* The constant below is (2 * PI * 0x4000), where 0x4000 is AXI_ADC_IQCOR_INT_1 */
#define AXI_ADC_2_X_PI_X_INT_1			102944ULL

#define AXI_ADC_REG_CHAN_CNTRL_3(c)		(0x0418 + (c) * 0x40)
#define   AXI_ADC_ADC_PN_SEL_MSK		AXI_ADC_UPPER16_MSK
#define   AXI_ADC_ADC_PN_SEL_SET		AXI_ADC_UPPER16_SET
#define   AXI_ADC_ADC_PN_SEL_GET		AXI_ADC_UPPER16_GET
#define   AXI_ADC_ADC_DATA_SEL_MSK		AXI_ADC_LOWER16_MSK
#define   AXI_ADC_ADC_DATA_SEL_SET		AXI_ADC_LOWER16_SET
#define   AXI_ADC_ADC_DATA_SEL_GET		AXI_ADC_LOWER16_GET

#define AXI_ADC_REG_CHAN_USR_CNTRL_2(c)		(0x0424 + (c) * 0x40)
#define   AXI_ADC_USR_DECIMATION_M_MSK		AXI_ADC_UPPER16_MSK
#define   AXI_ADC_USR_DECIMATION_M_SET		AXI_ADC_UPPER16_SET
#define   AXI_ADC_USR_DECIMATION_M_GET		AXI_ADC_UPPER16_GET
#define   AXI_ADC_USR_DECIMATION_N_MSK		AXI_ADC_LOWER16_MSK
#define   AXI_ADC_USR_DECIMATION_N_SET		AXI_ADC_LOWER16_SET
#define   AXI_ADC_USR_DECIMATION_N_GET		AXI_ADC_LOWER16_GET

#define AXI_ADC_REG_DELAY(l)			(0x0800 + (l) * 0x4)

/* debugfs direct register access */
#define DEBUGFS_DRA_PCORE_REG_MAGIC		BIT(31)

struct axi_adc_core_info {
	unsigned int			version;
};

struct axi_adc_state {
	struct mutex			lock;

	struct axi_adc_client		*client;
	void __iomem			*regs;
	unsigned int			regs_size;
};

struct axi_adc_client {
	struct list_head		entry;
	struct axi_adc_conv		conv;
	struct axi_adc_state		*state;
	struct device			*dev;
	const struct axi_adc_core_info	*info;
};

static LIST_HEAD(axi_adc_registered_clients);
static DEFINE_MUTEX(axi_adc_registered_clients_lock);

static struct axi_adc_client *axi_adc_conv_to_client(struct axi_adc_conv *conv)
{
	if (!conv)
		return NULL;
	return container_of(conv, struct axi_adc_client, conv);
}

void *axi_adc_conv_priv(struct axi_adc_conv *conv)
{
	struct axi_adc_client *cl = axi_adc_conv_to_client(conv);

	if (!cl)
		return NULL;

	return (char *)cl + ALIGN(sizeof(struct axi_adc_client), IIO_ALIGN);
}
EXPORT_SYMBOL_GPL(axi_adc_conv_priv);

static void axi_adc_write(struct axi_adc_state *st, unsigned int reg,
			  unsigned int val)
{
	iowrite32(val, st->regs + reg);
}

static unsigned int axi_adc_read(struct axi_adc_state *st, unsigned int reg)
{
	return ioread32(st->regs + reg);
}

static void axi_adc_set_bits(struct axi_adc_state *st, unsigned int reg,
			     unsigned int msk)
{
	unsigned int val = axi_adc_read(st, reg);

	val |= msk;
	axi_adc_write(st, reg, val);
}

static void axi_adc_clr_bits(struct axi_adc_state *st,
			     unsigned int reg,
			     unsigned int msk)
{
	unsigned int val = axi_adc_read(st, reg);

	val &= ~msk;
	axi_adc_write(st, reg, val);
}

static void axi_adc_set_pnsel(struct axi_adc_state *st, int channel,
			      enum axi_adc_pn_sel sel)
{
	axi_adc_clr_bits(st, AXI_ADC_REG_CHAN_CNTRL_3(channel),
			 AXI_ADC_ADC_PN_SEL_MSK);

	axi_adc_set_bits(st, AXI_ADC_REG_CHAN_CNTRL_3(channel),
			 AXI_ADC_ADC_PN_SEL_SET(sel));
}

int axi_adc_lvds_idelay_calibrate(struct axi_adc_conv *conv)
{
	const struct axi_adc_chan_spec *chan;
	struct axi_adc_client *cl;
	struct axi_adc_state *st;
	int ret, val, cnt, start, max_start, max_cnt;
	unsigned int i, stat, inv_range = 0, do_inv, lane,
		 max_val = 31, nb_lanes;
	unsigned char errors[64];

	cl = axi_adc_conv_to_client(conv);
	if (!cl || !cl->state)
		return -ENODEV;

	if (!conv->pnsel_set)
		return -ENOTSUPP;

	st = cl->state;

	nb_lanes = 0;
	for (i = 0; i < conv->chip_info->num_channels; i++) {
		chan = &conv->chip_info->channels[i];
		axi_adc_clr_bits(st, AXI_ADC_REG_CHAN_CNTRL(i),
				 AXI_ADC_ENABLE);
		nb_lanes += chan->num_lanes;
	}

	do {
		if (inv_range)
			axi_adc_set_bits(st, AXI_ADC_REG_CNTRL,
					 AXI_ADC_DDR_EDGESEL);
		else
			axi_adc_clr_bits(st, AXI_ADC_REG_CNTRL,
					 AXI_ADC_DDR_EDGESEL);

		for (i = 0; i < conv->chip_info->num_channels; i++) {
			chan = &conv->chip_info->channels[i];

			ret = conv->pnsel_set(conv, i, chan->pnsel);
			if (ret)
				return ret;

			axi_adc_set_bits(st, AXI_ADC_REG_CHAN_CNTRL(i),
					 AXI_ADC_ENABLE);
			axi_adc_set_pnsel(st, i, chan->pnsel);
			axi_adc_write(st, AXI_ADC_REG_CHAN_STATUS(i), ~0);
		}

		for (val = 0; val < max_val; val++) {
			for (lane = 0; lane < nb_lanes; lane++)
				axi_adc_write(st, AXI_ADC_REG_DELAY(lane), val);

			for (i = 0; i < conv->chip_info->num_channels; i++)
				axi_adc_write(st,
					      AXI_ADC_REG_CHAN_STATUS(0), ~0);

			mdelay(1);

			stat = 0;
			for (i = 0; i < conv->chip_info->num_channels; i++)
				stat |= axi_adc_read(st,
						AXI_ADC_REG_CHAN_STATUS(i));

			stat = !!(stat & (AXI_ADC_PN_ERR | AXI_ADC_PN_OOS));
			errors[val + (inv_range * (max_val + 1))] = stat;
		}

		for (val = 0, cnt = 0, max_cnt = 0, start = -1, max_start = 0;
		     val <= (max_val + (inv_range * (max_val + 1))); val++) {
			if (errors[val] == 0) {
				if (start == -1)
					start = val;
				cnt++;
			} else {
				if (cnt > max_cnt) {
					max_cnt = cnt;
					max_start = start;
				}
				start = -1;
				cnt = 0;
			}
		}

		if (cnt > max_cnt) {
			max_cnt = cnt;
			max_start = start;
		}

		if ((inv_range == 0) &&
		    ((max_cnt < 3) || (errors[max_val] == 0))) {
			do_inv = 1;
			inv_range = 1;
		} else {
			do_inv = 0;
		}

	} while (do_inv);

	val = max_start + (max_cnt / 2);

	if (val > max_val) {
		val -= max_val + 1;
		axi_adc_set_bits(st, AXI_ADC_REG_CNTRL, AXI_ADC_DDR_EDGESEL);
	} else {
		axi_adc_clr_bits(st, AXI_ADC_REG_CNTRL, AXI_ADC_DDR_EDGESEL);
	}

	for (i = 0; i < conv->chip_info->num_channels; i++)
		conv->pnsel_set(conv, i, AXI_ADC_PN_OFF);

	for (lane = 0; lane < nb_lanes; lane++)
		axi_adc_write(st, AXI_ADC_REG_DELAY(lane), val);

	return 0;
}
EXPORT_SYMBOL_GPL(axi_adc_lvds_idelay_calibrate);

static int axi_adc_chan_to_regoffset(struct iio_chan_spec const *chan)
{
	if (chan->modified)
		return chan->scan_index;

	return chan->channel;
}

static int axi_adc_config_dma_buffer(struct device *dev,
				     struct iio_dev *indio_dev)
{
	struct iio_buffer *buffer;
	const char *dma_name;

	if (!device_property_present(dev, "dmas"))
		return 0;

	if (device_property_read_string(dev, "dma-names", &dma_name))
		dma_name = "rx";

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent,
						 dma_name);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	iio_device_attach_buffer(indio_dev, buffer);

	return 0;
}

static int axi_adc_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg, unsigned int writeval,
			      unsigned int *readval)
{
	struct axi_adc_state *st = iio_priv(indio_dev);
	int ret;

	/* Check that the register is in range and aligned */
	if ((reg & DEBUGFS_DRA_PCORE_REG_MAGIC) &&
	    ((reg & 0xffff) >= st->regs_size || (reg & 0x3)))
		return -EINVAL;

	mutex_lock(&st->lock);

	if (!(reg & DEBUGFS_DRA_PCORE_REG_MAGIC)) {
		struct axi_adc_conv *conv = &st->client->conv;

		if (!conv->reg_access)
			ret = -ENOSYS;
		else
			ret = conv->reg_access(conv, reg, writeval,
					       readval);
	} else {
		if (readval == NULL)
			axi_adc_write(st, reg & 0xFFFF, writeval);
		else
			*readval = axi_adc_read(st, reg & 0xFFFF);
		ret = 0;
	}
	mutex_unlock(&st->lock);

	return 0;
}

static int axi_adc_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct axi_adc_state *st = iio_priv(indio_dev);
	struct axi_adc_conv *conv = &st->client->conv;
	unsigned int tmp, phase = 0, channel;
	unsigned long long llval;
	long long clk_freq;
	int ret, sign;

	channel = axi_adc_chan_to_regoffset(chan);

	switch (m) {
	case IIO_CHAN_INFO_CALIBPHASE:
		phase = 1;
		/* fall-through */
	case IIO_CHAN_INFO_CALIBSCALE:
		tmp = axi_adc_read(st, AXI_ADC_REG_CHAN_CNTRL_2(channel));
		/*  format is 1.1.14 (sign, integer and fractional bits) */

		if (((phase + channel) % 2) == 0)
			tmp = AXI_ADC_IQCOR_COEFF_1_GET(tmp);
		else
			tmp = AXI_ADC_IQCOR_COEFF_2_GET(tmp);

		if (tmp & AXI_ADC_IQCOR_SIGN_BIT)
			sign = -1;
		else
			sign = 1;

		if (tmp & AXI_ADC_IQCOR_INT_1)
			*val = 1 * sign;
		else
			*val = 0;

		tmp &= ~(AXI_ADC_IQCOR_INT_1 | AXI_ADC_IQCOR_SIGN_BIT);

		llval = tmp * 1000000ULL + (AXI_ADC_IQCOR_INT_1 / 2);
		do_div(llval, AXI_ADC_IQCOR_INT_1);
		if (*val == 0)
			*val2 = llval * sign;
		else
			*val2 = llval;

		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_CALIBBIAS:
		tmp = axi_adc_read(st, AXI_ADC_REG_CHAN_CNTRL_1(channel));
		*val = AXI_ADC_DCFILT_OFFSET_GET(tmp);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
		/*
		 * approx: F_cut = C * Fsample / (2 * pi)
		 */

		tmp = axi_adc_read(st, AXI_ADC_REG_CHAN_CNTRL(channel));
		if (!(tmp & AXI_ADC_DCFILT_ENB)) {
			*val = 0;
			return IIO_VAL_INT;
		}

		if (!conv->get_clk_rate)
			return -ENOSYS;

		clk_freq = conv->get_clk_rate(conv);
		if (clk_freq < 0)
			return clk_freq;

		tmp = axi_adc_read(st, AXI_ADC_REG_CHAN_CNTRL_1(channel));
		llval = AXI_ADC_DCFILT_COEFF_GET(tmp) * clk_freq;
		do_div(llval, AXI_ADC_2_X_PI_X_INT_1);
		*val = llval;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (conv->read_raw) {
			ret = conv->read_raw(conv, chan, val, val2, m);
			if (ret)
				return ret;
		}

		tmp = axi_adc_read(st, AXI_ADC_REG_CLK_FREQ);
		llval = axi_adc_read(st, AXI_ADC_REG_CLK_RATIO);
		llval *= tmp * 100000000ULL;

		*val = llval >> 16;

		return IIO_VAL_INT;
	default:
		return conv->read_raw(conv, chan, val, val2, m);
	}

	return -EINVAL;
}

static int axi_adc_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct axi_adc_state *st = iio_priv(indio_dev);
	struct axi_adc_conv *conv = &st->client->conv;
	unsigned int fract, tmp, phase = 0, channel;
	unsigned long long llval;
	long long clk_freq;

	channel = axi_adc_chan_to_regoffset(chan);

	switch (mask) {
	case IIO_CHAN_INFO_CALIBPHASE:
		phase = 1;
		/* fall-through */
	case IIO_CHAN_INFO_CALIBSCALE:
		/*  format is 1.1.14 (sign, integer and fractional bits) */
		switch (val) {
		case 1:
			fract = AXI_ADC_IQCOR_INT_1;
			break;
		case -1:
			fract = AXI_ADC_IQCOR_INT_1 |
				AXI_ADC_IQCOR_SIGN_BIT;
			break;
		case 0:
			fract = 0;
			if (val2 < 0) {
				fract = AXI_ADC_IQCOR_SIGN_BIT;
				val2 *= -1;
			}
			break;
		default:
			return -EINVAL;
		}

		llval = (unsigned long long)val2 * AXI_ADC_IQCOR_INT_1;
		llval += (1000000UL / 2);
		do_div(llval, 1000000UL);
		fract |= llval;

		tmp = axi_adc_read(st, AXI_ADC_REG_CHAN_CNTRL_2(channel));

		if (((channel + phase) % 2) == 0) {
			tmp &= ~AXI_ADC_IQCOR_COEFF_1_MSK;
			tmp |= AXI_ADC_IQCOR_COEFF_1_SET(fract);
		} else {
			tmp &= ~AXI_ADC_IQCOR_COEFF_2_MSK;
			tmp |= AXI_ADC_IQCOR_COEFF_2_SET(fract);
		}

		axi_adc_write(st, AXI_ADC_REG_CHAN_CNTRL_2(channel), tmp);

		return 0;

	case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
		/* C = 1 – e^(-2 * pi * F_cut / Fsample)
		 * approx: C = 2 * pi * F_cut / Fsample
		 */

		tmp = axi_adc_read(st, AXI_ADC_REG_CHAN_CNTRL(channel));

		if (val == 0 && val2 == 0) {
			tmp &= ~AXI_ADC_DCFILT_ENB;
			axi_adc_write(st, AXI_ADC_REG_CHAN_CNTRL(channel), tmp);
			return 0;
		}

		if (!conv->get_clk_rate)
			return -ENOSYS;

		clk_freq = conv->get_clk_rate(conv);
		if (clk_freq < 0)
			return clk_freq;

		tmp |= AXI_ADC_DCFILT_ENB;

		llval = AXI_ADC_2_X_PI_X_INT_1 * val;
		do_div(llval, clk_freq);

		llval = clamp_t(unsigned short, llval, 1, AXI_ADC_IQCOR_INT_1);
		axi_adc_write(st, AXI_ADC_REG_CHAN_CNTRL_1(channel),
			      AXI_ADC_DCFILT_COEFF_SET(llval));
		axi_adc_write(st, AXI_ADC_REG_CHAN_CNTRL(channel), tmp);

		return 0;

	case IIO_CHAN_INFO_CALIBBIAS:
		tmp = axi_adc_read(st, AXI_ADC_REG_CHAN_CNTRL_1(channel));
		tmp &= ~AXI_ADC_DCFILT_OFFSET_MSK;
		tmp |= AXI_ADC_DCFILT_OFFSET_SET(val);
		axi_adc_write(st, AXI_ADC_REG_CHAN_CNTRL_1(channel), tmp);
		return 0;

	case IIO_CHAN_INFO_SAMP_FREQ:
		/* fall-through */
	default:
		if (!conv->write_raw)
			return -ENOSYS;

		return conv->write_raw(conv, chan, val, val2, mask);
	}
}

static int axi_adc_update_scan_mode(struct iio_dev *indio_dev,
				    const unsigned long *scan_mask)
{
	struct axi_adc_state *st = iio_priv(indio_dev);
	struct axi_adc_conv *conv = &st->client->conv;
	unsigned int i, ctrl;

	for (i = 0; i < conv->chip_info->num_channels; i++) {
		ctrl = axi_adc_read(st, AXI_ADC_REG_CHAN_CNTRL(i));

		if (test_bit(i, scan_mask))
			ctrl |= AXI_ADC_ENABLE;
		else
			ctrl &= ~AXI_ADC_ENABLE;

		axi_adc_write(st, AXI_ADC_REG_CHAN_CNTRL(i), ctrl);
	}

	return 0;
}

struct axi_adc_conv *axi_adc_conv_register(struct device *dev, int sizeof_priv)
{
	struct axi_adc_client *cl;
	size_t alloc_size;

	alloc_size = sizeof(struct axi_adc_client);
	if (sizeof_priv) {
		alloc_size = ALIGN(alloc_size, IIO_ALIGN);
		alloc_size += sizeof_priv;
	}
	alloc_size += IIO_ALIGN - 1;

	cl = kzalloc(alloc_size, GFP_KERNEL);
	if (!cl)
		return ERR_PTR(-ENOMEM);

	mutex_lock(&axi_adc_registered_clients_lock);

	get_device(dev);
	cl->dev = dev;

	list_add_tail(&cl->entry, &axi_adc_registered_clients);

	mutex_unlock(&axi_adc_registered_clients_lock);

	return &cl->conv;
}
EXPORT_SYMBOL_GPL(axi_adc_conv_register);

void axi_adc_conv_unregister(struct axi_adc_conv *conv)
{
	struct axi_adc_client *cl = axi_adc_conv_to_client(conv);

	if (!cl)
		return;

	mutex_lock(&axi_adc_registered_clients_lock);

	put_device(cl->dev);
	list_del(&cl->entry);
	kfree(cl);

	mutex_unlock(&axi_adc_registered_clients_lock);
}
EXPORT_SYMBOL(axi_adc_conv_unregister);

static void devm_axi_adc_conv_release(struct device *dev, void *res)
{
	axi_adc_conv_unregister(*(struct axi_adc_conv **)res);
}

static int devm_axi_adc_conv_match(struct device *dev, void *res, void *data)
{
	struct axi_adc_conv **r = res;

	return *r == data;
}

struct axi_adc_conv *devm_axi_adc_conv_register(struct device *dev,
						int sizeof_priv)
{
	struct axi_adc_conv **ptr, *conv;

	ptr = devres_alloc(devm_axi_adc_conv_release, sizeof(*ptr),
			   GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	conv = axi_adc_conv_register(dev, sizeof_priv);
	if (IS_ERR(conv)) {
		devres_free(ptr);
		return ERR_CAST(conv);
	}

	*ptr = conv;
	devres_add(dev, ptr);

	return conv;
}
EXPORT_SYMBOL_GPL(devm_axi_adc_conv_register);

void devm_axi_adc_conv_unregister(struct device *dev,
				  struct axi_adc_conv *conv)
{
	int rc;

	rc = devres_release(dev, devm_axi_adc_conv_release,
			    devm_axi_adc_conv_match, conv);
	WARN_ON(rc);
}
EXPORT_SYMBOL_GPL(devm_axi_adc_conv_unregister);

static ssize_t in_voltage_scale_available_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct axi_adc_state *st = iio_priv(indio_dev);
	struct axi_adc_conv *conv = &st->client->conv;
	size_t len = 0;
	int i;

	if (!conv->chip_info->num_scales) {
		buf[0] = '\n';
		return 1;
	}

	for (i = 0; i < conv->chip_info->num_scales; i++) {
		const unsigned int *s = conv->chip_info->scale_table[i];

		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "%u.%06u ", s[0], s[1]);
	}
	buf[len - 1] = '\n';

	return len;
}

static IIO_DEVICE_ATTR_RO(in_voltage_scale_available, 0);

static struct attribute *axi_adc_attributes[] = {
	&iio_dev_attr_in_voltage_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group axi_adc_attribute_group = {
	.attrs = axi_adc_attributes,
};

static const struct iio_info axi_adc_info = {
	.read_raw = &axi_adc_read_raw,
	.write_raw = &axi_adc_write_raw,
	.debugfs_reg_access = &axi_adc_reg_access,
	.attrs = &axi_adc_attribute_group,
	.update_scan_mode = &axi_adc_update_scan_mode,
};

static const struct axi_adc_core_info axi_adc_10_0_a_info = {
	.version = ADI_AXI_PCORE_VER(10, 0, 'a'),
};

/* Match table for of_platform binding */
static const struct of_device_id axi_adc_of_match[] = {
	{ .compatible = "adi,axi-adc-10.0.a", .data = &axi_adc_10_0_a_info },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, axi_adc_of_match);

struct axi_adc_client *axi_adc_attach_client(struct device *dev)
{
	const struct of_device_id *id;
	struct axi_adc_client *cl;
	struct device_node *cln;

	if (!dev->of_node) {
		dev_err(dev, "DT node is null\n");
		return ERR_PTR(-ENODEV);
	}

	id = of_match_node(axi_adc_of_match, dev->of_node);
	if (!id)
		return ERR_PTR(-ENODEV);

	cln = of_parse_phandle(dev->of_node, "axi-adc-client", 0);
	if (!cln) {
		dev_err(dev, "No 'axi-adc-client' node defined\n");
		return ERR_PTR(-ENODEV);
	}

	mutex_lock(&axi_adc_registered_clients_lock);

	list_for_each_entry(cl, &axi_adc_registered_clients, entry) {
		if (!cl->dev)
			continue;
		if (cl->dev->of_node == cln) {
			if (!try_module_get(dev->driver->owner)) {
				mutex_unlock(&axi_adc_registered_clients_lock);
				return ERR_PTR(-ENODEV);
			}
			get_device(dev);
			cl->info = id->data;
			mutex_unlock(&axi_adc_registered_clients_lock);
			return cl;
		}
	}

	mutex_unlock(&axi_adc_registered_clients_lock);

	return ERR_PTR(-EPROBE_DEFER);
}

static int axi_adc_setup_channels(struct device *dev, struct axi_adc_state *st)
{
	struct axi_adc_conv *conv = conv = &st->client->conv;
	unsigned int val;
	int i, ret;

	if (conv->preenable_setup) {
		ret = conv->preenable_setup(conv);
		if (ret)
			return ret;
	}

	for (i = 0; i < conv->chip_info->num_channels; i++) {
		if (i & 1)
			val = AXI_ADC_IQCOR_COEFF_2_SET(AXI_ADC_IQCOR_INT_1);
		else
			val = AXI_ADC_IQCOR_COEFF_1_SET(AXI_ADC_IQCOR_INT_1);
		axi_adc_write(st, AXI_ADC_REG_CHAN_CNTRL_2(i), val);

		axi_adc_write(st, AXI_ADC_REG_CHAN_CNTRL(i),
			      AXI_ADC_FORMAT_SIGNEXT | AXI_ADC_FORMAT_ENABLE |
			      AXI_ADC_IQCOR_ENB | AXI_ADC_ENABLE);
	}

	return 0;
}

static int axi_adc_alloc_channels(struct iio_dev *indio_dev,
				  struct axi_adc_conv *conv)
{
	unsigned int i, num = conv->chip_info->num_channels;
	struct device *dev = indio_dev->dev.parent;
	struct iio_chan_spec *channels;

	channels = devm_kcalloc(dev, num, sizeof(*channels), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	for (i = 0; i < conv->chip_info->num_channels; i++)
		channels[i] = conv->chip_info->channels->iio_chan;

	indio_dev->num_channels = num;
	indio_dev->channels = channels;

	return 0;
}

struct axi_adc_cleanup_data {
	struct axi_adc_state	*st;
	struct axi_adc_client	*cl;
};

static void axi_adc_cleanup(void *data)
{
	struct axi_adc_client *cl = data;

	put_device(cl->dev);
	module_put(cl->dev->driver->owner);
}

static int axi_adc_probe(struct platform_device *pdev)
{
	struct axi_adc_conv *conv;
	struct iio_dev *indio_dev;
	struct axi_adc_client *cl;
	struct axi_adc_state *st;
	struct resource *mem;
	unsigned int ver;
	int ret;

	cl = axi_adc_attach_client(&pdev->dev);
	if (IS_ERR(cl))
		return PTR_ERR(cl);

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->client = cl;
	cl->state = st;
	mutex_init(&st->lock);

	ret = devm_add_action_or_reset(&pdev->dev, axi_adc_cleanup, cl);
	if (ret)
		return ret;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs_size = resource_size(mem);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	conv = &st->client->conv;

	/* Reset HDL Core */
	axi_adc_write(st, AXI_ADC_REG_RSTN, 0);
	mdelay(10);
	axi_adc_write(st, AXI_ADC_REG_RSTN, AXI_ADC_MMCM_RSTN);
	mdelay(10);
	axi_adc_write(st, AXI_ADC_REG_RSTN, AXI_ADC_RSTN | AXI_ADC_MMCM_RSTN);

	ver = axi_adc_read(st, ADI_AXI_REG_VERSION);

	if (cl->info->version > ver) {
		dev_err(&pdev->dev,
			"IP core version is too old. Expected %d.%.2d.%c, Reported %d.%.2d.%c\n",
			ADI_AXI_PCORE_VER_MAJOR(cl->info->version),
			ADI_AXI_PCORE_VER_MINOR(cl->info->version),
			ADI_AXI_PCORE_VER_PATCH(cl->info->version),
			ADI_AXI_PCORE_VER_MAJOR(ver),
			ADI_AXI_PCORE_VER_MINOR(ver),
			ADI_AXI_PCORE_VER_PATCH(ver));
		return -ENODEV;
	}

	indio_dev->info = &axi_adc_info;
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = axi_adc_alloc_channels(indio_dev, conv);
	if (ret)
		return ret;

	ret = axi_adc_config_dma_buffer(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	ret = axi_adc_setup_channels(&pdev->dev, st);
	if (ret)
		return ret;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "AXI ADC IP core (%d.%.2d.%c) probed\n",
		ADI_AXI_PCORE_VER_MAJOR(ver),
		ADI_AXI_PCORE_VER_MINOR(ver),
		ADI_AXI_PCORE_VER_PATCH(ver));

	return 0;
}

static struct platform_driver axi_adc_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = axi_adc_of_match,
	},
	.probe = axi_adc_probe,
};

module_platform_driver(axi_adc_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices Generic AXI ADC IP core driver");
MODULE_LICENSE("GPL v2");
