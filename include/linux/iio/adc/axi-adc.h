/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Analog Devices Generic AXI ADC IP core driver/library
 * Link: https://wiki.analog.com/resources/fpga/docs/axi_adc_ip
 *
 * Copyright 2012-2020 Analog Devices Inc.
 */
#ifndef __AXI_ADC_H__
#define __AXI_ADC_H__

struct device;

struct axi_adc_chan_spec {
	struct iio_chan_spec		iio_chan;
	unsigned int			num_lanes;
};

struct axi_adc_chip_info {
	const char			*name;
	unsigned int			id;

	const struct axi_adc_chan_spec	*channels;
	unsigned int			num_channels;

	const unsigned int		(*scale_table)[2];
	int				num_scales;

	unsigned long			max_rate;
};

/**
 * struct axi_adc_conv - data of the ADC attached to the AXI ADC
 * @chip_info		chip info details for the client ADC
 * @preenable_setup	op to run in the client before enabling the AXI ADC
 * @reg_access		debugfs hook for accessing client regs via IIO debugfs
 * @read_raw		IIO read_raw hook for the client ADC
 * @write_raw		IIO write_raw hook for the client ADC
 */
struct axi_adc_conv {
	const struct axi_adc_chip_info		*chip_info;

	long long (*get_clk_rate)(struct axi_adc_conv *conv);
	int (*preenable_setup)(struct axi_adc_conv *conv);
	int (*reg_access)(struct axi_adc_conv *conv, unsigned int reg,
			  unsigned int writeval, unsigned int *readval);
	int (*read_raw)(struct axi_adc_conv *conv,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long mask);
	int (*write_raw)(struct axi_adc_conv *conv,
			 struct iio_chan_spec const *chan,
			 int val, int val2, long mask);
};

struct axi_adc_conv *axi_adc_conv_register(struct device *dev,
					   int sizeof_priv);
void axi_adc_conv_unregister(struct axi_adc_conv *conv);

struct axi_adc_conv *devm_axi_adc_conv_register(struct device *dev,
						int sizeof_priv);
void devm_axi_adc_conv_unregister(struct device *dev,
				  struct axi_adc_conv *conv);

void *axi_adc_conv_priv(struct axi_adc_conv *conv);

#endif
