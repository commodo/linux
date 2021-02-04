// SPDX-License-Identifier: GPL-2.0
/*
 * AXI ADC Trigger ADC driver for ADALM-2000
 *
 * Copyright 2015-2021 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define M2K_TRIGGER_ADC_REG_VERSION			0x00
#define M2K_TRIGGER_ADC_REG_SCRATH			0x04

#define M2K_TRIGGER_ADC_REG_TRIGGER_O			0x08
#define M2K_TRIGGER_ADC_REG_IO_SELECTION		0x0C
#define M2K_TRIGGER_ADC_REG_CONFIG_TRIGGER		0x10

#define M2K_TRIGGER_ADC_REG_LIMIT(x)			(0x14 + ((x) * 0x10))
#define M2K_TRIGGER_ADC_REG_FUNCTION(x)			(0x18 + ((x) * 0x10))
#define M2K_TRIGGER_ADC_REG_HYSTERESIS(x)		(0x1C + ((x) * 0x10))
#define M2K_TRIGGER_ADC_REG_TRIGGER_MIX(x)		(0x20 + ((x) * 0x10))
#define M2K_TRIGGER_ADC_REG_TRIGGER_OUT_CTRL		0x34
#define M2K_TRIGGER_ADC_REG_FIFO_DEPTH			0x38
#define M2K_TRIGGER_ADC_REG_TRIGGERED			0x3c
#define M2K_TRIGGER_ADC_REG_DELAY			0x40
#define M2K_TRIGGER_ADC_REG_STREAMING			0x44
#define M2K_TRIGGER_ADC_REG_HOLDOFF			0x48

/* M2K_TRIGGER_ADC_REG_CONFIG_TRIGGER */
#define CONF_LOW_LEVEL					0
#define CONF_HIGH_LEVEL					1
#define CONF_ANY_EDGE					2
#define CONF_RISING_EDGE				3
#define CONF_FALLING_EDGE				4

#define CONFIG_PIN_TRIGGER(t0_conf, t1_conf)	\
		((1 << ((t0_conf) * 2)) | (2 << ((t1_conf) * 2)))
#define CONFIG_IO_SELECTION(pin_nr, val)	\
		((val & 0x7) << (3 * pin_nr + 2))

#define TRIGGER_PIN_CHAN				2
#define TRIGGER_ADC_CHAN				2
#define TRIGGER_MIX_CHAN				2

#define TRIGGER_HOLDOFF_MASK				GENMASK(31, 0)

#define IIO_ENUM_AVAILABLE_SEPARATE(_name, _e) \
{ \
	.name = (_name "_available"), \
	.shared = IIO_SEPARATE, \
	.read = iio_enum_available_read, \
	.private = (uintptr_t)(_e), \
}

enum trig_ext_info {
	TRIG_LEVEL,
	TRIG_HYST,
	TRIG_DELAY,
	TRIG_ITEMS
};

struct m2k_trigger_adc_state {
	void __iomem *regs;
	struct clk *clk;
	struct mutex lock;
	unsigned int trigger_pin_config[TRIGGER_PIN_CHAN];
	int trigger_ext_info[TRIG_ITEMS][TRIGGER_ADC_CHAN];
};

static void m2k_trigger_adc_write(struct m2k_trigger_adc_state *st,
				  unsigned int reg, unsigned int val)
{
	writel_relaxed(val, st->regs + reg);
}

static unsigned int m2k_trigger_adc_read(struct m2k_trigger_adc_state *st,
					 unsigned int reg)
{
	return readl_relaxed(st->regs + reg);
}

static int m2k_trigger_adc_reg_access(struct iio_dev *indio_dev,
				      unsigned int reg, unsigned int writeval,
				      unsigned int *readval)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);
	if (readval == NULL)
		m2k_trigger_adc_write(st, reg & 0xFF, writeval);
	else
		*readval = m2k_trigger_adc_read(st, reg & 0xFF);

	mutex_unlock(&st->lock);

	return 0;
}

static int m2k_trigger_adc_read_raw(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    int *val, int *val2, long info)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = clk_get_rate(st->clk);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

/* Trigger_O selection */

static int m2k_trigger_adc_set_out_pin_select(struct iio_dev *indio_dev,
					      const struct iio_chan_spec *chan,
					      unsigned int val)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);
	unsigned int mask;

	mutex_lock(&st->lock);

	mask = m2k_trigger_adc_read(st, M2K_TRIGGER_ADC_REG_IO_SELECTION);
	mask &= ~GENMASK((3 * chan->address + 2) + 2, (3 * chan->address + 2));
	val = mask | CONFIG_IO_SELECTION(chan->address, val);

	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_IO_SELECTION, val);

	mutex_unlock(&st->lock);

	return 0;
}

static int m2k_trigger_adc_get_out_pin_select(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);
	unsigned int val, mask;

	mask = GENMASK((3 * chan->address + 2) + 2, (3 * chan->address + 2));
	val = m2k_trigger_adc_read(st, M2K_TRIGGER_ADC_REG_IO_SELECTION) & mask;

	return val >> (3 * chan->address + 2);
}

static const char * const m2k_trigger_adc_out_pin_select_items[] = {
	"sw-trigger",
	"trigger-i-same-chan",
	"trigger-i-swap-chan",
	"trigger-adc",
	"trigger-in",
};

static const struct iio_enum m2k_trigger_adc_out_pin_select_enum = {
	.items = m2k_trigger_adc_out_pin_select_items,
	.num_items = ARRAY_SIZE(m2k_trigger_adc_out_pin_select_items),
	.set = m2k_trigger_adc_set_out_pin_select,
	.get = m2k_trigger_adc_get_out_pin_select,
};

static int m2k_trigger_adc_set_io_direction(struct iio_dev *indio_dev,
					    const struct iio_chan_spec *chan,
					    unsigned int val)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);
	unsigned int mask;

	mutex_lock(&st->lock);

	mask = m2k_trigger_adc_read(st, M2K_TRIGGER_ADC_REG_IO_SELECTION);
	mask &= ~BIT(chan->address);

	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_IO_SELECTION,
			      mask | (val << chan->address));

	mutex_unlock(&st->lock);

	return 0;
}

static int m2k_trigger_adc_get_io_direction(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);
	unsigned int val, mask;

	mask = BIT(chan->address);
	val = m2k_trigger_adc_read(st, M2K_TRIGGER_ADC_REG_IO_SELECTION) & mask;

	return val >> chan->address;
}

static const char * const m2k_trigger_adc_io_direction_items[] = {
	"out",
	"in",
};

static const struct iio_enum m2k_trigger_adc_io_direction_enum = {
	.items = m2k_trigger_adc_io_direction_items,
	.num_items = ARRAY_SIZE(m2k_trigger_adc_io_direction_items),
	.set = m2k_trigger_adc_set_io_direction,
	.get = m2k_trigger_adc_get_io_direction,
};

/* PIN/Digital trigger */

static int m2k_trigger_adc_pin_set_trigger(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan,
					   unsigned int val)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);
	st->trigger_pin_config[chan->address] = val;

	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_CONFIG_TRIGGER,
			      CONFIG_PIN_TRIGGER(st->trigger_pin_config[0],
			      st->trigger_pin_config[1]));
	mutex_unlock(&st->lock);

	return 0;
}

static int m2k_trigger_adc_pin_get_trigger(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);

	return st->trigger_pin_config[chan->address];
}

static const char * const m2k_trigger_adc_pin_trigger_items[] = {
	"level-low",
	"level-high",
	"edge-any",
	"edge-rising",
	"edge-falling",
};

static const struct iio_enum m2k_trigger_adc_trigger_pin_enum = {
	.items = m2k_trigger_adc_pin_trigger_items,
	.num_items = ARRAY_SIZE(m2k_trigger_adc_pin_trigger_items),
	.set = m2k_trigger_adc_pin_set_trigger,
	.get = m2k_trigger_adc_pin_get_trigger,
};

/* Analog ADC trigger */

static int m2k_trigger_adc_analog_set_trigger(struct iio_dev *indio_dev,
					      const struct iio_chan_spec *chan,
					      unsigned int val)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);
	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_FUNCTION(chan->address), val);
	mutex_unlock(&st->lock);

	return 0;
}

static int m2k_trigger_adc_analog_get_trigger(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);

	return m2k_trigger_adc_read(st, M2K_TRIGGER_ADC_REG_FUNCTION(chan->address));
}

static const char * const m2k_trigger_adc_analog_trigger_items[] = {
	"level-low",
	"level-high",
	"edge-rising",
	"edge-falling",
};

static const struct iio_enum m2k_trigger_adc_trigger_analog_enum = {
	.items = m2k_trigger_adc_analog_trigger_items,
	.num_items = ARRAY_SIZE(m2k_trigger_adc_analog_trigger_items),
	.set = m2k_trigger_adc_analog_set_trigger,
	.get = m2k_trigger_adc_analog_get_trigger,
};

/* Trigger MIX */

static int m2k_trigger_adc_set_trigger_mix_mode(struct iio_dev *indio_dev,
						const struct iio_chan_spec *chan,
						unsigned int val)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);
	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_TRIGGER_MIX(chan->address), val);
	mutex_unlock(&st->lock);

	return 0;
}

static int m2k_trigger_adc_get_trigger_mix_mode(struct iio_dev *indio_dev,
						const struct iio_chan_spec *chan)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);

	return m2k_trigger_adc_read(st, M2K_TRIGGER_ADC_REG_TRIGGER_MIX(chan->address));
}

static const char * const m2k_trigger_adc_trigger_mix_mode_items[] = {
	"always",
	"digital",
	"analog",
	NULL,
	"digital_OR_analog",
	"digital_AND_analog",
	"digital_XOR_analog",
	"!digital_OR_analog",
	"!digital_AND_analog",
	"!digital_XOR_analog",
};

static const struct iio_enum m2k_trigger_adc_trigger_mix_mode_enum = {
	.items = m2k_trigger_adc_trigger_mix_mode_items,
	.num_items = ARRAY_SIZE(m2k_trigger_adc_trigger_mix_mode_items),
	.set = m2k_trigger_adc_set_trigger_mix_mode,
	.get = m2k_trigger_adc_get_trigger_mix_mode,
};

/* Trigger OUT MIX */

static int m2k_trigger_adc_set_trigger_out_mix_mode(struct iio_dev *indio_dev,
						    const struct iio_chan_spec *chan,
						    unsigned int val)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);
	unsigned int mask;

	mutex_lock(&st->lock);

	mask = m2k_trigger_adc_read(st, M2K_TRIGGER_ADC_REG_TRIGGER_OUT_CTRL);
	mask &= ~GENMASK(3, 0);
	val = mask | val;

	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_TRIGGER_OUT_CTRL, val);

	mutex_unlock(&st->lock);

	return 0;
}

static int m2k_trigger_adc_get_trigger_out_mix_mode(struct iio_dev *indio_dev,
						    const struct iio_chan_spec *chan)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);

	return m2k_trigger_adc_read(st, M2K_TRIGGER_ADC_REG_TRIGGER_OUT_CTRL) & GENMASK(3, 0);

}

static const char * const m2k_trigger_adc_trigger_out_mix_mode_items[] = {
	"a",
	"b",
	"a_OR_b",
	"a_AND_b",
	"a_XOR_b",
	"trigger_in",
	"a_OR_trigger_in",
	"b_OR_trigger_in",
	"a_OR_b_OR_trigger_in",
	"disabled"
};

static const struct iio_enum m2k_trigger_adc_trigger_out_mix_mode_enum = {
	.items = m2k_trigger_adc_trigger_out_mix_mode_items,
	.num_items = ARRAY_SIZE(m2k_trigger_adc_trigger_out_mix_mode_items),
	.set = m2k_trigger_adc_set_trigger_out_mix_mode,
	.get = m2k_trigger_adc_get_trigger_out_mix_mode,
};

static ssize_t m2k_trigger_adc_set_extinfo(struct iio_dev *indio_dev,
					   uintptr_t priv,
					   const struct iio_chan_spec *chan,
					   const char *buf,
					   size_t len)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);
	int ret, val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0)
		return ret;

	mutex_lock(&st->lock);

	switch (priv) {
	case TRIG_DELAY:
		if (val < -8192) {
			ret = -EINVAL;
			goto out_unlock;
		}

		if (val < 0) {
			m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_FIFO_DEPTH, -val);
			m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_DELAY, 0);
		} else {
			m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_FIFO_DEPTH, 0);
			m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_DELAY, val);
		}

		st->trigger_ext_info[priv][chan->address] = val;
		break;
	case TRIG_LEVEL:
		/* FIXME add check for lvl + hyst */
		st->trigger_ext_info[priv][chan->address] = val;
		m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_LIMIT(chan->address), val);
		break;
	case TRIG_HYST:
		/* FIXME add check for lvl + hyst */
		st->trigger_ext_info[priv][chan->address] = val;
		m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_HYSTERESIS(chan->address), val);
		break;
	default:
		ret = -EINVAL;
	}

out_unlock:
	mutex_unlock(&st->lock);

	return len;
}

static ssize_t m2k_trigger_adc_get_extinfo(struct iio_dev *indio_dev,
					   uintptr_t priv,
					   const struct iio_chan_spec *chan,
					   char *buf)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", st->trigger_ext_info[priv][chan->address]);
}

static ssize_t m2k_trigger_adc_get_triggered(struct iio_dev *indio_dev,
					     uintptr_t priv,
					     const struct iio_chan_spec *chan,
					     char *buf)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);
	unsigned int val;

	val = m2k_trigger_adc_read(st, M2K_TRIGGER_ADC_REG_TRIGGERED) & 1;

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t m2k_trigger_adc_set_triggered(struct iio_dev *indio_dev,
					     uintptr_t priv,
					     const struct iio_chan_spec *chan, const char *buf,
	size_t len)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret < 0)
		return ret;

	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_TRIGGERED, val);

	return len;
}

static ssize_t m2k_trigger_adc_get_streaming(struct iio_dev *indio_dev,
					     uintptr_t priv,
					     const struct iio_chan_spec *chan,
					     char *buf)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);
	unsigned int val;

	val = m2k_trigger_adc_read(st, M2K_TRIGGER_ADC_REG_STREAMING) & 1;

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t m2k_trigger_adc_set_streaming(struct iio_dev *indio_dev,
					     uintptr_t priv,
					     const struct iio_chan_spec *chan,
					     const char *buf,
					     size_t len)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret < 0)
		return ret;

	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_STREAMING, val);

	return len;
}

static ssize_t m2k_trigger_adc_get_holdoff(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, char *buf)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);
	unsigned int val;

	val = m2k_trigger_adc_read(st, M2K_TRIGGER_ADC_REG_HOLDOFF);
	val &= TRIGGER_HOLDOFF_MASK;

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t m2k_trigger_adc_set_holdoff(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, const char *buf,
	size_t len)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret < 0)
		return ret;

	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_HOLDOFF, val);

	return len;
}

static ssize_t m2k_trigger_adc_get_embedded_trigger(struct iio_dev *indio_dev,
						    uintptr_t priv,
						    const struct iio_chan_spec *chan,
						    char *buf)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);
	unsigned int val;

	val = m2k_trigger_adc_read(st, M2K_TRIGGER_ADC_REG_TRIGGER_OUT_CTRL) >> 16;

	return scnprintf(buf, PAGE_SIZE, "%d\n", val & 1);
}

static ssize_t m2k_trigger_adc_set_embedded_trigger(struct iio_dev *indio_dev,
						    uintptr_t priv,
						    const struct iio_chan_spec *chan,
						    const char *buf,
						    size_t len)
{
	struct m2k_trigger_adc_state *st = iio_priv(indio_dev);
	unsigned int val, mask;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret < 0)
		return ret;

	mask = m2k_trigger_adc_read(st, M2K_TRIGGER_ADC_REG_TRIGGER_OUT_CTRL);

	if (val)
		mask |= BIT(16);
	else
		mask &= ~BIT(16);

	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_TRIGGER_OUT_CTRL, mask);

	return len;
}

static const struct iio_chan_spec_ext_info m2k_trigger_adc_analog_info[] = {
	IIO_ENUM("trigger", IIO_SEPARATE, &m2k_trigger_adc_trigger_analog_enum),
	IIO_ENUM_AVAILABLE_SEPARATE("trigger",
				    &m2k_trigger_adc_trigger_analog_enum),
	{
		.name = "trigger_level",
		.shared = IIO_SEPARATE,
		.write = m2k_trigger_adc_set_extinfo,
		.read = m2k_trigger_adc_get_extinfo,
		.private = TRIG_LEVEL,
	},
	{
		.name = "trigger_hysteresis",
		.shared = IIO_SEPARATE,
		.write = m2k_trigger_adc_set_extinfo,
		.read = m2k_trigger_adc_get_extinfo,
		.private = TRIG_HYST,
	},
	{
		.name = "triggered",
		.shared = IIO_SHARED_BY_ALL,
		.write = m2k_trigger_adc_set_triggered,
		.read = m2k_trigger_adc_get_triggered,
	},
	{
		.name = "streaming",
		.shared = IIO_SHARED_BY_ALL,
		.write = m2k_trigger_adc_set_streaming,
		.read = m2k_trigger_adc_get_streaming,
	},
	{
		.name = "holdoff_raw",
		.shared = IIO_SHARED_BY_ALL,
		.write = m2k_trigger_adc_set_holdoff,
		.read = m2k_trigger_adc_get_holdoff,
	},
	{}
};

static const struct iio_chan_spec_ext_info m2k_trigger_adc_digital_info[] = {
	IIO_ENUM("trigger", IIO_SEPARATE, &m2k_trigger_adc_trigger_pin_enum),
	IIO_ENUM_AVAILABLE_SEPARATE("trigger", &m2k_trigger_adc_trigger_pin_enum),
	{}
};

static const struct iio_chan_spec_ext_info m2k_trigger_adc_adc_pin_mix[] = {
	IIO_ENUM("trigger_logic_mode", IIO_SEPARATE,
		&m2k_trigger_adc_trigger_mix_mode_enum),
	IIO_ENUM_AVAILABLE_SEPARATE("trigger_logic_mode",
		&m2k_trigger_adc_trigger_mix_mode_enum),
	IIO_ENUM("trigger_logic_out_select", IIO_SEPARATE,
		&m2k_trigger_adc_out_pin_select_enum),
	IIO_ENUM_AVAILABLE_SEPARATE("trigger_logic_out_select",
		&m2k_trigger_adc_out_pin_select_enum),
	IIO_ENUM("trigger_logic_out_direction", IIO_SEPARATE,
		&m2k_trigger_adc_io_direction_enum),
	IIO_ENUM_AVAILABLE_SEPARATE("trigger_logic_out_direction",
		&m2k_trigger_adc_io_direction_enum),
	{}
};

static const struct iio_chan_spec_ext_info m2k_trigger_adc_a_b_mix[] = {
	IIO_ENUM("trigger_logic_mode", IIO_SEPARATE,
		&m2k_trigger_adc_trigger_out_mix_mode_enum),
	IIO_ENUM_AVAILABLE_SEPARATE("trigger_logic_mode",
		&m2k_trigger_adc_trigger_out_mix_mode_enum),
	{
		.name = "trigger_delay",
		.shared = IIO_SEPARATE,
		.write = m2k_trigger_adc_set_extinfo,
		.read = m2k_trigger_adc_get_extinfo,
		.private = TRIG_DELAY,
	},
	{
		.name = "trigger_embedded",
		.shared = IIO_SEPARATE,
		.write = m2k_trigger_adc_set_embedded_trigger,
		.read = m2k_trigger_adc_get_embedded_trigger,
	},
	{}
};

#define AXI_TRIG_CHAN_ADC(x, a, n) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.address = (a), \
	.info_mask_separate = 0, \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = 0, \
	.ext_info = m2k_trigger_adc_analog_info, \
	.extend_name = n,\
}

#define AXI_TRIG_CHAN_PIN(x, a, n) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.address = (a), \
	.info_mask_separate = 0, \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = 0, \
	.ext_info = m2k_trigger_adc_digital_info, \
	.extend_name = n,\
}

#define AXI_TRIG_CHAN_ADC_X_PIN(x, a, n) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.address = (a), \
	.info_mask_separate = 0, \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = 0, \
	.ext_info = m2k_trigger_adc_adc_pin_mix, \
	.extend_name = n,\
}

#define AXI_TRIG_CHAN_OUT_A_X_B(x, a, n) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.address = (a), \
	.info_mask_separate = 0, \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = 0, \
	.ext_info = m2k_trigger_adc_a_b_mix, \
	.extend_name = n,\
}

static const struct iio_chan_spec m2k_trigger_adc_chan_spec[] = {
	AXI_TRIG_CHAN_ADC(0, 0, NULL),
	AXI_TRIG_CHAN_ADC(1, 1, NULL),
	AXI_TRIG_CHAN_PIN(2, 0, NULL),
	AXI_TRIG_CHAN_PIN(3, 1, NULL),
	AXI_TRIG_CHAN_ADC_X_PIN(4, 0, NULL),
	AXI_TRIG_CHAN_ADC_X_PIN(5, 1, NULL),
	AXI_TRIG_CHAN_OUT_A_X_B(6, 0, NULL),
};

static const struct iio_info m2k_trigger_adc_iio_info = {
	.read_raw = m2k_trigger_adc_read_raw,
	.debugfs_reg_access = m2k_trigger_adc_reg_access,
};

static void m2k_trigger_adcger_clk_disable(void *data)
{
	struct clk *clk = data;

	clk_disable_unprepare(clk);
}

static int m2k_trigger_adc_probe(struct platform_device *pdev)
{
	struct m2k_trigger_adc_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	mutex_init(&st->lock);

	st->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(st->clk))
		return PTR_ERR(st->clk);

	ret = clk_prepare_enable(st->clk);
	if (ret < 0)
		return -EINVAL;

	ret = devm_add_action_or_reset(&pdev->dev, m2k_trigger_adcger_clk_disable,
				       st->clk);
	if (ret)
		return ret;

	st->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	indio_dev->name = "m2k-trigger-adc";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &m2k_trigger_adc_iio_info;
	indio_dev->channels = m2k_trigger_adc_chan_spec,
	indio_dev->num_channels = ARRAY_SIZE(m2k_trigger_adc_chan_spec);

	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_FUNCTION(0), 2);
	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_FUNCTION(1), 2);

	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_TRIGGER_MIX(0), 0);
	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_TRIGGER_MIX(1), 0);
	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_TRIGGER_OUT_CTRL, 0);

	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_HYSTERESIS(0), 1);
	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_HYSTERESIS(1), 1);
	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_LIMIT(0), 0);
	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_LIMIT(1), 0);
	m2k_trigger_adc_write(st, M2K_TRIGGER_ADC_REG_DELAY, 0);

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id m2k_trigger_adc_of_match[] = {
	{ .compatible = "adi,m2k-trigger-adc" },
	{}
};

static struct platform_driver m2k_trigger_adc_driver = {
	.driver = {
		.name = "m2k-trigger-adc",
		.of_match_table = m2k_trigger_adc_of_match,
	},
	.probe = m2k_trigger_adc_probe,
};
module_platform_driver(m2k_trigger_adc_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AXI ADC trigger driver for ADALM-2000");
MODULE_LICENSE("GPL v2");
