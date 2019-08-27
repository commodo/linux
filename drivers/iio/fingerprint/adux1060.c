// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices ADUX1060 SPI fingerprint scanner
 *
 * Copyright 2019 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define ADUX1060_ACK_OK		0x41

#define ADUX1060_START_PACKET	0x0E07
#define ADUX1060_CMD_NORMAL	0x4E
#define ADUX1060_CMD_END	0x45

#define ADUX1060_REG_SW_STAT1		0x20000014
#define ADUX1060_REG_AFE_CFG1		0x20000024
#define ADUX1060_REG_AFE_CFG2		0x20000028
#define ADUX1060_REG_AFE_CFG3		0x2000002C
#define ADUX1060_REG_SCAN_CFG1		0x20000030
#define ADUX1060_REG_SCAN_CFG2		0x20000034
#define ADUX1060_REG_CAL_CFG1		0x20000038
#define ADUX1060_REG_AFE_CTRL		0x20000048
#define ADUX1060_REG_SCAN_CTRL		0x2000004C
#define ADUX1060_REG_CAL_CTRL		0x20000050
#define ADUX1060_REG_TX_CFG1		0x20000054
#define ADUX1060_REG_TX_CFG2		0x20000058
#define ADUX1060_REG_RX_CFG		0x2000005C
#define ADUX1060_REG_INTG_CFG01		0x20000060
#define ADUX1060_REG_INTG_CFG023	0x20000064
#define ADUX1060_REG_CHAN_CTRL		0x20000068

#define ADUX1060_REG_TOU_DET_CFG1	0x20000070
#define ADUX1060_REG_TOU_DET_CFG2	0x20000074
#define ADUX1060_REG_TOU_DET_CFG3	0x20000078
#define ADUX1060_REG_TOU_DET_TX_FREQ	0x2000007C
#define ADUX1060_REG_TOU_DET_SCAN_CTRL1	0x20000080
#define ADUX1060_REG_TOU_DET_SCAN_CTRL2	0x20000084
#define ADUX1060_REG_TOU_DET_W_INT_CTRL 0x20000088

/* ADUX1060_REG_SCAN_CFG2 */
#define ADUX1060_NUM_IMG_ROWS_MSK	GENMASK(31, 24)
#define ADUX1060_NUM_IMG_ROWS(x)	FIELD_PREP(ADUX1060_NUM_IMG_ROWS_MSK, x)
#define ADUX1060_UNTOU_IRQ_EN_MSK	BIT(22)
#define ADUX1060_UNTOU_IRQ_EN(x)	FIELD_PREP(ADUX1060_UNTOU_IRQ_EN_MSK, x)
#define ADUX1060_TOU_IRQ_EN_MSK		BIT(21)
#define ADUX1060_TOU_IRQ_EN(x)		FIELD_PREP(ADUX1060_TOU_IRQ_EN_MSK, x)

/* ADUX1060_REG_AFE_CFG3 */
#define ADUX1060_TX_FREQ_MSK		GENMASK(24, 0)
#define ADUX1060_TX_FREQ(x)		FIELD_PREP(ADUX1060_TX_FREQ_MSK, x)

/* Read up to 14400 registers (57600 bytes) of data */
#define ADUX1060_READ_CMD	0xB0B0
/* Write up to 512 registers (2048 bytes) of data */
#define ADUX1060_WRITE_CMD	0xA0A0

#define ADUX1060_TOTAL_NUM_OF_ROWS	120

struct adux1060_state {
	struct spi_device *spi;
	/*
	 * Lock for accessing device registers. Some operations require
	 * multiple consecutive R/W operations, during which the device
	 * shouldn't be interrupted.
	 */
	struct mutex lock;
	struct completion completion;
	struct gpio_desc *gpio_reset;
	struct gpio_desc *gpio_cs;

	union {
		u8 d8;
		__le32 d32;
	} data ____cacheline_aligned;
};

enum {
	ADUX1060_CAPTURE_UNTOUCH,
	ADUX1060_CAPTURE_TOUCH,
};

enum adux1060_scan_type {
	ADUX1060_TOUCH_SCAN = 0x03,
	ADUX1060_UNTOUCH_SCAN = 0x05,
	ADUX1060_TOUCH_DET_SCAN = 0x09,
};

struct adux1060_preboot {
	u16 id;
	u16 bytes;
	u8 command;
	u32 addr;
	u32 hcrc;
	u32 pcrc;
} __packed;

struct adux1060_postboot_cmd {
	u16 cmd1;
	u16 n_regs;
	u32 reg_addr;
	u16 cmd2;
	u8 reserved[58];
} __packed;

static const struct iio_chan_spec adux1060_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 0,
	},
};

struct adux1060_reg_seq {
	u32 reg_addr;
	u32 reg_val;
};

static const struct adux1060_reg_seq adux1060_reg_defaults[] = {
	{ ADUX1060_REG_SCAN_CFG1,	   0x81000678 },
	{ ADUX1060_REG_SCAN_CFG2,	   0x3C214000 },
	{ ADUX1060_REG_SCAN_CTRL,	   0x00000001 },
	{ ADUX1060_REG_AFE_CFG1,	   0x0C00211A },
	{ ADUX1060_REG_AFE_CFG2,	   0x00000000 },
	{ ADUX1060_REG_AFE_CFG3,	   0x0138D0A0 },
	{ ADUX1060_REG_AFE_CTRL,	   0x00000001 },
	{ ADUX1060_REG_CAL_CFG1,	   0x00205030 },
	{ ADUX1060_REG_CAL_CTRL,	   0x00000008 },
	{ ADUX1060_REG_TX_CFG1,		   0x32646464 },
	{ ADUX1060_REG_TX_CFG2,		   0x21001200 },
	{ ADUX1060_REG_RX_CFG,		   0x06780700 },
	{ ADUX1060_REG_INTG_CFG01,	   0x18141713 },
	{ ADUX1060_REG_INTG_CFG023,	   0x1A161915 },
	{ ADUX1060_REG_CHAN_CTRL,	   0x00000005 },
	{ ADUX1060_REG_TOU_DET_CFG1,	   0x1E020F64 },
	{ ADUX1060_REG_TOU_DET_CFG2,	   0x013C4E20 },
	{ ADUX1060_REG_TOU_DET_CFG3,	   0x08320006 },
	{ ADUX1060_REG_TOU_DET_TX_FREQ,	   0x0121EAC0 },
	{ ADUX1060_REG_TOU_DET_SCAN_CTRL1, 0x47321F0A },
	{ ADUX1060_REG_TOU_DET_SCAN_CTRL2, 0x00006D5A },
	{ ADUX1060_REG_TOU_DET_W_INT_CTRL, 0x0000130D },
};

static int adux1060_spi_reg_read(struct adux1060_state *st,
				 unsigned int *buf,
				 unsigned int addr,
				 unsigned int len)
{
	struct adux1060_postboot_cmd read;
	struct spi_transfer t[] = {
		{
			.tx_buf = (char *)&read,
			.len = 68,
		}, {
			.rx_buf = &st->data.d32,
			.len = len,
		}
	};
	int ret;

	/*
	 * A 68-byte read header consists of a 10-byte command followed
	 * by 58 bytes of zeroes.
	 */
	read.cmd1 = cpu_to_le16(ADUX1060_READ_CMD);
	read.n_regs = cpu_to_le16(1);
	read.reg_addr = cpu_to_le32(addr);
	read.cmd2 = read.cmd1;
	memset(read.reserved, 0, 58);

	gpiod_set_value(st->gpio_cs, 1);
	usleep_range(250, 300);
	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	gpiod_set_value(st->gpio_cs, 0);

	*buf = le32_to_cpu(st->data.d32);

	return ret;
}

static int adux1060_spi_reg_write(struct adux1060_state *st,
				  unsigned int val, unsigned int addr)
{
	struct adux1060_postboot_cmd write;
	struct spi_transfer t[] = {
		{
			.tx_buf = (char *)&write,
			.len = 68,
		}, {
			.tx_buf = &st->data.d32,
			.len = 4,
		}
	};
	int ret;

	/*
	 * A 68-byte write header consists of a 10-byte command followed
	 * by 58 bytes of zeroes.
	 */
	write.cmd1 = cpu_to_le16(ADUX1060_WRITE_CMD);
	write.n_regs = cpu_to_le16(1);
	write.reg_addr = cpu_to_le32(addr);
	write.cmd2 = write.cmd1;
	memset(write.reserved, 0, 58);

	st->data.d32 = cpu_to_le32(val);

	gpiod_set_value(st->gpio_cs, 1);
	usleep_range(250, 300);
	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	gpiod_set_value(st->gpio_cs, 0);

	return ret;
}

static int adux1060_spi_write_mask(struct adux1060_state *st,
				   unsigned int addr,
				   unsigned long int mask,
				   unsigned int val)
{
	unsigned int regval;
	int ret;

	ret = adux1060_spi_reg_read(st, &regval, addr, 4);
	if (ret < 0)
		return ret;

	regval &= ~mask;
	regval |= val;

	return adux1060_spi_reg_write(st, regval, addr);
}

static int adux1060_spi_read_ack(struct adux1060_state *st, char *buf)
{
	struct spi_transfer t[] = {
		{
			.rx_buf = &st->data.d8,
			.len = 1,
		},
	};
	int ret;

	reinit_completion(&st->completion);

	gpiod_set_value(st->gpio_cs, 1);
	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	gpiod_set_value(st->gpio_cs, 0);
	if (ret < 0)
		return ret;

	ret = wait_for_completion_timeout(&st->completion,
					  msecs_to_jiffies(100));
	if (!ret)
		return -ETIMEDOUT;

	*buf = st->data.d8;

	return 0;
}

static int adux1060_preboot_spi_write(struct adux1060_state *st,
				      void *data, unsigned int len)
{
	struct spi_transfer t[] = {
		{
			.tx_buf = data,
			.len = len,
		},
	};
	char buf;
	int ret;

	reinit_completion(&st->completion);

	gpiod_set_value(st->gpio_cs, 1);
	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	gpiod_set_value(st->gpio_cs, 0);
	if (ret < 0)
		return ret;

	ret = wait_for_completion_timeout(&st->completion,
					  msecs_to_jiffies(100));
	if (!ret)
		return -ETIMEDOUT;

	/* Read the ack byte */
	ret = adux1060_spi_read_ack(st, &buf);
	if (ret < 0)
		return ret;

	if (buf != ADUX1060_ACK_OK) {
		dev_err(&st->spi->dev, "Failed to read ACK: %x\n", buf);
		ret = -EINVAL;
	}

	return ret;
}

static int adux1060_reset(struct adux1060_state *st)
{
	int ret;

	reinit_completion(&st->completion);

	if (st->gpio_reset) {
		gpiod_set_value(st->gpio_reset, 1);
		usleep_range(50000, 50001);
		gpiod_set_value(st->gpio_reset, 0);
	} else {
		return -ENODEV;
	}

	/*
	 * The ADUX1060 indicates readiness to receive a packet by asserting
	 * the IRQ pin high, on reset, the IRQ pin will be in the low state
	 */
	ret = wait_for_completion_timeout(&st->completion,
					  msecs_to_jiffies(100));
	if (!ret)
		return -ETIMEDOUT;

	return 0;
}


static int adux1060_init_scan(struct adux1060_state *st,
			      enum adux1060_scan_type scan)
{
	unsigned int regval, num_int, num_rows;
	unsigned long int mask;
	int ret;

	mask = ADUX1060_UNTOU_IRQ_EN_MSK | ADUX1060_TOU_IRQ_EN_MSK;
	if (scan == ADUX1060_UNTOUCH_SCAN) {
		/* Disable the touch irq and enable the untouch irq */
		regval = ADUX1060_UNTOU_IRQ_EN(1) |
			 ADUX1060_TOU_IRQ_EN(0);
		/*
		 * One interrupt is expected to signal the completion of the
		 * untouch scan
		 */
		num_int = 1;
	} else {
		/* Disable the untouch irq and enable the touch irq */
		regval = ADUX1060_UNTOU_IRQ_EN(0) |
			 ADUX1060_TOU_IRQ_EN(1);
		/*
		 * After the completion of the scan of the number of rows set
		 * in NUM_IMG_ROWS field, an interrupt will occur. Therefore,
		 * the number of interrupts that we need to wait for in case of
		 * a touch scan equals the number of total rows divided by the
		 * number of image rows.
		 */
		ret = adux1060_spi_reg_read(st, &num_rows,
					    ADUX1060_REG_SCAN_CFG2, 4);
		if (ret < 0)
			return ret;

		num_rows = (num_rows & ADUX1060_NUM_IMG_ROWS_MSK) >> 24;
		num_int = ADUX1060_TOTAL_NUM_OF_ROWS / num_rows;
	}

	ret = adux1060_spi_write_mask(st, ADUX1060_REG_SCAN_CFG2, mask, regval);
	if (ret < 0)
		return ret;
	/*
	 * Set bit CFG_SCAN in register SCAN_CTRL[0]. When set,
	 * AFE parameters from SCAN_CFG_1 and SCAN_CFG2 are written
	 * to ADUX1060 hw registers. This bit self clears after
	 * configuration
	 */
	ret = adux1060_spi_reg_write(st, scan, ADUX1060_REG_SCAN_CTRL);
	if (ret < 0)
		return ret;

	do {
		reinit_completion(&st->completion);
		ret = wait_for_completion_timeout(&st->completion,
						  msecs_to_jiffies(1000));
		if (!ret)
			return -ETIMEDOUT;
	} while (--num_int);

	return 0;
}

static ssize_t adux1060_read(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adux1060_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	unsigned int regval;
	int ret;

	mutex_lock(&st->lock);
	switch ((u32)this_attr->address) {
	case ADUX1060_CAPTURE_UNTOUCH:
		ret = adux1060_init_scan(st, ADUX1060_UNTOUCH_SCAN);
		break;
	case ADUX1060_CAPTURE_TOUCH:
		ret = adux1060_init_scan(st, ADUX1060_TOUCH_SCAN);
		break;
	default:
		ret = -EINVAL;
	};
	mutex_unlock(&st->lock);

	return ret;
}

static IIO_DEVICE_ATTR(capture_untouch_scan, 0444,
		       adux1060_read, NULL, ADUX1060_CAPTURE_UNTOUCH);

static IIO_DEVICE_ATTR(capture_touch_scan, 0444,
		       adux1060_read, NULL, ADUX1060_CAPTURE_TOUCH);

static struct attribute * adux1060_attributes[] = {
	&iio_dev_attr_capture_untouch_scan.dev_attr.attr,
	&iio_dev_attr_capture_touch_scan.dev_attr.attr,
	NULL
};

static const struct attribute_group adux1060_attribute_group = {
	.attrs = adux1060_attributes,
};

static int adux1060_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)

{
	struct adux1060_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	if (readval)
		ret = adux1060_spi_reg_read(st, readval, reg, 4);
	else
		ret = adux1060_spi_reg_write(st, writeval, reg);
	mutex_unlock(&st->lock);

	return ret;
}

static const struct iio_info adux1060_info = {
	.debugfs_reg_access = &adux1060_reg_access,
	.attrs = &adux1060_attribute_group,
};

static int adux1060_load_firmware(struct adux1060_state *st)
{
	const struct firmware *fw;
	struct adux1060_preboot writeCom = {
		.id = ADUX1060_START_PACKET,
		.command = ADUX1060_CMD_NORMAL,
		/*
		 * The program code should have its Interrupt Vector Table
		 * placed at SRAM address 0x10000000.
		 */
		.addr = 0x10000000,
	};
	struct adux1060_preboot endCom = {
		.id = ADUX1060_START_PACKET,
		.command = ADUX1060_CMD_END,
	};
	int ret;

	mutex_lock(&st->lock);
	ret = request_firmware(&fw, "adux1060.bin", &st->spi->dev);
	if (ret < 0) {
		dev_err(&st->spi->dev, "Unable to open firmware\n");
		goto error;
	}

	/* The payload size is up to 65532 bytes */
	if (fw->size > 65532) {
		ret = -EFBIG;
		goto error;
	}

	ret = adux1060_reset(st);
	if (ret < 0)
		goto error;

	/* add the firmware size to header */
	writeCom.bytes = fw->size;
	/* Compute the header CRC */
	writeCom.hcrc = crc32(~0, &writeCom.bytes, 7);
	writeCom.hcrc ^= ~0;
	/* Compute the payload CRC */
	writeCom.pcrc = crc32(~0, fw->data, fw->size);
	writeCom.pcrc ^= ~0;
	/*
	 * The packet protocol consists of a 17-byte header and a variable
	 * size payload. The packet header is transmited first
	 */
	ret = adux1060_preboot_spi_write(st, &writeCom, 17);
	if (ret < 0)
		goto error;

	/* Send firmware */
	ret = adux1060_preboot_spi_write(st, (void *)fw->data, fw->size);
	if (ret < 0)
		goto error;

	/* Compute the end command header CRC */
	endCom.hcrc = crc32(~0, &endCom.bytes, 7);
	endCom.hcrc ^= ~0;

	ret = adux1060_preboot_spi_write(st, &endCom, 17);
	if (ret < 0)
		goto error;
	/*
	 * A delay of 50ms must be allowed after the completion of a
	 * successful boot.
	 */
	usleep_range(50000, 50001);
error:
	release_firmware(fw);
	mutex_unlock(&st->lock);
	return ret;
}

static int adux1060_request_gpios(struct adux1060_state *st)
{
	struct device *dev = &st->spi->dev;

	st->gpio_reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_reset))
		return PTR_ERR(st->gpio_reset);

	st->gpio_cs = devm_gpiod_get(dev, "adi,cs", GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_cs))
		return PTR_ERR(st->gpio_cs);

	return 0;
}

static irqreturn_t adux1060_interrupt(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct adux1060_state *st = iio_priv(indio_dev);

	complete(&st->completion);

	return IRQ_HANDLED;
};


static int adux1060_setup(struct adux1060_state *st)
{
	unsigned int regval;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(adux1060_reg_defaults); i++) {
		ret = adux1060_spi_reg_write(st,
					     adux1060_reg_defaults[i].reg_val,
					     adux1060_reg_defaults[i].reg_addr);
		if (ret < 0)
			return ret;
	}

	/* Dummy read clears the software status register */
	return adux1060_spi_reg_read(st, &regval, ADUX1060_REG_SW_STAT1, 4);
}

static int adux1060_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct iio_dev *indio_dev;
	struct adux1060_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);
	st->spi = spi;
	mutex_init(&st->lock);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = id->name;
	indio_dev->info = &adux1060_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adux1060_channels;
	indio_dev->num_channels = ARRAY_SIZE(adux1060_channels);

	init_completion(&st->completion);

	ret = devm_request_irq(&st->spi->dev, spi->irq,
			       &adux1060_interrupt,
			       IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			       id->name, indio_dev);
	if (ret < 0)
		return ret;

	ret = adux1060_request_gpios(st);
	if (ret < 0)
		return ret;

	ret = adux1060_load_firmware(st);
	if (ret < 0) {
		dev_err(&st->spi->dev, "Unable to load firmware\n");
		return ret;
	}

	ret = adux1060_setup(st);
	if (ret < 0) {
		dev_err(&st->spi->dev, "ADUX1060 setup failed\n");
		return ret;
	}

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id adux1060_id_table[] = {
	{ "adi,adux1060" },
	{}
};
MODULE_DEVICE_TABLE(spi, adux1060_id_table);

static const struct of_device_id adux1060_of_match[] = {
	{ .compatible = "adi,adux1060" },
	{ },
};
MODULE_DEVICE_TABLE(of, adux1060_of_match);

static struct spi_driver adux1060_driver = {
	.driver = {
		.name = "adux1060",
		.of_match_table = adux1060_of_match,
	},
	.probe = adux1060_probe,
	.id_table = adux1060_id_table,
};
module_spi_driver(adux1060_driver);

MODULE_AUTHOR("Beniamin Bia <beniamin.bia@analog.com>");
MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADUX1060");
MODULE_LICENSE("GPL v2");
