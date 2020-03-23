// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2013-2015 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@xxxxxxxxxx>
 * based on iio_simple_dummy_buffer.c
 *  Copyright (c) 2011 Jonathan Cameron
 */

#include <linux/bitmap.h>
#include <linux/dma-mapping.h>
#include <linux/export.h>
#include <linux/fixp-arith.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <linux/iio/iio.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer_impl.h>

#include "iio_simple_dummy.h"

/*
 * The dummy DMA buffer driver implements a buffer for the IIO simple dummy
 * device driver. The buffer driver uses the generic IIO DMA buffer
 * infrastructure and can be used as a template when implementing drivers using
 * this infrastructure as well as for testing the infrastructure without any
 * actual DMA capable hardware being present.
 *
 * This dummy driver is divided into two sections. The first part emulates the
 * behavior of a multi-channel converter and the attached DMA filling a buffer
 * with waveforms from the enabled channels. This part provides functions for
 * setting up the "DMA" and the "converter".
 *
 * The second part implements the typical driver structure that you'd expect
 * from a DMA buffer driver. It uses the functions provided by part 1 to perform
 * the "hardware" access.
 *
 * When using this driver as a template part 1 can be ignored.
 */

typedef void (*iio_dummy_dma_source_fn)(unsigned int, unsigned int, void *);

struct iio_dummy_dma_source {
	iio_dummy_dma_source_fn fn;
	unsigned int period;
	unsigned int pos;
};

struct iio_dummy_dma_transfer {
	struct list_head head;

	/* Memory address and length of the transfer */
	void *addr;
	unsigned int length;

	/* Data passed to the IRQ callback when the transfer completes */
	void *irq_data;
};

struct iio_dummy_dma {
	/* IRQ routine for the dummy DMA */
	void (*irq_fn)(unsigned int bytes_transferred, void *data);

	/* Pending DMA transfers */
	struct mutex transfer_list_lock;
	struct list_head transfer_list;

	/* Used to emulate periodic completion of DMA transfers */
	struct delayed_work work;

	/* Information about the connected data sources */
	unsigned int num_sources;
	struct iio_dummy_dma_source sources[4];
};

static void iio_dummy_dma_buffer_fn_rect(unsigned int n, unsigned int period,
	void *data)
{
	/* 13 bit unsigned */
	if (n > period / 2)
		*(uint16_t *)data = 1 << 12;
	else
		*(uint16_t *)data = 0;
}

static void iio_dummy_dma_buffer_fn_sine(unsigned int n, unsigned int period,
	void *data)
{
	/* 12 bit signed */
	*(int16_t *)data = fixp_sin32_rad(n, period) >> 20;
}

static void iio_dummy_dma_buffer_fn_tri(unsigned int n, unsigned int period,
	void *data)
{
	unsigned int x;

	if (n > period / 2)
		x = period - n;
	else
		x = n;

	/* 11 bit signed */
	*(int16_t *)data = ((((1 << 11) - 1) * x) / (period / 2)) - (1 << 10);
}

static void iio_dummy_dma_buffer_fn_saw(unsigned int n, unsigned int period,
	void *data)
{
	/* 16 bit signed */
	*(int16_t *)data = ((((1 << 16) - 1) * n) / period) - (1 << 15);
}

static const struct iio_dummy_dma_source iio_dummy_dma_sources[] = {
	{
		.fn = iio_dummy_dma_buffer_fn_rect,
		.period = 1000,
	}, {
		.fn = iio_dummy_dma_buffer_fn_sine,
		.period = 2000,
	}, {
		.fn = iio_dummy_dma_buffer_fn_tri,
		.period = 5000,
	}, {
		.fn = iio_dummy_dma_buffer_fn_saw,
		.period = 6789,
	},
};

static void iio_dummy_dma_schedule_next(struct iio_dummy_dma *dma)
{
	struct iio_dummy_dma_transfer *transfer;
	unsigned int num_samples;

	if (list_empty(&dma->transfer_list))
		return;

	transfer = list_first_entry(&dma->transfer_list,
		struct iio_dummy_dma_transfer, head);

	num_samples = transfer->length / (dma->num_sources * sizeof(uint16_t));

	/* 10000 SPS */
	schedule_delayed_work(&dma->work, msecs_to_jiffies(num_samples / 10));
}

static void iio_dummy_dma_work(struct work_struct *work)
{
	struct iio_dummy_dma *dma = container_of(work, struct iio_dummy_dma,
		work.work);
	struct iio_dummy_dma_transfer *transfer;
	struct iio_dummy_dma_source *src;
	unsigned int num_samples;
	unsigned int i, j;
	void *data;

	/* Get the next pending transfer and then fill it with data. */
	mutex_lock(&dma->transfer_list_lock);
	transfer = list_first_entry(&dma->transfer_list,
		struct iio_dummy_dma_transfer, head);
	list_del(&transfer->head);
	iio_dummy_dma_schedule_next(dma);
	mutex_unlock(&dma->transfer_list_lock);

	/*
	 * For real hardware copying of the data will be done by the DMA in the
	 * background. Here it is done in software.
	 */
	num_samples = transfer->length / (dma->num_sources * sizeof(uint16_t));
	data = transfer->addr;
	for (i = 0; i < num_samples; i++) {
		for (j = 0; j < dma->num_sources; j++) {
			src = &dma->sources[j];
			src->fn(src->pos, src->period, data);
			src->pos = (src->pos + 1) % src->period;
			data += 2;
		}
	}

	/* Generate "interrupt" */
	dma->irq_fn(num_samples * dma->num_sources * sizeof(uint16_t),
		transfer->irq_data);
	kfree(transfer);
}

static int iio_dummy_dma_issue_transfer(struct iio_dummy_dma *dma, void *addr,
	unsigned int length, void *irq_data)
{
	struct iio_dummy_dma_transfer *transfer;

	transfer = kzalloc(sizeof(*transfer), GFP_KERNEL);
	if (!transfer)
		return -ENOMEM;

	transfer->addr = addr;
	transfer->length = length;
	transfer->irq_data = irq_data;

	mutex_lock(&dma->transfer_list_lock);
	list_add_tail(&transfer->head, &dma->transfer_list);

	/* Start "DMA" transfer */
	iio_dummy_dma_schedule_next(dma);
	mutex_unlock(&dma->transfer_list_lock);

	return 0;
}

static void iio_dummy_dma_stop(struct iio_dummy_dma *dma)
{
	mutex_lock(&dma->transfer_list_lock);
	cancel_delayed_work(&dma->work);
	INIT_LIST_HEAD(&dma->transfer_list);
	mutex_unlock(&dma->transfer_list_lock);
}

static void iio_dummy_dma_setup(struct iio_dummy_dma *dma,
	void (*irq_fn)(unsigned int, void *))
{
	INIT_LIST_HEAD(&dma->transfer_list);
	mutex_init(&dma->transfer_list_lock);
	INIT_DELAYED_WORK(&dma->work, iio_dummy_dma_work);
	dma->irq_fn = irq_fn;
}

/*
 * Part two: Typical DMA driver implementation.
 */

struct iio_dummy_dma_buffer {
	/* Generic IIO DMA buffer base struct */
	struct iio_dma_buffer_queue queue;

	/* Handle to the "DMA" controller */
	struct iio_dummy_dma dma;

	/* List of submitted blocks */
	struct list_head block_list;
};

static struct iio_dummy_dma_buffer *iio_buffer_to_dummy_dma_buffer(
	struct iio_buffer *buffer)
{
	return container_of(buffer, struct iio_dummy_dma_buffer, queue.buffer);
}

static void io_dummy_dma_buffer_irq(unsigned int bytes_transferred,
	void *data)
{
	struct iio_dma_buffer_block *block = data;
	struct iio_dma_buffer_queue *queue = block->queue;
	unsigned long flags;

	/* Protect against races with submit() */
	spin_lock_irqsave(&queue->list_lock, flags);
	list_del(&block->head);
	spin_unlock_irqrestore(&queue->list_lock, flags);

	/*
	 * Update actual number of bytes transferred. This might be less than
	 * the requested number, e.g. due to alignment requirements of the
	 * controller, but must be a multiple of the sample size.
	 */
	block->block.bytes_used = bytes_transferred;

	/*
	 * iio_dma_buffer_block_done() must be called after the DMA transfer for
	 * the block that has been completed. This will typically be done from
	 * some kind of completion interrupt routine or callback.
	 */
	iio_dma_buffer_block_done(block);
}

static int iio_dummy_dma_buffer_submit(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	struct iio_dummy_dma_buffer *buffer =
		iio_buffer_to_dummy_dma_buffer(&queue->buffer);
	unsigned long flags;
	int ret;

	/*
	 * submit() is called when the buffer is active and a block becomes
	 * available. It should start a DMA transfer for the submitted block as
	 * soon as possible. submit() can be called even when a DMA transfer is
	 * already active. This gives the driver to prepare and setup the next
	 * transfer to allow a seamless switch to the next block without losing
	 * any samples.
	 */

	spin_lock_irqsave(&queue->list_lock, flags);
	list_add(&block->head, &buffer->block_list);
	spin_unlock_irqrestore(&queue->list_lock, flags);

	ret = iio_dummy_dma_issue_transfer(&buffer->dma, block->vaddr,
		block->block.size, block);
	if (ret) {
		spin_lock_irqsave(&queue->list_lock, flags);
		list_del(&block->head);
		spin_unlock_irqrestore(&queue->list_lock, flags);
		return ret;
	}

	return 0;
}

static void iio_dummy_dma_buffer_abort(struct iio_dma_buffer_queue *queue)
{
	struct iio_dummy_dma_buffer *buffer =
		iio_buffer_to_dummy_dma_buffer(&queue->buffer);

	/*
	 * When abort() is called is is guaranteed that that submit() is not
	 * called again until abort() has completed. This means no new blocks
	 * will be added to the list. Once the pending DMA transfers are
	 * canceled no blocks will be removed either. So it is save to release
	 * the uncompleted blocks still on the list.
	 *
	 * If a DMA does not support aborting transfers it is OK to keep the
	 * currently active transfers running. In that case the blocks
	 * associated with the transfer must not be marked as done until they
	 * are completed.  Otherwise their memory might be freed while the DMA
	 * transfer is still in progress.
	 *
	 * Special care needs to be taken if the DMA controller does not
	 * support aborting transfers but the converter will stop sending
	 * samples once disabled. In this case the DMA might get stuck until the
	 * converter is re-enabled.
	 */
	iio_dummy_dma_stop(&buffer->dma);

	/*
	 * None of the blocks are any longer in use at this point, give them
	 * back.
	 */
	iio_dma_buffer_block_list_abort(queue, &buffer->block_list);
}

static void iio_dummy_dma_buffer_release(struct iio_buffer *buf)
{
	struct iio_dummy_dma_buffer *buffer =
		iio_buffer_to_dummy_dma_buffer(buf);

	/*
	 * This function is called when all references to the buffer have been
	 * dropped should free any memory or other resources associated with the
	 * buffer.
	 */

	/*
	 * iio_dma_buffer_release() must be called right before freeing the
	 * memory.
	 */
	iio_dma_buffer_release(&buffer->queue);
	kfree(buffer);
}

/*
 * Most drivers will be able to use the default DMA buffer callbacks. But if
 * necessary it is possible to overwrite certain functions with custom
 * implementations. One exception is the release callback, which always needs to
 * be implemented.
 */
static const struct iio_buffer_access_funcs iio_dummy_dma_buffer_ops = {
	.read = iio_dma_buffer_read,
	.set_bytes_per_datum = iio_dma_buffer_set_bytes_per_datum,
	.set_length = iio_dma_buffer_set_length,
	.request_update = iio_dma_buffer_request_update,
	.enable = iio_dma_buffer_enable,
	.disable = iio_dma_buffer_disable,
	.data_available = iio_dma_buffer_data_available,
	.release = iio_dummy_dma_buffer_release,

	.modes = INDIO_BUFFER_HARDWARE,
	.flags = INDIO_BUFFER_FLAG_FIXED_WATERMARK,
};

static const struct iio_dma_buffer_ops iio_dummy_dma_buffer_dma_ops = {
	.submit = iio_dummy_dma_buffer_submit,
	.abort = iio_dummy_dma_buffer_abort,
};

/**
 * iio_simple_dummy_update_scan_mode() - Update active channels
 * @indio_dev: The IIO device
 * @scan_mask: Scan mask with the new active channels
 */
int iio_simple_dummy_update_scan_mode(struct iio_dev *indio_dev,
	const unsigned long *scan_mask)
{
	struct iio_dummy_dma_buffer *buffer =
		iio_buffer_to_dummy_dma_buffer(indio_dev->buffer);
	struct iio_dummy_dma *dma = &buffer->dma;
	unsigned int i, j;

	/*
	 * Setup the converter to output the selected channels to the DMA. For
	 * real hardware the connection between the converter and the DMA will
	 * be in hardware, here we use the struct to exchange this information.
	 */
	j = 0;
	for_each_set_bit(i, scan_mask, indio_dev->masklength) {
		dma->sources[j] = iio_dummy_dma_sources[i];
		j++;
	}

	dma->num_sources = j;

	return 0;
}

int iio_simple_dummy_configure_buffer(struct iio_dev *indio_dev)
{
	struct iio_dummy_dma_buffer *buffer;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	/*
	 * Setup DMA controller. For real hardware this should acquire and setup
	 * all resources that are necessary to operate the DMA controller, like
	 * IRQs, clocks, IO mem regions, etc.
	 */
	iio_dummy_dma_setup(&buffer->dma, iio_dummy_dma_buffer_irq);

	/*
	 * For a real device the device passed to iio_dma_buffer_init() must be
	 * the device that performs the DMA transfers. Often this is not the
	 * device for the converter, but a dedicated DMA controller.
	 */
	dma_coerce_mask_and_coherent(&indio_dev->dev, DMA_BIT_MASK(32));
	iio_dma_buffer_init(&buffer->queue, &indio_dev->dev,
		&iio_dummy_dma_buffer_dma_ops, NULL);
	buffer->queue.buffer.access = &iio_dummy_dma_buffer_ops;

	INIT_LIST_HEAD(&buffer->block_list);

	indio_dev->buffer = &buffer->queue.buffer;
	indio_dev->modes |= INDIO_BUFFER_HARDWARE;

	return 0;
}

/**
 * iio_dummy_dma_unconfigure_buffer() - release buffer resources
 * @indio_dev: device instance state
 */
void iio_simple_dummy_unconfigure_buffer(struct iio_dev *indio_dev)
{
	struct iio_dummy_dma_buffer *buffer =
		iio_buffer_to_dummy_dma_buffer(indio_dev->buffer);

	/*
	 * Once iio_dma_buffer_exit() has been called none of the DMA buffer
	 * callbacks will be called. This means it is save to free any resources
	 * that are only used in those callbacks at this point. The memory for
	 * the buffer struct must not be freed since it might be still in use
	 * elsewhere. It will be freed in the buffers release callback.
	 */
	iio_dma_buffer_exit(&buffer->queue);

	/*
	 * Drop our reference to the buffer. Since this might be the last one
	 * the buffer structure must no longer be accessed after this.
	 */
	iio_buffer_put(&buffer->queue.buffer);
}
