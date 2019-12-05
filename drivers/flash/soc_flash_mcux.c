/*
 * Copyright (c) 2016 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <string.h>
#include <drivers/flash.h>
#include <errno.h>
#include <init.h>
#include <soc.h>
#include "flash_priv.h"

#include "fsl_common.h"
#ifdef CONFIG_SOC_FAMILY_LPC
#include "fsl_iap.h"
#else
#include "fsl_flash.h"
#endif

#ifdef CONFIG_SW_MANAGED_FLASH_WRITE
#ifdef DT_INST_0_SOC_NV_FLASH_ERASE_BLOCK_SIZE
/* Assumes that erase size is larger than write size */
#define MAX_BLOCK_SIZE DT_INST_0_SOC_NV_FLASH_ERASE_BLOCK_SIZE
#elif defined(DT_FLASH_WRITE_BLOCK_SIZE)
#define MAX_BLOCK_SIZE DT_FLASH_WRITE_BLOCK_SIZE
#else
#define MAX_BLOCK_SIZE FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE
#endif

static s8_t before_data[MAX_BLOCK_SIZE] = { 0 };
static s8_t after_data[MAX_BLOCK_SIZE] = { 0 };
static const s32_t max_blk_sz = MAX_BLOCK_SIZE;
#endif

static const struct flash_driver_api flash_mcux_api;

struct flash_priv {
	flash_config_t config;
	/*
	 * HACK: flash write protection is managed in software.
	 */
	struct k_sem write_lock;
	u32_t pflash_block_base;
};

/*
 * Interrupt vectors could be executed from flash hence the need for locking.
 * The underlying MCUX driver takes care of copying the functions to SRAM.
 *
 * For more information, see the application note below on Read-While-Write
 * http://cache.freescale.com/files/32bit/doc/app_note/AN4695.pdf
 *
 */

static int flash_mcux_erase(struct device *dev, off_t offset, size_t len)
{
	struct flash_priv *priv = dev->driver_data;
	u32_t addr;
	status_t rc;
	unsigned int key;

	if (k_sem_take(&priv->write_lock, K_NO_WAIT)) {
		return -EACCES;
	}

	addr = offset + priv->pflash_block_base;

	key = irq_lock();
	rc = FLASH_Erase(&priv->config, addr, len, kFLASH_ApiEraseKey);
	irq_unlock(key);

	k_sem_give(&priv->write_lock);

	return (rc == kStatus_Success) ? 0 : -EINVAL;
}

static int flash_mcux_read(struct device *dev, off_t offset,
				void *data, size_t len)
{
	struct flash_priv *priv = dev->driver_data;
	u32_t addr;

	/*
	 * The MCUX supports different flash chips whose valid ranges are
	 * hidden below the API: until the API export these ranges, we can not
	 * do any generic validation
	 */
	addr = offset + priv->pflash_block_base;

	memcpy(data, (void *)addr, len);

	return 0;
}

static int flash_mcux_write(struct device *dev, off_t offset,
				const void *data, size_t len)
{
	struct flash_priv *priv = dev->driver_data;
	u32_t addr;
	status_t rc;
	unsigned int key;

	if (k_sem_take(&priv->write_lock, K_NO_WAIT)) {
		return -EACCES;
	}

	addr = offset + priv->pflash_block_base;

	key = irq_lock();
	rc = FLASH_Program(&priv->config, (u32_t)addr, (u8_t *)data, len);
	irq_unlock(key);

	k_sem_give(&priv->write_lock);

	return (rc == kStatus_Success) ? 0 : -EINVAL;
}

#ifdef CONFIG_SW_MANAGED_FLASH_WRITE
static int flash_mcux_write_unsafe(struct device *dev, off_t offset,
				   const void *data, size_t len)
{
	struct flash_priv *priv = dev->driver_data;
	u32_t addr = offset + priv->pflash_block_base;
	size_t size_before = addr % max_blk_sz;
	size_t size_after = max_blk_sz - (size_before + len) % max_blk_sz;
	size_t aligned_size = size_before + len + size_after;
	off_t start_block = addr - size_before;
	off_t last_block = start_block + aligned_size - max_blk_sz;

	bool single_blk_write = (size_before + len < max_blk_sz);

	if (0 == size_before && 0 == size_after) {
		/* Aligned write */
		flash_mcux_erase(dev, offset, len);
		flash_mcux_write(dev, offset, data, len);
		return kStatus_Success;
	}

	/* Stash useful data from the blocks that will be affected. */
	if (single_blk_write) {
		/* Read old data before new data is written. */
		if (size_before) {
			flash_mcux_read(dev, start_block, before_data,
					size_before);
		}
		/* Fill the with new data. */
		memcpy((void *)(before_data + size_before), data, len);
		/* Fill the last part of old data. */
		if (size_after) {
			flash_mcux_read(dev, addr + len,
					before_data + size_before + len,
					size_after);
		}
	} else {
		/* Multiblock write, different start and end blocks. */
		if (size_before) {
			flash_mcux_read(dev, start_block, before_data,
					size_before);
			/* Fill the rest with new data. */
			memcpy((void *)(before_data + size_before), data,
			       max_blk_sz - size_before);
		}
		if (size_after) {
			/* Copy ending part of new data. */
			memcpy((void *)after_data,
			       (void *)((u8_t *)data +
				   len - ((len + size_before) % max_blk_sz)),
			       max_blk_sz - size_after);
			/* Copy ending part of flash block. */
			flash_mcux_read(dev, addr + len,
					after_data + (max_blk_sz - size_after),
					size_after);
		}
	}

	/* Erase all the blocks that overlap with new data. */
	flash_mcux_api.erase(dev, start_block, aligned_size);

	/* Write stashed and new data. */
	if (single_blk_write || size_before > 0) {
		/* Write first block if available. */
		flash_mcux_write(dev, start_block, (u8_t *)before_data,
				 max_blk_sz);
		/* Clear stashed data. */
		memset(before_data, 0, max_blk_sz);
	}
	if (!single_blk_write) {
		size_t middle_data_len = aligned_size;
		off_t middle_block_start = start_block;
		off_t data_offset = (off_t)data;

		/* Write the middle bit if available */
		if (size_before > 0) {
			middle_block_start += max_blk_sz;
			middle_data_len -= max_blk_sz;
			data_offset += (max_blk_sz - size_before);
		}
		if (size_after > 0) {
			middle_data_len -= max_blk_sz;
		}
		if (middle_data_len > 0) {
			flash_mcux_write(dev, middle_block_start,
					 (const void *)data_offset,
					 middle_data_len);
		}

		/* Write the last block if needed. */
		if (size_after > 0) {
			flash_mcux_write(dev, last_block, (u8_t *)after_data,
					 max_blk_sz);
			/* Clear stashed data for security reasons. */
			memset(after_data, 0, max_blk_sz);
		}
	}

	return kStatus_Success;
}
#endif

static int flash_mcux_write_protection(struct device *dev, bool enable)
{
	struct flash_priv *priv = dev->driver_data;
	int rc = 0;

	if (enable) {
		rc = k_sem_take(&priv->write_lock, K_FOREVER);
	} else {
		k_sem_give(&priv->write_lock);
	}

	return rc;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static const struct flash_pages_layout dev_layout = {
	.pages_count = KB(CONFIG_FLASH_SIZE) / DT_INST_0_SOC_NV_FLASH_ERASE_BLOCK_SIZE,
	.pages_size = DT_INST_0_SOC_NV_FLASH_ERASE_BLOCK_SIZE,
};

static void flash_mcux_pages_layout(struct device *dev,
									const struct flash_pages_layout **layout,
									size_t *layout_size)
{
	*layout = &dev_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static struct flash_priv flash_data;

static const struct flash_driver_api flash_mcux_api = {
	.write_protection = flash_mcux_write_protection,
	.erase = flash_mcux_erase,
#if CONFIG_SW_MANAGED_FLASH_WRITE
	.write = flash_mcux_write_unsafe,
#else
	.write = flash_mcux_write,
#endif
	.read = flash_mcux_read,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_mcux_pages_layout,
#endif
#ifdef DT_FLASH_WRITE_BLOCK_SIZE
	.write_block_size = DT_FLASH_WRITE_BLOCK_SIZE,
#else
	.write_block_size = FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE,
#endif
};

static int flash_mcux_init(struct device *dev)
{
	struct flash_priv *priv = dev->driver_data;
	u32_t pflash_block_base;
	status_t rc;

	k_sem_init(&priv->write_lock, 0, 1);

	rc = FLASH_Init(&priv->config);

#ifdef CONFIG_SOC_FAMILY_LPC
	FLASH_GetProperty(&priv->config, kFLASH_PropertyPflashBlockBaseAddr,
			  &pflash_block_base);
#else
	FLASH_GetProperty(&priv->config, kFLASH_PropertyPflash0BlockBaseAddr,
			  &pflash_block_base);
#endif
	priv->pflash_block_base = (u32_t) pflash_block_base;

	return (rc == kStatus_Success) ? 0 : -EIO;
}

DEVICE_AND_API_INIT(flash_mcux, DT_FLASH_DEV_NAME,
			flash_mcux_init, &flash_data, NULL, POST_KERNEL,
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &flash_mcux_api);
