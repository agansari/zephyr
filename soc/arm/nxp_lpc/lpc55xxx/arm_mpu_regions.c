/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <arch/arm/cortex_m/mpu/arm_mpu.h>

static const struct arm_mpu_region mpu_regions[] = {
	/* Region 0 */
	MPU_REGION_ENTRY("FLASH_0",
		CONFIG_FLASH_BASE_ADDRESS,
		REGION_FLASH_ATTR(CONFIG_FLASH_BASE_ADDRESS, \
			CONFIG_FLASH_SIZE * 1024)),
	/* Region 1 */
	MPU_REGION_ENTRY("SRAM_0",
		CONFIG_SRAM_BASE_ADDRESS,
		REGION_RAM_ATTR(CONFIG_SRAM_BASE_ADDRESS, \
			CONFIG_SRAM_SIZE * 1024)),
	/* Region 2 */
	MPU_REGION_ENTRY("SRAM_X",
		DT_INST_5_MMIO_SRAM_BASE_ADDRESS,
		REGION_RAM_ATTR(DT_INST_5_MMIO_SRAM_BASE_ADDRESS, \
			DT_INST_5_MMIO_SRAM_SIZE * 1024)),
};

const struct arm_mpu_config mpu_config = {
	.num_regions = ARRAY_SIZE(mpu_regions),
	.mpu_regions = mpu_regions,
};
