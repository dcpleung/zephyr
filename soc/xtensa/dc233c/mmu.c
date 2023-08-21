/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <xtensa/config/core-isa.h>
#include <zephyr/arch/xtensa/xtensa_mmu.h>
#include <zephyr/sys/util.h>

const struct xtensa_mmu_range xtensa_soc_mmu_ranges[] = {
	{
		.start = (uint32_t)XCHAL_VECBASE_RESET_VADDR,
		.end   = (uint32_t)XCHAL_VECBASE_RESET_VADDR + 0x1000U,
		.attrs = Z_XTENSA_MMU_X | Z_XTENSA_MMU_CACHED_WB,
		.name = "vecbase",
	},
	{
		/* The ROM is 32MB but the address wraps around back to 0x00000000.
		 * So just skip the last page so we don't have to deal with integer
		 * overflow.
		 */
		.start = (uint32_t)XCHAL_RESET_VECTOR_VADDR,
		.end   = (uint32_t)XCHAL_RESET_VECTOR_VADDR + 0x2000U,
		.attrs = Z_XTENSA_MMU_X | Z_XTENSA_MMU_CACHED_WB,
		.name = "rom",
	},
};

int xtensa_soc_mmu_ranges_num = ARRAY_SIZE(xtensa_soc_mmu_ranges);
