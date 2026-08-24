/*
 * Copyright (c) 2026 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Virtual memory manager page related symbols
 *
 * Contains VMM related page relate symbols.
 */

#ifndef ZEPHYR_INCLUDE_MEM_MGMT_VMM_PAGES_H_
#define ZEPHYR_INCLUDE_MEM_MGMT_VMM_PAGES_H_

/**
 * @ingroup mem_mgmt
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Status of a particular page location.
 */
enum vmm_page_location {
	/** The page has been evicted to the backing store. */
	VMM_PAGE_LOCATION_PAGED_OUT,

	/** The page is resident in memory. */
	VMM_PAGE_LOCATION_PAGED_IN,

	/** The page is not mapped. */
	VMM_PAGE_LOCATION_BAD
};

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_MEM_MGMT_VMM_PAGES_H_ */
