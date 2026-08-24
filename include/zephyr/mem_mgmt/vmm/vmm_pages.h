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

#include <zephyr/sys/util.h>

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

/**
 * Bit indicating the data page was accessed since the value was last cleared.
 *
 * Used by marking eviction algorithms. Safe to set this if uncertain.
 *
 * This bit is undefined if VMM_DATA_PAGE_LOADED is not set.
 */
#define VMM_DATA_PAGE_ACCESSED BIT(0)

 /**
  * Bit indicating the data page, if being evicted, will need to be paged out.
  *
  * Set if the data page was modified since it was last paged out, or if
  * it has never been paged out before. Safe to set this if uncertain.
  *
  * This bit is undefined if VMM_DATA_PAGE_LOADED is not set.
  */
#define VMM_DATA_PAGE_DIRTY BIT(1)

 /**
  * Bit indicating that the data page is loaded into a physical page frame.
  *
  * If un-set, the data page is paged out or not mapped.
  */
#define VMM_DATA_PAGE_LOADED BIT(2)

/**
 * Bit indicating that the data page is not mapped.
 *
 * If VMM_DATA_PAGE_LOADED is un-set, this will indicate that the page
 * is not mapped at all. This bit is undefined if VMM_DATA_PAGE_LOADED is set.
 */
#define VMM_DATA_PAGE_NOT_MAPPED BIT(3)

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_MEM_MGMT_VMM_PAGES_H_ */
