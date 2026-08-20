/*
 * Copyright (c) 2026 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System Virtual Memory Manager interface
 *
 * APIs for the system virtual memory manager which handles
 * virtual memory.
 */

#ifndef ZEPHYR_INCLUDE_MEM_MGMT_VMM_IMPL_H_
#define ZEPHYR_INCLUDE_MEM_MGMT_VMM_IMPL_H_

#ifndef ZEPHYR_INCLUDE_MEM_MGMT_VMM_H_
#error  Please do not include this header directly, use <zephyr/mem_mgmt/vmm.h> instead
#endif

#include <stdint.h>
#include <stdio.h>

/**
 * @brief Virtual Memory Manager Implementation
 * @defgroup mem_mgmt_vmm_impl Virtual Memory Manager Implementation
 * @ingroup mem_mgmt_vmm
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_MEM_MGMT_VMM_IMPL_ARCH)

/* Forward to the architecture-specific implementation. */
#include <zephyr/mem_mgmt/vmm/impl/vmm_impl_arch.h>

#elif defined(CONFIG_MEM_MGMT_VMM_IMPL_NONE)

/* No virtual memory manager. All vmm_*() will return -ENOTSUP or simply return. */
#include <zephyr/mem_mgmt/vmm/impl/vmm_impl_none.h>

#else
#error "No CONFIG_MEM_MGMT_VMM_IMPL_* selected"
#endif

/**
 * Map a physical memory region into the virtual address space.
 *
 * This is a low-level interface to mapping pages into the address space.
 * Behavior when providing unaligned addresses/sizes is undefined, these
 * are assumed to be aligned to CONFIG_MMU_PAGE_SIZE.
 *
 * Management of virtual address space is handled outside of this function.
 * By the time this function is invoked, we know exactly where this mapping
 * will be established. If the pages have already been mapped for
 * the virtual memory region, they will be overwritten.
 *
 * The memory range itself is never accessed by this operation.
 *
 * This API must be safe to call in ISRs or exception handlers. Calls
 * to this API are assumed to be serialized, and indeed all usage will
 * originate from functions which handles virtual memory management.
 *
 * @param virt Page-aligned destination virtual address to map
 * @param phys Page-aligned source physical address to map
 * @param size Page-aligned size of the mapped region in bytes
 * @param flags Caching, access and control flags (see K_MEM_* macros)
 *
 * @retval 0 On success
 * @retval -EINVAL If invalid arguments are given (for example, zero size)
 * @retval -ENOTSUP If the requested cache/attribute combination in @a flags
 *                  is not supported by the target architecture
 * @retval -ENOMEM If the memory mapping fails
 * @retval -errno Other negative error codes may be returned by
 *                architecture-specific implementations for other failure modes
 */
int vmm_impl_mem_map(void *virt, uintptr_t phys, size_t size, uint32_t flags);

/**
 * Remove mappings for a provided virtual address range
 *
 * This is a low-level interface for un-mapping pages from the address space.
 * When this completes, the relevant mapped memory pages will be updated as
 * if no mapping was ever made for that memory range. No previous context
 * needs to be preserved.
 *
 * Behavior when providing unaligned addresses/sizes is undefined, these
 * are assumed to be aligned to CONFIG_MMU_PAGE_SIZE.
 *
 * Behavior when providing an address range that is not already mapped is
 * undefined.
 *
 * @param addr Page-aligned base virtual address to un-map
 * @param size Page-aligned region size
 *
 * @retval 0 On success
 * @retval -EINVAL If invalid arguments are given (for example, zero size)
 * @retval -errno Other negative error codes may be returned by
 *                architecture-specific implementations for other failure modes
 */
int vmm_impl_mem_unmap(void *addr, size_t size);

/**
  * Get the mapped physical memory address from virtual address.
  *
  * @param[in]  virt Page-aligned virtual address
  * @param[out] phys Mapped physical address (can be NULL if only checking
  *                  if virtual address is mapped)
  *
  * @retval 0 if mapping is found and valid
  * @retval -EFAULT if virtual address is not mapped
  */
int vmm_impl_page_phys_get(void *virt, uintptr_t *phys);

/**
 * Mark reserved page frames in the page frame database.
 *
 * Some page frames within system RAM are not available for use (for example
 * reserved regions in the first megabyte on PC-like systems).
 *
 * Implementations of this function should mark all relevant entries in
 * k_mem_page_frames with K_PAGE_FRAME_RESERVED. This function is called at
 * early system initialization.
 */
void vmm_impl_reserved_pages_update(void);

/**
 * Update all memory mappings for a paged-out data page
 *
 * This function:
 * - Sets the data page virtual address to trigger a fault if accessed that
 *   can be distinguished from access violations or un-mapped pages.
 * - Saves the provided location value so that it can retrieved for that
 *   data page in the page fault handler.
 * - The location value semantics are undefined here but the value will be
 *   always be page-aligned. It could be 0.
 *
 * This function is called with interrupts locked.
 *
 * Calling this function on data pages which are already paged out is
 * undefined behavior.
 *
 * @param addr Virtual data page address being evicted
 * @param location Backing store location value to associate with the page
 */
void vmm_impl_mem_page_out(void *addr, uintptr_t location);

/**
 * Update all memory mappings for a paged-in data page
 *
 * This function:
 * - Maps the specified virtual data page address to the provided physical
 *   page frame address, such that future memory accesses will function as
 *   expected. Access and caching attributes are undisturbed.
 * - Clears any accounting for "accessed" and "dirty" states.
 *
 * This function is called with interrupts locked.
 *
 * @param addr Virtual data page address being paged in
 * @param phys Physical page frame address to map it to
 */
void vmm_impl_mem_page_in(void *addr, uintptr_t phys);

/**
 * Temporarily map a physical page frame for scratch access.
 *
 * Map the physical page @p phys to a special virtual address
 * K_MEM_SCRATCH_PAGE, with supervisor read/write access, such that
 * when this function returns, the calling context can read/write the page
 * frame's contents from the K_MEM_SCRATCH_PAGE address.
 *
 * This function is called with interrupts locked.
 *
 * @param phys Physical page frame address to map at the scratch page
 */
void vmm_impl_mem_scratch(uintptr_t phys);

/**
 * Fetch location information about a page at a particular address
 *
 * This function is called with interrupts locked, so that the reported
 * information can't become stale while decisions are being made based on it.
 *
 * @param[in]  addr     Virtual data page address that took the page fault
 * @param[out] location In the case of ARCH_PAGE_LOCATION_PAGED_OUT, the backing store
 *                      location value used to retrieve the data page. In the case of
 *                      ARCH_PAGE_LOCATION_PAGED_IN, the physical address the page is
 *                      mapped to.
 *
 * @retval ARCH_PAGE_LOCATION_PAGED_OUT The page was evicted to the backing store
 * @retval ARCH_PAGE_LOCATION_PAGED_IN The page is resident in memory
 * @retval ARCH_PAGE_LOCATION_BAD The page is un-mapped or otherwise has had invalid access
 */
enum arch_page_location vmm_impl_page_location_get(void *addr, uintptr_t *location);

/**
 * Retrieve page characteristics of the mapped memory
 *
 * For the provided virtual address, report the logical OR of the accessed
 * and dirty states for the relevant entries in the system if the page is
 * mapped and not paged out.
 *
 * If @p clear_accessed is true, the ARCH_DATA_PAGE_ACCESSED flag will be reset.
 * This function will report its prior state. If multiple mappings are in
 * use, this function clears accessed state in all of them.
 *
 * This function is called with interrupts locked, so that the reported
 * information can't become stale while decisions are being made based on it.
 *
 * The return value may have other bits set which the caller must ignore.
 *
 * Clearing accessed state for data pages that are not ARCH_DATA_PAGE_LOADED
 * is undefined behavior.
 *
 * ARCH_DATA_PAGE_DIRTY and ARCH_DATA_PAGE_ACCESSED bits in the return value
 * are only significant if ARCH_DATA_PAGE_LOADED is set, otherwise ignore
 * them.
 *
 * ARCH_DATA_PAGE_NOT_MAPPED bit in the return value is only significant
 * if ARCH_DATA_PAGE_LOADED is un-set, otherwise ignore it.
 *
 * @param[in]  addr           Virtual address of the page to get information on
 * @param[out] location       If non-NULL, updated with either physical page frame
 *                            address or backing store location depending on
 *                            ARCH_DATA_PAGE_LOADED state. This is not touched if
 *                            ARCH_DATA_PAGE_NOT_MAPPED.
 * @param[in]  clear_accessed Whether to clear ARCH_DATA_PAGE_ACCESSED state
 *
 * @retval Value with ARCH_DATA_PAGE_* bits set reflecting the data page
 *         configuration
 */
uintptr_t vmm_impl_page_info_get(void *addr, uintptr_t *location, bool clear_accessed);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* ZEPHYR_INCLUDE_MEM_MGMT_VMM_IMPL_H_ */
