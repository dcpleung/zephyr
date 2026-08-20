/*
 * Copyright (c) 2026 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Architecture bypass for virtual memory manager implementation.
 *
 * This virtual memory manager implementation simply bypasses function calls
 * to the architecture-specific arch_mem_*() implementation. Using this
 * effectively bypasses the virtual memory manager.
 */

#ifndef ZEPHYR_INCLUDE_MEM_MGMT_VMM_IMPL_ARCH_H_
#define ZEPHYR_INCLUDE_MEM_MGMT_VMM_IMPL_ARCH_H_

#ifndef ZEPHYR_INCLUDE_MEM_MGMT_VMM_IMPL_H_
#error  Please do not include this header directly, use <zephyr/mem_mgmt/vmm.h> instead
#endif

#include <zephyr/toolchain.h>
#include <kernel_arch_interface.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */

static ALWAYS_INLINE int vmm_impl_mem_map(void *virt, uintptr_t phys, size_t size, uint32_t flags)
{
	return arch_mem_map(virt, phys, size, flags);
}

static ALWAYS_INLINE int vmm_impl_mem_unmap(void *addr, size_t size)
{
	return arch_mem_unmap(addr, size);
}

static ALWAYS_INLINE int vmm_impl_page_phys_get(void *virt, uintptr_t *phys)
{
	return arch_page_phys_get(virt, phys);
}

static ALWAYS_INLINE void vmm_impl_reserved_pages_update(void)
{
	arch_reserved_pages_update();
}

static ALWAYS_INLINE void vmm_impl_mem_page_out(void *addr, uintptr_t location)
{
	arch_mem_page_out(addr, location);
}

static ALWAYS_INLINE void vmm_impl_mem_page_in(void *addr, uintptr_t phys)
{
	arch_mem_page_in(addr, phys);
}

static ALWAYS_INLINE void vmm_impl_mem_scratch(uintptr_t phys)
{
	arch_mem_scratch(phys);
}

static ALWAYS_INLINE enum arch_page_location
vmm_impl_page_location_get(void *addr, uintptr_t *location)
{
	return arch_page_location_get(addr, location);
}

static ALWAYS_INLINE uintptr_t vmm_impl_page_info_get(void *addr, uintptr_t *location,
						      bool clear_accessed)
{
	return arch_page_info_get(addr, location, clear_accessed);
}

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_MEM_MGMT_VMM_IMPL_ARCH_H_ */
