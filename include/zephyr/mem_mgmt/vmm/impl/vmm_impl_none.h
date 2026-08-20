/*
 * Copyright (c) 2026 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Null implementation of virtual memory manager.
 *
 * All vmm_impl_*() APIs will return -ENOTSUP or do nothing.
 */

#ifndef ZEPHYR_INCLUDE_MEM_MGMT_VMM_IMPL_NONE_H_
#define ZEPHYR_INCLUDE_MEM_MGMT_VMM_IMPL_NONE_H_

#ifndef ZEPHYR_INCLUDE_MEM_MGMT_VMM_IMPL_H_
#error  Please do not include this header directly, use <zephyr/mem_mgmt/vmm.h> instead
#endif

#include <errno.h>
#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */

static ALWAYS_INLINE int vmm_impl_mem_map(void *virt, uintptr_t phys, size_t size, uint32_t flags)
{
	ARG_UNUSED(virt);
	ARG_UNUSED(phys);
	ARG_UNUSED(size);
	ARG_UNUSED(flags);

	return -ENOTSUP;
}

static ALWAYS_INLINE int vmm_impl_mem_unmap(void *addr, size_t size)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(size);

	return -ENOTSUP;
}

static ALWAYS_INLINE int vmm_impl_page_phys_get(void *virt, uintptr_t *phys)
{
	ARG_UNUSED(virt);
	ARG_UNUSED(phys);

	return -ENOTSUP;
}

static ALWAYS_INLINE void vmm_impl_reserved_pages_update(void)
{
	return;
}

static ALWAYS_INLINE void vmm_impl_mem_page_out(void *addr, uintptr_t location)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(location);

	return;
}

static ALWAYS_INLINE void vmm_impl_mem_page_in(void *addr, uintptr_t phys)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(phys);

	return;
}

static ALWAYS_INLINE void vmm_impl_mem_scratch(uintptr_t phys)
{
	ARG_UNUSED(phys);

	return;
}

static ALWAYS_INLINE enum arch_page_location
vmm_impl_page_location_get(void *addr, uintptr_t *location)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(location);

	return -ENOTSUP;
}

static ALWAYS_INLINE uintptr_t vmm_impl_page_info_get(void *addr, uintptr_t *location,
						 bool clear_accessed)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(location);
	ARG_UNUSED(clear_accessed);

	return -ENOTSUP;
}

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_MEM_MGMT_VMM_IMPL_NONE_H_ */
