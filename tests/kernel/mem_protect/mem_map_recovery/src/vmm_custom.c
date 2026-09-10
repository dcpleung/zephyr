/*
 * Copyright (c) 2026 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel_arch_interface.h>
#include <zephyr/mem_mgmt/vmm.h>
#include <errno.h>

#include "vmm_custom.h"

/** Whether to fake mapping failure */
static bool vmm_map_fake_fail;

/** Number of times mapping should succeed before faking a failure */
static int vmm_map_num_till_fail;

/** Number of times mapping succeeded before emitting a fake failure */
static int vmm_map_num_succeed;

/** Whether to fake unmapping failure */
static bool vmm_unmap_fake_fail;

/** Number of times unmapping should succeed before faking a failure */
static int vmm_unmap_num_till_fail;

/** Number of times unmapping succeeded before emitting a fake failure */
static int vmm_unmap_num_succeed;

/** Error number to be emitted with fake failure */
static int vmm_failure_errno;

void vmm_fake_failure_reset(void)
{
	vmm_map_fake_fail = false;
	vmm_unmap_fake_fail = false;
}

void vmm_fake_failure_setup(bool map_fail, int map_num_till_fail, bool unmap_fail,
			    int unmap_num_till_fail, int emit_errno)
{
	vmm_map_fake_fail = map_fail;
	vmm_map_num_till_fail = map_num_till_fail;
	vmm_map_num_succeed = 0;

	vmm_unmap_fake_fail = unmap_fail;
	vmm_unmap_num_till_fail = unmap_num_till_fail;
	vmm_unmap_num_succeed = 0;

	vmm_failure_errno = emit_errno;
}

int vmm_map_num_succeed_get(void)
{
	return vmm_map_num_succeed;
}

int vmm_unmap_num_succeed_get(void)
{
	return vmm_unmap_num_succeed;
}

void vmm_impl_init(void)
{
	vmm_map_fake_fail = false;
	vmm_map_num_till_fail = 0;
	vmm_map_num_succeed = 0;

	vmm_unmap_fake_fail = false;
	vmm_unmap_num_till_fail = 0;
	vmm_unmap_num_succeed = 0;

	vmm_failure_errno = -ENOTSUP;
}

int vmm_impl_mem_map(void *virt, uintptr_t phys, size_t size, uint32_t flags)
{
	if (vmm_map_fake_fail) {
		if (vmm_map_num_till_fail == 0) {
			vmm_map_fake_fail = false;

			return vmm_failure_errno;
		}

		vmm_map_num_till_fail--;
		vmm_map_num_succeed++;
	}

	return arch_mem_map(virt, phys, size, flags);
}

int vmm_impl_mem_unmap(void *addr, size_t size)
{
	if (vmm_unmap_fake_fail) {
		if (vmm_unmap_num_till_fail == 0) {
			vmm_unmap_fake_fail = false;

			return vmm_failure_errno;
		}

		vmm_unmap_num_till_fail--;
		vmm_unmap_num_succeed++;
	}

	return arch_mem_unmap(addr, size);
}

int vmm_impl_page_phys_get(void *virt, uintptr_t *phys)
{
	return arch_page_phys_get(virt, phys);
}

void vmm_impl_reserved_pages_update(void)
{
	arch_reserved_pages_update();
}

void vmm_impl_mem_page_out(void *addr, uintptr_t location)
{
	arch_mem_page_out(addr, location);
}

void vmm_impl_mem_page_in(void *addr, uintptr_t phys)
{
	arch_mem_page_in(addr, phys);
}

void vmm_impl_mem_scratch(uintptr_t phys)
{
	arch_mem_scratch(phys);
}

enum vmm_page_location vmm_impl_page_location_get(void *addr, uintptr_t *location)
{
	return arch_page_location_get(addr, location);
}

uintptr_t vmm_impl_page_info_get(void *addr, uintptr_t *location, bool clear_accessed)
{
	return arch_page_info_get(addr, location, clear_accessed);
}
