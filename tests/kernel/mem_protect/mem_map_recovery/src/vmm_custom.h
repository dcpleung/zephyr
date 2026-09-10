/*
 * Copyright (c) 2026 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_TESTS_KERNEL_MEM_PROTECT_MEM_MAP_RECOVERY_SRC_VMM_CUSTOM_H
#define ZEPHYR_TESTS_KERNEL_MEM_PROTECT_MEM_MAP_RECOVERY_SRC_VMM_CUSTOM_H

#include <kernel_arch_interface.h>
#include <zephyr/mem_mgmt/vmm.h>

/**
 * Reset the failure faking so mapping and unmapping always succeed.
 */
void vmm_fake_failure_reset(void);

/**
 * Setup the custom VMM to fake error after a number of mapping or unmapping operations.
 *
 * @param map_fail True if mapping should fail, false otherwise.
 * @param map_num_till_fail Number of times mapping should succeed before fail.
 * @param unmap_fail True if unmapping should fail, false otherwise.
 * @param unmap_num_till_fail Number of times unmapping should succeed before fail.
 * @param emit_errno Error to be emitted for failed mapping/unmapping.
 */
void vmm_fake_failure_setup(bool map_fail, int map_num_till_fail, bool unmap_fail,
			    int unmap_num_till_fail, int emit_errno);

/**
 * Get the number of times mapping has succeeded.
 */
int vmm_map_num_succeed_get(void);

/**
 * Get the number of times unmapping has succeeded.
 */
int vmm_unmap_num_succeed_get(void);

#endif /* ZEPHYR_TESTS_KERNEL_MEM_PROTECT_MEM_MAP_RECOVERY_SRC_VMM_CUSTOM_H */
