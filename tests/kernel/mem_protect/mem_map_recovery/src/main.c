/*
 * Copyright (c) 2026 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/toolchain.h>
#include <mmu.h>
#include <zephyr/linker/sections.h>
#include <zephyr/cache.h>

#include "vmm_custom.h"

/** Number of pages to map */
#define NUM_PAGES_TO_MAP (5)

/**
 * Size to be mapped.
 *
 * Note that this must be at least a few pages so we can
 * fake a mapping failure in the middle.
 */
#define MAPPING_SZ (CONFIG_MMU_PAGE_SIZE * NUM_PAGES_TO_MAP)

/**
 * Find out how many pages are actually mapped.
 *
 * @param virt Starting virtual address
 * @param num_pages_to_check Number of pages to check
 *
 * @return Number of pages mapped
 */
static unsigned int calc_num_mapped(const char *virt, const unsigned int num_pages_to_check)
{
	unsigned int num_mapped = 0;
	int ret;

	for (int i = 0; i < num_pages_to_check; i++) {
		uintptr_t chk_addr = (uintptr_t)virt + (CONFIG_MMU_PAGE_SIZE * i);

		ret = vmm_impl_page_phys_get((void *)chk_addr, NULL);
		if (ret == 0) {
			num_mapped++;
		}
	}

	return num_mapped;
}

/**
 * Match the number of mapped pages.
 *
 * Calculate the number of mapped pages with a region and match it to an expected
 * value.
 *
 * @param virt Starting virtual address
 * @param num_pages_to_check Number of pages to check
 * @param expected_num_pages Expected number of mapped pages
 */
static void match_num_mapped(const char *virt, const unsigned int num_pages_to_check,
			     const unsigned int expected_num_pages)
{
	if (!IS_ENABLED(CONFIG_DEMAND_PAGING)) {
		/* Check to make sure no pages are actually mapped in physical memory.
		 *
		 * Note that with demand paging, it is possible that no pages are mapped to
		 * physical memory so we cannot use this to test correctness. And...
		 * obviously we cannot memset() to force paging them in as those pages are
		 * supposed to not be mapped.
		 */
		unsigned int num_mapped = calc_num_mapped(virt, NUM_PAGES_TO_MAP);

		zassert_equal(num_mapped, expected_num_pages,
			      "Should have %u pages mapped but got %u", expected_num_pages,
			      num_mapped);
	}
}

/**
 * Fill the mapped memory with predetermined value.
 *
 * @param virt Starting virtual memory address
 */
static void fill_mapped_memory(const char *virt, const unsigned int num_pages)
{
	/* When demand paging is enabled, it is possible the anonymously
	 * mapped memory is not actually mapped to physical memory. So
	 * we need to write some data to these pages to make sure they are
	 * taking up some physical memory.
	 *
	 * Although filling the memory is not exactly required when demand
	 * paging is not enabled, it is a good idea to access those memory
	 * to make sure they are actually mapped.
	 */
	for (int i = 0; i < num_pages; i++) {
		uintptr_t chk_addr = (uintptr_t)virt + (CONFIG_MMU_PAGE_SIZE * i);

		memset((void *)chk_addr, 0xAA, CONFIG_MMU_PAGE_SIZE);
	}
}

/**
 * Try a mapping and unmapping pair to make sure they can succeed.
 *
 * @param[in] free_mem Amount of free memory to match after mapping/unmapping.
 * @param[out] mapped_addr The virtual address of mapped memory. It is being
 *                         used in the test to check if the allocated virtual
 *                         memory region has been freed. Do not use this
 *                         address to access memory directly.
 */
static void precheck_map_unmap(size_t free_mem, char **mapped_addr)
{
	size_t free_mem_after_map, free_mem_after_unmap;
	void *mapped;

	/*
	 * Try a successful mapping/unmapping pair first.
	 *
	 * This is to make sure the pair works without issue before
	 * we try the fake failure path. Also we get the virtual address
	 * for this mapped region, as we need this to test if the failed
	 * mapping releases the allocated virtual memory region.
	 */
	printk("Try successful mapping/unmapping pair...\n");
	vmm_fake_failure_reset();

	mapped = k_mem_map(MAPPING_SZ, K_MEM_PERM_RW);
	zassert_not_null(mapped, "memory mapping should not fail");
	printk("- mapped a page to %p\n", mapped);

	*mapped_addr = mapped;

	fill_mapped_memory(mapped, NUM_PAGES_TO_MAP);

	/* Check to make sure all pages are actually mapped in physical memory. */
	match_num_mapped(mapped, NUM_PAGES_TO_MAP, NUM_PAGES_TO_MAP);

	/* There should be less free memory after successful mapping. */
	free_mem_after_map = k_mem_free_get();
	printk("- Free memory after successful mapping: %zu\n", free_mem_after_map);
	zassert_not_equal(free_mem, free_mem_after_map, "successful mapping should take up memory");

	/* Free the memory and see if we have the same free memory before
	 * the mapping above.
	 */
	k_mem_unmap(mapped, MAPPING_SZ);
	free_mem_after_unmap = k_mem_free_get();
	zassert_equal(free_mem, free_mem_after_unmap, "k_mem_unmap has not freed memory");
	printk("- Free memory after successful unmapping: %zu\n", free_mem_after_unmap);

	/* Check to make sure all pages are unmapped in physical memory. */
	match_num_mapped(mapped, NUM_PAGES_TO_MAP, 0);
}

/**
 * Test the failure recovery path for anonymous memory in k_mem_map().
 */
ZTEST(mem_map_api, test_anon_mem_map_anon_recovery)
{
	size_t free_mem_before, free_mem_after_map;
	char *mapped, *last_successful_mapped;
	int op_num_succeed;

	free_mem_before = k_mem_free_get();
	zassert_not_equal(free_mem_before, 0, "no free memory");
	printk("Free memory at the beginning: %zu\n", free_mem_before);

	precheck_map_unmap(free_mem_before, &last_successful_mapped);

	/* Note that when a mapping fails, no page frames should be taken after recovering
	 * from failed mapping, such that the free memory size should be the same before
	 * and after the failed mapping attempt. So we check for free memory size and
	 * number of mapped pages (which should be 0).
	 */

	/* Try to fake a failure immediately without mapping any pages. */
	printk("Try immediately failed mapping...\n");
	vmm_fake_failure_setup(true, 0, false, 0, -ENOMEM);

	mapped = k_mem_map(MAPPING_SZ, K_MEM_PERM_RW);
	zassert_is_null(mapped, "memory mapping should fail");

	op_num_succeed = vmm_map_num_succeed_get();
	zassert_equal(op_num_succeed, 0, "should not have succeed in any mappings (%d)",
		      op_num_succeed);

	match_num_mapped(last_successful_mapped, NUM_PAGES_TO_MAP, 0);

	free_mem_after_map = k_mem_free_get();
	printk("- Free memory after failed mapping: %zu\n", free_mem_after_map);
	zassert_equal(free_mem_before, free_mem_after_map,
		      "failed mapping should not take up memory");

	/* Try to fake a failure when allocating the 3rd page, out of NUM_PAGES_TO_MAP. */
	printk("Try halfway failed mapping...\n");
	vmm_fake_failure_setup(true, 2, false, 0, -ENOMEM);

	mapped = k_mem_map(MAPPING_SZ, K_MEM_PERM_RW);
	zassert_is_null(mapped, "memory mapping should fail");

	op_num_succeed = vmm_map_num_succeed_get();
	zassert_equal(op_num_succeed, 2, "should have succeed in some mappings (%d)",
		      op_num_succeed);

	match_num_mapped(last_successful_mapped, NUM_PAGES_TO_MAP, 0);

	free_mem_after_map = k_mem_free_get();
	printk("- Free memory after failed mapping: %zu\n", free_mem_after_map);
	zassert_equal(free_mem_before, free_mem_after_map,
		      "failed mapping should not take up memory");

	/* Return to normal operation */
	vmm_fake_failure_reset();

	/* Map again to see if this mapping returns the same address as
	 * the first mapping done above. If the virtual memory region is released
	 * correctly during failed mapping, the addresses should match.
	 */
	mapped = k_mem_map(MAPPING_SZ, K_MEM_PERM_RW);
	zassert_not_null(mapped, "memory mapping should not fail");

	zassert_equal(mapped, last_successful_mapped,
		      "virtual memory region not released (%p != %p)", mapped,
		      last_successful_mapped);

	k_mem_unmap(mapped, MAPPING_SZ);

	free_mem_after_map = k_mem_free_get();
	printk("- Free memory after cleanup: %zu\n", free_mem_after_map);
	zassert_equal(free_mem_before, free_mem_after_map,
		      "cleanup should free up all used memory");
}

/**
 * Test the failure recovery path for anonymous memory in k_mem_unmap().
 */
ZTEST(mem_map_api, test_anon_mem_unmap_anon_recovery)
{
	size_t free_mem_before, free_mem_after_map, free_mem_after_unmap;
	char *mapped, *last_successful_mapped, *finishing_mapped;
	int op_num_succeed;

	free_mem_before = k_mem_free_get();
	zassert_not_equal(free_mem_before, 0, "no free memory");
	printk("Free memory at the beginning: %zu\n", free_mem_before);

	precheck_map_unmap(free_mem_before, &last_successful_mapped);

	/* Map the memory so we can try the unmapping recovery path. */
	vmm_fake_failure_reset();
	mapped = k_mem_map(MAPPING_SZ, K_MEM_PERM_RW);
	zassert_not_null(mapped, "memory mapping should not fail");
	fill_mapped_memory(mapped, NUM_PAGES_TO_MAP);

	free_mem_after_map = k_mem_free_get();
	printk("- Free memory after successful mapping: %zu\n", free_mem_after_map);
	zassert(free_mem_before > free_mem_after_map,
		"successful mapping should take up some free memory");

	/* Try to fake a failure immediately without mapping any pages. */
	printk("Try immediately failed unmapping...\n");
	vmm_fake_failure_setup(false, 0, true, 0, -ENOMEM);

	k_mem_unmap(mapped, MAPPING_SZ);

	op_num_succeed = vmm_unmap_num_succeed_get();
	zassert_equal(op_num_succeed, 0, "should not have succeed in any unmappings (%d)",
		      op_num_succeed);

	match_num_mapped(last_successful_mapped, NUM_PAGES_TO_MAP, NUM_PAGES_TO_MAP);

	free_mem_after_unmap = k_mem_free_get();
	printk("- Free memory after failed unmapping: %zu\n", free_mem_after_unmap);
	zassert_equal(free_mem_after_map, free_mem_after_unmap,
		      "failed unmapping should not free any memory");

	/* Try to fake a failure when allocating the 3rd page, out of NUM_PAGES_TO_MAP. */
	printk("Try halfway failed mapping...\n");
	vmm_fake_failure_setup(false, 0, true, 2, -ENOMEM);

	k_mem_unmap(mapped, MAPPING_SZ);

	op_num_succeed = vmm_unmap_num_succeed_get();
	zassert_equal(op_num_succeed, 2, "should have succeed in some unmappings (%d)",
		      op_num_succeed);

	/* Since we are failing the 3rd unmapping, so we have already unmapped 2 pages,
	 * and 3 pages remained unmapped.
	 */
	match_num_mapped(last_successful_mapped, NUM_PAGES_TO_MAP, 3);

	free_mem_after_unmap = k_mem_free_get();
	printk("- Free memory after failed unmapping: %zu\n", free_mem_after_unmap);
	zassert_equal(free_mem_after_map, free_mem_after_unmap - (CONFIG_MMU_PAGE_SIZE * 2),
		      "failed unmapping should have freed exactly 2 pages of memory");

	/* Check to see if the virtual memory region allocated above is not released
	 * during failed unmapping.
	 */
	precheck_map_unmap(free_mem_after_unmap, &finishing_mapped);
	zassert_not_equal(last_successful_mapped, finishing_mapped,
			  "new mapping should have differnet virtual address");

	/* Note that we should leave the mapped region untouched. Due to some pages are
	 * unmapped but not all, it is hard to predict what would happen if the unmapping
	 * code tries to unmap already unmapped memory.
	 */

	/* Return to normal operation */
	vmm_fake_failure_reset();
}

/* ztest main entry*/
void *mem_map_env_setup(void)
{
	return NULL;
}

/* For CPUs with incoherent cache under SMP, the tests to read/write
 * buffer (... majority of tests here) may not work correctly if
 * the test thread jumps between CPUs. So use the test infrastructure
 * to limit the test to 1 CPU.
 */
#ifdef CONFIG_CPU_CACHE_INCOHERENT
#define FUNC_BEFORE ztest_simple_1cpu_before
#define FUNC_AFTER  ztest_simple_1cpu_after
#else
#define FUNC_BEFORE NULL
#define FUNC_AFTER  NULL
#endif

ZTEST_SUITE(mem_map_api, NULL, mem_map_env_setup, FUNC_BEFORE, FUNC_AFTER, NULL);
