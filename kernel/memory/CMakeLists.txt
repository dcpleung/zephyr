# SPDX-License-Identifier: Apache-2.0
# Copyright (c) The Zephyr Project Contributors

target_sources(kernel PRIVATE
  kheap.c
  mem_slab.c
  )

target_sources_ifdef(CONFIG_MMU kernel PRIVATE mmu.c)

target_sources_ifdef(CONFIG_DEMAND_PAGING_STATS kernel PRIVATE
  paging/statistics.c
  )

if(${CONFIG_DYNAMIC_THREAD})
  target_sources(kernel PRIVATE dynamic.c)
else()
  target_sources(kernel PRIVATE dynamic_disabled.c)
endif()

if(${CONFIG_KERNEL_MEM_POOL})
  target_sources(kernel PRIVATE mempool.c)

  if(CONFIG_HEAP_MEM_POOL_IGNORE_MIN)
    set(final_heap_size ${CONFIG_HEAP_MEM_POOL_SIZE})
  else()
    # Import all custom HEAP_MEM_POOL size requirements
    import_kconfig(CONFIG_HEAP_MEM_POOL_ADD_SIZE_ ${DOTCONFIG} add_size_keys)

    # Calculate the sum of all "ADD_SIZE" requirements
    set(add_size_sum 0)
    foreach(add_size ${add_size_keys})
      math(EXPR add_size_sum "${add_size_sum} + ${${add_size}}")
    endforeach()

    if(CONFIG_HEAP_MEM_POOL_SIZE LESS "${add_size_sum}")
      # Only warn if default value 0 has been modified
      if(NOT CONFIG_HEAP_MEM_POOL_SIZE EQUAL 0)
        message(WARNING "
        CONFIG_HEAP_MEM_POOL_SIZE is less than requested minimum:
          ${CONFIG_HEAP_MEM_POOL_SIZE} < ${add_size_sum}
        Setting the system heap size to ${add_size_sum}")
      endif()

      set(final_heap_size ${add_size_sum})
    else()
      # CONFIG_HEAP_MEM_POOL_SIZE was greater than the sum of the requirements
      set(final_heap_size ${CONFIG_HEAP_MEM_POOL_SIZE})
    endif()

  endif()

  zephyr_compile_definitions(K_HEAP_MEM_POOL_SIZE=${final_heap_size})
endif()
