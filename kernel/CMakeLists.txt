# SPDX-License-Identifier: Apache-2.0

# kernel is a normal CMake library and not a zephyr_library because it
# should usually not be --whole-archive'd

zephyr_syscall_header(
  ${ZEPHYR_BASE}/include/zephyr/device.h
  ${ZEPHYR_BASE}/include/zephyr/kernel.h
  ${ZEPHYR_BASE}/include/zephyr/sys/kobject.h
  ${ZEPHYR_BASE}/include/zephyr/sys/time_units.h
)

if(NOT CONFIG_ERRNO_IN_TLS AND NOT CONFIG_LIBC_ERRNO)
  zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/sys/errno_private.h)
endif()

zephyr_syscall_header_ifdef(
  CONFIG_ATOMIC_OPERATIONS_C
  ${ZEPHYR_BASE}/include/zephyr/sys/atomic_c.h
)

zephyr_syscall_header_ifdef(
  CONFIG_MMU
  ${ZEPHYR_BASE}/include/zephyr/kernel/mm.h
  ${ZEPHYR_BASE}/include/zephyr/sys/mem_manage.h
)

zephyr_syscall_header_ifdef(
  CONFIG_DEMAND_PAGING
  ${ZEPHYR_BASE}/include/zephyr/kernel/mm/demand_paging.h
)

# If a pre-built static library containing kernel code exists in
# this directory, libkernel.a, link it with the application code
# instead of building from source.
zephyr_library_get_current_dir_lib_name(${ZEPHYR_BASE} libkernel_stem)
set(libkernel ${CMAKE_CURRENT_SOURCE_DIR}/lib${libkernel_stem}${CMAKE_STATIC_LIBRARY_SUFFIX})
unset(libkernel_stem)

if(EXISTS ${libkernel})

add_library(kernel INTERFACE)
target_link_libraries(kernel INTERFACE ${libkernel})

else()
# Own the kernel target in this directory so subdirectories can
# contribute sources via target_sources().
add_library(kernel)
# FIXME: SHADOW_VARS: Remove this once we have enabled -Wshadow globally.
add_compile_options($<TARGET_PROPERTY:compiler,warning_shadow_variables>)

# Kernel files has the macro __ZEPHYR_SUPERVISOR__ set so that it
# optimizes the code when userspace is enabled.
set_target_properties(
  kernel
  PROPERTIES
  COMPILE_DEFINITIONS
  __ZEPHYR_SUPERVISOR__
  )

add_subdirectory(core)
add_subdirectory(ipc)
add_subdirectory(timing)
add_subdirectory(memory)
add_subdirectory(misc)

add_subdirectory_ifdef(CONFIG_SMP smp)
add_subdirectory_ifdef(CONFIG_USERSPACE userspace)
add_subdirectory(sys)

target_include_directories(kernel PRIVATE
  ${ZEPHYR_BASE}/kernel/include
  ${ARCH_DIR}/${ARCH}/include
  )

target_link_libraries(kernel zephyr_interface)

endif()

# Optionally build kernel sources without LTO
if(CONFIG_KERNEL_NO_LTO)
  set_target_properties(kernel PROPERTIES LTO 0)
endif()

add_dependencies(kernel zephyr_generated_headers)

unset(libkernel)
