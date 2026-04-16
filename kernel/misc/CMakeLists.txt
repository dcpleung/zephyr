# SPDX-License-Identifier: Apache-2.0
# Copyright (c) The Zephyr Project Contributors

target_sources(kernel PRIVATE
  banner.c
  device.c
  errno.c
  float.c
  main_weak.c
  version.c
  )

target_sources_ifdef(CONFIG_REQUIRES_STACK_CANARIES kernel PRIVATE compiler_stack_protect.c)
target_sources_ifdef(CONFIG_ATOMIC_OPERATIONS_C     kernel PRIVATE atomic_c.c)
target_sources_ifdef(CONFIG_OBJ_CORE                kernel PRIVATE obj_core.c)
target_sources_ifdef(CONFIG_BOOTARGS                kernel PRIVATE boot_args.c)
