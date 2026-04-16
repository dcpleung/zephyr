# SPDX-License-Identifier: Apache-2.0
# Copyright (c) The Zephyr Project Contributors

target_sources_ifdef(CONFIG_SYS_CLOCK_EXISTS kernel PRIVATE
  timer.c
  )

target_sources(kernel PRIVATE busy_wait.c)

if(CONFIG_TIMER_READS_ITS_FREQUENCY_AT_RUNTIME OR
   CONFIG_SYSTEM_CLOCK_HW_CYCLES_PER_SEC_RUNTIME_UPDATE)
  target_sources(kernel PRIVATE sys_clock_hw_cycles.c)
endif()
