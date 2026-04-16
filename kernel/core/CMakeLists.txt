# SPDX-License-Identifier: Apache-2.0
# Copyright (c) The Zephyr Project Contributors

# Toolchain workaround: on ARM Thumb, gcc will normally enable
# -fomit-frame-pointer at all optimization levels except -O0. When
# frame pointers ARE enabled, gcc will also infuriatingly disallow a
# clobber of the r7 frame pointer register in inline assembly
# (vs. just spilling filling as needed). Our optimized arch_switch
# requires the ability to clobber this (callee-saved) register. Turn
# the flag back on for the two files that need it. (The error
# behavior only gets modified at the level of a whole file, I tried
# #pragmas and attributes to change the code generation for the
# specific functions without success).
if(CONFIG_CPU_CORTEX_M)
  if(CONFIG_NO_OPTIMIZATIONS)
    if(${COMPILER} STREQUAL "gcc")
      set_source_files_properties(init.c PROPERTIES COMPILE_OPTIONS -fomit-frame-pointer)
      set_source_files_properties(sched.c PROPERTIES COMPILE_OPTIONS -fomit-frame-pointer)
    endif()
  endif()
endif()

target_sources(kernel PRIVATE
  fatal.c
  init.c
  timeout.c
  )

target_sources_ifdef(CONFIG_MULTITHREADING kernel PRIVATE
  idle.c
  sched.c
  thread.c
  )

if(CONFIG_MULTITHREADING)
  if(CONFIG_SCHED_SCALABLE OR CONFIG_WAITQ_SCALABLE)
    target_sources(kernel PRIVATE priority_queues.c)
  endif()
else()
  target_sources(kernel PRIVATE nothread.c)
endif()

target_sources_ifdef(CONFIG_TIMESLICING        kernel PRIVATE timeslicing.c)
target_sources_ifdef(CONFIG_SPIN_VALIDATE      kernel PRIVATE spinlock_validate.c)
target_sources_ifdef(CONFIG_IRQ_OFFLOAD        kernel PRIVATE irq_offload.c)
target_sources_ifdef(CONFIG_THREAD_MONITOR     kernel PRIVATE thread_monitor.c)
target_sources_ifdef(CONFIG_SCHED_THREAD_USAGE kernel PRIVATE usage.c)
