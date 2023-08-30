/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_XTENSA_THREAD_H_
#define ZEPHYR_INCLUDE_ARCH_XTENSA_THREAD_H_

#include <stdint.h>
#include <zephyr/arch/xtensa/xtensa-win0.h>

#ifndef _ASMLANGUAGE

struct _callee_saved {
	char dummy;
};

typedef struct _callee_saved _callee_saved_t;

struct _thread_arch {
#ifdef CONFIG_XTENSA_CALL0_ABI
	xtensa_win0_ctx_t ctx; /* must be first */
#endif
	uint32_t last_cpu;
#ifdef CONFIG_USERSPACE
	uint32_t *ptables;

	/* Initial privilege mode stack pointer when doing a system call.
	 * Un-set for surpervisor threads.
	 */
	uint8_t *psp;
#endif
};

typedef struct _thread_arch _thread_arch_t;

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_XTENSA_THREAD_H_ */
