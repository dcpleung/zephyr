/*
 * Copyright (c) 2017, 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <kernel_internal.h>

#include <xtensa_asm2_context.h>
#include <xtensa_internal.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

#ifdef CONFIG_USERSPACE

#ifdef CONFIG_THREAD_LOCAL_STORAGE
/*
 * Per-thread (TLS) variable indicating whether execution is in user mode.
 */
__thread uint32_t is_user_mode;
#endif

#endif /* CONFIG_USERSPACE */

#if defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING)
int arch_float_disable(struct k_thread *thread)
{
	/* xtensa always has FPU enabled so cannot be disabled */
	return -ENOTSUP;
}

int arch_float_enable(struct k_thread *thread, unsigned int options)
{
	/* xtensa always has FPU enabled so nothing to do here */
	return 0;
}
#endif /* CONFIG_FPU && CONFIG_FPU_SHARING */

#ifdef CONFIG_USERSPACE
FUNC_NORETURN void arch_user_mode_enter(k_thread_entry_t user_entry,
					void *p1, void *p2, void *p3)
{
	struct k_thread *current = _current;
	size_t stack_end;

	/* Transition will reset stack pointer to initial, discarding
	 * any old context since this is a one-way operation
	 */
	stack_end = Z_STACK_PTR_ALIGN(current->stack_info.start +
				      current->stack_info.size -
				      current->stack_info.delta);

	xtensa_userspace_enter(user_entry, p1, p2, p3,
			       stack_end, current->stack_info.start);

	CODE_UNREACHABLE;
}
#endif /* CONFIG_USERSPACE */
