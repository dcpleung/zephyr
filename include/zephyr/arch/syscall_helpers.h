/*
 * Copyright (c) 2018 Linaro Limited.
 * Copyright (c) 2023 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_SYSCALL_HELPERS_H_
#define ZEPHYR_INCLUDE_ARCH_SYSCALL_HELPERS_H_

#ifdef CONFIG_USERSPACE
#ifndef _ASMLANGUAGE

#include <zephyr/toolchain.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uintptr_t arch_syscall_invoke6_helper(uintptr_t arg1, uintptr_t arg2,
					     uintptr_t arg3, uintptr_t arg4,
					     uintptr_t arg5, uintptr_t arg6,
					     uintptr_t call_id);

extern uintptr_t arch_syscall_invoke5_helper(uintptr_t arg1, uintptr_t arg2,
					     uintptr_t arg3, uintptr_t arg4,
					     uintptr_t arg5, uintptr_t call_id);

extern uintptr_t arch_syscall_invoke4_helper(uintptr_t arg1, uintptr_t arg2,
					     uintptr_t arg3, uintptr_t arg4,
					     uintptr_t call_id);

extern uintptr_t arch_syscall_invoke3_helper(uintptr_t arg1, uintptr_t arg2,
					     uintptr_t arg3, uintptr_t call_id);

extern uintptr_t arch_syscall_invoke2_helper(uintptr_t arg1, uintptr_t arg2,
					     uintptr_t call_id);

extern uintptr_t arch_syscall_invoke1_helper(uintptr_t arg1, uintptr_t call_id);

extern uintptr_t arch_syscall_invoke0_helper(uintptr_t call_id);

static ALWAYS_INLINE
uintptr_t arch_syscall_invoke6(uintptr_t arg1, uintptr_t arg2,
			       uintptr_t arg3, uintptr_t arg4,
			       uintptr_t arg5, uintptr_t arg6,
			       uintptr_t call_id)
{
	return arch_syscall_invoke6_helper(arg1, arg2, arg3, arg4, arg5, arg6, call_id);
}

static ALWAYS_INLINE
uintptr_t arch_syscall_invoke5(uintptr_t arg1, uintptr_t arg2,
			       uintptr_t arg3, uintptr_t arg4,
			       uintptr_t arg5, uintptr_t call_id)
{
	return arch_syscall_invoke5_helper(arg1, arg2, arg3, arg4, arg5, call_id);
}

static ALWAYS_INLINE
uintptr_t arch_syscall_invoke4(uintptr_t arg1, uintptr_t arg2,
			       uintptr_t arg3, uintptr_t arg4,
			       uintptr_t call_id)
{
	return arch_syscall_invoke4_helper(arg1, arg2, arg3, arg4, call_id);
}

static ALWAYS_INLINE
uintptr_t arch_syscall_invoke3(uintptr_t arg1, uintptr_t arg2,
			       uintptr_t arg3, uintptr_t call_id)
{
	return arch_syscall_invoke3_helper(arg1, arg2, arg3, call_id);
}

static ALWAYS_INLINE
uintptr_t arch_syscall_invoke2(uintptr_t arg1, uintptr_t arg2,
			       uintptr_t call_id)
{
	return arch_syscall_invoke2_helper(arg1, arg2, call_id);
}

static ALWAYS_INLINE
uintptr_t arch_syscall_invoke1(uintptr_t arg1, uintptr_t call_id)
{
	return arch_syscall_invoke1_helper(arg1, call_id);
}

static ALWAYS_INLINE
uintptr_t arch_syscall_invoke0(uintptr_t call_id)
{
	return arch_syscall_invoke0_helper(call_id);
}

#endif /* _ASMLANGUAGE */
#endif /* CONFIG_USERSPACE */
#endif /* ZEPHYR_INCLUDE_ARCH_SYSCALL_HELPERS_H_ */
