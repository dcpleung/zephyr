/*
 * Copyright (c) 2018 Linaro Limited.
 * Copyright (c) 2023 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/syscall.h>
#include <zephyr/types.h>
#include <zephyr/linker/sections.h>

__pinned_func
uintptr_t arch_syscall_invoke6_helper(uintptr_t arg1, uintptr_t arg2,
				      uintptr_t arg3, uintptr_t arg4,
				      uintptr_t arg5, uintptr_t arg6,
				      uintptr_t call_id)
{
	uint32_t ret;

#ifdef __clang__
	uint32_t esi;

	__asm__ volatile("pushl %%esi" : : : "memory");

	__asm__ volatile("push %%ebp\n\t"
			 "mov %[arg6], %%ebp\n\t"
			 "int $0x80\n\t"
			 "pop %%ebp\n\t"
			 "popl %%esi\n\t"
			 : "=a" (ret), "=S" (esi)
			 : "a" (arg1), "d" (arg2),
			   "c" (arg3), "b" (arg4), "D" (arg5),
			   [arg6] "m" (arg6),
			   "S" (call_id)
			 : "memory");
#else
	__asm__ volatile("push %%ebp\n\t"
			 "mov %[arg6], %%ebp\n\t"
			 "int $0x80\n\t"
			 "pop %%ebp\n\t"
			 : "=a" (ret)
			 : "S" (call_id), "a" (arg1), "d" (arg2),
			   "c" (arg3), "b" (arg4), "D" (arg5),
			   [arg6] "m" (arg6)
			 : "memory");
#endif

	return ret;
}

__pinned_func
uintptr_t arch_syscall_invoke5_helper(uintptr_t arg1, uintptr_t arg2,
				      uintptr_t arg3, uintptr_t arg4,
				      uintptr_t arg5,
				      uintptr_t call_id)
{
	uint32_t ret;

#ifdef __clang__
	uint32_t esi;

	__asm__ volatile("pushl %%esi" : : : "memory");

	__asm__ volatile("int $0x80\n\t"
			 "popl %%esi\n\t"
			 : "=a" (ret), "=S" (esi)
			 : "a" (arg1), "d" (arg2), "c" (arg3),
			   "b" (arg4), "D" (arg5),
			   "S" (call_id)
			 : "memory");
#else
	__asm__ volatile("int $0x80"
			 : "=a" (ret)
			 : "S" (call_id), "a" (arg1), "d" (arg2),
			   "c" (arg3), "b" (arg4), "D" (arg5)
			 : "memory");
#endif

	return ret;
}

__pinned_func
uintptr_t arch_syscall_invoke4_helper(uintptr_t arg1, uintptr_t arg2,
				      uintptr_t arg3, uintptr_t arg4,
				      uintptr_t call_id)
{
	uint32_t ret;

#ifdef __clang__
	uint32_t esi;

	__asm__ volatile("pushl %%esi" : : : "memory");

	__asm__ volatile("int $0x80\n\t"
			 "popl %%esi\n\t"
			 : "=a" (ret), "=S" (esi)
			 : "a" (arg1), "d" (arg2), "c" (arg3),
			   "b" (arg4),
			   "S" (call_id)
			 : "memory");
#else
	__asm__ volatile("int $0x80"
			 : "=a" (ret)
			 : "S" (call_id), "a" (arg1), "d" (arg2), "c" (arg3),
			   "b" (arg4)
			 : "memory");
#endif

	return ret;
}

__pinned_func
uintptr_t arch_syscall_invoke3_helper(uintptr_t arg1, uintptr_t arg2,
				      uintptr_t arg3,
				      uintptr_t call_id)
{
	uint32_t ret;

#ifdef __clang__
	uint32_t esi;

	__asm__ volatile("pushl %%esi" : : : "memory");

	__asm__ volatile("int $0x80\n\t"
			 "popl %%esi\n\t"
			 : "=a" (ret), "=S" (esi)
			 : "a" (arg1), "d" (arg2), "c" (arg3),
			   "S" (call_id)
			 : "memory");
#else
	__asm__ volatile("int $0x80"
			 : "=a" (ret)
			 : "S" (call_id), "a" (arg1), "d" (arg2), "c" (arg3)
			 : "memory");
#endif

	return ret;
}

__pinned_func
uintptr_t arch_syscall_invoke2_helper(uintptr_t arg1, uintptr_t arg2,
					     uintptr_t call_id)
{
	uint32_t ret;

#ifdef __clang__
	uint32_t esi;

	__asm__ volatile("pushl %%esi" : : : "memory");

	__asm__ volatile("int $0x80\n\t"
			 "popl %%esi\n\t"
			 : "=a" (ret), "=S" (esi)
			 : "a" (arg1), "d" (arg2),
			   "S" (call_id)
			 : "memory");
#else
	__asm__ volatile("int $0x80"
			 : "=a" (ret)
			 : "S" (call_id), "a" (arg1), "d" (arg2)
			 : "memory"
			 );
#endif

	return ret;
}

__pinned_func
uintptr_t arch_syscall_invoke1_helper(uintptr_t arg1,
				      uintptr_t call_id)
{
	uint32_t ret;

#ifdef __clang__
	uint32_t esi;

	__asm__ volatile("pushl %%esi" : : : "memory");

	__asm__ volatile("int $0x80\n\t"
			 "popl %%esi\n\t"
			 : "=a" (ret), "=S" (esi)
			 : "a" (arg1), "S" (call_id)
			 : "memory");
#else
	__asm__ volatile("int $0x80"
			 : "=a" (ret)
			 : "S" (call_id), "a" (arg1)
			 : "memory"
			 );
#endif

	return ret;
}

__pinned_func
uintptr_t arch_syscall_invoke0_helper(uintptr_t call_id)
{
	uint32_t ret;

#ifdef __clang__
	uint32_t esi;

	__asm__ volatile("pushl %%esi" : : : "memory");

	__asm__ volatile("int $0x80\n\t"
			 "popl %%esi\n\t"
			 : "=a" (ret), "=S" (esi)
			 : "S" (call_id)
			 : "memory");
#else
	__asm__ volatile("int $0x80"
			 : "=a" (ret)
			 : "S" (call_id)
			 : "memory"
			 );
#endif

	return ret;
}
