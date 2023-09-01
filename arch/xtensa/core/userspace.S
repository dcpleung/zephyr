/*
 * Copyright (c) 2022, Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <xtensa_asm2_s.h>
#include <offsets.h>
#include <offsets_short.h>
#include <zephyr/syscall.h>
#include <zsr.h>

/*
 * size_t arch_user_string_nlen(const char *s, size_t maxsize, int *err_arg)
 */
.global arch_user_string_nlen
.type arch_user_string_nlen, @function
.align 4
arch_user_string_nlen:
	entry a1, 32

	/* error value, set to -1. */
	movi a5, -1
	s32i a5, a4, 0

	/* length count */
	xor a5, a5, a5

	/* This code might page fault */
strlen_loop:
.global xtensa_user_string_nlen_fault_start
xtensa_user_string_nlen_fault_start:
	l8ui a6, a2, 0 /* Current char */

.global xtensa_user_string_nlen_fault_end
xtensa_user_string_nlen_fault_end:
	beqz a6, strlen_done
	addi a5, a5, 1
	addi a2, a2, 1
	beq a5, a3, strlen_done
	j strlen_loop

strlen_done:
	/* Set return value */
	mov a2, a5

	/* Set error value to 0 since we succeeded */
	movi a5, 0x0
	s32i a5, a4, 0

.global xtensa_user_string_nlen_fixup
xtensa_user_string_nlen_fixup:
	retw
