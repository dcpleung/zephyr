/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * void xtensa_arch_except(int reason_p);
 *
 * Implements hardware exception for Xtensa ARCH_EXCEPT to save
 * interrupted stack frame and reason_p for use in exception handler
 * and coredump
 */
.global xtensa_arch_except
.global xtensa_arch_except_epc
.align 4
xtensa_arch_except:
	entry a1, 16
xtensa_arch_except_epc:
	ill
	retw

/*
 * void xtensa_arch_kernel_oops(int reason_p, void *ssf);
 *
 * Simply to raise hardware exception for Kernel OOPS.
 */
.global xtensa_arch_kernel_oops
.global xtensa_arch_kernel_oops_epc
.align 4
xtensa_arch_kernel_oops:
	entry a1, 16
xtensa_arch_kernel_oops_epc:
	ill
	retw
