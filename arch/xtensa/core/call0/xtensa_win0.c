#include <stdint.h>
#include <xtensa/corebits.h>
#include <xtensa/config/core-isa.h>
#include <zephyr/toolchain.h>
#include <zephyr/kernel_structs.h>
#include <ksched.h>
#include <zephyr/arch/xtensa/xtensa-win0.h>
#include <_soc_inthandlers.h>
#include <xtensa_internal.h>

/* These are used as part of a mocking layer to test syscall handling
 * without a full userspace.  Will be removed.
 */
__weak int _mock_priv_stack, _k_syscall_table;

#define DEF_INT_C_HANDLER(l)						\
void *xtensa_int##l##_c(void *interrupted)		   		\
{									\
	uint32_t irqs, intenable, m;					\
	IF_ENABLED(CONFIG_SCHED_THREAD_USAGE, z_sched_usage_stop());	\
	__asm__ volatile("rsr.interrupt %0" : "=r"(irqs));		\
	__asm__ volatile("rsr.intenable %0" : "=r"(intenable));		\
	irqs &= intenable;						\
	while ((m = _xtensa_handle_one_int##l(irqs))) {			\
		irqs ^= m;						\
		__asm__ volatile("wsr.intclear %0" : : "r"(m));		\
	}								\
	return z_get_next_switch_handle(NULL);				\
}

#if XCHAL_NMILEVEL >= 2
DEF_INT_C_HANDLER(2)
#endif
#if XCHAL_NMILEVEL >= 3
DEF_INT_C_HANDLER(3)
#endif
#if XCHAL_NMILEVEL >= 4
DEF_INT_C_HANDLER(4)
#endif
#if XCHAL_NMILEVEL >= 5
DEF_INT_C_HANDLER(5)
#endif
#if XCHAL_NMILEVEL >= 6
DEF_INT_C_HANDLER(6)
#endif
#if XCHAL_NMILEVEL >= 7
DEF_INT_C_HANDLER(7)
#endif

static ALWAYS_INLINE DEF_INT_C_HANDLER(1)

/* FIXME: in win0 that reason argument in a2 is rotated out and hidden
 * when it gets to the handler.  Need to either dig it out or find
 * some other convention.  Right now we just abort the thread.
 */
__unused static char arch_except_pc;
void xtensa_arch_except(__unused int reason_p)
{
	__asm__("arch_except_pc: ill");
}

/* FIXME: same, need a mechanism for spilling rotated frames inside
 * the handler.  Also this isn't a "stack" (in the sense of a call
 * stack), it's dumping the interrupted context, which just happens to
 * be stored on the stack in asm2...
 */
void xtensa_dump_stack(const __unused z_arch_esf_t *stack)
{
}

void *xtensa_excint1_c(xtensa_win0_ctx_t *ctx)
{
	uint32_t cause, vaddr, reason = K_ERR_CPU_EXCEPTION;

	__asm__ volatile("rsr %0, EXCCAUSE" : "=r"(cause));
	if (cause == EXCCAUSE_LEVEL1_INTERRUPT) {
		return xtensa_int1_c(NULL);
	}

	/* In win0, everything else is fatal (syscalls and TLB
	 * exceptions have their own path in the asm upstream, alloca
	 * exceptions don't happen)
	 */
	__asm__ volatile("rsr %0, EXCVADDR" : "=r"(vaddr));
	printk(" ** FATAL EXCEPTION\n");
	printk(" ** CPU %d EXCCAUSE %d\n", arch_curr_cpu()->id, cause);
	printk(" **  PC %p VADDR %p\n", (void *)ctx->pc, (void *)vaddr);
	printk(" **  PS 0x%x\n", ctx->ps);

	printk(" **  A0 %p  A1 %p  A2 %p  A3 %p\n",
	       (void *)ctx->a0, (void *)ctx->a1,
	       (void *)ctx->a2, (void *)ctx->a3);

	printk(" **  A4 %p  A5 %p  A6 %p  A7 %p\n",
	       (void *)ctx->a4, (void *)ctx->a5,
	       (void *)ctx->a6, (void *)ctx->a7);

	printk(" **  A8 %p  A9 %p A10 %p A11 %p\n",
	       (void *)ctx->a8, (void *)ctx->a9,
	       (void *)ctx->a10, (void *)ctx->a11);

	printk(" ** A12 %p A13 %p A14 %p A15 %p\n",
	       (void *)ctx->a12, (void *)ctx->a13,
	       (void *)ctx->a14, (void *)ctx->a15);

#if XCHAL_HAVE_LOOPS
	printk(" ** LBEG %p LEND %p LCOUNT %p\n",
	       (void *)ctx->lbeg,
	       (void *)ctx->lend,
	       (void *)ctx->lcount);
#endif

	printk(" ** SAR %p\n", (void *)ctx->sar);

	xtensa_fatal_error(reason, (void *)ctx);
	return z_get_next_switch_handle(NULL);
}

void arch_new_thread(struct k_thread *thread, k_thread_stack_t *stack,
		     char *stack_ptr, k_thread_entry_t entry,
		     void *p1, void *p2, void *p3)
{
#ifdef CONFIG_KERNEL_COHERENCE
	__ASSERT((((size_t)stack) % XCHAL_DCACHE_LINESIZE) == 0, "");
	__ASSERT((((size_t)stack_ptr) % XCHAL_DCACHE_LINESIZE) == 0, "");
	sys_cache_data_flush_and_invd_range(stack, stack_ptr - (char *)stack);
#endif

	memset(&thread->arch.ctx, 0, sizeof(thread->arch.ctx));
	thread->arch.ctx.pc = (uint32_t) z_thread_entry;
	thread->arch.ctx.a1 = (uint32_t) stack_ptr;
	thread->arch.ctx.a2 = (uint32_t) entry;
	thread->arch.ctx.a3 = (uint32_t) p1;
	thread->arch.ctx.a4 = (uint32_t) p2;
	thread->arch.ctx.a5 = (uint32_t) p3;

	thread->switch_handle = &thread->arch.ctx;
}
