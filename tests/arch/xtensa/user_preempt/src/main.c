/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Interrupts that switch away from a user thread running in its own
 * memory domain, on its user stack, and switch back to it later.
 *
 * The interrupt exit spills the register windows of the interrupted
 * thread to its user stack and swaps page tables or MPU entries. Both
 * have to happen in the right order for the thread to find its stack
 * intact and accessible when it runs again.
 */

#include <zephyr/kernel.h>
#include <zephyr/app_memory/app_memdomain.h>
#include <zephyr/ztest.h>

#define STACK_SIZE 2048
#define ROUNDS     100U

K_APPMEM_PARTITION_DEFINE(part_spin);
K_APPMEM_PARTITION_DEFINE(part_wake);
K_APP_BMEM(part_spin) static volatile uint32_t spin_count;
K_APP_BMEM(part_wake) static volatile uint32_t wake_count;

static K_SEM_DEFINE(wake_sem, 0, 1);

/* Never blocks: only interrupts switch away from it. */
static void spin_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		spin_count++;
	}
}

static void wake_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		(void)k_sem_take(&wake_sem, K_FOREVER);
		wake_count++;
	}
}

static void wake_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	k_sem_give(&wake_sem);
}

ZTEST(xtensa_user_preempt, test_user_preempt)
{
	static K_THREAD_STACK_DEFINE(spin_stack, STACK_SIZE);
	static K_THREAD_STACK_DEFINE(wake_stack, STACK_SIZE);
	static struct k_thread spin_thread;
	static struct k_thread wake_thread;
	static struct k_mem_domain spin_domain;
	static struct k_mem_domain wake_domain;
	static struct k_timer wake_timer;
	struct k_mem_partition *spin_parts[] = {&part_spin};
	struct k_mem_partition *wake_parts[] = {&part_wake};
	int prio = k_thread_priority_get(k_current_get());
	uint32_t last_spin = 0U;

	k_timer_init(&wake_timer, wake_expiry, NULL);
	zassert_ok(k_mem_domain_init(&spin_domain, ARRAY_SIZE(spin_parts), spin_parts));
	zassert_ok(k_mem_domain_init(&wake_domain, ARRAY_SIZE(wake_parts), wake_parts));

	/* Lower priority than this thread, so it runs only while this one sleeps. */
	k_thread_create(&spin_thread, spin_stack, STACK_SIZE, spin_entry, NULL, NULL, NULL,
			prio + 1, K_USER, K_FOREVER);
	/* Higher priority than the spinning thread, so waking it preempts that one. */
	k_thread_create(&wake_thread, wake_stack, STACK_SIZE, wake_entry, NULL, NULL, NULL,
			prio - 1, K_USER, K_FOREVER);
	zassert_ok(k_mem_domain_add_thread(&spin_domain, &spin_thread));
	zassert_ok(k_mem_domain_add_thread(&wake_domain, &wake_thread));
	k_object_access_grant(&wake_sem, &wake_thread);
	k_thread_start(&spin_thread);
	k_thread_start(&wake_thread);

	for (uint32_t i = 0U; i < ROUNDS; i++) {
		/* The timer interrupt ending the sleep switches from the
		 * spinning thread back to this one.
		 */
		k_msleep(1);
		zassert_true(spin_count != last_spin, "spinning thread did not run");
		last_spin = spin_count;

		/* The timer interrupt switches from the spinning thread to
		 * the woken one, in another memory domain, and back to the
		 * spinning thread once it blocks again.
		 */
		k_timer_start(&wake_timer, K_MSEC(1), K_NO_WAIT);
		k_msleep(3);
		zassert_equal(wake_count, i + 1U, "woken thread ran %u times", wake_count);
	}

	k_thread_abort(&spin_thread);
	k_thread_abort(&wake_thread);
}

ZTEST_SUITE(xtensa_user_preempt, NULL, NULL, NULL, NULL, NULL);
