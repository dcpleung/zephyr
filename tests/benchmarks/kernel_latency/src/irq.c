/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Interrupt entry and exit. irq_offload() raises an interrupt on the
 * current CPU and returns once its handler has run, so it goes through
 * the architecture's regular interrupt entry and exit code. The spans
 * include the cost of irq_offload() itself, which is the same on every
 * run and does not hide changes to the entry and exit paths.
 */

#include <zephyr/irq_offload.h>

#include "bench.h"

ZTEST_BENCHMARK_SUITE(irq, NULL, NULL);

static void offload_noop(const void *arg)
{
	ARG_UNUSED(arg);
}

static void offload_end(const void *arg)
{
	ARG_UNUSED(arg);

	ztest_benchmark_end();
}

static void offload_start(const void *arg)
{
	ARG_UNUSED(arg);

	ztest_benchmark_start();
}

/* An interrupt whose handler does nothing, until the thread runs again. */
ZTEST_BENCHMARK(irq, offload_round_trip, BENCH_SAMPLES, NULL, NULL)
{
	irq_offload(offload_noop, NULL);
}

/* From the thread raising the interrupt to the handler running. */
ZTEST_BENCHMARK_MANUAL(irq, offload_entry, BENCH_SAMPLES, NULL, NULL)
{
	ztest_benchmark_start();
	irq_offload(offload_end, NULL);
}

/* From the handler to the interrupted thread running again. */
ZTEST_BENCHMARK_MANUAL(irq, offload_exit, BENCH_SAMPLES, NULL, NULL)
{
	irq_offload(offload_start, NULL);
	ztest_benchmark_end();
}

/*
 * From the handler giving a semaphore to the higher priority thread
 * blocked on it running: the interrupt exit that switches to another
 * thread instead of returning to the interrupted one.
 */
static K_SEM_DEFINE(wake_sem, 0, 1);
static K_SEM_DEFINE(done_sem, 0, 1);
static K_THREAD_STACK_DEFINE(waiter_stack, BENCH_PARTNER_STACK_SIZE);
static struct k_thread waiter_thread;

static void waiter_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	(void)k_sem_take(&wake_sem, K_FOREVER);
	ztest_benchmark_end();
	k_sem_give(&done_sem);
}

static void waiter_create(void)
{
	int priority = k_thread_priority_get(k_current_get()) - 1;

	k_thread_create(&waiter_thread, waiter_stack, BENCH_PARTNER_STACK_SIZE, waiter_entry, NULL,
			NULL, NULL, priority, 0, K_NO_WAIT);
}

static void waiter_join(void)
{
	(void)k_thread_join(&waiter_thread, K_FOREVER);
}

static void offload_give(const void *arg)
{
	ARG_UNUSED(arg);

	ztest_benchmark_start();
	k_sem_give(&wake_sem);
}

ZTEST_BENCHMARK_MANUAL(irq, offload_wake_switch, BENCH_SAMPLES, waiter_create, waiter_join)
{
	irq_offload(offload_give, NULL);
	(void)k_sem_take(&done_sem, K_FOREVER);
}
