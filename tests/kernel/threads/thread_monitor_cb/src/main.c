/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#define STACK_SIZE (1024 + CONFIG_TEST_EXTRA_STACK_SIZE)

/* Event bit not used by the kernel, to check mask filtering */
#define UNUSED_EVENT BIT(31)

struct test_monitor_cb {
	struct k_thread_monitor_cb cb;
	uint32_t count;
	bool foreach_saw_thread;
};

static struct test_monitor_cb monitor_a;
static struct test_monitor_cb monitor_b;

static struct k_thread crash_thread;

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	ARG_UNUSED(esf);

	/* K_ERR_KERNEL_PANIC comes from aborting an essential thread */
	if ((k_current_get() != &crash_thread) ||
	    ((reason != K_ERR_KERNEL_OOPS) && (reason != K_ERR_KERNEL_PANIC))) {
		printk("Unexpected fatal error %u\n", reason);
		TC_END_REPORT(TC_FAIL);
		k_fatal_halt(reason);
	}

	/* Return so the kernel aborts the faulting thread */
}

/* Only counts; the event is read back from the callback after the fact */
static void monitor_handler(struct k_thread_monitor_cb *cb)
{
	struct test_monitor_cb *mon = CONTAINER_OF(cb, struct test_monitor_cb, cb);

	mon->count++;
}

/* Counts through the user's private data */
static void priv_data_handler(struct k_thread_monitor_cb *cb)
{
	uint32_t *count = cb->priv_data;

	(*count)++;
}

static void foreach_cb(const struct k_thread *thread, void *user_data)
{
	struct test_monitor_cb *mon = user_data;

	if (thread == mon->cb.thread) {
		mon->foreach_saw_thread = true;
	}
}

/* Walks the thread list from the handler, which must not deadlock */
static void foreach_handler(struct k_thread_monitor_cb *cb)
{
	struct test_monitor_cb *mon = CONTAINER_OF(cb, struct test_monitor_cb, cb);

	mon->count++;
	k_thread_foreach(foreach_cb, mon);
}

static void crash_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_oops();
}

static void exit_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
}

static void run_thread(k_thread_entry_t entry, uint32_t options)
{
	static K_THREAD_STACK_DEFINE(crash_stack, STACK_SIZE);

	k_thread_create(&crash_thread, crash_stack, K_THREAD_STACK_SIZEOF(crash_stack), entry, NULL,
			NULL, NULL, K_PRIO_PREEMPT(0), options, K_NO_WAIT);

	zassert_ok(k_thread_join(&crash_thread, K_FOREVER));
}

ZTEST(thread_monitor_cb, test_fatal_error_notified)
{
	k_thread_monitor_cb_init(&monitor_a.cb, monitor_handler, K_THREAD_MONITOR_EVENT_FATAL_ERROR,
				 &crash_thread, NULL);
	zassert_ok(k_thread_monitor_cb_register(&monitor_a.cb));

	run_thread(crash_entry, 0);

	/* The event stays stored in the callback for later processing */
	zassert_equal(monitor_a.count, 1U);
	zassert_equal(monitor_a.cb.event, K_THREAD_MONITOR_EVENT_FATAL_ERROR, NULL);
	zassert_equal(monitor_a.cb.info.fatal_error.reason, K_ERR_KERNEL_OOPS);
}

ZTEST(thread_monitor_cb, test_priv_data)
{
	static uint32_t count;

	count = 0U;
	k_thread_monitor_cb_init(&monitor_a.cb, priv_data_handler,
				 K_THREAD_MONITOR_EVENT_FATAL_ERROR, &crash_thread, &count);
	zassert_equal_ptr(monitor_a.cb.priv_data, &count);
	zassert_ok(k_thread_monitor_cb_register(&monitor_a.cb));

	run_thread(crash_entry, 0);

	zassert_equal(count, 1U);
	zassert_equal_ptr(monitor_a.cb.priv_data, &count);
}

ZTEST(thread_monitor_cb, test_thread_filter)
{
	k_thread_monitor_cb_init(&monitor_a.cb, monitor_handler, K_THREAD_MONITOR_EVENT_FATAL_ERROR,
				 &crash_thread, NULL);
	/* Bound to the test thread, which does not fault */
	k_thread_monitor_cb_init(&monitor_b.cb, monitor_handler, K_THREAD_MONITOR_EVENT_FATAL_ERROR,
				 k_current_get(), NULL);
	zassert_ok(k_thread_monitor_cb_register(&monitor_a.cb));
	zassert_ok(k_thread_monitor_cb_register(&monitor_b.cb));

	run_thread(crash_entry, 0);

	zassert_equal(monitor_a.count, 1U);
	/* Neither invoked nor written to */
	zassert_equal(monitor_b.count, 0U);
	zassert_equal(monitor_b.cb.event, 0U);
}

ZTEST(thread_monitor_cb, test_mask_filters_events)
{
	k_thread_monitor_cb_init(&monitor_a.cb, monitor_handler, K_THREAD_MONITOR_EVENT_FATAL_ERROR,
				 &crash_thread, NULL);
	k_thread_monitor_cb_init(&monitor_b.cb, monitor_handler, UNUSED_EVENT, &crash_thread, NULL);
	zassert_ok(k_thread_monitor_cb_register(&monitor_a.cb));
	zassert_ok(k_thread_monitor_cb_register(&monitor_b.cb));

	run_thread(crash_entry, 0);

	zassert_equal(monitor_a.count, 1U);
	zassert_equal(monitor_b.count, 0U);
}

ZTEST(thread_monitor_cb, test_unregistered_not_notified)
{
	k_thread_monitor_cb_init(&monitor_a.cb, monitor_handler, K_THREAD_MONITOR_EVENT_FATAL_ERROR,
				 &crash_thread, NULL);
	zassert_ok(k_thread_monitor_cb_register(&monitor_a.cb));
	zassert_ok(k_thread_monitor_cb_unregister(&monitor_a.cb));

	run_thread(crash_entry, 0);

	zassert_equal(monitor_a.count, 0U);
}

ZTEST(thread_monitor_cb, test_handler_calls_foreach)
{
	k_thread_monitor_cb_init(&monitor_a.cb, foreach_handler, K_THREAD_MONITOR_EVENT_FATAL_ERROR,
				 &crash_thread, NULL);
	zassert_ok(k_thread_monitor_cb_register(&monitor_a.cb));

	run_thread(crash_entry, 0);

	zassert_equal(monitor_a.count, 1U);
	/* The thread is still in the monitor list when reported */
	zassert_true(monitor_a.foreach_saw_thread);
}

ZTEST(thread_monitor_cb, test_essential_fault)
{
	/*
	 * Aborting a faulting essential thread panics; only POSIX can
	 * resume from that panic, other arches stop in an assertion or a
	 * nested exception.
	 */
	Z_TEST_SKIP_IFNDEF(CONFIG_ARCH_POSIX);

	k_thread_monitor_cb_init(&monitor_a.cb, monitor_handler, K_THREAD_MONITOR_EVENT_FATAL_ERROR,
				 &crash_thread, NULL);
	k_thread_monitor_cb_init(&monitor_b.cb, monitor_handler,
				 K_THREAD_MONITOR_EVENT_ESSENTIAL_EXIT, &crash_thread, NULL);
	zassert_ok(k_thread_monitor_cb_register(&monitor_a.cb));
	zassert_ok(k_thread_monitor_cb_register(&monitor_b.cb));

	run_thread(crash_entry, K_ESSENTIAL);

	/* Reported as an essential thread ending, not as a fatal error */
	zassert_equal(monitor_a.count, 0U);
	zassert_equal(monitor_b.count, 1U);
	zassert_equal(monitor_b.cb.event, K_THREAD_MONITOR_EVENT_ESSENTIAL_EXIT);
}

ZTEST(thread_monitor_cb, test_essential_exit)
{
	/*
	 * The panic is raised on the exiting thread itself, which SPARC
	 * cannot resume from yet (see thread_apis essential tests).
	 */
	if (IS_ENABLED(CONFIG_SPARC)) {
		ztest_test_skip();
	}

	k_thread_monitor_cb_init(&monitor_a.cb, monitor_handler,
				 K_THREAD_MONITOR_EVENT_ESSENTIAL_EXIT, &crash_thread, NULL);
	zassert_ok(k_thread_monitor_cb_register(&monitor_a.cb));

	run_thread(exit_entry, K_ESSENTIAL);

	zassert_equal(monitor_a.count, 1U);
	zassert_equal(monitor_a.cb.event, K_THREAD_MONITOR_EVENT_ESSENTIAL_EXIT);
}

ZTEST(thread_monitor_cb, test_register_errors)
{
	k_thread_monitor_cb_init(&monitor_a.cb, monitor_handler, K_THREAD_MONITOR_EVENT_FATAL_ERROR,
				 &crash_thread, NULL);

	zassert_equal(k_thread_monitor_cb_unregister(&monitor_a.cb), -EINVAL);
	zassert_ok(k_thread_monitor_cb_register(&monitor_a.cb));
	zassert_equal(k_thread_monitor_cb_register(&monitor_a.cb), -EALREADY);
	zassert_ok(k_thread_monitor_cb_unregister(&monitor_a.cb));
	zassert_equal(k_thread_monitor_cb_unregister(&monitor_a.cb), -EINVAL);
}

static void thread_monitor_cb_after(void *fixture)
{
	ARG_UNUSED(fixture);

	/* Either may already be unregistered */
	(void)k_thread_monitor_cb_unregister(&monitor_a.cb);
	(void)k_thread_monitor_cb_unregister(&monitor_b.cb);

	monitor_a = (struct test_monitor_cb){0};
	monitor_b = (struct test_monitor_cb){0};
}

ZTEST_SUITE(thread_monitor_cb, NULL, NULL, NULL, thread_monitor_cb_after, NULL);
