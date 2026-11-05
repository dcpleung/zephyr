/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define STACK_SIZE    1024
#define PRIORITY      K_PRIO_PREEMPT(5)
#define STEPS         3
#define MAX_RESTARTS  3U
#define RESTART_DELAY K_MSEC(100)

/* A thread the supervisor restarts when it ends unexpectedly */
struct supervised_thread {
	struct k_thread thread;
	k_thread_stack_t *stack;
	size_t stack_size;
	k_thread_entry_t entry;
	uint32_t options;
	const char *name;
	uint32_t restarts;
	/* Set when the thread is reported as ending */
	bool ended;
	/* Set when the thread ending is followed by a kernel panic */
	bool expect_panic;
	/* Reports this thread's events */
	struct k_thread_monitor_cb monitor;
	struct k_work_delayable restart_work;
};

static void faulty_entry(void *p1, void *p2, void *p3);
static void essential_entry(void *p1, void *p2, void *p3);

static K_THREAD_STACK_DEFINE(faulty_stack, STACK_SIZE);
static K_THREAD_STACK_DEFINE(essential_stack, STACK_SIZE);

static struct supervised_thread supervised[] = {
	{
		.stack = faulty_stack,
		.stack_size = K_THREAD_STACK_SIZEOF(faulty_stack),
		.entry = faulty_entry,
		.options = 0,
		.name = "faulty",
	},
	{
		.stack = essential_stack,
		.stack_size = K_THREAD_STACK_SIZEOF(essential_stack),
		.entry = essential_entry,
		.options = K_ESSENTIAL,
		.name = "essential",
	},
};

K_SEM_DEFINE(all_stopped, 0, 1);

static struct supervised_thread *find_supervised(const struct k_thread *thread)
{
	for (size_t i = 0; i < ARRAY_SIZE(supervised); i++) {
		if (&supervised[i].thread == thread) {
			return &supervised[i];
		}
	}

	return NULL;
}

static void start_thread(struct supervised_thread *st)
{
	st->ended = false;
	st->expect_panic = false;
	k_thread_create(&st->thread, st->stack, st->stack_size, st->entry, st, NULL, NULL, PRIORITY,
			st->options, K_NO_WAIT);
	k_thread_name_set(&st->thread, st->name);
}

/* Faults after a few steps of work */
static void faulty_entry(void *p1, void *p2, void *p3)
{
	struct supervised_thread *st = p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (int step = 1; step <= STEPS; step++) {
		printk("%s: run %u, step %d\n", st->name, st->restarts, step);
		k_msleep(100);
	}

	k_oops();
}

/* Essential thread that returns after a few steps of work */
static void essential_entry(void *p1, void *p2, void *p3)
{
	struct supervised_thread *st = p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (int step = 1; step <= STEPS; step++) {
		printk("%s: run %u, step %d\n", st->name, st->restarts, step);
		k_msleep(150);
	}
}

/*
 * Runs in the system work queue, where threads may be joined and created,
 * unlike in the thread monitor handler. The event stored in the callback
 * stays valid: the thread cannot end again before it is restarted here.
 */
static void restart_handler(struct k_work *work)
{
	/* Work items run one at a time on the system work queue */
	static size_t stopped;
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct supervised_thread *st = CONTAINER_OF(dwork, struct supervised_thread, restart_work);

	/* The thread is aborted right after the event is reported */
	(void)k_thread_join(&st->thread, K_FOREVER);

	if (st->monitor.event == K_THREAD_MONITOR_EVENT_FATAL_ERROR) {
		printk("%s: fatal error %u, ", st->name, st->monitor.info.fatal_error.reason);
	} else {
		printk("%s: exited, ", st->name);
	}

	if (st->restarts == MAX_RESTARTS) {
		printk("giving up\n");
		stopped++;
		if (stopped == ARRAY_SIZE(supervised)) {
			k_sem_give(&all_stopped);
		}
		return;
	}

	st->restarts++;
	printk("restart %u/%u\n", st->restarts, MAX_RESTARTS);
	start_thread(st);
}

/*
 * Runs in the context of the ending thread with interrupts locked, so the
 * restart is queued to the system work queue, after a delay that keeps a
 * thread that keeps failing from monopolizing the CPU.
 */
static void monitor_handler(struct k_thread_monitor_cb *cb)
{
	struct supervised_thread *st = cb->priv_data;

	st->ended = true;
	st->expect_panic = (cb->event == K_THREAD_MONITOR_EVENT_ESSENTIAL_EXIT);

	(void)k_work_schedule(&st->restart_work, RESTART_DELAY);
}

/*
 * Returning lets the kernel abort the faulting thread instead of halting the
 * system. Only do so for a supervised thread that is still running, and once
 * for the kernel panic that follows an essential supervised thread ending.
 * Any other fatal error halts, including one raised after the thread ended,
 * which happens on architectures that cannot resume from that panic.
 */
void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	struct supervised_thread *st = find_supervised(k_current_get());

	ARG_UNUSED(esf);

	if (st != NULL) {
		if (!st->ended) {
			return;
		}

		if (st->expect_panic && (reason == K_ERR_KERNEL_PANIC)) {
			st->expect_panic = false;
			return;
		}

		printk("%s: fatal error %u after it ended\n", st->name, reason);
	}

	printk("Unrecoverable fatal error %u\n", reason);
	k_fatal_halt(reason);
}

int main(void)
{
	const uint32_t event_mask =
		K_THREAD_MONITOR_EVENT_FATAL_ERROR | K_THREAD_MONITOR_EVENT_ESSENTIAL_EXIT;

	for (size_t i = 0; i < ARRAY_SIZE(supervised); i++) {
		struct supervised_thread *st = &supervised[i];
		int ret;

		k_work_init_delayable(&st->restart_work, restart_handler);
		k_thread_monitor_cb_init(&st->monitor, monitor_handler, event_mask, &st->thread,
					 st);

		ret = k_thread_monitor_cb_register(&st->monitor);
		if (ret != 0) {
			printk("%s: failed to register thread monitor callback: %d\n", st->name,
			       ret);
			return 0;
		}

		start_thread(st);
	}

	(void)k_sem_take(&all_stopped, K_FOREVER);
	printk("All supervised threads stopped after %u restarts each\n", MAX_RESTARTS);

	return 0;
}
