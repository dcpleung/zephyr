/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/irq_offload.h>
#include <zephyr/ztest.h>

/* Internal APIs */
#include <kernel_internal.h>
#include <kthread.h>

#define STACK_SIZE   (1024 + CONFIG_TEST_EXTRA_STACK_SIZE)
#define WORKER_PRIO  K_PRIO_PREEMPT(5)
#define CHANGED_PRIO K_PRIO_PREEMPT(7)
#define JOINER_PRIO  K_PRIO_PREEMPT(1)
#define WAIT_MS      1000
#define STRESS_RUNS  200
#define MAX_RECORDS  4

#define P1_MAGIC ((void *)0x1111)
#define P2_MAGIC ((void *)0x2222)
#define P3_MAGIC ((void *)0x3333)

/* What the worker does in one run */
enum behavior {
	BEHAVE_WAIT,
	BEHAVE_OOPS,
	BEHAVE_PANIC,
	BEHAVE_RETURN,
	BEHAVE_SELF_ABORT,
	BEHAVE_USER_OOPS,
	BEHAVE_STRESS,
	BEHAVE_TRAP,
	BEHAVE_ISR_FATAL,
};

/* What the worker saw in one run */
struct run_record {
	void *p1;
	void *p2;
	void *p3;
	int prio;
	uint32_t cpu;
	char name[CONFIG_THREAD_MAX_NAME_LEN];
};

static struct k_thread worker;
static K_THREAD_STACK_DEFINE(worker_stack, STACK_SIZE);
static bool worker_created;

static struct k_thread joiner;
static K_THREAD_STACK_DEFINE(joiner_stack, STACK_SIZE);
static bool joiner_created;
static int joiner_ret;

static atomic_t runs;
static atomic_t fatal_count;
static unsigned int last_reason;
static volatile bool ran_on;
static enum behavior script[MAX_RECORDS];
static struct run_record records[MAX_RECORDS];

static K_SEM_DEFINE(started, 0, K_SEM_MAX_LIMIT);
static K_SEM_DEFINE(joined, 0, 1);
static K_SEM_DEFINE(workq_blocked, 0, 1);
static K_SEM_DEFINE(workq_unblock, 0, 1);

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	ARG_UNUSED(esf);

	last_reason = reason;
	(void)atomic_inc(&fatal_count);
}

static uint32_t cpu_id_get(void)
{
	unsigned int key = irq_lock();
	uint32_t id = _current_cpu->id;

	irq_unlock(key);

	return id;
}

static void user_oops(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_oops();
}

/* A fatal error raised by an interrupt handler on the current thread */
static void isr_fatal(const void *arg)
{
	ARG_UNUSED(arg);

	z_fatal_error(K_ERR_KERNEL_OOPS, NULL);
}

static void worker_entry(void *p1, void *p2, void *p3)
{
	atomic_val_t run = atomic_inc(&runs);
	struct run_record *rec = &records[MIN(run, MAX_RECORDS - 1)];
	enum behavior behavior = script[MIN(run, MAX_RECORDS - 1)];

	rec->p1 = p1;
	rec->p2 = p2;
	rec->p3 = p3;
	rec->prio = k_thread_priority_get(k_current_get());
	rec->cpu = cpu_id_get();
	(void)k_thread_name_copy(k_current_get(), rec->name, sizeof(rec->name));

	k_sem_give(&started);

	switch (behavior) {
	case BEHAVE_OOPS:
		/* The restarted thread must get its original priority back */
		k_thread_priority_set(k_current_get(), CHANGED_PRIO);
		k_oops();
		break;
	case BEHAVE_PANIC:
		k_panic();
		break;
	case BEHAVE_RETURN:
		return;
	case BEHAVE_SELF_ABORT:
		k_thread_abort(k_current_get());
		break;
	case BEHAVE_USER_OOPS:
		k_thread_user_mode_enter(user_oops, NULL, NULL, NULL);
		break;
	case BEHAVE_TRAP:
		/* A CPU exception rather than a software fatal error */
		if (IS_ENABLED(CONFIG_XTENSA)) {
			/* __builtin_trap() is a debug breakpoint there: run an
			 * illegal instruction instead
			 */
			volatile uint32_t illegal = 0U;

			((void (*)(void))&illegal)();
		} else {
			__builtin_trap();
		}
		break;
	case BEHAVE_ISR_FATAL:
		irq_offload(isr_fatal, NULL);
		break;
	case BEHAVE_STRESS:
		/* Alternate between the two ways an essential thread exits:
		 * abort itself in odd runs, return in even runs
		 */
		if (run >= STRESS_RUNS) {
			k_sleep(K_FOREVER);
		}
		if ((run % 2) != 0) {
			k_thread_abort(k_current_get());
		}
		return;
	default:
		k_sleep(K_FOREVER);
		break;
	}

	/* Not reached in any run */
	ran_on = true;
	k_sleep(K_FOREVER);
}

static k_tid_t create_worker(uint32_t options, int prio)
{
	k_tid_t tid = k_thread_create(&worker, worker_stack, STACK_SIZE, worker_entry, P1_MAGIC,
				      P2_MAGIC, P3_MAGIC, prio, options, K_FOREVER);

	worker_created = true;
	(void)k_thread_name_set(tid, "restartee");

	return tid;
}

/* Arm and start the worker */
static void start_worker(k_tid_t tid, uint8_t limit)
{
	zassert_ok(k_thread_restart_limit_set(tid, limit));
	zassert_equal(k_thread_restart_limit_get(tid), limit);
	zassert_equal(k_thread_restart_count_get(tid), 0);
	k_thread_start(tid);
}

static void wait_runs(int n)
{
	for (int i = 0; i < n; i++) {
		zassert_ok(k_sem_take(&started, K_MSEC(WAIT_MS)), "run %d did not start", i);
	}
}

static void wait_dead(struct k_thread *thread)
{
	for (int i = 0; i < WAIT_MS; i++) {
		if (z_is_thread_dead(thread)) {
			return;
		}
		k_msleep(1);
	}

	zassert_unreachable("thread %p did not die", thread);
}

/* Disarm the worker and stop it for good */
static void stop_worker(void)
{
	if (!worker_created) {
		return;
	}

	(void)k_thread_restart_limit_set(&worker, 0);
	z_thread_essential_clear(&worker);
	k_thread_abort(&worker);
	zassert_ok(k_thread_join(&worker, K_MSEC(WAIT_MS)));
	worker_created = false;
}

static void block_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	k_sem_give(&workq_blocked);
	(void)k_sem_take(&workq_unblock, K_FOREVER);
}

static K_WORK_DEFINE(block_work, block_handler);

/* Keep the system work queue busy, so a restart stays pending */
static void block_workq(void)
{
	zassert_true(k_work_submit(&block_work) >= 0);
	zassert_ok(k_sem_take(&workq_blocked, K_MSEC(WAIT_MS)));
}

static void unblock_workq(void)
{
	k_sem_give(&workq_unblock);
	k_msleep(50);
}

static void joiner_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	joiner_ret = k_thread_join(p1, K_FOREVER);
	k_sem_give(&joined);
}

/* Start a thread blocked in k_thread_join() on the worker */
static void start_joiner(void)
{
	joiner_ret = -1;
	k_thread_create(&joiner, joiner_stack, STACK_SIZE, joiner_entry, &worker, NULL, NULL,
			JOINER_PRIO, 0, K_NO_WAIT);
	joiner_created = true;
	k_msleep(20);
	zassert_equal(k_sem_take(&joined, K_NO_WAIT), -EBUSY, "join did not block");
}

static void check_record(int run, int prio)
{
	struct run_record *rec = &records[run];

	zassert_equal(rec->p1, P1_MAGIC);
	zassert_equal(rec->p2, P2_MAGIC);
	zassert_equal(rec->p3, P3_MAGIC);
	zassert_equal(rec->prio, prio, "run %d priority %d", run, rec->prio);
	zassert_str_equal(rec->name, "restartee");
}

static void restart_before(void *fixture)
{
	ARG_UNUSED(fixture);

	atomic_clear(&runs);
	atomic_clear(&fatal_count);
	last_reason = 0U;
	ran_on = false;
	k_sem_reset(&started);
	k_sem_reset(&joined);
	k_sem_reset(&workq_blocked);
	k_sem_reset(&workq_unblock);
	(void)memset(script, 0, sizeof(script));
	(void)memset(records, 0, sizeof(records));
}

static void restart_after(void *fixture)
{
	ARG_UNUSED(fixture);

	stop_worker();

	if (joiner_created) {
		k_thread_abort(&joiner);
		joiner_created = false;
	}

	zassert_false(ran_on, "a run continued after it should have ended");
}

/**
 * @brief Verify the return codes of k_thread_restart_limit_set().
 */
ZTEST(thread_restart, test_limit_set_errors)
{
	k_tid_t tid;

	zassert_equal(k_thread_restart_limit_set(&z_idle_threads[0], 1), -EINVAL);
	zassert_equal(k_thread_restart_limit_set(&z_main_thread, 1), -EINVAL);
	zassert_equal(k_thread_restart_limit_set(k_work_queue_thread_get(&k_sys_work_q), 1),
		      -ENOTSUP);

	tid = create_worker(K_USER, WORKER_PRIO);
	zassert_equal(k_thread_restart_limit_set(tid, 1), -ENOTSUP);
	k_thread_abort(tid);
	worker_created = false;

	script[0] = BEHAVE_RETURN;
	tid = create_worker(0, WORKER_PRIO);
	zassert_ok(k_thread_restart_limit_set(tid, 3));
	zassert_ok(k_thread_restart_limit_set(tid, 0), "disarming failed");
	zassert_equal(k_thread_restart_limit_get(tid), 0);
	k_thread_start(tid);
	zassert_ok(k_thread_join(tid, K_MSEC(WAIT_MS)));
	zassert_equal(k_thread_restart_limit_set(tid, 1), -EINVAL);
	zassert_equal(atomic_get(&runs), 1);
	worker_created = false;
}

/**
 * @brief Verify that a thread hitting k_oops() is created again.
 *
 * The restarted thread gets its original entry point, arguments and
 * priority, and keeps its name.
 */
ZTEST(thread_restart, test_oops_restarts)
{
	k_tid_t tid = create_worker(0, WORKER_PRIO);

	script[0] = BEHAVE_OOPS;
	start_worker(tid, 3);
	wait_runs(2);

	zassert_equal(atomic_get(&runs), 2);
	zassert_equal(atomic_get(&fatal_count), 1);
	zassert_equal(last_reason, K_ERR_KERNEL_OOPS);
	zassert_equal(k_thread_restart_count_get(tid), 1);
	zassert_equal(k_thread_restart_limit_get(tid), 3);
	check_record(0, WORKER_PRIO);
	check_record(1, WORKER_PRIO);
	zassert_equal(k_thread_priority_get(tid), WORKER_PRIO);
}

/**
 * @brief Verify that restarts stop once the limit is reached.
 */
ZTEST(thread_restart, test_limit_exhausted)
{
	k_tid_t tid = create_worker(0, WORKER_PRIO);

	script[0] = BEHAVE_OOPS;
	script[1] = BEHAVE_OOPS;
	script[2] = BEHAVE_OOPS;
	script[3] = BEHAVE_OOPS;
	start_worker(tid, 2);

	zassert_ok(k_thread_join(tid, K_MSEC(WAIT_MS)));
	wait_runs(3);
	k_msleep(50);
	zassert_equal(atomic_get(&runs), 3);
	zassert_equal(atomic_get(&fatal_count), 3);
	zassert_equal(k_thread_restart_count_get(tid), 2);
	zassert_true(z_is_thread_dead(tid));
	worker_created = false;
}

/**
 * @brief Verify that a kernel panic is never restarted.
 */
ZTEST(thread_restart, test_panic_not_restarted)
{
	k_tid_t tid = create_worker(0, WORKER_PRIO);

	script[0] = BEHAVE_PANIC;
	script[1] = BEHAVE_PANIC;
	start_worker(tid, 3);

	zassert_ok(k_thread_join(tid, K_MSEC(WAIT_MS)));
	k_msleep(50);
	zassert_equal(atomic_get(&runs), 1);
	zassert_equal(last_reason, K_ERR_KERNEL_PANIC);
	zassert_equal(k_thread_restart_count_get(tid), 0);
	worker_created = false;
}

/**
 * @brief Verify that an armed essential thread hitting k_oops() is restarted.
 */
ZTEST(thread_restart, test_essential_oops)
{
	k_tid_t tid = create_worker(K_ESSENTIAL, WORKER_PRIO);

	script[0] = BEHAVE_OOPS;
	start_worker(tid, 1);
	wait_runs(2);

	zassert_equal(atomic_get(&fatal_count), 1);
	zassert_equal(last_reason, K_ERR_KERNEL_OOPS);
	zassert_equal(k_thread_restart_count_get(tid), 1);
	zassert_true(z_is_thread_essential(tid));
	check_record(1, WORKER_PRIO);
}

/**
 * @brief Verify that an armed essential thread returning from its entry
 * point is restarted instead of panicking.
 */
ZTEST(thread_restart, test_essential_return)
{
	k_tid_t tid = create_worker(K_ESSENTIAL, WORKER_PRIO);

	script[0] = BEHAVE_RETURN;
	start_worker(tid, 1);
	wait_runs(2);

	zassert_equal(atomic_get(&fatal_count), 0);
	zassert_equal(k_thread_restart_count_get(tid), 1);
	check_record(1, WORKER_PRIO);
}

/**
 * @brief Verify that an armed essential thread aborting itself is restarted.
 */
ZTEST(thread_restart, test_essential_self_abort)
{
	k_tid_t tid = create_worker(K_ESSENTIAL, WORKER_PRIO);

	script[0] = BEHAVE_SELF_ABORT;
	start_worker(tid, 1);
	wait_runs(2);

	zassert_equal(atomic_get(&fatal_count), 0);
	zassert_equal(k_thread_restart_count_get(tid), 1);
	check_record(1, WORKER_PRIO);
}

/**
 * @brief Verify the state of a thread while its restart is pending, and that
 * k_thread_abort() cancels the restart.
 */
ZTEST(thread_restart, test_abort_cancels_pending)
{
	k_tid_t tid = create_worker(0, WORKER_PRIO);

	block_workq();
	script[0] = BEHAVE_OOPS;
	start_worker(tid, 3);
	wait_runs(1);
	wait_dead(tid);

	zassert_equal(k_thread_join(tid, K_NO_WAIT), -EBUSY);
	zassert_equal(k_thread_join(tid, K_MSEC(10)), -EAGAIN);
	zassert_equal(k_thread_restart_limit_set(tid, 1), -EBUSY);
	zassert_equal(k_thread_restart_count_get(tid), 1);

	start_joiner();
	k_thread_abort(tid);
	zassert_ok(k_sem_take(&joined, K_MSEC(WAIT_MS)));
	zassert_ok(joiner_ret);
	zassert_ok(k_thread_join(tid, K_NO_WAIT));

	unblock_workq();
	zassert_equal(atomic_get(&runs), 1);
	zassert_equal(atomic_get(&fatal_count), 1);
	worker_created = false;
}

/**
 * @brief Verify that k_thread_join() waits for the restarted thread to exit.
 */
ZTEST(thread_restart, test_join_waits_for_restart)
{
	k_tid_t tid = create_worker(0, WORKER_PRIO);

	block_workq();
	script[0] = BEHAVE_OOPS;
	start_worker(tid, 3);
	wait_runs(1);
	wait_dead(tid);
	start_joiner();

	unblock_workq();
	wait_runs(1);
	zassert_equal(k_sem_take(&joined, K_MSEC(20)), -EAGAIN, "join returned on restart");

	stop_worker();
	zassert_ok(k_sem_take(&joined, K_MSEC(WAIT_MS)));
	zassert_ok(joiner_ret);
}

static void other_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_sem_give(&started);
	k_sleep(K_FOREVER);
}

/**
 * @brief Verify that k_thread_create() on a thread waiting to be restarted
 * cancels the restart and wakes its joiners.
 */
ZTEST(thread_restart, test_create_cancels_pending)
{
	k_tid_t tid = create_worker(0, WORKER_PRIO);

	block_workq();
	script[0] = BEHAVE_OOPS;
	start_worker(tid, 3);
	wait_runs(1);
	wait_dead(tid);
	start_joiner();

	tid = k_thread_create(&worker, worker_stack, STACK_SIZE, other_entry, NULL, NULL, NULL,
			      WORKER_PRIO, 0, K_NO_WAIT);
	zassert_ok(k_sem_take(&joined, K_MSEC(WAIT_MS)));
	zassert_ok(joiner_ret);
	zassert_equal(k_thread_restart_limit_get(tid), 0);
	zassert_equal(k_thread_restart_count_get(tid), 0);
	wait_runs(1);

	unblock_workq();
	zassert_equal(atomic_get(&runs), 1, "restart was not cancelled");
}

/**
 * @brief Restart an essential thread many times, alternating between
 * returning from its entry point and aborting itself.
 */
ZTEST(thread_restart, test_essential_stress)
{
	k_tid_t tid = create_worker(K_ESSENTIAL, WORKER_PRIO);

	script[0] = BEHAVE_STRESS;
	script[1] = BEHAVE_STRESS;
	script[2] = BEHAVE_STRESS;
	script[3] = BEHAVE_STRESS;
	start_worker(tid, UINT8_MAX);

	for (int i = 0; (i < 10 * WAIT_MS) && (atomic_get(&runs) <= STRESS_RUNS); i++) {
		k_msleep(1);
	}

	zassert_equal(atomic_get(&runs), STRESS_RUNS + 1);
	zassert_equal(k_thread_restart_count_get(tid), STRESS_RUNS);
	zassert_equal(atomic_get(&fatal_count), 0);
}

/**
 * @brief Verify that a thread pinned to another CPU is restarted there.
 */
ZTEST(thread_restart, test_pinned_restart)
{
#ifdef CONFIG_SCHED_CPU_MASK
	k_tid_t tid;

	if (arch_num_cpus() < 2) {
		ztest_test_skip();
	}

	tid = create_worker(0, WORKER_PRIO);
	zassert_ok(k_thread_cpu_pin(tid, 1));
	script[0] = BEHAVE_OOPS;
	start_worker(tid, 1);
	wait_runs(2);

	zassert_equal(records[0].cpu, 1);
	zassert_equal(records[1].cpu, 1);
	zassert_equal(tid->base.cpu_mask, BIT(1));
	zassert_equal(atomic_get(&fatal_count), 1);
#else
	ztest_test_skip();
#endif /* CONFIG_SCHED_CPU_MASK */
}

#ifdef CONFIG_USERSPACE
static K_SEM_DEFINE(user_sem, 0, 1);

static void user_join_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	if (k_thread_join(p1, K_NO_WAIT) == -EBUSY) {
		k_sem_give(&user_sem);
	}
}
#endif /* CONFIG_USERSPACE */

/**
 * @brief Verify that a thread that dropped to user mode is not restarted.
 */
ZTEST(thread_restart, test_user_mode_not_restarted)
{
	Z_TEST_SKIP_IFNDEF(CONFIG_USERSPACE);

	k_tid_t tid = create_worker(0, WORKER_PRIO);

	script[0] = BEHAVE_USER_OOPS;
	start_worker(tid, 3);
	zassert_ok(k_thread_join(tid, K_MSEC(WAIT_MS)));
	k_msleep(50);

	zassert_equal(atomic_get(&runs), 1);
	zassert_equal(atomic_get(&fatal_count), 1);
	zassert_equal(k_thread_restart_count_get(tid), 0);
	worker_created = false;
}

/**
 * @brief Verify that a user mode thread sees a thread waiting to be
 * restarted as alive.
 */
ZTEST(thread_restart, test_user_join_pending)
{
#ifdef CONFIG_USERSPACE
	k_tid_t tid = create_worker(0, WORKER_PRIO);
	k_tid_t user;

	k_sem_reset(&user_sem);
	block_workq();
	script[0] = BEHAVE_OOPS;
	start_worker(tid, 3);
	wait_runs(1);
	wait_dead(tid);

	user = k_thread_create(&joiner, joiner_stack, STACK_SIZE, user_join_entry, tid, NULL, NULL,
			       JOINER_PRIO, K_USER, K_FOREVER);
	joiner_created = true;
	k_thread_access_grant(user, tid, &user_sem);
	k_thread_start(user);
	zassert_ok(k_thread_join(user, K_MSEC(WAIT_MS)));
	joiner_created = false;
	zassert_ok(k_sem_take(&user_sem, K_NO_WAIT), "user mode join did not see a live thread");

	unblock_workq();
	wait_runs(1);
#else
	ztest_test_skip();
#endif /* CONFIG_USERSPACE */
}

/**
 * @brief Verify that the stack of a thread waiting to be restarted cannot be
 * freed.
 */
ZTEST(thread_restart, test_stack_free_busy)
{
#ifdef CONFIG_DYNAMIC_THREAD
	k_thread_stack_t *stack = k_thread_stack_alloc(CONFIG_DYNAMIC_THREAD_STACK_SIZE, 0);
	k_tid_t tid;

	zassert_not_null(stack);
	tid = k_thread_create(&worker, stack, CONFIG_DYNAMIC_THREAD_STACK_SIZE, worker_entry,
			      P1_MAGIC, P2_MAGIC, P3_MAGIC, WORKER_PRIO, 0, K_FOREVER);
	worker_created = true;

	block_workq();
	script[0] = BEHAVE_OOPS;
	start_worker(tid, 1);
	wait_runs(1);
	wait_dead(tid);

	zassert_equal(k_thread_stack_free(stack), -EBUSY);
	k_thread_abort(tid);
	zassert_ok(k_thread_join(tid, K_NO_WAIT));
	worker_created = false;
	zassert_ok(k_thread_stack_free(stack));
	unblock_workq();
#else
	ztest_test_skip();
#endif /* CONFIG_DYNAMIC_THREAD */
}

/**
 * @brief Verify that a thread hitting a CPU exception is restarted.
 */
ZTEST(thread_restart, test_cpu_exception_restarts)
{
	k_tid_t tid;

	/* Host processes do not take CPU exceptions as fatal errors */
	if (IS_ENABLED(CONFIG_ARCH_POSIX)) {
		ztest_test_skip();
	}

	tid = create_worker(0, WORKER_PRIO);
	script[0] = BEHAVE_TRAP;
	start_worker(tid, 1);
	wait_runs(2);

	zassert_equal(atomic_get(&fatal_count), 1);
	zassert_equal(k_thread_restart_count_get(tid), 1);
	check_record(1, WORKER_PRIO);
}

/**
 * @brief Verify that a thread hit by a fatal error in an interrupt handler
 * is restarted.
 */
ZTEST(thread_restart, test_isr_fatal_restarts)
{
	k_tid_t tid;

	/* irq_offload() does not run an interrupt handler there */
	if (IS_ENABLED(CONFIG_ARCH_POSIX)) {
		ztest_test_skip();
	}

	tid = create_worker(0, WORKER_PRIO);
	script[0] = BEHAVE_ISR_FATAL;
	start_worker(tid, 1);
	wait_runs(2);

	zassert_equal(atomic_get(&fatal_count), 1);
	zassert_equal(last_reason, K_ERR_KERNEL_OOPS);
	zassert_equal(k_thread_restart_count_get(tid), 1);
	check_record(0, WORKER_PRIO);
	check_record(1, WORKER_PRIO);
}

#ifdef CONFIG_SCHED_CPU_MASK
static volatile bool spinner_running;

static void spinner_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	spinner_running = true;
	while (true) {
		k_busy_wait(100);
	}
}
#endif /* CONFIG_SCHED_CPU_MASK */

/**
 * @brief Verify that a thread halted by an interrupt handler is restarted
 * when another thread of that CPU is aborted before the restart runs.
 *
 * The abort from another CPU halts the running thread of CPU 1 while that CPU
 * switches away from it. Where the next thread is picked before the interrupt
 * handler exits, as on Xtensa, this sets up the dummy thread of CPU 1 again.
 */
ZTEST(thread_restart, test_isr_halt_then_remote_abort)
{
#ifdef CONFIG_SCHED_CPU_MASK
	static struct k_thread spinner;
	static K_THREAD_STACK_DEFINE(spinner_stack, STACK_SIZE);
	k_tid_t tid;
	k_tid_t spinner_tid;

	if (IS_ENABLED(CONFIG_ARCH_POSIX) || (arch_num_cpus() < 2)) {
		ztest_test_skip();
	}

	block_workq();

	tid = create_worker(0, WORKER_PRIO);
	zassert_ok(k_thread_cpu_pin(tid, 1));
	script[0] = BEHAVE_ISR_FATAL;
	start_worker(tid, 1);
	wait_runs(1);
	wait_dead(&worker);

	/* Occupy CPU 1, so this thread runs on another CPU once it sleeps */
	spinner_running = false;
	spinner_tid = k_thread_create(&spinner, spinner_stack, STACK_SIZE, spinner_entry, NULL,
				      NULL, NULL, K_PRIO_COOP(0), 0, K_FOREVER);
	zassert_ok(k_thread_cpu_pin(spinner_tid, 1));
	k_thread_start(spinner_tid);
	for (int i = 0; (i < WAIT_MS) && !spinner_running; i++) {
		k_msleep(1);
	}
	zassert_true(spinner_running, "spinner did not start");

	k_thread_abort(spinner_tid);
	zassert_ok(k_thread_join(spinner_tid, K_MSEC(WAIT_MS)));

	unblock_workq();
	wait_runs(1);

	zassert_equal(atomic_get(&fatal_count), 1);
	zassert_equal(k_thread_restart_count_get(tid), 1);
	zassert_equal(records[1].cpu, 1);
	check_record(1, WORKER_PRIO);
#else
	ztest_test_skip();
#endif /* CONFIG_SCHED_CPU_MASK */
}

/* What sys_trace_thread_create_user() does while the worker is set up again */
enum race_action {
	RACE_NONE,
	RACE_WAKEUP,
	RACE_WAKEUP_ISR,
	RACE_SUSPEND,
	RACE_ABORT,
	RACE_ABORT_ISR,
	RACE_CREATE,
	RACE_ABORT_OTHER,
};

static volatile enum race_action race_action;
static volatile bool race_done;
static volatile bool race_dead;
static K_SEM_DEFINE(race_go, 0, 1);

#ifdef CONFIG_TRACING_USER
static void isr_abort(const void *arg)
{
	k_thread_abort((k_tid_t)arg);
}

static void isr_wakeup(const void *arg)
{
	k_tid_t tid = (k_tid_t)arg;

	k_wakeup(tid);
	k_thread_start(tid);
	k_thread_resume(tid);
}

/* Called at the end of the thread setup, also when the system work queue
 * sets up a thread to restart it
 */
void sys_trace_thread_create_user(struct k_thread *thread)
{
	enum race_action action = race_action;

	if ((thread != &worker) || (action == RACE_NONE) ||
	    (k_current_get() != k_work_queue_thread_get(&k_sys_work_q))) {
		return;
	}

	race_action = RACE_NONE;

	switch (action) {
	case RACE_WAKEUP:
		isr_wakeup(thread);
		break;
	case RACE_WAKEUP_ISR:
		irq_offload(isr_wakeup, thread);
		break;
	case RACE_SUSPEND:
		k_thread_suspend(thread);
		break;
	case RACE_ABORT:
		k_thread_abort(thread);
		break;
	case RACE_ABORT_ISR:
		irq_offload(isr_abort, thread);
		break;
	case RACE_CREATE:
	case RACE_ABORT_OTHER:
		/* Wait for k_thread_create() or k_thread_abort() on another
		 * CPU to block
		 */
		k_sem_give(&race_go);
		for (int i = 0; i < WAIT_MS; i++) {
			if (joiner.base.pended_on == &worker.join_queue) {
				break;
			}
			k_busy_wait(1000);
		}
		break;
	default:
		break;
	}

	race_dead = z_is_thread_dead(thread);
	race_done = true;
}
#endif /* CONFIG_TRACING_USER */

static bool worker_listed;

static void find_worker(const struct k_thread *thread, void *user_data)
{
	ARG_UNUSED(user_data);

	if (thread == &worker) {
		worker_listed = true;
	}
}

/* Run @p action while the system work queue sets up the worker again */
static k_tid_t start_race(enum race_action action, uint32_t options)
{
	k_tid_t tid = create_worker(options, WORKER_PRIO);

	race_done = false;
	race_dead = false;
	race_action = action;
	script[0] = BEHAVE_OOPS;
	start_worker(tid, 1);

	return tid;
}

static void check_race_dead(void)
{
	zassert_true(race_done, "rebuild not reached");
	zassert_true(race_dead, "thread was alive while being set up");
}

/**
 * @brief Verify that waking up, starting or resuming a thread being set up
 * again does not make it runnable early, from a thread or an ISR.
 */
ZTEST(thread_restart, test_wakeup_races_restart)
{
	Z_TEST_SKIP_IFNDEF(CONFIG_TRACING_USER);

	(void)start_race(RACE_WAKEUP, 0);
	wait_runs(2);
	check_race_dead();
	stop_worker();

	atomic_clear(&runs);
	k_sem_reset(&started);
	(void)start_race(RACE_WAKEUP_ISR, 0);
	wait_runs(2);
	check_race_dead();
	k_msleep(20);
	zassert_equal(atomic_get(&runs), 2);
}

/**
 * @brief Verify that a k_thread_suspend() of a thread being set up again has
 * no effect.
 */
ZTEST(thread_restart, test_suspend_races_restart)
{
	Z_TEST_SKIP_IFNDEF(CONFIG_TRACING_USER);

	(void)start_race(RACE_SUSPEND, 0);
	wait_runs(2);
	check_race_dead();
}

static void check_race_aborted(k_tid_t tid)
{
	zassert_ok(k_thread_join(tid, K_MSEC(WAIT_MS)));
	check_race_dead();
	k_msleep(20);
	zassert_equal(atomic_get(&runs), 1, "aborted thread was started");
	zassert_true(z_is_thread_dead(tid));

	worker_listed = false;
	k_thread_foreach(find_worker, NULL);
	zassert_false(worker_listed, "aborted thread still listed");
	worker_created = false;

	/* The thread object can be used again */
	atomic_clear(&runs);
	k_sem_reset(&started);
	script[0] = BEHAVE_WAIT;
	tid = create_worker(0, WORKER_PRIO);
	k_thread_start(tid);
	wait_runs(1);
}

/**
 * @brief Verify that aborting a thread being set up again leaves it dead.
 */
ZTEST(thread_restart, test_abort_races_restart)
{
	k_tid_t tid;

	Z_TEST_SKIP_IFNDEF(CONFIG_TRACING_USER);

	tid = start_race(RACE_ABORT, 0);
	check_race_aborted(tid);
	zassert_equal(atomic_get(&fatal_count), 1);
}

/**
 * @brief Verify that aborting a thread being set up again from an ISR leaves
 * it dead.
 */
ZTEST(thread_restart, test_isr_abort_races_restart)
{
	k_tid_t tid;

	Z_TEST_SKIP_IFNDEF(CONFIG_TRACING_USER);

	tid = start_race(RACE_ABORT_ISR, 0);
	check_race_aborted(tid);
}

static void creator_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	(void)k_sem_take(&race_go, K_FOREVER);
	k_thread_create(&worker, worker_stack, STACK_SIZE, other_entry, NULL, NULL, NULL,
			WORKER_PRIO, 0, K_NO_WAIT);
	k_sem_give(&joined);
}

/**
 * @brief Verify that k_thread_create() on a thread being set up again on
 * another CPU waits for the restart to be cancelled.
 */
ZTEST(thread_restart, test_create_races_restart)
{
	Z_TEST_SKIP_IFNDEF(CONFIG_TRACING_USER);

	if (arch_num_cpus() < 2) {
		ztest_test_skip();
	}

	k_sem_reset(&race_go);
	k_thread_create(&joiner, joiner_stack, STACK_SIZE, creator_entry, NULL, NULL, NULL,
			JOINER_PRIO, 0, K_NO_WAIT);
	joiner_created = true;

	(void)start_race(RACE_CREATE, 0);
	zassert_ok(k_sem_take(&joined, K_MSEC(WAIT_MS)));
	zassert_true(race_done, "rebuild not reached");
	zassert_true(race_dead, "thread was alive while being set up");

	/* worker_entry once, then other_entry from k_thread_create() */
	wait_runs(2);
	k_msleep(20);
	zassert_equal(atomic_get(&runs), 1, "restart was not cancelled");
	zassert_equal(k_thread_restart_count_get(&worker), 0);
}

static void race_aborter_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	(void)k_sem_take(&race_go, K_FOREVER);
	k_thread_abort(&worker);

	/* Nothing may be left to wait for once k_thread_abort() returns */
	joiner_ret = k_thread_join(&worker, K_NO_WAIT);
	k_sem_give(&joined);
}

/**
 * @brief Verify that k_thread_abort() on a thread being set up again on
 * another CPU returns only once the system work queue is done with it.
 */
ZTEST(thread_restart, test_abort_other_cpu_races_restart)
{
	k_tid_t tid;

	Z_TEST_SKIP_IFNDEF(CONFIG_TRACING_USER);

	if (arch_num_cpus() < 2) {
		ztest_test_skip();
	}

	k_sem_reset(&race_go);
	joiner_ret = -1;
	k_thread_create(&joiner, joiner_stack, STACK_SIZE, race_aborter_entry, NULL, NULL, NULL,
			JOINER_PRIO, 0, K_NO_WAIT);
	joiner_created = true;

	tid = start_race(RACE_ABORT_OTHER, 0);
	zassert_ok(k_sem_take(&joined, K_MSEC(WAIT_MS)));
	zassert_ok(joiner_ret, "abort returned while the thread was set up again");
	check_race_aborted(tid);
}

ZTEST_SUITE(thread_restart, NULL, NULL, restart_before, restart_after, NULL);
