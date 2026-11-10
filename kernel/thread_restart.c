/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Restart of threads after a fatal error
 *
 * A thread armed with k_thread_restart_limit_set() is queued here when it is
 * about to be aborted, and created again on the same thread object from the
 * system work queue once nothing runs on its stack any more. The thread stays
 * dead until it is started again under the scheduler spinlock, which also
 * protects all the state here.
 */

#include <zephyr/kernel.h>
#include <zephyr/internal/syscall_handler.h>
#include <zephyr/sys/barrier.h>
#include <kernel_internal.h>
#include <ksched.h>
#include <kspinlock.h>
#include <kswap.h>
#include <kthread.h>
#include <timeout_q.h>

/* Restart requested, thread in restart_list */
#define Z_THREAD_RESTART_PENDING    BIT(0)
/* Never restart until created again */
#define Z_THREAD_RESTART_VETO       BIT(1)
/* Halted by an interrupt handler while it was the current thread */
#define Z_THREAD_RESTART_ISR_HALT   BIT(2)
/* Being set up again by restart_handler(), still in restart_list */
#define Z_THREAD_RESTART_REBUILDING BIT(3)
/* Aborted while a restart was pending or in progress: finish the abort
 * instead of starting the thread. Cleared when created again.
 */
#define Z_THREAD_RESTART_CANCEL     BIT(4)
/* Requested by z_fatal_error(), whose own abort of the thread follows */
#define Z_THREAD_RESTART_FATAL      BIT(5)
/* Requested by z_fatal_error() in an ISR */
#define Z_THREAD_RESTART_FATAL_ISR  BIT(6)

static void restart_handler(struct k_work *work);
static void retry_expired(struct k_timer *timer);

static sys_slist_t restart_list = SYS_SLIST_STATIC_INIT(&restart_list);
static K_WORK_DELAYABLE_DEFINE(restart_work, restart_handler);
static K_TIMER_DEFINE(retry_timer, retry_expired, NULL);

/* Return true if nothing uses the stack or the thread object of a dead
 * thread any more, so it can be set up again or released without waiting.
 */
static bool restart_safe_locked(struct k_thread *thread)
{
	unsigned int num_cpus = arch_num_cpus();

	if (!z_is_thread_dead(thread) || z_is_thread_halting(thread)) {
		return false;
	}

	/* A timeout handler of the thread may still be in flight on another
	 * CPU. z_smp_halt_remote() leaves waiting for it to the restart.
	 */
	if (IS_ENABLED(CONFIG_SMP) && (z_try_abort_thread_timeout(thread) == -EAGAIN)) {
		return false;
	}

	for (unsigned int i = 0U; i < num_cpus; i++) {
		if (_kernel.cpus[i].current == thread) {
			return false;
		}
	}

#ifdef CONFIG_USE_SWITCH
	/* Context not saved yet, see z_sched_switch_spin() */
	if (thread->switch_handle == NULL) {
		return false;
	}
#endif /* CONFIG_USE_SWITCH */

	/* An interrupt handler that halted the thread replaced it with the
	 * dummy thread of its CPU, and may use the stack of the thread until
	 * that CPU has switched away from the dummy thread.
	 */
	if ((thread->restart.flags & Z_THREAD_RESTART_ISR_HALT) != 0U) {
		const struct k_thread *dummy = &_thread_dummies[thread->restart.halt_cpu];

#ifdef CONFIG_USE_SWITCH
		/* Saved by the context switch once done with the old stack */
		if (*(void *const volatile *)&dummy->switch_handle == NULL) {
			return false;
		}
#else
		if (_kernel.cpus[thread->restart.halt_cpu].current == dummy) {
			return false;
		}
#endif /* CONFIG_USE_SWITCH */
	}

	/* Order the checks above before reusing the stack */
	barrier_dmem_fence_full();

	return true;
}

/* Finish the abort of a dead thread whose restart is cancelled. */
static void finish_cancel_locked(struct k_thread *thread)
{
	(void)sys_slist_find_and_remove(&restart_list, &thread->restart.node);
	thread->restart.flags &= ~Z_THREAD_RESTART_PENDING;
	thread->restart.flags |= Z_THREAD_RESTART_CANCEL;

	/* halt_thread() left both for the restarted thread */
	z_sched_unpend_all_locked(&thread->join_queue);
#ifdef CONFIG_USERSPACE
	k_object_uninit(thread->stack_obj);
	k_object_uninit(thread);
#endif /* CONFIG_USERSPACE */
}

/* Spin with the scheduler spinlock released, for a short wait on another CPU */
static void spin_unlocked(k_spinlock_key_t *key)
{
	unsigned int irq_key;

	z_sched_spinlock_unlock(*key);
	irq_key = arch_irq_lock();
	arch_spin_relax(); /* Requires interrupts be masked */
	arch_irq_unlock(irq_key);
	*key = z_sched_spinlock_lock();
}

/* Return true if the system work queue thread runs on another CPU */
static bool work_queue_runs_elsewhere_locked(void)
{
	const struct k_thread *wq_thread = k_work_queue_thread_get(&k_sys_work_q);
	unsigned int num_cpus = arch_num_cpus();

	for (unsigned int i = 0U; i < num_cpus; i++) {
		if ((i != _current_cpu->id) && (_kernel.cpus[i].current == wq_thread)) {
			return true;
		}
	}

	return false;
}

/* Make sure restart_handler() runs, or that retry_expired() cancels the
 * pending restarts if the system work queue refuses work. Called without the
 * scheduler spinlock held.
 */
static void kick(void)
{
	if (k_work_schedule(&restart_work, K_NO_WAIT) < 0) {
		k_timer_start(&retry_timer, K_TICKS(1), K_NO_WAIT);
	}
}

/* Wait until restart_thread() is done with @p thread, whose restart in
 * progress is cancelled, with the scheduler spinlock held as @p key. Return
 * false if the caller cannot wait: an ISR while the system work queue does
 * not run on another CPU, or the system work queue thread itself.
 */
static bool wait_rebuild_locked(struct k_thread *thread, k_spinlock_key_t *key)
{
	while ((thread->restart.flags & Z_THREAD_RESTART_REBUILDING) != 0U) {
		if (!arch_is_in_isr() && (_current != k_work_queue_thread_get(&k_sys_work_q))) {
			/* Woken when restart_thread() finishes the abort */
			z_sched_add_to_waitq_locked(_current, &thread->join_queue);
			(void)z_swap_locked(*key);
			*key = z_sched_spinlock_lock();
		} else if (work_queue_runs_elsewhere_locked()) {
			spin_unlocked(key);
		} else {
			return false;
		}
	}

	return true;
}

/* Finish the cancelled restart of the dead @p thread, with the scheduler
 * spinlock held as @p key: wait for a restart in progress to be undone, or
 * for the thread to be safe to release. The wait only depends on other CPUs.
 * Return false if the caller cannot wait: then restart_thread(),
 * restart_handler() or retry_expired() finishes it later.
 */
static bool wait_cancel_locked(struct k_thread *thread, k_spinlock_key_t *key)
{
	struct _thread_restart *restart = &thread->restart;

	while (sys_slist_find(&restart_list, &restart->node, NULL)) {
		if ((restart->flags & Z_THREAD_RESTART_REBUILDING) != 0U) {
			if (!wait_rebuild_locked(thread, key)) {
				return false;
			}
			continue;
		}

		if (restart_safe_locked(thread)) {
			finish_cancel_locked(thread);
			continue;
		}

		/* This CPU only leaves the dead thread once this ISR returns */
		if (arch_is_in_isr() && ((restart->flags & Z_THREAD_RESTART_ISR_HALT) != 0U) &&
		    (restart->halt_cpu == _current_cpu->id)) {
			return false;
		}

		/* Another CPU is still halting or switching away */
		spin_unlocked(key);
	}

	return true;
}

void z_thread_restart_init(struct k_thread *thread, k_thread_stack_t *stack, size_t stack_size,
			   k_thread_entry_t entry, void *p1, void *p2, void *p3, int prio,
			   uint32_t options)
{
	struct _thread_restart *restart = &thread->restart;
	k_spinlock_key_t key = z_sched_spinlock_lock();

	/* The object may be uninitialized memory: trust its flags only once it
	 * is found in the list. Cancel a pending restart, or one in progress,
	 * and wait for it, so that the object is never set up twice at once.
	 */
	if (sys_slist_find(&restart_list, &restart->node, NULL)) {
		restart->flags |= Z_THREAD_RESTART_CANCEL;
		if (!wait_cancel_locked(thread, &key)) {
			/* Threads may not be created in ISRs */
			__ASSERT(false, "thread %p is being restarted", thread);
			z_sched_spinlock_unlock(key);
			k_panic();
			key = z_sched_spinlock_lock();
			while (sys_slist_find(&restart_list, &restart->node, NULL)) {
				spin_unlocked(&key);
			}
		}
	}

	restart->stack = stack;
	restart->stack_size = stack_size;
	restart->entry = entry;
	restart->p1 = p1;
	restart->p2 = p2;
	restart->p3 = p3;
	restart->options = options;
	restart->prio = (int8_t)prio;
	restart->limit = 0U;
	restart->count = 0U;
	restart->flags = 0U;
	restart->halt_cpu = 0U;

	z_sched_spinlock_unlock(key);
}

/* Return true if a restart of the live @p thread may be requested */
static bool may_restart_locked(const struct k_thread *thread)
{
	const struct _thread_restart *restart = &thread->restart;
	const uint8_t refused =
		Z_THREAD_RESTART_VETO | Z_THREAD_RESTART_REBUILDING | Z_THREAD_RESTART_CANCEL;

	if (is_thread_dummy(thread) || z_is_thread_dead(thread)) {
		return false;
	}

	/* An abort in progress wins: its caller may still use the thread
	 * object once the thread is dead, see z_smp_halt_remote()
	 */
	if ((thread->base.thread_state & _THREAD_ABORTING) != 0U) {
		return false;
	}

	/* User mode threads are never restarted */
	if ((thread->base.user_options & K_USER) != 0U) {
		return false;
	}

	/* An essential thread that exits panics the kernel */
	if (z_is_thread_essential(thread)) {
		return false;
	}

	return (restart->count < restart->limit) && ((restart->flags & refused) == 0U);
}

/* Record that z_fatal_error() requested the restart of @p thread */
static void mark_fatal_locked(struct k_thread *thread)
{
	struct _thread_restart *restart = &thread->restart;

	restart->flags &= ~Z_THREAD_RESTART_FATAL_ISR;
	restart->flags |= Z_THREAD_RESTART_FATAL;
	if (arch_is_in_isr()) {
		restart->flags |= Z_THREAD_RESTART_FATAL_ISR;
	}
}

/* Return true if this abort of @p thread is the one z_fatal_error() makes
 * after requesting its restart: from the thread itself, in the same context.
 * Interrupts are not locked in between, but a different context is.
 */
static bool own_fatal_abort_locked(const struct k_thread *thread)
{
	uint8_t flags = thread->restart.flags;

	return ((flags & Z_THREAD_RESTART_FATAL) != 0U) && (thread == _current) &&
	       (((flags & Z_THREAD_RESTART_FATAL_ISR) != 0U) == arch_is_in_isr());
}

bool z_thread_restart_request(struct k_thread *thread, bool fatal)
{
	struct _thread_restart *restart = &thread->restart;
	k_spinlock_key_t key = z_sched_spinlock_lock();
	bool restarting = false;

	if ((restart->flags & Z_THREAD_RESTART_PENDING) != 0U) {
		if (fatal) {
			mark_fatal_locked(thread);
		}
		z_sched_spinlock_unlock(key);
		return true;
	}

	if (!may_restart_locked(thread)) {
		z_sched_spinlock_unlock(key);
		return false;
	}

	if (fatal) {
		mark_fatal_locked(thread);
	}
	restart->flags |= Z_THREAD_RESTART_PENDING;
	restart->count++;
	sys_slist_append(&restart_list, &restart->node);
	z_sched_spinlock_unlock(key);

	/* Outside the scheduler spinlock, which the work queue takes. Unlike
	 * k_work_submit(), this does not reschedule, so it is safe right
	 * before the halt.
	 */
	if (k_work_schedule(&restart_work, K_NO_WAIT) >= 0) {
		return true;
	}

	/* The system work queue refuses work: undo the request, unless an
	 * earlier run of the handler took the thread already.
	 */
	key = z_sched_spinlock_lock();
	if ((restart->flags & Z_THREAD_RESTART_PENDING) == 0U) {
		/* Taken by the handler, or cancelled by an abort */
		restarting = (restart->flags & Z_THREAD_RESTART_CANCEL) == 0U;
	} else if (!z_is_thread_dead(thread)) {
		(void)sys_slist_find_and_remove(&restart_list, &restart->node);
		restart->flags &= ~Z_THREAD_RESTART_PENDING;
		restart->count--;
	} else {
		/* Halted meanwhile on another CPU, relying on this request:
		 * finish its abort, which left the joiners waiting.
		 */
		while (((restart->flags & Z_THREAD_RESTART_PENDING) != 0U) &&
		       !restart_safe_locked(thread)) {
			spin_unlocked(&key);
		}
		if ((restart->flags & Z_THREAD_RESTART_PENDING) != 0U) {
			finish_cancel_locked(thread);
		}
		z_reschedule_locked(key);
		return false;
	}
	z_sched_spinlock_unlock(key);

	return restarting;
}

void z_thread_restart_veto(struct k_thread *thread)
{
	K_SPINLOCK(&_sched_spinlock) {
		thread->restart.flags |= Z_THREAD_RESTART_VETO;
	}
}

bool z_thread_restart_is_pending_locked(const struct k_thread *thread)
{
	const uint8_t busy = Z_THREAD_RESTART_PENDING | Z_THREAD_RESTART_REBUILDING;

	return (thread->restart.flags & busy) != 0U;
}

bool z_thread_restart_abort(struct k_thread *thread, bool *restarting, k_spinlock_key_t key)
{
	struct _thread_restart *restart = &thread->restart;
	const uint8_t busy = Z_THREAD_RESTART_PENDING | Z_THREAD_RESTART_REBUILDING;

	if ((restart->flags & busy) == 0U) {
		return false;
	}

	if (!z_is_thread_dead(thread)) {
		/* Restart requested, the thread is about to abort */
		if (own_fatal_abort_locked(thread)) {
			restart->flags &= ~(Z_THREAD_RESTART_FATAL | Z_THREAD_RESTART_FATAL_ISR);
			*restarting = true;
		} else if (!*restarting) {
			/* Another abort wins over the restart, which
			 * z_thread_restart_halted() cancels once it is halted
			 */
			restart->flags |= Z_THREAD_RESTART_CANCEL;
		} else {
			/* Requested by this caller, the thread itself */
		}

		return false;
	}

	/* Dead until restart_thread() starts it, which it will not do. Wait
	 * until the restart is undone, so that the caller may reuse the
	 * thread object once this returns.
	 */
	restart->flags |= Z_THREAD_RESTART_CANCEL;
	if (wait_cancel_locked(thread, &key)) {
		/* Cancelling may have woken joiners */
		z_reschedule_locked(key);
	} else {
		z_sched_spinlock_unlock(key);
		kick();
	}

	return true;
}

void z_thread_restart_halted(struct k_thread *thread)
{
	k_spinlock_key_t key = z_sched_spinlock_lock();
	struct _thread_restart *restart = &thread->restart;

	if (!sys_slist_find(&restart_list, &restart->node, NULL)) {
		z_sched_spinlock_unlock(key);
		return;
	}

	if (((restart->flags & Z_THREAD_RESTART_CANCEL) != 0U) &&
	    wait_cancel_locked(thread, &key)) {
		z_reschedule_locked(key);
		return;
	}

	z_sched_spinlock_unlock(key);

	/* The request may not have scheduled the work yet: its caller can be
	 * halted right after it
	 */
	kick();
}

void z_thread_restart_isr_halt_locked(struct k_thread *thread)
{
	thread->restart.flags |= Z_THREAD_RESTART_ISR_HALT;
	thread->restart.halt_cpu = (uint8_t)_current_cpu->id;
}

void z_thread_restart_dummify_locked(void)
{
	uint8_t cpu = (uint8_t)_current_cpu->id;
	struct k_thread *thread;

	/* The dummy thread no longer tells when this CPU left their stack */
	SYS_SLIST_FOR_EACH_CONTAINER(&restart_list, thread, restart.node) {
		if (thread->restart.halt_cpu == cpu) {
			thread->restart.flags &= ~Z_THREAD_RESTART_ISR_HALT;
		}
	}
}

bool z_thread_restart_stack_busy(const k_thread_stack_t *stack)
{
	struct k_thread *thread;
	bool busy = false;

	K_SPINLOCK(&_sched_spinlock) {
		SYS_SLIST_FOR_EACH_CONTAINER(&restart_list, thread, restart.node) {
			if (thread->restart.stack == stack) {
				busy = true;
				break;
			}
		}
	}

	return busy;
}

/* Set up the thread again, then start it, or finish its abort if it was
 * aborted meanwhile. The limit and count are kept: k_thread_restart_limit_set()
 * is refused meanwhile.
 */
static void restart_thread(struct k_thread *thread)
{
	k_spinlock_key_t key;

	z_thread_restart_setup(thread);

	key = z_sched_spinlock_lock();
	(void)sys_slist_find_and_remove(&restart_list, &thread->restart.node);
	thread->restart.flags &= ~Z_THREAD_RESTART_REBUILDING;

	if ((thread->restart.flags & Z_THREAD_RESTART_CANCEL) != 0U) {
		/* Undo the setup without ever making the thread runnable. This
		 * wakes the joiners, including a k_thread_create() waiting in
		 * z_thread_restart_init().
		 */
		thread->base.thread_state &= ~_THREAD_DEAD;
		z_sched_halt_thread_locked(thread, _THREAD_DEAD, &key);
	} else {
		/* A suspend of the dead thread does not carry over */
		thread->base.thread_state &= ~(_THREAD_DEAD | _THREAD_SUSPENDED);
		z_sched_ready_locked(thread);
	}

	z_reschedule_locked(key);
}

/* Take the first pending thread that is safe to set up again, after
 * finishing the abort of those whose restart is cancelled. Set *retry if some
 * thread is not safe yet. The thread stays in the list, so that its stack
 * stays busy, until restart_thread() is done.
 */
static struct k_thread *take_safe_thread(bool *retry)
{
	struct k_thread *thread;
	struct k_thread *next;
	struct k_thread *found = NULL;
	k_spinlock_key_t key = z_sched_spinlock_lock();

	*retry = false;
	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&restart_list, thread, next, restart.node) {
		if ((thread->restart.flags & Z_THREAD_RESTART_PENDING) == 0U) {
			continue;
		}

		if (!restart_safe_locked(thread)) {
			*retry = true;
		} else if ((thread->restart.flags & Z_THREAD_RESTART_CANCEL) != 0U) {
			finish_cancel_locked(thread);
		} else if (found == NULL) {
			found = thread;
		} else {
			/* Taken by a later iteration of restart_handler() */
		}
	}

	if (found != NULL) {
		found->restart.flags &= ~(Z_THREAD_RESTART_PENDING | Z_THREAD_RESTART_ISR_HALT |
					  Z_THREAD_RESTART_FATAL | Z_THREAD_RESTART_FATAL_ISR);
		found->restart.flags |= Z_THREAD_RESTART_REBUILDING;
	}

	/* Wakes the joiners of cancelled threads */
	z_reschedule_locked(key);

	return found;
}

static void restart_handler(struct k_work *work)
{
	struct k_thread *thread;
	bool retry = false;

	ARG_UNUSED(work);

	for (thread = take_safe_thread(&retry); thread != NULL; thread = take_safe_thread(&retry)) {
		restart_thread(thread);
	}

	if (retry) {
		/* Another CPU still runs on the stack of a dead thread. Check
		 * again later instead of spinning in the work queue.
		 */
		k_timer_start(&retry_timer, K_TICKS(1), K_NO_WAIT);
	}
}

static void retry_expired(struct k_timer *timer)
{
	struct k_thread *thread;
	struct k_thread *next;
	bool retry = false;
	k_spinlock_key_t key;

	if (k_work_schedule(&restart_work, K_NO_WAIT) >= 0) {
		return;
	}

	/* The system work queue refuses work: finish the abort of every thread
	 * waiting to be restarted, so that its joiners do not wait forever
	 */
	key = z_sched_spinlock_lock();
	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&restart_list, thread, next, restart.node) {
		if ((thread->restart.flags & Z_THREAD_RESTART_PENDING) == 0U) {
			continue;
		}

		if (restart_safe_locked(thread)) {
			finish_cancel_locked(thread);
		} else {
			retry = true;
		}
	}
	z_reschedule_locked(key);

	if (retry) {
		k_timer_start(timer, K_TICKS(1), K_NO_WAIT);
	}
}

int k_thread_restart_limit_set(k_tid_t thread, uint8_t limit)
{
	int ret = 0;

#ifdef CONFIG_USERSPACE
	/* Freed once the last permission on it is released, as when the thread
	 * dies, so it may not exist any more when the restart runs.
	 */
	if (k_object_is_dynamic(thread) || k_object_is_dynamic(thread->restart.stack)) {
		return -ENOTSUP;
	}
#endif /* CONFIG_USERSPACE */

	K_SPINLOCK(&_sched_spinlock) {
		if (z_thread_restart_is_pending_locked(thread)) {
			ret = -EBUSY;
		} else if (z_is_thread_dead(thread) || z_is_idle_thread_object(thread) ||
			   (thread == &z_main_thread)) {
			ret = -EINVAL;
		} else if (((thread->base.user_options & K_USER) != 0U) ||
			   ((thread->restart.flags & Z_THREAD_RESTART_VETO) != 0U) ||
			   z_work_q_is_queue_thread(thread)) {
			ret = -ENOTSUP;
		} else {
			thread->restart.limit = limit;
			thread->restart.count = 0U;
		}
	}

	return ret;
}

uint8_t k_thread_restart_limit_get(k_tid_t thread)
{
	return thread->restart.limit;
}

uint8_t k_thread_restart_count_get(k_tid_t thread)
{
	return thread->restart.count;
}
