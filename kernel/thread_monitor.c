/*
 * Copyright (c) 2010-2014 Wind River Systems, Inc.
 * Copyright (c) 2024 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <kthread.h>

struct k_spinlock z_thread_monitor_lock;
/*
 * Remove a thread from the kernel's list of active threads.
 */
void z_thread_monitor_exit(struct k_thread *thread)
{
	k_spinlock_key_t key = k_spin_lock(&z_thread_monitor_lock);

	if (thread == _kernel.threads) {
		_kernel.threads = _kernel.threads->next_thread;
	} else {
		struct k_thread *prev_thread;

		prev_thread = _kernel.threads;
		while ((prev_thread != NULL) &&
			(thread != prev_thread->next_thread)) {
			prev_thread = prev_thread->next_thread;
		}
		if (prev_thread != NULL) {
			prev_thread->next_thread = thread->next_thread;
		}
	}

	k_spin_unlock(&z_thread_monitor_lock, key);
}

/*
 * Helper function to iterate over threads with optional filtering and locking behavior.
 */
static void thread_foreach_helper(k_thread_user_cb_t user_cb, void *user_data,
		bool unlocked, bool filter_by_cpu, unsigned int cpu)
{
	struct k_thread *thread;
	k_spinlock_key_t key;

	__ASSERT(user_cb != NULL, "user_cb can not be NULL");

	if (filter_by_cpu) {
		__ASSERT(cpu < CONFIG_MP_MAX_NUM_CPUS, "cpu filter out of bounds");
	}

	key = k_spin_lock(&z_thread_monitor_lock);

	for (thread = _kernel.threads; thread; thread = thread->next_thread) {
		/* cpu is only defined when SMP=y*/
#ifdef CONFIG_SMP
		bool on_cpu = (thread->base.cpu == cpu);
#else
		bool on_cpu = false;
#endif
		if (filter_by_cpu && !on_cpu) {
			continue;
		}

		if (unlocked) {
			k_spin_unlock(&z_thread_monitor_lock, key);
			user_cb(thread, user_data);
			key = k_spin_lock(&z_thread_monitor_lock);
		} else {
			user_cb(thread, user_data);
		}
	}

	k_spin_unlock(&z_thread_monitor_lock, key);
}

/*
 * Public API functions using the helper.
 */
void k_thread_foreach(k_thread_user_cb_t user_cb, void *user_data)
{
	SYS_PORT_TRACING_FUNC_ENTER(k_thread, foreach);
	thread_foreach_helper(user_cb, user_data, false, false, 0);
	SYS_PORT_TRACING_FUNC_EXIT(k_thread, foreach);
}

void k_thread_foreach_unlocked(k_thread_user_cb_t user_cb, void *user_data)
{
	SYS_PORT_TRACING_FUNC_ENTER(k_thread, foreach_unlocked);
	thread_foreach_helper(user_cb, user_data, true, false, 0);
	SYS_PORT_TRACING_FUNC_EXIT(k_thread, foreach_unlocked);
}

#ifdef CONFIG_SMP
void k_thread_foreach_filter_by_cpu(unsigned int cpu, k_thread_user_cb_t user_cb,
		void *user_data)
{
	SYS_PORT_TRACING_FUNC_ENTER(k_thread, foreach);
	thread_foreach_helper(user_cb, user_data, false, true, cpu);
	SYS_PORT_TRACING_FUNC_EXIT(k_thread, foreach);
}

void k_thread_foreach_unlocked_filter_by_cpu(unsigned int cpu, k_thread_user_cb_t user_cb,
		void *user_data)
{
	SYS_PORT_TRACING_FUNC_ENTER(k_thread, foreach_unlocked);
	thread_foreach_helper(user_cb, user_data, true, true, cpu);
	SYS_PORT_TRACING_FUNC_EXIT(k_thread, foreach_unlocked);
}
#endif /* CONFIG_SMP */

#ifdef CONFIG_THREAD_MONITOR_CALLBACK
/*
 * Separate from z_thread_monitor_lock so that handlers may call
 * k_thread_foreach(). Lock order: monitor_cb_lock, then
 * z_thread_monitor_lock, so register/unregister must not be called from
 * a locked k_thread_foreach() callback.
 */
static struct k_spinlock monitor_cb_lock;
static sys_slist_t monitor_cbs = SYS_SLIST_STATIC_INIT(&monitor_cbs);

/* Set while the CPU holds monitor_cb_lock, to detect fatal error re-entry */
static bool monitor_cb_lock_held[CONFIG_MP_MAX_NUM_CPUS];

static k_spinlock_key_t monitor_cb_lock_take(void)
{
	k_spinlock_key_t key = k_spin_lock(&monitor_cb_lock);

	/* Catches register/unregister from a handler, where the lock is an IRQ lock on UP */
	__ASSERT(!monitor_cb_lock_held[_current_cpu->id],
		 "thread monitor callbacks can not be (un)registered from a handler");
	monitor_cb_lock_held[_current_cpu->id] = true;

	return key;
}

static void monitor_cb_lock_give(k_spinlock_key_t key)
{
	monitor_cb_lock_held[_current_cpu->id] = false;
	k_spin_unlock(&monitor_cb_lock, key);
}

int k_thread_monitor_cb_register(struct k_thread_monitor_cb *cb)
{
	int ret = 0;
	k_spinlock_key_t key;

	__ASSERT(cb != NULL, "cb can not be NULL");
	__ASSERT(cb->handler != NULL, "handler can not be NULL");

	key = monitor_cb_lock_take();

	if (sys_slist_find(&monitor_cbs, &cb->node, NULL)) {
		ret = -EALREADY;
	} else {
		sys_slist_append(&monitor_cbs, &cb->node);
	}

	monitor_cb_lock_give(key);

	return ret;
}

int k_thread_monitor_cb_unregister(struct k_thread_monitor_cb *cb)
{
	int ret = -EINVAL;
	k_spinlock_key_t key;

	__ASSERT(cb != NULL, "cb can not be NULL");

	key = monitor_cb_lock_take();

	if (sys_slist_find_and_remove(&monitor_cbs, &cb->node)) {
		ret = 0;
	}

	monitor_cb_lock_give(key);

	return ret;
}

void z_thread_monitor_notify(struct k_thread *thread, uint32_t event,
			     const union k_thread_monitor_event_info *info)
{
	struct k_thread_monitor_cb *cb;
	k_spinlock_key_t key;
	unsigned int irq_key = arch_irq_lock();

	if (monitor_cb_lock_held[_current_cpu->id]) {
		/*
		 * A fault while this CPU holds monitor_cb_lock, in a handler
		 * or in register/unregister, re-entered the fatal error path.
		 * Whether the interrupted code resumes once its thread is
		 * aborted is architecture specific, so the lock can neither
		 * be taken nor released safely.
		 */
		k_fatal_halt(K_ERR_KERNEL_PANIC);
	}

	key = monitor_cb_lock_take();

	SYS_SLIST_FOR_EACH_CONTAINER(&monitor_cbs, cb, node) {
		if ((cb->thread != thread) || ((cb->event_mask & event) == 0U)) {
			continue;
		}

		cb->event = event;
		if (info != NULL) {
			cb->info = *info;
		} else {
			cb->info = (union k_thread_monitor_event_info){0};
		}

		cb->handler(cb);
	}

	monitor_cb_lock_give(key);

	arch_irq_unlock(irq_key);
}
#endif /* CONFIG_THREAD_MONITOR_CALLBACK */
