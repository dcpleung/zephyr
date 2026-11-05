.. zephyr:code-sample:: thread_monitor
   :name: Thread monitor
   :relevant-api: thread_apis

   Restart faulting and essential threads using thread monitor callbacks.

Overview
********

This sample shows how the callbacks enabled by
:kconfig:option:`CONFIG_THREAD_MONITOR_CALLBACK` can be used to supervise
threads and restart them when they end unexpectedly. It starts two threads:

* ``faulty`` does a few steps of work and then raises a fatal error with
  :c:macro:`k_oops`;
* ``essential`` is created with :c:macro:`K_ESSENTIAL`, does a few steps of
  work and then returns from its entry function.

Each thread has its own thread monitor callback for
:c:macro:`K_THREAD_MONITOR_EVENT_FATAL_ERROR` and
:c:macro:`K_THREAD_MONITOR_EVENT_ESSENTIAL_EXIT`, bound to that thread with
:c:func:`k_thread_monitor_cb_init`, and its own delayable work item, which the
callback reaches through its private data. The handler runs with interrupts
locked in the context of the ending thread, so it may neither block nor create
threads; it only schedules the work item with :c:func:`k_work_schedule`. After
a short delay, which keeps a thread that keeps failing from monopolizing the
CPU, the work item joins the ended thread, reads the event the kernel stored in
the callback, and creates the thread again from the same thread object and
stack, up to three times per thread. Since a callback is bound to a single
thread, which cannot end again before the work item restarts it, the stored
event is not overwritten before it is read.

The callbacks only report events; whether the system survives them is decided
by :c:func:`k_sys_fatal_error_handler`. The default handler halts the system,
so the sample provides one that returns for a fault in a supervised thread that
is still running, letting the kernel abort the thread, and halts for any other
thread.

An essential thread ending is followed by a kernel panic, raised on that
thread. The sample's fatal error handler returns once for that panic too, which
the kernel only tolerates with :kconfig:option:`CONFIG_ASSERT` disabled. Any
other fatal error raised after a supervised thread was reported as ending halts
the system. Recovering from that panic depends on the architecture: SPARC
cannot resume from it, and on ARM64 scheduling breaks on a later essential
thread exit, so the sample halts there and :file:`tests.yaml` excludes these. Restarting an essential thread is shown here for demonstration;
a real system typically resets instead.

Building and Running
********************

This application can be built and executed on QEMU as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/kernel/thread_monitor
   :host-os: unix
   :board: qemu_cortex_m3
   :goals: run
   :compact:

To build for another board, change ``qemu_cortex_m3`` above to that board's
name.

Sample Output
=============

The kernel prints its usual fatal error report each time a thread ends; it is
abbreviated below.

.. code-block:: console

   faulty: run 0, step 1
   essential: run 0, step 1
   faulty: run 0, step 2
   essential: run 0, step 2
   faulty: run 0, step 3
   essential: run 0, step 3
   >>> ZEPHYR FATAL ERROR 3: Kernel oops on CPU 0
   faulty: fatal error 3, restart 1/3
   faulty: run 1, step 1
   >>> ZEPHYR FATAL ERROR 4: Kernel panic on CPU 0
   faulty: run 1, step 2
   essential: exited, restart 1/3
   essential: run 1, step 1
   faulty: run 1, step 3
   ...
   essential: run 3, step 3
   essential: exited, giving up
   All supervised threads stopped after 3 restarts each
