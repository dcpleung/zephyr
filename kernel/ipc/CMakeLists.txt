# SPDX-License-Identifier: Apache-2.0
# Copyright (c) The Zephyr Project Contributors

target_sources_ifdef(CONFIG_MULTITHREADING kernel PRIVATE
  condvar.c
  mailbox.c
  msg_q.c
  mutex.c
  pipe.c
  queue.c
  sem.c
  stack.c
  work.c
  system_work_q.c
  )

target_sources_ifdef(CONFIG_POLL   kernel PRIVATE poll.c)
target_sources_ifdef(CONFIG_EVENTS kernel PRIVATE events.c)
