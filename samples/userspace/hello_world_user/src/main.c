/*
 * Copyright (c) 2020 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdio.h>

#include <app_memory/app_memdomain.h>
#include <app_memory/mem_domain.h>

#define USER_STACKSIZE	2048

K_APPMEM_PARTITION_DEFINE(extra_obj_files_part);

extern uint32_t test_bss;
extern uint32_t test_data;

struct k_thread user_thread;
K_THREAD_STACK_DEFINE(user_stack, USER_STACKSIZE);

struct k_mem_domain test_mem_domain;

static void user_function(void *p1, void *p2, void *p3)
{
	printf("Hello World from UserSpace! %s\n", CONFIG_BOARD);

	printk("UT: bss 0x%x, data 0x%x\n", test_bss, test_data);

	test_bss = 0xFF;
	test_data = 0x87654321;

	printk("UT: bss 0x%x, data 0x%x\n", test_bss, test_data);
}


void main(void)
{
	k_tid_t tid;

	printk("ST: bss 0x%x, data 0x%x\n", test_bss, test_data);

	k_mem_domain_init(&test_mem_domain, 0, NULL);
	k_mem_domain_add_partition(&test_mem_domain, &extra_obj_files_part);

	tid = k_thread_create(&user_thread, user_stack, USER_STACKSIZE,
			user_function, NULL, NULL, NULL,
			-1, K_USER, K_FOREVER);

	k_mem_domain_add_thread(&test_mem_domain, tid);

	k_thread_start(tid);

	k_msleep(100);

	printk("ST: bss 0x%x, data 0x%x\n", test_bss, test_data);
}
