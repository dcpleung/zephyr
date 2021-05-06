/* SPDX-License-Identifier: Apache-2.0 */

#include <assert.h>
#include <kernel.h>
#include <string.h>
#include <app_memory/app_memdomain.h>
#include <sys/mem_manage.h>
#include <sys/util.h>
#include <random/rand32.h>

#define STACKSIZE 1024
#define NUM_GOODS 16

K_SEM_DEFINE(sem_producer, 0, 1);
K_SEM_DEFINE(sem_consumer, 0, 1);
K_SEM_DEFINE(sem_transport, 0, 1);

struct k_mem_partition producer_part;
struct k_mem_domain producer_domain;
struct k_mem_partition *producer_domain_parts[] = {&producer_part};
struct k_thread producer_thread;
K_THREAD_STACK_DEFINE(producer_stack, STACKSIZE);

struct k_mem_partition consumer_part;
struct k_mem_domain consumer_domain;
struct k_mem_partition *consumer_domain_parts[] = {&consumer_part};
struct k_thread consumer_thread;
K_THREAD_STACK_DEFINE(consumer_stack, STACKSIZE);

struct k_mem_domain transport_domain;
struct k_mem_partition *transport_domain_parts[] = {&producer_part, &consumer_part};
struct k_thread transport_thread;
K_THREAD_STACK_DEFINE(transport_stack, STACKSIZE);

struct package {
	unsigned char goods[NUM_GOODS];
};

void print_goods(char *prefix, struct package *pkg)
{
	printk("%s:", prefix);

	for (int i = 0; i < NUM_GOODS; i++) {
		printk(" 0x%02x", pkg->goods[i]);
	}

	printk("\n");
}

void producer_entry(void *p1, void *p2, void *p3)
{
	struct package *ppkg = (struct package *)p1;
	struct package *cpkg = (struct package *)p2;

	ARG_UNUSED(cpkg);

	while (true) {
		/* Make sure we have space to store produced goods */
		k_sem_take(&sem_producer, K_FOREVER);

		/* Start working */
		sys_rand_get(ppkg->goods, NUM_GOODS);

		/* Uncomment to cause faults with this one */
		/* sys_rand_get(cpkg->goods, NUM_GOODS); */

		/* Fake that it takes some times to produce */
		k_msleep(10);

		print_goods("P", ppkg);

		/* Tell transport to move goods */
		k_sem_give(&sem_transport);
	}
}

void consumer_entry(void *p1, void *p2, void *p3)
{
	struct package *ppkg = (struct package *)p1;
	struct package *cpkg = (struct package *)p2;

	ARG_UNUSED(ppkg);

	while (true) {
		/* Goods arrived via transport */
		k_sem_take(&sem_consumer, K_FOREVER);

		/* Consume */
		print_goods("C", cpkg);

		/* Uncomment to cause faults with this one */
		/* print_goods("-", ppkg); */
	}
}

void transport_entry(void *p1, void *p2, void *p3)
{
	struct package *ppkg = (struct package *)p1;
	struct package *cpkg = (struct package *)p2;

	while (true) {
		/* Produce signals we can transport */
		k_sem_take(&sem_transport, K_FOREVER);

		/* Fake that it takes some times to transport */
		k_msleep(100);

		/* Transport from producer to consumer */
		memcpy(cpkg->goods, ppkg->goods, NUM_GOODS);

		/* Tell consumer goods are there */
		k_sem_give(&sem_consumer);

		/* Producer can produce again */
		k_sem_give(&sem_producer);
	}
}

void main(void)
{
	void *producer_mem, *consumer_mem;
	k_tid_t tProducer, tConsumer, tTransport;

	/* Allocate memory for producer and consumer, and
	 * setup memory domains.
	 */
	producer_mem = k_mem_map(CONFIG_MMU_PAGE_SIZE, K_MEM_PERM_RW);
	__ASSERT_NO_MSG(producer_mem != NULL);

	producer_part.start = POINTER_TO_UINT(producer_mem);
	producer_part.size = CONFIG_MMU_PAGE_SIZE;
	producer_part.attr = K_MEM_PARTITION_P_RW_U_RW;

	k_mem_domain_init(&producer_domain,
			  ARRAY_SIZE(producer_domain_parts),
			  producer_domain_parts);

	consumer_mem = k_mem_map(CONFIG_MMU_PAGE_SIZE, K_MEM_PERM_RW);
	__ASSERT_NO_MSG(consumer_mem != NULL);

	consumer_part.start = POINTER_TO_UINT(consumer_mem);
	consumer_part.size = CONFIG_MMU_PAGE_SIZE;
	consumer_part.attr = K_MEM_PARTITION_P_RW_U_RW;

	k_mem_domain_init(&consumer_domain,
			  ARRAY_SIZE(consumer_domain_parts),
			  consumer_domain_parts);

	k_mem_domain_init(&transport_domain,
			  ARRAY_SIZE(transport_domain_parts),
			  transport_domain_parts);

	/* Setup Producer Thread */
	tProducer = k_thread_create(&producer_thread, producer_stack,
				    STACKSIZE, producer_entry,
				    producer_mem, consumer_mem, NULL, -1,
				    K_USER, K_FOREVER);
	k_mem_domain_add_thread(&producer_domain, tProducer);
	k_thread_access_grant(tProducer, &sem_producer, &sem_transport);

	/* Setup Consumer */
	tConsumer = k_thread_create(&consumer_thread, consumer_stack,
				    STACKSIZE, consumer_entry,
				    producer_mem, consumer_mem, NULL, -1,
				    K_USER, K_FOREVER);
	k_mem_domain_add_thread(&consumer_domain, tConsumer);
	k_thread_access_grant(tConsumer, &sem_consumer);

	/* Setup Transport */
	tTransport = k_thread_create(&transport_thread, transport_stack,
				     STACKSIZE, transport_entry,
				     producer_mem, consumer_mem, NULL, -1,
				     K_USER, K_FOREVER);
	k_mem_domain_add_thread(&transport_domain, tTransport);
	k_thread_access_grant(tTransport, &sem_producer,
			      &sem_consumer, &sem_transport);

	k_thread_access_grant(k_current_get(), &sem_producer, &sem_consumer);

	k_thread_start(tProducer);
	k_thread_start(tConsumer);
	k_thread_start(tTransport);

	k_sem_give(&sem_producer);
}
