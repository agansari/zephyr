/*
 * Copyright (c) 2019,2020 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <logging/log_ctrl.h>
#include <logging/log.h>

#include "util_app_log.h"
#include "util_sformat.h"

#include "common_objects.h"

/** Declare a reference to the application logging interface. */
LOG_MODULE_DECLARE(app, CONFIG_LOG_DEFAULT_LEVEL);

#define STACK_SIZE 16384

#define END_NODE_PRIORITY 		7
#define PROVISIONING_PRIORITY	6
#define ROOT_CA_PRIORITY 		7

K_KERNEL_STACK_DEFINE(end_node_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(provisioning_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(root_ca_stack, STACK_SIZE);

K_MBOX_DEFINE(ca_root_mailbox);
K_MBOX_DEFINE(end_node_mailbox);

struct k_thread end_node;
struct k_thread provisioning;
struct k_thread root_ca;

k_tid_t end_node_tid;
k_tid_t provisioning_tid;
k_tid_t root_ca_tid;

void main(void)
{
	/* Initialise the logger subsys and dump the current buffer. */
	log_init();

	LOG_INF(">>>>> Begin main initialization <<<<<");

	/* provisioning thread is the main thread that starts the process and
	handles end-node <-> root-ca interactions. Provisioning thread is the
	equivalent to a user terminal that connects another device to a cloud.
	*/

	provisioning_tid = k_thread_create(&provisioning, provisioning_stack,
								STACK_SIZE,
								(k_thread_entry_t)provisioning_entry,
								NULL, NULL, NULL,
								K_PRIO_COOP(PROVISIONING_PRIORITY),
								0, K_NO_WAIT);

	root_ca_tid = k_thread_create(&root_ca, root_ca_stack,
								STACK_SIZE,
								(k_thread_entry_t)root_ca_entry,
								NULL, NULL, NULL,
								K_PRIO_COOP(ROOT_CA_PRIORITY),
								0, K_NO_WAIT);

	/* end-node needs to handle all the communications with the secure side. */
	end_node_tid = k_thread_create(&end_node, end_node_stack,
								STACK_SIZE,
								(k_thread_entry_t)end_node_entry,
								NULL, NULL, NULL,
								K_PRIO_COOP(END_NODE_PRIORITY),
								0, K_NO_WAIT);

	while(1){
		/* Sleep for some time and ... */
		k_msleep(CONFIG_APPLICATION_THREAD_TIMEOUT);

		/* dump any queued log messages, and wait for system events. */
		al_dump_log();
	};
	
	LOG_INF(">>>>> Done main initialization <<<<<");
}
