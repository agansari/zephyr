/*
 * Copyright (c) 2019,2020 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <logging/log_ctrl.h>
#include <logging/log.h>

#include "tfm_ns_interface.h"
#include "psa_attestation.h"
#include "psa_crypto.h"
#include "util_app_cfg.h"
#include "util_app_log.h"
#include "util_sformat.h"

/** Declare a reference to the application logging interface. */
LOG_MODULE_DECLARE(app, CONFIG_LOG_DEFAULT_LEVEL);

#define STACK_SIZE 4000
#define END_NODE_PRIORITY 		5
#define PROVISIONING_PRIORITY	6
#define ROOT_CA_PRIORITY 		7

extern void end_node_entry(void *, void *, void *);
extern void provisioning_entry(void *, void *, void *);
extern void root_ca_entry(void *, void *, void *);

K_THREAD_DEFINE(end_node_tid, STACK_SIZE,
                end_node_entry, NULL, NULL, NULL,
                END_NODE_PRIORITY, 0, 0);
K_THREAD_DEFINE(provisioning_tid, STACK_SIZE,
                provisioning_entry, NULL, NULL, NULL,
                PROVISIONING_PRIORITY, 0, 0);
K_THREAD_DEFINE(root_ca_tid, STACK_SIZE,
				root_ca_entry, NULL, NULL, NULL,
                ROOT_CA_PRIORITY, 0, 0);


/* Create an instance of the system config struct for the application. */
static struct cfg_data cfg;

void main(void)
{
	/* Initialise the logger subsys and dump the current buffer. */
	log_init();

	/* Load app config struct from secure storage (create if missing). */
	if (cfg_load_data(&cfg)) {
		LOG_ERR("Error loading/generating app config data in SS.");
	}

	/* Get the entity attestation token (requires ~1kB stack memory!). */
	att_test();

	/* Crypto tests */
	crp_test();

	/* Dump any queued log messages, and wait for system events. */
	al_dump_log();
}
