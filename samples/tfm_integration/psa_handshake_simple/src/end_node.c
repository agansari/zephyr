/*
 * Copyright (c) 2020 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log_ctrl.h>
#include <logging/log.h>

#include "util_app_log.h"
#include "util_sformat.h"

#include "tfm_ns_interface.h"
#include "psa_attestation.h"
#include "psa_crypto.h"
#include "util_app_cfg.h"

#include "common_objects.h"

LOG_MODULE_REGISTER(end_node);

/* Create an instance of the system config struct for the application. */
static struct cfg_data cfg;

void send_provisioning_msg()
{
    struct k_mbox_msg send_msg;

    /* generate random value to send */
    uint32_t random_value = 27;

    /* prepare to send empty message */
    send_msg.info = random_value;
    send_msg.size = 0;
    send_msg.tx_data = NULL;
    send_msg.tx_block.data = NULL;
    send_msg.tx_target_thread = provisioning_tid;

    /* send message and wait until a consumer receives it */
    k_mbox_put(&end_node_mailbox, &send_msg, K_FOREVER);
    LOG_INF("send_provisioning_msg");
}

void get_provisioning_msg()
{
    struct k_mbox_msg recv_msg;
    char buffer[10000];

    /* prepare to receive message */
    recv_msg.size = 10000;
    recv_msg.rx_source_thread = provisioning_tid;

    /* get message, but not its data */
    k_mbox_get(&end_node_mailbox, &recv_msg, NULL, K_FOREVER);

    /* get message data for only certain types of messages */
    /* retrieve message data and delete the message */
    k_mbox_data_get(&recv_msg, buffer);
    LOG_INF("get_provisioning_msg");
}


void end_node_entry(void *dummy1, void *dummy2, void *dummy3)
{
    LOG_INF("Thread started.");

    // wait to get initialization commands
    get_provisioning_msg();
    send_provisioning_msg();//Send ACK

	/* Initialize the TFM NS interface */
	tfm_ns_interface_init();

	/* Load app config struct from secure storage (create if missing). */
	if (cfg_load_data(&cfg)) {
		LOG_ERR("Error loading/generating app config data in SS.");
	}

	/* Get the entity attestation token (requires ~1kB stack memory!). */
	att_test();

	/* Crypto tests */
	crp_test();

    while (1){

        // if ('Provision device' is received) {
        //     access key
        //     unique device identifier
        //     add public key/unique id to CSR binary

        //     send CSR binary

        //     wait for abort or go
        //     if ('Abort Provisioning') {
        //         cleanup
        //     } else {
        //         get certificate chain
        //         store certificate chain
        //         send 'Provisioning complete'
        //     }
        // } else {
            k_msleep(CONFIG_APPLICATION_THREAD_TIMEOUT);
        // }

        //maybe exit in case of reset and provider launch thread.
    }

    LOG_INF("Thread exited, provisioning is complete.");
}
