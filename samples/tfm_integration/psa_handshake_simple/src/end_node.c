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

void send_provisioning_msg(uint32_t type, char* buffer, uint32_t receive_size)
{
    struct k_mbox_msg send_msg;

    /* prepare to send empty message */
    send_msg.info = type;
    send_msg.size = 0;
    send_msg.tx_data = NULL;
    send_msg.tx_block.data = NULL;
    send_msg.tx_target_thread = provisioning_tid;

    /* send message and wait until a consumer receives it */
    LOG_INF("send_provisioning_msg %u <<", type);
    k_mbox_put(&end_node_mailbox, &send_msg, K_FOREVER);
}

/*  input buffer[size]
    return message type */
uint32_t get_provisioning_msg(char* buffer, size_t size)
{
    struct k_mbox_msg recv_msg;

    /* prepare to receive message */
    recv_msg.size = size;
    recv_msg.rx_source_thread = provisioning_tid;

    /* get message, but not its data */
    k_mbox_get(&end_node_mailbox, &recv_msg, NULL, K_FOREVER);

    /* get message data for only certain types of messages */
    /* retrieve message data and delete the message */
    k_mbox_data_get(&recv_msg, buffer);
    LOG_INF("get_provisioning_msg %u <<", recv_msg.info);

    return recv_msg.info;
}


void end_node_entry(void *dummy1, void *dummy2, void *dummy3)
{
    msg_types info_type;
    enum tfm_status_e tfm_status;
    char buffer[100] = {99,88,77};
    LOG_INF("Thread started.");

    /* wait to get initialization commands */
    info_type = get_provisioning_msg(NULL, 0);
    if (info_type == ARE_YOU_UP) {
        /* Initialize the TFM NS interface */
        tfm_status = tfm_ns_interface_init();

        /* When done, send ACK, we are up */
        if (TFM_SUCCESS == tfm_status) {
            send_provisioning_msg(ACK, NULL, 0);
        } else {
            send_provisioning_msg(NACK, NULL, 0);
        }
    }

	/* Load app config struct from secure storage (create if missing). */
	if (cfg_load_data(&cfg)) {
		LOG_ERR("Error loading/generating app config data in SS.");
	}

	/* Get the entity attestation token (requires ~1kB stack memory!). */
	// att_test();

    info_type = get_provisioning_msg(NULL, 0);
    if (info_type == GIVE_YOURSELF_A_KEY) {
        LOG_INF("GIVE_YOURSELF_A_KEY <<");
	    /* Randomly generate the private key. */
	    uint8_t priv_key_data[32] = { 0 };
	    tfm_status = psa_generate_random(priv_key_data, sizeof(priv_key_data));
        LOG_INF("priv_key_data %x %x %x %x <<", priv_key_data[0],priv_key_data[1],priv_key_data[2],priv_key_data[3]);

        if (TFM_SUCCESS == tfm_status) {
            //this does not actually works
            tfm_status = psa_ps_set(cfg.magic, sizeof(priv_key_data),
                        (void *)priv_key_data, 0);

            if (TFM_SUCCESS == tfm_status)
            send_provisioning_msg(ACK, NULL, 0);
        } else {
            send_provisioning_msg(NACK, NULL, 0);
        }

        /* Crypto tests */
	    crp_test(priv_key_data);
    } else {
        LOG_INF("No give a key mssg? <<");
    }

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
