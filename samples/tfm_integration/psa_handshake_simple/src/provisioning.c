/*
 * Copyright (c) 2020 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log_ctrl.h>
#include <logging/log.h>

#include "common_objects.h"

LOG_MODULE_REGISTER(provisioning);

void send_end_node()
{
    struct k_mbox_msg send_msg;

    /* generate random value to send */
    uint32_t random_value = 13;

    /* prepare to send empty message */
    send_msg.info = random_value;
    send_msg.size = 0;
    send_msg.tx_data = NULL;
    send_msg.tx_block.data = NULL;
    send_msg.tx_target_thread = end_node_tid;

    /* send message and wait until a consumer receives it */
    k_mbox_put(&end_node_mailbox, &send_msg, K_FOREVER);
    LOG_INF("send_end_node");
}

void get_end_node()
{
    struct k_mbox_msg recv_msg;
    char buffer[10000];

    /* prepare to receive message */
    recv_msg.size = 10000;
    recv_msg.rx_source_thread = end_node_tid;

    /* get message, but not its data */
    k_mbox_get(&end_node_mailbox, &recv_msg, NULL, K_FOREVER);

    /* get message data for only certain types of messages */
    /* retrieve message data and delete the message */
    k_mbox_data_get(&recv_msg, buffer);
    LOG_INF("get_end_node");
}

void provisioning_entry(void *p1, void *p2, void *p3)
{
    LOG_INF("Thread started.");

    /* Connect to end-node. */
    /* Is end-node thread up? */
    //message and wait for ACK
    send_end_node();
    get_end_node();

    /* Send 'Provision Device' command to end-node. */
    //message starting provisioning
    //wait for CSR binary blob

    /* Send CSR binary to root-ca. */
    //copy and send copy CSR to root-ca thread
    //wait for rejection or signed certificate

    // if (rejection) {}
    //     send 'abort provision'
    // } else {

    //     receive certificare chain binary blob
    //     copy and send copy to end-node

    //     receive 'provisioning complete' from end-node
    //     send 'provisioning complete' to root-ca
    // }

    // reset end-node

    // check the certificate in end-node with root-ca

    k_msleep(CONFIG_APPLICATION_THREAD_TIMEOUT);

    LOG_INF("Thread exited, end node was reset.");
}