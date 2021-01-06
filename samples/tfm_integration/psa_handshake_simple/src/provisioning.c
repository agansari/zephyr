/*
 * Copyright (c) 2020 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log_ctrl.h>
#include <logging/log.h>

#include "common_objects.h"

LOG_MODULE_REGISTER(provisioning);

void send_end_node(uint32_t type, char* buffer, size_t size)
{
    struct k_mbox_msg send_msg;

    /* prepare to send empty message */
    send_msg.info = type;
    send_msg.size = 0;
    send_msg.tx_data = NULL;
    send_msg.tx_block.data = NULL;
    send_msg.tx_target_thread = end_node_tid;

    /* send message and wait until a consumer receives it */
    LOG_INF("send_end_node %u <<", type);
    k_mbox_put(&end_node_mailbox, &send_msg, K_FOREVER);
}

/*  input buffer[size]
    return message type */
uint32_t get_end_node(char* buffer, size_t size)
{
    struct k_mbox_msg recv_msg;

    /* prepare to receive message */
    recv_msg.size = size;
    recv_msg.rx_source_thread = end_node_tid;

    /* get message, but not its data */
    k_mbox_get(&end_node_mailbox, &recv_msg, NULL, K_FOREVER);

    /* get message data for only certain types of messages */
    /* retrieve message data and delete the message */
    k_mbox_data_get(&recv_msg, buffer);
    LOG_INF("get_end_node %u <<", recv_msg.info);

    return recv_msg.info;
}

void provisioning_entry(void *p1, void *p2, void *p3)
{
    msg_types info_type;
    char buffer[100]={66,55,44,22,100};
    LOG_INF("Thread started.");

    /* Connect to end-node. */
    /* Is end-node thread up? */
    send_end_node(ARE_YOU_UP, NULL, 0);
    info_type = get_end_node(NULL, 0);
    if (info_type == ACK) {
        LOG_INF("end_node thread is up.");
    } else {
        LOG_INF("Does not work.");
    }

    /* Send 'Provision Device' command to end-node. */
    send_end_node(GIVE_YOURSELF_A_KEY, NULL, 0);
    info_type = get_end_node(NULL, 0);
    if (info_type == ACK) {
        LOG_INF("end_node generated a private key.");
    } else {
        LOG_INF("end_node can't generate a private key.");
    }

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