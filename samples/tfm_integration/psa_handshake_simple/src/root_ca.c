/*
 * Copyright (c) 2020 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log_ctrl.h>
#include <logging/log.h>

#include "common_objects.h"

LOG_MODULE_REGISTER(root_ca);

/* Very important, see https://github.com/microbuilder/linaroca
   This is a server implementation of the root-ca that should work.
   */

void root_ca_entry(void *dummy1, void *dummy2, void *dummy3)
{
    LOG_INF("Thread started.");

    while(1) {
        k_msleep(CONFIG_APPLICATION_THREAD_TIMEOUT);

        /* Wait for CSR binary blob from provisioning device. */
        //check some mailbox maybe

        // if (bad) {
        //     send reject provisioning
        // } else {
        //     sign the certificate
        //     register device details
        //     generate signed certificate

        //     send certificate to provider

        //     wait for 'Provisioning complete'

        //     maybe list the new certified device
        //     maybe timeout removes the device
        // }
    }


    LOG_ERR("Thread exited, should not happen;\r\n"\
        "Certificate Authority should always be on and respond "\
        "to incoming requests.");
}