/*
 * Copyright (c) 2018, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>

void main(void)
{
	printk("Hello World from Non-Secure! %s\n", CONFIG_ARCH);

	while (1) {
	};
}

