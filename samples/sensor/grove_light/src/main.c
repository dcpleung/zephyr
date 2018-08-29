/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <init.h>
#include <misc/printk.h>
#include <stdio.h>
#include <sensor.h>
#include <pinmux.h>

#ifdef CONFIG_BOARD_QUARK_D2000_CRB
#define ADC_PIN 3
#endif

#define SLEEP_TIME	1000

void main(void)
{
	struct device *dev = device_get_binding(CONFIG_GROVE_LIGHT_SENSOR_NAME);

	if (dev == NULL) {
		printk("device not found.  aborting test.\n");
		return;
	}

#ifdef CONFIG_BOARD_QUARK_D2000_CRB
	struct device *pinmux = device_get_binding(CONFIG_PINMUX_NAME);

	if (pinmux_pin_set(pinmux, ADC_PIN, PINMUX_FUNC_B)) {
		printk("Fail to set pin func, %u : %u\n",
			  ADC_PIN, PINMUX_FUNC_B);
		return;
	}
#endif

	while (1) {
		int read;
		struct sensor_value lux;

		read = sensor_sample_fetch(dev);
		if (read) {
			printk("sample fetch error %d\n", read);
			continue;
		}

		sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &lux);

#ifdef CONFIG_NEWLIB_LIBC_FLOAT_PRINTF
		printk("lux: %d\n", sensor_value_to_double(&lux));
#else
		printk("lux: %d\n", lux.val1);
#endif
		k_sleep(SLEEP_TIME);
	}
}
