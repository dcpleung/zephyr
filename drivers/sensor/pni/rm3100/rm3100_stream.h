/*
 * Copyright (c) 2025 Croxel Inc.
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_RM3100_STREAM_H_
#define ZEPHYR_DRIVERS_SENSOR_RM3100_STREAM_H_

#include "rm3100.h"

int rm3100_stream_init(const struct device *dev);

void rm3100_stream_submit(const struct device *dev,
			  struct rtio_iodev_sqe *iodev_sqe);

#endif /* ZEPHYR_DRIVERS_SENSOR_RM3100_STREAM_H_ */
