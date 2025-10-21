/*
 * Copyright (c) 2025 Sentry Technologies ApS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/rtio/rtio.h>
#include <zephyr/drivers/sensor.h>

#ifndef ZEPHYR_DRIVERS_SENSOR_SCH16T_STREAM_H_
#define ZEPHYR_DRIVERS_SENSOR_SCH16T_STREAM_H_

void sch16t_stream_submit(const struct device *dev);

int sch16t_stream_init(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_SENSOR_SCH16T_STREAM_H_ */
