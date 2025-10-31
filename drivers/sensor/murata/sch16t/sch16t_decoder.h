/*
 * Copyright (c) 2025 Sentry Technologies ApS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_SCH16T_DECODER_H_
#define ZEPHYR_DRIVERS_SENSOR_SCH16T_DECODER_H_

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

int sch16t_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder);

int sch16t_encode(const struct device *dev, uint8_t *buf);

#endif /* ZEPHYR_DRIVERS_SENSOR_SCH16T_DECODER_H_ */
