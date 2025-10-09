/*
 * Copyright (c) 2025 Sentry Technologies ApS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_SCH16T_SUBMIT_H_
#define ZEPHYR_DRIVERS_SENSOR_SCH16T_SUBMIT_H_

#include <zephyr/device.h>
#include <zephyr/rtio/rtio.h>

void sch16t_submit_single(const struct device *dev);

void sch16t_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe);

#endif // ZEPHYR_DRIVERS_SENSOR_SCH16T_SUBMIT_H_
