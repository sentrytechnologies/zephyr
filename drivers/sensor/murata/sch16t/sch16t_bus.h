/*
 * Copyright (c) 2025 Sentry Technologies ApS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_SCH16T_BUS_H_
#define ZEPHYR_DRIVERS_SENSOR_SCH16T_BUS_H_

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/rtio/rtio.h>

int sch16t_bus_sched_bulk_write(const struct device *dev, const int len, const uint8_t *regs,
				const uint16_t *write_vals, rtio_callback_t end_cb);

int sch16t_bus_sched_bulk_read(const struct device *dev, const int len, const uint8_t *regs,
			       uint16_t *read_vals, rtio_callback_t end_cb);

int sch16t_bus_submit_and_wait(const struct device *dev);

static inline int sch16t_bus_sched_single_write(const struct device *dev, const uint8_t reg,
						const uint16_t write_val, rtio_callback_t end_cb)
{
	return sch16t_bus_sched_bulk_write(dev, 1, &reg, &write_val, end_cb);
}

static inline int sch16t_bus_sched_single_read(const struct device *dev, const uint8_t reg,
					       uint16_t *read_val, rtio_callback_t end_cb)
{
	return sch16t_bus_sched_bulk_read(dev, 1, &reg, read_val, end_cb);
}

static inline int sch16t_bus_bulk_read_submit_and_wait(const struct device *dev, const int len,
						       const uint8_t *regs, uint16_t *read_vals,
						       rtio_callback_t end_cb)
{
	int ret;

	ret = sch16t_bus_sched_bulk_read(dev, len, regs, read_vals, end_cb);
	if (ret) {
		return ret;
	}

	return sch16t_bus_submit_and_wait(dev);
}

static inline int sch16t_bus_single_write_submit_and_wait(const struct device *dev,
							  const uint8_t reg,
							  const uint16_t write_val,
							  rtio_callback_t end_cb)
{
	int ret;

	ret = sch16t_bus_sched_bulk_write(dev, 1, &reg, &write_val, end_cb);
	if (ret) {
		return ret;
	}

	return sch16t_bus_submit_and_wait(dev);
}

#endif /* ZEPHYR_DRIVERS_SENSOR_SCH16T_BUS_H_ */
