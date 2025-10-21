/*
 * Copyright (c) 2025 Sentry Technologies ApS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_SCH16T_H_
#define ZEPHYR_DRIVERS_SENSOR_SCH16T_H_

#define DT_DRV_COMPAT murata_sch16t

#include <zephyr/types.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/drivers/gpio.h>

#define SCH16T_QUEUE_SIZE 32

struct sch16t_encoded_data {
	struct {
		float acc_lsb;
		float rate_lsb;
		uint64_t timestamp;
	} header;
	union {
		uint8_t buf[12];
		struct {
			struct {
				int16_t x;
				int16_t y;
				int16_t z;
			} acc;
			struct {
				int16_t x;
				int16_t y;
				int16_t z;
			} rate;
		} __packed;
	};
};

struct sch16t_stream {
	struct gpio_callback cb;
	const struct device *dev;
};

struct sch16t_rtio_payloads {
	int len;
	uint32_t in[SCH16T_QUEUE_SIZE];
	uint32_t out[SCH16T_QUEUE_SIZE];
};

struct sch16t_data {
	struct {
		struct rtio *ctx;
		struct rtio_iodev *iodev;
		struct rtio_iodev_sqe *iodev_sqe;
		struct sch16t_rtio_payloads payloads;
	} rtio;
#if defined(CONFIG_SCH16T_STREAM)
	struct sch16t_stream stream;
#endif /* CONFIG_SCH16T_STREAM */
};

struct sch16t_config {
	uint8_t acc_dr;
	uint8_t rate_dr;
	uint8_t decimation;
	struct gpio_dt_spec int_gpio;
	struct gpio_dt_spec reset_gpio;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_SCH16T_H_ */
