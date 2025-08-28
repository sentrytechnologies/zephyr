/*
 * Copyright (c) 2025 Sentry Technologies ApS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_TDK_IIM4623X_H
#define ZEPHYR_DRIVERS_SENSOR_TDK_IIM4623X_H

#include "iim4623x_reg.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sys/atomic.h>

/* Metadata used for parsing the encoded payload */
struct iim4623x_encoded_header {
	uint8_t format: 1;   /* See IIM4623X_DT_FMT_* macros */
	uint8_t accel_fs: 2; /* See IIM4623X_DT_ACCEL_FS_* macros */
	uint8_t gyro_fs: 2;  /* See IIM4623X_DT_GYRO_FS_* macros */
	uint8_t accel_bw: 2; /* See IIM4623X_DT_ACCEL_BW_* macros */
	uint8_t gyro_bw: 2;  /* See IIM4623X_DT_GYRO_BW_* macros */
	/* TODO: include custom gravity setting */
	struct {
		uint8_t delta_vel: 1;
		uint8_t delta_ang: 1;
		uint8_t temp: 1;
		uint8_t gyro: 1;
		uint8_t accel: 1;
	} chans; /* Enabled data output mask */
};

struct iim4623x_encoded_data {
	struct iim4623x_encoded_header header;
	struct iim4623x_pck_strm_payload payload;
};

struct iim4623x_config {
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec int_gpio;
	uint8_t odr_div;
};

struct iim4623x_data {
	struct {
		struct rtio_iodev *iodev;
		struct rtio *ctx;
	} rtio;
	const struct device *dev;
	struct gpio_callback int_cb;

	/* State */
	struct rtio_sqe *await_sqe;
	atomic_t busy;
	uint8_t rx_buf[IIM4623X_PACKET_LEN(72) + 10 /* margin */];
	uint8_t tx_buf[32];

	/* Encoded data instance to support fetch/get API */
	struct iim4623x_encoded_data edata;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_TDK_IIM4623X_H */
