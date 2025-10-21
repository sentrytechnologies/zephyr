/*
 * Copyright (c) 2025 Sentry Technologies ApS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/rtio/rtio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include "sch16t.h"
#include "sch16t_stream.h"
#include "sch16t_submit.h"

LOG_MODULE_REGISTER(SCH16T_STREAM, CONFIG_SENSOR_LOG_LEVEL);

static void sch16t_gpio_callback(const struct device *gpio_dev, struct gpio_callback *cb,
				 uint32_t pins)
{
	struct sch16t_stream *stream = CONTAINER_OF(cb, struct sch16t_stream, cb);
	const struct device *dev = stream->dev;
	const struct sch16t_config *cfg = dev->config;
	int ret;

	ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_MODE_DISABLED);
	if (ret) {
		LOG_ERR("failed to disable interrupt (ret=%d)", ret);
		return;
	}

	sch16t_submit_single(dev);
}

void sch16t_stream_submit(const struct device *dev)
{
	const struct sch16t_config *cfg = dev->config;
	struct sch16t_data *data = dev->data;
	struct rtio_iodev_sqe *iodev_sqe = data->rtio.iodev_sqe;
	int ret;

	ret = gpio_pin_get_dt(&cfg->int_gpio);
	if (ret == 1) {
		sch16t_submit_single(dev);
		return;
	} else if (ret < 0) {
		LOG_ERR("failed to get pin value (ret=%d)", ret);
		rtio_iodev_sqe_err(iodev_sqe, ret);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		LOG_ERR("failed to enable interrupt (ret=%d)", ret);
		rtio_iodev_sqe_err(iodev_sqe, ret);
		return;
	}
}

int sch16t_stream_init(const struct device *dev)
{
	const struct sch16t_config *cfg = dev->config;
	struct sch16t_data *data = dev->data;
	int ret;

	data->stream.dev = dev;

	if (!cfg->int_gpio.port) {
		LOG_ERR("interrupt GPIO not supplied (ret=%d)", -ENODEV);
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cfg->int_gpio)) {
		LOG_ERR("interrupt GPIO not ready (ret=%d)", -ENODEV);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
	if (ret) {
		LOG_ERR("failed to configure interrupt GPIO (ret=%d)", ret);
		return -EIO;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_MODE_DISABLED);
	if (ret) {
		LOG_ERR("failed to configure interrupt as disabled (ret=%d)", ret);
		return -EIO;
	}

	gpio_init_callback(&data->stream.cb, sch16t_gpio_callback, BIT(cfg->int_gpio.pin));

	ret = gpio_add_callback(cfg->int_gpio.port, &data->stream.cb);
	if (ret) {
		LOG_ERR("failed to add interrupt callback (ret=%d)", ret);
		return -EIO;
	}

	return 0;
}
