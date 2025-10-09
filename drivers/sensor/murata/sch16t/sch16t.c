/*
 * Copyright (c) 2025 Sentry Technologies ApS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT murata_sch16t

#include <zephyr/rtio/rtio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>

#include "sch16t.h"
#include "sch16t_reg.h"
#include "sch16t_bus.h"
#include "sch16t_submit.h"
#include "sch16t_decoder.h"

LOG_MODULE_REGISTER(SCH16T, CONFIG_SENSOR_LOG_LEVEL);

static const uint8_t sch16t_status_regs[] = {
	SCH16T_REG_STAT_SUM_SAT,     SCH16T_REG_STAT_COM,    SCH16T_REG_STAT_RATE_COM,
	SCH16T_REG_STAT_RATE_X,      SCH16T_REG_STAT_RATE_Y, SCH16T_REG_STAT_RATE_Z,
	SCH16T_REG_STAT_ACC_X,       SCH16T_REG_STAT_ACC_Y,  SCH16T_REG_STAT_ACC_Z,
	SCH16T_REG_STAT_SYNC_ACTIVE, SCH16T_REG_STAT_INFO,   SCH16T_REG_STAT_SUM,
};

static DEVICE_API(sensor, sch16t_driver_api) = {
	.submit = sch16t_submit,
	.get_decoder = sch16t_get_decoder,
};

static int sch16t_start_up(const struct device *dev)
{
	uint16_t read_vals[ARRAY_SIZE(sch16t_status_regs)];
	const struct sch16t_config *cfg = dev->config;
	uint16_t write_val;
	int ret;

	/* start up sequence as described in chapter 5.2 of the datasheet */

	/* TODO: power up regulator at this stage */

	k_msleep(SCH16T_BOOT_TIME_MS);

	ret = gpio_pin_set_dt(&cfg->reset_gpio, 0);
	if (ret) {
		LOG_ERR("failed getting the device out of reset (ret=%d)", ret);
		return -EIO;
	}

	k_msleep(SCH16T_INIT_TIME_MS);

	/* configuring non-default settings */
	write_val = FIELD_PREP(GENMASK(14, 12), cfg->acc_dr) |
		    FIELD_PREP(GENMASK(11, 9), cfg->acc_dr) |
		    FIELD_PREP(GENMASK(8, 6), cfg->decimation) |
		    FIELD_PREP(GENMASK(5, 3), cfg->decimation) |
		    FIELD_PREP(GENMASK(2, 0), cfg->decimation);

	ret = sch16t_bus_single_write_submit_and_wait(dev, SCH16T_REG_CTRL_ACC12, write_val, NULL);
	if (ret) {
		return ret;
	}

	write_val = FIELD_PREP(GENMASK(14, 12), cfg->rate_dr) |
		    FIELD_PREP(GENMASK(11, 9), cfg->rate_dr) |
		    FIELD_PREP(GENMASK(8, 6), cfg->decimation) |
		    FIELD_PREP(GENMASK(5, 3), cfg->decimation) |
		    FIELD_PREP(GENMASK(2, 0), cfg->decimation);

	ret = sch16t_bus_single_write_submit_and_wait(dev, SCH16T_REG_CTRL_RATE, write_val, NULL);
	if (ret) {
		return ret;
	}

	/* enable interrupt pin */
	write_val = 0x200C | BIT(5);
	ret = sch16t_bus_single_write_submit_and_wait(dev, SCH16T_REG_CTRL_USER_IF, write_val,
						      NULL);
	if (ret) {
		return ret;
	}

	ret = sch16t_bus_single_write_submit_and_wait(dev, SCH16T_REG_CTRL_MODE,
						      SCH16T_MODE_EN_SENSOR, NULL);
	if (ret) {
		return ret;
	}

	k_msleep(SCH16T_ENABLE_TIME_MS);

	ret = sch16t_bus_bulk_read_submit_and_wait(dev, ARRAY_SIZE(sch16t_status_regs),
						   sch16t_status_regs, read_vals, NULL);
	if (ret) {
		return ret;
	}

	ret = sch16t_bus_single_write_submit_and_wait(
		dev, SCH16T_REG_CTRL_MODE, SCH16T_MODE_EN_SENSOR | SCH16T_MODE_EOI_CTRL, NULL);
	if (ret) {
		return ret;
	}

	/* yes, we have to read the status registers twice... */
	ret = sch16t_bus_bulk_read_submit_and_wait(dev, ARRAY_SIZE(sch16t_status_regs),
						   sch16t_status_regs, read_vals, NULL);
	if (ret) {
		return ret;
	}
	ret = sch16t_bus_bulk_read_submit_and_wait(dev, ARRAY_SIZE(sch16t_status_regs),
						   sch16t_status_regs, read_vals, NULL);
	if (ret) {
		return ret;
	}

	if (read_vals[ARRAY_SIZE(sch16t_status_regs) - 1] != 0xFFFF) {
		LOG_ERR("STATUS_SUM invalid: expected 0xFFFF, got 0x%04x (ret=%d)",
			read_vals[ARRAY_SIZE(sch16t_status_regs) - 1], -EIO);
		return -EIO;
	}

	return 0;
}

static int sch16t_init(const struct device *dev)
{
	const struct sch16t_config *cfg = dev->config;
	int ret;

	if (!cfg->reset_gpio.port) {
		LOG_ERR("reset GPIO not supplied (ret=%d)", -ENODEV);
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
		LOG_ERR("reset GPIO not ready (ret=%d)", -ENODEV);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		LOG_ERR("failed to configure reset GPIO (ret=%d)", ret);
		return -EIO;
	}

	ret = sch16t_start_up(dev);
	if (ret) {
		return ret;
	}

	return 0;
}

#define SCH16T_INIT(inst)                                                                          \
                                                                                                   \
	BUILD_ASSERT(DT_INST_PROP(inst, decimation) >= 0 && DT_INST_PROP(inst, decimation) <= 4,   \
		     "decimation must be between 0 and 4");                                        \
                                                                                                   \
	BUILD_ASSERT(DT_INST_PROP(inst, acc_dynamic_range) >= 1 &&                                 \
			     DT_INST_PROP(inst, acc_dynamic_range) <= 4,                           \
		     "acc-dynamic-range must be between 1 and 4");                                 \
                                                                                                   \
	BUILD_ASSERT(DT_INST_PROP(inst, rate_dynamic_range) >= 1 &&                                \
			     DT_INST_PROP(inst, rate_dynamic_range) <= 4,                          \
		     "rate-dynamic-range must be between 1 and 4");                                \
                                                                                                   \
	RTIO_DEFINE(sch16t_rtio_ctx_##inst, SCH16T_QUEUE_SIZE, SCH16T_QUEUE_SIZE);                 \
                                                                                                   \
	SPI_DT_IODEV_DEFINE(sch16t_bus_##inst, DT_DRV_INST(inst),                                  \
			    SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB);              \
                                                                                                   \
	static const struct sch16t_config sch16t_cfg_##inst = {                                    \
		.decimation = (uint8_t)DT_INST_PROP(inst, decimation),                             \
		.acc_dr = (uint8_t)DT_INST_PROP(inst, acc_dynamic_range),                          \
		.rate_dr = (uint8_t)DT_INST_PROP(inst, rate_dynamic_range),                        \
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),                        \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                    \
	};                                                                                         \
                                                                                                   \
	static struct sch16t_data sch16t_data_##inst = {                                           \
		.rtio =                                                                            \
			{                                                                          \
				.iodev = &sch16t_bus_##inst,                                       \
				.ctx = &sch16t_rtio_ctx_##inst,                                    \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, sch16t_init, NULL, &sch16t_data_##inst,                 \
				     &sch16t_cfg_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
				     &sch16t_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCH16T_INIT)
