/*
 * Copyright (c) 2025 Sentry Technologies ApS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/dsp/utils.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_clock.h>

#include "sch16t.h"
#include "sch16t_decoder.h"

LOG_MODULE_REGISTER(SCH16T_DECODER, CONFIG_SENSOR_LOG_LEVEL);

static float sch16t_decoder_get_acc_lsb(const struct device *dev)
{
	const struct sch16t_config *cfg = dev->config;

	switch (cfg->acc_dr) {
	case 1:
		return 200;
	case 2:
		return 400;
	case 3:
		return 800;
	case 4:
		return 1600;
	default:
		return 200;
	}
}

static float sch16t_decoder_get_rate_lsb(const struct device *dev)
{
	const struct sch16t_config *cfg = dev->config;

	switch (cfg->rate_dr) {
	case 1:
		return 100;
	case 2:
		return 100;
	case 3:
		return 200;
	case 4:
		return 400;
	default:
		return 100;
	}
}

static int sch16t_decoder_get_frame_count(const uint8_t *buf, struct sensor_chan_spec chan_spec,
					  uint16_t *frame_count)
{
	if (chan_spec.chan_idx != 0) {
		return -ENOTSUP;
	}

	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_ACCEL_XYZ:
	case SENSOR_CHAN_GYRO_XYZ:
		*frame_count = 1;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int sch16t_decoder_get_size_info(struct sensor_chan_spec chan_spec, size_t *base_size,
					size_t *frame_size)
{
	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_ACCEL_XYZ:
	case SENSOR_CHAN_GYRO_XYZ:
		*base_size = sizeof(struct sensor_three_axis_data);
		*frame_size = sizeof(struct sensor_three_axis_sample_data);
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int sch16t_decoder_decode(const uint8_t *buf, struct sensor_chan_spec chan_spec,
				 uint32_t *fit, uint16_t max_count, void *data_out)
{
	struct sensor_three_axis_data *out = (struct sensor_three_axis_data *)data_out;
	struct sch16t_encoded_data *edata = (struct sch16t_encoded_data *)buf;
	const float deg_to_rad = ((float)SENSOR_PI / 1000000) / 180;

	if (*fit != 0) {
		return 0;
	}

	if (max_count == 0 || chan_spec.chan_idx != 0) {
		return -EINVAL;
	}

	out->header.base_timestamp_ns = edata->header.timestamp;
	out->header.reading_count = 1;

	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_ACCEL_XYZ:
		out->shift = 8;
		out->readings[0].x =
			Z_SHIFT_F32_TO_Q31((float)edata->acc.x / edata->header.acc_lsb, out->shift);
		out->readings[0].y =
			Z_SHIFT_F32_TO_Q31((float)edata->acc.y / edata->header.acc_lsb, out->shift);
		out->readings[0].z =
			Z_SHIFT_F32_TO_Q31((float)edata->acc.z / edata->header.acc_lsb, out->shift);

		*fit = 1;
		return 1;
	case SENSOR_CHAN_GYRO_XYZ:
		out->shift = 3;
		out->readings[0].x = Z_SHIFT_F32_TO_Q31(
			(float)edata->rate.x * deg_to_rad / edata->header.rate_lsb, out->shift);
		out->readings[0].y = Z_SHIFT_F32_TO_Q31(
			(float)edata->rate.y * deg_to_rad / edata->header.rate_lsb, out->shift);
		out->readings[0].z = Z_SHIFT_F32_TO_Q31(
			(float)edata->rate.z * deg_to_rad / edata->header.rate_lsb, out->shift);

		*fit = 1;
		return 1;
	default:
		return -EINVAL;
	}
}

static bool sch16t_decoder_has_trigger(const uint8_t *buf, enum sensor_trigger_type trigger)
{
	/* TODO: add trigger for streaming here (data ready) */

	switch (trigger) {
	default:
		return 0;
	}
}

SENSOR_DECODER_API_DT_DEFINE() = {
	.get_frame_count = sch16t_decoder_get_frame_count,
	.get_size_info = sch16t_decoder_get_size_info,
	.decode = sch16t_decoder_decode,
	.has_trigger = sch16t_decoder_has_trigger,
};

int sch16t_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder)
{
	*decoder = &SENSOR_DECODER_NAME();

	return 0;
}

int sch16t_encode(const struct device *dev, uint8_t *buf)
{
	struct sch16t_encoded_data *edata = (struct sch16t_encoded_data *)buf;
	uint64_t cycles;
	int ret;

	edata->header.acc_lsb = sch16t_decoder_get_acc_lsb(dev);
	edata->header.rate_lsb = sch16t_decoder_get_rate_lsb(dev);

	ret = sensor_clock_get_cycles(&cycles);
	if (ret) {
		LOG_ERR("sensor_clock_get_cycles() failed (ret=%d)", ret);
		return ret;
	}

	edata->header.timestamp = sensor_clock_cycles_to_ns(cycles);

	return 0;
}
