/*
 * Copyright (c) 2025 Sentry Technologies ApS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "iim4623x.h"
#include "iim4623x_reg.h"
#include "iim4623x_decoder.h"

#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
LOG_MODULE_REGISTER(iim4623x_decoder, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT invensense_iim4623x

static union iim4623x_encoded_channels iim4623x_encode_channel(enum sensor_channel chan)
{
	union iim4623x_encoded_channels enc_chan = {.msk = 0};

	switch (chan) {
	case SENSOR_CHAN_DIE_TEMP:
		enc_chan.temp = 1;
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
		enc_chan.accel = 1;
		break;
	case SENSOR_CHAN_GYRO_XYZ:
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
		enc_chan.gyro = 1;
		break;
	default:
		break;
	};

	return enc_chan;
}

static int iim4623x_decoder_get_frame_count(const uint8_t *buffer,
					    struct sensor_chan_spec chan_spec,
					    uint16_t *frame_count)
{
	struct iim4623x_encoded_data *edata = (struct iim4623x_encoded_data *)buffer;
	union iim4623x_encoded_channels chan_req = iim4623x_encode_channel(chan_spec.chan_type);

	/* TODO: why did icm45686 do this? */
	if (chan_spec.chan_idx != 0) {
		return -ENOTSUP;
	}

	if (!(edata->header.chans.msk & chan_req.msk)) {
		return -ENODATA;
	}

	if (!edata->payload.status.data_ready) {
		return -ENODATA;
	}

	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_DIE_TEMP:
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
	case SENSOR_CHAN_GYRO_XYZ:
		*frame_count = 1;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int iim4623x_decoder_get_size_info(struct sensor_chan_spec chan_spec, size_t *base_size,
					  size_t *frame_size)
{
	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_DIE_TEMP:
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
		*base_size = sizeof(struct sensor_q31_data);
		*frame_size = sizeof(struct sensor_q31_sample_data);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
	case SENSOR_CHAN_GYRO_XYZ:
		*base_size = sizeof(struct sensor_three_axis_data);
		*frame_size = sizeof(struct sensor_three_axis_sample_data);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int iim4623x_decode_chan(const struct iim4623x_encoded_data *edata, enum sensor_channel chan,
				struct sensor_q31_data *out)
{
	struct sensor_value *val = (void *)&out->readings[0].value;
	struct sensor_three_axis_sample_data *tri_axis = (void *)&out->readings[0];

	switch (chan) {
	case SENSOR_CHAN_DIE_TEMP:
		sensor_value_from_float(val, edata->payload.temp.val);
		break;
	case SENSOR_CHAN_ACCEL_X:
		iim4623x_accel_ms(edata->payload.accel.x, val);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		iim4623x_accel_ms(edata->payload.accel.y, val);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		iim4623x_accel_ms(edata->payload.accel.z, val);
		break;
	case SENSOR_CHAN_GYRO_X:
		iim4623x_gyro_rads(edata->payload.gyro.x, val);
		break;
	case SENSOR_CHAN_GYRO_Y:
		iim4623x_gyro_rads(edata->payload.gyro.y, val);
		break;
	case SENSOR_CHAN_GYRO_Z:
		iim4623x_gyro_rads(edata->payload.gyro.z, val);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		iim4623x_accel_ms(edata->payload.accel.x, tri_axis->x);
		iim4623x_accel_ms(edata->payload.accel.y, tri_axis->y);
		iim4623x_accel_ms(edata->payload.accel.z, tri_axis->z);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		iim4623x_gyro_rads(edata->payload.gyro.x, tri_axis->x);
		iim4623x_gyro_rads(edata->payload.gyro.y, tri_axis->y);
		iim4623x_gyro_rads(edata->payload.gyro.z, tri_axis->z);
		break;
	default:
		return -EINVAL;
	};

	return 0;
}

static int iim4623x_decoder_decode(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
				   uint32_t *fit, uint16_t max_count, void *data_out)
{
	struct iim4623x_encoded_data *edata = (struct iim4623x_encoded_data *)buffer;
	union iim4623x_encoded_channels chan_req = iim4623x_encode_channel(chan_spec.chan_type);
	struct sensor_q31_data *out = data_out;

	/* TODO: bmp581 also breaks out on: chan_spec.chan_idx != 0 */
	if (max_count == 0) {
		return -EINVAL;
	}

	if (!(chan_req.msk & edata->header.chans.msk)) {
		return -ENODATA;
	}

	/* TODO: convert timestamp? */
	out->header.base_timestamp_ns = edata->payload.timestamp;
	/* TODO: support more readings? */
	out->header.reading_count = 1;
	/* TODO: I guess shift is always 0 for our data when it's in "pure" units */
	out->shift = 0;

	/* TODO: consider a helper func for this stuff */
	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_DIE_TEMP:
	case SENSOR_CHAN_ACCEL_XYZ:
	case SENSOR_CHAN_GYRO_XYZ:
		iim4623x_decode_chan(edata, chan_spec.chan_type, out);
		break;
	default:
		return -EINVAL;
	};

	/* TODO: what is fit (The current frame iterator) and why do everybody set it (because we
	 * decode a single frame, the iterator becomes straight up 1)? */
	*fit = 1;
	return 1;
}

static bool iim4623x_decoder_has_trigger(const uint8_t *buffer, enum sensor_trigger_type trigger)
{
	struct iim4623x_encoded_data *edata = (struct iim4623x_encoded_data *)buffer;

	switch (trigger) {
	case SENSOR_TRIG_DATA_READY:
		/* TODO: this will basically always be true. We don't track if new data has arrived
		 * since last time. We probably should do so in the header field */
		return edata->payload.status.data_ready;
	default:
		break;
	}

	return false;
}

SENSOR_DECODER_API_DT_DEFINE() = {
	.get_frame_count = iim4623x_decoder_get_frame_count,
	.get_size_info = iim4623x_decoder_get_size_info,
	.decode = iim4623x_decoder_decode,
	.has_trigger = iim4623x_decoder_has_trigger,
};

int iim4623x_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);
	*decoder = &SENSOR_DECODER_NAME();

	return 0;
}
