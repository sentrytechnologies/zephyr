/*
 * Copyright (c) 2025 Sentry Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://drive.google.com/file/d/1YbTTGyutkJSeyYGsHK9ENOlcqxMWdtJm/view?usp=drive_link
 *
 */

#define DT_DRV_COMPAT witmotion_hwt905ttl

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/sensor.h>

#include "wit-protocol.h"

LOG_MODULE_REGISTER(hwt905ttl_sensor, CONFIG_SENSOR_LOG_LEVEL);

static const uint8_t hwt905ttl_unlock_buffer[] = {WIT_CMD_HEAD1, WIT_CMD_HEAD2, WIT_REG_KEY, 0x88,
						  0xB5};

static const uint8_t hwt905ttl_save_buffer[] = {WIT_CMD_HEAD1, WIT_CMD_HEAD2, WIT_REG_SAVE, 0x00,
						0x00};

static enum sensor_channel hwt905ttl_supported_channels[] = {
	SENSOR_CHAN_ACCEL_X,  SENSOR_CHAN_ACCEL_Y, SENSOR_CHAN_ACCEL_Z, SENSOR_CHAN_ACCEL_XYZ,
	SENSOR_CHAN_GYRO_X,   SENSOR_CHAN_GYRO_Y,  SENSOR_CHAN_GYRO_Z,  SENSOR_CHAN_GYRO_XYZ,
	SENSOR_CHAN_MAGN_X,   SENSOR_CHAN_MAGN_Y,  SENSOR_CHAN_MAGN_Z,  SENSOR_CHAN_MAGN_XYZ,
	SENSOR_CHAN_DIE_TEMP, SENSOR_CHAN_ALL,
};

struct hwt905ttl_measurement_acc {
	struct sensor_value ax;
	struct sensor_value ay;
	struct sensor_value az;
};

struct hwt905ttl_measurement_gyro {
	struct sensor_value wx;
	struct sensor_value wy;
	struct sensor_value wz;
};

struct hwt905ttl_measurement_magn {
	struct sensor_value hx;
	struct sensor_value hy;
	struct sensor_value hz;
};

struct hwt905ttl_measurements {
	struct hwt905ttl_measurement_acc acc;
	struct hwt905ttl_measurement_gyro gyro;
	struct hwt905ttl_measurement_magn magn;
	struct sensor_value temp;
};

/* TODO: Mutex to protect most of this */
struct hwt905ttl_data {
	uint8_t xfer_bytes;
	struct hwt905ttl_measurements measu;
	uint8_t rd_data[WIT_MEASU_READ_LEN];
	uint8_t data[WIT_TYPE_COUNT][WIT_MEASU_READ_LEN];
};

struct hwt905ttl_cfg {
	const struct device *uart_dev;
};

static void hwt905ttl_uart_flush(const struct device *uart_dev)
{
	uint8_t c;

	while (uart_fifo_read(uart_dev, &c, 1) > 0) {
		continue;
	}
}

static int hwt905ttl_checksum_compare(const uint8_t *buf)
{
	int ret;
	uint8_t sum = 0;

	for (int i = 0; i < WIT_MEASU_READ_LEN - 1; i++) {
		sum += buf[i];
	}

	ret = sum == buf[WIT_MEASU_READ_LEN - 1] ? 0 : -EINVAL;
	if (ret) {
		LOG_ERR("Invalid checksum, received 0x%02X, expected 0x%02X",
			buf[WIT_MEASU_READ_LEN - 1], sum);
	}

	return ret;
}

static int hwt905ttl_write(const struct device *dev, const uint8_t *buf)
{
	int ret;
	const struct hwt905ttl_cfg *cfg = dev->config;
	const struct device *uart_dev = cfg->uart_dev;

	/* const int sleep_ms = 1000; */

	if (!uart_dev) {
		LOG_ERR("UART device is NULL");
		return -EINVAL;
	}

	for (int i = 0; i < WIT_DATA_WRITE_LEN; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}

	/* k_msleep(sleep_ms); /\* Device needs some time between writes *\/ */

	return 0;
}

static int hwt905_send_command(const struct device *dev, enum wit_register reg, uint16_t value)
{
	uint8_t cmd[WIT_DATA_WRITE_LEN];
	int ret;

	ret = hwt905ttl_write(dev, hwt905ttl_unlock_buffer);
	if (ret) {
		return ret;
	}

	cmd[0] = WIT_CMD_HEAD1;
	cmd[1] = WIT_CMD_HEAD2;
	cmd[2] = reg;
	*(uint16_t *)(&cmd[3]) = sys_cpu_to_le16(value);

	ret = hwt905ttl_write(dev, cmd);
	if (ret) {
		return ret;
	}

	ret = hwt905ttl_write(dev, hwt905ttl_save_buffer);
	if (ret) {
		return ret;
	}

	return 0;
}

static int hwt905ttl_attr_set(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, const struct sensor_value *val)
{
	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		if (chan != SENSOR_CHAN_ALL) {
			return -ENOTSUP;
		}

		switch (val->val1) {
		case 0:
			return hwt905_send_command(dev, WIT_REG_RRATE, WIT_RRATE_NONE);
		case 1:
			return hwt905_send_command(dev, WIT_REG_RRATE, WIT_RRATE_1HZ);
		case 2:
			return hwt905_send_command(dev, WIT_REG_RRATE, WIT_RRATE_2HZ);
		case 5:
			return hwt905_send_command(dev, WIT_REG_RRATE, WIT_RRATE_5HZ);
		case 10:
			return hwt905_send_command(dev, WIT_REG_RRATE, WIT_RRATE_10HZ);
		case 20:
			return hwt905_send_command(dev, WIT_REG_RRATE, WIT_RRATE_20HZ);
		case 50:
			return hwt905_send_command(dev, WIT_REG_RRATE, WIT_RRATE_50HZ);
		case 100:
			return hwt905_send_command(dev, WIT_REG_RRATE, WIT_RRATE_100HZ);
		case 200:
			return hwt905_send_command(dev, WIT_REG_RRATE, WIT_RRATE_200HZ);
		default:
			return -ENOTSUP;
		}

		break;
	default:
		return -ENOTSUP;
	}
}

static int hwt905ttl_poll_acc(const struct device *dev)
{
	struct hwt905ttl_data *data = dev->data;
	struct hwt905ttl_measurements *measu = &data->measu;
	static const double aux = (double)16LL / 32768LL * 1000000LL; /* converts output to ug */
	const int16_t *raw_16 = (int16_t *)data->data[WIT_TYPE_ACCEL - WIT_TYPE_OFFSET];

	sensor_ug_to_ms2((double)sys_le16_to_cpu(raw_16[1]) * aux, &measu->acc.ax);
	sensor_ug_to_ms2((double)sys_le16_to_cpu(raw_16[2]) * aux, &measu->acc.ay);
	sensor_ug_to_ms2((double)sys_le16_to_cpu(raw_16[3]) * aux, &measu->acc.az);

	return 0;
}

static int hwt905ttl_poll_gyro(const struct device *dev)
{
	struct hwt905ttl_data *data = dev->data;
	struct hwt905ttl_measurements *measu = &data->measu;
	static const double aux = (double)2000 / 32768 * 100000LL; /* converts output to 10 udeg */
	const int16_t *raw_16 = (int16_t *)data->data[WIT_TYPE_GYRO - WIT_TYPE_OFFSET];

	sensor_10udegrees_to_rad((double)sys_le16_to_cpu(raw_16[1]) * aux, &measu->gyro.wx);
	sensor_10udegrees_to_rad((double)sys_le16_to_cpu(raw_16[2]) * aux, &measu->gyro.wy);
	sensor_10udegrees_to_rad((double)sys_le16_to_cpu(raw_16[3]) * aux, &measu->gyro.wz);

	return 0;
}

static int hwt905ttl_poll_magn(const struct device *dev)
{
	struct hwt905ttl_data *data = dev->data;
	struct hwt905ttl_measurements *measu = &data->measu;
	const int16_t *raw_16 = (int16_t *)data->data[WIT_TYPE_MAG - WIT_TYPE_OFFSET];

	measu->magn.hx.val1 = sys_le16_to_cpu(raw_16[1]);
	measu->magn.hy.val1 = sys_le16_to_cpu(raw_16[2]);
	measu->magn.hz.val1 = sys_le16_to_cpu(raw_16[3]);

	return 0;
}

static int hwt905ttl_poll_temp(const struct device *dev)
{
	int ret;
	struct hwt905ttl_data *data = dev->data;
	struct hwt905ttl_measurements *measu = &data->measu;
	const int16_t *raw_16 = (int16_t *)data->data[WIT_TYPE_ACCEL - WIT_TYPE_OFFSET];

	ret = sensor_value_from_double(&measu->temp, (double)sys_le16_to_cpu(raw_16[4]) / 100LL);
	if (ret) {
		return ret;
	}

	return 0;
}

static int hwt905ttl_poll_data(const struct device *dev, enum sensor_channel chan)
{
	int ret;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		ret = hwt905ttl_poll_acc(dev);
		if (ret) {
			return ret;
		}
		break;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		ret = hwt905ttl_poll_gyro(dev);
		if (ret) {
			return ret;
		}
		break;
	case SENSOR_CHAN_MAGN_X:
	case SENSOR_CHAN_MAGN_Y:
	case SENSOR_CHAN_MAGN_Z:
	case SENSOR_CHAN_MAGN_XYZ:
		ret = hwt905ttl_poll_magn(dev);
		if (ret) {
			return ret;
		}
		break;
	case SENSOR_CHAN_DIE_TEMP:
		ret = hwt905ttl_poll_temp(dev);
		if (ret) {
			return ret;
		}
		break;
	case SENSOR_CHAN_ALL:
		ret = hwt905ttl_poll_acc(dev);
		if (ret) {
			return ret;
		}
		ret = hwt905ttl_poll_gyro(dev);
		if (ret) {
			return ret;
		}
		ret = hwt905ttl_poll_magn(dev);
		if (ret) {
			return ret;
		}
		ret = hwt905ttl_poll_temp(dev);
		if (ret) {
			return ret;
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int hwt905ttl_channel_get(const struct device *dev, enum sensor_channel chan,
				 struct sensor_value *val)
{
	struct hwt905ttl_data *data = dev->data;
	struct hwt905ttl_measurements *measu = &data->measu;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		*val = measu->acc.ax;
		break;
	case SENSOR_CHAN_ACCEL_Y:
		*val = measu->acc.ay;
		break;
	case SENSOR_CHAN_ACCEL_Z:
		*val = measu->acc.az;
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		val[0] = measu->acc.ax;
		val[1] = measu->acc.ay;
		val[2] = measu->acc.az;
		break;
	case SENSOR_CHAN_GYRO_X:
		*val = measu->gyro.wx;
	case SENSOR_CHAN_GYRO_Y:
		*val = measu->gyro.wy;
	case SENSOR_CHAN_GYRO_Z:
		*val = measu->gyro.wz;
	case SENSOR_CHAN_GYRO_XYZ:
		val[0] = measu->gyro.wx;
		val[1] = measu->gyro.wy;
		val[2] = measu->gyro.wz;
		break;
	case SENSOR_CHAN_MAGN_X:
		*val = measu->magn.hx;
		break;
	case SENSOR_CHAN_MAGN_Y:
		*val = measu->magn.hy;
		break;
	case SENSOR_CHAN_MAGN_Z:
		*val = measu->magn.hz;
		break;
	case SENSOR_CHAN_MAGN_XYZ:
		val[0] = measu->magn.hx;
		val[1] = measu->magn.hy;
		val[2] = measu->magn.hz;
		break;
	case SENSOR_CHAN_DIE_TEMP:
		*val = measu->temp;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int hwt905ttl_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	ARRAY_FOR_EACH(hwt905ttl_supported_channels, i) {
		if (chan == hwt905ttl_supported_channels[i]) {
			return hwt905ttl_poll_data(dev, chan);
		}
	}

	return -ENOTSUP;
}

static DEVICE_API(sensor, hwt905ttl_api_funcs) = {
	.sample_fetch = hwt905ttl_sample_fetch,
	.channel_get = hwt905ttl_channel_get,
};

static void hwt905ttl_uart_isr(const struct device *uart_dev, void *user_data)
{
	enum wit_data_type data_type;
	const struct device *dev = user_data;
	struct hwt905ttl_data *data = dev->data;

	if (!uart_dev) {
		LOG_ERR("UART device is NULL");
		return;
	}

	if (!uart_irq_update(uart_dev)) {
		LOG_ERR("Unable to start processing interrupts");
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		LOG_ERR("UART RX is not ready yet");
		return;
	}

	data->xfer_bytes += uart_fifo_read(uart_dev, &data->rd_data[data->xfer_bytes],
					   WIT_MEASU_READ_LEN - data->xfer_bytes);

	if ((data->rd_data[0] != WIT_HDR_START)) {
		LOG_DBG("First byte not header! Resetting # of bytes read.");
		data->xfer_bytes = 0;
		return;
	}

	if (data->xfer_bytes != WIT_MEASU_READ_LEN) {
		return;
	}

	data_type = data->rd_data[1];
	if (data_type < WIT_TYPE_START || data_type > WIT_TYPE_END) {
		LOG_ERR("Data type out of range: 0x%02X", data->rd_data[1]);
		data->xfer_bytes = 0;
		return;
	}

	if (hwt905ttl_checksum_compare(data->rd_data)) {
		data->xfer_bytes = 0;
		return;
	}

	memcpy(data->data[data_type - WIT_TYPE_OFFSET], data->rd_data, WIT_MEASU_READ_LEN);

	hwt905ttl_uart_flush(uart_dev);
	data->xfer_bytes = 0;
}

static int hwt905ttl_init(const struct device *dev)
{
	const struct hwt905ttl_cfg *cfg = dev->config;
	int ret = 0;

	if (!device_is_ready(cfg->uart_dev)) {
		LOG_ERR("UART device not ready");
		return -ENODEV;
	}

	uart_irq_rx_disable(cfg->uart_dev);
	uart_irq_tx_disable(cfg->uart_dev);

	hwt905ttl_uart_flush(cfg->uart_dev);

	ret = uart_irq_callback_user_data_set(cfg->uart_dev, hwt905ttl_uart_isr, (void *)dev);

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			LOG_ERR("Interrupt-driven UART API support not enabled");
		} else if (ret == -ENOSYS) {
			LOG_ERR("UART device does not support interrupt-driven API");
		} else {
			LOG_ERR("Error setting UART callback: %d", ret);
		}
		return ret;
	}

	uart_irq_rx_enable(cfg->uart_dev);

	return ret;
}

#define HWT905TTL_INIT(inst)                                                                       \
                                                                                                   \
	static struct hwt905ttl_data hwt905ttl_data_##inst;                                        \
                                                                                                   \
	static const struct hwt905ttl_cfg hwt905ttl_cfg_##inst = {                                 \
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                                      \
	};                                                                                         \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, hwt905ttl_init, NULL, &hwt905ttl_data_##inst,           \
				     &hwt905ttl_cfg_##inst, POST_KERNEL,                           \
				     CONFIG_SENSOR_INIT_PRIORITY, &hwt905ttl_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(HWT905TTL_INIT)
