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

#include <sensor/witmotion/wit-protocol.h>

LOG_MODULE_REGISTER(hwt905ttl_sensor, CONFIG_SENSOR_LOG_LEVEL);

/*
 * Empirical measurements show that after every packet write, hwt905ttl needs at least 3ms to
 * register the command
 */
#define HWT905TTL_WRITE_SLEEP_MS 3

#define HWT905TTL_TIMEOUT_MS 500

static const uint8_t hwt905ttl_unlock_buffer[] = {WIT_CMD_HEAD1, WIT_CMD_HEAD2, WIT_REG_KEY, 0x88,
						  0xB5};

static const uint8_t hwt905ttl_save_buffer[] = {WIT_CMD_HEAD1, WIT_CMD_HEAD2, WIT_REG_SAVE, 0x00,
						0x00};

static enum sensor_channel hwt905ttl_supported_channels[] = {
	SENSOR_CHAN_ACCEL_X,
	SENSOR_CHAN_ACCEL_Y,
	SENSOR_CHAN_ACCEL_Z,
	SENSOR_CHAN_ACCEL_XYZ,
	SENSOR_CHAN_GYRO_X,
	SENSOR_CHAN_GYRO_Y,
	SENSOR_CHAN_GYRO_Z,
	SENSOR_CHAN_GYRO_XYZ,
	SENSOR_CHAN_MAGN_X,
	SENSOR_CHAN_MAGN_Y,
	SENSOR_CHAN_MAGN_Z,
	SENSOR_CHAN_MAGN_XYZ,
	SENSOR_CHAN_HWT905TTL_ANG_X,
	SENSOR_CHAN_HWT905TTL_ANG_Y,
	SENSOR_CHAN_HWT905TTL_ANG_Z,
	SENSOR_CHAN_DIE_TEMP,
	SENSOR_CHAN_ALL,
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

struct hwt905ttl_measurement_ang {
	struct sensor_value angx;
	struct sensor_value angy;
	struct sensor_value angz;
};

struct hwt905ttl_measurements {
	struct hwt905ttl_measurement_acc acc;
	struct hwt905ttl_measurement_gyro gyro;
	struct hwt905ttl_measurement_magn magn;
	struct hwt905ttl_measurement_ang ang;
	struct sensor_value temp;
};

struct hwt905ttl_rx_data {
	uint8_t xfer_bytes;
	struct k_sem read_done;
	uint8_t rd[WIT_MEASU_READ_LEN];
	uint8_t raw[WIT_TYPE_COUNT][WIT_MEASU_READ_LEN];
};

struct hwt905ttl_tx_data {
	uint8_t xfer_bytes;
	const uint8_t *cmd;
	struct k_sem tx_done;
};

/* TODO: Mutex to protect most of this */
struct hwt905ttl_data {
	struct hwt905ttl_rx_data rx_data;
	struct hwt905ttl_tx_data tx_data;
	struct hwt905ttl_measurements measu;
};

struct hwt905ttl_cfg {
	const struct device *uart_dev;
};

struct hwt905ttl_rrate_map_entry {
	uint32_t freq_uhz;
	enum wit_output_rate code;
};

/* clang-format off */
static const struct hwt905ttl_rrate_map_entry hwt905ttl_rrate_map[] = {
	{0U,		WIT_RRATE_NONE},
	{200000U,	WIT_RRATE_0_2HZ},
	{500000U,	WIT_RRATE_0_5HZ},
	{1000000U,	WIT_RRATE_1HZ},
	{2000000U,	WIT_RRATE_2HZ},
	{5000000U,	WIT_RRATE_5HZ},
	{10000000U,	WIT_RRATE_10HZ},
	{20000000U,	WIT_RRATE_20HZ},
	{50000000U,	WIT_RRATE_50HZ},
	{100000000U,	WIT_RRATE_100HZ},
	{200000000U,	WIT_RRATE_200HZ},
};
/* clang-format on */

static inline uint32_t hwt905ttl_sensor_value_to_uhz(const struct sensor_value *v)
{
	return (uint32_t)v->val1 * 1000000U + (uint32_t)v->val2;
}

static inline void hwt905ttl_uhz_to_sensor_value(uint32_t uhz, struct sensor_value *v)
{
	v->val1 = uhz / 1000000U;
	v->val2 = uhz % 1000000U;
}

static int hwt905ttl_rrate_code_from_value(const struct sensor_value *val, uint8_t *out_code)
{
	uint32_t uhz = hwt905ttl_sensor_value_to_uhz(val);

	for (size_t i = 0; i < ARRAY_SIZE(hwt905ttl_rrate_map); i++) {
		if (hwt905ttl_rrate_map[i].freq_uhz == uhz) {
			*out_code = hwt905ttl_rrate_map[i].code;
			return 0;
		}
	}
	return -ENOTSUP;
}

static int hwt905ttl_rrate_value_from_code(uint8_t code, struct sensor_value *out_val)
{
	for (size_t i = 0; i < ARRAY_SIZE(hwt905ttl_rrate_map); i++) {
		if (hwt905ttl_rrate_map[i].code == code) {
			hwt905ttl_uhz_to_sensor_value(hwt905ttl_rrate_map[i].freq_uhz, out_val);
			return 0;
		}
	}
	return -ENOTSUP;
}

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
	struct hwt905ttl_data *data = dev->data;
	const struct hwt905ttl_cfg *cfg = dev->config;

	data->tx_data.cmd = buf;
	uart_irq_tx_enable(cfg->uart_dev);
	ret = k_sem_take(&data->tx_data.tx_done, K_MSEC(HWT905TTL_TIMEOUT_MS));
	if (ret) {
		return ret;
	}
	k_msleep(HWT905TTL_WRITE_SLEEP_MS);

	return 0;
}

static int hwt905ttl_send_command(const struct device *dev, enum wit_register reg, uint16_t value)
{
	int ret;
	uint8_t local_cmd[WIT_DATA_WRITE_LEN];

	local_cmd[0] = WIT_CMD_HEAD1;
	local_cmd[1] = WIT_CMD_HEAD2;
	local_cmd[2] = reg;
	*(uint16_t *)(&local_cmd[3]) = sys_cpu_to_le16(value);

	ret = hwt905ttl_write(dev, hwt905ttl_unlock_buffer);
	if (ret) {
		return ret;
	}

	ret = hwt905ttl_write(dev, local_cmd);
	if (ret) {
		return ret;
	}

	ret = hwt905ttl_write(dev, hwt905ttl_save_buffer);
	if (ret) {
		return ret;
	}

	return 0;
}

static int hwt905ttl_read(const struct device *dev, enum wit_register reg, uint8_t *value)
{
	int ret;
	struct hwt905ttl_data *data = dev->data;

	ret = hwt905ttl_send_command(dev, WIT_REG_READADDR, reg);
	if (ret) {
		return ret;
	}

	ret = k_sem_take(&data->rx_data.read_done, K_MSEC(HWT905TTL_TIMEOUT_MS));
	if (ret) {
		return ret;
	}

	*value = data->rx_data.raw[WIT_TYPE_READ_REG - WIT_TYPE_OFFSET][2];

	return 0;
}

static int hwt905ttl_attr_get(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, struct sensor_value *val)
{
	int ret;
	uint8_t value;

	val->val1 = 0;
	val->val2 = 0;

	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		if (chan != SENSOR_CHAN_ALL) {
			return -ENOTSUP;
		}

		ret = hwt905ttl_read(dev, WIT_REG_RRATE, &value);
		if (ret) {
			return ret;
		}

		return hwt905ttl_rrate_value_from_code(value, val);
	case SENSOR_ATTR_CALIBRATION:
		if (chan > SENSOR_CHAN_ACCEL_XYZ &&
		    chan < (enum sensor_channel)SENSOR_CHAN_HWT905TTL_ANG_X) {
			return -ENOTSUP;
		}

		ret = hwt905ttl_read(dev, WIT_REG_CALSW, &value);
		if (ret) {
			return ret;
		}

		switch (value) {
		case WIT_CAL_NORMAL:
			val->val1 = 0;
			return 0;
		case WIT_CAL_ACCEL_AUTO:
			if (chan >= SENSOR_CHAN_ACCEL_X && chan <= SENSOR_CHAN_ACCEL_XYZ) {
				val->val1 = 1;
			} else {
				val->val1 = 0;
			}
			return 0;
		case WIT_CAL_MAG_SPHERE:
			if (chan == (enum sensor_channel)SENSOR_CHAN_HWT905TTL_ANG_Z) {
				val->val1 = 1;
			} else {
				val->val1 = 0;
			}
			return 0;
		default:
			return -ENOTSUP;
		}
	case SENSOR_ATTR_FEATURE_MASK:
		if (chan != SENSOR_CHAN_ALL) {
			return -ENOTSUP;
		}

		/* Check enum wit_rsw_mask for interpertation of configuration values */
		ret = hwt905ttl_read(dev, WIT_REG_RSW, &value);
		val->val1 = value;
		return ret;
	default:
		return -ENOTSUP;
	}
}

static int hwt905ttl_attr_set(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, const struct sensor_value *val)
{
	int ret;
	uint8_t code;

	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY: {
		if (chan != SENSOR_CHAN_ALL) {
			return -ENOTSUP;
		}

		ret = hwt905ttl_rrate_code_from_value(val, &code);
		if (ret) {
			return ret;
		}

		return hwt905ttl_send_command(dev, WIT_REG_RRATE, code);
	}
	case SENSOR_ATTR_CALIBRATION: {
		if (val->val2 != 0) {
			return -ENOTSUP;
		}
		if (val->val1 == 0) {
			return hwt905ttl_send_command(dev, WIT_REG_CALSW, WIT_CAL_NORMAL);
		} else if (val->val1 != 1) {
			return -ENOTSUP;
		}

		if (chan >= SENSOR_CHAN_ACCEL_X && chan <= SENSOR_CHAN_ACCEL_XYZ) {
			return hwt905ttl_send_command(dev, WIT_REG_CALSW, WIT_CAL_ACCEL_AUTO);
		}
		if (chan >= (enum sensor_channel)SENSOR_CHAN_HWT905TTL_ANG_X &&
		    chan <= (enum sensor_channel)SENSOR_CHAN_HWT905TTL_ANG_Y) {
			return hwt905ttl_send_command(dev, WIT_REG_CALSW, WIT_CAL_REF_ANGLE);
		}
		if (chan == (enum sensor_channel)SENSOR_CHAN_HWT905TTL_ANG_Z) {
			return hwt905ttl_send_command(dev, WIT_REG_CALSW, WIT_CAL_MAG_SPHERE);
		}

		return -ENOTSUP;
	}
	case SENSOR_ATTR_FEATURE_MASK:
		if (chan != SENSOR_CHAN_ALL) {
			return -ENOTSUP;
		}

		/* Check enum wit_rsw_mask for valid configuration values */
		return hwt905ttl_send_command(dev, WIT_REG_RSW, val->val1);
	default:
		return -ENOTSUP;
	}
}

static int hwt905ttl_poll_acc(const struct device *dev)
{
	struct hwt905ttl_data *data = dev->data;
	struct hwt905ttl_measurements *measu = &data->measu;
	static const double aux = (double)16LL / 32768LL * 1000000LL; /* converts output to ug */
	const int16_t *raw_16 = (int16_t *)data->rx_data.raw[WIT_TYPE_ACCEL - WIT_TYPE_OFFSET];

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
	const int16_t *raw_16 = (int16_t *)data->rx_data.raw[WIT_TYPE_GYRO - WIT_TYPE_OFFSET];

	sensor_10udegrees_to_rad((double)sys_le16_to_cpu(raw_16[1]) * aux, &measu->gyro.wx);
	sensor_10udegrees_to_rad((double)sys_le16_to_cpu(raw_16[2]) * aux, &measu->gyro.wy);
	sensor_10udegrees_to_rad((double)sys_le16_to_cpu(raw_16[3]) * aux, &measu->gyro.wz);

	return 0;
}

static int hwt905ttl_poll_magn(const struct device *dev)
{
	struct hwt905ttl_data *data = dev->data;
	struct hwt905ttl_measurements *measu = &data->measu;
	const int16_t *raw_16 = (int16_t *)data->rx_data.raw[WIT_TYPE_MAG - WIT_TYPE_OFFSET];

	measu->magn.hx.val1 = sys_le16_to_cpu(raw_16[1]);
	measu->magn.hy.val1 = sys_le16_to_cpu(raw_16[2]);
	measu->magn.hz.val1 = sys_le16_to_cpu(raw_16[3]);

	return 0;
}

static int hwt905ttl_poll_ang(const struct device *dev)
{
	struct hwt905ttl_data *data = dev->data;
	struct hwt905ttl_measurements *measu = &data->measu;
	static const double aux = (double)180 / 32768 * 100000LL; /* converts output to 10 udeg */
	const int16_t *raw_16 = (int16_t *)data->rx_data.raw[WIT_TYPE_ANGLE - WIT_TYPE_OFFSET];

	sensor_10udegrees_to_rad((double)sys_le16_to_cpu(raw_16[1]) * aux, &measu->ang.angx);
	sensor_10udegrees_to_rad((double)sys_le16_to_cpu(raw_16[2]) * aux, &measu->ang.angy);
	sensor_10udegrees_to_rad((double)sys_le16_to_cpu(raw_16[3]) * aux, &measu->ang.angz);

	return 0;
}

static int hwt905ttl_poll_temp(const struct device *dev)
{
	int ret;
	struct hwt905ttl_data *data = dev->data;
	struct hwt905ttl_measurements *measu = &data->measu;
	const int16_t *raw_16 = (int16_t *)data->rx_data.raw[WIT_TYPE_ACCEL - WIT_TYPE_OFFSET];

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
	case (enum sensor_channel)SENSOR_CHAN_HWT905TTL_ANG_X:
	case (enum sensor_channel)SENSOR_CHAN_HWT905TTL_ANG_Y:
	case (enum sensor_channel)SENSOR_CHAN_HWT905TTL_ANG_Z:
		ret = hwt905ttl_poll_ang(dev);
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
		ret = hwt905ttl_poll_ang(dev);
		if (ret) {
			return ret;
		}
		break;
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
	case (enum sensor_channel)SENSOR_CHAN_HWT905TTL_ANG_X:
		*val = measu->ang.angx;
		break;
	case (enum sensor_channel)SENSOR_CHAN_HWT905TTL_ANG_Y:
		*val = measu->ang.angy;
		break;
	case (enum sensor_channel)SENSOR_CHAN_HWT905TTL_ANG_Z:
		*val = measu->ang.angz;
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

/* TODO: triggers */
static DEVICE_API(sensor, hwt905ttl_api_funcs) = {
	.sample_fetch = hwt905ttl_sample_fetch,
	.channel_get = hwt905ttl_channel_get,
	.attr_set = hwt905ttl_attr_set,
	.attr_get = hwt905ttl_attr_get,
};

static void hwt905ttl_uart_isr_rx(const struct device *uart_dev, const struct device *dev)
{
	enum wit_data_type data_type;
	struct hwt905ttl_data *data = dev->data;
	struct hwt905ttl_rx_data *rx_data = &data->rx_data;

	rx_data->xfer_bytes += uart_fifo_read(uart_dev, &rx_data->rd[rx_data->xfer_bytes],
					      WIT_MEASU_READ_LEN - rx_data->xfer_bytes);

	if ((rx_data->rd[0] != WIT_HDR_START)) {
		LOG_DBG("First byte not header! Resetting # of bytes read.");
		rx_data->xfer_bytes = 0;
		return;
	}

	if (rx_data->xfer_bytes != WIT_MEASU_READ_LEN) {
		return;
	}

	data_type = rx_data->rd[1];
	if (data_type < WIT_TYPE_START || data_type > WIT_TYPE_END) {
		LOG_ERR("Data type out of range: 0x%02X", rx_data->rd[1]);
		rx_data->xfer_bytes = 0;
		return;
	}

	if (hwt905ttl_checksum_compare(rx_data->rd)) {
		rx_data->xfer_bytes = 0;
		return;
	}

	memcpy(rx_data->raw[data_type - WIT_TYPE_OFFSET], rx_data->rd, WIT_MEASU_READ_LEN);

	if (data_type == WIT_TYPE_READ_REG) {
		k_sem_give(&rx_data->read_done);
	}

	hwt905ttl_uart_flush(uart_dev);
	rx_data->xfer_bytes = 0;
}

static void hwt905ttl_uart_isr_tx(const struct device *uart_dev, const struct device *dev)
{
	struct hwt905ttl_data *data = dev->data;
	struct hwt905ttl_tx_data *tx_data = &data->tx_data;

	tx_data->xfer_bytes += uart_fifo_fill(uart_dev, &tx_data->cmd[tx_data->xfer_bytes],
					      WIT_DATA_WRITE_LEN - tx_data->xfer_bytes);

	if (tx_data->xfer_bytes == WIT_DATA_WRITE_LEN) {
		tx_data->xfer_bytes = 0;
		uart_irq_tx_disable(uart_dev);
		k_sem_give(&data->tx_data.tx_done);
	}
}

static void hwt905ttl_uart_isr(const struct device *uart_dev, void *user_data)
{
	const struct device *dev = user_data;

	if (!uart_dev) {
		LOG_ERR("UART device is NULL");
		return;
	}

	if (!uart_irq_update(uart_dev)) {
		LOG_ERR("Unable to start processing interrupts");
		return;
	}

	if (uart_irq_rx_ready(uart_dev)) {
		hwt905ttl_uart_isr_rx(uart_dev, dev);
	}

	if (uart_irq_tx_ready(uart_dev)) {
		hwt905ttl_uart_isr_tx(uart_dev, dev);
	}
}

static int hwt905ttl_init(const struct device *dev)
{
	const struct hwt905ttl_cfg *cfg = dev->config;
	struct hwt905ttl_data *data = dev->data;
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
			LOG_ERR("UART device does not support interrupt-driven "
				"API");
		} else {
			LOG_ERR("Error setting UART callback: %d", ret);
		}
		return ret;
	}

	k_sem_init(&data->tx_data.tx_done, 0, 1);
	k_sem_init(&data->rx_data.read_done, 0, 1);

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
