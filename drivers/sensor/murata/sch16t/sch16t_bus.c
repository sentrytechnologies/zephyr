/*
 * Copyright (c) 2025 Sentry Technologies ApS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "sch16t.h"
#include "sch16t_bus.h"

LOG_MODULE_REGISTER(SCH16T_BUS, CONFIG_SENSOR_LOG_LEVEL);

static uint8_t sch16t_bus_calculate_crc3(uint32_t spi_frame)
{
	uint32_t data = spi_frame & 0xFFFFFFF8;
	uint8_t crc = 0x05;
	uint8_t data_bit;

	for (int i = 31; i >= 0; i--) {
		data_bit = (data >> i) & 0x01;
		crc = crc & 0x4 ? (uint8_t)((crc << 1) ^ 0x3) ^ data_bit
				: (uint8_t)(crc << 1) | data_bit;
		crc &= 0x7;
	}

	return crc;
}

/* for SPI frame structure, check datasheet chapter 6.2 */
static uint32_t sch16t_bus_build_spi_frame(const uint8_t reg, const bool write,
					   const uint16_t write_val)
{
	uint32_t spi_frame = 0;
	uint8_t crc;

	spi_frame |= FIELD_PREP(GENMASK(29, 22), reg);
	if (write) {
		WRITE_BIT(spi_frame, 21, 1);
		spi_frame |= FIELD_PREP(GENMASK(18, 3), write_val);
	}

	crc = sch16t_bus_calculate_crc3(spi_frame);

	spi_frame |= FIELD_PREP(GENMASK(2, 0), crc);

	sys_put_be32(spi_frame, (uint8_t *)&spi_frame);

	return spi_frame;
}

static void sch16t_bus_read_callback(struct rtio *ctx, const struct rtio_sqe *sqe, int res,
				     void *arg0)
{
	const struct device *dev = arg0;
	struct sch16t_data *data = dev->data;
	struct sch16t_rtio_payloads *payloads = &data->rtio.payloads;
	uint16_t *read_val = sqe->userdata;
	uint8_t crc;

	for (int i = 1; i < payloads->len; i++) {
		read_val[i - 1] = 0;
		payloads->in[i] = sys_get_be32((uint8_t *)&payloads->in[i]);

		crc = sch16t_bus_calculate_crc3(payloads->in[i]);

		if (crc != FIELD_GET(GENMASK(2, 0), payloads->in[i])) {
			LOG_ERR("CRC check failed (ret=%d)", -EBADMSG);
			return;
		}

		read_val[i - 1] = FIELD_GET(GENMASK(19, 4), payloads->in[i]);
	}
}

static int sch16t_bus_sched_transactions(const struct device *dev, const int len)
{
	struct sch16t_data *data = dev->data;
	struct sch16t_rtio_payloads *payloads = &data->rtio.payloads;
	struct rtio_iodev *iodev = data->rtio.iodev;
	struct rtio *ctx = data->rtio.ctx;
	struct rtio_sqe *sqe_transceive;

	for (int i = 0; i < len; i++) {
		sqe_transceive = rtio_sqe_acquire(ctx);
		if (!sqe_transceive) {
			LOG_ERR("failed to acquire transceive SQE at idx %d: len=%d (ret=%d)", i,
				len, -ENOMEM);
			return -ENOMEM;
		}

		rtio_sqe_prep_transceive(sqe_transceive, iodev, RTIO_PRIO_HIGH,
					 (uint8_t *)&payloads->out[i], (uint8_t *)&payloads->in[i],
					 sizeof(uint32_t), NULL);
		sqe_transceive->flags |= RTIO_SQE_CHAINED;
	}

	return 0;
}

int sch16t_bus_sched_bulk_write(const struct device *dev, const int len, const uint8_t *regs,
				const uint16_t *write_vals, rtio_callback_t end_cb)
{
	struct sch16t_data *data = dev->data;
	struct sch16t_rtio_payloads *payloads = &data->rtio.payloads;
	struct rtio *ctx = data->rtio.ctx;
	struct rtio_sqe *sqe_final;
	int ret;

	if (len <= 0 || len >= SCH16T_QUEUE_SIZE) {
		LOG_ERR("invalid length for bulk write: len=%d, queue_size=%d (ret=%d)", len,
			SCH16T_QUEUE_SIZE, -EINVAL);
		return -EINVAL;
	}

	payloads->len = len;

	for (int i = 0; i < len; i++) {
		payloads->out[i] = sch16t_bus_build_spi_frame(regs[i], true, write_vals[i]);
	}

	ret = sch16t_bus_sched_transactions(dev, len);
	if (ret) {
		return ret;
	}

	sqe_final = rtio_sqe_acquire(ctx);
	if (!sqe_final) {
		LOG_ERR("failed to acquire final SQE for bulk write (ret=%d)", -ENOMEM);
		return -ENOMEM;
	}

	if (end_cb) {
		rtio_sqe_prep_callback_no_cqe(sqe_final, end_cb, (void *)dev, NULL);
	} else {
		rtio_sqe_prep_nop(sqe_final, NULL, NULL);
		sqe_final->flags |= RTIO_SQE_NO_RESPONSE;
	}

	return 0;
}

int sch16t_bus_sched_bulk_read(const struct device *dev, const int len, const uint8_t *regs,
			       uint16_t *read_vals, rtio_callback_t end_cb)
{
	struct sch16t_data *data = dev->data;
	struct sch16t_rtio_payloads *payloads = &data->rtio.payloads;
	struct rtio *ctx = data->rtio.ctx;
	struct rtio_sqe *sqe_callback;
	struct rtio_sqe *sqe_final;
	int ret;

	if (len <= 0 || len >= SCH16T_QUEUE_SIZE) {
		LOG_ERR("invalid length for bulk read: len=%d, queue_size=%d (ret=%d)", len,
			SCH16T_QUEUE_SIZE, -EINVAL);
		return -EINVAL;
	}

	payloads->len = len;

	for (int i = 0; i < len; i++) {
		payloads->out[i] = sch16t_bus_build_spi_frame(regs[i], false, 0);
	}

	/* final stub transaction to extract last read */
	payloads->out[len] = sch16t_bus_build_spi_frame(0, false, 0);
	payloads->len++;

	ret = sch16t_bus_sched_transactions(dev, payloads->len);
	if (ret) {
		return ret;
	}

	sqe_callback = rtio_sqe_acquire(ctx);
	if (!sqe_callback) {
		LOG_ERR("failed to acquire callback SQE for bulk read (ret=%d)", -ENOMEM);
		return -ENOMEM;
	}

	rtio_sqe_prep_callback_no_cqe(sqe_callback, sch16t_bus_read_callback, (void *)dev,
				      read_vals);
	sqe_callback->flags |= RTIO_SQE_CHAINED;

	sqe_final = rtio_sqe_acquire(ctx);
	if (!sqe_final) {
		LOG_ERR("failed to acquire final SQE for bulk read (ret=%d)", -ENOMEM);
		return -ENOMEM;
	}

	if (end_cb) {
		rtio_sqe_prep_callback_no_cqe(sqe_final, end_cb, (void *)dev, NULL);
	} else {
		rtio_sqe_prep_nop(sqe_final, NULL, NULL);
		sqe_final->flags |= RTIO_SQE_NO_RESPONSE;
	}

	return 0;
}

int sch16t_bus_submit_and_wait(const struct device *dev)
{
	struct sch16t_data *data = dev->data;
	struct sch16t_rtio_payloads *payloads = &data->rtio.payloads;
	struct rtio *ctx = data->rtio.ctx;
	struct rtio_cqe *cqe;
	int ret;

	ret = rtio_submit(ctx, payloads->len);
	if (ret) {
		LOG_ERR("rtio_submit() failed for %d SQEs (ret=%d)", payloads->len, ret);
		return ret;
	}

	do {
		cqe = rtio_cqe_consume(ctx);
		if (cqe) {
			ret = cqe->result;
			rtio_cqe_release(ctx, cqe);
		}
	} while (cqe);

	if (ret) {
		LOG_ERR("transaction completion error (ret=%d)", ret);
		return ret;
	}

	return 0;
}
