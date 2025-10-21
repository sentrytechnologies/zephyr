/*
 * Copyright (c) 2025 Sentry Technologies ApS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>

#include "sch16t.h"
#include "sch16t_bus.h"
#include "sch16t_reg.h"
#include "sch16t_submit.h"
#include "sch16t_stream.h"
#include "sch16t_decoder.h"

LOG_MODULE_REGISTER(SCH16T_SUBMIT, CONFIG_SENSOR_LOG_LEVEL);

static const uint8_t sch16t_measu_regs[] = {
	SCH16T_REG_ACC_X2,  SCH16T_REG_ACC_Y2,  SCH16T_REG_ACC_Z2,
	SCH16T_REG_RATE_X2, SCH16T_REG_RATE_Y2, SCH16T_REG_RATE_Z2,
};

static void sch16t_complete_cb(struct rtio *ctx, const struct rtio_sqe *sqe, int res, void *arg)
{
	const struct device *dev = arg;
	struct sch16t_data *data = dev->data;
	struct rtio_iodev_sqe *iodev_sqe = data->rtio.iodev_sqe;
	struct rtio_cqe *cqe;
	int ret = 0;

	do {
		cqe = rtio_cqe_consume(ctx);
		if (cqe != NULL) {
			ret = cqe->result;
			rtio_cqe_release(ctx, cqe);
		}
	} while (cqe != NULL);

	if (ret) {
		LOG_ERR("single fetch transaction failed (ret=%d)", ret);
		rtio_iodev_sqe_err(iodev_sqe, ret);
	} else {
		rtio_iodev_sqe_ok(iodev_sqe, 0);
	}
}

void sch16t_submit_single(const struct device *dev)
{
	uint32_t min_buf_len = sizeof(struct sch16t_encoded_data);
	struct sch16t_data *data = dev->data;
	struct rtio_iodev_sqe *iodev_sqe = data->rtio.iodev_sqe;
	struct sch16t_encoded_data *edata;
	uint32_t buf_len;
	uint8_t *buf;
	int ret;

	ret = rtio_sqe_rx_buf(iodev_sqe, min_buf_len, min_buf_len, &buf, &buf_len);
	if (ret) {
		LOG_ERR("failed to get a read buffer of size %u bytes (ret=%d)", min_buf_len, ret);
		rtio_iodev_sqe_err(iodev_sqe, ret);
		return;
	}

	edata = (struct sch16t_encoded_data *)buf;

	ret = sch16t_encode(dev, buf);
	if (ret) {
		rtio_iodev_sqe_err(iodev_sqe, ret);
		return;
	}

	ret = sch16t_bus_sched_bulk_read(dev, ARRAY_SIZE(sch16t_measu_regs), sch16t_measu_regs,
					 (uint16_t *)edata->buf, sch16t_complete_cb);
	if (ret) {
		rtio_iodev_sqe_err(iodev_sqe, ret);
		return;
	}

	rtio_submit(data->rtio.ctx, 0);
}

void sch16t_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	struct sch16t_data *data = dev->data;

	data->rtio.iodev_sqe = iodev_sqe;

	if (!cfg->is_streaming) {
		sch16t_submit_single(dev);
	} else if (IS_ENABLED(CONFIG_SCH16T_STREAM)) {
		sch16t_stream_submit(dev);
	} else {
		LOG_ERR("streaming not supported (ret=%d)", -ENOTSUP);
		rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
	}
}
