/*
 * Copyright (c) 2025 Sentry Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT u_blox_x20p

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gnss/gnss_publish.h>

#include <zephyr/modem/ubx.h>
#include <zephyr/modem/ubx/protocol.h>
#include <zephyr/modem/ubx/keys.h>
#include <zephyr/modem/backend/uart.h>

#include "gnss_ubx_common.h"
#include <zephyr/gnss/rtk/rtk.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ubx_x20p, CONFIG_GNSS_LOG_LEVEL);

#if CONFIG_GNSS_U_BLOX_X20P_RTK
enum ubx_x20p_rtk_mode {
	UBX_X20P_RTK_MODE_NONE,
	UBX_X20P_RTK_MODE_STATION,
	UBX_X20P_RTK_MODE_ROVER,
};
#endif /* CONFIG_GNSS_U_BLOX_X20P_RTK */

struct ubx_x20p_config {
	const struct device *bus;
	uint16_t fix_rate_ms;
	bool reset;
#if CONFIG_GNSS_U_BLOX_X20P_RTK
	enum ubx_x20p_rtk_mode rtk_mode;
#endif /* CONFIG_GNSS_U_BLOX_X20P_RTK */
#if CONFIG_GNSS_U_BLOX_X20P_STATION
	const struct ubx_frame *svin_acc_lim;
	const struct ubx_frame *svin_min_dur;
#endif /* CONFIG_GNSS_U_BLOX_X20P_STATION */
};

struct ubx_x20p_data {
	struct gnss_ubx_common_data common_data;
	struct {
		struct modem_pipe *pipe;
		struct modem_backend_uart uart_backend;
		uint8_t receive_buf[1024];
		uint8_t transmit_buf[256];
	} backend;
	struct {
		struct modem_ubx inst;
		uint8_t receive_buf[1024];
	} ubx;
	struct {
		struct modem_ubx_script inst;
		uint8_t response_buf[512];
		uint8_t request_buf[256];
		struct k_sem req_buf_lock;
		struct k_sem lock;
	} script;
#if CONFIG_GNSS_SATELLITES
	struct gnss_satellite satellites[CONFIG_GNSS_U_BLOX_X20P_SATELLITES_COUNT];
#endif
};

#if CONFIG_GNSS_U_BLOX_X20P_STATION
static void x20p_ubx_svin_callback(struct modem_ubx *ubx, const struct ubx_frame *frame, size_t len,
				  void *user_data)
{
	struct ubx_nav_svin {
		uint8_t version;
		uint8_t reserved1[3];
		uint32_t itow;
		uint32_t dur;
		int32_t meanx;
		int32_t meany;
		int32_t meanz;
		int8_t meanxhp;
		int8_t meanyhp;
		int8_t meanzhp;
		uint8_t reserved2;
		uint32_t acc;
		uint32_t obs;
		uint8_t valid;
		uint8_t active;
	} __packed;

	struct ubx_nav_svin *svin = (struct ubx_nav_svin *)&frame->payload_and_checksum;

	/* TODO: foward this information to userspace instead of just logging */
	if (!svin->valid) {
		LOG_INF("station svin: dur: %d s, acc: %d mm, obs: %d", svin->dur, svin->acc / 10,
			svin->obs);
	}
};
#endif

#ifdef CONFIG_GNSS_U_BLOX_X20P_RTK
static void x20p_ubx_relposned_callback(struct modem_ubx *ubx, const struct ubx_frame *frame,
				       size_t len, void *user_data)
{
	struct ubx_nav_relposned {
		uint8_t version;
		uint8_t reserved1;
		uint16_t ref_station_id;
		uint32_t itow;
		int32_t rel_pos_n;
		int32_t rel_pos_e;
		int32_t rel_pos_d;
		int32_t rel_pos_len;
		int32_t rel_pos_heading;
		uint8_t reserved2[4];
		int8_t rel_pos_hpn;
		int8_t rel_pos_hpe;
		int8_t rel_pos_hpd;
		int8_t rel_pos_hp_len;
		uint32_t acc_n;
		uint32_t acc_e;
		uint32_t acc_d;
		uint32_t acc_len;
		uint32_t acc_head;
		uint8_t reserved3[4];
		uint32_t flags;
	} __packed;

	struct ubx_nav_relposned *r = (void *)&frame->payload_and_checksum;
	struct gnss_ubx_common_data *data = user_data;

	sys_le_to_cpu(&r->rel_pos_heading, sizeof(r->rel_pos_heading));
	sys_le_to_cpu(&r->flags, sizeof(r->flags));
	data->heading.value = r->rel_pos_heading;
	data->heading.valid = (r->flags >> 8) & 0x01;
}
#endif

UBX_FRAME_DEFINE(disable_nmea_gga,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_NMEA_GGA_UART1, 0));
UBX_FRAME_DEFINE(disable_nmea_rmc,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_NMEA_RMC_UART1, 0));
UBX_FRAME_DEFINE(disable_nmea_gsv,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_NMEA_GSV_UART1, 0));
UBX_FRAME_DEFINE(disable_nmea_dtm,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_NMEA_DTM_UART1, 0));
UBX_FRAME_DEFINE(disable_nmea_gbs,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_NMEA_GBS_UART1, 0));
UBX_FRAME_DEFINE(disable_nmea_gll,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_NMEA_GLL_UART1, 0));
UBX_FRAME_DEFINE(disable_nmea_gns,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_NMEA_GNS_UART1, 0));
UBX_FRAME_DEFINE(disable_nmea_grs,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_NMEA_GRS_UART1, 0));
UBX_FRAME_DEFINE(disable_nmea_gsa,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_NMEA_GSA_UART1, 0));
UBX_FRAME_DEFINE(disable_nmea_gst,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_NMEA_GST_UART1, 0));
UBX_FRAME_DEFINE(disable_nmea_vlw,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_NMEA_VLW_UART1, 0));
UBX_FRAME_DEFINE(disable_nmea_vtg,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_NMEA_VTG_UART1, 0));
UBX_FRAME_DEFINE(disable_nmea_zda,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_NMEA_ZDA_UART1, 0));
UBX_FRAME_DEFINE(enable_nav,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_UBX_NAV_PVT_UART1, 1));
UBX_FRAME_DEFINE(nav_fix_mode_auto,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_NAV_CFG_FIX_MODE, UBX_FIX_MODE_AUTO));
UBX_FRAME_DEFINE(enable_prot_in_ubx,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_UART1_PROTO_IN_UBX, 1));
UBX_FRAME_DEFINE(enable_prot_in_rtcm3,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_UART1_PROTO_IN_RTCM3X, 1));
UBX_FRAME_DEFINE(enable_prot_out_ubx,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_UART1_PROTO_OUT_UBX, 1));
UBX_FRAME_DEFINE(disable_prot_out_rtcm3_uart1,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_UART1_PROTO_OUT_RTCM3X, 0));
UBX_FRAME_DEFINE(set_rtk_fix_mode,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_NAV_HP_CFG_GNSS_MODE,
					     UBX_NAV_HP_DGNSS_MODE_RTK_FIXED));
UBX_FRAME_DEFINE(enable_prot_en_uart2,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_UART2_PROTO_ENABLED, 1));
UBX_FRAME_DEFINE(enable_prot_in_rtcm3_uart2,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_UART2_PROTO_IN_RTCM3X, 1));
#if CONFIG_GNSS_SATELLITES
UBX_FRAME_DEFINE(enable_sat,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_UBX_NAV_SAT_UART1, 1));
#endif

/* Antenna configuration */
UBX_FRAME_DEFINE(enable_ant_voltctrl,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_HW_ANT_CFG_VOLTCTRL, 1));
UBX_FRAME_DEFINE(enable_ant_shortdet,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_HW_ANT_CFG_SHORTDET, 1));
UBX_FRAME_DEFINE(enable_ant_opendet,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_HW_ANT_CFG_OPENDET, 1));
UBX_FRAME_DEFINE(enable_ant_pwrdown,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_HW_ANT_CFG_PWRDOWN, 1));

#if CONFIG_GNSS_U_BLOX_X20P_STATION
UBX_FRAME_DEFINE(enable_prot_out_rtcm3_uart2,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_UART2_PROTO_OUT_RTCM3X, 1));

UBX_FRAME_DEFINE(enable_rtcm3_1005,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_UBX_RTCM_3X_TYPE1005_UART2, 1));
UBX_FRAME_DEFINE(enable_rtcm3_1074,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_UBX_RTCM_3X_TYPE1074_UART2, 1));
UBX_FRAME_DEFINE(enable_rtcm3_1077,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_UBX_RTCM_3X_TYPE1077_UART2, 1));
UBX_FRAME_DEFINE(enable_rtcm3_1094,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_UBX_RTCM_3X_TYPE1094_UART2, 1));
UBX_FRAME_DEFINE(enable_rtcm3_1097,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_UBX_RTCM_3X_TYPE1097_UART2, 1));
UBX_FRAME_DEFINE(enable_rtcm3_1124,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_UBX_RTCM_3X_TYPE1124_UART2, 1));
UBX_FRAME_DEFINE(enable_rtcm3_1127,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_UBX_RTCM_3X_TYPE1127_UART2, 1));

UBX_FRAME_DEFINE(enable_ubx_nav_svin,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_UBX_NAV_SVIN_UART1, 1));

UBX_FRAME_DEFINE(enable_tmode_survey_in,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_TMODE_MODE, 1));
UBX_FRAME_DEFINE(enable_tmode_pos_llh,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_TMODE_POS_TYPE, 1));
#endif

#ifdef CONFIG_GNSS_U_BLOX_X20P_RTK
UBX_FRAME_DEFINE(enable_ubx_nav_relposned,
	UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_UBX_NAV_RELPOSNED_UART1, 1));
#endif

UBX_FRAME_ARRAY_DEFINE(u_blox_x20p_init_seq,
	&disable_nmea_gga, &disable_nmea_rmc, &disable_nmea_gsv, &disable_nmea_dtm,
	&disable_nmea_gbs, &disable_nmea_gll, &disable_nmea_gns, &disable_nmea_grs,
	&disable_nmea_gsa, &disable_nmea_gst, &disable_nmea_vlw, &disable_nmea_vtg,
	&disable_nmea_zda, &enable_nav, &nav_fix_mode_auto,
	&enable_prot_in_ubx, &enable_prot_in_rtcm3, &enable_prot_out_ubx,
	&disable_prot_out_rtcm3_uart1, &set_rtk_fix_mode,
	&enable_prot_en_uart2, &enable_prot_in_rtcm3_uart2,
#if CONFIG_GNSS_SATELLITES
	&enable_sat,
#endif
	/* Antenna configuration */
	&enable_ant_voltctrl, &enable_ant_shortdet,
	&enable_ant_opendet, &enable_ant_pwrdown,
);

#if CONFIG_GNSS_U_BLOX_X20P_STATION
UBX_FRAME_ARRAY_DEFINE(u_blox_x20p_init_station_seq, &enable_prot_out_rtcm3_uart2,
		       &enable_rtcm3_1005, &enable_rtcm3_1074, &enable_rtcm3_1077,
		       &enable_rtcm3_1094,
		       &enable_rtcm3_1097, &enable_rtcm3_1124, &enable_rtcm3_1127,
		       &enable_ubx_nav_svin, &enable_tmode_survey_in,
		       &enable_tmode_pos_llh);
#endif

#ifdef CONFIG_GNSS_U_BLOX_X20P_RTK
UBX_FRAME_ARRAY_DEFINE(u_blox_x20p_init_rover_seq, &enable_ubx_nav_relposned, );
#endif

MODEM_UBX_MATCH_ARRAY_DEFINE(u_blox_x20p_unsol_messages,
	MODEM_UBX_MATCH_DEFINE(UBX_CLASS_ID_NAV, UBX_MSG_ID_NAV_PVT,
			       gnss_ubx_common_pvt_callback),
#if CONFIG_GNSS_SATELLITES
	MODEM_UBX_MATCH_DEFINE(UBX_CLASS_ID_NAV, UBX_MSG_ID_NAV_SAT,
			       gnss_ubx_common_satellite_callback),
#endif
#if CONFIG_GNSS_U_BLOX_X20P_STATION
	MODEM_UBX_MATCH_DEFINE(UBX_CLASS_ID_NAV, UBX_MSG_ID_NAV_SVIN,
			       x20p_ubx_svin_callback),
#endif
#ifdef CONFIG_GNSS_U_BLOX_X20P_RTK
	MODEM_UBX_MATCH_DEFINE(UBX_CLASS_ID_NAV, UBX_MSG_ID_NAV_RELPOSNED,
			       x20p_ubx_relposned_callback),
#endif
);

static int ubx_x20p_msg_get(const struct device *dev, const struct ubx_frame *req,
			  size_t len, void *rsp, size_t min_rsp_size)
{
	struct ubx_x20p_data *data = dev->data;
	struct ubx_frame *rsp_frame = (struct ubx_frame *)data->script.inst.response.buf;
	int err;

	err = k_sem_take(&data->script.lock, K_SECONDS(3));
	if (err != 0) {
		LOG_ERR("Failed to take script lock: %d", err);
		return err;
	}

	data->script.inst.timeout = K_SECONDS(3);
	data->script.inst.retry_count = 2;
	data->script.inst.match.filter.class = req->class;
	data->script.inst.match.filter.id = req->id;
	data->script.inst.request.buf = req;
	data->script.inst.request.len = len;

	err = modem_ubx_run_script(&data->ubx.inst, &data->script.inst);
	if (err != 0 || (data->script.inst.response.buf_len < UBX_FRAME_SZ(min_rsp_size))) {
		err = -EIO;
	} else {
		memcpy(rsp, rsp_frame->payload_and_checksum, min_rsp_size);
	}

	k_sem_give(&data->script.lock);

	return err;
}

static int ubx_x20p_msg_send(const struct device *dev, const struct ubx_frame *req,
			  size_t len, bool wait_for_ack)
{
	struct ubx_x20p_data *data = dev->data;
	int err;

	err = k_sem_take(&data->script.lock, K_SECONDS(3));
	if (err != 0) {
		LOG_ERR("Failed to take script lock: %d", err);
		return err;
	}

	data->script.inst.timeout = K_SECONDS(3);
	data->script.inst.retry_count = wait_for_ack ? 2 : 0;
	data->script.inst.match.filter.class = wait_for_ack ? UBX_CLASS_ID_ACK : 0;
	data->script.inst.match.filter.id = UBX_MSG_ID_ACK;
	data->script.inst.request.buf = (const void *)req;
	data->script.inst.request.len = len;

	err = modem_ubx_run_script(&data->ubx.inst, &data->script.inst);

	k_sem_give(&data->script.lock);

	return err;
}

static int ubx_x20p_msg_payload_send(const struct device *dev, uint8_t class, uint8_t id,
				   const uint8_t *payload, size_t payload_size, bool wait_for_ack)
{
	int err;
	struct ubx_x20p_data *data = dev->data;
	struct ubx_frame *frame = (struct ubx_frame *)data->script.request_buf;

	err = k_sem_take(&data->script.req_buf_lock, K_SECONDS(3));
	if (err != 0) {
		LOG_ERR("Failed to take script lock: %d", err);
		return err;
	}

	err = ubx_frame_encode(class, id, payload, payload_size,
			       (uint8_t *)frame, sizeof(data->script.request_buf));
	if (err > 0) {
		err = ubx_x20p_msg_send(dev, frame, err, wait_for_ack);
	}

	k_sem_give(&data->script.req_buf_lock);

	return err;
}

#if CONFIG_GNSS_U_BLOX_X20P_RTK

static void x20p_rtk_data_cb(const struct device *dev, const struct gnss_rtk_data *data)
{
	/** In this case, we forward the frame directly to the modem. It can either use
	 * it or not depending on the RTCM3 message type and its alignment with what the
	 * GNSS modem has observed.
	 */
	const struct ubx_x20p_config *cfg = dev->config;

	if (cfg->rtk_mode == UBX_X20P_RTK_MODE_ROVER) {
		(void)ubx_x20p_msg_send(dev, (const void *)data->data, data->len, false);
	}
}

#endif /* CONFIG_GNSS_U_BLOX_X20P_RTK */

static inline int init_modem(const struct device *dev)
{
	int err;
	struct ubx_x20p_data *data = dev->data;
	const struct ubx_x20p_config *cfg = dev->config;

	const struct modem_ubx_config ubx_config = {
		.user_data = (void *)&data->common_data,
		.receive_buf = data->ubx.receive_buf,
		.receive_buf_size = sizeof(data->ubx.receive_buf),
		.unsol_matches = {
			.array = u_blox_x20p_unsol_messages,
			.size = ARRAY_SIZE(u_blox_x20p_unsol_messages),
		},
	};

	const struct modem_backend_uart_config uart_backend_config = {
		.uart = cfg->bus,
		.receive_buf = data->backend.receive_buf,
		.receive_buf_size = sizeof(data->backend.receive_buf),
		.transmit_buf = data->backend.transmit_buf,
		.transmit_buf_size = sizeof(data->backend.transmit_buf),
	};

	(void)modem_ubx_init(&data->ubx.inst, &ubx_config);

	data->backend.pipe = modem_backend_uart_init(&data->backend.uart_backend,
						     &uart_backend_config);
	err = modem_pipe_open(data->backend.pipe, K_SECONDS(1));
	if (err != 0) {
		LOG_ERR("Failed to open Modem pipe: %d", err);
		return err;
	}

	err = modem_ubx_attach(&data->ubx.inst, data->backend.pipe);
	if (err != 0) {
		LOG_ERR("Failed to attach UBX inst to modem pipe: %d", err);
		return err;
	}

	(void)k_sem_init(&data->script.lock, 1, 1);
	(void)k_sem_init(&data->script.req_buf_lock, 1, 1);

	data->script.inst.response.buf = data->script.response_buf;
	data->script.inst.response.buf_len = sizeof(data->script.response_buf);

	return err;
}

static inline int init_match(const struct device *dev)
{
	struct ubx_x20p_data *data = dev->data;
	struct gnss_ubx_common_config match_config = {
		.gnss = dev,
#if CONFIG_GNSS_SATELLITES
		.satellites = {
			.buf = data->satellites,
			.size = ARRAY_SIZE(data->satellites),
		},
#endif
	};

	gnss_ubx_common_init(&data->common_data, &match_config);

	return 0;
}

static int ublox_x20p_init(const struct device *dev)
{
	int err = 0;
	const struct ubx_x20p_config *cfg = dev->config;

	const static struct ubx_frame version_get = UBX_FRAME_GET_INITIALIZER(
						UBX_CLASS_ID_MON,
						UBX_MSG_ID_MON_VER);
	struct ubx_mon_ver ver;

	(void)init_match(dev);

	err = init_modem(dev);
	if (err < 0) {
		LOG_ERR("Failed to initialize modem: %d", err);
	}

	if (cfg->reset) {
		const static struct ubx_frame reset_gnss =
			UBX_FRAME_CFG_RST_INITIALIZER(UBX_CFG_RST_COLD_START, UBX_CFG_RST_MODE_HW);

		err = ubx_x20p_msg_send(dev, &reset_gnss, UBX_FRAME_SZ(reset_gnss.payload_size),
				       false);
		if (err != 0) {
			LOG_ERR("Failed to reset GNSS module: %d", err);
			return err;
		}
	}

	err = ubx_x20p_msg_get(dev, &version_get,
			      UBX_FRAME_SZ(version_get.payload_size),
			      (void *)&ver, sizeof(ver));
	if (err != 0) {
		LOG_ERR("Failed to get Modem Version info: %d", err);
		return err;
	}
	LOG_INF("SW Version: %s, HW Version: %s", ver.sw_ver, ver.hw_ver);

	err = gnss_set_fix_rate(dev, cfg->fix_rate_ms);
	if (err != 0) {
		LOG_ERR("Failed to set fix-rate: %d", err);
		return err;
	}

	for (size_t i = 0 ; i < ARRAY_SIZE(u_blox_x20p_init_seq) ; i++) {
		err = ubx_x20p_msg_send(dev,
				       u_blox_x20p_init_seq[i],
				       UBX_FRAME_SZ(u_blox_x20p_init_seq[i]->payload_size),
				       true);
		if (err < 0) {
			LOG_ERR("Failed to send init sequence - idx: %d, result: %d", i, err);
			return err;
		}
	}

#ifdef CONFIG_GNSS_U_BLOX_X20P_STATION
	if (cfg->rtk_mode == UBX_X20P_RTK_MODE_STATION) {
		for (size_t i = 0; i < ARRAY_SIZE(u_blox_x20p_init_station_seq); i++) {
			err = ubx_x20p_msg_send(
				dev, u_blox_x20p_init_station_seq[i],
				UBX_FRAME_SZ(u_blox_x20p_init_station_seq[i]->payload_size), true);
			if (err < 0) {
				LOG_ERR("Failed to send station init sequence - idx: %d, result: "
					"%d",
					i, err);
				return err;
			}
		}

		err = ubx_x20p_msg_send(dev, cfg->svin_acc_lim,
				       UBX_FRAME_SZ(cfg->svin_acc_lim->payload_size), true);
		if (err < 0) {
			LOG_ERR("Failed to send station init sequence result: %d", err);
			return err;
		}

		err = ubx_x20p_msg_send(dev, cfg->svin_min_dur,
				       UBX_FRAME_SZ(cfg->svin_min_dur->payload_size), true);
		if (err < 0) {
			LOG_ERR("Failed to send station init sequence result: %d", err);
			return err;
		}
	}
#endif

#ifdef CONFIG_GNSS_U_BLOX_X20P_RTK
	if (cfg->rtk_mode == UBX_X20P_RTK_MODE_ROVER) {
		for (size_t i = 0; i < ARRAY_SIZE(u_blox_x20p_init_rover_seq); i++) {
			err = ubx_x20p_msg_send(
				dev, u_blox_x20p_init_rover_seq[i],
				UBX_FRAME_SZ(u_blox_x20p_init_rover_seq[i]->payload_size), true);
			if (err < 0) {
				LOG_ERR("Failed to send init sequence - idx: %d, result: %d", i,
					err);
				return err;
			}
		}
	}
#endif

	return 0;
}

static int ubx_x20p_set_fix_rate(const struct device *dev, uint32_t fix_interval_ms)
{
	struct ubx_cfg_val_u16 rate = {
		.hdr = {
			.ver = UBX_CFG_VAL_VER_SIMPLE,
			.layer = 1,
		},
		.key = UBX_KEY_RATE_MEAS,
		.value = fix_interval_ms,
	};

	if (fix_interval_ms < 50 || fix_interval_ms > 65535) {
		return -EINVAL;
	}

	return ubx_x20p_msg_payload_send(dev, UBX_CLASS_ID_CFG, UBX_MSG_ID_CFG_VAL_SET,
					(const uint8_t *)&rate, sizeof(rate),
					true);
}

static int ubx_x20p_get_fix_rate(const struct device *dev, uint32_t *fix_interval_ms)
{
	int err;
	struct ubx_cfg_val_u16 rate;

	const static struct ubx_frame get_fix_rate =
		UBX_FRAME_CFG_VAL_GET_INITIALIZER(UBX_KEY_RATE_MEAS);

	err = ubx_x20p_msg_get(dev, &get_fix_rate,
			      UBX_FRAME_SZ(get_fix_rate.payload_size),
			      (void *)&rate, sizeof(rate));
	if (err == 0) {
		*fix_interval_ms = rate.value;
	}

	return err;
}


/** As this GNSS modem may be used for many applications, the definition of
 * High Dynamics Navigation mode is configurable through Kconfig, in order to
 * maintain a balance between API re-usability and flexibility.
 */
#if CONFIG_GNSS_U_BLOX_X20P_NAV_MODE_HIGH_DYN_AIRBORNE_1G
#define GNSS_U_BLOX_X20P_NAV_MODE_HIGH_DYN	UBX_DYN_MODEL_AIRBORNE_1G
#elif CONFIG_GNSS_U_BLOX_X20P_NAV_MODE_HIGH_DYN_AIRBORNE_2G
#define GNSS_U_BLOX_X20P_NAV_MODE_HIGH_DYN	UBX_DYN_MODEL_AIRBORNE_2G
#elif CONFIG_GNSS_U_BLOX_X20P_NAV_MODE_HIGH_DYN_AIRBORNE_4G
#define GNSS_U_BLOX_X20P_NAV_MODE_HIGH_DYN	UBX_DYN_MODEL_AIRBORNE_4G
#elif CONFIG_GNSS_U_BLOX_X20P_NAV_MODE_HIGH_DYN_AUTOMOTIVE
#define GNSS_U_BLOX_X20P_NAV_MODE_HIGH_DYN	UBX_DYN_MODEL_AUTOMOTIVE
#elif CONFIG_GNSS_U_BLOX_X20P_NAV_MODE_HIGH_DYN_SEA
#define GNSS_U_BLOX_X20P_NAV_MODE_HIGH_DYN	UBX_DYN_MODEL_SEA
#elif CONFIG_GNSS_U_BLOX_X20P_NAV_MODE_HIGH_DYN_BIKE
#define GNSS_U_BLOX_X20P_NAV_MODE_HIGH_DYN	UBX_DYN_MODEL_BIKE
#endif

static int ubx_x20p_set_navigation_mode(const struct device *dev, enum gnss_navigation_mode mode)
{
	enum ubx_dyn_model nav_model;

	struct ubx_cfg_val_u8 rate = {
		.hdr = {
			.ver = UBX_CFG_VAL_VER_SIMPLE,
			.layer = 1,
		},
		.key = UBX_KEY_NAV_CFG_DYN_MODEL,
	};

	switch (mode) {
	case GNSS_NAVIGATION_MODE_ZERO_DYNAMICS:
		nav_model = UBX_DYN_MODEL_STATIONARY;
		break;
	case GNSS_NAVIGATION_MODE_LOW_DYNAMICS:
		nav_model = UBX_DYN_MODEL_PEDESTRIAN;
		break;
	case GNSS_NAVIGATION_MODE_BALANCED_DYNAMICS:
		nav_model = UBX_DYN_MODEL_PORTABLE;
		break;
	case GNSS_NAVIGATION_MODE_HIGH_DYNAMICS:
		nav_model = GNSS_U_BLOX_X20P_NAV_MODE_HIGH_DYN;
		break;
	default:
		return -EINVAL;
	}

	rate.value = nav_model;

	return ubx_x20p_msg_payload_send(dev, UBX_CLASS_ID_CFG, UBX_MSG_ID_CFG_VAL_SET,
					(const uint8_t *)&rate, sizeof(rate),
					true);
}

static int ubx_x20p_get_navigation_mode(const struct device *dev, enum gnss_navigation_mode *mode)
{
	int err;
	struct ubx_cfg_val_u8 nav_mode;

	const static struct ubx_frame get_nav_mode =
		UBX_FRAME_CFG_VAL_GET_INITIALIZER(UBX_KEY_NAV_CFG_DYN_MODEL);

	err = ubx_x20p_msg_get(dev, &get_nav_mode,
			      UBX_FRAME_SZ(get_nav_mode.payload_size),
			      (void *)&nav_mode, sizeof(nav_mode));
	switch (nav_mode.value) {
	case UBX_DYN_MODEL_STATIONARY:
		*mode = GNSS_NAVIGATION_MODE_ZERO_DYNAMICS;
		break;
	case UBX_DYN_MODEL_PEDESTRIAN:
		*mode = GNSS_NAVIGATION_MODE_LOW_DYNAMICS;
		break;
	case UBX_DYN_MODEL_PORTABLE:
		*mode = GNSS_NAVIGATION_MODE_BALANCED_DYNAMICS;
		break;
	case UBX_DYN_MODEL_AIRBORNE_1G:
	case UBX_DYN_MODEL_AIRBORNE_2G:
	case UBX_DYN_MODEL_AIRBORNE_4G:
	case UBX_DYN_MODEL_AUTOMOTIVE:
	case UBX_DYN_MODEL_SEA:
	case UBX_DYN_MODEL_BIKE:
		*mode = GNSS_NAVIGATION_MODE_HIGH_DYNAMICS;
		break;
	default:
		return -EIO;
	}

	return err;
}

static int ubx_x20p_set_enabled_systems(const struct device *dev, gnss_systems_t systems)
{
	return -ENOTSUP;
}

static int ubx_x20p_get_enabled_systems(const struct device *dev, gnss_systems_t *systems)
{
	static const struct ubx_frame get_enabled_systems = UBX_FRAME_GET_INITIALIZER(
									UBX_CLASS_ID_MON,
									UBX_MSG_ID_MON_GNSS);
	struct ubx_mon_gnss gnss_selection;
	int err;

	err = ubx_x20p_msg_get(dev, &get_enabled_systems,
			      UBX_FRAME_SZ(get_enabled_systems.payload_size),
			      (void *)&gnss_selection, sizeof(gnss_selection));
	if (err != 0) {
		return err;
	}

	*systems = 0;
	*systems |= (gnss_selection.selection.enabled & UBX_GNSS_SELECTION_GPS) ?
			GNSS_SYSTEM_GPS : 0;
	*systems |= (gnss_selection.selection.enabled & UBX_GNSS_SELECTION_GLONASS) ?
			GNSS_SYSTEM_GLONASS : 0;
	*systems |= (gnss_selection.selection.enabled & UBX_GNSS_SELECTION_BEIDOU) ?
			GNSS_SYSTEM_BEIDOU : 0;
	*systems |= (gnss_selection.selection.enabled & UBX_GNSS_SELECTION_GALILEO) ?
			GNSS_SYSTEM_GALILEO : 0;

	return 0;
}

static int ubx_x20p_get_supported_systems(const struct device *dev, gnss_systems_t *systems)
{
	static const struct ubx_frame get_enabled_systems = UBX_FRAME_GET_INITIALIZER(
									UBX_CLASS_ID_MON,
									UBX_MSG_ID_MON_GNSS);
	struct ubx_mon_gnss gnss_selection;
	int err;

	err = ubx_x20p_msg_get(dev, &get_enabled_systems,
			      UBX_FRAME_SZ(get_enabled_systems.payload_size),
			      (void *)&gnss_selection, sizeof(gnss_selection));
	if (err != 0) {
		return err;
	}

	*systems = 0;
	*systems |= (gnss_selection.selection.supported & UBX_GNSS_SELECTION_GPS) ?
			GNSS_SYSTEM_GPS : 0;
	*systems |= (gnss_selection.selection.supported & UBX_GNSS_SELECTION_GLONASS) ?
			GNSS_SYSTEM_GLONASS : 0;
	*systems |= (gnss_selection.selection.supported & UBX_GNSS_SELECTION_BEIDOU) ?
			GNSS_SYSTEM_BEIDOU : 0;
	*systems |= (gnss_selection.selection.supported & UBX_GNSS_SELECTION_GALILEO) ?
			GNSS_SYSTEM_GALILEO : 0;

	return 0;
}


static DEVICE_API(gnss, ublox_x20p_driver_api) = {
	.set_fix_rate = ubx_x20p_set_fix_rate,
	.get_fix_rate = ubx_x20p_get_fix_rate,
	.set_navigation_mode = ubx_x20p_set_navigation_mode,
	.get_navigation_mode = ubx_x20p_get_navigation_mode,
	.set_enabled_systems = ubx_x20p_set_enabled_systems,
	.get_enabled_systems = ubx_x20p_get_enabled_systems,
	.get_supported_systems = ubx_x20p_get_supported_systems,
};


#define UBX_X20P(inst)										   \
												   \
	BUILD_ASSERT((DT_INST_PROP(inst, fix_rate) >= 50) &&					   \
		     (DT_INST_PROP(inst, fix_rate) < 65536),					   \
		     "Invalid fix-rate. Please set it higher than 50-ms"			   \
		     " and must fit in 16-bits.");						   \
												   \
	IF_ENABLED(CONFIG_GNSS_U_BLOX_X20P_STATION,						   \
	(UBX_FRAME_DEFINE(set_tmode_svin_acc_lim_##inst,					   \
			 UBX_FRAME_CFG_VAL_SET_U32_INITIALIZER(					   \
				 UBX_KEY_TMODE_SVIN_ACC_LIM,					   \
				 DT_INST_PROP_OR(inst, svin_acc_lim, 5000) * 10));))		   \
												   \
	IF_ENABLED(CONFIG_GNSS_U_BLOX_X20P_STATION,						   \
	(UBX_FRAME_DEFINE(									   \
		set_tmode_svin_min_dur_##inst,							   \
		UBX_FRAME_CFG_VAL_SET_U32_INITIALIZER(UBX_KEY_TMODE_SVIN_MIN_DUR,		   \
						      DT_INST_PROP_OR(inst, svin_min_dur, 60)));)) \
												   \
	static const struct ubx_x20p_config ubx_x20p_cfg_##inst = {				   \
		.bus = DEVICE_DT_GET(DT_INST_BUS(inst)),					   \
		.fix_rate_ms = DT_INST_PROP(inst, fix_rate),					   \
		.reset = (bool)DT_INST_PROP(inst, reset),					   \
	IF_ENABLED(CONFIG_GNSS_U_BLOX_X20P_RTK,							   \
		(.rtk_mode = DT_INST_ENUM_IDX_OR(inst, rtk_mode, UBX_X20P_RTK_MODE_NONE),))	   \
	IF_ENABLED(CONFIG_GNSS_U_BLOX_X20P_STATION,						   \
		(.svin_acc_lim = &set_tmode_svin_acc_lim_##inst,))				   \
	IF_ENABLED(CONFIG_GNSS_U_BLOX_X20P_STATION,						   \
		(.svin_min_dur = &set_tmode_svin_min_dur_##inst,))				   \
	};											   \
												   \
	static struct ubx_x20p_data ubx_x20p_data_##inst;						   \
												   \
	IF_ENABLED(CONFIG_GNSS_U_BLOX_X20P_RTK,							   \
		   (GNSS_DT_RTK_DATA_CALLBACK_DEFINE(DT_DRV_INST(inst), x20p_rtk_data_cb)));	   \
												   \
	DEVICE_DT_INST_DEFINE(inst,								   \
			      ublox_x20p_init,							   \
			      NULL,								   \
			      &ubx_x20p_data_##inst,						   \
			      &ubx_x20p_cfg_##inst,						   \
			      POST_KERNEL, CONFIG_GNSS_INIT_PRIORITY,				   \
			      &ublox_x20p_driver_api);

DT_INST_FOREACH_STATUS_OKAY(UBX_X20P)
