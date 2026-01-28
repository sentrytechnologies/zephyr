/*
 * Copyright (c) 2025 Sentry Technologies ApS
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * sch16t_regs.h — SCH16T register map and bit definitions
 *
 * Sources: SCH16T datasheet
 * Register map overview and blocks, filter, range, decimation, user IF, self-test, mode/reset, IDs,
 * and status registers.
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_SCH16T_REGS_H_
#define ZEPHYR_DRIVERS_SENSOR_SCH16T_REGS_H_

#include <zephyr/sys/util.h>

#define SCH16T_BOOT_TIME_MS   250
#define SCH16T_INIT_TIME_MS   32
#define SCH16T_ENABLE_TIME_MS 215

/* Public register addresses (8-bit TA[7:0]) */
/* Bank0: 0x00–0x0F — Sensor data (read-only) */
#define SCH16T_REG_RATE_X1 0x01
#define SCH16T_REG_RATE_Y1 0x02
#define SCH16T_REG_RATE_Z1 0x03
#define SCH16T_REG_ACC_X1  0x04
#define SCH16T_REG_ACC_Y1  0x05
#define SCH16T_REG_ACC_Z1  0x06
#define SCH16T_REG_ACC_X3  0x07
#define SCH16T_REG_ACC_Y3  0x08
#define SCH16T_REG_ACC_Z3  0x09
#define SCH16T_REG_RATE_X2 0x0A
#define SCH16T_REG_RATE_Y2 0x0B
#define SCH16T_REG_RATE_Z2 0x0C
#define SCH16T_REG_ACC_X2  0x0D
#define SCH16T_REG_ACC_Y2  0x0E
#define SCH16T_REG_ACC_Z2  0x0F

/* Bank1: 0x10–0x1F — Temperature, counters, status */
#define SCH16T_REG_TEMP             0x10
#define SCH16T_REG_RATE_DCNT        0x11
#define SCH16T_REG_ACC_DCNT         0x12
#define SCH16T_REG_FREQ_CNTR        0x13
#define SCH16T_REG_STAT_SUM         0x14
#define SCH16T_REG_STAT_SUM_SAT     0x15
#define SCH16T_REG_STAT_COM         0x16
#define SCH16T_REG_STAT_RATE_COM    0x17
#define SCH16T_REG_STAT_RATE_X      0x18
#define SCH16T_REG_STAT_RATE_Y      0x19
#define SCH16T_REG_STAT_RATE_Z      0x1A
#define SCH16T_REG_STAT_ACC_X       0x1B
#define SCH16T_REG_STAT_ACC_Y       0x1C
#define SCH16T_REG_STAT_ACC_Z       0x1D
#define SCH16T_REG_STAT_SYNC_ACTIVE 0x1E
#define SCH16T_REG_STAT_INFO        0x1F

/* Bank2: 0x20–0x2F — Filters, ranges, decimation */
#define SCH16T_REG_CTRL_FILT_RATE  0x25
#define SCH16T_REG_CTRL_FILT_ACC12 0x26
#define SCH16T_REG_CTRL_FILT_ACC3  0x27
#define SCH16T_REG_CTRL_RATE       0x28
#define SCH16T_REG_CTRL_ACC12      0x29
#define SCH16T_REG_CTRL_ACC3       0x2A

/* Bank3: 0x30–0x3F — User IF, test, mode/reset, IDs */
#define SCH16T_REG_CTRL_USER_IF 0x33
#define SCH16T_REG_CTRL_ST      0x34
#define SCH16T_REG_CTRL_MODE    0x35
#define SCH16T_REG_CTRL_RESET   0x36
#define SCH16T_REG_SYS_TEST     0x37
#define SCH16T_REG_SPARE_1      0x38
#define SCH16T_REG_SPARE_2      0x39
#define SCH16T_REG_SPARE_3      0x3A
#define SCH16T_REG_ASIC_ID      0x3B
#define SCH16T_REG_COMP_ID      0x3C
#define SCH16T_REG_SN_ID1       0x3D
#define SCH16T_REG_SN_ID2       0x3E
#define SCH16T_REG_SN_ID3       0x3F

/* Mode and reset */
#define SCH16T_MODE_EOI_CTRL  BIT(1)
#define SCH16T_MODE_EN_SENSOR BIT(0)

#endif /* ZEPHYR_DRIVERS_SENSOR_SCH16T_REGS_H_ */
