/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Based on “WIT Standard Communication Protocol” specification.
 * See WIT "Standard Communication Protocol.pdf"
 * (https://drive.google.com/file/d/1xrfK9bAEncgFQYjvT_c6vwSEH0ZhzaUZ)
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_WITMOTION_WIT_PROTOCOL_H_
#define ZEPHYR_DRIVERS_SENSOR_WITMOTION_WIT_PROTOCOL_H_

#include <zephyr/sys/util.h>

/* ------------------------------------------------------------------------- */
/* Frame and command constants                                               */
/* ------------------------------------------------------------------------- */

/* Data write length */
#define WIT_DATA_WRITE_LEN		5

/* Size of a measurement */
#define WIT_MEASU_READ_LEN		11

/* Data frame header byte */
#define WIT_HDR_START			0x55

/* Command frame header bytes */
#define WIT_CMD_HEAD1			0xFF
#define WIT_CMD_HEAD2			0xAA

/* ------------------------------------------------------------------------- */
/* Register addresses                                                        */
/* ------------------------------------------------------------------------- */
enum wit_register {
	WIT_REG_SAVE           = 0x00, /* Save/Reboot/Factory reset */
	WIT_REG_CALSW          = 0x01, /* Calibration mode */
	WIT_REG_RSW            = 0x02, /* Output content mask */
	WIT_REG_RRATE          = 0x03, /* Output rate */
	WIT_REG_BAUD           = 0x04, /* Serial baud rate */
	WIT_REG_AXOFFSET       = 0x05, /* Accel X zero offset */
	WIT_REG_AYOFFSET       = 0x06, /* Accel Y zero offset */
	WIT_REG_AZOFFSET       = 0x07, /* Accel Z zero offset */
	WIT_REG_GXOFFSET       = 0x08, /* Gyro X zero offset */
	WIT_REG_GYOFFSET       = 0x09, /* Gyro Y zero offset */
	WIT_REG_GZOFFSET       = 0x0A, /* Gyro Z zero offset */
	WIT_REG_HXOFFSET       = 0x0B, /* Mag X zero offset */
	WIT_REG_HYOFFSET       = 0x0C, /* Mag Y zero offset */
	WIT_REG_HZOFFSET       = 0x0D, /* Mag Z zero offset */
	WIT_REG_D0MODE         = 0x0E, /* D0 pin mode */
	WIT_REG_D1MODE         = 0x0F, /* D1 pin mode */
	WIT_REG_D2MODE         = 0x10, /* D2 pin mode */
	WIT_REG_D3MODE         = 0x11, /* D3 pin mode */
	WIT_REG_IICADDR        = 0x1A, /* I²C/Modbus address */
	WIT_REG_LEDOFF         = 0x1B, /* LED off control */
	WIT_REG_MAGRANGX       = 0x1C, /* Mag calibration range X */
	WIT_REG_MAGRANGY       = 0x1D, /* Mag calibration range Y */
	WIT_REG_MAGRANGZ       = 0x1E, /* Mag calibration range Z */
	WIT_REG_BANDWIDTH      = 0x1F, /* Filter bandwidth */
	WIT_REG_GYRORANGE      = 0x20, /* Gyro range */
	WIT_REG_ACCRANGE       = 0x21, /* Accel range */
	WIT_REG_SLEEP          = 0x22, /* Sleep control */
	WIT_REG_ORIENT         = 0x23, /* Install orientation */
	WIT_REG_AXIS6          = 0x24, /* 6/9-axis select */
	WIT_REG_FILTK          = 0x25, /* Filter K-value */
	WIT_REG_GPSBAUD        = 0x26, /* GPS baud rate */
	WIT_REG_READADDR       = 0x27, /* Read-register command */
	WIT_REG_ACCFILT        = 0x2A, /* Accel filter */
	WIT_REG_POWONSEND      = 0x2D, /* Power-on data output */
	WIT_REG_VERSION        = 0x2E, /* Firmware version */
	WIT_REG_YYMM           = 0x30, /* Time: year/month */
	WIT_REG_DDHH           = 0x31, /* Time: day/hour */
	WIT_REG_MMSS           = 0x32, /* Time: minute/second */
	WIT_REG_MS             = 0x33, /* Time: millisecond */
	WIT_REG_AX             = 0x34, /* Accel X data */
	WIT_REG_AY             = 0x35, /* Accel Y data */
	WIT_REG_AZ             = 0x36, /* Accel Z data */
	WIT_REG_GX             = 0x37, /* Gyro X data */
	WIT_REG_GY             = 0x38, /* Gyro Y data */
	WIT_REG_GZ             = 0x39, /* Gyro Z data */
	WIT_REG_HX             = 0x3A, /* Mag X data */
	WIT_REG_HY             = 0x3B, /* Mag Y data */
	WIT_REG_HZ             = 0x3C, /* Mag Z data */
	WIT_REG_ROLL           = 0x3D, /* Roll angle */
	WIT_REG_PITCH          = 0x3E, /* Pitch angle */
	WIT_REG_YAW            = 0x3F, /* Yaw angle */
	WIT_REG_TEMP           = 0x40, /* Temperature */
	WIT_REG_D0STATUS       = 0x41, /* D0 status */
	WIT_REG_D1STATUS       = 0x42, /* D1 status */
	WIT_REG_D2STATUS       = 0x43, /* D2 status */
	WIT_REG_D3STATUS       = 0x44, /* D3 status */
	WIT_REG_PRESSUREL      = 0x45, /* Pressure low byte */
	WIT_REG_PRESSUREH      = 0x46, /* Pressure high byte */
	WIT_REG_HEIGHTL        = 0x47, /* Altitude low byte */
	WIT_REG_HEIGHTH        = 0x48, /* Altitude high byte */
	WIT_REG_LONL           = 0x49, /* Longitude low word */
	WIT_REG_LONH           = 0x4A, /* Longitude high word */
	WIT_REG_LATL           = 0x4B, /* Latitude low word */
	WIT_REG_LATH           = 0x4C, /* Latitude high word */
	WIT_REG_GPSHEIGHT      = 0x4D, /* GPS height */
	WIT_REG_GPSYAW         = 0x4E, /* GPS heading */
	WIT_REG_GPSVL          = 0x4F, /* GPS velocity low byte */
	WIT_REG_GPSVH          = 0x50, /* GPS velocity high byte */
	WIT_REG_Q0             = 0x51, /* Quaternion q0 */
	WIT_REG_Q1             = 0x52, /* Quaternion q1 */
	WIT_REG_Q2             = 0x53, /* Quaternion q2 */
	WIT_REG_Q3             = 0x54, /* Quaternion q3 */
	WIT_REG_SVNUM          = 0x55, /* GPS satellite count */
	WIT_REG_PDOP           = 0x56, /* Position DOP */
	WIT_REG_HDOP           = 0x57, /* Horizontal DOP */
	WIT_REG_VDOP           = 0x58, /* Vertical DOP */
	WIT_REG_DELAYT         = 0x59, /* Trigger delay time */
	WIT_REG_XMIN           = 0x5A, /* Calibration X min */
	WIT_REG_XMAX           = 0x5B, /* Calibration X max */
	WIT_REG_BATVAL         = 0x5C, /* Battery voltage */
	WIT_REG_ALARMPIN       = 0x5D, /* Alarm output pin */
	WIT_REG_YMIN           = 0x5E, /* Calibration Y min */
	WIT_REG_YMAX           = 0x5F, /* Calibration Y max */
	WIT_REG_GYROCALITHR    = 0x61, /* Gyro calibration threshold */
	WIT_REG_ALARMLEVEL     = 0x62, /* Alarm level */
	WIT_REG_GYROCALTIME    = 0x63, /* Gyro calibration time */
	WIT_REG_TRIGTIME       = 0x68, /* Trigger time */
	WIT_REG_KEY            = 0x69, /* Unlock key register */
	WIT_REG_WERROR         = 0x6A, /* Warning/error code */
	WIT_REG_TIMEZONE       = 0x6B, /* GPS time zone */
	WIT_REG_WZTIME         = 0x6E, /* Wake zone time */
	WIT_REG_WZSTATIC       = 0x6F, /* Wake zone static */
	WIT_REG_MODDELAY       = 0x74, /* Motion detection delay */
	WIT_REG_XREFROLL       = 0x79, /* Reference roll */
	WIT_REG_YREFPITCH      = 0x7A, /* Reference pitch */
	WIT_REG_NUMBERID1      = 0x7F, /* Custom ID byte 1 */
	WIT_REG_NUMBERID2      = 0x80, /* Custom ID byte 2 */
	WIT_REG_NUMBERID3      = 0x81, /* Custom ID byte 3 */
	WIT_REG_NUMBERID4      = 0x82, /* Custom ID byte 4 */
	WIT_REG_NUMBERID5      = 0x83, /* Custom ID byte 5 */
	WIT_REG_NUMBERID6      = 0x84, /* Custom ID byte 6 */
};

/* ------------------------------------------------------------------------- */
/* Data-type codes in data frames                                            */
/* ------------------------------------------------------------------------- */
enum wit_data_type {
	WIT_TYPE_TIME     = 0x50, /* Time: YY MM DD HH MN SS MS */
	WIT_TYPE_ACCEL    = 0x51, /* Accel: Ax Ay Az Temp */
	WIT_TYPE_GYRO     = 0x52, /* Gyro: Wx Wy Wz Volt */
	WIT_TYPE_ANGLE    = 0x53, /* Angle: Roll Pitch Yaw Ver */
	WIT_TYPE_MAG      = 0x54, /* Mag: Hx Hy Hz Temp */
	WIT_TYPE_PORT     = 0x55, /* Port: D0..D3 */
	WIT_TYPE_PRESS    = 0x56, /* Pressure/Altitude */
	WIT_TYPE_GPS      = 0x57, /* Lon/Lat */
	WIT_TYPE_GPS_EX   = 0x58, /* GPS: Height Yaw Vel */
	WIT_TYPE_QUAT     = 0x59, /* Quaternion */
	WIT_TYPE_GPS_ACC  = 0x5A, /* GPS accuracy (DOP/SV) */
	WIT_TYPE_READ_REG = 0x5F, /* Read-register reply */
};

#define WIT_TYPE_OFFSET			WIT_TYPE_TIME
#define WIT_TYPE_START			WIT_TYPE_TIME
#define WIT_TYPE_END			WIT_TYPE_READ_REG
#define WIT_TYPE_COUNT			(WIT_TYPE_END - WIT_TYPE_START + 1)

/* ------------------------------------------------------------------------- */
/* RSW (Output content) bitmasks                                             */
/* ------------------------------------------------------------------------- */
enum wit_rsw_mask {
	WIT_RSW_TIME     = BIT(0),  /* TIME      (0x50) */
	WIT_RSW_ACC      = BIT(1),  /* ACCEL     (0x51) */
	WIT_RSW_GYRO     = BIT(2),  /* GYRO      (0x52) */
	WIT_RSW_ANGLE    = BIT(3),  /* ANGLE     (0x53) */
	WIT_RSW_MAG      = BIT(4),  /* MAG       (0x54) */
	WIT_RSW_PORT     = BIT(5),  /* PORT      (0x55) */
	WIT_RSW_PRESS    = BIT(6),  /* PRESS     (0x56) */
	WIT_RSW_GPS      = BIT(7),  /* GPS       (0x57) */
	WIT_RSW_VELOCITY = BIT(8),  /* VELOCITY  (0x58) */
	WIT_RSW_QUAT     = BIT(9),  /* QUAT      (0x59) */
	WIT_RSW_GPS_ACC  = BIT(10), /* GPS_ACC   (0x5A) */
};

/* ------------------------------------------------------------------------- */
/* Output rate (RRATE) values                                                */
/* ------------------------------------------------------------------------- */
enum wit_output_rate {
	WIT_RRATE_0_2HZ      = 0x01,
	WIT_RRATE_0_5HZ      = 0x02,
	WIT_RRATE_1HZ        = 0x03,
	WIT_RRATE_2HZ        = 0x04,
	WIT_RRATE_5HZ        = 0x05,
	WIT_RRATE_10HZ       = 0x06,
	WIT_RRATE_20HZ       = 0x07,
	WIT_RRATE_50HZ       = 0x08,
	WIT_RRATE_100HZ      = 0x09,
	WIT_RRATE_200HZ      = 0x0B,
	WIT_RRATE_SINGLE     = 0x0C, /* Single return */
	WIT_RRATE_NONE       = 0x0D, /* No return */
};

/* ------------------------------------------------------------------------- */
/* Serial baud-rate (BAUD) values                                            */
/* ------------------------------------------------------------------------- */
enum wit_baud_rate {
	WIT_BAUD_4800     = 0x01,
	WIT_BAUD_9600     = 0x02,
	WIT_BAUD_19200    = 0x03,
	WIT_BAUD_38400    = 0x04,
	WIT_BAUD_57600    = 0x05,
	WIT_BAUD_115200   = 0x06,
	WIT_BAUD_230400   = 0x07,
	WIT_BAUD_460800   = 0x08, /* Extended devices */
	WIT_BAUD_921600   = 0x09, /* Extended devices */
};

/* ------------------------------------------------------------------------- */
/* Calibration mode (CALSW)                                                  */
/* ------------------------------------------------------------------------- */
enum wit_calibration_mode {
	WIT_CAL_NORMAL         = 0x00,
	WIT_CAL_ACCEL_AUTO     = 0x01,
	WIT_CAL_HEIGHT_RESET   = 0x03,
	WIT_CAL_HEADING_ZERO   = 0x04,
	WIT_CAL_MAG_SPHERE     = 0x07,
	WIT_CAL_REF_ANGLE      = 0x08,
	WIT_CAL_MAG_DUAL_PLANE = 0x09,
};

/* ------------------------------------------------------------------------- */
/* Port modes (D0MODE-D3MODE)                                                */
/* ------------------------------------------------------------------------- */
enum wit_port_mode {
	WIT_PORT_ANALOG_IN     = 0x0, /* Analog input */
	WIT_PORT_DIGITAL_IN    = 0x1,
	WIT_PORT_DIGITAL_HI    = 0x2,
	WIT_PORT_DIGITAL_LO    = 0x3,
	WIT_PORT_RELATIVE_POSE = 0x5, /* D1 only */
};

/* ------------------------------------------------------------------------- */
/* Accelerometer range (ACCRANGE)                                            */
/* ------------------------------------------------------------------------- */
enum wit_acc_range {
	WIT_ACC_2G  = 0x0,
	WIT_ACC_16G = 0x3, /* Auto-switch above 2 g */
};

/* ------------------------------------------------------------------------- */
/* Gyro range (GYRORANGE)                                                    */
/* ------------------------------------------------------------------------- */
#define WIT_GYRO_RANGE_2000DPS  0x3U /* Fixed +/- 2000 deg/s */

/* ------------------------------------------------------------------------- */
/* Filter bandwidth (BANDWIDTH)                                              */
/* ------------------------------------------------------------------------- */
enum wit_bandwidth {
	WIT_BW_256HZ = 0x0,
	WIT_BW_188HZ = 0x1,
	WIT_BW_98HZ  = 0x2,
	WIT_BW_42HZ  = 0x3,
	WIT_BW_20HZ  = 0x4,
	WIT_BW_10HZ  = 0x5,
	WIT_BW_5HZ   = 0x6,
};

/* ------------------------------------------------------------------------- */
/* Sleep control                                                             */
/* ------------------------------------------------------------------------- */
enum wit_sleep {
	WIT_SLEEP_HIBERNATE = 0x0,
	WIT_SLEEP_SLEEP     = 0x1,
};

/* ------------------------------------------------------------------------- */
/* Orientation                                                               */
/* ------------------------------------------------------------------------- */
enum wit_orient {
	WIT_ORIENT_HORIZONTAL = 0x0,
	WIT_ORIENT_VERTICAL   = 0x1,
};

/* ------------------------------------------------------------------------- */
/* Axis mode select                                                          */
/* ------------------------------------------------------------------------- */
enum wit_axis6 {
	WIT_AXIS6_9AXIS = 0x0,
	WIT_AXIS6_6AXIS = 0x1,
};

/* ------------------------------------------------------------------------- */
/* Power-on send                                                             */
/* ------------------------------------------------------------------------- */
enum wit_powonsend {
	WIT_POW_ON_DISABLED = 0x0,
	WIT_POW_ON_ENABLED  = 0x1,
};

/* ------------------------------------------------------------------------- */
/* LED control                                                               */
/* ------------------------------------------------------------------------- */
enum wit_led {
	WIT_LED_ON  = 0x0,
	WIT_LED_OFF = 0x1,
};

/* ------------------------------------------------------------------------- */
/* Alarm level                                                               */
/* ------------------------------------------------------------------------- */
enum wit_alarm_level {
	WIT_ALARM_LEVEL_LOW  = 0x0,
	WIT_ALARM_LEVEL_HIGH = 0x1,
};

/* ------------------------------------------------------------------------- */
/* GPS time zones                                                            */
/* ------------------------------------------------------------------------- */
enum wit_timezone {
	WIT_TZ_UTC_MINUS_12 = 0x00,
	WIT_TZ_UTC_MINUS_11 = 0x01,
	WIT_TZ_UTC_MINUS_10 = 0x02,
	WIT_TZ_UTC_MINUS_9  = 0x03,
	WIT_TZ_UTC_MINUS_8  = 0x04,
	WIT_TZ_UTC_MINUS_7  = 0x05,
	WIT_TZ_UTC_MINUS_6  = 0x06,
	WIT_TZ_UTC_MINUS_5  = 0x07,
	WIT_TZ_UTC_MINUS_4  = 0x08,
	WIT_TZ_UTC_MINUS_3  = 0x09,
	WIT_TZ_UTC_MINUS_2  = 0x0A,
	WIT_TZ_UTC_MINUS_1  = 0x0B,
	WIT_TZ_UTC          = 0x0C,
	WIT_TZ_UTC_PLUS_1   = 0x0D,
	WIT_TZ_UTC_PLUS_2   = 0x0E,
	WIT_TZ_UTC_PLUS_3   = 0x0F,
	WIT_TZ_UTC_PLUS_4   = 0x10,
	WIT_TZ_UTC_PLUS_5   = 0x11,
	WIT_TZ_UTC_PLUS_6   = 0x12,
	WIT_TZ_UTC_PLUS_7   = 0x13,
	WIT_TZ_UTC_PLUS_8   = 0x14, /* Default */
	WIT_TZ_UTC_PLUS_9   = 0x15,
	WIT_TZ_UTC_PLUS_10  = 0x16,
	WIT_TZ_UTC_PLUS_11  = 0x17,
	WIT_TZ_UTC_PLUS_12  = 0x18,
};

#endif /* ZEPHYR_DRIVERS_SENSOR_WITMOTION_WIT_PROTOCOL_H_ */
