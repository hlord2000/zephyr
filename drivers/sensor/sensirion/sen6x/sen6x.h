/*
 * Copyright (c) 2025 Kelly Lord
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_SEN6X_H_
#define ZEPHYR_DRIVERS_SENSOR_SEN6X_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#define SEN6X_MAX_REGISTER_LEN 48
#define SEN6X_READ_WIDTH_BYTES 3
#define SEN6X_MAX_READINGS_LEN 18

#if DT_HAS_COMPAT_STATUS_OKAY(sensirion_sen60)
#define SEN60_CMD_START_MEASUREMENT  0x2152
#define SEN60_CMD_STOP_MEASUREMENT   0x3F86
#define SEN60_CMD_GET_DATA_READY     0xE4B8
#define SEN60_CMD_READ_MEASURED_VALS 0xEC05
#define SEN60_CMD_GET_SERIAL_NUM     0x3682
#define SEN60_CMD_READ_DEVICE_STATUS 0xE00B
#define SEN60_CMD_DEVICE_RESET       0x3F8D
#define SEN60_CMD_START_FAN_CLEANING 0x3730

#define SEN60_READ_MEASURED_VALS_EXEC_TIME K_MSEC(1)
#define SEN60_NUM_MEASURED_VALS 9
#define SEN60_PM_1_0_IDX 0x0
#define SEN60_PM_2_5_IDX 0x1
#define SEN60_PM_4_0_IDX 0x2
#define SEN60_PM_10_IDX  0x3

#endif

#if DT_HAS_COMPAT_STATUS_OKAY(sensirion_sen63c)
#define SEN63C_CMD_READ_MEASURED_VALS 0x0471
#define SEN63C_CMD_READ_MEASURED_RAW  0x0492
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(sensirion_sen65)
#define SEN65_CMD_READ_MEASURED_VALS 0x0446
#define SEN65_CMD_READ_MEASURED_RAW  0x0455
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(sensirion_sen66)
#define SEN66_CMD_READ_MEASURED_VALS 0x0300
#define SEN66_CMD_READ_MEASURED_RAW  0x0405
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(sensirion_sen68)
#define SEN68_CMD_READ_MEASURED_VALS 0x0467
#define SEN68_CMD_READ_MEASURED_RAW  0x0455
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(sensirion_sen65) || DT_HAS_COMPAT_STATUS_OKAY(sensirion_sen66) || DT_HAS_COMPAT_STATUS_OKAY(sensirion_sen68)
#define SEN6X_REG_VOC_TUNING_PARAMS        0x60D0
#define SEN6X_REG_VOC_ALGO_STATE           0x6181
#define SEN6X_REG_NOX_TUNING_PARAMS        0x60E1
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(sensirion_sen63c) || DT_HAS_COMPAT_STATUS_OKAY(sensirion_sen66)
#define SEN6X_CMD_FORCE_CO2_RECAL          0x6707
#define SEN6X_REG_CO2_SENSOR_AUTO_CAL      0x6711
#define SEN6X_REG_AMBIENT_PRESSURE         0x6720
#define SEN6X_REG_SENSOR_ALTITUDE          0x6736
#endif

#define SEN6X_CMD_START_MEASUREMENT        0x0021
#define SEN6X_CMD_STOP_MEASUREMENT         0x0104
#define SEN6X_CMD_GET_DATA_READY           0x0202
#define SEN6X_CMD_READ_CONC_VALS           0x0316
#define SEN6X_CMD_SET_TEMP_OFFSET_PARAMS   0x60B2
#define SEN6X_CMD_SET_TEMP_ACCEL_PARAMS    0x6100
#define SEN6X_CMD_GET_PRODUCT_NAME         0xD014
#define SEN6X_CMD_GET_SERIAL_NUM           0xD033
#define SEN6X_CMD_READ_DEVICE_STATUS       0xD206
#define SEN6X_CMD_READ_CLEAR_DEVICE_STATUS 0xD210
#define SEN6X_CMD_DEVICE_RESET             0xD304
#define SEN6X_CMD_START_FAN_CLEANING       0x5607
#define SEN6X_CMD_START_SHT_HEATER         0x6765

/*
 * CRC parameters were taken from the
 * "Checksum Calculation" section of the datasheet.
 */
#define SEN6X_CRC_POLY		0x31
#define SEN6X_CRC_INIT		0xFF

#define SEN6X_POR_WAIT_MS       100

enum sensor_variant {
	DEVICE_VARIANT_SEN60 =  0,
	DEVICE_VARIANT_SEN63C = 1,
	DEVICE_VARIANT_SEN65 =  2,
	DEVICE_VARIANT_SEN66 =  3,
	DEVICE_VARIANT_SEN68 =  4,
};

struct sen6x_config {
	struct i2c_dt_spec bus;
	enum sensor_variant variant;
};

struct sen6x_data {
	uint16_t pm_1_sample;
	uint16_t pm_2_5_sample;
	uint16_t pm_4_sample;
	uint16_t pm_10_sample;
	int16_t rh_sample;
	int16_t t_sample;
	int16_t voc_index_sample;
	int16_t nox_index_sample;
	uint16_t co2_sample;
	uint16_t ch2o_sample;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_SEN6X_H_ */
