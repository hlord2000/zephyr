/*
 * Copyright (c) 2024 Kelly Helmut Lord
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_OPT4001_H_
#define ZEPHYR_DRIVERS_SENSOR_OPT4001_H_

#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>

#define OPT4001_REG_RESULT_MSB       0x00
#define OPT4001_REG_RESULT_LSB       0x01
#define OPT4001_REG_RESULT_MSB_FIFO0 0x02
#define OPT4001_REG_RESULT_LSB_FIFO0 0x03
#define OPT4001_REG_RESULT_MSB_FIFO1 0x04
#define OPT4001_REG_RESULT_LSB_FIFO1 0x05
#define OPT4001_REG_RESULT_MSB_FIFO2 0x06
#define OPT4001_REG_RESULT_LSB_FIFO2 0x07
#define OPT4001_REG_THRESHOLD_LOW    0x08
#define OPT4001_REG_THRESHOLD_HIGH   0x09
#define OPT4001_REG_CONFIG           0x0A
#define OPT4001_REG_INT_CONFIG       0x0B
#define OPT4001_REG_FLAG             0x0C
#define OPT4001_REG_DEVICE_ID        0x11

#define OPT4001_DEVICE_ID_VALUE 0x0121

struct opt4001_data {
	uint8_t exponent;
	uint32_t mantissa;
};

struct opt4001_config {
	struct i2c_dt_spec i2c;
};

#endif /* _SENSOR_OPT4001_ */
