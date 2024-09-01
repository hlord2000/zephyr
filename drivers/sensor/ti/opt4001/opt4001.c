/*
 * Copyright (c) 2024 Kelly Helmut Lord
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "opt4001.h"

LOG_MODULE_REGISTER(opt4001, CONFIG_SENSOR_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(ti_opt4001ymn)
#define DT_DRV_COMPAT ti_opt4001ymn
#define ADC_CODE_SCALE 3125
#elif DT_HAS_COMPAT_STATUS_OKAY(ti_opt4001dts)
#define DT_DRV_COMPAT ti_opt4001dts
#define ADC_CODE_SCALE 4375
#else
#error OPT4001: unsupported devicetree compatible
#endif

static int opt4001_reg_read(const struct device *dev, uint8_t reg,
			    uint16_t *val) {
	int err;
	const struct opt4001_config *config = dev->config;
	uint8_t buf[2] = {0};

	err = i2c_burst_read_dt(&config->i2c, reg, buf, 2);
	if (err < 0) {
		LOG_ERR("Failed to read register 0x%02x: %d", reg, err);
		return err;
	}

	*val = (buf[0] << 8) | buf[1];

	return 0;
}

static int opt4001_reg_write(const struct device *dev, uint8_t reg,
			     uint16_t val) {
	int err;
	const struct opt4001_config *config = dev->config;
	uint8_t buf[2] = {val >> 8, val & 0xFF};

	err = i2c_burst_write_dt(&config->i2c, reg, buf, 2);
	if (err < 0) {
		LOG_ERR("Failed to write register 0x%02x: %d", reg, err);
		return err;
	}
	return 0;
}

static int opt4001_reg_update(const struct device *dev, uint8_t reg,
			      uint16_t mask, uint16_t val) {
	uint16_t reg_val;
	opt4001_reg_read(dev, reg, &reg_val);

	reg_val = (reg_val & ~mask) | (val & mask);

	return opt4001_reg_write(dev, reg, reg_val);
}

static int opt4001_sample_fetch(const struct device *dev,
				enum sensor_channel chan) {
	int err;
	struct opt4001_data *data = dev->data;

	uint8_t register_results[4] = {0};

	err = opt4001_reg_read(dev, OPT4001_REG_RESULT_LSB, &register_results[0]);
	if (err < 0) {
		LOG_ERR("Failed to read sample: %d", err);
		return err;
	}

	err = opt4001_reg_read(dev, OPT4001_REG_RESULT_MSB, &register_results[2]);
	if (err < 0) {
		LOG_ERR("Failed to read sample: %d", err);
		return err;
	}
	LOG_HEXDUMP_INF(register_results, sizeof(register_results), "OPT4001 sample");

	/* Last byte is CRC + counter, dropped here */
	data->mantissa = ((((register_results[3] << 8) & 0x0FFF) | register_results[2]) << 8)
				+ register_results[1];
	data->exponent = register_results[3] >> 4;

	return 0;
}

static int opt4001_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val) {
	if (chan != SENSOR_CHAN_LIGHT) {
		return -ENOTSUP;
	}

	struct opt4001_data *data = dev->data;
	uint64_t adc_codes = data->mantissa << data->exponent;

	uint64_t lux_scaled = (adc_codes * ADC_CODE_SCALE);

	val->val1 = lux_scaled / 10000000;
	val->val2 = (lux_scaled % 10000000) / 10;

	return 0;
}

static const struct sensor_driver_api opt4001_driver_api = {
	.sample_fetch = opt4001_sample_fetch,
	.channel_get = opt4001_channel_get,
};

int opt4001_init(const struct device *dev) {
	int err;

	uint16_t device_id;
	err = opt4001_reg_read(dev, OPT4001_REG_DEVICE_ID, &device_id);
	if (err < 0) {
		LOG_ERR("Failed to read device ID: %d", err);
		return err;
	}

	if (device_id != OPT4001_DEVICE_ID_VALUE) {
		LOG_ERR("Invalid device ID: 0x%04x", device_id);
		return -EIO;
	}

	err = opt4001_reg_update(dev, OPT4001_REG_CONFIG, 0x30, 0x30);
	if (err < 0) {
		LOG_ERR("Failed to configure device: %d", err);
		return err;
	}

	return 0;
}

#define OPT4001_DEFINE(inst)									\
	static struct opt4001_data opt4001_data_##inst;						\
												\
	static const struct opt4001_config opt4001_config_##inst = {				\
		.i2c = I2C_DT_SPEC_INST_GET(inst),						\
	};											\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, opt4001_init, NULL,					\
			      &opt4001_data_##inst, &opt4001_config_##inst, POST_KERNEL,	\
			      CONFIG_SENSOR_INIT_PRIORITY, &opt4001_driver_api);		\

DT_INST_FOREACH_STATUS_OKAY(OPT4001_DEFINE)
