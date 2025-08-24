/*
 * Copyright (c) 2025 Kelly Lord
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#include <zephyr/drivers/sensor/sen6x.h>
#include "sen6x.h"

LOG_MODULE_REGISTER(SEN6X, CONFIG_SENSOR_LOG_LEVEL);

static uint8_t sen6x_compute_crc(uint16_t value)
{
	uint8_t buf[2];

	sys_put_be16(value, buf);

	return crc8(buf, 2, SEN6X_CRC_POLY, SEN6X_CRC_INIT, false);
}

static int sen6x_send_command(const struct device *dev, uint16_t cmd_id)
{
	const struct sen6x_config *cfg = dev->config;
	uint8_t tx_buf[3] = {(cmd_id & 0xFF), ((cmd_id >> 8) & 0xFF)};

	tx_buf[2] = sen6x_compute_crc(cmd_id);

	return i2c_write_dt(&cfg->bus, tx_buf, sizeof(tx_buf));
}

static int sen6x_write_command(const struct device *dev, uint16_t cmd_id, uint16_t cmd)
{
	const struct sen6x_config *cfg = dev->config;
	uint8_t tx_buf[5] = {(cmd_id & 0xFF), ((cmd_id >> 8) & 0xFF), (cmd & 0xFF),
			     ((cmd >> 8) & 0xFF)};

	tx_buf[4] = sen6x_compute_crc(cmd);

	return i2c_write_dt(&cfg->bus, tx_buf, sizeof(tx_buf));
}

static int sen6x_read_command(const struct device *dev, uint16_t cmd_id, k_timeout_t execution_time, uint16_t *readings, size_t num_readings)
{
	const struct sen6x_config *cfg = dev->config;
	uint8_t tx_buf[2] = {(cmd_id & 0xFF), ((cmd_id >> 8) & 0xFF)};
	uint8_t rx_buf[SEN6X_MAX_REGISTER_LEN];
	int ret;

	ret = i2c_write_dt(&cfg->bus, tx_buf, sizeof(tx_buf));
	if (ret < 0) {
		LOG_ERR("Failed to initiate read command.");
		return ret;
	}

	k_sleep(execution_time);

	ret = i2c_read_dt(&cfg->bus, rx_buf, num_readings * SEN6X_READ_WIDTH_BYTES);
	if (ret < 0) {
		LOG_ERR("Failed to read sensor.");
		return ret;
	}

	uint16_t *reg_val;
	for (int i = 0; i < num_readings; i++) {
		reg_val = &rx_buf[i * SEN6X_READ_WIDTH_BYTES];
		readings[i] = sys_get_be16(reg_val);
		if (sen6x_compute_crc(reg_val) != rx_buf[(i * SEN6X_READ_WIDTH_BYTES) + 2]) {
			LOG_ERR("Invalid CRC for read command.");
			return -EIO;
		}
	}

	return 0;
}

static int sen6x_read_sample_particulate_matter(const struct device *dev)
{
	const struct sen6x_config *cfg = dev->config;
	struct sen6x_data *data = dev->data;
	int ret;
	uint16_t readings[SEN6X_MAX_READINGS_LEN];

}

static int sen6x_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct sen6x_config *cfg = dev->config;
	struct sen6x_data *data = dev->data;
	int ret;

	uint16_t readings[SEN6X_MAX_READINGS_LEN];

	switch (cfg->variant) {
	case DEVICE_VARIANT_SEN60:
		ret = sen6x_read_command(dev, SEN60_CMD_READ_MEASURED_VALS, SEN60_READ_MEASURED_VALS_EXEC_TIME, readings, SEN60_NUM_MEASURED_VALS);
		if (ret < 0) {
			return ret;
		}

	case DEVICE_VARIANT_SEN63C:
	case DEVICE_VARIANT_SEN65:
	case DEVICE_VARIANT_SEN66:
	case DEVICE_VARIANT_SEN68:
	default:
		return -EINVAL;
	}

	return 0;
}

static int sen6x_channel_get(const struct device *dev, enum sensor_channel chan,
			     struct sensor_value *val)
{
	const struct sen6x_data *data = dev->data;

	return 0;
}

static int sen6x_attr_set(const struct device *dev, enum sensor_channel chan,
			  enum sensor_attribute attr, const struct sensor_value *val)
{
	struct sen6x_data *data = dev->data;

	if (val->val1 < 0) {
		return -EINVAL;
	}

	switch ((enum sensor_attribute_sen6x)attr) {
	case SENSOR_ATTR_SHT4X_HEATER_POWER:
		if (val->val1 > SHT4X_HEATER_POWER_IDX_MAX) {
			return -EINVAL;
		}
		data->heater_power = val->val1;
		break;
	case SENSOR_ATTR_SHT4X_HEATER_DURATION:
		if (val->val1 > SHT4X_HEATER_DURATION_IDX_MAX) {
			return -EINVAL;
		}
		data->heater_duration = val->val1;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int sen6x_init_chip(const struct device *dev)
{
	int ret;

	/* 1 ms (max) power up time according to datasheet */
	k_sleep(K_MSEC(SEN6X_POR_WAIT_MS));

	ret = sen6x_write_command(dev, SHT4X_CMD_RESET);
	if (ret < 0) {
		LOG_ERR("Failed to reset the device.");
		return ret;
	}

	k_sleep(K_MSEC(SHT4X_RESET_WAIT_MS));
	return 0;
}

static int sen6x_pm_action(const struct device *dev, enum pm_device_action action)
{
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_TURN_ON:
		ret = sen6x_init_chip(dev);
		break;

	case PM_DEVICE_ACTION_RESUME:
	case PM_DEVICE_ACTION_SUSPEND:
	case PM_DEVICE_ACTION_TURN_OFF:
		break;

	default:
		return -ENOTSUP;
	}

	return ret;
}

static int sen6x_init(const struct device *dev)
{
	const struct sen6x_config *cfg = dev->config;

	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("Device not ready.");
		return -ENODEV;
	}

	return pm_device_driver_init(dev, sen6x_pm_action);
}

static DEVICE_API(sensor, sen6x_api) = {
	.sample_fetch = sen6x_sample_fetch,
	.channel_get = sen6x_channel_get,
	.attr_set = sen6x_attr_set,
	.attr_get = sen6x_attr_set,
};

#define SEN6X_DEFINE(n, compat, variant_enum)                                                      \
	static struct sen6x_data sen6x_data_##n;                                                   \
                                                                                                   \
	static const struct sen6x_config sen6x_config_##n = {.bus = I2C_DT_SPEC_INST_GET(n),       \
							     .variant = variant_enum};             \
	PM_DEVICE_DT_INST_DEFINE(n, sen6x_pm_action);                                              \
	SENSOR_DEVICE_DT_INST_DEFINE(n, sen6x_init, PM_DEVICE_DT_INST_GET(n), &sen6x_data_##n,     \
				     &sen6x_config_##n, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,  \
				     &sen6x_api);

#define SEN6X_DEFINE(n, compat, variant_enum)				\
	static struct sen6x_data sen6x_data_##n;		\
								\
	static const struct sen6x_config sen6x_config_##n = {	\
		.bus = I2C_DT_SPEC_INST_GET(n),			\
		.variant = variant_enum				\
	};							\
	PM_DEVICE_DT_INST_DEFINE(n, sen6x_pm_action);	\
	SENSOR_DEVICE_DT_INST_DEFINE(n,				\
				sen6x_init,			\
				PM_DEVICE_DT_INST_GET(n),		\
				&sen6x_data_##n,		\
				&sen6x_config_##n,		\
				POST_KERNEL,			\
				CONFIG_SENSOR_INIT_PRIORITY,	\
				&sen6x_api);

#define DT_DRV_COMPAT sensirion_sen60
DT_INST_FOREACH_STATUS_OKAY_VARGS(SEN6X_DEFINE, DT_DRV_COMPAT, DEVICE_VARIANT_SEN60)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT sensirion_sen63c
DT_INST_FOREACH_STATUS_OKAY_VARGS(SEN6X_DEFINE, DT_DRV_COMPAT, DEVICE_VARIANT_SEN63C)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT sensirion_sen65
DT_INST_FOREACH_STATUS_OKAY_VARGS(SEN6X_DEFINE, DT_DRV_COMPAT, DEVICE_VARIANT_SEN65)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT sensirion_sen66
DT_INST_FOREACH_STATUS_OKAY_VARGS(SEN6X_DEFINE, DT_DRV_COMPAT, DEVICE_VARIANT_SEN66)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT sensirion_sen68
DT_INST_FOREACH_STATUS_OKAY_VARGS(SEN6X_DEFINE, DT_DRV_COMPAT, DEVICE_VARIANT_SEN68)
#undef DT_DRV_COMPAT
