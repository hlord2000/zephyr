/*
 * Copyright (c) 2023 Kelly Helmut Lord
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_at42qt2120

#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#include "at42qt2120.h"

LOG_MODULE_REGISTER(AT42QT2120, CONFIG_SENSOR_LOG_LEVEL);

static int at42qt2120_reset(const struct device *dev)
{
	const struct at42qt2120_config *cfg = dev->config;
	int ret;

	ret = i2c_reg_write_byte_dt(&cfg->i2c, AT42QT2120_REG_RESET, AT42QT2120_RESET_VALUE);

	k_msleep(300);

	return ret;
}

static int at42qt2120_get_slider_position(const struct at42qt2120_config *cfg, struct at42qt2120_data *data) {
	int ret;

	uint8_t status;
	/* All four status registers must be read to clear the CHANGE pin */	
	ret = i2c_reg_read_byte_dt(&cfg->i2c, AT42QT2120_REG_DETECTION_STATUS, &status);
	if (ret) {
		LOG_ERR("Could not read detection status %d", ret);
	}

	for (int i = 0; i < 2; i++) {
		ret = i2c_reg_read_byte_dt(&cfg->i2c, AT42QT2120_REG_KEY_STATUS_BASE + i, &status);
		if (ret) {
			LOG_ERR("Could not read key status %d", ret);
		}
	}

	ret = i2c_reg_read_byte_dt(&cfg->i2c, AT42QT2120_REG_SLIDER_POSITION, &data->rotation);
	if (ret) {
		LOG_ERR("Could not read slider position %d", ret);
	}

	return ret;
}

#ifdef CONFIG_PM_DEVICE
static int at42qt2120_set_power_mode(const struct device *dev, uint8_t power_mode) {
	const struct at42qt2120_config *cfg = dev->config;
	int ret;

	ret = i2c_reg_write_byte_dt(&cfg->i2c, AT42QT2120_REG_LOW_POWER, power_mode);
	return ret;
}

static int at42qt2120_device_pm_action(const struct device *dev,
				    enum pm_device_action action)
{
	int ret;
	const struct at42qt2120_config *cfg = dev->config;
	enum pm_device_state curr_state;

	(void)pm_device_state_get(dev, &curr_state);

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:

		ret = at42qt2120_set_power_mode(dev, cfg->acquisition_time);
		if (ret) {
			return ret;
		}

		break;
	case PM_DEVICE_ACTION_SUSPEND:
		ret = at42qt2120_set_power_mode(dev, AT42QT2120_LOW_POWER_VALUE);
		if (ret) {
			return ret;
		}

		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}
#endif

static int at42qt2120_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct at42qt2120_data *data = dev->data;
	const struct at42qt2120_config *cfg = dev->config;
	int ret;
#ifdef CONFIG_PM_DEVICE
	enum pm_device_state state;

	(void)pm_device_state_get(dev, &state);
	if (state != PM_DEVICE_STATE_ACTIVE) {
		LOG_ERR("Sample fetch failed, device is not in active mode");
		return -ENXIO;
	}
#endif
	switch (chan) {
		case SENSOR_CHAN_ALL:
		case SENSOR_CHAN_ROTATION:
			ret = at42qt2120_get_slider_position(cfg, data);
			break;
		default:
			ret = -ENOTSUP;
			break;
	}

	return ret;
}

static int at42qt2120_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val) {
	struct at42qt2120_data *data = dev->data;
	switch (chan) {
	case SENSOR_CHAN_ROTATION:
		val->val1 = ((int32_t)data->rotation * 360)/255;
		break;
	default:
		LOG_ERR("Channel type not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api at42qt2120_api_funcs = {
	.sample_fetch = at42qt2120_sample_fetch,
	.channel_get = at42qt2120_channel_get,
};

static void at42qt2120_work_handler(struct k_work *work) {
	int ret;

	struct at42qt2120_data *data = CONTAINER_OF(work, struct at42qt2120_data, work);

	ret = at42qt2120_sample_fetch(data->dev, SENSOR_CHAN_ROTATION);
	if (ret) {
		LOG_ERR("Could not fetch sample %d", ret);
		return;
	}
}

static void at42qt2120_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
	struct at42qt2120_data *data = CONTAINER_OF(cb, struct at42qt2120_data, gpio_cb);

	k_work_submit(&data->work);
}

static int at42qt2120_init_interrupt(const struct device *dev) {

	struct at42qt2120_data *data = dev->data;
	const struct at42qt2120_config *cfg = dev->config;
	int ret;

	ret = gpio_is_ready_dt(&cfg->change_gpio);
	if (!ret) {
		LOG_ERR("GPIO port %s not ready", cfg->change_gpio.port->name);
		return -ENODEV;
	}

	data->dev = dev;

	ret = gpio_pin_configure_dt(&cfg->change_gpio, GPIO_INPUT);
	if (ret) {
		LOG_ERR("Failed to configure gpio %s pin %d", cfg->change_gpio.port->name, cfg->change_gpio.pin);
		return ret;
	}

	gpio_init_callback(&data->gpio_cb, at42qt2120_gpio_callback, BIT(cfg->change_gpio.pin));

	ret = gpio_add_callback(cfg->change_gpio.port, &data->gpio_cb);
	if (ret) {
		LOG_ERR("Failed to add gpio callback");
		return ret;
	}

	data->work.handler = at42qt2120_work_handler; 

	uint8_t res;
	i2c_reg_read_byte_dt(&cfg->i2c, AT42QT2120_REG_DETECTION_STATUS, &res);

	ret = gpio_pin_interrupt_configure_dt(&cfg->change_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		LOG_ERR("Failed to configure gpio interrupt");
		return ret;
	}

	return ret;
}

static int at42qt2120_calibrate(const struct device *dev) {

	const struct at42qt2120_config *cfg = dev->config;
	int ret = 1;

	ret = i2c_reg_write_byte_dt(&cfg->i2c, AT42QT2120_REG_CALIBRATE, AT42QT2120_CALIBRATE_VALUE);
	if (ret) {
		LOG_ERR("Could not start calibration %d", ret);
		return ret;
	}

	uint8_t calibrate_status;

	while (true) {
		ret = i2c_reg_read_byte_dt(&cfg->i2c, AT42QT2120_REG_DETECTION_STATUS, &calibrate_status);
		if (ret) {
			LOG_ERR("Could not read calibration status %d", ret);
			return ret;
		}
		if (AT42QT2120_CALIBRATE_GET(calibrate_status) == 0) {
			break;
		}
	}

	k_msleep(300);

	return ret;
}

static int at42qt2120_init_config(const struct device *dev) {
	const struct at42qt2120_config *cfg = dev->config;
	int ret;

	ret = i2c_reg_write_byte_dt(&cfg->i2c, AT42QT2120_REG_ACQUISITION_TIME, cfg->acquisition_time);
	if (ret) {
		LOG_ERR("Could not set acquisition time %d", ret);
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, AT42QT2120_REG_TOWARD_TOUCH_DRIFT_COMPENSATION, cfg->toward_touch_drift);
	if (ret) {
		LOG_ERR("Could not set toward touch drift %d", ret);
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, AT42QT2120_REG_AWAY_TOUCH_DRIFT_COMPENSATION, cfg->away_touch_drift);
	if (ret) {
		LOG_ERR("Could not set away touch drift %d", ret);
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, AT42QT2120_REG_DETECTION_INTEGRATOR, cfg->detection_integrator);
	if (ret) {
		LOG_ERR("Could not set detection integrator %d", ret);
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, AT42QT2120_REG_TOUCH_RECALIBRATION_DELAY, cfg->touch_recal_delay);
	if (ret) {
		LOG_ERR("Could not set touch recal delay %d", ret);
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, AT42QT2120_REG_DRIFT_HOLD_TIME, cfg->drift_hold_time);
	if (ret) {
		LOG_ERR("Could not set drift hold time %d", ret);
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, AT42QT2120_REG_CHARGE_TIME, cfg->charge_time);
	if (ret) {
		LOG_ERR("Could not set charge time %d", ret);
		return ret;
	}

	for (int i = 3; i < 12; i++) {
		ret = i2c_reg_write_byte_dt(&cfg->i2c, AT42QT2120_REG_KEY_CONTROL_BASE + i, AT42QT2120_KEY_CONTROL_DISABLE);
		if (ret) {
			LOG_ERR("Could not enable key %d %d", i, ret);
			return ret;
		}
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, AT42QT2120_REG_SLIDER_OPTIONS, AT42QT2120_SLIDER_ENABLE_MSK);
	if (ret) {
		LOG_ERR("Could not enable slider %d", ret);
		return ret;
	}

	return ret;
}

static int at42qt2120_init(const struct device *dev)
{
	const struct at42qt2120_config *cfg = dev->config;
	struct at42qt2120_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}

	ret = at42qt2120_reset(dev);
	if (ret) {
		LOG_ERR("Could not reset device %d", ret);
		return ret;
	}

	ret = at42qt2120_calibrate(dev);
	if (ret) {
		LOG_ERR("Could not calibrate device %d", ret);
		return ret;
	}

	ret = at42qt2120_init_config(dev);
	if (ret) {
		LOG_ERR("Could not initialize device %d", ret);
		return ret;
	}
	
	k_work_init(&data->work, at42qt2120_work_handler);

	if (at42qt2120_init_interrupt(dev) < 0) {
		LOG_ERR("Failed to initialize interrupt!");
		return -EIO;
	}

#ifdef CONFIG_PM_DEVICE

	ret = pm_device_runtime_enable(dev);
	if (ret){ 
		LOG_ERR("Could not enable runtime power management %d", ret);
		return ret;
	}
#endif

	return 0;
}

#define AT42QT2120_CHANGE_PROPS(n)                                      \
	.change_gpio = GPIO_DT_SPEC_INST_GET(n, change_gpios),              \

#define AT42QT2120_CHANGE(n)				                            \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(n, change_gpios),                  \
		   (AT42QT2120_CHANGE_PROPS(n)))

#define AT42QT2120_INIT(n)							                    \
	static const struct at42qt2120_config at42qt2120_config_##n = {	    \
		.i2c = I2C_DT_SPEC_INST_GET(n),				                    \
		.acquisition_time = DT_INST_PROP(n, acquisition_time),	        \
		.toward_touch_drift = DT_INST_PROP(n, toward_touch_drift),	    \
		.away_touch_drift = DT_INST_PROP(n, away_touch_drift),	        \
		.detection_integrator = DT_INST_PROP(n, detection_integrator),  \
		.touch_recal_delay = DT_INST_PROP(n, touch_recal_delay),	    \
		.drift_hold_time = DT_INST_PROP(n, drift_hold_time),	        \
		.charge_time = DT_INST_PROP(n, charge_time),		            \
		AT42QT2120_CHANGE(n)				                            \
	};								                                    \
																		\
	static struct at42qt2120_data at42qt2120_data_##n = {               \
		.rotation = 0,                                                  \
	};																	\
                                 									    \
	PM_DEVICE_DT_INST_DEFINE(n, at42qt2120_device_pm_action);		    \
									                                    \
	SENSOR_DEVICE_DT_INST_DEFINE(n,					                    \
			      at42qt2120_init,				                        \
			      PM_DEVICE_DT_INST_GET(n),			                    \
			      &at42qt2120_data_##n,			                        \
			      &at42qt2120_config_##n,			                    \
			      POST_KERNEL,				                            \
			      CONFIG_SENSOR_INIT_PRIORITY,		                    \
			      &at42qt2120_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(AT42QT2120_INIT)
