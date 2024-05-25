/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT ti_tmag5253

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#include "tmag5253.h"

#define LOG_LEVEL CONFIG_SENSOR_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sensor_tmag5253, LOG_LEVEL);

static int tmag5253_attr_set(const struct device *dev, enum sensor_channel chan,
                             enum sensor_attribute attr, const struct sensor_value *val) {
    struct tmag5253_data *const data = dev->data;

    switch ((uint32_t)attr) {
    case SENSOR_ATTR_TMAG_CHANNEL:
        k_sem_take(&data->lock, K_FOREVER);
        data->active_enable_idx = val->val1;
        k_sem_give(&data->lock);
        break;
    }

    return 0;
}

static int tmag5253_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct tmag5253_data *const data = dev->data;
    const struct tmag5253_config *config = dev->config;

    k_sem_take(&data->lock, K_FOREVER);

    int err = 0;

    if (chan == SENSOR_CHAN_MAGN_Z || chan == SENSOR_CHAN_ALL) {
        struct gpio_dt_spec *enable_gpio = &config->en_gpios[data->active_enable_idx];
        err = gpio_pin_set_dt(enable_gpio, 1);
        if (err < 0) {
            LOG_ERR("Failed to enable TMAG5253: %d", data->active_enable_idx);
        }

	k_usleep(config->enable_timeout);

        err = adc_read_dt(&config->adc, &data->sequence);
        if (err < 0) {
            LOG_ERR("Failed to read ADC channel %d", config->adc.channel_id);
        }

	LOG_INF("Raw voltage: %d", data->raw_voltage);

        err = gpio_pin_set_dt(enable_gpio, 0);
        if (err < 0) {
            LOG_ERR("Failed to disable TMAG5253: %d", data->active_enable_idx);
        }
    }

    k_sem_give(&data->lock);
    return err;
}

static int tmag5253_channel_get(const struct device *dev, enum sensor_channel chan,
                                struct sensor_value *val) {
    int err = 0;

    struct tmag5253_data *const data = dev->data;
    k_sem_take(&data->lock, K_FOREVER);

    switch (chan) {
	case SENSOR_CHAN_MAGN_Z:
	val->val1 = data->raw_voltage;
	val->val2 = 0;
	break;
    default:
        err = -ENOTSUP;
        break;
    }

    k_sem_give(&data->lock);
    return err;
}

static int tmag5253_init(const struct device *dev) {
    struct tmag5253_data *data = dev->data;
    const struct tmag5253_config *config = dev->config;

    struct adc_dt_spec adc_channel = config->adc;

    LOG_INF("Enable gpios: %d", config->en_gpios_len);

    if (adc_is_ready_dt(&adc_channel) == false) {
        LOG_ERR("ADC controller devivce %s not ready", adc_channel.dev->name);
        return 0;
    }

    int err;

    err = adc_channel_setup_dt(&adc_channel);
    if (err < 0) {
        LOG_ERR("Failed to setup ADC channel: err %d", err);
        return 0;
    }

    data->sequence = (struct adc_sequence){
        .buffer = &data->raw_voltage,
        .buffer_size = sizeof(data->raw_voltage),
        .calibrate = true,
    };

    err = adc_sequence_init_dt(&adc_channel, &data->sequence);
    if (err < 0) {
        LOG_ERR("Failed to initialise ADC sequence for channel %d", adc_channel.channel_id);
        return 0;
    }

    for (int i = 0; i < config->en_gpios_len; i++) {
        err = gpio_pin_configure_dt(&config->en_gpios[i], GPIO_OUTPUT_LOW);
        if (err < 0) {
            LOG_ERR("Failed to configure enable index %d", i);
            return err;
        }
    }

    // the functions below need the semaphore, so initialise it here
    k_sem_init(&data->lock, 1, 1);

    return 0;
}

static const struct sensor_driver_api tmag5253_api_table = {.attr_set = tmag5253_attr_set,
                                                            .sample_fetch = tmag5253_sample_fetch,
                                                            .channel_get = tmag5253_channel_get};

#define TMAG5253_INIT(inst)                                                                        \
    static struct gpio_dt_spec enable_specs_##inst[] = {                                           \
        DT_INST_FOREACH_PROP_ELEM_SEP(inst, enable_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};         \
    static struct tmag5253_config tmag5253_##inst##_config = {                                     \
        .vdd = DT_INST_PROP(inst, vdd),                                                            \
        .b_sensitivity = DT_INST_PROP(inst, b_sensitivity),                                        \
	.enable_timeout = DT_INST_PROP(inst, enable_timeout),                                      \
        .adc = ADC_DT_SPEC_INST_GET(inst),                                                         \
        .en_gpios = enable_specs_##inst,                                                           \
        .en_gpios_len = ARRAY_SIZE(enable_specs_##inst)};                                          \
    static struct tmag5253_data tmag5253_driver_data##inst;                                        \
                                                                                                   \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, tmag5253_init, NULL, &tmag5253_driver_data##inst,           \
                                 &tmag5253_##inst##_config, POST_KERNEL,                           \
                                 CONFIG_SENSOR_INIT_PRIORITY, &tmag5253_api_table);

DT_INST_FOREACH_STATUS_OKAY(TMAG5253_INIT)
