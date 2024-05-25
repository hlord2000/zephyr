/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

enum sensor_attribute_tmag5253 {
    SENSOR_ATTR_TMAG_CHANNEL = SENSOR_ATTR_PRIV_START,
};

struct tmag5253_config {
    struct adc_dt_spec adc;
    struct gpio_dt_spec *en_gpios;
    size_t en_gpios_len;

    uint16_t vdd;
    uint16_t b_sensitivity;
    uint16_t enable_timeout;
};

struct tmag5253_data {
    struct k_sem lock;

    struct adc_sequence sequence;
    uint8_t active_enable_idx;
    uint16_t raw_voltage;
};

#ifdef __cplusplus
}
#endif
