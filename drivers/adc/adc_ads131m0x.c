/*
 * Copyright (c) 2023 Kelly Helmut Lord
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

LOG_MODULE_REGISTER(ADC_ADS131M08, CONFIG_ADC_LOG_LEVEL);

#ifdef CONFIG_ADC_ASYNC
BUILD_ASSERT(DT_NODE_HAS_PROP(DT_DRV_INST(0), drdy_gpios), \
		"Async mode requires drdy-gpios to trigger interrupt");
#endif

enum ads131m0x_reg {
	ADS131M0X_REG_ID = 0x00,
	ADS131M0X_REG_STATUS = 0x01,
	ADS131M0X_REG_MODE = 0x02,
	ADS131M0X_REG_CLOCK = 0x03,
#ifdef DT_HAS_COMPAT_STATUS_OKAY(ti_ads131m06) || DT_HAS_COMPAT_STATUS_OKAY(ti_ads131m08)
	ADS131M0X_REG_GAIN_1 = 0x04,
	ADS131M0X_REG_GAIN_2 = 0x05,
#else
	ADS131M0X_REG_GAIN = 0x04,
#endif
	ADS131M0X_REG_CFG = 0x06,
	ADS131M0X_REG_THRSHLD_MSB = 0x07,
	ADS131M0X_REG_THRSHLD_LSB = 0x08,
	ADS131M0X_REG_CH0_CFG = 0x09,
	ADS131M0X_REG_CH0_OCAL_MSB = 0x0a,
	ADS131M0X_REG_CH0_OCAL_LSB = 0x0b,
	ADS131M0X_REG_CH0_GCAL_MSB = 0x0c,
	ADS131M0X_REG_CH0_GCAL_LSB = 0x0d,
	ADS131M0X_REG_CH1_CFG = 0x0e,
	ADS131M0X_REG_CH1_OCAL_MSB = 0x0f,
	ADS131M0X_REG_CH1_OCAL_LSB = 0x10,
	ADS131M0X_REG_CH1_GCAL_MSB = 0x11,
	ADS131M0X_REG_CH1_GCAL_LSB = 0x12,
#ifdef DT_HAS_COMPAT_STATUS_OKAY(ti_ads131m03) || DT_HAS_COMPAT_STATUS_OKAY(ti_ads131m04) || \
	DT_HAS_COMPAT_STATUS_OKAY(ti_ads131m06) || DT_HAS_COMPAT_STATUS_OKAY(ti_ads131m08)
	ADS131M0X_REG_CH2_CFG = 0x13,
	ADS131M0X_REG_CH2_OCAL_MSB = 0x14,
	ADS131M0X_REG_CH2_OCAL_LSB = 0x15,
	ADS131M0X_REG_CH2_GCAL_MSB = 0x16,
	ADS131M0X_REG_CH2_GCAL_LSB = 0x17,
#endif
#ifdef DT_HAS_COMPAT_STATUS_OKAY(ti_ads131m04) || DT_HAS_COMPAT_STATUS_OKAY(ti_ads131m06) || \
	DT_HAS_COMPAT_STATUS_OKAY(ti_ads131m08)
	ADS131M0X_REG_CH3_CFG = 0x18,
	ADS131M0X_REG_CH3_OCAL_MSB = 0x19,
	ADS131M0X_REG_CH3_OCAL_LSB = 0x1a,
	ADS131M0X_REG_CH3_GCAL_MSB = 0x1b,
	ADS131M0X_REG_CH3_GCAL_LSB = 0x1c,
#endif
#ifdef DT_HAS_COMPAT_STATUS_OKAY(ti_ads131m06) || DT_HAS_COMPAT_STATUS_OKAY(ti_ads131m08)
	ADS131M0X_REG_CH4_CFG = 0x1d,
	ADS131M0X_REG_CH4_OCAL_MSB = 0x1e,
	ADS131M0X_REG_CH4_OCAL_LSB = 0x1f,
	ADS131M0X_REG_CH4_GCAL_MSB = 0x20,
	ADS131M0X_REG_CH4_GCAL_LSB = 0x21,
	ADS131M0X_REG_CH5_CFG = 0x22,
	ADS131M0X_REG_CH5_OCAL_MSB = 0x23,
	ADS131M0X_REG_CH5_OCAL_LSB = 0x24,
	ADS131M0X_REG_CH5_GCAL_MSB = 0x25,
	ADS131M0X_REG_CH5_GCAL_LSB = 0x26,
#endif
#ifdef DT_HAS_COMPAT_STATUS_OKAY(ti_ads131m08)
	ADS131M0X_REG_CH6_CFG = 0x27,
	ADS131M0X_REG_CH6_OCAL_MSB = 0x28,
	ADS131M0X_REG_CH6_OCAL_LSB = 0x29,
	ADS131M0X_REG_CH6_GCAL_MSB = 0x2a,
	ADS131M0X_REG_CH6_GCAL_LSB = 0x2b,
	ADS131M0X_REG_CH7_CFG = 0x2c,
	ADS131M0X_REG_CH7_OCAL_MSB = 0x2d,
	ADS131M0X_REG_CH7_OCAL_LSB = 0x2e,
	ADS131M0X_REG_CH7_GCAL_MSB = 0x2f,
	ADS131M0X_REG_CH7_GCAL_LSB = 0x30,
#endif
	ADS131M0X_REG_REGMAP_CRC = 0x3e,
};

#define ADS131M0X_CMD_WRITE      0x03
#define ADS131M0X_CMD_READ       0x05
#define ADS131M0X_CMD_RESET      0x11
#define ADS131M0X_CMD_STDBY      0x22
#define ADS131M0X_CMD_WAKEUP     0x33
#define ADS131M0X_CMD_LOCK       0x555
#define ADS131M0X_CMD_UNLOCK     0x655

struct ads131m0x_config {
	struct spi_dt_spec bus;
#if DT_NODE_HAS_PROP(DT_DRV_INST(0), drdy_gpios)
	struct gpio_dt_spec drdy_gpio;
#endif
#if DT_NODE_HAS_PROP(DT_DRV_INST(0), reset_gpios)
	struct gpio_dt_spec reset_gpio;
#endif
};

union ads131m08_cmd_union {
    struct {
        unsigned int start : 3;
        unsigned int address : 6;
        unsigned int count : 7;
    } cmd;
    uint8_t bytes[2];
};

struct ads131m0x_data {
	const struct device *dev;
	struct adc_context ctx;
	uint8_t rate;
	struct gpio_callback callback_data_ready;
	struct k_sem acq_sem;
	struct k_sem data_ready_signal;
	int32_t *buffer;
	int32_t *repeat_buffer;
	struct k_thread thread;
	bool differential;

	K_THREAD_STACK_MEMBER(stack, CONFIG_ADC_ADS131M0X_ACQUISITION_THREAD_STACK_SIZE);
};

static int ads131m0x_init(const struct device *dev)
{
	int err;
	const struct ads131m0x_config *config = dev->config;
	struct ads131m0x_data *data = dev->data;

	data->dev = dev;

	k_sem_init(&data->acq_sem, 0, 1);
	k_sem_init(&data->data_ready_signal, 0, 1);

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("spi bus %s not ready", config->bus.bus->name);
		return -ENODEV;
	}

/*
	err = gpio_pin_configure_dt(&config->drdy_gpio, GPIO_INPUT);
	if (err != 0) {
		LOG_ERR("failed to initialize GPIO for data ready (err %d)", err);
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(&config->drdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		LOG_ERR("failed to configure data ready interrupt (err %d)", err);
		return -EIO;
	}

	gpio_init_callback(&data->callback_data_ready, ads131m0x_data_ready_handler,
			   BIT(config->drdy_gpio.pin));
	err = gpio_add_callback(config->drdy_gpio.port, &data->callback_data_ready);
	if (err != 0) {
		LOG_ERR("failed to add data ready callback (err %d)", err);
		return -EIO;
	}
*/

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct adc_driver_api ads131m0x_api = {
	.channel_setup = ads131m0x_channel_setup,
	.read = ads131m0x_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = ads131m0x_adc_read_async,
#endif
	.ref_internal = 1200,
};

#define DT_INST_ADS131M0X(inst, t) DT_INST(inst, ti_ads131m0##t)

#define ADS131M0X_INIT(t, n)                                                                       \
	static const struct ads131m0x_config ads131m0##t##_cfg_##n = {                             \
		.bus = SPI_DT_SPEC_GET(DT_INST_ADS131M0X(n, t),                                    \
				       SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |                      \
				       SPI_TRANSFER_MSB | SPI_MODE_CPHA,                           \
				       1),                                                         \
		.drdy_gpio = GPIO_DT_SPEC_GET_OR(DT_INST_ADS131M0X(n, t), drdy_gpios, {0}),        \
		.reset_gpio = GPIO_DT_SPEC_GET_OR(DT_INST_ADS131M0X(n, t), reset_gpios, {0}),      \
	};                                                                                         \
	static struct ads131m0x_data ads131m0##t##_data_##n = {                                    \
		ADC_CONTEXT_INIT_LOCK(ads131m0##t##_data_##n, ctx),                                \
		ADC_CONTEXT_INIT_TIMER(ads131m0##t##_data_##n, ctx),                               \
		ADC_CONTEXT_INIT_SYNC(ads131m0##t##_data_##n, ctx),                                \
	};                                                                                         \
	DEVICE_DT_DEFINE(DT_INST_ADS131M0X(n, t), ads131m0x_init, NULL, &ads131m0##t##_data_##n,   \
			 &ads131m0##t##_cfg_##n, POST_KERNEL, CONFIG_ADC_ADS131M0X_INIT_PRIORITY,  \
			 &ads131m0x_api);

#define ADS131M02_INIT(n) \
	ADS131M0X_INIT(n, 2)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT ti_ads131m02
DT_INST_FOREACH_STATUS_OKAY(ADS131M02_INIT)

#define ADS131M03_INIT(n) \
	ADS131M0X_INIT(n, 3)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT ti_ads131m03
DT_INST_FOREACH_STATUS_OKAY(ADS131M03_INIT)

#define ADS131M04_INIT(n) \
	ADS131M0X_INIT(n, 4)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT ti_ads131m04
DT_INST_FOREACH_STATUS_OKAY(ADS131M04_INIT)

#define ADS131M06_INIT(n) \
	ADS131M0X_INIT(n, 6)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT ti_ads131m06
DT_INST_FOREACH_STATUS_OKAY(ADS131M06_INIT)

#define ADS131M08_INIT(n) \
	ADS131M0X_INIT(n, 8)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT ti_ads131m08
DT_INST_FOREACH_STATUS_OKAY(ADS131M08_INIT)
