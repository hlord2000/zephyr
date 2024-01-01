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
#if DT_NODE_HAS_PROP(DT_DRV_INST(0), sync_gpios)
	struct gpio_dt_spec sync_gpio;
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

static void ads131m0x_data_ready_handler(const struct device *dev, struct gpio_callback *gpio_cb,
					uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pins);

	struct ads131m0x_data *data =
		CONTAINER_OF(gpio_cb, struct ads131m0x_data, callback_data_ready);

	k_sem_give(&data->data_ready_signal);
}

static int ads131m0x_read_reg(const struct device *dev, enum max1125x_reg reg_addr, uint8_t *buffer,
			     size_t reg_size)
{
	int ret;
	const struct ads131m0x_config *config = dev->config;
	uint8_t buffer_tx[3];
	uint8_t buffer_rx[ARRAY_SIZE(buffer_tx)];
	const struct spi_buf tx_buf[] = {{
		.buf = buffer_tx,
		.len = ARRAY_SIZE(buffer_tx),
	}};
	const struct spi_buf rx_buf[] = {{
		.buf = buffer_rx,
		.len = ARRAY_SIZE(buffer_rx),
	}};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};
	buffer_tx[0] = MAX1125X_CMD_READ | reg_addr;
	/* read one register */
	buffer_tx[1] = 0x00;

	ret = spi_transceive_dt(&config->bus, &tx, &rx);
	if (ret != 0) {
		LOG_ERR("MAX1125X: error writing register 0x%X (%d)", reg_addr, ret);
		return ret;
	}
	*buffer = buffer_rx[1];
	LOG_DBG("read from register 0x%02X value 0x%02X", reg_addr, *buffer);

	return 0;
}

static int ads131m0x_write_reg(const struct device *dev, enum max1125x_reg reg_addr,
			      uint8_t *reg_val, size_t reg_size)
{
	int ret;
	const struct ads131m0x_config *config = dev->config;
	uint8_t command = MAX1125X_CMD_WRITE | reg_addr;

	const struct spi_buf spi_buf[2] = {{.buf = &command, .len = sizeof(command)},
					   {.buf = reg_val, .len = reg_size}};
	const struct spi_buf_set tx = {.buffers = spi_buf, .count = ARRAY_SIZE(spi_buf)};

	ret = spi_write_dt(&config->bus, &tx);
	if (ret != 0) {
		LOG_ERR("MAX1125X: error writing register 0x%X (%d)", reg_addr, ret);
		return ret;
	}

	return 0;
}

static int ads131m0x_send_command(const struct device *dev, enum max1125x_mode mode, uint8_t rate)
{
	int ret;
	const struct ads131m0x_config *config = dev->config;
	uint8_t command = MAX1125X_CMD_CONV | mode | rate;
	const struct spi_buf spi_buf = {.buf = &command, .len = sizeof(command)};
	const struct spi_buf_set tx = {.buffers = &spi_buf, .count = 1};

	ret = spi_write_dt(&config->bus, &tx);
	if (ret != 0) {
		LOG_ERR("MAX1125X: error writing register 0x%X (%d)", rate, ret);
		return ret;
	}

	return 0;
}

static int ads131m0x_start_conversion(const struct device *dev)
{
	const struct ads131m0x_data *data = dev->data;

	return ads131m0x_send_command(dev, MAX1125X_CMD_SEQUENCER, data->rate);
}

static inline int ads131m0x_acq_time_to_dr(const struct device *dev, uint16_t acq_time)
{
	struct ads131m0x_data *data = dev->data;
	const struct ads131m0x_config *config = dev->config;
	const uint32_t *odr_delay = config->odr_delay;
	uint32_t odr_delay_us = 0;
	uint16_t acq_value = ADC_ACQ_TIME_VALUE(acq_time);
	int odr = -EINVAL;

	if (acq_time != ADC_ACQ_TIME_DEFAULT && ADC_ACQ_TIME_UNIT(acq_time) != ADC_ACQ_TIME_TICKS) {
		LOG_ERR("MAX1125X: invalid acq time value (%d)", acq_time);
		return -EINVAL;
	}

	if (acq_value < MAX1125X_CONFIG_RATE_1_9 || acq_value > MAX1125X_CONFIG_RATE_64000) {
		LOG_ERR("MAX1125X: invalid acq value (%d)", acq_value);
		return -EINVAL;
	}

	odr = acq_value;
	odr_delay_us = odr_delay[acq_value];

	data->rate = odr;

	return odr;
}

static int ads131m0x_wait_data_ready(const struct device *dev)
{
	struct ads131m0x_data *data = dev->data;

	return k_sem_take(&data->data_ready_signal, ADC_CONTEXT_WAIT_FOR_COMPLETION_TIMEOUT);
}

static int ads131m0x_read_sample(const struct device *dev)
{
	const struct ads131m0x_config *config = dev->config;
	struct ads131m0x_data *data = dev->data;
	bool is_positive;
	uint8_t buffer_tx[(config->resolution / 8) + 1];
	uint8_t buffer_rx[ARRAY_SIZE(buffer_tx)];
	uint8_t current_channel = find_msb_set(data->ctx.sequence.channels) - 1;
	int rc;

	const struct spi_buf tx_buf[] = {{
		.buf = buffer_tx,
		.len = ARRAY_SIZE(buffer_tx),
	}};
	const struct spi_buf rx_buf[] = {{
		.buf = buffer_rx,
		.len = ARRAY_SIZE(buffer_rx),
	}};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	buffer_tx[0] = MAX1125X_CMD_READ | MAX1125X_REG_DATA(current_channel);

	rc = spi_transceive_dt(&config->bus, &tx, &rx);
	if (rc != 0) {
		LOG_ERR("spi_transceive failed with error %i", rc);
		return rc;
	}

	/* The data format while in unipolar mode is always offset binary.
	 * In offset binary format the most negative value is 0x000000,
	 * the midscale value is 0x800000 and the most positive value is
	 * 0xFFFFFF. In bipolar mode if the FORMAT bit = ‘1’ then the
	 * data format is offset binary. If the FORMAT bit = ‘0’, then
	 * the data format is two’s complement. In two’s complement the
	 * negative full-scale value is 0x800000, the midscale is 0x000000
	 * and the positive full scale is 0x7FFFFF. Any input exceeding
	 * the available input range is limited to the minimum or maximum
	 * data value.
	 */
	is_positive = buffer_rx[(config->resolution / 8)] >> 7;
	if (is_positive) {
		*data->buffer++ = sys_get_be24(buffer_rx) - (1 << (config->resolution - 1));
	} else {
		*data->buffer++ = sys_get_be24(buffer_rx + 1);
	}

	adc_context_on_sampling_done(&data->ctx, dev);

	return rc;
}

static int ads131m0x_configure_chmap(const struct device *dev, const uint8_t channel_id)
{
	uint8_t last_order = 0;
	uint8_t chmap1_register[3] = {0};
	uint8_t chmap0_register[3] = {0};

	if (channel_id > 6) {
		LOG_ERR("MAX1125X: invalid channel (%u)", channel_id);
		return -EINVAL;
	}

	ads131m0x_read_reg(dev, MAX1125X_REG_CHMAP1, chmap1_register, MAX1125X_REG_CHMAP1_LEN);
	for (int index = 0; index < 3; index++) {
		if ((chmap1_register[index] >> 2) >= last_order) {
			last_order = chmap1_register[index] >> 2;
		} else {
			continue;
		}
	}

	ads131m0x_read_reg(dev, MAX1125X_REG_CHMAP0, chmap0_register, MAX1125X_REG_CHMAP0_LEN);
	for (int index = 0; index < 3; index++) {
		if ((chmap0_register[index] >> 2) >= last_order) {
			last_order = chmap0_register[index] >> 2;
		} else {
			continue;
		}
	}

	last_order++;

	switch (channel_id) {
	case MAX1125X_CHANNEL_0:
		chmap0_register[2] = MAX1125X_CONFIG_CHMAP(last_order);
		break;
	case MAX1125X_CHANNEL_1:
		chmap0_register[1] = MAX1125X_CONFIG_CHMAP(last_order);
		break;
	case MAX1125X_CHANNEL_2:
		chmap0_register[0] = MAX1125X_CONFIG_CHMAP(last_order);
		break;
	case MAX1125X_CHANNEL_3:
		chmap1_register[2] = MAX1125X_CONFIG_CHMAP(last_order);
		break;
	case MAX1125X_CHANNEL_4:
		chmap1_register[1] = MAX1125X_CONFIG_CHMAP(last_order);
		break;
	case MAX1125X_CHANNEL_5:
		chmap1_register[0] = MAX1125X_CONFIG_CHMAP(last_order);
		break;
	default:
		break;
	}

	if (channel_id > 3) {
		/* CHMAP 1 register configuration */
		ads131m0x_write_reg(dev, MAX1125X_REG_CHMAP1, chmap1_register,
				   MAX1125X_REG_CHMAP1_LEN);
	} else {
		/* CHMAP 0 register configuration */
		ads131m0x_write_reg(dev, MAX1125X_REG_CHMAP0, chmap0_register,
				   MAX1125X_REG_CHMAP0_LEN);
	}

	return 0;
}

static int ads131m0x_channel_setup(const struct device *dev,
				  const struct adc_channel_cfg *channel_cfg)
{
	const struct ads131m0x_config *max_config = dev->config;
	uint8_t seq_register = 0;
	uint8_t ctrl2_register = 0;
	uint8_t gpio_reg = 0;
	uint8_t gpo_reg = 0;

	/* sequencer register configuration */
	ads131m0x_read_reg(dev, MAX1125X_REG_SEQ, &seq_register, MAX1125X_REG_SEQ_LEN);
	seq_register |= BIT(MAX1125X_SEQ_MDREN);
	seq_register |= BIT(MAX1125X_SEQ_MODE0);
	ads131m0x_write_reg(dev, MAX1125X_REG_SEQ, &seq_register, MAX1125X_REG_SEQ_LEN);

	/* configuration multiplexer */
	if (max_config->multiplexer) {
		if (!channel_cfg->differential) {
			LOG_ERR("6 channel fully supported only supported differential "
				"differemtial option %i",
				channel_cfg->differential);
			return -ENOTSUP;
		}
	}

	ads131m0x_acq_time_to_dr(dev, channel_cfg->acquisition_time);

	/* ctrl2 register configuration */
	if (max_config->pga) {
		/* programmable gain amplifier support */
		ctrl2_register |= MAX1125X_CONFIG_PGA(MAX1125X_CTRL2_PGAEN);
		switch (channel_cfg->gain) {
		case ADC_GAIN_1:
			ctrl2_register |= MAX1125X_CTRL2_PGA_GAIN_1;
			break;
		case ADC_GAIN_2:
			ctrl2_register |= MAX1125X_CTRL2_PGA_GAIN_2;
			break;
		case ADC_GAIN_4:
			ctrl2_register |= MAX1125X_CTRL2_PGA_GAIN_4;
			break;
		case ADC_GAIN_8:
			ctrl2_register |= MAX1125X_CTRL2_PGA_GAIN_8;
			break;
		case ADC_GAIN_16:
			ctrl2_register |= MAX1125X_CTRL2_PGA_GAIN_16;
			break;
		case ADC_GAIN_32:
			ctrl2_register |= MAX1125X_CTRL2_PGA_GAIN_32;
			break;
		case ADC_GAIN_64:
			ctrl2_register |= MAX1125X_CTRL2_PGA_GAIN_64;
			break;
		case ADC_GAIN_128:
			ctrl2_register |= MAX1125X_CTRL2_PGA_GAIN_128;
			break;
		default:
			LOG_ERR("MAX1125X: unsupported channel gain '%d'", channel_cfg->gain);
			return -ENOTSUP;
		}
	}

	if (channel_cfg->reference == ADC_REF_INTERNAL) {
		ctrl2_register |= BIT(MAX1125X_CTRL2_LDOEN);

	} else if (channel_cfg->reference == ADC_REF_EXTERNAL1) {
		ctrl2_register &= ~BIT(MAX1125X_CTRL2_LDOEN);
	} else {
		LOG_ERR("MAX1125X: unsupported channel reference type '%d'",
			channel_cfg->reference);
		return -ENOTSUP;
	}
	ads131m0x_write_reg(dev, MAX1125X_REG_CTRL2, &ctrl2_register, MAX1125X_REG_CTRL2_LEN);

	/* GPIO_CTRL register configuration */
	gpio_reg |= max_config->gpio.gpio0_enable << MAX1125X_GPIO_CTRL_GPIO0_EN;
	gpio_reg |= max_config->gpio.gpio1_enable << MAX1125X_GPIO_CTRL_GPIO1_EN;
	gpio_reg |= max_config->gpio.gpio0_direction << MAX1125X_GPIO_CTRL_DIRO;
	gpio_reg |= max_config->gpio.gpio1_direction << MAX1125X_GPIO_CTRL_DIR1;
	ads131m0x_write_reg(dev, MAX1125X_REG_GPIO_CTRL, &gpio_reg, MAX1125X_REG_GPIO_CTRL_LEN);

	/* GPO_DIR register configuration */
	gpo_reg |= max_config->gpo.gpo0_enable << MAX1125X_GPO_DIR_GPO0;
	gpo_reg |= max_config->gpo.gpo1_enable << MAX1125X_GPO_DIR_GPO1;
	ads131m0x_write_reg(dev, MAX1125X_REG_GPO_DIR, &gpo_reg, MAX1125X_REG_GPO_DIR_LEN);

	/* configuration of channel order */
	ads131m0x_configure_chmap(dev, channel_cfg->channel_id);

	return 0;
}

static int ads131m0x_validate_buffer_size(const struct adc_sequence *sequence)
{
	size_t needed = sizeof(uint8_t) * (sequence->resolution / 8);

	if (sequence->options) {
		needed *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed) {
		return -ENOMEM;
	}

	return 0;
}

static int ads131m0x_validate_sequence(const struct device *dev, const struct adc_sequence *sequence)
{
	int err;

	if (sequence->oversampling) {
		LOG_ERR("MAX1125X: oversampling not supported");
		return -ENOTSUP;
	}

	err = ads131m0x_validate_buffer_size(sequence);
	if (err) {
		LOG_ERR("MAX1125X: buffer size too small");
		return -ENOTSUP;
	}

	return 0;
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct ads131m0x_data *data = CONTAINER_OF(ctx, struct max1125x_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct ads131m0x_dSYNC / RESETata *data = CONTAINER_OF(ctx, struct max1125x_data, ctx);

	data->repeat_buffer = data->buffer;

	ads131m0x_start_conversion(data->dev);

	k_sem_give(&data->acq_sem);
}

static int ads131m0x_adc_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	int rc;
	struct ads131m0x_data *data = dev->data;

	rc = ads131m0x_validate_sequence(dev, sequence);
	if (rc != 0) {
		return rc;
	}

	data->buffer = sequence->buffer;

	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int ads131m0x_adc_read_async(const struct device *dev, const struct adc_sequence *sequence,
				   struct k_poll_signal *async)
{
	int rc;
	struct ads131m0x_data *data = dev->data;

	adc_context_lock(&data->ctx, async ? true : false, async);
	rc = ads131m0x_adc_start_read(dev, sequence);
	adc_context_release(&data->ctx, rc);

	return rc;
}

static int ads131m0x_adc_perform_read(const struct device *dev)
{
	struct ads131m0x_data *data = dev->data;
	int rc;

	rc = ads131m0x_read_sample(dev);
	if (rc != 0) {
		LOG_ERR("reading sample failed (err %d)", rc);
		adc_context_complete(&data->ctx, rc);
		return rc;
	}

	return rc;
}

static int ads131m0x_read(const struct device *dev, const struct adc_sequence *sequence)
{
	return ads131m0x_adc_read_async(dev, sequence, NULL);
}

static void ads131m0x_acquisition_thread(const struct device *dev)
{
	struct ads131m0x_data *data = dev->data;
	int rc;

	while (true) {
		k_sem_take(&data->acq_sem, K_FOREVER);

		rc = ads131m0x_wait_data_ready(dev);
		if (rc != 0) {
			LOG_ERR("MAX1125X: failed to get ready status (err %d)", rc);
			adc_context_complete(&data->ctx, rc);
			break;
		}

		ads131m0x_adc_perform_read(dev);
	}
}

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

	const k_tid_t tid = k_thread_create(
		&data->thread, data->stack, K_THREAD_STACK_SIZEOF(data->stack),
		(k_thread_entry_t)ads131m0x_acquisition_thread, (void *)dev, NULL, NULL,
		CONFIG_ADC_ADS131M0X_ACQUISITION_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(tid, "adc_ads131m0x");

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

#define DT_INST_ADS131M0X(inst, t) DT_INST(inst, maxim_max##t)

#define ADS131M0X_INIT(t, n, odr_delay_us, res, mux, pgab)                                          \
	static const struct ads131m0x_config max##t##_cfg_##n = {                                   \
		.bus = SPI_DT_SPEC_GET(DT_INST_MAX1125X(n, t),                                     \
				       SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,    \
				       1),                                                         \
		.drdy_gpio = GPIO_DT_SPEC_GET_OR(DT_INST_MAX1125X(n, t), drdy_gpios, {0}),         \
	};                                                                                         \
	static struct ads131m0x_data max##t##_data_##n = {                                          \
		ADC_CONTEXT_INIT_LOCK(max##t##_data_##n, ctx),                                     \
		ADC_CONTEXT_INIT_TIMER(max##t##_data_##n, ctx),                                    \
		ADC_CONTEXT_INIT_SYNC(max##t##_data_##n, ctx),                                     \
	};                                                                                         \
	DEVICE_DT_DEFINE(DT_INST_MAX1125X(n, t), ads131m0x_init, NULL, &max##t##_data_##n,          \
			 &max##t##_cfg_##n, POST_KERNEL, CONFIG_ADC_ADS131M0X_INIT_PRIORITY,        \
			 &ads131m0x_api);

#define ADS131M02_INIT(n) \
	ADS131M0X_INIT()
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT ti_ads131m02
DT_INST_FOREACH_STATUS_OKAY(ADS131M02_INIT)

#define ADS131M03_INIT(n) \
	ADS131M0X_INIT()
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT ti_ads131m03
DT_INST_FOREACH_STATUS_OKAY(ADS131M03_INIT)

#define ADS131M04_INIT(n) \
	ADS131M0X_INIT()
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT ti_ads131m04
DT_INST_FOREACH_STATUS_OKAY(ADS131M04_INIT)

#define ADS131M06_INIT(n) \
	ADS131M0X_INIT()
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT ti_ads131m06
DT_INST_FOREACH_STATUS_OKAY(ADS131M06_INIT)

#define ADS131M08_INIT(n) \
	ADS131M0X_INIT()
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT ti_ads131m08
DT_INST_FOREACH_STATUS_OKAY(ADS131M08_INIT)
