/*
 * Copyright (c) 2025 Long EDC Framework
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file line_sensor.c
 * @brief 8-channel grayscale line sensor via 4051 analog mux + ADC
 *        with junction/lost event detection.
 */

#define DT_DRV_COMPAT long_line_sensor

#include "line_sensor.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <stdlib.h>
#include <string.h>

LOG_MODULE_REGISTER(line_sensor, CONFIG_SENSOR_LOG_LEVEL);

/* ---------- event detection thresholds ---------- */

/** @brief norm < this → "dark" (line) for event detection */
#define EVENT_DARK_THRESH 500

/** @brief min dark channels in center group (2-5) to consider "line present" */
#define EVENT_CENTER_MIN  1

/* ---------- config / data ---------- */

/** @brief ROM — devicetree-derived hardware config. */
struct line_sensor_config {
	/** ADC device + channel for mux output */
	struct adc_dt_spec adc;
	/** Three 4051 address GPIOs: S0, S1, S2 */
	struct gpio_dt_spec mux[3];
	/** Number of connected sensors (1–8) */
	uint8_t num_sensors;
};

/** @brief RAM — calibration state and runtime cache. */
struct line_sensor_data {
	/** Raw ADC cache from last sample_fetch */
	uint16_t raw[LINE_SENSOR_MAX_CHANNELS];
	/** Normalized cache from last sample_fetch */
	uint16_t norm[LINE_SENSOR_MAX_CHANNELS];
	/** White calibration (反射面 / reflect surface) */
	uint16_t calib_white[LINE_SENSOR_MAX_CHANNELS];
	/** Black calibration (黑线 / dark line) */
	uint16_t calib_black[LINE_SENSOR_MAX_CHANNELS];
	/** Gray_white threshold: (white*2+black)/3 */
	uint16_t gray_white[LINE_SENSOR_MAX_CHANNELS];
	/** Gray_black threshold: (white+black*2)/3 */
	uint16_t gray_black[LINE_SENSOR_MAX_CHANNELS];
	/** Normalization factor per channel */
	float normal_factor[LINE_SENSOR_MAX_CHANNELS];
	/** Digital (binary) output bitmap */
	atomic_t digital;
	/** White calibration done */
	bool white_calibrated;
	/** Black calibration done */
	bool black_calibrated;
	/** Fully calibrated (white && black) */
	bool calibrated;
	/** Previous event for edge detection (LOST→FOUND) */
	line_event_t prev_event;
	/** Lock for concurrent access */
	struct k_mutex lock;
};

/* ---------- 4051 mux helpers ---------- */

/** @brief Select 4051 channel by setting S0/S1/S2. */
static inline void mux_select(const struct line_sensor_config *cfg, uint8_t ch)
{
	gpio_pin_set_dt(&cfg->mux[0], (ch >> 0) & 1);
	gpio_pin_set_dt(&cfg->mux[1], (ch >> 1) & 1);
	gpio_pin_set_dt(&cfg->mux[2], (ch >> 2) & 1);
}

/* ---------- compare helper ---------- */

static int cmp_u16(const void *a, const void *b)
{
	return (int)(*(const uint16_t *)a - *(const uint16_t *)b);
}

/* ---------- internal: read one channel ---------- */

/**
 * @brief Read one channel — select mux, sample ADC *n* times, return median.
 *
 * @param dev      line-sensor device
 * @param ch       channel index (0..num_sensors-1)
 * @param n_sample number of samples (1..32; for ≥5 the min/max are dropped)
 * @return averaged ADC reading, or 0 on error / invalid n_sample
 */
static uint16_t read_channel(const struct device *dev, uint8_t ch, int n_sample)
{
	const struct line_sensor_config *cfg = dev->config;
	uint16_t buf[32];
	int16_t val;
	int err;

	if (n_sample < 1 || n_sample > (int)(sizeof(buf) / sizeof(buf[0]))) {
		return 0;
	}

	mux_select(cfg, ch);
	k_busy_wait(50); /* 4051 MUX settling — 3.3V 下留足余量 */

	for (int i = 0; i < n_sample; i++) {
		struct adc_sequence seq = {
			.buffer      = &val,
			.buffer_size = sizeof(val),
			.resolution  = 12,
			.channels    = BIT(cfg->adc.channel_id),
			.oversampling = 0,
		};

		err = adc_read(cfg->adc.dev, &seq);
		if (err < 0) {
			LOG_ERR("adc_read ch=%d err=%d", ch, err);
			return 0;
		}
		/* adc_read returns signed; handle negative gracefully */
		buf[i] = (uint16_t)(val < 0 ? 0 : val);
	}

	qsort(buf, n_sample, sizeof(buf[0]), cmp_u16);

	/* Return median after dropping min and max for n_sample >= 5 */
	if (n_sample >= 5) {
		uint32_t sum = 0;
		for (int i = 1; i < n_sample - 1; i++) {
			sum += buf[i];
		}
		return (uint16_t)(sum / (uint32_t)(n_sample - 2));
	}

	/* For small n, just average all */
	uint32_t sum = 0;
	for (int i = 0; i < n_sample; i++) {
		sum += buf[i];
	}
	return (uint16_t)(sum / (uint32_t)n_sample);
}

/* ---------- read all channels ---------- */

/** @brief Sample all channels and fill data->raw[]. */
static int sample_all_channels(const struct device *dev)
{
	struct line_sensor_data *data = dev->data;
	const struct line_sensor_config *cfg = dev->config;

	for (uint8_t ch = 0; ch < cfg->num_sensors; ch++) {
		data->raw[ch] = read_channel(dev, ch, 8);
	}

	return 0;
}

/* ---------- normalization ---------- */

/**
 * @brief Recompute thresholds and normalization factors from calib_white/black.
 *
 * Gray_white = (white*2 + black) / 3
 * Gray_black = (white + black*2) / 3
 * Normal_factor = 1000.0 / (white - black)
 */
static void recompute_calibration(struct line_sensor_data *data)
{
	for (int i = 0; i < LINE_SENSOR_MAX_CHANNELS; i++) {
		uint16_t w = data->calib_white[i];
		uint16_t b = data->calib_black[i];

		if (w <= b || w == 0) {
			/* Uncalibrated or invalid — use safe defaults */
			data->gray_white[i] = 256;
			data->gray_black[i] = 0;
			data->normal_factor[i] = 1.0f;
			continue;
		}

		data->gray_white[i] = (uint16_t)(((uint32_t)w * 2 + (uint32_t)b) / 3);
		data->gray_black[i] = (uint16_t)(((uint32_t)w + (uint32_t)b * 2) / 3);
		data->normal_factor[i] = 1000.0f / (float)(w - b);
	}
}

/* ---------- digital (binary) output ---------- */

/** @brief Apply hysteresis threshold to update digital bitmap. */
static uint8_t compute_digital(const struct line_sensor_data *data, const uint8_t prev)
{
	uint8_t out = 0;
	for (int i = 0; i < LINE_SENSOR_MAX_CHANNELS; i++) {
		uint16_t v = data->raw[i];
		bool prev_white = (prev >> i) & 1;

		if (v > data->gray_white[i]) {
			out |= BIT(i);           /* white (reflect surface) */
		} else if (v < data->gray_black[i]) {
			/* keep bit 0 (black) */
		} else {
			/* in hysteresis zone — keep previous */
			if (prev_white) {
				out |= BIT(i);
			}
		}
	}
	return out;
}

/** @brief Compute normalized values from raw. */
static void compute_normalized(const struct line_sensor_data *data, uint16_t *norm)
{
	for (int i = 0; i < LINE_SENSOR_MAX_CHANNELS; i++) {
		if (data->raw[i] <= data->calib_black[i]) {
			norm[i] = 0;
		} else {
			float n = (float)(data->raw[i] - data->calib_black[i])
				* data->normal_factor[i];
			if (n > 1000.0f) {
				n = 1000.0f;
			}
			norm[i] = (uint16_t)n;
		}
	}
}

/* ---------- event detection ---------- */

/**
 * @brief 基于归一化数据分析巡线事件。
 *
 * 8 通道分区：
 *   - 左边缘 (0–1)      : 左路口判别
 *   - 中央区 (2–5)      : 主线判别
 *   - 右边缘 (6–7)      : 右路口判别
 *
 * 事件判定优先级（从高到低）：
 *   CROSS > LEFT/RIGHT > LOST > FOUND > NONE
 */
static line_event_t detect_event(const struct line_sensor_data *data)
{
	int dark_center = 0, dark_left = 0, dark_right = 0, dark_total = 0;

	for (int i = 0; i < LINE_SENSOR_MAX_CHANNELS; i++) {
		if (data->norm[i] < EVENT_DARK_THRESH) {
			dark_total++;
			if (i <= 1)       { dark_left++;   }
			else if (i <= 5)  { dark_center++;  }
			else              { dark_right++;   }
		}
	}

	/* 线完全消失 */
	if (dark_total == 0) {
		return LINE_EVT_LOST;
	}

	/* 中央无线 → 线偏到边缘了，不是路口，算正常（丢线情况由上层处理） */
	if (dark_center < EVENT_CENTER_MIN) {
		return LINE_EVT_NONE;
	}

	/* 十字/T 路口：左右边缘同时有暗区 */
	if (dark_left >= 1 && dark_right >= 1) {
		return LINE_EVT_CROSS;
	}

	/* 左路口 */
	if (dark_left >= 1 && dark_right == 0) {
		return LINE_EVT_LEFT;
	}

	/* 右路口 */
	if (dark_right >= 1 && dark_left == 0) {
		return LINE_EVT_RIGHT;
	}

	return LINE_EVT_NONE;
}

/* ========== Zephyr sensor API ========== */

static int line_sensor_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct line_sensor_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	sample_all_channels(dev);

	if (data->calibrated) {
		compute_normalized(data, data->norm);
		atomic_set(&data->digital, compute_digital(data,
			(uint8_t)atomic_get(&data->digital)));
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

static int line_sensor_channel_get(const struct device *dev, enum sensor_channel chan,
		struct sensor_value *val)
{
	(void)dev;
	(void)chan;
	(void)val;
	return -ENOTSUP;
}

static DEVICE_API(sensor, line_sensor_api) = {
	.sample_fetch = line_sensor_sample_fetch,
	.channel_get  = line_sensor_channel_get,
};

/* ========== Public API ========== */

int line_sensor_read_raw(const struct device *dev, uint16_t raw[LINE_SENSOR_MAX_CHANNELS])
{
	struct line_sensor_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	sample_all_channels(dev);
	memcpy(raw, data->raw, sizeof(data->raw));
	k_mutex_unlock(&data->lock);

	return 0;
}

int line_sensor_calibrate_white(const struct device *dev)
{
	struct line_sensor_data *data = dev->data;
	const struct line_sensor_config *cfg = dev->config;

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Sample each channel 32 times, take median */
	for (uint8_t ch = 0; ch < cfg->num_sensors; ch++) {
		data->calib_white[ch] = read_channel(dev, ch, 32);
	}

	/* Fill remaining channels with channel 0 */
	for (int i = cfg->num_sensors; i < LINE_SENSOR_MAX_CHANNELS; i++) {
		data->calib_white[i] = data->calib_white[0];
	}

	recompute_calibration(data);
	data->white_calibrated = true;
	data->calibrated = data->white_calibrated && data->black_calibrated;

	k_mutex_unlock(&data->lock);

	LOG_INF("white calibration done");
	return 0;
}

int line_sensor_calibrate_black(const struct device *dev)
{
	struct line_sensor_data *data = dev->data;
	const struct line_sensor_config *cfg = dev->config;

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Sample each channel 32 times, take median */
	for (uint8_t ch = 0; ch < cfg->num_sensors; ch++) {
		data->calib_black[ch] = read_channel(dev, ch, 32);
	}

	/* Fill remaining channels with channel 0 */
	for (int i = cfg->num_sensors; i < LINE_SENSOR_MAX_CHANNELS; i++) {
		data->calib_black[i] = data->calib_black[0];
	}

	recompute_calibration(data);
	data->black_calibrated = true;
	data->calibrated = data->white_calibrated && data->black_calibrated;

	k_mutex_unlock(&data->lock);

	LOG_INF("black calibration done");
	return 0;
}

int line_sensor_get_normalized(const struct device *dev, uint16_t norm[LINE_SENSOR_MAX_CHANNELS])
{
	struct line_sensor_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	memcpy(norm, data->norm, sizeof(data->norm));
	k_mutex_unlock(&data->lock);

	return 0;
}

uint8_t line_sensor_get_digital(const struct device *dev)
{
	return (uint8_t)atomic_get(&((struct line_sensor_data *)dev->data)->digital);
}

int line_sensor_get_thresholds(const struct device *dev,
		uint16_t gray_white[LINE_SENSOR_MAX_CHANNELS],
		uint16_t gray_black[LINE_SENSOR_MAX_CHANNELS])
{
	struct line_sensor_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	memcpy(gray_white, data->gray_white, sizeof(data->gray_white));
	memcpy(gray_black, data->gray_black, sizeof(data->gray_black));
	k_mutex_unlock(&data->lock);

	return 0;
}

bool line_sensor_is_calibrated(const struct device *dev)
{
	return ((struct line_sensor_data *)dev->data)->calibrated;
}

void line_sensor_reset_calibration(const struct device *dev)
{
	struct line_sensor_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	memset(data->calib_white, 0, sizeof(data->calib_white));
	memset(data->calib_black, 0, sizeof(data->calib_black));
	memset(data->gray_white, 0, sizeof(data->gray_white));
	memset(data->gray_black, 0, sizeof(data->gray_black));
	for (int i = 0; i < LINE_SENSOR_MAX_CHANNELS; i++) {
		data->normal_factor[i] = 1.0f;
	}
	data->white_calibrated = false;
	data->black_calibrated = false;
	data->calibrated = false;
	data->prev_event = LINE_EVT_NONE;
	atomic_set(&data->digital, 0);
	k_mutex_unlock(&data->lock);
}

line_event_t line_sensor_get_event(const struct device *dev)
{
	struct line_sensor_data *data = dev->data;
	line_event_t evt;

	k_mutex_lock(&data->lock, K_FOREVER);

	evt = detect_event(data);

	/* LOST → 有线 = FOUND（上升沿检测） */
	if (data->prev_event == LINE_EVT_LOST && evt != LINE_EVT_LOST) {
		evt = LINE_EVT_FOUND;
	}
	data->prev_event = evt;

	k_mutex_unlock(&data->lock);

	return evt;
}

/* ========== Zephyr device init ========== */

static int line_sensor_init(const struct device *dev)
{
	struct line_sensor_data *data = dev->data;
	const struct line_sensor_config *cfg = dev->config;
	int ret;

	/* Verify ADC */
	if (!device_is_ready(cfg->adc.dev)) {
		LOG_ERR("ADC not ready");
		return -ENODEV;
	}

	/* MSPM0 ADC requires adc_channel_setup before adc_read */
	{
		struct adc_channel_cfg ch_cfg = {
			.gain             = ADC_GAIN_1,
			.reference        = ADC_REF_VDD_1,
			.acquisition_time = ADC_ACQ_TIME_DEFAULT,
			.channel_id       = cfg->adc.channel_id,
			.differential     = 0,
		};
		int ch_ret = adc_channel_setup(cfg->adc.dev, &ch_cfg);
		if (ch_ret < 0) {
			LOG_ERR("adc channel setup ch=%d failed: %d",
				cfg->adc.channel_id, ch_ret);
			return ch_ret;
		}
	}

	/* diagnostic: read ADC directly once, no MUX */
	{
		int16_t test_val = -1;
		struct adc_sequence test_seq = {
			.buffer      = &test_val,
			.buffer_size = sizeof(test_val),
			.resolution  = 12,
			.channels    = BIT(cfg->adc.channel_id),
			.oversampling = 0,
		};
		int test_ret = adc_read(cfg->adc.dev, &test_seq);
		LOG_INF("ADC diag: ret=%d val=%d ch=%d",
			test_ret, (int)test_val, cfg->adc.channel_id);
	}

	/* Verify and configure mux GPIOs */
	for (int i = 0; i < 3; i++) {
		if (!gpio_is_ready_dt(&cfg->mux[i])) {
			LOG_ERR("MUX GPIO %d not ready", i);
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->mux[i], GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("MUX GPIO %d configure failed: %d", i, ret);
			return ret;
		}
	}

	k_mutex_init(&data->lock);
	line_sensor_reset_calibration(dev);

	LOG_INF("line sensor ready: adc_ch=%d num_sensors=%d",
		cfg->adc.channel_id, cfg->num_sensors);

	return 0;
}

/* ========== Device registration ========== */

#define LINE_SENSOR_DEFINE(n)                                            \
	static struct line_sensor_data line_sensor_data_##n = {            \
		.lock = Z_MUTEX_INITIALIZER(line_sensor_data_##n.lock),  \
	};                                                                 \
	static const struct line_sensor_config line_sensor_config_##n = { \
		.adc = ADC_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), 0),        \
		.mux = {                                                   \
			GPIO_DT_SPEC_INST_GET_BY_IDX(n, mux_gpios, 0),    \
			GPIO_DT_SPEC_INST_GET_BY_IDX(n, mux_gpios, 1),    \
			GPIO_DT_SPEC_INST_GET_BY_IDX(n, mux_gpios, 2),    \
		},                                                         \
		.num_sensors = DT_INST_PROP(n, num_sensors),              \
	};                                                                 \
	SENSOR_DEVICE_DT_INST_DEFINE(n, line_sensor_init, NULL,            \
			&line_sensor_data_##n, &line_sensor_config_##n,      \
			POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,            \
			&line_sensor_api);

DT_INST_FOREACH_STATUS_OKAY(LINE_SENSOR_DEFINE)
