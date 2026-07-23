/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file long_qdec.c
 * @brief GPIO-based Quadrature Decoder (QDEC) sensor driver.
 *
 * Channel A is configured for both-edge interrupts. Channel B is a plain
 * input read in the ISR to determine direction — 2x resolution decoding.
 *
 * Channels exposed via Zephyr sensor API:
 *   - SENSOR_CHAN_ROTATION — angular position (milli-degrees or raw steps)
 *   - SENSOR_CHAN_RPM      — speed in revolutions per minute
 */

#define DT_DRV_COMPAT long_qdec

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(qdec_gpio, CONFIG_SENSOR_LOG_LEVEL);

/** @brief ROM — 设备树解析出的硬件参数 */
struct qdec_config {
	struct gpio_dt_spec a;       /**< 通道 A — 中断引脚（双边沿触发） */
	struct gpio_dt_spec b;       /**< 通道 B — 方向参考引脚（只读） */
	uint32_t pulses_per_rev;     /**< 每转脉冲数（完整正交周期数），0=原始步数 */
};

/** @brief RAM — 编码器运行时状态 */
struct qdec_data {
	const struct device *dev;    /**< 自引用 */
	struct gpio_callback cb;     /**< GPIO 回调 */
	atomic_t position;           /**< ISR 原子累加，sample_fetch 快照捕获 */
	int32_t fetched;             /**< 最后一次快照值（供 channel_get 读取） */
	int32_t last_position;       /**< 上一次 fetch 时的位置（用于计算速度） */
	int64_t last_ticks;          /**< 上一次 fetch 时刻（k_uptime_ticks） */
	int32_t rpm;                 /**< 计算出的转速（RPM） */
};

/**
 * @brief ISR — 通道 A 双边沿触发。
 *
 * 方向规则：delta = (A == B) ? -1 : +1（2x 分辨率）。
 * 原子操作 position 以与主线程无锁共享。
 */
static void qdec_isr(const struct device *port, struct gpio_callback *cb,
		     gpio_port_pins_t pins)
{
	struct qdec_data *data = CONTAINER_OF(cb, struct qdec_data, cb);
	const struct qdec_config *cfg = data->dev->config;
	int a = gpio_pin_get_dt(&cfg->a);
	int b = gpio_pin_get_dt(&cfg->b);

	atomic_add(&data->position, (a == b) ? -1 : +1);
}

/**
 * @brief 原子快照 position 并计算瞬时 RPM。
 *
 * RPM = delta_steps / (2 * ppr) * 60 / dt_sec
 *
 * @param dev  QDEC 设备
 * @param chan 采样通道（SENSOR_CHAN_ALL / ROTATION / RPM）
 * @retval 0 成功
 * @retval -ENOTSUP 不支持的通道
 */
static int qdec_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct qdec_data *data = dev->data;
	const struct qdec_config *cfg = dev->config;
	int64_t now = k_uptime_ticks();
	int32_t cur, delta;
	int64_t dt_ticks;

	if (chan != SENSOR_CHAN_ALL &&
	    chan != SENSOR_CHAN_ROTATION &&
	    chan != SENSOR_CHAN_RPM) {
		return -ENOTSUP;
	}

	cur = atomic_get(&data->position);
	data->fetched = cur;

	delta = cur - data->last_position;
	dt_ticks = now - data->last_ticks;

	if (dt_ticks > 0 && cfg->pulses_per_rev > 0) {
		data->rpm = (int32_t)((int64_t)delta * 30 *
			       CONFIG_SYS_CLOCK_TICKS_PER_SEC /
			       ((int64_t)cfg->pulses_per_rev * dt_ticks));
	} else {
		data->rpm = 0;
	}

	data->last_position = cur;
	data->last_ticks = now;

	return 0;
}

/**
 * @brief 返回缓存的传感器通道值。
 *
 * @param dev  QDEC 设备
 * @param chan SENSOR_CHAN_ROTATION 或 SENSOR_CHAN_RPM
 * @param val  输出：milli-degrees 或 RPM
 * @retval 0 成功
 * @retval -ENOTSUP 不支持的通道
 */
static int qdec_channel_get(const struct device *dev, enum sensor_channel chan,
			    struct sensor_value *val)
{
	struct qdec_data *data = dev->data;
	const struct qdec_config *cfg = dev->config;

	switch (chan) {
	case SENSOR_CHAN_ROTATION:
		if (cfg->pulses_per_rev > 0) {
			int64_t tmp = (int64_t)data->fetched * 180000000LL
				      / (int32_t)cfg->pulses_per_rev;
			val->val1 = (int32_t)(tmp / 1000000LL);
			val->val2 = (int32_t)(tmp % 1000000LL);
		} else {
			val->val1 = data->fetched;
			val->val2 = 0;
		}
		break;

	case SENSOR_CHAN_RPM:
		val->val1 = data->rpm;
		val->val2 = 0;
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

/** @brief Zephyr sensor 驱动 API 表 */
static DEVICE_API(sensor, qdec_api) = {
	.sample_fetch = qdec_sample_fetch,
	.channel_get  = qdec_channel_get,
};

/**
 * @brief QDEC 硬件初始化 — 配置 GPIO、注册 ISR。
 *
 * POST_KERNEL 阶段自动调用。
 */
static int qdec_init(const struct device *dev)
{
	const struct qdec_config *cfg = dev->config;
	struct qdec_data *data = dev->data;
	int ret;

	if (!gpio_is_ready_dt(&cfg->a)) {
		LOG_ERR("channel A GPIO not ready");
		return -ENODEV;
	}
	if (!gpio_is_ready_dt(&cfg->b)) {
		LOG_ERR("channel B GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->a, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("configure channel A failed: %d", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&cfg->b, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("configure channel B failed: %d", ret);
		return ret;
	}

	data->dev = dev;
	data->last_ticks = k_uptime_ticks();

	gpio_init_callback(&data->cb, qdec_isr, BIT(cfg->a.pin));

	ret = gpio_add_callback_dt(&cfg->a, &data->cb);
	if (ret < 0) {
		LOG_ERR("add callback failed: %d", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->a, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		LOG_ERR("configure interrupt failed: %d", ret);
		return ret;
	}

	LOG_DBG("QDEC ready: A=pin%d B=pin%d", cfg->a.pin, cfg->b.pin);

	return 0;
}

/** @brief 设备注册宏 */
#define QDEC_GPIO_INIT(n)                                                    \
	static struct qdec_data qdec_data_##n = {                            \
		.position = ATOMIC_INIT(0),                                  \
	};                                                                   \
									     \
	static const struct qdec_config qdec_config_##n = {                  \
		.a = GPIO_DT_SPEC_INST_GET_BY_IDX(n, a_gpios, 0),           \
		.b = GPIO_DT_SPEC_INST_GET_BY_IDX(n, b_gpios, 0),           \
		.pulses_per_rev = DT_INST_PROP(n, pulses_per_rev),          \
	};                                                                   \
									     \
	SENSOR_DEVICE_DT_INST_DEFINE(n, qdec_init, NULL,                    \
				     &qdec_data_##n, &qdec_config_##n,       \
				     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
				     &qdec_api);

DT_INST_FOREACH_STATUS_OKAY(QDEC_GPIO_INIT)
