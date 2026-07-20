/* SPDX-License-Identifier: Apache-2.0 */
/*
 * GPIO-based Quadrature Decoder (QDEC) sensor driver.
 *
 * Only channel A is configured for both-edge interrupts. Channel B is a
 * plain input read in the ISR to determine direction — 2x resolution.
 *
 * Channels:
 *   SENSOR_CHAN_ROTATION  — angular position (degrees or raw steps)
 *   SENSOR_CHAN_RPM       — speed in revolutions per minute
 */
#define DT_DRV_COMPAT long_qdec

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(qdec_gpio, CONFIG_SENSOR_LOG_LEVEL);

struct qdec_config {
	struct gpio_dt_spec a;       /* interrupt pin */
	struct gpio_dt_spec b;       /* direction-reference pin */
	uint32_t pulses_per_rev;
};

struct qdec_data {
	const struct device *dev;
	struct gpio_callback cb;
	atomic_t position;           /* ISR writes, sample_fetch snapshots */
	int32_t fetched;             /* last snapshot for channel_get */
	int32_t last_position;       /* position at previous fetch */
	int64_t last_ticks;          /* k_uptime_ticks() at previous fetch */
	int32_t rpm;                 /* computed speed */
};

/*
 * ISR — fires on both edges of channel A.
 * Direction rule: delta = (A == B) ? -1 : +1
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

	/* Atomically snapshot current position */
	cur = atomic_get(&data->position);
	data->fetched = cur;

	/* Compute speed: RPM = delta_steps / (2 * ppr) * 60 / dt_sec */
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

static DEVICE_API(sensor, qdec_api) = {
	.sample_fetch = qdec_sample_fetch,
	.channel_get  = qdec_channel_get,
};

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
