/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file long_qdec.c
 * @brief GPIO-based Quadrature Decoder (QDEC) sensor driver.
 *
 * 4x decoding: both channels A and B interrupt on both edges; a 16-entry LUT
 * over (prev_state, new_state) resolves direction — 4× the resolution of 2x.
 *
 * Velocity uses the period method (T-method): the CPU cycle counter
 * (k_cycle_get_32, ~12.5ns @80MHz) timestamps each edge, and RPM is taken
 * from the average of the last N edge intervals (sliding window,
 * N = CONFIG_LONG_QDEC_AVG_TAP) to damp per-edge ISR jitter. Signed via
 * last_dir so forward/reverse are distinguished.
 *
 * Channels exposed via Zephyr sensor API:
 *   - SENSOR_CHAN_ROTATION — angular position (degrees, 4x)
 *   - SENSOR_CHAN_RPM      — speed in RPM (period method, signed)
 */

#define DT_DRV_COMPAT long_qdec

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys_clock.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(qdec_gpio, CONFIG_SENSOR_LOG_LEVEL);

/* 无 edge 超过此时长 → 视为静止，fetch 端 RPM 归零。
 * 仅用于 fetch 的 idle 判定；ISR 绝不据此丢弃 edge（否则低速/静止重启时
 * 每个边沿都被判为伪间隔、窗口永远空，曾导致“关电机再开采集不到”）。 */
#define QDEC_IDLE_TIMEOUT_MS	100
#define QDEC_IDLE_TIMEOUT_CYC	((uint32_t)((uint64_t)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC \
					      * QDEC_IDLE_TIMEOUT_MS / 1000))

/** @brief ROM — 设备树解析出的硬件参数 */
struct qdec_config {
	struct gpio_dt_spec a;       /**< 通道 A — 中断引脚（双边沿触发） */
	struct gpio_dt_spec b;       /**< 通道 B — 中断引脚（双边沿触发，4x） */
	uint32_t pulses_per_rev;     /**< 每转脉冲数（完整正交周期数），0=原始步数 */
	bool output_reversed;        /**< true：编码器方向取反（物理转向与期望相反时） */
};

/** @brief RAM — 编码器运行时状态 */
struct qdec_data {
	const struct device *dev;    /**< 自引用 */
	struct gpio_callback a_cb;   /**< 通道 A 的 GPIO 回调（A/B 可能不在同一 port） */
	struct gpio_callback b_cb;   /**< 通道 B 的 GPIO 回调 */
	atomic_t position;           /**< ISR 原子累加（4x 步数），sample_fetch 快照捕获 */
	int32_t fetched;             /**< 最后一次 fetch 的位置快照（供 ROTATION） */
	float rpm;                   /**< 周期法算出的转速（带符号） */
	uint8_t prev_state;          /**< 上一次 A/B 正交状态 (A<<1)|B ∈ 0..3，供 LUT */
	volatile uint32_t last_edge_cycles;     /**< 最近一次 edge 的 CPU cycle 时间戳 */
	/* 滑动窗口测速：环形存最近 N 个 edge 间隔(cycles)，N-edge 平均抑制单次
	 * ISR 抖动(周期法对单点间隔敏感)。N = CONFIG_LONG_QDEC_AVG_TAP。 */
	volatile uint32_t interval_buf[CONFIG_LONG_QDEC_AVG_TAP]; /**< 间隔环形 buffer */
	volatile uint32_t interval_sum;         /**< buffer 内间隔之和(滑动 O(1) 维护) */
	volatile uint8_t interval_count;        /**< buffer 内有效间隔数(0..N) */
	uint8_t interval_head;                  /**< 环形写入位置(ISR 私有) */
	int8_t last_dir;                        /**< 最近一次方向 ±1，供 RPM 符号 */
};

/* 4x 正交解码查找表：索引 = (prev_state << 2) | new_state，值 = +1/-1/0。
 * 状态编码 (A<<1)|B，正交正序 00→10→11→01→00 记为正向。 */
static const int8_t qdec_lut[16] = {
	 0, -1, +1,  0,
	+1,  0,  0, -1,
	-1,  0,  0, +1,
	 0, +1, -1,  0,
};

/** @brief 4x 正交解码 + 周期法时间戳采集（A/B 任一边沿触发）。 */
static void qdec_isr_common(struct qdec_data *data, const struct qdec_config *cfg)
{
	int a = gpio_pin_get_dt(&cfg->a);
	int b = gpio_pin_get_dt(&cfg->b);
	uint8_t new_state = (uint8_t)((a << 1) | b);

	int delta = qdec_lut[(data->prev_state << 2) | new_state];
	data->prev_state = new_state;

	if (cfg->output_reversed) {
		delta = -delta;
	}

	if (delta != 0) {
		uint32_t now = k_cycle_get_32();
		uint32_t interval = now - data->last_edge_cycles;
		data->last_edge_cycles = now;

		/* 所有有效 edge 一律进窗口——绝不按 interval 大小丢弃或重置。
		 * 早先用 interval<IDLE_TIMEOUT 过滤、否则清 count=0：低速或静止
		 * 重启时每个 edge 都被判为伪间隔、count 永远 0 → RPM 恒 0
		 * （“关电机再开采集不到”的根因）。idle 判定移到 fetch 端做。
		 * 跨 idle 的首个大 interval 会短暂拉低平均，被后续 N 个 edge 冲掉。 */
		data->interval_sum  -= data->interval_buf[data->interval_head];
		data->interval_buf[data->interval_head] = interval;
		data->interval_sum  += interval;
		data->interval_head  = (uint8_t)((data->interval_head + 1) %
						 CONFIG_LONG_QDEC_AVG_TAP);
		if (data->interval_count < CONFIG_LONG_QDEC_AVG_TAP) {
			data->interval_count++;
		}
		data->last_dir = (int8_t)delta;
		atomic_add(&data->position, delta);
	}
}

static void qdec_a_isr(const struct device *port, struct gpio_callback *cb,
		       gpio_port_pins_t pins)
{
	struct qdec_data *data = CONTAINER_OF(cb, struct qdec_data, a_cb);
	qdec_isr_common(data, data->dev->config);
}

static void qdec_b_isr(const struct device *port, struct gpio_callback *cb,
		       gpio_port_pins_t pins)
{
	struct qdec_data *data = CONTAINER_OF(cb, struct qdec_data, b_cb);
	qdec_isr_common(data, data->dev->config);
}

/**
 * @brief 快照位置 + 周期法(N-edge 平均)计算瞬时 RPM(带符号)。
 *
 * N 个 edge 跨 sum cycles → RPM = 15 × count × cyc_per_sec / (ppr × sum)，
 * 符号取 last_dir。最近 QDEC_IDLE_TIMEOUT_MS 内无 edge 则视为静止(RPM=0)。
 *
 * @retval 0 成功
 * @retval -ENOTSUP 不支持的通道
 */
static int qdec_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct qdec_data *data = dev->data;
	const struct qdec_config *cfg = dev->config;

	if (chan != SENSOR_CHAN_ALL &&
	    chan != SENSOR_CHAN_ROTATION &&
	    chan != SENSOR_CHAN_RPM) {
		return -ENOTSUP;
	}

	/* 关中断取一致快照：position(atomic) + 滑动窗口计速字段 */
	unsigned int key = irq_lock();
	int32_t cur = atomic_get(&data->position);
	uint32_t sum = data->interval_sum;
	uint8_t count = data->interval_count;
	int8_t dir = data->last_dir;
	uint32_t last_edge = data->last_edge_cycles;
	irq_unlock(key);

	data->fetched = cur;

	if (cfg->pulses_per_rev > 0 && count > 0 &&
	    (k_cycle_get_32() - last_edge) < QDEC_IDLE_TIMEOUT_CYC) {
		/* N 个 edge 跨 sum cycles → RPM = 15 × count × cyc_per_sec / (ppr × sum)
		 * (等价于用平均间隔 avg=sum/count 代入单点公式，但避免整数除法截断) */
		float mag = 15.0f * (float)count * (float)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC /
			    ((float)cfg->pulses_per_rev * (float)sum);
		data->rpm = (dir >= 0) ? mag : -mag;
	} else {
		data->rpm = 0.0f;
	}

	return 0;
}

/**
 * @brief 返回缓存的传感器通道值。
 * ROTATION：度（4x，count×90/ppr）；RPM：周期法转速(带符号)。
 */
static int qdec_channel_get(const struct device *dev, enum sensor_channel chan,
			    struct sensor_value *val)
{
	struct qdec_data *data = dev->data;
	const struct qdec_config *cfg = dev->config;

	switch (chan) {
	case SENSOR_CHAN_ROTATION:
		if (cfg->pulses_per_rev > 0) {
			/* 4x：每转 4×ppr 个 count → deg = count × 360/(4×ppr) = count×90/ppr */
			int64_t tmp = (int64_t)data->fetched * 90000000LL /
				      (int32_t)cfg->pulses_per_rev;
			val->val1 = (int32_t)(tmp / 1000000LL);
			val->val2 = (int32_t)(tmp % 1000000LL);
		} else {
			val->val1 = data->fetched;
			val->val2 = 0;
		}
		break;

	case SENSOR_CHAN_RPM:
		sensor_value_from_double(val, (double)data->rpm);
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
 * @brief QDEC 硬件初始化 — A/B 都配双边沿中断，注册各自的回调。
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
	/* prev_state 取当前 A/B，避免首个边沿 LUT 误判；时间戳基准取此刻 */
	data->prev_state = (uint8_t)((gpio_pin_get_dt(&cfg->a) << 1) |
				     gpio_pin_get_dt(&cfg->b));
	data->last_edge_cycles = k_cycle_get_32();

	/* A、B 可能不在同一 GPIO port（如左轮 PA2/PB24），各自注册一个回调 */
	gpio_init_callback(&data->a_cb, qdec_a_isr, BIT(cfg->a.pin));
	gpio_init_callback(&data->b_cb, qdec_b_isr, BIT(cfg->b.pin));

	ret = gpio_add_callback_dt(&cfg->a, &data->a_cb);
	if (ret < 0) {
		LOG_ERR("add callback A failed: %d", ret);
		return ret;
	}
	ret = gpio_add_callback_dt(&cfg->b, &data->b_cb);
	if (ret < 0) {
		LOG_ERR("add callback B failed: %d", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->a, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		LOG_ERR("configure interrupt A failed: %d", ret);
		return ret;
	}
	ret = gpio_pin_interrupt_configure_dt(&cfg->b, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		LOG_ERR("configure interrupt B failed: %d", ret);
		return ret;
	}

	LOG_DBG("QDEC ready (4x): A=pin%d B=pin%d", cfg->a.pin, cfg->b.pin);

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
		.output_reversed = DT_INST_PROP(n, output_reversed),        \
	};                                                                   \
									     \
	SENSOR_DEVICE_DT_INST_DEFINE(n, qdec_init, NULL,                    \
				     &qdec_data_##n, &qdec_config_##n,       \
				     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
				     &qdec_api);

DT_INST_FOREACH_STATUS_OKAY(QDEC_GPIO_INIT)
