/*
 * Copyright (c) 2025 Long EDC Framework
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file long_motor.c
 * @brief Motor driver — PWM speed control + direction GPIO + QDEC feedback
 *        with built-in PID speed loop.
 */

#define DT_DRV_COMPAT long_motor

#include "long_motor.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <math.h>

LOG_MODULE_REGISTER(long_motor, CONFIG_MOTOR_LOG_LEVEL);

/* ---------- data / config ---------- */

/** @brief ROM — 设备树解析出的硬件参数 */
struct motor_config {
	struct pwm_dt_spec  pwm;       /**< PWM specifier (devicetree) */
	struct gpio_dt_spec dir;       /**< Direction GPIO (devicetree) */
	const struct device *qdec;     /**< QDEC sensor device pointer */
	int                 gear_ratio; /**< 减速比 N:1，电机 N 转 = 轮子 1 转 */
};

/** @brief RAM — 闭环运行时状态 */
struct motor_data {
	const struct motor_pid_config *pid_cfg;  /* ptr to app's live pid_cfg — Ozone tuneable */

	float target_speed;       /**< 目标轮子 RPM（带符号） */
	float current_speed;      /**< 缓存：最近一次 PID 周期测得的轮子 RPM */
	float distance;           /**< 累计轮子转数 */
	float rotation_offset;    /**< qdec rotation 基准值（reset_distance 时重设） */

	float pid_integral;       /**< PID 积分项 */
	float pid_prev_error;     /**< 上一周期 PID 误差 */

	bool  initialized;        /**< motor_init() 是否已被调用 */

	const struct device      *dev; /**< 自引用 — work handler 需要拿 config */
	struct k_mutex            lock;        /**< 保护 data 的互斥锁 */
	struct k_work_delayable   pid_work;    /**< PID 周期调度工作项 */
};

/* ---------- PWM 输出辅助 ---------- */

/**
 * @brief 写电机 PWM 占空比，pulse ∈ [0, period] 对应 0%..100%。
 *
 * MSPM0 GPTimer 在 EDGE_ALIGN 模式下，driverlib 把 CC 输出动作配成
 * LACT=高 / CDACT=低（见 dl_timer.c 的 DL_Timer_initTwoCCPWMMode），
 * 也就是 CC 比较值越小、占空比越大，CC=0 时输出恒高（=100%）。
 * 若把 pulse 原样透传给 pwm_set_pulse_dt，pulse=0 反而是全速运转
 * （这正是“一上电轮子就自己转”的根因）。这里翻转 pulse，使外部语义
 * 符合直觉：pulse=0 → 0% 占空比（停车），pulse=period → 100%。
 */
static void motor_pwm_write(const struct motor_config *cfg, uint32_t pulse)
{
	if (pulse > cfg->pwm.period) {
		pulse = cfg->pwm.period;
	}
	pwm_set_pulse_dt(&cfg->pwm, cfg->pwm.period - pulse);
}

/* ---------- PID work handler ---------- */

/**
 * @brief PID 周期控制回调。
 *
 * 在 system workqueue 中执行，周期 = pid.period_ms。
 * 1. 从 QDEC 读取电机侧 RPM 和累计角度
 * 2. 除以 gear_ratio 转换到轮子侧
 * 3. 以绝对转速做 PID 闭环
 * 4. 更新方向 GPIO 和 PWM 占空比
 */
static void motor_pid_tick(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct motor_data *data = CONTAINER_OF(dwork, struct motor_data, pid_work);
	const struct motor_config *cfg = data->dev->config;
	struct sensor_value val;
	float motor_rpm, motor_dist;
	float feedback, error, p, i, d, out;
	float dt;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (!data->initialized) {
		k_mutex_unlock(&data->lock);
		k_work_schedule(&data->pid_work, K_MSEC(data->pid_cfg->period_ms));
		return;
	}

	/* 目标转速 0 也走完整 PID（error = -feedback，PID 输出被下限 clamp 到 0%），
	 * 全程闭环，无开环关断特判。 */

	/* 1. 读取 QDEC */
	if (sensor_sample_fetch(cfg->qdec) < 0) {
		LOG_WRN("qdec fetch failed");
		k_mutex_unlock(&data->lock);
		k_work_schedule(&data->pid_work, K_MSEC(data->pid_cfg->period_ms));
		return;
	}

	sensor_channel_get(cfg->qdec, SENSOR_CHAN_RPM, &val);
	motor_rpm = sensor_value_to_double(&val);

	sensor_channel_get(cfg->qdec, SENSOR_CHAN_ROTATION, &val);
	motor_dist = sensor_value_to_double(&val) / 360.0; /* deg → motor rev */

	/* 2. 转换为轮子侧 */
	data->current_speed = motor_rpm / cfg->gear_ratio;
	data->distance = (motor_dist / cfg->gear_ratio) - data->rotation_offset;

	/* 3. PID（绝对转速控制） */
	feedback = fabsf(data->current_speed);
	error    = fabsf(data->target_speed) - feedback;
	dt       = data->pid_cfg->period_ms / 1000.0f;

	p = data->pid_cfg->kp * error;

	data->pid_integral += data->pid_cfg->ki * error * dt;
	if (data->pid_integral > data->pid_cfg->integral_max) {
		data->pid_integral = data->pid_cfg->integral_max;
	} else if (data->pid_integral < -data->pid_cfg->integral_max) {
		data->pid_integral = -data->pid_cfg->integral_max;
	}
	i = data->pid_integral;

	d = data->pid_cfg->kd * (error - data->pid_prev_error) / dt;
	data->pid_prev_error = error;

	out = p + i + d;
	if (out > data->pid_cfg->output_max) {
		out = data->pid_cfg->output_max;
	} else if (out < 0.0f) {
		out = 0.0f;
	}

	/* 4. 方向 + PWM 输出 */
	gpio_pin_set_dt(&cfg->dir, data->target_speed >= 0.0f ? 1 : 0);
	motor_pwm_write(cfg, (uint32_t)(cfg->pwm.period * out / 100.0f));

	k_mutex_unlock(&data->lock);
	k_work_schedule(&data->pid_work, K_MSEC(data->pid_cfg->period_ms));
}

/* ========== Public API ========== */

int motor_init(const struct device *dev, const struct motor_pid_config *cfg)
{
	struct motor_data *data = dev->data;

	if (cfg == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	if (data->initialized) {
		/* 允许运行时重新调参 */
		data->pid_cfg = cfg;
		data->pid_integral = 0.0f;
		data->pid_prev_error = 0.0f;
		k_mutex_unlock(&data->lock);
		return 0;
	}

	data->pid_cfg = cfg;
	data->initialized = true;

	motor_reset_distance(dev);

	k_mutex_unlock(&data->lock);

	k_work_schedule(&data->pid_work, K_MSEC(cfg->period_ms));

	LOG_INF("motor pid started: Kp=%.3f Ki=%.3f Kd=%.3f period=%ums",
		(double)cfg->kp, (double)cfg->ki, (double)cfg->kd,
		cfg->period_ms);

	return 0;
}

int motor_set_speed(const struct device *dev, float rpm)
{
	struct motor_data *data = dev->data;

	if (!data->initialized) {
		return -EACCES;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* 大幅变速（>50%）才复位积分，避免正常巡线时 I 项无效 */
	float delta = rpm - data->target_speed;
	if (fabsf(delta) > fabsf(data->target_speed) * 0.5f) {
		data->pid_integral = 0.0f;
		data->pid_prev_error = 0.0f;
	}
	data->target_speed = rpm;

	k_mutex_unlock(&data->lock);

	return 0;
}

float motor_get_speed(const struct device *dev)
{
	struct motor_data *data = dev->data;
	float speed;

	k_mutex_lock(&data->lock, K_FOREVER);
	speed = data->current_speed;
	k_mutex_unlock(&data->lock);

	return speed;
}

float motor_get_distance(const struct device *dev)
{
	struct motor_data *data = dev->data;
	float dist;

	k_mutex_lock(&data->lock, K_FOREVER);
	dist = data->distance;
	k_mutex_unlock(&data->lock);

	return dist;
}

void motor_reset_distance(const struct device *dev)
{
	struct motor_data *data = dev->data;
	const struct motor_config *cfg = dev->config;
	struct sensor_value val;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (sensor_sample_fetch(cfg->qdec) == 0) {
		sensor_channel_get(cfg->qdec, SENSOR_CHAN_ROTATION, &val);
		data->rotation_offset = sensor_value_to_double(&val) / 360.0
					/ cfg->gear_ratio;
	}
	data->distance = 0.0f;

	k_mutex_unlock(&data->lock);
}

int motor_stop(const struct device *dev)
{
	struct motor_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	/* 全程 PID：stop 仅把目标设为 0，由 PID 周期闭环制动到 0，不做开环关断。
	 * 复位积分/微分状态，避免残留影响下次启动。 */
	data->target_speed = 0.0f;
	data->pid_integral = 0.0f;
	data->pid_prev_error = 0.0f;

	k_mutex_unlock(&data->lock);

	return 0;
}

/* ========== Zephyr 设备注册 ========== */

/**
 * @brief Zephyr 设备初始化 — 硬件检查 + 默认状态设置。
 *
 * POST_KERNEL 阶段自动调用，仅做硬件准备，不启动 PID。
 */
static int motor_device_init(const struct device *dev)
{
	struct motor_data *data = dev->data;
	const struct motor_config *cfg = dev->config;

	if (!device_is_ready(cfg->qdec)) {
		LOG_ERR("qdec not ready");
		return -ENODEV;
	}
	if (!pwm_is_ready_dt(&cfg->pwm)) {
		LOG_ERR("pwm not ready");
		return -ENODEV;
	}
	if (!gpio_is_ready_dt(&cfg->dir)) {
		LOG_ERR("dir gpio not ready");
		return -ENODEV;
	}

	k_mutex_init(&data->lock);
	k_work_init_delayable(&data->pid_work, motor_pid_tick);

	gpio_pin_configure_dt(&cfg->dir, GPIO_OUTPUT_INACTIVE);
	gpio_pin_set_dt(&cfg->dir, 1);          /* 默认正向 */
	motor_pwm_write(cfg, 0);                /* 0% 占空比（停车）*/

	data->dev = dev;

	return 0;
}

/** @brief 设备注册宏 — 为每个 DT status=okay 的实例生成 data/config/device */
#define MOTOR_DEFINE(n)                                                        \
	static struct motor_data motor_data_##n = {                            \
		.lock = Z_MUTEX_INITIALIZER(motor_data_##n.lock),              \
	};                                                                     \
	static const struct motor_config motor_config_##n = {                  \
		.pwm  = PWM_DT_SPEC_GET(DT_DRV_INST(n)),                       \
		.dir  = GPIO_DT_SPEC_GET(DT_DRV_INST(n), dir_gpios),           \
		.qdec = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(n), qdec)),       \
		.gear_ratio = DT_INST_PROP(n, gear_ratio),                      \
	};                                                                     \
	DEVICE_DT_INST_DEFINE(n, motor_device_init, NULL,                      \
			      &motor_data_##n, &motor_config_##n,              \
			      POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY,         \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(MOTOR_DEFINE)
