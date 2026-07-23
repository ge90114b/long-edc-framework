/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_DRIVERS_MOTOR_LONG_MOTOR_H_
#define ZEPHYR_DRIVERS_MOTOR_LONG_MOTOR_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PID 参数（应用层注入，方便调试时在线调参）
 */
struct motor_pid_config {
	/** 比例增益 */
	float kp;
	/** 积分增益 */
	float ki;
	/** 微分增益 */
	float kd;
	/** 积分限幅（抗饱和） */
	float integral_max;
	/** 输出限幅（PWM 占空比百分比，0～100） */
	float output_max;
	/** PID 控制周期（毫秒）。motor 驱动以此间隔读取 QDEC 并计算 PID 输出 */
	uint32_t period_ms;
};

/**
 * @brief 注入 PID 参数并激活电机闭环控制。
 *
 *        调用前：PWM 占空比为 0，电机不转。
 *        调用后：按 period_ms 周期启动 QDEC 采样 → PID → PWM 更新。
 *
 *        减速比 gear-ratio 从设备树读取，get_speed/get_distance
 *        返回的已经是轮子侧的 RPM 和转数。
 *
 * @param dev  motor 设备
 * @param cfg  PID 参数，不可为 NULL
 * @return 0 成功，负值为错误码
 */
int motor_init(const struct device *dev, const struct motor_pid_config *cfg);

/**
 * @brief 设定目标转速（轮子侧 RPM），PID 自动跟踪。
 * @param dev  motor 设备
 * @param rpm  目标转速，正 = 前进，负 = 后退
 * @return 0 成功
 */
int motor_set_speed(const struct device *dev, float rpm);

/**
 * @brief 获取当前转速（轮子侧 RPM，已考虑减速比）。
 * @param dev  motor 设备
 * @return 转速 RPM，正 = 前进，负 = 后退
 */
float motor_get_speed(const struct device *dev);

/**
 * @brief 获取累计距离（轮子转数，已考虑减速比）。
 * @param dev  motor 设备
 * @return 累计转数（revolutions）
 */
float motor_get_distance(const struct device *dev);

/**
 * @brief 清零累计距离。轮式里程计每次出发前调用。
 * @param dev  motor 设备
 */
void motor_reset_distance(const struct device *dev);

/**
 * @brief 急停：目标归零，PWM 占空比归零。
 * @param dev  motor 设备
 * @return 0 成功
 */
int motor_stop(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_MOTOR_LONG_MOTOR_H_ */
