/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file line_sensor.h
 * @brief Long EDC Line Sensor — 4051 mux + ADC 8-channel grayscale sensor.
 *
 * Provides raw read, white/black calibration, normalization and binary
 * thresholding for line-following robots.
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_LINE_SENSOR_H_
#define ZEPHYR_DRIVERS_SENSOR_LINE_SENSOR_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Maximum number of sensor channels. */
#define LINE_SENSOR_MAX_CHANNELS 8

/** @brief 巡线事件类型 */
typedef enum {
	LINE_EVT_NONE  = 0, /**< 正常巡线，线在中间 */
	LINE_EVT_LOST  = 1, /**< 线消失（全部通道看到白） */
	LINE_EVT_FOUND = 2, /**< 线出现（从 LOST 恢复） */
	LINE_EVT_LEFT  = 3, /**< 左路口（左侧出现暗区分支） */
	LINE_EVT_RIGHT = 4, /**< 右路口（右侧出现暗区分支） */
	LINE_EVT_CROSS = 5, /**< 十字/T 路口（左右两侧同时出现暗区） */
} line_event_t;

/**
 * @brief 读取 8 通道原始 ADC 值（均值滤波后）。
 *
 * 内部自动切 4051 地址、每通道采 8 次取平均。
 *
 * @param dev  line-sensor 设备
 * @param raw  输出数组，raw[i] = 通道 i 的原始 ADC 值
 * @return 0 成功，负值为错误码
 */
int line_sensor_read_raw(const struct device *dev, uint16_t raw[LINE_SENSOR_MAX_CHANNELS]);

/**
 * @brief 白校准 — 将所有传感器置于白色（反射面）上，记录白基准。
 *
 * 每个通道采样 32 次取中值，写入 calib_white[]。
 *
 * @param dev  line-sensor 设备
 * @return 0 成功，负值为错误码
 */
int line_sensor_calibrate_white(const struct device *dev);

/**
 * @brief 黑校准 — 将所有传感器置于黑色（线）上，记录黑基准。
 *
 * 每个通道采样 32 次取中值，写入 calib_black[]。
 *
 * @param dev  line-sensor 设备
 * @return 0 成功，负值为错误码
 */
int line_sensor_calibrate_black(const struct device *dev);

/**
 * @brief 获取归一化值（0～1000，基于 calib_white/calib_black 线性映射）。
 *
 * @param dev    line-sensor 设备
 * @param norm   输出数组，norm[i] = 通道 i 的归一化值
 * @return 0 成功，负值为错误码
 */
int line_sensor_get_normalized(const struct device *dev, uint16_t norm[LINE_SENSOR_MAX_CHANNELS]);

/**
 * @brief 获取二值化结果（每个通道 1 bit）。
 *
 * @param dev  line-sensor 设备
 * @return 8-bit bitmap，bit[i]=1 表示通道 i 看到白（反射面），0 表示黑（线）
 */
uint8_t line_sensor_get_digital(const struct device *dev);

/**
 * @brief 获取灰度阈值。
 *
 * Gray_white = (white*2 + black) / 3
 * Gray_black = (white + black*2) / 3
 *
 * @param dev         line-sensor 设备
 * @param gray_white  输出数组，白阈值
 * @param gray_black  输出数组，黑阈值
 * @return 0 成功
 */
int line_sensor_get_thresholds(const struct device *dev,
		uint16_t gray_white[LINE_SENSOR_MAX_CHANNELS],
		uint16_t gray_black[LINE_SENSOR_MAX_CHANNELS]);

/**
 * @brief 检查校准是否已完成。
 *
 * @param dev  line-sensor 设备
 * @return true  已校准
 * @return false 未校准
 */
bool line_sensor_is_calibrated(const struct device *dev);

/**
 * @brief 恢复出厂黑/白基准（raw_min / raw_max）。
 *
 * @param dev  line-sensor 设备
 */
void line_sensor_reset_calibration(const struct device *dev);

/**
 * @brief 检测巡线事件（路口、线消失等）。
 *
 * 基于最后一次 sample_fetch 得到的归一化数据，
 * 按 8 通道暗区分布判断事件类型。
 *
 * @param dev  line-sensor 设备
 * @return 当前事件类型
 */
line_event_t line_sensor_get_event(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_LINE_SENSOR_H_ */
