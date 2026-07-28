/* SPDX-License-Identifier: MIT */
/**
 * @file icm45686.c
 * @brief ICM45686 6-axis IMU driver with integrated VQF orientation filter.
 *
 * Uses the sensor's internal 32 kHz RC oscillator — no external crystal
 * required.  Runs at 100 Hz ODR with all-float VQF sensor fusion, suitable
 * for Cortex-M0+/M4 MCUs without FPU.
 *
 * Outputs (via Zephyr sensor API):
 *   - SENSOR_CHAN_ROTATION     — yaw (degrees)
 *   - SENSOR_CHAN_GYRO_X/Y/Z   — debiased angular velocity (rad/s)
 *
 * @see dts/bindings/sensor/long-icm45686.yaml
 */

#define DT_DRV_COMPAT long_icm45686
#define VQF_SINGLE_PRECISION   /* all-float VQF — safe at 100 Hz */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <math.h>

#include "vqf.h"
#include "icm45686_regs.h"

LOG_MODULE_REGISTER(icm45686, LOG_LEVEL_INF);

/* ------------------------------------------------------------------ */
/*  Per-instance data                                                 */
/* ------------------------------------------------------------------ */

/** @brief ROM — I²C 总线配置 */
struct icm_config {
	struct i2c_dt_spec bus;
};

/** @brief RAM — VQF 状态 + 缓存传感器输出 */
struct icm_data {
	vqf_params_t  vqf_params;     /**< VQF 可调参数 */
	vqf_state_t   vqf_state;      /**< VQF 滤波器状态 */
	vqf_coeffs_t  vqf_coeffs;     /**< VQF 预计算系数 */
	bool          vqf_ready;      /**< VQF 初始化完成 */

	struct i2c_dt_spec bus;       /**< 运行期总线 spec（地址自动探测后填入） */

	float euler[3];               /**< 缓存：roll, pitch, yaw (度) */
	float gyro_debiased[3];       /**< 缓存：去偏置角速度 (rad/s) */

	uint8_t fifo_buf[256] __aligned(4); /**< FIFO 读取缓冲 */

	struct k_work_delayable feed_work;  /**< 100Hz 自动喂 VQF */
	const struct device      *dev;      /**< 自引用 */
	struct k_mutex lock;                /**< 保护缓存 */
};

/* ------------------------------------------------------------------ */
/*  I²C helpers                                                        */
/* ------------------------------------------------------------------ */

/** @brief 写一字节寄存器 */
static inline int icm_write(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t val)
{
	uint8_t buf[2] = { reg, val };
	return i2c_write_dt(bus, buf, sizeof(buf));
}

/** @brief 读一字节寄存器 */
static inline int icm_read(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t *val)
{
	return i2c_reg_read_byte_dt(bus, reg, val);
}

/** @brief 连续读多字节 */
static int icm_burst_read(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t *data, size_t len)
{
	return i2c_burst_read_dt(bus, reg, data, len);
}

/** @brief 连续写多字节 */
static int icm_burst_write(const struct i2c_dt_spec *bus, uint8_t reg, const uint8_t *data, size_t len)
{
	uint8_t buf[64];

	if (len + 1 > sizeof(buf)) {
		return -E2BIG;
	}
	buf[0] = reg;
	memcpy(&buf[1], data, len);
	return i2c_write_dt(bus, buf, len + 1);
}

/* ------------------------------------------------------------------ */
/*  IREG access (indirect registers)                                   */
/* ------------------------------------------------------------------ */

/**
 * @brief 写一个间接寄存器（IREG）。
 *
 * 数据格式：bank | addr | data，发到 ICM45686_IREG_ADDR_15_8。
 */
static int icm_write_ireg(const struct i2c_dt_spec *bus,
			  uint8_t bank, uint8_t addr, uint8_t data)
{
	uint8_t buf[3] = { bank, addr, data };
	return icm_burst_write(bus, ICM45686_IREG_ADDR_15_8, buf, 3);
}

/**
 * @brief 读间接寄存器（IREG），支持批量读取。
 *
 * 每次 IREG 读取需间隔 ≥ 4 µs（datasheet §5.2）。
 */
static int icm_read_ireg(const struct i2c_dt_spec *bus,
			 uint8_t bank, uint8_t addr, uint8_t *data, size_t len)
{
	int ret;
	uint8_t addr_buf[2] = { bank, addr };

	ret = icm_burst_write(bus, ICM45686_IREG_ADDR_15_8, addr_buf, 2);
	if (ret) { return ret; }

	for (size_t i = 0; i < len; i++) {
		k_usleep(4);
		ret = icm_read(bus, ICM45686_IREG_DATA, &data[i]);
		if (ret) { return ret; }
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/*  FIFO helpers                                                       */
/* ------------------------------------------------------------------ */

/**
 * @brief 读取 FIFO 数据帧计数。
 *
 * 参考驱动双读法：读两次，取第二次结果。stream 模式下 FIFO 写入指针
 * 滞后一帧，返回值减去 1（与验证过的参考驱动一致）。
 *
 * @param[out] count FIFO 中可读数据帧数（每帧 = ICM45686_FIFO_PACKET_SIZE 字节）
 * @retval 0 成功
 * @retval 负值 I²C 错误
 */
static int read_fifo_count(const struct i2c_dt_spec *bus, uint16_t *count)
{
	int ret;

	ret = icm_burst_read(bus, ICM45686_FIFO_COUNT_0, (uint8_t *)count, 2);
	if (ret) { return ret; }
	ret = icm_burst_read(bus, ICM45686_FIFO_COUNT_0, (uint8_t *)count, 2);
	if (ret) { return ret; }

	if (*count > 1) { *count -= 1; }
	return 0;
}

/** @brief 通过翻转 FIFO_CONFIG2 bit7 清空 FIFO。 */
static int flush_fifo(const struct i2c_dt_spec *bus)
{
	uint8_t val;
	int ret = icm_read(bus, ICM45686_FIFO_CONFIG2, &val);
	if (ret) { return ret; }
	val |= BIT(7);
	return icm_write(bus, ICM45686_FIFO_CONFIG2, val);
}

/**
 * @brief 解析一包 20-byte FIFO 数据（20-bit hires 模式）。
 *
 * 包格式（LE）：
 *   [0]         Header（期望 0x78）
 *   [1 — 6]     Accel X/Y/Z  MSB (16-bit)
 *   [7 — 12]    Gyro  X/Y/Z  MSB (16-bit)
 *   [13 — 14]   温度 (16-bit)
 *   [15 — 16]   时间戳 (16-bit, 1µs)
 *   [17 — 19]   扩展：高 nibble = accel LSB，低 nibble = gyro LSB
 *
 * 20-bit 有符号 → 32G/4000dps 物理值。
 *
 * @param[in]  data  20-byte 原始包
 * @param[out] accel 加速度 (m/s²)
 * @param[out] gyro  角速度 (rad/s)
 * @retval 0  成功
 * @retval -1 header 无效
 */
static int parse_fifo_packet(const uint8_t *data, float accel[3], float gyro[3])
{
	if (data[0] != ICM45686_FIFO_HEADER_20BIT) {
		return -1;
	}

	for (int i = 0; i < 3; i++) {
		int16_t a_hi = sys_get_le16(data + 1 + i * 2);
		int16_t g_hi = sys_get_le16(data + 7 + i * 2);
		uint8_t ext = data[17 + i];

		int32_t a_20 = ((int32_t)a_hi << 4) | ((ext >> 4) & 0x0F);
		int32_t g_20 = ((int32_t)g_hi << 4) | (ext & 0x0F);

		/* 符号扩展 20-bit → 32-bit */
		if (a_20 & 0x80000) { a_20 |= 0xFFF00000UL; }
		if (g_20 & 0x80000) { g_20 |= 0xFFF00000UL; }

		accel[i] = (float)a_20 * ICM45686_ACCEL_SENSITIVITY_20BIT
			   * ICM45686_EARTH_GRAVITY;
		gyro[i]  = (float)g_20 * ICM45686_GYRO_SENSITIVITY_20BIT
			   * ICM45686_DEG_TO_RAD;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/*  Quaternion → Euler angles                                          */
/* ------------------------------------------------------------------ */

/**
 * @brief 四元数 → 欧拉角（roll / pitch / yaw，度）。
 *
 * @param[in]  q     单位四元数 (w, x, y, z)
 * @param[out] euler [roll, pitch, yaw] 度
 */
static void quat_to_euler(const float q[4], float euler[3])
{
	float w = q[0], x = q[1], y = q[2], z = q[3];

	euler[0] = atan2f(2.0f * (w * x + y * z),
			  1.0f - 2.0f * (x * x + y * y));

	float sin_pitch = 2.0f * (w * y - z * x);
	if (sin_pitch > 1.0f)  { sin_pitch = 1.0f; }
	if (sin_pitch < -1.0f) { sin_pitch = -1.0f; }
	euler[1] = asinf(sin_pitch);

	euler[2] = atan2f(2.0f * (w * z + x * y),
			  1.0f - 2.0f * (y * y + z * z));

	euler[0] *= ICM45686_RAD_TO_DEG;
	euler[1] *= ICM45686_RAD_TO_DEG;
	euler[2] *= ICM45686_RAD_TO_DEG;
}

/* ------------------------------------------------------------------ */
/*  Internal: FIFO → VQF pipeline (caller must hold lock)             */
/* ------------------------------------------------------------------ */

/** @brief 读一次 FIFO，喂 VQF，缓存最新的欧拉角 + 去偏置陀螺仪。 */
static void icm_process_fifo(struct icm_data *data)
{
	uint16_t fifo_count;

	if (read_fifo_count(&data->bus, &fifo_count) < 0 || fifo_count == 0) {
		return;
	}

	size_t bytes = fifo_count * ICM45686_FIFO_PACKET_SIZE;
	if (bytes > sizeof(data->fifo_buf)) {
		bytes = sizeof(data->fifo_buf);
		fifo_count = (uint16_t)(bytes / ICM45686_FIFO_PACKET_SIZE);
	}

	if (icm_burst_read(&data->bus, ICM45686_FIFO_DATA, data->fifo_buf, bytes) < 0) {
		return;
	}

	int parsed = 0;
	float last_gyro[3] = { 0 };
	for (uint16_t i = 0; i < fifo_count; i++) {
		float accel[3], gyro[3];

		if (parse_fifo_packet(&data->fifo_buf[i * ICM45686_FIFO_PACKET_SIZE],
				      accel, gyro) != 0) {
			continue;
		}
		parsed++;

		updateGyr(&data->vqf_params, &data->vqf_state, &data->vqf_coeffs, gyro);
		updateAcc(&data->vqf_params, &data->vqf_state, &data->vqf_coeffs, accel);

		last_gyro[0] = gyro[0];
		last_gyro[1] = gyro[1];
		last_gyro[2] = gyro[2];
	}

	if (parsed == 0) {
		return;
	}

	float quat[4];
	getQuat6D(&data->vqf_state, quat);
	quat_to_euler(quat, data->euler);

	float bias[3];
	getBiasEstimate(&data->vqf_state, &data->vqf_coeffs, bias);

	data->gyro_debiased[0] = last_gyro[0] - bias[0];
	data->gyro_debiased[1] = last_gyro[1] - bias[1];
	data->gyro_debiased[2] = last_gyro[2] - bias[2];
}

/* ------------------------------------------------------------------ */
/*  Auto-feed timer (100 Hz)                                           */
/* ------------------------------------------------------------------ */

/** @brief 100Hz work handler — 锁 → 处理 FIFO → 解锁 → 重新调度。 */
static void icm_feed_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct icm_data *data = CONTAINER_OF(dwork, struct icm_data, feed_work);

	k_mutex_lock(&data->lock, K_FOREVER);
	icm_process_fifo(data);
	k_mutex_unlock(&data->lock);

	k_work_schedule(&data->feed_work, K_MSEC(10));
}

/* ------------------------------------------------------------------ */
/*  Zephyr sensor driver API                                           */
/* ------------------------------------------------------------------ */

static int icm_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct icm_data *data = dev->data;

	ARG_UNUSED(chan);

	k_mutex_lock(&data->lock, K_FOREVER);
	icm_process_fifo(data);
	k_mutex_unlock(&data->lock);

	return 0;
}

/**
 * @brief 返回缓存传感器通道值。
 *
 * 支持的通道：
 *   - SENSOR_CHAN_ROTATION — yaw（度）
 *   - SENSOR_CHAN_GYRO_X/Y/Z — 去偏置角速度（rad/s）
 */
static int icm_channel_get(const struct device *dev,
			   enum sensor_channel chan,
			   struct sensor_value *val)
{
	struct icm_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	switch (chan) {
	case SENSOR_CHAN_ROTATION:
		sensor_value_from_double(val, (double)data->euler[2]);
		break;
	case SENSOR_CHAN_GYRO_X:
		sensor_value_from_double(val, (double)data->gyro_debiased[0]);
		break;
	case SENSOR_CHAN_GYRO_Y:
		sensor_value_from_double(val, (double)data->gyro_debiased[1]);
		break;
	case SENSOR_CHAN_GYRO_Z:
		sensor_value_from_double(val, (double)data->gyro_debiased[2]);
		break;
	default:
		k_mutex_unlock(&data->lock);
		return -ENOTSUP;
	}

	k_mutex_unlock(&data->lock);
	return 0;
}

/** @brief Zephyr sensor 驱动 API 表 */
static const struct sensor_driver_api icm_api = {
	.sample_fetch = icm_sample_fetch,
	.channel_get  = icm_channel_get,
};

/* ------------------------------------------------------------------ */
/*  Address auto-detection (AD0 unreliable → probe 0x68/0x69)          */
/* ------------------------------------------------------------------ */

/* AD0 can leave the IMU at 0x68 or 0x69 depending on module wiring, so probe
 * both and adopt whichever answers with WHO_AM_I = 0xE9. Retries ride out the
 * power-on bus flakiness seen on this board. */
static int icm_detect_addr(const struct device *ctrl, uint16_t *out_addr)
{
	static const uint16_t addrs[] = { 0x68, 0x69 };

	for (int i = 0; i < ARRAY_SIZE(addrs); i++) {
		struct i2c_dt_spec spec = { .bus = ctrl, .addr = addrs[i] };

		for (int attempt = 0; attempt < 5; attempt++) {
			uint8_t who = 0;

			if (icm_read(&spec, ICM45686_WHO_AM_I, &who) == 0) {
				if (who == 0xE9) {
					*out_addr = addrs[i];
					return 0;
				}
				break;	/* answers, but wrong device — stop retrying */
			}
			k_msleep(5);
		}
	}
	return -ENODEV;
}

/* ------------------------------------------------------------------ */
/*  Initialisation                                                     */
/* ------------------------------------------------------------------ */

/**
 * @brief ICM45686 硬件初始化 + VQF 启动。
 *
 * 步骤：
 *   1. I²C 总线检查 + WHO_AM_I 验证（期望 0xE9）
 *   2. 软复位
 *   3. 内部上拉配置 + 端序设置
 *   4. 传感器配置：100 Hz ODR, 32G, 4000 dps, 低噪声
 *   5. 数字滤波器：AAF only, UI LPF = ODR/4
 *   6. FIFO 配置：2 KB 深度, timestamp, hi-res + accel + gyro, stream 模式
 *   7. 等待 FIFO 填充，验证数据流
 *   8. VQF 初始化（gyrTs = accTs = 0.01, 无磁力计）
 */
static int icm_init(const struct device *dev)
{
	const struct icm_config *cfg = dev->config;
	struct icm_data *data = dev->data;
	int ret;

	/* I²C bus check */
	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* Let the IMU finish power-on boot and settle its I2C address (AD0).
	 * We run at POST_KERNEL within ~10 ms of power-on; in that window the
	 * device's address is unstable and it can ACK the wrong address or NACK,
	 * making WHO_AM_I / register reads fail with -EIO. */
	k_sleep(K_MSEC(50));

	/* Auto-detect address (0x68 or 0x69). AD0 is unreliable on this board, so
	 * probe both and keep whichever answers with a valid WHO_AM_I. */
	uint16_t addr;
	ret = icm_detect_addr(cfg->bus.bus, &addr);
	if (ret) {
		LOG_ERR("ICM45686 not found at 0x68/0x69 (WHO_AM_I failed)");
		return ret;
	}
	data->bus.bus = cfg->bus.bus;
	data->bus.addr = addr;
	LOG_INF("ICM45686 found at 0x%02X", addr);

	/* Soft reset DISABLED for bring-up: resetting re-samples the floating AD0
	 * pin and flips the I²C address, breaking every transaction after it. The
	 * device is already in its power-on default state, so configure it directly.
	 * Re-enable this block (and tie AD0 in hardware) once the pin is fixed. */
#if 0
	ret = icm_write(&data->bus, ICM45686_REG_MISC2, 0x02);
	if (ret) { return ret; }
	k_sleep(K_MSEC(50));

	ret = icm_detect_addr(cfg->bus.bus, &addr);
	if (ret) {
		LOG_ERR("ICM45686 not responding after reset");
		return ret;
	}
	if (addr != data->bus.addr) {
		LOG_INF("address flipped after reset: 0x%02X -> 0x%02X",
			data->bus.addr, addr);
		data->bus.addr = addr;
	}
#endif

	/* Internal pull-ups + little-endian config */
	ret  = icm_write_ireg(&data->bus, ICM45686_IPREG_BAR, ICM45686_IPREG_BAR_REG_58, 0xD9);
	ret |= icm_write_ireg(&data->bus, ICM45686_IPREG_BAR, ICM45686_IPREG_BAR_REG_59, 0xBA);
	ret |= icm_write_ireg(&data->bus, ICM45686_IPREG_TOP1, ICM45686_SREG_CTRL, 0x00);
	if (ret) {
		LOG_ERR("IREG config failed: %d", ret);
		return ret;
	}

	/* Sensor ODR + FS (100 Hz, 32G, 4000 dps) */
	ret  = icm_write(&data->bus, ICM45686_ACCEL_CONFIG0,
			 (ACCEL_UI_FS_SEL_32G << 4) | ACCEL_ODR_100HZ);
	ret |= icm_write(&data->bus, ICM45686_GYRO_CONFIG0,
			 (GYRO_UI_FS_SEL_4000DPS << 4) | GYRO_ODR_100HZ);
	ret |= icm_write(&data->bus, ICM45686_PWR_MGMT0,
			 (GYRO_MODE_LN << 2) | ACCEL_MODE_LN);
	k_usleep(250);
	if (ret) {
		LOG_ERR("Sensor config failed: %d", ret);
		return ret;
	}

	/* Digital filter chain (AAF only; UI LPF @ ODR/4) */
	{
		uint8_t reg;
		ret  = icm_read_ireg(&data->bus, ICM45686_IPREG_SYS1,
				     ICM45686_IPREG_SYS1_REG_166, &reg, 1);
		reg = (reg & ~(0x03 << 5)) | (SRC_CTRL_AAF_ONLY << 5);
		ret |= icm_write_ireg(&data->bus, ICM45686_IPREG_SYS1,
				      ICM45686_IPREG_SYS1_REG_166, reg);

		ret |= icm_read_ireg(&data->bus, ICM45686_IPREG_SYS2,
				     ICM45686_IPREG_SYS2_REG_123, &reg, 1);
		reg = (reg & ~0x03) | SRC_CTRL_AAF_ONLY;
		ret |= icm_write_ireg(&data->bus, ICM45686_IPREG_SYS2,
				      ICM45686_IPREG_SYS2_REG_123, reg);

		ret |= icm_read_ireg(&data->bus, ICM45686_IPREG_SYS1,
				     ICM45686_IPREG_SYS1_REG_172, &reg, 1);
		reg = (reg & ~0x07) | UI_LPFBW_ODR_DIV4;
		ret |= icm_write_ireg(&data->bus, ICM45686_IPREG_SYS1,
				      ICM45686_IPREG_SYS1_REG_172, reg);

		ret |= icm_read_ireg(&data->bus, ICM45686_IPREG_SYS2,
				     ICM45686_IPREG_SYS2_REG_131, &reg, 1);
		reg = (reg & ~0x07) | UI_LPFBW_ODR_DIV4;
		ret |= icm_write_ireg(&data->bus, ICM45686_IPREG_SYS2,
				      ICM45686_IPREG_SYS2_REG_131, reg);
	}
	if (ret) {
		LOG_ERR("Digital filter config failed: %d", ret);
		return ret;
	}

	/* FIFO configuration */
	{
		uint8_t fifo_cfg[6];
		uint8_t smc_ctrl;

		ret  = icm_burst_read(&data->bus, ICM45686_FIFO_CONFIG0, fifo_cfg, 6);
		ret |= icm_read_ireg(&data->bus, ICM45686_IPREG_TOP1,
				     ICM45686_SMC_CONTROL_0, &smc_ctrl, 1);
		if (ret) { return ret; }

		/* Stop FIFO → bypass mode */
		fifo_cfg[4] &= 0xF0;
		ret |= icm_write(&data->bus, ICM45686_FIFO_CONFIG3, fifo_cfg[4]);
		fifo_cfg[0] &= 0x3F;
		ret |= icm_write(&data->bus, ICM45686_FIFO_CONFIG0, fifo_cfg[0]);

		smc_ctrl |= BIT(0);
		ret |= icm_write_ireg(&data->bus, ICM45686_IPREG_TOP1,
				      ICM45686_SMC_CONTROL_0, smc_ctrl);
		fifo_cfg[5] |= BIT(1);

		/* 2 KB depth */
		fifo_cfg[0] = (fifo_cfg[0] & 0xC0) | 0x07;

		/* watermark 64 bytes */
		fifo_cfg[1] = 0x40;
		fifo_cfg[2] = 0x00;

		/* hires + accel + gyro */
		fifo_cfg[4] |= 0x0E;

		/* Write back, switch to stream mode */
		ret |= icm_burst_write(&data->bus, ICM45686_FIFO_CONFIG0, fifo_cfg, 6);
		fifo_cfg[0] |= 0x40;
		ret |= icm_write(&data->bus, ICM45686_FIFO_CONFIG0, fifo_cfg[0]);
		fifo_cfg[4] |= BIT(0);
		ret |= icm_write(&data->bus, ICM45686_FIFO_CONFIG3, fifo_cfg[4]);
	}
	if (ret) {
		LOG_ERR("FIFO config failed: %d", ret);
		return ret;
	}

	/* Wait for FIFO to fill */
	{
		uint16_t check = 0;
		k_sleep(K_MSEC(15));
		read_fifo_count(&data->bus, &check);
		if (check == 0) {
			LOG_WRN("FIFO empty after startup — check I2C wiring / INT pin");
		} else {
			LOG_INF("FIFO running (%u bytes)", check);
		}
	}

	/* VQF init (100 Hz, no mag) */
	init_params(&data->vqf_params);
	initVqf(&data->vqf_params, &data->vqf_state, &data->vqf_coeffs,
		0.01f, 0.01f, -1.0f);
	data->vqf_ready = true;

	/* start 100Hz auto-feed timer */
	data->dev = dev;
	k_mutex_init(&data->lock);
	k_work_init_delayable(&data->feed_work, icm_feed_handler);
	k_work_schedule(&data->feed_work, K_MSEC(10));

	LOG_INF("ICM45686 + VQF ready (100 Hz, internal RC osc)");
	return 0;
}

/* ------------------------------------------------------------------ */
/*  Driver instantiation macro                                         */
/* ------------------------------------------------------------------ */

/** @brief 设备注册宏 */
#define ICM45686_INIT(n)                                                       \
	static struct icm_data icm_data_##n = {                                \
		.vqf_ready = false,                                            \
	};                                                                     \
	static const struct icm_config icm_config_##n = {                      \
		.bus = I2C_DT_SPEC_INST_GET(n),                                \
	};                                                                     \
	SENSOR_DEVICE_DT_INST_DEFINE(n, icm_init, NULL,                        \
				     &icm_data_##n, &icm_config_##n,            \
				     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,  \
				     &icm_api);

DT_INST_FOREACH_STATUS_OKAY(ICM45686_INIT)
