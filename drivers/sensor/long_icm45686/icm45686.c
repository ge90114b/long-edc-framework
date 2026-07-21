/* SPDX-License-Identifier: MIT */
/*
 * ICM45686 6-axis IMU driver with integrated VQF orientation filter.
 *
 * Uses the sensor's internal 32 kHz RC oscillator — no external crystal
 * required.  Runs at 100 Hz ODR with all-float VQF sensor fusion, suitable
 * for Cortex-M0+/M4 MCUs without FPU.
 *
 * Outputs (via Zephyr sensor API):
 *   SENSOR_CHAN_ROTATION_X/Y/Z  — roll / pitch / yaw  (degrees)
 *   SENSOR_CHAN_GYRO_X/Y/Z      — debiased angular velocity (rad/s)
 *
 * DTS bindings: dts/bindings/sensor/icm45686.yaml
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
/*  Per-instance config & data                                        */
/* ------------------------------------------------------------------ */

struct icm_config {
	struct i2c_dt_spec bus;
};

struct icm_data {
	/* VQF sensor fusion state */
	vqf_params_t  vqf_params;
	vqf_state_t   vqf_state;
	vqf_coeffs_t  vqf_coeffs;
	bool          vqf_ready;

	/* Cached sensor outputs (updated by sample_fetch, read by channel_get) */
	float euler[3];        /* roll, pitch, yaw  (degrees) */
	float gyro_debiased[3]; /* debiased angular velocity (rad/s) */

	/* FIFO read buffer — enough for ~12 packets at 100 Hz */
	uint8_t fifo_buf[256] __aligned(4);
};

/* ------------------------------------------------------------------ */
/*  I²C helpers                                                        */
/* ------------------------------------------------------------------ */

static inline int icm_write(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t val)
{
	uint8_t buf[2] = { reg, val };
	return i2c_write_dt(bus, buf, sizeof(buf));
}

static inline int icm_read(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t *val)
{
	return i2c_reg_read_byte_dt(bus, reg, val);
}

static int icm_burst_read(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t *data, size_t len)
{
	return i2c_burst_read_dt(bus, reg, data, len);
}

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

static int icm_write_ireg(const struct i2c_dt_spec *bus,
			  uint8_t bank, uint8_t addr, uint8_t data)
{
	uint8_t buf[3] = { bank, addr, data };
	return icm_burst_write(bus, ICM45686_IREG_ADDR_15_8, buf, 3);
}

static int icm_read_ireg(const struct i2c_dt_spec *bus,
			 uint8_t bank, uint8_t addr, uint8_t *data, size_t len)
{
	int ret;
	uint8_t addr_buf[2] = { bank, addr };

	ret = icm_burst_write(bus, ICM45686_IREG_ADDR_15_8, addr_buf, 2);
	if (ret) { return ret; }

	for (size_t i = 0; i < len; i++) {
		k_usleep(4);  /* min gap between IREG reads (datasheet §5.2) */
		ret = icm_read(bus, ICM45686_IREG_DATA, &data[i]);
		if (ret) { return ret; }
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/*  FIFO helpers                                                       */
/* ------------------------------------------------------------------ */

/** Read FIFO byte count.  Per reference driver, read twice and use
 *  the second result for reliability.  In stream mode the count is
 *  one behind the written pointer — subtract 1 to avoid hanging byte.
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

/** Flush FIFO by toggling the flush bit in FIFO_CONFIG2. */
static int flush_fifo(const struct i2c_dt_spec *bus)
{
	uint8_t val;
	int ret = icm_read(bus, ICM45686_FIFO_CONFIG2, &val);
	if (ret) { return ret; }
	val |= BIT(7);
	return icm_write(bus, ICM45686_FIFO_CONFIG2, val);
}

/**
 * Parse one 20-byte FIFO packet into accel (m/s²) and gyro (rad/s).
 *
 * Packet layout (little-endian, 20-bit hires mode):
 *   [0]       : Header (expected: 0x78)
 *   [1 — 6]   : Accel  X / Y / Z  (16-bit MSB, LE)
 *   [7 — 12]  : Gyro   X / Y / Z  (16-bit MSB, LE)
 *   [13 — 14] : Temperature       (16-bit, LE)
 *   [15 — 16] : Timestamp         (16-bit, LE, 1 µs)
 *   [17 — 19] : Extension         (upper nibble = accel LSB per axis,
 *                                   lower nibble = gyro  LSB per axis)
 *
 * Returns 0 on success, -1 if the header is invalid.
 */
static int parse_fifo_packet(const uint8_t *data, float accel[3], float gyro[3])
{
	if (data[0] != ICM45686_FIFO_HEADER_20BIT) {
		return -1;
	}

	for (int i = 0; i < 3; i++) {
		/* 16-bit MSBs */
		int16_t a_hi = sys_get_le16(data + 1 + i * 2);
		int16_t g_hi = sys_get_le16(data + 7 + i * 2);

		/* Extension byte: upper nibble = accel LSB, lower = gyro LSB */
		uint8_t ext = data[17 + i];

		/* Assemble 20-bit signed value */
		int32_t a_20 = ((int32_t)a_hi << 4) | ((ext >> 4) & 0x0F);
		int32_t g_20 = ((int32_t)g_hi << 4) | (ext & 0x0F);

		/* Sign-extend 20-bit → 32-bit */
		if (a_20 & 0x80000) { a_20 |= 0xFFF00000UL; }
		if (g_20 & 0x80000) { g_20 |= 0xFFF00000UL; }

		/* Convert to physical units:
		 *   accel: raw × (32G / 2^19) × 9.80665  →  m/s²
		 *   gyro:  raw × (4000dps / 2^19) × π/180 →  rad/s
		 */
		accel[i] = (float)a_20 * ICM45686_ACCEL_SENSITIVITY_20BIT * ICM45686_EARTH_GRAVITY;
		gyro[i]  = (float)g_20 * ICM45686_GYRO_SENSITIVITY_20BIT  * ICM45686_DEG_TO_RAD;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/*  Quaternion → Euler angles (roll / pitch / yaw in degrees)          */
/* ------------------------------------------------------------------ */

static void quat_to_euler(const float q[4], float euler[3])
{
	float w = q[0], x = q[1], y = q[2], z = q[3];

	/* roll  = atan2(2(wx + yz), 1 - 2(x² + y²)) */
	euler[0] = atan2f(2.0f * (w * x + y * z),
			  1.0f - 2.0f * (x * x + y * y));

	/* pitch = asin(2(wy - zx)) */
	float sin_pitch = 2.0f * (w * y - z * x);
	if (sin_pitch > 1.0f)  { sin_pitch = 1.0f; }
	if (sin_pitch < -1.0f) { sin_pitch = -1.0f; }
	euler[1] = asinf(sin_pitch);

	/* yaw   = atan2(2(wz + xy), 1 - 2(y² + z²)) */
	euler[2] = atan2f(2.0f * (w * z + x * y),
			  1.0f - 2.0f * (y * y + z * z));

	/* Convert to degrees */
	euler[0] *= ICM45686_RAD_TO_DEG;
	euler[1] *= ICM45686_RAD_TO_DEG;
	euler[2] *= ICM45686_RAD_TO_DEG;
}

/* ------------------------------------------------------------------ */
/*  Sensor driver API                                                  */
/* ------------------------------------------------------------------ */

/**
 * @brief Read FIFO, run VQF updates, cache results.
 *
 * Called by the application at ≥ 100 Hz.  Processes all available FIFO
 * packets and updates the VQF filter once per packet.  If no data is
 * available the cached values remain valid.
 */
static int icm_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct icm_data *data = dev->data;
	const struct icm_config *cfg = dev->config;
	uint16_t fifo_count;
	int ret;

	ARG_UNUSED(chan);

	ret = read_fifo_count(&cfg->bus, &fifo_count);
	if (ret || fifo_count == 0) {
		return 0;   /* no new data — cached values still good */
	}

	size_t bytes = fifo_count * ICM45686_FIFO_PACKET_SIZE;
	if (bytes > sizeof(data->fifo_buf)) {
		bytes = sizeof(data->fifo_buf);
		fifo_count = (uint16_t)(bytes / ICM45686_FIFO_PACKET_SIZE);
	}

	ret = icm_burst_read(&cfg->bus, ICM45686_FIFO_DATA, data->fifo_buf, bytes);
	if (ret) {
		LOG_WRN("FIFO read failed: %d", ret);
		return ret;
	}

	/* Process each packet: parse → VQF gyro update → VQF accel update.
	 * Cache the last valid raw gyro for debiased output.
	 */
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
		return 0;
	}

	/* Cache latest orientation + debiased gyroscope */
	float quat[4];
	getQuat6D(&data->vqf_state, quat);
	quat_to_euler(quat, data->euler);

	float bias[3];
	getBiasEstimate(&data->vqf_state, &data->vqf_coeffs, bias);

	data->gyro_debiased[0] = last_gyro[0] - bias[0];
	data->gyro_debiased[1] = last_gyro[1] - bias[1];
	data->gyro_debiased[2] = last_gyro[2] - bias[2];

	return 0;
}

static int icm_channel_get(const struct device *dev,
			   enum sensor_channel chan,
			   struct sensor_value *val)
{
	struct icm_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ROTATION:     /* yaw (degrees) — most useful for car */
		sensor_value_from_double(val, (double)data->euler[2]);
		break;
	case SENSOR_CHAN_GYRO_X:       /* debiased gyro (rad/s) */
		sensor_value_from_double(val, (double)data->gyro_debiased[0]);
		break;
	case SENSOR_CHAN_GYRO_Y:
		sensor_value_from_double(val, (double)data->gyro_debiased[1]);
		break;
	case SENSOR_CHAN_GYRO_Z:
		sensor_value_from_double(val, (double)data->gyro_debiased[2]);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api icm_api = {
	.sample_fetch = icm_sample_fetch,
	.channel_get  = icm_channel_get,
};

/* ------------------------------------------------------------------ */
/*  Initialisation                                                     */
/* ------------------------------------------------------------------ */

static int icm_init(const struct device *dev)
{
	const struct icm_config *cfg = dev->config;
	struct icm_data *data = dev->data;
	uint8_t whoami;
	int ret;

	/* --- I²C bus check --- */
	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* --- WHO_AM_I --- */
	ret = icm_read(&cfg->bus, ICM45686_WHO_AM_I, &whoami);
	if (ret) {
		LOG_ERR("WHO_AM_I read failed: %d", ret);
		return -EIO;
	}
	if (whoami != 0xE9) {
		LOG_ERR("Unexpected WHO_AM_I: 0x%02X (expected 0xE9)", whoami);
		return -ENODEV;
	}
	LOG_INF("ICM45686 found at 0x%02X", cfg->bus.addr);

	/* --- Reset --- */
	ret = icm_write(&cfg->bus, ICM45686_REG_MISC2, 0x02);
	if (ret) { return ret; }
	k_sleep(K_MSEC(10));

	/* --- Internal pull-ups (stability) --- */
	ret  = icm_write_ireg(&cfg->bus, ICM45686_IPREG_BAR, ICM45686_IPREG_BAR_REG_58, 0xD9);
	ret |= icm_write_ireg(&cfg->bus, ICM45686_IPREG_BAR, ICM45686_IPREG_BAR_REG_59, 0xBA);
	/* FIFO little-endian */
	ret |= icm_write_ireg(&cfg->bus, ICM45686_IPREG_TOP1, ICM45686_SREG_CTRL, 0x00);
	if (ret) {
		LOG_ERR("IREG config failed: %d", ret);
		return ret;
	}

	/* --- Sensor ODR + full-scale (100 Hz, 32 G, 4000 dps) --- */
	ret  = icm_write(&cfg->bus, ICM45686_ACCEL_CONFIG0,
			 (ACCEL_UI_FS_SEL_32G << 4) | ACCEL_ODR_100HZ);
	ret |= icm_write(&cfg->bus, ICM45686_GYRO_CONFIG0,
			 (GYRO_UI_FS_SEL_4000DPS << 4) | GYRO_ODR_100HZ);
	/* Power on (low-noise) */
	ret |= icm_write(&cfg->bus, ICM45686_PWR_MGMT0,
			 (GYRO_MODE_LN << 2) | ACCEL_MODE_LN);
	k_usleep(250);
	if (ret) {
		LOG_ERR("Sensor config failed: %d", ret);
		return ret;
	}

	/* --- Digital filter chain (AAF only; UI LPF @ ODR/4) --- */
	{
		uint8_t reg;
		ret  = icm_read_ireg(&cfg->bus, ICM45686_IPREG_SYS1,
				     ICM45686_IPREG_SYS1_REG_166, &reg, 1);
		reg = (reg & ~(0x03 << 5)) | (SRC_CTRL_AAF_ONLY << 5);
		ret |= icm_write_ireg(&cfg->bus, ICM45686_IPREG_SYS1,
				      ICM45686_IPREG_SYS1_REG_166, reg);

		ret |= icm_read_ireg(&cfg->bus, ICM45686_IPREG_SYS2,
				     ICM45686_IPREG_SYS2_REG_123, &reg, 1);
		reg = (reg & ~0x03) | SRC_CTRL_AAF_ONLY;
		ret |= icm_write_ireg(&cfg->bus, ICM45686_IPREG_SYS2,
				      ICM45686_IPREG_SYS2_REG_123, reg);

		/* UI LPF */
		ret |= icm_read_ireg(&cfg->bus, ICM45686_IPREG_SYS1,
				     ICM45686_IPREG_SYS1_REG_172, &reg, 1);
		reg = (reg & ~0x07) | UI_LPFBW_ODR_DIV4;
		ret |= icm_write_ireg(&cfg->bus, ICM45686_IPREG_SYS1,
				      ICM45686_IPREG_SYS1_REG_172, reg);

		ret |= icm_read_ireg(&cfg->bus, ICM45686_IPREG_SYS2,
				     ICM45686_IPREG_SYS2_REG_131, &reg, 1);
		reg = (reg & ~0x07) | UI_LPFBW_ODR_DIV4;
		ret |= icm_write_ireg(&cfg->bus, ICM45686_IPREG_SYS2,
				      ICM45686_IPREG_SYS2_REG_131, reg);
	}
	if (ret) {
		LOG_ERR("Digital filter config failed: %d", ret);
		return ret;
	}

	/* --- FIFO configuration --- */
	{
		uint8_t fifo_cfg[6];
		uint8_t smc_ctrl;

		ret  = icm_burst_read(&cfg->bus, ICM45686_FIFO_CONFIG0, fifo_cfg, 6);
		ret |= icm_read_ireg(&cfg->bus, ICM45686_IPREG_TOP1,
				     ICM45686_SMC_CONTROL_0, &smc_ctrl, 1);
		if (ret) { return ret; }

		/* Stop FIFO, set bypass mode */
		fifo_cfg[4] &= 0xF0;                    /* FIFO_CONFIG3: disable */
		ret |= icm_write(&cfg->bus, ICM45686_FIFO_CONFIG3, fifo_cfg[4]);
		fifo_cfg[0] &= 0x3F;                    /* FIFO_CONFIG0: bypass */
		ret |= icm_write(&cfg->bus, ICM45686_FIFO_CONFIG0, fifo_cfg[0]);

		/* Timestamp enable */
		smc_ctrl |= BIT(0);
		ret |= icm_write_ireg(&cfg->bus, ICM45686_IPREG_TOP1,
				      ICM45686_SMC_CONTROL_0, smc_ctrl);
		fifo_cfg[5] |= BIT(1);                   /* FIFO_TMST_FSYNC_EN */

		/* FIFO depth: 2 KB */
		fifo_cfg[0] = (fifo_cfg[0] & 0xC0) | 0x07;

		/* Watermark (not used for polling, but set for safety) */
		fifo_cfg[1] = 0x40;                      /* 64 bytes (LSB) */
		fifo_cfg[2] = 0x00;                      /* (MSB) */

		/* HIRES + accel + gyro enabled */
		fifo_cfg[4] |= 0x0E;

		/* Write back, then switch to stream mode */
		ret |= icm_burst_write(&cfg->bus, ICM45686_FIFO_CONFIG0, fifo_cfg, 6);
		fifo_cfg[0] |= 0x40;                     /* stream-to-FIFO */
		ret |= icm_write(&cfg->bus, ICM45686_FIFO_CONFIG0, fifo_cfg[0]);
		fifo_cfg[4] |= BIT(0);                   /* FIFO_CONFIG3: enable */
		ret |= icm_write(&cfg->bus, ICM45686_FIFO_CONFIG3, fifo_cfg[4]);
	}
	if (ret) {
		LOG_ERR("FIFO config failed: %d", ret);
		return ret;
	}

	/* --- Wait for FIFO to fill --- */
	{
		uint16_t check = 0;
		k_sleep(K_MSEC(15));
		read_fifo_count(&cfg->bus, &check);
		if (check == 0) {
			LOG_WRN("FIFO empty after startup — check I2C wiring / INT pin");
		} else {
			LOG_INF("FIFO running (%u bytes)", check);
		}
	}

	/* --- VQF initialisation (100 Hz: gyrTs = accTs = 0.01, no mag) --- */
	init_params(&data->vqf_params);
	initVqf(&data->vqf_params, &data->vqf_state, &data->vqf_coeffs,
		0.01f, 0.01f, -1.0f);
	data->vqf_ready = true;

	LOG_INF("ICM45686 + VQF ready (100 Hz, internal RC osc)");
	return 0;
}

/* ------------------------------------------------------------------ */
/*  Driver instantiation macro                                         */
/* ------------------------------------------------------------------ */

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
