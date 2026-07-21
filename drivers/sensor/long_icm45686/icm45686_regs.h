/* SPDX-License-Identifier: MIT */
/*
 * ICM45686 register map — register addresses, field values, and configuration
 * constants.  Based on the TDK ICM-45686 datasheet (DS-000577) and user guide
 * (AN-000478).
 *
 * Only the registers used by the long-edc-framework driver are listed here.
 */

#ifndef ICM45686_REGS_H__
#define ICM45686_REGS_H__

/* ---- User Bank 0 (direct register access) ---- */

#define ICM45686_WHO_AM_I                 0x72
#define ICM45686_ACCEL_DATA_X1_UI         0x00
#define ICM45686_GYRO_DATA_X1_UI          0x06
#define ICM45686_TEMP_DATA1_UI            0x0C
#define ICM45686_TEMP_DATA0_UI            0x0D
#define ICM45686_PWR_MGMT0                0x10
#define ICM45686_FIFO_COUNT_0             0x12
#define ICM45686_FIFO_COUNT_1             0x13
#define ICM45686_FIFO_DATA                0x14
#define ICM45686_INT1_CONFIG0             0x16
#define ICM45686_INT1_CONFIG1             0x17
#define ICM45686_INT1_CONFIG2             0x18
#define ICM45686_INT1_STATUS0             0x19
#define ICM45686_ACCEL_CONFIG0            0x1B
#define ICM45686_GYRO_CONFIG0             0x1C
#define ICM45686_FIFO_CONFIG0             0x1D
#define ICM45686_FIFO_CONFIG1_0           0x1E
#define ICM45686_FIFO_CONFIG1_1           0x1F
#define ICM45686_FIFO_CONFIG2             0x20
#define ICM45686_FIFO_CONFIG3             0x21
#define ICM45686_FIFO_CONFIG4             0x22
#define ICM45686_TMST_WOM_CONFIG          0x23
#define ICM45686_RTC_CONFIG               0x26
#define ICM45686_IOC_PAD_SCENARIO_AUX_OVRD 0x30
#define ICM45686_IOC_PAD_SCENARIO_OVRD    0x31
#define ICM45686_REG_MISC1                0x35
#define ICM45686_IREG_ADDR_15_8           0x7C
#define ICM45686_IREG_DATA                0x7E
#define ICM45686_REG_MISC2                0x7F

/* ---- IREG banks (indirect register access) ---- */

#define ICM45686_IPREG_BAR                0xA0
#define ICM45686_IPREG_BAR_REG_58         0x3A
#define ICM45686_IPREG_BAR_REG_59         0x3B

#define ICM45686_IPREG_TOP1               0xA2
#define ICM45686_SMC_CONTROL_0            0x58
#define ICM45686_SREG_CTRL                0x67

#define ICM45686_IPREG_SYS1               0xA4
#define ICM45686_IPREG_SYS1_REG_166       0xA6
#define ICM45686_IPREG_SYS1_REG_172       0xAC

#define ICM45686_IPREG_SYS2               0xA5
#define ICM45686_IPREG_SYS2_REG_123       0x7B
#define ICM45686_IPREG_SYS2_REG_131       0x83

/* ---- Power modes ---- */

#define GYRO_MODE_OFF                     0x00
#define GYRO_MODE_STANDBY                 0x01
#define GYRO_MODE_LP                      0x02
#define GYRO_MODE_LN                      0x03

#define ACCEL_MODE_OFF                    0x01
#define ACCEL_MODE_LP                     0x02
#define ACCEL_MODE_LN                     0x03

/* ---- Accelerometer full-scale ---- */

#define ACCEL_UI_FS_SEL_32G               0x00
#define ACCEL_UI_FS_SEL_16G               0x01
#define ACCEL_UI_FS_SEL_8G                0x02
#define ACCEL_UI_FS_SEL_4G                0x03
#define ACCEL_UI_FS_SEL_2G                0x04

/* ---- Accelerometer ODR (Low Noise) ---- */

#define ACCEL_ODR_6_4KHZ                  0x03
#define ACCEL_ODR_3_2KHZ                  0x04
#define ACCEL_ODR_1_6KHZ                  0x05
#define ACCEL_ODR_800HZ                   0x06
#define ACCEL_ODR_400HZ                   0x07
#define ACCEL_ODR_200HZ                   0x08
#define ACCEL_ODR_100HZ                   0x09
#define ACCEL_ODR_50HZ                    0x0A
#define ACCEL_ODR_25HZ                    0x0B
#define ACCEL_ODR_12_5HZ                  0x0C

/* ---- Gyroscope full-scale ---- */

#define GYRO_UI_FS_SEL_4000DPS            0x00
#define GYRO_UI_FS_SEL_2000DPS            0x01
#define GYRO_UI_FS_SEL_1000DPS            0x02
#define GYRO_UI_FS_SEL_500DPS             0x03
#define GYRO_UI_FS_SEL_250DPS             0x04
#define GYRO_UI_FS_SEL_125DPS             0x05
#define GYRO_UI_FS_SEL_62_5DPS            0x06
#define GYRO_UI_FS_SEL_31_25DPS           0x07
#define GYRO_UI_FS_SEL_15_625DPS          0x08

/* ---- Gyroscope ODR (Low Noise) ---- */

#define GYRO_ODR_6_4KHZ                   0x03
#define GYRO_ODR_3_2KHZ                   0x04
#define GYRO_ODR_1_6KHZ                   0x05
#define GYRO_ODR_800HZ                    0x06
#define GYRO_ODR_400HZ                    0x07
#define GYRO_ODR_200HZ                    0x08
#define GYRO_ODR_100HZ                    0x09
#define GYRO_ODR_50HZ                     0x0A
#define GYRO_ODR_25HZ                     0x0B
#define GYRO_ODR_12_5HZ                   0x0C

/* ---- Signal source control ---- */

#define SRC_CTRL_OFF                      0
#define SRC_CTRL_AAF_ONLY                 1
#define SRC_CTRL_AAF_INTERP               2

/* ---- UI low-pass filter bandwidth ---- */

#define UI_LPFBW_BYPASS                   0
#define UI_LPFBW_ODR_DIV4                 1
#define UI_LPFBW_ODR_DIV8                 2
#define UI_LPFBW_ODR_DIV16                3
#define UI_LPFBW_ODR_DIV32                4
#define UI_LPFBW_ODR_DIV64                5
#define UI_LPFBW_ODR_DIV128               6

/* ---- Conversion constants ---- */

/* 20-bit sensitivities: raw × sensitivity = physical value */
#define ICM45686_ACCEL_SENSITIVITY_20BIT  (32.0f / 524288.0f)  /* G / LSB  */
#define ICM45686_GYRO_SENSITIVITY_20BIT   (4000.0f / 524288.0f) /* dps / LSB */

#define ICM45686_EARTH_GRAVITY            9.80665f
#define ICM45686_DEG_TO_RAD               (3.14159265358979323846f / 180.0f)
#define ICM45686_RAD_TO_DEG               (180.0f / 3.14159265358979323846f)

/* ---- FIFO ---- */

#define ICM45686_FIFO_PACKET_SIZE         20   /* 20-bit hires + accel + gyro + timestamp */
#define ICM45686_FIFO_HEADER_20BIT        0x78 /* expected header for 20-bit packets */
#define ICM45686_FIFO_SIZE                2048 /* hardware FIFO depth in bytes */

#endif /* ICM45686_REGS_H__ */
