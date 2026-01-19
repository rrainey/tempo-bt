/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 *
 * MEMSIC MMC5983MA 3-axis magnetometer driver - internal header
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MMC5983MA_MMC5983MA_H_
#define ZEPHYR_DRIVERS_SENSOR_MMC5983MA_MMC5983MA_H_

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

/*
 * Register addresses
 */
#define MMC5983MA_REG_XOUT0          0x00  /* X output [17:10] */
#define MMC5983MA_REG_XOUT1          0x01  /* X output [9:2] */
#define MMC5983MA_REG_YOUT0          0x02  /* Y output [17:10] */
#define MMC5983MA_REG_YOUT1          0x03  /* Y output [9:2] */
#define MMC5983MA_REG_ZOUT0          0x04  /* Z output [17:10] */
#define MMC5983MA_REG_ZOUT1          0x05  /* Z output [9:2] */
#define MMC5983MA_REG_XYZOUT2        0x06  /* XYZ output [1:0] bits */
#define MMC5983MA_REG_TOUT           0x07  /* Temperature output */
#define MMC5983MA_REG_STATUS         0x08  /* Device status */
#define MMC5983MA_REG_CTRL0          0x09  /* Internal control 0 */
#define MMC5983MA_REG_CTRL1          0x0A  /* Internal control 1 */
#define MMC5983MA_REG_CTRL2          0x0B  /* Internal control 2 */
#define MMC5983MA_REG_CTRL3          0x0C  /* Internal control 3 */
#define MMC5983MA_REG_PRODUCT_ID     0x2F  /* Product ID */

/*
 * Status register bits (0x08)
 */
#define MMC5983MA_STATUS_MEAS_M_DONE BIT(0)  /* Magnetic measurement done */
#define MMC5983MA_STATUS_MEAS_T_DONE BIT(1)  /* Temperature measurement done */
#define MMC5983MA_STATUS_OTP_RD_DONE BIT(4)  /* OTP read done */

/*
 * Control register 0 bits (0x09)
 */
#define MMC5983MA_CTRL0_TM_M         BIT(0)  /* Take magnetic measurement */
#define MMC5983MA_CTRL0_TM_T         BIT(1)  /* Take temperature measurement */
#define MMC5983MA_CTRL0_INT_DONE_EN  BIT(2)  /* Interrupt on measurement done */
#define MMC5983MA_CTRL0_SET          BIT(3)  /* Perform SET operation */
#define MMC5983MA_CTRL0_RESET        BIT(4)  /* Perform RESET operation */
#define MMC5983MA_CTRL0_AUTO_SR_EN   BIT(5)  /* Auto SET/RESET enable */
#define MMC5983MA_CTRL0_OTP_READ     BIT(6)  /* Re-read OTP */

/*
 * Control register 1 bits (0x0A)
 */
#define MMC5983MA_CTRL1_BW0          BIT(0)  /* Bandwidth bit 0 */
#define MMC5983MA_CTRL1_BW1          BIT(1)  /* Bandwidth bit 1 */
#define MMC5983MA_CTRL1_X_INHIBIT    BIT(2)  /* Disable X channel */
#define MMC5983MA_CTRL1_YZ_INHIBIT   GENMASK(4, 3)  /* Disable Y/Z channels */
#define MMC5983MA_CTRL1_SW_RST       BIT(7)  /* Software reset */

/*
 * Control register 2 bits (0x0B)
 */
#define MMC5983MA_CTRL2_CM_FREQ_MASK GENMASK(2, 0)  /* Continuous mode frequency */
#define MMC5983MA_CTRL2_CMM_EN       BIT(3)  /* Continuous mode enable */
#define MMC5983MA_CTRL2_PRD_SET_MASK GENMASK(6, 4)  /* Periodic SET frequency */
#define MMC5983MA_CTRL2_EN_PRD_SET   BIT(7)  /* Enable periodic SET */

/*
 * Control register 3 bits (0x0C)
 */
#define MMC5983MA_CTRL3_ST_ENP       BIT(1)  /* Self-test positive */
#define MMC5983MA_CTRL3_ST_ENM       BIT(2)  /* Self-test negative */
#define MMC5983MA_CTRL3_SPI_3W       BIT(6)  /* 3-wire SPI mode */

/*
 * Continuous mode frequency values (CM_FREQ)
 */
#define MMC5983MA_CM_FREQ_OFF        0  /* Continuous mode off */
#define MMC5983MA_CM_FREQ_1HZ        1
#define MMC5983MA_CM_FREQ_10HZ       2
#define MMC5983MA_CM_FREQ_20HZ       3
#define MMC5983MA_CM_FREQ_50HZ       4
#define MMC5983MA_CM_FREQ_100HZ      5
#define MMC5983MA_CM_FREQ_200HZ      6  /* Requires BW=01 */
#define MMC5983MA_CM_FREQ_1000HZ     7  /* Requires BW=11 */

/*
 * Device constants
 */
#define MMC5983MA_PRODUCT_ID         0x30  /* Expected product ID */
#define MMC5983MA_I2C_ADDR           0x30  /* 7-bit I2C address */

/*
 * Measurement constants
 */
#define MMC5983MA_18BIT_MAX          262143  /* 2^18 - 1 */
#define MMC5983MA_18BIT_OFFSET       131072  /* 2^17 (null field output) */
#define MMC5983MA_SENSITIVITY_18BIT  16384   /* Counts per Gauss */
#define MMC5983MA_GAUSS_PER_COUNT    0.00006103515625f  /* 1/16384 */

/* Temperature: -75°C to +125°C, 0.8°C per count, 0 = -75°C */
#define MMC5983MA_TEMP_OFFSET        (-75.0f)
#define MMC5983MA_TEMP_SCALE         0.8f

/*
 * Timing constants (milliseconds)
 */
#define MMC5983MA_STARTUP_TIME_MS    10  /* Power-up time */
#define MMC5983MA_SET_RESET_TIME_US  500 /* SET/RESET pulse duration */
#define MMC5983MA_MEAS_TIME_BW00_MS  8   /* Measurement time BW=00 */
#define MMC5983MA_MEAS_TIME_BW01_MS  4   /* Measurement time BW=01 */
#define MMC5983MA_MEAS_TIME_BW10_MS  2   /* Measurement time BW=10 */
#define MMC5983MA_MEAS_TIME_BW11_MS  1   /* Measurement time BW=11 */

/*
 * Calibration data structure
 */
struct mmc5983ma_calibration {
	/* Hard-iron offsets in raw 18-bit counts */
	int32_t offset_x;
	int32_t offset_y;
	int32_t offset_z;

	/* Scale factors (Q1.15 fixed-point: 32768 = 1.0) */
	uint16_t scale_x;
	uint16_t scale_y;
	uint16_t scale_z;

	/* Soft-iron correction matrix (Q1.15), row-major */
	/* For diagonal-only: [0]=xx, [4]=yy, [8]=zz, others=0 */
	int16_t soft_iron[9];

	/* Validity and metadata */
	uint8_t valid;
	uint8_t method;
};

/* Calibration methods */
#define MMC5983MA_CAL_NONE           0x00
#define MMC5983MA_CAL_FACTORY        0x01  /* From devicetree */
#define MMC5983MA_CAL_MINMAX         0x02  /* Simple min/max */
#define MMC5983MA_CAL_ELLIPSOID      0x03  /* Ellipsoid fit */

/*
 * Driver configuration (from devicetree)
 */
struct mmc5983ma_config {
	struct i2c_dt_spec i2c;
#ifdef CONFIG_MMC5983MA_TRIGGER
	struct gpio_dt_spec int_gpio;
#endif
	uint8_t bandwidth;           /* BW[1:0] setting */
	uint8_t continuous_rate;     /* CM_FREQ setting */
	bool auto_sr;                /* Auto SET/RESET enable */

	/* Factory calibration defaults */
	int32_t cal_offset_x;
	int32_t cal_offset_y;
	int32_t cal_offset_z;
	uint16_t cal_scale_x;
	uint16_t cal_scale_y;
	uint16_t cal_scale_z;
};

/*
 * Driver runtime data
 */
struct mmc5983ma_data {
	/* Raw readings (18-bit, offset-adjusted to signed) */
	int32_t raw_x;
	int32_t raw_y;
	int32_t raw_z;
	uint8_t raw_temp;

	/* Calibrated readings (after hard/soft iron correction) */
	int32_t cal_x;
	int32_t cal_y;
	int32_t cal_z;

	/* Calibration data */
	struct mmc5983ma_calibration cal;

	/* Cached register values */
	uint8_t ctrl0_cache;
	uint8_t ctrl1_cache;
	uint8_t ctrl2_cache;

	/* Continuous mode state */
	bool continuous_mode;

#ifdef CONFIG_MMC5983MA_TRIGGER
	const struct device *dev;
	struct gpio_callback gpio_cb;
	sensor_trigger_handler_t data_ready_handler;
	const struct sensor_trigger *data_ready_trigger;
#ifdef CONFIG_MMC5983MA_TRIGGER_OWN_THREAD
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_MMC5983MA_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif CONFIG_MMC5983MA_TRIGGER_GLOBAL_THREAD
	struct k_work work;
#endif
#endif /* CONFIG_MMC5983MA_TRIGGER */

#ifdef CONFIG_MMC5983MA_CALIBRATION
	/* Calibration collection state */
	bool cal_collecting;
	int32_t cal_min_x, cal_max_x;
	int32_t cal_min_y, cal_max_y;
	int32_t cal_min_z, cal_max_z;
	uint32_t cal_sample_count;
#endif
};

/*
 * Internal functions
 */

/* I2C operations */
int mmc5983ma_i2c_read(const struct device *dev, uint8_t reg, uint8_t *buf, size_t len);
int mmc5983ma_i2c_write(const struct device *dev, uint8_t reg, uint8_t val);
int mmc5983ma_i2c_read_reg(const struct device *dev, uint8_t reg, uint8_t *val);

/* Core operations */
int mmc5983ma_set_operation(const struct device *dev);
int mmc5983ma_reset_operation(const struct device *dev);
int mmc5983ma_software_reset(const struct device *dev);
int mmc5983ma_take_measurement(const struct device *dev);
int mmc5983ma_read_raw(const struct device *dev);

/* Calibration */
void mmc5983ma_apply_calibration(struct mmc5983ma_data *data);

#endif /* ZEPHYR_DRIVERS_SENSOR_MMC5983MA_MMC5983MA_H_ */
