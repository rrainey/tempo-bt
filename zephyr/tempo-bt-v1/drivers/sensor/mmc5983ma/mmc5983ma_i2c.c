/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 *
 * MEMSIC MMC5983MA I2C transport layer
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "mmc5983ma.h"

LOG_MODULE_DECLARE(mmc5983ma, CONFIG_SENSOR_LOG_LEVEL);

/*
 * Read multiple bytes from a register
 */
int mmc5983ma_i2c_read(const struct device *dev, uint8_t reg, uint8_t *buf, size_t len)
{
	const struct mmc5983ma_config *config = dev->config;

	return i2c_burst_read_dt(&config->i2c, reg, buf, len);
}

/*
 * Write a single byte to a register
 */
int mmc5983ma_i2c_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct mmc5983ma_config *config = dev->config;

	return i2c_reg_write_byte_dt(&config->i2c, reg, val);
}

/*
 * Read a single register
 */
int mmc5983ma_i2c_read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
	const struct mmc5983ma_config *config = dev->config;

	return i2c_reg_read_byte_dt(&config->i2c, reg, val);
}
