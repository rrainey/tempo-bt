/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 *
 * MEMSIC MMC5983MA 3-axis magnetometer driver
 */

#define DT_DRV_COMPAT memsic_mmc5983ma

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "mmc5983ma.h"

LOG_MODULE_REGISTER(mmc5983ma, CONFIG_SENSOR_LOG_LEVEL);

/*
 * Apply calibration to raw readings
 */
void mmc5983ma_apply_calibration(struct mmc5983ma_data *data)
{
	if (!data->cal.valid) {
		/* No calibration - just copy raw to calibrated */
		data->cal_x = data->raw_x;
		data->cal_y = data->raw_y;
		data->cal_z = data->raw_z;
		return;
	}

	/* Apply hard-iron offset correction */
	int32_t x = data->raw_x - data->cal.offset_x;
	int32_t y = data->raw_y - data->cal.offset_y;
	int32_t z = data->raw_z - data->cal.offset_z;

	/* Apply scale correction (Q1.15 fixed-point) */
	x = (x * data->cal.scale_x) >> 15;
	y = (y * data->cal.scale_y) >> 15;
	z = (z * data->cal.scale_z) >> 15;

	/* Apply soft-iron matrix if non-identity */
	/* For now, assume diagonal matrix (scale-only soft-iron) */
	/* Full matrix: cal = M * [x; y; z] where M is 3x3 */

	data->cal_x = x;
	data->cal_y = y;
	data->cal_z = z;
}

/*
 * Perform SET operation to magnetize sensor in positive direction
 */
int mmc5983ma_set_operation(const struct device *dev)
{
	int ret;

	ret = mmc5983ma_i2c_write(dev, MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_SET);
	if (ret < 0) {
		LOG_ERR("SET operation failed: %d", ret);
		return ret;
	}

	/* SET pulse takes ~500ns, but we need to wait for completion */
	k_busy_wait(MMC5983MA_SET_RESET_TIME_US);
	k_sleep(K_MSEC(1));

	return 0;
}

/*
 * Perform RESET operation to magnetize sensor in negative direction
 */
int mmc5983ma_reset_operation(const struct device *dev)
{
	int ret;

	ret = mmc5983ma_i2c_write(dev, MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_RESET);
	if (ret < 0) {
		LOG_ERR("RESET operation failed: %d", ret);
		return ret;
	}

	k_busy_wait(MMC5983MA_SET_RESET_TIME_US);
	k_sleep(K_MSEC(1));

	return 0;
}

/*
 * Perform software reset
 */
int mmc5983ma_software_reset(const struct device *dev)
{
	int ret;

	ret = mmc5983ma_i2c_write(dev, MMC5983MA_REG_CTRL1, MMC5983MA_CTRL1_SW_RST);
	if (ret < 0) {
		LOG_ERR("Software reset failed: %d", ret);
		return ret;
	}

	/* Wait for reset to complete and OTP to reload */
	k_sleep(K_MSEC(MMC5983MA_STARTUP_TIME_MS));

	return 0;
}

/*
 * Trigger a single magnetic measurement
 */
int mmc5983ma_take_measurement(const struct device *dev)
{
	struct mmc5983ma_data *data = dev->data;
	const struct mmc5983ma_config *config = dev->config;
	int ret;
	uint8_t status;
	int timeout_ms;

	/* Determine measurement time based on bandwidth */
	switch (config->bandwidth) {
	case 0:
		timeout_ms = MMC5983MA_MEAS_TIME_BW00_MS + 2;
		break;
	case 1:
		timeout_ms = MMC5983MA_MEAS_TIME_BW01_MS + 2;
		break;
	case 2:
		timeout_ms = MMC5983MA_MEAS_TIME_BW10_MS + 2;
		break;
	case 3:
		timeout_ms = MMC5983MA_MEAS_TIME_BW11_MS + 2;
		break;
	default:
		timeout_ms = 10;
	}

	/* Trigger measurement */
	ret = mmc5983ma_i2c_write(dev, MMC5983MA_REG_CTRL0,
				  data->ctrl0_cache | MMC5983MA_CTRL0_TM_M);
	if (ret < 0) {
		LOG_ERR("Failed to trigger measurement: %d", ret);
		return ret;
	}

	/* Poll for measurement complete */
	int elapsed = 0;

	while (elapsed < timeout_ms) {
		k_sleep(K_MSEC(1));
		elapsed++;

		ret = mmc5983ma_i2c_read_reg(dev, MMC5983MA_REG_STATUS, &status);
		if (ret < 0) {
			continue;
		}

		if (status & MMC5983MA_STATUS_MEAS_M_DONE) {
			return 0;
		}
	}

	LOG_WRN("Measurement timeout after %d ms", elapsed);
	return -ETIMEDOUT;
}

/*
 * Read raw magnetic data from sensor
 */
int mmc5983ma_read_raw(const struct device *dev)
{
	struct mmc5983ma_data *data = dev->data;
	uint8_t buf[7];
	int ret;

	/* Read all 7 data bytes in one burst (Xout0 through XYZout2) */
	ret = mmc5983ma_i2c_read(dev, MMC5983MA_REG_XOUT0, buf, 7);
	if (ret < 0) {
		LOG_ERR("Failed to read magnetic data: %d", ret);
		return ret;
	}

	/* Assemble 18-bit values
	 * Xout[17:10] = buf[0], Xout[9:2] = buf[1], Xout[1:0] = buf[6][7:6]
	 * Yout[17:10] = buf[2], Yout[9:2] = buf[3], Yout[1:0] = buf[6][5:4]
	 * Zout[17:10] = buf[4], Zout[9:2] = buf[5], Zout[1:0] = buf[6][3:2]
	 */
	uint32_t raw_x = ((uint32_t)buf[0] << 10) | ((uint32_t)buf[1] << 2) |
			 ((buf[6] >> 6) & 0x03);
	uint32_t raw_y = ((uint32_t)buf[2] << 10) | ((uint32_t)buf[3] << 2) |
			 ((buf[6] >> 4) & 0x03);
	uint32_t raw_z = ((uint32_t)buf[4] << 10) | ((uint32_t)buf[5] << 2) |
			 ((buf[6] >> 2) & 0x03);

	/* Convert to signed by subtracting null field offset (2^17) */
	data->raw_x = (int32_t)raw_x - MMC5983MA_18BIT_OFFSET;
	data->raw_y = (int32_t)raw_y - MMC5983MA_18BIT_OFFSET;
	data->raw_z = (int32_t)raw_z - MMC5983MA_18BIT_OFFSET;

	/* Apply calibration */
	mmc5983ma_apply_calibration(data);

	return 0;
}

/*
 * Read temperature
 */
static int mmc5983ma_read_temp(const struct device *dev)
{
	struct mmc5983ma_data *data = dev->data;
	int ret;
	uint8_t status;

	/* Trigger temperature measurement */
	ret = mmc5983ma_i2c_write(dev, MMC5983MA_REG_CTRL0,
				  data->ctrl0_cache | MMC5983MA_CTRL0_TM_T);
	if (ret < 0) {
		return ret;
	}

	/* Wait for completion */
	k_sleep(K_MSEC(10));

	ret = mmc5983ma_i2c_read_reg(dev, MMC5983MA_REG_STATUS, &status);
	if (ret < 0) {
		return ret;
	}

	if (!(status & MMC5983MA_STATUS_MEAS_T_DONE)) {
		LOG_WRN("Temperature measurement not complete");
	}

	/* Read temperature */
	ret = mmc5983ma_i2c_read_reg(dev, MMC5983MA_REG_TOUT, &data->raw_temp);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/*
 * Configure continuous measurement mode
 */
static int mmc5983ma_set_continuous_mode(const struct device *dev, uint8_t cm_freq)
{
	struct mmc5983ma_data *data = dev->data;
	int ret;

	/* Update control register 2 */
	data->ctrl2_cache &= ~(MMC5983MA_CTRL2_CM_FREQ_MASK | MMC5983MA_CTRL2_CMM_EN);

	if (cm_freq > 0) {
		data->ctrl2_cache |= (cm_freq & MMC5983MA_CTRL2_CM_FREQ_MASK);
		data->ctrl2_cache |= MMC5983MA_CTRL2_CMM_EN;
		data->continuous_mode = true;
	} else {
		data->continuous_mode = false;
	}

	ret = mmc5983ma_i2c_write(dev, MMC5983MA_REG_CTRL2, data->ctrl2_cache);
	if (ret < 0) {
		LOG_ERR("Failed to set continuous mode: %d", ret);
		return ret;
	}

	LOG_DBG("Continuous mode %s (CM_FREQ=%d)",
		data->continuous_mode ? "enabled" : "disabled", cm_freq);

	return 0;
}

/*
 * Map ODR Hz value to CM_FREQ register value
 */
static uint8_t odr_hz_to_cm_freq(uint16_t odr_hz)
{
	switch (odr_hz) {
	case 0:
		return MMC5983MA_CM_FREQ_OFF;
	case 1:
		return MMC5983MA_CM_FREQ_1HZ;
	case 10:
		return MMC5983MA_CM_FREQ_10HZ;
	case 20:
		return MMC5983MA_CM_FREQ_20HZ;
	case 50:
		return MMC5983MA_CM_FREQ_50HZ;
	case 100:
		return MMC5983MA_CM_FREQ_100HZ;
	case 200:
		return MMC5983MA_CM_FREQ_200HZ;
	case 1000:
		return MMC5983MA_CM_FREQ_1000HZ;
	default:
		/* Round to nearest supported rate */
		if (odr_hz < 5) {
			return MMC5983MA_CM_FREQ_1HZ;
		}
		if (odr_hz < 15) {
			return MMC5983MA_CM_FREQ_10HZ;
		}
		if (odr_hz < 35) {
			return MMC5983MA_CM_FREQ_20HZ;
		}
		if (odr_hz < 75) {
			return MMC5983MA_CM_FREQ_50HZ;
		}
		if (odr_hz < 150) {
			return MMC5983MA_CM_FREQ_100HZ;
		}
		if (odr_hz < 600) {
			return MMC5983MA_CM_FREQ_200HZ;
		}
		return MMC5983MA_CM_FREQ_1000HZ;
	}
}

/*
 * Sensor API: sample_fetch
 */
static int mmc5983ma_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct mmc5983ma_data *data = dev->data;
	int ret;

	switch (chan) {
	case SENSOR_CHAN_MAGN_X:
	case SENSOR_CHAN_MAGN_Y:
	case SENSOR_CHAN_MAGN_Z:
	case SENSOR_CHAN_MAGN_XYZ:
	case SENSOR_CHAN_ALL:
		if (!data->continuous_mode) {
			ret = mmc5983ma_take_measurement(dev);
			if (ret < 0) {
				return ret;
			}
		}
		ret = mmc5983ma_read_raw(dev);
		if (ret < 0) {
			return ret;
		}
		if (chan == SENSOR_CHAN_ALL) {
			/* Also read temperature */
			ret = mmc5983ma_read_temp(dev);
			if (ret < 0) {
				return ret;
			}
		}
		break;

	case SENSOR_CHAN_AMBIENT_TEMP:
	case SENSOR_CHAN_DIE_TEMP:
		ret = mmc5983ma_read_temp(dev);
		if (ret < 0) {
			return ret;
		}
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

/*
 * Sensor API: channel_get
 */
static int mmc5983ma_channel_get(const struct device *dev, enum sensor_channel chan,
				 struct sensor_value *val)
{
	struct mmc5983ma_data *data = dev->data;
	float gauss;

	switch (chan) {
	case SENSOR_CHAN_MAGN_X:
		gauss = data->cal_x * MMC5983MA_GAUSS_PER_COUNT;
		val->val1 = (int32_t)gauss;
		val->val2 = (int32_t)((gauss - val->val1) * 1000000);
		break;

	case SENSOR_CHAN_MAGN_Y:
		gauss = data->cal_y * MMC5983MA_GAUSS_PER_COUNT;
		val->val1 = (int32_t)gauss;
		val->val2 = (int32_t)((gauss - val->val1) * 1000000);
		break;

	case SENSOR_CHAN_MAGN_Z:
		gauss = data->cal_z * MMC5983MA_GAUSS_PER_COUNT;
		val->val1 = (int32_t)gauss;
		val->val2 = (int32_t)((gauss - val->val1) * 1000000);
		break;

	case SENSOR_CHAN_MAGN_XYZ:
		gauss = data->cal_x * MMC5983MA_GAUSS_PER_COUNT;
		val[0].val1 = (int32_t)gauss;
		val[0].val2 = (int32_t)((gauss - val[0].val1) * 1000000);

		gauss = data->cal_y * MMC5983MA_GAUSS_PER_COUNT;
		val[1].val1 = (int32_t)gauss;
		val[1].val2 = (int32_t)((gauss - val[1].val1) * 1000000);

		gauss = data->cal_z * MMC5983MA_GAUSS_PER_COUNT;
		val[2].val1 = (int32_t)gauss;
		val[2].val2 = (int32_t)((gauss - val[2].val1) * 1000000);
		break;

	case SENSOR_CHAN_AMBIENT_TEMP:
	case SENSOR_CHAN_DIE_TEMP: {
		float temp_c = MMC5983MA_TEMP_OFFSET +
			       (data->raw_temp * MMC5983MA_TEMP_SCALE);
		val->val1 = (int32_t)temp_c;
		val->val2 = (int32_t)((temp_c - val->val1) * 1000000);
		break;
	}

	default:
		return -ENOTSUP;
	}

	return 0;
}

/*
 * Sensor API: attr_set
 */
static int mmc5983ma_attr_set(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, const struct sensor_value *val)
{
	struct mmc5983ma_data *data = dev->data;
	int ret;

	if (chan != SENSOR_CHAN_MAGN_X && chan != SENSOR_CHAN_MAGN_Y &&
	    chan != SENSOR_CHAN_MAGN_Z && chan != SENSOR_CHAN_MAGN_XYZ) {
		return -ENOTSUP;
	}

	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY: {
		uint8_t cm_freq = odr_hz_to_cm_freq(val->val1);

		ret = mmc5983ma_set_continuous_mode(dev, cm_freq);
		if (ret < 0) {
			return ret;
		}
		break;
	}

	default:
		return -ENOTSUP;
	}

	return 0;
}

/*
 * Sensor API: attr_get
 */
static int mmc5983ma_attr_get(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, struct sensor_value *val)
{
	struct mmc5983ma_data *data = dev->data;

	if (chan != SENSOR_CHAN_MAGN_X && chan != SENSOR_CHAN_MAGN_Y &&
	    chan != SENSOR_CHAN_MAGN_Z && chan != SENSOR_CHAN_MAGN_XYZ) {
		return -ENOTSUP;
	}

	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		/* Return current ODR based on CM_FREQ */
		if (!data->continuous_mode) {
			val->val1 = 0;
		} else {
			uint8_t cm_freq = data->ctrl2_cache & MMC5983MA_CTRL2_CM_FREQ_MASK;

			switch (cm_freq) {
			case MMC5983MA_CM_FREQ_1HZ:
				val->val1 = 1;
				break;
			case MMC5983MA_CM_FREQ_10HZ:
				val->val1 = 10;
				break;
			case MMC5983MA_CM_FREQ_20HZ:
				val->val1 = 20;
				break;
			case MMC5983MA_CM_FREQ_50HZ:
				val->val1 = 50;
				break;
			case MMC5983MA_CM_FREQ_100HZ:
				val->val1 = 100;
				break;
			case MMC5983MA_CM_FREQ_200HZ:
				val->val1 = 200;
				break;
			case MMC5983MA_CM_FREQ_1000HZ:
				val->val1 = 1000;
				break;
			default:
				val->val1 = 0;
			}
		}
		val->val2 = 0;
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

/*
 * Initialize calibration from devicetree defaults
 */
static void mmc5983ma_init_calibration(const struct device *dev)
{
	struct mmc5983ma_data *data = dev->data;
	const struct mmc5983ma_config *config = dev->config;

	/* Initialize soft-iron matrix to identity */
	memset(data->cal.soft_iron, 0, sizeof(data->cal.soft_iron));
	data->cal.soft_iron[0] = 32768;  /* xx = 1.0 */
	data->cal.soft_iron[4] = 32768;  /* yy = 1.0 */
	data->cal.soft_iron[8] = 32768;  /* zz = 1.0 */

	/* Load factory calibration from devicetree */
	data->cal.offset_x = config->cal_offset_x;
	data->cal.offset_y = config->cal_offset_y;
	data->cal.offset_z = config->cal_offset_z;
	data->cal.scale_x = config->cal_scale_x;
	data->cal.scale_y = config->cal_scale_y;
	data->cal.scale_z = config->cal_scale_z;

	/* Mark as valid if any non-default values */
	if (config->cal_offset_x != 0 || config->cal_offset_y != 0 ||
	    config->cal_offset_z != 0 || config->cal_scale_x != 32768 ||
	    config->cal_scale_y != 32768 || config->cal_scale_z != 32768) {
		data->cal.valid = 1;
		data->cal.method = MMC5983MA_CAL_FACTORY;
		LOG_INF("Loaded factory calibration from devicetree");
	} else {
		data->cal.valid = 0;
		data->cal.method = MMC5983MA_CAL_NONE;
	}
}

/*
 * Device initialization
 */
static int mmc5983ma_init(const struct device *dev)
{
	struct mmc5983ma_data *data = dev->data;
	const struct mmc5983ma_config *config = dev->config;
	int ret;
	uint8_t product_id;

	LOG_INF("Initializing MMC5983MA magnetometer");

	/* Check I2C bus */
	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* Wait for power-up */
	k_sleep(K_MSEC(MMC5983MA_STARTUP_TIME_MS));

	/* Read and verify product ID */
	ret = mmc5983ma_i2c_read_reg(dev, MMC5983MA_REG_PRODUCT_ID, &product_id);
	if (ret < 0) {
		LOG_ERR("Failed to read product ID: %d", ret);
		return ret;
	}

	if (product_id != MMC5983MA_PRODUCT_ID) {
		LOG_ERR("Invalid product ID: 0x%02x (expected 0x%02x)",
			product_id, MMC5983MA_PRODUCT_ID);
		return -ENODEV;
	}

	LOG_INF("MMC5983MA detected (product ID: 0x%02x)", product_id);

	/* Perform software reset */
	ret = mmc5983ma_software_reset(dev);
	if (ret < 0) {
		return ret;
	}

	/* Perform initial SET to establish known magnetization state */
	ret = mmc5983ma_set_operation(dev);
	if (ret < 0) {
		return ret;
	}

	/* Configure bandwidth */
	data->ctrl1_cache = (config->bandwidth & 0x03);
	ret = mmc5983ma_i2c_write(dev, MMC5983MA_REG_CTRL1, data->ctrl1_cache);
	if (ret < 0) {
		LOG_ERR("Failed to set bandwidth: %d", ret);
		return ret;
	}

	/* Configure auto SET/RESET if requested */
	data->ctrl0_cache = 0;
	if (config->auto_sr) {
		data->ctrl0_cache |= MMC5983MA_CTRL0_AUTO_SR_EN;
	}
	ret = mmc5983ma_i2c_write(dev, MMC5983MA_REG_CTRL0, data->ctrl0_cache);
	if (ret < 0) {
		LOG_ERR("Failed to configure auto SR: %d", ret);
		return ret;
	}

	/* Configure continuous mode if requested */
	data->ctrl2_cache = 0;
	if (config->continuous_rate > 0) {
		uint8_t cm_freq = odr_hz_to_cm_freq(config->continuous_rate);

		ret = mmc5983ma_set_continuous_mode(dev, cm_freq);
		if (ret < 0) {
			return ret;
		}
	}

	/* Initialize calibration */
	mmc5983ma_init_calibration(dev);

	LOG_INF("MMC5983MA initialized (BW=%d, auto_sr=%d, cm_rate=%d)",
		config->bandwidth, config->auto_sr, config->continuous_rate);

	return 0;
}

/*
 * Sensor driver API
 */
static const struct sensor_driver_api mmc5983ma_driver_api = {
	.sample_fetch = mmc5983ma_sample_fetch,
	.channel_get = mmc5983ma_channel_get,
	.attr_set = mmc5983ma_attr_set,
	.attr_get = mmc5983ma_attr_get,
};

/*
 * Device instantiation macros
 */
#define MMC5983MA_INIT(inst)                                                      \
	static struct mmc5983ma_data mmc5983ma_data_##inst;                       \
                                                                                  \
	static const struct mmc5983ma_config mmc5983ma_config_##inst = {          \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                \
		.bandwidth = DT_INST_PROP(inst, bandwidth),                       \
		.continuous_rate = DT_INST_PROP(inst, continuous_mode_rate),      \
		.auto_sr = DT_INST_PROP(inst, auto_sr),                           \
		.cal_offset_x = DT_INST_PROP(inst, cal_offset_x),                 \
		.cal_offset_y = DT_INST_PROP(inst, cal_offset_y),                 \
		.cal_offset_z = DT_INST_PROP(inst, cal_offset_z),                 \
		.cal_scale_x = DT_INST_PROP(inst, cal_scale_x),                   \
		.cal_scale_y = DT_INST_PROP(inst, cal_scale_y),                   \
		.cal_scale_z = DT_INST_PROP(inst, cal_scale_z),                   \
	};                                                                        \
                                                                                  \
	SENSOR_DEVICE_DT_INST_DEFINE(inst,                                        \
				     mmc5983ma_init,                               \
				     NULL,                                        \
				     &mmc5983ma_data_##inst,                      \
				     &mmc5983ma_config_##inst,                    \
				     POST_KERNEL,                                 \
				     CONFIG_SENSOR_INIT_PRIORITY,                 \
				     &mmc5983ma_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MMC5983MA_INIT)
