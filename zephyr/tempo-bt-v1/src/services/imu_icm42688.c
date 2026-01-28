/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tempo-BT V1 - IMU Service Implementation (FIFO-based)
 *
 * Uses the ICM42688 hardware FIFO for consistent sample timing.
 * The orientation service polls imu_fifo_read() to drain samples.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>

#include "services/imu.h"
#include "services/timebase.h"

LOG_MODULE_REGISTER(imu_service, LOG_LEVEL_INF);

/*
 * ICM42688 Register Definitions (subset needed for FIFO operation)
 * These match the Zephyr driver's icm42688_reg.h
 */
#define REG_DEVICE_CONFIG       0x11
#define REG_FIFO_CONFIG         0x16
#define REG_INT_STATUS          0x2D
#define REG_FIFO_COUNTH         0x2E
#define REG_FIFO_COUNTL         0x2F
#define REG_FIFO_DATA           0x30
#define REG_SIGNAL_PATH_RESET   0x4B
#define REG_PWR_MGMT0           0x4E
#define REG_GYRO_CONFIG0        0x4F
#define REG_ACCEL_CONFIG0       0x50
#define REG_FIFO_CONFIG1        0x5F
#define REG_FIFO_CONFIG2        0x60
#define REG_FIFO_CONFIG3        0x61
#define REG_WHO_AM_I            0x75

/* Register bit definitions */
#define BIT_SOFT_RESET          0x01
#define BIT_FIFO_FLUSH          0x02

/* FIFO_CONFIG mode bits */
#define FIFO_MODE_BYPASS        0x00
#define FIFO_MODE_STREAM        0x40

/* FIFO_CONFIG1 bits */
#define BIT_FIFO_TEMP_EN        0x04
#define BIT_FIFO_GYRO_EN        0x02
#define BIT_FIFO_ACCEL_EN       0x01

/* PWR_MGMT0 bits */
#define GYRO_MODE_LN            0x0C  /* Low-noise mode */
#define ACCEL_MODE_LN           0x03  /* Low-noise mode */

/* FIFO header bits */
#define FIFO_HEADER_ACCEL       0x40
#define FIFO_HEADER_GYRO        0x20
#define FIFO_HEADER_EMPTY       0x80

/* ODR values for ACCEL_CONFIG0 and GYRO_CONFIG0 */
#define ODR_200HZ               0x09
#define ODR_100HZ               0x08
#define ODR_400HZ               0x07

/* Full-scale values */
#define ACCEL_FS_16G            0x00
#define ACCEL_FS_8G             0x20
#define ACCEL_FS_4G             0x40
#define ACCEL_FS_2G             0x60

#define GYRO_FS_2000DPS         0x00
#define GYRO_FS_1000DPS         0x20
#define GYRO_FS_500DPS          0x40
#define GYRO_FS_250DPS          0x60

/* Expected WHO_AM_I values */
#define WHO_AM_I_ICM42688       0x47
#define WHO_AM_I_ICM42688V      0xDB

/* SPI read/write bit */
#define SPI_READ_BIT            0x80

/* FIFO packet size: header(1) + accel(6) + gyro(6) + temp(1) = 14 bytes
 * Note: We use the simple 16-byte packet format without timestamp */
#define FIFO_PACKET_SIZE        16

/* Device state */
static bool device_ready = false;
static bool fifo_running = false;

static imu_config_t current_config = {
    .odr_hz = 200,
    .accel_range_g = 8,
    .gyro_range_dps = 500
};

/* Sensitivity values based on current config */
static float accel_sensitivity;  /* LSB per m/s^2 */
static float gyro_sensitivity;   /* LSB per rad/s */

/* Get device tree SPI specification */
#define IMU_SPI_NODE DT_NODELABEL(icm42688)

#if !DT_NODE_EXISTS(IMU_SPI_NODE)
#error "ICM42688 device not found in device tree"
#endif

/* SPI device specification from device tree */
static const struct spi_dt_spec spi_spec = SPI_DT_SPEC_GET(IMU_SPI_NODE,
    SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER, 0);

/*
 * Low-level SPI functions
 */
static int spi_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[2] = { reg & 0x7F, value };
    struct spi_buf tx = { .buf = tx_buf, .len = 2 };
    struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

    return spi_write_dt(&spi_spec, &tx_set);
}

static int spi_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx_buf[2] = { reg | SPI_READ_BIT, 0 };
    uint8_t rx_buf[2] = { 0 };
    struct spi_buf tx = { .buf = tx_buf, .len = 2 };
    struct spi_buf rx = { .buf = rx_buf, .len = 2 };
    struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

    int ret = spi_transceive_dt(&spi_spec, &tx_set, &rx_set);

    if (ret == 0) {
        *value = rx_buf[1];
    }
    return ret;
}

static int spi_read_burst(uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t tx_buf = reg | SPI_READ_BIT;
    struct spi_buf tx = { .buf = &tx_buf, .len = 1 };
    struct spi_buf rx_bufs[2] = {
        { .buf = &tx_buf, .len = 1 },  /* dummy for tx byte */
        { .buf = data, .len = len }
    };
    struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = rx_bufs, .count = 2 };

    return spi_transceive_dt(&spi_spec, &tx_set, &rx_set);
}

/*
 * Update sensitivity values based on current config
 */
static void update_sensitivity(void)
{
    /* Accelerometer sensitivity in LSB per g, then convert to LSB per m/s^2 */
    switch (current_config.accel_range_g) {
    case 2:  accel_sensitivity = 16384.0f / 9.80665f; break;
    case 4:  accel_sensitivity = 8192.0f / 9.80665f; break;
    case 8:  accel_sensitivity = 4096.0f / 9.80665f; break;
    case 16: accel_sensitivity = 2048.0f / 9.80665f; break;
    default: accel_sensitivity = 2048.0f / 9.80665f; break;
    }

    /* Gyroscope sensitivity in LSB per deg/s, then convert to LSB per rad/s */
    switch (current_config.gyro_range_dps) {
    case 250:  gyro_sensitivity = 131.0f * 57.2957795f; break;
    case 500:  gyro_sensitivity = 65.5f * 57.2957795f; break;
    case 1000: gyro_sensitivity = 32.8f * 57.2957795f; break;
    case 2000: gyro_sensitivity = 16.4f * 57.2957795f; break;
    default:   gyro_sensitivity = 16.4f * 57.2957795f; break;
    }
}

/*
 * Get ODR register value from Hz
 */
static uint8_t hz_to_odr_reg(uint16_t hz)
{
    if (hz >= 400) return 0x07;       /* 400 Hz */
    if (hz >= 200) return 0x09;       /* 200 Hz */
    if (hz >= 100) return 0x08;       /* 100 Hz */
    if (hz >= 50)  return 0x0A;       /* 50 Hz */
    return 0x09;                       /* Default 200 Hz */
}

/*
 * Get full-scale register value for accelerometer
 */
static uint8_t accel_fs_to_reg(uint8_t g)
{
    if (g >= 16) return 0x00;
    if (g >= 8)  return 0x20;
    if (g >= 4)  return 0x40;
    return 0x60;
}

/*
 * Get full-scale register value for gyroscope
 */
static uint8_t gyro_fs_to_reg(uint16_t dps)
{
    if (dps >= 2000) return 0x00;
    if (dps >= 1000) return 0x20;
    if (dps >= 500)  return 0x40;
    return 0x60;
}

/*
 * Public API implementation
 */

int imu_init(void)
{
    int ret;
    uint8_t who_am_i;

    LOG_INF("Initializing IMU service (FIFO mode)");

    /* Check if SPI bus is ready */
    if (!spi_is_ready_dt(&spi_spec)) {
        LOG_ERR("SPI bus device not ready");
        return -ENODEV;
    }

    /* Wait for device power-up */
    k_msleep(3);

    /* Soft reset */
    ret = spi_write_reg(REG_DEVICE_CONFIG, BIT_SOFT_RESET);
    if (ret) {
        LOG_ERR("Failed to send soft reset: %d", ret);
        return ret;
    }

    /* Wait for reset to complete */
    k_msleep(10);

    /* Verify WHO_AM_I */
    ret = spi_read_reg(REG_WHO_AM_I, &who_am_i);
    if (ret) {
        LOG_ERR("Failed to read WHO_AM_I: %d", ret);
        return ret;
    }

    if (who_am_i != WHO_AM_I_ICM42688 && who_am_i != WHO_AM_I_ICM42688V) {
        LOG_ERR("Invalid WHO_AM_I: 0x%02X (expected 0x47 or 0xDB)", who_am_i);
        return -EINVAL;
    }

    LOG_INF("ICM42688 detected (WHO_AM_I=0x%02X)", who_am_i);

    /* Apply default configuration */
    update_sensitivity();

    device_ready = true;
    return 0;
}

int imu_configure(const imu_config_t *config)
{
    int ret;
    uint8_t accel_cfg, gyro_cfg;

    if (!device_ready) {
        return -ENODEV;
    }

    if (!config) {
        return -EINVAL;
    }

    current_config = *config;
    update_sensitivity();

    /* Configure accelerometer: ODR + full-scale */
    accel_cfg = hz_to_odr_reg(config->odr_hz) | accel_fs_to_reg(config->accel_range_g);
    ret = spi_write_reg(REG_ACCEL_CONFIG0, accel_cfg);
    if (ret) {
        LOG_ERR("Failed to configure accelerometer: %d", ret);
        return ret;
    }

    /* Configure gyroscope: ODR + full-scale */
    gyro_cfg = hz_to_odr_reg(config->odr_hz) | gyro_fs_to_reg(config->gyro_range_dps);
    ret = spi_write_reg(REG_GYRO_CONFIG0, gyro_cfg);
    if (ret) {
        LOG_ERR("Failed to configure gyroscope: %d", ret);
        return ret;
    }

    LOG_INF("IMU configured: %d Hz, %dg, %d dps",
            config->odr_hz, config->accel_range_g, config->gyro_range_dps);

    return 0;
}

int imu_fifo_start(void)
{
    int ret;

    if (!device_ready) {
        return -ENODEV;
    }

    if (fifo_running) {
        return 0;  /* Already running */
    }

    LOG_INF("Starting IMU FIFO");

    /* Enable gyro and accel in low-noise mode */
    ret = spi_write_reg(REG_PWR_MGMT0, GYRO_MODE_LN | ACCEL_MODE_LN);
    if (ret) {
        LOG_ERR("Failed to set power mode: %d", ret);
        return ret;
    }

    /* Wait for sensors to start */
    k_msleep(50);

    /* Apply configuration */
    ret = imu_configure(&current_config);
    if (ret) {
        return ret;
    }

    /* Configure FIFO: enable accel, gyro, temp */
    ret = spi_write_reg(REG_FIFO_CONFIG1, BIT_FIFO_ACCEL_EN | BIT_FIFO_GYRO_EN | BIT_FIFO_TEMP_EN);
    if (ret) {
        LOG_ERR("Failed to configure FIFO content: %d", ret);
        return ret;
    }

    /* Flush FIFO before starting */
    ret = spi_write_reg(REG_SIGNAL_PATH_RESET, BIT_FIFO_FLUSH);
    if (ret) {
        LOG_ERR("Failed to flush FIFO: %d", ret);
        return ret;
    }

    k_msleep(1);

    /* Enable FIFO in stream mode */
    ret = spi_write_reg(REG_FIFO_CONFIG, FIFO_MODE_STREAM);
    if (ret) {
        LOG_ERR("Failed to enable FIFO stream: %d", ret);
        return ret;
    }

    fifo_running = true;
    LOG_INF("IMU FIFO started");

    return 0;
}

int imu_fifo_stop(void)
{
    int ret;

    if (!device_ready) {
        return -ENODEV;
    }

    if (!fifo_running) {
        return 0;
    }

    LOG_INF("Stopping IMU FIFO");

    /* Set FIFO to bypass mode */
    ret = spi_write_reg(REG_FIFO_CONFIG, FIFO_MODE_BYPASS);
    if (ret) {
        LOG_ERR("Failed to disable FIFO: %d", ret);
        return ret;
    }

    /* Flush FIFO */
    ret = spi_write_reg(REG_SIGNAL_PATH_RESET, BIT_FIFO_FLUSH);
    if (ret) {
        LOG_ERR("Failed to flush FIFO: %d", ret);
        return ret;
    }

    fifo_running = false;
    LOG_INF("IMU FIFO stopped");

    return 0;
}

int imu_fifo_read(imu_sample_t *samples, size_t max_count)
{
    int ret;
    uint8_t count_buf[2];
    uint16_t fifo_count;
    size_t packets_to_read;
    static uint8_t fifo_buf[FIFO_PACKET_SIZE * 16];  /* Read up to 16 packets at once */
    uint64_t now_us;

    if (!device_ready || !fifo_running) {
        return -ENODEV;
    }

    if (!samples || max_count == 0) {
        return -EINVAL;
    }

    /* Read FIFO count (2 bytes, big-endian) */
    ret = spi_read_burst(REG_FIFO_COUNTH, count_buf, 2);
    if (ret) {
        LOG_ERR("Failed to read FIFO count: %d", ret);
        return ret;
    }

    fifo_count = (count_buf[0] << 8) | count_buf[1];

    if (fifo_count == 0) {
        return 0;  /* No data available */
    }

    /* Calculate number of complete packets */
    packets_to_read = fifo_count / FIFO_PACKET_SIZE;
    if (packets_to_read == 0) {
        return 0;  /* Not enough data for a complete packet */
    }

    /* Limit to max_count and buffer size */
    if (packets_to_read > max_count) {
        packets_to_read = max_count;
    }
    if (packets_to_read > 16) {
        packets_to_read = 16;
    }

    /* Read FIFO data */
    ret = spi_read_burst(REG_FIFO_DATA, fifo_buf, packets_to_read * FIFO_PACKET_SIZE);
    if (ret) {
        LOG_ERR("Failed to read FIFO data: %d", ret);
        return ret;
    }

    /* Get current time for timestamping */
    now_us = time_now_us();

    /* Parse packets
     * Packet format (16 bytes):
     * [0]    Header
     * [1-2]  Accel X (big-endian, signed)
     * [3-4]  Accel Y
     * [5-6]  Accel Z
     * [7-8]  Gyro X
     * [9-10] Gyro Y
     * [11-12] Gyro Z
     * [13]   Temperature (8-bit)
     * [14-15] Timestamp (not used in basic mode)
     */
    size_t valid_count = 0;
    uint64_t sample_period_us = 1000000 / current_config.odr_hz;

    for (size_t i = 0; i < packets_to_read; i++) {
        uint8_t *pkt = &fifo_buf[i * FIFO_PACKET_SIZE];
        uint8_t header = pkt[0];

        /* Check for empty packet marker */
        if (header & FIFO_HEADER_EMPTY) {
            continue;
        }

        /* Only process packets with both accel and gyro data */
        if ((header & (FIFO_HEADER_ACCEL | FIFO_HEADER_GYRO)) !=
            (FIFO_HEADER_ACCEL | FIFO_HEADER_GYRO)) {
            continue;
        }

        /* Parse raw values (big-endian, signed 16-bit) */
        int16_t accel_x = (int16_t)((pkt[1] << 8) | pkt[2]);
        int16_t accel_y = (int16_t)((pkt[3] << 8) | pkt[4]);
        int16_t accel_z = (int16_t)((pkt[5] << 8) | pkt[6]);
        int16_t gyro_x  = (int16_t)((pkt[7] << 8) | pkt[8]);
        int16_t gyro_y  = (int16_t)((pkt[9] << 8) | pkt[10]);
        int16_t gyro_z  = (int16_t)((pkt[11] << 8) | pkt[12]);
        int8_t temp_raw = (int8_t)pkt[13];

        /* Convert to physical units */
        samples[valid_count].accel_x = (float)accel_x / accel_sensitivity;
        samples[valid_count].accel_y = (float)accel_y / accel_sensitivity;
        samples[valid_count].accel_z = (float)accel_z / accel_sensitivity;
        samples[valid_count].gyro_x  = (float)gyro_x / gyro_sensitivity;
        samples[valid_count].gyro_y  = (float)gyro_y / gyro_sensitivity;
        samples[valid_count].gyro_z  = (float)gyro_z / gyro_sensitivity;
        samples[valid_count].temperature = 25.0f + ((float)temp_raw / 2.07f);

        /* Estimate timestamp: oldest sample first, working forward */
        samples[valid_count].timestamp_us = now_us -
            ((packets_to_read - 1 - valid_count) * sample_period_us);

        valid_count++;
    }

    return (int)valid_count;
}

int imu_get_config(imu_config_t *config)
{
    if (!config) {
        return -EINVAL;
    }

    *config = current_config;
    return 0;
}
