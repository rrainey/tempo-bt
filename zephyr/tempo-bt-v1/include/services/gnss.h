/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - GNSS Service
 */

#ifndef SERVICES_GNSS_H
#define SERVICES_GNSS_H

#include <stdint.h>
#include <stdbool.h>

/* GNSS fix data structure */
typedef struct {
    /* Position */
    double latitude;        /* Degrees, -90 to +90 */
    double longitude;       /* Degrees, -180 to +180 */
    double altitude;        /* Meters above MSL */
    
    /* Velocity */
    float speed_mps;        /* Speed in m/s */
    float course_deg;       /* Course over ground in degrees, true */
    
    /* Quality */
    uint8_t fix_quality;    /* 0=invalid, 1=GPS, 2=DGPS, etc */
    uint8_t num_satellites; /* Number of satellites used */
    float hdop;             /* Horizontal dilution of precision */
    float vdop;             /* Vertical dilution of precision */

    uint16_t year;          /* UTC year (4 digits, 2000 = 0) */
    uint8_t month;          /* UTC month (1-12) */    
    uint8_t day;             /* UTC day (1-31) */
    
    /* Time */
    uint8_t hours;          /* UTC hours (0-23) */
    uint8_t minutes;        /* UTC minutes (0-59) */
    uint8_t seconds;        /* UTC seconds (0-59) */
    uint16_t milliseconds;  /* UTC milliseconds (0-999) */
    
    /* Validity */
    bool position_valid;
    bool time_valid;
    bool date_valid;        /* True if date is valid */
    
    /* Timestamp */
    uint64_t timestamp_us;  /* Monotonic timestamp when fix was received */
} gnss_fix_t;

/* Forward declaration for NAV5 config structure */
struct ubx_cfg_nav5;

/* Dynamic platform models */
typedef enum {
    GNSS_DYNMODEL_PORTABLE = 0,     /* Default, low acceleration */
    GNSS_DYNMODEL_STATIONARY = 2,   /* Stationary, 2m/s max speed */
    GNSS_DYNMODEL_PEDESTRIAN = 3,   /* Walking, 30m/s max speed */
    GNSS_DYNMODEL_AUTOMOTIVE = 4,   /* Car, 100m/s max speed */
    GNSS_DYNMODEL_SEA = 5,          /* Marine, 25m/s max speed */
    GNSS_DYNMODEL_AIRBORNE_1G = 6,  /* Airborne <1g, 100m/s max speed */
    GNSS_DYNMODEL_AIRBORNE_2G = 7,  /* Airborne <2g, 250m/s max speed */
    GNSS_DYNMODEL_AIRBORNE_4G = 8,  /* Airborne <4g, 500m/s max speed */
    GNSS_DYNMODEL_WRIST = 9         /* Wrist watch */
} gnss_dynmodel_t;

/* Callback type for GNSS fix updates */
typedef void (*gnss_fix_callback_t)(const gnss_fix_t *fix);

/* Callback type for raw NMEA sentences */
typedef void (*gnss_nmea_callback_t)(const char *sentence, size_t len);

/**
 * @brief Early GNSS quiet - call very early in boot
 *
 * Sends commands to quiet the GNSS module and drain any pending data.
 * This should be called before other initialization to prevent buffer
 * overruns from a still-running GNSS after CPU reset.
 *
 * @return 0 on success, negative error code on failure
 */
int gnss_early_quiet(void);

/**
 * @brief Initialize the GNSS service
 *
 * Sets up UART for GNSS communication
 *
 * @return 0 on success, negative error code on failure
 */
int gnss_init(void);

/**
 * @brief Register callback for fix updates
 * 
 * @param callback Function to call when new fix is available
 */
void gnss_register_fix_callback(gnss_fix_callback_t callback);

/**
 * @brief Register callback for raw NMEA sentences
 * 
 * @param callback Function to call for each NMEA sentence
 */
void gnss_register_nmea_callback(gnss_nmea_callback_t callback);

/**
 * @brief Get current fix data
 * 
 * @param fix Output structure to fill
 * @return true if fix is valid, false otherwise
 */
bool gnss_get_current_fix(gnss_fix_t *fix);


/**
 * @brief Configure GNSS update rate
 * 
 * @param rate_hz Update rate in Hz (1 or 10 for SAM-M10Q)
 * @return 0 on success, negative error code on failure
 */
int gnss_set_rate(uint8_t rate_hz);

/**
 * @brief Configure GNSS update rate
 * 
 * @param rate_hz Update rate in Hz (1 or 10 for SAM-M10Q)
 * @return 0 on success, negative error code on failure
 */
int gnss_set_rate(uint8_t rate_hz);

/**
 * @brief Set GNSS power mode
 * 
 * @param low_power true for low power mode, false for full power
 * @return 0 on success, negative error code on failure
 */
int gnss_set_power_mode(bool low_power);

/**
 * @brief Put GNSS into standby mode
 * 
 * Reduces power consumption while maintaining settings
 * 
 * @return 0 on success, negative error code on failure
 */
int gnss_standby(void);

/**
 * @brief Wake GNSS from standby
 * 
 * @return 0 on success, negative error code on failure
 */
int gnss_wakeup(void);

/**
 * @brief Save current GNSS configuration to non-volatile memory
 * 
 * @return 0 on success, negative error code on failure
 */
int gnss_save_config(void);


/**
 * @brief Get current NAV5 configuration
 * 
 * Polls the GNSS receiver for its current navigation settings
 * 
 * @param dynmodel Output: Current dynamic platform model (can be NULL)
 * @param fix_mode Output: Current fix mode (can be NULL)
 * @return 0 on success, negative error code on failure
 */
int gnss_get_nav5_config(gnss_dynmodel_t *dynmodel, uint8_t *fix_mode);

/**
 * @brief Set dynamic platform model
 * 
 * Configures the receiver's dynamic model for optimal performance
 * based on the expected platform dynamics.
 * 
 * For skydiving, use GNSS_DYNMODEL_AIRBORNE_4G
 * 
 * @param dynmodel Dynamic platform model to set
 * @return 0 on success, negative error code on failure
 */
int gnss_set_dynmodel(gnss_dynmodel_t dynmodel);

/**
 * @brief Set full NAV5 configuration
 * 
 * Advanced configuration of all NAV5 parameters
 * 
 * @param config Pointer to complete NAV5 configuration structure
 * @return 0 on success, negative error code on failure
 */
int gnss_set_nav5_config(const struct ubx_cfg_nav5 *config);

/**
 * @brief Configure GNSS for skydiving use
 *
 * Sets optimal configuration for skydiving:
 * - Dynamic model: Airborne 4g
 * - Update rate: 1Hz (will be increased during freefall)
 *
 * @return 0 on success, negative error code on failure
 */
int gnss_init_skydiving(void);

/**
 * @brief Set session start time for NMEA timestamping
 *
 * @param start_us Session start time in microseconds
 */
void gnss_set_session_start_time(uint64_t start_us);

/**
 * @brief Enable GNSS time pulse output
 *
 * Configures the SAM-M10Q to output a 1 Hz time pulse on the TIMEPULSE pin.
 * The pulse is aligned to GPS time-of-week when locked, and runs at 1 Hz
 * regardless of lock status.
 *
 * Configuration:
 * - Period: 1,000,000 us (1 Hz)
 * - Pulse length: 100,000 us (100ms high, 900ms low)
 * - Time grid: GPS
 * - Aligned to TOW when locked
 * - Rising edge polarity
 *
 * @return 0 on success, negative error code on failure
 */
int gnss_enable_timepulse(void);

/**
 * @brief Check if GNSS has a valid position fix
 *
 * A valid fix requires fix_quality > 0 and at least 3 satellites.
 * This is useful for UI indicators (e.g., LED status).
 *
 * @return true if GNSS has a valid position fix, false otherwise
 */
bool gnss_has_fix(void);

/**
 * @brief Check if system time has been set from GNSS
 *
 * Returns true once valid date/time has been received from GNSS
 * and used to set the system clock.
 *
 * @return true if system time was set from GNSS, false otherwise
 */
bool gnss_time_is_set(void);

#ifdef CONFIG_GNSS_TEST_MODE
/**
 * @brief Test UBX communication with GNSS module
 *
 * Runs a series of tests to validate UBX binary protocol communication:
 * - TX completion tracking (no EBUSY errors)
 * - UBX message framing and checksum
 * - ACK/NAK response parsing
 *
 * @return 0 if all tests pass, negative error code if any fail
 */
int gnss_test_ubx_communication(void);
#endif

#endif /* SERVICES_GNSS_H */