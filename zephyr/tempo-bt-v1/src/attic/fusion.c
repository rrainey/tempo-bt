/*
 * SPDX-License-Identifier: MIT
 * 
 * Fusion AHRS algorithm implementation - subset for Tempo-BT
 * Based on https://github.com/xioTechnologies/Fusion
 * Copyright (c) 2021 Sebastian Madgwick
 * 
 * This is a simplified implementation containing only the functions
 * needed for IMU-only (no magnetometer) orientation tracking.
 */

#include "fusion.h"

#define INITIAL_GAIN (10.0f)
#define INITIALISATION_PERIOD (3.0f)

/**
 * @brief Initialises the AHRS algorithm structure.
 * @param ahrs AHRS algorithm structure.
 */
void FusionAhrsInitialise(FusionAhrs *const ahrs) {
    const FusionAhrsSettings settings = {
        .gain = 0.5f,
        .accelerationRejection = 90.0f,
        .magneticRejection = 90.0f,
        .rejectionTimeout = 5.0f,
    };
    FusionAhrsSetSettings(ahrs, &settings);
    
    ahrs->quaternion = (FusionQuaternion) {1.0f, 0.0f, 0.0f, 0.0f};
    ahrs->accelerometer = (FusionVector) {0.0f, 0.0f, 0.0f};
    ahrs->initialising = true;
    ahrs->rampedGain = INITIAL_GAIN;
    ahrs->angularRateRecovery = false;
    ahrs->halfAccelerometerFeedback = (FusionVector) {0.0f, 0.0f, 0.0f};
    ahrs->halfMagnetometerFeedback = (FusionVector) {0.0f, 0.0f, 0.0f};
    ahrs->accelerometerIgnored = false;
    ahrs->accelerationRejectionTimer = 0;
    ahrs->magnetometerIgnored = false;
    ahrs->magneticRejectionTimer = 0;
}

/**
 * @brief Sets the AHRS algorithm settings.
 * @param ahrs AHRS algorithm structure.
 * @param settings Settings.
 */
void FusionAhrsSetSettings(FusionAhrs *const ahrs, const FusionAhrsSettings *const settings) {
    ahrs->settings.gain = settings->gain;
    ahrs->settings.accelerationRejection = settings->accelerationRejection == 0.0f ? FLT_MAX : settings->accelerationRejection;
    ahrs->settings.magneticRejection = settings->magneticRejection == 0.0f ? FLT_MAX : settings->magneticRejection;
    ahrs->settings.rejectionTimeout = settings->rejectionTimeout;
    
    if (!ahrs->initialising) {
        ahrs->rampedGain = ahrs->settings.gain;
    }
    ahrs->rampedGainStep = (INITIAL_GAIN - ahrs->settings.gain) / INITIALISATION_PERIOD;
}

/**
 * @brief Updates the AHRS algorithm without magnetometer data.
 * @param ahrs AHRS algorithm structure.
 * @param gyroscope Gyroscope measurement in degrees per second.
 * @param accelerometer Accelerometer measurement in g.
 * @param deltaTime Delta time in seconds.
 */
void FusionAhrsUpdateNoMagnetometer(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const float deltaTime) {
    
    // Store accelerometer
    ahrs->accelerometer = accelerometer;
    
    // Reinitialise if gyroscope data is invalid
    if ((gyroscope.x == 0.0f) && (gyroscope.y == 0.0f) && (gyroscope.z == 0.0f)) {
        FusionAhrsInitialise(ahrs);
        return;
    }
    
    // Ramp gain during initialisation
    if (ahrs->initialising) {
        ahrs->rampedGain -= ahrs->rampedGainStep * deltaTime;
        if (ahrs->rampedGain < ahrs->settings.gain) {
            ahrs->rampedGain = ahrs->settings.gain;
            ahrs->initialising = false;
            ahrs->angularRateRecovery = false;
        }
    }
    
    // Calculate feedback error
    FusionVector halfGravity = {0.0f, 0.0f, 0.0f};
    ahrs->halfAccelerometerFeedback = (FusionVector) {0.0f, 0.0f, 0.0f};
    ahrs->accelerometerIgnored = true;
    
    if ((accelerometer.x != 0.0f) || (accelerometer.y != 0.0f) || (accelerometer.z != 0.0f)) {
        
        // Calculate direction of gravity assumed by quaternion (NED convention)
        const FusionQuaternion q = ahrs->quaternion;
        halfGravity = (FusionVector) {
            .x = q.w * q.y - q.x * q.z,
            .y = -(q.y * q.z + q.w * q.x),
            .z = 0.5f - q.w * q.w - q.z * q.z,
        };
        
        // Calculate accelerometer feedback
        const FusionVector accelerometerNormalised = FusionVectorNormalise(accelerometer);
        ahrs->halfAccelerometerFeedback = FusionVectorCrossProduct(accelerometerNormalised, halfGravity);
        
        // Ignore accelerometer if acceleration distortion detected
        const float accelerationMagnitude = FusionVectorMagnitude(accelerometer);
        if ((accelerationMagnitude < (1.0f - ahrs->settings.accelerationRejection)) ||
            (accelerationMagnitude > (1.0f + ahrs->settings.accelerationRejection))) {
            ahrs->accelerationRejectionTimer = 0;
        } else {
            if (ahrs->accelerationRejectionTimer >= (int)(ahrs->settings.rejectionTimeout / deltaTime)) {
                ahrs->angularRateRecovery = false;
                ahrs->accelerometerIgnored = false;
            } else {
                ahrs->accelerationRejectionTimer++;
            }
        }
    }
    
    // Apply feedback to gyroscope
    const float halfGain = 0.5f * ahrs->rampedGain;
    const FusionVector adjustedGyroscope = {
        .x = FusionDegreesToRadians(gyroscope.x) + (ahrs->accelerometerIgnored ? 0.0f : halfGain * ahrs->halfAccelerometerFeedback.x),
        .y = FusionDegreesToRadians(gyroscope.y) + (ahrs->accelerometerIgnored ? 0.0f : halfGain * ahrs->halfAccelerometerFeedback.y),
        .z = FusionDegreesToRadians(gyroscope.z) + (ahrs->accelerometerIgnored ? 0.0f : halfGain * ahrs->halfAccelerometerFeedback.z),
    };
    
    // Integrate rate of change of quaternion
    const FusionQuaternion q = ahrs->quaternion;
    ahrs->quaternion = FusionQuaternionNormalise((FusionQuaternion) {
        .w = q.w - deltaTime * (0.5f * (q.x * adjustedGyroscope.x + q.y * adjustedGyroscope.y + q.z * adjustedGyroscope.z)),
        .x = q.x + deltaTime * (0.5f * (q.w * adjustedGyroscope.x + q.y * adjustedGyroscope.z - q.z * adjustedGyroscope.y)),
        .y = q.y + deltaTime * (0.5f * (q.w * adjustedGyroscope.y - q.x * adjustedGyroscope.z + q.z * adjustedGyroscope.x)),
        .z = q.z + deltaTime * (0.5f * (q.w * adjustedGyroscope.z + q.x * adjustedGyroscope.y - q.y * adjustedGyroscope.x)),
    });
}

/**
 * @brief Updates the AHRS algorithm with magnetometer data.
 * @param ahrs AHRS algorithm structure.
 * @param gyroscope Gyroscope measurement in degrees per second.
 * @param accelerometer Accelerometer measurement in g.
 * @param magnetometer Magnetometer measurement in arbitrary units.
 * @param deltaTime Delta time in seconds.
 */
void FusionAhrsUpdate(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const FusionVector magnetometer, const float deltaTime) {
    // For Tempo V1, we don't have a magnetometer, so just call the no-magnetometer version
    FusionAhrsUpdateNoMagnetometer(ahrs, gyroscope, accelerometer, deltaTime);
}

/**
 * @brief Gets the quaternion describing the AHRS algorithm's orientation.
 * @param ahrs AHRS algorithm structure.
 * @return Quaternion.
 */
FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs *const ahrs) {
    return ahrs->quaternion;
}

/**
 * @brief Sets the quaternion describing the AHRS algorithm's orientation.
 * @param ahrs AHRS algorithm structure.
 * @param quaternion Quaternion.
 */
void FusionAhrsSetQuaternion(FusionAhrs *const ahrs, const FusionQuaternion quaternion) {
    ahrs->quaternion = quaternion;
}

/**
 * @brief Gets the Euler angles describing the AHRS algorithm's orientation.
 * @param ahrs AHRS algorithm structure.
 * @return Euler angles.
 */
FusionEuler FusionAhrsGetEuler(const FusionAhrs *const ahrs) {
    return FusionQuaternionToEuler(ahrs->quaternion);
}

/**
 * @brief Returns true if the AHRS algorithm is initialising.
 * @param ahrs AHRS algorithm structure.
 * @return True if the AHRS algorithm is initialising.
 */
bool FusionAhrsIsInitialising(const FusionAhrs *const ahrs) {
    return ahrs->initialising;
}

/**
 * @brief Returns true if the AHRS algorithm is in angular rate recovery mode.
 * @param ahrs AHRS algorithm structure.
 * @return True if the AHRS algorithm is in angular rate recovery mode.
 */
bool FusionAhrsIsAngularRateRecovery(const FusionAhrs *const ahrs) {
    return ahrs->angularRateRecovery;
}