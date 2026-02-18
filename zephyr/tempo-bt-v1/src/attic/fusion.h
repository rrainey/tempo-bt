/*
 * SPDX-License-Identifier: MIT
 * 
 * Fusion library types - subset for Tempo-BT
 * Based on https://github.com/xioTechnologies/Fusion
 * Copyright (c) 2021 Sebastian Madgwick
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef FUSION_H
#define FUSION_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define M_PI 3.14159265358979323846f

/**
 * @brief Three-axis vector.
 */
typedef struct {
    float x;
    float y;
    float z;
} FusionVector;

/**
 * @brief Quaternion.
 */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} FusionQuaternion;

/**
 * @brief Euler angles.
 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
} FusionEuler;

/**
 * @brief AHRS algorithm settings.
 */
typedef struct {
    float gain;
    float accelerationRejection;
    float magneticRejection;
    float rejectionTimeout;
} FusionAhrsSettings;

/**
 * @brief AHRS algorithm internal states.
 */
typedef struct {
    FusionAhrsSettings settings;
    FusionQuaternion quaternion;
    FusionVector accelerometer;
    bool initialising;
    float rampedGain;
    float rampedGainStep;
    bool angularRateRecovery;
    FusionVector halfAccelerometerFeedback;
    FusionVector halfMagnetometerFeedback;
    bool accelerometerIgnored;
    int accelerationRejectionTimer;
    bool magnetometerIgnored;
    int magneticRejectionTimer;
} FusionAhrs;

/**
 * @brief Converts degrees to radians.
 * @param degrees Degrees.
 * @return Radians.
 */
static inline float FusionDegreesToRadians(const float degrees) {
    return degrees * (M_PI / 180.0f);
}

/**
 * @brief Converts radians to degrees.
 * @param radians Radians.
 * @return Degrees.
 */
static inline float FusionRadiansToDegrees(const float radians) {
    return radians * (180.0f / M_PI);
}

/**
 * @brief Returns the arc sine of the value.
 * @param value Value.
 * @return Arc sine of the value.
 */
static inline float FusionAsin(const float value) {
    if (value <= -1.0f) {
        return M_PI / -2.0f;
    }
    if (value >= 1.0f) {
        return M_PI / 2.0f;
    }
    return asinf(value);
}

/**
 * @brief Returns the arc tangent of the value.
 * @param y y component.
 * @param x x component.
 * @return Arc tangent of the value.
 */
static inline float FusionAtan2(const float y, const float x) {
    return atan2f(y, x);
}

/**
 * @brief Returns the vector magnitude squared.
 * @param vector Vector.
 * @return Vector magnitude squared.
 */
static inline float FusionVectorMagnitudeSquared(const FusionVector vector) {
    return (vector.x * vector.x) + (vector.y * vector.y) + (vector.z * vector.z);
}

/**
 * @brief Returns the vector magnitude.
 * @param vector Vector.
 * @return Vector magnitude.
 */
static inline float FusionVectorMagnitude(const FusionVector vector) {
    return sqrtf(FusionVectorMagnitudeSquared(vector));
}

/**
 * @brief Returns the normalised vector.
 * @param vector Vector.
 * @return Normalised vector.
 */
static inline FusionVector FusionVectorNormalise(const FusionVector vector) {
    const float magnitude = FusionVectorMagnitude(vector);
    if (magnitude == 0.0f) {
        return vector;
    }
    const float reciprocal = 1.0f / magnitude;
    return (FusionVector) {
        .x = vector.x * reciprocal,
        .y = vector.y * reciprocal,
        .z = vector.z * reciprocal,
    };
}

/**
 * @brief Returns the vector cross product.
 * @param vectorA Vector A.
 * @param vectorB Vector B.
 * @return Vector cross product.
 */
static inline FusionVector FusionVectorCrossProduct(const FusionVector vectorA, const FusionVector vectorB) {
    return (FusionVector) {
        .x = (vectorA.y * vectorB.z) - (vectorA.z * vectorB.y),
        .y = (vectorA.z * vectorB.x) - (vectorA.x * vectorB.z),
        .z = (vectorA.x * vectorB.y) - (vectorA.y * vectorB.x),
    };
}

/**
 * @brief Returns the quaternion conjugate.
 * @param quaternion Quaternion.
 * @return Quaternion conjugate.
 */
static inline FusionQuaternion FusionQuaternionConjugate(const FusionQuaternion quaternion) {
    return (FusionQuaternion) {
        .w = quaternion.w,
        .x = -quaternion.x,
        .y = -quaternion.y,
        .z = -quaternion.z,
    };
}

/**
 * @brief Returns the normalised quaternion.
 * @param quaternion Quaternion.
 * @return Normalised quaternion.
 */
static inline FusionQuaternion FusionQuaternionNormalise(const FusionQuaternion quaternion) {
    const float magnitude = sqrtf((quaternion.w * quaternion.w) + 
                                  (quaternion.x * quaternion.x) + 
                                  (quaternion.y * quaternion.y) + 
                                  (quaternion.z * quaternion.z));
    if (magnitude == 0.0f) {
        return quaternion;
    }
    const float reciprocal = 1.0f / magnitude;
    return (FusionQuaternion) {
        .w = quaternion.w * reciprocal,
        .x = quaternion.x * reciprocal,
        .y = quaternion.y * reciprocal,
        .z = quaternion.z * reciprocal,
    };
}

/**
 * @brief Returns the quaternion multiplied by the quaternion.
 * @param quaternionA Quaternion A.
 * @param quaternionB Quaternion B.
 * @return Quaternion multiplied by the quaternion.
 */
static inline FusionQuaternion FusionQuaternionMultiply(const FusionQuaternion quaternionA, const FusionQuaternion quaternionB) {
    return (FusionQuaternion) {
        .w = (quaternionA.w * quaternionB.w) - (quaternionA.x * quaternionB.x) - (quaternionA.y * quaternionB.y) - (quaternionA.z * quaternionB.z),
        .x = (quaternionA.w * quaternionB.x) + (quaternionA.x * quaternionB.w) + (quaternionA.y * quaternionB.z) - (quaternionA.z * quaternionB.y),
        .y = (quaternionA.w * quaternionB.y) - (quaternionA.x * quaternionB.z) + (quaternionA.y * quaternionB.w) + (quaternionA.z * quaternionB.x),
        .z = (quaternionA.w * quaternionB.z) + (quaternionA.x * quaternionB.y) - (quaternionA.y * quaternionB.x) + (quaternionA.z * quaternionB.w),
    };
}

/**
 * @brief Converts a quaternion to Euler angles.
 * @param quaternion Quaternion.
 * @return Euler angles.
 */
static inline FusionEuler FusionQuaternionToEuler(const FusionQuaternion quaternion) {
    return (FusionEuler) {
        .roll = FusionRadiansToDegrees(FusionAtan2(2.0f * ((quaternion.w * quaternion.x) + (quaternion.y * quaternion.z)), 
                                                    1.0f - 2.0f * ((quaternion.x * quaternion.x) + (quaternion.y * quaternion.y)))),
        .pitch = FusionRadiansToDegrees(FusionAsin(2.0f * ((quaternion.w * quaternion.y) - (quaternion.z * quaternion.x)))),
        .yaw = FusionRadiansToDegrees(FusionAtan2(2.0f * ((quaternion.w * quaternion.z) + (quaternion.x * quaternion.y)), 
                                                   1.0f - 2.0f * ((quaternion.y * quaternion.y) + (quaternion.z * quaternion.z)))),
    };
}

/* AHRS algorithm functions */
void FusionAhrsInitialise(FusionAhrs *const ahrs);
void FusionAhrsSetSettings(FusionAhrs *const ahrs, const FusionAhrsSettings *const settings);
void FusionAhrsUpdate(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const FusionVector magnetometer, const float deltaTime);
void FusionAhrsUpdateNoMagnetometer(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const float deltaTime);
FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs *const ahrs);
void FusionAhrsSetQuaternion(FusionAhrs *const ahrs, const FusionQuaternion quaternion);
FusionEuler FusionAhrsGetEuler(const FusionAhrs *const ahrs);
bool FusionAhrsIsInitialising(const FusionAhrs *const ahrs);
bool FusionAhrsIsAngularRateRecovery(const FusionAhrs *const ahrs);

#endif /* FUSION_H */