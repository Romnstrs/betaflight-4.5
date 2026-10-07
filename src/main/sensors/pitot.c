/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_PITOT

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/pitot/pitot_ms4525.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "scheduler/scheduler.h"

#include "sensors/barometer.h"
#include "sensors/pitot.h"
#include "sensors/sensors.h"

// Board configs rarely set I2C_DEVICE, so default to the bus the baro or mag
// uses, which is normally the one broken out on the external I2C pads.
#ifndef PITOT_I2C_INSTANCE
#if defined(BARO_I2C_INSTANCE)
#define PITOT_I2C_INSTANCE BARO_I2C_INSTANCE
#elif defined(MAG_I2C_INSTANCE)
#define PITOT_I2C_INSTANCE MAG_I2C_INSTANCE
#else
#define PITOT_I2C_INSTANCE I2C_DEVICE
#endif
#endif

#define PITOT_CALIBRATION_SAMPLES TASK_PITOT_RATE_HZ  // ~1 s of at-rest samples
#define AIR_DENSITY_SEA_LEVEL     1.225f    // kg/m^3
#define AIR_GAS_CONSTANT          287.05f   // J/(kg*K), dry air
#define ISA_SEA_LEVEL_PRESSURE    101325.0f // Pa
#define ISA_SEA_LEVEL_TEMP        288.15f   // K
#define AIR_TEMP_MIN_K            233.0f    // -40 C; outside this a temperature reading is ignored
#define AIR_TEMP_MAX_K            333.0f    // +60 C
#define AIR_DENSITY_MIN           0.3f      // kg/m^3; outside this the density estimate is rejected
#define AIR_DENSITY_MAX           1.5f
#define PITOT_LPF_HZ              5.0f      // differential-pressure smoothing
#define PITOT_SIGNAL_TIMEOUT_US   1000000   // no valid sample from any source -> sensor lost

pitot_t pitot;

static uint16_t calibrationCount = 0;
static float calibrationAccum = 0.0f;
static float sourceZero[PITOT_HARDWARE_COUNT];     // at-rest offset captured per source
static bool sourceCalibrated[PITOT_HARDWARE_COUNT];
static pt1Filter_t diffPressureLpf;
static bool lpfInitialised = false;
static bool i2cReady = false;
static pitotSensor_e activeSource = PITOT_NONE;
static timeUs_t lastValidSampleUs = 0;

PG_REGISTER_WITH_RESET_FN(pitotConfig_t, pitotConfig, PG_PITOT_CONFIG, 0);

void pgResetFn_pitotConfig(pitotConfig_t *config)
{
    config->pitot_hardware = PITOT_NONE;
    config->pitot_busType = BUS_TYPE_I2C;
    config->pitot_i2c_device = I2C_DEV_TO_CFG(PITOT_I2C_INSTANCE);
    config->pitot_i2c_address = 0;  // 0 = driver default (MS4525: 0x28)
    config->pitot_use_tas = 0;
}

bool pitotIsConfigured(void)
{
    return pitotConfig()->pitot_hardware != PITOT_NONE;
}

static bool detectI2C(void)
{
#ifdef USE_PITOT_MS4525
    extDevice_t *extDev = &pitot.dev.dev;
    if (pitotConfig()->pitot_busType != BUS_TYPE_I2C) {
        return false;
    }
    i2cBusSetInstance(extDev, pitotConfig()->pitot_i2c_device);
    extDev->busType_u.i2c.address = pitotConfig()->pitot_i2c_address;
    return ms4525Detect(&pitot.dev);
#else
    return false;
#endif
}

void pitotInit(void)
{
    if (!pitotIsConfigured()) {
        return;
    }
    // The I2C part is probed synchronously here (registers the bus device).
    // SENSOR_PITOT is only set once it actually delivers a sample (see pitotUpdate).
    const pitotSensor_e hardware = pitotConfig()->pitot_hardware;
    if (hardware == PITOT_MS4525 || hardware == PITOT_DEFAULT) {
        i2cReady = detectI2C();
    }
    // Each source is zeroed on its first at-rest sample (see pitotUpdate).
}

static bool readI2C(float *diffPressurePa, float *temperatureK)
{
    return i2cReady && pitot.dev.read && pitot.dev.read(&pitot.dev, diffPressurePa, temperatureK);
}

// Reads the configured source. MS4525 (I2C) is the only backend in this build,
// so AUTO and MS4525 behave the same. Returns the source that produced the sample.
static pitotSensor_e readActiveSample(float *diffPressurePa, float *temperatureK)
{
    switch (pitotConfig()->pitot_hardware) {
    case PITOT_MS4525:
    case PITOT_DEFAULT:
        return readI2C(diffPressurePa, temperatureK) ? PITOT_MS4525 : PITOT_NONE;
    default:
        return PITOT_NONE;
    }
}

void pitotStartCalibration(void)
{
    calibrationCount = PITOT_CALIBRATION_SAMPLES;
    calibrationAccum = 0.0f;
    if (activeSource != PITOT_NONE) {
        sourceCalibrated[activeSource] = false;
    }
}

bool pitotIsCalibrated(void)
{
    return activeSource != PITOT_NONE && sourceCalibrated[activeSource];
}

static float airspeedFromPressure(float diffPressurePa)
{
    // Indicated airspeed, cm/s: v = sqrt(2*q/rho). Sign carries flow direction.
    const float q = fabsf(diffPressurePa);
    const float v = sqrtf(2.0f * q / AIR_DENSITY_SEA_LEVEL);
    return (diffPressurePa < 0.0f ? -v : v) * 100.0f;
}

static bool isPlausibleAirTemp(float temperatureK)
{
    return temperatureK >= AIR_TEMP_MIN_K && temperatureK <= AIR_TEMP_MAX_K;
}

// Static (ambient) pressure in Pa from the barometer, or 0 if there is none.
static float staticPressure(void)
{
#ifdef USE_BARO
    if (sensors(SENSOR_BARO) && baro.pressure > 0) {
        return baro.pressure;
    }
#endif
    return 0.0f;
}

// Outside air temperature in kelvin. Prefers the pitot sensor's own reading,
// then the barometer's, and otherwise assumes the ISA standard atmosphere at the
// current pressure altitude. Both sensor readings are die temperatures, so they
// can read warm if the sensor sits next to heat sources.
static float airTemperature(float pitotTemperatureK, float staticPa)
{
    if (isPlausibleAirTemp(pitotTemperatureK)) {
        return pitotTemperatureK;
    }
#ifdef USE_BARO
    if (sensors(SENSOR_BARO) && baro.pressure > 0) {
        const float baroTemperatureK = baro.temperature / 100.0f + 273.15f;
        if (isPlausibleAirTemp(baroTemperatureK)) {
            return baroTemperatureK;
        }
    }
#endif
    // ISA troposphere: T = T0 * (p / p0)^(R * L / g)
    return ISA_SEA_LEVEL_TEMP * powf(staticPa / ISA_SEA_LEVEL_PRESSURE, 0.190263f);
}

// Air density from the ideal gas law, or 0 when there is no usable estimate.
static float airDensity(float pitotTemperatureK)
{
    const float staticPa = staticPressure();
    if (staticPa <= 0.0f) {
        return 0.0f;
    }
    const float density = staticPa / (AIR_GAS_CONSTANT * airTemperature(pitotTemperatureK, staticPa));
    return (density >= AIR_DENSITY_MIN && density <= AIR_DENSITY_MAX) ? density : 0.0f;
}

uint32_t pitotUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    float diffPressurePa;
    float temperatureK;
    const pitotSensor_e source = readActiveSample(&diffPressurePa, &temperatureK);
    if (source == PITOT_NONE) {
        // No source has data. After a bounded gap, declare the sensor lost so a
        // caller does not keep reading a frozen airspeed.
        if (activeSource != PITOT_NONE
                && cmpTimeUs(currentTimeUs, lastValidSampleUs) > PITOT_SIGNAL_TIMEOUT_US) {
            activeSource = PITOT_NONE;
            sensorsClear(SENSOR_PITOT);
            pitot.diffPressure = 0.0f;
            pitot.airspeed = 0.0f;
            pitot.trueAirspeed = 0.0f;
            pitot.airDensity = 0.0f;
        }
        return TASK_PERIOD_HZ(TASK_PITOT_RATE_HZ);
    }
    lastValidSampleUs = currentTimeUs;

    if (source != activeSource) {
        // The sensor delivered its first sample, or came back after a dropout.
        // A source is only zeroed once, and only while disarmed, so a mid-flight
        // recovery reuses the stored offset instead of capturing dynamic
        // pressure as the zero.
        activeSource = source;
        sensorsSet(SENSOR_PITOT);
        lpfInitialised = false;
        if (!sourceCalibrated[source] && !ARMING_FLAG(ARMED)) {
            pitotStartCalibration();
        }
    }

    if (!lpfInitialised) {
        pt1FilterInit(&diffPressureLpf, pt1FilterGain(PITOT_LPF_HZ, 1.0f / TASK_PITOT_RATE_HZ));
        // Start from the first sample rather than 0, so the zeroing average and
        // the airspeed after a source switch are not dragged towards zero while
        // the filter settles.
        diffPressureLpf.state = diffPressurePa;
        lpfInitialised = true;
    }
    diffPressurePa = pt1FilterApply(&diffPressureLpf, diffPressurePa);

    // Zeroing only runs while disarmed (at rest); arming abandons a partial pass.
    if (calibrationCount > 0) {
        if (ARMING_FLAG(ARMED)) {
            calibrationCount = 0;
        } else {
            calibrationAccum += diffPressurePa;
            if (--calibrationCount == 0) {
                sourceZero[source] = calibrationAccum / PITOT_CALIBRATION_SAMPLES;
                sourceCalibrated[source] = true;
            }
        }
    }

    pitot.pressureZero = sourceZero[source];
    pitot.temperature = temperatureK;
    pitot.diffPressure = diffPressurePa - sourceZero[source];
    pitot.airspeed = airspeedFromPressure(pitot.diffPressure);

    // TAS = IAS * sqrt(rho0 / rho). Without a density estimate, fall back to IAS.
    pitot.airDensity = airDensity(temperatureK);
    pitot.trueAirspeed = pitot.airDensity > 0.0f
        ? pitot.airspeed * sqrtf(AIR_DENSITY_SEA_LEVEL / pitot.airDensity)
        : pitot.airspeed;

    DEBUG_SET(DEBUG_PITOT, 0, lrintf(pitot.airspeed));
    DEBUG_SET(DEBUG_PITOT, 1, lrintf(pitot.diffPressure));
    DEBUG_SET(DEBUG_PITOT, 2, lrintf(diffPressurePa));
    DEBUG_SET(DEBUG_PITOT, 3, lrintf(pitot.temperature - 273.15f));
    DEBUG_SET(DEBUG_PITOT, 4, lrintf(pitot.trueAirspeed));
    DEBUG_SET(DEBUG_PITOT, 5, lrintf(pitot.airDensity * 1000.0f));   // g/m^3

    return TASK_PERIOD_HZ(TASK_PITOT_RATE_HZ);
}

float pitotGetAirspeed(void)
{
    return pitotConfig()->pitot_use_tas ? pitot.trueAirspeed : pitot.airspeed;
}

float pitotGetIndicatedAirspeed(void)
{
    return pitot.airspeed;
}

pitotSensor_e pitotGetActiveSource(void)
{
    return activeSource;
}

#endif // USE_PITOT
