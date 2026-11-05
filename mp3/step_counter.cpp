#include "step_counter.h"
#include <Wire.h>
#include <math.h>

#define MPU6050_ADDR 0x68
#define ACCEL_XOUT_H 0x3B

void StepCounter::begin() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0);    // Wake up the MPU6050
    Wire.endTransmission();
    // Optional: set accelerometer range to +/- 2g (default)
}

bool StepCounter::readAccel(float &ax, float &ay, float &az) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return false;
    
    Wire.requestFrom(MPU6050_ADDR, 6);
    if (Wire.available() < 6) return false;
    
    int16_t rawX = (Wire.read() << 8) | Wire.read();
    int16_t rawY = (Wire.read() << 8) | Wire.read();
    int16_t rawZ = (Wire.read() << 8) | Wire.read();
    
    // Convert to g (assuming +/- 2g range, 16384 LSB/g)
    ax = rawX / 16384.0;
    ay = rawY / 16384.0;
    az = rawZ / 16384.0;
    return true;
}

void StepCounter::calibrate() {
    float sum = 0;
    for (int i = 0; i < 100; ++i) {
        float ax, ay, az;
        if (readAccel(ax, ay, az)) {
            sum += sqrt(ax*ax + ay*ay + az*az); // Magnitude
        }
        delay(10);
    }
    baseline_ = sum / 100.0f;
}

void StepCounter::update() {
    float ax, ay, az;
    if (!readAccel(ax, ay, az)) return; // I2C error
    
    // We use the magnitude of acceleration to be orientation-independent
    float magnitude = sqrt(ax*ax + ay*ay + az*az);
    uint32_t now = millis();
    
    // Dynamic baseline: slowly adapts to the average signal level
    baseline_ = (baseline_ * 0.99f) + (magnitude * 0.01f);
    
    switch (state_) {
        case S_IDLE:
            if (magnitude > baseline_ + STEP_THRESHOLD) {
                state_ = S_ASCEND;
                peakAccel_ = magnitude;
                peakTimeMs_ = now;
            }
            break;
            
        case S_ASCEND:
            if (magnitude > peakAccel_) {
                peakAccel_ = magnitude;
                peakTimeMs_ = now;
            } else if (magnitude < baseline_) {
                state_ = S_DESCEND;
            }
            break;
            
        case S_DESCEND:
            uint32_t dt = now - peakTimeMs_;
            if (dt >= MIN_STEP_MS && dt <= MAX_STEP_MS) {
                stepCount_++;
            }
            state_ = S_IDLE;
            break;
    }
}