/*
 * =========================================================================
 * ESP32 + MPU-6050 High-Rate IMU Streamer for OpenVINS
 * =========================================================================
 *
 * Update Rate: 200 Hz (Every 5,000 microseconds / 5 ms)
 * Baud Rate:   115200 baud
 * Hardware I2C: 400 kHz Fast-Mode
 *
 * Hardware Wiring:
 *   ESP32 GPIO 21  <--->  MPU-6050 SDA
 *   ESP32 GPIO 22  <--->  MPU-6050 SCL
 *   ESP32 3.3V/5V  <--->  MPU-6050 VCC
 *   ESP32 GND      <--->  MPU-6050 GND
 *
 * Output Format:
 *   ASCII CSV Line: ax,ay,az,gx,gy,gz\n
 *   Units:
 *     ax, ay, az : Linear Acceleration in m/s^2 (includes gravity)
 *     gx, gy, gz : Angular Velocity in rad/s
 *
 * Dependencies:
 *   - Electronic Cats / I2Cdevlib MPU6050 library
 * =========================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// I2C Pins for ESP32
#define IMU_SDA 21
#define IMU_SCL 22

MPU6050 mpu;

// Scale factors:
// Default ranges: ±2g (16384 LSB/g) and ±250 deg/s (131 LSB/(deg/s))
// 1g = 9.80665 m/s^2
const float ACCEL_SCALE = 9.80665f / 16384.0f;
const float GYRO_SCALE = (3.14159265359f / 180.0f) / 131.0f;

// 200 Hz update rate (5000 us = 5.0 ms)
const uint32_t SAMPLE_PERIOD_US = 5000;
uint32_t lastSampleTime = 0;

void setup() {
  Serial.begin(115200);

  // Fast I2C bus at 400 kHz
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("ERR: MPU6050 connection failed! Check SDA/SCL wiring.");
    while (1) delay(1000);
  }

  // Pre-calibrated stationary zero-offsets (Instant boot, immune to startup movement)
  mpu.setXAccelOffset(822);
  mpu.setYAccelOffset(-1085);
  mpu.setZAccelOffset(1812);
  mpu.setXGyroOffset(-36);
  mpu.setYGyroOffset(44);
  mpu.setZGyroOffset(39);

  // Hardware Digital Low Pass Filter (DLPF):
  // MPU6050_DLPF_BW_20 = 20 Hz hardware cutoff to eliminate mechanical vibrations
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);

  lastSampleTime = micros();
}

void loop() {
  uint32_t now = micros();

  // Strict 200 Hz timing loop
  if (now - lastSampleTime >= SAMPLE_PERIOD_US) {
    lastSampleTime += SAMPLE_PERIOD_US;

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert raw ADC integers to standard SI metric units
    float ax_m = (float)ax * ACCEL_SCALE;
    float ay_m = (float)ay * ACCEL_SCALE;
    float az_m = (float)az * ACCEL_SCALE;

    float gx_r = (float)gx * GYRO_SCALE;
    float gy_r = (float)gy * GYRO_SCALE;
    float gz_r = (float)gz * GYRO_SCALE;

    // Send formatted line: ax,ay,az,gx,gy,gz
    Serial.printf("%.4f,%.4f,%.4f,%.5f,%.5f,%.5f\n", ax_m, ay_m, az_m, gx_r, gy_r, gz_r);
  }
}
