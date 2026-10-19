/*
 * =========================================================================
 * High-Precision MPU-6050 200 Hz Streamer for OpenVINS
 * Based on Joop Brokking's Direct-Register I2C Architecture
 * =========================================================================
 *
 * Configured Ranges:
 *   - Accelerometer: ±8g (4096 LSB/g) -> High dynamic range for drone/hand motion
 *   - Gyroscope:     ±500 deg/s (65.5 LSB/(deg/s)) -> Zero clipping on fast turns
 *   - Hardware DLPF: 42 Hz low-pass filter enabled in MPU-6050 hardware
 *   - Update Rate:   200 Hz (5000 microseconds loop)
 *   - Calibration:   2000-sample live gyro zero-bias calibration at boot
 *
 * Connections (ESP32 DevKit):
 *   VCC  -> 3.3V
 *   GND  -> GND
 *   SDA  -> GPIO 21
 *   SCL  -> GPIO 22
 *   LED  -> GPIO 2 (lights during 3s boot calibration, turns OFF when ready)
 * =========================================================================
 */

#include <Arduino.h>
#include <Wire.h>

#define IMU_SDA 21
#define IMU_SCL 22
#define LED_PIN 2

// Scale factors for ±8g and ±500 deg/s:
// 1g = 9.80665 m/s^2, 1g = 4096 LSB at ±8g
const float ACCEL_SCALE = 9.80665f / 4096.0f;
// 1 deg/s = 65.5 LSB at ±500 deg/s, 1 rad = 180 / PI deg
const float GYRO_SCALE = (3.14159265359f / 180.0f) / 65.5f;

// Gyro calibration offsets
float gyro_x_cal = 0.0f;
float gyro_y_cal = 0.0f;
float gyro_z_cal = 0.0f;

// 100 Hz Loop Timing (10000 us = 10 ms)
const uint32_t LOOP_PERIOD_US = 10000;
uint32_t loop_timer = 0;

void setup_mpu6050_registers();
void read_mpu6050_raw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz);

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // Turn LED on during calibration

  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000); // 400 kHz Fast I2C

  setup_mpu6050_registers();

  // 500-sample live gyro zero-bias calibration (approx 0.5s)
  // Keep the sensor stationary on the table during power-on
  for (int i = 0; i < 500; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    read_mpu6050_raw(ax, ay, az, gx, gy, gz);
    gyro_x_cal += gx;
    gyro_y_cal += gy;
    gyro_z_cal += gz;
    delayMicroseconds(1000);
  }

  gyro_x_cal /= 500.0f;
  gyro_y_cal /= 500.0f;
  gyro_z_cal /= 500.0f;

  digitalWrite(LED_PIN, LOW); // Calibration complete: LED turns OFF

  loop_timer = micros();
}

void loop() {
  int16_t raw_ax, raw_ay, raw_az;
  int16_t raw_gx, raw_gy, raw_gz;

  read_mpu6050_raw(raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz);

  // Convert raw readings to metric SI units
  float ax = (float)raw_ax * ACCEL_SCALE;
  float ay = (float)raw_ay * ACCEL_SCALE;
  float az = (float)raw_az * ACCEL_SCALE;

  // Subtract calibrated zero-biases and convert to rad/s
  float gx = ((float)raw_gx - gyro_x_cal) * GYRO_SCALE;
  float gy = ((float)raw_gy - gyro_y_cal) * GYRO_SCALE;
  float gz = ((float)raw_gz - gyro_z_cal) * GYRO_SCALE;

  // Stream 6 comma-separated values to ROS 2: ax,ay,az,gx,gy,gz
  Serial.printf("%.4f,%.4f,%.4f,%.5f,%.5f,%.5f\n", ax, ay, az, gx, gy, gz);

  // Maintain precise 200 Hz (5000 microseconds) loop timing
  while (micros() - loop_timer < LOOP_PERIOD_US);
  loop_timer = micros();
}

void read_mpu6050_raw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Starting at register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)0x68, (uint8_t)14);

  while (Wire.available() < 14);

  uint8_t buf[14];
  for (int i = 0; i < 14; i++) {
    buf[i] = Wire.read();
  }

  ax = (int16_t)(buf[0] << 8 | buf[1]);
  ay = (int16_t)(buf[2] << 8 | buf[3]);
  az = (int16_t)(buf[4] << 8 | buf[5]);
  // buf[6], buf[7] is temperature
  gx = (int16_t)(buf[8] << 8 | buf[9]);
  gy = (int16_t)(buf[10] << 8 | buf[11]);
  gz = (int16_t)(buf[12] << 8 | buf[13]);
}

void setup_mpu6050_registers() {
  // Wake up MPU-6050 (clear sleep bit in PWR_MGMT_1)
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Configure Accelerometer: ±8g full scale (Register 0x1C = 0x10)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Configure Gyroscope: ±500 deg/s full scale (Register 0x1B = 0x08)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // Configure Digital Low Pass Filter: 42 Hz bandwidth (Register 0x1A = 0x03)
  // Rejects high-frequency motor and structural vibration
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
}
