#include "mpu6050.h"
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_CHAN i2c0
#define ACCEL_CALIB_SAMPLES 1000
#define GYRO_CALIB_SAMPLES 1000

// Calibration constants
double accel_offset[3] = {0.0}; // Offset values for accelerometer (x, y, z)
double gyro_offset[3] = {0.0};  // Offset values for gyroscope (x, y, z)

// MPU6050 sensor data
static double acceleration[3];
static int16_t gyro[3];

// Orientation angles
static double yaw = 0.0;
static double pitch = 0.0;
static double roll = 0.0;

// Global variables
double thetaM = 0.0;
double phiM = 0.0;
double phiFnew = 0.0;
double thetaFnew = 0.0;
double phiFold = 0.0;
double thetaFold = 0.0;
double phi = 0.0;
double theta = 0.0;
double dt = 0.0;
double phiG = 0.0;
double thetaG = 0.0;
double phiRad = 0.0;
double thetaRad = 0.0;
uint64_t millisOld = 0;

void mpu6050_reset(){
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(I2C_CHAN, MPU6050_ADDR, buf, 2, false);

    uint8_t gyro_rate[] = {0x19, 0b00000111};
    i2c_write_blocking(I2C_CHAN, MPU6050_ADDR, gyro_rate, 2, false);

    uint8_t gyro_settings[] = {0x1b, 0b00000000};
    i2c_write_blocking(I2C_CHAN, MPU6050_ADDR, gyro_settings, 2, false);

    uint8_t accel_settings[] = {0x1c, 0b00000000};
    i2c_write_blocking(I2C_CHAN, MPU6050_ADDR, accel_settings, 2, false);

    uint8_t pin_settings[] = {0x37, 0b00010000};
    i2c_write_blocking(I2C_CHAN, MPU6050_ADDR, pin_settings, 2, false);

    uint8_t int_config[] = {0x38, 0x01};
    i2c_write_blocking(I2C_CHAN, MPU6050_ADDR, int_config, 2, false);
}

int32_t multfix15(int16_t a, int16_t b) {
    // Perform fixed-point multiplication
    int32_t result = (int32_t)a * (int32_t)b;
    
    // Adjust the result to keep only the 15 most significant bits
    result >>= 15;
    
    return result;
}

void mpu6050_read_raw(double accel[3], int16_t gyro[3]) {
    uint8_t buffer[6];
    int16_t temp_accel, temp_gyro;

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C_CHAN, MPU6050_ADDR, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(I2C_CHAN, MPU6050_ADDR, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        temp_accel = (buffer[i<<1] << 8 | buffer[(i<<1) + 1]);
        accel[i] = temp_accel * 9.8 / 16384.0; // Convert raw data to m/s²
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(I2C_CHAN, MPU6050_ADDR, &val, 1, true);
    i2c_read_blocking(I2C_CHAN, MPU6050_ADDR, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        temp_gyro = (buffer[i<<1] << 8 | buffer[(i<<1) + 1]);
        gyro[i] = temp_gyro ;
        gyro[i] = multfix15(gyro[i], 500); // deg/sec
    }
}

void mpu6050_calibrate(double accel_offsets[3], double gyro_offsets[3]) {
    double accel_sum[3] = {0.0, 0.0, 0.0};
    double gyro_sum[3] = {0.0, 0.0, 0.0};
    double accel[3];
    int16_t gyro[3];

    // Collect and process calibration data
    for (int i = 0; i < ACCEL_CALIB_SAMPLES; i++) {
        mpu6050_read_raw(accel, gyro);

        // Accumulate the raw accelerometer and gyroscope data
        for (int j = 0; j < 3; j++) {
            accel_sum[j] += accel[j];
            gyro_sum[j] += gyro[j];
        }

        sleep_ms(1); // Delay between samples
    }

    // Calculate the accelerometer offsets
    for (int i = 0; i < 3; i++) {
        accel_offsets[i] = accel_sum[i] / ACCEL_CALIB_SAMPLES;
    }

    // Calculate the gyroscope offsets
    for (int i = 0; i < 3; i++) {
        gyro_offsets[i] = gyro_sum[i] / GYRO_CALIB_SAMPLES;
    }

    // Adjust the accelerometer offsets to account for gravity
    accel_offsets[2] -= 9.8;
}

void mpu6050_read_calibrated(double accel[3], int16_t gyro[3], double accel_offsets[3], double gyro_offsets[3]) {
    double raw_accel[3];
    int16_t raw_gyro[3];
    mpu6050_read_raw(raw_accel, raw_gyro);

    for (int i = 0; i < 3; i++) {
        accel[i] = raw_accel[i] - accel_offsets[i];
        gyro[i] = raw_gyro[i] - gyro_offsets[i];
    }
}

void print_calibrated_data(double accel[3], int16_t gyro[3], double *accel_calib_offsets, double *gyro_calib_offsets) {
    printf("Calibrated Accelerometer Data (m/s^2): X=%.2lf, Y=%.2lf, Z=%.2lf\n",
           accel[0] + accel_calib_offsets[0], accel[1] + accel_calib_offsets[1], accel[2] + accel_calib_offsets[2]);
    printf("Calibrated Gyroscope Data (degrees/s): X=%d, Y=%d, Z=%d\n",
           gyro[0] + (int16_t)gyro_calib_offsets[0], gyro[1] + (int16_t)gyro_calib_offsets[1], gyro[2] + (int16_t)gyro_calib_offsets[2]);
}

void calculate_orientation(double accel[3], int16_t gyro[3], double *roll, double *pitch, double *yaw) {
    static double roll_angle = 0.0, pitch_angle = 0.0, yaw_angle = 0.0;
    static double last_gyro_x = 0.0, last_gyro_y = 0.0;

    // Calculate roll and pitch angles using accelerometer data
    *pitch = -atan2(accel[1], accel[2]) * 180 / M_PI;
    *roll= atan2(-accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2])) * 180 / M_PI;

    // Complementary filter to combine accelerometer and gyroscope data
    roll_angle = ALPHA * (roll_angle + gyro[0] * 0.001) + (1 - ALPHA) * *roll;
    pitch_angle = ALPHA * (pitch_angle + gyro[1] * 0.001) + (1 - ALPHA) * *pitch;
    yaw_angle += (gyro[2] - last_gyro_x) * 0.001; // Integrate yaw rate
    last_gyro_x = gyro[2];

    *roll = roll_angle;
    *pitch = pitch_angle;
    *yaw = yaw_angle;
}
void mpu6050_init(){
    i2c_init(I2C_CHAN, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}