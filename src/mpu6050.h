#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#define I2C_SDA_PIN 20
#define I2C_SCL_PIN 21
#define ACCEL_CALIB_SAMPLES 1000
#define GYRO_CALIB_SAMPLES 1000
#define MPU6050_ADDR 0x68
#define DEG_TO_RAD 0.017453292519943295
#define RAD_TO_DEG 57.2957795131f
#define ALPHA 0.7

void mpu6050_reset();
void mpu6050_init();
void mpu6050_read_raw(double accel[3], int16_t gyro[3]);
void mpu6050_calibrate(double accel_offsets[3], double gyro_offsets[3]);
void mpu6050_read_calibrated(double accel[3], int16_t gyro[3], double accel_offsets[3], double gyro_offsets[3]);
void display_calibrated_values(void);
void print_calibrated_data(double accel[3], int16_t gyro[3], double *accel_calib_offsets, double *gyro_calib_offsets);
void calculate_orientation(double accel[3], int16_t gyro[3], double *roll, double *pitch, double *yaw);
void calculate_orientation_quaternion(double accel[3], int16_t gyro[3], double *roll, double *pitch, double *yaw);

#endif // MPU6050_H