#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "mpu6050.h"
#include "math.h"

extern float kp;
extern float ki;
extern float kd;
extern float MAX_SPEED;
extern float ITERM_MAX;
void move_both_steppers_to_angle(double left_angle, double right_angle);
void setup_robot_control();
void move_stepper_motor(int motor_num, int steps, int direction);
void control_stepper_motors(float balance_output);
void set_stepper_motor_direction(int motor_num, int direction);
void move_both_stepper_motors(int steps1, int steps2);
// float stabilityPDControl(float dt, float angle, float target_angle, float Kp, float Kd);
// float speedPIControl(float dt, int16_t estimated_speed, int16_t throttle, float Kp, float Ki);
// float positionPDControl(long actual_pos, long target_pos, float Kp, float Kd, int16_t speed);
// void pid_control_motors(bool is_raiseup);
void control_robot(char command, int speed);

#endif // ROBOT_CONTROL_H