#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "robot_control.h"
#include "max7219.h"
#include "hardware/uart.h"
#include "hc-05.h"
#include "pico/multicore.h"

// Pin assignments for stepper motor driver 1
#define STEP_PIN1 7
#define DIR_PIN1 6

// Pin assignments for stepper motor driver 2
#define STEP_PIN2 9
#define DIR_PIN2 8

#define STEPS_PER_REV 1600    // Number of steps per revolution for the stepper motor
#define MICROSTEP_MODE 8     // Microstep mode (1, 2, 4, 8, 16, 32)
#define GEAR_RATIO 1.0       // Gear ratio between the motor and the output shaft (if applicable)
double accel_offsets[3], gyro_offsets[3];

// Create a repeating timer
struct repeating_timer timer;
uint32_t timer_period_us = 100; // Timer period in microseconds (adjust as needed)

int32_t current_position1 = (STEPS_PER_REV * MICROSTEP_MODE * GEAR_RATIO * 90) / 360; // Initial position at 90 degrees
int32_t current_position2 = (STEPS_PER_REV * MICROSTEP_MODE * GEAR_RATIO * 90) / 360; // Initial position at 90 degrees
int32_t target_position1 ;  // Target position for motor 1 (in steps)
int32_t target_position2 ;  // Target position for motor 2 (in steps)

bool motor_moving1 = false; // Flag to indicate if motor 1 is moving
bool motor_moving2 = false; // Flag to indicate if motor 2 is moving

// PID controller parameters
float kp = 10.0; // Proportional gain
float ki = 0.1;  // Integral gain
float kd = 0.5;  // Derivative gain

float angle_error = 0.0;
float angle_error_integral = 0.0;
float angle_error_derivative = 0.0;
float angle_error_previous = 0.0;

float pid_output = 0.0;
double read_angle_from_sensor() {
        // Read the MPU6050 sensor data
        double accel[3];
        int16_t gyro[3];
        mpu6050_read_calibrated(accel, gyro, accel_offsets, gyro_offsets);

        double roll, pitch, yaw;
        calculate_orientation(accel, gyro, &roll, &pitch, &yaw);
    return pitch;
}
// Timer callback function
bool timer_callback(struct repeating_timer *timer) {
    static bool step_state = false;

    // Read sensor data and calculate the angle error
    double angle = read_angle_from_sensor(); // Implement this function to read the angle from the sensor
    angle_error = 0.0 - angle; // Assuming the desired angle is 0 (upright position)

    // Calculate the PID terms
    angle_error_integral += angle_error;
    angle_error_derivative = angle_error - angle_error_previous;
    angle_error_previous = angle_error;

    pid_output = kp * angle_error + ki * angle_error_integral + kd * angle_error_derivative;

    // Limit the PID output to a suitable range
    if (pid_output > 650000) {
        pid_output = 650000;
    } else if (pid_output < -650000) {
        pid_output = -650000;
    }

    // Set the target positions based on the PID output
    target_position1 = current_position1 + (int32_t)(pid_output * STEPS_PER_REV * MICROSTEP_MODE * GEAR_RATIO / 360.0);
    target_position2 = current_position2 + (int32_t)(pid_output * STEPS_PER_REV * MICROSTEP_MODE * GEAR_RATIO / 360.0);

    // Move motor 1 if it's not at the target position
    if (motor_moving1) {
        gpio_put(STEP_PIN1, step_state);
        if (current_position1 < target_position1) {
            current_position1++;
        } else if (current_position1 > target_position1) {
            current_position1--;
        } else {
            motor_moving1 = false;
        }
    }

    // Move motor 2 if it's not at the target position
    if (motor_moving2) {
        gpio_put(STEP_PIN2, step_state);
        if (current_position2 < target_position2) {
            current_position2++;
        } else if (current_position2 > target_position2) {
            current_position2--;
        } else {
            motor_moving2 = false;
        }
    }

    // Invert the step state for the next cycle
    step_state = !step_state;

    return motor_moving1 || motor_moving2; // Keep the timer running if any motor is moving
}

void change_speed(uint32_t new_speed_us) {
    timer_period_us = new_speed_us;
    cancel_repeating_timer(&timer);
    add_repeating_timer_us(timer_period_us, timer_callback, NULL, &timer);
}

void core1_entry() {
    spi0_init();
    clear_display();

    while (true) {
        display_character('F',0,0);
        display_character('S',0,1);
        display_character('T',0,2);
        display_character('H',0,3);
        sleep_ms(8000);
        clear_display();
        blink_eyes(0,7);
        sleep_ms(2000);
        clear_display();
    }
}

void bluetoothTask(void* pvParameters) {
    setup_HC_05();

    while (1) {
        if (uart_is_readable(UART_ID)) {
            char c = uart_getc(UART_ID);
            control_robot(c, 50);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
void setup_stepper_motors() {
    // Set pin modes for motor 1
    gpio_init(STEP_PIN1);
    gpio_set_dir(STEP_PIN1, GPIO_OUT);
    gpio_init(DIR_PIN1);
    gpio_set_dir(DIR_PIN1, GPIO_OUT);
    // Set pin modes for motor 2
    gpio_init(STEP_PIN2);
    gpio_set_dir(STEP_PIN2, GPIO_OUT);
    gpio_init(DIR_PIN2);
    gpio_set_dir(DIR_PIN2, GPIO_OUT);
    // Set the direction for both motors
    gpio_put(DIR_PIN1, true); // Motor 1: forward
    gpio_put(DIR_PIN2, false); // Motor 2: backward
}
int main() {
    stdio_init_all();

    setup_HC_05();

    multicore_launch_core1(core1_entry);

    //setup_robot_control();
    setup_stepper_motors();
    // Initialize and calibrate the MPU6050 sensor
    mpu6050_init();
    mpu6050_reset();
    mpu6050_calibrate(accel_offsets, gyro_offsets);

    add_repeating_timer_us(timer_period_us, timer_callback, NULL, &timer);

    //xTaskCreate(balancingTask, "Balancing Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(bluetoothTask, "Bluetooth Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();

    while (1) {
        // This should never be reached
        
    }

    return 0;
}