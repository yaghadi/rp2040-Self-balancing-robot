#include <stdio.h>
#include <stdlib.h>
#include <pico/stdlib.h>
#include <math.h>
#include "hardware/pwm.h"
#include "robot_control.h"
#include "mpu6050.h"
#include "pico_servo.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pico/bootrom.h"

#define MOTOR1_DIR_PIN 6
#define MOTOR1_STEP_PIN 7
#define MOTOR2_DIR_PIN 8
#define MOTOR2_STEP_PIN 9


#define BUZZER_PIN 14

#define STEPS_PER_REVOLUTION 200
#define MICROSTEP_RESOLUTION 8 // Set the microstepping resolution (1, 2, 4, 8, 16, etc.)
#define ANGLE_OFFSET -0.1

#define MOTOR_SPEED_LIMIT 60 // Maximum motor speed
#define MOTOR_SPEED 400
// Number of steps per revolution
#define STEPS_PER_REV 1600

// Function to move both stepper motors to a specific angle
void move_both_steppers_to_angle(double left_angle, double right_angle) {
    // Calculate the number of steps needed for each motor
    int left_steps = (int)(left_angle / 360.0 * STEPS_PER_REV);
    int right_steps = (int)(right_angle / 360.0 * STEPS_PER_REV);

    // Determine the direction of rotation for each motor
    if (left_steps < 0) {
        gpio_put(MOTOR1_DIR_PIN, 0); // Backward direction
        
    } else {
        gpio_put(MOTOR1_DIR_PIN, 1); // Forward direction
        left_steps = -left_steps;    
    }

    if (right_steps < 0) {
        gpio_put(MOTOR2_DIR_PIN, 1); // Backward direction
        right_steps = -right_steps;
    } else {
        gpio_put(MOTOR2_DIR_PIN, 0); // Forward direction
    }

    // Move both motors the required number of steps
    for (int i = 0; i < (int)fmax(left_steps, right_steps); i++) {
        if (i < left_steps) {
            gpio_put(MOTOR1_STEP_PIN, 1);
            sleep_us(100); // Adjust the delay as needed for your stepper motor
            gpio_put(MOTOR1_STEP_PIN, 0);
            sleep_us(100); // Adjust the delay as needed for your stepper motor
        }

        if (i < right_steps) {
            gpio_put(MOTOR2_STEP_PIN, 1);
            sleep_us(100); // Adjust the delay as needed for your stepper motor
            gpio_put(MOTOR2_STEP_PIN, 0);
            sleep_us(100); // Adjust the delay as needed for your stepper motor
        }
    }
}
void set_stepper_motor_direction(int motor_num, int direction) {
    if (motor_num == 1) {
        gpio_put(MOTOR1_DIR_PIN, direction);
    } else if (motor_num == 2) {
        gpio_put(MOTOR2_DIR_PIN, direction);
    }
}
void move_stepper_motor(int motor_num, int steps, int direction) {
    set_stepper_motor_direction(motor_num, direction);

    // Ramp-up phase
    for (int i = 0; i < steps / 3; i++) {
        gpio_put(motor_num == 1 ? MOTOR1_STEP_PIN : MOTOR2_STEP_PIN, 1);
        sleep_us(200 / (MICROSTEP_RESOLUTION * (i + 1))); // Gradually decrease the delay
        gpio_put(motor_num == 1 ? MOTOR1_STEP_PIN : MOTOR2_STEP_PIN, 0);
        sleep_us(200 / (MICROSTEP_RESOLUTION * (i + 1))); // Gradually decrease the delay
    }

    // Constant speed phase
    for (int i = steps / 3; i < (steps * 2) / 3; i++) {
        gpio_put(motor_num == 1 ? MOTOR1_STEP_PIN : MOTOR2_STEP_PIN, 1);
        sleep_us(200 / MICROSTEP_RESOLUTION);
        gpio_put(motor_num == 1 ? MOTOR1_STEP_PIN : MOTOR2_STEP_PIN, 0);
        sleep_us(200 / MICROSTEP_RESOLUTION);
    }

    // Ramp-down phase
    for (int i = (steps * 2) / 3; i < steps; i++) {
        gpio_put(motor_num == 1 ? MOTOR1_STEP_PIN : MOTOR2_STEP_PIN, 1);
        sleep_us(200 / (MICROSTEP_RESOLUTION * (steps - i))); // Gradually increase the delay
        gpio_put(motor_num == 1 ? MOTOR1_STEP_PIN : MOTOR2_STEP_PIN, 0);
        sleep_us(200 / (MICROSTEP_RESOLUTION * (steps - i))); // Gradually increase the delay
    }
}
void control_stepper_motors(float balance_output) {
    static int motor1_steps = 0;
    static int motor2_steps = 0;
    static int motor1_direction = 0;
    static int motor2_direction = 0;

    // Determine the direction and number of steps for each motor
    if (balance_output > 0) {
        motor1_direction = 0; // Forward
        motor2_direction = 1; // Backward
    } else {
        motor1_direction = 1; // Backward
        motor2_direction = 0; // Forward
    }

    motor1_steps = abs((int)(balance_output * MICROSTEP_RESOLUTION));
    motor2_steps = abs((int)(balance_output * MICROSTEP_RESOLUTION));

    // Move the stepper motors
    move_stepper_motor(1, motor1_steps, motor1_direction);
    move_stepper_motor(2, motor2_steps, motor2_direction);
}
void move_both_stepper_motors(int steps1, int steps2) {
    // Generate step pulses for both motors simultaneously
    for (int i = 0; i < fmax(steps1, steps2); i++) {
        if (i < steps1) {
            for (int j = 0; j < steps1; j++) {
                gpio_put(MOTOR1_STEP_PIN, 1);
                sleep_us(100); // Adjust the delay to control the speed
                gpio_put(MOTOR1_STEP_PIN, 0);
                sleep_us(100); // Adjust the delay to control the speed
            }
        }
        if (i < steps2) {
            for (int j = 0; j < steps1; j++) {
                gpio_put(MOTOR2_STEP_PIN, 1);
                sleep_us(100); // Adjust the delay to control the speed
                gpio_put(MOTOR2_STEP_PIN, 0);
                sleep_us(100); // Adjust the delay to control the speed
            }
        }
    }
}
void move_both_stepper_motors1(int steps1, int steps2) {
    // Set up the PWM channels for the motors
    pwm_config config = pwm_get_default_config();
    pwm_init(MOTOR1_STEP_PIN, &config, true);
    pwm_init(MOTOR2_STEP_PIN, &config, true);

    // Calculate the PWM duty cycle based on the desired speed
    uint16_t duty_cycle = (uint16_t)(65535 * (1.0 - 1.0 / MOTOR_SPEED));

    // Generate step pulses for both motors simultaneously
    for (int i = 0; i < fmax(steps1, steps2); i++) {
        if (i < steps1) {
            pwm_set_enabled(MOTOR1_STEP_PIN, true);
            sleep_us(duty_cycle);
            pwm_set_enabled(MOTOR1_STEP_PIN, false);
            sleep_us(65535 - duty_cycle);
        }
        if (i < steps2) {
            pwm_set_enabled(MOTOR2_STEP_PIN, true);
            sleep_us(duty_cycle);
            pwm_set_enabled(MOTOR2_STEP_PIN, false);
            sleep_us(65535 - duty_cycle);
        }
    }

    // Disable the PWM channels
    pwm_set_enabled(MOTOR1_STEP_PIN, false);
    pwm_set_enabled(MOTOR2_STEP_PIN, false);
}
void setup_robot_control() {
    gpio_init(MOTOR1_DIR_PIN);
    gpio_init(MOTOR1_STEP_PIN);
    gpio_init(MOTOR2_DIR_PIN);
    gpio_init(MOTOR2_STEP_PIN);

    gpio_set_dir(MOTOR1_DIR_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR1_STEP_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR2_DIR_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR2_STEP_PIN, GPIO_OUT);

    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    mpu6050_init();
    mpu6050_reset();
    servo_init();
    servo_clock_auto();
    servo_attach(SERVO_PIN);
}

void control_robot(char command, int speed) {
    int buzzer_duration=0;
    // Limit the speed to the maximum value
    speed = fmax(fmin(speed, MOTOR_SPEED_LIMIT), -MOTOR_SPEED_LIMIT);

    switch (command) {
        case 'F':
            set_stepper_motor_direction(1, 0);
            set_stepper_motor_direction(2, 1);
            move_both_stepper_motors(speed, speed);
            break;
        case 'B':
            set_stepper_motor_direction(1, 1);
            set_stepper_motor_direction(2, 0);
            move_both_stepper_motors(speed, speed);
            break;
        case 'L':
            set_stepper_motor_direction(1, 0);
            set_stepper_motor_direction(2, 1);
            move_both_stepper_motors(speed, speed);
            break;
        case 'R':
            set_stepper_motor_direction(1, 1);
            set_stepper_motor_direction(2, 0);
            move_both_stepper_motors(speed, speed);
            break;
        case 'X':
            printf("servo is moving\n");
            servo_move_to(SERVO_PIN,0);
            sleep_ms(500);
            servo_move_to(SERVO_PIN,80);
            sleep_ms(500);
            servo_move_to(SERVO_PIN,0);
            sleep_ms(500);
            break;
        case 'P':
            kp += 0.001;
            printf("Kp updated to: %.4f\n", kp);
            break;
        case 'p':
            kp -= 0.001;
            printf("Kp updated to: %.4f\n", kp);
            break;
        case 'I':
            ki += 0.00001;
            printf("Ki updated to: %.4f\n", ki);
            break;
        case 'i':
            ki -= 0.00001;
            printf("Ki updated to: %.4f\n", ki);
            break;
        case 'D':
            kd += 0.001;
            printf("Kd updated to: %.4f\n",kd);
            break;
        case 'd':
            kd -= 0.001;
            printf("Kd updated to: %.4f\n",kd);
            break;    
        case 'Y':
            buzzer_duration = 500; // 0.5 second buzzer honk
            printf("HONK\n");
            break;
        case 'E':
            printf("Resetting to USB bootloader mode...\n");
            reset_usb_boot(21, 0);
            sleep_ms(500);
            break;
        // case 'K':
        //     MAX_SPEED += 20;
        //     printf("speed updated to: %.2f\n", MAX_SPEED);
        //     break;
        // case 'k':
        //     MAX_SPEED -= 10;
        //     printf("speed updated to: %.2f\n", MAX_SPEED);
        //     break;
        // case 'N':
        //     ITERM_MAX += 20;
        //     printf("ITERM updated to: %.2f\n", ITERM_MAX);
        //     break;
        // case 'n':
        //     ITERM_MAX -= 20;
        //     printf("ITERM updated to: %.2f\n", ITERM_MAX);
        //     break;
        case 'S':
        default:
            move_both_stepper_motors(0, 0);
            break;
    }
    if (buzzer_duration > 0) {
        gpio_put(BUZZER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(buzzer_duration));
        gpio_put(BUZZER_PIN, 0);
    }
}