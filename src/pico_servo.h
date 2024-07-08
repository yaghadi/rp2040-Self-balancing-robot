#ifndef PICO_SERVO_H
#define PICO_SERVO_H

#include <stdint.h>
#define SERVO_PIN 3
#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned int uint;

/**
 * @brief Set min and max microseconds.
 * 
 * Set min and max microseconds.
 * 
 * @param a the min time in microseconds
 * @param b the max time in microseconds
 */
void servo_set_bounds(uint a, uint b);

/**
 * @brief Set up the servo system.
 * 
 * Attach IRQ handler, allocate and initialize memory.
 */
int servo_init(void);

/**
 * @brief Reference the primary clock.
 * 
 * Set the clock source to CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY
 * 
 */
int servo_clock_auto(void);

/**
 * @brief Specify the clock source.
 *
 * Specify the clock source.
 * 
 */
int servo_clock_source(uint src);

/**
 * @brief Allocate a pin to controlling a servo
 *
 * Binds the specified pin to PWM output.
 * 
 * @param pin The pin to make PWM output.
 */
int servo_attach(uint pin);

/**
 * @brief Move a servo.
 * Move the servo on the specified pin to the specified angle in degrees.
 *
 * @param pin The PWM pin controlling a servo.
 * @param angle The angle to move to.
 */
int servo_move_to(uint pin, uint angle);

/**
 * @brief Move a servo.
 *
 * Move a servo by specifing microseconds.
 * Note that this is dangerous and can damage your servo if you are not careful!
 *
 * @param pin The PWM pin.
 * @param us The amount of time in microseconds the PWM signal is high.
 */
int servo_microseconds(uint pin, uint us);

#ifdef __cplusplus
}
#endif

#endif // PICO_SERVO_H