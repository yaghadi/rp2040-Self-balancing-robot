#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pico/stdlib.h>
#include <hardware/uart.h>
#include <math.h>
#include "hc-05.h"
#include "robot_control.h"


// float Kp = 0.5;
// float Ki = 0.1;
// float Kd = 0.1;

void setup_HC_05() {
    // Initialize the UART interface for the HC-05 Bluetooth module
    uart_init(UART_ID, 9600);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Set the UART flow control CTS/RTS lines
    uart_set_hw_flow(UART_ID, false, false);

    // Set the UART format
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);

    // Set the UART FIFO threshold
    uart_set_fifo_enabled(UART_ID, true);
}
void send_bluetooth_message(const char* message) {
    uart_puts(uart0, message);
}
void handle_bluetooth_command(char command,int speed) {
    control_robot(command,speed);
}
void bluetooth_loop() {
    char buffer[64] = {0};
    int idx = 0;

    while (true) {
        // Check if there's any data available on the UART
        if (uart_is_readable(uart0)) {
            char c = uart_getc(uart0);
            if (c == ',') {
                char command = buffer[0];
                int speed = atoi(buffer + 1);
                handle_bluetooth_command(command, speed);
                idx = 0;
                memset(buffer, 0, sizeof(buffer));
            } else {
                buffer[idx++] = c;
            }
        }
    }
}