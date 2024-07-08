// hc-05.h

#ifndef HC_05_H
#define HC_05_H
// Define the UART pins for the HC-05 Bluetooth module
#define UART_ID uart0
#define UART_TX_PIN 0
#define UART_RX_PIN 1
void setup_HC_05();
void send_bluetooth_message(const char* message);
void handle_bluetooth_command(char command,int speed);
void bluetooth_loop();
#endif // HC_05_H