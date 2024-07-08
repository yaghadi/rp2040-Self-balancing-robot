// max7219.h
#ifndef MAX7219_H
#define MAX7219_H

#include <stdint.h>

#define PICO_DEFAULT_SPI_CSN_PIN 17
#define NUM_MODULES 4
#define FONT_WIDTH 8
#define FONT_HEIGHT 8
#define NUM_ROBOT_EYES_FRAMES (sizeof(ROBOT_EYES_FRAMES) / sizeof(ROBOT_EYES_FRAMES[0]))
void spi0_init();
void blink_eyes(uint16_t startNumber,uint16_t endNumber);
void clear_display();
void refresh_display();
void display_character(char c, int offset, int module);
void display_text(const char* text);
void display_pattern(const uint8_t* pattern, int pattern_size, int module_index);
void display_robot_pupils(const uint8_t left_pupil[], const uint8_t right_pupil[], int left_pupil_size, int right_pupil_size);
void display_robot_eyes(int frame_index);
#endif // MAX7219_H