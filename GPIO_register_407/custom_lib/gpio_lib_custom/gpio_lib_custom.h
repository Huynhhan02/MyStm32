
#ifndef GPIO_LIB_CUSTOM_H_
#define GPIO_LIB_CUSTOM_H_
#include "stdint.h"
#define GPIOD_base_adress 0x40020C00
#define GPIOA_base_adress 0x40020000

typedef enum
{
	led_1 = 12, led_2 ,led_3, led_4
} GPIOA_led;

void led_init();
void led_control(GPIOA_led led, int led_state);
void toggle_led(GPIOA_led led);
char button_read(int button_number);




#endif /* GPIO_LIB_CUSTOM_H_ */
