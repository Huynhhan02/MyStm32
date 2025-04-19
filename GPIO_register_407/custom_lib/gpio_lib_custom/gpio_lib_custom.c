#include "gpio_lib_custom.h"

void led_init(){

		uint32_t* GPIOD_MODER = (uint32_t*)(GPIOD_base_adress + 0x00);
		uint32_t* GPIOD_OTYPER = (uint32_t*)(GPIOD_base_adress + 0x04);

		*GPIOD_MODER &= ~(0b11111111<<24);
		*GPIOD_MODER |= (0b01010110<<24);
		*GPIOD_OTYPER &= ~(0b1111<<12);


}


void led_control(GPIOA_led led, int led_state)
{
	uint32_t* GPIOD_ODR = (uint32_t*)(GPIOD_base_adress + 0x14);
	if(led_state == 1){
		*GPIOD_ODR |= (0b1<<led);
	}
	else{
		*GPIOD_ODR &= ~(0b1<<led);
	}
}
void toggle_led(GPIOA_led led)
{
	uint32_t* GPIOD_ODR = (uint32_t*)(GPIOD_base_adress + 0x14);
	if(((*GPIOD_ODR>>led)& 0x01) == 1)
		led_control(led, 0);
	else
		led_control(led, 1);
}

char button_read(int button_number)
{
	uint32_t* GPIOA_IDR = (uint32_t*) (GPIOA_base_adress + 0x10);
	return (*GPIOA_IDR>>button_number) & 0x01;
}
