/*
 * RCC_lib_custom.h
 *
 *  Created on: Apr 27, 2025
 *      Author: deffen
 */

#ifndef RCC_LIB_CUSTOM_RCC_LIB_CUSTOM_H_
#define RCC_LIB_CUSTOM_RCC_LIB_CUSTOM_H_

#include "stdint.h"
#define RCC_base_adress 	0x40023800
#define PWR_base_adress 	0x40007000

typedef enum{
	HSI = 0,
	HSE,
}RCC_type_t;

typedef enum{
	GPIOAen = 0,
	GPIOBen,
	GPIOCen,
	GPIODen,
	GPIOEen,
	GPIOFen,
	GPIOGen,
	GPIOHen,
	GPIOIen,
	CRCen = 12,
	ETHMACTXen = 26,
	ETHMACRXen,
	ETHMACPTPen

}RCC_AHB1_bit;
typedef enum{
	TIM2en,
	TIM3en,
	TIM4en,
	TIM5en,
	TIM6en,
	TIM7en,
	TIM12en,
	TIM13en,
	TIM14en,
	WWDGen = 11,
	SPI2en = 14,
	SPI3en,
	UART2en = 17,
	UART3en
}RCC_APB1_bit;
void Clock_init(RCC_type_t RCC_type);
void AHB1_clk_setup(RCC_AHB1_bit AHB1_bit);
void APB1_clk_setup(RCC_APB1_bit APB1_bit);

#endif /* RCC_LIB_CUSTOM_RCC_LIB_CUSTOM_H_ */
