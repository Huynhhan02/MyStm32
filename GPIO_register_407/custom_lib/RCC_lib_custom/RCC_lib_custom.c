/*
 * RCC_lib_custom.c
 *
 *  Created on: Apr 27, 2025
 *      Author: deffen
 */

#include "RCC_lib_custom.h"


void Clock_init(RCC_type_t RCC_type)
{
	//set power mode
	uint32_t* PWR_CR = (uint32_t*)(PWR_base_adress + 0x00);
	*PWR_CR |= (0x01<<14);

	// __HAL_RCC_PWR_CLK_ENABLE();
	 uint32_t* RCC_AHB1ENR = (uint32_t*)(RCC_base_adress + 0x30);
	 *RCC_AHB1ENR |= (0x01<<28);
	//set HSI clock
	uint32_t* RCC_CR = (uint32_t*)(RCC_base_adress + 0x00);
	*RCC_CR |= 0x01;
	//set RCC_CFGR
	uint32_t* RCC_CFGR = (uint32_t*)(RCC_base_adress + 0x08);
//	*RCC_CFGR |= (0b111<13);
//	*RCC_CFGR |= (0b111<10);
	*RCC_CFGR &= ~(0b11<<0);

	//set interrupt by HSI
	uint32_t* RCC_CIR = (uint32_t)(RCC_base_adress + 0x0c);
	*RCC_CIR |= (0b01<<10);
//	while((*RCC_CIR>>2 & 0x01) != 1);
}

void AHB1_clk_setup()
{
	uint32_t* RCC_AHB1 = (uint32_t*)(RCC_base_adress + 0x30);
	*RCC_AHB1 |= (0x01<<3); // GPIOD_clk_enable
}


