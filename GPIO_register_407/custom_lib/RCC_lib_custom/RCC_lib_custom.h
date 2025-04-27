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
	HSE,HSI
}RCC_type_t;

void Clock_init(RCC_type_t RCC_type);
void AHB1_clk_setup();

#endif /* RCC_LIB_CUSTOM_RCC_LIB_CUSTOM_H_ */
