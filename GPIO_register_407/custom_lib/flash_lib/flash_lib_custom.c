/*
 * flash_lib_custom.c
 *
 *  Created on: Apr 28, 2025
 *      Author: User
 */


#include "flash_lib_custom.h"


void flash_init()
{
	uint32_t* FLASH_ACR = (uint32_t*)(FLASH_base_registor + 0x00);
	*FLASH_ACR |= ((1<<8) | (1<<9) | (1<<10));
}

void clock_init(){
	APB1_clk_setup(PWRen);
	APB2_clk_setup(SYSCFGen);
}
