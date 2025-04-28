/*
 * flash_lib_custom.h
 *
 *  Created on: Apr 28, 2025
 *      Author: User
 */

#ifndef FLASH_LIB_FLASH_LIB_CUSTOM_H_
#define FLASH_LIB_FLASH_LIB_CUSTOM_H_

#include "RCC_lib_custom.h"
#define FLASH_base_registor		0x40023c00

void flash_init();
void clock_init();

#endif /* FLASH_LIB_FLASH_LIB_CUSTOM_H_ */
