/*
 * UART_lib_custom.h
 *
 *  Created on: Apr 27, 2025
 *      Author: deffen
 */

#ifndef UART_LIB_CUSTOM_UART_LIB_CUSTOM_H_
#define UART_LIB_CUSTOM_UART_LIB_CUSTOM_H_


#include "stdint.h"

#define USART2_base_adress 0x40004400
#define GPIOA_base_adress 0x40020000



void USART2_init();
void send_byte(char data);
void send_data(char* data, int data_leght);
char recv_byte();

#endif /* UART_LIB_CUSTOM_UART_LIB_CUSTOM_H_ */
