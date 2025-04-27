#include "UART_lib_custom.h"



void USART2_init()
{
	//__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_base_adress);
	*GPIOA_MODER &= ~(0xf<<4);
	*GPIOA_MODER |= (0b1010<<4);

	uint32_t* GPIOA_AFRL = (uint32_t*)(GPIOA_base_adress + 0x20);
	*GPIOA_AFRL &= ~(0xff<<8);
	*GPIOA_AFRL |= (((0b0111)<<8)|(0b0111<<12));
	//__HAL_RCC_USART2_CLK_ENABLE();
	//uint32_t* USART2_SR = (uint32_t*)(USART2_base_adress + 0x00);
	uint32_t* USART2_BRR = (uint32_t*)(USART2_base_adress + 0x08);
	*USART2_BRR = ((104<<4)|(3<<0));
	uint32_t* USART2_CR1 = (uint32_t*)(USART2_base_adress + 0x0c);
	*USART2_CR1 |= (0x01<<13)| (0x01<<3) | (0x01<<2);
	uint32_t* USART2_CR3 = (uint32_t*)(USART2_base_adress + 0x14);
	*USART2_CR3 |= 1<<6;


}


void send_byte(char data)
{
	uint32_t* USART2_SR = (uint32_t*)(USART2_base_adress + 0x00);
	uint32_t* USART2_DR = (uint32_t*)(USART2_base_adress + 0x04);

	while(((*USART2_SR>>7) &1) != 1);
	*USART2_DR = data;
	while(((*USART2_SR>>6) &1) != 0);
}
void send_data(char* data, int data_leght)
{
	for(int i =0; i< data_leght; i++)
	{
		send_byte(data[i]);
	}
}
char recv_byte()
{
	uint32_t* USART2_SR = (uint32_t*)(USART2_base_adress + 0x00);
	uint32_t* USART2_DR = (uint32_t*)(USART2_base_adress + 0x04);

	while(((*USART2_SR>>5)&1)==0);
	char data = * USART2_DR;
	return data;
}
