/*
 * vector_table_custom.c
 *
 *  Created on: Apr 28, 2025
 *      Author: User
 */
#include "vector_table_custom.h"

void vector_table_init()
{
	 char* vectortable_base = 0;
	 char* newVectortable = (char*) 0x20000000;
	  for(int i = 0;i<0x188;i++)
	  {
		  newVectortable[i] = vectortable_base[i];
	  }
	  uint32_t* VectorTableOffset = (uint32_t*) 0xE000ED08;
	  *VectorTableOffset = 0x20000000;
//	  uint32_t* EXTI0_registor = (uint32_t*) 0x20000058;
//	  *EXTI0_registor = (uint32_t)EXTI0_ctHandler|1;
//	  uint32_t* DMA1_str5_registor = (uint32_t*) 0x20000080;
//	  *DMA1_str5_registor = (uint32_t)DMA1_str5_interrupt|1;

}
