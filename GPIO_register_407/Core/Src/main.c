/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"


#include "gpio_lib_custom.h"
#include "UART_lib_custom.h"
#include "RCC_lib_custom.h"
#define GPIOD_base_adress 0x40020C00
#define SYSCFG_base_adress 0x40013800
#define NVIC_base_adress 0xE000E100
#define EXTI_base_adress 0x40013C00


char data[128];
void DMA1_str5_interrupt();
int cout = 0;


void EXTI_init()
{
	uint32_t* SYSCFG_EXTICR1 = (uint32_t*) (SYSCFG_base_adress + 0x08);
	*SYSCFG_EXTICR1 &= ~(0b1111<<0);

	uint32_t* NVIC_ISER0 = (uint32_t*) (NVIC_base_adress + 0x00);

	*NVIC_ISER0 |= (1<<6);

	uint32_t* EXTI_IMR = (uint32_t*) (EXTI_base_adress + 0x00);
	*EXTI_IMR |= (0x01<<0);

	uint32_t* EXTI_FTSR =(uint32_t*) (EXTI_base_adress + 0x0c);
	*EXTI_FTSR |= (0x01<<0);

}



//a littler change


//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);

void EXTI0_ctHandler()
{
	toggle_led(led_2);
	toggle_led(led_3);
	toggle_led(led_4);

	uint32_t* EXTI_PR =(uint32_t*) (EXTI_base_adress + 0x14);
	*EXTI_PR |= (0x01<<0);
}

void DMA_init()
{
	__HAL_RCC_DMA1_CLK_ENABLE();

	uint32_t* DMA1_S5NDTR = (uint32_t*)(0x4002608c);
	*DMA1_S5NDTR = 10;
	uint32_t* DMA1_S5PAR = (uint32_t*)0x40026090;
	//uint32_t* USART2_DR = (uint32_t*)(USART2_base_adress + 0x04);
	*DMA1_S5PAR = 0x40004404;
	uint32_t* DMA1_S5M0AR = (uint32_t*)(0x40026094);
	*DMA1_S5M0AR = (uint32_t)data;
	uint32_t* DMA1_S5CR = (uint32_t*)(0x40026088);
	*DMA1_S5CR &= ~(0b111<<25);

	*DMA1_S5CR &= ~(0b1<<9);
	*DMA1_S5CR |= (0b100<<25 | 1<<10 |1<<0|1<<4);
	uint32_t* NVIC_ISER0 = (uint32_t*) 0xE000E100;
	*NVIC_ISER0 |= 1<<16;
}
void DMA1_str5_interrupt()
{
	toggle_led(led_3);
//	DMA_init();
	uint32_t* DMA1_HISR = (uint32_t*)(DMA1_BASE + 0x04);
	*DMA1_HISR |= (1<<11);
}

void TIM1_init()
{
	__HAL_RCC_TIM1_CLK_ENABLE();
	uint32_t* TIM1_CR1 = (uint32_t*)(TIM1_BASE);
	*TIM1_CR1 |= 1;
	uint32_t* TIM1_DIER = (uint32_t*)(TIM1_BASE + 0x0c);
	*TIM1_DIER |= 1;
	uint32_t* TIM1_PSC = (uint32_t*)(TIM1_BASE + 0x28);
	*TIM1_PSC = 16000;

	uint32_t* TIM1_ARR = (uint32_t*)(TIM1_BASE + 0x2C);
	*TIM1_ARR = 2000;

	uint32_t* NVIC_ISER0 = (uint32_t*) 0xE000E100;
	*NVIC_ISER0 |= 1<<25;
}

void TIM1_UP_TIM10_IRQHandler()
{

	toggle_led(led_4);
	uint32_t* TIM1_SR = (uint32_t*)(TIM1_BASE + 0x10);
	*TIM1_SR &= ~(1<<0);
}


void TIM4_init()
{
	__HAL_RCC_TIM4_CLK_ENABLE();
	uint32_t* TIM4_CR1 = (uint32_t*)(TIM4_BASE);
	*TIM4_CR1 |= 1;
//	uint32_t* TIM4_DIER = (uint32_t*)(TIM4_BASE + 0x0c);
//	*TIM4_DIER |= 1;
	uint32_t* TIM4_PSC = (uint32_t*)(TIM4_BASE + 0x28);
	*TIM4_PSC = 160-1;

	uint32_t* TIM4_ARR = (uint32_t*)(TIM4_BASE + 0x2C);
	*TIM4_ARR = 1000-1;
	uint32_t* TIM4_CCR1 = (uint32_t*)(TIM4_BASE + 0x34);
	*TIM4_CCR1 = 300;
	uint32_t* TIM4_CCMR1 = (uint32_t*)(TIM4_BASE + 0x18);
	*TIM4_CCMR1 &= ~(0x11);
	*TIM4_CCMR1 |= (0b110<<4);

	uint32_t* TIM4_CCER = (uint32_t*)(TIM4_BASE + 0x20);
	*TIM4_CCER |= 1;
	uint32_t* GPIOD_AFRH = (uint32_t*)(GPIOD_base_adress + 0x24);
	*GPIOD_AFRH |= (0b0010<<16);


//	uint32_t* NVIC_ISER0 = (uint32_t*) 0xE000E100;
//	*NVIC_ISER0 |= 1<<25;
}


int main(void)
{


	HAL_Init();

	Clock_init(HSI);
	AHB1_clk_setup(GPIOAen);
	AHB1_clk_setup(GPIODen);
	APB1_clk_setup(UART2en);
//  /* USER CODE BEGIN Init */
//	__HAL_RCC_GPIOD_CLK_ENABLE();

    led_init();

  USART2_init();

  while (1)
  {
	 led_control(led_2, 1);
	 HAL_Delay(100);

	 led_control(led_2, 0);
	 HAL_Delay(100);
  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_RCC_PWR_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
//static void MX_GPIO_Init(void)
//{
//  /* USER CODE BEGIN MX_GPIO_Init_1 */
//
//  /* USER CODE END MX_GPIO_Init_1 */
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//
//  /* USER CODE BEGIN MX_GPIO_Init_2 */
//
//  /* USER CODE END MX_GPIO_Init_2 */
//}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
