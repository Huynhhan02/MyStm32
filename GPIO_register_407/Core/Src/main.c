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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define GPIOA_base_adress 0x40020000
#define GPIOD_base_adress 0x40020C00
#define SYSCFG_base_adress 0x40013800
#define NVIC_base_adress 0xE000E100
#define EXTI_base_adress 0x40013C00
#define USART2_base_adress 0x40004400
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void led_init(){

	__HAL_RCC_GPIOD_CLK_ENABLE();
		uint32_t* GPIOD_MODER = (uint32_t*)(GPIOD_base_adress + 0x00);
		uint32_t* GPIOD_OTYPER = (uint32_t*)(GPIOD_base_adress + 0x04);

		*GPIOD_MODER &= ~(0b11111111<<24);
		*GPIOD_MODER |= (0x55<<24);
		*GPIOD_OTYPER &= ~(0b1111<<12);
}

void button_init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_base_adress);
	uint32_t* GPIOA_PUPDR = (uint32_t*)(GPIOA_base_adress + 0x0c);

	*GPIOA_MODER &= ~(0b11<<0);
	*GPIOA_PUPDR &= ~(0b11<<0);
}
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
typedef enum
{
	led_1 = 12, led_2 ,led_3, led_4
} GPIOA_led;
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
void USART2_init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_base_adress);
	*GPIOA_MODER &= ~(0xf<<4);
	*GPIOA_MODER |= (0b1010<<4);

	uint32_t* GPIOA_AFRL = (uint32_t*)(GPIOA_base_adress + 0x20);
	*GPIOA_AFRL &= ~(0xff<<8);
	*GPIOA_AFRL |= (((0b0111)<<8)|(0b0111<<12));
	__HAL_RCC_USART2_CLK_ENABLE();
	//uint32_t* USART2_SR = (uint32_t*)(USART2_base_adress + 0x00);
	uint32_t* USART2_BRR = (uint32_t*)(USART2_base_adress + 0x08);
	*USART2_BRR = ((104<<4)|(3<<0));
	uint32_t* USART2_CR1 = (uint32_t*)(USART2_base_adress + 0x0c);
	*USART2_CR1 |= (0x01<<13)| (0x01<<3) | (0x01<<2);


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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void EXTI0_ctHandler()
{
	toggle_led(led_2);
	toggle_led(led_3);
	toggle_led(led_4);

	uint32_t* EXTI_PR =(uint32_t*) (EXTI_base_adress + 0x14);
	*EXTI_PR |= (0x01<<0);
}
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
	  uint32_t* EXTI0_registor = (uint32_t*) 0x20000058;
	  *EXTI0_registor = (uint32_t)EXTI0_ctHandler|1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  led_init();
  USART2_init();
  //button_init();
  EXTI_init();
  vector_table_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  char msg[] = "im a stm32\r\n";
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  toggle_led(led_1);
	 send_data(msg, sizeof(msg));
	  HAL_Delay(2000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

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
