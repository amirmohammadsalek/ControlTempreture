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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define FLASH_PAGE_ADDR 0x0800F800
void send_Idata(uint8_t data)
{
	HAL_Delay(10);
	int temp[4];

	for (int i=0; i<4;i++)
	{

		temp[i]=data%2;
		data=data>>1;

	}



	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,temp[0]);  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,temp[1]);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,temp[2]);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,temp[3]);

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);

}
void send_command(uint8_t data)
{
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);


	send_Idata(data >>4);
	send_Idata(data);
}
void send_data(uint8_t data)
{
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);


	send_Idata(data >>4);

	send_Idata(data);


}
void lcd_init(void)
{
  HAL_Delay(2);
  send_command(0x02);
  send_command(0x28);
  send_command(0x0F);
}
void lcd_puts(uint8_t *str)
{
  HAL_Delay(2);
  while(*str !=0x00)
  {
	  send_data(*str);
    str++;
  }
}

void lcd_clear(void)
{
  HAL_Delay(1);
  send_command(0x01);

}
void lcd_gotoxy(uint8_t x, uint8_t y)
{
  HAL_Delay(1);

    switch(y){
    case 0:
      send_command( 0x80 + x );
    break;
    case 1:
      send_command( 0xC0 + x );
      break;

  }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int test = 0;
uint8_t Address = 0x38;
uint8_t reset_command = 0xBA;
uint8_t init_command [3] = {0xE1,0x08,0x00};
HAL_StatusTypeDef status;
uint8_t buff1 [32];
float temp = 50;
uint8_t min = 0;
uint8_t max = 60;
uint8_t send = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
        if (strncmp((uint8_t *)buff1, "Report ",7) == 0) {
					sprintf(buff1,"Temp is : %.1f \r\n",temp);
					HAL_Delay(1000);
					HAL_UART_Transmit(&huart1, (uint8_t *)buff1, sizeof(buff1), HAL_MAX_DELAY);
        }
				HAL_UART_Receive_IT(&huart1, buff1, 10);
}
float read_temp() {
			float temp;
			uint8_t measure_command [3] = {0xAC,0x33,0x00};
			uint8_t data [7] = {0};
			HAL_I2C_Master_Transmit(&hi2c1, 0x38<<1 , measure_command, 3, HAL_MAX_DELAY);
			HAL_Delay(80);
			HAL_I2C_Master_Receive(&hi2c1, 0x38<<1, data, 7, HAL_MAX_DELAY);
			temp = ((float)((((uint32_t)data[3]<<16)+((uint16_t)data[2]<<8)+(data[1]))&(0x0FFFFF))/1048576*200)-50;
			return temp;
}
void setting () {
	int mode = 0;
	lcd_clear();
	sprintf(buff1,"Max : %d",max);
	lcd_gotoxy(0,0);
	lcd_puts((uint8_t *)buff1);
	sprintf(buff1,"Min : %d",min);
	lcd_gotoxy(0,1);
	lcd_puts((uint8_t *)buff1);
	HAL_Delay(1000);
	while (mode == 0) {
		if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12) == 0) {
			mode = 1;
		}
		if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14) == 0) { //changing max
				lcd_clear();
				sprintf(buff1,"Max : %d",max);
				lcd_gotoxy(0,0);
				lcd_puts((uint8_t *)buff1);
				HAL_Delay(1000);
				while (1) {
					if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14) == 0) { //increase
						max+=1;
						lcd_clear();
						sprintf(buff1,"Max : %d",max);
						lcd_gotoxy(0,0);
						lcd_puts((uint8_t *)buff1);
						HAL_Delay(1000);
					}
					if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13) == 0) {	//Deacrease
						max-=1;
						lcd_clear();
						sprintf(buff1,"Max : %d",max);
						lcd_gotoxy(0,0);
						lcd_puts((uint8_t *)buff1);
						HAL_Delay(1000);
					}
					if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15) == 0) {	//ok
						lcd_clear();
						lcd_gotoxy(0,0);
						lcd_puts((uint8_t *) "Changed" );
						HAL_Delay(1000);
						mode = 1;
						break;
					}
				}
		}
		if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13) == 0) { //changing min
				lcd_clear();
				sprintf(buff1,"Min : %d",min);
				lcd_gotoxy(0,0);
				lcd_puts((uint8_t *)buff1);
				HAL_Delay(1000);
				while (1) {
					if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14) == 0) { //increase
						min+=1;
						lcd_clear();
						sprintf(buff1,"Min : %d",min);
						lcd_gotoxy(0,0);
						lcd_puts((uint8_t *)buff1);
						HAL_Delay(1000);
					}
					if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13) == 0) {	//Deacrease
						min-=1;
						lcd_clear();
						sprintf(buff1,"Min : %d",min);
						lcd_gotoxy(0,0);
						lcd_puts((uint8_t *)buff1);
						HAL_Delay(1000);
					}
					if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15) == 0) {	//ok
						lcd_clear();
						lcd_gotoxy(0,0);
						lcd_puts((uint8_t *) "Changed" );
						HAL_Delay(1000);
						mode = 1;
						break;
					}
				}
			}
	}
}
void check_temp(float temp,uint8_t min,uint8_t max) {
				
				if (temp > min && temp < max) {
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
					lcd_clear();
					lcd_gotoxy(0,0);
					sprintf(buff1,"Temp is : %.1f",temp);
					lcd_puts((uint8_t *) buff1 );
					HAL_Delay(100);
					send = 0;
				}
				else if (temp<min){
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
					lcd_clear();
					lcd_gotoxy(0,0);
					lcd_puts((uint8_t *) "Temp is too low" );
					lcd_gotoxy(0,1);
					sprintf(buff1,"Temp is : %.1f",temp);
					lcd_puts((uint8_t *) buff1);
					if (send == 0) {
						sprintf(buff1,"WARNING!!!\r\nTemp is too low\r\n");
						HAL_UART_Transmit(&huart1, (uint8_t *)buff1, strlen(buff1), HAL_MAX_DELAY);
						send = 1;
					}
					HAL_Delay(100);
				}
				else {
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
					lcd_clear();
					lcd_gotoxy(0,0);
					lcd_puts((uint8_t *) "Temp is too high" );
					lcd_gotoxy(0,1);
					sprintf(buff1,"Temp is : %.1f",temp);
					lcd_puts((uint8_t *) buff1);
					if (send == 0) {
						sprintf(buff1,"WARNING!!!\r\nTemp is too high\r\n");
						HAL_UART_Transmit(&huart1, (uint8_t *)buff1, strlen(buff1), HAL_MAX_DELAY);
						send = 1;
					}
					HAL_Delay(100);
				}
}
void Flash_Write(uint8_t Max, uint8_t Min) {

    HAL_FLASH_Unlock();


    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_PAGE_ADDR;
    EraseInitStruct.NbPages = 1;
    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);


    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_PAGE_ADDR, Max);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_PAGE_ADDR + 2, Min);


    HAL_FLASH_Lock();
}
void Flash_Read(uint8_t *Max, uint8_t *Min) {
    *Max = *(uint8_t *)FLASH_PAGE_ADDR;       
    *Min = *(uint8_t *)(FLASH_PAGE_ADDR + 2); 
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	lcd_init();
	HAL_I2C_Master_Transmit(&hi2c1, Address<<1 , init_command, 3, HAL_MAX_DELAY); //Sensor Init
	HAL_Delay(10);
	Flash_Read(&max, &min);
	HAL_UART_Receive_IT(&huart1, buff1, 10);	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			temp = read_temp();
			check_temp(temp,min,max);
			if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12) == 0) {
				HAL_Delay(100);
				setting();
				Flash_Write(max, min);
			}
			
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
