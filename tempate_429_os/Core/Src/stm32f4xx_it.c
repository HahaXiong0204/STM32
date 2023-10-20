/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim14;

/* USER CODE BEGIN EV */
uint8_t beep_num = 0;
uint8_t beep_people = 0;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
	HAL_GPIO_TogglePin(led0_pwm_GPIO_Port,led0_pwm_Pin);
	if(time_flag)
	{
		time_set();
	}
	
	if(beep_flag==1)
	{
		printf("================================\r\n");
		beep_people++;
		if(beep_people==1)
		{
//			OneNet_Send_Int("beep_flag",1);
		}
		if(beep_people>4)
		{
//			OneNet_Send_Int("beep_flag",0);
			beep_On_Off(1);
			beep_flag = 0;
			beep_people = 0;
		}
	}
	
  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	
		
	if((tem_flag == 0)&&(Hum_flag ==0)&&(Gas_flag==0))
	{
		if(beep_num)
		{
			PCF8574_WriteBit(BEEP_IO,1);   // 关闭
			HAL_TIM_Base_Stop_IT(&htim2);
		}
		beep_num = 0;
	}
	
	// 响0.1 秒  不响 0.9
	if(tem_flag == 1)
	{
		
		if(beep_num == 1)
			PCF8574_WriteBit(BEEP_IO,0);
		else if(beep_num == 2)
			PCF8574_WriteBit(BEEP_IO,1);
		beep_num++;
		if(beep_num>=10)
		{
			beep_num = 1;
		}
		
	}
	
	// 响 0.9
	if(Hum_flag == 1)
	{
		
		if(beep_num == 1)
			PCF8574_WriteBit(BEEP_IO,0);
		else if(beep_num == 9)
			PCF8574_WriteBit(BEEP_IO,1);
		beep_num++;
		if(beep_num>=10)
		{
			beep_num = 1;
		}
		
	}
	
	// 响 0.5
	if(Gas_flag == 1)
	{
		
		if(beep_num == 1)
			PCF8574_WriteBit(BEEP_IO,0);
		else if(beep_num == 5)
			PCF8574_WriteBit(BEEP_IO,1);
		beep_num++;
		if(beep_num>=10)
		{
			beep_num = 1;
		}
		
	}
	
	
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
	if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET)   // 空闲中断的标志位
    {
        HAL_UART_DMAStop(&huart3);                               //停止接收
        rx_cnt = MAX_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);    // 计算接收的数据长度
//        printf("%s", rx_buff);
		if(net_connect == 1)
        {
            if((strstr((const char *)rx_buff, "link is not valid") != NULL))		//如果检索到关键词
            {
                ESP8266_Clear();									//清空缓存
                printf("网络断开，准备重连\r\n");
                net_connect = 2;
            }

        }
		else{
				
			//"\r\nERROR\r\nCLOSED\r\n"
			if((strstr((const char *)rx_buff,"ERROR")!=NULL))
			{
				if((strstr((const char *)rx_buff,"CLOSED")!=NULL))
				{
					printf("连接服务器失败\r\n");
					ESP8266_Clear();									//清空缓存
					net_connect = 3;
				}
			
			}
		
		}
        HAL_UART_Receive_DMA(&huart3, rx_buff, 256);       // 开启DMA继续接收
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    }

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
