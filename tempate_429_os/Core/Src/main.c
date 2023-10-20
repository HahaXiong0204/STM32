/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

rt_thread_t sensor_thread_id;   // 读取传感器的线程
rt_thread_t logical_thread_id;   // 逻辑线程
rt_thread_t showlcd_thread_id;   // 显示线程
rt_mutex_t mutex1;    			// 线程锁
rt_thread_t Rfid_thread_id;    // 读rfid
rt_thread_t key_thread_id;    // 按键检测
rt_thread_t ping_thread_id;   // 保活线程

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uint16_t timeCount = 0;
    unsigned char *dataPtr = NULL;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_FMC_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_SPI5_Init();
  /* USER CODE BEGIN 2 */

	Mx_Init();
	
	mutex1 = rt_mutex_create("mutex1",RT_IPC_FLAG_PRIO);
	
	// 显示线程
	showlcd_thread_id = rt_thread_create("showlcd_thread",lcd_show,NULL,1024,11,50);
	if(showlcd_thread_id!=RT_NULL){
		rt_thread_startup(showlcd_thread_id);
		printf("线程 showlcd_thread_id 创建成功\r\n");
	}
	
	// 传感器线程
	sensor_thread_id = rt_thread_create("sensor_thread",collect_data,NULL,512,11,30);
	if(sensor_thread_id!=RT_NULL){
		rt_thread_startup(sensor_thread_id);
		printf("线程 sensor_thread_id 创建成功\r\n");
	}
	
	// 读rfid
	Rfid_thread_id = rt_thread_create("rfid_thread",read_rfid,NULL,1024,12,30);
	if(Rfid_thread_id!=RT_NULL){
	rt_thread_startup(Rfid_thread_id);
	printf("线程 Rfid_thread_id 创建成功\r\n");
	}
	
	// 逻辑线程
	logical_thread_id = rt_thread_create("logical_thread",Logical_code_entry,NULL,512,11,20);
	if(logical_thread_id!=RT_NULL){
		rt_thread_startup(logical_thread_id);	
		printf("线程 logical_thread_id 创建成功\r\n");
	}
	
	// 按键的扫描线程
	key_thread_id = rt_thread_create("key_thread",key_thread_entry,RT_NULL,512,12,30);
	if(key_thread_id!=RT_NULL){
		rt_thread_startup(key_thread_id);
		printf("线程 key_thread_id 创建成功\r\n");
	}
	
	// 保活线程
	ping_thread_id = rt_thread_create("ping_thread",ping_thread_entry,RT_NULL,512,2,20);
	if(key_thread_id!=RT_NULL){
		rt_thread_startup(ping_thread_id);
		printf("线程 ping_thread_id 创建成功\r\n");
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(net_connect == 1)
	  {
		  
            ++timeCount;
			

		  
            if(timeCount >= 200)		//时间间隔5s 发送
            {

				// 判断是否有时间同步
				if(time_flag==0)
				{
					OneNet_Send_Device_ID_Data();
				}
//              rt_thread_mdelay(100);
                printf( "开始发送数据\r\n");//通过串口1发送提示信息（要开始发送数据了）
                OneNet_SendData(0);//发送数据给onenet
                printf("发送数据结束\n");
                //				while(OneNet_DevLink())			//接入OneNET
                timeCount = 0;
                ESP8266_Clear();
            }
            dataPtr = ESP8266_GetIPD(0);//获取平台返回的数据
            if(dataPtr != NULL)//如果返回数据不为空
			{
                OneNet_RevPro(dataPtr);//平台返回数据检测
//				timeCount = 0;
				
			}	
            rt_thread_mdelay(40);
	  }
	  else if(net_connect == 2||net_connect==3)
	  {
		  rt_thread_mdelay(200);
		  if(net_connect==2)
		  {
			resest_wifi();
		  }
		  if(net_connect==3)
		  {
			resest_server();
		  }
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  else
	  {
			printf("开始连接服务器\r\n");
            ESP8266_Init();					//初始化ESP8266

            printf("8266初始化成功\n");


            while(OneNet_DevLink())			//接入OneNET
			
			
			rt_thread_mdelay(200);
            net_connect = 1;
//            HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value, 40);

            printf("接入onenet成功");
	  
	  
	  
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
