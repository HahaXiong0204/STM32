/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define XUNJIAN_FLAG    0    // 巡检标志位

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void mainTask_entry(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	Mx_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
	
	send_mutex = xSemaphoreCreateMutex();
	reve_mutex = xSemaphoreCreateMutex();
	
	iic_spi_mutex = xSemaphoreCreateMutex();   
	
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  hreve_queue = xQueueCreate(REVE_MAX_QUEUE,sizeof(char *));
  
  hsend_queue = xQueueCreate(SEND_MAX_QUEUE,sizeof(char *));

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(mainTask_entry, NULL, &mainTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

#if XUNJIAN_FLAG==0

/*==========================巡检不需要=============================================*/
#if RFID_FLAG
		xTaskCreate(read_rfid_thread_entry,"rfid_id",64,NULL,4,&read_rfid_id);	
		vTaskSuspend(read_rfid_id);
#endif
		
		
		/*  创建逻辑线程 */
		xTaskCreate(logical_thread_entry,"logical_id",128,NULL,9,&Logical_code_id);
		vTaskSuspend(Logical_code_id);
		
		
		/*  创建按键扫描线程 */
		xTaskCreate(key_thread_entry,"key_id",128,NULL,8,&key_scan_id);
		vTaskSuspend(key_scan_id);
		
#endif	
			/*  创建发送线程 */
		xTaskCreate(Send_data_Entry,"send_data_id",2048,NULL,9,&send_data_id);
			
		
		

		
		/*  创建采集数据线程 */
		xTaskCreate(collect_data_thread_entry,"collect_id",256,NULL,9,&collect_data_id);	
		vTaskSuspend(collect_data_id);
		
		
		
		/*  创建lcd显示线程 */
		xTaskCreate(lcd_show_thread_entry,"lcd_id",256,NULL,osPriorityNormal+1,&show_lcd_id);
		vTaskSuspend(show_lcd_id);
		
		


  
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_mainTask_entry */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_mainTask_entry */
void mainTask_entry(void *argument)
{
  /* USER CODE BEGIN mainTask_entry */
	 uint16_t timeCount = 0;
	char *receivedStr = NULL;    // 接收缓存区接收指针
	char ping_buff[64];
	memset(ping_buff,0,sizeof(ping_buff));
	sprintf(ping_buff,"{\"clientId\":\"%s\",\"Device_ID\":\"%s\"}",DEVID,PROID);
	
  /* Infinite loop */
  for(;;)
  {	  
	  if(net_connect == 1)
	  {
// ======================================== ================================================================================
			++timeCount;
			if((timeCount%470==0))
			{
#if IWDG_FLAG
				HAL_IWDG_Refresh(&hiwdg);               // 喂狗
#endif
			}
			
			if ((xQueueReceive(hreve_queue, &receivedStr, 0))) {
					// 处理接收到的字符串指针
					if(receivedStr != NULL)//如果返回数据不为空
					{
						Wait_reve_mutex();
						OneNet_RevPro((uint8_t*)receivedStr);//平台返回数据检测
						Release_reve_mutex();
					}	
					
			}
			osDelay(5);
		
	  }
	  else if(net_connect == 2||net_connect==3)
	  {
		  if(net_connect==2)
		  {
			resest_wifi();
		  }
		  if(net_connect==3)
		  {
			resest_server();
		  }
	  }
	  else
	  {
			printf("开始连接服务器\r\n");
			Show_Str(10, 760, 200, 32, "等待连接!!!", 32, 0);
            ESP8266_Init();					//初始化ESP8266

            printf("8266初始化成功\n");


            while(OneNet_DevLink())			//接入OneNET
			
			
			HAL_Delay(50);
			
            net_connect = 1;
//            HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value, 40);


			vTaskResume(show_lcd_id);
			vTaskResume(collect_data_id);
#if XUNJIAN_FLAG==0
#if RFID_FLAG
			vTaskResume(read_rfid_id);
#endif
			vTaskResume(Logical_code_id);
			vTaskResume(key_scan_id);
#endif
            printf("接入onenet成功");
			
			

	
	  
	  }
	  
	  
  }
  /* USER CODE END mainTask_entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

