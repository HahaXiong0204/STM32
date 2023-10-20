#include "key_exit.h"
#include "PCF8574.h"
#include "delay.h"
#include "tim.h"


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	delay_ms(100);
	printf("exit\r\n");
	// WK_UP
	if(GPIO_Pin == WK_UP_Pin)    
    {
		if(HAL_GPIO_ReadPin(WK_UP_GPIO_Port,WK_UP_Pin)==1)
		{
			beep_flag = 0;
			printf("\r\nWK_UP\r\n");
		
		}

    }
	
	// key0
	if(GPIO_Pin == KEY0_Pin)
	{
		if(HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin)==0)
		{
			beep_flag = 1;
			HAL_TIM_Base_Start_IT(&htim2);
			printf("\r\nKEY0\r\n");
		
		}
	}
	
	// key1
    if(GPIO_Pin == KEY1_Pin)   // KEY2
    {
		if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==0)
		{
			beep_flag = 2;
			HAL_TIM_Base_Start_IT(&htim2);
			printf("\r\nKEY1\r\n");
		
		}

    }
	
	// key2
	if(GPIO_Pin == KEY2_Pin)
	{
		if(HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin)==0)
		{
			printf("\r\nKEY2\r\n");
		
		}
	}
	

	// bom_red
	if(GPIO_Pin == bom_red_Pin)
	{
		if(HAL_GPIO_ReadPin(bom_red_GPIO_Port,bom_red_Pin)==0)
		{
			printf("\r\n bom_red \r\n");
		
		}
	}
	
	// bom_yellow
	if(GPIO_Pin == bom_yellow_Pin)
	{
		if(HAL_GPIO_ReadPin(bom_yellow_GPIO_Port,bom_yellow_Pin)==0)
		{
			printf("\r\n bom_yellow \r\n");
		
		}
	}
	
	
	// bom_green
	if(GPIO_Pin == bom_green_Pin)
	{
		if(HAL_GPIO_ReadPin(bom_green_GPIO_Port,bom_green_Pin)==0)
		{
			printf("\r\n bom_green \r\n");
		
		}
	}
	

	
//	delay_ms(100);
}
