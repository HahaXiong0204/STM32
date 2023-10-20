#ifndef _KEY_H
#define _KEY_H
#include "sys.h"
#include "main.h"
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F429开发板
//KEY驱动代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/1/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////

//下面的方式是通过位带操作方式读取IO
//#define KEY0        PHin(3) //KEY0按键PH3
//#define KEY1        PHin(2) //KEY1按键PH2
//#define KEY2        PCin(13)//KEY2按键PC13
//#define WK_UP       PAin(0) //WKUP按键PA0


//下面的方式是通过直   接操作HAL库函数方式读取IO
#define KEY0           		HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_3)  //KEY0按键PH3
#define KEY1           		HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_2)  //KEY1按键PH2
#define KEY2           		HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) //KEY2按键PC13
#define WK_UP          		HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)  //WKUP按键PA0
#define bom_red        		HAL_GPIO_ReadPin(bom_red_GPIO_Port,bom_red_Pin)  //WKUP按键PA0
#define bom_yellow          HAL_GPIO_ReadPin(bom_yellow_GPIO_Port,bom_yellow_Pin)  //WKUP按键PA0
#define bom_green           HAL_GPIO_ReadPin(bom_green_GPIO_Port,bom_green_Pin)  //WKUP按键PA0

#define KEY0_PRES 	           1
#define KEY1_PRES	           2
#define KEY2_PRES	           3
#define WKUP_PRES              4
#define RED_PRES               5
#define YELLOW_PRES            6
#define GRREN_PRES             7


u8 KEY_Scan(uint8_t mode);
#endif
