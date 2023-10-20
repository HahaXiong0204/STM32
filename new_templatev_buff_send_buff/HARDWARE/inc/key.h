#ifndef _KEY_H
#define _KEY_H
#include "sys.h"
#include "main.h"
//////////////////////////////////////////////////////////////////////////////////
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEK STM32F429������
//KEY��������
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/1/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////

//����ķ�ʽ��ͨ��λ��������ʽ��ȡIO
//#define KEY0        PHin(3) //KEY0����PH3
//#define KEY1        PHin(2) //KEY1����PH2
//#define KEY2        PCin(13)//KEY2����PC13
//#define WK_UP       PAin(0) //WKUP����PA0


//����ķ�ʽ��ͨ��ֱ   �Ӳ���HAL�⺯����ʽ��ȡIO
#define KEY0           		HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_3)  //KEY0����PH3
#define KEY1           		HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_2)  //KEY1����PH2
#define KEY2           		HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) //KEY2����PC13
#define WK_UP          		HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)  //WKUP����PA0
#define bom_red        		HAL_GPIO_ReadPin(bom_red_GPIO_Port,bom_red_Pin)  //WKUP����PA0
#define bom_yellow          HAL_GPIO_ReadPin(bom_yellow_GPIO_Port,bom_yellow_Pin)  //WKUP����PA0
#define bom_green           HAL_GPIO_ReadPin(bom_green_GPIO_Port,bom_green_Pin)  //WKUP����PA0

#define KEY0_PRES 	           1
#define KEY1_PRES	           2
#define KEY2_PRES	           3
#define WKUP_PRES              4
#define RED_PRES               5
#define YELLOW_PRES            6
#define GRREN_PRES             7


u8 KEY_Scan(uint8_t mode);
#endif