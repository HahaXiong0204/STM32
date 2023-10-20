#ifndef DHT11_H
#define DHT11_H
#include "main.h"
#include "sys.h"
#include "delay.h"

//IO方向设置
//IO方向设置
#define DHT11_IO_IN()  {GPIOG->MODER &= ~(GPIO_MODER_MODER14_0);GPIOG->MODER &= ~(GPIO_MODER_MODER14_1);} // 设置为输出模式
// PC10 IN  MODE
#define DHT11_IO_OUT() {GPIOG->MODER &= ~(GPIO_MODER_MODER14); GPIOG->MODER |= GPIO_MODER_MODER14_0;} // PC10 OUT MODE
////IO操作函数
#define	DHT11_DQ_OUT PGout(14) //数据端口	PB0
#define	DHT11_DQ_IN  PGin(14)  //数据端口	PB0


uint8_t DHT11_Init(void);//初始化DHT11
uint8_t DHT11_Read_Data(double *temp, double *humi); //读取温湿度
uint8_t DHT11_Read_Byte(void);//读出一个字节
uint8_t DHT11_Read_Bit(void);//读出一个位
uint8_t DHT11_Check(void);//检测是否存在DHT11
void DHT11_Rst(void);//复位DHT11


void DHT11_Show_Data(void);
#endif

