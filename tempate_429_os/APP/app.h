#ifndef _APP_H
#define _APP_H

#include "main.h"
#include "public.h"
#include "wifi.h"
#include "onenet.h"
#include "dht11.h"
#include "delay.h"
#include "24c02.h"
#include "PCF8574.h"
#include "sdram.h"
#include "rc522.h"
#include "malloc.h"
#include "key.h"
#include "fontupd.h"
#include "w25qxx.h"
#include "text.h"



void Mx_Init(void);  // 初始化数据
void ping_thread_entry(void *pargem);
void lcd_show(void *pargem); // 显示数据在lcd屏幕上
void collect_data(void *pargem);   // 采集传感器数据

void Logical_code_entry(void *pargem);   // 逻辑判断
void at_sava_th(void);    // 保存阈值数据
void read_rfid(void *pargem);   // 读rfid

void resest_wifi(void);      // 断网重连wifi
void resest_server(void);   // 断网重连服务器


// 按键处理线程
void key_thread_entry(void *pargem);


// 设置LED档位
void Led_pwm_mode(uint8_t mode);
// 设置电机档位
void machinery_pwm_mode(uint8_t mode);




#endif
