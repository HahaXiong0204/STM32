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


 

extern char at_data_arr[256];
extern struct AT24CDATA at_data_struct;    // 掉电的结构体


void Mx_Init(void);  // 初始化数据

void Net_Connect_entery(void *pargem);   // 网络处理函数


void at_sava_th(void);    // 保存阈值数据

#if RFID_FLAG
void read_rfid(void *pargem);   // 读rfid
#endif

void resest_wifi(void);      // 断网重连wifi
void resest_server(void);   // 断网重连服务器


// 设置LED档位
void Led_pwm_mode(uint8_t mode);
// 设置电机档位
void machinery_pwm_mode(uint8_t mode);


// 发送到发送缓冲区
void Send_queue(char *msg);


/*  通过串口2 发送给zigbee */
void Send_Zigbee(void);
/*  通过串口6 发送给天问 */
void Send_tianwen(uint8_t mode);

// =============================  锁操作 ============================
/*等待发送锁*/
void Wait_send_mutex(void);
/*释放发送锁*/
void Release_send_mutex(void);
/*等待iic和spi锁*/
void Wait_iic_spi_mutex(void);
/*释放iic和spi锁*/
void Release_iic_spi_mutex(void);
/*等待接收锁*/
void Wait_reve_mutex(void);
/*释放接收锁*/
void Release_reve_mutex(void);
	
// ======================================================================	

#endif
