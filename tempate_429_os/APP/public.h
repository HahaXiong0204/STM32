#ifndef _PUBLIC_H
#define _PUBLIC_H

#include "rtthread.h"
#include "main.h"



#define Device_ID   2002
extern uint16_t  ID_Cart;
extern uint8_t i;

// ============================ 主题 =========================
extern char Publish_Topc[64];
extern char Submit_Topc[64];
extern char Publish_Topc_ID[64];   // 给id的用户准备的
extern char Submit_Topc_ID[64];
extern const char *Publish_Buf[];  // 发布的主题
extern const char *topics[];      // 订阅的主题
extern const char *topics_id[];
// ==============================================================


// =========================== wifi ==========================
extern uint8_t cmd_flag;    // 命令打印标志位
extern uint8_t net_connect;            // 网咯是否连接的标志位
extern uint8_t rx_buff[1024];   // 接收数组
extern uint16_t rx_cnt;		// 接收数组的个数
extern uint8_t again_num;   // 重发数据的条数
extern uint8_t again_mode;   // 重发标志问
extern uint8_t at_th_size;   // 阈值的数据大小
extern char at_th_buff[64];  // 阈值的数据

extern uint8_t user_mode;   // 0 自动  1 手动
extern uint8_t state; 		   // 是否复位
// ============================================================

// ========================== 传感器数据 ==========================
#define ADC_MAX 	40
extern uint32_t adc_value[ADC_MAX];			//adc数据缓冲区
extern double temperature;               //温度
extern double humidity;                  //湿度
extern uint32_t Lighting,Turbidity,Gas;   //光照，浊度，烟雾的数据 
extern double voltage;    // 电压
extern uint16_t speed;     // 电机速度
extern uint8_t people;   // 人体传感器
extern uint8_t Led_position;         // led的档位 
extern uint8_t machinery_position;    // 电机档位

extern uint8_t tem_flag;     // 温度标志位
extern uint8_t Hum_flag;	  // 湿度标志物
extern uint8_t Gas_flag;	  // 空气标志位
extern uint8_t light_flag;    // 光照标志位

// ================================================================


// ============================= 传感器的阈值 ====================
extern int TH_Temp;   //  温度阈值
extern int TH_Hum;   // 湿度阈值
extern int TH_Gas;   // 空气阈值
extern int TH_light;   // 光照强度阈值
// ==============================================================


// ============================ user start ==============================
extern uint8_t beep_flag;    // 1 warning1   2 Error1
extern  uint8_t user_uing;    // 用户使用标志位
// ============================ user end ==============================



#endif
