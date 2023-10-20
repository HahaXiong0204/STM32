#include "public.h"


uint8_t i;


// =========================== wifi ==========================
uint8_t cmd_flag = 0;    // 命令打印标志位
uint8_t net_connect = 0;            // 网咯是否连接的标志位
uint8_t rx_buff[1024];
uint16_t rx_cnt = 0;
uint8_t again_num = 0;

uint8_t again_mode = 0;   // 重发标志问
uint8_t state = 0;    // 是否复位
uint8_t user_uing = 0;    // 用户使用标志位
uint8_t user_mode = 0;   // 0 自动  1 手动
// ============================================================



// ========================== 传感器数据 start ==========================
uint32_t adc_value[ADC_MAX];              //adc数据缓冲区
double temperature=0;               //温度
double humidity=0;                  //湿度
uint32_t Lighting=0,Turbidity=0,Gas=0;   //光照，浊度
double voltage = 0;  // 电压
uint16_t speed = 0;     // 电机速度
uint8_t people = 0;   // 人体传感器
uint8_t Led_position = 0;         // led的档位 
uint8_t machinery_position = 0;    // 电机档位


uint8_t tem_flag = 0;     // 温度标志位
uint8_t Hum_flag = 0;	  // 湿度标志物
uint8_t Gas_flag = 0;	  // 空气标志位
uint8_t light_flag = 0;    // 光照标志位

// ===========================传感器数据 end =====================================




// ============================= 传感器的阈值 start====================
int TH_Temp = 48;   //  温度阈值
int TH_Hum = 60;   // 湿度阈值
int TH_Gas = 80;   // 空气阈值
int TH_light = 30;   // 光照强度阈值
// ==============================传感器的阈值 end================================


// ============================ user start ==============================
uint8_t beep_flag = 0;    // 1 warning1   2 Error1

// ============================ user end ==============================




