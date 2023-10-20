#include "public.h"


uint8_t i;

// =========================== 系统 ===============================================
char time_buff[56];

// ==============================================================================


// ======================= 线程定义 ===============================================
QueueHandle_t hsend_queue;    // 发送的队列
char *send_buffer = NULL;    // 发送到缓冲区的buff



QueueHandle_t hreve_queue;    // 接收的队列
BaseType_t hreve_queue_base = pdFALSE;   // 接收的宏


TaskHandle_t  send_data_id;    // 发送数据线程
TaskHandle_t  reve_data_id;    // 接收数据线程

TaskHandle_t  show_lcd_id;   	// 屏幕显示
TaskHandle_t  collect_data_id;   // 数据采集线程
TaskHandle_t  read_rfid_id;   // 读取rfid线程
TaskHandle_t  Logical_code_id;   // 逻辑判断线程
TaskHandle_t  key_scan_id;   // 按键扫描线程


SemaphoreHandle_t send_mutex;   // 发送锁
SemaphoreHandle_t reve_mutex;   // 接收锁
SemaphoreHandle_t iic_spi_mutex;   // 保护iic和spi的锁

SemaphoreHandle_t send_sem = NULL;

// ===============================================================================

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

uint8_t reconnect_num = 0;    // 重连次数
uint8_t reconnect_one_num = 0;   // 单次重连次数
// ============================================================




// ========================== 传感器数据 start ==========================
uint32_t adc_value[ADC_MAX];              //adc数据缓冲区
double temperature = 0;             //温度
double temperature_ds18 = 0;             //温度
double humidity = 0;                //湿度
uint32_t Lighting = 0, Turbidity = 0, Gas = 0; //光照，浊度
double voltage = 0;  // 电压
uint16_t speed = 0;     // 电机速度
uint8_t people = 0;   // 人体传感器  0无人  1有人
uint8_t red_people = 0;   // 人体传感器  0无人  1有人
uint8_t Led_position = 0;         // led的档位
uint8_t machinery_position = 0;    // 电机档位  
uint8_t approch_switch = 0;  // 接近开关 1 没有铁，0，有铁


uint8_t tem_flag = 0;     // 温度标志位
uint8_t Hum_flag = 0;	  // 湿度标志物
uint8_t Gas_flag = 0;	  // 空气标志位
uint8_t light_flag = 0;    // 光照标志位
uint8_t fire_flag = 0;   // 火焰标志位
uint8_t approch_flag = 0;    // 接近开关的标志位
// ===========================传感器数据 end =====================================




// ============================= 传感器的阈值 start====================
int TH_Temp = 48;   //  温度阈值
int TH_Hum = 60;   // 湿度阈值
int TH_Gas = 100;   // 空气阈值
int TH_light = 30;   // 光照强度阈值
// ==============================传感器的阈值 end================================


// ============================ user start ==============================
uint8_t beep_flag = 0;    // 1 warning1   2 Error1

float Total_amount = 0.0;   // 总金额

// ============================ user end ==============================




