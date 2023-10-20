#ifndef _PUBLIC_H
#define _PUBLIC_H
#include "cmsis_os.h"
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "queue.h"
#include "sys.h"
#include "wifi.h"
#include "string.h"
#include "tim.h"
#include "onenet.h"
#include "thread.h"
#include "hc-sr04.h"
#include "semphr.h"
#include "iwdg.h"
#include "cjson.h"
#include "duoji.h"
#include "ds18b20.h"




#define AT_24_DATA_MAX  100   // 最大条数
#define AT_24_Init_ADRR	 4     // 开始的地址

struct AT24CDATA
{
    uint8_t 	arr[AT_24_DATA_MAX];
    uint16_t 	len;
    uint16_t	addr;
};


extern uint16_t  ID_Cart;
extern uint8_t i;


// =========================== 系统 ===============================================
extern char time_buff[56];     // 拼装时间

// ==============================================================================



// ======================= 线程定义 ===============================================
#define SEND_MAX_QUEUE		10  // 最大队列发送
extern QueueHandle_t hsend_queue;    // 发送的队列
extern char *send_buffer;    // 发送到缓冲区的buff


#define REVE_MAX_QUEUE		10  // 最大队列接收
extern QueueHandle_t hreve_queue;    // 接收的队列
extern BaseType_t hreve_queue_base;   // 接收的宏

extern TaskHandle_t  send_data_id;    // 发送数据线程
extern TaskHandle_t  reve_data_id;    // 接收数据线程

extern TaskHandle_t  show_lcd_id;   	// 屏幕显示
extern TaskHandle_t  collect_data_id;   // 数据采集线程
extern TaskHandle_t  read_rfid_id;   // 读取rfid线程
extern TaskHandle_t  Logical_code_id;   // 逻辑判断线程
extern TaskHandle_t  key_scan_id;   // 按键扫描线程

extern SemaphoreHandle_t send_mutex;   // 发送锁
extern SemaphoreHandle_t reve_mutex;   // 接收锁
extern SemaphoreHandle_t iic_spi_mutex;   // 保护iic和spi的锁
// ===============================================================================

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
extern uint8_t reconnect_num;    // 重连次数
extern uint8_t reconnect_one_num;   // 单次重连次数
// ============================================================



// ========================== 传感器数据 ==========================
#define ADC_MAX 	40
extern uint32_t adc_value[ADC_MAX];			//adc数据缓冲区
extern double temperature;               //温度
extern double temperature_ds18;             //温度
extern double humidity;                  //湿度
extern uint32_t Lighting, Turbidity, Gas; //光照，浊度，烟雾的数据
extern double voltage;    // 电压
extern uint16_t speed;     // 电机速度
extern uint8_t people;   // 人体传感器
extern uint8_t red_people;   // 人体传感器  0无人  1有人
extern uint8_t Led_position;         // led的档位
extern uint8_t machinery_position;    // 电机档位

extern uint8_t tem_flag;     // 温度标志位
extern uint8_t Hum_flag;	  // 湿度标志物
extern uint8_t Gas_flag;	  // 空气标志位
extern uint8_t light_flag;    // 光照标志位
extern uint8_t fire_flag;   // 火焰标志位
extern uint8_t approch_switch;  // 接近开关 1 没有铁，0，有铁
extern uint8_t approch_flag;    // 接近开关的标志位
// ================================================================


// ============================= 传感器的阈值 ====================
extern int TH_Temp;   //  温度阈值
extern int TH_Hum;   // 湿度阈值
extern int TH_Gas;   // 空气阈值
extern int TH_light;   // 光照强度阈值
// ==============================================================


// ============================ user start ==============================
extern uint8_t beep_flag;    // 1 warning1   2 Error1    1->0.1s响、0.9不响    2->0.5s
extern  uint8_t user_uing;    // 用户使用标志位
extern float Total_amount;   // 总金额
// ============================ user end ==============================



#endif
