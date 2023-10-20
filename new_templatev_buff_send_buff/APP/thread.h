#include "main.h"

#define RFID_FLAG   1    // 是否使用rfid
#define IWDG_FLAG   0   // 启动开门狗


/*  采集数据线程  */
void collect_data_thread_entry(void *pargem);



/* 按键处理线程 */
void key_thread_entry(void *pargem);

/*  蜂鸣器线程  */
void beep_thread_entry(void *pargem);

/* 显示lcd上 */
void lcd_show_thread_entry(void *pargem);

/* 逻辑判断线程  */
void logical_thread_entry(void *pargem);


#if RFID_FLAG
// 读rfid
void read_rfid_thread_entry(void *pargem);
#endif


/* 发送数据线程 */
void Send_data_Entry(void *pargem);
