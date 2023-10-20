#ifndef _DELAY_H

#include "main.h"

#include "lcd.h"
#include "sys.h"


extern void delay_init(void);
extern void delay_us(uint32_t nus);
extern void delay_ms(uint32_t nms);

#define time  1

#ifdef time
extern uint8_t	time_flag;
extern int year_time;
extern int moon_time;
extern int sun_time;
extern int hour_time;   // 小时
extern int minute_time;  // 分钟
extern int second_time;  // 秒

void time_set(void);
void time_init(int year, int moon , int sun, int hour, int minute, int second);
#endif

//#define delay_ms    HAL_Delay

//#define CPU_FREQUENCY_MHZ    180		// STM32时钟主频
//void delay_us(uint32_t udelay);


//void delay_init(u8 SYSCLK);
//void delay_ms(u16 nms);
//void delay_us(u32 nus);

#endif
