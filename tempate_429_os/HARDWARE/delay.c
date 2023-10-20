#include "delay.h"



#ifdef  time
int year_time = 0;
int moon_time = 0;
int sun_time = 0;
int hour_time = 0;
int minute_time = 0;
int second_time = 0;

uint8_t	time_flag = 0;


void time_init(int year, int moon , int sun, int hour, int minute, int second)
{

    year_time = year;  // 年
    moon_time = moon;  // 月
    sun_time = sun;    // 日
    hour_time = hour;  // 小时
    minute_time = minute;  // 分钟
    second_time = second;  // 秒
    time_flag = 1;
    printf("时钟数据初始化成功\r\n");
}


void time_set(void)
{
    second_time++;
    if(second_time > 60)
    {
        minute_time++;
        second_time = 0;
        if(minute_time >= 60)  // 分钟
        {
            hour_time++;
            minute_time = 0;
            if(hour_time > 23)
            {
                sun_time++;
                hour_time = 0;
                if(sun_time > 31)
                {
                    moon_time++;
                    sun_time = 1;
                    year_time++;
                }
            }
        }
    }
    printf("%d年%d月%d日%d时%d分%d秒\r\n", year_time, moon_time, sun_time, hour_time, minute_time, second_time);
    //	lcd_show_string()
}

#endif

static u32 fac_us = 0;							//us延时倍乘数
void delay_init(u8 SYSCLK)
{
#if SYSTEM_SUPPORT_OS 						//如果需要支持OS.
    u32 reload;
#endif
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);//SysTick频率为HCLK
    fac_us = SYSCLK;						//不论是否使用OS,fac_us都需要使用
#if SYSTEM_SUPPORT_OS 						//如果需要支持OS.
    reload = SYSCLK;					  //每秒钟的计数次数 单位为K
    reload *= 1000000 / delay_ostickspersec;	//根据delay_ostickspersec设定溢出时间
    //reload为24位寄存器,最大值:16777216,在180M下,约合0.745s左右
    fac_ms = 1000 / delay_ostickspersec;		//代表OS可以延时的最少单位
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; //开启SYSTICK中断
    SysTick->LOAD = reload; 					//每1/OS_TICKS_PER_SEC秒中断一次
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //开启SYSTICK
#else
#endif
}

void delay_us(u32 nus)
{
    u32 ticks;
    u32 told, tnow, tcnt = 0;
    u32 reload = SysTick->LOAD;				//LOAD的值
    ticks = nus * fac_us; 						//需要的节拍数
    told = SysTick->VAL;        				//刚进入时的计数器值
    while(1)
    {
        tnow = SysTick->VAL;
        if(tnow != told)
        {
            if(tnow < told)tcnt += told - tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
            else tcnt += reload - tnow + told;
            told = tnow;
            if(tcnt >= ticks)break;			//时间超过/等于要延迟的时间,则退出.
        }
    };
}

//延时nms
//nms:要延时的ms数
void delay_ms(u16 nms)
{
	u32 i;
	for(i=0;i<nms;i++) delay_us(1000);
}

//void delay_us(__IO uint32_t delay) {
//    uint32_t us_delay = delay * (HAL_RCC_GetHCLKFreq() / 1000000);
//
//    // 使用DWT寄存器来进行微秒级别的延时
//    uint32_t start = DWT->CYCCNT;
//    while ((DWT->CYCCNT - start) < us_delay);
//}

//void delay_us(__IO uint32_t delay)
//{
//    int last, curr, val;
//    int temp;

//    while (delay != 0)
//    {
//        temp = delay > 900 ? 900 : delay;
//        last = SysTick->VAL;
//        curr = last - CPU_FREQUENCY_MHZ * temp;
//        if (curr >= 0)
//        {
//            do
//            {
//                val = SysTick->VAL;
//            }
//            while ((val < last) && (val >= curr));
//        }
//        else
//        {
//            curr += CPU_FREQUENCY_MHZ * 1000;
//            do
//            {
//                val = SysTick->VAL;
//            }
//            while ((val <= last) || (val > curr));
//        }
//        delay -= temp;
//    }
//}



