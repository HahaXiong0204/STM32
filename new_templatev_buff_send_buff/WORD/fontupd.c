#include "fontupd.h"
#include "w25qxx.h"
#include "lcd.h"
#include "string.h"
#include "delay.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//字库更新 驱动代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/1/7
//版本：V1.1
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//********************************************************************************
//升级说明
//V1.1 20160528
//修改部分代码,以支持fatfs  r0.12版本
//////////////////////////////////////////////////////////////////////////////////

//字库区域占用的总扇区数大小(4个字库+unigbk表+字库信息=6302984字节,约占1539个W25QXX扇区,一个扇区4K字节)
#define FONTSECSIZE	 	1539
//字库存放起始地址
#define FONTINFOADDR 	1024*1024*25 					//Apollo STM32开发板是从25M地址以后开始存放字库
//前面25M被fatfs占用了.
//25M以后紧跟4个字库+UNIGBK.BIN,总大小6.01M,被字库占用了,不能动!
//31.01M以后,用户可以自由使用.

//用来保存字库基本信息，地址，大小等
_font_info ftinfo;



//初始化字体
//返回值:0,字库完好.
//		 其他,字库丢失
u8 font_init(void)
{
    u8 t = 0;
    W25QXX_Init();
    while(t < 10) //连续读取10次,都是错误,说明确实是有问题,得更新字库了
    {
        t++;
        W25QXX_Read((u8 *)&ftinfo, FONTINFOADDR, sizeof(ftinfo)); //读出ftinfo结构体数据
        if(ftinfo.fontok == 0XAA)break;
        delay_ms(20);
    }
    if(ftinfo.fontok != 0XAA)return 1;
    return 0;
}





























