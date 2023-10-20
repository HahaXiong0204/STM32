#include "app.h"

#include "adc.h"
#include "tim.h"
#include "pic.h"

#define AT_24_DATA_MAX  100   // 最大条数
#define AT_24_Init_ADRR	 4     // 开始的地址
struct AT24CDATA
{
    uint8_t 	arr[AT_24_DATA_MAX];
    uint16_t 	len;
    uint16_t	addr;
};
struct AT24CDATA at_data_struct = {0};    // 掉电的结构体
char at_data_arr[256];


void Mx_Init(void)
{
	delay_init(180);
    LCD_Init();
	DHT11_Init();
    while(AT24C02_Check()) {}
    while(pcf8574_Check()) {}
	SDRAM_Init();
	RC522_Init();
	
	
		
    my_mem_init(SRAMIN);		    //初始化内部内存池
    my_mem_init(SRAMEX);		    //初始化外部内存池
    my_mem_init(SRAMCCM);		    //初始化CCM内存池
	HAL_ADC_Start_DMA(&hadc1,adc_value,ADC_MAX);
	HAL_TIM_Base_Start_IT(&htim1);
		HAL_TIM_Base_Start(&htim4);
		__HAL_TIM_SET_COUNTER(&htim4,0); 
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
		

	while(W25QXX_ReadID()!=W25Q256)								//检测不到W25Q256
	{
		LCD_ShowString(30,150,200,16,16,"W25Q256 Check Failed!");
		delay_ms(500);
		LCD_ShowString(30,150,200,16,16,"Please Check!        ");
		delay_ms(500);
	}
	while(font_init()) 		//检查字库
	{	    
		LCD_ShowString(30,50,200,16,16,"Font Error!");
		delay_ms(200);				  
		LCD_Fill(30,50,240,66,WHITE);//清除显示	     
		delay_ms(200);				  
	}  	 

		
	show_picture(0,0,320,320,(uint16_t *)gImage_pic);
		
		at_sava_th();
		
	printf("读阈值前  %d-%d-%d-%d\r\n",TH_Temp,TH_Hum,TH_Gas,TH_light);
	// 读取阈值数据
	AT24C02_read(0,&at_th_size,1);
	AT24C02_read(1,(uint8_t *)at_th_buff,at_th_size);
	
	sscanf(at_th_buff,"%d-%d-%d-%d",&TH_Temp,&TH_Hum,&TH_Gas,&TH_light);
	printf("读阈值后  %d-%d-%d-%d\r\n",TH_Temp,TH_Hum,TH_Gas,TH_light);

	
}

void Logical_code_entry(void *pargem)
{
	while(1)
	{
		if(net_connect==1)
		{
			if(user_mode==0)  // 自动模式
			{
//			printf("user_mode = %d\r\n",user_mode);
				if(temperature > TH_Temp)
				{
					if(tem_flag!=1)
						OneNet_Send_Int("tem_flag",1);
					tem_flag = 1;
					machinery_pwm_mode(1);
//					printf("beep_flag = %d\r\n",beep_flag);
					HAL_TIM_Base_Start_IT(&htim2);
				}
				else
				{
					if(tem_flag!=0){
						OneNet_Send_Int("tem_flag",0);
						beep_On_Off(1);
					}
					tem_flag = 0;
					machinery_pwm_mode(0);
				}
				

				if(humidity>TH_Hum)
				{
					if(Hum_flag!=1)
						OneNet_Send_Int("Hum_flag",1);
					Hum_flag = 1;
//					printf("beep_flag = %d\r\n",beep_flag);
					HAL_TIM_Base_Start_IT(&htim2);
				}
				else
				{
					if(Hum_flag!=0){
						OneNet_Send_Int("Hum_flag",0);
						beep_On_Off(1);
					}
					Hum_flag = 0;
//					printf("beep_flag = %d\r\n",beep_flag);
//					HAL_TIM_Base_Start_IT(&htim2);
				}
				
				if(Gas>TH_Gas)
				{
					if(Gas_flag!=1)
						OneNet_Send_Int("Gas_flag",1);
					Gas_flag = 1;
//					printf("beep_flag = %d\r\n",beep_flag);
					HAL_TIM_Base_Start_IT(&htim2);
				}
				else{
					if(Gas_flag!=0){
						OneNet_Send_Int("Gas_flag",0);
						beep_On_Off(1);
					}
					Gas_flag = 0;
//					printf("beep_flag = %d\r\n",beep_flag);
//					HAL_TIM_Base_Start_IT(&htim2);
				}
				
				if(Lighting<TH_light)
				{
					Led_pwm_mode(1);
					if(light_flag!=1)
						OneNet_Send_Int("light_flag",1);
					light_flag = 1;
				}
				else
				{
					Led_pwm_mode(0);
					if(light_flag!=0){
						OneNet_Send_Int("light_flag",0);
						beep_On_Off(1);
					}
					light_flag = 0;
				
				}

			
			}
			
			
			
			rt_thread_mdelay(40);
		}
		
	}
	
}


// 按键处理线程
uint8_t key = 0;
void key_thread_entry(void *pargem)
{
	while(1)
	{
		key = KEY_Scan(0);
		switch(key)
		{
			case KEY0_PRES:
			{
				OneNet_Send_Int("key",1);
				printf("KEY0_PRES\r\n");
				break;
			}
			case KEY1_PRES:
			{
				printf("KEY1_PRES\r\n");
				break;
			}
			case KEY2_PRES:
			{
				printf("KEY2_PRES\r\n");
				break;
			}
			case WKUP_PRES:
			{
				printf("WKUP_PRES\r\n");
				break;
			}
			case RED_PRES:
			{
				beep_flag = 1;
				beep_On_Off(0);
				OneNet_Send_Int("beep_flag",1);
				printf("RED_PRES\r\n");
				break;
			}
			case YELLOW_PRES:
			{
				printf("YELLOW_PRES\r\n");
				break;
			}
			case GRREN_PRES:
			{
				printf("grean_pres\r\n");
				break;
			}

		}
		rt_thread_mdelay(10);
	
	}


}


// 保存阈值数据到at24c02
char at_th_buff[64];
uint8_t at_th_size = 0;   // 阈值的数据大小
void at_sava_th(void)
{
	uint8_t temp = 0;
	memset(at_th_buff,0,sizeof(at_th_buff));
	sprintf(at_th_buff,"%d-%d-%d-%d",TH_Temp,TH_Hum,TH_Gas,TH_light);
	printf("%s\r\n",at_th_buff);
	at_th_size = strlen(at_th_buff);
	AT24C02_write(0,&at_th_size,1);
	if(at_th_size==AT24C02_read(1,&temp,1))
	{
		printf("数据长度保存成功 = %d \r\n",at_th_size);
	}
	AT24C02_write(1,(uint8_t *)at_th_buff,at_th_size);
	
	memset(at_th_buff,0,sizeof(at_th_buff));
	AT24C02_read(1,(uint8_t *)at_th_buff,at_th_size);
	printf("保存数据 = %s\r\n",at_th_buff);
	if(net_connect==0||net_connect==1)
		at_data_struct.addr = at_th_size;
}

// 读rfid
void read_rfid(void *pargem)
{
	while(1)
	{
//		printf("\r\nid");
		if(user_uing==0)
		{
			while(ReaderCard()==0){
				printf("wait id\r\n");
			}
			user_uing = 1;
		}
		rt_thread_mdelay(2000);
	}
}

// 显示lcd上
char show_buff[124];
void lcd_show(void *pargem)
{
	while(1)
	{
		if(net_connect)
		{
			LCD_Clear(WHITE);
			// 显示时间
			memset(show_buff,0,sizeof(show_buff));
			sprintf(show_buff,"time:%d-%d-%d-%d-%d-%d",year_time,moon_time,sun_time,hour_time,minute_time,second_time);
			LCD_ShowString(10, 40, 460, 32, 32, (uint8_t *)show_buff);
			
				// 显示设备号
			memset(show_buff,0,sizeof(show_buff));
			sprintf(show_buff,"device:%d",Device_ID);
			LCD_ShowString(10,80,560,32,32,(uint8_t *)show_buff);
			
				// 显示阈值水温
			memset(show_buff,0,sizeof(show_buff));
			sprintf(show_buff,"TH_tem:%d  TH_Hum:%d",TH_Temp,TH_Hum);
			LCD_ShowString(10,120,560,32,32,(uint8_t *)show_buff);
			
			// 显示温度 湿度
			memset(show_buff,0,sizeof(show_buff));
			sprintf(show_buff,"temp:%.2f  Hum:%.2f",temperature,humidity);
			LCD_ShowString(10,160,560,32,32,(uint8_t *)show_buff);
			
				// 显示gas
			memset(show_buff,0,sizeof(show_buff));
			sprintf(show_buff,"Gas:%d   TH_Gas:%d ",Gas,TH_Gas);
			LCD_ShowString(10,200,560,32,32,(uint8_t *)show_buff);
			
				// 显示light
			memset(show_buff,0,sizeof(show_buff));
			sprintf(show_buff,"light:%d   TH_light:%d ",Lighting,TH_light);
			LCD_ShowString(10,240,560,32,32,(uint8_t *)show_buff);

			memset(show_buff,0,sizeof(show_buff));
			
			rt_interrupt_enter();
			Show_Str(30,280,200,32,"阿波罗STM32F4/F7开发板",32,0);	
			
			rt_interrupt_leave();
			
			//最后
			memset(show_buff,0,sizeof(show_buff));
			if(net_connect==1)
				sprintf(show_buff,"connect success !!! %d bp = %d",user_mode,beep_flag);
			else if(net_connect == 0 )
				sprintf(show_buff,"wait connect!!! %d  bp = %d",user_mode,beep_flag);
			else
				sprintf(show_buff,"Waiting for reconnection !!!   %d   bp = %d",user_mode,beep_flag);
			LCD_ShowString(10,760,560,32,32,(uint8_t *)show_buff);
			
			
			rt_thread_mdelay(1000);
		
		}
		
	}
	

}


// 通知上位机保活线程
void ping_thread_entry(void *pargem)
{
	
	char ping_buff[64];
	memset(ping_buff,0,sizeof(ping_buff));
	sprintf(ping_buff,"{\"clientId\":\"%s\",\"Device_ID\":\"%s\"}",DEVID,PROID);
	while(1)
	{
//		Wait_send_mutex();
		if(net_connect == 1)
		{
			OneNet_Publish("deviceOnlineStatus",ping_buff);	
		}
		rt_thread_mdelay(3000);
//		Release_send_mutex();
//		osDelay(3000);
	}
}

static uint8_t	net_cone_num =0 ;
static uint8_t at24_addr = 4;
// 断网重连 wifi
void resest_wifi(void)
{
	net_cone_num = 0;
	if(ESP8266_SendCmd("AT\r\n", "OK") == 0)
	{
			if(ESP8266_SendCmd("AT+CWMODE=1\r\n", "OK") == 0)
			{
				if(ESP8266_SendCmd("AT+CWDHCP=1,1\r\n", "OK") == 0)
				{
					//								HAL_Delay(200);
					while(ESP8266_SendCmd(ESP8266_WIFI_INFO, "GOT IP")) {}
					while(ESP8266_SendCmd(ESP8266_WIFI_INFO, "GOT IP"))
					{
						net_cone_num++;
						if(net_cone_num > 20)
						{

							break;
						}
						//									}
					}
					if(net_cone_num < 20)
					{
						net_connect = 3;
						printf("net_connet = %d\r\n", net_connect);
					}
					net_cone_num = 0;
				}
			}

	}
	rt_thread_mdelay(200);
}

// 断网重连服务器
void resest_server(void)
{
	net_cone_num = 5;
	printf("进入net_connet = %d\r\n", net_connect);
	while(ESP8266_SendCmd(ESP8266_ONENET_INFO, "CONNECT") == 0)
	{
		while(OneNet_DevLink())			//接入OneNET
		{
			if(net_connect == 2)
				break;
		}
		if(net_connect != 2)
		{
			HAL_Delay(1000);
			printf("接入onenet成功");
			at24_addr = AT_24_Init_ADRR;
			again_num = 0;
			net_connect  = 1;
			for(i = 0; i < at_data_struct.len; i++)
			{
				memset(at_data_arr, 0, sizeof(at_data_arr));
				AT24C02_read(at24_addr, (uint8_t *)at_data_arr, at_data_struct.arr[i]);
				sscanf(at_data_arr,"%.2f-%.2f-%d-%.2f-%d-%d",&temperature,&humidity,&Lighting,&voltage,&Turbidity,&Gas);
				printf("拼接数据  =%.2f-%.2f-%d-%.2f-%d-%d\r\n",temperature,humidity,Lighting,voltage,Turbidity,Gas);
				OneNet_SendData(1);
				at24_addr += at_data_struct.arr[i];
				printf("重发数据 %d = %s\r\n", i, at_data_arr);
			}
			at_data_struct.addr = AT_24_Init_ADRR;
			memset(at_data_struct.arr,0,sizeof(at_data_struct.arr));
			at_data_struct.len = 0;
			ESP8266_Clear();									//清空缓存

			break;
		}


	}
	rt_thread_mdelay(200);

}


// 采集传感器的数据
// mode = 0 正常模式
// mode = 1 保存到at24c02
uint32_t value1=0,value2=0,value3=0,value4=0;
uint8_t sensor_again_num = 0;
void collect_data(void *pargem)
{
	while(1)
	{
//		printf("net_connect = %d\r\n",net_connect);
		if(net_connect)
		{
			// =============================================== 速度 ==================================================
			speed = TIM4->CNT;
			printf("speed = %d\r\n",TIM4->CNT);
			TIM4->CNT = 0;
			// =============================================================================================================
			
			// =============================================== 人体传感器 ==================================================
			if(HAL_GPIO_ReadPin(People_GPIO_Port,People_Pin))
			{
				printf("有人\r\n");
			}
			else{
				printf("无人\r\n");
			
			}
			// ==============================================================================================================
			
			
			//===============================================dht11读取======================================================
			DHT11_Read_Data(&humidity, &temperature);
			printf("temperature = %.2f		humidity = %.2f\r\n", temperature, humidity);
			//================================================================================================================
			for(i = 0, value1 = 0, value2 = 0, value3 = 0, value4 = 0; i < 40;)
			{
				value1 += adc_value[i++];
				value2 += adc_value[i++];
				value3 += adc_value[i++];
				value4 += adc_value[i++];
			}
			printf("value1 = %d    value2 = %d     value3 = %d    value4 = %d\r\n", value1, value2, value3, value4);
			// PA4
			Lighting = (4096 - value1 / 10) * 100 / 4096; //亮度
			printf("检测光亮值为：%d%%\r\n", Lighting);

			// PA5
			voltage = (float)value2 / 10 * (3.3 / 4096);
			printf("检测电压值为1：%.3fV\r\n", voltage);

			// PA6
			Turbidity = (value3 / 10) * 100 / 4096; //浊度
			printf("检测浊度值为：%d\r\n", Turbidity);
			
			// PA7
			Gas = (value4 / 10) * 100 / 4096; //烟雾
			printf("检测烟雾值为：%d%%\r\n", Gas);

			rt_thread_mdelay(2000);
			value1 = value2 = value3 = value4 = 0;
			
			// 保存到at24c02
			if(net_connect == 2|| net_connect == 3)
			{
				sensor_again_num++;
				if(sensor_again_num>3)
				{
					sensor_again_num = 0;
					memset(at_data_arr,0,sizeof(at_data_arr));
					sprintf(at_data_arr,"%.2f-%.2f-%d-%.2f-%d-%d",temperature,humidity,Lighting,voltage,Turbidity,Gas);
					printf("保存数据:%s    len = %d\r\n",at_data_arr,strlen(at_data_arr));
					
					AT24C02_write(at_data_struct.addr,(uint8_t *)at_data_arr,strlen(at_data_arr));
					at_data_struct.arr[at_data_struct.len++] = strlen(at_data_arr);
					memset(at_data_arr, 0, sizeof(at_data_arr));
					AT24C02_read(at_data_struct.addr, (uint8_t *)at_data_arr, at_data_struct.arr[at_data_struct.len-1]);
					at_data_struct.addr += strlen(at_data_arr);
					printf("保存数据%d 条 %d   %s\r\n",at_data_struct.len,at_data_struct.addr,at_data_arr);
				}
			
			}
			
		
		}
	
	}


}

// 设置电机档位
void machinery_pwm_mode(uint8_t mode)
{
	machinery_position = mode;
	if(mode == 0)	TIM3->CCR3 = 499;
	if(mode == 1)   TIM3->CCR3 = 200;
	if(mode == 2)	TIM3->CCR3 = 0;
}


// 设置LED档位
void Led_pwm_mode(uint8_t mode)
{
	Led_position = mode;
	if(mode == 0)	TIM3->CCR4 = 500;
	if(mode == 1)   TIM3->CCR4 = 300;
	if(mode == 2)	TIM3->CCR4 = 0;
}


