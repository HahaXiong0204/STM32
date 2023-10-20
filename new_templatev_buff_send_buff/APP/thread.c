#include "thread.h"
#include "app.h"

float Distance;                      //��������þ���
struct AT24CDATA at_data_struct = {0};    // ����Ľṹ��
char at_data_arr[256];

/*
	�ɼ�������������
	mode = 0 ����ģʽ
	mode = 1 ���浽at24c02
*/
void collect_data_thread_entry(void *pargem)
{
//	uint8_t c_num = 0;
	uint32_t value1 = 0, value2 = 0, value3 = 0, value4 = 0;
	uint8_t sensor_again_num = 0;
    while(1)
    {
        //		printf("net_connect = %d\r\n",net_connect);
		
		osDelay(2000);
        if(net_connect)
        {
            // =============================================== �ٶ� ==================================================
            speed = TIM4->CNT;
            printf("speed = %d\r\n", TIM4->CNT);
            TIM4->CNT = 0;
            // =============================================================================================================

            // =============================================== ���崫���� ==================================================
//			
			Hcsr04Start();
			printf("dis = %.1f\r\n",Hcsr04Read());
//			
			
            if(HAL_GPIO_ReadPin(People_GPIO_Port, People_Pin))
            {
                printf("����\r\n");
				people = 1;
				
            }
            else
            {
                printf("����\r\n");
				people = 0;
				
            }
			
			
			if(HAL_GPIO_ReadPin(red_input_GPIO_Port, red_input_Pin)==1)
            {
                printf("����\r\n");
				red_people = 1;
				
            }
            else
            {
                printf("����\r\n");
				red_people = 0;
				
            }
			
			if(HAL_GPIO_ReadPin(approach_switch_GPIO_Port,approach_switch_Pin)==0)
			{
			
				printf("����\r\n");
				approch_switch = 1;
			}else{
				printf("����\r\n");
				approch_switch = 0;
			}
			
			
			
			
            // ==============================================================================================================


            //===============================================dht11��ȡ======================================================
            DHT11_Read_Data(&humidity, &temperature);
//			osDelay(50);
			temperature_ds18=(double)DS18B20_Get_Temp()/10;
            printf("temperature = %.2f		humidity = %.2f   temperature_ds18:%.2f\r\n", temperature, humidity,temperature_ds18);
			
			osDelay(50);
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
            Lighting = (4096 - value1 / 10) * 100 / 4096; //����
            printf("������ֵΪ��%d%%\r\n", Lighting);

            // PA5
            voltage = (float)value2 / 10 * (3.3 / 4096);
            printf("����ѹֵΪ1��%.3fV\r\n", voltage);

            // PA6
            Turbidity = (value3 / 10) * 100 / 4096; //�Ƕ�
            printf("����Ƕ�ֵΪ��%d\r\n", Turbidity);

            // PA7
            Gas = (value4 / 10) * 100 / 4096; //����
            printf("�������ֵΪ��%d%%\r\n", Gas);
            value1 = value2 = value3 = value4 = 0;

            // ���浽at24c02
            if(net_connect == 2 || net_connect == 3)
            {
                sensor_again_num++;
                if(sensor_again_num > 3)
                {
                    sensor_again_num = 0;
                    memset(at_data_arr, 0, sizeof(at_data_arr));
                    sprintf(at_data_arr, "%.2f-%.2f-%d-%.2f-%d-%d", temperature, humidity, Lighting, voltage, Turbidity, Gas);
                    printf("��������:%s    len = %d\r\n", at_data_arr, strlen(at_data_arr));

                    AT24C02_write(at_data_struct.addr, (uint8_t *)at_data_arr, strlen(at_data_arr));
                    at_data_struct.arr[at_data_struct.len++] = strlen(at_data_arr);
                    memset(at_data_arr, 0, sizeof(at_data_arr));
                    AT24C02_read(at_data_struct.addr, (uint8_t *)at_data_arr, at_data_struct.arr[at_data_struct.len - 1]);
                    at_data_struct.addr += strlen(at_data_arr);
                    printf("��������%d �� %d   %s\r\n", at_data_struct.len, at_data_struct.addr, at_data_arr);
                }
				osDelay(200);

            }

			Send_Zigbee();   // ���͵�zigbee��
        }
		

    }

}

/*
	���������߳�
*/
uint8_t t = 0;
void key_thread_entry(void *pargem)
{
	
	uint8_t key = 0;
    while(1)
    {
        key = KEY_Scan(0);
        switch(key)
        {
        case KEY0_PRES:
        {
            OneNet_Send_Int("key", 1);
            printf("KEY0_PRES\r\n");
            break;
        }
        case KEY1_PRES:
        {
			OneNet_Send_one_Data(0);
			SG90_Rotate(1,Degrees_45);
            printf("KEY1_PRES\r\n");
            break;
        }
        case KEY2_PRES:
        {
			SG90_Rotate(1,Degrees_0);
            printf("KEY2_PRES\r\n");
            break;
        }
        case WKUP_PRES:
        {
			SG90_Rotate(1,Degrees_90);
            printf("WKUP_PRES\r\n");
            break;
        }
        case RED_PRES:
        {
			SG90_Rotate(1,Degrees_135);

            printf("RED_PRES\r\n");
            break;
        }
        case YELLOW_PRES:
        {
			SG90_Rotate(1,Degrees_180);
			Send_tianwen(t--);
            printf("YELLOW_PRES\r\n");
            break;
        }
        case GRREN_PRES:
        {
			Send_tianwen(t++);
            printf("grean_pres\r\n");
            break;
        }

        }
        osDelay(10);

    }


}

/*
	��rfid��ֵ
*/
#if RFID_FLAG
// ��rfid
void read_rfid_thread_entry(void *pargem)
{
    while(1)
    {
        //		printf("\r\nid");

		/*  ɨ�赽rfid �����߳�    ���Ҫ���¿����߳���Ҫ�� vTaskResume(read_rfid_id); */
        if(user_uing == 0)
        {
            while(ReaderCard() == 0)
            {
                printf("wait id\r\n");
            }
            user_uing = 1;
			vTaskSuspend(read_rfid_id);
        }
        osDelay(2000);
    }
}
#endif




/* 
	�߼��ж��߳�  
*/

void logical_thread_entry(void *pargem)
{
	while(1)
	{
		osDelay(500);
		if(net_connect == 1)
        {		
            if(user_mode == 0) // �Զ�ģʽ
            {
				/*�ӽ�����*/
				if(approch_switch==1)
				{
					if(approch_flag!=1)
					{
						OneNet_Send_Int("fire_flag", 1);
					}
					fire_flag = 1;
					HAL_TIM_Base_Start_IT(&htim10);
					
				}
				else{
						if(fire_flag != 0)
						{
							OneNet_Send_Int("fire_flag", 0);
							beep_On_Off(1);
						}
						fire_flag = 0;
						
					}
				
				
                if(temperature > TH_Temp)
                {
                    if(tem_flag != 1)
                        OneNet_Send_Int("tem_flag", 1);
                    tem_flag = 1;
                    machinery_pwm_mode(1);
                    HAL_TIM_Base_Start_IT(&htim10);
                }
                else
                {
                    if(tem_flag != 0)
                    {
                        OneNet_Send_Int("tem_flag", 0);
                        beep_On_Off(1);
                    }
                    tem_flag = 0;
                    machinery_pwm_mode(0);
                }


                if(humidity > TH_Hum)
                {
                    if(Hum_flag != 1)
                        OneNet_Send_Int("Hum_flag", 1);
                    Hum_flag = 1;
                    HAL_TIM_Base_Start_IT(&htim10);
                }
                else
                {
                    if(Hum_flag != 0)
                    {
                        OneNet_Send_Int("Hum_flag", 0);
                        beep_On_Off(1);
                    }
                    Hum_flag = 0;
                }

                if(Gas > TH_Gas)
                {
                    if(Gas_flag != 1)
                        OneNet_Send_Int("Gas_flag", 1);
                    Gas_flag = 1;
                    HAL_TIM_Base_Start_IT(&htim10);
                }
                else
                {
                    if(Gas_flag != 0)
                    {
                        OneNet_Send_Int("Gas_flag", 0);
                        beep_On_Off(1);
                    }
                    Gas_flag = 0;

                }

                if(Lighting < TH_light)
                {
                    Led_pwm_mode(1);
                    if(light_flag != 1)
                        OneNet_Send_Int("light_flag", 1);
                    light_flag = 1;
                }
                else
                {
                    Led_pwm_mode(0);
                    if(light_flag != 0)
                    {
                        OneNet_Send_Int("light_flag", 0);
                        beep_On_Off(1);
                    }
                    light_flag = 0;

                }


            }



            
        }
		
		
		
	}

}



/*
	��ʾlcd��
*/
void lcd_show_thread_entry(void *pargem)
{
	char show_buff[124];
	// һ�����ִ��ռ40
    while(1)
    {
		osDelay(1000);
        if(net_connect)
        {
//            LCD_Clear(WHITE);
            // ��ʾʱ��
            memset(show_buff, 0, sizeof(show_buff));
			Show_Str(10,40,200,32,"ʱ�䣺",32,0);
            sprintf(show_buff, "%d-%d-%d-%d-%d-%d", year_time, moon_time, sun_time, hour_time, minute_time, second_time);
            LCD_ShowString(90, 40, 460, 32, 32, (uint8_t *)show_buff);
			
			


            // ��ʾ�豸��
            memset(show_buff, 0, sizeof(show_buff));
            sprintf(show_buff, "%s      RFID:%d", PROID,ID_Cart);
			Show_Str(10,80,200,32,"�豸��:",32,0);
            LCD_ShowString(130, 80, 560, 32, 32, (uint8_t *)show_buff);

            // ��ʾ��ֵˮ��
            memset(show_buff, 0, sizeof(show_buff));
            sprintf(show_buff, "%d          %d", TH_Temp, TH_Hum);
            LCD_ShowString(160, 120, 560, 32, 32, (uint8_t *)show_buff);
			Show_Str(10,120,200,32,"�¶���ֵ:",32,0);
			Show_Str(210,120,200,32,"ʪ����ֵ:",32,0);

            // ��ʾ�¶� ʪ��
            memset(show_buff, 0, sizeof(show_buff));
            sprintf(show_buff, "%.2f        %.2f", temperature, humidity);
            LCD_ShowString(90, 160, 560, 32, 32, (uint8_t *)show_buff);
			Show_Str(10,160,200,32,"�¶�:",32,0);
			Show_Str(200,160,200,32,"ʪ��:",32,0);
			
            // ��ʾgas
            memset(show_buff, 0, sizeof(show_buff));
            sprintf(show_buff, "%d               %d ", Gas, TH_Gas);
            LCD_ShowString(160, 200, 560, 32, 32, (uint8_t *)show_buff);
			Show_Str(10,200,200,32,"��������:",32,0);
			Show_Str(210,200,200,32,"������ֵ:",32,0);

            // ��ʾlight
            memset(show_buff, 0, sizeof(show_buff));
            sprintf(show_buff, "%d%%                %d ", Lighting, TH_light);
            LCD_ShowString(90, 240, 560, 32, 32, (uint8_t *)show_buff);
			Show_Str(10,240,200,32,"����:",32,0);
			Show_Str(180,240,200,32,"������ֵ:",32,0);

// ==================== ==========================================
			// ���崫����
			if(people == 1)
			{
				Show_Str(10, 720, 200, 32, "����", 32, 0);
			}
			else
			{
				Show_Str(10, 720, 200, 32, "����", 32, 0);
			}
			/*����*/
			if(red_people==1){
				Show_Str(120, 720, 200, 32, "���ڵ�", 32, 0);
			}
			else{
				Show_Str(120, 720, 200, 32, "���ڵ�", 32, 0);
			}
			
			/*�ӽ�����*/
			if(approch_switch==1){
				Show_Str(260, 720, 200, 32, "�ӽ���", 32, 0);
			}
			else{
				Show_Str(260, 720, 200, 32, "�ӽ���", 32, 0);
			}
				
//===========================================================
            //���
            if(net_connect == 1)
				Show_Str(10, 760, 200, 32, "���ӳɹ�!!!", 32, 0);
//                sprintf(show_buff, "connect success !!! %d bp = %d", user_mode, beep_flag);
            else if(net_connect == 0 )
				Show_Str(10, 760, 200, 32, "�ȴ�����!!!", 32, 0);
//                sprintf(show_buff, "wait connect!!! %d  bp = %d", user_mode, beep_flag);
            else
                Show_Str(10, 760, 200, 32, "������������!!!", 32, 0);
			
			// ��������
			Show_Str(260, 760, 200, 32, "��������:", 32, 0);
			memset(show_buff, 0, sizeof(show_buff));
			sprintf(show_buff,"%d",reconnect_num);
            LCD_ShowString(400, 760, 560, 32, 32, (uint8_t *)show_buff);
        }

    }


}

/* 
	���������߳� 
*/
void Send_data_Entry(void *pargem)
{
	char *Send_rece_Str = NULL;    // ���ͻ���������ָ��
	cJSON *cjson = NULL;
	while(1)
	{
			if((net_connect==1)&&(xQueueReceive( hsend_queue, &Send_rece_Str, portMAX_DELAY))) /* �õ�buffer��ַ��ֻ��4�ֽ� */
			{
				/* ���������� */
				if(Send_rece_Str!=NULL)
				{
					Wait_send_mutex();
					printf("Get***************************: %s", Send_rece_Str);
					cjson = cJSON_Parse(Send_rece_Str); 
					if(cjson!=NULL){
						if(cJSON_GetObjectItem(cjson,"clientId")!=NULL)
						{
							OneNet_Publish("deviceOnlineStatus",Send_rece_Str);
						}
						else{
							OneNet_Publish("one",Send_rece_Str);
						}
						Send_rece_Str = NULL;
					}
					Release_send_mutex(); 
					
				}
				
			}
//			osDelay(100);
		
	
	}

}





