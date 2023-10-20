#include "hc-sr04.h"

Hcsr04InfoTypeDef Hcsr04Info;

/**
 * @description: ������ģ������벶��ʱ��ͨ����ʼ��
 * @param {TIM_HandleTypeDef} *htim
 * @param {uint32_t} Channel
 * @return {*}
 */
void Hcsr04Init(TIM_HandleTypeDef *htim, uint32_t Channel)
{
  /*--------[ Configure The HCSR04 IC Timer Channel ] */
  // MX_TIM2_Init();  // cubemx������
  Hcsr04Info.prescaler = htim->Init.Prescaler; //  72-1
  Hcsr04Info.period = htim->Init.Period;       //  65535
	Hcsr04Info.ic_tim_ch = Channel;
  Hcsr04Info.instance = htim->Instance;        
  if(Hcsr04Info.ic_tim_ch == TIM_CHANNEL_1)
  {
    Hcsr04Info.active_channel = HAL_TIM_ACTIVE_CHANNEL_1;             //  TIM_CHANNEL_1
  }
  else if(Hcsr04Info.ic_tim_ch == TIM_CHANNEL_2)
  {
    Hcsr04Info.active_channel = HAL_TIM_ACTIVE_CHANNEL_2;             //  TIM_CHANNEL_2
  }
  else if(Hcsr04Info.ic_tim_ch == TIM_CHANNEL_3)
  {
    Hcsr04Info.active_channel = HAL_TIM_ACTIVE_CHANNEL_3;             //  TIM_CHANNEL_3
  }
  else if(Hcsr04Info.ic_tim_ch == TIM_CHANNEL_4)
  {
    Hcsr04Info.active_channel = HAL_TIM_ACTIVE_CHANNEL_4;             //  TIM_CHANNEL_4
  }
  /*--------[ Start The ICU Channel ]-------*/
  HAL_TIM_Base_Start_IT(htim);
  HAL_TIM_IC_Start_IT(htim, Channel);//�����ж�
}

/**
 * @description: HC-SR04����
 * @param {*}
 * @return {*}
 */
void Hcsr04Start(void)
{
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
  HAL_Delay(1);  //  10us����
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}

/**
 * @description: ��ʱ����������жϴ�������
 * @param {*}    main.c���ض���void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
 * @return {*}
 */
void Hcsr04TimOverflowIsr(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == Hcsr04Info.instance) //  
  {
    Hcsr04Info.tim_overflow_counter++;
  }
}

/**
 * @description: ���벶�����ߵ�ƽʱ��->����
 * @param {*}    main.c���ض���void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
 * @return {*}
 */
void Hcsr04TimIcIsr(TIM_HandleTypeDef* htim)
{
  if((htim->Instance == Hcsr04Info.instance) && (htim->Channel == Hcsr04Info.active_channel))
  {
    if(Hcsr04Info.edge_state == 0)      //  ����������
    {
	//	printf("a\r\n");
      // �õ������ؿ�ʼʱ��T1�����������벶��Ϊ�½���
      Hcsr04Info.t1 = HAL_TIM_ReadCapturedValue(htim, Hcsr04Info.ic_tim_ch);
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, Hcsr04Info.ic_tim_ch, TIM_INPUTCHANNELPOLARITY_FALLING);
      Hcsr04Info.tim_overflow_counter = 0;  //  ��ʱ���������������
      Hcsr04Info.edge_state = 1;        //  �����ء��½��ز����־λ
    }
    else if(Hcsr04Info.edge_state == 1) //  �����½���
    {
		//	printf("saaaaa\r\n");
      // �����½���ʱ��T2��������ߵ�ƽʱ��
      Hcsr04Info.t2 = HAL_TIM_ReadCapturedValue(htim, Hcsr04Info.ic_tim_ch);
      Hcsr04Info.t2 += Hcsr04Info.tim_overflow_counter * Hcsr04Info.period; //  ��Ҫ���Ƕ�ʱ������ж�
      Hcsr04Info.high_level_us = Hcsr04Info.t2 - Hcsr04Info.t1; //  �ߵ�ƽ����ʱ�� = �½���ʱ��� - ������ʱ���
      // �������
      Hcsr04Info.distance = (Hcsr04Info.high_level_us / 1000000.0) * 340.0 / 2.0 * 100.0;
      // ���¿��������ز���
      Hcsr04Info.edge_state = 0;  //  һ�βɼ���ϣ�����
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, Hcsr04Info.ic_tim_ch, TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }
}

/**
 * @description: ��ȡ���� 
 * @param {*}
 * @return {*}
 */
float Hcsr04Read(void)
{
  // ������޷�
  if(Hcsr04Info.distance >= 450)
  {
    Hcsr04Info.distance = 450;
  }
  return Hcsr04Info.distance;
}

