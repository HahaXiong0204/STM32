#ifndef _WIFI_H_
#define _WIFI_H_
#include "main.h"
#define MAX_SIZE 1024

#define  	WIFI_USART		USART3

#define ESP8266_WIFI_INFO		"AT+CWJAP=\"my_computer\",\"2960493425\"\r\n"
//#define ESP8266_WIFI_INFO		"AT+CWJAP=\"m_phone\",\"2960493425\"\r\n"
#define ESP8266_ONENET_INFO		"AT+CIPSTART=\"TCP\",\"192.168.137.1\",1883\r\n"


//#define ESP8266_WIFI_INFO		"AT+CWJAP=\"hnyyjsxy01\",\"1235678\"\r\n"
////#define ESP8266_WIFI_INFO		"AT+CWJAP=\"m_phone\",\"2960493425\"\r\n"
//#define ESP8266_ONENET_INFO		"AT+CIPSTART=\"TCP\",\"192.168.1.109\",1883\r\n"
void ESP8266_Clear(void);
_Bool ESP8266_SendCmd(char *cmd, char *res);
void ESP8266_Init(void);
unsigned char *ESP8266_GetIPD(unsigned short timeOut);
void Usart_SendString(USART_TypeDef *USARTx, unsigned char *str, unsigned short len);
void ESP8266_SendData(unsigned char *data, unsigned short len);
#endif

