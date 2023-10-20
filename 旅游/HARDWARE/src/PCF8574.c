#include "PCF8574.h"


uint8_t pcf8574_Check(void)
{
    //	HAL_I2C_GetState()
    if(HAL_I2C_Master_Transmit(&hi2c2, PCF8574_ADDR, NULL, 0, HAL_MAX_DELAY) == HAL_OK)
    {
        printf("PCF8574 存在\r\n");
        pcf8574_WriteOneByte(0XFF);
        uint8_t temp = pcf8574_ReadOneByte();
        printf("temp = %d\r\n", temp);
        return 0;
    }
    printf("PCF8574 不存在\r\n");
    return 1;
}



//设置PCF8574某个IO的高低电平
//bit:要设置的IO编号,0~7
//sta:IO的状态;0或1
void PCF8574_WriteBit(uint8_t bit, uint8_t sta)
{
    uint8_t data;
    data = pcf8574_ReadOneByte(); //先读出原来的设置
    if(sta == 0)data &= ~(1 << bit);
    else data |= 1 << bit;
    pcf8574_WriteOneByte(data); //写入新的数据
}


//读取PCF8574的某个IO的值
//bit：要读取的IO编号,0~7
//返回值:此IO的值,0或1
uint8_t PCF8574_ReadBit(u8 bit)
{
    uint8_t data;
    data = pcf8574_ReadOneByte(); //先读取这个8位IO的值
    if(data & (1 << bit))return 1;
    else return 0;
}




uint8_t pcf8574_ReadOneByte(void)
{
    uint8_t temp;
    HAL_I2C_Master_Receive(&hi2c2, PCF8574_ADDR, &temp, 1, HAL_MAX_DELAY);
    return temp;

}

void pcf8574_WriteOneByte(uint8_t date)
{

    HAL_I2C_Mem_Write(&hi2c2, PCF8574_ADDR, 0, I2C_MEMADD_SIZE_8BIT, &date, 1, HAL_MAX_DELAY);

}


void beep_On_Off(uint8_t flag)
{
    PCF8574_WriteBit(BEEP_IO, flag);
}


