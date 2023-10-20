#ifndef __DUOJI_H
#define __DUOJI_H

#include "main.h"
#include "delay.h"

#define Degrees_OPEN    50  
#define Degrees_CLOSE	250 

#define Degrees_0    50  //正转2档
#define Degrees_45	100 //正转1档
#define Degrees_90   150 //停
#define Degrees_135  200 //反转1档
#define Degrees_180  250 //反转2档
extern TIM_HandleTypeDef htim5;

void SG90_Rotate(uint16_t ch,uint16_t Degrees);

#endif


