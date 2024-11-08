#ifndef __st188_H
#define __st188_H
#include "stm32f10x.h"

//定义st188传感器引脚
#define ST188L GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)
#define ST188M GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2)
#define ST188R GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1)

#define ST188DATA 7-((ST188L<<2)+(ST188M<<1)+ST188R)


void ST188_init(void);
void ST188_control(u8 temp);
u8 ST188_DATA_CHECK(float Angle);


#endif















