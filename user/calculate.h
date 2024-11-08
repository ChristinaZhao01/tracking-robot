#ifndef __calculate_H
#define __calculate_H
#include "stm32f10x_conf.h"
#define Bias 0
void pid(float angle, float angle_dot);
void receive_parameter(u8 cmd);
#endif
