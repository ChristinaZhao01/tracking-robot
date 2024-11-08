#ifndef __mpu6050_H
#define __mpu6050_H
#include "stm32f10x_conf.h"
#define	GYRO_SCALE 0.06097609f  //+-2000

void mpu6050_init(void);
void mpu6050_get_data(s16 *gx,s16 *gy,s16 *gz,s16 *ax,s16 *ay,s16 *az,s16 *temperature);
#endif
