#ifndef __iic_H
#define __iic_H

#include "stm32f10x_conf.h"
#define	WRITE 0
#define	READ  1

void iic_init(void);
void iic_rw(u8 *DataBuff,u8 ByteQuantity,u8 RegAddress,u8 SlaveAddress,u8 ControlByte);
#endif
