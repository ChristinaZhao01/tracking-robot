#ifndef __usart_H
#define __usart_H
#include "stm32f10x_conf.h"
void usart1_init(void);
void usart3_init(void);
void Bluetooth_init(void);
void UART_send_data(USART_TypeDef* USARTx,char data);
#endif
