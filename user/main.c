/******************************************************************************
��    ����     main.c 
��    �ߣ�     Subolt
Q   Q  :  		1512195337
��    �ڣ� 		2017/3/22
����汾�� 		 MDK5.23
�������ӣ�     http://pan.baidu.com/s/1bo0Wu7d
******************************************************************************/
#include "stm32f10x.h"
#include "iic.h"
#include "timer.h"
#include "usart.h"
#include "mpu6050.h"
#include "filter.h"
#include "calculate.h"
#include "stepmotor.h"
#include "st188.h"
#include <stdio.h>
#include <math.h>
//�˴�����Ϊ�ֶ�У׼ֵ�����ݴ�������ֹ״�µ�ƫ������趨������ƫ��
int GX_OFFSET =0x01;
int GY_OFFSET =0x01;
int AX_OFFSET =0x01;
int AY_OFFSET =0x01;
int AZ_OFFSET =0x01;


extern float rout;//��������һ���м����
extern s16 turn_need_l;//��Ϊ��������ߵ���ٶ�
extern s16 turn_need_r;//��Ϊ�������ұߵ���ٶ�
extern s16 speed_need;//��Ϊ����������ǰ���ٶ��ٶ�
u8 Time2_flag;//��ʱ��2�������Ϊ������־����������������Ϊʱ��ο���
u8 receive_data;//�������ڽ��յ�������
extern s16 Stop_Flag1;

float angle, angle_dot, f_angle, f_angle_dot;//6050�ɼ��Ƕȣ�6050�ɼ����ٶȣ������������˲���ĽǶȣ������������˲���Ľ��ٶ�
void delay(u32 count);//��ʱ��������
void LEDGPIO_init(void);//led���ź�������
void BEEP_init(void);//��������������
int main(void)
{
 
	u16 i=0;
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);//�����жϷ���Ϊ2����ռʽ���ȼ�Ϊ2λ����ӦҲΪ2λ
	delay(0x888888);//�ʵ�����ʱʹ�������ȶ�
	LEDGPIO_init();//��ʼ��led������
	
	ST188_init();	 //��ʼ��st188������
	usart1_init(); //��ʼ������1
	BEEP_init();//��������ʼ��	(�����Ƿ������ģ������ڿ�ʼ��һֱ�죩
	iic_init();		 //��ʼ��iic�ӿ�����6050 
	mpu6050_init();//��ʼ��6050������
	timer_init();  //��ʼ����ʱ�������ڿ���
	stepmotor_init();//�������������ʼ��
  
  
  while (1)
  {
//��������ѭ������������ʱ����Ҫ��ӡ�ı����Ҳ���Ӱ��ϵͳ�����ʱ��		
		printf("%f\r\n",angle);
//Time2_flag�ڶ�ʱ��2�ж�����1
		if(Stop_Flag1)
		{
   if(Time2_flag )
   {	   
			Time2_flag=0;
    //ledÿ��50*10ms״̬�ı�һ��		 
  	  if(i++ >= 50)
  	  { 
				printf("%d\r\n",(int)angle);
  	    i = 0;
  	    GPIOE->ODR ^= (1 << 0); //LED
      }
   }  						   
  }  
}
}



//�򵥵���ʱ����ʵ��
void delay(u32 count)
{
  for(; count != 0; count--);
}

//��ʼ���õ���ָʾ��
void LEDGPIO_init()
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_SetBits(GPIOE,GPIO_Pin_0);
	GPIO_SetBits(GPIOE,GPIO_Pin_10);

}

//��ʼ��������
void BEEP_init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	 //ʹ��GPIOB�˿�ʱ��
 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				 //BEEP-->PB.0 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		 //
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //�ٶ�Ϊ50MHz
 GPIO_Init(GPIOE, &GPIO_InitStructure);	 //���ݲ�����ʼ��GPIOB.0
 GPIO_SetBits(GPIOE,GPIO_Pin_10);//���0���رշ��������

}




