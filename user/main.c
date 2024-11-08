/******************************************************************************
文    件：     main.c 
作    者：     Subolt
Q   Q  :  		1512195337
日    期： 		2017/3/22
软件版本： 		 MDK5.23
下载链接：     http://pan.baidu.com/s/1bo0Wu7d
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
//此处变量为手动校准值，根据传感器静止状下的偏差进行设定，消除偏差
int GX_OFFSET =0x01;
int GY_OFFSET =0x01;
int AX_OFFSET =0x01;
int AY_OFFSET =0x01;
int AZ_OFFSET =0x01;


extern float rout;//电机输出的一个中间变量
extern s16 turn_need_l;//人为给定的左边电机速度
extern s16 turn_need_r;//人为给定的右边电机速度
extern s16 speed_need;//人为给定的整体前进速度速度
u8 Time2_flag;//定时器2进入的人为给定标志量，用于主函数作为时间参考量
u8 receive_data;//蓝牙串口接收到的数据
extern s16 Stop_Flag1;

float angle, angle_dot, f_angle, f_angle_dot;//6050采集角度，6050采集角速度，经过卡尔曼滤波后的角度，经过卡尔曼滤波后的角速度
void delay(u32 count);//延时函数声明
void LEDGPIO_init(void);//led引脚函数声明
void BEEP_init(void);//蜂鸣器函数声明
int main(void)
{
 
	u16 i=0;
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);//配置中断分组为2，先占式优先级为2位，响应也为2位
	delay(0x888888);//适当的延时使得外设稳定
	LEDGPIO_init();//初始化led的引脚
	
	ST188_init();	 //初始化st188传感器
	usart1_init(); //初始化串口1
	BEEP_init();//蜂鸣器初始化	(本来是放在最后的）（放在开始就一直响）
	iic_init();		 //初始化iic接口用于6050 
	mpu6050_init();//初始化6050传感器
	timer_init();  //初始化定时器，用于控制
	stepmotor_init();//步进电机驱动初始化
  
  
  while (1)
  {
//可以在主循环里面加入调试时候需要打印的变量且不会影响系统整体的时序		
		printf("%f\r\n",angle);
//Time2_flag在定时器2中断中置1
		if(Stop_Flag1)
		{
   if(Time2_flag )
   {	   
			Time2_flag=0;
    //led每隔50*10ms状态改变一次		 
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



//简单的延时函数实现
void delay(u32 count)
{
  for(; count != 0; count--);
}

//初始化用到的指示灯
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

//初始化蜂鸣器
void BEEP_init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	 //使能GPIOB端口时钟
 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				 //BEEP-->PB.0 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		 //
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //速度为50MHz
 GPIO_Init(GPIOE, &GPIO_InitStructure);	 //根据参数初始化GPIOB.0
 GPIO_SetBits(GPIOE,GPIO_Pin_10);//输出0，关闭蜂鸣器输出

}




