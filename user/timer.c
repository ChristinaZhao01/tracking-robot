/******************************************************************************
文    件：     timer.c 
作    者：     Subolt
软件版本： 		 MDK5.23
Q   Q  :  		1512195337
日    期： 		2017/3/22
******************************************************************************/
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "timer.h"
#include "mpu6050.h"
#include "st188.h"
#include "filter.h"
#include "calculate.h"
#include <math.h>


//此处定义是否使用蓝牙遥控，屏蔽则使用传感器循迹控制方案
//#define Bluetooth_Control
extern int GX_OFFSET;
extern int GY_OFFSET;
extern int AX_OFFSET;
extern int AY_OFFSET;
extern int AZ_OFFSET;
extern u8 receive_data;//蓝牙数据接收变量
extern s16 gx, gy, gz, ax ,ay, az, temperature;//mpu6050原始3轴角速度，3轴加速度，温度等变量
extern float angle, angle_dot, f_angle, f_angle_dot;//角度，角速度。滤波后的角度，角速度
extern u8 Open_Stop_FlagCheck;//是否开启停车标志检测的变量
extern u8 Open_Stop_FlagCheck1;
extern u8 Open_Stop_FlagCheck2;
s16 Stop_Flag=0;//停车标志

u8 ERROR_SENSOR_FLAG=0;//判定传感器错误信号标志变量
u32 Timer2_counter1=0;
extern u32 Timer2_counter2;
extern u32 Timer2_counter;
/*************************************************
名称：timer_init(void)
功能：timer2外设初始化（中断 定时时间）
输入参数：无
输出参数：无
返回值：  无
**************************************************/
void timer_init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;   
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure); 

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);     
 
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 10000-1;   //10ms
  TIM_TimeBaseStructure.TIM_Prescaler = 72-1; //时钟预分频数 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;//时钟分割
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式  

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);

  /* TIM IT enable */
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

}
void MY_TIM2_IRQHandler(void)
{
//每10ms进一次中断  
//刚发车时屏蔽停车标志检测，x秒后开启检测，防止发车时误检测停车标志提前停车
		Timer2_counter++;
	  Timer2_counter1++;
	  if(Timer2_counter>1375)  //
		{
			Open_Stop_FlagCheck=1;//置位停车检测标志，可以进行停车检测
		}
	  if(Timer2_counter1>1575)                                                                                 // yuan shu ju 1800
		{
			Open_Stop_FlagCheck1=1;//置位停车检测标志，可以进行停车检测
			Timer2_counter1=0;    //对计数值清零
		}
		if(Stop_Flag==1)   
		{
			Timer2_counter2++;       //
			Open_Stop_FlagCheck2=1;     
		}
		mpu6050_get_data(&gx, &gy, &gz, &ax, &ay , &az, &temperature);//获取mpu6050的数据
		acc_filter();//对加速度进行滤波

	  gx -= GX_OFFSET;//x轴角速度作手动校正
	  ax -= AX_OFFSET;//x轴加速度作手动校正 
	  ay -=	AY_OFFSET;//y轴加速度作手动校正
	  az -=	AZ_OFFSET;//z轴加速度作手动校正
  //作尺度转换
	  angle_dot = gx * GYRO_SCALE;  //+-2000  0.060975 °/LSB   GYRO_SCALE 0.06097609f
	//通过重力加速度计算出z轴与重力加速度方向夹角。
    angle = atan2(ay,az); 	
	//尺度转换
    angle = angle * 57.295780;    //180/pi
		angle=angle-Bias;
  //使用卡尔曼滤波去除运动噪声，得出角度最优估计值，作为实际角度，用于后面控制
	  kalman_filter(angle, angle_dot, &f_angle, &f_angle_dot);

  //若定义了使用遥控则使用串口接收到的命令进行控制，否则使用st188传感器
#ifdef Bluetooth_Control
	  receive_parameter(receive_data);//蓝牙通过串口发送的控制指令
#else   
	//由于平衡小车保持自身平衡是会前后俯仰，传感器高度忽高忽低，太高或者太低监测到的信号都不是有效信号，故将其屏蔽掉
	//此处标志在st188.c中将会作为控制依据
	  ERROR_SENSOR_FLAG=ST188_DATA_CHECK(angle);//此处对角度检测判断传感器信号是否错误
		ST188_control(ST188DATA);//直行st188控制
#endif
  //传入经过卡尔曼滤波后的角度和角速度，进行PID运算，得出电机的输出值
	  pid(f_angle, f_angle_dot);
}
