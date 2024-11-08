/******************************************************************************
��    ����     timer.c 
��    �ߣ�     Subolt
����汾�� 		 MDK5.23
Q   Q  :  		1512195337
��    �ڣ� 		2017/3/22
******************************************************************************/
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "timer.h"
#include "mpu6050.h"
#include "st188.h"
#include "filter.h"
#include "calculate.h"
#include <math.h>


//�˴������Ƿ�ʹ������ң�أ�������ʹ�ô�����ѭ�����Ʒ���
//#define Bluetooth_Control
extern int GX_OFFSET;
extern int GY_OFFSET;
extern int AX_OFFSET;
extern int AY_OFFSET;
extern int AZ_OFFSET;
extern u8 receive_data;//�������ݽ��ձ���
extern s16 gx, gy, gz, ax ,ay, az, temperature;//mpu6050ԭʼ3����ٶȣ�3����ٶȣ��¶ȵȱ���
extern float angle, angle_dot, f_angle, f_angle_dot;//�Ƕȣ����ٶȡ��˲���ĽǶȣ����ٶ�
extern u8 Open_Stop_FlagCheck;//�Ƿ���ͣ����־���ı���
extern u8 Open_Stop_FlagCheck1;
extern u8 Open_Stop_FlagCheck2;
s16 Stop_Flag=0;//ͣ����־

u8 ERROR_SENSOR_FLAG=0;//�ж������������źű�־����
u32 Timer2_counter1=0;
extern u32 Timer2_counter2;
extern u32 Timer2_counter;
/*************************************************
���ƣ�timer_init(void)
���ܣ�timer2�����ʼ�����ж� ��ʱʱ�䣩
�����������
�����������
����ֵ��  ��
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
  TIM_TimeBaseStructure.TIM_Prescaler = 72-1; //ʱ��Ԥ��Ƶ�� 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;//ʱ�ӷָ�
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���ģʽ  

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);

  /* TIM IT enable */
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

}
void MY_TIM2_IRQHandler(void)
{
//ÿ10ms��һ���ж�  
//�շ���ʱ����ͣ����־��⣬x�������⣬��ֹ����ʱ����ͣ����־��ǰͣ��
		Timer2_counter++;
	  Timer2_counter1++;
	  if(Timer2_counter>1375)  //
		{
			Open_Stop_FlagCheck=1;//��λͣ������־�����Խ���ͣ�����
		}
	  if(Timer2_counter1>1575)                                                                                 // yuan shu ju 1800
		{
			Open_Stop_FlagCheck1=1;//��λͣ������־�����Խ���ͣ�����
			Timer2_counter1=0;    //�Լ���ֵ����
		}
		if(Stop_Flag==1)   
		{
			Timer2_counter2++;       //
			Open_Stop_FlagCheck2=1;     
		}
		mpu6050_get_data(&gx, &gy, &gz, &ax, &ay , &az, &temperature);//��ȡmpu6050������
		acc_filter();//�Լ��ٶȽ����˲�

	  gx -= GX_OFFSET;//x����ٶ����ֶ�У��
	  ax -= AX_OFFSET;//x����ٶ����ֶ�У�� 
	  ay -=	AY_OFFSET;//y����ٶ����ֶ�У��
	  az -=	AZ_OFFSET;//z����ٶ����ֶ�У��
  //���߶�ת��
	  angle_dot = gx * GYRO_SCALE;  //+-2000  0.060975 ��/LSB   GYRO_SCALE 0.06097609f
	//ͨ���������ٶȼ����z�����������ٶȷ���нǡ�
    angle = atan2(ay,az); 	
	//�߶�ת��
    angle = angle * 57.295780;    //180/pi
		angle=angle-Bias;
  //ʹ�ÿ������˲�ȥ���˶��������ó��Ƕ����Ź���ֵ����Ϊʵ�ʽǶȣ����ں������
	  kalman_filter(angle, angle_dot, &f_angle, &f_angle_dot);

  //��������ʹ��ң����ʹ�ô��ڽ��յ���������п��ƣ�����ʹ��st188������
#ifdef Bluetooth_Control
	  receive_parameter(receive_data);//����ͨ�����ڷ��͵Ŀ���ָ��
#else   
	//����ƽ��С����������ƽ���ǻ�ǰ�������������߶Ⱥ��ߺ��ͣ�̫�߻���̫�ͼ�⵽���źŶ�������Ч�źţ��ʽ������ε�
	//�˴���־��st188.c�н�����Ϊ��������
	  ERROR_SENSOR_FLAG=ST188_DATA_CHECK(angle);//�˴��ԽǶȼ���жϴ������ź��Ƿ����
		ST188_control(ST188DATA);//ֱ��st188����
#endif
  //���뾭���������˲���ĽǶȺͽ��ٶȣ�����PID���㣬�ó���������ֵ
	  pid(f_angle, f_angle_dot);
}
