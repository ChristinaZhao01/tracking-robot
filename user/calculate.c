#include "stm32f10x.h"
#include "calculate.h"
#include "math.h"


s16 speed_l = 0;//��������������ٶ�
s16 speed_r = 0;//�����������ұ��ٶ�
s16 speed_need = 0;//��Ϊ�������ٶ�
s16 turn_need_r = 0;//�ұ�ת�����Ϊ�����ٶ�
s16 turn_need_l = 0;//���ת�����Ϊ�����ٶ�
s16 speed_out = 0;//�˲�����ٶȣ�Ҳ���ǵ�����ǰ��ʵ���ٶ�
u8 stop = 0;//����ң��ʱ���õ���ͣ������
u8 previous_cmd;//��¼ǰһ��ң���źţ�Ŀ����ȷ����ǰҪ���Ŀ���

float rout;     //pid�����������          
float set_point= 0;//�趨�Ƕȣ�Ҳ����ƽ��λ�õĽǶ�
float now_error;//��ǰ���
float proportion = 370;   //����ϵ��  370
float integral2 = 0.03;   //����ϵ��
float derivative = 1;     //ֱ��΢��
float derivative2 = 1.5;    //�ٶ�΢��
double position;          //λ��
double speed;             //�ٶ�


extern float f_angle, f_angle_dot;//�������˲���ĽǶȣ����ٶ�
#define FILTER_COUNT  20  //�������˲����ڿ��
#define MAX_SPEED  10000   //�޶�����ٶ�
s16 speed_buf[FILTER_COUNT]={0};//�����ٶ��˲��Ļ������飬�����˲���FILTER_COUNT��������ƽ��
/*************************************************

���ƣ�void receive_parameter(u8 cmd)
���ܣ������ڽ��յ���ң�����ݣ����Բ��ù����������
���������ò�����֮���Ը�����Ϊ�˴�ұ������˿���������ң��
��С�����á�
�����������
�����������
����ֵ��  ��
**************************************************/
void receive_parameter(u8 cmd)
{
  switch(cmd)
  {
    case 'u':  // forward
	  if(previous_cmd == 'd')
	  {
	    integral2 = 0.01;        
			derivative2 = 1;		       
	    speed_need = 0;
			turn_need_r = 0;
			turn_need_l = 0;
			stop = 200;  
	  }
	  else 
	  {
	    if(stop == 0)
		{
		  integral2 = 0.05;        
			derivative2 = 1.5;		       
			speed_need = 10000;
			turn_need_r = 0;
			turn_need_l = 0;
		}
		else 
		{
		  stop--;
		  integral2 += 0.0002;        
			derivative2 += 0.0025;		       
			speed_need += 50;
		}
	  }
    break;
    case 'd': // back
	  if(previous_cmd == 'u')
	  {
	    integral2 = 0.01;        
			derivative2 = 1;		       
	    speed_need = 0;
			turn_need_r = 0;
			turn_need_l = 0;
		stop = 200;  
	  }
	  else 
	  {
	    if(stop == 0)
		{
		  integral2 = 0.05;        
			derivative2 = 1.5;		       
			speed_need = -10000;
			turn_need_r = 0;
			turn_need_l = 0;
		}
		else 
		{
			stop--;
			integral2 += 0.0002;        
			derivative2 += 0.0025;		       
			speed_need += -50;
		}
	  }
			break;
	case 'l':  // turn left
	    integral2 = 0.03;        
      derivative2 = 1.5;       
	    speed_need = 0;
      turn_need_r = -500;
      turn_need_l = 500;
			stop = 0;
			break;
	case 'r':  // turn right
	    integral2 = 0.03;        
      derivative2 = 1.5; 	       
	    speed_need = 0;
      turn_need_r = 500;
      turn_need_l = -500;
			stop = 0;
			break;
	case 's':  // stop
			integral2 = 0.01;
			derivative2 = 1;
      speed_need = 0;
      turn_need_r = 0;
      turn_need_l = 0;
			stop = 0;
			break;
  }
  previous_cmd = cmd;
}
/*************************************************
���ƣ�void speed_filter(void)
���ܣ��ٶ��˲��������˲��㷨
�����������
�����������
����ֵ��  ��
**************************************************/
void speed_filter(void)
{
  u8 i;
  s32 speed_sum = 0; 

  for(i = 1 ; i < FILTER_COUNT; i++)
  {
    speed_buf[i - 1] = speed_buf[i];
  }

  speed_buf[FILTER_COUNT - 1] = ((speed_l + speed_r) / 2);

  for(i = 0 ; i < FILTER_COUNT; i++)
  {
    speed_sum += speed_buf[i];
  }
   
  speed_out = (s16)(speed_sum / FILTER_COUNT);
}



/*************************************************

���ƣ�void pid(float angle, float angle_dot)
���ܣ�PID����
���������
float angle     ��б�Ƕ�
float angle_dot ��б���ٶ�
�����������
����ֵ��  ��
**************************************************/
void pid(float angle, float angle_dot)
{
  u32 temp;
  u16 sl, sr;

  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  now_error = set_point - angle;

  speed_filter();//���ٶ���һ��20���ڵĻ����˲����õ��Ƚ��ȶ����ٶ�ֵ
//�ó����ٶȽ��е�ͨ�˲�����ֹ�ٶȹ���仯Ӱ��С����ƽ��
  speed *= 0.7;
  speed += speed_out * 0.3;//speed_out�ǵ�ǰʵ���ٶȣ�Ȩ��Ϊ0.3
	

	
  position += speed;//�˴�ͨ����ͼ����С����ǰ��λ�ƣ�ͨ��PID����ʹ��λ��Ϊ0
  position += speed_need;//�˴�ÿ��10ms����λ���ϼ�һ��ֵ�����ֳ����ı����ٶ�
  
	//�ó���λ�ƽ������Ʒ��ȣ���ֹ������Χ
  if(position < -85000) 
  {
    position = -85000;
  }
  else if(position > 85000) 
  {
    position =  85000;
  }
	
	//�˴�����ó�ʹ��С������ֱ������Ҫ�ĵ������ٶ�
	//˵����proportion��derivative�ֱ���ʹ��С������ֱ���ı�����΢��
	//integral2�����൱���ٶȵı����������ٶȸ�����λ�ƣ���derivative2���������ٶȵ�΢�֣�����С�����й�����ǰ��ζ�
	rout = proportion * now_error-derivative * angle_dot+ position * integral2+ derivative2 * speed;   //����PID
	//����ת���ٶȣ����շֱ����������������
  speed_l = rout + turn_need_l;
  speed_r = rout + turn_need_r;
	
	//������Ƿ�Χ��������Χ��������0��ʵ����ֻ�Ǻܳ�ʱ���һ�����壬���ֳ�������ת��Ϊ0��
	if(f_angle>40||f_angle<-40)
	{ 
		speed_l=0;
		speed_r=0;
	}
	
	//�˴��Ե��ת�ٽ������ƣ���ֹ������Χ
  if(speed_l > MAX_SPEED)	
  { 
    speed_l = MAX_SPEED;	
  }
  else if(speed_l < -MAX_SPEED)	
  {
	speed_l = -MAX_SPEED;
  }
  if(speed_r > MAX_SPEED)	
  {
    speed_r = MAX_SPEED;	
  }
  else if(speed_r < -MAX_SPEED)	
  {
	speed_r = -MAX_SPEED;
  }
  //�˴�ͨ���жϵ�����ֵ������ͨ�����������������оƬio�ߵ͵�ƽ�������������
  if(speed_l > 0)
  {
    GPIO_ResetBits(GPIOD, GPIO_Pin_7);  //�趨�������ת������
	sl = speed_l;
  }
  else	
  {
    GPIO_SetBits(GPIOD, GPIO_Pin_7);//�趨�������ת������
	sl = speed_l * (-1);//��������ȥ���ö�ʱ�����pwmƵ����û�и����ģ����Խ���ת��Ϊ��ֵ
  }
//�˴�ͨ���жϵ�����ֵ������ͨ�����������������оƬio�ߵ͵�ƽ�������������
  if(speed_r > 0)
  {
    GPIO_SetBits(GPIOE, GPIO_Pin_11);  //�趨�������ת������
	sr = speed_r;
  }
  else	
  {
    GPIO_ResetBits(GPIOE, GPIO_Pin_11);//�趨�������ת������
	sr = speed_r * (-1);//��������ȥ���ö�ʱ�����pwmƵ����û�и����ģ����Խ���ת��Ϊ��ֵ
  }
  
	//����PID��������ٶ�Խ�������ö�ʱ����װ��ֵԽС��װ��ֵ��ΧΪ16λ��Ҳ����0X0000-0XFFFF
  temp = 1000000 / sl;//�������temp���з�Χת�������1000000�Ǹ��ݲ�����������ת�ٵ�������һ������ֵ
  if(temp > 65535) 
  {
    sl = 65535;
  }
  else           
  {
    sl = (u16)temp;
  }
  	
  temp = 1000000 / sr;
  if(temp > 65535)
  {
    sr = 65535;
  }
  else 
  {
    sr = (u16)temp;
  }	
	//���ö�ʱ�����PWMƵ��
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = sl - 1;  // (Period + 1) * (Prescaler + 1) / 72M = x ms��Ҳ����x�����һ������  
  TIM_TimeBaseStructure.TIM_Prescaler = 30 - 1;  //ʱ��Ԥ��Ƶ30.Ҳ����72mhz/30=2.4mhz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;//ʱ�ӷָ�0
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ��� 

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//�Ƚ������
  TIM_OCInitStructure.TIM_Pulse = sl >> 1;//�˴�����һλ��ʾ����2��Ҳ�����趨ռ�ձ�Ϊ50%

  TIM_OC3Init(TIM4, &TIM_OCInitStructure);  

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = sr - 1;    
  TIM_TimeBaseStructure.TIM_Prescaler = 30  - 1;  
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_Pulse = sr >> 1;//�˴�����һλ��ʾ����2��Ҳ�����趨ռ�ձ�Ϊ50%

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);  
}
