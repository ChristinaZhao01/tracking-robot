#include "stm32f10x.h"
#include "calculate.h"
#include "math.h"


s16 speed_l = 0;//运算后给定得左边速度
s16 speed_r = 0;//运算后给定得右边速度
s16 speed_need = 0;//人为给定得速度
s16 turn_need_r = 0;//右边转向的人为给定速度
s16 turn_need_l = 0;//左边转向的人为给定速度
s16 speed_out = 0;//滤波后的速度，也就是当作当前的实际速度
u8 stop = 0;//蓝牙遥控时候用到的停车变量
u8 previous_cmd;//记录前一个遥控信号，目的是确定当前要给的控制

float rout;     //pid计算输出变量          
float set_point= 0;//设定角度，也就是平衡位置的角度
float now_error;//当前误差
float proportion = 370;   //比例系数  370
float integral2 = 0.03;   //积分系数
float derivative = 1;     //直立微分
float derivative2 = 1.5;    //速度微分
double position;          //位移
double speed;             //速度


extern float f_angle, f_angle_dot;//卡尔曼滤波后的角度，角速度
#define FILTER_COUNT  20  //滑动窗滤波窗口宽度
#define MAX_SPEED  10000   //限定最大速度
s16 speed_buf[FILTER_COUNT]={0};//用于速度滤波的缓冲数组，滑窗滤波，FILTER_COUNT个数据做平均
/*************************************************

名称：void receive_parameter(u8 cmd)
功能：处理串口接收到的遥控数据，可以不用管这个函数，
我们先且用不到，之所以给出是为了大家比赛完了可以做蓝牙遥控
的小车来用。
输入参数：无
输出参数：无
返回值：  无
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
名称：void speed_filter(void)
功能：速度滤波，滑窗滤波算法
输入参数：无
输出参数：无
返回值：  无
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

名称：void pid(float angle, float angle_dot)
功能：PID运算
输入参数：
float angle     倾斜角度
float angle_dot 倾斜角速度
输出参数：无
返回值：  无
**************************************************/
void pid(float angle, float angle_dot)
{
  u32 temp;
  u16 sl, sr;

  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  now_error = set_point - angle;

  speed_filter();//对速度做一个20窗口的划窗滤波，得到比较稳定的速度值
//该出对速度进行低通滤波，防止速度过快变化影响小车的平衡
  speed *= 0.7;
  speed += speed_out * 0.3;//speed_out是当前实际速度，权重为0.3
	

	
  position += speed;//此处通过求和计算出小车当前的位移，通过PID控制使得位移为0
  position += speed_need;//此处每隔10ms都在位移上加一个值，体现出来的便是速度
  
	//该出对位移进行限制幅度，防止超出范围
  if(position < -85000) 
  {
    position = -85000;
  }
  else if(position > 85000) 
  {
    position =  85000;
  }
	
	//此处计算得出使得小车保持直立所需要的电机输出速度
	//说明：proportion和derivative分别是使得小车保持直立的比例和微分
	//integral2这里相当于速度的比例（这里速度给的是位移），derivative2在这里是速度的微分，改善小车运行过程中前后晃动
	rout = proportion * now_error-derivative * angle_dot+ position * integral2+ derivative2 * speed;   //串级PID
	//叠加转向速度，最终分别给定两个电机的输出
  speed_l = rout + turn_need_l;
  speed_r = rout + turn_need_r;
	
	//加入倾角范围，超出范围电机输出给0（实际上只是很长时间给一个脉冲，表现出来好像转速为0）
	if(f_angle>40||f_angle<-40)
	{ 
		speed_l=0;
		speed_r=0;
	}
	
	//此处对电机转速进行限制，防止超出范围
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
  //此处通过判断电机输出值的正负通过操作步进电机驱动芯片io高低电平来给定电机方向
  if(speed_l > 0)
  {
    GPIO_ResetBits(GPIOD, GPIO_Pin_7);  //设定步进电机转动方向
	sl = speed_l;
  }
  else	
  {
    GPIO_SetBits(GPIOD, GPIO_Pin_7);//设定步进电机转动方向
	sl = speed_l * (-1);//由于最终去配置定时器输出pwm频率是没有负数的，所以将其转换为正值
  }
//此处通过判断电机输出值的正负通过操作步进电机驱动芯片io高低电平来给定电机方向
  if(speed_r > 0)
  {
    GPIO_SetBits(GPIOE, GPIO_Pin_11);  //设定步进电机转动方向
	sr = speed_r;
  }
  else	
  {
    GPIO_ResetBits(GPIOE, GPIO_Pin_11);//设定步进电机转动方向
	sr = speed_r * (-1);//由于最终去配置定时器输出pwm频率是没有负数的，所以将其转换为正值
  }
  
	//由于PID计算出的速度越大，在配置定时器是装载值越小，装载值范围为16位，也就是0X0000-0XFFFF
  temp = 1000000 / sl;//定义变量temp进行范围转换，这个1000000是根据步进电机在最大转速掉步给的一个估计值
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
	//配置定时器输出PWM频率
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = sl - 1;  // (Period + 1) * (Prescaler + 1) / 72M = x ms，也就是x毫秒给一个脉冲  
  TIM_TimeBaseStructure.TIM_Prescaler = 30 - 1;  //时钟预分频30.也就是72mhz/30=2.4mhz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;//时钟分割0
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数 

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWM模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//比较输出高
  TIM_OCInitStructure.TIM_Pulse = sl >> 1;//此处右移一位表示除以2，也就是设定占空比为50%

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
  TIM_OCInitStructure.TIM_Pulse = sr >> 1;//此处右移一位表示除以2，也就是设定占空比为50%

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);  
}
