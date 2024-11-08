/******************************************************************************
文    件：     st188.c 
作    者：     Subolt
软件版本： 		 MDK5.23
Q   Q  :  		1512195337
日    期： 		2017/3/22
******************************************************************************/
#include "st188.h"
#include "stm32f10x_conf.h"
#include "usart.h"
s16 speed_straightaway=4000;    //给定的直线速度4000	
   
s16 Stop_Flag1=0;
extern float integral2;        
extern float derivative2;
float integra=0.030; //积分系数（0.030）
float deriva=1.53;//微分系数   1.53

extern s16 speed_need;  //速度的变化
extern s16 turn_need_r; //左轮速度的变化
extern s16 turn_need_l; //右轮速度的变化

extern u8 ERROR_SENSOR_FLAG;//检测传感器是否数据错误
void delay1(u32 time);

extern s16 Stop_Flag;

u8 Open_Stop_FlagCheck=0;   //是否开始检测我们停车标志
u8 Open_Stop_FlagCheck1=0;  
u8 Open_Stop_FlagCheck2=0;

u8 SlowFlag=0;
u16 turn_left_flag=0;   //向左转标记
u16 turn_right_flag=0;  //向右转标记

u32 Timer2_counter2=0; 
u32 Timer2_counter=0;   //定时器2计数变量

/*************************************************
名称：void ST188_init()
功能：初始化ST188传感器
说明：使用的引脚 E8 E9 E10
输入参数：无
输出参数：无
返回值：  无
**************************************************/


/**初始化红外传感器模块连接的引脚**/
void ST188_init()
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    //设置为输入上拉模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);     //初始化PE1
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_2; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //设置为输入上拉模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);    //初始化PA2,PA3
	
}



/*******************************************************************************
名称：void ST188_control(void)
功能：使用ST188实现循迹功能
说明：具体8转向控制需要大家自己给控制量
speed_need是前进或者后退的速度，给正值是前进，给
负值是后退
turn_need_r转向右边轮子速度
turn_need_l转向左边轮子速度
integral2 和derivative2是控制速度和稳定的积分系数和微分系数，建议不要随便改动，
改动过大可能导致直立不稳，甚至不能直立。
输入参数：ST188数据
输出参数：无
返回值：  无
*******************************************************************************/
void ST188_control(u8 temp)   //在定时器中断服务函数中执行
{
	static u8 Prestate=0;
	if(Timer2_counter<25)//1秒用于启动   100
	{
		integral2 = 0.01;        
		derivative2 = 0.9; 	       
		speed_need = 0;
		turn_need_r = 0;
		turn_need_l = 0;
	}
	else if(Timer2_counter<600)//停5秒，用于放置乒乓球
	{
		integral2 = integra;        //integra
		derivative2 = deriva ; 	          //deriva（该值标准值为1.45，该值越小冲刺距离越远）
		speed_need =  0;      
		turn_need_r = 0;
		turn_need_l = 0;
	}
//	else if(Timer2_counter<550)
//	{
//		integral2 = integra;        
//		derivative2 = deriva ; 	       
//		speed_need = 0;
//		turn_need_r = 0;
//		turn_need_l = 0;
//	}
	else if(!Open_Stop_FlagCheck2||SlowFlag)
	{
		if(SlowFlag==1)
		{
			// ***************************************************
			// ***************************************************
			speed_straightaway-30;   //直行速度缓慢减少
		}
			switch(temp)  //temp在
			{
				case 0:	//全是黑色，那么要保持直行
								integral2 = integra;        
								derivative2 = deriva; 	       
								speed_need = speed_straightaway;  //直行速度
								turn_need_r = 0;
								turn_need_l = 0;
								Prestate=0;
									      if(Open_Stop_FlagCheck)
												{
													Stop_Flag=1;     
													SlowFlag=1;     //将SlowFlag置1
												}
													if(Timer2_counter2>30)     //Stop_Flag持续0.3s后，则取消减速标志位
													{
														SlowFlag=0;
													}
								break;
				case 1://左中黑色，那么小左转
								integral2 = integra;        
								derivative2 = deriva;		        
								speed_need = speed_straightaway;
								turn_need_r = 650;    //1600   750
								turn_need_l = -650; 
								turn_left_flag++;
								Prestate=1;
								break;
				case 2:	//两边是黑色，那么保持直行
								integral2 = integra;        
								derivative2 = deriva; 	       
								speed_need = speed_straightaway;
								turn_need_r = 0;
								turn_need_l = 0;
								Prestate=2;		        
								break;
				case 3://左边黑色，进行大左转
								integral2 = integra;        
								derivative2 = deriva; 	       
								speed_need = speed_straightaway;
								turn_need_r = 1800;   //2100    1800
								turn_need_l = -1800;
								turn_left_flag++;
								Prestate=3;		        
				        //GPIO_ResetBits(GPIOE,GPIO_Pin_10);  //蜂鸣器响一下
								break;
				case 4://右中黑，进行小右转
								integral2 = integra;        
								derivative2 = deriva; 	       
								speed_need = speed_straightaway;
								turn_need_r = -650;  //750
								turn_need_l = 650;
								//turn_right_flag++;
								Prestate=4;		        
								break;
				case 5://中间是黑色，那么保持直行
								integral2 = integra;        
								derivative2 = deriva; 	       
								speed_need = speed_straightaway;
								turn_need_r = 0;
								turn_need_l = 0;
								Prestate=5;		        
								break;
				case 6://右边是黑色，进行大右转
								integral2 = integra;        
								derivative2 = deriva; 	       
								speed_need = speed_straightaway;
								turn_need_r = -1800;//原本是1400（有点超前）1600（挺好
								turn_need_l = 1800;
								turn_right_flag++;
								Prestate=6;		        
								break;
				
				case 7://全是白色，这时候进行丢线处理，可根据之前的状态来判断当前需要执行的动作
					switch(Prestate)
					{
					case 0:	//全是黑色，那么要保持直行
									integral2 = integra;        
									derivative2 = deriva; 	       
									speed_need = speed_straightaway;
									turn_need_r = 0;
									turn_need_l = 0;
									break;
					case 1://左中黑色，那么小左转
									integral2 = integra;        
									derivative2 = deriva;		       
									speed_need = speed_straightaway;
									turn_need_r = 650;
									turn_need_l = -650;
									break;
					case 2:	//两边是黑色，那么保持直行
									integral2 = integra;        
									derivative2 = deriva; 	       
									speed_need = speed_straightaway;
									turn_need_r = 0;
									turn_need_l = 0;        
									break;
					case 3://左边黑色，进行大大左转
									integral2 = integra;        
									derivative2 = deriva; 	       
									speed_need = speed_straightaway;
									turn_need_r = 1800;      //yuanlai 1800
									turn_need_l = -1800;       
									break;
					case 4://右中黑，进行小右转
									integral2 = integra;        
									derivative2 = deriva; 	       
									speed_need = speed_straightaway;
									turn_need_r = -650;    //750
									turn_need_l = 650;        
									break;
					case 5://中间是黑色，那么保持直行
									integral2 = integra;        
									derivative2 = deriva; 	       
									speed_need = speed_straightaway;
									turn_need_r = 0;
									turn_need_l = 0;        
									break;
					case 6://右边是黑色，进行大大右转
									integral2 = integra;        
									derivative2 = deriva; 	       
									speed_need = speed_straightaway;
									turn_need_r = -1800; 
									turn_need_l = 1800 ;        
									break;
						default:break;
					}
				default:break;
			}
	}
	else //入库操作
	{
		if(speed_straightaway < 2100)//(这里本来是小于现在改成大于）
		{
					speed_straightaway = 2100;
					if(Stop_Flag&&Timer2_counter2<140)//缓慢减速（100的时候不能到框里，只是踩线）add 110   140 a little chao 130 youdianb ugou
							{
							speed_straightaway-=20;  //(100)
							integral2 = integra;        
							derivative2 = deriva;
							speed_need =speed_straightaway;
							turn_need_r = 0;
							turn_need_l = 0;
							}
							else if(Stop_Flag&&Timer2_counter2<660)//停车4秒
							{
								GPIO_ResetBits(GPIOE,GPIO_Pin_10);
								Stop_Flag1=1;
								speed_straightaway=1000;
								integral2 = integra;        
								derivative2 = deriva;
								speed_need =0;
								turn_need_r = 0;
								turn_need_l = 0;
							}
							else if(Timer2_counter2>660&&Timer2_counter2<700)//直行一小段（710入库擦侧边）【710~790】
							{
								Stop_Flag1=0;
								GPIO_SetBits(GPIOE,GPIO_Pin_10);    //
								integral2 = integra;        
								derivative2 = deriva;
								speed_need =speed_straightaway;
								turn_need_r = 0;
								turn_need_l = 0;
							}
							else if(Timer2_counter2>700&&Timer2_counter2<780)//将车头转向车库   790   all jian10
							{
								integral2 = integra;        
								derivative2 = deriva;
								speed_need =speed_straightaway;
								if(turn_right_flag>turn_left_flag)
								{
									turn_need_r = 500;
									turn_need_l = -500;
								}
								else
								{
									turn_need_r = -500;
									turn_need_l = 500;
								}
							}
							else if(Timer2_counter2>780&&Timer2_counter2<785)//直行（900的时候撞墙了,830擦线）【790~830】zaijian 5
							{
								integral2 = integra;        
								derivative2 = deriva;
								speed_need =speed_straightaway;
								turn_need_r = 0;
								turn_need_l = 0;
							}
							else if(Timer2_counter2>785&&Timer2_counter2<920)//停车
							{
								Stop_Flag1=1;
								GPIO_ResetBits(GPIOE,GPIO_Pin_10);
								integral2 = integra;        
								derivative2 = deriva;
								speed_need =0;
								turn_need_r = 0;
								turn_need_l = 0;
							}
							else if(Timer2_counter2>920)//停车，关闭蜂鸣器
							{
								Stop_Flag1=1;
								GPIO_SetBits(GPIOE,GPIO_Pin_10);
								integral2 = integra;        
								derivative2 = deriva;
								speed_need =0;
								turn_need_r = 0;
								turn_need_l = 0;
							}
		}
		else
		{
					if(Stop_Flag&&Timer2_counter2<140)//缓慢减速
					{
						speed_straightaway-=20;
					integral2 = integra;        
					derivative2 = deriva;
					speed_need =speed_straightaway;
					turn_need_r = 0;
					turn_need_l = 0;
					}
					else if(Stop_Flag&&Timer2_counter2<660)//停车4秒
					{
						GPIO_ResetBits(GPIOE,GPIO_Pin_10);
						Stop_Flag1=1;
						speed_straightaway=1000;
					integral2 = integra;        
					derivative2 = deriva;
					speed_need =0;
					turn_need_r = 0;
					turn_need_l = 0;
					}
					else if(Timer2_counter2>660&&Timer2_counter2<700)//直行一小段    jian 30
					{
						Stop_Flag1=0;
						GPIO_SetBits(GPIOE,GPIO_Pin_10);
						integral2 = integra;        
						derivative2 = deriva;
						speed_need =speed_straightaway;
						turn_need_r = 0;
						turn_need_l = 0;
					}
					else if(Timer2_counter2>700&&Timer2_counter2<780)//将车头转向车库
					{
						integral2 = integra;        
						derivative2 = deriva;
						speed_need =speed_straightaway;
					if(turn_right_flag>turn_left_flag)
					{
						turn_need_r = 500;
						turn_need_l = -500;
					}
					else
					{
						turn_need_r = -500;
						turn_need_l = 500;
					}
					}
					else if(Timer2_counter2>780&&Timer2_counter2<785)//直行
					{
						integral2 = integra;        
						derivative2 = deriva;
						speed_need =speed_straightaway;
						turn_need_r = 0;
						turn_need_l = 0;
					}
					else if(Timer2_counter2>785&&Timer2_counter2<920)//停车
					{
						Stop_Flag1=1;
						GPIO_ResetBits(GPIOE,GPIO_Pin_10);
						integral2 = integra;        
						derivative2 = deriva;
						speed_need =0;
						turn_need_r = 0;
						turn_need_l = 0;
					}
					else if(Timer2_counter2>920)//停车，关闭蜂鸣器
					{
						Stop_Flag1=1;
						GPIO_SetBits(GPIOE,GPIO_Pin_10);
						integral2 = integra;        
						derivative2 = deriva;
						speed_need =0;
						turn_need_r = 0;
						turn_need_l = 0;
					}
		}
	}
	}

/*************************************************
名称：void ST188_DATA_CHECK()
功能：判断传感器数据是否是错误数据
输入参数：当前角度.
输出参数：无
返回值：  无效数据为1，有效数据为0
**************************************************/

u8 ST188_DATA_CHECK(float Angle)
{
	if((Angle>-3)&&(Angle<3))return 0;
		else return 1;
}


void delay1(u32 time)
{
	u16 i=0;
	while(time--)
	{
		i=12000;
		while(i--)
		{

		}
	}
}

