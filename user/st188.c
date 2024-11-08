/******************************************************************************
��    ����     st188.c 
��    �ߣ�     Subolt
����汾�� 		 MDK5.23
Q   Q  :  		1512195337
��    �ڣ� 		2017/3/22
******************************************************************************/
#include "st188.h"
#include "stm32f10x_conf.h"
#include "usart.h"
s16 speed_straightaway=4000;    //������ֱ���ٶ�4000	
   
s16 Stop_Flag1=0;
extern float integral2;        
extern float derivative2;
float integra=0.030; //����ϵ����0.030��
float deriva=1.53;//΢��ϵ��   1.53

extern s16 speed_need;  //�ٶȵı仯
extern s16 turn_need_r; //�����ٶȵı仯
extern s16 turn_need_l; //�����ٶȵı仯

extern u8 ERROR_SENSOR_FLAG;//��⴫�����Ƿ����ݴ���
void delay1(u32 time);

extern s16 Stop_Flag;

u8 Open_Stop_FlagCheck=0;   //�Ƿ�ʼ�������ͣ����־
u8 Open_Stop_FlagCheck1=0;  
u8 Open_Stop_FlagCheck2=0;

u8 SlowFlag=0;
u16 turn_left_flag=0;   //����ת���
u16 turn_right_flag=0;  //����ת���

u32 Timer2_counter2=0; 
u32 Timer2_counter=0;   //��ʱ��2��������

/*************************************************
���ƣ�void ST188_init()
���ܣ���ʼ��ST188������
˵����ʹ�õ����� E8 E9 E10
�����������
�����������
����ֵ��  ��
**************************************************/


/**��ʼ�����⴫����ģ�����ӵ�����**/
void ST188_init()
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    //����Ϊ��������ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);     //��ʼ��PE1
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_2; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //����Ϊ��������ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);    //��ʼ��PA2,PA3
	
}



/*******************************************************************************
���ƣ�void ST188_control(void)
���ܣ�ʹ��ST188ʵ��ѭ������
˵��������8ת�������Ҫ����Լ���������
speed_need��ǰ�����ߺ��˵��ٶȣ�����ֵ��ǰ������
��ֵ�Ǻ���
turn_need_rת���ұ������ٶ�
turn_need_lת����������ٶ�
integral2 ��derivative2�ǿ����ٶȺ��ȶ��Ļ���ϵ����΢��ϵ�������鲻Ҫ���Ķ���
�Ķ�������ܵ���ֱ�����ȣ���������ֱ����
���������ST188����
�����������
����ֵ��  ��
*******************************************************************************/
void ST188_control(u8 temp)   //�ڶ�ʱ���жϷ�������ִ��
{
	static u8 Prestate=0;
	if(Timer2_counter<25)//1����������   100
	{
		integral2 = 0.01;        
		derivative2 = 0.9; 	       
		speed_need = 0;
		turn_need_r = 0;
		turn_need_l = 0;
	}
	else if(Timer2_counter<600)//ͣ5�룬���ڷ���ƹ����
	{
		integral2 = integra;        //integra
		derivative2 = deriva ; 	          //deriva����ֵ��׼ֵΪ1.45����ֵԽС��̾���ԽԶ��
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
			speed_straightaway-30;   //ֱ���ٶȻ�������
		}
			switch(temp)  //temp��
			{
				case 0:	//ȫ�Ǻ�ɫ����ôҪ����ֱ��
								integral2 = integra;        
								derivative2 = deriva; 	       
								speed_need = speed_straightaway;  //ֱ���ٶ�
								turn_need_r = 0;
								turn_need_l = 0;
								Prestate=0;
									      if(Open_Stop_FlagCheck)
												{
													Stop_Flag=1;     
													SlowFlag=1;     //��SlowFlag��1
												}
													if(Timer2_counter2>30)     //Stop_Flag����0.3s����ȡ�����ٱ�־λ
													{
														SlowFlag=0;
													}
								break;
				case 1://���к�ɫ����ôС��ת
								integral2 = integra;        
								derivative2 = deriva;		        
								speed_need = speed_straightaway;
								turn_need_r = 650;    //1600   750
								turn_need_l = -650; 
								turn_left_flag++;
								Prestate=1;
								break;
				case 2:	//�����Ǻ�ɫ����ô����ֱ��
								integral2 = integra;        
								derivative2 = deriva; 	       
								speed_need = speed_straightaway;
								turn_need_r = 0;
								turn_need_l = 0;
								Prestate=2;		        
								break;
				case 3://��ߺ�ɫ�����д���ת
								integral2 = integra;        
								derivative2 = deriva; 	       
								speed_need = speed_straightaway;
								turn_need_r = 1800;   //2100    1800
								turn_need_l = -1800;
								turn_left_flag++;
								Prestate=3;		        
				        //GPIO_ResetBits(GPIOE,GPIO_Pin_10);  //��������һ��
								break;
				case 4://���кڣ�����С��ת
								integral2 = integra;        
								derivative2 = deriva; 	       
								speed_need = speed_straightaway;
								turn_need_r = -650;  //750
								turn_need_l = 650;
								//turn_right_flag++;
								Prestate=4;		        
								break;
				case 5://�м��Ǻ�ɫ����ô����ֱ��
								integral2 = integra;        
								derivative2 = deriva; 	       
								speed_need = speed_straightaway;
								turn_need_r = 0;
								turn_need_l = 0;
								Prestate=5;		        
								break;
				case 6://�ұ��Ǻ�ɫ�����д���ת
								integral2 = integra;        
								derivative2 = deriva; 	       
								speed_need = speed_straightaway;
								turn_need_r = -1800;//ԭ����1400���е㳬ǰ��1600��ͦ��
								turn_need_l = 1800;
								turn_right_flag++;
								Prestate=6;		        
								break;
				
				case 7://ȫ�ǰ�ɫ����ʱ����ж��ߴ����ɸ���֮ǰ��״̬���жϵ�ǰ��Ҫִ�еĶ���
					switch(Prestate)
					{
					case 0:	//ȫ�Ǻ�ɫ����ôҪ����ֱ��
									integral2 = integra;        
									derivative2 = deriva; 	       
									speed_need = speed_straightaway;
									turn_need_r = 0;
									turn_need_l = 0;
									break;
					case 1://���к�ɫ����ôС��ת
									integral2 = integra;        
									derivative2 = deriva;		       
									speed_need = speed_straightaway;
									turn_need_r = 650;
									turn_need_l = -650;
									break;
					case 2:	//�����Ǻ�ɫ����ô����ֱ��
									integral2 = integra;        
									derivative2 = deriva; 	       
									speed_need = speed_straightaway;
									turn_need_r = 0;
									turn_need_l = 0;        
									break;
					case 3://��ߺ�ɫ�����д����ת
									integral2 = integra;        
									derivative2 = deriva; 	       
									speed_need = speed_straightaway;
									turn_need_r = 1800;      //yuanlai 1800
									turn_need_l = -1800;       
									break;
					case 4://���кڣ�����С��ת
									integral2 = integra;        
									derivative2 = deriva; 	       
									speed_need = speed_straightaway;
									turn_need_r = -650;    //750
									turn_need_l = 650;        
									break;
					case 5://�м��Ǻ�ɫ����ô����ֱ��
									integral2 = integra;        
									derivative2 = deriva; 	       
									speed_need = speed_straightaway;
									turn_need_r = 0;
									turn_need_l = 0;        
									break;
					case 6://�ұ��Ǻ�ɫ�����д����ת
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
	else //������
	{
		if(speed_straightaway < 2100)//(���ﱾ����С�����ڸĳɴ��ڣ�
		{
					speed_straightaway = 2100;
					if(Stop_Flag&&Timer2_counter2<140)//�������٣�100��ʱ���ܵ����ֻ�ǲ��ߣ�add 110   140 a little chao 130 youdianb ugou
							{
							speed_straightaway-=20;  //(100)
							integral2 = integra;        
							derivative2 = deriva;
							speed_need =speed_straightaway;
							turn_need_r = 0;
							turn_need_l = 0;
							}
							else if(Stop_Flag&&Timer2_counter2<660)//ͣ��4��
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
							else if(Timer2_counter2>660&&Timer2_counter2<700)//ֱ��һС�Σ�710������ߣ���710~790��
							{
								Stop_Flag1=0;
								GPIO_SetBits(GPIOE,GPIO_Pin_10);    //
								integral2 = integra;        
								derivative2 = deriva;
								speed_need =speed_straightaway;
								turn_need_r = 0;
								turn_need_l = 0;
							}
							else if(Timer2_counter2>700&&Timer2_counter2<780)//����ͷת�򳵿�   790   all jian10
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
							else if(Timer2_counter2>780&&Timer2_counter2<785)//ֱ�У�900��ʱ��ײǽ��,830���ߣ���790~830��zaijian 5
							{
								integral2 = integra;        
								derivative2 = deriva;
								speed_need =speed_straightaway;
								turn_need_r = 0;
								turn_need_l = 0;
							}
							else if(Timer2_counter2>785&&Timer2_counter2<920)//ͣ��
							{
								Stop_Flag1=1;
								GPIO_ResetBits(GPIOE,GPIO_Pin_10);
								integral2 = integra;        
								derivative2 = deriva;
								speed_need =0;
								turn_need_r = 0;
								turn_need_l = 0;
							}
							else if(Timer2_counter2>920)//ͣ�����رշ�����
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
					if(Stop_Flag&&Timer2_counter2<140)//��������
					{
						speed_straightaway-=20;
					integral2 = integra;        
					derivative2 = deriva;
					speed_need =speed_straightaway;
					turn_need_r = 0;
					turn_need_l = 0;
					}
					else if(Stop_Flag&&Timer2_counter2<660)//ͣ��4��
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
					else if(Timer2_counter2>660&&Timer2_counter2<700)//ֱ��һС��    jian 30
					{
						Stop_Flag1=0;
						GPIO_SetBits(GPIOE,GPIO_Pin_10);
						integral2 = integra;        
						derivative2 = deriva;
						speed_need =speed_straightaway;
						turn_need_r = 0;
						turn_need_l = 0;
					}
					else if(Timer2_counter2>700&&Timer2_counter2<780)//����ͷת�򳵿�
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
					else if(Timer2_counter2>780&&Timer2_counter2<785)//ֱ��
					{
						integral2 = integra;        
						derivative2 = deriva;
						speed_need =speed_straightaway;
						turn_need_r = 0;
						turn_need_l = 0;
					}
					else if(Timer2_counter2>785&&Timer2_counter2<920)//ͣ��
					{
						Stop_Flag1=1;
						GPIO_ResetBits(GPIOE,GPIO_Pin_10);
						integral2 = integra;        
						derivative2 = deriva;
						speed_need =0;
						turn_need_r = 0;
						turn_need_l = 0;
					}
					else if(Timer2_counter2>920)//ͣ�����رշ�����
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
���ƣ�void ST188_DATA_CHECK()
���ܣ��жϴ����������Ƿ��Ǵ�������
�����������ǰ�Ƕ�.
�����������
����ֵ��  ��Ч����Ϊ1����Ч����Ϊ0
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

