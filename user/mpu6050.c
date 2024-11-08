#include "stm32f10x.h"
#include "mpu6050.h"
#include "iic.h"

//AD0接VCC   MPU6050_ADDR 写地址0xd2 读地址0xd3
#define MPU6050_ADDR       0xd0      //AD0 = 0 时地址
//#define MPU6050_ADDR       0xd2    //AD0 = 1 时地址
#define CONFIG             0x1a
#define GYRO_CONFIG        0x1b
#define ACCEL_CONFIG       0x1c	
#define INT_PIN_CFG        0x37 
#define MPU6050_BURST_ADDR 0x3b 
#define USER_CTLR          0x6a	       
#define PWR_MGMT1          0x6b	     
#define PWR_MGMT2          0x6c
#define MPU6050_ID_ADDR	   0x75
#define MPU6050_ID	       0x68  		 //AD0 = 0时 who am i
//#define MPU6050_ID	       0x69    //AD0 = 1时 who am i
/*************************************************

名称：mpu6050_init(void)
功能：mpu6050初始化
输入参数：无
输出参数：无
返回值：无
**************************************************/
void mpu6050_init(void)
{
  u8 data_buf = 0;
 
  /* iic bypass en */
  data_buf = 0x02;
  iic_rw(&data_buf, 1, INT_PIN_CFG, MPU6050_ADDR, WRITE);

  /* iic master disable */
  data_buf  =0x00;
  iic_rw(&data_buf, 1, USER_CTLR, MPU6050_ADDR, WRITE);

  /* mpu6050 sleep disable, temperature en, in 8M osc */
  data_buf = 0x00;
  iic_rw(&data_buf, 1, PWR_MGMT1, MPU6050_ADDR, WRITE);

  /* mpu6050 no standby mode */
  data_buf = 0x00;
  iic_rw(&data_buf, 1, PWR_MGMT2, MPU6050_ADDR, WRITE);

  /* DLPF */
  data_buf = 0x06;
  iic_rw(&data_buf, 1, CONFIG, MPU6050_ADDR, WRITE);

   /* GYRO +-2000 °/s */  
  data_buf = 0x18;
  iic_rw(&data_buf, 1, GYRO_CONFIG, MPU6050_ADDR, WRITE);
  
  /* ACC +-4g */
  data_buf = 0x08;
  iic_rw(&data_buf, 1, ACCEL_CONFIG, MPU6050_ADDR, WRITE);
}
/*************************************************

名称：mpu6050_get_data(s16 *gx,s16 *gy,s16 *gz,s16 *ax,s16 *ay,s16 *az,s16 *temperature)
功能：mpu6050数据读取
输入参数：
  	s16 *gx	 变量指针
	s16 *gy
	s16 *gz
	s16 *ax
	s16 *ay
	s16 *az
	s16 *temperature
输出参数：mpu6050温度及三轴原始数据
返回值：无
**************************************************/
void mpu6050_get_data(s16 *gx, s16 *gy, s16 *gz, s16 *ax, s16 *ay, s16 *az, s16 *temperature)
{
  u8 data_buf[14];

  iic_rw(&data_buf[0], 14, MPU6050_BURST_ADDR, MPU6050_ADDR, READ);
  *ax = data_buf[0] * 0x100 + data_buf[1];
  *ay = data_buf[2] * 0x100 + data_buf[3];
  *az = data_buf[4] * 0x100 + data_buf[5];
  *temperature = data_buf[6] * 0x100 + data_buf[7];
  *gx = data_buf[8] * 0x100 + data_buf[9];
  *gy = data_buf[10] * 0x100 + data_buf[11];
  *gz = data_buf[12] * 0x100 + data_buf[13];
}
/***************************END OF FILE**********************************************************************/
