#include "common.h"
#include "include.h"
#include "math.h"
#define gyro_ratio      0.00763
#define GRAVITY_ADJUST_TIME_CONSTANT    4
#define	PERIOD				1000				//��ѹת��PWM����	����
#define h_gan           0.56
#define angle_set1      0
#define pi      3.1415926
#define MOTOR_OUT_DEAD_VAL 0.035        //������ѹ

void PIT0_IRQHandler(void);
int16 gyro_x,gyro_y,gyro_z;
int16 accel_x,accel_y,accel_z;
int16 accel_zero=0;                       //���ٻ����ֵ
int16 accel_zero1=0;
int16 gyro_zero;                              //x���������ֵ
int16 gyro_zero1;                             //y
float gyro_speed;                                //���ٶ�ֵ
float gyro_speed1;
float accel_accel;                                //�������ٶ�ֵ
float accel_accel1; 
float accel_angle;                               //���ٶȽǶ�
float accel_angle1;
float angle_fuse;
float angle_fuse1;
float balance_set=1;                           //ƽ���
float gyro_speed_set;
float h_set=50;
float display[5];
float gyro_angle;
float gyro_angle1;
float angle_fuse_old;
float red_motor,white_motor;
float angle_set=11;

uint8 yaoqiu,shang_tiao,xia_tiao;     //����Ҫ��ѡ��
uint16 delay;
/****************��ʱ����*************/
void delayms()     //��ʱ10ms
{
delay++;
if(delay==2)
{delay=0;
      }
}

/****************************************��mpu6050��ʼ��****************************************/
void mpu6050_init()
{
  i2c_init(I2C0,400000);
  lptmr_delay_ms(10);
  i2c_write_reg(I2C0,0x68,0x6b,0x00);
  i2c_write_reg(I2C0,0x68,0x19,0x00);
  i2c_write_reg(I2C0,0x68,0x1a,0x00);
  i2c_write_reg(I2C0,0x68,0x1b,0x00);
  i2c_write_reg(I2C0,0x68,0x1c,0x00);
}
/************************************
��ȡmpu6050��ֵ
*************************************/
void mpu6050_read()
{
    uint16 msb,lsb;
    msb=i2c_read_reg(I2C0,0x68,0x3b);
    lsb=i2c_read_reg(I2C0,0x68,0x3c);
    accel_x=msb<<8|lsb;
   msb=i2c_read_reg(I2C0,0x68,0x3d);
    lsb=i2c_read_reg(I2C0,0x68,0x3e);
    accel_y=msb<<8|lsb;
    msb=i2c_read_reg(I2C0,0x68,0x3f);
    lsb=i2c_read_reg(I2C0,0x68,0x40);
    accel_z=msb<<8|lsb;

    msb=i2c_read_reg(I2C0,0x68,0x43);
    lsb=i2c_read_reg(I2C0,0x68,0x44);
    gyro_x=msb<<8|lsb;  
    msb=i2c_read_reg(I2C0,0x68,0x45);
    lsb=i2c_read_reg(I2C0,0x68,0x46);
    gyro_y=msb<<8|lsb;
    msb=i2c_read_reg(I2C0,0x68,0x47);
    lsb=i2c_read_reg(I2C0,0x68,0x48);
    gyro_z=msb<<8|lsb;    
}
/**********************************
����С����ǰ�Ƕ�ֵ
**********************************/
void angle_calculate(void)
{
    float value;
    gyro_speed=(gyro_y-gyro_zero)*gyro_ratio;
    gyro_angle-=gyro_speed*0.005;
    accel_accel=(accel_x-accel_zero)/16384.0;
    if(accel_accel>1)   accel_accel=1;
    if(accel_accel<-1)  accel_accel=-1;
    accel_angle=180/3.1415926*(asin(accel_accel));
    value=(accel_angle-angle_fuse)/GRAVITY_ADJUST_TIME_CONSTANT;
    angle_fuse+=(value-gyro_speed)/200;   
}
void angle_calculate1()
{
    float value;
    gyro_speed1=(gyro_x-gyro_zero1)*gyro_ratio;
   // gyro_angle+=gyro_speed*0.005;
    accel_accel1=(accel_y-accel_zero1)/16384.0;
    if(accel_accel1>1)   accel_accel1=1;
    if(accel_accel1<-1)  accel_accel1=-1;
    accel_angle1=180/3.1415926*(asin(accel_accel1));
    value=(accel_angle1-angle_fuse1)/GRAVITY_ADJUST_TIME_CONSTANT;
    angle_fuse1+=(value-gyro_speed1)/200;   
}
/***************************************
�������������ֵ
****************************************/
int16 ad_ave(int16 N) //��ֵ�˲�
{
    int32 tmp = 0;
    int16 lsb,msb;
    int16 temp;
    int16  i;
    for(i = 0; i < N; i++)
	{
           msb=i2c_read_reg(I2C0,0x68,0x45);
           lsb=i2c_read_reg(I2C0,0x68,0x46);
           temp=msb<<8|lsb;
           tmp+=temp;
	   lptmr_delay_ms(5);
	}
    tmp = tmp / N;
    return (int16)tmp;
}
int16 ad_ave1(int16 N) //��ֵ�˲�
{
    int32 tmp = 0;
    int16 lsb,msb;
    int16 temp;
    int16  i;
    for(i = 0; i < N; i++)
	{
           msb=i2c_read_reg(I2C0,0x68,0x43);
           lsb=i2c_read_reg(I2C0,0x68,0x44);
           temp=msb<<8|lsb;
           tmp+=temp;
	   lptmr_delay_ms(5);
	}
    tmp = tmp / N;
    return (int16)tmp;
}
/************����������*************/
void SetMotorVoltage(float fLeftVoltage,float fRightVoltage)
{
    int nOut;
    if(fLeftVoltage>0)
    {
	ftm_pwm_duty(FTM0,FTM_CH3,0);//���������˶�PWMռ�ձ�Ϊ0
	nOut=(int)(fLeftVoltage*PERIOD);//
	if(nOut>1000)
	{
		nOut=1000;
	}
	ftm_pwm_duty(FTM0,FTM_CH4,nOut);//���ַ����˶�PWMռ�ձ�ΪnOut
    }                                                   //������ת
    else
    {
	ftm_pwm_duty(FTM0,FTM_CH4,0);//���ַ����˶�PWMռ�ձ�Ϊ0
	fLeftVoltage=-fLeftVoltage;
	nOut=(int)(fLeftVoltage*PERIOD);
	if(nOut>1000)
	{
		nOut=1000;
	}
	ftm_pwm_duty(FTM0,FTM_CH3,nOut);//���������˶�PWMռ�ձ�ΪnOut
    }                                                    //������ת

    if(fRightVoltage>0)
    {
	ftm_pwm_duty(FTM2,FTM_CH0,0);//���������˶�PWMռ�ձ�Ϊ0
	nOut=(int)(fRightVoltage*PERIOD);
	if(nOut>1000)
	{
		nOut=1000;
	}	
	ftm_pwm_duty(FTM2,FTM_CH1,nOut);//���ַ����˶�PWMռ�ձ�ΪnOut
    }                                                     //�ҵ����ת
    else
    {
	ftm_pwm_duty(FTM2,FTM_CH1,0);//���ַ����˶�PWMռ�ձ�Ϊ0
	fRightVoltage=-fRightVoltage;
	nOut=(int)(fRightVoltage*PERIOD);
	if(nOut>1000)
	{
		nOut=1000;
	}
	ftm_pwm_duty(FTM2,FTM_CH0,nOut);//���������˶�PWMռ�ձ�ΪnOut
    }                                                     //�ҵ����ת
}
/**************����������********************/
void MotorSpeedOut(void)
{
    float fLeftVal, fRightVal;
    fLeftVal = white_motor;
    fRightVal = red_motor;
    if(fLeftVal > 0)
        fLeftVal += MOTOR_OUT_DEAD_VAL;
    else if(fLeftVal < 0)
        fLeftVal -= MOTOR_OUT_DEAD_VAL;
    if(fRightVal > 0)
        fRightVal += MOTOR_OUT_DEAD_VAL;
    else if(fRightVal < 0)
        fRightVal -= MOTOR_OUT_DEAD_VAL;//��������
    SetMotorVoltage(fLeftVal,fRightVal);
}
void set_gyro()
{
  float h,angle,h1,h2;
  angle=angle_fuse-balance_set;
  if(angle<0)   angle=-angle;   
  h1=cos(angle/180*pi);
  h2=cos(angle_set/180*pi);
  h=h_gan*(h1-h2);
  if(h<0)       h=0;
  gyro_speed_set=sqrt(2*9.8*h)/h_gan;
  gyro_speed_set=gyro_speed_set/pi*180;
}

void motor_control()
{
//  if((angle_fuse>-2)&&(angle_fuse<4)&&(gyro_speed<10)&&(gyro_speed>-10))
//  {
//    red_motor=0.3;
//  }
    if((angle_fuse>-3)&&(angle_fuse<3))
  {
    red_motor=0.35;
   // DELAY_MS(100);
  }
  
  else
  {
    if(gyro_speed>0)
    {
      red_motor=(gyro_speed - gyro_speed_set)*0.012;  //0.012
    }
    else
    {
      red_motor=(gyro_speed + gyro_speed_set)*0.012;
    }
  }
 // white_motor=-gyro_speed1*0.03;
   white_motor=0;
}

/********************************oled��ʾ����**********************************/
void oledplay()                                            
{
  OLED_P6x8Str(0,0,"yaoqiu:");
  OLED_P6x8Str(0,1,"shang_tiao:");
  OLED_P6x8Str(0,2,"xia_tiao:");
  Display_number(30,0,yaoqiu);
  Display_number(30,1,shang_tiao);
  Display_number(30,2,xia_tiao);

}

void main()
{   
    OLED_Init();           //��ʼ��oled 

    ftm_pwm_init(FTM0,FTM_CH3,10000,0);
    ftm_pwm_init(FTM0,FTM_CH4,10000,0);
    ftm_pwm_init(FTM2,FTM_CH0,10000,0);
    ftm_pwm_init(FTM2,FTM_CH1,10000,0);
    
    mpu6050_init();
    
    lptmr_delay_ms(1000);
    
    gyro_zero=ad_ave(100);
    
    gyro_zero1=ad_ave1(100);
    
    mpu6050_read();
    
     accel_accel=(accel_x-accel_zero)/16384.0;
    if(accel_accel>1)   accel_accel=1;
    if(accel_accel<-1)  accel_accel=-1;
    angle_fuse=180/pi*(asin(accel_accel)); 
    
    accel_accel1=(accel_y-accel_zero1)/16384.0;
    if(accel_accel1>1)   accel_accel1=1;
    if(accel_accel1<-1)  accel_accel1=-1;
    angle_fuse1=180/3.1415926*(asin(accel_accel1));
    
    pit_init_ms(PIT0, 5);                                //��ʼ��PIT0����ʱʱ��Ϊ�� 5ms
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //����PIT0���жϷ�����Ϊ PIT0_IRQHandler    
    enable_irq (PIT0_IRQn);                                 //ʹ��PIT0�ж�
    
    uart_init(UART3, 115200);
    
    while(1)
    {   
        display[0]=gyro_speed;
        display[1]=angle_fuse;
        display[2]=accel_angle; 
        display[3]=angle_fuse1;
        display[4]=gyro_speed1;
        vcan_sendware((unsigned char *)display, 20);

    }
}
void PIT0_IRQHandler(void)
{  
    delayms();
    mpu6050_read();
    angle_calculate();
    angle_calculate1();
    set_gyro();
    motor_control(); 
    SetMotorVoltage(0.05,0);
    MotorSpeedOut();
    PIT_Flag_Clear(PIT0);       //���жϱ�־λ 
}