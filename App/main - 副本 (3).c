#include "common.h"
#include "include.h"
#include "math.h"
#define gyro_ratio      0.00763
#define GRAVITY_ADJUST_TIME_CONSTANT    4
#define	PERIOD				1000				//电压转换PWM比例	待定
#define h_gan           0.56
#define pi      3.1415926
#define MOTOR_OUT_DEAD_VAL 0.035        //死区电压

void PIT0_IRQHandler(void);
int16 gyro_x,gyro_y,gyro_z;
int16 accel_x,accel_y,accel_z;
int16 accel_zero=0;                       //加速机零点值
int16 accel_zero1=0;
int16 gyro_zero;                              //x陀螺仪零点值
int16 gyro_zero1;                             //y
float gyro_speed;                                //角速度值
float gyro_speed1;
float accel_accel;                                //重力加速度值
float accel_accel1; 
float accel_angle;                               //加速度角度
float accel_angle1;
float angle_fuse;
float angle_fuse1;



float balance_set=2;                           //平衡点
float gyro_speed_set;
float gyro_speed_set1;
float h_set=50;
float display[5];
float ww[31]={11,11.5,12,12.25,12.5,12.75,13,13.25,13.5,13.75,14,14.5,15,16,16.16,16.33,16.5,16.66,16.83,17,17.25,17.5,17.75,18,18.5,19,19.33,19.66,20,20.5,21};
float angle_set;
float angle_set1;
float gyro_angle;
float gyro_angle1;
float angle_fuse_old;
float red_motor,white_motor;
uint16 aa=0;
uint16 K1,K2,K3,K4,qd=1,distance=30;
uint8 ax3=0,ax4=0;
uint16 delay;
uint16 active=0,rollcount=0,zerocount=0,jiaomax=0;
uint16 stoptest=0;
uint8 flag;
/****************延时函数*************/
void delayms()     //延时10ms
{
delay++;
if(delay==2)
{delay=0;
      }
}

void DIP_switch()
{
  if( PTA26_IN==0)  
  { 
    flag=1;
  } 
  if( PTA28_IN==0)  
  { 
    flag=2;
  } 
  if( PTA27_IN==0)  
  { 
    flag=3;
  }
  if( PTA19_IN==0)  
  { 
    flag=4;
  } 


  
}

float absf(float floatin)
{
  if(floatin<0)
    return -floatin;
  else
    return floatin;
  
}

/****************************************
将mpu6050初始化****************************************/
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
读取mpu6050的值
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
/**********************************运算小车当前角度值**********************************/
/**********************************x轴*********************************/
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
/**********************************y轴*********************************/
void angle_calculate1()
{
    float value;
    gyro_speed1=(gyro_x - gyro_zero1)*gyro_ratio;
   gyro_angle1+=gyro_speed1*0.005;
    accel_accel1=(accel_y-accel_zero1)/16384.0;
    if(accel_accel1>1)   accel_accel1=1;
    if(accel_accel1<-1)  accel_accel1=-1;
    accel_angle1=180/3.1415926*(asin(accel_accel1));
    value=(accel_angle1-angle_fuse1)/GRAVITY_ADJUST_TIME_CONSTANT;
    angle_fuse1+=(value+gyro_speed1)/200;   
}
/***************************************
计算陀螺仪零点值
****************************************/
int16 ad_ave(int16 N) //均值滤波
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
int16 ad_ave1(int16 N) //均值滤波
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
/************电机输出函数*************/
void SetMotorVoltage(float fLeftVoltage,float fRightVoltage)
{
    int nOut;
    if(fLeftVoltage>0)
    {
	ftm_pwm_duty(FTM0,FTM_CH3,0);//左轮正向运动PWM占空比为0
	nOut=(int)(fLeftVoltage*PERIOD);//
	if(nOut>1000)
	{
		nOut=1000;
	}
	ftm_pwm_duty(FTM0,FTM_CH4,nOut);//左轮反向运动PWM占空比为nOut
    }                                                   //左电机正转
    else
    {
	ftm_pwm_duty(FTM0,FTM_CH4,0);//左轮反向运动PWM占空比为0
	fLeftVoltage=-fLeftVoltage;
	nOut=(int)(fLeftVoltage*PERIOD);
	if(nOut>1000)
	{
		nOut=1000;
	}
	ftm_pwm_duty(FTM0,FTM_CH3,nOut);//左轮正向运动PWM占空比为nOut
    }                                                    //左电机反转

    if(fRightVoltage>0)
    {
	ftm_pwm_duty(FTM2,FTM_CH0,0);//右轮正向运动PWM占空比为0
	nOut=(int)(fRightVoltage*PERIOD);
	if(nOut>1000)
	{
		nOut=1000;
	}	
	ftm_pwm_duty(FTM2,FTM_CH1,nOut);//右轮反向运动PWM占空比为nOut
    }                                                     //右电机正转
    else
    {
	ftm_pwm_duty(FTM2,FTM_CH1,0);//右轮反向运动PWM占空比为0
	fRightVoltage=-fRightVoltage;
	nOut=(int)(fRightVoltage*PERIOD);
	if(nOut>1000)
	{
		nOut=1000;
	}
	ftm_pwm_duty(FTM2,FTM_CH0,nOut);//右轮正向运动PWM占空比为nOut
    }                                                     //右电机反转
}
/**************电机输出函数********************/
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
        fRightVal -= MOTOR_OUT_DEAD_VAL;//死区处理
    SetMotorVoltage(fLeftVal,fRightVal);
}

float getsum()
{
  if(angle_fuse>angle_fuse1)
    return angle_fuse1/angle_fuse;
  else
    return angle_fuse/angle_fuse1;
  
  
}

void MotorSpeedOut_3(void)
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
        fRightVal -= MOTOR_OUT_DEAD_VAL;//死区处理
    SetMotorVoltage(-fLeftVal,fRightVal);
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
void set_gyro1()
{
  float h,angle,h1,h2;
  angle=angle_fuse1-balance_set;
  if(angle<0)   angle=-angle;   
  h1=cos(angle/180*pi);
  h2=cos(angle_set1/180*pi);
  h=h_gan*(h1-h2);
  if(h<0)       h=0;
  gyro_speed_set1=sqrt(2*9.8*h)/h_gan;
  gyro_speed_set1=gyro_speed_set1/pi*180;
  
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
    white_motor=-gyro_speed1*0.03;
   //white_motor=0;
}


void motor_control_3()
{
//  if((angle_fuse>-2)&&(angle_fuse<4)&&(gyro_speed<10)&&(gyro_speed>-10))
//  {
//    red_motor=0.3;
//  }
    if((angle_fuse1>-3)&&(angle_fuse1<3))
  {
    white_motor=0.35;
   // DELAY_MS(100);
  }
  
  else
  {
    if(gyro_speed1>0)
    {
      white_motor=(gyro_speed1 - gyro_speed_set1)*0.012;  //0.012
    }
    else
    {
      white_motor=(gyro_speed1 + gyro_speed_set1)*0.012;
    }
  } 
   //red_motor=gyro_speed*0.03;
   //white_motor=0;
 
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
   
  
}



void motor_control_4()
{
//  if((angle_fuse>-2)&&(angle_fuse<4)&&(gyro_speed<10)&&(gyro_speed>-10))
//  {
//    red_motor=0.3;
//  }
//    if((angle_fuse>-3)&&(angle_fuse<3))
//  {
//    red_motor=0.35;
//   // DELAY_MS(100);
//  }
//  
//  else
//  {
//    if(gyro_speed>0)
//    {
//      red_motor=(gyro_speed - gyro_speed_set)*0.012;  //0.012
//    }
//    else
//    {
//      red_motor=(gyro_speed + gyro_speed_set)*0.012;
//    }
//  } 
  red_motor=gyro_speed*0.03;
  white_motor=-gyro_speed1*0.03;
  
}
/********************************oled显示函数**********************************/
void oledplay()                                            
{
   
  OLED_P6x8Str(0,0,"angle_fusex:");
  OLED_P6x8Str(0,1,"angle_fusey:");
//  OLED_P6x8Str(0,2,"gyro_speedy:");
//  OLED_P6x8Str(0,3,"angle_fusey:");
  if(flag==2)
  {
    OLED_P6x8Str(0,4,"distance:");
    OLED_P6x8Str(0,5,"angle_set:");
    Display_number7(74,4,(int16)distance);
    DisplayFloat3(74,5,(int16)(angle_set*1000));
  }
  
  Display_number7(74,0,(int16)angle_fuse);
  Display_number7(74,1,(int16)angle_fuse1);
//  Display_number7(74,2,(int16)gyro_speed1);
//  Display_number7(74,3,(int16)angle_fuse1);
  
  
  
  OLED_P6x8Str(0,7,"mode");
  
          if(flag==1)
          OLED_P6x8Str(24,7,"1");
        else if (flag==2)
          OLED_P6x8Str(24,7,"2");
        else if (flag==3)
          OLED_P6x8Str(24,7,"3");
        else if (flag==4)
          OLED_P6x8Str(24,7,"4");

  
  
  



}

/************示波器函数*************/
void vcan_sendware(uint8 *wareaddr, uint32 waresize)
{
#define CMD_WARE     3
    uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};    //yy_摄像头串口调试 使用的命令
    uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};    //yy_摄像头串口调试 使用的命令

    uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));    //先发送命令

    uart_putbuff(VCAN_PORT, wareaddr, waresize); //再发送图像

    uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));    //先发送命令
}
void keyt()
{
 


    K2=adc_once (ADC1_SE10,ADC_8bit); 
    K1=adc_once (ADC1_SE11,ADC_8bit); 
    K3=adc_once (ADC1_SE12,ADC_8bit);
    K4=adc_once (ADC1_SE13,ADC_8bit);
    if(qd!=0)
    {
      if(K3==0&&ax3==0) 
      {
        distance++;
        ax3=1;
        if(distance<=30) { distance=60;}
        if(distance>=60) { distance=30;}
      }
      if(K3>0) ax3=0;
      if(K4==0&&ax4==0) 
      {
        distance--;
        ax4=1;
        if(distance<30) { distance=60;}
        if(distance>60) { distance=30;}
      }
      if(K4>0) ax4=0;

      angle_set=ww[distance-30];
    }
   
    if(K1==0) qd=0;    //break;
  

}



/******************************************
****************题目一*********************
*******************************************/
void mode1()
{
  
//         if(active==0)
//        {
//          SetMotorVoltage(speedy_1,-0.5);
//          if(angle_fuse<-8)
//          {
//            active=1;
//            rollcount=1;
//            SetMotorVoltage(speedy_1,0.12);
//          }
//        }else
//        {
//          if(angle_fuse<-10&&rollcount==0)
//          {
//            rollcount=1;
//            SetMotorVoltage(speedy_1,0.12);
//          }else if(angle_fuse>10&&rollcount==1)
//          {
//            rollcount=0;
//            SetMotorVoltage(speedy_1,-0.12);
//          }
//        }
//        if(angle_fuse>-10&&angle_fuse<10&&gyro_speed<50&&gyro_speed>-50)
//        {
//          stoptest++;
//          if(stoptest==500)
//          {
//            active=0;
//            stoptest=0;
//          }
//        }else
//        {
//          stoptest=0;
//        }
        angle_set=21;
        

  
  
  
  
}

/******************************************
****************题目二*********************
*******************************************/
void mode2()
{
  keyt(); 
}



/******************************************
****************题目三*********************
*******************************************/
void mode3()
{

  
}

/******************************************
****************题目四*********************
*******************************************/
void mode4()
{
 
}


void main()
{   
    OLED_Init();           //初始化oled 
    qd=1;
    ftm_pwm_init(FTM0,FTM_CH3,10000,0);
    ftm_pwm_init(FTM0,FTM_CH4,10000,0);
    ftm_pwm_init(FTM2,FTM_CH0,10000,0);
    ftm_pwm_init(FTM2,FTM_CH1,10000,0);
    adc_init (ADC1_SE10); 
    adc_init (ADC1_SE11);  
    adc_init (ADC1_SE12); 
    adc_init (ADC1_SE13);                               //按键初始化
    gpio_init (PTA13, GPI,HIGH);//拨码开关初始化 
    gpio_init (PTA19, GPI,HIGH); 
    gpio_init (PTA24, GPI,HIGH);
    gpio_init (PTA25, GPI,HIGH); 
    gpio_init (PTA26, GPI,HIGH);
    gpio_init (PTA27, GPI,HIGH);    
    gpio_init (PTA28, GPI,HIGH); 
    gpio_init (PTA29, GPI,HIGH);
    
    led_init (LED0);
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
    
    pit_init_ms(PIT0, 5);                                //初始化PIT0，定时时间为： 5ms
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //设置PIT0的中断服务函数为 PIT0_IRQHandler    
    enable_irq (PIT0_IRQn);                                 //使能PIT0中断
    
    uart_init(UART3, 115200);
     while(aa<200);   //初始化 1秒
    
    led (LED0,LED_ON);
    DIP_switch();   
    while(1)
    {   

        display[0]=angle_fuse;
        display[1]=angle_fuse1;
        display[2]=0; 
        display[3]=0;
        display[4]=0;
        oledplay();
        
        
        if(flag==1)
          mode1();
        else if (flag==2)
          mode2();
        else if (flag==3)
          mode3();
        else if (flag==4)
          mode4();
        
        
        
        vcan_sendware((unsigned char *)display, 20);

    }
}
void PIT0_IRQHandler(void)
{  if(aa<1000) aa++;
    delayms();
    mpu6050_read();
    angle_calculate();
    angle_calculate1();
    if (flag==2||flag==1)
      set_gyro();
    if (flag==3)
    {
      set_gyro1();
      set_gyro();
    }
//    oledplay();
    if(flag==1)
    {
      motor_control(); 
      SetMotorVoltage(0.05,0);
      MotorSpeedOut();      
    }
    if(qd==0)
    {
      motor_control(); 
      SetMotorVoltage(0.05,0);
      MotorSpeedOut();
    }
    if (flag==3)
    {
      motor_control_3(); 
//      SetMotorVoltage(0,0.5);
      MotorSpeedOut_3();
    
    }    
    if (flag==4)
    {
      motor_control_4(); 
      SetMotorVoltage(0.05,0);
      MotorSpeedOut();
    
    }
    PIT_Flag_Clear(PIT0);       //清中断标志位 
}