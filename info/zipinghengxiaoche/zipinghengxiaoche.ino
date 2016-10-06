#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "TimerOne.h"
#include "PID_v1.h"

struct  anglePID
          {
             double Input;
             double Output;         
             double Setpoint;      
           }anglepid;          

MPU6050 accelgyro;//mpu6050数据定义
int16_t ax, ay, az;
int16_t gx, gy, gz;
boolean switch6050;  //布尔型非零即一
boolean switchpwm;
boolean switchpid;
boolean switchposition;


volatile unsigned int motor1=0;	 //计QIAN
volatile unsigned int motor2=0;	 //计HOU
unsigned int speed1=0;	 //计QIAN
unsigned int speed2=0;	 //计QIAN
unsigned int speedcar=0; //车速//未用到
//******PWM参数*************
int   speed_mQ;		 //QIAN电机转速
int   speed_mH;		 //HOU电机转速
int   pwm;               //综合PWM计算
float PWMI;	         //PWM积分值
float Position;	         //位移
//******角度参数************
float Gyro_X;        //X轴陀螺仪数据暂存
float Angle;         //小车最终倾斜角度
char value;          //角度正负极性标记	
float arctanangle;   //反正切角度
//******卡尔曼参数************
static const double C_0 = 1;
static const double Q_angle=0.001, Q_gyro=0.003, R_angle=0.5, dt=0.01;//注意：dt的取值为kalman滤波器采样时间
double P[2][2] = {{ 1, 0 },
                  { 0, 1 }};
double Pdot[4] ={ 0,0,0,0};
double q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
double angle, angle_dot;
//------------------------------------------------------------------
void Kalman_Filter(double angle_m,double gyro_m)
{
angle+=(gyro_m-q_bias) * dt;
angle_err = angle_m - angle;
Pdot[0]=Q_angle - P[0][1] - P[1][0];
Pdot[1]=- P[1][1];
Pdot[2]=- P[1][1];
Pdot[3]=Q_gyro;
P[0][0] += Pdot[0] * dt;
P[0][1] += Pdot[1] * dt;
P[1][0] += Pdot[2] * dt;
P[1][1] += Pdot[3] * dt;
PCt_0 = C_0 * P[0][0];
PCt_1 = C_0 * P[1][0];
E = R_angle + C_0 * PCt_0;
K_0 = PCt_0 / E;
K_1 = PCt_1 / E;
t_0 = PCt_0;
t_1 = C_0 * P[0][1];
P[0][0] -= K_0 * t_0;
P[0][1] -= K_0 * t_1;
P[1][0] -= K_1 * t_0;
P[1][1] -= K_1 * t_1;
angle += K_0 * angle_err; //最优角度
q_bias += K_1 * angle_err;
angle_dot = gyro_m-q_bias;//最优角速度
}

PID myPID(&anglepid.Input, &anglepid.Output, &anglepid.Setpoint,6,5,1, REVERSE);	

//-----------------------------------------------------------
void setup() 
{
    Wire.begin();
    Serial.begin(115200);
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    pinMode(2, INPUT);    //QIAN
    pinMode(3, INPUT);    //HOU
    pinMode(4, INPUT);    //QIAN
    pinMode(5, INPUT);    //HOU
    
    pinMode(6, OUTPUT);    //方向
    pinMode(7, OUTPUT);    //方向
    
    pinMode(11, OUTPUT);   //PWM
    pinMode(13, OUTPUT);
    Timer1.initialize(10000);           //100hz // 1'000'000微秒 
    Timer1.attachInterrupt( timerIsr ); // 定时器服务函数
    
    attachInterrupt(0, stateQIAN, FALLING);  // 监视中断输入引脚的变化//下降沿
    attachInterrupt(1, stateHOU, FALLING);

  anglepid.Input = angle;//PID输入
  anglepid.Setpoint = 0;      //PID目标值
  myPID.SetOutputLimits(-200,200);//PID上下限
  myPID.SetSampleTime(10); //PID时间
  myPID.SetMode(AUTOMATIC);//打开PID开关
}

void loop() 
{ 
if(switch6050==1)
{
  angle_calculate();
  switch6050=0;
  if(abs(angle)<40)  switchpid=1;
  
}
if(switchpid==1)
 {
   anglepid.Input = angle;
   myPID.Compute(); 
   switchpid=0;
   switchpwm=1;
}
if(switchpwm==1)
{switchpwm=0;
// if(abs(angle)<=2&&abs(Position)>=10)
//{
//  if(Position>=10)Advance(100);
//  else Back(100); 
//  
//} 
//else 
//  {
    pwm_calculate();
  if(angle>=2)  
  {Advance(pwm);}
  else if (angle<=-2)
  Back(pwm);

  
//  }
}
if(abs(angle)>40) Stop();
// if(angle<2&&angle>-2)
//  {
//  if(Position>10) Advance(100); 
//  digitalWrite(13, LOW);
//  if(Position<-10) Back(100); 
//  }

}



void angle_calculate()
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//    Serial.println(ay); Serial.print(",");
//    Serial.print(gx); Serial.print(",");
    Gyro_X   = (gx+400)/131.0;         	  //去除零点偏移，计算角速度值,方向处理
    arctanangle=atan2(ay+1200,az)*180/3.14;    //俯仰角
    if(arctanangle>0) value=1;                  //角度正负标记
    else if (arctanangle<0) value=-1;           //乘以角速度PWM
    else value=0;                               //进行方向修正
//Angle = Angle + (((Angle_aZ-Angle)*0.5 + Gyro_X)*0.01);
    Kalman_Filter(arctanangle,Gyro_X);    //全局变量angle angle_dot
//    Serial.print(arctanangle);Serial.print(",");
//    Serial.print(Gyro_X);Serial.print(","); 
//    Serial.print(angle);Serial.print(",");  
//    Serial.println(angle_dot);

}
void timerIsr()
{
  speedcar =(motor1+motor2)/11.356;      //只是用一路PWM两路电机脉冲合成//*20CM*100HZ/34/334/2   cm/s
  
  Position+=(speed_mH+speed_mQ)/1600.0;  //位移cm
  Serial.println(speedcar);
  motor1=0,motor2=0,speed_mH=0;speed_mQ=0;//清零
  switch6050=1;                          //中断中不能读6050只能制标志位
  
}

void stateQIAN()
{
  motor1++; 
  if(digitalRead(4) == 0) 
 speed_mQ++;		 //Q电机前进//方向不同
 else		        
 speed_mQ--;		 //Q电机后退
}
void stateHOU()
{
  motor2++;
  if(digitalRead(5) == 1)
 speed_mH++;		 //H电机前进//方向正确
 else	
 speed_mH--;		 //H电机后退

}
//-------------------------------------------------------------------
void Advance(int speed)
{
digitalWrite(6,HIGH);
digitalWrite(7,LOW);
analogWrite(11,speed);
}
void Back(int speed)
{
digitalWrite(6,LOW);
digitalWrite(7,HIGH);
analogWrite(11,speed);

}
void Stop(void)
{
digitalWrite(6,HIGH);
digitalWrite(7,HIGH);
}
void pwm_calculate()
{
pwm=50+abs(anglepid.Output)+value*angle_dot*0.6;
pwm=constrain(pwm,50,255);
}

