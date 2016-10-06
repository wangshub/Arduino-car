#include "AFMotor.h"
#include <Wire.h>
#include <JY901.h>
#include<SoftwareSerial.h>
#include <PID_v1.h>

//********PID**************************
struct  anglePID {
  double Input;
  double Output;
  double Setpoint;
} anglepid;

double Kp = 2.25 , Ki = 5, Kd =0.102;
double Setpoint, Input, Output;
PID myPID(&anglepid.Input, &anglepid.Output, &anglepid.Setpoint, Kp, Ki, Kd, REVERSE);
int PWM;


//*********JY901**********************************
float AccData[3];
float GyroData[3];
float AngleData[3];
int i;
double angle;
SoftwareSerial mySerial(A0, A1);   //RX  TX
void mySerialEvent() ;  //Waiting JY901 serial data


//*********Motor Funtions*************************
AF_DCMotor MotorRight(2, MOTOR12_8KHZ);
AF_DCMotor MotorLeft(1,   MOTOR12_8KHZ); // create motor #2, 64KHz pwm
void MotorTest() ;
void Advance(int speed) ;
void Back(int speed);
void PWMCalculate() ;
void Stop();
//*********SETUP*********************************
void setup() {
  //communication
  mySerial.begin(115200);
  Serial.begin(9600);               // set up Serial library at 9600 bps

  anglepid.Input = angle;                       //PID input
  anglepid.Setpoint = 0;                         //PID目标值
  myPID.SetOutputLimits(-200, 200);    //PID上下限
  myPID.SetSampleTime(10);                //PID时间
  myPID.SetMode(AUTOMATIC);          //turn the PID on

}

//*********MAIN LOOP*****************************
void loop() {
  mySerialEvent() ;
  for (i = 0; i < 3; i++) {
    AccData[i]    = (float)JY901.stcAcc.a[i] / 32768 * 16;
    GyroData[i]  = (float)JY901.stcGyro.w[i] / 32768 * 2000;
    AngleData[i] = (float)JY901.stcAngle.Angle[i] / 32768 * 180;
  }
  Serial.print(AngleData[0]); Serial.print("  "); Serial.print(AngleData[1]); Serial.print("  "); Serial.println(AngleData[2]);


  angle = (double)AngleData[0];
  myPID.Compute();
  PWMCalculate();
  if (angle > 2)  Advance(PWM);
  if (angle < -2) Back(PWM);
  if (abs(angle) > 40) Stop();



  // MotorTest();

}

void mySerialEvent()
{
  while (mySerial.available())
  {
    JY901.CopeSerialData(mySerial.read()); //Call JY901 data cope function
  }
}

void MotorTest() {
  Serial.println("Motor test!");
  MotorLeft.setSpeed(50);       // set the speed to 200/255
  MotorRight.setSpeed(50);     // set the speed to 200/255

  MotorLeft.run(FORWARD);      // turn it on going forward
  MotorRight.run(FORWARD);      // turn it on going forward
  delay(1000);

  MotorLeft.run(BACKWARD);     // the other way
  MotorRight.run(BACKWARD);      // turn it on going forward
  delay(1000);

  MotorLeft.run(RELEASE);      // stopped
  MotorRight.run(RELEASE);      // turn it on going forward
  delay(1000);
}

void Advance(int speed) {
  int LeftOffset = 0;
  int RightOffset = 0;
  MotorLeft.setSpeed(speed + LeftOffset);
  MotorRight.setSpeed(speed + RightOffset);
  MotorLeft.run(FORWARD);      // turn it on going forward
  MotorRight.run(FORWARD);      // turn it on going forward
}
void Back(int speed) {
  int LeftOffset = 0;
  int RightOffset = 0;
  MotorLeft.setSpeed(speed + LeftOffset);
  MotorRight.setSpeed(speed + RightOffset);
  MotorLeft.run(BACKWARD);      // turn it on going forward
  MotorRight.run(BACKWARD);      // turn it on going forward
}

void PWMCalculate() {
  PWM = 45 + abs(anglepid.Output) ;  //+ value * angle_dot * 0.6;
}

void Stop() {
  MotorLeft.setSpeed(0);
  MotorRight.setSpeed(0);
  MotorLeft.run(BACKWARD);      // turn it on going forward
  MotorRight.run(BACKWARD);      // turn it on going forward
}

