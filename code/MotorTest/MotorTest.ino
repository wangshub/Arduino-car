#include "AFMotor.h"
#include <Wire.h>
#include <JY901.h>
#include<SoftwareSerial.h>
SoftwareSerial mySerial(A0, A1);   //RX  TX

AF_DCMotor MotorRight(3, MOTOR34_8KHZ);
AF_DCMotor MotorLeft(1,   MOTOR12_8KHZ); // create motor #2, 64KHz pwm

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");

  MotorLeft.setSpeed(200);     // set the speed to 200/255
  MotorRight.setSpeed(200);     // set the speed to 200/255
}

void loop() {
  Serial.print("tick");

  MotorLeft.run(FORWARD);      // turn it on going forward
  delay(500);
  MotorRight.run(FORWARD);      // turn it on going forward
  delay(1000);

  Serial.print("tock");
  MotorLeft.run(BACKWARD);     // the other way
  delay(500);
  MotorRight.run(BACKWARD);      // turn it on going forward
  delay(1000);

  Serial.print("tack");
  MotorLeft.run(RELEASE);      // stopped
  delay(500);
  MotorRight.run(RELEASE);      // turn it on going forward
  delay(1000);
}
