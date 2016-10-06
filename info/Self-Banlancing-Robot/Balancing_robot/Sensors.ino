#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>


// Gyro variables
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int L3G4200D_Address = 105; 

// Accel
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

//Set up accelerometer variables
float accBiasX, accBiasY, accBiasZ;
float accAngleX, accAngleY;
double accRoll = 0;

//Set up gyroscope variables
float gyroBiasX, gyroBiasY, gyroBiasZ;
float gyroRateX, gyroRateY, gyroRateZ;
float gyroRoll = 0;
//double gyro_sensitivity = 70;  //From datasheet, depends on Scale, 2000DPS = 70, 500DPS = 17.5, 250DPS = 8.75. 

// Gyro Variables
int x;
int y;
int z;

void InitSensors(){
  
  Wire.begin();      // Initialize I2C
  
  // ¿200?
  setupL3G4200D(2000);     // Configure L3G4200  - 250, 500 or 2000 deg/sec
  delay(1500); 

  if(!accel.begin())       // Initialize Accelerometer
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }
  
  accel.setRange(ADXL345_RANGE_2_G);    // Configure accel
  
  sensors_event_t event;     // Variable for accel data
  
  // Calculate bias for the Gyro and accel, the values it gives when it's not moving
  // You have to keep the robot vertically and static for a few seconds
  
  for(int i=1; i < 100; i++){      // Takes 100 values to get more precision
    
    getGyroValues();              // Get gyro data
    gyroBiasX += (int)x;
    gyroBiasY += (int)y;
    gyroBiasZ += (int)z;  

    accel.getEvent(&event);       // Get accel data
    accBiasX += event.acceleration.x;
    accBiasY += event.acceleration.y;
    accBiasZ += event.acceleration.z;

    delay(1);
    
  }
  
  // Final bias values for every axis
  gyroBiasX = gyroBiasX / 100;
  gyroBiasY = gyroBiasY / 100;
  gyroBiasZ = gyroBiasZ / 100;

  accBiasX = accBiasX / 100;
  accBiasY = accBiasY / 100;
  accBiasZ = accBiasZ / 100;
  

  //Get Starting Pitch and Roll
  accel.getEvent(&event);
  accRoll = (atan2(event.acceleration.y,-event.acceleration.z)+PI)*RAD_TO_DEG;

  if (accRoll <= 360 & accRoll >= 180){
    accRoll = accRoll - 360;
  }  
  
  //gyroRoll = accRoll;
  
}


void InitialValues(){
  
  //  Accelerometer   
  sensors_event_t event; 
  double InitialAngle = 0;
  double dGyro = 0;
  for(int i=1; i < 100; i++){      // Takes 100 values to get more precision
    
    accel.getEvent(&event);
    accRoll = (atan2(event.acceleration.y-accBiasY,-(event.acceleration.z-accBiasZ))+PI)*RAD_TO_DEG;
        
    getGyroValues();
    gyroRateX = ((int)x - gyroBiasX)*.07; 
  
    dGyro = gyroRateX * ((double)(micros() - timer)/1000000);
    
    InitialAngle = 0.98* (InitialAngle + dGyro) + 0.02 * (accRoll);
    timer = micros();
    
    delay(1);
    
  }
 
  InitialRoll = InitialAngle;
  
  Serial.print("Roll Inicial: ");
  Serial.println(InitialRoll);
  
}


// Roll from accelerometer
double getAccelRoll(){
  
  sensors_event_t event; 
  
  accRoll = (atan2(event.acceleration.y-accBiasY,-(event.acceleration.z-accBiasZ))+PI)*RAD_TO_DEG;   // Calculate the value of the angle 
  
  if (accRoll <= 360 & accRoll >= 180){
    accRoll = accRoll - 360;
  }
  
  return accRoll;
  
}

// Roll from gyroscope
double getGyroRoll(){
  
  getGyroValues();     // Get values from gyro
  
  // read raw angular velocity measurements from device  
  gyroRateX = -((int)x - gyroBiasX)*.07; 
  //gyroRateY = -((int)y - gyroBiasY)*.07; 
  //gyroRateZ = ((int)z - gyroBiasZ)*.07; 
  
  gyroRoll += gyroRateX * ((double)(micros() - timer)/1000000);
  
  return gyroRoll;
  
}

// Angular velocity of Roll by gyroscope
double getDGyroRoll(){
  
  getGyroValues();     // Get values from gyro
  
  // read raw angular velocity measurements from device  
  gyroRateX = -((int)x - gyroBiasX)*.07; 
  //gyroRateY = -((int)y - gyroBiasY)*.07; 
  //gyroRateZ = ((int)z - gyroBiasZ)*.07; 
  
  double dgyroRoll = gyroRateX * ((double)(micros() - timer)/1000000);
  
  return dgyroRoll;
  
}

/////////////////////////////////////////////////////////////
//////////////        Gyro Sensor Code     //////////////////
/////////////////////////////////////////////////////////////


// This part of code if from Jim Lindblom of Sparkfun's code
// used for read the data from the gyro sensor: L3G4200D

void getGyroValues(){

  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  x = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  y = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  z = ((zMSB << 8) | zLSB);
}

int setupL3G4200D(int scale){
  
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
  
}

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
    
}

