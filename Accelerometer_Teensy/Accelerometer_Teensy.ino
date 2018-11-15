// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>
#include<Filters.h>

// filters out changes faster that 5 Hz.
float filterFrequency = 1.0; 

// create a one pole (RC) lowpass filter
FilterOnePole lowpassFilterX( LOWPASS, filterFrequency );
FilterOnePole lowpassFilterY( LOWPASS, filterFrequency );
FilterOnePole lowpassFilterZ( LOWPASS, filterFrequency );

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
const float accRange = 4.0*9.81; //ms^-2
const float gyroRange = 2000.0*180/3.1415926535; //Radians per second
const float radiansToDegrees = 180/3.1415926535;

typedef struct
{
  float x = 0;
  float y = 0;
  float z = 0;
}vect3d;

vect3d acc;
vect3d rot;

//Returns dt in seconds
long prevTime;
float getDt()
{
  float dt = (float)(millis() - prevTime)*0.001;
  prevTime = millis();
  return dt;
}

void setup()
{
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}


void loop()
{  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //Filter readings
  lowpassFilterX.input(AcX);
  lowpassFilterY.input(AcY);
  lowpassFilterZ.input(AcZ);
  
  //Scale acceleration values
  acc.x = (float)lowpassFilterX.Y/65535.0*accRange;
  acc.y = (float)lowpassFilterY.Y/65535.0*accRange;
  acc.z = (float)lowpassFilterZ.Y/65535.0*accRange;

  //Calulate rotation from gravity vector
  rot.x = atan(acc.y/acc.z)*radiansToDegrees;
  rot.y = atan(acc.x/acc.z)*radiansToDegrees;
  rot.z = atan(acc.y/acc.x)*radiansToDegrees;

  //Turn led on, control example
  if (abs(rot.x) > 30.0 || abs(rot.y) > 30.0)
  {
    digitalWrite(13, HIGH);
  }
  else
  {
    digitalWrite(13, LOW);
  }
  
//  Serial.print("accX = "); Serial.print(acc.x);
//  Serial.print(" | accY = "); Serial.print(acc.y);
//  Serial.print(" | accZ = "); Serial.println(acc.z);
  Serial.print("rotX = "); Serial.print(rot.x);
  Serial.print(" | rotY = "); Serial.println(rot.y);
//  Serial.print(" | rotZ = "); Serial.println(rot.z);
  
//  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
//  Serial.print(" | GyX = "); Serial.print(GyX);
//  Serial.print(" | GyY = "); Serial.print(GyY);
//  Serial.print(" | GyZ = "); Serial.println(GyZ);
  delay(10);
}
