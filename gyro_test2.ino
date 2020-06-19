// MPU6050 - MPU6050.ino
//
// Description:
// Retrieves motion data from an MPU6050 sensor module using the
// I2Cdevlib (top level library) and MPU6050 (sub-library) combination.
//
// Created by John Woolsey on 07/21/2018.
// Copyright © 2018 Woolsey Workshop.  All rights reserved.
float scaledResult;
const int numReadings = 10;    //+++++++++++++-------------------------gyro signal stabilisation ( also creatse lag )--------------------++++++++++++++++++++++++++
float gyrogain = 2; //++++++++++----------------- gyrogain setting
float serv ;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average


#define rcPin1 8
#define rcPin2 10
#include <Servo.h>
Servo myservo;  // create servo object to control a servo
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
float gyro = 0;

int ch1 = 0;
int16_t val;  
// MPU6050 library constructor for module
// Uses default I2C address of 0x68; use mpu6050(0x69) to set alternate address
// if needed.
MPU6050 mpu6050;
int16_t ax, ay, az;  // raw accelerometer data register values
int16_t gx, gy, gz;  // raw gyroscope data register values
int16_t tr;          // raw temperature data register value
char buffer[7];      // temporary string buffer; used with dtostrf() function
void setup() {
  
  myservo.attach(9);
  myservo.write(90);
   Wire.begin();          // initialize I2C bus (MPU6050 module)
   Serial.begin(19200);    // initialize serial bus (Serial Monitor)
   mpu6050.initialize();  // initialize MPU6050 sensor module
   // Verify module connection
   Serial.print("MPU6050 module connection ");
   Serial.println(mpu6050.testConnection() ? "successful." : "failed.");
   // Optionally update accelerometer and gyroscope data register offsets
   // Offsets are not retained when module is disconnected from power.
   // Use offset values obtained from running IMU_Zero calibration sketch.
    setMPU6050Offsets(-3108,373,1040,54,11,15); // ++++++++++++++++++++----------------------- input the offset values for how u mount your IMU ----------------+++++++++++++++++++++++++
   Serial.println();  // blank line
   for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
    }
   
}
void loop() {
   // Read raw accel/gyro/temp sensor readings from module
   mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
   ch1 = pulseIn(rcPin1, HIGH, 20000);  
    ch1 = map((ch1), 870, 1950, 0, 180);     
    if ((ch1 < 2)) {
    ch1 = 90 ;
    } 
    if ((ch1)< 90){    
    ch1 = (90-(ch1));
    scaledResult = fscale( 0, 90, 0, 90, ch1, -1.7); // ++++------ last nr sets expo for steering input from the receiver--------+++++++++ 
    ch1 = (90-(scaledResult));
    }
     if ((ch1)> 90){    
    ch1 = ((ch1)-90);
    scaledResult = fscale( 0, 90, 0, 90, ch1, -1.7); // ++++------ last nr sets expo for steering input from the receiver--------+++++++++ 
    ch1 = (90+(scaledResult));
    }  
  total = total - readings[readIndex]; 
  readings[readIndex] = ((gz)/131); 
  total = total + readings[readIndex]; 
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {    
    readIndex = 0;
  } 
  gyro = total / numReadings; 
  serv = ((ch1)+((gyro)*(gyrogain)));
     if ((serv < 1)) {
    serv = 0 ;
    } 
       if ((serv >180)) {
    serv = 180 ;
    } 
  
  serv = map((serv), 0, 180, 30, 150);// ---------++++++++++++ set end points for steering servo here
  Serial.println(serv);
  myservo.write(serv);
   
}



float fscale( float originalMin, float originalMax, float newBegin, float
              newEnd, float inputValue, float curve) {

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;


  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  /*
    Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution
    Serial.println();
  */

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin) {
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  /*
    Serial.print(OriginalRange, DEC);
    Serial.print("   ");
    Serial.print(NewRange, DEC);
    Serial.print("   ");
    Serial.println(zeroRefCurVal, DEC);
    Serial.println();
  */

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0) {
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  }
  else     // invert the ranges
  {
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;
}

// Prints current MPU6050 values
void printMPU6050Values() {
   Serial.print("aX = ");  Serial.print(dtostrf(ax/16384.0, 4, 1, buffer));  Serial.print(" g, ");
   Serial.print("aY = ");  Serial.print(dtostrf(ay/16384.0, 4, 1, buffer));  Serial.print(" g, ");
   Serial.print("aZ = ");  Serial.print(dtostrf(az/16384.0, 4, 1, buffer));  Serial.print(" g, ");
   Serial.print("gX = ");  Serial.print(dtostrf(gx/131.0, 6, 1, buffer));  Serial.print(" °/s, ");
   Serial.print("gY = ");  Serial.print(dtostrf(gy/131.0, 6, 1, buffer));  Serial.print(" °/s, ");
   Serial.print("gZ = ");  Serial.print(dtostrf(gz/131.0, 6, 1, buffer));  Serial.print(" °/s, ");
   Serial.print("T = ");  Serial.print(dtostrf(tr/340.0+36.53, 5, 1, buffer));  Serial.println(" °C");
}
// Prints current MPU6050 offsets
void printMPU6050Offsets() {
    Serial.print("MPU6050 offsets: ");
    Serial.print("aX = ");  Serial.print(dtostrf(mpu6050.getXAccelOffset(), 5, 0, buffer));
    Serial.print(", aY = ");  Serial.print(dtostrf(mpu6050.getYAccelOffset(), 5, 0, buffer));
    Serial.print(", aZ = ");  Serial.print(dtostrf(mpu6050.getZAccelOffset(), 5, 0, buffer));
    Serial.print(", gX = ");  Serial.print(dtostrf(mpu6050.getXGyroOffset(), 5, 0, buffer));
    Serial.print(", gY = ");  Serial.print(dtostrf(mpu6050.getYGyroOffset(), 5, 0, buffer));
    Serial.print(", gZ = ");  Serial.println(dtostrf(mpu6050.getZGyroOffset(), 5, 0, buffer));
}
// Sets new MPU6050 offsets
void setMPU6050Offsets(int16_t aX, int16_t aY, int16_t aZ, int16_t gX, int16_t gY, int16_t gZ) {
   Serial.print("Old ");  printMPU6050Offsets();
   mpu6050.setXAccelOffset(aX);
   mpu6050.setYAccelOffset(aY);
   mpu6050.setZAccelOffset(aZ);
   mpu6050.setXGyroOffset(gX);
   mpu6050.setYGyroOffset(gY);
   mpu6050.setZGyroOffset(gZ);
   Serial.print("New ");  printMPU6050Offsets();
}
