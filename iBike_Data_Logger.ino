/*
  Final Year Project - Autonomous Electric Bicycle
    Completed by Vincent Renders B627526 (v.renders-16@student.lboro.ac.uk)
    Supervised by Georgios Mavros MSc, PhD (g.mavros@lboro.ac.uk)
  This is a data logging program recording data from two gyroscope and accelerometer boards, MPU-6050s, and approximates speed and distance travelled using a hall effect sensor.
*/

/*
  UNO connections to Hall (L to R, solder-side down):
    GND - GND
    VCC - 5V    
    SIG - pin 2
  Comments
    The HE Sensor will not function correctly with the code below with 3.3V supply. Ensure supply is 5V.
*/

/*
  UNO connections to SD card module:
    MOSI - pin 11
    MISO - pin 12
    SCK - pin 13
    CS - pin 10
  MEGA connections to SD card module:
    MOSI - pin 51
    MISO - pin 50
    SCK - pin 52
    CS - pin 53
  Comments
    The SD Card module will not function correctly with the code below with 3.3V supply. Ensure supply is 5V
*/

/*
  UNO connections for the first (L to R) GY-521/1 board:
    3.3V-->VCC
    GND-->GND
    SDA-->A4
    SCL-->A5
    AD0-->3.3V (0x69)
  UNO connections for the second (L to R) GY-521/2 board:
    3.3V-->VCC
    GND-->GND
    SDA-->SDA
    SCL-->SCL
    AD0-->GND (0x68)
  MEGA connections for the first (L to R) GY-521 board:
    VCC-->3.3V
    GND-->GND
    SDA-->A4
    SCL-->A5
    AD0-->3.3V (0x69)
  MEGA connections for the second (L to R) GY-521 board:
    VCC-->3.3V
    GND-->GND
    SDA-->SDA
    SCL-->SCL
    AD0-->GND (0x68)
  Comments
    When the AD0 pin of the GY-521 MPU board is connected to ground then its address is 0x68.
    When the AD0 pin is connected to 3.3V, this address changes to 0x69.
    This makes it possible to communicate separately with the two, using the A4 and A5 pins
*/

// Headers for SD card functions
#include <SPI.h> 
#include <SD.h>

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

// Header for MPU6050 / GY521
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

// Header for I2C communication
#include <Wire.h> 

// change this to match your SD shield or module (UNO = 10, MEGA = 53)
const int chipSelect = 10; 
//const int chipSelect = 53; 

int hallPin = 2;
int ledPin = 7;
int state = 0;

// conversion factors for measurements

float pi = 3.14159265359;
float gravity = 9.81; // m/s^2 per g
float deg_2_rad = pi/180; // degrees to rads

//Initialising parameters for soft realtime keeping 

unsigned long dt_soft = 0;
unsigned long tzero = 0;
unsigned long index = 1;
int corrector = 0;
unsigned long timestep = 40000; // timestep [microsecond] - options: 5000/200Hz, 10000/100Hz, 20000/50Hz, 25000/40Hz, 40000/25Hz (default), 50000/20Hz.
int difference = 0;
int accum = 0;
long microsecs = 1000000; // microseconds in one second
float time_vect = 0; //realtime timestep in s

//Initialising parameters for hall-effect interrupts

volatile long start_time_hall;
volatile long end_time_hall; 
volatile long duration_hall;
const int interrupt_pin_1_hall = digitalPinToInterrupt(2);
volatile byte index_hall;
float duration_secs_hall;

// initialising parameters for MPU - 6050 sensors

const int MPU2=0x68,MPU1=0x69;  // I2C addresses of the MPU-6050s - 1 is 0x69, 2 is 0x68 per configuration above
int16_t AcX1b,AcY1b,AcZ1b,Tmp1b,GyX1b,GyY1b,GyZ1b; // "-b" variable suffix indicates "bit" for bit manipulataion
int16_t AcX2b,AcY2b,AcZ2b,Tmp2b,GyX2b,GyY2b,GyZ2b; // "-b" variable suffix indicates "bit" for bit manipulataion
float AcX1,AcY1,AcZ1,Tmp1,GyX1,GyY1,GyZ1;
float AcX2,AcY2,AcZ2,Tmp2,GyX2,GyY2,GyZ2;

// initialising bike parameters

float wheel_diam = 0.672; // wheel diameter (rim + 2x 25mm Tyre Widths)[m]
float wheel_circ = pi*wheel_diam; // ~ 2.1115 [m]
float wheelbase_L =0.997; // CANYON ULTIMATE CF SL (SIZE L) wheelbase [m]

// initialising speed calculation parameters

float U_x = 0.1; // speed [m/s]
float Ukm = 0; // speed [km/h
float s_m = 0; // metres [m]
float skm = 0; // kilometres [m]
float n_rev = 0; // revolutions ticker

void setup() {
  // Open serial communications:
  Serial.begin(115200);
  Serial.println("Initialising vehicle data logging program...");

  // SD CARD INITALISATION

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    //return;
    
  }

  // SD CARD INITALISATION END

  // HALL EFFECT INTERRUPT SETUP
  
  attachInterrupt(interrupt_pin_1_hall, magnet_detect_hall, RISING);  //Initialize the intterrupt pin (Arduino digital pin 2)
  start_time_hall = 0;
  end_time_hall = 0;
  duration_hall = 10000000; // starting value is 1E7 microsecs (10 s)
  duration_secs_hall = 10; // starting value is 10 s
  index_hall = 0;

  // HALL INTERRUPT END

  // IMU INITIALISATION

  Wire.begin();
  Wire.beginTransmission(MPU1);       // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B);                       // PWR_MGMT_1 register
  Wire.write(0);                          // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU1);       //Start communication with the address found during search.
  Wire.write(0x1C);                       //Writing to the ACCEL_CONFIG register
  Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU1);       //Start communication with the address found during search.
  Wire.write(0x1B);                       //Writing to the GYRO_CONFIG register
  Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 1000 deg/s full scale range)
  Wire.endTransmission(true);

  Wire.begin();
  Wire.beginTransmission(MPU2);       // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B);                       // PWR_MGMT_1 register
  Wire.write(0);                          // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU2);       //Start communication with the address found during search.
  Wire.write(0x1C);                       //Writing to the ACCEL_CONFIG register
  Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU2);       //Start communication with the address found during search.
  Wire.write(0x1B);                       //Writing to the GYRO_CONFIG register
  Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 1000 deg/s full scale range)
  
  // IMU INITIALISATION END
  
  Serial.println("\nProgram Setup Complete.");
  Serial.println("VSS, Odometer, IMU and Data Logging Capabilities Online.");

}

void loop() {
  // run the following only in the first iteration
  if (index == 1)
  {
   tzero = micros();
  }
  
  //***************************************************************************************************************
  //***************************************************************************************************************
  // place the main body of the code to be run in regular time intervals (soft-realtime controlled by a throttler):

  // Hall Effect Loop

  state = digitalRead(hallPin);

  //hall-effect code
  duration_secs_hall = float(duration_hall)/1000000;

  // speed calculation
  U_x=pi*wheel_diam/duration_secs_hall; // [m/s]
  Ukm=U_x*3.6; // [km/h]

  // Odometer - distance calculation
  s_m=wheel_circ*n_rev; // [m]
  skm=s_m/1000; // [km]
  
  if (state == LOW) {        
    digitalWrite(ledPin, HIGH);
  } 
  else {
    digitalWrite(ledPin, LOW);
  }

  // IMU Loop
  // the code below reads accelerations, temperature and angular speeds from the MPU-6050
  
  Wire.beginTransmission(MPU1);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU1,14,true);  // request a total of 14 registers
  AcX1b=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY1b=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ1b=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp1b=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX1b=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY1b=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ1b=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Wire.endTransmission(true);

  AcX1=float(AcX1b)/4096*gravity;     
  AcY1=float(AcY1b)/4096*gravity;
  AcZ1=float(AcZ1b)/4096*gravity;
  Tmp1=float(Tmp1b)/340+36.53;
  GyX1=float(GyX1b)/32.8*deg_2_rad;
  GyY1=float(GyY1b)/32.8*deg_2_rad;
  GyZ1=float(GyZ1b)/32.8*deg_2_rad;

  Wire.beginTransmission(MPU2);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU2,14,true);  // request a total of 14 registers
  AcX2b=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY2b=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ2b=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp2b=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX2b=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY2b=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ2b=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Wire.endTransmission(true);

  AcX2=float(AcX2b)/4096*gravity;     
  AcY2=float(AcY2b)/4096*gravity;
  AcZ2=float(AcZ2b)/4096*gravity;
  Tmp2=float(Tmp2b)/340+36.53;
  GyX2=float(GyX2b)/32.8*deg_2_rad;
  GyY2=float(GyY2b)/32.8*deg_2_rad;
  GyZ2=float(GyZ2b)/32.8*deg_2_rad;

  // SD Card Loop

  String dataString = "";

  time_vect = time_vect + float(dt_soft)/float(microsecs);

  if (index==1){
    dataString = "T(s),U(m/s),U(km/h),S(m),S(km),Temp(C),AcX1(m/s^2),AcY1(m/s^2),AcZ1(m/s^2),GyX1(rad/s),GyY1(rad/s),GyZ1(rad/s),Temp2(C),AcX2(m/s^2),AcY2(m/s^2),AcZ2(m/s^2),GyX2(rad/s),GyY2(rad/s),GyZ2(rad/s)";
  }
  else{
    dataString = String(time_vect); // 01
    dataString +=",";
    dataString +=String(U_x);       // 02
    dataString +=",";
    dataString +=String(Ukm);       // 03
    dataString +=",";
    dataString +=String(s_m);       // 04
    dataString +=",";
    dataString +=String(skm);       // 05
    dataString +=",";
    dataString +=String(Tmp1);      // 06
    dataString +=",";
    dataString +=String(AcX1);      // 07
    dataString +=",";
    dataString +=String(AcY1);      // 08
    dataString +=",";
    dataString +=String(AcZ1);      // 09
    dataString +=",";
    dataString +=String(GyX1);      // 10
    dataString +=",";
    dataString +=String(GyY1);      // 11
    dataString +=",";
    dataString +=String(GyZ1);      // 12
    dataString +=",";
    dataString +=String(Tmp2);      // 13
    dataString +=",";
    dataString +=String(AcX2);      // 14
    dataString +=",";
    dataString +=String(AcY2);      // 15
    dataString +=",";
    dataString +=String(AcZ2);      // 16
    dataString +=",";
    dataString +=String(GyX2);      // 17
    dataString +=",";
    dataString +=String(GyY2);      // 18
    dataString +=",";
    dataString +=String(GyZ2);      // 19
  }
 
  // write sample to file on SD card
  
  File dataFile = SD.open("datalog.csv", FILE_WRITE); // Open the file or create it if none exists. Only one file can be open at a time, you have to close it before opening another.

  if (dataFile) {
    // if the file opened okay, write to it:
    dataFile.println(dataString);
    // save data & close the file:
    dataFile.close();
  }
  else {
    // if the file didn't open, print an error:
    Serial.println("\nError opening iBikeData.csv");
  }

  // print to the serial monitor:
  Serial.println(dataString);

  // end of main body of code
  //***************************************************************************************************************
  //***************************************************************************************************************

  // apply a delay of one microsecond for as long as dt_soft is smaller than (timestep + corrector)  
  do
  {
  delayMicroseconds(1); 
  dt_soft = micros()-tzero;
  }while(dt_soft<timestep+corrector);
  tzero = micros();
  difference = dt_soft - timestep;
  
  // adjust the value of the corrector depending on whether at the previous timestep the soft timestp was greater or smaller than the 
  
  accum = accum + difference; // offset in accumulated time
  corrector = -accum;
  
  // end of corrector adjustments
  
  index++; // advance the index by one
  
}

void magnet_detect_hall() //This function is called whenever a magnet/interrupt is detected by the arduino
  {
  if (index_hall == 0)
  {
   start_time_hall=micros();
   index_hall = 1;
  }
  else
  {
   end_time_hall = micros();
   duration_hall = end_time_hall-start_time_hall;
   start_time_hall = end_time_hall;
  }

  n_rev=n_rev + 1;
  
 }
