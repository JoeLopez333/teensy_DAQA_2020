/*Joe Lopez Illini Formula Electric 2020
 * Logging input from CAN, 9-DOF accelerometer, Cooling loop thermistors
 * the code aint jank, the programmer is
 * code for accelerometer adapted from Kris Winer's LSM6DSM library for arduino
 * https://github.com/kriswiner/LSM6DSM_LIS2MDL_LPS22HB 
 */
#include <Wire.h>
#include <FlexCAN_T4.h>
#include <TimeLib.h>
#include "LSM6DSM.h"
#include "LIS2MDL.h"
//note to self -> getAres() and getGres() will return 0.0 if switch-case is unsuccessful

#define LSM6DSM_ADDRESS 0x6A
#define LSM6DSM_intPin1 9  // interrupt1 pin definitions, significant motion
#define LSM6DSM_intPin2 10   // interrupt2 pin definitions, data ready

uint8_t Ascale = AFS_2G, Gscale = GFS_245DPS, AODR = AODR_12_5Hz, GODR = GODR_12_5Hz;

float aRes, gRes;              // scale resolutions per LSB for the accel and gyro sensor2
float accelBias[3] = {0}, gyroBias[3] = {0};//, -2.89, -0.81}; // offset biases for the accel and gyro
int16_t LSM6DSMData[7];        // Stores the 16-bit signed sensor output
float   Gtemperature;           // Stores the real internal gyro temperature in degrees Celsius
float ax, ay, az, gx, gy, gz;  // variables to hold latest accel/gyro data values 

volatile bool newLSM6DSMData = false;
volatile bool newLSM6DSMTap  = false;

LSM6DSM LSM6DSM(LSM6DSM_intPin1, LSM6DSM_intPin2); // instantiate LSM6DSM class

//LIS2MDL definitions
#define LIS2MDL_intPin  12 // interrupt for magnetometer data ready

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are: MODR_10Hz, MOIDR_20Hz, MODR_50 Hz and MODR_100Hz
*/ 
uint8_t MODR = MODR_20Hz;

float mRes = 0.0015f;            // mag sensitivity
float magBias[3] = {-778.5,-166.5,103.5}, magScale[3]  = {1.08,1.46,.72}; // Bias corrections for magnetometer
int16_t LIS2MDLData[4];          // Stores the 16-bit signed sensor output
float Mtemperature;              // Stores the real internal chip temperature in degrees Celsius
float mx, my, mz;                // variables to hold latest mag data values 
uint8_t LIS2MDLstatus;

volatile bool newLIS2MDLData = false;

LIS2MDL LIS2MDL(LIS2MDL_intPin); // instantiate LIS2MDL class

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
uint32_t delt_t = 0;                      // used to control display output rate
uint32_t sumCount = 0;                    // used to control display output rate
float pitch, yaw, roll;                   // absolute orientation
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

int availablebytes;
char databuf[64]; //used for data transmit to pi

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> myCan; //initalize to CAN 2.0 mode
CAN_message_t msg;

int cur_hour, cur_min, cur_second;
int cur_millis;
bool time_computed = false; //jank fix to time crashing in accelerometer

//Cooling loop stuff
IntervalTimer coolingTimer;
#define COOL_RADIATORBACK  14 //behind radiator
#define COOL_PUMPLEFT      15 //behind boxes
#define COOL_PUMPRIGHT     18 //right next to pump
#define COOL_RADIATORFRONT 19 //in front of radiator
volatile int cooling_val = 0;
volatile uint8_t cooling_counter = 0;
volatile bool cooling_delta = false;
volatile char cooling_data[128];

int ledPin = 13;

void setup() {
  // put your setup code here, to run once:

  //Set teensy time to RTC time
  setTime(Teensy3Clock.get());
  delay(10);
  
  Serial.begin(115200);
  delay(100);

  pinMode(ledPin, OUTPUT);

  //set thermistor analog inputs
  pinMode(COOL_RADIATORBACK, INPUT);
  pinMode(COOL_RADIATORBACK, INPUT);
  pinMode(COOL_RADIATORBACK, INPUT);
  pinMode(COOL_RADIATORBACK, INPUT);

  //Initialize CAN inputs
  myCan.setRx(DEF);
  myCan.setTx(DEF);

  //Initialize accel interrupt pins
  pinMode(LSM6DSM_intPin1, INPUT);
  pinMode(LSM6DSM_intPin2, INPUT);
  pinMode(LIS2MDL_intPin, INPUT);
  //I2C pins for accelerometer
  Wire1.setSCL(16);
  Wire1.setSDA(17);
  Wire1.begin();

  LSM6DSM.I2Cscan();
  // Read the LSM6DSM Chip ID register, this is a good test of communication
  Serial.println("LSM6DSM accel/gyro...");
  byte c = LSM6DSM.getChipID();  // Read CHIP_ID register for LSM6DSM
  Serial.print("LSM6DSM "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x6A, HEX);
  //Serial.println(" ");
  delay(100); 

  
  // Read the LIS2MDL Chip ID register, this is a good test of communication
  Serial.println("LIS2MDL mag...");
  byte d = LIS2MDL.getChipID();  // Read CHIP_ID register for LSM6DSM
  Serial.print("LIS2MDL "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x40, HEX);
  Serial.println(" ");
  delay(1000); 

  LSM6DSM.reset();  

  aRes = LSM6DSM.getAres(Ascale);
  gRes = LSM6DSM.getGres(Gscale);
   
  //LSM6DSM.selfTest();
  LSM6DSM.init(Ascale, Gscale, AODR, GODR);
  LSM6DSM.selfTest();

  //LSM6DSM.offsetBias(gyroBias, accelBias);
  Serial.println("accel biases (mg)"); Serial.println(1000.0f * accelBias[0]); Serial.println(1000.0f * accelBias[1]); Serial.println(1000.0f * accelBias[2]);
  Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
  delay(100); 

  LIS2MDL.reset(); // software reset LIS2MDL to default registers

   mRes = 0.0015f;  // fixed sensitivity and full scale (+/- 49.152 Gauss); 
   
   LIS2MDL.init(MODR);

   LIS2MDL.selfTest();

   //LIS2MDL.offsetBias(magBias, magScale);
   Serial.println("mag biases (mG)"); Serial.println(1000.0f * magBias[0]); Serial.println(1000.0f * magBias[1]); Serial.println(1000.0f * magBias[2]); 
   Serial.println("mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]); 
   delay(2000); // add delay to see results before serial spew of data

   //Initialize CAN rx to 250k bit/s
   myCan.begin();
   //myCan.enableFIFO();
   myCan.setBaudRate(250000);

   //enable interrupts
   attachInterrupt(digitalPinToInterrupt(LSM6DSM_intPin1), accelgyroIntHandler, CHANGE);
   attachInterrupt(digitalPinToInterrupt(LSM6DSM_intPin2), accelgyroIntHandler, CHANGE);
   attachInterrupt(digitalPinToInterrupt(LIS2MDL_intPin), magIntHandler, CHANGE);
   coolingTimer.priority(130); //lower priority than other interrupts
   coolingTimer.begin(cooling, 500000); //timer interval in us
}

void loop() {
  cli();
  if (myCan.read(msg)){
    
    //digitalWrite(ledPin, HIGH);
    //Serial.println("Message");
    updateTime();
    sprintf(databuf, "0 %4X %X/%X/%X/%X/%X/%X/%X/%X %d-%d-%d-%d", msg.id, msg.buf[0],msg.buf[1],  msg.buf[2],msg.buf[3],msg.buf[4],msg.buf[5],msg.buf[6],msg.buf[7], cur_hour, cur_min, cur_second, cur_millis);
    Serial.println(databuf);
    
  }
  sei();

  
  if (newLSM6DSMData){
    cli();
    newLSM6DSMData = false;
    LSM6DSM.readData(LSM6DSMData);

    if (!time_computed)
      updateTime();

    // Now we'll calculate the accleration value into actual g's
     ax = (float)LSM6DSMData[4]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
     ay = (float)LSM6DSMData[5]*aRes - accelBias[1];   
     az = (float)LSM6DSMData[6]*aRes - accelBias[2];  

   // Calculate the gyro value into actual degrees per second
     gx = (float)LSM6DSMData[1]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
     gy = (float)LSM6DSMData[2]*gRes - gyroBias[1];  
     gz = (float)LSM6DSMData[3]*gRes - gyroBias[2];

     //MadgwickQuaternionUpdate(-ax, ay, az, gx*pi/180.0f, -gy*pi/180.0f, -gz*pi/180.0f,  mx,  my, -mz);
     //can calculate these off car

     //slight formatting
     sprintf(databuf, "3 %4f %4f %4f %4f %4f %4f %4f %4f %4f %d-%d-%d-%d", (int)1000*ax, (int)1000*ay, (int)1000*az, gx, gy, gz, 
              (int)1000*mx, (int)1000*my, (int)1000*mz, cur_hour, cur_min, cur_second, cur_millis);
     Serial.println(databuf);
     sei();
  }

  if (newLIS2MDLData){
    cli();
    newLIS2MDLData = false;
    LIS2MDLstatus = LIS2MDL.status();
     
     if(LIS2MDLstatus & 0x08) // if all axes have new data ready
     {
         LIS2MDL.readData(LIS2MDLData); 
   
         //Now we'll calculate the accleration value into actual G's
         mx = (float)LIS2MDLData[0]*mRes - magBias[0];  // get actual G value 
         my = (float)LIS2MDLData[1]*mRes - magBias[1];   
         mz = (float)LIS2MDLData[2]*mRes - magBias[2]; 
         mx *= magScale[0];
         my *= magScale[1];
         mz *= magScale[2];  
     }
     sei();
  }

  if (cooling_delta){
    cli();
    if (!time_computed)
      updateTime();
    sprintf(databuf, "5 %d %d %d-%d-%d-%d", cooling_val, cooling_counter, cur_hour, cur_min, cur_second, cur_millis);
    Serial.println(databuf);
    cooling_delta = false;
    digitalWrite(ledPin, cooling_counter %2);
    sei();
  }
  time_computed = false; 
}

void accelgyroIntHandler(){
  newLSM6DSMData = true;
}

void magIntHandler(){
  newLIS2MDLData = true;
}

//IntervalTimer coolingTimer;
//#define COOL_RADIATORBACK  14 //behind radiator
//#define COOL_PUMPLEFT      15 //behind boxes
//#define COOL_PUMPRIGHT     18 //right next to pump
//#define COOL_RADIATORFRONT 19 //in front of radiator
//volatile int cooling_val = 0;
//volatile uint8_t cooling_counter = 0;
//volatile bool cooling_delta = false;
//volatile char cooling_data[128];
void cooling(){
  switch(cooling_counter){
    case(0): cooling_val = analogRead(COOL_RADIATORBACK); break;//digitalWrite(ledPin, LOW);break;
    case(1): cooling_val = analogRead(COOL_PUMPLEFT); break; //digitalWrite(ledPin, LOW); break;
    case(2): cooling_val = analogRead(COOL_PUMPRIGHT); break; //digitalWrite(ledPin, LOW); break;
    case(3): cooling_val = analogRead(COOL_RADIATORFRONT); break; //digitalWrite(ledPin, LOW); break;
  }
  //Rotate through the 4 thermistors at 2 Hz
  cooling_counter++;
  cooling_counter %= 4;
  //myCan.begin();
  //notify program we have data to send
  cooling_delta = true; 
}

//Only should update time once per loop
//Also only if there is data available to be sent
void updateTime(){
  cli();
  cur_hour = hour();
  cur_min = minute();
  cur_second = second();
  cur_millis = millis() %1000;
  time_computed = true;
  sei();
}

  
