#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h" 
#include "SparkFun_VCNL4040_Arduino_Library.h"
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "math.h"
VCNL4040 proximitySensor;

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3
ICM_20948_I2C myICM;  // create an ICM_20948_I2C object
SCMD myMotorDriver; //This creates the main object of one motor driver and connected slaves.

SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

#define AD0_VAL   1     // The value of the last bit of the I2C address. 
// motot constants
#define L_MOTOR 0
#define R_MOTOR 1
#define FWD 0
#define REV 1

// Global Variables
float pitch_a;
float roll_a;
float alpha = 0.6;
float pitch_a_LPF;
float old_pitch = 0;
float old_pitch_a = 0;
float roll_a_LPF;
float old_roll = 0;
float old_roll_a = 0;
float pitch_g=0;
float roll_g=0;
float yaw_g=0;
float pitch=0;
float roll=0;
float xm;
float ym;
float yaw;
unsigned long t0; // start time
unsigned long dt=0; // change in time

void setup(void)
{
  Serial.begin(115200);
  
  //motor setup
  myMotorDriver.settings.commInterface = I2C_MODE;
  myMotorDriver.settings.I2CAddress = 0x5D; //config pattern is "1000" (default) on board for address 0x5D
  myMotorDriver.settings.chipSelectPin = 10;
  //*****initialize the driver get wait for idle*****//
  while ( myMotorDriver.begin() != 0xA9 ) //Wait until a valid ID word is returned
  {
    Serial.println( "ID mismatch, trying again" );
    delay(500);
  }
  Serial.println( "ID matches 0xA9" );
  //  Check to make sure the driver is done looking for slaves before beginning
  Serial.print("Waiting for enumeration...");
  while ( myMotorDriver.ready() == false );
  // motor 1 inversion so that foward is the same for both motors
  while ( myMotorDriver.busy() ); //Waits until the SCMD is available.
  myMotorDriver.inversionMode(1, 1); //invert motor 1
  while ( myMotorDriver.busy() );
  myMotorDriver.enable();

  // sensor setup  
  Wire.begin();
  Wire.setClock(400000);
  bool initialized = false;
  while( !initialized )
  {

    myICM.begin( Wire, AD0_VAL );

    Serial.print( F("Initialization of the sensor returned: ") );
    Serial.println( myICM.statusString() );
    if( myICM.status != ICM_20948_Stat_Ok )
    {
      Serial.println( "Trying again..." );
      delay(500);
    }
    else{
      initialized = true;
    }
  }
  myICM.begin( Wire, AD0_VAL );
  if (proximitySensor.begin() == false)
  {
    Serial.println("Device not found. Please check wiring.");
    while (1); //Freeze!
  }
  //Serial.println("VL53L1X Qwiic Test");
  //VL53L1_SetInterMeasurementPeriodMilliSeconds(&VL53L1Dev, 1000 );
  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1);
  }
  Serial.println("Sensors online!");
  distanceSensor.setTimingBudgetInMs(50);
  distanceSensor.setIntermeasurementPeriod(5);
  distanceSensor.setDistanceModeShort();
  distanceSensor.startRanging(); //just continue ranging the whole time to save time turning it on/off
}



void loop(void)
{
  if( myICM.dataReady() ){
    t0 = micros();
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
    
    Serial.print("\n");
    
    //---------------------roll---------------------
    roll_a = -atan2(myICM.accY(),myICM.accZ())*180/M_PI;//flip sign to be consistent with gyro data
    roll_a_LPF = 0.2*roll_a+(1-0.2)*old_roll_a;
    old_roll_a = roll_a_LPF;
    roll_g = roll_g-myICM.gyrX()*(float)dt/1000000;
    roll = (old_roll+roll_g*dt/1000000)*(1-alpha)+roll_a_LPF*alpha;
    old_roll = roll;
    Serial.print(roll);
    Serial.print("\t");
    
    //---------------------pitch---------------------
    pitch_a = atan2(myICM.accX(),myICM.accZ())*180/M_PI;
    pitch_a_LPF = 0.2*pitch_a+(1-0.2)*old_pitch_a;
    old_pitch_a = pitch_a_LPF;
    pitch_g = pitch_g-myICM.gyrY()*(float)dt/1000000;
    pitch = (old_pitch+pitch_g*dt/1000000)*(1-alpha)+pitch_a_LPF*alpha;
    old_pitch = pitch;
    Serial.print(pitch);
    Serial.print("\t");
    
    //---------------------yaw---------------------
    yaw_g = yaw_g-myICM.gyrZ()*(float)dt/1000000;
    
    xm = myICM.magX()*cos(pitch*M_PI/180)-myICM.magY()*sin(roll*M_PI/180)*sin(pitch)+myICM.magZ()*cos(roll*M_PI/180)*sin(pitch*M_PI/180); //these were saying theta=pitch and roll=phi
    ym = myICM.magY()*cos(roll*M_PI/180) + myICM.magZ()*sin(roll*M_PI/180);
    yaw = atan2(ym, xm)*180/M_PI;
    Serial.print(yaw_g);
    Serial.print("\t");
    
    delay(30);
    dt = (micros()-t0); //step size in microseconds
  }
  else{
    Serial.println("Waiting for data");
    delay(500);
  }
  myMotorDriver.setDrive( L_MOTOR, FWD, 120);
  myMotorDriver.setDrive( R_MOTOR, FWD, 120);
  delay(100);
  
  myMotorDriver.setDrive( L_MOTOR, FWD, 0);
  myMotorDriver.setDrive( R_MOTOR, FWD, 0);
  while(1){};
  /*
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from ToF sensor
  unsigned int proxValue = proximitySensor.getProximity();  //Get result from prox sensor
  distanceSensor.clearInterrupt();

  
  Serial.print("Distance(mm): ");
  Serial.print(distance);
  Serial.print("\tProx: ");
  Serial.print(proxValue);
  Serial.print("\n");
  
  
  byte rangeStatus = distanceSensor.getRangeStatus();
  if(rangeStatus==0)  //only act if sensor reading was good
  {
    if((distance==0 || distance>300) && proxValue<20) //0 if there is no object nearby
    {
      myMotorDriver.setDrive( L_MOTOR, FWD, 120);
      myMotorDriver.setDrive( R_MOTOR, FWD, 120);
    }
    else
    {
      // hit the breaks
      myMotorDriver.setDrive( L_MOTOR, REV, 225);
      myMotorDriver.setDrive( R_MOTOR, REV, 225);
      delay(200);
  
      //turn
      myMotorDriver.setDrive( L_MOTOR, FWD, 200);
      myMotorDriver.setDrive( R_MOTOR, REV, 200);
      delay(300);
    }
  }*/

  //Serial.println();
}
