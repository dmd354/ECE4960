/******************************************************************************
  MotorTest.ino
  Serial Controlled Motor Driver
  Marshall Taylor @ SparkFun Electronics
  Sept 15, 2016
  https://github.com/sparkfun/Serial_Controlled_Motor_Driver
  https://github.com/sparkfun/SparkFun_Serial_Controlled_Motor_Driver_Arduino_Library

  Resources:
  Uses Wire.h for i2c operation
  Uses SPI.h for SPI operation

  Development environment specifics:
  Arduino IDE 1.6.7
  Teensy loader 1.27

  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
  Please review the LICENSE.md file included with this example. If you have any questions
  or concerns with licensing, please contact techsupport@sparkfun.com.
  Distributed as-is; no warranty is given.
******************************************************************************/
//This example steps through all motor positions moving them forward, then backwards.
//To use, connect a redboard to the user port, as many slaves as desired on the expansion
//port, and run the sketch.
//
// Notes:
//    While using SPI, the defualt LEDPIN will not toggle
//    This steps through all 34 motor positions, which takes a few seconds to loop.

#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include "Wire.h"

#define LEDPIN 13

SCMD myMotorDriver; //This creates the main object of one motor driver and connected slaves.



void setup()
{
  Serial.begin(9600);
  pinMode(LEDPIN, OUTPUT);

  Serial.println("Starting sketch.");

  //***** Configure the Motor Driver's Settings *****//
  //  .commInter face can be I2C_MODE or SPI_MODE
  myMotorDriver.settings.commInterface = I2C_MODE;
  //myMotorDriver.settings.commInterface = SPI_MODE;

  //  set address if I2C configuration selected with the config jumpers
  myMotorDriver.settings.I2CAddress = 0x5D; //config pattern is "1000" (default) on board for address 0x5D

  //  set chip select if SPI selected with the config jumpers
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
  Serial.println("Done.");
  Serial.println();

  //*****Set application settings and enable driver*****//

  // motor 1 inversion so that foward is the same for both motors
  while ( myMotorDriver.busy() ); //Waits until the SCMD is available.
  myMotorDriver.inversionMode(1, 1); //invert motor 1

  while ( myMotorDriver.busy() );
  myMotorDriver.enable();

  Serial.println("setup complete");

}
unsigned char L_motor_val=70;  //value to send to left motor
unsigned char R_motor_val=70;  //value to send to right motor
unsigned int counts_control=0;
//set motor constants
#define L_MOTOR 0
#define R_MOTOR 1
#define FWD 0
#define REV 1
bool increase=true;

void loop()
{
  //***** Operate the Motor Driver *****//
  //  It uses .setDrive( motorName, direction, level ) to drive the motors.
  counts_control++; //increment counts
  if(counts_control>0)
  {
    // if its time for the next motor value, incease/decrease it
    if(increase)
    {
      L_motor_val++;
      R_motor_val++;
    }
    else
    {
      L_motor_val--;  //decrease speed
      R_motor_val--;
    }
    counts_control=0;
    if(L_motor_val==255||L_motor_val==0)
    {
        increase = !increase; //switch direction if cant increase anymore
    }
  }
  //set motors
  Serial.println(L_motor_val);
  myMotorDriver.setDrive( L_MOTOR, REV, L_motor_val); 
  myMotorDriver.setDrive( R_MOTOR, FWD, R_motor_val);
  delay(100);
}
