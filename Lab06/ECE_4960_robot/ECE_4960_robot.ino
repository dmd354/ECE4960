#include <stdint.h>
// maximum length of reply / data message
#define MAXREPLY 100
#define TODO_VAL 0
// buffer to reply to client
uint8_t val[MAXREPLY];
uint8_t *val_data = &val[2]; // start of optional data area
uint8_t *val_len = &val[1];  // store length of optional data

/********************************************************************************************************************
                 INCLUDES
 *******************************************************************************************************************/
#include "BLE_example.h"
#include "commands.h"
#include "related_funcs.h"

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

/********************************************************************************************************************
                  OBJECTS
  *******************************************************************************************************************/

/********************************************************************************************************************
                GLOBAL VARIABLES
 *******************************************************************************************************************/
String s_Rev = "Rev 1.0";
String s_Rcvd = "100"; //KHE declare extern in BLE_example_funcs.cpp to accept messages, if 100 don't bother checking case statements
uint16_t l_Rcvd = 0;
uint8_t *m_Rcvd = NULL;
String s_AdvName = "MyRobot"; //KHE 2 0 TOTAL CHARACHTERS ONLY!!  any more will be dropped
uint32_t pkg_count = 0;

cmd_t empty_cmd = {NOT_A_COMMAND, 1, {0}};
cmd_t *cmd = &empty_cmd;
cmd_t *res_cmd = &empty_cmd;
bt_debug_msg_t *bt_debug_head = NULL;
bt_debug_msg_t *bt_debug_tail = NULL;
//SCMD motorDriver;
present_t presentSensors = {
    .motorDriver = 0,
    .ToF_sensor = 0,
    .prox_sensor = 0,
    .IMU = 0};
int bytestream_active = 0;

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
float gyro_z;
unsigned long t0; // start time
unsigned long dt=0; // change in time
unsigned char L_motor_val=1;  //value to send to left motor
unsigned char R_motor_val=1;  //value to send to right motor
unsigned int counts_control=0;
unsigned int counts_data=0;
bool increase=true;

/*************************************************************************************************/
/*!
     \fn     setup

     \brief  Arduino setup function.  Set up the board BLE and GPIO - BLE first...

     \param  none

     \called Arduino startup

     \return None.
 */
/*************************************************************************************************/
void setup()
{
//-----------------------------------SETUP FOR BT-----------------------------------------------------------------
    
    Serial.begin(115200);
    delay(1000);

#ifdef BLE_SHOW_DATA
//SERIAL_PORT.begin(115200);
//delay(1000);
//SERIAL_PORT.printf("Viper. Compiled: %s\n" , __TIME__);
#endif

#ifdef AM_DEBUG_PRINTF
    //
    // Enable printing to the console.
    //
    enable_print_interface();
#endif

    //Serial.printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    Serial.print("Revision = ");
    Serial.print(s_Rev);
    Serial.printf("  ECE 4960 Robot Compiled: %s   %s\n", __DATE__, __TIME__);
    //Serial.printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    analogWriteResolution(16); //Set AnalogWrite resolution to 16 bit = 0 - 65535 (but make max 64k or trouble)

    /********************************************************************************************************************
                    Set Advertising name:  uses global string s_AdvName set above.
     *******************************************************************************************************************/
    set_Adv_Name(); //BLE_example_funcs.cpp near end of file, fn declared extern in BLE_example.h

    /********************************************************************************************************************
                     Boot the radio
                      SDK/third_party/exactle/sw/hci/apollo3/hci_drv_apollo3.c
                      = huge program to handle the ble radio stuff in this file
     *******************************************************************************************************************/
    HciDrvRadioBoot(0);

    /************************************************************************************************
          Initialize the main ExactLE stack: BLE_example_funcs.cpp
          - One time timer
          - timer for handler
          - dynamic memory buffer
          - security
          - HCI host conroller interface
          - DM device manager software
          - L2CAP data transfer management
          - ATT - Low Energy handlers
          - SMP - Low Energy security
          - APP - application handlers..global settings..etc
          - NUS - nordic location services

     ************************************************************************************************/
    exactle_stack_init();

    /*************************************************************************************************
        Set the power level to it's maximum of 15 decimal...defined in hci_drv_apollo3.h as 0xF
        needs to come after the HCI stack is initialized in previous line
          - poss. levels = 0x03=-20,0x04=-10,0x05=-5,0x08=0,0x0F=4 but have to use definitions, not these ints
            extremes make a difference of about 10 at 1 foot.
     ************************************************************************************************/
    HciVsA3_SetRfPowerLevelEx(TX_POWER_LEVEL_PLUS_3P0_dBm); //= 15 decimal = max power WORKS..default = 0

    /*************************************************************************************************
        Start the "Amdtp" (AmbiqMicro Data Transfer Protocol) profile. Function in amdtp_main.c

         Register for stack callbacks
         - Register callback with DM for scan and advertising events with security
         - Register callback with Connection Manager with client id
         - Register callback with ATT low energy handlers
         - Register callback with ATT low enerty connection handlers
         - Register callback with ATT CCC = client charachteristic configuration array
         - Register for app framework discovery callbacks
         - Initialize attribute server database
         - Reset the device

     ************************************************************************************************/
    AmdtpStart();

    /*************************************************************************************************
       On first boot after upload and boot from battery, pwm on pin 14 not working
        need to reset nano board several times with battery power applied to get
        working.  Delay 5 seconds works..haven't tried lesser values.
     ************************************************************************************************/
    //delay(5000);

    /************************************************************************************************
        Arduino device GPIO control setup.
          Place after board BLE setup stuff happens.  ie.:
            could not get A14 to PWM untill I moved the set_stop() call from the
            beginning of setup to this location...then works great.
     ************************************************************************************************/

    pinMode(LED_BUILTIN, OUTPUT);

    //Set a starting point...for motors, servo, and LED_BUILTIN
    //delay(1000);

    // Bluetooth would start after blinking
    for (int i = 0; i < 20; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
    }

    //setupwdt();

    pinMode(blinkPin, OUTPUT);
    digitalWrite(blinkPin, LOW);

    // Configure the watchdog.
    //setupTimerA(myTimer, 31); // timerNum, period - //moved to BLE_example_funcs.cpp scheduler_timer_init
    setupWdt();
    am_hal_wdt_init(&g_sWatchdogConfig);
    //NVIC_EnableIRQ(CTIMER_IRQn); // Enable CTIMER interrupt in nested vector interrupt controller.
    NVIC_EnableIRQ(WDT_IRQn); // Enable WDT interrupt in nested vector interrupt controller.

    uint8_t a = 0;
    m_Rcvd = &a;
    //Serial.printf("Size of command: %d", sizeof(cmd_t));
    am_hal_interrupt_master_enable();
    //interrupts(); // Enable interrupt operation. Equivalent to am_hal_rtc_int_enable().
    //am_hal_wdt_start();
    //am_hal_wdt_int_enable(); - freezes boot
//-----------------------------------SETUP FOR MOTORS-----------------------------------------------------------------
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

//-----------------------------------SETUP FOR SENSORS-----------------------------------------------------------------
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
} /*** END setup FCN ***/

void loop()
{
  //------------------------------------ROBOT CONTROL----------------------------------------------------
  t0 = micros();
  counts_control++; //increment counts
  if(counts_control>200)
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
  myMotorDriver.setDrive( L_MOTOR, REV, L_motor_val); 
  myMotorDriver.setDrive( R_MOTOR, FWD, R_motor_val);

  
  if( myICM.dataReady() )
  {
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
    
    //---------------------yaw---------------------
    gyro_z = myICM.gyrZ();  //z measurement from gyroscope (angular speed)
    yaw_g = yaw_g-gyro_z*(float)dt/1000000;
    }
  else{
    Serial.println("Waiting for data");
    delay(500);
  }

    
    //Serial.println("Loop...."); //KHE Loops constantly....no delays
    //--------------------------------------BLUETOOTH-----------------------------------------------------------
    counts_data++;
    if (l_Rcvd > 1) //Check if we have a new message from amdtps_main.c through BLE_example_funcs.cpp
    {

        cmd = (cmd_t *)m_Rcvd;
        /*
        Serial.print("Message Buffer: ");
        for (int i = 0; i < l_Rcvd; i++)
            Serial.printf("%d ", m_Rcvd[i]);
        Serial.println();
        Serial.printf("Got command: 0x%x Length: 0x%x Data: ", cmd->command_type, cmd->length);

        for (int i = 0; i < cmd->length; i++)
        {
            Serial.printf("0x%x ", cmd->data[i]);
        }
        Serial.println();
        */

        switch (cmd->command_type)
        {
        case SET_MOTORS:

            Serial.println("Placeholder: Set Motors");

            break;
        case GET_MOTORS:

            Serial.println("Placeholder: Set Motors");
            //amdtpsSendData((uint8_t *)res_cmd, *val_len);
            break;
        case SER_RX:
            Serial.println("Got a serial message");
            pushMessage((char *)&cmd->data, cmd->length);
            break;
        case REQ_FLOAT:
            Serial.println("Going to send a float");
            //TODO: Put a float (perhaps pi) into a command response and send it.
            res_cmd->command_type = GIVE_FLOAT;     //set command type as GIVE_FLOAT
            res_cmd->length=6;                      //length doesn't matter since the handler will take care of this
            ((float *)(res_cmd->data))[0] = 1.23f;  //put a float into data to send
            amdtpsSendData((uint8_t *)res_cmd, 6);  //2 bytes for type and length, 4 bytes of data
            break;
        case PING:
            Serial.println("Ping Pong");
            cmd->command_type = PONG;
            amdtpsSendData(m_Rcvd, l_Rcvd);
            break;
        case START_BYTESTREAM_TX:
            bytestream_active = (int)cmd->data[0];
            //Serial.printf("Start bytestream with active %d \n", bytestream_active);
            ((uint32_t *)res_cmd->data)[0] = 0;
            bytestream_active = 1;
            break;
        case STOP_BYTESTREAM_TX:
            bytestream_active = 0;
            break;
        default:
            Serial.printf("Unsupported Command 0x%x \n", cmd->command_type);
            break;
        }

        l_Rcvd = 0;
        am_hal_wdt_restart();
        free(m_Rcvd);
    } //End if s_Rcvd != 100
    else if ((s_Rcvd[0] == '6' && s_Rcvd[1] == '7'))
    {
        s_Rcvd[0] = 0;
        digitalWrite(LED_BUILTIN, HIGH);
        //Serial.printf("Connected, length was %d", l_Rcvd);
    }
    else if ((s_Rcvd[0] == '6' && s_Rcvd[1] == '8'))
    {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("disconnected");
        //Decimal value of D for Disconnect
        //Serial.println("got disconnect from case in ino file - set_Stop");
        digitalWrite(LED_BUILTIN, LOW);
        //amdtps_conn_close();
        DmDevReset();
    }

    if (availableMessage())
    {
        Serial.println("Bluetooth Message:");
        Serial.println(pullMessage());
        printOverBluetooth("Message Received.");
    }

    if (bytestream_active)
    {
        if(counts_data>=20) //no need to send ALL the data
        {
          res_cmd->command_type = BYTESTREAM_TX;  //set command type to bytestream transmit
          res_cmd->length = 14;                    //length doesn't matter since the handler will take care of this
          //TODO: Put an example of a 32-bit integer and a 64-bit integer
          //for the stream. Be sure to add a corresponding case in the
          //python program.
          //Serial.printf("Stream %d \n", bytestream_active);
           
          // pack up data to send
          unsigned long t=micros(); //send current time for x axis
          memcpy(res_cmd->data, &t, 4); 
          memcpy(res_cmd->data+4, &L_motor_val, 1);
          memcpy(res_cmd->data+5, &R_motor_val, 1);
          memcpy(res_cmd->data+6, &gyro_z, 4);
          memcpy(res_cmd->data+10, &yaw_g, 4);    
          amdtpsSendData((uint8_t *)res_cmd, 16);  //2 bytes for type and length, 14 bytes of data
          counts_data=0;
        }
    }

    trigger_timers();
    dt = (micros()-t0); //time to run through entire loop in us
    // Disable interrupts.


} //END LOOP
