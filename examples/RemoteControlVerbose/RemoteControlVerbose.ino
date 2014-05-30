/************************************************************************
* File Name          : RemoteControlVerbose
* Author             : Evan
* Updated            : Evladov
* Version            : V0.0.1
* Date               : 30 May, 2014
* Description        : Serial Terminal Control 
* License            : 
* Copyright(C) 2014 UFactory Team. All right reserved.
*************************************************************************/
#include <Servo.h>
#include <EEPROM.h>
#include <UF_uArm.h>



/* Include definition of serial commands */
#include "commands.h"

/* Serial port baud rate */
#define BAUDRATE     9600
//#define BAUDRATE     115200


//#define NO_LIMIT_SWITCH
//#define PIEZOBUZZER

/* Variable initialization */
// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

UF_uArm uarm;           // initialize the uArm library 


/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int arm_args[5];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1
, arg2);
    Serial.println("OK");
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK");
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case SERVO_WRITE:
    //servos[arg1].write(arg2);
    Serial.println("NOT SUPPORTED");
    break;
  case SERVO_READ:
    //Serial.println(servos[arg1].read());
    Serial.println("NOT SUPPORTED");
    break;
   case ARM_TEST:   
    Serial.println("OK");
    while(digitalRead(BTN_D7)){
      motion();
      motionReturn();
    } 
    break;
  case ARM_CALIBRATION:
    uarm.manual_calibration(arg1, arg2);
    Serial.println("OK");
    break;
  case ARM_ALERT:
    if(!digitalRead(BTN_D7)) uarm.alert(1, 20, 0);
    Serial.println("OK");
    break;
  case ARM_POSITION:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       arm_args[i] = atoi(str);
       i++;
    }
    //setPIDParams(pid_args[0], pid_args[1], pid_args[2], pid_args[3], PID_RATE);
    // uarm.setPosition(stretchTemp, heightTemp, rotationTemp, handRotTemp);
    uarm.setPosition(arm_args[0], arm_args[1], arm_args[2], arm_args[3]);
    /* pump action, Valve Stop. */
    if(arm_args[4] & CATCH)   uarm.gripperCatch();
    /* pump stop, Valve action.
       Note: The air relief valve can not work for a long time,
       should be less than ten minutes. */
    if(arm_args[4] & RELEASE) uarm.gripperRelease();

    Serial.println("OK");
    break;

  default:
    Serial.println("Invalid Command");
    break;
  }
}



void setup() 
{
  Serial.begin(BAUDRATE);   // start serial port
  while(!Serial);   // wait for serial port to connect. Needed for Leonardo only
  uarm.init();          // initialize the uArm position
}

void loop()
{
//  uarm.calibration();   // if corrected, you could remove it, no harm though

  while (Serial.available() > 0) {

    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

  /* delay release valve, this function must be in the main loop */
  uarm.gripperDetach();  
} 


void motion()
{
  uarm.setPosition(60, 80, 0, 0);    // stretch out
  delay(400);
  uarm.setPosition(60, 0, 0, 0);  // down
  delay(400);
  uarm.gripperCatch();               // catch
  delay(400);
  uarm.setPosition(60, 80, 0, 0);    // up
  delay(400);
  uarm.setPosition(60, 80, 35, 0);   // rotate
  delay(400);
  uarm.setPosition(60, 0, 35, 0); // down
  delay(400);
  uarm.gripperRelease();             // release
  delay(100);
  uarm.setPosition(60, 80, 35, 0);   // up
  delay(400);
  uarm.setPosition(0, 80, 0, 0);      // original position
  delay(400);
  uarm.gripperDirectDetach();        // direct detach 
  delay(500);
}

void motionReturn()
{
  uarm.setPosition(60, 80, 35, 0);    // stretch out
  delay(400);
  uarm.setPosition(60, 0, 35, 0);  // down
  delay(400);
  uarm.gripperCatch();                // catch
  delay(400);
  uarm.setPosition(60, 80, 35, 0);    // up
  delay(400);
  uarm.setPosition(60, 80, 0, 0);     // rotate
  delay(400);
  uarm.setPosition(60, 0, 0, 0);   // down
  delay(400);
  uarm.gripperRelease();              // release
  delay(100);
  uarm.setPosition(60, 80, 0, 0);     // up
  delay(400);
  uarm.setPosition(0, 80, 0, 0);       // original position
  delay(400);
  uarm.gripperDirectDetach();         // direct detach 
  delay(500);
}
