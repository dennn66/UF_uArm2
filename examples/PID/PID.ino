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
//#define BAUDRATE     9600
#define BAUDRATE     115200

#define DEBUG
//#undef DEBUG

//#define WATCHDOG
#undef WATCHDOG

#ifdef WATCHDOG
  #include <avr/wdt.h>
#endif

UF_uArm uarm;           // initialize the uArm library 


/* Initial PID Parameters */
const int INIT_KP = 1;
const int INIT_KD = 0;
const int INIT_KI = 30;
const int INIT_KO = 5;

/* Run the PID loop at 30 times per second */
#define PID_RATE           30     // Hz

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

int testSequence[][5] = 
//motion
  {{70, 80, 0, 0, RELEASE},    // stretch out
  {70, -85, 0, 0, RELEASE},  // down
  {70, -85, 0, 0, CATCH},  //catch
  {70, 80, 0, 0, CATCH},    // up
  {70, 80, 35, 0, CATCH},   // rotate
  {70, -85, 35, 0, CATCH}, // down
  {70, -85, 35, 0, RELEASE}, // release
  {70, 80, 35, 0,RELEASE},   // up
  {0, 80, 0, 0,RELEASE},      // original position
//motionReturn
  {70, 80, 35, 0, RELEASE},    // stretch out
  {70, -85, 35, 0, RELEASE},  // down
  {70, -85, 35, 0, CATCH},   // catch
  {70, 80, 35, 0,CATCH},    // up
  {70, 80, 0, 0, CATCH},     // rotate
  {70, -85, 0, 0, CATCH},   // down
  {70, -85, 0, 0, CATCH},  // release
  {70, 80, 0, 0, RELEASE},     // up
  {0, 80, 0, 0, RELEASE}}     ;  // original position;
int testState=0;  // current command from test sequence
int test = 0;    //is the test enabled

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
  int pid_args[4];
  int arm_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
   case ARM_TEST:   
    testState = 0;
    test = test==0?1:0;
    Serial.println("OK");
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    uarm.setPIDParams(pid_args[0], pid_args[1], pid_args[2], pid_args[3], PID_RATE);
    Serial.println("OK");
    break;
  case ARM_CALIBRATION:
    uarm.manual_calibration(arg1, arg2);
    Serial.println("OK");
    break;
  case ARM_ALERT:
    uarm.alert(1, 200, 0);
    Serial.println("OK");
    break;
  case ARM_SET_POSITION:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       arm_args[i] = atoi(str);
       i++;
    }
    uarm.setPosition(arm_args[0],
                    arm_args[1],
                    arm_args[2],
                    arm_args[3]);
    Serial.println("OK");
    break;
  case ARM_HOLD:
          /* pump action, Valve Stop. */
    if(arg1 & CATCH) uarm.gripperCatch();
    /* pump stop, Valve action.
       Note: The air relief valve can not work for a long time,
       should be less than ten minutes. */
    if(arg1 & RELEASE) uarm.gripperRelease();
    Serial.println("OK");
    break;
  case ARM_GET_POSITION:
    Serial.print(uarm.getPosition(0));
    Serial.print(" ");
    Serial.print(uarm.getPosition(1));
    Serial.print(" ");
    Serial.print(uarm.getPosition(2));
    Serial.print(" ");
    Serial.print(uarm.getPosition(3));
    Serial.print(" ");
    Serial.println(uarm.getPosition(4));
    break;
  case ARM_GET_ANGLE:
    Serial.print(uarm.readAngle(SERVO_ROT));
    Serial.print(" ");
    Serial.print(uarm.readAngle(SERVO_L));
    Serial.print(" ");
    Serial.print(uarm.readAngle(SERVO_R));
    Serial.print(" ");
    Serial.print(uarm.readAngle(SERVO_HAND_ROT));
    Serial.print(" ");
    Serial.println(uarm.readAngle(SERVO_HAND));
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

  //init PID
  uarm.setPIDParams(INIT_KP, INIT_KD, INIT_KI, INIT_KO, PID_RATE);
  for(int i=0;i<PIDS_NUM;i++) uarm.resetPID(i);

#ifdef WATCHDOG
  wdt_reset();
  wdt_enable(WDTO_1S);  //reset after 1s of inactivity
#endif

  uarm.init();          // initialize the uArm position

}

void loop()
{
//  uarm.calibration();   // if corrected, you could remove it, no harm though
#ifdef WATCHDOG
    //watchdog protection; if we dont reset this every X ms
    //arduino will reset
    wdt_reset();
#endif

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

  if (millis() > nextPID) {
    nextPID = millis() + PID_INTERVAL;
    uarm.updatePID();

    if(test&!uarm.isMoving()){
          uarm.setPosition(testSequence[testState][0],
                    testSequence[testState][1],
                    testSequence[testState][2],
                    testSequence[testState][3]);
                   /* pump action, Valve Stop. */
          if(testSequence[testState][4] & CATCH) {
            uarm.gripperCatch();
          } else {
          /* pump stop, Valve action.
             Note: The air relief valve can not work for a long time,
             should be less than ten minutes. */
             uarm.gripperRelease();
          }
          testState++;
          testState = testState==18?0:testState;
    } 
  }
  
  /* delay release valve, this function must be in the main loop */
  uarm.gripperDetach();  
} 


