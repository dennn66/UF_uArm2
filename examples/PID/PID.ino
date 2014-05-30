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


#define POSITION_PID
//#define VELOCITY_PID
int readEncoder(int servo);
#define MAX_PWM 5
#define MIN_PWM 0

/* PID parameters and functions */
#include "diff_controller.h"

/* Estimated gripper sate */
int targetGripperState = CATCH;

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

#define NO_LIMIT_SWITCH
#define PIEZOBUZZER

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

int test = 0;


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
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
   case ARM_TEST:   
    Serial.println("OK");
    while(digitalRead(BTN_D7)){
      motion();
      motionReturn();
    } 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    setPIDParams(pid_args[0], pid_args[1], pid_args[2], pid_args[3], PID_RATE);
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
       PID[i].targetPosition = atoi(str);
       i++;
    }
    uarm.setPosition(PID[0].targetPosition,
                    PID[1].targetPosition,
                    PID[2].targetPosition,
                    PID[3].targetPosition);
    Serial.println("OK");
    break;
  case ARM_HOLD:
    targetGripperState = arg1;

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
  setPIDParams(INIT_KP, INIT_KD, INIT_KI, INIT_KO, PID_RATE);
  for(int i=0;i<PIDS_NUM;i++) resetPID(i);

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
    updatePID();
  }
  
  if(test&!moving){
    motion();
    motionReturn();
  } 
  if(!moving){
    /* pump action, Valve Stop. */
    if(targetGripperState & CATCH)   uarm.gripperCatch();
    /* pump stop, Valve action.
       Note: The air relief valve can not work for a long time,
       should be less than ten minutes. */
    if(targetGripperState & RELEASE) uarm.gripperRelease();
  } 
  /* delay release valve, this function must be in the main loop */
  uarm.gripperDetach();  
} 


void motion()
{
  if(test)uarm.setPosition(60, 80, 0, 0);    // stretch out
  if(test)delay(400);
  if(test)uarm.setPosition(60, 0, 0, 0);  // down
  if(test)delay(400);
  if(test)uarm.gripperCatch();               // catch
  if(test)delay(400);
  if(test)uarm.setPosition(60, 80, 0, 0);    // up
  if(test)delay(400);
  if(test)uarm.setPosition(60, 80, 35, 0);   // rotate
  if(test)delay(400);
  if(test)uarm.setPosition(60, 0, 35, 0); // down
  if(test)delay(400);
  if(test)uarm.gripperRelease();             // release
  if(test)delay(100);
  if(test)uarm.setPosition(60, 80, 35, 0);   // up
  if(test)delay(400);
  if(test)uarm.setPosition(0, 80, 0, 0);      // original position
  if(test)delay(400);
  if(test)uarm.gripperDirectDetach();        // direct detach 
  if(test)delay(500);
}

void motionReturn()
{
  if(test)uarm.setPosition(60, 80, 35, 0);    // stretch out
  if(test)delay(400);
  if(test)uarm.setPosition(60, 0, 35, 0);  // down
  if(test)delay(400);
  if(test)uarm.gripperCatch();                // catch
  if(test)delay(400);
  if(test)uarm.setPosition(60, 80, 35, 0);    // up
  if(test)delay(400);
  if(test)uarm.setPosition(60, 80, 0, 0);     // rotate
  if(test)delay(400);
  if(test)uarm.setPosition(60, 0, 0, 0);   // down
  if(test)delay(400);
  if(test)uarm.gripperRelease();              // release
  if(test)delay(100);
  if(test)uarm.setPosition(60, 80, 0, 0);     // up
  if(test)delay(400);
  if(test)uarm.setPosition(0, 80, 0, 0);       // original position
  if(test)delay(400);
  if(test)uarm.gripperDirectDetach();         // direct detach 
  if(test)delay(500);
}
int readEncoder(int _positionNum){
  return uarm.getPosition(_positionNum);
}
