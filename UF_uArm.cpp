/******************************************************************************
* File Name          : UF_uArm.cpp
* Author             : Evan
* Updated            : Evan & Scott Gray
* Version            : V0.1 (BATE)
* Date               : 2 May, 2014
* Modified Date      : 29 May 2014
* Description        :
* License            : 
* Copyright(C) 2014 UFactory Team. All right reserved.
*******************************************************************************/

#include "UF_uArm.h"


UF_uArm::UF_uArm()
{
	heightLst  = 0;
	height     = 0;
	stretch    = 0;
	rotation   = 0;
	handRot    = 0;
    lstTime    = 0;
	delay_loop = 0;
    gripperRst  = true;

	Kp = 0;    
	Kd = 0;
	Ki = 0;      
	Ko = 1; 

	moving = 0; // is the arm in motion?
}

void UF_uArm::init()
{
    // read offset data
    offsetL = EEPROM.read(1);
    offsetR = EEPROM.read(2);
    // initialization the pin
    pinMode(LIMIT_SW, INPUT);  digitalWrite(LIMIT_SW, HIGH);
    pinMode(BTN_D4,   INPUT);  digitalWrite(BTN_D4,   HIGH);
    pinMode(BTN_D7,   INPUT);  digitalWrite(BTN_D7,   HIGH);
    pinMode(BUZZER,   OUTPUT); digitalWrite(BUZZER,   LOW);
    pinMode(PUMP_EN,  OUTPUT); digitalWrite(PUMP_EN,  LOW);
    pinMode(VALVE_EN, OUTPUT); digitalWrite(VALVE_EN, LOW);
    if (EEPROM.read(0) == CALIBRATION_FLAG) // read of offset flag
    {
	// attaches the servo on pin to the servo object
	servoL.attach(SERVO_L, D150A_SERVO_MIN_PUL, D150A_SERVO_MAX_PUL);
	servoR.attach(SERVO_R, D150A_SERVO_MIN_PUL, D150A_SERVO_MAX_PUL);
	servoRot.attach(SERVO_ROT, D150A_SERVO_MIN_PUL, D150A_SERVO_MAX_PUL);
	servoHand.attach(SERVO_HAND, D009A_SERVO_MIN_PUL, D009A_SERVO_MAX_PUL);
	servoHandRot.attach(SERVO_HAND_ROT, D009A_SERVO_MIN_PUL, D009A_SERVO_MAX_PUL);
	servoHand.write(HAND_ANGLE_OPEN);delay(50);
	servoHand.detach();
	// initialization postion
	setPosition(stretch, height, rotation, handRot);
    } else {	// buzzer alert if calibration needed
	alert(3, 200, 200);
    }
}

void UF_uArm::calibration()
{

    int initPosL = INIT_POS_L + 20; // Added 20 degrees here to start at reasonable point
    int initPosR = INIT_POS_R + 20; // Added 20 degrees here to start at reasonable point

    if(!digitalRead(BTN_D7)){
        delay(20);
        // buzzer alert
        alert(1, 20, 0);
    }

    lstTime = millis();
    while(!digitalRead(BTN_D7))
    {
        if(millis() - lstTime > BTN_TIMEOUT_MS)
        {
            // buzzer alert
            alert(2, 50, 100);
            while(!digitalRead(BTN_D7))
            {
                servoL.detach();
                servoR.detach();
            }
			while(1)
			{
				// SG-> Commentary: While user adjusts for minimum angles, keep track of angle and add
				//                  margin of analog reading value of 12, which is about 3 degrees.
				int minAngle_L = readAngle(SERVO_L) + 12;
				int minAngle_R = readAngle(SERVO_R) + 12;

				if(!digitalRead(BTN_D7))
				{
					delay(20);
					// buzzer alert
					alert(1, 20, 0);
					delay(500); //SG-> Added to delay for user to remove hand
					// buzzer alert
					servoL.attach(SERVO_L, D150A_SERVO_MIN_PUL, D150A_SERVO_MAX_PUL);
					servoR.attach(SERVO_R, D150A_SERVO_MIN_PUL, D150A_SERVO_MAX_PUL);
					servoL.write(initPosL);
					servoR.write(initPosR);
					delay(500);
					while(readAngle(SERVO_R) < minAngle_R - SAMPLING_DEADZONE)
					{
						servoR.write(++initPosR);
						delay(50);
					}
					while(readAngle(SERVO_R) > minAngle_R + SAMPLING_DEADZONE)
					{
						servoR.write(--initPosR);
						delay(50);
					}
					while(readAngle(SERVO_L) < minAngle_L - SAMPLING_DEADZONE)
					{
						servoL.write(++initPosL);
						delay(50);
					}
					while(readAngle(SERVO_L) > minAngle_L + SAMPLING_DEADZONE)
					{
						servoL.write(--initPosL);
						delay(50);
					}
					offsetL = initPosL - INIT_POS_L;
					offsetR = initPosR - INIT_POS_R;
					EEPROM.write(0, CALIBRATION_FLAG);  // Added flag to know if offset is done
					EEPROM.write(1, offsetL);			// offsetL
					EEPROM.write(2, offsetR);			// offsetR
					// buzzer alert
					alert(1, 500, 0);      
					// reset postion
					init();
					break;
				}
			}
        }
    }

}

void UF_uArm::manual_calibration(long int initPosL, long int initPosR)
{
        offsetL = initPosL - INIT_POS_L;
        offsetR = initPosR - INIT_POS_R;
            EEPROM.write(0, CALIBRATION_FLAG);  // Added flag to know if offset is done
            EEPROM.write(1, offsetL);			// offsetL
            EEPROM.write(2, offsetR);			// offsetR
            // buzzer alert
            alert(1, 500, 0);
            // reset postion
            init();
}

void UF_uArm::setPosition(double _stretch, double _height, int _armRot, int _handRot)
{

	_armRot = -_armRot;
#ifdef DEBUG
  Serial.println("Start");
#endif
#ifndef NO_LIMIT_SWITCH
    if(!digitalRead(LIMIT_SW) && _height < heightLst) //limit switch protection
    _height = heightLst;
#endif
	// input limit
	_stretch = constrain(_stretch, ARM_STRETCH_MIN,   ARM_STRETCH_MAX) + 55;		// +55, set stretch zero
	_height  = constrain(_height,  ARM_HEIGHT_MIN,    ARM_HEIGHT_MAX);
	_armRot  = constrain(_armRot,  ARM_ROTATION_MIN,  ARM_ROTATION_MAX) + 90;		// +90, change -90~90 to 0~180
	_handRot = constrain(_handRot, HAND_ROTATION_MIN, HAND_ROTATION_MAX) + 90;	    // +90, change -90~90 to 0~180

	height     = _height;
	stretch    = _stretch;
	rotation   = _armRot;
	handRot    = _handRot;
#ifdef DEBUG
  Serial.print("Target pos: height ");
  Serial.print(height);
  Serial.print(" stretch ");
  Serial.print(stretch);
  Serial.print(" rotation ");
  Serial.print(rotation);
  Serial.print(" handRot ");
  Serial.println(handRot);
#endif

	// angle calculation
	double stretch2height2 = _stretch * _stretch + _height * _height;              //
	double angleA = (acos( (ARM_A2B2 - stretch2height2) / ARM_2AB )) * RAD_TO_DEG; // angle between the upper and the lower
	double angleB = (atan(_height/_stretch)) * RAD_TO_DEG;                         // 
	double angleC = (acos((ARM_A2 + stretch2height2 -ARM_B2)/(2 * ARM_A * sqrt(stretch2height2)))) * RAD_TO_DEG; // 
	int angleR = 180 - angleA - angleB - angleC + FIXED_OFFSET_R + offsetR;        // 
	int angleL = angleB + angleC + FIXED_OFFSET_L + offsetL;                       // 
	// angle limit
	angleL = constrain(angleL, 10 + offsetL, 145 + offsetL);
	angleR = constrain(angleR, 25 + offsetR, 150 + offsetR);
	angleR = constrain(angleR, angleL - 90 + offsetR, angleR);	// behind  -120+30 = -90
	if(angleL<15+offsetL)
	angleR = constrain(angleR, 70 + offsetR, angleR);			// front down
	// set servo position
#ifdef DEBUG
  Serial.print("Target serv: angleR ");
  Serial.print(angleR);
  Serial.print(" angleL ");
  Serial.print(angleL);
  Serial.print(" _armRot ");
  Serial.print(_armRot);
  Serial.print(" _handRot ");
  Serial.println(_handRot);
#endif
	PID[0].targetPosition     = map(angleR, 0, 180, D150A_SERVO_MIN_PUL,  D150A_SERVO_MAX_PUL); 
	PID[1].targetPosition    = map(angleL, 0, 180, D150A_SERVO_MIN_PUL,  D150A_SERVO_MAX_PUL); 
	PID[2].targetPosition   = map(_armRot, 0, 180, D150A_SERVO_MIN_PUL,  D150A_SERVO_MAX_PUL);
	PID[3].targetPosition    = map(_handRot, 0, 180, D009A_SERVO_MIN_PUL,  D009A_SERVO_MAX_PUL);

#ifdef DEBUG
  Serial.print("Target ms: angleR ");
  Serial.print(PID[0].targetPosition);
  Serial.print(" angleL ");
  Serial.print(PID[1].targetPosition);
  Serial.print(" _armRot ");
  Serial.print(PID[2].targetPosition);
  Serial.print(" _handRot ");
  Serial.println(PID[3].targetPosition);
#endif

//	servoR.write(angleR);
//	servoL.write(angleL);
//	servoRot.write(_armRot);
//	servoHandRot.write(_handRot);
	heightLst = _height;
}


int UF_uArm::getPosition(int _positionNum){
	switch(_positionNum)
	{
		case 0:
			return stretch-55;
			break;
		case 1:
			return height;
			break;
		case 2:
			return 90-rotation;
			break;
		case 3:
			return handRot-90;
			break;
		case 4:
			return gripperRst ? CATCH:RELEASE;
			break;
		default: return 0; 
			break;
	}


return 0;
}    // 

int UF_uArm::getPositionMicroseconds(int _positionNum){
  int positionMS;
	switch(_positionNum)
	{
		case 0:
			positionMS = servoR.readMicroseconds();
			break;
		case 1:
			positionMS = servoL.readMicroseconds();
			break;
		case 2:
			positionMS = servoRot.readMicroseconds();
			break;
		case 3:
			positionMS = servoHandRot.readMicroseconds();
			break;
		case 4:
			positionMS = servoHand.readMicroseconds();
			break;
		default: return 0; 
			break;
	}

	return positionMS;
}    // 

int UF_uArm::readAngle(char _servoNum)
{
	int portAd;
	switch(_servoNum)
	{
		case SERVO_L:
			portAd = A0;
			break;
		case SERVO_R:
			portAd = A1;
			break;
		case SERVO_ROT:
			portAd = A2;
			break;
		case SERVO_HAND_ROT:
			portAd = A3;
			break;
		case SERVO_HAND:
			portAd = A4;
			break;
		default: return 0; break;
	}
	int adAdd = 0;
	for(char i=0; i<5; i++)
	{
		adAdd += analogRead(portAd);
	}
	return adAdd/5;
}

void UF_uArm::gripperCatch()
{
    servoHand.attach(SERVO_HAND, D009A_SERVO_MIN_PUL, D009A_SERVO_MAX_PUL);
    servoHand.write(HAND_ANGLE_CLOSE);
    digitalWrite(VALVE_EN, LOW); // valve disnable
    digitalWrite(PUMP_EN, HIGH); // pump enable
    gripperRst = true;
}

void UF_uArm::gripperRelease()
{
    if(gripperRst)
    {
      servoHand.attach(SERVO_HAND, D009A_SERVO_MIN_PUL, D009A_SERVO_MAX_PUL);
      servoHand.write(HAND_ANGLE_OPEN);
      digitalWrite(VALVE_EN, HIGH); // valve enable, decompression
      digitalWrite(PUMP_EN, LOW);   // pump disnable
      gripperRst = false;
      delay_loop = 0;
    }
}

void UF_uArm::gripperDetach()
{
    if(++delay_loop > 300000)        // delay release valve
    {
        servoHand.detach();
        digitalWrite(VALVE_EN, LOW); // valve disnable
        delay_loop=0;
    }
}

void UF_uArm::gripperDirectDetach()
{
    servoHand.detach();
    digitalWrite(VALVE_EN, LOW); // valve disnable
}

void UF_uArm::pumpOn()
{
    digitalWrite(PUMP_EN, HIGH);    // pump enable
}

void UF_uArm::pumpOff()
{
    digitalWrite(PUMP_EN, LOW);     // pump disnable
}

void UF_uArm::valveOn()
{
    digitalWrite(VALVE_EN, HIGH);   // valve enable, decompression
}

void UF_uArm::valveOff()
{
    digitalWrite(VALVE_EN, LOW);    // valve disnable
}

void UF_uArm::detachServo(char _servoNum)
{
    switch(_servoNum)
    {
        case SERVO_L:
        servoL.detach();
        break;
        case SERVO_R:
        servoR.detach();
        break;
        case SERVO_ROT:
        servoRot.detach();
        break;
        case SERVO_HAND_ROT:
        servoHandRot.detach();
        break;
        case SERVO_HAND:
        servoHand.detach();
        break;
        default: break;
    }
}

void UF_uArm::sendData(byte _dataAdd, int _dataIn)
{
	Serial.write(0xFF); Serial.write(0xAA); // send data head
	Serial.write(_dataAdd);
	Serial.write(*((char *)(&_dataIn) + 1));
	Serial.write(*((char *)(&_dataIn)));
}

void UF_uArm::alert(int _times, int _runTime, int _stopTime)
{
	for(int _ct=0; _ct < _times; _ct++)
	{
#ifdef PIEZOBUZZER
        delay(_stopTime);
        analogWrite(BUZZER, 20);      // Almost any value can be used except 0 and 255
        delay(_runTime);
        analogWrite(BUZZER, 0);       // 0 turns it off
#else
        delay(_stopTime);
        digitalWrite(BUZZER, HIGH);
        delay(_runTime);
        digitalWrite(BUZZER, LOW);
#endif
	}
}

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both encoder and prevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void UF_uArm::resetPID(int pidnum){
     PID[pidnum].targetTicksPerFrame = 0;
     PID[pidnum].encoder = getPositionMicroseconds(pidnum);
     PID[pidnum].targetPosition = PID[pidnum].prevEnc = PID[pidnum].encoder;
     PID[pidnum].output = 0;
     PID[pidnum].prevInput = 0;
     PID[pidnum].iTerm = 0;
}


/* PID routine to compute the next motor commands */
void UF_uArm::doPID(SetPointInfo * p) {
  int error;
  int output;
  int input;

  input = p->encoder - p->prevEnc;
  error = p->targetTicksPerFrame - input;
  
  p->iTerm += (Ki * error) / Ko;
  if (p->iTerm > (MAX_DELTA-MIN_DELTA)) p->iTerm = MAX_DELTA;
  else if (p->iTerm < (-MAX_DELTA+MIN_DELTA)) p->iTerm = -MAX_DELTA;

  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  output = (((long)Kp) * error - Kd * (input - p->prevInput))/ Ko + p->iTerm;
  p->prevEnc = p->encoder;

  /*
  * Accumulate Integral error *or* Limit output.
  * 
  * Avoid motor moving back when requesting forward movement, and vice versa (avoid oscillating around 0)
  * Also avoid sending output of 0 (stopping motors)
  *
  * Stop accumulating integral error when output is limited.
  */
  if (p->targetTicksPerFrame > 0){
    output += MIN_DELTA;
    if (output < MIN_DELTA) output = MIN_DELTA;
  } else if (p->targetTicksPerFrame < 0){
    output += -MIN_DELTA;
    if (output > -MIN_DELTA) output = -MIN_DELTA;
  } 
  
  if (output > MAX_DELTA)
    output = MAX_DELTA;
  else if (output < -MAX_DELTA)
    output = -MAX_DELTA;

  p->output = output;
  p->prevInput = input;
}


/* Read the encoder values and call the PID routine */
void UF_uArm::updatePID() {
  /* Read the encoders */
  for(int i=0;i<PIDS_NUM;i++) PID[i].encoder = getPositionMicroseconds(i);

  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * Most importantly, keep Encoder and PrevEnc synced; use that as criteria whether we need reset
    */
    for(int i=0;i<PIDS_NUM;i++) if (PID[i].prevEnc != PID[i].encoder) resetPID(i);
    //return;
  }
  
  /* Compute PID update for each motor */
  moving = 0;
  for(int i=0;i<PIDS_NUM;i++){
      if(abs(PID[i].targetPosition - PID[i].encoder) > MIN_DELTA ) {
        PID[i].targetTicksPerFrame = (PID[i].targetPosition - PID[i].encoder)/2;
        moving = 1;
        doPID(&(PID[i]));
      } else {
        PID[i].prevEnc = PID[i].encoder;
      }
   }
  /* Set the motor position accordingly */

//   setPosition(PID[0].encoder+PID[0].output,
//                    PID[1].encoder+PID[1].output,
//                    PID[2].encoder+PID[2].output,
//                    PID[3].encoder+PID[3].output);

  servoR.writeMicroseconds(PID[0].encoder+PID[0].output);
  servoL.writeMicroseconds(PID[1].encoder+PID[1].output);
  servoRot.writeMicroseconds(PID[2].encoder+PID[2].output);
  servoHandRot.writeMicroseconds(PID[3].encoder+PID[3].output);


#ifdef DEBUG
  if (moving){

    Serial.print("PID encoders ");
    for(int i=0;i<PIDS_NUM;i++) {
      Serial.print(PID[i].encoder); 
      Serial.print(" ");
    }
       Serial.print("PID targets  ");
    for(int i=0;i<PIDS_NUM;i++) {
      Serial.print(PID[i].targetPosition); 
      Serial.print(" ");
    }

      Serial.print("PID next pos ");
    Serial.print(PID[0].encoder+PID[0].output);
    Serial.print(":");
    Serial.print(PID[1].encoder+PID[1].output);
    Serial.print(":");
    Serial.print(PID[2].encoder+PID[2].output);
    Serial.print(":");
    Serial.println(PID[3].encoder+PID[3].output);


    delay(100);
  }
#endif  
}

/* Set PID parameters */
//Assuming pid_rate and Kx parameters all given in s
//Doing some effort to keep things in integer math, through use of Ko
void UF_uArm::setPIDParams(int newKp, int newKd, int newKi, int newKo, int pidRate){
    Kp = newKp * pidRate;
    Ki = newKi;
    Kd = newKd * pidRate * pidRate;  
    Ko = newKo * pidRate;
}
boolean  UF_uArm::isMoving(){return moving;};
