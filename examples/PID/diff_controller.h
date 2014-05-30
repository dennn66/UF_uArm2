/* 
*  Functions and type-defs for PID control.
*
*  Based on the Beginner PID's series by Brett Beauregard - http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
*  Adapted to use ideal velocity form or position form.
*
*  Originally adapted from Mike Ferguson's ArbotiX code which lives at:
*   
*  http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/
#define POSITION_PID
#ifdef POSITION_PID
  /* PID setpoint info For a Motor */
  typedef struct {
    int targetPosition;         // estimated position
    int targetTicksPerFrame;    // target speed in ticks per frame
    long encoder;                  // current position
    long prevEnc;                  // last position
  
    /*
    * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
    */
    int prevInput;                // last input
    //int prevErr;                   // last error
  
    /*
    * Using integrated term (ITerm) instead of integrated error (Ierror),
    * to allow tuning changes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
    long iTerm;                    //integrated term
    int output;                    // last motor setting
  }
  SetPointInfo;
#endif

//SetPointInfo leftPID, rightPID;
#define PIDS_NUM 4
SetPointInfo      PID[PIDS_NUM];

/* PID Parameters 
* Do not SET these directly here, unless you know what you are doing 
* Use setPIDParameters() instead
*/
int Kp = 0;    
int Kd = 0;
int Ki = 0;      
int Ko = 1; 

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both encoder and prevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(int pidnum){
#ifdef  POSITION_PID
     PID[pidnum].targetTicksPerFrame = 0;
     PID[pidnum].encoder = readEncoder(0);
     PID[pidnum].targetPosition = PID[pidnum].prevEnc = PID[pidnum].encoder;
     PID[pidnum].output = 0;
     PID[pidnum].prevInput = 0;
     PID[pidnum].iTerm = 0;
#endif
}

#ifdef  POSITION_PID
/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  int error;
  int output;
  int input;

  input = p->encoder - p->prevEnc;
  error = p->targetTicksPerFrame - input;
  
  p->iTerm += (Ki * error) / Ko;
  if (p->iTerm > (MAX_PWM-MIN_PWM)) p->iTerm = MAX_PWM;
  else if (p->iTerm < (-MAX_PWM+MIN_PWM)) p->iTerm = -MAX_PWM;

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
    output += MIN_PWM;
    if (output < MIN_PWM) output = MIN_PWM;
  } else if (p->targetTicksPerFrame < 0){
    output += -MIN_PWM;
    if (output > -MIN_PWM) output = -MIN_PWM;
  } 
  
  if (output > MAX_PWM)
    output = MAX_PWM;
  else if (output < -MAX_PWM)
    output = -MAX_PWM;

  p->output = output;
  p->prevInput = input;
}
#endif

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  for(int i=0;i<PIDS_NUM;i++) PID[i].encoder = readEncoder(i);
    
  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * Most importantly, keep Encoder and PrevEnc synced; use that as criteria whether we need reset
    */
    for(int i=0;i<PIDS_NUM;i++) if (PID[i].prevEnc != PID[i].encoder) resetPID(i);
    return;
  }
  
  /* Compute PID update for each motor */
  moving = 0;
  for(int i=0;i<PIDS_NUM;i++){
      if(PID[i].targetPosition != PID[i].encoder ) {
        PID[i].targetTicksPerFrame = PID[i].targetPosition - PID[i].encoder;
        moving = 1;
        doPID(&(PID[i]));
      } else {
        PID[i].prevEnc = PID[i].encoder;
      }
   }
  /* Set the motor position accordingly */

   uarm.setPosition(PID[0].encoder+PID[0].output,
                    PID[1].encoder+PID[1].output,
                    PID[2].encoder+PID[2].output,
                    PID[3].encoder+PID[3].output);
}

/* Set PID parameters */
//Assuming pid_rate and Kx parameters all given in s
//Doing some effort to keep things in integer math, through use of Ko
void setPIDParams(int newKp, int newKd, int newKi, int newKo, int pidRate){
    Kp = newKp * pidRate;
    Ki = newKi;
    Kd = newKd * pidRate * pidRate;  
    Ko = newKo * pidRate;
}




