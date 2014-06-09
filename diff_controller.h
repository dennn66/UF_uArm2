/*
* Functions and type-defs for PID control.
*
* Based on the Beginner PID's series by Brett Beauregard - http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
* Adapted to use ideal velocity form or position form.
*
* Originally adapted from Mike Ferguson's ArbotiX code which lives at:
*
* http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/
#define POSITION_PID
#ifdef POSITION_PID
  /* PID setpoint info For a Motor */
  typedef struct {
    int targetPosition; // estimated position
    int targetTicksPerFrame; // target speed in ticks per frame
    long encoder; // current position
    long prevEnc; // last position
  
    /*
* Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
* see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
*/
    int prevInput; // last input
    //int prevErr; // last error
  
    /*
* Using integrated term (ITerm) instead of integrated error (Ierror),
* to allow tuning changes,
* see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
*/
    long iTerm; //integrated term
    int output; // last motor setting
  }
  SetPointInfo;
#endif


