/**
 * Autonomous
 * @author Curt Henrichs
 * @date 8-8-20
 * 
 * Autonomous module defines the state-machine to control Rovebot. The
 * state-machine architecture is built on the switch-case / subroutine
 * 'dispatcher' model. Everything is dependent on timer events running in the
 * main loop so it should be called every main loop tick.
 */
 
//==============================================================================
//                               Libraries
//==============================================================================

#include "Autonomous.h"
#include <elapsedMillis.h>

//==============================================================================
//                       Constant and Macro Definitions
//==============================================================================

/**
 * Minimum distance before switching from forward to turning
 */
#define LEFT_STOPPING_DISTANCE           0.30f
#define CENTER_STOPPING_DISTANCE         0.30f
#define SHARP_STOPPING_STOPPING_DISTANCE 0.20f
#define RIGHT_STOPPING_DISTANCE          0.30f
#define MIN_TURN_RADIUS                  0.13f

/**
 * Timing constants for physical robot movement
 */
#define FORWARD_TIME              200
#define BACKUP_TIME               600
#define MOVING_LEFT_TIME          600
#define MOVING_RIGHT_TIME         600

/**
 * Motor power scalar
 */

#define LEFT_MOTOR_POWER          35
#define RIGHT_MOTOR_POWER         30

/**
 * AI state maintainer used to handle robot stuck cases
 */
typedef struct CornerAI {
  int leftCount = 0;            //! Counts number of left turns
  int rightCount = 0;           //! Counts number of right turns
} CornerAI_t;

/**
 * Maxium turn count before considered stuck in corner
 */
#define MAX_LEFT_COUNT            5
#define MAX_RIGHT_COUNT           5

//==============================================================================
//                           Private Attributes
//==============================================================================

// state of the autonomous state-machine
static AutonState_e _state;

// timing Variables
static elapsedMillis _delayTimer;

// Stuck AI structure to maintain extra-state-machine state
static CornerAI_t ai;

// motor states
static int _leftMotor;
static int _rightMotor;

//==============================================================================
//                       Private Function Prototypes
//==============================================================================

/**
 * State MOVE_FWD function move forward until min distance is met.
 * then switches to turning head left
 * @param ul is left ultrasonic sensor reading
 * @param uc is center ultrasonic sensor reading
 * @param ur is right ultrasonic sensor reading
 * @param sl is left sharp ir sensor reading
 * @param sr is right sharp ir sensor reading
 * @return next state (MOVE_FWD or WHICH_WAY_SLCT)
 */
static inline AutonState_e _forward(float ul, float uc, float ur, float sl, float sr);

/**
 * State MOVE_BWD function moves Bubbles backward for a predefined amount of
 * time. Switches to turning head left when time elapsed.
 * @return next state (MOVE_BWD or WHICH_WAY_SLCT)
 */
static inline AutonState_e _backward();

/**
 * State MOVE_LEFT function moves Bubbles left for a predefined amount of
 * time. Switches to moving forward when time elapsed.
 * @param ul is left ultrasonic sensor reading
 * @param uc is center ultrasonic sensor reading
 * @param ur is right ultrasonic sensor reading
 * @param sl is left sharp ir sensor reading
 * @param sr is right sharp ir sensor reading
 * @return next state (MOVE_RIGHT, MOVE_FWD, or WHICH_WAY_SLCT)
 */
static inline AutonState_e _left(float ul, float uc, float ur, float sl, float sr);

/**
 * State MOVE_RIGHT function moves Bubbles right for a predefined amount of
 * time. Switches to moving forward when time elapsed.
 * @param ul is left ultrasonic sensor reading
 * @param uc is center ultrasonic sensor reading
 * @param ur is right ultrasonic sensor reading
 * @param sl is left sharp ir sensor reading
 * @param sr is right sharp ir sensor reading
 * @return next state (MOVE_RIGHT, MOVE_FWD, or WHICH_WAY_SLCT)
 */
static inline AutonState_e _right(float ul, float uc, float ur, float sl, float sr);

/**
 * State WHICH_WAY_SLCT function to handle the selection of which direction to
 * move Bubbles for the next several states. If the center distance is less than
 * the minimum turning radius then move backward. If left or right distance is
 * less than the minimum turning radius then move backward. Else turn left or
 * right dependign on which is greater.
 * @param ul is left ultrasonic sensor reading
 * @param uc is center ultrasonic sensor reading
 * @param ur is right ultrasonic sensor reading
 * @param sl is left sharp ir sensor reading
 * @param sr is right sharp ir sensor reading
 * @return next state (MOVE_LEFT, MOVE_RIGHT, MOVE_BWD)
 */
static inline AutonState_e _whichWay(float ul, float uc, float ur, float sl, float sr);

//==============================================================================
//                      Public Function Implementation
//==============================================================================

/**
 * Resets autonomous state-machine. Call once after a need to reset the state
 * machine such as moving from manual operation to autonomous.
 */
void auton_begin(void) {
  _state = MOVE_FWD;
  ai.leftCount = 0;
  ai.rightCount = 0;

  _leftMotor = 0;
  _rightMotor = 0;
}

/**
 * Main loop function that will handle the current auton state. This functions
 * could take a large finite amount of time to run due to the nature of the
 * robot.
 * @param ul is left ultrasonic sensor reading
 * @param uc is center ultrasonic sensor reading
 * @param ur is right ultrasonic sensor reading
 * @param sl is left sharp ir sensor reading
 * @param sr is right sharp ir sensor reading
 */
void auton_update(float ul, float uc, float ur, float sl, float sr) {
  // Run current state and update for next update call
  switch(_state){
    case MOVE_FWD:
      _state = _forward(ul, uc, ur, sl, sr);
      break;
    case MOVE_BWD:
      _state = _backward();
      break;
    case MOVE_LEFT:
      _state = _left(ul, uc, ur, sl, sr);
      break;
    case MOVE_RIGHT:
      _state = _right(ul, uc, ur, sl, sr);
      break;
    case WHICH_WAY_SLCT:
      _state = _whichWay(ul, uc, ur, sl, sr);
      break;
    case ERROR:
    default:
      auton_begin(); //something broke, reset state-machine
      break;
  }
}

/**
 * @return Current state control value for left motor
 */
int auton_left_motor(void) {
  return _leftMotor;
}

/**
 * @return Current state control value for right motor
 */
int auton_right_motor(void) {
  return _rightMotor;
}

/**
 * @return current state of the autonomous state-machine
 */
AutonState_e auton_get_state(void) {
  return _state;
}

//==============================================================================
//                       Private Function Implementation
//==============================================================================

/**
 * State MOVE_FWD function move forward until min distance is met.
 * then switches to turning head left
 * @param ul is left ultrasonic sensor reading
 * @param uc is center ultrasonic sensor reading
 * @param ur is right ultrasonic sensor reading
 * @param sl is left sharp ir sensor reading
 * @param sr is right sharp ir sensor reading
 * @return next state (MOVE_FWD or WHICH_WAY_SLCT)
 */
static inline AutonState_e _forward(float ul, float uc, float ur, float sl, float sr){
  bool moveForward = true;
  AutonState_e retVal;
  
  if(_delayTimer >= FORWARD_TIME) {
    _delayTimer = 0;
    
    if(uc < CENTER_STOPPING_DISTANCE || ul < LEFT_STOPPING_DISTANCE || ur < RIGHT_STOPPING_DISTANCE || sl < SHARP_STOPPING_STOPPING_DISTANCE || sr < SHARP_STOPPING_STOPPING_DISTANCE){
      moveForward = false;

      //stop the robot going to crash
      _leftMotor = 0;
      _rightMotor = 0;

      //start scan for new direction
      retVal = WHICH_WAY_SLCT;
    }
  }
  
  if(moveForward) {
      // move forward as nothing is in the way
      _leftMotor = LEFT_MOTOR_POWER * 1;
      _rightMotor = RIGHT_MOTOR_POWER * 1;
      
      retVal = MOVE_FWD;
  }

  return retVal;
}

/**
 * State MOVE_BWD function moves Bubbles backward for a predefined amount of
 * time. Switches to turning head left when time elapsed.
 * @return next state (MOVE_BWD or WHICH_WAY_SLCT)
 */
static inline AutonState_e _backward(){
  AutonState_e retVal;

  if(_delayTimer >= BACKUP_TIME){
    _delayTimer = 0;
    
    //done backing up
    _leftMotor = 0;
    _rightMotor = 0;

    retVal = WHICH_WAY_SLCT;
    
  }else{
    // continue backing up
    _leftMotor = -1 * LEFT_MOTOR_POWER * 1;
    _rightMotor = -1 * RIGHT_MOTOR_POWER * 1;
    retVal = MOVE_BWD;
  }
  return retVal;
}

/**
 * State MOVE_LEFT function moves Bubbles left for a predefined amount of
 * time. Switches to moving forward when time elapsed.
 * @param ul is left ultrasonic sensor reading
 * @param uc is center ultrasonic sensor reading
 * @param ur is right ultrasonic sensor reading
 * @param sl is left sharp ir sensor reading
 * @param sr is right sharp ir sensor reading
 * @return next state (MOVE_RIGHT, MOVE_FWD, or WHICH_WAY_SLCT)
 */
static inline AutonState_e _left(float ul, float uc, float ur, float sl, float sr){
  AutonState_e retVal;
  
  if(_delayTimer >= MOVING_LEFT_TIME){
    _delayTimer = 0;

    if(uc < CENTER_STOPPING_DISTANCE || ul < LEFT_STOPPING_DISTANCE || ur < RIGHT_STOPPING_DISTANCE || sl < SHARP_STOPPING_STOPPING_DISTANCE || sr < SHARP_STOPPING_STOPPING_DISTANCE){
      
      //stop the robot going to crash
      _leftMotor = 0;
      _rightMotor = 0;

      //start scan for new direction
      retVal = WHICH_WAY_SLCT;
    } else {
      //done turning right, go forward
      retVal = MOVE_FWD;
    }
    
  }else{
    
    //continue turning left
    _leftMotor = -1 * LEFT_MOTOR_POWER * 1;
    _rightMotor = RIGHT_MOTOR_POWER * 1;
    retVal = MOVE_LEFT;
    
  }
  return retVal;
}

/**
 * State MOVE_RIGHT function moves Bubbles right for a predefined amount of
 * time. Switches to moving forward when time elapsed.
 * @param ul is left ultrasonic sensor reading
 * @param uc is center ultrasonic sensor reading
 * @param ur is right ultrasonic sensor reading
 * @param sl is left sharp ir sensor reading
 * @param sr is right sharp ir sensor reading
 * @return next state (MOVE_RIGHT, MOVE_FWD, or WHICH_WAY_SLCT)
 */
static inline AutonState_e _right(float ul, float uc, float ur, float sl, float sr){
  AutonState_e retVal;
  
  if(_delayTimer >= MOVING_RIGHT_TIME){
    _delayTimer = 0;

    if(uc < CENTER_STOPPING_DISTANCE || ul < LEFT_STOPPING_DISTANCE || ur < RIGHT_STOPPING_DISTANCE || sl < SHARP_STOPPING_STOPPING_DISTANCE || sr < SHARP_STOPPING_STOPPING_DISTANCE){
      
      //stop the robot going to crash
      _leftMotor = 0;
      _rightMotor = 0;

      //start scan for new direction
      retVal = WHICH_WAY_SLCT;
    } else {
      //done turning right, go forward
      retVal = MOVE_FWD;
    }
    
  }else{
    
    //continue turning right
    _leftMotor = LEFT_MOTOR_POWER * 1;
    _rightMotor = -1 * RIGHT_MOTOR_POWER * 1;
    retVal = MOVE_RIGHT;
    
  }
  return retVal;
}

/**
 * State WHICH_WAY_SLCT function to handle the selection of which direction to
 * move Bubbles for the next several states. If the center distance is less than
 * the minimum turning radius then move backward. If left or right distance is
 * less than the minimum turning radius then move backward. Else turn left or
 * right dependign on which is greater.
 * @param ul is left ultrasonic sensor reading
 * @param uc is center ultrasonic sensor reading
 * @param ur is right ultrasonic sensor reading
 * @param sl is left sharp ir sensor reading
 * @param sr is right sharp ir sensor reading
 * @return next state (MOVE_LEFT, MOVE_RIGHT, MOVE_BWD)
 */
static inline AutonState_e _whichWay(float ul, float uc, float ur, float sl, float sr){
  AutonState_e retVal;

  if (ai.rightCount > MAX_RIGHT_COUNT || ai.leftCount > MAX_LEFT_COUNT) {
    
    ai.rightCount = 0;
    ai.leftCount = 0;
    retVal = MOVE_BWD;
    
  } else {
    
    if(uc < MIN_TURN_RADIUS || sl < MIN_TURN_RADIUS || sr < MIN_TURN_RADIUS){
      
      retVal = MOVE_BWD;
    }else if(ul < MIN_TURN_RADIUS && ur < MIN_TURN_RADIUS){
      
      retVal = MOVE_BWD;
    }else if(ul >= ur){
      
      retVal = MOVE_LEFT;
      ai.leftCount++;
    }else{
      
      ai.rightCount++;
      retVal = MOVE_RIGHT;
    }
    
  }

  _delayTimer = 0;
  return retVal;
}
