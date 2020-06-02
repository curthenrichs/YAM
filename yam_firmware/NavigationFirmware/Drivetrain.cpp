/**
 * Drivetrain
 * @author Curt Henrichs
 * @date 1-2-19
 *
 * Drivetrain is a software module that defines the tank drive mechanism used
 * for robot locomotion. Relevant mathematical conversions from input space to
 * PWM signals is built into the module so that the drivetrain can appear to
 * act more intuitively. The module attempts to provide cartesian to
 * differential conversion so that a joystick's x,y coordinates can drive this
 * device.
 *
 * Note that a mechanism to invert one of the motors is built into the math and
 * should not be done externally.
 *
 * Adjusting the tuning parameters should be done sparingly and only to decrease
 * power to motor to bring to match least performant motor.
 */

//==============================================================================
//                               Libraries
//==============================================================================

#include "Drivetrain.h"
#include "HardwareConfig.h"
#include <math.h>

//==============================================================================
//                       Constant and Macro Definitions
//==============================================================================

/*
 * Experimentally derived PWM range for Victor SP motor controllers
 */
#define PWM_NEG_MAX -255   //! Lower limt turns in negaitve direction
#define PWM_POS_MAX 255   //! Upper limit turns in positive direction

//==============================================================================
//                           Private Attributes
//==============================================================================

/**
 * Assigned pin connected to left motor controller
 */
static MotorController* _leftMotor;
/**
 * Assigned pin connected to right motor controller
 */
static MotorController* _rightMotor;
/**
 * Drive status of the motors. True if a motor is moving else False
 */
static bool _active = false;

//==============================================================================
//                      Public Function Implementation
//==============================================================================

/**
 * Intialization function to set drivetrain as an output device and to turn off
 * until commanded.
 * @param leftMotor is left motor controller
 * @param rightMotor is right motor controller
 */
void drive_init(MotorController* leftMotor, MotorController* rightMotor){
  _leftMotor = leftMotor;
  _rightMotor = rightMotor;
  drive_hard_stop();
}

/**
 * Cartesian drivetrain takes an x and y coordinate pair to drive robot.
 * A transformation for rectangular to polor coordinates is performed meaning
 * heavy use of floating point numbers. Thus this code should not run on just
 * an Arduino.
 * @param x is x-axis value from defined min to max represents amount turning
 * @param y is y-axis value from defined min to max represents amount forward
 * @param invert (optional) inverts a motor to allow for correct hardware movement
 * @return true if had to clamp input values down to defined range else false
 */
bool drive_cartesian(float x, float y, bool invert){
  //bound clamping
  bool clamped = false;
  if(x > CARTESIAN_RANGE_MAX){
    x = CARTESIAN_RANGE_MAX;
    clamped = true;
  }else if(x < CARTESIAN_RANGE_MIN){
    x = CARTESIAN_RANGE_MIN;
    clamped = true;
  }
  if(y > CARTESIAN_RANGE_MAX){
    y = CARTESIAN_RANGE_MAX;
    clamped = true;
  }else if(y < CARTESIAN_RANGE_MIN){
    y = CARTESIAN_RANGE_MIN;
    clamped = true;
  }
  x *= -1; //invert x for math to work out

  //convert to polar cooardinates
  float radius = sqrt(pow(x,2) + pow(y,2));
  float angle = atan2(y,x);

  //rotate 45 degrees (for diamond coordinates)
  angle += PI / 4;
  float dl = radius * cos(angle);
  float dr = radius * sin(angle);

  //drive tank
  return drive_differential(dl,dr,invert) || clamped;
}

/**
 * Differential drive is left, right motor pair that uses the difference in power to
 * steer.
 * @param l is left motor value from defined min to max
 * @param r is right motor value from defined min to max
 * @param invert (optional) inverts a motor to allow for correct hardware movement
 * @return true if had to clamp input values down to defined range else false
 */
bool drive_differential(float l, float r, bool invert){
  //bound clamping
  bool clamped = false;
  if(l > DIFFERENTIAL_RANGE_MAX){
    l = DIFFERENTIAL_RANGE_MAX;
    clamped = true;
  }else if(l < DIFFERENTIAL_RANGE_MIN){
    l = DIFFERENTIAL_RANGE_MIN;
    clamped = true;
  }
  if(r > DIFFERENTIAL_RANGE_MAX){
    r = DIFFERENTIAL_RANGE_MAX;
    clamped = true;
  }else if(r < DIFFERENTIAL_RANGE_MIN){
    r = DIFFERENTIAL_RANGE_MIN;
    clamped = true;
  }

  //convert to PWM signal
  float pwm_slope = (PWM_POS_MAX - PWM_NEG_MAX)/((DIFFERENTIAL_RANGE_MAX - DIFFERENTIAL_RANGE_MIN)*1.0f);
  float pwm_inter = PWM_POS_MAX - pwm_slope * DIFFERENTIAL_RANGE_MAX;
  int l_pwm = (int)(l * pwm_slope * (invert ? -1 : 1) + pwm_inter);
  int r_pwm = (int)(r * pwm_slope + pwm_inter);

  //set PWM
  drive_set_pwm(l_pwm,r_pwm);

  return clamped;
}

/**
 * Signals a hard stop which cuts the PWM wave to zero thereby signaling the
 * motorcontrollers that the signal has ended.
 */
void drive_hard_stop(void){
  _leftMotor->stop();
  _rightMotor->stop();
}

/**
 * Sets the pwm for each motor directly without bound checking. Left up to
 * user of function for best practice.
 * @param l is pwm signal on left motor
 * @param r is pwm signal on right motor
 */
void drive_set_pwm(int l, int r){
  _leftMotor->drive(l);
  _rightMotor->drive(r);
}
