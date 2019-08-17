/**
 * MotorController
 * @author Curt Henrichs
 * @date 7-22-2018
 *
 * Abstraction of the Arduino motor controller shield to control the single
 * drive motor of roverbot.
 */

//==============================================================================
//                                Libraries
//==============================================================================

#include "MotorController.h"
#include "HardwareConfig.h"

//==============================================================================
//                           Class Implementation
//==============================================================================

/**
 * Constructor for motor controller maps the pins addresses to the software
 * structure.
 * @param enable_pin :: enable pin
 * @param clw_pwm_pin :: clockwise pin
 * @param cclw_pwm_pin :: counter-clockwise pin
 */
MotorController::MotorController(byte enable_pin, byte clw_pwm_pin, byte cclw_pwm_pin){
    _enable = enable_pin;
    _clw_pwm = clw_pwm_pin;
    _cclw_pwm = cclw_pwm_pin;
}

/**
 * Sets up interface with hardware, as constructor is called before hardware
 * in enumerated, thus an error occurs.
 */
void MotorController::begin(void){
  pinMode(_enable,OUTPUT);
  pinMode(_clw_pwm,OUTPUT);
  pinMode(_cclw_pwm,OUTPUT);
  digitalWrite(_enable,LOW);
  digitalWrite(_clw_pwm,LOW);
  digitalWrite(_cclw_pwm,LOW);
}

/**
 * Command the motor to stop moving (what ever that means)
 */
void MotorController::stop(void) {
  digitalWrite(_enable,LOW);
  digitalWrite(_clw_pwm,LOW);
  digitalWrite(_cclw_pwm,LOW);
}

/**
 * Set motor with signed PWM signal
 * @param speed is signed PWM speed to travel at
 */
void MotorController::drive(int speed) {
	if (speed < 0) {
    reverse(abs(speed));
  } else {
    forward(speed);
  }
}

/**
 * Commands the motor in the forward direction (what ever that means)
 * @param speed is scaler PWM speed to travel at
 */
void MotorController::forward(byte speed){
  digitalWrite(_enable,HIGH);
	digitalWrite(_cclw_pwm,LOW);
	analogWrite(_clw_pwm,speed);
}

/**
 * Commands the motor in the reverse direction (what ever that means)
 * @param speed is scaler PWM speed to travel at
 */
void MotorController::reverse(byte speed){
  digitalWrite(_enable,HIGH);
	digitalWrite(_clw_pwm,LOW);
	analogWrite(_cclw_pwm,speed);
}
