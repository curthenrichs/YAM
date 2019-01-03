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
//                       Constants and Macro Definitions
//==============================================================================

#define OFF_PWM 										(0)	//! No signal on pin = 0ff

//==============================================================================
//                           Class Implementation
//==============================================================================

/**
 * Constructor for motor controller maps the pins addresses to the software
 * structure.
 * @param dirPin direction control pin
 * @param stopPin enable/disable pin for controller
 * @param speedPin PWM pin to control speed of motor
 */
MotorController::MotorController(byte dirPin, byte stopPin, byte speedPin){
	_dirPin = dirPin;
	_stopPin = stopPin;
	_speedPin = speedPin;
}

/**
 * Sets up interface with hardware, as constructor is called before hardware
 * in enumerated, thus an error occurs.
 */
void MotorController::begin(void){
  pinMode(_dirPin,OUTPUT);
  pinMode(_stopPin,OUTPUT);
  pinMode(_speedPin,OUTPUT);
  digitalWrite(_dirPin,LOW);
  digitalWrite(_stopPin,LOW);
  digitalWrite(_speedPin,LOW);
}

/**
 * Command the motor to stop moving (what ever that means)
 */
void MotorController::stop(void) {
	analogWrite(_speedPin,OFF_PWM);
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
	digitalWrite(_dirPin,LOW);
	digitalWrite(_stopPin,LOW);
	analogWrite(_speedPin,speed);
}

/**
 * Commands the motor in the reverse direction (what ever that means)
 * @param speed is scaler PWM speed to travel at
 */
void MotorController::reverse(byte speed){
	digitalWrite(_dirPin,HIGH);
	digitalWrite(_stopPin,LOW);
	analogWrite(_speedPin,speed);
}
