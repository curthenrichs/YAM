/**
 * MotorController
 * @author Curt Henrichs
 * @date 12-25-19
 *
 * Controller interface for the Cytron RB-Cyt-153 Motor Controller.
 */

//==============================================================================
//                                Libraries
//==============================================================================

#include "MotorController.h"
#include "HardwareConfig.h"

#include <Arduino.h>

//==============================================================================
//                           Class Implementation
//==============================================================================

/**
 * Constructor for motor controller maps the pins addresses to the software
 * structure.
 * @param dir_pin :: direction pin
 * @param pwm_pin :: pwm pin
 */
MotorController::MotorController(byte dir_pin, byte pwm_pin){
  _dir_pin = dir_pin;
  _pwm_pin = pwm_pin;
}

/**
 * Sets up interface with hardware, as constructor is called before hardware
 * in enumerated, thus an error occurs.
 */
void MotorController::begin(void){
  pinMode(_dir_pin,OUTPUT);
  pinMode(_pwm_pin,OUTPUT);
  digitalWrite(_dir_pin,LOW);
  digitalWrite(_pwm_pin,LOW);
}

/**
 * Command the motor to stop moving (what ever that means)
 */
void MotorController::stop(void) {
  digitalWrite(_dir_pin,LOW);
  digitalWrite(_pwm_pin,LOW);
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
	digitalWrite(_dir_pin,LOW);
	analogWrite(_pwm_pin,speed);
}

/**
 * Commands the motor in the reverse direction (what ever that means)
 * @param speed is scaler PWM speed to travel at
 */
void MotorController::reverse(byte speed){
	digitalWrite(_dir_pin,HIGH);
	analogWrite(_pwm_pin,speed);
}
