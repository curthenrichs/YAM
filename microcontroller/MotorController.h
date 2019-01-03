/**
 * MotorController
 * @author Curt Henrichs
 * @date 7-22-2018
 *
 * Abstraction of the Arduino motor controller shield to control the single
 * drive motor of roverbot.
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

//==============================================================================
//                                Libraries
//==============================================================================

#include <Arduino.h>

//==============================================================================
//                             Class Declaration
//==============================================================================

/**
 * MotorController handles the interface between the hardware motor, Roverbot's
 * internal control signals, and the aims of the firmware interface.
 */
class MotorController{

	private:
		// Direction control pin (clockwise, counter-clockwise)
		byte _dirPin;
		// Stop pin, disables the active control of the motor.
		byte _stopPin;
		// Speed pin, controls the scaler speed of the motor.
		byte _speedPin;

	public:
		/**
		 * Constructor for motor controller maps the pins addresses to the software
		 * structure.
		 * @param dirPin direction control pin
		 * @param stopPin enable/disable pin for controller
		 * @param speedPin PWM pin to control speed of motor
		 */
		MotorController(byte dirPin, byte stopPin, byte speedPin);
		/**
     * Sets up interface with hardware, as constructor is called before hardware
     * in enumerated, thus an error occurs.
     */
    void begin(void);
		/**
		 * Command the motor to stop moving (what ever that means)
		 */
		void stop(void);
		/**
		 * Set motor with signed PWM signal
		 * @param speed is signed PWM speed to travel at
		 */
		void drive(int speed) ;
		/**
		 * Commands the motor in the forward direction (what ever that means)
		 * @param speed is scaler PWM speed to travel at
		 */
		void forward(byte speed);
		/**
		 * Commands the motor in the reverse direction (what ever that means)
		 * @param speed is scaler PWM speed to travel at
		 */
		void reverse(byte speed);
};

#endif
