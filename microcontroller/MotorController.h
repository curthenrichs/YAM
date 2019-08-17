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
		// Enable motor pin
    byte _enable;
    // Clockwise PWM output
    byte _clw_pwm;
    // Counter-clockwise PWM output
    byte _cclw_pwm;

	public:
		/**
		 * Constructor for motor controller maps the pins addresses to the software
		 * structure.
		 * @param enable_pin :: enable pin
     * @param clw_pwm_pin :: clockwise pin
     * @param cclw_pwm_pin :: counter-clockwise pin
		 */
		MotorController(byte enable_pin, byte clw_pwm_pin, byte cclw_pwm_pin);
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
