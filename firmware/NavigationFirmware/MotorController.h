/**
 * MotorController
 * @author Curt Henrichs
 * @date 12-25-19
 *
 * Controller interface for the Cytron RB-Cyt-153 Motor Controller.
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
 * MotorController handles the interface between the hardware motor
 */
class MotorController {

	private:
		// Direction pin
		byte _dir_pin;
		// PWM speed pin
		byte _pwm_pin;

	public:
		/**
		 * Constructor for motor controller maps the pins addresses to the software
		 * structure.
		 * @param dir_pin :: direction pin
     * @param pwm_pin :: pwm pin
		 */
		MotorController(byte dir_pin, byte pwm_pin);
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
