/**
 * UltrasonicSensor
 * @author Curt Henrichs
 * @date 6-24-17
 *
 * UltrasonicSensor abstracts the seeedstudio ultransonic sensor into a discrete
 * software object. Algorithm implemented is drawn from the Ping ultrasonic
 * example software.
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

//==============================================================================
//                               Libraries
//==============================================================================

#include <Arduino.h>

//==============================================================================
//                           Class Declaration
//==============================================================================

/**
 * Sensor object that produces a distance value when requested.
 */
class UltrasonicSensor{
	private:
		// pin connected to sensor IO line
		byte _pin;

	public:
		/**
		 * Default constructor uses pin parameter to map structure to physical
		 * device.
		 * @param pin is sensor IO pin on ultrasonic sensor.
		 */
		UltrasonicSensor(byte pin);
		/**
		 * Gets the distance to closest object from the sensor. If distance to
		 * object is a really large value then this is effectively infinity due
		 * to limit on sensor data output.
		 * @return long with a distance in inches
		 */
		long getDistance(void);
};

#endif
