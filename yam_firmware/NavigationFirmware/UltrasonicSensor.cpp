/**
 * UltrasonicSensor
 * @author Curt Henrichs
 * @date 6-24-17
 *
 * UltrasonicSensor abstracts the seeedstudio ultransonic sensor into a discrete
 * software object. Algorithm implemented is drawn from the Ping ultrasonic
 * example software.
 */

//==============================================================================
//                               Libraries
//==============================================================================

#include "UltrasonicSensor.h"

//==============================================================================
//                           Class Implementation
//==============================================================================

/**
 * Default constructor uses pin parameter to map structure to physical
 * device.
 * @param pin is sensor IO pin on ultrasonic sensor.
 */
UltrasonicSensor::UltrasonicSensor(byte pin){
	//member initialization
	_pin = pin;
	_value = -1;

	//configure hardware
	pinMode(_pin, OUTPUT);
	digitalWrite(_pin, LOW);
}

/**
 * Gets the distance to closest object from the sensor from last update.
 * If distance to object is a really large value then this is effectively
 * infinity due to limit on sensor data output.
 * @return float with a distance in meters
 */
float UltrasonicSensor::getDistance(void) {
	return _value;
}

/**
 * Performs actual sensor scan in environment.
 * @return float with a distance in meters
 */
float UltrasonicSensor::update(void){
	//these are the variabels for the duration of the high pulse and the distance
	// away an object is
	long duration;
	float meters;

	//trigger the ping with a two mircosecond high pulse then drop low before
	// switching to input
	pinMode(_pin, OUTPUT);
	digitalWrite(_pin, LOW);
	delayMicroseconds(2);
	digitalWrite(_pin, HIGH);
	delayMicroseconds(5);
	digitalWrite(_pin, LOW);

	//switch to input then wait for the trigger
	pinMode(_pin, INPUT);
	duration = pulseIn(_pin, HIGH); //PULSE_IN_TIMEOUT

	//convert to meters
	meters = duration / 5800.0f; // 29.0f / 2.0f / 100.0f;

	//return the value to the program
	_value = meters;
	return meters;
}
