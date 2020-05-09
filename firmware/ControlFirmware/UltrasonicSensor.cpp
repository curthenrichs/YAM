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
	_value = 0;

	//configure hardware
	pinMode(_pin, OUTPUT);
	digitalWrite(_pin, LOW);
}

/**
 * Gets the distance to closest object from the sensor from last update.
 * If distance to object is a really large value then this is effectively
 * infinity due to limit on sensor data output.
 * @return long with a distance in inches
 */
long UltrasonicSensor::getDistance(void) {
	return _value;
}

/**
 * Performs actual sensor scan in environment.
 */
void UltrasonicSensor::update(void){
	//these are the variabels for the duration of the high pulse and the distance
	// away an object is
	long duration, inches;

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
	duration = pulseIn(_pin, HIGH);

	//convert to inches
	inches = duration/74/2;

	//return the value to the program
	_value = inches;
}
