/**
 * InfraredSensor
 * @author Curt Henrichs
 * @date 1-1-21
 *
 * 
 */

//==============================================================================
//                               Libraries
//==============================================================================

#include "InfraredSensor.h"
#include "HardwareConfig.h"

//==============================================================================
//                           Class Implementation
//==============================================================================

/**
 * Default constructor uses pin parameter to map structure to physical
 * device.
 * @param pin is analog IO pin for InfraredSensor.
 */
InfraredSensor::InfraredSensor(byte pin) : {
  _pin = pin;
  _value = -1;
}

/**
 * Gets the distance to closest object from the sensor from last update.
 * Voltage read from analog signal is converted into distance estimate. 
 * Voltage saturation to power-rail is effectively infinite distance measured.
 */
float InfraredSensor::getDistance(void) {
  return _value;
}

/**
 * Performs actual sensor scan in environment.
 * @return float with a distance in meters
 */
float InfraredSensor::update(void){
  float analogRead(_pin) * 5.0f / 1024;
  _value = 12.08f * pow(volt, -1.058f) * 0.01f;
  return _value;
}
