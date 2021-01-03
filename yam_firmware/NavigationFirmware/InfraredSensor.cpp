/**
 * InfraredSensor
 * @author Curt Henrichs
 * @date 1-1-21
 *
 * Barrowing code from this library to perform the noise handling 
 * (https://github.com/mcc-robotics/sharp-distance)
 */

//==============================================================================
//                               Libraries
//==============================================================================

#include "InfraredSensor.h"
#include "HardwareConfig.h"

//==============================================================================
//                     Constants and Macro Definitions
//==============================================================================

#define SAMPLE_RATE_MICROS  38000
#define BURST_DELAY_MICROS  1500

//==============================================================================
//                           Class Implementation
//==============================================================================

/**
 * Default constructor uses pin parameter to map structure to physical
 * device.
 * @param pin is analog IO pin for InfraredSensor.
 */
InfraredSensor::InfraredSensor(byte pin) {
  _pin = pin;
  _value = -1;

  pinMode(_pin, INPUT);

  _lastUpdateMicros = micros();
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

  // Sensor has changed (likely)
  if (micros() - _lastUpdateMicros < SAMPLE_RATE_MICROS) {
    return _value;
  }
  _lastUpdateMicros = micros();

  // Burst read
  int currReading;
  int lowestReading = 1024;
  for (int i=0; i<_numBurstSamples; i++) {
    currReading = analogRead(_pin);

    if (currReading < lowestReading) {
      lowestReading = currReading;
    }

    delayMicroseconds(BURST_DELAY_MICROS);
  }

  // Compute distance in meters
  float volt = lowestReading * 5.0f / 1024;
  _value = 12.08f * pow(volt, -1.058f) * 0.01f;
  return _value;
}

/**
 * Change number of burst samples
 * @param samples is number of samples to take per update
 */
void InfraredSensor::setNumBurstSamples(byte samples) {
  _numBurstSamples = samples;
}
