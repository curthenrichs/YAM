/**
 * InfraredSensor
 * @author Curt Henrichs
 * @date 1-1-21
 *
 * 
 */

#ifndef INFRARED_H
#define INFRARED_H

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
class InfraredSensor{
  private:
    // pin connected to sensor IO line
    byte _pin;
    float _value;

  public:
    /**
     * Default constructor uses pin parameter to map structure to physical
     * device.
     * @param pin is analog IO pin for InfraredSensor.
     */
    InfraredSensor(byte pin);
    /**
     * Gets the distance to closest object from the sensor from last update.
     * Voltage read from analog signal is converted into distance estimate. 
     * Voltage saturation to power-rail is effectively infinite distance measured.
     */
    float getDistance(void);
    /**
     * Performs actual sensor scan in environment.
     * @return float with a distance in meters
     */
    float update(void);
};

#endif
