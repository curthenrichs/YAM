/**
 * VoltageMonitor
 * @author Curt Henrichs
 * @date 12-25-19
 *
 * Interface ADC and provide voltage conversions
 */

#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

//==============================================================================
//                                Libraries
//==============================================================================

#include <Arduino.h>

//==============================================================================
//                      Constants and Macro Definitions
//==============================================================================

enum State {
  POWER_GOOD,
  POWER_LOW,
  POWER_EMERGENCY,
  ESTOP_ACTIVE
};

//==============================================================================
//                             Class Declaration
//==============================================================================

/**
 * Wrap ADC interface and conversions for each battery
 */
class VoltageMonitor {

  private:
    // store last value from ADC
    int _value;
    // physical ADC pin to read
    byte _pin;
    // slope of conversion line
    float _slope;
    // intercept of conversion line
    float _intercept;

  public:

    VoltageMonitor(byte pin, float slope, float intercept);

    VoltageMonitor(byte pin, float slope);

    VoltageMonitor(byte pin);

    float getVoltage(void);

    int getRaw(void);

    enum State getState(void);

    void update(void);
};

#endif
