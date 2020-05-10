/**
 * BatteryMonitor
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
//                             Class Declaration
//==============================================================================

/**
 * Wrap ADC interface and conversions for each battery
 */
class BatteryMonitor {

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

    BatteryMonitor(byte pin, float slope, float intercept);

    BatteryMonitor(byte pin, float slope);

    BatteryMonitor(byte pin);

    float getVoltage(void);

    int getRaw(void);

    void update(void);
};

#endif
