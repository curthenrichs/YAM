/**
 * VoltageMonitor
 * @author Curt Henrichs
 * @date 12-25-19
 *
 * Interface ADC and provide voltage conversions
 */

//==============================================================================
//                                Libraries
//==============================================================================

#include "VoltageMonitor.h"
#include "HardwareConfig.h"

//==============================================================================
//                      Constants and Macro Definitions
//==============================================================================

#define POWER_GOOD_VOLTAGE    (12.0f)
#define POWER_LOW_VOLTAGE     (11.5f)
#define ESTOP_VOLTAGE         (1.0f)

//==============================================================================
//                           Class Implementation
//==============================================================================

VoltageMonitor::VoltageMonitor(byte pin, float slope, float intercept) {
  _value = 0;
  _pin = pin;
  _slope = slope;
  _intercept = intercept;
}

VoltageMonitor::VoltageMonitor(byte pin, float slope) {
  VoltageMonitor(pin,slope,0.0f);
}

VoltageMonitor::VoltageMonitor(byte pin) {
  VoltageMonitor(pin,1.0f,0.0f);
}

void VoltageMonitor::update(void) {
  _value = analogRead(_pin);
}

float VoltageMonitor::getVoltage(void) {
  return _value * _slope + _intercept;
}

int VoltageMonitor::getRaw(void) {
  return _value;
}

PowerState_e VoltageMonitor::getState(void) {
  float voltage = getVoltage();

  if (voltage > POWER_GOOD_VOLTAGE) {
    return POWER_GOOD;
  } else if (voltage > POWER_LOW_VOLTAGE) {
    return POWER_LOW;
  } else if (voltage <= ESTOP_VOLTAGE) {
    return ESTOP_ACTIVE;
  } else {
    return POWER_EMERGENCY;
  }
}
