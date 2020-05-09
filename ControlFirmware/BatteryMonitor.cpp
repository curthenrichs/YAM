/**
 * BatteryMonitor
 * @author Curt Henrichs
 * @date 12-25-19
 *
 * Interface ADC and provide voltage conversions
 */

//==============================================================================
//                                Libraries
//==============================================================================

#include "BatteryMonitor.h"
#include "HardwareConfig.h"

//==============================================================================
//                           Class Implementation
//==============================================================================

BatteryMonitor::BatteryMonitor(byte pin, float slope, float intercept) {
  _value = 0;
  _pin = pin;
  _slope = slope;
  _intercept = intercept;
}

BatteryMonitor::BatteryMonitor(byte pin, float slope) {
  BatteryMonitor(pin,slope,0.0f);
}

BatteryMonitor::BatteryMonitor(byte pin) {
  BatteryMonitor(pin,1.0f,0.0f);
}

void BatteryMonitor::update(void) {
  _value = analogRead(_pin);
}

float BatteryMonitor::getVoltage(void) {
  return _value * _slope + _intercept;
}

int BatteryMonitor::getRaw(void) {
  return _value;
}
