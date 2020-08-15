/**
 * RaspberryPiPowerInterface
 * @author Curt Henrichs
 * @date 5-29-20
 *
 * Interface to raspberry pi power subsystem used to control halt / boot state
 * for the raspberry pi. Requires custom script on the pi (found in the
 * yam_firmware package's scripts directory).
 *
 * Arduino should be connected to RUN pin and a GPIO pin for halt on the pi. Take
 * care in making sure the connection is 3.3V safe for the raspberry pi.
 * Additionally, connect the arduino to the UART TX pin on the raspberry pi as
 * a means of monitoring active state. Lastly connect a momentary pushbutton to
 * the arduino. The pushbutton is debounced in software.
 */

//==============================================================================
//                                Libraries
//==============================================================================

#include "RaspberryPiPowerInterface.h"

//==============================================================================
//                      Constants and Macro Definitions
//==============================================================================

#define ACTIVE_DEBOUNCE_INTERVAL  (25)
#define BUTTON_DEBOUNCE_INTERVAL  (25)

//==============================================================================
//                             Class Implemenation
//==============================================================================

RaspberryPiPowerInterface::RaspberryPiPowerInterface(byte runPin, byte haltPin,
      byte activePin, byte buttonPin) {
  _isOn = false;

  _runPin = runPin;
  _haltPin = haltPin;
  _activePin = activePin;
  _buttonPin = buttonPin;

  _activeDebounce = Bounce(); // Active Low
  _buttonDebounce = Bounce(); // Active High
}

void RaspberryPiPowerInterface::begin(void) {
  pinMode(_runPin,OUTPUT);
  pinMode(_haltPin,OUTPUT);
  pinMode(_activePin,INPUT);
  pinMode(_buttonPin,INPUT);

  digitalWrite(_runPin,LOW);
  digitalWrite(_haltPin,HIGH);

  _activeDebounce.attach(_activePin);
  _activeDebounce.interval(ACTIVE_DEBOUNCE_INTERVAL);
  _buttonDebounce.attach(_buttonPin);
  _buttonDebounce.interval(BUTTON_DEBOUNCE_INTERVAL);
}

void RaspberryPiPowerInterface::update(void) {
  _activeDebounce.update();
  _buttonDebounce.update();

  if (_isOn) {
    //Serial.println("On");
    if (_buttonDebounce.rose() || _activeDebounce.read() == HIGH) { // Then stop
      //Serial.print("Stopping ");

      _isOn = false;
      digitalWrite(_haltPin,HIGH);
      while (_activeDebounce.read() == LOW) {
        /* Loop until condition - Okay to block since ROS-Comm not needed */
        //Serial.print(".");
        _activeDebounce.update();
        _buttonDebounce.update();
        delay(10);
      }

      delay(30000); // Wait 30s before pulling the plug
      
      digitalWrite(_runPin,LOW);
      //Serial.println(" Done");
    }
  } else {
    //Serial.println("Off");
    if (_buttonDebounce.rose() || _activeDebounce.read() == LOW) { // Then start
      //Serial.print("Starting ");

      _isOn = true;
      digitalWrite(_haltPin,LOW);
      digitalWrite(_runPin,HIGH);

      // Delay until stable On
      while (_activeDebounce.read() == HIGH) {
        /* Loop until condition - Okay to block since ROS-Comm not needed */
        _activeDebounce.update();
        _buttonDebounce.update();
        delay(10);
      }

      //Serial.println(" Done");
    }
  }
}

bool RaspberryPiPowerInterface::isOn(void) {
  return _isOn;
}
