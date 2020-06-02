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

#include <Arduino.h>
#include <Bounce2.h>

//==============================================================================
//                             Class Declaration
//==============================================================================

/**
 *
 */
class RaspberryPiPowerInterface {

  private:

    bool _isOn;

    byte _runPin;
    byte _haltPin;
    byte _activePin;
    byte _buttonPin;

    Bounce _activeDebounce;
    Bounce _buttonDebounce;

  public:

    RaspberryPiPowerInterface(byte runPin, byte haltPin, byte activePin,
          byte buttonPin);

    void begin(void);

    void update(void);

};
