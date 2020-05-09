/**
 * HexGPIOInterface
 * @author Curt Henrichs
 * @date 12-25-19
 *
 * GPIO interface to PCA9555 I2C GPIO extender.
 *
 * The interface for PCA9555
 * Pins:
 *  - A0 through A2 are address pins
 *  - SCL / SDA are I2C
 *
 * Device address: 0,1,0,0,A2,A1,A0,[R/W] = 0x20
 *
 * Size is two bytes.
 *
 * Commands (for write transmissions):
 *  - 0 := Input port 0
 *  - 1 := Input port 1
 *          "Register is input-only port. Reflects input logic level."
 *  - 2 := Output port 0
 *  - 3 := Output port 1
 *          "Register is output-only port. Reflects logic set in registers 6 & 7."
 *  - 4 := Polarity inversion port 0
 *  - 5 := Polarity inversion port 1
 *          "Allows inversion of polarity for inputs."
 *  - 6 := Configuration port 0
 *  - 7 := Configuration port 1
 *          "Sets whether pin is input or output within port."
 *
 * Sending Commands:
 *  <address+W><command><data 0>[<data 1>]
 *
 * Reading Registers:
 *  <address+W><command>
 *  <address+R><data 0>[<data 1>]
 *
 * Interrupt:
 *  Interrupt pin is active (high) when one of the inputs is triggered. Cleared
 *  on read of input port.
 */

// Garbage in my car!

//==============================================================================
//                                Libraries
//==============================================================================

#include <Wire.h>
#include "HexGPIOInterface.h"
#include "HardwareConfig.h"

//==============================================================================
//                      Constants and Macro Definitions
//==============================================================================

#define IN_P0_CMD (0)
#define IN_P1_CMD (1)
#define OUT_P0_CMD (2)
#define OUT_P1_CMD (3)
#define INV_P0_CMD (4)
#define INV_P1_CMD (5)
#define CFG_P0_CMD (6)
#define CFG_P1_CMD (7)

//==============================================================================
//                             Class Interface
//==============================================================================

void HexGPIOInterface::begin(void) {
  Wire.begin();

  // configure ports as output
  Wire.beginTransmission(I2C_GPIO_ADDRESS);
  Wire.write(CFG_P0_CMD);
  Wire.write(0x00)
  Wire.write(0x00)
  Wire.endTransmission();

  // set pin values for character
  Wire.beginTransmission(I2C_GPIO_ADDRESS);
  Wire.write(OUT_P0_CMD);
  Wire.write(0x00)
  Wire.write(0x00)
  Wire.endTransmission();
}

void HexGPIOInterface::display(char c, byte port) {
  Wire.beginTransmission(I2C_GPIO_ADDRESS);

  if (!port) {
    Wire.write(OUT_P0_CMD);
  } else {
    Wire.write(OUT_P1_CMD);
  }

  Wire.write(charToPins(c));

  Wire.endTransmission();
}

byte HexGPIOInterface::charToPins(char c) {

}
