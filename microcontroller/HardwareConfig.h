/**
 * HardwareConfig
 * @author Curt Henrichs
 * @date 1-2-19
 *
 * Hardware configuration file defines the pinout of the platform in addditon to
 * defining common constants used for program executation.
 *
 * Pinout:
 *    -> 0  Serial RX
 *    -> 1  Serial TX
 */

#ifndef HARDWARECONFIG_H
#define HARDWARECONFIG_H

//==============================================================================
//                               Libraries
//==============================================================================

#include <Arduino.h>

//==============================================================================
//                              Device Pinout
//==============================================================================

#define DEBUG_SERIAL_RX_PIN                 0
#define DEBUG_SERIAL_TX_PIN                 1

//==============================================================================
//                        Constant and Macro Definition
//==============================================================================

#define DEBUGGING_MODE                      false

//==============================================================================
//                                 Debug Serial
//==============================================================================

#define DEBUG_SERIAL                        Serial
#define DEBUG_SERIAL_BAUD                   115200

#endif
