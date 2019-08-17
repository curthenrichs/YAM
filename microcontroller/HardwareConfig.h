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
// ...
#define MOTOR_A_PWM_1_PIN                   8
#define MOTOR_A_ENABLE_PIN                  9
#define MOTOR_B_ENABLE_PIN                  10
#define MOTOR_A_PWM_2_PIN                   11
#define MOTOR_B_PWM_1_PIN                   12
#define MOTOR_B_PWM_2_PIN                   13

//==============================================================================
//                        Constant and Macro Definition
//==============================================================================

#define DEBUGGING_MODE                      true

//==============================================================================
//                                 Debug Serial
//==============================================================================

#define DEBUG_SERIAL                        Serial
#define DEBUG_SERIAL_BAUD                   115200

#endif
