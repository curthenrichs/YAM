/**
 * HardwareConfig
 * @author Curt Henrichs
 * @date 1-2-19
 *
 * Hardware configuration file defines the pinout of the platform in addditon to
 * defining common constants used for program executation.
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

#define MOTOR_0_PWM_PIN                     (11)
#define MOTOR_0_DIR_PIN                     (10)
#define MOTOR_1_PWM_PIN                     (13)
#define MOTOR_1_DIR_PIN                     (12)
#define ULTRASONIC_0_PIN                    (2)
#define ULTRASONIC_1_PIN                    (3)
#define UlTRASONIC_2_PIN                    (4)
#define VOLTAGE_VIN_PIN                     (A0)

//==============================================================================
//                        Constant and Macro Definition
//==============================================================================

#define VOLTAGE_VIN_SLOPE                   (12.6f/1023)
#define VOLTAGE_VIN_INTERCEPT               (0)

#endif
