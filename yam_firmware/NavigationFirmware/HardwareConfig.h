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
#define ULTRASONIC_0_PIN                    (2)       // Left
#define ULTRASONIC_1_PIN                    (3)       // Center
#define UlTRASONIC_2_PIN                    (4)       // Right
#define VOLTAGE_VIN_PIN                     (A0)      
#define INFRARED_0_PIN                      (A1)      // Left
#define INFRARED_1_PIN                      (A2)      // Right

//==============================================================================
//                        Constant and Macro Definition
//==============================================================================

#define VOLTAGE_VIN_SLOPE                   (12.6f/1023)
#define VOLTAGE_VIN_INTERCEPT               (0)

#define SHARP_IR_MODEL                      (430) // GP2YA41SK0F Model Code for lib

#endif
