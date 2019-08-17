/*
 * TODO write this
 */

// Note need to develop battery interface
// Need to develop JSON serial interface

//==============================================================================
//                               Libraries
//==============================================================================

#include "HardwareConfig.h"
#include "Drivetrain.h"
#include "MotorController.h"

#include "RunningAverage.h"
#include <elapsedMillis.h>
#include <ArduinoJson.h>

//==============================================================================
//                         Constants and Macro Declaration
//==============================================================================

/**
 * Default size of tha running average window. Increasing this will increase the
 * memory foot.
 */
#define WATCHDOG_WINDOW_SIZE 10
/**
 * Average time between messages must be below this in order to not lockout motors
 */
#define WATCHDOG_TIME_THRESHOLD 75

//==============================================================================
//                           Private Attributes
//==============================================================================

/**
 * Running average of packet times
 */
static RunningAverage _watchdog_connection_status(WATCHDOG_WINDOW_SIZE);
/**
 * Track time between messages starting at connection
 */
static elapsedMillis _watchdog_time_from_last;
/**
 * Drivetrain is set to move
 */
static bool _drivetrain_active;
/**
 *
 */
static MotorController motor_left(MOTOR_A_ENABLE_PIN,MOTOR_A_PWM_1_PIN,MOTOR_A_PWM_2_PIN);
/**
 *
 */
static MotorController motor_right(MOTOR_B_ENABLE_PIN,MOTOR_B_PWM_1_PIN,MOTOR_B_PWM_2_PIN);

//==============================================================================
//                        Private Function Prototypes
//==============================================================================

/**
 * Updates the motor watchdog, used to auto-disable if connection is bad
 */
void _feed_motor_watchdog(void);

//==============================================================================
//                                 MAIN
//==============================================================================

/**
 * Main intialization will setup the wifi connection and web server
 */
void setup() {` 
  _watchdog_connection_status.clear();
  _watchdog_connection_status.addValue(WATCHDOG_TIME_THRESHOLD/2); //remove nan
  _watchdog_time_from_last = 0;
  _drivetrain_active = false;

  drive_init(&motor_left,&motor_right);
  drive_hard_stop();

#if DEBUGGING_MODE
  DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUD);
  DEBUG_SERIAL.println("Starting Serial Debugger");
#endif
}

/**
 *
 */
void loop() {

  _feed_motor_watchdog();
  drive_cartesian(0,100);

  // motor lock
  if(_watchdog_locked_out() && _drivetrain_active){
    drive_hard_stop();
    _drivetrain_active = false;
#if DEBUGGING_MODE
    DEBUG_SERIAL.println("Motor lockout");
#endif
  }
}

//==============================================================================
//                         Private Function Implementation
//==============================================================================

/**
 * Updates the motor watchdog, used to auto-disable if connection is bad
 */
void _feed_motor_watchdog(void){
  _watchdog_connection_status.addValue(_watchdog_time_from_last);
  _watchdog_time_from_last = 0;
}

/**
 * Determines whether the watchdog is currently locking motor control
 */
bool _watchdog_locked_out(void){
  return _watchdog_connection_status.getAverage() > WATCHDOG_TIME_THRESHOLD;
}
