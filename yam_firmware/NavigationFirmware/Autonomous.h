/**
 * Autonomous
 * @author Curt Henrichs
 * @date 8-8-20
 * 
 * Autonomous module defines the state-machine to control Rovebot. The
 * state-machine architecture is built on the switch-case / subroutine
 * 'dispatcher' model. Everything is dependent on timer events running in the
 * main loop so it should be called every main loop tick.
 */

#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

//==============================================================================
//                               Libraries
//==============================================================================

#include <Arduino.h>

//==============================================================================
//                       Constant and Macro Definitions
//==============================================================================

/**
 * Defines the movement state possible for autonomous process.
 */
typedef enum AutonState {
  MOVE_FWD,
  MOVE_BWD,
  MOVE_LEFT,
  MOVE_RIGHT,
  WHICH_WAY_SLCT,
  DELAY_FWD_HEAD,
  DELAY_BWD_HEAD,
  ERROR
} AutonState_e;

//==============================================================================
//                         Pulic Function Prototypes
//==============================================================================

/**
 * Resets autonomous state-machine. Call once after a need to reset the state
 * machine such as moving from manual operation to autonomous.
 */
void auton_begin(void);

/**
 * Main loop function that will handle the current auton state. This functions
 * could take a large finite amount of time to run due to the nature of the
 * robot.
 * @param ul is left ultrasonic sensor reading
 * @param uc is center ultrasonic sensor reading
 * @param ur is right ultrasonic sensor reading
 * @param sl is left sharp ir sensor reading
 * @param sr is right sharp ir sensor reading
 */
void auton_update(float ul, float uc, float ur, float sl, float sr);

/**
 * @return Current state control value for left motor
 */
int auton_left_motor(void);

/**
 * @return Current state control value for right motor
 */
int auton_right_motor(void);

/**
 * @return current state of the autonomous state-machine
 */
AutonState_e auton_get_state(void);

#endif
