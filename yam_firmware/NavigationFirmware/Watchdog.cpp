/**
 * Watchdog
 * @author Curt Henrichs
 * @date 1-27-19 
 * 
 * Provides a simple watchdog subsystem that should be fed when receiving 
 * messages. This can be used to lock unsafe subsystems in case of communication
 * failure.
 */
 
//==============================================================================
//                               Libraries
//==============================================================================

#include "Watchdog.h"
#include <elapsedMillis.h>

//==============================================================================
//                       Constant and Macro Definitions
//==============================================================================

/**
 * Average time between messages must be below this in order to not lockout motors
 */
#define WATCHDOG_TIME_THRESHOLD 1000

//==============================================================================
//                           Private Attributes
//==============================================================================

static elapsedMillis _watchdog_time_from_last;

//==============================================================================
//                      Public Function Implementation
//==============================================================================

/**
 * Initialize the watchdog. Starts not locked out.
 */
void watchdog_begin(void) {
  _watchdog_time_from_last = 0;
}

/**
 * Feed watchdog when message arrives. If not fed fast enough then lockout 
 * occurs. Should also feed periodic above the threshold rate to induce
 * lockout.
 */
void watchdog_feed(void){
  _watchdog_time_from_last = 0;
}

/**
 * @return boolean whether the watchdog is currently locked out
 */
bool watchdog_is_locked(void){
  return _watchdog_time_from_last > WATCHDOG_TIME_THRESHOLD;
}
