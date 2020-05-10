//==============================================================================
//                               Libraries
//==============================================================================

#include "Watchdog.h"
#include "RunningAverage.h"
#include <elapsedMillis.h>

//==============================================================================
//                       Constant and Macro Definitions
//==============================================================================

/**
 * Default size of tha running average window. Increasing this will increase the
 * memory footprint.
 */
#define WATCHDOG_WINDOW_SIZE 10

/**
 * Average time between messages must be below this in order to not lockout motors
 */
#define WATCHDOG_TIME_THRESHOLD 1000

//==============================================================================
//                           Private Attributes
//==============================================================================

static elapsedMillis _watchdog_time_from_last;
static RunningAverage _watchdog_connection_status(WATCHDOG_WINDOW_SIZE);

//==============================================================================
//                      Public Function Implementation
//==============================================================================

/**
 * Initialize the watchdog. Starts not locked out.
 */
void watchdog_begin(void) {
  _watchdog_connection_status.clear();
  _watchdog_connection_status.addValue(WATCHDOG_TIME_THRESHOLD/2); //remove nan
  _watchdog_time_from_last = 0;
}

/**
 * Feed watchdog when message arrives. If not fed fast enough then lockout 
 * occurs. Should also feed periodic above the threshold rate to induce
 * lockout.
 */
void watchdog_feed(void){
  _watchdog_connection_status.addValue(_watchdog_time_from_last);
  _watchdog_time_from_last = 0;
}

/**
 * @return boolean whether the watchdog is currently locked out
 */
bool watchdog_is_locked(void){
  return _watchdog_connection_status.getAverage() > WATCHDOG_TIME_THRESHOLD;
}
