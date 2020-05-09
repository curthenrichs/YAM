/**
 * Watchdog
 * @author Curt Henrichs
 * @date 1-27-19 
 */
 
#ifndef WATCHDOG_H
#define WATCHDOG_H

//==============================================================================
//                               Libraries
//==============================================================================

#include <Arduino.h>

//==============================================================================
//                      Public Function Prototypes
//==============================================================================

/**
 * Initialize the watchdog. Starts not locked out.
 */
void watchdog_begin(void);

/**
 * Feed watchdog when message arrives. If not fed fast enough then lockout 
 * occurs. Should also feed periodic above the threshold rate to induce
 * lockout.
 */
void watchdog_feed(void);

/**
 * @return boolean whether the watchdog is currently locked out
 */
bool watchdog_is_locked(void);

#endif
