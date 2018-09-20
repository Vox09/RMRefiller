#ifndef _MAIN_H_
#define _MAIN_H_

#include "ch.h"
#include "hal.h"

#include "usbcfg.h"
#include "flash.h"
#include "chprintf.h"

#include "math_misc.h"
#include "canBusProcess.h"
#include "dbus.h"
#include "params.h"

#include "feeder.h"
#include "rangefinder.h"
#include "exti.h"
#include "sdlog.h"
#include "servo.h"
#include "lift.h"

void shellStart(void);

/*
 * ===============================================================================================
 * Definitions for the Refiller  TODO move to their own .h file
 * ===============================================================================================
*/

#define BULLETS_INIT 120
#define BULLETS_MAX  120
#define SET_DISTANCE 50.0
#define DOOR_DELAY 1000

typedef enum{
    REFILLER_IDLE = 0,
    REFILLER_FEEDER_R,
    REFILLER_FEEDER_L,
    REFILLER_FEEDER_B
} refiller_state_t;

//#define DEBUG
bool* getDoor(void);
bool* getDistanceOK(void);
bool* getFinished(void);
#endif
/*
 * 200 bullets when game start
 * then 150 bullets per min
 */