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

void shellStart(void);

/*
 * ===============================================================================================
 * Definitions for the Refiller  TODO move to their own .h file
 * ===============================================================================================
*/

#define LEFT 1
#define RIGHT 0
#define BULLETS_MAX 10
#define SET_DISTANCE 5
#define DOOR_DELAY 1000

//#define DEBUG
#endif
/*
 * 200 bullets when game start
 * then 150 bullets per min
 */