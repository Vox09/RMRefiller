#ifndef _MAIN_H_
#define _MAIN_H_

#include "ch.h"
#include "hal.h"

#include "usbcfg.h"
#include "flash.h"
#include "chprintf.h"
#include "tft_display.h"

#include "math_misc.h"
#include "canBusProcess.h"
#include "dbus.h"
#include "params.h"

#include "mpu6500.h"
#include "ist8310.h"
#include "adis16265.h"
#include "attitude.h"
#include "calibrate_sensor.h"
#include "rangefinder.h"
#include "feeder.h"

#include "gimbal.h"
#include "chassis.h"
#include "lift.h"

#include "exti.h"
#include "imu_temp.h"
#include "sdlog.h"

void shellStart(void);

/*
 * ===============================================================================================
 * Definitions for the Refiller  TODO move to their own .h file
 * ===============================================================================================
*/


#define LEFT -1
#define RIGHT 1
#define TANK_SX 0
#define TANK_DX 1
#define FEEDER_MOTOR_SPEED 1200
#define BULLETS_MAX 100
#define BULLETS_PER_CYCLE 6
#define SET_DISTANCE 5

#endif
