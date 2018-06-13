/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#include "main.h"

static BaseSequentialStream* chp = (BaseSequentialStream*)&SDU1;

void open_tank (int tank);
void close_tank (int tank);

typedef enum{
  REFILLER_IDLE = 0,
  REFILLER_FEEDER_CW,
  REFILLER_FEEDER_CCW
} refiller_state_t;

static refiller_state_t refiller_state;

static THD_WORKING_AREA(refiller_control_wa, 512);
static THD_FUNCTION(refiller_control, p){
    (void) p;
    chRegSetThreadName("refiller_control_right");

    bool      door_open[2]    = {false, false};
    float     distance[2];
    uint32_t  bullet_count[2] = {0, 0};
    systime_t refill_start_time[2];
    systime_t robot_present_time[2];
    systime_t door_open_time[2];
    while(!chThdShouldTerminateX())
    {

      // Calculate the distance from rangefinder sensors on both sides of the Refiller
      distance[TANK_SX] = rangeFinder_getDistance(RANGEFINDER_INDEX_0);
      distance[TANK_DX] = rangeFinder_getDistance(RANGEFINDER_INDEX_1);
      systime_t curr_time = chVTGetSystemTimeX();

      // Decide which tank needs to be refilled first, prioritizing the right one (easier to access during the game)
      if(bullet_count[TANK_DX] >= BULLETS_MAX && bullet_count[TANK_SX] >= BULLETS_MAX)
      {
        refiller_state == REFILLER_IDLE;
      }

      switch (refiller_state)
      {
        case REFILLER_IDLE:
          if (bullet_count[TANK_SX] < BULLETS_MAX && !door_open[TANK_SX])
          {
            feeder_refill(FEEDER_CW, 100);
            refiller_state = REFILLER_FEEDER_CW;
            refill_start_time[TANK_SX] = curr_time;
          }
          else if (bullet_count[TANK_DX] < BULLETS_MAX && !door_open[TANK_DX])
          {
            feeder_refill(FEEDER_CCW, 100);
            refiller_state = REFILLER_FEEDER_CCW;
            refill_start_time[TANK_DX] = curr_time;
          }
        break;
        case REFILLER_FEEDER_CW:
          LEDR_ON();
          if(bullet_count[TANK_SX] >= BULLETS_MAX || door_open[TANK_SX])
          {
            feeder_brake();
            refiller_state = REFILLER_IDLE;
          }
          else if(curr_time > S2ST(5) + refill_start_time[TANK_SX])
          //Not enough bullets... Trying to balance the two refilling station
          {
            feeder_brake();
            chThdSleepMilliseconds(10);
            feeder_refill(FEEDER_CCW, 100);
            refill_start_time[TANK_DX] = curr_time;
            refiller_state = REFILLER_FEEDER_CCW;
          }
        break;
        case REFILLER_FEEDER_CCW:
          LEDR_OFF();
          if(bullet_count[TANK_DX] >= BULLETS_MAX || door_open[TANK_DX])
          {
            feeder_brake();
            chThdSleepMilliseconds(10);
            refiller_state = REFILLER_IDLE;
          }
          else if(curr_time > S2ST(5) + refill_start_time[TANK_DX])
          //Not enough bullets... Trying to balance the two ref2illing station
          {
            feeder_brake();
            chThdSleepMilliseconds(10);
            feeder_refill(FEEDER_CW, 100);
            refill_start_time[TANK_SX] = curr_time;
            refiller_state = REFILLER_FEEDER_CW;
          }
        break;
      }

      // Check if the robots are present, if they are the timer is not running start counting
  		// TODO: implement using chVTGetSystemTime() at the moment this was more reliable due to some bugs
  		if (distance[TANK_SX] >= SET_DISTANCE){
  			// If the robot is not present anymore (fake positive) reset the timer
  			robot_present_time[TANK_SX] = curr_time;
  		}

      if (distance[TANK_DX] >= SET_DISTANCE){
  			// If the robot is not present anymore (fake positive) reset the timer
  			robot_present_time[TANK_DX] = curr_time;
  		}
/*
  		// If the robot has been present for more than 4 seconds stop the motor and refill
  		if (curr_time - robot_present_time[TANK_SX] > MS2ST(1000) && !door_open[TANK_SX])
      {
  			// Actuate the Servo Motor to open the tank for 3 seconds, then close
  			open_tank(TANK_SX);					//TODO: write Servo function
        door_open_time[TANK_SX] = curr_time;
        door_open[TANK_SX] = true;
  		}
      else if(door_open[TANK_SX] && curr_time - door_open_time[TANK_SX] > S2ST(3))
      {
        close_tank(TANK_SX);					//TODO: write Servo function
        bullet_count[TANK_SX] = 0;
        door_open[TANK_SX] = false;
      }

  		// If the robot has been present for more than 4 seconds refill
  		if (curr_time - robot_present_time[TANK_DX] > MS2ST(1000))
      {
  			// Actuate the Servo Motor to open the tank for 3 seconds, then close
  			open_tank(TANK_DX);				//TODO: write Servo function
        door_open_time[TANK_DX] = curr_time;
  			door_open[TANK_DX] = true;
  		}
      else if(door_open[TANK_DX] && curr_time - door_open_time[TANK_DX] > S2ST(3))
      {
        close_tank(TANK_DX);					//TODO: write Servo function
        bullet_count[TANK_DX] = 0;
        door_open[TANK_DX] = false;
      }
*/
      chThdSleepMilliseconds(10);
    }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  shellStart();
  params_init();
  extiinit();

  can_processInit();
  RC_init();
  rangeFinder_init();

	sdlog_init();
  feeder_init();

  chThdCreateStatic(refiller_control_wa, sizeof(refiller_control_wa), NORMALPRIO,
                    refiller_control, NULL);

	while (true)
  {
		chThdSleepMilliseconds(100);
  }

  return 0;
}

// ====================================================================================================
// REFILLER FUNCTIONS: BEGIN
// ====================================================================================================

// Function for opening the sub tank using the servo motor TODO: implement with the new servos
void open_tank (int tank){
	if (tank) LEDG_ON();
	else LEDR_ON();
}

// Function for opening the sub tank using the servo motor TODO: implement with the new servos
void close_tank (int tank){
	if (tank) LEDG_OFF();
	else LEDR_OFF();
}

// ====================================================================================================
// REFILLER FUNCTIONS: END
// ====================================================================================================
