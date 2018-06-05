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

	// ====================================================================================================
	// REFILLER SETUP: BEGIN
	// ====================================================================================================

	// VARIABLES
	int feeder_speed = 0;
	int distance[2] = {0, 0};
	int bullets [2] = {0, 0};
	int bullet_count [2] = {0, 0};
	uint32_t timer [2] = {0, 0};
	int error_count;

  ChassisEncoder_canStruct* feeder_encoder;
	feeder_encoder = can_getChassisMotor();

	// FUNCTIONS
	void feeder_write(int val);
	void open_tank (int tank);
	void close_tank (int tank);

	// ====================================================================================================
	// REFILLER SETUP: END
	// ====================================================================================================

	while (true)
  {
		//LEDR_TOGGLE();	// Blink red LED for testing

		// ====================================================================================================
		// REFILLER FLOW DIAGRAM: BEGIN TODO implement hardware for the counting functions and Servo motors
		// ====================================================================================================

		// Calculate the distance from rangefinder sensors on both sides of the Refiller
		distance [TANK_SX] = rangeFinder_getDistance(RANGEFINDER_INDEX_1);
		distance [TANK_DX] = rangeFinder_getDistance(RANGEFINDER_INDEX_0);
		// The chVTGetSystemTimeX() workaround is needed since on startup the position is 1000 times the
		// correct one (WARMUP is empirical) TODO: solve the rangefinder bug

		// Set up counting routine, (BULLETS_PER_CYLE is empirical) TODO set up optical/contact switch for precision
		bullet_count[TANK_SX] += (feeder_speed < 0);
		bullet_count[TANK_DX] += (feeder_speed > 0);
		bullets[TANK_SX] = bullet_count[TANK_SX]/BULLETS_PER_CYCLE;
		bullets[TANK_DX] = bullet_count[TANK_DX]/BULLETS_PER_CYCLE;

		// Decide which tank needs to be refilled first, prioritizing the right one (easier to access during the game)
		if (bullets[TANK_DX] < BULLETS_MAX)
			feeder_speed = FEEDER_MOTOR_SPEED;
		else if (bullets[TANK_SX] < BULLETS_MAX)
			feeder_speed = -FEEDER_MOTOR_SPEED;
		else
			feeder_speed = 0;

		//feeder_speed = 1200*(distance[1]<=5) - 1200*(distance[0]<=5); 	//Only for testing purpose

		// Actuate the motor with the correct speed
		feeder_write (feeder_speed);

		// Error detection, reverses direction for 0.2 seconds if the motor gets stuck
		if ( feeder_speed != 0 ) {
			if (feeder_speed*feeder_speed - feeder_encoder[0].raw_speed*feeder_encoder[0].raw_speed > feeder_speed*feeder_speed*0.7f) {
				error_count++;
				if (error_count > 90) {
					LEDG_TOGGLE();
					systime_t error_start_time = chVTGetSystemTime();
					while ( chVTIsSystemTimeWithin(error_start_time, (error_start_time + MS2ST(200))) ){
						feeder_write (-feeder_speed);
						chThdSleepMilliseconds(1);
					}
					error_count = 0;
				}
			}
		}

		// Check if the robots are present, if they are the timer is not running start counting
		// TODO: implement using chVTGetSystemTime() at the moment this was more reliable due to some bugs
		if (distance[TANK_SX] < SET_DISTANCE){
			timer[TANK_SX]++;
		} else if (distance[TANK_SX] >= SET_DISTANCE){
			// If the robot is not present anymore (fake positive) reset the timer
			timer[TANK_SX] = 0;
		}
		if (distance[TANK_DX] < SET_DISTANCE){
			timer[TANK_DX]++;
		} else if (distance[TANK_DX] >= SET_DISTANCE){
			// If the robot is not present anymore (fake positive) reset the timer
			timer[TANK_DX] = 0;
		}

    int i;
		// If the robot has been present for more than 4 seconds stop the motor and refill
		if (timer[TANK_SX] > 400){
			// Stop the motor if it is running to avoid wasting bullets
			// Need to send the message more than once for it to take action immediately

			for (i = 0; i < 10; i++)
				feeder_write (0);

			// Actuate the Servo Motor to open the tank for 3 seconds, then close
			open_tank (TANK_SX);					//TODO: write Servo function
			chThdSleepSeconds(3);
			close_tank (TANK_SX);					//TODO: write Servo function

			// Reset the bullet counter and timer for the tank
			bullet_count[TANK_SX] = 0;
			timer [TANK_SX] = 0;
		}

		// If the robot has been present for more than 4 seconds refill
		if (timer[TANK_DX] > 400){
			// Stop the motor if it is running to avoid wasting balls
			for (i = 0; i < 10; i++)
				feeder_write (0);
			chThdSleepMilliseconds(100);

			// Actuate the Servo Motor to open the tank for 3 seconds, then close
			open_tank (TANK_DX);				//TODO: write Servo function
			chThdSleepSeconds(3);
			close_tank (TANK_DX);				//TODO: write Servo function

			// Reset the bullet counter and timer for the tank
			bullet_count[TANK_DX] = 0;
			timer [TANK_DX] = 0;
		}


		chThdSleepMilliseconds(10);
  }

	// ====================================================================================================
	// REFILLER FLOW DIAGRAM: END
	// ====================================================================================================

  return 0;
}

// ====================================================================================================
// REFILLER FUNCTIONS: BEGIN
// ====================================================================================================

// Redefine function name for code readability (the Refiller uses only one motor, instead of 4)
void feeder_write(int val){
	can_motorSetCurrent(&CAND1, 0x200, val, 0, 0, 0);
}

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
