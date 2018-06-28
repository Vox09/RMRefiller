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

typedef enum{
    REFILLER_IDLE = 0,
    REFILLER_FEEDER_R,
    REFILLER_FEEDER_L,
    REFILLER_FEEDER_B
} refiller_state_t;

static refiller_state_t refiller_state;
volatile uint16_t bullet_count[2] = {0, 0};// changed in exti.c

static THD_WORKING_AREA(refiller_control_wa, 512);
static THD_FUNCTION(refiller_control, p){
    (void) p;
    chRegSetThreadName("refiller_control_right");

    bool      door_open[2]    = {false, false};
    bool      finished[2]     = {false, false};


    float     distance[2];
    systime_t robot_present_time[2];
#ifdef DEBUG
    param_t debugController[4];
    static const DEBUG_CONTROLLER="DEBUG_CONTROLLER";
    static const char subname_debug_controller[]="RF LF RT LT";
    params_set(debugController,10,4,DEBUG_CONTROLLER,subname_debug_controller,PARAM_PUBLIC);
#endif
    while(!chThdShouldTerminateX())
    {

        systime_t curr_time = chVTGetSystemTimeX();
#ifndef DEBUG
        // Decide which tank needs to be refilled first, prioritizing the right one (easier to access during the game)
        // Decide refiller status
        if(bullet_count[RIGHT] >= BULLETS_MAX && bullet_count[LEFT] >= BULLETS_MAX)
            refiller_state = REFILLER_IDLE;
        else if(bullet_count[RIGHT] < BULLETS_MAX && bullet_count[LEFT] < BULLETS_MAX)
            refiller_state = REFILLER_FEEDER_B;
        else if(bullet_count[RIGHT] < BULLETS_MAX && bullet_count[LEFT] >= BULLETS_MAX)
            refiller_state = REFILLER_FEEDER_R;
        else if(bullet_count[RIGHT] >= BULLETS_MAX && bullet_count[LEFT] < BULLETS_MAX)
            refiller_state = REFILLER_FEEDER_L;
        // Do the staff
        switch (refiller_state) {
            case REFILLER_IDLE:
                RIGHT_FULL();
                LEFT_FULL();
                feeder_brake(RIGHT);
                feeder_brake(LEFT);
                break;
            case REFILLER_FEEDER_B:
                RIGHT_EMPTY();
                LEFT_EMPTY();
                feeder_refill(RIGHT);
                feeder_refill(LEFT);
                break;
            case REFILLER_FEEDER_R:
                RIGHT_EMPTY();
                LEFT_FULL();
                feeder_refill(RIGHT);
                feeder_brake(LEFT);
                break;
            case REFILLER_FEEDER_L:
                RIGHT_FULL();
                LEFT_EMPTY();
                feeder_brake(RIGHT);
                feeder_refill(LEFT);
                break;
        }
        // Calculate the distance from rangefinder sensors on both sides of the Refiller
        distance[RIGHT] = rangeFinder_getDistance(RANGEFINDER_INDEX_0);
        distance[LEFT] = rangeFinder_getDistance(RANGEFINDER_INDEX_1);

        // Should not use chThdSleepMilliseconds()
        //RIGHT
        if (distance[RIGHT] <= SET_DISTANCE &&
            LS_R1_DOWN() && LS_R2_DOWN()
            && !door_open[RIGHT] // avoid repeated opening door
            && !finished[RIGHT]) // ensure when robots present only done once
        {
            robot_present_time[RIGHT] = curr_time;
            open_tank(RIGHT);
            door_open[RIGHT] = true;
        }
        // close tank after DOOR_DELAY time
        if ( door_open[RIGHT] &&
             curr_time > robot_present_time[RIGHT] + MS2ST(DOOR_DELAY))
        {
            close_tank(RIGHT);
            door_open[RIGHT] = false;
            bullet_count[RIGHT] = 0;
            finished[RIGHT] = true;
        }
        // reset finished status after robot leave
        if(finished[RIGHT] && distance[RIGHT]> SET_DISTANCE)
            finished[RIGHT] = false;
        //LEFT
        if (distance[LEFT] <= SET_DISTANCE &&
            LS_L1_DOWN() && LS_L2_DOWN()
            && !door_open[LEFT] // avoid repeated opening door
            && !finished[LEFT]) // ensure when robots present only done once
        {
            robot_present_time[LEFT] = curr_time;
            open_tank(LEFT);
            door_open[LEFT] = true;
        }
        // close tank after DOOR_DELAY time
        if ( door_open[LEFT] &&
             curr_time > robot_present_time[LEFT] + MS2ST(DOOR_DELAY))
        {
            close_tank(LEFT);
            door_open[LEFT] = false;
            bullet_count[LEFT] = 0;
            finished[LEFT] = true;
        }
        // reset finished status after robot leave
        if(finished[LEFT] && distance[LEFT]> SET_DISTANCE)
            finished[LEFT] = false;

#else
        if(debugController[0]>0)//RF
            feeder_refill(RIGHT);
        else feeder_brake(RIGHT);
        if(debugController[1]>0)//LF
            feeder_refill(LEFT);
        else feeder_brake(LEFT);
        if(debugController[2]>0)//RT
            open_tank(RIGHT);
        else close_tank(RIGHT);
        if(debugController[3]>0)//LT
            open_tank(LEFT);
        else close_tank(LEFT);

#endif
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
    servosInit();

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
