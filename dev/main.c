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


static refiller_state_t refiller_state;
volatile uint16_t bullet_count[2] = {0, 0};// changed in exti.c

static uint8_t bullet_per_time = BULLETS_INIT;
static int8_t refill_count= 0;

static bool      distance_OK[2]  = {false, false};
static bool      door_open[2]    = {false, false};
static bool      finished[2]     = {false, false};

static float     distance[2];
static systime_t robot_present_time[2];

// Debug functions
bool* getDoor(){ return door_open;}
bool* getDistanceOK(){return distance_OK;}
bool* getFinished(){return finished;}

static THD_WORKING_AREA(refiller_control_wa, 1024);
static THD_FUNCTION(refiller_control, p){
    (void) p;
    chRegSetThreadName("refiller_control");


#ifdef DEBUG
    param_t debugController[6];
    static const DEBUG_CONTROLLER="DEBUG_CONTROLLER";
    static const char subname_debug_controller[]="RF LF RT LT LIFT DOOR";
    params_set(debugController,16,6,DEBUG_CONTROLLER,subname_debug_controller,PARAM_PUBLIC);
#endif
    while(!chThdShouldTerminateX())
    {

        systime_t curr_time = chVTGetSystemTimeX();
        if(refill_count>3) bullet_per_time= BULLETS_MAX;
#ifndef DEBUG
        // Decide which tank needs to be refilled first, prioritizing the right one (easier to access during the game)
        // Decide refiller status
        if(bullet_count[RIGHT] >= bullet_per_time && bullet_count[LEFT] >= bullet_per_time )
            refiller_state = REFILLER_IDLE;
        else if(bullet_count[RIGHT] < bullet_per_time && bullet_count[LEFT] < bullet_per_time)
            refiller_state = REFILLER_FEEDER_B;
        else if(bullet_count[RIGHT] < bullet_per_time && bullet_count[LEFT] >=bullet_per_time )
            refiller_state = REFILLER_FEEDER_R;
        else if(bullet_count[RIGHT] >= bullet_per_time && bullet_count[LEFT] < bullet_per_time)
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

        // TANK STUFF
        // Calculate the distance from rangefinder sensors on both sides of the Refiller
        distance[RIGHT] = rangeFinder_getDistance(RANGEFINDER_INDEX_0);
        distance[LEFT] = rangeFinder_getDistance(RANGEFINDER_INDEX_1);

        distance_OK[RIGHT] = (distance[RIGHT] <= SET_DISTANCE || distance[RIGHT]>1000.0);
        distance_OK[LEFT] = (distance[LEFT] <= SET_DISTANCE || distance[LEFT]>1000.0);
        // Should not use chThdSleepMilliseconds()
        //RIGHT
        if (distance_OK[RIGHT] &&
            LS_R1_DOWN() && LS_R2_DOWN()
            && !door_open[RIGHT] // avoid repeated opening door
            && !finished[RIGHT]) // ensure when robots present only done once
        {
            robot_present_time[RIGHT] = curr_time;
            open_tank(RIGHT);
            door_open[RIGHT] = true;
            refill_count++;
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
        if(finished[RIGHT] && !distance_OK[RIGHT])
            finished[RIGHT] = false;

        //LEFT
        if (distance_OK[LEFT] &&
            LS_L1_DOWN() && LS_L2_DOWN()
            && !door_open[LEFT] // avoid repeated opening door
            && !finished[LEFT]) // ensure when robots present only done once
        {
            robot_present_time[LEFT] = curr_time;
            open_tank(LEFT);
            door_open[LEFT] = true;
            refill_count++;
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
        if(finished[LEFT] && !distance_OK[LEFT])
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
//        if(debugController[4]>0)//LIFT
//            lift_go_up();
        //can_motorSetCurrent(FEEDER_CAN, LIFT_CAN_EID,(int16_t)debugController[5],0,0,0);
//        else
//            lift_go_down();
        //can_motorSetCurrent(FEEDER_CAN, LIFT_CAN_EID,-(int16_t)debugController[5],0,0,0);
        if(debugController[5]>0)//DOOR
            open_mid_door();
        else close_mid_door();

//        can_motorSetCurrent(FEEDER_CAN, LIFT_CAN_EID,0,(int16_t)debugController[5],0,0);
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
    lift_init();

    RIGHT_EMPTY();
    RIGHT_FINISH();
    LEFT_EMPTY();
    LEFT_FINISH();
    chThdCreateStatic(refiller_control_wa, sizeof(refiller_control_wa), NORMALPRIO+5,
                      refiller_control, NULL);

    while (true)
    {
        chThdSleepMilliseconds(100);
    }

    return 0;
}
