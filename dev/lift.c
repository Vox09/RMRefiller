/*
 * File:   lift.c
 * Author: Xu Xinyuan
 * Date:   2018-07-03
 */

#include <inc/canBusProcess.h>
#include "canBusProcess.h"
#include "math_misc.h"
#include "params.h"
#include "feeder.h"
#include "rangefinder.h"

#include "lift.h"

static int16_t      lift_output;
static int16_t      door_output;
static lift_mode_t  lift_state;
static door_mode_t  door_state;
static float        lift_limit_pos[2] = {0.0f,0.0f};     //
static float        setpoint[2][2] = {{0.0f,0.0f},{0.0f,0.0f}};
static float        hero_lift_sp = 0.0f;

uint16_t engineer_come_count=0;
uint16_t hero_come_count=0;
uint16_t hero_stuck_count=0;
static systime_t hero_present_time;
static systime_t engineer_present_time;
static bool isEng = false;
static bool isHero = false;
static bool lift_success_down = false;
static bool fuck_hero = false;

static volatile ChassisEncoder_canStruct* lift_encoder;
static lpfilterStruct lp_spd_lift[2];


pid_struct  lift_vel_pid[2]/*= {0, 0, 0, 0}*/;
pid_struct  lift_limit_pos_pid[2] /*= {5.0, 0, 0, 0, 0}*/;

uint16_t init_error_count[2] = {0,0};

void lift_canUpdate()
{
    can_motorSetCurrent(FEEDER_CAN, LIFT_CAN_EID,
                        lift_output,door_output,0,0);
}

static int16_t lift_controlVel(uint8_t i, float target, const float output_max){
    static float last_error[2];
    static float current_error[2];

    last_error[i] = current_error[i];
    int16_t current_speed = lift_encoder[i].raw_speed;
    current_speed = lpfilter_apply(&(lp_spd_lift[i]), current_speed);
    current_error[i] = target - (float) current_speed;
    lift_vel_pid[i].inte += current_error[i] * lift_vel_pid[i].ki;
    lift_vel_pid[i].inte = lift_vel_pid[i].inte > lift_vel_pid[i].inte_max?  lift_vel_pid[i].inte_max:lift_vel_pid[i].inte;
    lift_vel_pid[i].inte = lift_vel_pid[i].inte <-lift_vel_pid[i].inte_max? -lift_vel_pid[i].inte_max:lift_vel_pid[i].inte;

    float output = lift_vel_pid[i].kp * current_error[i] + lift_vel_pid[i].inte + lift_vel_pid[i].kd * (current_error[i] - last_error[i]);
    output = output > output_max?  output_max:output;
    output = output <-output_max? -output_max:output;

    return (int16_t) output;

}
static int16_t lift_controlPos(uint8_t i, float target, const float output_max){

    float error = target - (float)lift_encoder[i].total_ecd;

    lift_limit_pos_pid[i].inte += error * lift_limit_pos_pid[i].ki;
    lift_limit_pos_pid[i].inte = lift_limit_pos_pid[i].inte > lift_limit_pos_pid[i].inte_max?  lift_limit_pos_pid[i].inte_max:lift_limit_pos_pid[i].inte;
    lift_limit_pos_pid[i].inte = lift_limit_pos_pid[i].inte <-lift_limit_pos_pid[i].inte_max? -lift_limit_pos_pid[i].inte_max:lift_limit_pos_pid[i].inte;

    float speed_sp = lift_limit_pos_pid[i].kp * error +
                     lift_limit_pos_pid[i].inte -
                     lift_limit_pos_pid[i].kd * lift_encoder[i].raw_speed;

    return lift_controlVel(i,speed_sp, output_max);
}


static void lift_func(){
    switch(lift_state) {
        case LIFT_INITIALING_DOWN:
            if(lift_encoder[LIFT].total_ecd > lift_limit_pos[DOWN] - 10* CAN_ENCODER_RANGE)
                lift_output = - 3*LIFT_INIT_DOWN_SPEED;
            else
                lift_output = - LIFT_INIT_DOWN_SPEED;
            //top detecting
            if (state_count((lift_encoder[LIFT].total_ecd < -CAN_ENCODER_RANGE)&& //at least one round
                            (lift_encoder[LIFT].raw_speed < 30) &&
                            (lift_encoder[LIFT].raw_speed > -30),
                            LIFT_INIT_COUNT,&init_error_count[LIFT]))
            {
                lift_limit_pos[DOWN] = lift_encoder[LIFT].total_ecd;
                lift_limit_pos[UP] = lift_limit_pos[DOWN] + LIFT_ENCODER;
                setpoint[LIFT][DOWN] = lift_limit_pos[DOWN]+
                                       (lift_limit_pos[UP]-lift_limit_pos[DOWN])
                                       *LIFT_DOWN_LENGTH/LIFT_TOTAL_LENGTH;
                setpoint[LIFT][UP] = lift_limit_pos[DOWN]+
                                     (lift_limit_pos[UP]-lift_limit_pos[DOWN])
                                     *LIFT_UP_LENGTH/LIFT_TOTAL_LENGTH;
                lift_state = LIFT_UP;
            }
            break;
/*        case LIFT_INITIALING_UP:
            if(lift_encoder[LIFT].total_ecd < lift_limit_pos[DOWN] + 10 * CAN_ENCODER_RANGE)
            lift_output = (int)3*LIFT_INIT_UP_SPEED;//times 2 in case stuck at top
            else
                lift_output = LIFT_INIT_UP_SPEED;
            //bottom detecting
            if (state_count((lift_encoder[LIFT].total_ecd > lift_limit_pos[DOWN]+10*CAN_ENCODER_RANGE)&& //at least ten rounds
                            (lift_encoder[LIFT].raw_speed < 20) &&
                            (lift_encoder[LIFT].raw_speed > -20),
                            LIFT_INIT_COUNT,&init_error_count[LIFT]))
            {
                lift_limit_pos[UP] = lift_encoder[LIFT].total_ecd;
                //set set points in total_ecd
                setpoint[LIFT][DOWN] = lift_limit_pos[DOWN]+
                                (lift_limit_pos[UP]-lift_limit_pos[DOWN])
                                *LIFT_DOWN_LENGTH/LIFT_TOTAL_LENGTH;
                setpoint[LIFT][UP] = lift_limit_pos[DOWN]+
                              (lift_limit_pos[UP]-lift_limit_pos[DOWN])
                              *LIFT_UP_LENGTH/LIFT_TOTAL_LENGTH;
                hero_lift_sp = (setpoint[LIFT][DOWN]+setpoint[LIFT][UP])/2;
                lift_state = LIFT_UP;
            }
            break;*/
        case LIFT_DOWN:
            lift_output = lift_controlPos(LIFT,setpoint[LIFT][DOWN],LIFT_OUT_MAX);
            break;
        case LIFT_UP:
            lift_output = lift_controlPos(LIFT,setpoint[LIFT][UP],LIFT_OUT_MAX);
            break;
        case LIFT_SLOW_DOWN:
            if(lift_encoder[LIFT].total_ecd > lift_limit_pos[UP] - 10* CAN_ENCODER_RANGE)
                lift_output = - 3*LIFT_INIT_DOWN_SPEED;
            else lift_output = - LIFT_INIT_DOWN_SPEED;
            break;

    }
}

//static int init_pos = -DOOR_INIT_STEP;
static void door_func(){
    switch(door_state) {
//        case DOOR_INITIALING_OPEN:
//            door_output = -800;
//            //top detecting
//            if (state_count((lift_encoder[DOOR].total_ecd < -CAN_ENCODER_RANGE)&& //at least one round
//                                                                                   (lift_encoder[DOOR].raw_speed < 20) &&
//                                                                                   (lift_encoder[DOOR].raw_speed > -20),
//                             DOOR_INIT_COUNT,&init_error_count[DOOR])) {
//                door_output = lift_controlPos(DOOR, init_pos, DOOR_OUT_MAX);
//                init_pos -= DOOR_INIT_STEP;
//            }
//            else{
//                setpoint[DOOR][OPEN] = lift_encoder[DOOR].total_ecd;
//                setpoint[DOOR][CLOSE] = lift_encoder[DOOR].total_ecd+CLOSE_DOOR_ENCODER;
//                door_state = DOOR_CLOSE;
//            }
//            break;
        case DOOR_INITIALING_CLOSE:
            door_output = DOOR_INIT_SPEED;
            //bottom detecting
            if (state_count((lift_encoder[DOOR].raw_speed < 40) &&
                            (lift_encoder[DOOR].raw_speed > -40),
                            DOOR_INIT_COUNT,&init_error_count[DOOR]))
            {
                setpoint[DOOR][CLOSE] = lift_encoder[DOOR].total_ecd;
                setpoint[DOOR][OPEN] = setpoint[DOOR][CLOSE] - CLOSE_DOOR_ENCODER;
                door_state = DOOR_CLOSE;
            }
            break;
        case DOOR_CLOSE:
            door_output = lift_controlPos(DOOR,setpoint[DOOR][CLOSE],DOOR_OUT_MAX);
            break;
        case DOOR_OPEN:
            door_output = lift_controlPos(DOOR,setpoint[DOOR][OPEN],DOOR_OUT_MAX);
            break;
    }
}

bool stuck_hero(){
    return (lift_state == LIFT_SLOW_DOWN
       && state_count(lift_encoder[LIFT].raw_speed > -10
                      && lift_encoder[LIFT].raw_speed < 10
            ,HERO_STUCK_COUNT,&hero_stuck_count));
}


static void control_func() {
    // check if big car come
    // !hero to prevent conflict when both sensors was triggered
    if (!isEng && !isHero) {
        if (state_count(rangeFinder_getDistance(LEAVE_SENSOR) < SET_ENGINEER_DISTANCE, ENGINEER_COUNT,
                        &engineer_come_count)) {
            isEng = true;
            lift_go_down();
            engineer_present_time = chVTGetSystemTimeX();
            MIDDLE_PROCESSING();
        }
        if (state_count(rangeFinder_getDistance(COME_SENSOR) < SET_HERO_DISTANCE, HERO_COUNT, &hero_come_count)
                && chVTGetSystemTimeX() > hero_present_time + S2ST(8)) {
            isHero = true;
            lift_state = LIFT_SLOW_DOWN;
            MIDDLE_PROCESSING();
        }
    } else {
        if (isEng && lift_done_down() && chVTGetSystemTimeX() > engineer_present_time + S2ST(10)) {
            lift_go_up();
            isEng = false;
            MIDDLE_IDLE();
        }
        if (isHero ) {
            if (lift_encoder[LIFT].total_ecd <
                lift_limit_pos[DOWN] + 0.8 * (lift_limit_pos[UP] - lift_limit_pos[DOWN])) {
                lift_go_up();
                isHero = false;
                MIDDLE_IDLE();
            }
            if (stuck_hero()) {
                lift_go_up();
                hero_present_time = chVTGetSystemTimeX();
                fuck_hero = true;
            }
            if (fuck_hero && chVTGetSystemTimeX() > hero_present_time + MS2ST(500))
                open_mid_door();
            if (fuck_hero && door_done_open()
                && chVTGetSystemTimeX() > hero_present_time + MS2ST(3000)) {
                close_mid_door();
                fuck_hero = false;
                isHero = false;
                // in case both sensors was triggered and clear flag
                isEng = false;
                //clear LED
                MIDDLE_IDLE();
            }
        }
    }


/*
    if(!big_car_come) {
        if (!wait_path&&
                state_count((rangeFinder_getDistance(COME_SENSOR) < SET_COME_DISTANCE), COME_COUNT, &big_car_count)
            && chVTGetSystemTimeX() > hero_present_time + MS2ST(6000)) {
            wait_path_time = chVTGetSystemTimeX();
            wait_path = true;
            // set LED up
            MIDDLE_PROCESSING();
        }
        // delay to wait path
        if (chVTGetSystemTimeX() > ) {
            big_car_come = true;
            wait_path = false;
            lift_state = LIFT_SLOW_DOWN;
        }
    }
    // big car present
    else{
        // hero
        if(stuck_hero()){
            lift_go_up();
            hero_present_time = chVTGetSystemTimeX();
            fuck_hero = true;
        }
        if(fuck_hero && chVTGetSystemTimeX() > hero_present_time + MS2ST(500))
            open_mid_door();
        if(fuck_hero && door_done_open()
                && chVTGetSystemTimeX()> hero_present_time + MS2ST(3000)){
            close_mid_door();
            big_car_come = false;
            fuck_hero = false;
            //clear LED
            MIDDLE_IDLE();
        }
        // after some distance accelerate
        if(!lift_success_down
           &&lift_encoder[LIFT].total_ecd <
                lift_limit_pos[DOWN]+0.8*(lift_limit_pos[UP]-lift_limit_pos[DOWN]))
            lift_go_down();
        // engineer
        // record first down time
        if(lift_done_down() && !lift_success_down){
            wait_since_time = chVTGetSystemTimeX();
            lift_success_down = true;
        }
        if(lift_success_down) {
            // if not leave update wait since time
            if (rangeFinder_getDistance(LEAVE_SENSOR) < SET_LEAVE_DISTANCE) {
                MIDDLE_PROCESSING();
                wait_since_time = chVTGetSystemTimeX();
            } else {
                MIDDLE_IDLE();
            }
            // wait and go up
            if (chVTGetSystemTimeX() > wait_since_time + MS2ST(LIFT_WAIT_MS)) {
                lift_go_up();
                //clear LED
                MIDDLE_IDLE();
            }
            if(lift_done_up()) {
                big_car_come = false;
                lift_success_down = false;
                MIDDLE_IDLE();
            }
        }
    }
*/

}

static THD_WORKING_AREA(lift_control_wa, 512);
static THD_FUNCTION(lift_control,p){
    (void) p;
    chRegSetThreadName("lift_controller");
    while(!chThdShouldTerminateX())
    {
        control_func();
        lift_func();
        door_func();
        lift_canUpdate();
        chThdSleepMilliseconds(1);
    }
}
static const LIFT_VEL  = "LIFT_VEL_R";
static const LIFT_POS  = "LIFT_POS_R";
static const DOOR_VEL = "DOOR_VEL_R";
static const DOOR_POS  = "DOOR_POS_R";
static const char subname_feeder_PID[] = "KP KI KD Imax";
//static const TMP = "TMP";
//param_t tmp_debug;

void lift_init(void)
{
    lift_encoder = can_getLiftMotor();
    lift_state = LIFT_INITIALING_DOWN;
    door_state = DOOR_INITIALING_CLOSE;
    params_set(&lift_vel_pid[LIFT],7,4,LIFT_VEL,subname_feeder_PID,PARAM_PUBLIC);
    params_set(&lift_limit_pos_pid[LIFT],8,4,LIFT_POS,subname_feeder_PID,PARAM_PUBLIC);
    params_set(&lift_vel_pid[DOOR],9,4,DOOR_VEL,subname_feeder_PID,PARAM_PUBLIC);
    params_set(&lift_limit_pos_pid[DOOR],10,4,DOOR_POS,subname_feeder_PID,PARAM_PUBLIC);
//    params_set(&tmp_debug,11,1,TMP,subname_feeder_PID,PARAM_PUBLIC);
    lpfilter_init(&lp_spd_lift[LIFT],1000,30);
    lpfilter_init(&lp_spd_lift[DOOR],1000,30);

    lift_limit_pos[DOWN] = lift_encoder[LIFT].total_ecd;
    MIDDLE_IDLE();
    chThdCreateStatic(lift_control_wa, sizeof(lift_control_wa),
                      NORMALPRIO, lift_control, NULL);
}

// public interface
// ========================
void lift_go_down(){if (lift_state == LIFT_UP || lift_state == LIFT_SLOW_DOWN)lift_state = LIFT_DOWN;}

void lift_go_up(){if (lift_state == LIFT_DOWN || lift_state == LIFT_SLOW_DOWN) lift_state = LIFT_UP;}

//void lift_hold(){
//    if (lift_state != LIFT_HOLD) {
//        hero_lift_sp = lift_encoder[LIFT].total_ecd;
//        lift_state = LIFT_HOLD;
//    }
//}


void open_mid_door(){
    if (door_state == DOOR_CLOSE) {
        door_state = DOOR_OPEN;
    }
}

void close_mid_door(){
    if (door_state == DOOR_OPEN) {
        door_state = DOOR_CLOSE;
    }
}
//int get_lift_state() {return lift_state; }

bool lift_done_up(){
    return(lift_state == LIFT_UP &&
       lift_encoder[LIFT].total_ecd > setpoint[LIFT][UP]-20*CAN_ENCODER_RANGE &&
       lift_encoder[LIFT].total_ecd < setpoint[LIFT][UP]+20*CAN_ENCODER_RANGE);
}
bool lift_done_down(){
    return(lift_state == LIFT_DOWN &&
       lift_encoder[LIFT].total_ecd > setpoint[LIFT][DOWN]-20*CAN_ENCODER_RANGE &&
       lift_encoder[LIFT].total_ecd < setpoint[LIFT][DOWN]+20*CAN_ENCODER_RANGE);
}

bool door_done_open(){
    return(door_state == DOOR_OPEN &&
       lift_encoder[DOOR].total_ecd > setpoint[DOOR][OPEN]-30*CAN_ENCODER_RANGE &&
       lift_encoder[DOOR].total_ecd < setpoint[DOOR][OPEN]+30*CAN_ENCODER_RANGE);
}
bool door_done_close(){
    return(door_state == DOOR_CLOSE &&
       lift_encoder[DOOR].total_ecd > setpoint[DOOR][CLOSE]-30*CAN_ENCODER_RANGE &&
       lift_encoder[DOOR].total_ecd < setpoint[DOOR][CLOSE]+30*CAN_ENCODER_RANGE);
}


//debug
bool get_lift_success(){ return  lift_success_down;}
int8_t get_lift_state(){return lift_state;}
int8_t get_door_state(){return door_state;}
