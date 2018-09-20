#include "ch.h"
#include "hal.h"

#include "params.h"
#include "canBusProcess.h"
#include "dbus.h"

#include "math_misc.h"

#include "feeder.h"

#define FEEDER_SPEED_SP_RPM     (float)(FEEDER_SET_BPS * FEEDER_GEAR * 60 /FEEDER_BULLET_PER_TURN)
#define FEEDER_TURNBACK_ANGLE   360.0f / FEEDER_BULLET_PER_TURN     //165.0f;

static int16_t        feeder_output[2];
int16_t *getFeederOutput(){
    return feeder_output;
}

static feeder_mode_t  feeder_state[2] = {FEEDER_STOP,FEEDER_STOP};
static float          feeder_brakePos[2] = {0.0f, 0.0f};
static systime_t      feeder_stop_time[2];

static volatile ChassisEncoder_canStruct*   feeder_encode;
RC_Ctl_t*                   p_dbus;
static lpfilterStruct lp_spd_feeder[2];

pid_struct  vel_pid[2] /*= {0, 0, 0, 0}*/;
pid_struct  pos_pid[2] /*= {5.0, 0, 0, 0, 0}*/;
pid_struct  rest_pid[2] /*= {0.45, 0, 0, 0, 0}*/;

uint16_t error_count[2] = {0,0};


void feeder_canUpdate()
{
        can_motorSetCurrent(FEEDER_CAN, FEEDER_CAN_EID,\
        feeder_output[RIGHT],feeder_output[LEFT], 0, 0);
}


void feeder_brake(uint8_t i)
{
    if(feeder_state[i] == FEEDER_AUTO_GO) {
        feeder_brakePos[i] = (float) feeder_encode[i].total_ecd;
        feeder_stop_time[i] = chVTGetSystemTimeX();
        feeder_state[i] = FEEDER_STOP;
    }
}

void feeder_refill(uint8_t i)
{
   if(feeder_state[i] == FEEDER_STOP)
    feeder_state[i] =  FEEDER_AUTO_GO;
}

static void feeder_rest(uint8_t i)
{
    int16_t current_speed = lpfilter_apply(&lp_spd_feeder[i], feeder_encode[i].raw_speed);
    float error = - (float) current_speed;
    rest_pid[i].inte += error * rest_pid[i].ki;
    rest_pid[i].inte = rest_pid[i].inte > rest_pid[i].inte_max?  rest_pid[i].inte_max:rest_pid[i].inte;
    rest_pid[i].inte = rest_pid[i].inte <-rest_pid[i].inte_max? -rest_pid[i].inte_max:rest_pid[i].inte;

    feeder_output[i] = rest_pid[i].kp * error + rest_pid[i].inte;
    feeder_output[i] = feeder_output[i] > 4000?  4000:feeder_output[i];
    feeder_output[i] = feeder_output[i] <-4000? -4000:feeder_output[i];
}

static int16_t feeder_controlVel(uint8_t i, float target, const float output_max){
    static float last_error[2];
    static float current_error[2];

    last_error[i] = current_error[i];
    int16_t current_speed = feeder_encode[i].raw_speed;
    current_speed = lpfilter_apply(&(lp_spd_feeder[i]), current_speed);
    current_error[i] = target - (float) current_speed;
    vel_pid[i].inte += current_error[i] * vel_pid[i].ki;
    vel_pid[i].inte = vel_pid[i].inte > vel_pid[i].inte_max?  vel_pid[i].inte_max:vel_pid[i].inte;
    vel_pid[i].inte = vel_pid[i].inte <-vel_pid[i].inte_max? -vel_pid[i].inte_max:vel_pid[i].inte;

    float output = vel_pid[i].kp * current_error[i] + vel_pid[i].inte + vel_pid[i].kd * (current_error[i] - last_error[i]);
    output = output > output_max?  output_max:output;
    output = output <-output_max? -output_max:output;

    return (int16_t) output;

}

static int16_t feeder_controlPos(uint8_t i, float target, const float output_max){

    float error = target - (float)feeder_encode[i].total_ecd;

    pos_pid[i].inte += error * pos_pid[i].ki;
    pos_pid[i].inte = pos_pid[i].inte > pos_pid[i].inte_max?  pos_pid[i].inte_max:pos_pid[i].inte;
    pos_pid[i].inte = pos_pid[i].inte <-pos_pid[i].inte_max? -pos_pid[i].inte_max:pos_pid[i].inte;

    float speed_sp = pos_pid[i].kp * error +
                     pos_pid[i].inte -
                     pos_pid[i].kd * feeder_encode[i].raw_speed;

    return feeder_controlVel(i,speed_sp, output_max);
}

static void feeder_func(uint8_t i){
    switch (feeder_state[i]){
        case FEEDER_STOP:
            if(chVTGetSystemTimeX() > feeder_stop_time[i] + S2ST(1))
              feeder_rest(i);
            else
              feeder_output[i] = feeder_controlPos(i,feeder_brakePos[i], FEEDER_OUTPUT_MAX);

            break;
        case FEEDER_GO:
        case FEEDER_AUTO_GO:
            //error detecting
            if (
                 state_count((feeder_encode[i].raw_speed < 30) &&
                             (feeder_encode[i].raw_speed > -30),
                 FEEDER_ERROR_COUNT, &(error_count[i]))
               )
            {
              float error_angle_sp = feeder_encode[i].total_ecd -
                FEEDER_TURNBACK_ANGLE / 360.0f * FEEDER_GEAR * 8192;
              systime_t error_start_time = chVTGetSystemTime();
              while (chVTIsSystemTimeWithin(error_start_time, (error_start_time + MS2ST(200))))
              {
                feeder_output[i] = feeder_controlPos(i,error_angle_sp, FEEDER_OUTPUT_MAX_BACK);
                feeder_canUpdate();
                chThdSleepMilliseconds(1);
              }
            }

            feeder_output[i] = feeder_controlVel(i,FEEDER_SPEED_SP_RPM, FEEDER_OUTPUT_MAX);
            break;
        default:
            feeder_output[i] = 0;
            break;
    }
}


static THD_WORKING_AREA(feeder_control_wa,512);
static THD_FUNCTION(feeder_control, p){
    (void) p;
    chRegSetThreadName("feeder controller");
    while(!chThdShouldTerminateX())
    {
        feeder_func(RIGHT);
        feeder_func(LEFT);
        feeder_canUpdate();
        chThdSleepMilliseconds(1);
    }
}


//static THD_WORKING_AREA(feeder_control_L_wa, 512);
//static THD_FUNCTION(feeder_control_L, p){
//    (void) p;
//    chRegSetThreadName("feeder controller_L");
//    while(!chThdShouldTerminateX())
//    {
//        feeder_func(LEFT);
//        chThdSleepMilliseconds(1);
//    }
//}

static const FEEDER_VEL_R  = "FEEDER_VEL_R";
static const FEEDER_POS_R  = "FEEDER_POS_R";
static const FEEDER_rest_name_R = "FEEDER_REST_R";
static const FEEDER_VEL_L  = "FEEDER_VEL_L";
static const FEEDER_POS_L  = "FEEDER_POS_L";
static const FEEDER_rest_name_L = "FEEDER_REST_L";
static const char subname_feeder_PID[] = "KP KI KD Imax";
void feeder_init(void)
{

    feeder_encode = can_getFeederMotor();
    p_dbus = RC_get();

    params_set(&vel_pid[RIGHT], 1,4,FEEDER_VEL_R,subname_feeder_PID,PARAM_PUBLIC);
    params_set(&pos_pid[RIGHT], 2,4,FEEDER_POS_R,subname_feeder_PID,PARAM_PUBLIC);
    params_set(&rest_pid[RIGHT],3,4,FEEDER_rest_name_R,subname_feeder_PID,PARAM_PUBLIC);

    params_set(&vel_pid[LEFT], 4,4,FEEDER_VEL_L,subname_feeder_PID,PARAM_PUBLIC);
    params_set(&pos_pid[LEFT], 5,4,FEEDER_POS_L,subname_feeder_PID,PARAM_PUBLIC);
    params_set(&rest_pid[LEFT],6,4,FEEDER_rest_name_L,subname_feeder_PID,PARAM_PUBLIC);

    lpfilter_init(&(lp_spd_feeder[RIGHT]), 1000, 30);
    lpfilter_init(&(lp_spd_feeder[LEFT]), 1000, 30);
    feeder_brakePos[RIGHT] = (float)feeder_encode[RIGHT].total_ecd;
    feeder_brakePos[LEFT] = (float)feeder_encode[LEFT].total_ecd;

    chThdCreateStatic(feeder_control_wa, sizeof(feeder_control_wa),
                      NORMALPRIO - 5, feeder_control, NULL);
//    chThdCreateStatic(feeder_control_L_wa, sizeof(feeder_control_L_wa),
//                      NORMALPRIO - 5, feeder_control_L, NULL);

}

