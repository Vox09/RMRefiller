#include "ch.h"
#include "hal.h"

#include "params.h"
#include "canBusProcess.h"
#include "dbus.h"

#include "math_misc.h"

#include "feeder.h"

#define FEEDER_SPEED_SP_RPM     (float)(FEEDER_SET_RPS * FEEDER_GEAR * 60 / FEEDER_BULLET_PER_TURN)
#define FEEDER_TURNBACK_ANGLE   360.0f / FEEDER_BULLET_PER_TURN     //165.0f;

static int16_t        feeder_output;

static feeder_mode_t  feeder_state = FEEDER_STOP;
static float          feeder_brakePos = 0.0f;
static systime_t      feeder_start_time;
static systime_t      bullet_out_time;
static systime_t      feeder_stop_time;

static thread_reference_t refiller_thread = NULL; //Used for refiller

ChassisEncoder_canStruct*   feeder_encode;
RC_Ctl_t*                   p_dbus;
static lpfilterStruct lp_spd_feeder;

pid_struct  vel_pid /*= {0, 0, 0, 0}*/;
pid_struct  pos_pid /*= {5.0, 0, 0, 0, 0}*/;
pid_struct  rest_pid /*= {0.45, 0, 0, 0, 0}*/;

uint16_t error_count = 0;

static uint32_t bulletCount[2];
static uint32_t bulletCount_stop[2];

int16_t feeder_canUpdate(void)
{
  #if (FEEDER_CAN_EID == 0x1FF)
    return feeder_output;
  #elif (FEEDER_CAN_EID == 0x200)
    can_motorSetCurrent(FEEDER_CAN, FEEDER_CAN_EID,\
        feeder_output, 0, 0, 0);
  #endif
}

void feeder_brake(void)
{
  feeder_brakePos = (float)feeder_encode->total_ecd;
  feeder_stop_time = chVTGetSystemTimeX();
  feeder_state = FEEDER_FINISHED;
}

/*
 *  @brief            bullet counting update function
 *  @param[in] dir  (for dual-direction feeder)index of direction
 */
void feeder_bulletOut(uint8_t dir)
{
  if(dir != FEEDER_CW && dir != FEEDER_CCW)
    return;

  if(chVTGetSystemTimeX() > bullet_out_time + MS2ST(10))
  {feeder_brake();
    bulletCount[dir]++;
    bullet_out_time = chVTGetSystemTimeX();

    if(bulletCount[dir] > bulletCount_stop[dir])
    {
      feeder_brake();
      if(refiller_thread != NULL)
      {
        chThdResumeI(&refiller_thread, MSG_OK);
        refiller_thread = NULL;
      }
    }
  }
}

/*
 *  @brief                  refiller control function
 *  @param[in] dir          (for dual-direction feeder)index of direction
 *  @param[in] bullet_num   bullets to deliver
 */
void feeder_refill(uint8_t dir, const uint32_t bullet_num)
{
  if(feeder_state == FEEDER_STOP)
  {
    feeder_state = (dir == FEEDER_CW) ? FEEDER_AUTO_CW : FEEDER_AUTO_CCW;
    bulletCount_stop[dir] = bulletCount[dir] + bullet_num;
  }
}

static void feeder_rest(void)
{
    int16_t current_speed = lpfilter_apply(&lp_spd_feeder, feeder_encode->raw_speed);
    float error = - (float) current_speed;
    rest_pid.inte += error * rest_pid.ki;
    rest_pid.inte = rest_pid.inte > rest_pid.inte_max?  rest_pid.inte_max:rest_pid.inte;
    rest_pid.inte = rest_pid.inte <-rest_pid.inte_max? -rest_pid.inte_max:rest_pid.inte;

    feeder_output = rest_pid.kp * error + rest_pid.inte;
    feeder_output = feeder_output > 4000?  4000:feeder_output;
    feeder_output = feeder_output <-4000? -4000:feeder_output;
}

static int16_t feeder_controlVel(const float target, const float output_max){
    static float last_error;
    static float current_error;

    last_error = current_error;
    int16_t current_speed = feeder_encode->raw_speed;
    current_speed = lpfilter_apply(&lp_spd_feeder, current_speed);
    current_error = target - (float) current_speed;
    vel_pid.inte += current_error * vel_pid.ki;
    vel_pid.inte = vel_pid.inte > vel_pid.inte_max?  vel_pid.inte_max:vel_pid.inte;
    vel_pid.inte = vel_pid.inte <-vel_pid.inte_max? -vel_pid.inte_max:vel_pid.inte;

    float output = vel_pid.kp * current_error + vel_pid.inte + vel_pid.kd * (current_error - last_error);
    output = output > output_max?  output_max:output;
    output = output <-output_max? -output_max:output;

    return (int16_t) output;

}

static int16_t feeder_controlPos(const float target, const float output_max){

    float error = target - (float)feeder_encode->total_ecd;

    pos_pid.inte += error * pos_pid.ki;
    pos_pid.inte = pos_pid.inte > pos_pid.inte_max?  pos_pid.inte_max:pos_pid.inte;
    pos_pid.inte = pos_pid.inte <-pos_pid.inte_max? -pos_pid.inte_max:pos_pid.inte;

    float speed_sp = pos_pid.kp * error +
                     pos_pid.inte -
                     pos_pid.kd * feeder_encode->raw_speed;

    return feeder_controlVel(speed_sp, output_max);
}

static void feeder_func(){
    feeder_output = 0.0f;
    switch (feeder_state){
        case FEEDER_FINISHED:
        case FEEDER_STOP:
            if(chVTGetSystemTimeX() > feeder_stop_time + S2ST(1))
              feeder_rest();
            else
              feeder_output = feeder_controlPos(feeder_brakePos, FEEDER_OUTPUT_MAX);

            break;
        case FEEDER_CW:
        case FEEDER_AUTO_CW:
            //error detecting
            if (
                 state_count((feeder_encode->raw_speed < 30) &&
                             (feeder_encode->raw_speed > -30),
                 FEEDER_ERROR_COUNT, &error_count)
               )
            {
              float error_angle_sp = feeder_encode->total_ecd -
                FEEDER_TURNBACK_ANGLE / 360.0f * FEEDER_GEAR * 8192;
              systime_t error_start_time = chVTGetSystemTime();
              while (chVTIsSystemTimeWithin(error_start_time, (error_start_time + MS2ST(200))))
              {
                feeder_output = feeder_controlPos(error_angle_sp, FEEDER_OUTPUT_MAX_BACK);
                feeder_canUpdate();
                chThdSleepMilliseconds(1);
              }
            }

            feeder_output = feeder_controlVel(FEEDER_SPEED_SP_RPM, FEEDER_OUTPUT_MAX);

            break;
        case FEEDER_CCW:
        case FEEDER_AUTO_CCW:
            //error detecting
            if (
                     state_count((feeder_encode->raw_speed < 30) &&
                                 (feeder_encode->raw_speed > -30),
                     FEEDER_ERROR_COUNT, &error_count)
               )
            {
                float error_angle_sp = feeder_encode->total_ecd +
                  FEEDER_TURNBACK_ANGLE / 360.0f * FEEDER_GEAR * 8192;
                systime_t error_start_time = chVTGetSystemTime();
                while (chVTIsSystemTimeWithin(error_start_time, (error_start_time + MS2ST(200))))
                {
                  feeder_output = feeder_controlPos(error_angle_sp, FEEDER_OUTPUT_MAX_BACK);
                  feeder_canUpdate();
                  chThdSleepMilliseconds(1);
                }
            }

            feeder_output = feeder_controlVel(-FEEDER_SPEED_SP_RPM, FEEDER_OUTPUT_MAX);

            break;
        default:
            feeder_output = 0;
            break;
    }

    feeder_canUpdate();
}

static THD_WORKING_AREA(feeder_control_wa, 512);
static THD_FUNCTION(feeder_control, p){
    (void) p;
    chRegSetThreadName("feeder controller");
    while(!chThdShouldTerminateX())
    {
        feeder_func();

        if(
            feeder_state == FEEDER_STOP &&
            (p_dbus->rc.s1 == RC_S_DOWN)
          )
        {
          feeder_start_time = chVTGetSystemTimeX();
          feeder_state = FEEDER_CW;
        }
        else if(
            feeder_state == FEEDER_STOP &&
            (p_dbus->rc.s1 == RC_S_UP)
          )
        {
          feeder_start_time = chVTGetSystemTimeX();
          feeder_state = FEEDER_CCW;
        }
        else if((feeder_state == FEEDER_CW || feeder_state == FEEDER_CCW) &&
                p_dbus->rc.s1 == RC_S_MIDDLE)
          feeder_brake();
        else if(feeder_state == FEEDER_FINISHED)
          feeder_state = FEEDER_STOP;
        chThdSleepMilliseconds(1);
    }
}

static const FEEDER_VEL  = "FEEDER_VEL";
static const FEEDER_POS  = "FEEDER_POS";
static const FEEDER_rest_name = "FEEDER_REST";
static const char subname_feeder_PID[] = "KP KI KD Imax";
void feeder_init(void)
{

    feeder_encode = can_getChassisMotor();
    p_dbus = RC_get();

    params_set(&vel_pid, 14,4,FEEDER_VEL,subname_feeder_PID,PARAM_PUBLIC);
    params_set(&pos_pid, 15,4,FEEDER_POS,subname_feeder_PID,PARAM_PUBLIC);
    params_set(&rest_pid, 16,4,FEEDER_rest_name,subname_feeder_PID,PARAM_PUBLIC);

    lpfilter_init(&lp_spd_feeder, 1000, 30);
    feeder_brakePos = (float)feeder_encode->total_ecd;

    chThdCreateStatic(feeder_control_wa, sizeof(feeder_control_wa),
                     NORMALPRIO - 5, feeder_control, NULL);

}
