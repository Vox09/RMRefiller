/**
 * Edward ZHANG, 20171101
 * @file    canBusProcess.c
 * @brief   CAN driver configuration file
 * @reference   RM2017_Archive
 */

#include "ch.h"
#include "hal.h"
#include "string.h"

#include "canBusProcess.h"
#include "dbus.h"

RC_Ctl_t* P_Get_Dbus;


static volatile ChassisEncoder_canStruct feeder_encoder[FEEDER_MOTOR_NUM];
static volatile ChassisEncoder_canStruct lift_encoder[LIFT_MOTOR_NUM];


/*
 * 500KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 */
static const CANConfig cancfg = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP, //HAL LIB, hcan1.Init.ABOM = DISABLE;hcan1.Init.AWUM = DISABLE;
  CAN_BTR_SJW(0) | CAN_BTR_TS2(3) |
  CAN_BTR_TS1(8) | CAN_BTR_BRP(2)
};

#define CAN_FILTER_NUM 28U
static CANFilter canfilter[CAN_FILTER_NUM];


volatile ChassisEncoder_canStruct* can_getFeederMotor(void)
{
  return feeder_encoder;
}

volatile ChassisEncoder_canStruct* can_getLiftMotor(void)
{
    return lift_encoder;
}

static inline void can_getMotorOffset
        (volatile ChassisEncoder_canStruct* cm, const CANRxFrame* const rxmsg)
{
    chSysLock();
    cm->updated = true;
    cm->raw_angle = (uint16_t)(rxmsg->data8[0]) << 8 | rxmsg->data8[1];
    cm->raw_speed = (int16_t)(rxmsg->data8[2]) << 8 | rxmsg->data8[3];
    cm->act_current = (int16_t)(rxmsg->data8[4]) << 8 | rxmsg->data8[5];
    cm->temperature = (uint8_t)rxmsg->data8[6];
    chSysUnlock();

    cm->offset_raw_angle = cm->raw_angle;
}

static inline void can_processChassisEncoder
  (volatile ChassisEncoder_canStruct* cm, const CANRxFrame* const rxmsg)
{
    cm->last_raw_angle = cm->raw_angle;

    chSysLock();
    cm->updated = true;
    cm->raw_angle = (uint16_t)(rxmsg->data8[0]) << 8 | rxmsg->data8[1];
    cm->raw_speed = (int16_t)(rxmsg->data8[2]) << 8 | rxmsg->data8[3];
    cm->act_current = (int16_t)(rxmsg->data8[4]) << 8 | rxmsg->data8[5];
    cm->temperature = (uint8_t)rxmsg->data8[6];
    chSysUnlock();

    if      (cm->raw_angle - cm->last_raw_angle >  CAN_ENCODER_RANGE / 2) cm->round_count--;
    else if (cm->raw_angle - cm->last_raw_angle < -CAN_ENCODER_RANGE / 2) cm->round_count++;

    cm->total_ecd = cm->round_count * CAN_ENCODER_RANGE + cm->raw_angle - cm->offset_raw_angle;
    cm->radian_angle = cm->total_ecd * CAN_ENCODER_RADIAN_RATIO;
}

static void can_processEncoderMessage(CANDriver* const canp, const CANRxFrame* const rxmsg)
{
    switch(rxmsg->SID)
    {
        case FEEDERR_CAN_SID:
            feeder_encoder[RIGHT].msg_count++;
            feeder_encoder[RIGHT].msg_count <= 50 ? can_getMotorOffset(&feeder_encoder[RIGHT],rxmsg) : can_processChassisEncoder(&feeder_encoder[RIGHT],rxmsg);
            break;
        case FEEDERL_CAN_SID:
            feeder_encoder[LEFT].msg_count++;
            feeder_encoder[LEFT].msg_count <= 50 ? can_getMotorOffset(&feeder_encoder[LEFT],rxmsg) : can_processChassisEncoder(&feeder_encoder[LEFT],rxmsg);
            break;
        case LIFT_CAN_SID:
            lift_encoder[LIFT].msg_count++;
            lift_encoder[LIFT].msg_count <= 50 ? can_getMotorOffset(&lift_encoder[LIFT],rxmsg) : can_processChassisEncoder(&lift_encoder[LIFT],rxmsg);
            break;
        case LIFT_DOOR_CAN_SID:
            lift_encoder[DOOR].msg_count++;
            lift_encoder[DOOR].msg_count <= 50 ? can_getMotorOffset(&lift_encoder[DOOR],rxmsg) : can_processChassisEncoder(&lift_encoder[DOOR],rxmsg);
            break;
    }
}

/*
 * Receiver thread.
 */
static THD_WORKING_AREA(can_rx1_wa, 256);
static THD_WORKING_AREA(can_rx2_wa, 256);
static THD_FUNCTION(can_rx, p) {

  CANDriver* canp = (CANDriver*)p;
  event_listener_t el;
  CANRxFrame rxmsg;

  (void)p;
  chRegSetThreadName("can receiver");
  chEvtRegister(&canp->rxfull_event, &el, 0);
  while(!chThdShouldTerminateX())
  {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0)
      continue;
    while (canReceive(canp, CAN_ANY_MAILBOX,
                      &rxmsg, TIME_IMMEDIATE) == MSG_OK)
    {
      can_processEncoderMessage(canp, &rxmsg);
    }

  }
  chEvtUnregister(&canp->rxfull_event, &el);
}

/*
 * @brief              Send motor current cmd using CAN driver
 * @param[in] cand     Pointer to CANDriver object we are currently using
 * @param[in] cmx_iq   Current (Torque) cmd of motor
 *
 * @notapi
 */
void can_motorSetCurrent(CANDriver *const CANx,
  const uint16_t EID,
  const int16_t cm1_iq,
  const int16_t cm2_iq,
  const int16_t cm3_iq,
  const int16_t cm4_iq)
{
    CANTxFrame txmsg;

    txmsg.IDE = CAN_IDE_STD;
    txmsg.EID = EID;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    chSysLock();
    txmsg.data8[0] = (uint8_t)(cm1_iq >> 8);
    txmsg.data8[1] = (uint8_t)cm1_iq;

    txmsg.data8[2] = (uint8_t)(cm2_iq >> 8);
    txmsg.data8[3] = (uint8_t)cm2_iq;

    txmsg.data8[4] = (uint8_t)(cm3_iq >> 8);
    txmsg.data8[5] = (uint8_t)cm3_iq;

    txmsg.data8[6] = (uint8_t)(cm4_iq >> 8);
    txmsg.data8[7] = (uint8_t)cm4_iq;
    chSysUnlock();

    canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}

void can_processInit(void)
{

  memset((void *)feeder_encoder, 0, sizeof(ChassisEncoder_canStruct)*FEEDER_MOTOR_NUM);

  uint8_t i;
  for (i = 0; i < CAN_FILTER_NUM; i++)
  {
    canfilter[i].filter = i;
    canfilter[i].mode = 0; //CAN_FilterMode_IdMask
    canfilter[i].scale = 1; //CAN_FilterScale_32bit
    canfilter[i].assignment = 0;
    canfilter[i].register1 = 0;
    canfilter[i].register2 = 0;
  }

  canSTM32SetFilters(14, CAN_FILTER_NUM, canfilter);

  canStart(&CAND1, &cancfg);
  canStart(&CAND2, &cancfg);

    P_Get_Dbus = RC_get();

  /*
   * Starting the transmitter and receiver threads.
   */
  chThdCreateStatic(can_rx1_wa, sizeof(can_rx1_wa), NORMALPRIO + 7,
                    can_rx, (void *)&CAND1);
  chThdCreateStatic(can_rx2_wa, sizeof(can_rx2_wa), NORMALPRIO + 7,
                    can_rx, (void *)&CAND2);

  chThdSleepMilliseconds(20);
}
