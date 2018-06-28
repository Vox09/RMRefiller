#ifndef _CAN_BUS_PROCESS_H_
#define _CAN_BUS_PROCESS_H_

#include "stdint.h"
#include "stdbool.h"
#include "hal.h"
#include "string.h"

#define FEEDER_MOTOR_NUM  2U

#define FEEDERR_CAN_SID       0x201
#define FEEDERL_CAN_SID       0x205

#define CAN_GIMBAL_SEND_DBUS_ID                     0x001

#define CAN_ENCODER_RANGE           8192            // 0x2000
#define CAN_ENCODER_RADIAN_RATIO    7.669904e-4f    // 2*M_PI / 0x2000

typedef enum
{
  GIMBAL_YAW = 0,
  GIMBAL_PITCH
}gimbal_num_t;

typedef enum
{ RIGHT = 0,
  LEFT = 1
}feeder_num_t;

typedef struct {
    uint16_t raw_angle;
    int16_t  raw_current;
    int16_t  current_setpoint;

    uint16_t last_raw_angle;
    uint16_t offset_raw_angle;
    int32_t round_count;
    int32_t total_ecd;
    float radian_angle; // Continuous

    bool updated;
} GimbalEncoder_canStruct;

typedef struct {
    uint16_t raw_angle;
    int16_t  raw_speed;
    int16_t act_current;
    uint8_t temperature;

    uint16_t last_raw_angle;
    uint16_t offset_raw_angle;
    uint32_t msg_count;
    int32_t round_count;
    int32_t total_ecd;
    float radian_angle; // Continuous

    bool updated;
} ChassisEncoder_canStruct;

typedef struct{
    uint16_t channel0;
    uint16_t channel1;
    uint8_t  s1;
    uint8_t  s2;
    uint16_t key_code;
} Gimbal_Send_Dbus_canStruct;

volatile GimbalEncoder_canStruct* can_getGimbalMotor(void);
volatile ChassisEncoder_canStruct* can_getFeederMotor(void);
volatile ChassisEncoder_canStruct* can_getExtraMotor(void);

void can_processInit(void);
void can_motorSetCurrent(CANDriver *const CANx,
  const uint16_t EID,
  const int16_t cm1_iq,
  const int16_t cm2_iq,
  const int16_t cm3_iq,
  const int16_t cm4_iq);

void can_gimbal_send_dbus(CANDriver *const CANx);


#endif
