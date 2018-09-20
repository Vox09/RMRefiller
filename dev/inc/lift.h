/*
 * File:   lift.h
 * Author: Xu Xinyuan
 * Date:   2018-07-03
 */

#ifndef LIFT_H
#define LIFT_H

#define LIFT_CAN_EID         0x200

#define LIFT_TOTAL_LENGTH 60    //cm
#define LIFT_UP_LENGTH 59      //cm
#define LIFT_DOWN_LENGTH 35      //cm

#define LIFT_OUT_MAX 10000U       //C620 +-16384 // old +-32768
#define DOOR_OUT_MAX 1500U       //C620 +-16384 // old +-32768

// init setup
#define LIFT_INIT_COUNT 300U
#define DOOR_INIT_COUNT 200U
#define LIFT_INIT_UP_SPEED 1100U
#define LIFT_INIT_DOWN_SPEED 400U
#define DOOR_INIT_SPEED 450
#define CLOSE_DOOR_ENCODER 2300000
#define LIFT_ENCODER 4000000
//2400000
#define SET_HERO_DISTANCE 20.0
#define SET_ENGINEER_DISTANCE 17.5
#define ENGINEER_COUNT 100U
#define HERO_COUNT 100U
#define HERO_STUCK_COUNT 50U

typedef enum{
    LIFT_INITIALING_DOWN = 0,
    LIFT_INITIALING_UP,
    LIFT_DOWN,
    LIFT_UP,
//    LIFT_HOLD,
    LIFT_SLOW_DOWN
}lift_mode_t;

typedef enum{
    DOOR_INITIALING_OPEN = 0,
    DOOR_INITIALING_CLOSE,
    DOOR_CLOSE,
    DOOR_OPEN,
}door_mode_t;

typedef enum{
    DOWN = 0,
    UP = 1
}lift_plc_t;

typedef enum{
    OPEN = 0,
    CLOSE = 1
}door_plc_t;

typedef enum{
    HERO = 0,
    ENG =1
}big_car_t;


// public interface
void lift_init(void);
void lift_go_down(void);
void lift_go_up(void);
void lift_hold(void);
void open_mid_door(void);
void close_mid_door(void);
//int  get_lift_state(void);
bool lift_done_up(void);
bool lift_done_down(void);
bool door_done_open(void);
bool door_done_close(void);
bool is_hero(void);

//debug
bool get_big_car(void);
bool get_lift_success(void);
int8_t get_lift_state(void);
int8_t get_door_state(void);
#endif