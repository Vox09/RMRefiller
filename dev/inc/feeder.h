#ifndef FEEDER_H
#define FEEDER_H

#define FEEDER_SINGLE_TIMEOUT_MS 100U

#define FEEDER_CAN          &CAND1
#define FEEDER_CAN_EID       0x1ff

#define FEEDER_BULLET_PER_TURN    7U
#define FEEDER_GEAR              36U
#define FEEDER_SET_BPS           12U     //Bullets per second of feeder

#define FEEDER_OUTPUT_MAX       10000U
#define FEEDER_OUTPUT_MAX_BACK  10000U  //output limit for stuck-bullet turning back
#define FEEDER_ERROR_COUNT        100U

typedef enum{
  FEEDER_STOP = 0,
  FEEDER_GO,       //Manual operation right tank
  FEEDER_AUTO_GO, //refiller auto operation right tank
  //FEEDER_FINISHED,  //Finished a round of shooting
}feeder_mode_t;


typedef struct{
    float kp;
    float ki;
    float kd;
    float inte_max;
    float inte;
} __attribute__((packed)) pid_struct;

void feeder_canUpdate(void); //In case the feeder ESC is using the same EID as gimbal does
void feeder_bulletOut(uint8_t dir);  //Used as limit switch EXTI funtion
void feeder_singleShot(void); //Rune shooting function
void feeder_init(void);

void feeder_brake(uint8_t);
void feeder_refill(uint8_t);
int16_t *getFeederOutput(void);
#endif
