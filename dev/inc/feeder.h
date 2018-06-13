#ifndef FEEDER
#define FEEDER

#define FEEDER_SINGLE_TIMEOUT_MS 100U

#define FEEDER_CAN          &CAND1
#define FEEDER_CAN_EID       0x200

#define FEEDER_BULLET_PER_TURN    9U
#define FEEDER_GEAR              36U
#define FEEDER_SET_RPS           25U     //Rounds per second of feeder

#define FEEDER_OUTPUT_MAX       10000U
#define FEEDER_OUTPUT_MAX_BACK  10000U  //output limit for stuck-bullet turning back
#define FEEDER_ERROR_COUNT        100U

typedef enum{
  FEEDER_STOP = 0,
  FEEDER_CW,       //Manual operation CW
  FEEDER_CCW,      //Manual operation CCW
  FEEDER_AUTO_CW,  //refiller auto operation CW
  FEEDER_AUTO_CCW, //refiller auto operation CCW
  FEEDER_FINISHED,  //Finished a round of shooting
}feeder_mode_t;

typedef struct{
    float kp;
    float ki;
    float kd;
    float inte_max;
    float inte;
} __attribute__((packed)) pid_struct;

int16_t feeder_canUpdate(void); //In case the feeder ESC is using the same EID as gimbal does
void feeder_bulletOut(uint8_t dir);  //Used as limit switch EXTI funtion
void feeder_singleShot(void); //Rune shooting function
void feeder_init(void);

void feeder_brake(void);
void feeder_refill(uint8_t dir, const uint32_t bullet_num);
//Used for refiller to deliver certain amount of bullet

#endif
