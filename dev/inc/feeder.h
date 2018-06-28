#ifndef FEEDER
#define FEEDER

#define FEEDER_SINGLE_TIMEOUT_MS 100U

#define FEEDER_CAN          &CAND1
#define FEEDERR_CAN_EID       0x200
#define FEEDERL_CAN_EID       0x1ff

#define FEEDER_BULLET_PER_TURN    7U
#define FEEDER_GEAR              36U
#define FEEDER_SET_BPS           10U     //Bullets per second of feeder

#define FEEDER_OUTPUT_MAX       10000U
#define FEEDER_OUTPUT_MAX_BACK  10000U  //output limit for stuck-bullet turning back
#define FEEDER_ERROR_COUNT        100U

#define RIGHT 0
#define LEFT  1
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

void feeder_canUpdate(uint8_t); //In case the feeder ESC is using the same EID as gimbal does
void feeder_bulletOut(uint8_t dir);  //Used as limit switch EXTI funtion
void feeder_singleShot(void); //Rune shooting function
void feeder_init(void);

void feeder_brake(uint8_t);
void feeder_refill(uint8_t);
//Used for refiller to deliver certain amount of bullet
int16_t *getFeederOutput();
#endif
