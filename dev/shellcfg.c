/**
 * Edward ZHANG
 * @file    shellcfg.c
 * @brief   definitions of shell command functions
 */
#include "main.h"
#include "shell.h"
#include <string.h>
#include <inc/canBusProcess.h>

#define SERIAL_CMD       &SDU1
#define SERIAL_DATA      &SDU1

static thread_t* matlab_thread_handler = NULL;
/**
 * @brief Transmit uint32_t and float through serial port to host machine
 * @require Initialization of ChibiOS serial driver before using this function
 *
 * @param[in] chp         pointer to a @p BaseSequentialStream implementing object
 * @param[in] txbuf_d     array of 32-bit integers to tramsmit, can be signed or unsigned
 * @param[in] txbuf_f     array of float point numbers to tramsmit
 * @param[in] num_int     number of 32-bit integers to tramsmit
 * @param[in] num_float   number of float point numbers to tramsmit
 *
 * @TODO improve the transmission protocol to enable easier setup for the host machine
 */
#define SYNC_SEQ  0xaabbccdd
static void transmit_matlab
  (BaseSequentialStream* chp,
    uint32_t* const txbuf_d, float* const txbuf_f,
    const uint8_t num_int, const uint8_t num_float)
{
  uint32_t sync = SYNC_SEQ;
  char* byte = (char*)&sync;

  uint8_t i;
  for (i = 0; i < 4; i++)
    chSequentialStreamPut(chp, *byte++);

  byte = (char*)txbuf_d;
  for (i = 0; i < 4*num_int; i++)
    chSequentialStreamPut(chp, *byte++);

  byte = (char*)txbuf_f;
  for (i = 0; i < 4*num_float; i++)
    chSequentialStreamPut(chp, *byte++);
}

#define HOST_TRANSMIT_FREQ  100U
static THD_WORKING_AREA(matlab_thread_wa, 512);
static THD_FUNCTION(matlab_thread, p)
{
  (void)p;
  chRegSetThreadName("matlab tramsmitter");

  int32_t txbuf_d[16];
  float txbuf_f[16];
  BaseSequentialStream* chp = (BaseSequentialStream*)SERIAL_DATA;

//  GimbalStruct* gimbal = gimbal_get();

  uint32_t tick = chVTGetSystemTimeX();
  const uint16_t period = US2ST(1000000/HOST_TRANSMIT_FREQ);
  while (!chThdShouldTerminateX())
  {
    tick += period;
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    //txbuf_f[0] = PIMU->euler_angle[Roll];

    transmit_matlab(chp, NULL, txbuf_f, 0, 1);
  }
}

/*===========================================================================*/
/* Definitions of shell command functions                                    */
/*===========================================================================*/
static THD_WORKING_AREA(Shell_thread_wa, 1024);
void cmd_test(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;

    extern uint16_t bullet_count[2];

//  volatile ChassisEncoder_canStruct* feeder_encoder = can_getFeederMotor();
  volatile ChassisEncoder_canStruct* lift_encoder = can_getLiftMotor();
//  chprintf(chp," right speed: %f \r\n", feeder_encoder[RIGHT].msg_count);
//  chprintf(chp," right speed: %f \r\n", feeder_encoder[LEFT].msg_count);
//  chprintf(chp," left speed: %f \r\n", feeder_encoder[LIFT].msg_count);
//  chprintf(chp," left speed: %f \r\n", feeder_encoder[DOOR].msg_count);
//    chprintf(chp," left cmd: %d \r\n", getFeederOutput()[LEFT]);
//  chprintf(chp,"count 0: %d\r\n", bullet_count[0]);
//  chprintf(chp,"count 1: %d\r\n", bullet_count[1]);
  while(true) {
//    chprintf(chp, "rangefinder : %f  ||  %f \r\n",
//             rangeFinder_getDistance(RANGEFINDER_INDEX_0),
//             rangeFinder_getDistance(RANGEFINDER_INDEX_1));
//    chprintf(chp, "R1 L1:        %d  ||  %d \r\n", LS_R1_DOWN(),LS_L1_DOWN());
//    chprintf(chp, "R2 L2:        %d  ||  %d \r\n", LS_R2_DOWN(),LS_L2_DOWN());
//    chprintf(chp, "Distance   :  %d  ||  %d \r\n", getDistanceOK()[0],getDistanceOK()[1]);
//    chprintf(chp, "Door status:  %d  ||  %d \r\n", getDoor()[0],getDoor()[1]);
//    chprintf(chp, "Finished   :  %d  ||  %d \r\n", getFinished()[0],getFinished()[1]);
//      chprintf(chp, "come :  %f   \r\n",rangeFinder_getDistance(COME_SENSOR));
//    chprintf(chp, "leave :  %f   \r\n",rangeFinder_getDistance(LEAVE_SENSOR));
//      chprintf(chp, "lift state :  %d   \r\n",get_lift_state());
//      chprintf(chp, "door state :  %d   \r\n",get_door_state());
//      chprintf(chp, "big car :  %d   \r\n",get_big_car());
//    chprintf(chp, "success :  %d   \r\n",get_lift_success());
//    chprintf(chp, "lift done up :  %d   \r\n",lift_done_up());
//    chprintf(chp, "lift done down :  %d   \r\n",lift_done_down());
//      chprintf(chp, "door done open :  %d   \r\n",door_done_open());
//      chprintf(chp, "door done close :  %d   \r\n",door_done_close());
//      chprintf(chp, "lift encoder :  %d   \r\n", lift_encoder[LIFT].total_ecd);
      chprintf(chp, " up range :  %f   \r\n", rangeFinder_getDistance(LEAVE_SENSOR));
    chThdSleepMilliseconds(500);
  }
//  chprintf(chp,"Lift state: %d\r\n", get_lift_state());
//  chprintf(chp,"lift count: %d\r\n", can_getLiftMotor()->msg_count);
//  chprintf(chp,"lift total_ecd: %d\r\n", can_getLiftMotor()->total_ecd);
}

void cmd_zero(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc, argv;
  extern uint16_t bullet_count[2];
  bullet_count[0] = 0;
  bullet_count[1] = 0;
}

void cmd_up(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;
    lift_go_up();
}

void cmd_down(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;
    lift_go_down();
}

void cmd_stop(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;
  can_motorSetCurrent(FEEDER_CAN,LIFT_CAN_EID,0,0,0,0);
}

void cmd_error(BaseSequentialStream * chp, int argc, char *argv[])
{

}

/**
 * @brief Start the data tramsmission to matlab
 * @note caution of data flooding to the serial port
 */
void cmd_data(BaseSequentialStream * chp, int argc, char *argv[])
{
  uint8_t sec = 10;

  if(argc && matlab_thread_handler == NULL)
  {
    char *toNumber = argv[0];
    uint32_t finalNum=0;
    while(*toNumber>='0' && *toNumber<='9')
      finalNum=finalNum*10+*(toNumber++)-'0';

    if(finalNum == 0)
      finalNum = 10;

    sec = (finalNum < 60 ? finalNum : 60);

    chprintf(chp,"Data transmission start in %d seconds...\r\n", sec);
    chThdSleepSeconds(sec);

    matlab_thread_handler = chThdCreateStatic(matlab_thread_wa, sizeof(matlab_thread_wa),
        NORMALPRIO - 3,
        matlab_thread, NULL);
  }
  else if(matlab_thread_handler != NULL)
  {
    chThdTerminate(matlab_thread_handler);
    matlab_thread_handler = NULL;
  }
}

/*extern volatile int16_t measured_speed_fuck;
void cmd_measure(BaseSequentialStream * chp, int argc, char *argv[])
{
	(void) argc,argv;
	chprintf(chp,"speed: %d\n", measured_speed_fuck);
}*/


/**
 * @brief array of shell commands, put the corresponding command and functions below
 * {"command", callback_function}
 */
static const ShellCommand commands[] =
{
  {"t", cmd_test},
  {"z", cmd_zero},
  {"u",cmd_up},
  {"d",cmd_down},
  {"s",cmd_stop},
  {"WTF", cmd_error},
	//{"m", cmd_measure},
	{"\xEE", cmd_data},
  #ifdef PARAMS_USE_USB
    {"\xFD",cmd_param_scale},
    {"\xFB",cmd_param_update},
    {"\xFA",cmd_param_tx},
    {"\xF9",cmd_param_rx},
  #endif
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 =
{
  (BaseSequentialStream *)SERIAL_CMD,
  commands
};

/**
 * @brief start the shell service
 * @require enable the corresponding serial ports in mcuconf.h and board.h
 *          Select the SERIAL_CMD port in main.h
 *
 * @api
 */
void shellStart(void)
{
  //sdStart(SERIAL_CMD, NULL);
  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */


  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);

  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  shellInit();

  shellCreateStatic(&shell_cfg1, Shell_thread_wa,
      sizeof(Shell_thread_wa), NORMALPRIO);

}
