/*
 * ChibiOS/Servo is a small library to easily interface RC servos with ChibiOS/RT.
 *
 * BSD licensed
 *
 * Author: Joel Bodenmann aka Tectu   <joel@unormal.org>
 */

#include "servo.h"

Servo servo_L = {
	GPIOI,
	5,
	&PWMD8,
	1,
	SERVO_MIN,
	SERVO_MAX
};

Servo servo_R = {
	GPIOI,
	6,
	&PWMD8,
	2,
	SERVO_MIN,
	SERVO_MAX
};

static PWMConfig pwmcfg = {
    1000000,	// 1MHz PWM clock frequency
    20000,		// PWM period 20 milliseconds
	  NULL,		// no callback
	  NULL,		// channel configuration set dynamically in servoInit()
    0
};

void servoInit(Servo *servo) {
	/* create the channel configuration */
    PWMChannelConfig chcfg = { PWM_OUTPUT_ACTIVE_HIGH, NULL };
    pwmcfg.channels[servo->pwm_channel] = chcfg;

	/* start the PWM unit */
	pwmStart(servo->pwm_driver, &pwmcfg);
}

void servoSetValue(Servo *servo, uint16_t value) {
	/* a bit of safty here */
	if(value > servo->max)
		value = servo->max;
	if(value < servo->min)
		value = servo->min;

	pwmEnableChannel(servo->pwm_driver, servo->pwm_channel, (pwmcnt_t)value);
}

void servoSetMax(Servo *servo, uint16_t value) {
	servo->max = value;
}

void servoSetMin(Servo *servo, uint16_t value) {
	servo->min = value;
}

uint16_t servoGetMax(Servo *servo) {
	return (servo->max);
}

uint16_t servoGetMin(Servo *servo) {
	return (servo->min);
}

void servosInit(){
	servoInit(&servo_L);
	servoInit(&servo_R);
	open_tank(0);
	open_tank(1);
	chThdSleepMilliseconds(500);
	close_tank(0);
	close_tank(1);
}

// Function for opening the sub tank using the servo motor
// tank = 0 servo 1
// tank = 1 servo 2
void open_tank (int tank){
	if (!tank){
        RIGHT_OK();
		servoSetValue(&servo_L,SERVO_OPEN);
	}
	else {
		LEFT_OK();
		servoSetValue(&servo_R,SERVO_OPEN);
	}
}

// Function for opening the sub tank using the servo motor
void close_tank (int tank){
	if (!tank){
		RIGHT_FINISH();
		servoSetValue(&servo_L,SERVO_CLOSE);
	}
	else {
		LEFT_FINISH();
		servoSetValue(&servo_R,SERVO_CLOSE);
	}
}

