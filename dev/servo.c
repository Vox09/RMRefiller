/*
 * ChibiOS/Servo is a small library to easily interface RC servos with ChibiOS/RT.
 *
 * BSD licensed
 *
 * Author: Joel Bodenmann aka Tectu   <joel@unormal.org>
 */

#include "servo.h"

static PWMConfig pwmcfg = {
    1000000,	// 1MHz PWM clock frequency
    20000,		// PWM period 20 milliseconds
	NULL,		// no callback
		{
						{PWM_OUTPUT_ACTIVE_HIGH, NULL},
						{PWM_OUTPUT_ACTIVE_HIGH, NULL},
						{PWM_OUTPUT_DISABLED, NULL},
						{PWM_OUTPUT_DISABLED, NULL}
		},
	NULL,		// channel configuration set dynamically in servoInit()
    0
};

void servoInit(Servo *servo) {

	palSetPad(servo->port, servo->pin);
	palClearPad(servo->port, servo->pin);

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

