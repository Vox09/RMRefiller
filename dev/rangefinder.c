/*
 * rangefinder.c
 *
 *  Created on: 26 Jan, 2018
 *      Author: ASUS
 */
#include "ch.h"
#include "hal.h"

#include "rangefinder.h"

static uint8_t  state          [RANGEFINDER_NUM];
static float    distance_cm    [RANGEFINDER_NUM];
static uint32_t captureDistance[RANGEFINDER_NUM];

#define RANGEFINDER_PSC  S_TO_MS / (float)(RANGEFINDER_TIM_FREQ) * RATIO /MM_TO_CM

/* Callback function for timer 5 (Sensor 1 2 3 4) */
static void gpt5_cb(GPTDriver *gptp)
{
	uint16_t SR = gptp->tim->SR;
	uint16_t CCER = gptp->tim->CCER;

	// Channel 1
	if(SR & STM32_TIM_SR_CC1IF)
	{
		if(CCER & STM32_TIM_CCER_CC1P)
			distance_cm[RANGEFINDER_INDEX_0] =
							(float)(gptp->tim->CCR[RANGEFINDER_INDEX_0] - captureDistance[RANGEFINDER_INDEX_0])
							* RANGEFINDER_PSC;
		else
			captureDistance[RANGEFINDER_INDEX_0] =
							gptp->tim->CCR[RANGEFINDER_INDEX_0];

		CCER ^= STM32_TIM_CCER_CC1P;
	}

	// Channel 2
	if(SR & STM32_TIM_SR_CC2IF)
	{
		if(CCER & STM32_TIM_CCER_CC2P)
			distance_cm[RANGEFINDER_INDEX_1] =
							(float)(gptp->tim->CCR[RANGEFINDER_INDEX_1] - captureDistance[RANGEFINDER_INDEX_1])
							* RANGEFINDER_PSC;
		else
			captureDistance[RANGEFINDER_INDEX_1] =
					gptp->tim->CCR[RANGEFINDER_INDEX_1];

		CCER ^= STM32_TIM_CCER_CC2P;
	}

	// Channel 3
	if(SR & STM32_TIM_SR_CC3IF)
	{
		if(CCER & STM32_TIM_CCER_CC3P)
			distance_cm[COME_SENSOR] =
					(float)(gptp->tim->CCR[COME_SENSOR] - captureDistance[COME_SENSOR])
					* RANGEFINDER_PSC;
		else
			captureDistance[COME_SENSOR] =
					gptp->tim->CCR[COME_SENSOR];

		CCER ^= STM32_TIM_CCER_CC3P;
	}
	gptp->tim->CCER = CCER;

    // Channel 4
	if(SR & STM32_TIM_SR_CC4IF)
	{
		if(CCER & STM32_TIM_CCER_CC4P)
			distance_cm[LEAVE_SENSOR] =
					(float)(gptp->tim->CCR[LEAVE_SENSOR] - captureDistance[LEAVE_SENSOR])
					* RANGEFINDER_PSC;
		else
			captureDistance[LEAVE_SENSOR] =
					gptp->tim->CCR[LEAVE_SENSOR];

		CCER ^= STM32_TIM_CCER_CC4P;
	}
	gptp->tim->CCER = CCER;
}

/*
 * GPT5 configuration.
 */
static const GPTConfig gpt5_cfg = {
				RANGEFINDER_TIM_FREQ,
				gpt5_cb,   /* Timer callback.*/
				0,
				0
};

void rangeFinder_control(const uint8_t index, const bool enable)
{
	if(index >= RANGEFINDER_NUM || state[index] == enable)
		return;

	uint16_t CCER, DIER;
	GPTDriver* gpt = &GPTD5;

	CCER = gpt->tim->CCER;
	DIER = gpt->tim->DIER;

	captureDistance[index] = 0;
	distance_cm[index] = 0.0f;

	switch (index)
	{
		case RANGEFINDER_INDEX_0:
			DIER ^= STM32_TIM_DIER_CC1IE;
			CCER ^= STM32_TIM_CCER_CC1E;
			break;
		case RANGEFINDER_INDEX_1:
			DIER ^= STM32_TIM_DIER_CC2IE;
			CCER ^= STM32_TIM_CCER_CC2E;
            break;
		case COME_SENSOR:
			DIER ^= STM32_TIM_DIER_CC3IE;
			CCER ^= STM32_TIM_CCER_CC3E;
			break;
		case LEAVE_SENSOR:
			DIER ^= STM32_TIM_DIER_CC4IE;
			CCER ^= STM32_TIM_CCER_CC4E;
			break;
	}

	gpt->tim->DIER = DIER;
	gpt->tim->CCER = CCER;

	state[index] = enable;
}

float rangeFinder_getDistance(const uint8_t index)
{
	if(index > RANGEFINDER_NUM || distance_cm[index] == 0.0f)
		return 999.9f;
	return distance_cm[index];
}

void rangeFinder_init(void)
{
	memset(distance_cm, 0, 4* RANGEFINDER_NUM);
	memset(state,       0,    RANGEFINDER_NUM);

	//  GPTD5 setup
	gptStart(&GPTD5, &gpt5_cfg);
	GPTD5.tim->DIER = 0;
	GPTD5.tim->CCER = 0;
	GPTD5.tim->CCMR1 |= STM32_TIM_CCMR1_CC1S(1) | STM32_TIM_CCMR1_CC2S(1)|
											STM32_TIM_CCMR1_IC1F(3) | STM32_TIM_CCMR1_IC2F(3);
	GPTD5.tim->CCMR2 |= STM32_TIM_CCMR2_CC3S(1) |STM32_TIM_CCMR2_IC3F(3) |
                  STM32_TIM_CCMR2_CC4S(1) |STM32_TIM_CCMR2_IC4F(3);
	GPTD5.tim->CNT = 10000000U;
	GPTD5.tim->CR1 |= STM32_TIM_CR1_ARPE | STM32_TIM_CR1_CEN;

	// Enable all the available rangefinders
	int i;
	for (i = 0; i < RANGEFINDER_NUM; i++){
		rangeFinder_control(i, ENABLE);
	}
}
