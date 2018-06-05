/*
 * hcsr04.h
 *
 *  Created on: 26 Jan, 2018
 *      Author: ASUS
 */

#ifndef INC_RANGEFINDER_H_
#define INC_RANGEFINDER_H_

#define RANGEFINDER_WARM_UP 			 2550000
#define RANGEFINDER_TIM_FREQ        100000
#define M_TO_CM                     100.0f

#define S_TO_MS 							   	 1000.0f
#define RATIO												100.0f
#define MM_TO_CM												10

#define RANGEFINDER_INDEX_0        	     0
#define RANGEFINDER_INDEX_1    					 1

#define RANGEFINDER_NUM             	   2

typedef enum{
		RANGEFINDER_DISABLE = 0,
		RANGEFINDER_ENABLE
} rangefinder_cmd_t;

void rangeFinder_control(const uint8_t index, const bool enable);
float rangeFinder_getDistance(const uint8_t index);

void rangeFinder_init(void);

#endif /* INC_HCSR04_H_ */
