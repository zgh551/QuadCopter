 /**
 ******************************************************************************
 * @file    stm32_qdcpt.h
 * @author  Zhu Guohua
 * @version V1.0
 * @date    07-March-2015
 * @brief   This file provides
 *            - over the printf function
******************************************************************************
* @attention
* Quadcopter = qdcpt
* COPYRIGHT 2015 
******************************************************************************  
*/
#ifndef __MOTORS_H
#define __MOTORS_H

#ifdef __cplusplus
extern C{
#endif
#include "stm32f10x.h"
#include <stdbool.h>	
/******** Defines ********/
//The following defines gives a PWM of 9 bits at ~140KHz for a sysclock of 72MHz
#define MOTORS_PWM_BITS     9
#define MOTORS_PWM_PERIOD   ((1<<MOTORS_PWM_BITS) - 1)
#define MOTORS_PWM_PRESCALE 0

// Test defines
#define MOTORS_TEST_RATIO         (uint16_t)(0.3*(1<<16))
#define MOTORS_TEST_ON_TIME_MS    10
#define MOTORS_TEST_DELAY_TIME_MS 50
	
typedef enum{MOTOR_M1,MOTOR_M2,MOTOR_M3,MOTOR_M4}MOTOR;

void motorsInit(void);
void motorsSetRatio(MOTOR id, uint16_t ratio);
int motorsGetRatio(MOTOR id);
bool motorsTest(void);
#ifdef __cplusplus
}
#endif
#endif
