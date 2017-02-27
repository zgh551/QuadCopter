/**
******************************************************************************
* @file    motors.c
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


#include "motors.h"
#include "stm32_qdcpt.h"
//FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

/* Utils Conversion macro */
#define C_BITS_TO_16(X) ((X)<<(16-MOTORS_PWM_BITS))
#define C_16_TO_BITS(X) ((X)>>(16-MOTORS_PWM_BITS)&((1<<MOTORS_PWM_BITS)-1))
//																											01 1111 1111

static bool isInit = false;

void motorsInit(void)
{
	
	//Init structures
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	if (isInit)
  return;

  //GPIOA clock enable
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | QDCPT_TIM_PWM_GPIO_CLK, ENABLE);
	//TIM2 clock enable
	RCC_APB1PeriphClockCmd(QDCPT_TIM_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = (QDCPT_TIM_PWM1_PIN |
																 QDCPT_TIM_PWM2_PIN |
																 QDCPT_TIM_PWM3_PIN |
																 QDCPT_TIM_PWM4_PIN);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(QDCPT_TIM_PWM_GPIO_PORT, &GPIO_InitStructure);
	
	//Timer configuration
  TIM_TimeBaseStructure.TIM_Period = MOTORS_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(QDCPT_TIM, &TIM_TimeBaseStructure);

  //PWM channels configuration (All identical!)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(QDCPT_TIM, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(QDCPT_TIM, TIM_OCPreload_Enable);

  TIM_OC2Init(QDCPT_TIM, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(QDCPT_TIM, TIM_OCPreload_Enable);

  TIM_OC3Init(QDCPT_TIM, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(QDCPT_TIM, TIM_OCPreload_Enable);

  TIM_OC4Init(QDCPT_TIM, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(QDCPT_TIM, TIM_OCPreload_Enable);
  
	//Enable the ARR register 	
  TIM_ARRPreloadConfig(QDCPT_TIM, ENABLE);
  //Enable the timer
  TIM_Cmd(QDCPT_TIM, ENABLE);
  //Enable the timer PWM outputs
  TIM_CtrlPWMOutputs(QDCPT_TIM, ENABLE);
	
	isInit = true;
}

void motorsSetRatio(MOTOR id, uint16_t ratio)
{
  switch(id)
  {
    case MOTOR_M1:
      TIM_SetCompare1(QDCPT_TIM, C_16_TO_BITS(ratio));
      break;
    case MOTOR_M2:
      TIM_SetCompare2(QDCPT_TIM, C_16_TO_BITS(ratio));
      break;
    case MOTOR_M3:
      TIM_SetCompare3(QDCPT_TIM, C_16_TO_BITS(ratio));
      break;
    case MOTOR_M4:
      TIM_SetCompare4(QDCPT_TIM, C_16_TO_BITS(ratio));
      break;
  }
}

int motorsGetRatio(MOTOR id)
{
  switch(id)
  {
    case MOTOR_M1:
      return C_BITS_TO_16(TIM_GetCapture1(QDCPT_TIM));
    case MOTOR_M2:
      return C_BITS_TO_16(TIM_GetCapture2(QDCPT_TIM));
    case MOTOR_M3:
      return C_BITS_TO_16(TIM_GetCapture3(QDCPT_TIM));
    case MOTOR_M4:
      return C_BITS_TO_16(TIM_GetCapture4(QDCPT_TIM));
  }

  return -1;
}

bool motorsTest(void)
{
    motorsSetRatio(MOTOR_M1, MOTORS_TEST_RATIO);
    vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
    motorsSetRatio(MOTOR_M1, 0);
    vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
	
	  motorsSetRatio(MOTOR_M2, MOTORS_TEST_RATIO);
    vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
    motorsSetRatio(MOTOR_M2, 0);
    vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
	
	  motorsSetRatio(MOTOR_M3, MOTORS_TEST_RATIO);
    vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
    motorsSetRatio(MOTOR_M3, 0);
    vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
	
	  motorsSetRatio(MOTOR_M4, MOTORS_TEST_RATIO);
    vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
    motorsSetRatio(MOTOR_M4, 0);
    vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
  return isInit;
}
