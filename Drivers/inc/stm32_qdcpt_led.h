/**
 ******************************************************************************
 * @file    stm32_qdcpt.h
 * @author  Zhu Guohua
 * @version V1.0
 * @date    07-February-2015
 * @brief   This file provides
 *            - over the printf function
 ******************************************************************************
 * @attention
 * Quadcopter = qdcpt
 * COPYRIGHT 2015 
 ******************************************************************************  
 */
 #ifndef __STM32_QDCPT_LED_H
 #define __STM32_QDCPT_LED_H
 
 #ifdef __cplusplus
  extern C{
 #endif
 /*include system file*/
 #include "stm32f10x.h"
 #include "stm32_qdcpt.h"
		
 #define  ON 1
 #define  OFF 0
		
 #define LED_NUM 2
		
void Stm32QdcptInitLed(void);		
void Stm32QdcptLedSet(Led_TypeDef led, uint8_t value);
void Stm32QdcptLedToggle(Led_TypeDef led);
		
 #ifdef __cplusplus
	}
 #endif
	
 #endif
