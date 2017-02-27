 /**
 ******************************************************************************
 * @file    stm32_qdcpt_exti.h
 * @author  Zhu Guohua
 * @version V1.0
 * @date    08-February-2015
 * @brief   This file provides
 *            - over the printf function
 ******************************************************************************
 * @attention
 * Quadcopter = qdcpt
 * COPYRIGHT 2015 
 ******************************************************************************  
 */ 
 #ifndef __STM32_QDCPT_EXTI_H
 #define __STM32_QDCPT_EXTI_H
 
 #ifdef __cplusplus
	extern C{
 #endif	
  #include "stm32f10x.h"
	void extiInit(void);
	void nrfIsr(void);
	void nrfSetInterruptCallback(void (*cb)(void));
	void extiInterruptHandler(void);
		
 #ifdef __cplusplus
	}
 #endif
 #endif
