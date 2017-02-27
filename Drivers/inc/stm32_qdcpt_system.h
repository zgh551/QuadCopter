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
 #ifndef __STM32_QDCPT_SYSTEM_H
 #define __STM32_QDCPT_SYSTEM_H
 
 #ifdef __cplusplus
	extern C{
 #endif
	#include "stm32f10x.h"
		 /* Scheduler include files. */
	#include <stdlib.h>
	#include "FreeRTOS.h"
	#include "task.h"
		
 void Qdcpt_Usart_Tasks( unsigned portBASE_TYPE uxPriority );
 #ifdef __cplusplus
	}
 #endif
 #endif