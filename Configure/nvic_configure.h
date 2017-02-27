  /**
 ******************************************************************************
 * @file    nvic_configure.h
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
 #ifndef __NVIC_CONFIGURE_H
 #define __NVIC_CONFIGURE_H
 
 #ifdef __cplusplus
	extern C{
 #endif
		
	//#include "stm32f10x.h"
		
#define NVIC_I2C_PRI          5
#define NVIC_UART_PRI         6
#define NVIC_TRACE_TIM_PRI    7
#define NVIC_ADC_PRI          12
#define NVIC_RADIO_PRI        13 
		
 #ifdef __cplusplus
	}
 #endif
	
 #endif
