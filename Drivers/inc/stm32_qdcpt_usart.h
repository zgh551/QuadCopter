/**
******************************************************************************
* @file    stm32_qdcpt_usart.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32_QDCPT__USART_H
#define __STM32_QDCPT__USART_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdlib.h>
#include "crtp.h"	 
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "portmacro.h"	 
	 

	 
typedef void * xComPortHandle;

void QdcptUsartInit(void);
void QdcptUsartDmaInit(void);
void uartDmaIsr(void);
void uartIsr( void );	 
void uartSendData(uint32_t size, uint8_t* data);
int  uartPutchar(int ch);
void uartSendDataDma(uint32_t size, uint8_t* data);	 
int  uartSendCRTPPacket(CRTPPacket *p);
int  uartReceiveCRTPPacket(CRTPPacket *p);
void uartRxTask(void *param);
struct crtpLinkOperations * uartGetLink(void);
//use the queue to transmit the data
xComPortHandle QdcptUsartNvicInit(void);
signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed portCHAR *pcRxedChar, portTickType xBlockTime ); 
signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed portCHAR cOutChar, portTickType xBlockTime );
void   vSerialPutString( xComPortHandle pxPort, const signed portCHAR * const pcString, unsigned portSHORT usStringLength );

void Qdcpt_Usart_Tasks( unsigned portBASE_TYPE uxPriority );
#ifdef __cplusplus
 }
#endif
	 
#endif
