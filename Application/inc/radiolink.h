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
#ifndef __RADIOLINK_H
#define __RADIOLINK_H

#ifdef __cplusplus
extern C{
#endif
//FREE RTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
//
#include "stm32f10x.h"
#include "crtp.h"

void radiolinkInit(void);
void radiolinkReInit(void);
static void interruptCallback(void);
int radioReceivePacket(CRTPPacket * pk);
int radioSendPacket(CRTPPacket * pk);
	
struct crtpLinkOperations * radiolinkGetLink(void);
#ifdef __cplusplus
}
#endif
#endif
