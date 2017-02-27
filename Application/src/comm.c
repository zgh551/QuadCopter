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
 comm.c - High level communication module
 */
 #include <stdbool.h>
 
 #include "radiolink.h"
 #include "crtp.h"
 #include "console.h"
 #include "comm.h"
 #include "stm32_qdcpt_usart.h"
 static bool isInit =0;
 
 void commInit(void)
 {
	if(isInit)
		return;
	
	radiolinkInit();
	//QdcptUsartNvicInit();
	crtpInit();
	//consoleInit();
	//crtpSetLink(uartGetLink());
	crtpSetLink(radiolinkGetLink());
	
	//crtpInitTaskQueue(CRTP_PORT_PID);

	isInit = true;
 }
