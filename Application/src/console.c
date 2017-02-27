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
  console.c - Used to send console data to client
 */
 
 

 /*FreeRtos includes*/
 #include "FreeRTOS.h"
 #include "task.h"
 #include "semphr.h"
 #include "console.h"
 #include "crtp.h"
 
 CRTPPacket messageToPrint;
 xSemaphoreHandle synch = NULL;
 
 static bool isInit;
 
/***********************************
  @brief Send the data to the client
	@param none 
	@retun none
*************************************/
static void consoleSendMessage(void)
{
  crtpSendPacketBlock(&messageToPrint);
  messageToPrint.size = 0;
}

/**
	@breif initialise the console 
  @param none
	@retun none
*/
void consoleInit(void)
{
  if (isInit)
    return;

  messageToPrint.size = 0;
  messageToPrint.header = CRTP_HEADER(CRTP_PORT_CONSOLE,0);
	//create the binary value
  vSemaphoreCreateBinary(synch);
  
  isInit = true;
}

bool consoleTest(void)
{
  return isInit;
}

int consolePutchar(int ch)
{
  if (xSemaphoreTake(synch, portMAX_DELAY) == pdTRUE)
  {
    messageToPrint.data[messageToPrint.size] = (unsigned char)ch;
    messageToPrint.size++;
    if (ch == '\n' || messageToPrint.size == CRTP_MAX_DATA_SIZE)
    {
      consoleSendMessage();
    }
    xSemaphoreGive(synch);
  }
  
  return (unsigned char)ch;
}

int consolePuts(char *str)
{
  int ret = 0;
  
  while(*str)
    ret |= consolePutchar(*str++);
  
  return ret;
}

void consoleFlush(void)
{
  if (xSemaphoreTake(synch, portMAX_DELAY) == pdTRUE)
  {
    consoleSendMessage();
    xSemaphoreGive(synch);
  }
}
