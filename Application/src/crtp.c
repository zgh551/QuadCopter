 /**
 ******************************************************************************
 * @file    crtp.c
 * @author  Zhu Guohua
 * @version V1.0
 * @date    08-Merch-2015
 * @brief   This file provides
 *            - over the printf function
 ******************************************************************************
 * @attention
 * Quadcopter = qdcpt
 * COPYRIGHT 2015 
 ******************************************************************************  
 */
 /**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * crtp.c - CrazyRealtimeTransferProtocol stack
 */
#include <stdbool.h>

#include "errno.h"
         
/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "crtp.h"
#include "cfassert.h"
#include "ledseq.h"
#include "stabilizer.h"
static bool isInit;

static int nopFunc(void);
/*
static struct crtpLinkOperations nopLink = {
  .setEnable         = (void*) nopFunc,
  .sendPacket        = (void*) nopFunc,
  .receivePacket     = (void*) nopFunc,
}; 
*/
/*function  nop pointer structure to be filled by the CRTP link to permits CRTP use manu link*/
static struct crtpLinkOperations nopLink = 
{
  (int (*)(_Bool)) nopFunc,           //int (*setEnable)(bool enable);
  (int (*)(CRTPPacket *pk)) nopFunc,  //int (*sendPacket)(CRTPPacket *pk);
  (int (*)(CRTPPacket *pk)) nopFunc,  //int (*receivePacket)(CRTPPacket *pk);
  NULL,                               //bool (*isConnected)(void);
  NULL                                //int (*reset)(void);
}; 
/*fill the structure variable with the noplink*/
static struct crtpLinkOperations *link = &nopLink;

/*define the queue variable*/
static xQueueHandle  tmpQueue;
static xQueueHandle  rxQueue;

#define CRTP_NBR_OF_PORTS 16
#define CRTP_TX_QUEUE_SIZE 20
#define CRTP_RX_QUEUE_SIZE 2

static xQueueHandle  queues[CRTP_NBR_OF_PORTS];

static volatile CrtpCallback callbacks[CRTP_NBR_OF_PORTS];



/*function declare*/
static void crtpTxTask(void *param);
static void crtpRxTask(void *param);


/***************************************
 *@brief  initialise the crtp function 
 *@param  None
 *@retval None
 ***************************************/
void crtpInit(void)
{
  if(isInit)
    return;

  tmpQueue = xQueueCreate(CRTP_TX_QUEUE_SIZE, sizeof(CRTPPacket));
  //rxQueue  = xQueueCreate(CRTP_RX_QUEUE_SIZE, sizeof(CRTPPacket));
  /* Start Rx/Tx tasks */
  xTaskCreate(crtpTxTask, "CRTP-Tx",configMINIMAL_STACK_SIZE, NULL, /*priority*/2, NULL);
  xTaskCreate(crtpRxTask, "CRTP-Rx",configMINIMAL_STACK_SIZE, NULL, /*priority*/2, NULL);  
  //LedseqRun(LEDR, seq_testPassed);
	isInit = true;
}
/***************************************************
 *@brief test the function whether initialise or not
 *@retval the result of test 
 ***************************************************
 */
bool crtpTest(void)
{
  return isInit;
}

/***********************************************************
 *@brief initialise the receive queue and dispatch of a task
 *@param[in] portId:the id of the CRTP port
 *	this parameter can be one of following parameters
 *	@arg CRTP_PORT_CONSOLE     = 0x00,
 *	@arg CRTP_PORT_PARAM       = 0x02,
 *	@arg CRTP_PORT_COMMANDER   = 0x03,
 *	@arg CRTP_PORT_LOG         = 0x05,
 *	@arg CRTP_PORT_DEBUG       = 0x08,  //use for 24L01 debug
 *	@arg CRTP_PORT_LINK        = 0x0F,
 *@retval None
 ***********************************************************/
void crtpInitTaskQueue(CRTPPort portId)
{
  ASSERT(queues[portId] == NULL);
  
  queues[portId] = xQueueCreate(1, sizeof(CRTPPacket));
}
/********************************************************************************************************************
 *@brief  if queue is empty then immediately return and put the data of the queue using portid into the variable 'p'
 *@param[in]  portId:the id of the CRTP port
 *	this parameter can be one of following parameters
 *	@arg CRTP_PORT_CONSOLE     = 0x00,
 *	@arg CRTP_PORT_PARAM       = 0x02,
 *	@arg CRTP_PORT_COMMANDER   = 0x03,
 *	@arg CRTP_PORT_LOG         = 0x05,
 *	@arg CRTP_PORT_DEBUG       = 0x08,  //use for 24L01 debug
 *	@arg CRTP_PORT_LINK        = 0x0F,
 *@param[out] *p:The CRTP Packet with information
 *@retval status of fetch from queue
 *  @arg pdPASS:receive the data successfully
 *	@arg errQUEUE_FULL:if queue is full so that data can not write in
*********************************************************************************************************************/
int crtpReceivePacket(CRTPPort portId, CRTPPacket *p)
{
  ASSERT(queues[portId]);
  ASSERT(p);
  return xQueueReceive(queues[portId], p, 0);
}

/**************************************************************************************************
 *@brief  Wait for a packet to arrive for the specified taskID
 *@param[in] portId:The id of the CRTP port
 *	this parameter can be one of following parameters
 *	@arg CRTP_PORT_CONSOLE     = 0x00,
 *	@arg CRTP_PORT_PARAM       = 0x02,
 *	@arg CRTP_PORT_COMMANDER   = 0x03,
 *	@arg CRTP_PORT_LOG         = 0x05,
 *	@arg CRTP_PORT_DEBUG       = 0x08,  //use for 24L01 debug
 *	@arg CRTP_PORT_LINK        = 0x0F,
 *@param[out] *p:The CRTP Packet with information
 *@retval status of fetch from queue
 *  @arg pdPASS:receive the data successfully
 *	@arg errQUEUE_FULL:if queue is full so that data can not write in
*****************************************************************************************************/
int crtpReceivePacketBlock(CRTPPort portId, CRTPPacket *p)
{
  ASSERT(queues[portId]);
  ASSERT(p);
  
  return xQueueReceive(queues[portId], p, portMAX_DELAY);
}

/**********************************************************************************************************************
 *@brief  Fetch a packet with a special task ID. Wait some time before giving up
 *@param[in] portId: The id of the CRTP port
 *	this parameter can be one of following parameters
 *	@arg CRTP_PORT_CONSOLE     = 0x00,
 *	@arg CRTP_PORT_PARAM       = 0x02,
 *	@arg CRTP_PORT_COMMANDER   = 0x03,
 *	@arg CRTP_PORT_LOG         = 0x05,
 *	@arg CRTP_PORT_DEBUG       = 0x08,  //use for 24L01 debug
 *	@arg CRTP_PORT_LINK        = 0x0F,
 *@param[out] *p: The CRTP Packet with information (unchanged if nothing to fetch)
 *@param[in]  wait: Wait time in millisecond
 *@retval return whether receive queue is success or not
 *   @arg pdPASS:receive the data successfully
 *	  @arg errQUEUE_FULL:if queue is full so that data can not write in
***********************************************************************************************************************/
int crtpReceivePacketWait(CRTPPort portId, CRTPPacket *p, int wait) 
{
  ASSERT(queues[portId]);
  ASSERT(p);
  
  return xQueueReceive(queues[portId], p, M2T(wait));
}

/*****************************************************************************************************************
 *@brief transmit task:The transmit packet put into the temQueue,monitoring the temQueue,if have then transmit
 *@param *param:null pointer
 *@retval None
 *****************************************************************************************************************/
void crtpTxTask(void *param)
{
  CRTPPacket p;
  while (1)
  {
		//LedseqRun(LEDR, seq_testPassed);
    if (xQueueReceive(tmpQueue, &p, portMAX_DELAY) == pdTRUE)
    {
      link->sendPacket(&p);
    }
  }
}

/**
 *@breif receive task:
 *@param *param:null pointer 
 *@retval None
*/
void crtpRxTask(void *param)
{
  CRTPPacket p;
  static unsigned int droppedPacket=0;

  while (1)
  {
    if (!link->receivePacket(&p))//如果接收到数据
    {
			StationLink();//8.14
			if(queues[p.port])//判断port端口的队列是否生成
			{
					// TODO: If full, remove one packet and then send
					xQueueSend(queues[p.port], &p, 0);//将接收到的数据发送给queues[p.port]
				                                    //实际在PID控制时用了port =6发送pid值
			} 
			else 
			{
					droppedPacket++;
			}
      
      if(callbacks[p.port])//遥控器更新期望欧拉角用
        callbacks[p.port](&p);  //Dangerous?
//			  注册了两个函数，根据上位机发送来的数据来判断条用哪个
//			  crtpRegisterPortCB(CRTP_PORT_COMMANDER, commanderCrtpCB);
//        crtpRegisterPortCB(CRTP_PORT_DEBUG, mydebugCrtpCB);
//			  一个用于无线调试用，一个用于接收指令
    }
  }
}

/****************************************************************
 *@breif Register a callback to be called for a particular port.
 *@param[in] port:CRTP port for which the callback is set
 *@param[out] cb :Callback that will be called when a packet is received on 'port'.
 *@note Only one callback can be registered per port! The last callback registered will be the one called
 *@retval None 
 ****************************************************************/
void crtpRegisterPortCB(int port, CrtpCallback cb)
{
  if (port>CRTP_NBR_OF_PORTS)
    return;
  
  callbacks[port] = cb;
}

/**********************************************************************
 *@brief send the packet function:not waiting
 *@param *P:CRTPPacket pointer 
 *@retval return whether receive queue is successful or not
 *	@arg pdPASS:receive the data successfully
 *	@arg errQUEUE_FULL:if queue is full so that data can not write in
***********************************************************************/
int crtpSendPacket(CRTPPacket *p)
{
  ASSERT(p); 

  return xQueueSend(tmpQueue, ( void * )&p, 0);
}

/**********************************************************************
 *@brief send the packet function:block
 *@param *P:CRTPPacket pointer 
 *@retval return whether receive queue is successful or not
 *	@arg pdPASS:receive the data successfully
 *	@arg errQUEUE_FULL:if queue is full so that data can not write in
***********************************************************************/
int crtpSendPacketBlock(CRTPPacket *p)
{
  ASSERT(p); 

  return xQueueSend(tmpQueue,p, portMAX_DELAY);
}

/*********************************************************************
 *@brief  Reset the CRTP communication by flushing all the queues that
 *        contain packages.
 *@param  None
 *@retval return 0 for success
*/
int crtpReset(void)
{
  xQueueReset(tmpQueue);
  if (link->reset) 
  {
    link->reset();
  }
  return 0;
}
/**************************************************************
 *@brief Check if the connection timeout has been reached,
 *       otherwise we will assume that we are connected.
 *@param None
 *@retval return the true if connected,otherwise false
 **************************************************************/
bool crtpIsConnected(void)
{
  if (link->isConnected)
  return link->isConnected();
  return true;
}

/*****************************************************************
 *@brief send p to the rxQueue
 *@param *p:CRTPPort pointer
 *@retval None
 *****************************************************************/
//void crtpPacketReveived(CRTPPacket *p)
//{
//  portBASE_TYPE xHigherPriorityTaskWoken;

//  xHigherPriorityTaskWoken = pdFALSE;
//  xQueueSendFromISR(rxQueue, p, &xHigherPriorityTaskWoken);
//}

/***********************************************************
 *@brief set the below link
 *@param *lk:crtpLinkOperations pointer
 *@retval None
 ***********************************************************/
void crtpSetLink(struct crtpLinkOperations * lk)
{
  if(link)
    link->setEnable(false);

  if (lk)
    link = lk;
  else
    link = &nopLink;

  link->setEnable(true);
}

/*******************************
 *@brief None do function
 *@param None
 *@retval None
 *******************************/
static int nopFunc(void)
{
  return ENETDOWN;
}
