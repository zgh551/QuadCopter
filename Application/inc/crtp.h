 /**
 ******************************************************************************
 * @file    crtp.h
 * @author  Zhu Guohua
 * @version V1.0
 * @date    08-March-2015
 * @brief   This file provides
 *            - over the printf function
 ******************************************************************************
 * @attention
 * Quadcopter = qdcpt
 * COPYRIGHT 2015 
 ******************************************************************************  
 */
 #ifndef __CRTP_H
 #define __CRTP_H
 
 #ifdef __cplusplus
	extern C{
 #endif
	#include "stm32f10x.h"
	#include <stdbool.h>
	#include <stdint.h>
		
	#define CRTP_MAX_DATA_SIZE 30

	#define CRTP_HEADER(port, channel) (((port & 0x0F) << 4) | (channel & 0x0F))
	#define CRTP_GET_NBR(port) (port)
//#define CRTP_PORT()

#define CRTP_IS_NULL_PACKET(P) ((P.header&0xF3)==0xF3)

typedef enum CRTPPort{
  CRTP_PORT_CONSOLE     = 0x00,
  CRTP_PORT_PARAM       = 0x02,
  CRTP_PORT_COMMANDER   = 0x03,
  CRTP_PORT_LOG         = 0x05,
	CRTP_PORT_PID         = 0X06,
  CRTP_PORT_DEBUG       = 0x08,  //ÓÃÓÚ24L01µ÷ÊÔ
  CRTP_PORT_LINK        = 0x0F,
}CRTPPort;

//add by shiqingziyang 2014.04.02
#ifdef KEIL_PROJECT
#pragma anon_unions
#endif

typedef struct _CRTPPacket
{
  uint8_t size;
   __packed union {
     __packed struct {
       __packed union {
        uint8_t header;
         __packed struct {
#ifndef CRTP_HEADER_COMPAT
          uint8_t channel     : 2;
          uint8_t reserved    : 2;
          uint8_t port        : 4;
#else
          uint8_t channel  : 2;
          uint8_t port     : 4;
          uint8_t reserved : 2;
#endif
        };
      };
      uint8_t data[CRTP_MAX_DATA_SIZE];
    };
    uint8_t raw[CRTP_MAX_DATA_SIZE+2];
  };
}CRTPPacket;// __attribute__((packed)) CRTPPacket;

typedef void (*CrtpCallback)(CRTPPacket *);
/**
 * Initialize the CRTP stack
 */
void crtpInit(void);

bool crtpTest(void);

/**
 * Initializes the queue and dispatch of an task.
 *
 * @param[in] taskId The id of the CRTP task
 */
void crtpInitTaskQueue(CRTPPort taskId);

/**
 * Register a callback to be called for a particular port.
 *
 * @param[in] port Crtp port for which the callback is set
 * @param[in] cb Callback that will be called when a packet is received on
 *            'port'.
 *
 * @note Only one callback can be registered per port! The last callback
 *       registered will be the one called
 */
void crtpRegisterPortCB(int port, CrtpCallback cb);

/**
 * Put a packet in the TX task
 *
 * If the TX stack is full, the oldest lowest priority packet is dropped
 *
 * @param[in] p CRTPPacket to send
 */
int crtpSendPacket(CRTPPacket *p);

/**
 * Put a packet in the TX task
 *
 * If the TX stack is full, the function block until one place is free (Good for console implementation)
 */
int crtpSendPacketBlock(CRTPPacket *p);

/**
 * Fetch a packet with a specidied task ID.
 *
 * @param[in]  taskId The id of the CRTP task
 * @param[out] p      The CRTP Packet with infomation (unchanged if nothing to fetch)
 *
 * @returns status of fetch from queue
 */
int crtpReceivePacket(CRTPPort taskId, CRTPPacket *p);

/**
 * Fetch a packet with a specidied task ID. Wait some time befor giving up
 *
 * @param[in]  taskId The id of the CRTP task
 * @param[out] p      The CRTP Packet with infomation (unchanged if nothing to fetch)
 * @param[in] wait    Wait time in milisecond
 *
 * @returns status of fetch from queue
 */
int crtpReceivePacketWait(CRTPPort taskId, CRTPPacket *p, int wait);

/**
 * Wait for a packet to arrive for the specified taskID
 *
 * @param[in]  taskId The id of the CRTP task
 * @paran[out] p      The CRTP Packet with information
 *
 * @return status of fetch from queue
 */
int crtpReceivePacketBlock(CRTPPort taskId, CRTPPacket *p);

void crtpPacketReveived(CRTPPacket *p);

/**
 * Function pointer structure to be filled by the CRTP link to permits CRTP to
 * use manu link
 */
struct crtpLinkOperations
{
  int (*setEnable)(bool enable);
  int (*sendPacket)(CRTPPacket *pk);
  int (*receivePacket)(CRTPPacket *pk);
  bool (*isConnected)(void);
  void (*reset)(void);
};

void crtpSetLink(struct crtpLinkOperations * lk);

/**
 * Check if the connection timeout has been reached, otherwise
 * we will assume that we are connected.
 *
 * @return true if conencted, otherwise false
 */
bool crtpIsConnected(void);

/**
 * Reset the CRTP communication by flushing all the queues that
 * contain packages.
 *
 * @return 0 for success
 */
int crtpReset(void);
 #ifdef __cplusplus
	}
 #endif
 #endif
