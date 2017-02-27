/**
******************************************************************************
* @file    stm32_qdcpt_usart.c
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
#include <string.h>
#include <stdbool.h>
/* Demo application includes. */
#include "stm32_qdcpt.h"
#include "stm32_qdcpt_usart.h"
#include "stm32_qdcpt_led.h"
#include "ledseq.h"
#include "nvic_configure.h"
#include "cfassert.h"
/* Misc defines. */
#define serINVALID_QUEUE				( ( xQueueHandle ) 0 )
#define serNO_BLOCK						  ( ( portTickType ) 0 )
#define serTX_BLOCK_TIME				( 40 / portTICK_RATE_MS )
#define uxQueueLength           40
/*-----------------------------------------------------------*/
#define CCR_ENABLE_SET  ((uint32_t)0x00000001)
#define CRTP_FIRST_BYTE  0xAA
#define CRTP_SECOND_BYTE 0x55
#define UART_DATA_TIMEOUT_MS 1000
#define UART_DATA_TIMEOUT_TICKS (UART_DATA_TIMEOUT_MS / portTICK_RATE_MS)
/* The queue used to hold received characters. */
static xQueueHandle xRxedChars;
static xQueueHandle xCharsForTx;

static xQueueHandle packetDelivery;
static xQueueHandle uartDataDelivery;

xSemaphoreHandle waitUntilSendDone = NULL;

static bool    isUartDmaInitialized;

static uint8_t outBuffer[64];
static uint8_t dataIndex;
static uint8_t dataSize;
static uint8_t crcIndex = 0;
static enum { notSentSecondStart, sentSecondStart} txState;
static DMA_InitTypeDef DMA_InitStructureShare;
enum {waitForFirstStart, waitForSecondStart,
        waitForPort, waitForSize, waitForData, waitForCRC } rxState;
uint8_t c;
CRTPPacket p;
static CRTPPacket pack;


 #define comSTACK_SIZE				configMINIMAL_STACK_SIZE
 #define comTX_LED_OFFSET			( 0 )
 #define comRX_LED_OFFSET			( 1 )
 #define comTOTAL_PERMISSIBLE_ERRORS ( 2 )

/* The Tx task will transmit the sequence of characters at a pseudo random
interval.  This is the maximum and minimum block time between sends. */
#define comTX_MAX_BLOCK_TIME		( ( portTickType ) 0x96 )
#define comTX_MIN_BLOCK_TIME		( ( portTickType ) 0x32 )
#define comOFFSET_TIME				( ( portTickType ) 3 )

/* We should find that each character can be queued for Tx immediately and we
don't have to block to send. */
#define comNO_BLOCK					( ( portTickType ) 0 )

/* The Rx task will block on the Rx queue for a long period. */
#define comRX_BLOCK_TIME			( ( portTickType ) 0xffff )

#define comBUFFER_LEN				( ( unsigned portBASE_TYPE ) ( comLAST_BYTE - comFIRST_BYTE ) + ( unsigned portBASE_TYPE ) 1 )
#define comINITIAL_RX_COUNT_VALUE	( 0 )

//receive byte 
static uint8_t RxtoTx;
/* Handle to the com port used by both tasks. */
static xComPortHandle xPort = NULL;

///* The transmit task as described at the top of the file. */
//static portTASK_FUNCTION_PROTO( vComTxTask, pvParameters );

///* The receive task as described at the top of the file. */
//static portTASK_FUNCTION_PROTO( vComRxTask, pvParameters );

static volatile unsigned portBASE_TYPE uxRxLoops = comINITIAL_RX_COUNT_VALUE;

/**
  * @brief  Init the USART1.not the DMA way
  * @param  None
  * @retval None
  */
void QdcptUsartInit(void)
{
	USART_InitTypeDef USART_InitStructure;
	
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	vSemaphoreCreateBinary(waitUntilSendDone);
	STM_QDCPT_USARTInit(&USART_InitStructure);
}

void QdcptUsartDmaInit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  // USART TX DMA Channel Config
	//APB2 DMA1 Channel 4
  DMA_InitStructureShare.DMA_PeripheralBaseAddr = (uint32_t)&QDCPT_USART->DR;
  DMA_InitStructureShare.DMA_MemoryBaseAddr = (uint32_t)outBuffer;
  DMA_InitStructureShare.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructureShare.DMA_BufferSize = 0;
  DMA_InitStructureShare.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructureShare.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructureShare.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructureShare.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  DMA_InitStructureShare.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructureShare.DMA_Priority = DMA_Priority_High;
  DMA_InitStructureShare.DMA_M2M = DMA_M2M_Disable;

  NVIC_InitStructure.NVIC_IRQChannel = QDCPT_USART_DMA_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  isUartDmaInitialized = 1;
}

void uartRxTask(void *param)
{

}

int uartReceiveCRTPPacket(CRTPPacket *p)
{
  if (xQueueReceive(packetDelivery, p, 0) == pdTRUE)
  {
    return 0;
  }

  return -1;
}

int uartSendCRTPPacket(CRTPPacket *p)
{
  outBuffer[0] = CRTP_FIRST_BYTE;
  outBuffer[1] = CRTP_SECOND_BYTE;
  outBuffer[2] = p->header;
  outBuffer[3] = p->size;

  memcpy(&outBuffer[4], p->raw, p->size);
  dataIndex = 1;
  txState = notSentSecondStart;
  dataSize = p->size + 5;
  crcIndex = dataSize - 1;
  outBuffer[crcIndex] = 0;
	
	
  USART_SendData(QDCPT_USART, outBuffer[0] & 0xFF);
  USART_ITConfig(QDCPT_USART, USART_IT_TXE, ENABLE);
  xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
  
  return 0;
}

static int uartSetEnable(bool enable)
{
  return 0;
}

static struct crtpLinkOperations uartOp =
{
  uartSetEnable,             //int (*setEnable)(bool enable);
  uartSendCRTPPacket,        //int (*sendPacket)(CRTPPacket *pk);
  uartReceiveCRTPPacket,     //int (*receivePacket)(CRTPPacket *pk);
  NULL,                      //bool (*isConnected)(void);
  NULL                       //int (*reset)(void);
};

struct crtpLinkOperations * uartGetLink(void)
{
  return &uartOp;
}

void uartDmaIsr(void)
{
  DMA_ITConfig(QDCPT_USART_DMA_CH, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(QDCPT_USART_DMA_IT_TC);
  USART_DMACmd(QDCPT_USART, USART_DMAReq_Tx, DISABLE);
  DMA_Cmd(QDCPT_USART_DMA_CH, DISABLE);
}

void uartSendData(uint32_t size, uint8_t* data)
{
  uint32_t i;
  for(i = 0; i < size; i++)
  {
    while (!(QDCPT_USART->SR & USART_FLAG_TXE));
    QDCPT_USART->DR = (data[i] & 0xFF);
  }
}

int uartPutchar(int ch)
{
   uartSendData(1, (uint8_t *)&ch);  
   return (unsigned char)ch;
}

void uartSendDataDma(uint32_t size, uint8_t* data)
{
  if (isUartDmaInitialized)
  {
    memcpy(outBuffer, data, size);
    DMA_InitStructureShare.DMA_BufferSize = size;
    // Wait for DMA to be free
    while(QDCPT_USART_DMA_CH->CCR & CCR_ENABLE_SET);
    DMA_Init(QDCPT_USART_DMA_CH, &DMA_InitStructureShare);
    // Enable the Transfer Complete interrupt
    DMA_ITConfig(QDCPT_USART_DMA_CH, DMA_IT_TC, ENABLE);
    USART_DMACmd(QDCPT_USART, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(QDCPT_USART_DMA_CH, ENABLE);
  }
}

/****************************************************************/
xComPortHandle QdcptUsartNvicInit(void)
{
	xComPortHandle xReturn;
	
	GPIO_InitTypeDef  GPIO_InitStructure;           
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	/* Create the queues used to hold Rx/Tx characters. */
	xRxedChars  = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
	xCharsForTx = xQueueCreate( uxQueueLength + 1, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
	packetDelivery = xQueueCreate(2, sizeof(CRTPPacket));
	//binary value
	vSemaphoreCreateBinary(waitUntilSendDone);
	
  /* If the queue/semaphore was created correctly then setup the serial port
	hardware. */
	if( ( xRxedChars != serINVALID_QUEUE ) && ( xCharsForTx != serINVALID_QUEUE ) )
	{	
		//RCC ENABLE
		RCC_APB2PeriphClockCmd(QDCPT_USART_TX_GPIO_CLK | QDCPT_USART_RX_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
		RCC_APB2PeriphClockCmd(QDCPT_USART_CLK, ENABLE);
		//GPIO config
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			
		GPIO_InitStructure.GPIO_Pin  = QDCPT_USART_TX_PIN;                                      
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                              
		GPIO_Init(QDCPT_USART_TX_GPIO_PORT, &GPIO_InitStructure);
			 
		GPIO_InitStructure.GPIO_Pin  = QDCPT_USART_RX_PIN;                                      
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		/**initialise the GPIO about the USART1*/
		GPIO_Init(QDCPT_USART_RX_GPIO_PORT, &GPIO_InitStructure);
			
		//SET USART
		USART_InitStructure.USART_BaudRate   					= 115200;                                    
		USART_InitStructure.USART_WordLength 					= USART_WordLength_8b;                     
		USART_InitStructure.USART_StopBits   					= USART_StopBits_1;                          
		USART_InitStructure.USART_Parity     					= USART_Parity_No;                             
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
		USART_InitStructure.USART_Mode 								= USART_Mode_Rx | USART_Mode_Tx;                 
		//Initialise the USART1
		USART_Init(QDCPT_USART, &USART_InitStructure);

		//Enable the interrupt and set the priority
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		NVIC_InitStructure.NVIC_IRQChannel = QDCPT_USART_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority= 7;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		//initialise the nvic
		NVIC_Init(&NVIC_InitStructure);

		USART_ITConfig(QDCPT_USART, USART_IT_RXNE, ENABLE);
		USART_Cmd(QDCPT_USART, ENABLE);
	}
	else
	{
		xReturn = ( xComPortHandle ) 0;
	}
	
	
	/* This demo file only supports a single port but we have to return
	something to comply with the standard demo header file. */
	return xReturn;
}


/**
	@breif use this function to program the queue data 
	@param none
*/
void uartIsr( void )
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
portCHAR rxDataInterrupt;

	if( USART_GetITStatus( QDCPT_USART, USART_IT_TXE ) == SET )//if one byte transmit finished
	{
		if (dataIndex < dataSize)
    {
      USART_SendData(QDCPT_USART, outBuffer[dataIndex] & 0xFF);
      dataIndex++;
      if (dataIndex < dataSize - 1 && dataIndex > 1)
      {
        outBuffer[crcIndex] = (outBuffer[crcIndex] + outBuffer[dataIndex]) % 0xFF;
      }
    }
    else
    {
      USART_ITConfig(QDCPT_USART, USART_IT_TXE, DISABLE);
      xHigherPriorityTaskWoken = pdFALSE;
      xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
    }
	}
	USART_ClearITPendingBit(QDCPT_USART,USART_IT_TXE);
	//USART_ClearFlag(QDCPT_USART,USART_FLAG_TXE);
	
	if( USART_GetITStatus( QDCPT_USART, USART_IT_RXNE ) == SET )
	{
		rxDataInterrupt = USART_ReceiveData( QDCPT_USART )&0xff;
		//USART_SendData(QDCPT_USART,cChar);
		xQueueSendFromISR( xRxedChars, &rxDataInterrupt, &xHigherPriorityTaskWoken );
	}	
	//portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*********************TX FUNCTION*****************************
	@breif define the transition fuction
	@param task name and the param
	@retun none 
**************************************************************/
/*define the transition fuction*/
static portTASK_FUNCTION( QdcptTxTask, pvParameters )
{
	uint8_t i=0;
	/*define the count time*/
  portTickType xLastWakeTime;
	/*get the last time tick*/
	xLastWakeTime = xTaskGetTickCount();
	/* Just to stop compiler warnings. */
	( void ) pvParameters;

	for( ;; )
	{
		pack.header=0xa5;
		pack.size=10;
		for(i=0;i<10;i++)
		{pack.data[i]=i;}
//uartReceiveCRTPPacket(&pack);
		uartSendCRTPPacket( &pack );
		vTaskDelayUntil( &xLastWakeTime,(200/portTICK_RATE_MS) );
	}
}

/*********************RX FUNCTION*****************************
	@breif define the receive fuction
	@param task name and the param
	@retun none 
**************************************************************/
static portTASK_FUNCTION( QdcptRxTask, pvParameters )
{

  
  uint8_t dataIndex = 0;
  uint8_t crc = 0; 
  uint8_t counter = 0;
  rxState = waitForFirstStart; 
	( void ) pvParameters;
  //CRTPpackt AA 55 header size data[size]  CRC
  for( ;; )
  {
		//xRxedChars -> c
    if (xQueueReceive(xRxedChars, &c, UART_DATA_TIMEOUT_TICKS) == pdTRUE)
    {
			//LedseqRun(LEDL,seq_testPassed);
      //counter++;
     /* if (counter > 4)
        ledSetRed(1);*/
      switch(rxState)
      {
        case waitForFirstStart:
          rxState = (c == CRTP_FIRST_BYTE) ? waitForSecondStart : waitForFirstStart;
				  //LedseqRun(LEDL,seq_testPassed);
          break;
        case waitForSecondStart:
          rxState = (c == CRTP_SECOND_BYTE) ? waitForPort : waitForFirstStart;
					//LedseqRun(LEDR,seq_testPassed);
				break;
        case waitForPort:
          p.header = c;
          crc = c;
          rxState = waitForSize;
          break;
        case waitForSize:
          if (c < CRTP_MAX_DATA_SIZE)
          {
            p.size = c;
            crc = (crc + c) % 0xFF;
            dataIndex = 0;
            rxState = (c > 0) ? waitForData : waitForCRC;
          }
          else
          {
            rxState = waitForFirstStart;
          }
          break;
        case waitForData:
          p.data[dataIndex] = c;
          crc = (crc + c) % 0xFF;
          dataIndex++;
          if (dataIndex == p.size)
          {
            rxState = waitForCRC;
          }
          break;
        case waitForCRC:
          if (crc == c)
          {
            xQueueSend(packetDelivery, &p, 0);
          }
          rxState = waitForFirstStart;
          break;
        default:
          ASSERT(0);
          break;
      }
    }
    else
    {
      // Timeout
      //rxState = waitForFirstStart;
    }
  }
}
/********************************TASK CREATED***********************************
	@breif initialise the usart and creat the task
	@param set the priority of task
********************************************************************************/
void Qdcpt_Usart_Tasks( unsigned portBASE_TYPE uxPriority )
{
	/* Initialise the com port then spawn the Rx and Tx tasks. */
	//QdcptUsartInit();
	//QdcptUsartNvicInit();
	/* The Tx task is spawned with a lower priority than the Rx task. */
	
	//xTaskCreate( QdcptTxTask, ( signed char * ) "COMTx", comSTACK_SIZE, NULL, uxPriority-1 ,( xTaskHandle * ) NULL );
	xTaskCreate( QdcptRxTask, ( signed char * ) "COMRx", comSTACK_SIZE, NULL, uxPriority , ( xTaskHandle * ) NULL );
}
