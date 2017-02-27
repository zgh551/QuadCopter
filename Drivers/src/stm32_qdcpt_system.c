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
 #include "stm32_qdcpt_system.h"
 #include "stm32_qdcpt_led.h"
 #include "stm32_qdcpt_usart.h"
 #include "crtp.h"
 
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

/* The transmit task as described at the top of the file. */
static portTASK_FUNCTION_PROTO( vComTxTask, pvParameters );

/* The receive task as described at the top of the file. */
static portTASK_FUNCTION_PROTO( vComRxTask, pvParameters );

static volatile unsigned portBASE_TYPE uxRxLoops = comINITIAL_RX_COUNT_VALUE;

