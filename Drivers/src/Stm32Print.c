/**
******************************************************************************
* @file    Stm32Print.c
* @author  Zhu Guohua
* @version V1.0
* @date    07-February-2015
* @brief   This file provides
*            - over the printf function
******************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2015 
******************************************************************************  
*/ 

/* Includes ------------------------------------------------------------------*/
#include "Stm32Print.h"
#include "stm32_qdcpt.h"

/**
* @brief  Retargets the C library printf function to the USART.
* @param  None
* @retval None
*/
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	USART_SendData(QDCPT_USART, (uint8_t) ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(QDCPT_USART, USART_FLAG_TC) == RESET)
	{}
	return ch;
}

/**
  * @brief  set scanf retarget to Uart
  * @param  *f: file poiter
  * @retval the input char
  */
int fgetc(FILE *f) 
{
    /* wait for the uart receive finish */
		while(USART_GetFlagStatus(QDCPT_USART,USART_FLAG_RXNE)==RESET)
		{}
	  /*return the receive value*/
    return USART_ReceiveData(QDCPT_USART); 
}
