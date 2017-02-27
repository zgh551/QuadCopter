/**
******************************************************************************
* @file    stm32_qdcpt.c
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

#include "stm32_qdcpt.h"

GPIO_TypeDef* GPIO_PORT[LEDn] = {LEDL_GPIO_PORT, LEDR_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LEDL_PIN, LEDR_PIN};
const uint32_t GPIO_CLK[LEDn] = {LEDL_GPIO_CLK, LEDR_GPIO_CLK};

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg LEDL
  *     @arg LEDR
  * @retval None
  */
void STM_QDCPT_LEDInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(GPIO_CLK[Led], ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_PORT[Led], &GPIO_InitStructure);
}

/**
  * @brief  Configures USART1 port.
  * @param  COM: Specifies the COM port to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg COM1
  *     @arg COM2  
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
void STM_QDCPT_USARTInit(USART_InitTypeDef* USART_InitStruct)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(QDCPT_USART_TX_GPIO_CLK | QDCPT_USART_RX_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);


  /* Enable UART clock */
  RCC_APB2PeriphClockCmd(QDCPT_USART_CLK, ENABLE); 

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin   = QDCPT_USART_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(QDCPT_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin   = QDCPT_USART_RX_PIN;
  GPIO_Init(QDCPT_USART_RX_GPIO_PORT, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(QDCPT_USART, USART_InitStruct);
    
  /* Enable USART */
  USART_Cmd(QDCPT_USART, ENABLE);
}
