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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32_QDCPT_H
#define __STM32_QDCPT_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
	 
	typedef enum 
	{
		LEDL = 0,
		LEDR = 1,
		QDCPT_INVALID_LED
	} Led_TypeDef;	 
	 
/*define the numbers of led in the Quadcopter*/	 
#define LEDn                             2
/*left led*/	 
#define LEDL_PIN 	 								       GPIO_Pin_9
#define LEDL_GPIO_PORT                   GPIOB
#define LEDL_GPIO_CLK                    RCC_APB2Periph_GPIOB  	
/*ri*/
#define LEDR_PIN	 								       GPIO_Pin_8
#define LEDR_GPIO_PORT                   GPIOB
#define LEDR_GPIO_CLK                    RCC_APB2Periph_GPIOB

#define LED_GPIO_PORT										 GPIOB	
#define LED_GPIO_CLK                     RCC_APB2Periph_GPIOB
/*The define of USART in Quadcopter*/
//Numbers
#define USARTn													 1
#define QDCPT_USART											 USART1
#define QDCPT_USART_CLK                  RCC_APB2Periph_USART1
//TX	 
#define QDCPT_USART_TX_PIN               GPIO_Pin_9
#define QDCPT_USART_TX_GPIO_PORT         GPIOA
#define QDCPT_USART_TX_GPIO_CLK          RCC_APB2Periph_GPIOA
//RX
#define QDCPT_USART_RX_PIN               GPIO_Pin_10
#define QDCPT_USART_RX_GPIO_PORT         GPIOA
#define QDCPT_USART_RX_GPIO_CLK          RCC_APB2Periph_GPIOA
//IRQ
#define QDCPT_USART_IRQn                 USART1_IRQn
//USART1 at the DMA1 channel4 
#define QDCPT_USART_DMA_IRQ							 DMA1_Channel4_IRQn
#define QDCPT_USART_DMA_IT_TC            DMA1_IT_TC4
#define QDCPT_USART_DMA_CH               DMA1_Channel4    

//SPI
#define QDCPT_SPI                        SPI1
#define QDCPT_SPI_CLK                    RCC_APB2Periph_SPI1

#define QDCPT_SPI_CSN_PIN                GPIO_Pin_4                 /* PA.04 */
#define QDCPT_SPI_CSN_GPIO_PORT          GPIOA                       /* GPIOA */
#define QDCPT_SPI_CSN_GPIO_CLK           RCC_APB2Periph_GPIOA

#define QDCPT_SPI_SCK_PIN                GPIO_Pin_5                  /* PA.05 */
#define QDCPT_SPI_SCK_GPIO_PORT          GPIOA                       /* GPIOA */
#define QDCPT_SPI_SCK_GPIO_CLK           RCC_APB2Periph_GPIOA

#define QDCPT_SPI_MISO_PIN               GPIO_Pin_6                  /* PA.06 */
#define QDCPT_SPI_MISO_GPIO_PORT         GPIOA                       /* GPIOA */
#define QDCPT_SPI_MISO_GPIO_CLK          RCC_APB2Periph_GPIOA

#define QDCPT_SPI_MOSI_PIN               GPIO_Pin_7                  /* PA.07 */
#define QDCPT_SPI_MOSI_GPIO_PORT         GPIOA                       /* GPIOA */
#define QDCPT_SPI_MOSI_GPIO_CLK          RCC_APB2Periph_GPIOA

#define QDCPT_SPI_PORT                   GPIOA                       //All SPI Pin in GPIOA 

#define QDCPT_SPI_CE_PIN                 GPIO_Pin_1                  /* PB.01 */
#define QDCPT_SPI_CE_GPIO_PORT           GPIOB                       /* GPIOB */
#define QDCPT_SPI_CE_GPIO_CLK            RCC_APB2Periph_GPIOB

#define QDCPT_SPI_IRQ_PIN                GPIO_Pin_0                  /* PB.00 */
#define QDCPT_SPI_IRQ_GPIO_PORT          GPIOB                       /* GPIOB */
#define QDCPT_SPI_IRQ_GPIO_CLK           RCC_APB2Periph_GPIOB
#define QDCPT_SPI_IRQ_SRC_PORT					 GPIO_PortSourceGPIOB
#define QDCPT_SPI_IRQ_SRC_PIN						 GPIO_PinSource0
#define QDCPT_SPI_IRQ_LINE               EXTI_Line0
//I2C
#define QDCPT_I2C                         I2C2
#define QDCPT_I2C_CLK                     RCC_APB1Periph_I2C2

#define QDCPT_I2C_SCL_PIN                 GPIO_Pin_10                  /* PB.10 */
#define QDCPT_I2C_SCL_GPIO_PORT           GPIOB                       /* GPIOB */
#define QDCPT_I2C_SCL_GPIO_CLK            RCC_APB2Periph_GPIOB

#define QDCPT_I2C_SDA_PIN                 GPIO_Pin_11                  /* PB.11 */
#define QDCPT_I2C_SDA_GPIO_PORT           GPIOB                       /* GPIOB */
#define QDCPT_I2C_SDA_GPIO_CLK            RCC_APB2Periph_GPIOB

#define QDCPT_I2C_INT_PIN                 GPIO_Pin_13                  /* PB.13 */
#define QDCPT_I2C_INT_GPIO_PORT           GPIOB                       /* GPIOB */
#define QDCPT_I2C_INT_GPIO_CLK            RCC_APB2Periph_GPIOB

#define QDCPT_I2C_DR                      ((uint32_t)0x40005410)

//TIM
#define QDCPT_TIM													TIM2
#define QDCPT_TIM_CLK											RCC_APB1Periph_TIM2

#define QDCPT_TIM_PWM_GPIO_CLK            RCC_APB2Periph_GPIOA
#define QDCPT_TIM_PWM_GPIO_PORT           GPIOA 

#define QDCPT_TIM_PWM1_PIN                GPIO_Pin_0                  /* PA.00 */

#define QDCPT_TIM_PWM2_PIN                GPIO_Pin_1                  /* PA.01 */

#define QDCPT_TIM_PWM3_PIN                GPIO_Pin_2                  /* PA.02 */

#define QDCPT_TIM_PWM4_PIN                GPIO_Pin_3                  /* PA.03 */


//
void STM_QDCPT_LEDInit(Led_TypeDef Led);
void STM_QDCPT_USARTInit( USART_InitTypeDef* USART_InitStruct);

#ifdef __cplusplus
 }
#endif	 
 
#endif
