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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32PRINT_H
#define __STM32PRINT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f10x.h"
//#include "stm32_eval.h"
	 
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
	 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/** @defgroup STM32PRINT_LEVEL_Exported_Functions
  * @{
  */ 
PUTCHAR_PROTOTYPE;
int fgetc(FILE *f);
/**
  * @}
  */ 	 
	 
#ifdef __cplusplus
 }
#endif
#endif /* __STM32PRINT_H */
