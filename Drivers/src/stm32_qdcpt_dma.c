/**
******************************************************************************
* @file    stm32_qdcpt_dma.c
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
#include "stm32_qdcpt_dma.h"

void STM_QDCPT_Dma_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    DMA_DeInit(DMA1_Channel4);//
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)SendBuff; //DMA
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//
    DMA_InitStructure.DMA_BufferSize = SENDBUFF_SIZE;//
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    //DMA_Mode_Normal, DMA_Mode_Circular
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);
}
