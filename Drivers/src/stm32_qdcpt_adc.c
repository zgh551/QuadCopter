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
 * adc.c - Analog Digital Conversion
 *
 * TODO: Describe functionality.
 *
 * Sample time: According to the formula in the stm32 product manual
 *              page 69, with a Ts of 28.5 samples, 12-bit, and ADC@12
 *              the highest impedance to use is 25.2kOhm.
 */
#include "stm32f10x_conf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stm32_qdcpt_adc.h"
#include "pm.h"
//#include "nvicconf.h"
#include "imu.h"

#ifdef ADC_OUTPUT_RAW_DATA
#include "uart.h"
//#include "acc.h"
#endif

// PORT C
#define GPIO_VBAT        GPIO_Pin_5

// CHANNELS
#define NBR_OF_ADC_CHANNELS   1
#define CH_VBAT               ADC_Channel_15

static bool isInit;


volatile int16_t u16BatteryVoltageAdv = 0xAA;



static void adcDmaInit(void)
{
  DMA_InitTypeDef DMA_InitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  // DMA channel1 configuration
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)&(ADC1->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr        = (uint32_t)&u16BatteryVoltageAdv;
    DMA_InitStructure.DMA_DIR                   = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize            = 1; // NBR_OF_ADC_CHANNELS * (ADC_MEAN_SIZE * 2);
    DMA_InitStructure.DMA_PeripheralInc         = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc             = DMA_MemoryInc_Disable;                        // Enable
    DMA_InitStructure.DMA_PeripheralDataSize    = DMA_PeripheralDataSize_HalfWord;  // Word?
    DMA_InitStructure.DMA_MemoryDataSize        = DMA_MemoryDataSize_HalfWord;          // Word?
    DMA_InitStructure.DMA_Mode                  = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority              = DMA_Priority_High;                                // Very High
    DMA_InitStructure.DMA_M2M                   = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  // Enable DMA channel1
    DMA_Cmd(DMA1_Channel1, ENABLE);

}


void adcInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef  ADC_InitStructure;
      
  if (isInit)
    return;
    
    // Enable GPIOB and ADC1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1  | 
                           RCC_APB2Periph_GPIOC | 
                           RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    adcDmaInit();
    
    // ADC1 configuration
    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode                  = ADC_Mode_Independent;//工作在独立模式
    ADC_InitStructure.ADC_ScanConvMode          = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode    = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv      = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel          = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
		
    ADC_RegularChannelConfig(ADC1, CH_VBAT,1,ADC_SampleTime_239Cycles5);
    ADC_DMACmd(ADC1, ENABLE);

    // Enable ADC1
    ADC_Cmd(ADC1, ENABLE);
    adcDmaStart();
    
//    xTaskCreate(adcTask, (signed char*)"ADC",
//                configMINIMAL_STACK_SIZE, NULL, /*priority*/3, NULL);


  isInit = true;
}

bool adcTest(void)
{
  return isInit;
}

float adcConvertToVoltageFloat(uint16_t v, uint16_t vref)
{
  return (v / (vref / ADC_INTERNAL_VREF));
}

void adcDmaStart(void)
{
  // Enable the Transfer Complete and Half Transfer Interrupt
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT, ENABLE);
  // Enable ADC1 DMA
  ADC_DMACmd(ADC1, ENABLE);

}


void adcInterruptHandler(void)
{
/*  portBASE_TYPE xHigherPriorityTaskWoken;
  AdcGroup* adcBuffer;

  if(DMA_GetITStatus(DMA1_IT_HT1))
  {
    DMA_ClearITPendingBit(DMA1_IT_HT1);
    adcBuffer = (AdcGroup*)&adcValues[0];
    xQueueSendFromISR(adcQueue, &adcBuffer, &xHigherPriorityTaskWoken);
  }
  if(DMA_GetITStatus(DMA1_IT_TC1))
  {
    DMA_ClearITPendingBit(DMA1_IT_TC1);
    adcBuffer = (AdcGroup*)&adcValues[ADC_MEAN_SIZE];
    xQueueSendFromISR(adcQueue, &adcBuffer, &xHigherPriorityTaskWoken);
  }
  */
}

uint16_t adcGetBatteryAdv(void) 
{
    return u16BatteryVoltageAdv;  
}

uint16_t adcGetBatteryVol(void) 
{
    uint16_t u16BatVol = (uint16_t)(((uint32_t)u16BatteryVoltageAdv) * 330l * 2l / 4096l);
    uint16_t u16BatteryVol;
    u16BatteryVol = ((((u16BatVol / 100)  % 10) << 8) & 0xF00) | ((((u16BatVol / 10) % 10) << 4) & 0xF0) | ((u16BatVol % 10) & 0x0F);
     
}

float adcGetBatteryVolReal(void) 
{
    return (float)((u16BatteryVoltageAdv * 2.8f*3.0f) / (1 << 12)) ;
} 

void adcTask(void *param)
{
  float fBatteryVoltage = 0.0;
  
  vTaskSetApplicationTaskTag(0, (pdTASK_HOOK_CODE)TASK_ADC_ID_NBR);
  vTaskDelay(1000);

  adcDmaStart();

  while(1)
  {
    fBatteryVoltage = adcGetBatteryVolReal();
    pmBatteryUpdate(fBatteryVoltage);
    
  }
}

