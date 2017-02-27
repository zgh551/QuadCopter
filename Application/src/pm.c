 /**
 ******************************************************************************
 * @file    stm32_qdcpt.h
 * @author  Zhu Guohua
 * @version V1.0
 * @date    07-March-2015
 * @brief   This file provides
 *            - over the printf function
 ******************************************************************************
 * @attention
 * Quadcopter = qdcpt
 * COPYRIGHT 2015 
 ******************************************************************************  
 * pm.c - Power Management driver and functions.
 */

#include "stm32f10x_conf.h"
#include <string.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "system.h"
#include "pm.h"
#include "stm32_qdcpt_led.h"
#include "stm32_qdcpt_adc.h"
#include "ledseq.h"
#include "commander.h"
#include "radiolink.h"
#include "fp16.h"


float    batteryVoltage;
static float    batteryVoltageMin = 6.0;
static float    batteryVoltageMax = 0.0;
//static int32_t  batteryVRawFilt = PM_BAT_ADC_FOR_3_VOLT;
//static int32_t  batteryVRefRawFilt = PM_BAT_ADC_FOR_1p2_VOLT;
static uint32_t batteryLowTimeStamp;
static uint32_t batteryCriticalLowTimeStamp;
static bool isInit;
PMStates pmState;
static PMChargeStates pmChargeState;

extern CRTPPacket EulerData;
float fBatteryVoltage = 0.0;

void pmInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  if(isInit)
    return;

  RCC_APB2PeriphClockCmd(PM_GPIO_IN_PGOOD_PERIF | PM_GPIO_IN_CHG_PERIF |
                         PM_GPIO_SYSOFF_PERIF | PM_GPIO_EN1_PERIF | 
                         PM_GPIO_EN2_PERIF | PM_GPIO_BAT_PERIF, ENABLE);

  // Configure PM PGOOD pin (Power good)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Pin = PM_GPIO_IN_PGOOD;
  GPIO_Init(PM_GPIO_IN_PGOOD_PORT, &GPIO_InitStructure);
  // Configure PM CHG pin (Charge)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Pin = PM_GPIO_IN_CHG;
  GPIO_Init(PM_GPIO_IN_CHG_PORT, &GPIO_InitStructure);
  // Configure PM EN2 pin
  GPIO_InitStructure.GPIO_Pin = PM_GPIO_EN2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(PM_GPIO_EN2_PORT, &GPIO_InitStructure);
  // Configure PM EN1 pin
  GPIO_InitStructure.GPIO_Pin = PM_GPIO_EN1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(PM_GPIO_EN1_PORT, &GPIO_InitStructure);
  // Configure PM SYSOFF pin
  GPIO_InitStructure.GPIO_Pin = PM_GPIO_SYSOFF;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(PM_GPIO_SYSOFF_PORT, &GPIO_InitStructure);
  // Configure battery ADC pin
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Pin = PM_GPIO_BAT;
  GPIO_Init(PM_GPIO_BAT_PORT, &GPIO_InitStructure);

	GPIO_ResetBits(PM_GPIO_SYSOFF_PORT, PM_GPIO_SYSOFF);
	
  xTaskCreate(pmTask, (signed char*)"PWRMGNT",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/2, NULL);
  
  isInit = true;
}

bool pmTest(void)
{
  return isInit;
}

/**
 * IIR low pass filter the samples.
 */
static int16_t pmBatteryIIRLPFilter(uint16_t in, int32_t* filt)
{
  int32_t inScaled;
  int32_t filttmp = *filt;
  int16_t out;

  // Shift to keep accuracy
  inScaled = in << PM_BAT_IIR_SHIFT;
  // Calculate IIR filter
  filttmp = filttmp + (((inScaled-filttmp) >> 8) * PM_BAT_IIR_LPF_ATT_FACTOR);
  // Scale and round
  out = (filttmp >> 8) + ((filttmp & (1 << (PM_BAT_IIR_SHIFT - 1))) >> (PM_BAT_IIR_SHIFT - 1));
  *filt = filttmp;

  return out;
}

/**
 * Sets the battery voltage and its min and max values
 */
static void pmSetBatteryVoltage(float voltage)
{
  batteryVoltage = voltage;
  if (batteryVoltageMax < voltage)
  {
    batteryVoltageMax = voltage;
  }
  if (batteryVoltageMin > voltage)
  {
    batteryVoltageMin = voltage;
  }
}

/**
 * Shutdown system
 */
static void pmSystemShutdown(void)
{
#ifdef ACTIVATE_AUTO_SHUTDOWN
  GPIO_SetBits(PM_GPIO_SYSOFF_PORT, PM_GPIO_SYSOFF);
#endif
	LedseqRun(LEDL, seq_lowbat);
}



float pmGetBatteryVoltage(void)
{
  return batteryVoltage;
}

void pmSetChargeState(PMChargeStates chgState)
{
  pmChargeState = chgState;

  switch (chgState)
  {
    case charge100mA:
      GPIO_ResetBits(PM_GPIO_EN1_PORT, PM_GPIO_EN1);
      GPIO_ResetBits(PM_GPIO_EN2_PORT, PM_GPIO_EN2);
      break;
    case charge500mA:
      GPIO_SetBits(PM_GPIO_EN1_PORT, PM_GPIO_EN1);
      GPIO_ResetBits(PM_GPIO_EN2_PORT, PM_GPIO_EN2);
      break;
    case chargeMax:
      GPIO_ResetBits(PM_GPIO_EN1_PORT, PM_GPIO_EN1);
      GPIO_SetBits(PM_GPIO_EN2_PORT, PM_GPIO_EN2);
      break;
  }
}

void pmBatteryUpdate(float adcValues)
{
  pmSetBatteryVoltage(adcValues);
}


PMStates pmUpdateState(void)
{
  PMStates state;

  uint32_t batteryLowTime;

  batteryLowTime = xTaskGetTickCount() - batteryLowTimeStamp;


  if (batteryLowTime > PM_BAT_LOW_TIMEOUT)
  {
    state = lowPower;
  }
  else
  {
    state = battery;
  }

  return state;
}


void pmTask(void *param)
{
  PMStates pmStateOld = lowPower;
  uint32_t tickCount;
  
  
  vTaskSetApplicationTaskTag(0, (pdTASK_HOOK_CODE)TASK_PM_ID_NBR);

  tickCount = xTaskGetTickCount();
  batteryLowTimeStamp = tickCount;
  batteryCriticalLowTimeStamp = tickCount;

  pmSetChargeState(charge500mA);

  vTaskDelay(1000);

  while(1)
  {
    vTaskDelay(100);
    tickCount = xTaskGetTickCount();
    batteryVoltage = adcGetBatteryVolReal();
    pmBatteryUpdate(batteryVoltage);

    if (pmGetBatteryVoltage() > PM_BAT_LOW_VOLTAGE)
    {
      batteryLowTimeStamp = tickCount;
    }
    if (pmGetBatteryVoltage() > PM_BAT_CRITICAL_LOW_VOLTAGE)
    {
      batteryCriticalLowTimeStamp = tickCount;
    }

    pmState = pmUpdateState();

    if (pmState != pmStateOld)
    {
      // Actions on state change
      switch (pmState)
      {
        case lowPower:
          LedseqRun(LEDL, seq_armed);
          systemSetCanFly(false);
          break;
        case battery:
          LedseqRun(LEDL, seq_alive);
          systemSetCanFly(true);
          //Due to voltage change radio must be restarted
//          radiolinkReInit();
          break;
        default:
          systemSetCanFly(false);
          break;
      }
      pmStateOld = pmState;
    }
    // Actions during state
    switch (pmState)
    {
      case lowPower:
        {
          uint32_t batteryCriticalLowTime;

          batteryCriticalLowTime = tickCount - batteryCriticalLowTimeStamp;
          if (batteryCriticalLowTime > PM_BAT_CRITICAL_LOW_TIMEOUT)
          {
            pmSystemShutdown();
          }
        }
        break;
      case battery:
        {
          if ((commanderGetInactivityTime() > PM_SYSTEM_SHUTDOWN_TIMEOUT))
          {
            pmSystemShutdown();
          }
        }
        break;
      default:
        break;
    }
  }
}

/*
LOG_GROUP_START(pm)
LOG_ADD(LOG_FLOAT, vbat, &batteryVoltage)
LOG_ADD(LOG_INT8, state, &pmState)
LOG_GROUP_STOP(pm)
*/
