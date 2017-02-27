/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 *
 */
#include "stm32f10x_conf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "crc.h"

#include "commander.h"
#include "crtp.h"
//#include "configblock.h"
//#include "param.h"
#include "ledseq.h"

#define MIN_THRUST  0       //10000
#define MAX_THRUST  65000

struct CommanderCrtpValues
{
	uint16_t thrust;// 推力
  int8_t roll;    // 倾斜角，飞机有时候斜着飞，一个翅膀指天一个翅膀指地，这就是说用90度倾斜角飞行
  int8_t pitch;   // 仰角，就是飞机的俯视角，一般说飞机俯冲或者拉升多少角度，都是说的仰角。
  int8_t yaw;     // 偏角，就是指相对于地磁北极的水平偏角，决定飞机飞向哪个方向             
} ;//__attribute__((packed));

static struct CommanderCrtpValues targetVal[2];
static struct CommanderCrtpValues ActualVal[2];

static bool isInit;
static int  side=0;
static uint32_t lastUpdate;
static bool isInactive;
static bool altHoldMode    = false;
static bool altHoldModeOld = false;

static void commanderCrtpCB(CRTPPacket* pk);
static void paramCrtpCB(CRTPPacket* pk);
static void commanderWatchdogReset(void);

static void mydebugCrtpCB(CRTPPacket* pk)
{
//    LedseqRun(LEDL, seq_testPassed);
}



void commanderInit(void)
{
  if(isInit)
    return;

  crtpInit();
	
	crtpInitTaskQueue(CRTP_PORT_COMMANDER);
	crtpInitTaskQueue(CRTP_PORT_DEBUG);
	crtpInitTaskQueue(CRTP_PORT_PARAM);
	
  crtpRegisterPortCB(CRTP_PORT_COMMANDER, commanderCrtpCB);
	crtpRegisterPortCB(CRTP_PORT_PARAM, paramCrtpCB);
  crtpRegisterPortCB(CRTP_PORT_DEBUG, mydebugCrtpCB);

  lastUpdate = xTaskGetTickCount();
  isInactive = TRUE;
  isInit = TRUE;
}

bool commanderTest(void)
{
  crtpTest();
  return isInit;
}

static void commanderCrtpCB(CRTPPacket* pk)
{
  targetVal[!side] = *((struct CommanderCrtpValues*)pk->data);
  side = !side;
  commanderWatchdogReset();
}

static void paramCrtpCB(CRTPPacket* pk)
{
  altHoldMode = (bool)pk->data[0];
	Stm32QdcptLedToggle(LEDL);
  commanderWatchdogReset();
}

void commanderWatchdog(void)
{
  int usedSide = side;
  uint32_t ticktimeSinceUpdate;

  ticktimeSinceUpdate = xTaskGetTickCount() - lastUpdate;

  if (ticktimeSinceUpdate > COMMANDER_WDT_TIMEOUT_STABALIZE)
  {
    targetVal[usedSide].roll = 0;
    targetVal[usedSide].pitch = 0;
    targetVal[usedSide].yaw = 0;
  }
  if (ticktimeSinceUpdate > COMMANDER_WDT_TIMEOUT_SHUTDOWN)
  {
    targetVal[usedSide].thrust = 0;
    altHoldMode = false; // do we need this? It would reset the target altitude upon reconnect if still hovering
    isInactive = TRUE;
  }
  else
  {
    isInactive = FALSE;
  }
}

static void commanderWatchdogReset(void)
{
  lastUpdate = xTaskGetTickCount();
}

uint32_t commanderGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}



void commanderGetAltHold(bool* altHold, bool* setAltHold, float* altHoldChange)
{
  *altHold = altHoldMode; // Still in altitude hold mode
  *setAltHold = !altHoldModeOld && altHoldMode; // Hover just activated
  *altHoldChange = altHoldMode ? ((float) targetVal[side].thrust - 32767.) / 32767. : 0.0; //32767 Amount to change altitude hold target
  altHoldModeOld = altHoldMode;
}


void commanderGetRPYType(RPYType* rollType, RPYType* pitchType, RPYType* yawType)
{
  *rollType  = ANGLE;
  *pitchType = ANGLE;
  *yawType   = RATE;
}

void commanderGetRPY(float* eulerRollDesired, float* eulerPitchDesired, float* eulerYawDesired)
{
  int usedSide = side;
	
  *eulerRollDesired  = targetVal[usedSide].roll;
  *eulerPitchDesired = targetVal[usedSide].pitch;
  *eulerYawDesired   = targetVal[usedSide].yaw;
}

void commanderGetThrust(uint16_t* thrust)
{
  int usedSide = side;
  uint16_t rawThrust = targetVal[usedSide].thrust;

  if (rawThrust > MIN_THRUST)
  {
    *thrust = rawThrust;
  }
  else
  {
    *thrust = 0;
  }

  if (rawThrust > MAX_THRUST)
  {
    *thrust = MAX_THRUST;
  }

  commanderWatchdog();
}

void commanderPutRPY(float* eulerRollActual, float* eulerPitchActual, float* eulerYawActual,CRTPPacket* pk)
{
  int usedSide = side;
	uint8_t* temp; 
	uint8_t  Dat[30];
	uint8_t i=0;

  ActualVal[usedSide].roll  = *eulerRollActual ;
  ActualVal[usedSide].pitch = *eulerPitchActual;
  ActualVal[usedSide].yaw   = *eulerYawActual  ;
	temp = (uint8_t*)(&ActualVal[usedSide]);
	for(i=0;i<12;i++)
	{
		pk->data[i]= temp[i];
	}
	//Dat = temp;
	//&((*pk).data[0]) = Dat;
	//(pk->data) = Dat;
}

//#define PARAM_ADD(TYPE, NAME, ADDRESS) \
//   { .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },

/**/
// Params for flight modes
//PARAM_GROUP_START(flightmode)
//PARAM_ADD(PARAM_UINT8, althold, &altHoldMode)
//PARAM_GROUP_STOP(flightmode)

