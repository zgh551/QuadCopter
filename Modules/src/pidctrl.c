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
 * pidctrl.c - Used to receive/answer requests from client and to receive updated PID values from client
 */
 
/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#include "crtp.h"
#include "pidctrl.h"
#include "pid.h"
#include "crc.h"
  extern PidObject pidRollRate;
  extern PidObject pidPitchRate;
  extern PidObject pidYawRate;
  extern PidObject pidRoll;
  extern PidObject pidPitch;
  extern PidObject pidYaw;
	extern PidObject altHoldPID;
typedef enum {
  pidCtrl_RPValues  = 0x01,
	pidCtrl_YValues   = 0x02,
	pidCtrl_ALTValues = 0x03,
} PIDCrtlNbr;

void pidCrtlTask(void *param);

void pidCtrlInit()
{
  xTaskCreate(pidCrtlTask, (signed char*)"PIDCrtl",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/2, NULL);
  crtpInitTaskQueue(CRTP_PORT_PID);
}

void pidCrtlTask(void *param)
{
  CRTPPacket p;

	
	 
  struct pidValues
  {
    float rateKp;
    float rateKi;
    float rateKd;
    float attKp;
    float attKi;
    float attKd;
  } ;// __attribute__((packed));
  struct pidValues *pPid;

  while (TRUE)
  {
    if (crtpReceivePacketBlock(CRTP_PORT_PID, &p) == pdTRUE)
    {
      PIDCrtlNbr pidNbr = (PIDCrtlNbr) p.channel;
      
      switch (pidNbr)
      {
        case pidCtrl_RPValues:
          pPid = (struct pidValues *)p.data;
          {
            pidSetKp(&pidRollRate, pPid->rateKp);
            pidSetKi(&pidRollRate, pPid->rateKi);
            pidSetKd(&pidRollRate, pPid->rateKd);
            pidSetKp(&pidRoll, pPid->attKp);
            pidSetKi(&pidRoll, pPid->attKi);
            pidSetKd(&pidRoll, pPid->attKd);
						
            pidSetKp(&pidPitchRate, pPid->rateKp);
            pidSetKi(&pidPitchRate, pPid->rateKi);
            pidSetKd(&pidPitchRate, pPid->rateKd);
            pidSetKp(&pidPitch, pPid->attKp);
            pidSetKi(&pidPitch, pPid->attKi);
            pidSetKd(&pidPitch, pPid->attKd);
          }
          break;
					case pidCtrl_YValues:
						pPid = (struct pidValues *)p.data;
          {
            pidSetKp(&pidYawRate, pPid->rateKp);
            pidSetKi(&pidYawRate, pPid->rateKi);
            pidSetKd(&pidYawRate, pPid->rateKd);
            pidSetKp(&pidYaw, pPid->attKp);
            pidSetKi(&pidYaw, pPid->attKi);
            pidSetKd(&pidYaw, pPid->attKd);
					}
					break;
					case pidCtrl_ALTValues:
						pPid = (struct pidValues *)p.data;
          {
            pidSetKp(&altHoldPID, pPid->rateKp);
            pidSetKi(&altHoldPID, pPid->rateKi);
            pidSetKd(&altHoldPID, pPid->rateKd);
            //pidSetKp(&pidYaw, pPid->attKp);
            //pidSetKi(&pidYaw, pPid->attKi);
            //pidSetKd(&pidYaw, pPid->attKd);
					}
					break;
					
        default:
          break;
      } 
    }
  }
}

