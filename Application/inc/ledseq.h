 /**
 ******************************************************************************
 * @file    ledseq.h
 * @author  Zhu Guohua
 * @version V1.0
 * @date    11-March-2015
 * @brief   This file provides
 *          
 ******************************************************************************
 * @attention
 * Quadcopter = qdcpt
 * COPYRIGHT 2015 
 ******************************************************************************  
 */
 /**
 *    |          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/_/__/_/   _,_/ /___/__/
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
 */
 #ifndef __LEDSEQ_H
 #define __LEDSEQ_H
 
 #ifdef __cplusplus
	extern C{
 #endif
 
 #include "stm32f10x.h"
 #include <stdbool.h>
 #include <stm32_qdcpt_led.h>
 #include "stm32_qdcpt.h"
 
 #define LEDSEQ_CHARGE_CYCLE_TIME  1000
 //Led sequence action
 #define LEDSEQ_WAITMS(X)  X
 #define LEDSEQ_STOP      -1
 #define LEDSEQ_LOOP      -2
 typedef struct {
   bool value;
   int action;
 } ledseq_t;

//Public API
void LedseqInit(void);
bool LedseqTest(void);

void LedseqRun(Led_TypeDef led, ledseq_t * sequence);
void LedseqStop(Led_TypeDef led, ledseq_t * sequence);
void LedseqSetTimes(ledseq_t *sequence, int32_t onTime, int32_t offTime);

//Existing led sequences
extern ledseq_t seq_armed[];
extern ledseq_t seq_calibrated[];
extern ledseq_t seq_alive[];
extern ledseq_t seq_lowbat[];
extern ledseq_t seq_linkup[];
extern ledseq_t seq_altHold[];
extern ledseq_t seq_charged[];
extern ledseq_t seq_charging[];
extern ledseq_t seq_bootloader[];
extern ledseq_t seq_testPassed[];	
 #ifdef __cplusplus
	}
 #endif
 #endif
