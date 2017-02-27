 /**
 ******************************************************************************
* @file    stm32_qdcpt.h
* @author  Zhu Guohua
* @version V1.0
* @date    07-March-2015
* @brief   This file provides
*          the method to inertial measurement unit
******************************************************************************
* @attention
* Quadcopter = qdcpt
* COPYRIGHT 2015 
******************************************************************************  
*/ 
#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern C{
#endif
#include "stm32f10x.h"
#include <stdbool.h>
#include "filter.h"
	
 typedef struct {
         int16_t x;
         int16_t y;
         int16_t z;
 } Axis3i16;

 typedef struct {
         int32_t x;
         int32_t y;
         int32_t z;
 } Axis3i32;

 typedef struct {
         float x;
         float y;
         float z;
 } Axis3f;
 
/**
 * IMU update frequency dictates the overall update frequency.
 */
#define IMU_UPDATE_FREQ   500
#define IMU_UPDATE_DT     (float)(1.0/IMU_UPDATE_FREQ)

/**
 * Set ACC_WANTED_LPF1_CUTOFF_HZ to the wanted cut-off freq in Hz.
 * The highest cut-off freq that will have any affect is fs /(2*pi).
 * E.g. fs = 350 Hz -> highest cut-off = 350/(2*pi) = 55.7 Hz -> 55 Hz
 */
#define IMU_ACC_WANTED_LPF_CUTOFF_HZ  10
/**
 * Attenuation should be between 1 to 256.
 *
 * f0 = fs / 2*pi*attenuation ->
 * attenuation = fs / 2*pi*f0
 */
#define IMU_ACC_IIR_LPF_ATTENUATION (IMU_UPDATE_FREQ / (2 * 3.1415 * IMU_ACC_WANTED_LPF_CUTOFF_HZ))
#define IMU_ACC_IIR_LPF_ATT_FACTOR  (int)(((1<<IIR_SHIFT) / IMU_ACC_IIR_LPF_ATTENUATION) + 0.5)

void imu6Init(void);
bool imu6Test(void);
void imu6Read(Axis3f* gyro, Axis3f* acc);
void imu9Read(Axis3f* gyroOut, Axis3f* accOut, Axis3f* magOut);
bool imu6IsCalibrated(void);
bool imuHasBarometer(void);
bool imuHasMangnetometer(void);	
	
	
#ifdef __cplusplus
}
#endif
#endif
