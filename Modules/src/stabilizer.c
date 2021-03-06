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
//#include "config.h"

#include "math.h"

#include "system.h"
//#include "pm.h"
#include "stabilizer.h"
#include "commander.h"
#include "controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "pid.h"
#include "ledseq.h"
#include "crc.h"
#include "worker.h"

#include "nRF24L01.h"
#include "fp16.h"
#include "crtp.h"
#include "radiolink.h"
#include "stm32_qdcpt_led.h"
#include "pm.h"
//#include "param.h"
#include "ms5611.h"
//#include "uart.h"

uint8_t euler_data[3][4];
uint8_t i,j,cnt;
uint8_t text_tx[12];
CRTPPacket EulerData;
CRTPPacket CmdData;
uint32_t attitudeCounter = 0;
uint32_t altHoldCounter = 0;

float test;

#undef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#undef min
#define min(a,b) ((a) < (b) ? (a) : (b))


/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

// Barometer/ Altitude hold stuff
#define ALTHOLD_UPDATE_RATE_DIVIDER  5 // 500hz/5 = 100hz for barometer measurements
#define ALTHOLD_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ALTHOLD_UPDATE_RATE_DIVIDER))   // 500hz

static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static Axis3f mag;  // Magnetometer axis data in testla

 float eulerRollActual;
 float eulerPitchActual;
 float eulerYawActual;

float eulerRollDesired;
float eulerPitchDesired;
float eulerYawDesired;
//期望高度
uint8_t HightDesired;


static float rollRateDesired;
static float pitchRateDesired;
static float yawRateDesired;

// Baro variables
static float temperature; // temp from barometer
static float pressure;    // pressure from barometer
static float asl;     // smoothed asl
static float aslRaw;  // raw asl
static float aslLong; // long term asl

// Altitude hold variables
PidObject altHoldPID; // Used for altitute hold mode. I gets reset when the bat status changes
bool altHold = false;          // Currently in altitude hold mode
bool setAltHold = false;      // Hover mode has just been activated
float accWZ     = 0.0;
static float accMAG    = 0.0;
float vSpeedASL = 0.0;
float vSpeedAcc = 0.0;
static float vSpeed    = 0.0; // Vertical speed (world frame) integrated from vertical acceleration
static float altHoldPIDVal;                    // Output of the PID controller
static float altHoldErr;                       // Different between target and current altitude
float BatVal=0.0;
// Altitude hold & Baro Params
static float altHoldKp              = 3;  // 0.5PID gain constants, used everytime we reinitialise the PID controller
static float altHoldKi              = 0.18; //0.18
static float altHoldKd              = 1;		//0.8
static float altHoldChange          = 0;     // Change in target altitude
static float altHoldTarget          = -1;    // Target altitude
static float altHoldErrMax          = 1.0;   // max cap on current estimated altitude vs target altitude in meters
static float altHoldChange_SENS     = 200;   // sensitivity of target altitude change (thrust input control) while hovering. Lower = more sensitive & faster changes
static float pidAslFac              = 13000; // relates meters asl to thrust
static float pidAlpha               = 0.8;   // PID Smoothing //TODO: shouldnt need to do this
static float vSpeedASLFac           = 0;    // multiplier
static float vSpeedAccFac           = 0;  // multiplier -48
static float vAccDeadband           = 0.05;  // Vertical acceleration deadband  0.05
static float vSpeedASLDeadband      = 0.005; // Vertical speed based on barometer readings deadband
static float vSpeedLimit            = 10;  // used to constrain vertical velocity
static float errDeadband            = 0.00;  // error (target - altitude) deadband
static float vBiasAlpha             = 0.91; // Blending factor we use to fuse vSpeedASL and vSpeedAcc
static float aslAlpha               = 0.92; // Short term smoothing
static float aslAlphaLong           = 0.93; // Long term smoothing
static uint16_t altHoldMinThrust    = 00000; // minimum hover thrust - not used yet
static uint16_t altHoldBaseThrust   = 43000; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
static uint16_t altHoldMaxThrust    = 65000; // max altitude hold thrust


RPYType rollType;
RPYType pitchType;
RPYType yawType;


uint16_t actuatorThrust;

int16_t  actuatorRoll;
int16_t  actuatorPitch;
int16_t  actuatorYaw;

uint32_t motorPowerM4;
uint32_t motorPowerM2;
uint32_t motorPowerM1;
uint32_t motorPowerM3;

static bool isInit= false;

static void stabilizerAltHoldUpdate(void);
static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw);
static uint16_t limitThrust(int32_t value);
//TASK FUNCTION
static void stabilizerTask(void* param);
static void StationLinkTask(void* param);

static float constrain(float value, const float minVal, const float maxVal);
static float deadband(float value, const float threshold);

void stabilizerInit(void)
{
  if(isInit)
    return;
  motorsInit();
  imu6Init();
  sensfusion6Init();
  controllerInit();

  rollRateDesired  = 0;
  pitchRateDesired = 0;
  yawRateDesired = 0;

  xTaskCreate(stabilizerTask, ( signed portCHAR* )"STABILIZER",
              2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);
	
//	xTaskCreate(StationLinkTask,( signed portCHAR* )"StationLink",
//              2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);

  isInit = TRUE;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= motorsTest();
  pass &= imu6Test();
  pass &= sensfusion6Test();
  pass &= controllerTest();

  return pass;
}

static void stabilizerTask(void* param)
{

	
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (pdTASK_HOOK_CODE)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount();

  for(;;)
  {
		
    vTaskDelayUntil((portTickType *)&lastWakeTime,F2T(IMU_UPDATE_FREQ));//// 500Hz 

    // Magnetometer not yet used more then for logging.
		//imu9Read(&gyro, &acc, &mag);
	//taskENTER_CRITICAL();
	//nrfSetEnable(false);
    imu6Read(&gyro,&acc);
	//nrfSetEnable(true);
	//taskEXIT_CRITICAL();
		//Stm32QdcptLedToggle(LEDL);
    if (imu6IsCalibrated())//判断传感器是否校准完成
    {
			//获取遥控器的期望欧拉角
      commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
			//获取数据类型
      commanderGetRPYType(&rollType, &pitchType, &yawType);
      // 250HZ
      if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
      {
				//Stm32QdcptLedToggle(LEDR);
				//四元素更新
        sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, FUSION_UPDATE_DT);
				//计算实际的欧拉角
        sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
				
				//返回在没有重力加速度下的垂直加速度
        accWZ = sensfusion6GetAccZWithoutGravity(acc.x, acc.y, acc.z)-0.13;
				
        accMAG = (acc.x*acc.x) + (acc.y*acc.y) + (acc.z*acc.z) ;
        // Estimate speed from acc (drifts)
        vSpeed += deadband(accWZ, vAccDeadband) * FUSION_UPDATE_DT;

        controllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual,
                                     eulerRollDesired, eulerPitchDesired, -eulerYawDesired,
                                     &rollRateDesired, &pitchRateDesired, &yawRateDesired);
        attitudeCounter = 0;				
      }
			
			if(cnt++>100)
			{	
				Stm32QdcptLedToggle(LEDR);
				cnt = 0;
			}

      // 100HZ 如果有气压计
      if (imuHasBarometer() && (++altHoldCounter >= ALTHOLD_UPDATE_RATE_DIVIDER))//100hz进入一次
      {
        stabilizerAltHoldUpdate();
        altHoldCounter = 0;
      }

      if (rollType == RATE)
      {
        rollRateDesired = eulerRollDesired;
      }
      if (pitchType == RATE)
      {
        pitchRateDesired = eulerPitchDesired;
      }
      if (yawType == RATE)
      {
        yawRateDesired = -eulerYawDesired;
      }

      // TODO: Investigate possibility to subtract gyro drift.
      controllerCorrectRatePID(gyro.x, -gyro.y, gyro.z,
                               rollRateDesired, pitchRateDesired, yawRateDesired);

      controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw);

      if (!altHold || !imuHasBarometer())//如果没有使用气压计定高
      {
        // Use thrust from controller if not in altitude hold mode
        commanderGetThrust(&actuatorThrust);
//				actuatorThrust += accWZ*10000;
      }
      else
      {
        // Added so thrust can be set to 0 while in altitude hold mode after disconnect
        commanderWatchdog();
      }
			
			BatVal=pmGetBatteryVoltage();
//			actuatorThrust = 0;
			if(systemCanFly() &&eulerRollActual<90 &&eulerRollActual>-90)//
			{
				if (actuatorThrust > 0)
				{
						#if defined(TUNE_ROLL)
					distributePower(actuatorThrust, actuatorRoll, 0, 0);
						#elif defined(TUNE_PITCH)
					distributePower(actuatorThrust, 0, actuatorPitch, 0);
						#elif defined(TUNE_YAW)
					distributePower(actuatorThrust, 0, 0, -actuatorYaw);
						#else
					distributePower(actuatorThrust, actuatorRoll, actuatorPitch, -actuatorYaw);
						#endif
				}
				else
				{
					distributePower(0, 0, 0, 0);
					controllerResetAllPID();
				}
			}
			else
			{
					distributePower(0, 0, 0, 0);
					controllerResetAllPID();
			}
    }
  }
}

void StationLink(void)
{
			FloatToByte(eulerRollActual ,&EulerData.data[0]);
			FloatToByte(eulerPitchActual,&EulerData.data[4]);	
			FloatToByte(eulerYawActual  ,&EulerData.data[8]);	
			FloatToByte(altHoldTarget,&EulerData.data[12]);//pmGetBatteryVoltage()
			FloatToByte(actuatorThrust,&EulerData.data[16]);//actuatorThrust
			FloatToByte(asl,&EulerData.data[20]);
			FloatToByte(BatVal,&EulerData.data[24]);//
			EulerData.size=28;
			EulerData.header=0x80;
			crtpSendPacketBlock(&EulerData);
			//radioSendPacket();
}

void GetBaseThrust(void)
{
		if(BatVal>3.6)
	{
		altHoldBaseThrust= 36000;
		altHoldMinThrust = 32000;	
	}
	else if(BatVal>3.5)
	{
		altHoldBaseThrust= 38000;
		altHoldMinThrust = 34000;
	}
	else if(BatVal>3.4)
	{
		altHoldBaseThrust= 40000;
		altHoldMinThrust = 36000;
	}
	else if(BatVal>3.3)
	{
		altHoldBaseThrust= 42000;
		altHoldMinThrust = 38000;
	}
	else
	{
		altHoldBaseThrust= 48000;
		altHoldMinThrust = 46000;
	}
}
static void stabilizerAltHoldUpdate(void)
{
  // Cache last integral term for reuse after pid init
  const float pre_integral = altHoldPID.integ;
  // Get altitude hold commands from pilot
	
  commanderGetAltHold(&altHold, &setAltHold, &altHoldChange);
	//获取目标高度值
	//commanderGetHight(&HightDesired);
  // Get barometer height estimates
  //TODO do the smoothing within getData
  ms5611GetData(&pressure, &temperature, &aslRaw);
	//set the base thrust
	
	GetBaseThrust();
	
  asl = asl * aslAlpha + aslRaw * (1 - aslAlpha);
  aslLong = aslLong * aslAlphaLong + aslRaw * (1 - aslAlphaLong);

  // Estimate vertical speed based on successive barometer readings. This is ugly :)
  vSpeedASL = deadband(asl - aslLong, vSpeedASLDeadband);

  // Estimate vertical speed based on Acc - fused with baro to reduce drift
  vSpeed = constrain(vSpeed, -vSpeedLimit, vSpeedLimit);
//  vSpeed = vSpeed * vBiasAlpha + vSpeedASL * (1.f - vBiasAlpha);
  vSpeedAcc = vSpeed;

  // Reset Integral gain of PID controller if being charged
// if (!pmIsDischarging())
//  {
//    altHoldPID.integ = 0.0;
//  }


  // Altitude hold mode just activated, set target altitude as current altitude. Reuse previous integral term as a starting point
  if (setAltHold)//
  {
    // Set to current altitude
    altHoldTarget = asl;

    // Reset PID controller
    pidInit(&altHoldPID, asl, altHoldKp, altHoldKi, altHoldKd,
            ALTHOLD_UPDATE_DT);
    // TODO set low and high limits depending on voltage
    // TODO for now just use previous I value and manually set limits for whole voltage range
    //                    pidSetIntegralLimit(&altHoldPID, 12345);
    //                    pidSetIntegralLimitLow(&altHoldPID, 12345);              /
//		Stm32QdcptLedToggle(LEDL);
    altHoldPID.integ = pre_integral;

    // Reset altHoldPID
    altHoldPIDVal = pidUpdate(&altHoldPID, asl, false);
  }
	
  // In altitude hold mode
  if (altHold)//altHold
  {
//		Stm32QdcptLedToggle(LEDR);
    // Update target altitude from joy controller input
    altHoldTarget += altHoldChange / altHoldChange_SENS;
//		altHoldErr =asl - altHoldTarget;
    pidSetDesired(&altHoldPID, altHoldTarget);

    // Compute error (current - target), limit the error
    altHoldErr = constrain(deadband(asl - altHoldTarget, errDeadband),
                           -altHoldErrMax, altHoldErrMax);
		
    pidSetError(&altHoldPID, -altHoldErr);

    // Get control from PID controller, dont update the error (done above)
    // Smooth it and include barometer vspeed
    // TODO same as smoothing the error??+accWZ*10
    altHoldPIDVal = (pidAlpha) * altHoldPIDVal + (1.f - pidAlpha) * ((vSpeedAcc * vSpeedAccFac) +
                    (vSpeedASL * vSpeedASLFac) + pidUpdate(&altHoldPID, asl, false));

    // compute new thrust
    actuatorThrust =  max(altHoldMinThrust, min(altHoldMaxThrust,
                          limitThrust( altHoldBaseThrust + (int32_t)(altHoldPIDVal*pidAslFac))));
		
    // i part should compensate for voltage drop

  }
  else
  {
    altHoldTarget = 0.0;
    altHoldErr = 0.0;
    altHoldPIDVal = 0.0;
  }
}

static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw)
{
#ifdef QUAD_FORMATION_X
  int16_t intRoll, intPitch;
  
  intRoll  = roll  >> 1;
  intPitch = pitch >> 1;
  motorPowerM1 = limitThrust(thrust + roll + pitch - yaw);
  motorPowerM2 = limitThrust(thrust + roll - pitch + yaw);
  motorPowerM3 = limitThrust(thrust - roll - pitch - yaw);
  motorPowerM4 = limitThrust(thrust - roll + pitch + yaw);
//  motorPowerM1 = limitThrust(thrust + roll + pitch + yaw);
//  motorPowerM2 = limitThrust(thrust + roll - pitch - yaw);
//  motorPowerM3 = limitThrust(thrust - roll - pitch + yaw);
//  motorPowerM4 = limitThrust(thrust - roll + pitch - yaw);
#else // QUAD_FORMATION_NORMAL
  motorPowerM1 = limitThrust(thrust + pitch + yaw);
  motorPowerM2 = limitThrust(thrust - roll - yaw);
  motorPowerM3 = limitThrust(thrust - pitch + yaw);
  motorPowerM4 = limitThrust(thrust + roll - yaw);
#endif

  motorsSetRatio(MOTOR_M1, motorPowerM1);
  motorsSetRatio(MOTOR_M2, motorPowerM2);
  motorsSetRatio(MOTOR_M3, motorPowerM3);
  motorsSetRatio(MOTOR_M4, motorPowerM4);
}

static uint16_t limitThrust(int32_t value)
{
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}

// Constrain value between min and max
static float constrain(float value, const float minVal, const float maxVal)
{
  return min(maxVal, max(minVal,value));
}

// Deadzone
static float deadband(float value, const float threshold)
{
  if (fabs(value) < threshold)
  {
    value = 0;
  }
  else if (value > 0)
  {
    value -= threshold;
  }
  else if (value < 0)
  {
    value += threshold;
  }
  return value;
}

/*
LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &acc.x)
LOG_ADD(LOG_FLOAT, y, &acc.y)
LOG_ADD(LOG_FLOAT, z, &acc.z)
LOG_ADD(LOG_FLOAT, zw, &accWZ)
LOG_ADD(LOG_FLOAT, mag2, &accMAG)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &gyro.x)
LOG_ADD(LOG_FLOAT, y, &gyro.y)
LOG_ADD(LOG_FLOAT, z, &gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &mag.x)
LOG_ADD(LOG_FLOAT, y, &mag.y)
LOG_ADD(LOG_FLOAT, z, &mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerM4)
LOG_ADD(LOG_INT32, m1, &motorPowerM1)
LOG_ADD(LOG_INT32, m2, &motorPowerM2)
LOG_ADD(LOG_INT32, m3, &motorPowerM3)
LOG_GROUP_STOP(motor)

// LOG altitude hold PID controller states
LOG_GROUP_START(vpid)
LOG_ADD(LOG_FLOAT, pid, &altHoldPID)
LOG_ADD(LOG_FLOAT, p, &altHoldPID.outP)
LOG_ADD(LOG_FLOAT, i, &altHoldPID.outI)
LOG_ADD(LOG_FLOAT, d, &altHoldPID.outD)
LOG_GROUP_STOP(vpid)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &asl)
LOG_ADD(LOG_FLOAT, aslRaw, &aslRaw)
LOG_ADD(LOG_FLOAT, aslLong, &aslLong)
LOG_ADD(LOG_FLOAT, temp, &temperature)
LOG_ADD(LOG_FLOAT, pressure, &pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(altHold)
LOG_ADD(LOG_FLOAT, err, &altHoldErr)
LOG_ADD(LOG_FLOAT, target, &altHoldTarget)
LOG_ADD(LOG_FLOAT, zSpeed, &vSpeed)
LOG_ADD(LOG_FLOAT, vSpeed, &vSpeed)
LOG_ADD(LOG_FLOAT, vSpeedASL, &vSpeedASL)
LOG_ADD(LOG_FLOAT, vSpeedAcc, &vSpeedAcc)
LOG_GROUP_STOP(altHold)

// Params for altitude hold
PARAM_GROUP_START(altHold)
PARAM_ADD(PARAM_FLOAT, aslAlpha, &aslAlpha)
PARAM_ADD(PARAM_FLOAT, aslAlphaLong, &aslAlphaLong)
PARAM_ADD(PARAM_FLOAT, errDeadband, &errDeadband)
PARAM_ADD(PARAM_FLOAT, altHoldChangeSens, &altHoldChange_SENS)
PARAM_ADD(PARAM_FLOAT, altHoldErrMax, &altHoldErrMax)
PARAM_ADD(PARAM_FLOAT, kd, &altHoldKd)
PARAM_ADD(PARAM_FLOAT, ki, &altHoldKi)
PARAM_ADD(PARAM_FLOAT, kp, &altHoldKp)
PARAM_ADD(PARAM_FLOAT, pidAlpha, &pidAlpha)
PARAM_ADD(PARAM_FLOAT, pidAslFac, &pidAslFac)
PARAM_ADD(PARAM_FLOAT, vAccDeadband, &vAccDeadband)
PARAM_ADD(PARAM_FLOAT, vBiasAlpha, &vBiasAlpha)
PARAM_ADD(PARAM_FLOAT, vSpeedAccFac, &vSpeedAccFac)
PARAM_ADD(PARAM_FLOAT, vSpeedASLDeadband, &vSpeedASLDeadband)
PARAM_ADD(PARAM_FLOAT, vSpeedASLFac, &vSpeedASLFac)
PARAM_ADD(PARAM_FLOAT, vSpeedLimit, &vSpeedLimit)
PARAM_ADD(PARAM_UINT16, baseThrust, &altHoldBaseThrust)
PARAM_ADD(PARAM_UINT16, maxThrust, &altHoldMaxThrust)
PARAM_ADD(PARAM_UINT16, minThrust, &altHoldMinThrust)
PARAM_GROUP_STOP(altHold)

*/
