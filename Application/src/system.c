 /**
 ******************************************************************************
 * @file    system.c
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
 */

#include "system.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "console.h"
#include "comm.h"
#include "stm32_qdcpt_led.h"
#include "stm32_qdcpt_adc.h"
#include "ledseq.h"
#include "crtp.h"
#include "debug.h"
#include "nRF24L01.h"
#include "radiolink.h"
#include "motors.h"
#include "mpu6050.h"
#include "imu.h"
#include "fp16.h"
#include "stabilizer.h"
#include "worker.h"
#include "pm.h"
#include "commander.h"
#include "pidctrl.h"
/* Private variable */
bool canFly;

static bool isInit;

//unsigned portBASE_TYPE wtmk;

Axis3f gyro; // Gyro axis data in deg/s
Axis3f acc;  // Accelerometer axis data in mG
uint8_t f_data[6][4];

uint8_t testtx[32]={0xaa};
CRTPPacket TxPack;
CRTPPacket RxPack;
CRTPPacket mypack;
//uint8_t i;

/* System wide synchronisation */
xSemaphoreHandle StartMutex;

/* Private functions */
static void vTaskFunction(void *pvParameters);
static void systemTask(void *arg);
void systemStart(void);

/* Public functions */
void systemLaunch(void)
{
//	LedseqInit();
	xTaskCreate(systemTask,( signed portCHAR* )"SYSTEM",
              configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);
}

//This must be the first module to be initialized!
void systemInit(void)
{
  if(isInit)
    return;
	StartMutex = xSemaphoreCreateMutex();
  xSemaphoreTake(StartMutex, portMAX_DELAY);
	
  //configblockInit();
  workerInit();
  adcInit();//6.13 add
  LedseqInit();
  pmInit();//6.13 add
  isInit = true;
}

bool systemTest()
{
  bool pass=isInit;
  
  pass &= adcTest();
  pass &= LedseqTest();
  pass &= pmTest();
  pass &= workerTest();
  
  return pass;
}

void systemTask(void *arg)
{
  bool pass = true;
  //Init the high-levels modules
  systemInit();
	//communication initialise
	commInit();
	commanderInit();
	pidCtrlInit();
  stabilizerInit();  
	
	//Test the modules
  pass &= systemTest();/////////////////
////  pass &= commTest();
////  pass &= commanderTest();
  pass &= stabilizerTest();/////////////////
	//Start the firmware
  if(pass)
  {
    systemStart();
    LedseqRun(LEDL,seq_testPassed);//seq_linkup
  }
  else
  {
    if (systemTest())
    {
      while(1)
      {
//        LedseqRun(LEDL, seq_testPassed); //Red passed == not passed!
        vTaskDelay(M2T(2000));
      }
    }
    else
    {
      Stm32QdcptInitLed();
      Stm32QdcptLedSet(LEDL,ON);
    }
  }
  workerLoop();
	

//	xTaskCreate(vTaskFunction,( signed portCHAR* )"task1",
//								configMINIMAL_STACK_SIZE,NULL,2,NULL);
	
	while(1)
	vTaskDelay(portMAX_DELAY);
}

void vTaskFunction(void *pvParameters)
{
	int j,cnt;
	for( ;; )
	{
		//taskENTER_CRITICAL();
		//mpu6050TestConnection();
//		//testtx[0]=mpu6050GetDeviceID();
//		imu6Read(&gyro,&acc);
//		
//		FloatToByte(gyro.x,f_data[0]);
//		FloatToByte(gyro.y,f_data[1]);
//		FloatToByte(gyro.z,f_data[2]);
//		
//		FloatToByte(acc.x,f_data[3]);
//		FloatToByte(acc.y,f_data[4]);
//		FloatToByte(acc.z,f_data[5]);
//		cnt =0 ;
//		for(i=0;i<6;i++)
//		{
//			for(j=0;j<4;j++)
//			{
//				testtx[cnt]=f_data[i][j];
//				cnt++;
//			}	
//		}
		
		//LedseqRun(LEDL,seq_testPassed);
		//LedseqRun(LEDR,seq_testPassed);
		radioReceivePacket(&mypack);
		radioSendPacket(&mypack);
		
		//nrfWritePayload(testtx,24);//TX mode use it

		//taskEXIT_CRITICAL();
		vTaskDelay(1000);
	}
}

/* Global system variables */
void systemStart()
{
  xSemaphoreGive(StartMutex);
}

void systemWaitStart(void)
{
  //This permits to guarantee that the system task is initialized before other
  //tasks waits for the start event.
  while(!isInit)
    vTaskDelay(2);

  xSemaphoreTake(StartMutex, portMAX_DELAY);
  xSemaphoreGive(StartMutex);
}

void systemSetCanFly(bool val)
{
  canFly = val;
}

bool systemCanFly(void)
{
  return canFly;
}
