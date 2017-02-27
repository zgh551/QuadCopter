 /**
 ******************************************************************************
 * @file    nRF24L01.h
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
 #ifndef __NRF24L01_H
 #define __NRF24L01_H
 
 #ifdef __cplusplus
	extern C{
 #endif
 
 //#include <string.h>
 #include <stdbool.h>	
		
 #include "stm32f10x.h"	
 #include "nRF24L01Reg.h"
 #include "stm32_qdcpt.h"
 #include "stm32_qdcpt_exti.h"

 
 enum
 {
	RADIO_RATE_1M,
	RADIO_RATE_2M
 };
 /* nRF24L SPI commands */
#define CMD_R_REG              0x00//read command and status registers
#define CMD_W_REG              0x20//write command and status registers(executable in power down or standby mode only)
#define CMD_R_RX_PAYLOAD       0x61//Read RX-payload:1-32 byte(used in RX mode)
#define CMD_W_TX_PAYLOAD       0xA0//Write TX-payload:1-32 byte (used in TX mode)
#define CMD_FLUSH_TX           0xE1//Flush the FIFO (used in TX mode)
#define CMD_FLUSH_RX           0xE2//Flush the FIFO (used in RX mode)
#define CMD_REUSE_TX_PL        0xE3//Reuse last transmitted payload 
#define CMD_ACTIVATE           0x50
#define ACTIVATE_DATA          0x73
#define CMD_RX_PL_WID          0x60//read RX payload width for the top(Flush RX FIFO if the read value is larger than 32 bytes.)
#define CMD_W_ACK_PAYLOAD(P)  (0xA8|(P&0x0F))//Write Payload to be transmitted together with ACK packet on PIPE PPP.(p from 0 to 5)
#define CMD_W_PAYLOAD_NO_ACK   0xD0//Disable AUTOACk on this specific packet
#define CMD_NOP                0xFF//No operation .Might used to read the STATUS regiters		
		
//usefull micro
#define RADIO_EN_CSN  GPIO_ResetBits(QDCPT_SPI_CSN_GPIO_PORT, QDCPT_SPI_CSN_PIN)//0
#define RADIO_DIS_CSN GPIO_SetBits  (QDCPT_SPI_CSN_GPIO_PORT, QDCPT_SPI_CSN_PIN)//1
#define RADIO_DIS_CE  GPIO_ResetBits(QDCPT_SPI_CE_GPIO_PORT, QDCPT_SPI_CE_PIN)  //0
#define RADIO_EN_CE   GPIO_SetBits  (QDCPT_SPI_CE_GPIO_PORT, QDCPT_SPI_CE_PIN)  //1

/******   STATUS寄存器bit位定义      *******/
#define BIT_RX_DR (1<<6)            //接收到数据中断
#define BIT_TX_DS (1<<5)            //TX发送完成中断
#define BIT_MAX_RT (1<<4)           //达到最大发送次数中断

//initialise the spi and exti of the chip
extern void nrfSpiInit(void);

//private function
static char SpiByte_RW(char byte);
//read and write register
uint8_t nrfReadBuf(uint8_t address,uint8_t *buf,uint8_t len);
uint8_t nrfReadReg(uint8_t address);

uint8_t nrfWriteBuf(uint8_t address ,uint8_t *buf,uint8_t len);
uint8_t nrfWriteReg(uint8_t address ,uint8_t byte);
//clear the RX and TX FIFO register
uint8_t nrfFlushTx(void);
uint8_t nrfFlushRx(void);

/*property function*/
uint8_t nrfGetStatus(void);
uint8_t nrfRxPayload(uint8_t *buf,uint8_t len);
uint8_t nrfRxLength(void);
uint8_t nrfActivate(void);
uint8_t nrfWriteAck(uint8_t pipe,uint8_t *buf,uint8_t len);
uint8_t nrfWritePayload(uint8_t *buf,uint8_t len);
uint8_t nrfWriteNoAck(uint8_t *buf,uint8_t len);
uint8_t nrfNOP(void);

/*set fuction*/
void nrfSetEnable(bool enable);
void nrfSetTxAddress(uint8_t *address);
void nrfSetRxAddress(uint8_t pipe,uint8_t *address);
void nrfSetDateRate(uint8_t datarate);
void nrfSetChannel(uint8_t channel);

/*interupt function*/
void nrfIsr(void);
void nrfSetInterruptCallback(void (*cb)(void));
	
 #ifdef __cplusplus
	}
 #endif
 #endif
