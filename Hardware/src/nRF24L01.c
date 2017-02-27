 /**
 ******************************************************************************
 * @file    nRF24L01.c
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

 #include "nRF24L01.h"
 
 /*****************************************************************************************************************
				                                        SPI privat method
 ******************************************************************************************************************/
 /*finish the one byte data read and write*/
 static char SpiByte_RW(char byte)
 { 
	//Wait for the DR register in no empty
	while(SPI_I2S_GetFlagStatus(QDCPT_SPI,SPI_I2S_FLAG_TXE)==RESET);
	/*send one byte through the qdcpt_spi peripheral*/
	SPI_I2S_SendData(QDCPT_SPI,byte);
	/*wait for receiving one byte*/
	while(SPI_I2S_GetFlagStatus(QDCPT_SPI,SPI_I2S_FLAG_RXNE )==RESET);
	/*return one byte from the SPI Bus*/
	return SPI_I2S_ReceiveData(QDCPT_SPI);
 }
 
 /********************************************************************************
 *nRF SPI Commands,Every commands return the status byte                         *
 *********************************************************************************/
 
 /*read length byte from nef24L01 register .5 byte max*/
 uint8_t nrfReadBuf(uint8_t address,uint8_t *buf,uint8_t len)
 {
	 uint8_t status;
	 uint8_t i=0;
	 //low level enable the CSN
	 RADIO_EN_CSN;
	 //send the read commond with the address
	 status=SpiByte_RW(CMD_R_REG |(address & 0x1f));
	 /*read len byte */
	 for(i=0;i<len;len++)
	 buf[i]=SpiByte_RW(0);
	 
	 RADIO_DIS_CSN;
	 return status;
 }
 
 //write len byte into nrf24L01 register .5 byte max
 uint8_t nrfWriteBuf(uint8_t address ,uint8_t *buf,uint8_t len)
 {
	 uint8_t status;
	 uint8_t i=0;
	 RADIO_EN_CSN;
	 
	 status = SpiByte_RW (CMD_W_REG |(address & 0x1f));
	 /*write len byte */
	 for(i=0;i<len;len++)
	 SpiByte_RW(buf[i]);
	 
	 RADIO_DIS_CSN;
	 return status;
 }
 
 //read only one byte from the register (usedful for most of reg.)
 uint8_t nrfReadReg(uint8_t address)
 {
	 uint8_t byte;
	 nrfReadBuf(address ,&byte ,1);
	 return byte;
 }
 
 //write only one byte into the register (usedful for most of reg.)
 uint8_t nrfWriteReg(uint8_t address ,uint8_t byte)
 {
	 return nrfWriteBuf(address ,&byte ,1);
 }
 
 /*set the nop command ,used to get the status*/
 uint8_t nrfNOP(void)
 {
		uint8_t status;
	  RADIO_EN_CSN;
	  status = SpiByte_RW(CMD_NOP);
	  RADIO_DIS_CSN;
	 return status;
 }
 
 //clear the RX FIFO register
 uint8_t nrfFlushRx(void)
 {
		uint8_t status;
	 RADIO_EN_CSN;
	 status = SpiByte_RW(CMD_FLUSH_RX);
	 SpiByte_RW(0xff);
	 RADIO_DIS_CSN;
	 return status;
 }
 
 //Clear the TX FIFO register
 uint8_t nrfFlushTx(void)
 {
		uint8_t  status;
	 RADIO_EN_CSN;
	 status=SpiByte_RW (CMD_FLUSH_TX);
	 SpiByte_RW(0xff);
	 RADIO_DIS_CSN;
	 return status;
 }
 
 //return the payload length
 uint8_t nrfRxLength(void)
 {
	 uint8_t  length;
	 RADIO_EN_CSN;
	 SpiByte_RW(CMD_RX_PL_WID);
	 length = SpiByte_RW (0);
	 RADIO_DIS_CSN;
	 return length ;
 }
 
 //active the register
 unsigned char nrfActivate(void)
{
  unsigned char status;
  
  RADIO_EN_CSN;
  status = SpiByte_RW(CMD_ACTIVATE);
  SpiByte_RW(ACTIVATE_DATA);
  RADIO_DIS_CSN;
	
  return status;
}
 
 //Write the ack payload of the pipe used in PRX mode
 uint8_t nrfWriteAck(uint8_t pipe,uint8_t *buf,uint8_t len)
 {
	 uint8_t status;
	 uint8_t i=0;
	 //test the pipe
	 assert_param(pipe<6);
	 
	 RADIO_DIS_CE;
	 RADIO_EN_CSN;
	 //send the read command with the address
	 status = SpiByte_RW (CMD_W_ACK_PAYLOAD(pipe));
	 /*write len byte*/
	 for(i=0;i<len;i++)
	 SpiByte_RW (buf[i]);
	 RADIO_DIS_CSN;
	 RADIO_EN_CE;
	 return status ;
 }
 
 //Write the tx payload used in PTX mode
 uint8_t nrfWritePayload(uint8_t *buf,uint8_t len)
 {
	 uint8_t status;
	 uint8_t i=0;
	 RADIO_DIS_CE;
	 RADIO_EN_CSN;
	 
	 //send the read command with the address
	 status = SpiByte_RW (CMD_W_TX_PAYLOAD);
	 /*write len byte*/
	 for(i=0;i<len;i++)
	 SpiByte_RW (buf[i]);
	 
	 RADIO_DIS_CSN;
	 RADIO_EN_CE;
	 //vTaskDelay(10); //Wait for the chip to be ready
	 //RADIO_DIS_CE;
	 return status ;
 }
 
  //Write the tx payload used in PTX mode ane disable the ACK
 uint8_t nrfWriteNoAck(uint8_t *buf,uint8_t len)
 {
	 uint8_t status;
	 uint8_t i=0;
	 RADIO_DIS_CE;
	 RADIO_EN_CSN;
	 //send the read command with the address
	 status = SpiByte_RW (CMD_W_PAYLOAD_NO_ACK);
	 /*write len byte*/
	 for(i=0;i<len;i++)
	 SpiByte_RW (buf[i]);
	 
	 RADIO_DIS_CSN;
	 RADIO_EN_CE;
	 return status ;
 }
 
 //read the RX payload 
 uint8_t nrfRxPayload(uint8_t *buf,uint8_t len)
 {
	 uint8_t status ;
	 uint8_t i;
	 RADIO_EN_CSN;
	 
	 /*send the command with address*/
	 status = SpiByte_RW (CMD_R_RX_PAYLOAD);
	 /*read len byte*/
	 for(i=0;i<len;i++)
	 buf[i]=SpiByte_RW(0);
	 
	 RADIO_DIS_CSN;
   return status ;
 }
 
 //set the nrf channel frequency (FO=24000+RF_CH [MHZ])
 void nrfSetChannel(uint8_t channel)
 {
	if(channel < 126)
	nrfWriteReg(REG_RF_CH,channel);
 }
 /*set the transfer date's rate*/
 void nrfSetDateRate(uint8_t datarate)
 {
	switch(datarate)
	{
		//0db增益,1Mbps,低噪声增益开启
		case RADIO_RATE_1M:
			nrfWriteReg (REG_RF_SETUP ,VAL_RF_SETUP_1M );
		break;
		//0db增益,2Mbps,低噪声增益开启
		case RADIO_RATE_2M:
			nrfWriteReg (REG_RF_SETUP ,VAL_RF_SETUP_2M );
		break;
	}
 }
 /*set nrf24L01's Tx address*/
 void nrfSetTxAddress(uint8_t *address)
 {
		nrfWriteBuf(REG_TX_ADDR,address,5);
 }
 
 /*set nrf24L01's Rx address*/
 void nrfSetRxAddress(uint8_t pipe,uint8_t *address)
 {
		uint8_t len=5;
		assert_param (pipe<6);
		if(pipe>1)
			len=1;
		nrfWriteBuf(REG_RX_ADDR_P0 + pipe,address ,len);
 }
 
 //the chip enable
 void nrfSetEnable(bool enable)
 {
	if(enable)
	{
		RADIO_EN_CE;
	}
	else
	{
		RADIO_DIS_CE;
	}	
 }
 
 //get status of register 
 uint8_t nrfGetStatus(void)
 {
	return nrfNOP();
 }
 
 /*nrf initialisation*/
 void nrfSpiInit(void)
 {
	SPI_InitTypeDef  SPI_InitStructure;
	//EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	 
	/*Enable the gpio clocks*/
	RCC_APB2PeriphClockCmd (QDCPT_SPI_CE_GPIO_CLK  | QDCPT_SPI_CSN_GPIO_CLK | QDCPT_SPI_IRQ_GPIO_CLK |
													QDCPT_SPI_MISO_GPIO_CLK |QDCPT_SPI_MOSI_GPIO_CLK| QDCPT_SPI_SCK_GPIO_CLK,ENABLE);
	/*Enable the spi clock*/
	RCC_APB2PeriphClockCmd (QDCPT_SPI_CLK,ENABLE);
	//enable the afio interrupt
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	 /*Configure SPI Pins:SCK and MOSI */
	 GPIO_InitStructure.GPIO_Pin=QDCPT_SPI_MOSI_PIN |QDCPT_SPI_SCK_PIN ;
	 GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	 GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	 GPIO_Init(QDCPT_SPI_PORT,&GPIO_InitStructure);
	 
	 /*Configure SPI Pin:MISO*/
	 GPIO_InitStructure.GPIO_Pin=QDCPT_SPI_MISO_PIN;
	 GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	 GPIO_Init(QDCPT_SPI_PORT,&GPIO_InitStructure);
	 
	 /*Configure the I/O for the chip select*/
	 GPIO_InitStructure.GPIO_Pin=QDCPT_SPI_CSN_PIN;
	 GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	 GPIO_Init(QDCPT_SPI_CSN_GPIO_PORT,&GPIO_InitStructure);
	 /*Configure the I/O pin for the Chip Enable*/
	 GPIO_InitStructure.GPIO_Pin=QDCPT_SPI_CE_PIN;
	 GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	 GPIO_Init(QDCPT_SPI_CE_GPIO_PORT,&GPIO_InitStructure);
	 
	 /*Configure the interruption (EXTI Source)*/
	 //GPIO_InitStructure.GPIO_Pin=QDCPT_SPI_IRQ_PIN;
	 //GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	 //GPIO_Init(QDCPT_SPI_IRQ_GPIO_PORT,&GPIO_InitStructure);
	 
	 /*select the GPIOB.00 as the EXTI Line*/
	 //GPIO_EXTILineConfig(QDCPT_SPI_IRQ_SRC_PORT,QDCPT_SPI_IRQ_SRC_PIN);  此处关闭外部中断
	 /*initialisture the EXTI*/
	 /*EXTI_InitStructure.EXTI_Line   =QDCPT_SPI_IRQ_LINE;
	 EXTI_InitStructure.EXTI_Mode   =EXTI_Mode_Interrupt;
	 EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
	 EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	 EXTI_Init(&EXTI_InitStructure);
	 
	 	/* Enable the EXTI interrupt router */
     //extiInit();
	 /*disable the chip select*/
	 RADIO_DIS_CSN;
	 /*disable the chip enable*/
	 //RADIO_DIS_CE;
	 
	 /*SPI Configuration*/
	 SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
	 SPI_InitStructure.SPI_Mode=SPI_Mode_Master;
	 SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b;
	 SPI_InitStructure.SPI_CPOL=SPI_CPOL_Low;
	 SPI_InitStructure.SPI_CPHA=SPI_CPHA_1Edge;
	 SPI_InitStructure.SPI_NSS=SPI_NSS_Soft;
	 SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_8;
	 SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB;
	 SPI_InitStructure.SPI_CRCPolynomial=7;
	 SPI_Init(QDCPT_SPI,&SPI_InitStructure);
	 
	 /*Enable the SPI*/
	 SPI_Cmd(QDCPT_SPI,ENABLE);
 }
