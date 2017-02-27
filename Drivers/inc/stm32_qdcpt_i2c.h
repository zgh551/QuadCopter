/**
******************************************************************************
* @file    stm32_qdcpt_i2c.h
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
 #ifndef __STM32_QDCPT_x_H
 #define __STM32_QDCPT_x_H
 
 #ifdef __cplusplus
	extern C{
 #endif
		
#include <stdint.h>
#include <stdbool.h>
	
#include "stm32f10x.h"
#include "crc.h"
// Delay is approx 0.2us per loop @64Mhz
#define I2CDEV_LOOPS_PER_US  5
#define I2CDEV_LOOPS_PER_MS  (1000 * I2CDEV_LOOPS_PER_US)
	
#define I2CDEV_NO_MEM_ADDR  0xFF

/**
 * Read bytes from an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param len  Number of bytes to read.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevRead(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress,
               uint16_t len, uint8_t *data);
/**
 * I2C device init function.
 * @param I2Cx  Pointer to I2C peripheral to initialize.
 *
 * @return TRUE if initialization went OK otherwise FALSE.
 */
int i2cdevInit(void);

/**
 * Read a byte from an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevReadByte(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress,
                    uint8_t *data);

/**
 * Read a bit from an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param bitNum  The bit number 0 - 7 to read.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevReadBit(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitNum, uint8_t *data);
/**
 * Read up to 8 bits from an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param bitStart The bit to start from, 0 - 7.
 * @param length  The number of bits to read, 1 - 8.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevReadBits(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitStart, uint8_t length, uint8_t *data);

/**
 * Write bytes to an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write to, I2CDEV_NO_MEM_ADDR if none.
 * @param len  Number of bytes to read.
 * @param data  Pointer to a buffer to read the data from that will be written.
 *
 * @return TRUE if write was successful, otherwise FALSE.
 */
bool i2cdevWrite(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress,
                 uint16_t len, uint8_t *data);

/**
 * Write a byte to an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write from, I2CDEV_NO_MEM_ADDR if none.
 * @param data  The byte to write.
 *
 * @return TRUE if write was successful, otherwise FALSE.
 */
bool i2cdevWriteByte(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress,
                     uint8_t data);

/**
 * Write a bit to an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write to, I2CDEV_NO_MEM_ADDR if none.
 * @param bitNum  The bit number, 0 - 7, to write.
 * @param data  The bit to write.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevWriteBit(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitNum, uint8_t data);

/**
 * Write up to 8 bits to an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write to, I2CDEV_NO_MEM_ADDR if none.
 * @param bitStart The bit to start from, 0 - 7.
 * @param length  The number of bits to write, 1 - 8.
 * @param data  The byte containing the bits to write.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevWriteBits(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitStart, uint8_t length, uint8_t data);

/**
 * I2C1 DMA interrupt handler
 */
void i2cDmaInterruptHandlerI2c1(void);
/**
 * I2C2 DMA interrupt handler
 */
void i2cDmaInterruptHandlerI2c2(void);
 #ifdef __cplusplus
	}
 #endif
 #endif
