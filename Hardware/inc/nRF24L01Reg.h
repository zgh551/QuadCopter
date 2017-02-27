 /**
 ******************************************************************************
 * @file    stm32_qdcpt.h
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
 #ifndef __NRF24L01REG_H
 #define __NRF24L01REG_H
 
 #ifdef __cplusplus
	extern C{
 #endif
		
/* Registers address definition */
#define REG_CONFIG              0x00 // 配置收发状态，CRC校验模式以及收发状态响应方式
#define REG_EN_AA               0x01 // 自动应答功能设置
#define REG_EN_RXADDR           0x02 // 可用信道设置
#define REG_SETUP_AW            0x03 // 收发地址宽度设置
#define REG_SETUP_RETR          0x04 // 自动重发功能设置
#define REG_RF_CH               0x05 // 工作频率设置 Fo=(2400+RF_CH)MHz
#define REG_RF_SETUP            0x06 // 发射速率、功耗功能设置
#define REG_STATUS              0x07 // 状态寄存器
#define REG_OBSERVE_TX          0x08 // 发送监测功能
#define REG_RPD                 0x09 // 地址检测   
#define REG_RX_ADDR_P0          0x0A // 频道0接收数据地址
#define REG_RX_ADDR_P1          0x0B // 频道1接收数据地址
#define REG_RX_ADDR_P2          0x0C // 频道2接收数据地址
#define REG_RX_ADDR_P3          0x0D // 频道3接收数据地址
#define REG_RX_ADDR_P4          0x0E // 频道4接收数据地址
#define REG_RX_ADDR_P5          0x0F // 频道5接收数据地址
#define REG_TX_ADDR             0x10 // 发送地址寄存器
#define REG_RX_PW_P0            0x11 // 接收频道0接收数据长度
#define REG_RX_PW_P1            0x12 // 接收频道0接收数据长度
#define REG_RX_PW_P2            0x13 // 接收频道0接收数据长度
#define REG_RX_PW_P3            0x14 // 接收频道0接收数据长度
#define REG_RX_PW_P4            0x15 // 接收频道0接收数据长度
#define REG_RX_PW_P5            0x16 // 接收频道0接收数据长度
#define REG_FIFO_STATUS         0x17 // FIFO栈入栈出状态寄存器设置
#define REG_DYNPD               0x1C // 使能动态负载长度
#define REG_FEATURE             0x1D // 特征寄存器
//RF Setup Regist 
#define VAL_RF_SETUP_1M         0x07 //
#define VAL_RF_SETUP_2M         0x0F //
//setup of Address Widths
#define VAL_SETUP_AW_3B         1    // 3 byte
#define VAL_SETUP_AW_4B         2    // 4 byte
#define VAL_SETUP_AW_5B         3    // 5 byte
		
 #ifdef __cplusplus
	}
 #endif
 #endif
