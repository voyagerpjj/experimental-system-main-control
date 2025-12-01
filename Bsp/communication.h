/*
 * @FilePath     : communication
 * @Author       : zhanglu
 * @Date         : 2022-10-11
 * @LastEditors  : zhanglu
 * @LastEditTime : 2022-10-11
 * @Description  : 
 * 
 * Copyright (c) 2022 by BZLZ, All Rights Reserved. 
 */

#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#define UART_RX_BUF_SIZE 120
#define UART_TX_BUF_SIZE 120

#include "main.h"
#include "usart.h"
#include "crc.h"

typedef enum
{
    DEVICE_POWER_MSSAGE = 0,   /* 处理类消息 */
    MOTOR_CRTL_MESSAGE,
    FORWARD_MESSAGE,    /* 转发类消息 */
    ERROR_MESSAGE,
}messageType_e;

typedef struct
{
  uint8_t transmit_buff[UART_TX_BUF_SIZE];
  uint8_t receive_buff[UART_RX_BUF_SIZE];
	uint8_t receiveTemporaryBuff[UART_RX_BUF_SIZE];
	UART_HandleTypeDef* huart;
	uint8_t receiveFlag;
	uint16_t receiveCounter;
}uartParams_t;

void commInit(void);
void dataTranspond(uartParams_t* huart_rx, uartParams_t* huart_tx, uint8_t len);
void dataTransmit(uartParams_t* huart, uint8_t* data_buff, uint8_t len);
messageType_e dataReserveDecoder(uint8_t* dataBuff, uint8_t len);
uint8_t GetReceiveCounter(uartParams_t* huart_rx);
bool CrcCheck(uint8_t *ptr, uint8_t len);
//void UART_IDLE_Callback(UART_HandleTypeDef *huart);

#endif
