/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: manager_usart.h
  * Version		: v3.0
  * Author		: LiuHao Lijiawei Albert panjiajun
  * Date			: 2025-11-10
  * Description	: IFR串口库驱动层，用于初始化串口设备、注册解析回调函数、调用中间层来发送和接收数据
  *********************************************************************
  */

#ifndef __MANAGER_USART_H_
#define __MANAGER_USART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED
#if USE_HAL_UART_REGISTER_CALLBACKS && USE_HAL_USART_REGISTER_CALLBACKS

#include "usart.h"


// 接收缓冲区大小
#define USART_RX_RING_BUFFER_SIZE 128

/**
 * @brief   串口通信结构体
 */
typedef struct
{
    UART_HandleTypeDef *_huart;                          // UART句柄
    void(*AnalysisFunc)(uint8_t *pData, uint8_t len);    // 数据解析函数指针
    uint8_t Buffer_Num;                                  // 当前缓冲区索引
    uint8_t rx_dma_buffer_[2][USART_RX_RING_BUFFER_SIZE]; // DMA接收双缓冲
    uint32_t _updata_systick;                            // 最后更新时间戳
    uint16_t Data_Length;                                // 接收数据长度
    HAL_StatusTypeDef UsartRxState;                      // 串口接收状态
} manager_usart_typedef;

// 串口初始化
void manager_usart_Init(manager_usart_typedef *manager_usart, UART_HandleTypeDef *huart, void(*UART_Analysis_Function)( uint8_t *pData, uint8_t len));

#endif  // USE_HAL_UART_REGISTER_CALLBACKS && USE_HAL_USART_REGISTER_CALLBACKS
#endif  // HAL_UART_MODULE_ENABLED

#ifdef __cplusplus
}
#endif

#endif  // __MANAGER_USART_H_
