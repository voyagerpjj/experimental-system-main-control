/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025,  China,  IFR Laboratory, DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_custom_usart_protocol.h
  * Version		: v1.0
  * Author		: panjiajun
  * Date			: 2025-11-10
  * Description	:
  *********************************************************************
  */

#ifndef __IFR_CUSTOM_USART_PROTOCOL_H_
#define __IFR_CUSTOM_USART_PROTOCOL_H_
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif
#include "main.h"
#ifdef HAL_UART_MODULE_ENABLED
#if USE_HAL_UART_REGISTER_CALLBACKS && USE_HAL_USART_REGISTER_CALLBACKS
#include "manager_usart.h"
#include "crc.h"

// ********** 配置项：按需修改最大数据段长度 **********
#define FRAME_MAX_DATA_LEN 24  						// 协议最大数据段长度（当前正常帧24，错误帧0）
#define FRAME_MIN_LEN 6  									// 协议最短长度（帧头2 + Type1 + len1 + CRC2 = 6）
#define FRAME_HEAD1          0x5A        // 帧头1
#define FRAME_HEAD2          0xA5        // 帧头2
#define FRAME_TYPE_LEN       1           // Type字段长度（字节）
#define FRAME_LEN_FIELD_LEN  1           // LEN字段长度（字节）
#define FRAME_CRC_LEN        2           // CRC字段长度（字节）

/************************ 解析结果状态 ************************/
typedef enum
{
    PARSE_SUCCESS = 0,        // 解析成功
    PARSE_ERR_NO_HEAD,        // 未找到有效帧头
    PARSE_ERR_BUF_NOT_ENOUGH, // 缓冲区数据不足（小于帧总长度）
    PARSE_ERR_CRC,            // CRC校验失败
} ifr_custom_protocol_parse_status_enum;

typedef struct
{
	uint8_t DataType;				// 数据帧类型
	uint8_t DataLenght;			// 数据段长度
	uint8_t Data[FRAME_MAX_DATA_LEN];	// 数据段数据
} ifr_custom_protocol_typedef;

HAL_StatusTypeDef ifr_custom_protocol_transmit(manager_usart_typedef *manager_usart, uint8_t frame_type, uint8_t data_len, const uint8_t *data);
ifr_custom_protocol_parse_status_enum ifr_custom_protocol_analysis(ifr_custom_protocol_typedef *prame_parse, uint8_t *pdata, uint8_t len);


#endif  // USE_HAL_UART_REGISTER_CALLBACKS && USE_HAL_USART_REGISTER_CALLBACKS
#endif  // HAL_UART_MODULE_ENABLED
#endif	// __IFR_CUSTOM_USART_PROTOCOL_H_
