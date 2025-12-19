#ifndef __MODBUS_H_
#define __MODBUS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "crc.h"
#include "manager_usart.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// -------------------------- 核心常量定义 --------------------------
// Modbus 功能码定义
#define MODBUS_FC_READ_COILS          0x01
#define MODBUS_FC_READ_DISCRETE_INPUT 0x02
#define MODBUS_FC_READ_HOLDING_REGS   0x03
#define MODBUS_FC_READ_INPUT_REGS     0x04
#define MODBUS_FC_WRITE_SINGLE_COIL   0x05
#define MODBUS_FC_WRITE_SINGLE_REG    0x06
#define MODBUS_FC_WRITE_MULTI_COILS   0x0F
#define MODBUS_FC_WRITE_MULTI_REGS    0x10

// Modbus 异常码定义
#define MODBUS_EX_NONE                0x00
#define MODBUS_EX_ILLEGAL_FUNCTION    0x01
#define MODBUS_EX_ILLEGAL_DATA_ADDR   0x02
#define MODBUS_EX_ILLEGAL_DATA_VALUE  0x03
#define MODBUS_EX_SLAVE_DEVICE_FAIL   0x04
#define MODBUS_EX_ACK                 0x05
#define MODBUS_EX_SLAVE_BUSY          0x06
#define MODBUS_EX_MEMORY_PARITY_ERROR 0x08
#define MODBUS_EX_GATEWAY_PATH_FAIL   0x0A
#define MODBUS_EX_GATEWAY_TARGET_FAIL 0x0B

// 协议相关常量
#define MODBUS_BROADCAST_ADDR         0
#define MODBUS_MAX_ADDR               247
#define MODBUS_MAX_PDU_SIZE           253
#define MODBUS_MAX_ADU_SIZE           256
#define MODBUS_RTU_MIN_SIZE           4
#define MODBUS_RTU_MAX_SIZE           256
#define MODBUS_CRC_SIZE               2
#define MODBUS_RESPONSE_TIMEOUT_MS    5  // 3ms响应超时

// -------------------------- 错误码定义 --------------------------
typedef enum {
    MODBUS_SUCCESS = 0,
    MODBUS_ERROR_INVALID_PARAM,    // 无效参数
    MODBUS_ERROR_TIMEOUT,          // 超时
    MODBUS_ERROR_CRC,              // CRC错误
    MODBUS_ERROR_FRAME,            // 帧格式错误
    MODBUS_ERROR_COMMUNICATION,    // 通信失败
    MODBUS_ERROR_BUSY              // 主站忙
} modbus_error_e;

// -------------------------- 帧结构定义 --------------------------
typedef struct {
    uint8_t  SlaveAddr;    // 从机地址 (1-247)
    uint8_t  FunctionId;   // 功能码
    uint16_t Crc;          // CRC校验值
    uint16_t Length;       // 数据段长度
    uint8_t  Data[MODBUS_MAX_PDU_SIZE];  // PDU数据
} modbus_frame_t;

// -------------------------- 主站状态机 --------------------------
typedef enum {
    MODBUS_STATE_IDLE = 0,          // 空闲
    MODBUS_STATE_WAITING_RESPONSE,  // 等待响应
    MODBUS_STATE_COMPLETE,          // 响应完成
    MODBUS_STATE_TIMEOUT,           // 超时
    MODBUS_STATE_ERROR              // 错误
} modbus_master_state_e;

// -------------------------- 回调函数类型 --------------------------
typedef void (*modbus_data_callback_t)(modbus_frame_t *frame, uint8_t *data, uint8_t len);

// -------------------------- 核心上下文（内嵌串口对象，单串口） --------------------------
typedef struct {
    manager_usart_typedef ManagerUsart; // 内嵌串口对象（无需单独定义）
    UART_HandleTypeDef *huart;        // HAL串口句柄
    modbus_data_callback_t DataPrameFuncPointer;   // 用户解析回调
		void(*TxCompleteFuncPointer)(UART_HandleTypeDef *huart);    // 发送完成函数指针
		uint32_t ResponseTimenow;					// 当前响应时间
    uint32_t ResponseTimeoutMs;     	// 超时时间
    modbus_master_state_e State;      // 状态机
    uint32_t StateTimestamp;         // 状态时间戳
    modbus_frame_t CurrentRequest;   // 当前请求帧
    modbus_frame_t CurrentResponse;  // 当前响应帧
    uint32_t TxCount;                // 发送计数
    uint32_t RxCount;                // 接收计数
    uint32_t ErrorCount;             // 错误计数
} modbus_rtu_t;

// -------------------------- 核心API --------------------------
modbus_error_e modbus_rtu_init(modbus_rtu_t *ctx, UART_HandleTypeDef *huart, modbus_data_callback_t cb);
modbus_error_e modbus_rtu_register_tx_callback(modbus_rtu_t *ctx, void (tx_callback)(UART_HandleTypeDef *huart));
modbus_error_e modbus_rtu_start(modbus_rtu_t *ctx);
modbus_error_e modbus_send_read_input_regs(modbus_rtu_t *ctx, 
                                           uint8_t slave_addr,
                                           uint8_t func_code,
                                           uint16_t start_addr,
                                           uint16_t reg_count);

modbus_error_e modbus_master_process(modbus_rtu_t *ctx);
bool modbus_master_is_busy(modbus_rtu_t *ctx);
void modbus_master_reset(modbus_rtu_t *ctx);

#ifdef __cplusplus
}
#endif

#endif  // MODBUS_H_
