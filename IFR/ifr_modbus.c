#include "ifr_modbus.h"
#include <stdio.h>

// -------------------------- 全局单串口上下文（替代多串口数组） --------------------------
static modbus_rtu_t *g_modbus_ctx = NULL;

// -------------------------- 私有函数声明 --------------------------
static modbus_error_t build_rtu_frame(modbus_rtu_t *ctx, const modbus_frame_t *frame, uint8_t *buffer, uint16_t *length);
static modbus_error_t parse_rtu_frame(modbus_rtu_t *ctx, uint8_t *buffer, uint16_t length, modbus_frame_t *frame);
static void parse_input_regs_response(modbus_rtu_t *ctx, modbus_frame_t *frame);
void modbus_rtu_receive_callback(uint8_t *data, uint8_t len, void *user_data);

// -------------------------- 中转回调（单串口简化版） --------------------------
void modbus_usart_analysis_cb(uint8_t *pData, uint8_t len) {
    if (g_modbus_ctx != NULL) {
        modbus_rtu_receive_callback(pData, len, g_modbus_ctx);
    }
}

// -------------------------- 初始化函数（单串口简化） --------------------------
modbus_error_t modbus_rtu_init(modbus_rtu_t *ctx, 
                               UART_HandleTypeDef *huart,
                               modbus_data_callback_t cb) {
    if (ctx == NULL || huart == NULL) {
        return MODBUS_ERROR_INVALID_PARAM;
    }
    
    // 清空上下文
    memset(ctx, 0, sizeof(modbus_rtu_t));
    
    // 初始化内嵌串口对象
    ctx->usart._huart = huart;
    ctx->huart = huart;
    ctx->data_cb = cb;
    ctx->response_timeout_ms = MODBUS_RESPONSE_TIMEOUT_MS;
    ctx->state = MODBUS_MASTER_IDLE;
    ctx->state_timestamp = HAL_GetTick();
    
    // 绑定全局上下文（单串口）
    g_modbus_ctx = ctx;
    
    // 注册串口回调
    IFR_USART_Init(&ctx->usart, huart, modbus_usart_analysis_cb);
    
    return MODBUS_SUCCESS;
}

// -------------------------- 帧构建 --------------------------
static modbus_error_t build_rtu_frame(modbus_rtu_t *ctx, const modbus_frame_t *frame, uint8_t *buffer, uint16_t *length) {
    if (frame == NULL || buffer == NULL || length == NULL) {
        return MODBUS_ERROR_INVALID_PARAM;
    }
    
    if (frame->slave_addr > MODBUS_MAX_ADDR || frame->function != MODBUS_FC_READ_INPUT_REGS) {
        return MODBUS_ERROR_INVALID_PARAM;
    }
    if (frame->length > MODBUS_MAX_PDU_SIZE) {
        return MODBUS_ERROR_FRAME;
    }
    
    // 构建RTU帧：地址 + 功能码 + 数据 + CRC
    uint16_t pos = 0;
    buffer[pos++] = frame->slave_addr;
    buffer[pos++] = frame->function;
    
    if (frame->length > 0) {
        memcpy(&buffer[pos], frame->data, frame->length);
        pos += frame->length;
    }
    
    // CRC计算
    uint16_t crc = CalcCRC16_Modbus(buffer, pos);
    buffer[pos++] = crc & 0xFF;
    buffer[pos++] = (crc >> 8) & 0xFF;
    
    *length = pos;
    return MODBUS_SUCCESS;
}

// -------------------------- 帧解析 --------------------------
static modbus_error_t parse_rtu_frame(modbus_rtu_t *ctx, uint8_t *buffer, uint16_t length, modbus_frame_t *frame) {
    if (buffer == NULL || frame == NULL || length < MODBUS_RTU_MIN_SIZE) {
        return MODBUS_ERROR_FRAME;
    }
    
    // CRC校验
    uint16_t crc_received = (buffer[length-1] << 8) | buffer[length-2];
    uint16_t crc_calc = CalcCRC16_Modbus(buffer, length-2);
    if (crc_received != crc_calc) {
        return MODBUS_ERROR_CRC;
    }
    
    // 解析帧结构
    memset(frame, 0, sizeof(modbus_frame_t));
    frame->slave_addr = buffer[0];
    frame->function = buffer[1];
    frame->length = length - 4;  // 地址+功能码+CRC(2)
    
    // 处理异常响应
    if (frame->function & 0x80) {
        frame->data[0] = buffer[2];
        return MODBUS_ERROR_FRAME;
    }
    
    // 复制数据段
    if (frame->length > 0) {
        if (frame->length > MODBUS_MAX_PDU_SIZE) {
            return MODBUS_ERROR_FRAME;
        }
        memcpy(frame->data, &buffer[2], frame->length);
    }
    
    frame->crc = crc_received;
    return MODBUS_SUCCESS;
}

// -------------------------- 响应解析 --------------------------
static void parse_input_regs_response(modbus_rtu_t *ctx, modbus_frame_t *frame) {
    if (ctx == NULL || frame == NULL || ctx->data_cb == NULL) {
        return;
    }
    
    // 提取寄存器数量
    uint16_t reg_count = (ctx->current_request.data[2] << 8) | ctx->current_request.data[3];
    uint16_t reg_values[125] = {0};
    
    // 校验响应合法性
    if (frame->function != MODBUS_FC_READ_INPUT_REGS) return;
    uint8_t expected_bytes = reg_count * 2;
    if (frame->length != (expected_bytes + 1) || frame->data[0] != expected_bytes) {
        return;
    }
    
    // 解析寄存器值
    for (uint16_t i = 0; i < reg_count; i++) {
        reg_values[i] = (frame->data[1 + i*2] << 8) | frame->data[2 + i*2];
    }
    
    // 调用用户回调
    ctx->data_cb(frame, reg_values, reg_count);
}

// -------------------------- 接收回调 --------------------------
void modbus_rtu_receive_callback(uint8_t *data, uint8_t len, void *user_data) {
    modbus_rtu_t *ctx = (modbus_rtu_t *)user_data;
    if (ctx == NULL || data == NULL || len == 0) {
        return;
    }
    
    // 仅处理等待响应的状态
    if (ctx->state != MODBUS_MASTER_WAITING_RESPONSE) {
        return;
    }
    
    // 解析帧
    modbus_frame_t frame;
    modbus_error_t err = parse_rtu_frame(ctx, data, len, &frame);
    
    if (err == MODBUS_SUCCESS) {
        // 校验从机地址匹配
        if (frame.slave_addr == ctx->current_request.slave_addr) 
				{
            ctx->current_response = frame;
            ctx->state = MODBUS_MASTER_COMPLETE;
            ctx->rx_count++;
            parse_input_regs_response(ctx, &frame);
        }
    } else {
        ctx->error_count++;
        ctx->state = MODBUS_MASTER_ERROR;
    }
}

// -------------------------- 构建+发送函数 --------------------------
modbus_error_t modbus_send_read_input_regs(modbus_rtu_t *ctx, 
                                           uint8_t slave_addr,
                                           uint8_t func_code,
                                           uint16_t start_addr,
                                           uint16_t reg_count) {
    // 参数校验
    if (ctx == NULL || func_code != MODBUS_FC_READ_INPUT_REGS || 
        slave_addr == 0 || slave_addr > MODBUS_MAX_ADDR || 
        reg_count == 0 || reg_count > 125 || modbus_master_is_busy(ctx)) {
        return MODBUS_ERROR_INVALID_PARAM;
    }
    
    // 构建请求帧
    modbus_frame_t request;
    memset(&request, 0, sizeof(modbus_frame_t));
    request.slave_addr = slave_addr;
    request.function = func_code;
    request.length = 4;
    request.data[0] = (start_addr >> 8) & 0xFF;
    request.data[1] = start_addr & 0xFF;
    request.data[2] = (reg_count >> 8) & 0xFF;
    request.data[3] = reg_count & 0xFF;
    
    // 构建RTU帧并发送
    uint8_t buffer[MODBUS_MAX_ADU_SIZE];
    uint16_t frame_len;
    modbus_error_t err = build_rtu_frame(ctx, &request, buffer, &frame_len);
    if (err != MODBUS_SUCCESS) return err;
    
    // 串口DMA发送
    HAL_StatusTypeDef hal_err = HAL_UART_Transmit_DMA(ctx->huart, buffer, frame_len);
    if (hal_err != HAL_OK) {
        ctx->error_count++;
        ctx->state = MODBUS_MASTER_ERROR;
        return MODBUS_ERROR_COMMUNICATION;
    }
    
    // 更新状态
    ctx->current_request = request;
    ctx->state = MODBUS_MASTER_WAITING_RESPONSE;
    ctx->state_timestamp = HAL_GetTick();
    ctx->tx_count++;
    
    return MODBUS_SUCCESS;
}

// -------------------------- 状态检查 --------------------------
bool modbus_master_is_busy(modbus_rtu_t *ctx) {
    if (ctx == NULL) return false;
    return (ctx->state == MODBUS_MASTER_WAITING_RESPONSE);
}

// -------------------------- 超时处理 --------------------------
modbus_error_t modbus_master_process(modbus_rtu_t *ctx) {
    if (ctx == NULL) return MODBUS_ERROR_INVALID_PARAM;
    
    uint32_t current_time = HAL_GetTick();
    if (ctx->state == MODBUS_MASTER_WAITING_RESPONSE) {
        if (current_time - ctx->state_timestamp > ctx->response_timeout_ms) {
            ctx->state = MODBUS_MASTER_TIMEOUT;
            ctx->error_count++;
            return MODBUS_ERROR_TIMEOUT;
        }
    }
    
    return MODBUS_SUCCESS;
}

// -------------------------- 状态重置 --------------------------
void modbus_master_reset(modbus_rtu_t *ctx) 
{
    if (ctx != NULL) 
		{
			ctx->state = MODBUS_MASTER_IDLE;
			memset(&ctx->current_request, 0, sizeof(modbus_frame_t));
			memset(&ctx->current_response, 0, sizeof(modbus_frame_t));
    }
}
// 文件末尾添加空行