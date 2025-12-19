#include "modbus.h"
#include <stdio.h>

// -------------------------- 全局单串口上下文（替代多串口数组） --------------------------
static modbus_rtu_t *g_modbus_ctx = NULL;

// -------------------------- 私有函数声明 --------------------------
static modbus_error_e modbus_build_rtu_frame(modbus_rtu_t *ctx, const modbus_frame_t *frame, uint8_t *buffer, uint16_t *length);
static modbus_error_e modbus_parse_rtu_frame(modbus_rtu_t *ctx, uint8_t *buffer, uint16_t length, modbus_frame_t *frame);
static void modbus_parse_input_regs_response(modbus_rtu_t *ctx, modbus_frame_t *frame, uint8_t *data, uint8_t len);
void modbus_rtu_receive_callback(uint8_t *data, uint8_t len, void *user_data);
void modbus_rtu_tx_complete_callback(UART_HandleTypeDef *huart, void *user_data);



// -------------------------- 中转回调（单串口简化版） --------------------------
void modbus_usart_analysis_cb(uint8_t *pData, uint8_t len) {
    if (g_modbus_ctx != NULL) {
        modbus_rtu_receive_callback(pData, len, g_modbus_ctx);
    }
}
// -------------------------- 中转回调（单串口简化版） --------------------------
void modbus_usart_tx_complete_cb(UART_HandleTypeDef *huart) {
    if (g_modbus_ctx != NULL) {
        modbus_rtu_tx_complete_callback(huart, g_modbus_ctx);
    }
}
// -------------------------- 初始化函数（单串口简化） --------------------------
modbus_error_e modbus_rtu_init(modbus_rtu_t *ctx, UART_HandleTypeDef *huart, modbus_data_callback_t cb) 
{
    if (ctx == NULL || huart == NULL) {
        return MODBUS_ERROR_INVALID_PARAM;
    }
    
    // 清空上下文
    memset(ctx, 0, sizeof(modbus_rtu_t));
    
    // 初始化内嵌串口对象
    ctx->ManagerUsart._huart = huart;
    ctx->huart = huart;
    ctx->DataPrameFuncPointer = cb;
    ctx->ResponseTimeoutMs = MODBUS_RESPONSE_TIMEOUT_MS;
    ctx->State = MODBUS_STATE_IDLE;
    ctx->StateTimestamp = HAL_GetTick();
    
    // 绑定全局上下文（单串口）
    g_modbus_ctx = ctx;
    
    // 注册串口回调
    manager_usart_init(&ctx->ManagerUsart, huart, modbus_usart_analysis_cb);
    
    return MODBUS_SUCCESS;
}
/**
  * @brief   ModBus注册发送完成中断回调函数
  * @param   manager_usart: 串口对象指针
  * @param   UART_Analysis_Function: 发送完成中断回调函数指针
  * @retval  void
  */
modbus_error_e modbus_rtu_register_tx_callback(modbus_rtu_t *ctx, void (tx_callback)(UART_HandleTypeDef *huart)) 
{
	if (ctx == NULL) 
        return MODBUS_ERROR_INVALID_PARAM;
	if (tx_callback != NULL)
			ctx->TxCompleteFuncPointer = tx_callback;
	manager_usart_register_tx_callback(&ctx->ManagerUsart, ctx->TxCompleteFuncPointer);
	return MODBUS_SUCCESS;
}

// -------------------------- 启动函数（单串口简化） --------------------------
modbus_error_e modbus_rtu_start(modbus_rtu_t *ctx)
{
    if (ctx == NULL) {
        return MODBUS_ERROR_INVALID_PARAM;
    }
    manager_usart_start(&ctx->ManagerUsart); // 启动串口接收
    return MODBUS_SUCCESS;
}
// -------------------------- 帧构建 --------------------------
static modbus_error_e modbus_build_rtu_frame(modbus_rtu_t *ctx, const modbus_frame_t *frame, uint8_t *buffer, uint16_t *length) {
    if (frame == NULL || buffer == NULL || length == NULL) {
        return MODBUS_ERROR_INVALID_PARAM;
    }
    
    if (frame->SlaveAddr > MODBUS_MAX_ADDR || frame->FunctionId != MODBUS_FC_READ_INPUT_REGS) {
        return MODBUS_ERROR_INVALID_PARAM;
    }
    if (frame->Length > MODBUS_MAX_PDU_SIZE) {
        return MODBUS_ERROR_FRAME;
    }
    
    // 构建RTU帧：地址 + 功能码 + 数据 + CRC
    uint16_t pos = 0;
    buffer[pos++] = frame->SlaveAddr;
    buffer[pos++] = frame->FunctionId;
    
    if (frame->Length > 0) {
        memcpy(&buffer[pos], frame->Data, frame->Length);
        pos += frame->Length;
    }
    
    // CRC计算
    uint16_t crc = CalcCRC16_Modbus(buffer, pos);
    buffer[pos++] = (crc >> 8) & 0xFF;
    buffer[pos++] = crc & 0xFF;
    
    *length = pos;
    return MODBUS_SUCCESS;
}

// -------------------------- 帧解析 --------------------------
static modbus_error_e modbus_parse_rtu_frame(modbus_rtu_t *ctx, uint8_t *buffer, uint16_t length, modbus_frame_t *frame) {
    if (buffer == NULL || frame == NULL || length < MODBUS_RTU_MIN_SIZE) {
        return MODBUS_ERROR_FRAME;
    }
    
    // CRC校验
    uint16_t crc_received = (buffer[length - 2] << 8) | buffer[length - 1];
    uint16_t crc_calc = CalcCRC16_Modbus(buffer, length - 2);
    if (crc_received != crc_calc) {
        return MODBUS_ERROR_CRC;
    }
    
    // 解析帧结构
    memset(frame, 0, sizeof(modbus_frame_t));
    frame->SlaveAddr = buffer[0];
    frame->FunctionId = buffer[1];
    frame->Length = length - 4;  // 地址+功能码+CRC(2)
    
    // 复制数据段
    if (frame->Length > 0) {
        if (frame->Length > MODBUS_MAX_PDU_SIZE) {
            return MODBUS_ERROR_FRAME;
        }
        memcpy(frame->Data, &buffer[2], frame->Length);
    }
    
    frame->Crc = crc_received;
    return MODBUS_SUCCESS;
}

// -------------------------- 响应解析 --------------------------
static void modbus_parse_input_regs_response(modbus_rtu_t *ctx, modbus_frame_t *frame, uint8_t *data, uint8_t len) 
{
    if (ctx == NULL || frame == NULL || ctx->DataPrameFuncPointer == NULL) {
        return;
    }
    // 调用用户回调
    ctx->DataPrameFuncPointer(frame, data, len);
}

// -------------------------- 接收回调 --------------------------
void modbus_rtu_receive_callback(uint8_t *data, uint8_t len, void *user_data) 
{
    modbus_rtu_t *ctx = (modbus_rtu_t *)user_data;
    if (ctx == NULL || data == NULL || len == 0) {
        return;
    }
    
    // 解析帧
    static modbus_frame_t frame;
    modbus_error_e err = modbus_parse_rtu_frame(ctx, data, len, &frame);
    
    if (err == MODBUS_SUCCESS) 
		{
			ctx->CurrentRequest = frame;
			ctx->State = MODBUS_STATE_COMPLETE;
			ctx->RxCount++;
			
			modbus_parse_input_regs_response(ctx, &frame, data, len);
    }
}
void modbus_rtu_tx_complete_callback(UART_HandleTypeDef *huart, void *user_data)
{
	modbus_rtu_t *ctx = (modbus_rtu_t *)user_data;
	if (ctx->TxCompleteFuncPointer != NULL)
		ctx->TxCompleteFuncPointer(huart);
}
// -------------------------- 构建+发送函数 --------------------------
modbus_error_e modbus_send_read_input_regs(modbus_rtu_t *ctx, 
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
    request.SlaveAddr = slave_addr;
    request.FunctionId = func_code;
    request.Length = 4;
    request.Data[0] = (start_addr >> 8) & 0xFF;
    request.Data[1] = start_addr & 0xFF;
    request.Data[2] = (reg_count >> 8) & 0xFF;
    request.Data[3] = reg_count & 0xFF;
    
    // 构建RTU帧并发送
    uint8_t buffer[MODBUS_MAX_ADU_SIZE];
    uint16_t frame_len;
    modbus_error_e err = modbus_build_rtu_frame(ctx, &request, buffer, &frame_len);
    if (err != MODBUS_SUCCESS) return err;
    
    // 串口DMA发送
    HAL_StatusTypeDef hal_err = manager_usart_transmit(&ctx->ManagerUsart, buffer, frame_len);
    if (hal_err != HAL_OK) 
		{
        ctx->ErrorCount++;
        ctx->State = MODBUS_STATE_ERROR;
        return MODBUS_ERROR_COMMUNICATION;
    }
    
    // 更新状态
    ctx->CurrentRequest = request;
    ctx->State = MODBUS_STATE_WAITING_RESPONSE;
    ctx->StateTimestamp = HAL_GetTick();
    ctx->TxCount++;
    
    return MODBUS_SUCCESS;
}

// -------------------------- 状态检查 --------------------------
bool modbus_master_is_busy(modbus_rtu_t *ctx) {
    if (ctx == NULL) return false;
    return (ctx->State == MODBUS_STATE_WAITING_RESPONSE);
}
float count_com, time_all = 0;
float time_avg = 0;
// -------------------------- 超时处理 --------------------------
modbus_error_e modbus_master_process(modbus_rtu_t *ctx) 
{
    if (ctx == NULL) return MODBUS_ERROR_INVALID_PARAM;
    
    uint32_t current_time = HAL_GetTick();
    if (ctx->State == MODBUS_STATE_WAITING_RESPONSE) 
		{
			ctx->ResponseTimenow = current_time - ctx->StateTimestamp;
			time_all += ctx->ResponseTimenow;
			time_avg = time_all / count_com ++ ;
			if (ctx->ResponseTimenow >= ctx->ResponseTimeoutMs) 
			{
				ctx->State = MODBUS_STATE_TIMEOUT;
				ctx->ErrorCount++;
				return MODBUS_ERROR_TIMEOUT;
			}
    }
    else if (ctx->State == MODBUS_STATE_ERROR)
    {
        return MODBUS_ERROR_COMMUNICATION;
    }
    
    return MODBUS_SUCCESS;
}

// -------------------------- 状态重置 --------------------------
void modbus_master_reset(modbus_rtu_t *ctx) 
{
    if (ctx != NULL) 
	{
        ctx->State = MODBUS_STATE_IDLE;
        memset(&ctx->CurrentRequest, 0, sizeof(modbus_frame_t));
        memset(&ctx->CurrentResponse, 0, sizeof(modbus_frame_t));
    }
}
