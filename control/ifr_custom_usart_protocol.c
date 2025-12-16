#include "ifr_custom_usart_protocol.h"
#ifdef HAL_UART_MODULE_ENABLED
#if USE_HAL_UART_REGISTER_CALLBACKS && USE_HAL_USART_REGISTER_CALLBACKS
// 全局/静态发送缓冲区（避免DMA发送时栈数据被覆盖）
static uint8_t ttl_tx_buffer[FRAME_MIN_LEN + FRAME_MAX_DATA_LEN] = {0}; // 帧头2+类型1+长度1+数据N+CRC2
// IFR自定义串口协议发送函数
HAL_StatusTypeDef ifr_custom_protocol_transmit(manager_usart_typedef *manager_usart, uint8_t frame_type, uint8_t data_len, const uint8_t *data)
{
	// 校验参数（数据段长度不超过最大值）
    if (data_len > FRAME_MAX_DATA_LEN) return HAL_ERROR;
    
    // 拼接帧头、帧类型、数据段长度
    uint16_t buf_offset = 0;
    ttl_tx_buffer[buf_offset++] = FRAME_HEAD1;    // 帧头1
    ttl_tx_buffer[buf_offset++] = FRAME_HEAD2;    // 帧头2
    ttl_tx_buffer[buf_offset++] = frame_type;         // 帧类型
    ttl_tx_buffer[buf_offset++] = data_len;           // 数据段长度
    
    // 3. 拼接数据段（无数据则跳过）
    if (data_len > 0 && data != NULL)
    {
        memcpy(&ttl_tx_buffer[buf_offset], data, data_len);
    }
    buf_offset += data_len;
    
    // 计算并拼接CRC16（校验范围：帧头到数据段末尾）
    uint16_t crc_val = CalcCRC16_Modbus(ttl_tx_buffer, buf_offset);
    ttl_tx_buffer[buf_offset++] = (crc_val >> 8) & 0xFF; // CRC高字节
    ttl_tx_buffer[buf_offset++] = crc_val & 0xFF;        // CRC低字节
    
    // DMA发送（检查DMA状态，避免冲突）
    return manager_usart_transmit(manager_usart, ttl_tx_buffer, buf_offset);
    
}

// IFR自定义串口协议接收解析函数
ifr_custom_protocol_parse_status_enum ifr_custom_protocol_analysis(ifr_custom_protocol_typedef *prame_parse, uint8_t *pdata, uint8_t len)
{
	if (len < FRAME_MIN_LEN)	// 若帧长度少于最少长度则退出
		return PARSE_ERR_BUF_NOT_ENOUGH;
	
	// 查找帧头（0x5A 0xA5）
	uint16_t head_offset = 0; // 帧头在缓冲区中的偏移
	for (; head_offset < len - 1; head_offset++)
	{
		if (pdata[head_offset] == FRAME_HEAD1 && pdata[head_offset + 1] == FRAME_HEAD2)
				break; // 找到有效帧头，退出遍历
	}
	if (head_offset >= len - 1)
		return PARSE_ERR_NO_HEAD; // 未找到帧头
	

	uint8_t load_len = pdata[head_offset + 3];			// 提取出帧的数据段长度，但是先不往帧指针里面填
	// 计算整帧总长度，检查缓冲区数据是否足够
	uint16_t frame_total_len = 2 + FRAME_TYPE_LEN + FRAME_LEN_FIELD_LEN + load_len + FRAME_CRC_LEN;
	if (head_offset + frame_total_len > len)
			return PARSE_ERR_BUF_NOT_ENOUGH; // 缓冲区数据不足整帧长度
	
	// CRC16校验（校验范围：帧头+Type+LEN+Load，共 head_offset到head_offset+4+load_len-1）
	uint16_t crc_calc = CalcCRC16_Modbus(&pdata[head_offset], 4 + load_len);
	// 提取帧中的CRC（高字节在前）
	uint16_t crc_frame = (pdata[head_offset + 4 + load_len] << 8) | pdata[head_offset + 5 + load_len];
	if (crc_calc != crc_frame)
			return PARSE_ERR_CRC; // CRC校验失败
	
	// 上方的所有校验完成说明此帧没有问题，开始存储帧
	prame_parse->DataType = pdata[head_offset + 2];	// 提取出帧的类型
	prame_parse->DataLenght = load_len;							// 提取出帧的数据段长度
	memcpy(prame_parse->Data, &pdata[head_offset + 4], load_len); // 提取帧的数据段数据
	
	return PARSE_SUCCESS;
}

#endif  // USE_HAL_UART_REGISTER_CALLBACKS && USE_HAL_USART_REGISTER_CALLBACKS
#endif  // HAL_UART_MODULE_ENABLED
