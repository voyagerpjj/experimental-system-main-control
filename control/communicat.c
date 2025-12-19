#include "communicat.h"
static rx_ttl_message_typedef rx_ttl_message = {0};
liquid_level_transmitter_typedef liquid_level_transmitter[4] = {0};
liquid_flow_collection_typedef liquid_flow_collection = {0};

modbus_rtu_t usart_485_modbus;
manager_usart_typedef usart_ttl;

void communicat_ttl_analysisFunc(uint8_t *pData, uint8_t len);
void communicat_485_analysisFunc(modbus_frame_t *frame, uint8_t *data, uint8_t len);
void communicat_485_tx_analysisFunc(UART_HandleTypeDef *huart);

static bool check_slave_idx_valid(void);
static void reset_device_state(uint8_t slave_addr, HAL_StatusTypeDef state);
static uint8_t slave_switch_idx(uint8_t current_idx, uint8_t slave_count);

// 通信初始化
void communicat_init(void)
{
	modbus_rtu_init(&usart_485_modbus, &huart1, communicat_485_analysisFunc);	// 下位机485串口1初始化
	modbus_rtu_register_tx_callback(&usart_485_modbus, communicat_485_tx_analysisFunc);
	manager_usart_init(&usart_ttl, &huart4, communicat_ttl_analysisFunc);		// 上位机TTL串口4初始化
	for (int i = 1; i < 5; i++)	// 初始化ID
	{
		liquid_level_transmitter[i - 1].ID = i;
	}
	liquid_flow_collection.ID = 0x05;
}
// 通信启动函数
void communicat_start(void)
{
	modbus_rtu_start(&usart_485_modbus);	// 启动485串口接收
	manager_usart_start(&usart_ttl);			// 启动TTL串口接收
}

uint32_t tt1, tt2, tt3 = 0;
// 上位机下发的ttl数据解析函数
// 帧头 			2字节 0x5A 0xA5 
// 帧类型 		1字节 0x01或0x03
// 数据段长度 1字节 
// 数据段 		n字节
// CRC16校验 	2字节
void communicat_ttl_analysisFunc(uint8_t *pData, uint8_t len)
{
	tt2 = HAL_GetTick() - tt1;
	tt1 = HAL_GetTick();
	HAL_GPIO_TogglePin(STATE_GPIO_Port, STATE_Pin);
	if (rx_ttl_message.rx_message_state == MOTOR_CONTROL_MSG)	// 若还未进入配置模式，还处于自定义串口协议中
	{
		ifr_custom_protocol_typedef ifr_custom_protocol_ttl_prame;
		ifr_custom_protocol_parse_status_enum ifr_custom_protocol_parse_status;
		ifr_custom_protocol_parse_status = ifr_custom_protocol_analysis(&ifr_custom_protocol_ttl_prame, pData, len);
		if (ifr_custom_protocol_parse_status == PARSE_SUCCESS)	// 若接收成功
		{
			rx_ttl_message.state = UPDATA;	// 正常接收则标识数据已更新
			if (ifr_custom_protocol_ttl_prame.DataType == 0x01)
			{
				rx_ttl_message.rx_message_state = MOTOR_CONTROL_MSG;
					
				// 变送器采集配置解析
				liquid_level_transmitter[0].liquid_level_transmitter_state = (ifr_custom_protocol_ttl_prame.Data[0] & 0x01) ? YES : NO;
				liquid_level_transmitter[1].liquid_level_transmitter_state = (ifr_custom_protocol_ttl_prame.Data[0] & 0x02) ? YES : NO;
				liquid_level_transmitter[2].liquid_level_transmitter_state = (ifr_custom_protocol_ttl_prame.Data[0] & 0x04) ? YES : NO;
				liquid_level_transmitter[3].liquid_level_transmitter_state = (ifr_custom_protocol_ttl_prame.Data[0] & 0x08) ? YES : NO;
				liquid_flow_collection.liquid_level_transmitter_state = (ifr_custom_protocol_ttl_prame.Data[0] & 0x10) ? YES : NO;
				
				// 电机转速命令解析
				for (int i = 0; i < 4; i++) 
				{
					uint16_t speed = (ifr_custom_protocol_ttl_prame.Data[1 + i * 2] << 8) | ifr_custom_protocol_ttl_prame.Data[2 + i * 2];
					if (speed > 6000) 
							speed = 6000;  // 限制最大值
					
					switch (i) 
					{
						case 0: rx_ttl_message.motor_control_data.motor_tar[0] = speed; break;
						case 1: rx_ttl_message.motor_control_data.motor_tar[1] = speed; break;
						case 2: rx_ttl_message.motor_control_data.motor_tar[2] = speed; break;
						case 3: rx_ttl_message.motor_control_data.motor_tar[3] = speed; break;
					}
				}
			}
		}
		else // 若数据接收出错,有可能是进入配置模式指令，10 06 00 00 00 00 8A 8B
		{
			if (len == 8 && pData[0] == 0x10 && pData[1] == 0x06 && pData[2] == 0x00 && pData[3] == 0x00 && pData[4] == 0x00 && pData[5] == 0x00 && pData[6] == 0x8A && pData[7] == 0x8B)
			{
				rx_ttl_message.rx_message_state = CONFIGURATION_MSG;	// 进入配置模式
				while (manager_usart_transmit(&usart_ttl, pData, len) != HAL_OK)
					HAL_Delay(1);	// 等待进入配置模式指令的反馈指令发送成功
			}
			else 
				rx_ttl_message.state = RX_ERROR;
			
			return;
		}
		
	}
	else if (rx_ttl_message.rx_message_state == CONFIGURATION_MSG) // 若已经进入配置模式，处于标准Modbus RTU协议中
	{
		if (pData[0] == 0x0F && pData[1] == 0x06 && (uint16_t)((uint16_t)(pData[2] << 8) | pData[3]) == 0x0001)	// 通过判断ID、功能码、寄存器地址来判断是否为电源配置命令
		{
			if ((uint16_t)((pData[6] << 8) | pData[7]) == CalcCRC16_Modbus(pData, 6))	// CRC16校验通过
			{
				rx_ttl_message.state = UPDATA;	// 正常接收则标识数据已更新
				rx_ttl_message.configuration_data.rx_modbus_message = DEVICE_POWER_MSG;	// 设置为电源控制指令
				liquid_level_transmitter[0].liquid_level_transmitter_power = (pData[5] & 0x01) ? YES : NO;
				liquid_level_transmitter[1].liquid_level_transmitter_power = (pData[5] & 0x02) ? YES : NO;
				liquid_level_transmitter[2].liquid_level_transmitter_power = (pData[5] & 0x04) ? YES : NO;
				liquid_level_transmitter[3].liquid_level_transmitter_power = (pData[5] & 0x08) ? YES : NO;
				for (int i = 0; i < 4; i++)	// 做对应的电源控制处理
				{
					GPIO_TypeDef* port = NULL;
					uint16_t pin = 0;
					// 匹配对应引脚
					switch(i) {
							case 0: port = TRAN1_EN_GPIO_Port; pin = TRAN1_EN_Pin; break;
							case 1: port = TRAN2_EN_GPIO_Port; pin = TRAN2_EN_Pin; break;
							case 2: port = TRAN3_EN_GPIO_Port; pin = TRAN3_EN_Pin; break;
							case 3: port = TRAN4_EN_GPIO_Port; pin = TRAN4_EN_Pin; break;
					}
					if (liquid_level_transmitter[i].liquid_level_transmitter_power == NO)
							HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
					else 
							HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
				}
				manager_usart_transmit(&usart_ttl, pData, len);	// 若将这个帧反馈回TTL上位机
			}
			else 
				rx_ttl_message.state = RX_ERROR;	// 校验错误接收则标识数据错误
		}
		else 
		{
			rx_ttl_message.state = UPDATA;	// 正常接收则标识数据已更新
			rx_ttl_message.configuration_data.rx_modbus_message = TRANSMITTER_CONFIGURATION_MSG;
			HAL_GPIO_WritePin(COM_CTS_GPIO_Port, COM_CTS_Pin, GPIO_PIN_SET);	// 拉高CTS引脚启动485通信
			manager_usart_transmit(&usart_485_modbus.ManagerUsart, pData, len);	// 若不为电源配置命令则直接转发给485
		}
	}
	
}
/*
// 往上位机发的ttl数据发送正常状态函数
 正常数据
	 帧头 			2字节 0x5A 0xA5 
	 帧类型 		1字节 0x02
	 数据段长度 1字节 24 
	 数据段 		24字节 
	 CRC16校验 	2字节 
*/
HAL_StatusTypeDef communicat_ttl_transmit_normal(void)
{
	static uint8_t normal_data_buffer[24] = {0};
	for (int i = 0; i < 4; i++)
	{
		memcpy(&normal_data_buffer[i * 4], liquid_level_transmitter[i].receive_data, 4);	// 复制液位变送器数据
	}
	memcpy(&normal_data_buffer[16], liquid_flow_collection.receive_data, 8);	// 复制流量采集卡数据

	return ifr_custom_protocol_transmit(&usart_ttl, 0x02, 24, normal_data_buffer);	// 发送正常状态的反馈指令
}
/*
// 往上位机发的ttl数据发送错误状态函数
错误帧
	 帧头 			2字节 0x5A 0xA5 
	 帧类型 		1字节 0xF0
	 数据段长度 1字节 0
	 CRC16校验 	2字节 
*/
HAL_StatusTypeDef communicat_ttl_transmit_error(void)
{
	return ifr_custom_protocol_transmit(&usart_ttl, 0xF0, 0, NULL);	// 发送错误状态的反馈指令
}


static uint8_t current_slave_idx = 0;
uint8_t slave_addr_list[5] = {0};
static uint8_t slave_count = 0;

/**
 * @brief  切换485从机索引（通用逻辑封装）
 * @param  current_idx: 当前从机索引
 * @param  slave_count: 从机总数
 * @retval 新的从机索引
 */
static uint8_t slave_switch_idx(uint8_t current_idx, uint8_t slave_count)
{
    if (slave_count == 0) return 0;
    // 最后一个从机则重置为0，否则+1
    return (current_idx == slave_count - 1) ? 0 : (current_idx + 1) % slave_count;
}

/**
 * @brief  重置设备数据缓存和状态（通用逻辑封装）
 * @param  slave_addr: 从机地址 （0x01 - 0x05）
 * @param  state: 要设置的状态（HAL_OK/HAL_ERROR/HAL_BUSY）
 */
static void reset_device_state(uint8_t slave_addr, HAL_StatusTypeDef state)
{
	if (slave_addr <= 0x04)
	{
		// 液位变送器
		liquid_level_transmitter[slave_addr - 1].state_normal = state;
		if (state == HAL_ERROR)
		{
				memset(liquid_level_transmitter[slave_addr - 1].receive_data, 0xFF, 4);
		}
	}
	else
	{
		// 流量传感器
		liquid_flow_collection.state_normal = state;
		if (state == HAL_ERROR)
		{
				memset(liquid_flow_collection.receive_data, 0xFF, 8);
		}
	}
}

/**
 * @brief  检查从机索引合法性（防止数组越界）
 * @retval true: 合法，false: 非法
 */
static bool check_slave_idx_valid(void)
{
    if (slave_count == 0 || current_slave_idx >= slave_count)
    {
        current_slave_idx = 0; // 重置索引
        return false;
    }
    return true;
}
// 检测需要轮询的485从机，并初始化相关参数
void communicat_485_transmit_reset(void)
{
	slave_count = 0;
	memset(slave_addr_list, 0, 5);
	current_slave_idx = 0;
	// 检测是否需要读取
	for (int i = 0; i < 4; i++)
	{
		if (liquid_level_transmitter[i].liquid_level_transmitter_state == YES)
		{
			slave_addr_list[slave_count] = liquid_level_transmitter[i].ID;
			slave_count++;
		}
		memset(liquid_level_transmitter[i].receive_data, 0, 4);	// 将数据清零
	}
	if (liquid_flow_collection.liquid_level_transmitter_state == YES)
	{
		slave_addr_list[slave_count] = liquid_flow_collection.ID;
		slave_count++;
	} 
	memset(liquid_flow_collection.receive_data, 0, 8);	// 将数据清零
}

/**
 * @brief  485轮询发送/接收函数（通用逻辑封装）
 * @retval HAL_OK: 所有从机轮询完成；HAL_BUSY: 正在轮询中
 */
HAL_StatusTypeDef communicat_485_transmit_all(void)
{
    // 1. 无从机直接返回OK
    if (slave_count == 0)
    {
        return HAL_OK;
    }

    // 2. 校验从机索引合法性
    if (!check_slave_idx_valid())
    {
        return HAL_OK;
    }

    // 3. 更新modbus主站状态（处理超时/错误）
    modbus_error_e modbus_err = modbus_master_process(&usart_485_modbus);
    uint8_t curr_slave_addr = slave_addr_list[current_slave_idx];
    HAL_StatusTypeDef ret = HAL_BUSY;

    // 4. 超时处理
    if (modbus_err == MODBUS_ERROR_TIMEOUT)
    {
			reset_device_state(curr_slave_addr, HAL_ERROR); // 重置设备状态+数据
			modbus_master_reset(&usart_485_modbus);         // 重置modbus状态机
			current_slave_idx = slave_switch_idx(current_slave_idx, slave_count); // 切换从机
			ret = (current_slave_idx == 0) ? HAL_OK : HAL_BUSY; // 轮询完所有从机则返回OK
    }
    // 5. 响应完成
    else if (usart_485_modbus.State == MODBUS_STATE_COMPLETE)
    {
			reset_device_state(curr_slave_addr, HAL_OK);    // 标记设备状态正常
			modbus_master_reset(&usart_485_modbus);         // 重置modbus状态机
			current_slave_idx = slave_switch_idx(current_slave_idx, slave_count); // 切换从机
			ret = (current_slave_idx == 0) ? HAL_OK : HAL_BUSY; // 轮询完所有从机则返回OK
    }
    // 6. 主站空闲，发送请求
    else if (!modbus_master_is_busy(&usart_485_modbus))
    {
			reset_device_state(curr_slave_addr, HAL_BUSY); // 标记设备忙
			modbus_error_e send_err = MODBUS_ERROR_INVALID_PARAM;
			HAL_GPIO_WritePin(COM_CTS_GPIO_Port, COM_CTS_Pin, GPIO_PIN_SET);
			// 发送液位变送器请求
			if (curr_slave_addr <= 0x04)
			{
					send_err = modbus_send_read_input_regs(&usart_485_modbus, curr_slave_addr, 0x04, 0, 2);
			}
			// 发送流量传感器请求
			else
			{
					send_err = modbus_send_read_input_regs(&usart_485_modbus, curr_slave_addr, 0x04, 0, 4);
			}

			// 发送失败处理（统一逻辑）
			if (send_err != MODBUS_SUCCESS)
			{
					HAL_GPIO_WritePin(COM_CTS_GPIO_Port, COM_CTS_Pin, GPIO_PIN_RESET);
					reset_device_state(curr_slave_addr, HAL_ERROR); // 标记设备错误
					modbus_master_reset(&usart_485_modbus);         // 重置modbus状态机
					current_slave_idx = slave_switch_idx(current_slave_idx, slave_count); // 切换从机
					ret = (current_slave_idx == 0) ? HAL_OK : HAL_BUSY; // 轮询完所有从机则返回OK
			}
    }

    return ret;
}

// 485的接收数据解析函数
void communicat_485_analysisFunc(modbus_frame_t *frame, uint8_t *data, uint8_t len)
{
	if (rx_ttl_message.rx_message_state == MOTOR_CONTROL_MSG)
	{
		if (frame->SlaveAddr >= 0x06)
			return ;
		for (int i = 0; i < 4; i++)	// 检查是否是液位变送器的指令
		{
			if (frame->SlaveAddr == liquid_level_transmitter[i].ID && frame->FunctionId == 0x04 && frame->Length == 0x05)
			{
				liquid_level_transmitter[i].receive_tick = HAL_GetTick();	// 记录接收的时间
				memcpy(liquid_level_transmitter[i].receive_data, &frame->Data[1], 4);	// 将4字节数据复制到结构体中
			}
		}
		if (frame->SlaveAddr == liquid_flow_collection.ID && frame->FunctionId == 0x04 && frame->Length == 0x09)
		{
				liquid_flow_collection.receive_tick = HAL_GetTick();	// 记录接收的时间
				memcpy(liquid_flow_collection.receive_data, &frame->Data[1], 8);	// 将8字节数据复制到结构体中
		}
	}
	else 
	{
		manager_usart_transmit(&usart_ttl, data, len);	// 若在配置模式下则直接转发给TTL
	}

}
void communicat_485_tx_analysisFunc(UART_HandleTypeDef *huart)
{
	HAL_GPIO_WritePin(COM_CTS_GPIO_Port, COM_CTS_Pin, GPIO_PIN_RESET);
}
rx_ttl_message_typedef communicat_get_ttl_rx_message()
{
	return rx_ttl_message;
}
void reset_communica_ttl_rx_message_state()
{
	rx_ttl_message.state = WAIT;
}
liquid_level_transmitter_typedef communicat_get_liquid_level_transmitter(int index)
{
	return liquid_level_transmitter[index];
}
liquid_flow_collection_typedef communicat_get_liquid_flow_collection()
{
	return liquid_flow_collection;
}
