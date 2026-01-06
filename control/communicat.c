#include "communicat.h"

/**
  * @brief   TTL接收消息结构体（存储上位机指令及解析状态）
  */
static rx_ttl_message_typedef rx_ttl_message = {0};

/**
  * @brief   液位变送器数据结构体数组（4路变送器）
  */
liquid_level_transmitter_typedef liquid_level_transmitter[4] = {0};

/**
  * @brief   流量采集卡数据结构体
  */
liquid_flow_collection_typedef liquid_flow_collection = {0};

/**
  * @brief   485 Modbus RTU通信句柄（管理485串口通信参数/状态）
  */
modbus_rtu_t usart_485_modbus;

/**
  * @brief   TTL串口管理句柄（管理上位机TTL串口通信）
  */
manager_usart_typedef usart_ttl;

/**
  * @brief   TTL串口数据解析函数（上位机指令解析）
  * @param   pData: 接收数据缓冲区指针
  * @param   len: 接收数据长度
  * @retval  void
  */
void communicat_ttl_analysisFunc(uint8_t *pData, uint8_t len);

/**
  * @brief   485 Modbus数据解析函数（从机响应解析）
  * @param   frame: Modbus帧结构体指针
  * @param   data: 响应数据缓冲区指针
  * @param   len: 响应数据长度
  * @retval  void
  */
void communicat_485_analysisFunc(modbus_frame_t *frame, uint8_t *data, uint8_t len);

/**
  * @brief   485串口发送完成回调函数（控制485收发切换）
  * @param   huart: 串口句柄指针
  * @retval  void
  */
void communicat_485_tx_analysisFunc(UART_HandleTypeDef *huart);

/**
  * @brief   检查从机索引合法性（防止数组越界）
  * @retval  bool: true-合法，false-非法
  */
static bool check_slave_idx_valid(void);

/**
  * @brief   重置设备状态和数据缓存（通用封装）
  * @param   slave_addr: 从机地址（0x01-0x05）
  * @param   state: 设备状态（HAL_OK/HAL_ERROR/HAL_BUSY）
  * @retval  void
  */
static void reset_device_state(uint8_t slave_addr, HAL_StatusTypeDef state);

/**
  * @brief   切换485从机轮询索引（循环轮询逻辑）
  * @param   current_idx: 当前从机索引
  * @param   slave_count: 需轮询的从机总数
  * @retval  uint8_t: 新的从机索引
  */
static uint8_t slave_switch_idx(uint8_t current_idx, uint8_t slave_count);

/**
  * @brief   通信模块初始化（485/TTL串口+设备ID初始化）
  * @param   void
  * @retval  void
  */
void communicat_init(void)
{
	// 初始化485串口1（Modbus RTU），注册数据解析回调
	modbus_rtu_init(&usart_485_modbus, &huart1, communicat_485_analysisFunc);	
	// 注册485发送完成回调（控制485收发切换）
	modbus_rtu_register_tx_callback(&usart_485_modbus, communicat_485_tx_analysisFunc);
	// 初始化TTL串口4（上位机通信），注册数据解析回调
	manager_usart_init(&usart_ttl, &huart4, communicat_ttl_analysisFunc);		
	// 初始化4路液位变送器ID（0x01-0x04）
	for (int i = 1; i < 5; i++)	
	{
		liquid_level_transmitter[i - 1].ID = i;
	}
	// 初始化流量采集卡ID（0x05）
	liquid_flow_collection.ID = 0x05;
}

/**
  * @brief   通信模块启动（开启485/TTL串口接收）
  * @param   void
  * @retval  void
  */
void communicat_start(void)
{
	modbus_rtu_start(&usart_485_modbus);	// 启动485串口Modbus RTU接收
	manager_usart_start(&usart_ttl);			// 启动TTL串口接收
}

/**
  * @brief   计时临时变量（TTL解析耗时统计）
  */
uint32_t tt1, tt2, tt3 = 0;

/**
  * @brief   上位机TTL数据解析函数（自定义协议+Modbus配置协议）
  * @note    TTL帧格式：帧头(0x5A 0xA5)+帧类型+数据段长度+数据段+CRC16
  * @param   pData: 接收数据缓冲区指针
  * @param   len: 接收数据长度
  * @retval  void
  */
void communicat_ttl_analysisFunc(uint8_t *pData, uint8_t len)
{
	tt2 = HAL_GetTick() - tt1;
	tt1 = HAL_GetTick();
	HAL_GPIO_TogglePin(STATE_GPIO_Port, STATE_Pin);
	
	// 电机控制模式（自定义串口协议）
	if (rx_ttl_message.rx_message_state == MOTOR_CONTROL_MSG)	
	{
		ifr_custom_protocol_typedef ifr_custom_protocol_ttl_prame;
		ifr_custom_protocol_parse_status_enum ifr_custom_protocol_parse_status;
		// 解析自定义协议
		ifr_custom_protocol_parse_status = ifr_custom_protocol_analysis(&ifr_custom_protocol_ttl_prame, pData, len);
		
		// 协议解析成功
		if (ifr_custom_protocol_parse_status == PARSE_SUCCESS)	
		{
			rx_ttl_message.state = UPDATA;	// 标记数据已更新
			if (ifr_custom_protocol_ttl_prame.DataType == 0x01)
			{
				rx_ttl_message.rx_message_state = MOTOR_CONTROL_MSG;
					
				// 解析变送器/流量卡采集使能状态
				liquid_level_transmitter[0].liquid_level_transmitter_state = (ifr_custom_protocol_ttl_prame.Data[0] & 0x01) ? YES : NO;
				liquid_level_transmitter[1].liquid_level_transmitter_state = (ifr_custom_protocol_ttl_prame.Data[0] & 0x02) ? YES : NO;
				liquid_level_transmitter[2].liquid_level_transmitter_state = (ifr_custom_protocol_ttl_prame.Data[0] & 0x04) ? YES : NO;
				liquid_level_transmitter[3].liquid_level_transmitter_state = (ifr_custom_protocol_ttl_prame.Data[0] & 0x08) ? YES : NO;
				liquid_flow_collection.liquid_level_transmitter_state = (ifr_custom_protocol_ttl_prame.Data[0] & 0x10) ? YES : NO;
				
				// 解析4路电机目标转速（0.1rpm单位，限制最大值6000）
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
		// 协议解析失败（检测配置模式指令）
		else 
		{
			// 配置模式指令：0x10 06 00 00 00 00 8A 8B
			if (len == 8 && pData[0] == 0x10 && pData[1] == 0x06 && pData[2] == 0x00 && pData[3] == 0x00 && pData[4] == 0x00 && pData[5] == 0x00 && pData[6] == 0x8A && pData[7] == 0x8B)
			{
				rx_ttl_message.rx_message_state = CONFIGURATION_MSG;	// 切换为配置模式
				// 发送配置模式指令反馈
				while (manager_usart_transmit(&usart_ttl, pData, len) != HAL_OK)
					HAL_Delay(1);	
			}
			else 
				rx_ttl_message.state = RX_ERROR;	// 标记接收错误
			
			return;
		}
		
	}
	// 配置模式（标准Modbus RTU协议）
	else if (rx_ttl_message.rx_message_state == CONFIGURATION_MSG) 
	{
		// 电源配置命令：ID=0x0F + 功能码0x06 + 寄存器0x0001
		if (pData[0] == 0x0F && pData[1] == 0x06 && (uint16_t)((uint16_t)(pData[2] << 8) | pData[3]) == 0x0001)	
		{
			// CRC16校验通过
			if ((uint16_t)((pData[6] << 8) | pData[7]) == CalcCRC16_Modbus(pData, 6))	
			{
				rx_ttl_message.state = UPDATA;	// 标记数据已更新
				rx_ttl_message.configuration_data.rx_modbus_message = DEVICE_POWER_MSG;	// 标记为电源控制指令
				// 解析4路变送器电源状态
				liquid_level_transmitter[0].liquid_level_transmitter_power = (pData[5] & 0x01) ? YES : NO;
				liquid_level_transmitter[1].liquid_level_transmitter_power = (pData[5] & 0x02) ? YES : NO;
				liquid_level_transmitter[2].liquid_level_transmitter_power = (pData[5] & 0x04) ? YES : NO;
				liquid_level_transmitter[3].liquid_level_transmitter_power = (pData[5] & 0x08) ? YES : NO;
				
				// 执行变送器电源控制
				for (int i = 0; i < 4; i++)	
				{
					GPIO_TypeDef* port = NULL;
					uint16_t pin = 0;
					// 匹配对应电源引脚
					switch(i) {
							case 0: port = TRAN1_EN_GPIO_Port; pin = TRAN1_EN_Pin; break;
							case 1: port = TRAN2_EN_GPIO_Port; pin = TRAN2_EN_Pin; break;
							case 2: port = TRAN3_EN_GPIO_Port; pin = TRAN3_EN_Pin; break;
							case 3: port = TRAN4_EN_GPIO_Port; pin = TRAN4_EN_Pin; break;
					}
					// 控制电源引脚电平
					if (liquid_level_transmitter[i].liquid_level_transmitter_power == NO)
							HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
					else 
							HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
				}
				// 反馈电源配置指令至上位机
				manager_usart_transmit(&usart_ttl, pData, len);	
			}
			else 
				rx_ttl_message.state = RX_ERROR;	// CRC校验错误
		}
		else 
		{
			rx_ttl_message.state = UPDATA;	// 标记数据已更新
			rx_ttl_message.configuration_data.rx_modbus_message = TRANSMITTER_CONFIGURATION_MSG;
			HAL_GPIO_WritePin(COM_CTS_GPIO_Port, COM_CTS_Pin, GPIO_PIN_SET);	// 拉高CTS启动485通信
			// 非电源配置指令转发至485总线
			manager_usart_transmit(&usart_485_modbus.ManagerUsart, pData, len);	
		}
	}
	
}

/**
  * @brief   TTL发送正常状态数据（至上位机）
  * @note    帧格式：帧头(0x5A 0xA5)+帧类型0x02+长度24+数据段(24字节)+CRC16
  * @retval  HAL_StatusTypeDef: HAL_OK-发送成功，其他-失败
  */
HAL_StatusTypeDef communicat_ttl_transmit_normal(void)
{
	static uint8_t normal_data_buffer[24] = {0};
	// 复制4路液位变送器数据（4*4=16字节）
	for (int i = 0; i < 4; i++)
	{
		memcpy(&normal_data_buffer[i * 4], liquid_level_transmitter[i].receive_data, 4);	
	}
	// 复制流量采集卡数据（8字节）
	memcpy(&normal_data_buffer[16], liquid_flow_collection.receive_data, 8);	

	// 发送自定义协议帧
	return ifr_custom_protocol_transmit(&usart_ttl, 0x02, 24, normal_data_buffer);	
}

/**
  * @brief   TTL发送错误状态数据（至上位机）
  * @note    帧格式：帧头(0x5A 0xA5)+帧类型0xF0+长度0+CRC16
  * @retval  HAL_StatusTypeDef: HAL_OK-发送成功，其他-失败
  */
HAL_StatusTypeDef communicat_ttl_transmit_error(void)
{
	// 发送错误帧（无数据段）
	return ifr_custom_protocol_transmit(&usart_ttl, 0xF0, 0, NULL);	
}

/**
  * @brief   当前485从机轮询索引
  */
static uint8_t current_slave_idx = 0;

/**
  * @brief   485从机地址列表（存储需轮询的从机地址）
  */
uint8_t slave_addr_list[5] = {0};

/**
  * @brief   需轮询的从机总数
  */
static uint8_t slave_count = 0;

/**
  * @brief   切换485从机轮询索引（通用逻辑封装）
  * @param   current_idx: 当前从机索引
  * @param   slave_count: 从机总数
  * @retval  uint8_t: 新的从机索引
  */
static uint8_t slave_switch_idx(uint8_t current_idx, uint8_t slave_count)
{
    if (slave_count == 0) return 0;
    // 最后一个从机则重置为0，否则索引+1
    return (current_idx == slave_count - 1) ? 0 : (current_idx + 1) % slave_count;
}

/**
  * @brief   重置设备数据缓存和状态（通用逻辑封装）
  * @param   slave_addr: 从机地址 （0x01 - 0x05）
  * @param   state: 要设置的状态（HAL_OK/HAL_ERROR/HAL_BUSY）
  * @retval  void
  */
static void reset_device_state(uint8_t slave_addr, HAL_StatusTypeDef state)
{
	if (slave_addr <= 0x04)
	{
		// 液位变送器状态重置
		liquid_level_transmitter[slave_addr - 1].state_normal = state;
		if (state == HAL_ERROR)
		{
				// 错误状态填充0xFF
				memset(liquid_level_transmitter[slave_addr - 1].receive_data, 0xFF, 4);
		}
	}
	else
	{
		// 流量采集卡状态重置
		liquid_flow_collection.state_normal = state;
		if (state == HAL_ERROR)
		{
				// 错误状态填充0xFF
				memset(liquid_flow_collection.receive_data, 0xFF, 8);
		}
	}
}

/**
  * @brief   检查从机索引合法性（防止数组越界）
  * @retval  bool: true-合法，false-非法
  */
static bool check_slave_idx_valid(void)
{
    if (slave_count == 0 || current_slave_idx >= slave_count)
    {
        current_slave_idx = 0; // 索引非法则重置为0
        return false;
    }
    return true;
}

/**
  * @brief   初始化485从机轮询参数（检测需轮询的设备）
  * @param   void
  * @retval  void
  */
void communicat_485_transmit_reset(void)
{
	slave_count = 0;
	memset(slave_addr_list, 0, 5);	// 清空从机地址列表
	current_slave_idx = 0;			// 重置轮询索引
	
	// 检测液位变送器是否需要轮询
	for (int i = 0; i < 4; i++)
	{
		if (liquid_level_transmitter[i].liquid_level_transmitter_state == YES)
		{
			slave_addr_list[slave_count] = liquid_level_transmitter[i].ID;
			slave_count++;
		}
		memset(liquid_level_transmitter[i].receive_data, 0, 4);	// 清空变送器数据缓存
	}
	
	// 检测流量采集卡是否需要轮询
	if (liquid_flow_collection.liquid_level_transmitter_state == YES)
	{
		slave_addr_list[slave_count] = liquid_flow_collection.ID;
		slave_count++;
	} 
	memset(liquid_flow_collection.receive_data, 0, 8);	// 清空流量卡数据缓存
}

/**
  * @brief   485从机轮询函数（发送请求+处理响应/超时）
  * @retval  HAL_StatusTypeDef: HAL_OK-轮询完成，HAL_BUSY-轮询中
  */
HAL_StatusTypeDef communicat_485_transmit_all(void)
{
    // 1. 无从机需要轮询，直接返回完成
    if (slave_count == 0)
    {
        return HAL_OK;
    }

    // 2. 校验从机索引合法性
    if (!check_slave_idx_valid())
    {
        return HAL_OK;
    }

    // 3. 更新Modbus主站状态（处理超时/错误）
    modbus_error_e modbus_err = modbus_master_process(&usart_485_modbus);
    uint8_t curr_slave_addr = slave_addr_list[current_slave_idx];
    HAL_StatusTypeDef ret = HAL_BUSY;

    // 4. 超时处理（从机无响应）
    if (modbus_err == MODBUS_ERROR_TIMEOUT)
    {
			reset_device_state(curr_slave_addr, HAL_ERROR); // 标记设备错误
			modbus_master_reset(&usart_485_modbus);         // 重置Modbus状态机
			current_slave_idx = slave_switch_idx(current_slave_idx, slave_count); // 切换从机
			// 轮询完所有从机则返回OK，否则返回BUSY
			ret = (current_slave_idx == 0) ? HAL_OK : HAL_BUSY; 
    }
    // 5. 从机响应完成
    else if (usart_485_modbus.State == MODBUS_STATE_COMPLETE)
    {
			reset_device_state(curr_slave_addr, HAL_OK);    // 标记设备状态正常
			modbus_master_reset(&usart_485_modbus);         // 重置Modbus状态机
			current_slave_idx = slave_switch_idx(current_slave_idx, slave_count); // 切换从机
			ret = (current_slave_idx == 0) ? HAL_OK : HAL_BUSY; 
    }
    // 6. Modbus主站空闲，发送轮询请求
    else if (!modbus_master_is_busy(&usart_485_modbus))
    {
			reset_device_state(curr_slave_addr, HAL_BUSY); // 标记设备忙
			modbus_error_e send_err = MODBUS_ERROR_INVALID_PARAM;
			HAL_GPIO_WritePin(COM_CTS_GPIO_Port, COM_CTS_Pin, GPIO_PIN_SET); // 开启485发送
			
			// 发送液位变送器轮询请求（读取2个输入寄存器）
			if (curr_slave_addr <= 0x04)
			{
					send_err = modbus_send_read_input_regs(&usart_485_modbus, curr_slave_addr, 0x04, 0, 2);
			}
			// 发送流量采集卡轮询请求（读取4个输入寄存器）
			else
			{
					send_err = modbus_send_read_input_regs(&usart_485_modbus, curr_slave_addr, 0x04, 0, 4);
			}

			// 发送失败处理
			if (send_err != MODBUS_SUCCESS)
			{
					HAL_GPIO_WritePin(COM_CTS_GPIO_Port, COM_CTS_Pin, GPIO_PIN_RESET); // 关闭485发送
					reset_device_state(curr_slave_addr, HAL_ERROR); // 标记设备错误
					modbus_master_reset(&usart_485_modbus);         // 重置Modbus状态机
					current_slave_idx = slave_switch_idx(current_slave_idx, slave_count); // 切换从机
					ret = (current_slave_idx == 0) ? HAL_OK : HAL_BUSY; 
			}
    }

    return ret;
}

/**
  * @brief   485 Modbus响应数据解析函数
  * @param   frame: Modbus帧结构体指针
  * @param   data: 响应数据缓冲区指针
  * @param   len: 响应数据长度
  * @retval  void
  */
void communicat_485_analysisFunc(modbus_frame_t *frame, uint8_t *data, uint8_t len)
{
	// 电机控制模式下解析数据
	if (rx_ttl_message.rx_message_state == MOTOR_CONTROL_MSG)
	{
		// 过滤非法从机地址（仅处理0x01-0x05）
		if (frame->SlaveAddr >= 0x06)
			return ;
		
		// 解析液位变送器响应数据
		for (int i = 0; i < 4; i++)	
		{
			if (frame->SlaveAddr == liquid_level_transmitter[i].ID && frame->FunctionId == 0x04 && frame->Length == 0x05)
			{
				liquid_level_transmitter[i].receive_tick = HAL_GetTick();	// 记录接收时间
				memcpy(liquid_level_transmitter[i].receive_data, &frame->Data[1], 4);	// 复制4字节数据
			}
		}
		
		// 解析流量采集卡响应数据
		if (frame->SlaveAddr == liquid_flow_collection.ID && frame->FunctionId == 0x04 && frame->Length == 0x09)
		{
				liquid_flow_collection.receive_tick = HAL_GetTick();	// 记录接收时间
				memcpy(liquid_flow_collection.receive_data, &frame->Data[1], 8);	// 复制8字节数据
		}
	}
	// 配置模式下转发数据至TTL上位机
	else 
	{
		manager_usart_transmit(&usart_ttl, data, len);	
	}

}

/**
  * @brief   485发送完成回调（关闭485发送，切换为接收）
  * @param   huart: 串口句柄指针
  * @retval  void
  */
void communicat_485_tx_analysisFunc(UART_HandleTypeDef *huart)
{
	// 拉低CTS引脚，关闭485发送，开启接收
	HAL_GPIO_WritePin(COM_CTS_GPIO_Port, COM_CTS_Pin, GPIO_PIN_RESET);
}

/**
  * @brief   获取TTL接收消息结构体（上位机指令/状态）
  * @param   void
  * @retval  rx_ttl_message_typedef: TTL接收消息结构体
  */
rx_ttl_message_typedef communicat_get_ttl_rx_message()
{
	return rx_ttl_message;
}

/**
  * @brief   重置TTL接收消息状态（标记为等待状态）
  * @param   void
  * @retval  void
  */
void reset_communica_ttl_rx_message_state()
{
	rx_ttl_message.state = WAIT;
}

/**
  * @brief   获取指定索引的液位变送器数据
  * @param   index: 变送器索引（0-3）
  * @retval  liquid_level_transmitter_typedef: 液位变送器数据结构体
  */
liquid_level_transmitter_typedef communicat_get_liquid_level_transmitter(int index)
{
	return liquid_level_transmitter[index];
}

/**
  * @brief   获取流量采集卡数据
  * @param   void
  * @retval  liquid_flow_collection_typedef: 流量采集卡数据结构体
  */
liquid_flow_collection_typedef communicat_get_liquid_flow_collection()
{
	return liquid_flow_collection;
}
