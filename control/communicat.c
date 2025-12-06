#include "communicat.h"
static rx_ttl_message_typedef rx_ttl_message = {0};
static liquid_level_transmitter_typedef liquid_level_transmitter[4] = {0};
static liquid_flow_collection_typedef liquid_flow_collection = {0};
static tx_ttl_message_typedef tx_ttl_message = {0};
modbus_rtu_t usart_485_modbus;
ifr_usart_typedef usart_ttl;

void usart_ttl_analysisFunc(uint8_t *pData, uint8_t len);
void usart_485_analysisFunc(modbus_frame_t *frame, uint16_t *reg_values, uint16_t reg_count);
// 通信初始化
void communicat_init(void)
{
	modbus_rtu_init(&usart_485_modbus, &huart1, usart_485_analysisFunc);	// 下位机485串口1初始化
	IFR_USART_Init(&usart_ttl, &huart4, usart_ttl_analysisFunc);					// 上位机TTL串口4初始化
	for (int i = 0; i < 4; i++)
	{
		liquid_level_transmitter[i].ID = i;
	}
	liquid_flow_collection.ID = 0x05;
}

// 上位机下发的ttl数据解析函数
void usart_ttl_analysisFunc(uint8_t *pData, uint8_t len)
{
	if (rx_ttl_message.rx_message_state == MOTOR_CONTROL_MSG)	// 若还未进入配置模式，还处于自定义串口协议中
	{
		uint8_t index = 0;
		while (index < len)	// 循环查找帧头
		{
			if (pData[index] == 0x5A && pData[index + 1] == 0xA5)	// 判断帧头是否正确，若不正确则继续寻找
			{
				if (index + pData[index + 3] > len)	// 判断剩余的数据长度是否不足，若不足则退出
					return; 
				if (pData[index + 2] == 0x01)	// 若为电机控制指令
				{
					rx_ttl_message.rx_message_state = MOTOR_CONTROL_MSG;
					
					// 变送器采集配置解析
					liquid_level_transmitter[0].liquid_level_transmitter_state = (pData[index + 4] & 0x01) ? YES : NO;
					liquid_level_transmitter[1].liquid_level_transmitter_state = (pData[index + 4] & 0x02) ? YES : NO;
					liquid_level_transmitter[2].liquid_level_transmitter_state = (pData[index + 4] & 0x04) ? YES : NO;
					liquid_level_transmitter[3].liquid_level_transmitter_state = (pData[index + 4] & 0x08) ? YES : NO;
					liquid_flow_collection.liquid_level_transmitter_state = (pData[index + 4] & 0x10) ? YES : NO;
					
					// 电机转速命令解析
					for (int i = 0; i < 4; i++) 
					{
						uint16_t speed = (pData[index + 5 + i * 2] << 8) | pData[index + 6 + i * 2];
						if (speed > 6000) 
								speed = 6000;  // 限制最大值
						
						switch (i) 
						{
							case 0: rx_ttl_message.motor_control_data.motor_tar1 = speed; break;
							case 1: rx_ttl_message.motor_control_data.motor_tar2 = speed; break;
							case 2: rx_ttl_message.motor_control_data.motor_tar3 = speed; break;
							case 3: rx_ttl_message.motor_control_data.motor_tar4 = speed; break;
						}
					}
					rx_ttl_message.motor_control_data.answer = NEED_ANSWER;
				}
				else if (pData[index + 2] == 0x0F)	// 若为配置模式指令
				{
					rx_ttl_message.rx_message_state = CONFIGURATION_MSG;
				}
				return;
			}
			index ++;
		}
	}
	else // 若进入配置模式，处于标准Modbus RTU协议中
	{
		if (pData[0] == 0x0F && pData[1] == 0x06 && (uint16_t)((uint16_t)(pData[2] << 8) | pData[3]) == 0x0001)	// 通过判断ID、地址码、寄存器地址来判断是否为电源配置命令
		{
			if ((uint16_t)((pData[6] << 8) | pData[7]) == CalcCRC16_Modbus(pData, 6))	// CRC16校验通过
			{
				rx_ttl_message.configuration_data.rx_modbus_message = DEVICE_POWER_MSG;
				liquid_level_transmitter[0].liquid_level_transmitter_power = (pData[5] & 0x01) ? YES : NO;
				liquid_level_transmitter[1].liquid_level_transmitter_power = (pData[5] & 0x02) ? YES : NO;
				liquid_level_transmitter[2].liquid_level_transmitter_power = (pData[5] & 0x04) ? YES : NO;
				liquid_level_transmitter[3].liquid_level_transmitter_power = (pData[5] & 0x08) ? YES : NO;
			}
		}
		else 
		{
			rx_ttl_message.configuration_data.rx_modbus_message = TRANSMITTER_CONFIGURATION_MSG;
			HAL_UART_Transmit_DMA(&huart1, pData, len);	// 若不为电源配置命令则直接转发给485
		}
	}
}

// 往上位机发的ttl数据发送函数
void usart_ttl_transmit(void)
{
	tx_ttl_message.frame_header_1 = 0x5A;
	tx_ttl_message.frame_header_2 = 0xA5;
	tx_ttl_message.frame_type = 0x02;
	for (int i = 0; i < 4; i++)
	{
		memcpy(&tx_ttl_message.data[i * 4], liquid_level_transmitter[i].receive_data, 4);
	}
	memcpy(&tx_ttl_message.data[16], liquid_flow_collection.receive_data, 8);
	HAL_UART_Transmit_DMA(&huart4, (uint8_t *)&tx_ttl_message, sizeof(tx_ttl_message));
}
uint8_t current_slave_idx = 1;
uint8_t slave_addr_list[5] = {0};
uint8_t slave_count = 0;
void usart_485_transmit_init(void)
{
	slave_count = 0;
	memset(slave_addr_list, 0, 5);
	current_slave_idx = 1;
	// 检测是否需要读取
	for (int i = 0; i < 4; i++)
	{
		if (liquid_level_transmitter[i].liquid_level_transmitter_state == YES)
		{
			slave_addr_list[slave_count] = liquid_level_transmitter[i].ID;
			slave_count++;
		}
		else 
			memset(liquid_level_transmitter[i].receive_data, 0, 4);
	}
	if (liquid_flow_collection.liquid_level_transmitter_state == YES)
	{
		slave_addr_list[slave_count] = liquid_flow_collection.ID;
		slave_count++;
	}
	else 
		memset(liquid_flow_collection.receive_data, 0, 8);	
}

HAL_StatusTypeDef usart_485_transmit_all(void)
{
 static uint8_t current_slave_idx = 0;
 modbus_error_t err = modbus_master_process(&usart_485_modbus);
	
	// 超时处理
	if (err == MODBUS_ERROR_TIMEOUT) 
	{
		if (slave_addr_list[current_slave_idx] <= 0x04)
		{
			memset(liquid_level_transmitter[slave_addr_list[current_slave_idx]].receive_data, 0xFF, 4);
			liquid_level_transmitter[slave_addr_list[current_slave_idx]].state_normal = HAL_ERROR;
		}
		else 
		{
			memset(liquid_flow_collection.receive_data, 0xFF, 8);
			liquid_flow_collection.state_normal = HAL_ERROR;
		}
		modbus_master_reset(&usart_485_modbus);
		if (current_slave_idx == slave_count)	// 所有需要的轮询已经完成
			return HAL_OK;
		current_slave_idx = (current_slave_idx + 1) % slave_count;
	}
	
	// 响应完成，切换从机
	if (usart_485_modbus.state == MODBUS_MASTER_COMPLETE) 
	{
		if (slave_addr_list[current_slave_idx] <= 0x04)
			liquid_level_transmitter[slave_addr_list[current_slave_idx]].state_normal = HAL_OK;
		else 
			liquid_flow_collection.state_normal = HAL_OK;
		modbus_master_reset(&usart_485_modbus);
		if (current_slave_idx == slave_count)	// 所有需要的轮询已经完成
			return HAL_OK;
		current_slave_idx = (current_slave_idx + 1) % slave_count;
	}
	
	// 主站空闲，发送请求
	if (!modbus_master_is_busy(&usart_485_modbus)) 
	{
			uint8_t slave_addr = slave_addr_list[current_slave_idx];
			if (slave_addr <= 0x04)
			{
				liquid_level_transmitter[slave_addr_list[current_slave_idx]].state_normal = HAL_BUSY;
				modbus_send_read_input_regs(&usart_485_modbus, slave_addr, 0x04, 0, 2);
			}
			else 
			{
				liquid_flow_collection.state_normal = HAL_BUSY;
				modbus_send_read_input_regs(&usart_485_modbus, slave_addr, 0x04, 0, 4);
			}
	}
	return HAL_BUSY;
}

// 485的接收数据解析函数
void usart_485_analysisFunc(modbus_frame_t *frame, uint16_t *reg_values, uint16_t reg_count)
{
	for (int i = 0; i < 4; i++)	// 检查是否是液位变送器的指令
	{
		if (frame->slave_addr == liquid_level_transmitter[i].ID && frame->function == 0x04 && frame->length == 0x04)
		{
			liquid_level_transmitter[i].receive_tick = HAL_GetTick();	// 记录接收的时间
			memcpy(liquid_level_transmitter[i].receive_data, &frame->data[1], 4);	// 将4字节数据复制到结构体中
		}
	}
	if (frame->slave_addr == liquid_flow_collection.ID && frame->function == 0x04 && frame->length == 0x04)
	{
			liquid_flow_collection.receive_tick = HAL_GetTick();	// 记录接收的时间
			memcpy(liquid_flow_collection.receive_data, &frame->data[1], 8);	// 将8字节数据复制到结构体中
	}
	
}

rx_ttl_message_typedef get_ttl_rx_message()
{
	return rx_ttl_message;
}
liquid_level_transmitter_typedef* get_liquid_level_transmitter()
{
	return liquid_level_transmitter;
}
liquid_flow_collection_typedef get_liquid_flow_collection()
{
	return liquid_flow_collection;
}
