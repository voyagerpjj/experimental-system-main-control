#include "communicat.h"
static rx_message_typedef rx_message = {0};

void communicat_init(void)
{
    
}

// 上位机下发的ttl数据解析函数
void usart_ttl_analysisFunc(uint8_t *pData, uint8_t len)
{
	if (rx_message.rx_message_state == MOTOR_CONTROL_MSG)	// 若还未进入配置模式，还处于自定义串口协议中
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
					rx_message.rx_message_state = MOTOR_CONTROL_MSG;
					
					// 变送器采集配置解析
					rx_message.motor_control_data.liquid_level_transmitter_1 = (pData[index + 4] & 0x01) ? YES : NO;
					rx_message.motor_control_data.liquid_level_transmitter_2 = (pData[index + 4] & 0x02) ? YES : NO;
					rx_message.motor_control_data.liquid_level_transmitter_3 = (pData[index + 4] & 0x04) ? YES : NO;
					rx_message.motor_control_data.liquid_level_transmitter_4 = (pData[index + 4] & 0x08) ? YES : NO;
					rx_message.motor_control_data.flow_collector = (pData[index + 4] & 0x10) ? YES : NO;
					
					// 电机转速命令解析
					for (int i = 0; i < 4; i++) 
					{
						uint16_t speed = (pData[index + 5 + i * 2] << 8) | pData[index + 6 + i * 2];
						if (speed > 6000) 
								speed = 6000;  // 限制最大值
						
						switch (i) 
						{
							case 0: rx_message.motor_control_data.motor_tar1 = speed; break;
							case 1: rx_message.motor_control_data.motor_tar2 = speed; break;
							case 2: rx_message.motor_control_data.motor_tar3 = speed; break;
							case 3: rx_message.motor_control_data.motor_tar4 = speed; break;
						}
						
					}
				}
				else if (pData[index + 2] == 0x0F)	// 若为配置模式指令
				{
					rx_message.rx_message_state = CONFIGURATION_MSG;
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
			if ((uint16_t)((pData[6] << 8) | pData[7]) == CalcCRC_Modbus(pData, 6))	// CRC16校验通过
			{
				rx_message.configuration_data.rx_modbus_message = DEVICE_POWER_MSG;
				rx_message.configuration_data.device_power.liquid_level_transmitter_1 = (pData[5] & 0x01) ? YES : NO;
				rx_message.configuration_data.device_power.liquid_level_transmitter_2 = (pData[5] & 0x02) ? YES : NO;
				rx_message.configuration_data.device_power.liquid_level_transmitter_3 = (pData[5] & 0x04) ? YES : NO;
				rx_message.configuration_data.device_power.liquid_level_transmitter_4 = (pData[5] & 0x08) ? YES : NO;
			}
		}
		else 
		{
			rx_message.configuration_data.rx_modbus_message = TRANSMITTER_CONFIGURATION_MSG;
			HAL_UART_Transmit_DMA(&usart_485, pData, len);	// 若不为电源配置命令则直接转发给485
		}
	}
}

void usart_485_analysisFunc(uint8_t *pData, uint8_t len)
{
	
}

rx_message_typedef get_rx_message()
{
	return rx_message;
}
