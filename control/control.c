#include "control.h"

target_speed_t target_motor_speed = {.target_speed = {0}};  //目标速度

system_state_typedef system_state = {STOP, INIT};

ifr_tim_typedef tim_6;  //定时器6

void control_init(void)
{
	motor_all_init();  //电机初始化
	ifr_tim_start(&tim_6, &htim6, control_runing);  //定时器6初始化
	target_motor_speed.target_speed[0] = 1000;
	target_motor_speed.target_speed[1] = 1000;	
}

void control_input(void)
{
    
}

void control_runing(void)
{
	communicat_runing();
	if (system_state.mode == MOTOR_CONTROL)
	{
		switch (system_state.motor_control_mode)
		{
			case INIT:	// 初始化需要控制的设备
				usart_485_transmit_init();
				system_state.motor_control_mode = GET_DATA;
				break;
			case GET_DATA:	// 获取流量传感器和液位变送器数据
				if (usart_485_transmit_all() == HAL_OK)
					system_state.motor_control_mode = TRANSMIT_DATA;
				break;
			case TRANSMIT_DATA:	// 向上位机发送当前流量传感器和液位变送器数据
				usart_ttl_transmit();
				system_state.motor_control_mode = SET_MOTOR_SPEED;
				break;
			case SET_MOTOR_SPEED:	// 电机控制
				for (int i = 0; i < MOTOR_COUNT; i++)
				{
					motor_control(i, target_motor_speed.target_speed[i]);
				}
				break;
			default:
				break;
		}
	}
	else 
	{
		for (int i = 0; i < MOTOR_COUNT; i++)
		{
			motor_control(i, 0);
		}
	}
}

void communicat_runing()
{
	rx_ttl_message_typedef rx_ttl_message_temp = get_ttl_rx_message();
	
	if (rx_ttl_message_temp.rx_message_state == MOTOR_CONTROL_MSG)
	{
		system_state.mode = MOTOR_CONTROL;
		system_state.motor_control_mode = INIT;
		for (int i = 0; i < 4; i++)
		{
			target_motor_speed.target_speed[i] = rx_ttl_message_temp.motor_control_data.motor_tar[i];	// 更新电机目标速度
		}
	}
	if (rx_ttl_message_temp.rx_message_state == CONFIGURATION_MSG)
	{
		system_state.mode = STOP;
		if (rx_ttl_message_temp.configuration_data.rx_modbus_message == DEVICE_POWER_MSG)	// 如果是电源配置指令
		{
			for (int i = 0; i < 4; i++)
			{
				liquid_level_transmitter_typedef liquid_level_transmitter_temp = get_liquid_level_transmitter(i);
				GPIO_TypeDef* port = NULL;
				uint16_t pin = 0;
				// 匹配对应引脚
				switch(i) {
						case 0: port = TRAN1_EN_GPIO_Port; pin = TRAN1_EN_Pin; break;
						case 1: port = TRAN2_EN_GPIO_Port; pin = TRAN2_EN_Pin; break;
						case 2: port = TRAN3_EN_GPIO_Port; pin = TRAN3_EN_Pin; break;
						case 3: port = TRAN4_EN_GPIO_Port; pin = TRAN4_EN_Pin; break;
				}
				if (liquid_level_transmitter_temp.liquid_level_transmitter_power == NO)
						HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
				else 
						HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
			}
		}
	}
}


