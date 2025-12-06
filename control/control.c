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
	// 电机控制 - 2
//	for (int i = 0; i < MOTOR_COUNT; i++)
//	{
			motor_control(0, target_motor_speed.target_speed[0]);
			motor_control(1, target_motor_speed.target_speed[1]);
//	}
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
			case TRANSMIT_DATA:
				usart_ttl_transmit();
				system_state.motor_control_mode = SET_MOTOR_SPEED;
			case SET_MOTOR_SPEED:	// 电机控制
				//	for (int i = 0; i < MOTOR_COUNT; i++)
				//	{
							motor_control(0, target_motor_speed.target_speed[0]);
							motor_control(1, target_motor_speed.target_speed[1]);
				//	}
				break;
			default:
				break;
		}
		
		
	}
}

void communicat_runing()
{
	rx_ttl_message_typedef rx_ttl_message_temp = get_ttl_rx_message();
	if (rx_ttl_message_temp.rx_message_state == MOTOR_CONTROL_MSG && rx_ttl_message_temp.motor_control_data.answer == NEED_ANSWER)
	{
		system_state.mode = MOTOR_CONTROL;
		system_state.motor_control_mode = INIT;
	}
	if (rx_ttl_message_temp.rx_message_state == CONFIGURATION_MSG)
		system_state.mode = STOP;
}


