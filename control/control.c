#include "control.h"

target_speed_t target_motor_speed = {.target_speed = {0}};  //目标速度

system_state_typedef system_state = {STOP, INIT};

manager_tim_typedef tim_6;  //定时器6
static void control_runing(void);  //定时器6回调函数
static void communicat_runing(void);  //通信运行函数
static void communicat_disconnection_protection(void);
void control_init(void)
{
	motor_all_init();  //电机初始化
	communicat_init();  //通信初始化
	manager_tim_init(&tim_6, &htim6, control_runing);  //定时器6初始化
//	target_motor_speed.target_speed[0] = 1000;
//	target_motor_speed.target_speed[1] = 1000;	
}

void control_start(void)
{
	communicat_start();  //通信启动
	manager_tim_start(&tim_6);  //定时器6启动
}
uint32_t t6, t7, t8, t9, t10 = 0;
static void control_runing(void)
{
	communicat_runing();
	if (system_state.mode == MOTOR_CONTROL)
	{
		switch (system_state.motor_control_mode)
		{
			case INIT:	// 初始化需要控制的设备
				communicat_485_transmit_reset();
				system_state.motor_control_mode = GET_DATA;
				break;
			case GET_DATA:	// 获取流量传感器和液位变送器数据
				if (communicat_485_transmit_all() == HAL_OK)
				{
					t7 = HAL_GetTick() - tt1;
					system_state.motor_control_mode = TRANSMIT_DATA;
				}
				break;
			case TRANSMIT_DATA:	// 向上位机发送当前流量传感器和液位变送器数据
				if (communicat_ttl_transmit_normal() == HAL_OK)
				{
					t6 = HAL_GetTick() - tt1;
					system_state.motor_control_mode = SET_MOTOR_SPEED;
				}
				break;
			case SET_MOTOR_SPEED:	// 电机控制
				for (int i = 0; i < MOTOR_COUNT; i++)
				{
					motor_start(i, target_motor_speed.target_speed[i]);
				}
				break;
			default:
				break;
		}
	}
	else 
	{
		for (int i = 0; i < MOTOR_COUNT; i++)
			motor_stop(i);
	}
}

rx_ttl_message_typedef rx_ttl_message_temp;
static void communicat_runing()
{
	rx_ttl_message_temp = communicat_get_ttl_rx_message();
	communicat_disconnection_protection();
	if (rx_ttl_message_temp.state == UPDATA)
	{
		reset_communica_ttl_rx_message_state();	// 重置TTL状态
		if (rx_ttl_message_temp.rx_message_state == MOTOR_CONTROL_MSG)	// 当前处在正常控制模式下
		{
			system_state.mode = MOTOR_CONTROL;
			system_state.motor_control_mode = INIT;
			for (int i = 0; i < 4; i++)
			{
				target_motor_speed.target_speed[i] = rx_ttl_message_temp.motor_control_data.motor_tar[i];	// 更新电机目标速度
			}
		}
		else if (rx_ttl_message_temp.rx_message_state == CONFIGURATION_MSG)	// 当前处在配置模式下
		{
			system_state.mode = STOP;
			system_state.motor_control_mode = INIT;
		}
	}
	else if (rx_ttl_message_temp.state == RX_ERROR)
	{
		if (communicat_ttl_transmit_error() == HAL_OK)
			reset_communica_ttl_rx_message_state();	// 若成功返回错误帧，则重置TTL状态
	}
}
static uint16_t disconnection_protection_time_ms = 0;
static void communicat_disconnection_protection(void)
{
	if (rx_ttl_message_temp.state == WAIT)
		disconnection_protection_time_ms++;
	else 
		disconnection_protection_time_ms = 0;

	if (disconnection_protection_time_ms > 2000)	// 超时2s
	{
		system_state.mode = STOP;
		system_state.motor_control_mode = INIT;
	}
	
}
