#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"
#include <math.h>
#include "motor_control.h"
#include "communicat.h"
typedef struct
{
  uint16_t target_speed[MOTOR_COUNT];
} target_speed_t;

typedef enum
{
	STOP = 0,		// 停止模式
	MOTOR_CONTROL,	// 电机控制模式
} mode_enum;

typedef enum
{
	INIT = 0,			// 初始化设备
	GET_DATA,			// 读取液位变送器和流量传感器反馈数值
	TRANSMIT_DATA,// 发送液位变送器和流量传感器反馈数值
	SET_MOTOR_SPEED,	// 计算电机速度模式
} motor_control_mode_enum;

typedef struct
{
  mode_enum mode;
	motor_control_mode_enum motor_control_mode;
} system_state_typedef;

void control_init(void);
void control_start(void);

#endif // CONTROL_H
