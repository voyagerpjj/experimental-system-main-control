#include "control.h"

target_speed_t target_motor_speed = {.target_speed = {0}};  //目标速度

timer_typedef tim_6;  //定时器6

void control_init(void)
{
	motor_all_init();  //电机初始化
	timer_start(&tim_6, &htim6, control_runing);  //定时器6初始化
	target_motor_speed.target_speed[0] = 1000;
	target_motor_speed.target_speed[1] = 1000;	
}

void control_input(void)
{
    
}

void control_runing(void)
{
	// 电机控制
//	for (int i = 0; i < MOTOR_COUNT; i++)
//	{
			motor_control(0, target_motor_speed.target_speed[0]);
			motor_control(1, target_motor_speed.target_speed[1]);
//	}
}

