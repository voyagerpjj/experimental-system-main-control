#ifndef _MANAGER_TIM_H_
#define _MANAGER_TIM_H_

#include "main.h"

#ifdef HAL_TIM_MODULE_ENABLED
#if USE_HAL_TIM_REGISTER_CALLBACKS //如果底下是虚的说明你没使能Register Callback TIM
#include "tim.h"

typedef struct 
{
	TIM_HandleTypeDef* _tim;	// 定时器句柄
	void (*CustomCallBack)(void);		// 定时器回调函数
} manager_tim_typedef;                /*定时器结构体*/

void manager_tim_init(manager_tim_typedef * timer_t, TIM_HandleTypeDef* htim, void (*callback)(void));	// 启动定时器
void manager_tim_start(manager_tim_typedef * manager_tim_t);	// 启动定时器
void manager_tim_stop(manager_tim_typedef * manager_tim_t);	// 停止定时器

#endif
#endif
#endif // _MANAGER_TIM_H_
