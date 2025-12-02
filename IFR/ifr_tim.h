#ifndef _TIMER_H_
#define _TIMER_H_

#include "main.h"

#ifdef HAL_TIM_MODULE_ENABLED
#include "tim.h"

typedef struct 
{
	TIM_HandleTypeDef* timer;	// 定时器句柄
	void (*callback)(void);		// 定时器回调函数
} timer_typedef;                /*定时器结构体*/

void timer_start(timer_typedef * timer_t, TIM_HandleTypeDef* htim, void (*callback)(void));

static void TaskFunction_Call(timer_typedef * timer_t);

static uint8_t IFR_TIM_ID_GET(TIM_HandleTypeDef *htim);
#endif
#endif // _TIMER_H_
