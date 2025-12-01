#include "timer.h"
timer_typedef* TIM_Pointer[15] = {0};//定时器类指针，用于定向查找任务函数

#if USE_HAL_TIM_REGISTER_CALLBACKS //如果底下是虚的说明你没使能Register Callback TIM

//  自定义的中断回调函数，禁止改写和调用!!!
void TIM_Overflow_Callback(TIM_HandleTypeDef *htim)
{
    uint8_t tim_id = IFR_TIM_ID_GET(htim);
    if(TIM_Pointer[tim_id] != NULL) TaskFunction_Call(TIM_Pointer[tim_id]);
}

// 定时器初始化(启动) 注册对应的定时器回调函数，并启动定时器
void timer_start(timer_typedef * timer_t, TIM_HandleTypeDef* htim, void (*callback)(void))
{
    timer_t->timer = htim;
    timer_t->callback = callback;   
    TIM_Pointer[IFR_TIM_ID_GET(htim)] = timer_t;    //注册定时器类指针，用于定向查找任务函数
    
    HAL_TIM_RegisterCallback(htim, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Overflow_Callback);    //注册定时器溢出回调函数
    HAL_TIM_Base_Start_IT(htim);    //启动定时器
}


/**
  * @概述	调用任务函数。一般不对外使用。
  * @返回值 void
  */
static void TaskFunction_Call(timer_typedef * timer_t)
{
	if(timer_t->callback!= NULL) (*timer_t->callback)();
}

//static函数外部无法调用，禁止改写!!!
static uint8_t IFR_TIM_ID_GET(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)			return 1;
	else if(htim->Instance == TIM2)		return 2;
	else if(htim->Instance == TIM3)		return 3;
	else if(htim->Instance == TIM4)		return 4;
	else if(htim->Instance == TIM5)		return 5;
	else if(htim->Instance == TIM6)		return 6;
	else if(htim->Instance == TIM7)		return 7;
	else if(htim->Instance == TIM8)		return 8;
	#if defined(STM32F405xx)
	else if(htim->Instance == TIM9)		return 9;
	else if(htim->Instance == TIM10)	return 10;
	else if(htim->Instance == TIM11)	return 11;
	#elif defined(STM32F103xx)
	else if(htim->Instance == TIM12)	return 12;
	else if(htim->Instance == TIM13)	return 13;
	else if(htim->Instance == TIM14)	return 14;
	#endif
	else return 0;
}

#endif
