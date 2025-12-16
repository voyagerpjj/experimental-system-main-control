#include "manager_tim.h"
#ifdef HAL_TIM_MODULE_ENABLED
#if USE_HAL_TIM_REGISTER_CALLBACKS //如果底下是虚的说明你没使能Register Callback TIM

static manager_tim_typedef* TIM_Pointer[15] = {0};//定时器类指针，用于定向查找任务函数
static uint8_t manager_tim_id_get(TIM_HandleTypeDef *htim);    //获取定时器ID
void manager_tim_over_callback(TIM_HandleTypeDef *htim);

//  自定义的中断回调函数，禁止改写和调用!!!
void manager_tim_over_callback(TIM_HandleTypeDef *htim)
{
    uint8_t tim_id = manager_tim_id_get(htim);
    if(TIM_Pointer[tim_id] != NULL) TIM_Pointer[tim_id]->CustomCallBack();
}

// 定时器初始化函数
void manager_tim_init(manager_tim_typedef * manager_tim_t, TIM_HandleTypeDef* htim, void (*callback)(void))
{
    manager_tim_t->_tim = htim;
    manager_tim_t->CustomCallBack = callback;   
    TIM_Pointer[manager_tim_id_get(htim)] = manager_tim_t;    //注册定时器类指针，用于定向查找任务函数
    
    HAL_TIM_RegisterCallback(htim, HAL_TIM_PERIOD_ELAPSED_CB_ID, manager_tim_over_callback);    //注册定时器溢出回调函数
}

// 定时器启动函数
void manager_tim_start(manager_tim_typedef * manager_tim_t)
{
    HAL_TIM_Base_Start_IT(manager_tim_t->_tim);    //启动定时器
}

// 定时器停止函数
void manager_tim_stop(manager_tim_typedef * manager_tim_t)
{
    HAL_TIM_Base_Stop_IT(manager_tim_t->_tim);    //停止定时器
}

//static函数外部无法调用，禁止改写!!!
static uint8_t manager_tim_id_get(TIM_HandleTypeDef *htim)
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
#endif
