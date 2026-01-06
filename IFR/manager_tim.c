#include "manager_tim.h"

#ifdef HAL_TIM_MODULE_ENABLED
#if USE_HAL_TIM_REGISTER_CALLBACKS // 需使能TIM回调注册功能

/**
  * @brief   定时器对象指针数组（最大支持15个定时器，用于中断定向回调）
  */
static manager_tim_typedef* TIM_Pointer[15] = {0};

/**
  * @brief   根据定时器句柄获取唯一ID（用于匹配回调函数）
  * @param   htim: 定时器句柄指针
  * @retval  uint8_t: 定时器ID（1-14，0为非法）
  */
static uint8_t manager_tim_id_get(TIM_HandleTypeDef *htim);    

/**
  * @brief   定时器溢出中断统一回调函数（分发至自定义回调）
  * @param   htim: 定时器句柄指针
  * @retval  void
  */
void manager_tim_over_callback(TIM_HandleTypeDef *htim);

/**
  * @brief   定时器溢出中断统一回调函数（禁止改写/直接调用）
  * @note    根据定时器ID匹配自定义回调函数并执行
  * @param   htim: 定时器句柄指针
  * @retval  void
  */
void manager_tim_over_callback(TIM_HandleTypeDef *htim)
{
    uint8_t tim_id = manager_tim_id_get(htim);
    // 存在自定义回调则执行
    if(TIM_Pointer[tim_id] != NULL) TIM_Pointer[tim_id]->CustomCallBack();
}

/**
  * @brief   定时器管理器初始化（注册回调+绑定定时器对象）
  * @param   manager_tim_t: 定时器管理器对象指针
  * @param   htim: 定时器句柄指针
  * @param   callback: 自定义溢出回调函数指针
  * @retval  void
  */
void manager_tim_init(manager_tim_typedef * manager_tim_t, TIM_HandleTypeDef* htim, void (*callback)(void))
{
    manager_tim_t->_tim = htim;                  // 绑定定时器句柄
    manager_tim_t->CustomCallBack = callback;    // 绑定自定义回调函数
    // 注册定时器指针，用于中断定向查找
    TIM_Pointer[manager_tim_id_get(htim)] = manager_tim_t;    
    // 注册定时器溢出中断回调函数
    HAL_TIM_RegisterCallback(htim, HAL_TIM_PERIOD_ELAPSED_CB_ID, manager_tim_over_callback);    
}

/**
  * @brief   启动定时器（开启溢出中断）
  * @param   manager_tim_t: 定时器管理器对象指针
  * @retval  void
  */
void manager_tim_start(manager_tim_typedef * manager_tim_t)
{
    // 启动定时器基础模式+溢出中断
    HAL_TIM_Base_Start_IT(manager_tim_t->_tim);    
}

/**
  * @brief   停止定时器（关闭溢出中断）
  * @param   manager_tim_t: 定时器管理器对象指针
  * @retval  void
  */
void manager_tim_stop(manager_tim_typedef * manager_tim_t)
{
    // 停止定时器基础模式+溢出中断
    HAL_TIM_Base_Stop_IT(manager_tim_t->_tim);    
}

/**
  * @brief   根据定时器句柄获取唯一ID（静态函数，外部禁止调用）
  * @param   htim: 定时器句柄指针
  * @retval  uint8_t: 定时器ID（1-14，0为非法）
  */
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
	else return 0; // 非法定时器返回0
}
#endif
#endif
