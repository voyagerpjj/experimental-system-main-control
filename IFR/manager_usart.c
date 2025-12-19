#include "manager_usart.h"

// 静态函数声明
static void manager_usart_doubleBuffer_recevice(manager_usart_typedef *manager_usart, uint16_t len);
static uint8_t manager_usart_id_get(UART_HandleTypeDef *huart);
static void manager_usart_restart(manager_usart_typedef *manager_usart);

// 回调函数声明
void manager_usart_rx_callback(UART_HandleTypeDef *huart, uint16_t len);
void manager_usart_tx_callback(UART_HandleTypeDef *huart);
void manager_usart_error_callback(UART_HandleTypeDef *huart);

// 串口对象指针数组，用于根据串口句柄查找对应的对象
static manager_usart_typedef* USART_Pointers[11] = {NULL};

/**
  * @brief   串口初始化
  * @param   manager_usart: 串口对象指针
  * @param   huart: 串口句柄
  * @param   UART_Analysis_Function: 数据解析函数指针
  * @retval  void
  */
void manager_usart_init(manager_usart_typedef *manager_usart, UART_HandleTypeDef *huart, void(*UART_Analysis_Function)(uint8_t *pData, uint8_t len)) 
{
	manager_usart->_huart = huart;
	uint8_t uart_id = manager_usart_id_get(huart);
	USART_Pointers[uart_id] = manager_usart;
	
	if (UART_Analysis_Function != NULL)
			manager_usart->ParseFrameFuncPointer = UART_Analysis_Function;
	
	// 注册接收回调函数 和 错误回调函数
	HAL_UART_RegisterRxEventCallback(huart, manager_usart_rx_callback);
	HAL_UART_RegisterCallback(huart, HAL_UART_TX_COMPLETE_CB_ID, manager_usart_tx_callback);
	HAL_UART_RegisterCallback(huart, HAL_UART_ERROR_CB_ID, manager_usart_error_callback);
}
/**
  * @brief   串口注册发送完成中断回调函数
  * @param   manager_usart: 串口对象指针
  * @param   UART_Analysis_Function: 发送完成中断回调函数指针
  * @retval  void
  */
void manager_usart_register_tx_callback(manager_usart_typedef *manager_usart, void (tx_callback)(UART_HandleTypeDef *huart)) 
{
	if (tx_callback != NULL)
			manager_usart->TxCompleteFuncPointer = tx_callback;
}
/**
  * @brief   启动DMA接收
  * @param   manager_usart: 串口对象指针
  * @retval  void
  */
void manager_usart_start(manager_usart_typedef *manager_usart) 
{ 
	// 启动DMA空闲接收
	manager_usart->UsartRxStatus = HAL_UARTEx_ReceiveToIdle_DMA(manager_usart->_huart, manager_usart->RxDmaBuffer[manager_usart->RxDmaBufferIdx], USART_RX_RING_BUFFER_SIZE);
	
	// 禁用DMA半传输中断
	__HAL_DMA_DISABLE_IT(manager_usart->_huart->hdmarx, DMA_IT_HT);
}
/**
  * @brief   启动DMA发送
  * @param   manager_usart: 串口对象指针
  * @retval  void
  */
HAL_StatusTypeDef manager_usart_transmit(manager_usart_typedef *manager_usart, uint8_t *pData, uint16_t Size) 
{ 
	// 若串口发送DMA空闲，启动DMA发送
	if (manager_usart->_huart->hdmatx->State == HAL_DMA_STATE_READY)
		manager_usart->UsartTxStatus = HAL_UART_Transmit_DMA(manager_usart->_huart, pData, Size);
	else 
		manager_usart->UsartTxStatus = HAL_ERROR;
	
  return manager_usart->UsartTxStatus;
}
/**
  * @brief   双缓存接收数据处理
  * @param   manager_usart: 串口对象指针
  * @param   len: 接收数据长度
  * @retval  void
  */
static void manager_usart_doubleBuffer_recevice(manager_usart_typedef *manager_usart, uint16_t len) 
{
	// 切换缓冲区并保存数据长度
	manager_usart->RxDmaBufferIdx = !manager_usart->RxDmaBufferIdx;
	manager_usart->RxDataLength = len;
	
	// 如果设置了解析函数，则调用
	if (manager_usart->ParseFrameFuncPointer != NULL) 
			manager_usart->ParseFrameFuncPointer(manager_usart->RxDmaBuffer[!manager_usart->RxDmaBufferIdx], len);
	
	// 重新启动DMA接收
	manager_usart_start(manager_usart);
	
	// 更新时间戳
	manager_usart->LastRxStamp = HAL_GetTick();
}


/**
  * @brief   串口接收回调函数
  * @param   huart: 串口句柄
  * @param   len: 接收数据长度
  * @retval  void
  */
void manager_usart_rx_callback(UART_HandleTypeDef *huart, uint16_t len) 
{
	uint8_t uart_id = manager_usart_id_get(huart);
	if (USART_Pointers[uart_id] != NULL)
			manager_usart_doubleBuffer_recevice(USART_Pointers[uart_id], len);
}
/**
  * @brief   串口发送完成回调函数
  * @param   huart: 串口句柄
  * @retval  void
  */
void manager_usart_tx_callback(UART_HandleTypeDef *huart)
{
	uint8_t uart_id = manager_usart_id_get(huart);
	if (USART_Pointers[uart_id] != NULL)
		if (USART_Pointers[uart_id]->TxCompleteFuncPointer != NULL)
			USART_Pointers[uart_id]->TxCompleteFuncPointer(huart);
}
/**
  * @brief   串口错误回调函数
  * @param   huart: 串口句柄
  * @retval  void
  */
void manager_usart_error_callback(UART_HandleTypeDef *huart) 
{
	uint8_t uart_id = manager_usart_id_get(huart);
	if (USART_Pointers[uart_id] != NULL) 
	{
			manager_usart_restart(USART_Pointers[uart_id]);
	}
}

/**
  * @brief   串口错误恢复，重新初始化串口
  * @param   manager_usart: 串口对象指针
  * @retval  void
  */
static void manager_usart_restart(manager_usart_typedef *manager_usart) 
{
	if (manager_usart->_huart == NULL) return;
	
	// 清除错误标志
	manager_usart->_huart->ErrorCode = HAL_UART_ERROR_NONE;
	__HAL_UNLOCK(manager_usart->_huart);
	
	// 重新初始化硬件资源
	HAL_UART_MspDeInit(manager_usart->_huart);
	HAL_UART_MspInit(manager_usart->_huart);
	
	// 重新注册回调函数并启动接收
	HAL_UART_RegisterRxEventCallback(manager_usart->_huart, manager_usart_rx_callback);
	HAL_UART_RegisterCallback(manager_usart->_huart, HAL_UART_ERROR_CB_ID, manager_usart_error_callback);
	
	manager_usart_start(manager_usart);
}

/**
  * @brief   获取串口ID
  * @param   huart: 串口句柄
  * @retval  串口ID (0-10，0表示无效串口)
  */
static uint8_t manager_usart_id_get(UART_HandleTypeDef *huart) 
{
    if (huart->Instance == USART1)           return 1;
    else if (huart->Instance == USART2)      return 2;
    else if (huart->Instance == USART3)      return 3;
#if defined(STM32F407xx) || defined(STM32F405xx) || defined(STM32F411xx)
    else if (huart->Instance == UART4)       return 4;
    else if (huart->Instance == UART5)       return 5;
    else if (huart->Instance == USART6)      return 6;
#elif defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx)|| defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)
    else if (huart->Instance == UART7)       return 7;
    else if (huart->Instance == UART8)       return 8;
    else if (huart->Instance == UART9)       return 9;
    else if (huart->Instance == UART10)      return 10;
#elif defined(STM32H723xx)
    else if (huart->Instance == UART7)       return 7;
    else if (huart->Instance == UART8)       return 8;
    else if (huart->Instance == UART9)       return 9;
    else if (huart->Instance == USART10)     return 10;
#endif
    else return 0;
}
