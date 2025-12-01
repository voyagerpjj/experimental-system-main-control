//#include "ifr_usart.h"

//// 串口类指针数组，用于定向查找解析函数
//Usart_ClassDef* USART_Pointers[11] = {NULL};

//// 串口初始化
//void Usart_init(Usart_ClassDef* _usart, UART_HandleTypeDef* huart, void(*UART_Analysis_Function)(uint8_t *pData, uint8_t len))
//{
//	if (_usart == NULL || huart == NULL)
//		return;  // 空指针检查
//	
//	if (USART_Pointers[IFR_Uart_ID_Get(huart)] == NULL)
//		USART_Pointers[IFR_Uart_ID_Get(huart)] = _usart;
//	
//	_usart->m_huart = huart;
//	_usart->receiveFlag = 0;
//	_usart->receiveCounter = 0;
//	_usart->receive_memorybuff_rear = 0;
//	memset(_usart->transmit_buff, 0, sizeof(_usart->transmit_buff));
//	memset(_usart->receive_buff, 0, sizeof(_usart->receive_buff));
//	memset(_usart->receiveMemoryBuff, 0, sizeof(_usart->receiveMemoryBuff));
//	HAL_UART_RegisterRxEventCallback(_usart->m_huart, IFR_UART_Rx_Callback);
//	_usart->AnalysisFunc = UART_Analysis_Function;
//	
//	// 启动DMA空闲接收
//  HAL_UARTEx_ReceiveToIdle_DMA(_usart->m_huart, _usart->receiveMemoryBuff[_usart->receive_memorybuff_rear], sizeof(_usart->receiveMemoryBuff[_usart->receive_memorybuff_rear]));
//	// 禁用DMA半传输中断
//	__HAL_DMA_DISABLE_IT(_usart->m_huart->hdmarx, DMA_IT_HT);
//}
//static void TX_Process
///**
//  * @概述	接收数据处理
//  * @参数1	接收数据长度
//  * @返回值 void
//  */
//static void Rx_Process(Usart_ClassDef* _usart, uint16_t len) 
//{
//	if (len > 0) 
//	{
//		// 将数据从缓存区中搬运到处理区 
//		memcpy(_usart->receive_buff, _usart->receiveMemoryBuff[_usart->receive_memorybuff_rear], len);
//		if (_usart->AnalysisFunc != NULL) 
//			_usart->AnalysisFunc(_usart->receive_buff, len);
//		
//		_usart->receive_memorybuff_rear = (_usart->receive_memorybuff_rear+ 1) % UART_RX_BUFF;
//		// 启动DMA空闲接收
//		HAL_UARTEx_ReceiveToIdle_DMA(_usart->m_huart, _usart->receiveMemoryBuff[_usart->receive_memorybuff_rear], sizeof(_usart->receiveMemoryBuff[_usart->receive_memorybuff_rear]));
//		// 禁用DMA半传输中断
//		__HAL_DMA_DISABLE_IT(_usart->m_huart->hdmarx, DMA_IT_HT);
//		// 更新最新消息时间戳
//		_usart->_updata_systick = HAL_GetTick();
//	}
//}
///**
//  * @概述	串口接收回调函数
//  * @参数1	串口句柄
//  * @参数2	接收数据长度
//  * @返回值 void
//  */
//static void IFR_UART_Rx_Callback(UART_HandleTypeDef *huart, uint16_t len) 
//{
//	uint8_t uart_id = IFR_Uart_ID_Get(huart);
//	if (USART_Pointers[uart_id] != NULL)
//			Rx_Process(USART_Pointers[uart_id], len);
//}

///**
//  * @概述	获取串口ID
//  * @参数1	串口句柄
//  * @返回值 串口ID
//  */
//static uint8_t IFR_Uart_ID_Get(UART_HandleTypeDef *huart) 
//{
//    if (huart->Instance == USART1)           return 1;
//    else if (huart->Instance == USART2)      return 2;
//    else if (huart->Instance == USART3)      return 3;
//#if defined(STM32F407xx) || defined(STM32F405xx) || defined(STM32F411xx)
//    else if (huart->Instance == UART4)       return 4;
//    else if (huart->Instance == UART5)       return 5;
//    else if (huart->Instance == USART6)      return 6;
//#elif defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx)|| defined(STM32F439xx) || defined(STM32F469xx) || defined(STM32F479xx)
//    else if (huart->Instance == UART7)       return 7;
//    else if (huart->Instance == UART8)       return 8;
//    else if (huart->Instance == UART9)       return 9;
//    else if (huart->Instance == UART10)      return 10;
//#elif defined(STM32H723xx)
//    else if (huart->Instance == UART7)       return 7;
//    else if (huart->Instance == UART8)       return 8;
//    else if (huart->Instance == UART9)       return 9;
//    else if (huart->Instance == USART10)     return 10;
//#endif
//    else return 0;
//}
