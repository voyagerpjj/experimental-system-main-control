///*
// * @FilePath     : communication
// * @Author       : zealerlu
// * @Date         : 2023-02-19
// * @LastEditors  : zealerlu
// * @LastEditTime : 2023-02-19
// * @Description  : 
// * 
// * Copyright (c) 2023 by BZLZ, All Rights Reserved. 
// */

//#include "communication.h"

//extern motorState_t motor[4];
//extern uint8_t g_Communication;

//uartParams_t rs485_uart = {
//    .huart = &huart1,
//    .receive_buff = {0},
//	.receiveTemporaryBuff = {0},
//    .transmit_buff = {0},
//	.receiveFlag = 0,
//	.receiveCounter = 0
//};

//uartParams_t computer_uart = {
//    .huart = &huart4,
//    .receive_buff = {0},
//	.receiveTemporaryBuff = {0},
//    .transmit_buff = {0},
//	.receiveFlag = 0,
//	.receiveCounter = 0
//};

//void commInit(void)
//{
//    /* 开启串口接收DMA */
//	
//	/***开启HAL库串口空闲中断***/
////    HAL_UARTEx_ReceiveToIdle_DMA(rs485_uart.huart, rs485_uart.receiveTemporaryBuff, UART_RX_BUF_SIZE);
////    HAL_UARTEx_ReceiveToIdle_DMA(computer_uart.huart, computer_uart.receiveTemporaryBuff, UART_RX_BUF_SIZE);
//	
//	/***开启IFR自研串口空闲中断**/
//	__HAL_UART_ENABLE_IT(rs485_uart.huart, UART_IT_IDLE); 
//    HAL_UART_Receive_DMA(rs485_uart.huart, rs485_uart.receiveTemporaryBuff, UART_RX_BUF_SIZE);
//	
//	__HAL_UART_ENABLE_IT(computer_uart.huart, UART_IT_IDLE); 
//	HAL_UART_Receive_DMA(computer_uart.huart, computer_uart.receiveTemporaryBuff, UART_RX_BUF_SIZE);
//}

///*获取接收位数函数*/
//uint8_t GetReceiveCounter(uartParams_t* huart_rx)
//{
//	uint8_t len;
//	len = UART_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart_rx->huart->hdmarx);
//	return len;
//}

//bool CrcCheck(uint8_t *ptr, uint8_t len)
//{
//	splice_u crcCheck;
//	crcCheck.data[0] = ptr[len - 1];
//    crcCheck.data[1] = ptr[len - 2];
//	if(crcCheck.result == Get_CRC16_Check(ptr, len - 2))
//		return true;
//	else
//		return false;
//}

//HAL_StatusTypeDef HAL_UART_DMA_StopRx(UART_HandleTypeDef *huart)
//{
//	/* The Lock is not implemented on this API to allow the user application
//	to call the HAL UART API under callbacks HAL_UART_TxCpltCallback() / HAL_UART_RxCpltCallback():
//	when calling HAL_DMA_Abort() API the DMA TX/RX Transfer complete interrupt is generated
//	and the correspond call back is executed HAL_UART_TxCpltCallback() / HAL_UART_RxCpltCallback()
//	*/
//	/* Stop UART DMA Rx request if ongoing */
//	uint32_t dmarequest = 0x00U;
//	dmarequest = HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR);
//	if ((huart->RxState == HAL_UART_STATE_BUSY_RX) && dmarequest)
//	{
//		ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

//		/* Abort the UART DMA Rx stream */
//		if (huart->hdmarx != NULL)
//		{
//		  HAL_DMA_Abort(huart->hdmarx);
//		}
//		//UART_EndRxTransfer(huart);//static Expand start
//		/* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
//		  ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
//		  ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

//		  /* In case of reception waiting for IDLE event, disable also the IDLE IE interrupt source */
//		  if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
//		  {
//			ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
//		  }

//		  /* At end of Rx process, restore huart->RxState to Ready */
//		  huart->RxState = HAL_UART_STATE_READY;
//		  huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
//		//static Expand start
//	}
//	return HAL_OK;
//}

///*
//串口数据发送
//*/
//void dataTransmit(uartParams_t* huart, uint8_t* data_buff, uint8_t len)
//{
//	if(huart == &rs485_uart)
//	{
////		__HAL_UART_ENABLE_IT(rs485_uart.huart, UART_IT_IDLE);
//		HAL_GPIO_WritePin(COM_CTS_GPIO_Port, COM_CTS_Pin, GPIO_PIN_SET);
//	}
////	while(__HAL_DMA_GET_COUNTER(huart->huart->hdmatx));
////	memset(huart->transmit_buff, 0, UART_TX_BUF_SIZE);
//	memcpy(huart->transmit_buff, data_buff, len);
//	HAL_UART_Transmit_DMA(huart->huart, huart->transmit_buff, len);
////	if(huart == &rs485_uart)
////		__HAL_UART_ENABLE_IT(rs485_uart.huart, UART_FLAG_TC);
//}

///*
//串口DMA数据发送（转发数据）
//param:
//    huart_rx 数据接收串口
//    huart_tx 数据发送串口
//*/
//void dataTranspond(uartParams_t* huart_rx, uartParams_t* huart_tx, uint8_t len)
//{
//    if(len > 0)
//    {
//        dataTransmit(huart_tx, huart_rx->receive_buff, len);
////        memset(huart_rx->receive_buff, 0, UART_RX_BUF_SIZE);
//    }
//}

///*串口接收解析函数*/
//messageType_e dataReserveDecoder(uint8_t* dataBuff, uint8_t len)
//{
//    messageType_e messageType = ERROR_MESSAGE;
//	if(CrcCheck(dataBuff, len))
//	{
//		if(dataBuff[0] == 0x0f)
//		{
//			if(dataBuff[1] == 0x06)
//			{
//				if((dataBuff[5] & 0x01) == 0x01)
//					HAL_GPIO_WritePin(TRAN1_EN_GPIO_Port, TRAN1_EN_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(TRAN1_EN_GPIO_Port, TRAN1_EN_Pin, GPIO_PIN_RESET);

//				if((dataBuff[5] & 0x02) == 0x02)
//					HAL_GPIO_WritePin(TRAN2_EN_GPIO_Port, TRAN2_EN_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(TRAN2_EN_GPIO_Port, TRAN2_EN_Pin, GPIO_PIN_RESET);
//				
//				if((dataBuff[5] & 0x04) == 0x04)
//					HAL_GPIO_WritePin(TRAN3_EN_GPIO_Port, TRAN3_EN_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(TRAN3_EN_GPIO_Port, TRAN3_EN_Pin, GPIO_PIN_RESET);
//				
//				if((dataBuff[5] & 0x08) == 0x08)
//					HAL_GPIO_WritePin(TRAN4_EN_GPIO_Port, TRAN4_EN_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(TRAN4_EN_GPIO_Port, TRAN4_EN_Pin, GPIO_PIN_RESET);
//				
//				if((dataBuff[5] & 0x10) == 0x10)
//					HAL_GPIO_WritePin(TRAN5_EN_GPIO_Port, TRAN5_EN_Pin, GPIO_PIN_SET);
//				else
//					HAL_GPIO_WritePin(TRAN5_EN_GPIO_Port, TRAN5_EN_Pin, GPIO_PIN_RESET);
//							
////				log_debug("Communication:Device Power Message!");
//				messageType = DEVICE_POWER_MSSAGE;
//			}
//			else if (dataBuff[1] == 0x10)
//			{
//				if(len != (7+dataBuff[6]+2))
//				{
////					log_error("Communication:Unkonwn Message!");
//					messageType = ERROR_MESSAGE;
//					return messageType;
//				}
//				splice_u crcCheckReturn;
//				
//				if(dataBuff[5] == 4)
//				{ 
//					motor[M1].target_speed.data[1] = dataBuff[7];
//					motor[M1].target_speed.data[0] = dataBuff[8];
//					if(motor[M1].target_speed.result > 6000)
//					{
//						messageType = ERROR_MESSAGE;
//						return messageType;
//					}
//				
//					motor[M2].target_speed.data[1] = dataBuff[9];
//					motor[M2].target_speed.data[0] = dataBuff[10];
//					if(motor[M2].target_speed.result > 6000)
//					{
//						messageType = ERROR_MESSAGE;
//						return messageType;
//					}
//				
//					motor[M3].target_speed.data[1] = dataBuff[11];
//					motor[M3].target_speed.data[0] = dataBuff[12];
//					if(motor[M3].target_speed.result > 6000)
//					{
//						messageType = ERROR_MESSAGE;
//						return messageType;
//					}
//				
//					motor[M4].target_speed.data[1] = dataBuff[13];
//					motor[M4].target_speed.data[0] = dataBuff[14];
//					if(motor[M4].target_speed.result > 6000)
//					{
//						messageType = ERROR_MESSAGE;
//						return messageType;
//					}
//					
//					crcCheckReturn.result = Get_CRC16_Check(dataBuff, 6);
//					dataBuff[6] = crcCheckReturn.data[1];
//					dataBuff[7] = crcCheckReturn.data[0];
//				}
//				else if(dataBuff[5] == 3)
//				{
//					motor[M1].target_speed.data[1] = dataBuff[7];
//					motor[M1].target_speed.data[0] = dataBuff[8];
//				
//					motor[M2].target_speed.data[1] = dataBuff[9];
//					motor[M2].target_speed.data[0] = dataBuff[10];
//				
//					motor[M3].target_speed.data[1] = dataBuff[11];
//					motor[M3].target_speed.data[0] = dataBuff[12];
//					
//					crcCheckReturn.result = Get_CRC16_Check(dataBuff, 6);
//					dataBuff[6] = crcCheckReturn.data[1];
//					dataBuff[7] = crcCheckReturn.data[0];
//				}
//				else if(dataBuff[5] == 2)
//				{
//					motor[M1].target_speed.data[1] = dataBuff[7];
//					motor[M1].target_speed.data[0] = dataBuff[8];
//				
//					motor[M2].target_speed.data[1] = dataBuff[9];
//					motor[M2].target_speed.data[0] = dataBuff[10];
//					
//					crcCheckReturn.result = Get_CRC16_Check(dataBuff, 6);
//					dataBuff[6] = crcCheckReturn.data[1];
//					dataBuff[7] = crcCheckReturn.data[0];
//				}
//				else if(dataBuff[5] == 1)
//				{
//					motor[M1].target_speed.data[1] = dataBuff[7];
//					motor[M1].target_speed.data[0] = dataBuff[8];
//					
//					crcCheckReturn.result = Get_CRC16_Check(dataBuff, 6);
//					dataBuff[6] = crcCheckReturn.data[1];
//					dataBuff[7] = crcCheckReturn.data[0];
//				}
//				else
//				{
//					crcCheckReturn.result = Get_CRC16_Check(dataBuff, 6);
//					dataBuff[6] = crcCheckReturn.data[1];
//					dataBuff[7] = crcCheckReturn.data[0];
//				}

////				log_debug("Communication:Motor Crtl Message!");
//				messageType = MOTOR_CRTL_MESSAGE;
//			}
//			else
//			{
////				log_error("Communication:Unkonwn Message!");
//				messageType = ERROR_MESSAGE;
//			}
//		}
//		else
//		{				
////			log_debug("Communication:Forward Message!");
//			messageType = FORWARD_MESSAGE;
//		}
//	}
//	else
//	{
////		log_error("Communication:Unkonwn Message!");
//		messageType = ERROR_MESSAGE;
//	}
//    return messageType;
//}

////void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
////{
////    if(huart == rs485_uart.huart)
////	{
////		rs485_uart.receiveCounter = GetReceiveCounter(&rs485_uart);
////		memcpy(rs485_uart.receive_buff, rs485_uart.receiveTemporaryBuff, rs485_uart.receiveCounter);
////		memset(rs485_uart.receiveTemporaryBuff, 0, UART_RX_BUF_SIZE);
////		rs485_uart.receiveFlag = 1;
////	}
////    if(huart == computer_uart.huart)
////    {
////		computer_uart.receiveCounter = GetReceiveCounter(&computer_uart);
////		memcpy(computer_uart.receive_buff, computer_uart.receiveTemporaryBuff, computer_uart.receiveCounter);
////		memset(computer_uart.receiveTemporaryBuff, 0, UART_RX_BUF_SIZE);
////		computer_uart.receiveFlag = 1;
////    }
////}

//void USER_UART_RxIdleCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == rs485_uart.huart)
//	{
//		if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))
//		{
//			__HAL_UART_CLEAR_IDLEFLAG(huart);
//			HAL_UART_DMA_StopRx(huart);
//			
//			rs485_uart.receiveCounter = GetReceiveCounter(&rs485_uart);
//			memcpy(rs485_uart.receive_buff, rs485_uart.receiveTemporaryBuff, rs485_uart.receiveCounter);
//			memset(rs485_uart.receiveTemporaryBuff, 0, UART_RX_BUF_SIZE);
//			rs485_uart.receiveFlag = 1;
//			
////			HAL_UART_Receive_DMA(rs485_uart.huart, rs485_uart.receiveTemporaryBuff, UART_RX_BUF_SIZE);
//		}
//	}
//    if(huart == computer_uart.huart)
//    {
//		if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))
//		{
//			__HAL_UART_CLEAR_IDLEFLAG(huart);
//			HAL_UART_DMA_StopRx(huart);
//			
//			computer_uart.receiveCounter = GetReceiveCounter(&computer_uart);
//			memcpy(computer_uart.receive_buff, computer_uart.receiveTemporaryBuff, computer_uart.receiveCounter);
//			memset(computer_uart.receiveTemporaryBuff, 0, UART_RX_BUF_SIZE);
//			computer_uart.receiveFlag = 1;
//			g_Communication = 0;
////			HAL_UART_Receive_DMA(computer_uart.huart, computer_uart.receiveTemporaryBuff, UART_RX_BUF_SIZE);
//		}
//    }
//}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//		if(huart == rs485_uart.huart)
//		{
//			HAL_GPIO_WritePin(COM_CTS_GPIO_Port, COM_CTS_Pin, GPIO_PIN_RESET);
////			__HAL_UART_DISABLE_IT(rs485_uart.huart, UART_IT_IDLE);
////			__HAL_UART_DISABLE_IT(rs485_uart.huart, UART_FLAG_TC);
//		}
//}
