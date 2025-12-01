//#include "test.h"

//void test_init()
//{
//	commInit(); // 串口接收初始化
//	motorInit(); // 电机对应PWM口初始化
//	HAL_TIM_Base_Start_IT(&htim6);	// 初始化主控定时器
//	HAL_GPIO_WritePin(STATE_GPIO_Port, STATE_Pin, GPIO_PIN_SET); // 点亮状态灯
//	
//}
//uint16_t tar_speed;
//void test_runing()
//{
//	if (tar_speed > 6100)
//		tar_speed = 6100;
//	motor_set_test_speed(M1, tar_speed);
//	motor_set_test_speed(M2, tar_speed);	
//	motor_set_test_speed(M3, tar_speed);
//	motor_set_test_speed(M4, tar_speed);	
//}


////void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
////{
////	if(htim == motor_params[M1].timer)
////	{		
////		/***debug***/
//////		HAL_GPIO_TogglePin(motor_params[M1].enable_port, motor_params[M1].enable_pin);
////		/***debug***/

////		motor[M1].state = STATE_AVESPEED;
////		memset(motor[M1].s_calc.toggle_pulse, 0, sizeof(motor[M1].s_calc.toggle_pulse));
////		motor[M1].speed = motor[M1].target_speed.result;
////	}
////   if(htim == motor_params[M2].timer)
////   {
////		/***debug***/
//////		HAL_GPIO_TogglePin(motor_params[M2].enable_port, motor_params[M2].enable_pin);
////		/***debug***/

////		motor[M2].state = STATE_AVESPEED;
////		memset(motor[M2].s_calc.toggle_pulse, 0, sizeof(motor[M2].s_calc.toggle_pulse));
////		motor[M2].speed = motor[M2].target_speed.result;
////   }
////    if(htim == motor_params[M3].timer)
////	{
////     	/***debug***/
//////		HAL_GPIO_TogglePin(motor_params[M3].enable_port, motor_params[M3].enable_pin);
////		/***debug***/

////		motor[M3].state = STATE_AVESPEED;
////		memset(motor[M3].s_calc.toggle_pulse, 0, sizeof(motor[M3].s_calc.toggle_pulse));
////		motor[M3].speed = motor[M3].target_speed.result;   
////	}
////    if(htim == motor_params[M4].timer)
////	{
////		/***debug***/
//////		HAL_GPIO_TogglePin(motor_params[M4].enable_port, motor_params[M4].enable_pin);
////		/***debug***/

////		motor[M4].state = STATE_AVESPEED;
////		memset(motor[M4].s_calc.toggle_pulse, 0, sizeof(motor[M4].s_calc.toggle_pulse));
////		motor[M4].speed = motor[M4].target_speed.result; 
////	}
////	if(htim == &htim6)
////	{
////		/* 心跳灯闪烁 */
////		//HAL_IWDG_Refresh(&hiwdg);
////		g_StatePeriod ++;
////		
////		if(g_StatePeriod > 1500)
////		{
////			g_StatePeriod = 0;
////			HAL_GPIO_WritePin(STATE_GPIO_Port, STATE_Pin, GPIO_PIN_SET);
////			g_StateLightFlag = 1;
////		}
//////		HAL_GPIO_WritePin(STATE_GPIO_Port, STATE_Pin, GPIO_PIN_SET);

////		if(g_StateLightFlag == 1)
////		{
////			g_StateTime ++;
////			if(g_StateTime > 50)
////			{
////				g_StateTime = 0;
////				HAL_GPIO_WritePin(STATE_GPIO_Port, STATE_Pin, GPIO_PIN_RESET);
////				g_StateLightFlag = 0;
////			}
////		}
////		
////		/*通信断开连接判断*/
////		g_CommunicationPeriod ++;
////		if(g_CommunicationPeriod >= 100)
////		{
////			g_Communication ++;
////			if(g_Communication >= 20)
////			{
////				for(motorIndex_e index = M1; index < Mmax; index ++)
////				{
////					motor[index].target_speed.result = 0;
////				}
////				g_MotorCtrlFlag = 1;
////			}
////			if(g_Communication >= 200)
////				g_Communication = 20;
////			g_CommunicationPeriod = 0;
////		}
////		if(rs485_uart.receiveFlag == 1)
////		{
//////			if(rs485_uart.receiveCounter == 14 || rs485_uart.receiveCounter == 10)
//////				rs485_uart.receiveCounter --;
////			dataTranspond(&rs485_uart, &computer_uart, rs485_uart.receiveCounter);
//////			HAL_GPIO_TogglePin(STATE_GPIO_Port, STATE_Pin);
////			rs485_uart.receiveFlag = 0;
////			
////			HAL_UART_Receive_DMA(rs485_uart.huart, rs485_uart.receiveTemporaryBuff, UART_RX_BUF_SIZE);
//////			HAL_UARTEx_ReceiveToIdle_DMA(rs485_uart.huart, rs485_uart.receiveTemporaryBuff, UART_RX_BUF_SIZE);
////		}
////		if(computer_uart.receiveFlag == 1)
////		{
////			switch(dataReserveDecoder(computer_uart.receive_buff, computer_uart.receiveCounter))
////			{
////				case FORWARD_MESSAGE:
////					dataTranspond(&computer_uart, &rs485_uart, computer_uart.receiveCounter);
////					break;
////				case DEVICE_POWER_MSSAGE:
////					dataTranspond(&computer_uart, &computer_uart, computer_uart.receiveCounter);
//////					HAL_GPIO_TogglePin(STATE_GPIO_Port, STATE_Pin);
////					break;
////				case MOTOR_CRTL_MESSAGE:
////					dataTransmit(&computer_uart, computer_uart.receive_buff, 8);
//////					HAL_GPIO_TogglePin(STATE_GPIO_Port, STATE_Pin);
////					g_MotorCtrlFlag = 1;
////					break;
////				case ERROR_MESSAGE:
////					break;
////				default:
////					break;
////			}
////			computer_uart.receiveFlag = 0;
////			
////			HAL_UART_Receive_DMA(computer_uart.huart, computer_uart.receiveTemporaryBuff, UART_RX_BUF_SIZE);
//////			HAL_UARTEx_ReceiveToIdle_DMA(computer_uart.huart, computer_uart.receiveTemporaryBuff, UART_RX_BUF_SIZE);
////		}
////	}
////}