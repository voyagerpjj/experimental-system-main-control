///*
// * @FilePath     : motor
// * @Author       : dingyun
// * @Date         : 2022-10-11
// * @LastEditors  : starryding
// * @LastEditTime : 2023-04-20
// * @Description  : 
// * 
// * Copyright (c) 2022 by BZLZ , All Rights Reserved. 
// */


//#include "motor.h"
//motorParams_t motor_params[Mmax];
//uint16_t accel_tab[SPEED_INDEX];     /*速度表格 单位 step/s */

//uint8_t g_CommunicationPeriod = 0;
//uint8_t g_Communication = 0;
//uint16_t g_StatePeriod = 0;
//uint8_t g_StateTime = 0;
//uint8_t g_StateLightFlag = 0;
//uint8_t g_MotorCtrlFlag = 0;
//extern uartParams_t rs485_uart;
//extern uartParams_t computer_uart;

//motorState_t motor[Mmax] = {
//	{
//		.state = STATE_AVESPEED,
//		.speed = 0,
//        .target_speed.result = 0,
//		.s_calc = {
//            .accel_step = 0,
//            .toggle_pulse = {0},
//        },
//	},
//	{
//		.state = STATE_AVESPEED,
//		.speed = 0,
//        .target_speed.result = 0,
//		.s_calc = {
//            .accel_step = 0,
//            .toggle_pulse = {0},
//        },
//	},
//	{
//		.state = STATE_AVESPEED,
//		.speed = 0,
//		 .target_speed.result = 0,
//		.s_calc = {
//			 .accel_step = 0,
//			 .toggle_pulse = {0},
//		 },
//	},
//	{
//		.state = STATE_AVESPEED,
//		.speed = 0,
//		 .target_speed.result = 0,
//		.s_calc = {
//			 .accel_step = 0,
//			 .toggle_pulse = {0},
//		 },
//	},
//};

//void motorInit(void)
//{
//    motor_params[M1].timer = &htim2;
//    motor_params[M2].timer = &htim3;
//    motor_params[M3].timer = &htim4;
//    motor_params[M4].timer = &htim5;

//    motor_params[M1].channel = TIM_CHANNEL_2;
//    motor_params[M2].channel = TIM_CHANNEL_3;
//    motor_params[M3].channel = TIM_CHANNEL_1;
//    motor_params[M4].channel = TIM_CHANNEL_2;

//    motor_params[M1].enable_port = M1_EN_GPIO_Port;
//    motor_params[M2].enable_port = M2_EN_GPIO_Port;
//    motor_params[M3].enable_port = M3_EN_GPIO_Port;
//    motor_params[M4].enable_port = M4_EN_GPIO_Port;

//    motor_params[M1].enable_pin = M1_EN_Pin;
//    motor_params[M2].enable_pin = M2_EN_Pin;
//    motor_params[M3].enable_pin = M3_EN_Pin;
//    motor_params[M4].enable_pin = M4_EN_Pin;

//    for(motorIndex_e i = M1; i < Mmax; i++)
//    {
//      __HAL_TIM_SET_PRESCALER(motor_params[i].timer, TIMER_PRESCALER);
//			__HAL_TIM_SET_COMPARE(motor_params[i].timer, motor_params[i].channel, 0);
//			__HAL_TIM_SET_AUTORELOAD(motor_params[i].timer, 0);

//			HAL_TIM_OC_Start(motor_params[i].timer, motor_params[i].channel);
//    }
//}

//void motorEnable(motorIndex_e index, bool enabled)
//{
//    if(index >= Mmax)
//        return;
//    if( true == enabled)
//        HAL_GPIO_WritePin(motor_params[index].enable_port, motor_params[index].enable_pin, GPIO_PIN_SET);
//    else
//        HAL_GPIO_WritePin(motor_params[index].enable_port, motor_params[index].enable_pin, GPIO_PIN_RESET);
//}

//void motor_set_test_speed(motorIndex_e index, int tar_speed)
//{
//	motor[index].target_speed.result = tar_speed;
//	
//}
//// S = 1 / 6 * J * (t ** 3)
//// V = 1 / 2 * J * (t ** 2)
//// a = J * t
//uint8_t sMoveSpeedCalculate(motorIndex_e index, uint16_t vi,  uint16_t vo)
//{
//	int8_t dection = IS_ACC;
//	float half_time = 0;
//	float ti = 0;                               /* 时间间隔 dt */
//	float ti_cube = 0;                          /* 时间间隔的立方 */
//	float sum_t = 0;                            /* 时间累加量 */
//	float delta_v = 0;                          /* 速度的增量dv (step/s) */
//	float vin = 0;
//	float vout = 0;
//	float vmid = 0;

//	uint16_t inc_acc_stp = 0;                    /* 加加速所需的步数 */
//	uint16_t accel_step = 0;
//	uint16_t i = 0;

////	motor[index].target_speed.result = vo;
//	if(vo == 0)
//	{
//		motorEnable(index, false);
//		motor[index].speed = 0;
//		motor[index].state = STATE_AVESPEED;
//		return FALSE;
//	}
//	
//	vin = _x01RPM_TO_STEP_S(vi);
//	vout = _x01RPM_TO_STEP_S(vo);
//	vmid = 0.5f * (vin + vout);

//	if (vi > vo)                                /* 初速度比末速度大,做减速运动,数值变化跟加速运动相同 *//* 只是建表的时候注意将速度倒序 */
//		dection = IS_DEC;                       /* 减速 */
//	else
//		dection = IS_ACC;                         /* 加速 */
//	
////    log_debug("Calculate:the is_dec :%d", is_dec);


//    half_time = HALF_TIME(vin, vout);	// 

//	accel_step = (uint16_t)((vout + vin) * half_time) + 1;		/* 加速需要的总步数 */
//	inc_acc_stp = (uint16_t)(vin * half_time + dection * INCACCELSTEP(half_time));

//	ti_cube = 6.0f * 1.0f / STEP_S_J;          /* 根据位移和时间的公式S = 1/6 * J * ti^3 第1步的时间:ti^3 = 6 * 1 / jerk */
//	ti = pow(ti_cube, (1 / 3.0f));             /* ti */
//	
//	sum_t = ti;
//	delta_v = DELTA_STEP_S(sum_t);


//	accel_tab[0] = vin + dection * delta_v;

////		if( motor[index].s_calc.accel_tab[0] <= SPEED_MIN )              /* 以当前定时器频率所能达到的最低速度 */
////			motor[index].s_calc.accel_tab[0] = SPEED_MIN;
//		
//	for (i = 1; i < accel_step; i++)
//	{
//		ti = 1.0f / accel_tab[i - 1];
//		if (i < inc_acc_stp)
//		{
//			sum_t += ti;
//			delta_v = DELTA_STEP_S(sum_t);
//			accel_tab[i] = (uint16_t)(vin + dection * delta_v);
//			if ((dection * accel_tab[i] >= dection * vmid) || (i == inc_acc_stp - 1))
//			{
//				sum_t = fabs(sum_t - half_time);
//				inc_acc_stp = i;
//			}
//		}
//		else
//		{
//			sum_t += ti;
//			if (sum_t >= half_time)
//				sum_t = half_time;
//			delta_v = DELTA_STEP_S((half_time - sum_t));
//			accel_tab[i] = (uint16_t)(vout - dection * delta_v);
//			if (dection * accel_tab[i] >= dection * vout)
//			{
//				accel_step = i + 1;
//				break;
//			}
//		}
//	}
//	
//	for (uint16_t j = 0; j < accel_step; j++)
//        motor[index].s_calc.toggle_pulse[j] =  STEP_S_TO_TIMER(accel_tab[j]);

//	motor[index].s_calc.accel_step = accel_step;
//	memset(accel_tab, 0, sizeof(accel_tab));
//	return TRUE;
//}

//void motorSetSpeedList(motorIndex_e index)  //_speed * 0.1rpm
//{
//	motorEnable(index, true);	
//	/***debug***/
////	HAL_GPIO_TogglePin(motor_params[index].enable_port, motor_params[index].enable_pin);
//	/***debug***/
//	
////	motor[M1].s_calc.debug = 0;
//	__HAL_TIM_SET_AUTORELOAD(motor_params[index].timer, motor[index].s_calc.toggle_pulse[0]);
//	motor_params[index].timer->Instance->CNT = 0;
//	
//	HAL_TIM_Base_Start_DMA(motor_params[index].timer, 
//								(uint32_t*)&motor[index].s_calc.toggle_pulse[0], 
//								motor[index].s_calc.accel_step);

//	/***debug***/
////	HAL_GPIO_TogglePin(motor_params[index].enable_port, motor_params[index].enable_pin);
//	/***debug***/
//	motor[index].state = STATE_ACTIVE;                              /* 电机为加速状态 */
//}

//void sMoveSpeedStart(motorIndex_e index, uint16_t tar_speed)
//{
////	log_debug("Code Runing Message: sMoveSpeedStart !");
//    uint16_t speed_index = 0;
//    switch (motor[index].state)
//    {
//        case STATE_AVESPEED:
//            if(sMoveSpeedCalculate(index, motor[index].speed, tar_speed) == FALSE) /* 计算出加速段的速度和步数 */
//            {
////              log_error("error: Speed Calculate fail !");
//                return;
//            }
//            motorSetSpeedList(index);
//            break;
//        
//        case STATE_ACTIVE:
//            speed_index = getDMACounter(index);
//						__HAL_TIM_SET_AUTORELOAD(motor_params[index].timer,
//																motor[index].s_calc.toggle_pulse[speed_index]);
//						motor_params[index].timer->Instance->CNT = 0;
//						motor[index].state = STATE_AVESPEED;

//            motor[index].speed = TIMER_TO_x01RPM(motor[index].s_calc.toggle_pulse[speed_index]);
//			/***debug***/
////			HAL_GPIO_TogglePin(motor_params[index].enable_port, motor_params[index].enable_pin);
//			/***debug***/
//            if(sMoveSpeedCalculate(index, motor[index].speed, tar_speed) == FALSE) /* 计算出加速段的速度和步数 */
//            {
////              log_error("error: Speed Calculate fail !");
//                return;
//            }
//            motorSetSpeedList(index);
//            break;

//        case STATE_IDLE:
////          motorEnable(index, FALSE);
//            break;
//		default:
//			break;
//    }
//}

//uint16_t getDMACounter(motorIndex_e index)
//{
//    uint16_t speed_index = 0;
//    
//    __HAL_DMA_DISABLE(motor_params[index].timer->hdma[TIM_DMA_ID_UPDATE]);
//    speed_index = motor[index].s_calc.accel_step - 
//				(__HAL_DMA_GET_COUNTER(motor_params[index].timer->hdma[TIM_DMA_ID_UPDATE])) - 1;
//    HAL_TIM_Base_Stop_DMA(motor_params[index].timer);
//    return speed_index;
//}

//void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
//{
////	log_debug("DMA error!");
////	HAL_GPIO_WritePin(motor_params[M1].enable_port, motor_params[M1].enable_pin, GPIO_PIN_RESET);
////	log_debug("ARR:%d", __HAL_TIM_GET_AUTORELOAD(motor_params[M1].timer));
////	log_debug("CCR:%d", __HAL_TIM_GET_COMPARE(motor_params[M1].timer, motor_params[M1].channel));
////	log_debug("TIM_STATE:%d", motor_params[M1].timer->State);		
//}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim == motor_params[M1].timer)
//	{		
//		/***debug***/
////		HAL_GPIO_TogglePin(motor_params[M1].enable_port, motor_params[M1].enable_pin);
//		/***debug***/

//		motor[M1].state = STATE_AVESPEED;
//		memset(motor[M1].s_calc.toggle_pulse, 0, sizeof(motor[M1].s_calc.toggle_pulse));
//		motor[M1].speed = motor[M1].target_speed.result;
//	}
//   if(htim == motor_params[M2].timer)
//   {
//		/***debug***/
////		HAL_GPIO_TogglePin(motor_params[M2].enable_port, motor_params[M2].enable_pin);
//		/***debug***/

//		motor[M2].state = STATE_AVESPEED;
//		memset(motor[M2].s_calc.toggle_pulse, 0, sizeof(motor[M2].s_calc.toggle_pulse));
//		motor[M2].speed = motor[M2].target_speed.result;
//   }
//    if(htim == motor_params[M3].timer)
//	{
//     	/***debug***/
////		HAL_GPIO_TogglePin(motor_params[M3].enable_port, motor_params[M3].enable_pin);
//		/***debug***/

//		motor[M3].state = STATE_AVESPEED;
//		memset(motor[M3].s_calc.toggle_pulse, 0, sizeof(motor[M3].s_calc.toggle_pulse));
//		motor[M3].speed = motor[M3].target_speed.result;   
//	}
//    if(htim == motor_params[M4].timer)
//	{
//		/***debug***/
////		HAL_GPIO_TogglePin(motor_params[M4].enable_port, motor_params[M4].enable_pin);
//		/***debug***/

//		motor[M4].state = STATE_AVESPEED;
//		memset(motor[M4].s_calc.toggle_pulse, 0, sizeof(motor[M4].s_calc.toggle_pulse));
//		motor[M4].speed = motor[M4].target_speed.result; 
//	}
//	if(htim == &htim6)
//	{
//		/* 心跳灯闪烁 */
////		HAL_IWDG_Refresh(&hiwdg);
//		g_StatePeriod ++;
//		
//		if(g_StatePeriod > 1500)
//		{
//			g_StatePeriod = 0;
//			HAL_GPIO_WritePin(STATE_GPIO_Port, STATE_Pin, GPIO_PIN_SET);
//			g_StateLightFlag = 1;
//		}
////		HAL_GPIO_WritePin(STATE_GPIO_Port, STATE_Pin, GPIO_PIN_SET);

//		if(g_StateLightFlag == 1)
//		{
//			g_StateTime ++;
//			if(g_StateTime > 50)
//			{
//				g_StateTime = 0;
//				HAL_GPIO_WritePin(STATE_GPIO_Port, STATE_Pin, GPIO_PIN_RESET);
//				g_StateLightFlag = 0;
//			}
//		}
//		
//		/*通信断开连接判断*/
//		g_CommunicationPeriod ++;
//		if(g_CommunicationPeriod >= 100)
//		{
//			g_Communication ++;
//			if(g_Communication >= 20)
//			{
//				for(motorIndex_e index = M1; index < Mmax; index ++)
//				{
//					motor[index].target_speed.result = 0;
//				}
//				g_MotorCtrlFlag = 1;
//			}
//			if(g_Communication >= 200)
//				g_Communication = 20;
//			g_CommunicationPeriod = 0;
//		}
//		if(rs485_uart.receiveFlag == 1)
//		{
////			if(rs485_uart.receiveCounter == 14 || rs485_uart.receiveCounter == 10)
////				rs485_uart.receiveCounter --;
//			dataTranspond(&rs485_uart, &computer_uart, rs485_uart.receiveCounter);
////			HAL_GPIO_TogglePin(STATE_GPIO_Port, STATE_Pin);
//			rs485_uart.receiveFlag = 0;
//			
//			HAL_UART_Receive_DMA(rs485_uart.huart, rs485_uart.receiveTemporaryBuff, UART_RX_BUF_SIZE);
////			HAL_UARTEx_ReceiveToIdle_DMA(rs485_uart.huart, rs485_uart.receiveTemporaryBuff, UART_RX_BUF_SIZE);
//		}
//		if(computer_uart.receiveFlag == 1)
//		{
//			switch(dataReserveDecoder(computer_uart.receive_buff, computer_uart.receiveCounter))
//			{
//				case FORWARD_MESSAGE:
//					dataTranspond(&computer_uart, &rs485_uart, computer_uart.receiveCounter);
//					break;
//				case DEVICE_POWER_MSSAGE:
//					dataTranspond(&computer_uart, &computer_uart, computer_uart.receiveCounter);
////					HAL_GPIO_TogglePin(STATE_GPIO_Port, STATE_Pin);
//					break;
//				case MOTOR_CRTL_MESSAGE:
//					dataTransmit(&computer_uart, computer_uart.receive_buff, 8);
////					HAL_GPIO_TogglePin(STATE_GPIO_Port, STATE_Pin);
//					g_MotorCtrlFlag = 1;
//					break;
//				case ERROR_MESSAGE:
//					break;
//				default:
//					break;
//			}
//			computer_uart.receiveFlag = 0;
//			
//			HAL_UART_Receive_DMA(computer_uart.huart, computer_uart.receiveTemporaryBuff, UART_RX_BUF_SIZE);
////			HAL_UARTEx_ReceiveToIdle_DMA(computer_uart.huart, computer_uart.receiveTemporaryBuff, UART_RX_BUF_SIZE);
//		}
//	}
//}
