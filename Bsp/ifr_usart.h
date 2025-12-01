//#ifndef IFR_USART_H
//#define IFR_USART_H

//#include "main.h"
//#define UART_RX_BUFF_SIZE 120
//#define UART_TX_BUFF_SIZE 120
//#define UART_RX_BUFF 2
//#define UART_TX_BUFF 4
//#if USE_HAL_UART_REGISTER_CALLBACKS || USE_HAL_USART_REGISTER_CALLBACKS	// 需要定义两者之一才能重定义callback
//typedef struct
//{
//	uint8_t TransmitMemoryBuff[UART_RX_BUFF][UART_RX_BUFF_SIZE];
//	uint8_t transmit_memorybuff_front;
//	uint8_t transmit_memorybuff_rear;
//	uint8_t transmit_memorybuff_count;
//}TransmitMemory;

//typedef struct
//{
//	uint8_t ReceiveMemoryBuff[UART_RX_BUFF][UART_RX_BUFF_SIZE];
//	uint8_t receive_memorybuff_rear;
//	
//}ReceiveMemory;

//// 串口结构体
//typedef struct
//{
//	UART_HandleTypeDef* m_huart;
//	uint8_t transmit_buff[UART_TX_BUFF][UART_TX_BUFF_SIZE];
//  uint8_t receive_buff[UART_RX_BUFF_SIZE];
//	
//	uint8_t receiveFlag;
//	uint16_t receiveCounter;

//	uint32_t _updata_systick;  // 最后更新时间戳
//  void(*AnalysisFunc)(uint8_t *pData, uint8_t len);   // 解析函数指针
//}Usart_ClassDef;

//// 内部接口部分
//static void IFR_UART_Rx_Callback(UART_HandleTypeDef *huart, uint16_t len);
//static void Rx_Process(Usart_ClassDef* _usart, uint16_t len);
//static uint8_t IFR_Uart_ID_Get(UART_HandleTypeDef *huart);

////外部接口部分
//void Usart_init(Usart_ClassDef* _usart, UART_HandleTypeDef* huart, void(*UART_Analysis_Function)(uint8_t *pData, uint8_t len));

//#endif
//#endif
