#include "ifr_usart.h"

// 静态函数声明
static void Start_DMA_Receive(ifr_usart_typedef *ifr_usart);
static void Uart_DoubleBuffer_Recevice(ifr_usart_typedef *ifr_usart, uint16_t len);
static uint8_t IFR_Uart_ID_Get(UART_HandleTypeDef *huart);
static void IFR_Usart_Restart(ifr_usart_typedef *ifr_usart);

// 回调函数声明
void IFR_UART_Rx_Callback(UART_HandleTypeDef *huart, uint16_t len);

// 串口对象指针数组，用于根据串口句柄查找对应的对象
ifr_usart_typedef* USART_Pointers[11] = {NULL};

/**
  * @brief   串口初始化
  * @param   ifr_usart: 串口对象指针
  * @param   huart: 串口句柄
  * @param   UART_Analysis_Function: 数据解析函数指针
  * @retval  void
  */
void IFR_USART_Init(ifr_usart_typedef *ifr_usart, UART_HandleTypeDef *huart, void(*UART_Analysis_Function)(uint8_t *pData, uint8_t len)) 
{
    ifr_usart->_huart = huart;
    uint8_t uart_id = IFR_Uart_ID_Get(huart);
    USART_Pointers[uart_id] = ifr_usart;
    
    if (UART_Analysis_Function != NULL)
        ifr_usart->AnalysisFunc = UART_Analysis_Function;

    // 注册接收回调函数
    HAL_UART_RegisterRxEventCallback(huart, IFR_UART_Rx_Callback);
    
    // 启动DMA接收
    Start_DMA_Receive(ifr_usart);
}

/**
  * @brief   接收数据处理
  * @param   ifr_usart: 串口对象指针
  * @param   len: 接收数据长度
  * @retval  void
  */
static void Uart_DoubleBuffer_Recevice(ifr_usart_typedef *ifr_usart, uint16_t len) 
{
    // 切换缓冲区并保存数据长度
    ifr_usart->Buffer_Num = !ifr_usart->Buffer_Num;
    ifr_usart->Data_Length = len;
    
    // 如果设置了解析函数，则调用
    if (ifr_usart->AnalysisFunc != NULL) 
        ifr_usart->AnalysisFunc(ifr_usart->rx_dma_buffer_[!ifr_usart->Buffer_Num], len);
    
    // 重新启动DMA接收
    Start_DMA_Receive(ifr_usart);
    
    // 更新时间戳
    ifr_usart->_updata_systick = HAL_GetTick();
}

/**
  * @brief   启动DMA接收
  * @param   ifr_usart: 串口对象指针
  * @retval  void
  */
static void Start_DMA_Receive(ifr_usart_typedef *ifr_usart) 
{ 
    // 启动DMA空闲接收
    ifr_usart->UsartRxState = HAL_UARTEx_ReceiveToIdle_DMA(ifr_usart->_huart, ifr_usart->rx_dma_buffer_[ifr_usart->Buffer_Num], USART_RX_RING_BUFFER_SIZE);
    
    // 禁用DMA半传输中断
    __HAL_DMA_DISABLE_IT(ifr_usart->_huart->hdmarx, DMA_IT_HT);
}

/**
  * @brief   串口接收回调函数
  * @param   huart: 串口句柄
  * @param   len: 接收数据长度
  * @retval  void
  */
void IFR_UART_Rx_Callback(UART_HandleTypeDef *huart, uint16_t len) 
{
    uint8_t uart_id = IFR_Uart_ID_Get(huart);
    if (USART_Pointers[uart_id] != NULL)
        Uart_DoubleBuffer_Recevice(USART_Pointers[uart_id], len);
}

/**
  * @brief   HAL库错误回调函数
  * @param   huart: 串口句柄
  * @retval  void
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) 
{
    uint8_t uart_id = IFR_Uart_ID_Get(huart);
    if (USART_Pointers[uart_id] != NULL) 
    {
        IFR_Usart_Restart(USART_Pointers[uart_id]);
    }
}

/**
  * @brief   串口错误恢复，重新初始化串口
  * @param   ifr_usart: 串口对象指针
  * @retval  void
  */
static void IFR_Usart_Restart(ifr_usart_typedef *ifr_usart) 
{
    if (ifr_usart->_huart == NULL) return;
    
    // 清除错误标志
    ifr_usart->_huart->ErrorCode = HAL_UART_ERROR_NONE;
    __HAL_UNLOCK(ifr_usart->_huart);
    
    // 重新初始化硬件资源
    HAL_UART_MspDeInit(ifr_usart->_huart);
    HAL_UART_MspInit(ifr_usart->_huart);
    
    // 重新注册回调函数并启动接收
    HAL_UART_RegisterRxEventCallback(ifr_usart->_huart, IFR_UART_Rx_Callback);
    Start_DMA_Receive(ifr_usart);
}

/**
  * @brief   获取串口ID
  * @param   huart: 串口句柄
  * @retval  串口ID (0-10，0表示无效串口)
  */
uint8_t IFR_Uart_ID_Get(UART_HandleTypeDef *huart) 
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