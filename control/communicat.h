#ifndef _COMMUNICAT_H_
#define _COMMUNICAT_H_

#include "main.h"
#include "ifr_crc.h"
/* 发送需求：
ttl转网口
1、四个变送器和四个流量传感器反馈数据

*/

/* 接收需求：

*/
#define usart_485 huart1
#define usart_ttl huart3
typedef enum
{
	 /* 处理类消息 */
	MOTOR_CONTROL_MSG = 0,  				// 步进电机控制指令
	CONFIGURATION_MSG,							// 配置模式指令
} rx_custom_message_enum;
typedef enum
{
	TRANSMITTER_CONFIGURATION_MSG,	// 变送器配置指令
	DEVICE_POWER_MSG,								// 电源控制指令
} rx_modbus_message_enum;
  
// typedef enum
// {
//     DEVICE_POWER = 0,   /* 处理类消息 */
//     MOTOR_CONTROL,
//     FORWARD,    /* 转发类消息 */
//     ERROR,
// }tx_messageType_enum;
typedef enum
{
	NO = 0,	// 不需要读取 或 不需要上电
	YES ,		// 需要读取 或 需要上电
} read_data_enum;

typedef __packed struct
{
	uint16_t motor_tar1;
	uint16_t motor_tar2;
	uint16_t motor_tar3;
	uint16_t motor_tar4;
	read_data_enum liquid_level_transmitter_1 : 1;	// 是否读取液位变送器1
	read_data_enum liquid_level_transmitter_2 : 1;	// 是否读取液位变送器2
	read_data_enum liquid_level_transmitter_3 : 1;	// 是否读取液位变送器3
	read_data_enum liquid_level_transmitter_4 : 1;	// 是否读取液位变送器4
	read_data_enum flow_collector : 1;							// 是否读取流量采集器
	uint8_t reserved : 3;														// 保留位
} motor_control_data_typedef;

typedef __packed struct
{
	rx_modbus_message_enum rx_modbus_message;
	__packed struct 
	{
		read_data_enum 	liquid_level_transmitter_1 : 1;		// 电源控制指令中 是否需要液位变送器1上电
		read_data_enum 	liquid_level_transmitter_2 : 1;		// 电源控制指令中 是否需要液位变送器2上电
		read_data_enum 	liquid_level_transmitter_3 : 1;		// 电源控制指令中 是否需要液位变送器3上电
		read_data_enum 	liquid_level_transmitter_4 : 1;		// 电源控制指令中 是否需要液位变送器4上电
		uint8_t reserved : 4;															// 保留位
	} device_power;
} configuration_data_typedef;

typedef struct
{
	rx_custom_message_enum rx_message_state;				// 自定义串口协议帧标识
	motor_control_data_typedef motor_control_data;	// 自定义串口协议电机数据
	configuration_data_typedef configuration_data;	// 配置模式下modbus协议数据
} rx_message_typedef;


#endif  // _COMMUNICAT_H_
