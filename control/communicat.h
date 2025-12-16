#ifndef _COMMUNICAT_H_
#define _COMMUNICAT_H_

#include "main.h"
#include "crc.h"
#include "modbus.h"
#include "ifr_custom_usart_protocol.h"
/* 发送需求：
ttl转网口
1、四个变送器和四个流量传感器反馈数据

*/

/* 接收需求：

*/

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
  
typedef enum
{
	YES = 0,	// 需要读取 或 需要上电
	NO,				// 不需要读取 或 不需要上电
} read_data_enum;

typedef enum
{
	WAIT = 0,		// 正在等待
	UPDATA ,		// 已更新
	RX_ERROR , 	// 通信校验错误
} communicat_state_enum;
typedef __packed struct  
{
	uint16_t motor_tar[4];	
} motor_control_data_typedef;

typedef  __packed struct
{
	rx_modbus_message_enum rx_modbus_message;
} configuration_data_typedef;

typedef struct
{
	rx_custom_message_enum rx_message_state;				// 自定义串口协议帧标识
	motor_control_data_typedef motor_control_data;	// 自定义串口协议电机数据
	configuration_data_typedef configuration_data;	// 配置模式下modbus协议数据
	communicat_state_enum state;
} rx_ttl_message_typedef;

typedef struct
{
	uint8_t ID;																			// 流量采集卡ID
	read_data_enum liquid_level_transmitter_state;	// 是否需要读取
	HAL_StatusTypeDef state_normal;									// 是否正常
	uint32_t transmit_tick;													// 上一次发送的时刻
	uint32_t receive_tick;													// 上一次接收的时刻
	uint8_t receive_data[8];												// 接收的数据 8字节
} liquid_flow_collection_typedef;

typedef struct
{
	uint8_t ID;																			// 液位变送器ID
	read_data_enum liquid_level_transmitter_state;	// 是否需要读取
	read_data_enum liquid_level_transmitter_power;	// 是否要开电
	HAL_StatusTypeDef state_normal;									// 是否正常
	uint32_t transmit_tick;													// 上一次发送的时刻
	uint32_t receive_tick;													// 上一次接收的时刻
	uint8_t receive_data[4];												// 接收的数据 4字节
} liquid_level_transmitter_typedef;

typedef enum
{
	LIQUID_LEVEL_TRANSMITTER = 0,	// 液位变送器
	LIQUID_FLOW_COLLECTION ,			// 流量采集卡
} equipment_enum;

typedef struct
{
	equipment_enum equipment_id;	// 标识是哪种设备
	liquid_flow_collection_typedef liquid_flow_collection;
	liquid_level_transmitter_typedef liquid_level_transmitter;
}	equipment_typedef;
void communicat_init(void);
void communicat_start(void);

rx_ttl_message_typedef communicat_get_ttl_rx_message(void);

liquid_level_transmitter_typedef communicat_get_liquid_level_transmitter(int index);

liquid_flow_collection_typedef communicat_get_liquid_flow_collection(void);

void communicat_485_transmit_reset(void);

HAL_StatusTypeDef communicat_485_transmit_all(void);

HAL_StatusTypeDef communicat_ttl_transmit_normal(void);

HAL_StatusTypeDef communicat_ttl_transmit_error(void);
	
void reset_communica_ttl_rx_message_state(void);
#endif  // _COMMUNICAT_H_

