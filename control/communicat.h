#ifndef _COMMUNICAT_H_
#define _COMMUNICAT_H_

#include "main.h"
/* 发送需求：
ttl转网口
1、四个变送器和四个流量传感器反馈数据

*/

/* 接收需求：

*/

typedef enum
{
    DEVICE_POWER_MSG = 0,   /* 处理类消息 */
    MOTOR_CONTROL_MSG,
    FORWARD_MSG,    /* 转发类消息 */
    ERROR_MSG,
}rx_message_enum;

// typedef enum
// {
//     DEVICE_POWER = 0,   /* 处理类消息 */
//     MOTOR_CONTROL,
//     FORWARD,    /* 转发类消息 */
//     ERROR,
// }tx_messageType_enum;

#endif  // _COMMUNICAT_H_
