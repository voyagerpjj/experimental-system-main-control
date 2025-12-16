#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "main.h"
#include "tim.h"
#include "math.h"
#include "string.h"
#include "iwdg.h"
#include "manager_tim.h"
#include <stdlib.h>

#define SPEED_MAX_PLUSE 3500

// 电机索引枚举
typedef enum
{
    MOTOR_1 = 0,
    MOTOR_2 = 1,
    MOTOR_3 = 2,
    MOTOR_4 = 3,
    MOTOR_COUNT,
} motorindex_enum;

// 电机状态枚举
typedef enum
{
    MOTOR_IDLE = 0,         // 电机空闲（停止）状态
    MOTOR_AVESPEED,         // 电机匀速状态
    MOTOR_ACTIVE,           // 电机加减速运动状态
} motor_state_typedef;

// 电机速度状态枚举
typedef enum
{
    MOTOR_STOP = 0,             // 电机停止状态
    MOTOR_LOW_SPEED_STATE,      // 电机低速阶段
    MOTOR_HIGH_SPEED_STATE,     // 电机高速阶段
    MOTOR_HIGH_MINOR_ADJUST,    // 电机高速微调阶段
} motor_speed_state_typedef;

// 目标速度联合体
typedef union {
    uint8_t data[2];
    uint16_t target_speed;
} motor_target_speed;

// 速度计算结构体
typedef struct 
{
    uint16_t psc;                         // 定时器预分频值
    uint16_t accel_pulse;                 // 加速段的脉冲数（单位：pulse）
    uint16_t toggle_pulse[SPEED_MAX_PLUSE]; // 存储每个CCR的更新值
} speed_calc_typedef;

// 电机参数结构体
typedef struct 
{
    TIM_HandleTypeDef* timer;  // 定时器句柄
    uint32_t channel;          // 定时器通道
    GPIO_TypeDef* enable_port; // 使能引脚端口
    uint32_t enable_pin;       // 使能引脚
} motor_params_typedef;

#define ACCEL 1.0f
#define DECEL -1.0f

// DMA模式枚举
typedef enum {
    DMA_MODE_NORMAL = 0,   // DMA正常模式，完成后停止
    DMA_MODE_CIRCULAR = 1  // DMA循环模式，完成后重新开始
} dma_mode_enum;

// DMA状态枚举
typedef enum 
{
    DMA_STATE_READY = 0,   // DMA就绪状态
    DMA_STATE_BUSY,        // DMA忙状态
    DMA_STATE_COMPLETE     // DMA完成状态
} dma_state_enum;

// DMA参数结构体
typedef struct
{
    dma_mode_enum dma_mode;
    dma_state_enum dma_state;
} dma_prames;

// 电机结构体
typedef struct
{
    motorindex_enum motorindex;               // 电机索引
    motor_state_typedef state;                // 电机状态
    motor_state_typedef last_state;           // 上一次电机状态
    motor_speed_state_typedef motor_speed_state;      // 电机速度状态
    motor_speed_state_typedef last_motor_speed_state; // 上一次速度状态
    float current_speed;                      // 电机当前转速（单位：rpm）
    float last_target_speed;                  // 上一次目标转速（单位：rpm）
    motor_target_speed target_speed;          // 电机目标转速结构体
    speed_calc_typedef speed_calc;            // 速度计算相关参数
    float accel_decel;                        // 加速度/减速度标志
    float sum_time_all;                       // 总时间
    motor_params_typedef motor_params;        // 电机参数
    dma_prames dma_prame;                     // DMA参数
} Motor;

// 公式推导：1转 = 360 / 1.8 * 32 = 6400 pulse -> 步进电机频率：72M / 6400 * 60 = 0.675M

// Jerk控制相关参数
#define JERK_MAX_SPEED RPM_TO_PULSE(MOTOR_MAX_SPEED_RPM)  // Jerk最大速度（单位：pulse/s）
#define JERK_MAX_TIM 0.1f                                 // Jerk最大时间（一半时间加速，一半时间减速）（单位：s）
// Jerk加加速度：分为加速度阶段和减速度阶段，Jerk = 2 * max_speed_error / max_half_time^2（单位：pulse/s^3）
#define JERK (float)(JERK_MAX_SPEED / ((JERK_MAX_TIM / 2.0f) * (JERK_MAX_TIM / 2.0f)))    

// 速度差转时间：t = sqrt(2 * speed / Jerk)（单位：s），只需要在加速阶段或减速阶段
#define SPEED_TO_TIME(speed_error) (float)(sqrtf(fabs(speed_error) / (float)JERK))

// 时间转速度：v = Jerk * time * time / 2（单位：pulse/s）
#define TIME_TO_SPEED(time) (float)(JERK * time * time / 2)

// 时间转脉冲数（考虑起始速度、加速度、时间）：s = start_speed * time + 1/6 * accel * time^3（单位：pulse）
#define TIME_TO_PULSE(start_speed, accel, time) (uint16_t)(time * start_speed + 1.0f / 6.0f * accel * time * time * time)

// 脉冲差转时间：t = pow(3 * pulse / Jerk)（单位：s），只需要在加速阶段或减速阶段
#define PULSE_TO_TIME(pulse_error) (float)(powf(6 * pulse_error / (float)JERK, 1.0f / 3.0f))    

// 速度分辨率计算：rpm * rpm * (psc + 1) / 0.675M（转速单位：rpm）
#define SPEED_RESOLUTION(rpm, psc) ARR_TO_RPM(RPM_TO_ARR_FLOAT(rpm, psc), psc) - ARR_TO_RPM(RPM_TO_ARR_FLOAT(rpm, psc) - 1.0f, psc)

// 步进电机参数
#define MOTOR_MAX_SPEED_RPM 600.0f     // 步进电机最大转速（单位：RPM）
#define MOTOR_STEP_ANGLE 1.8f          // 步进电机步距角（1.8度）
#define MOTOR_SUBDIVIDE 32             // 细分倍数（32细分）
#define R_TO_PULSE (float)(360.0f / MOTOR_STEP_ANGLE * MOTOR_SUBDIVIDE) // 一圈转动的脉冲数（6400 pulse）
#define _01RPM_TO_RPM(_01rpm) (float)((_01rpm) * 0.1f) // 0.1rpm转rpm
#define RPM_TO_PULSE(rpm) (float)((rpm) * (float)R_TO_PULSE / 60.0f) // RPM转脉冲频率（rpm转pulse/s）
#define PULSE_TO_ARR(pulse, psc) (TIM_COUNT_FREQ(psc) / pulse - 1.0f) // 脉冲频率转定时器ARR值（pulse/s转ARR值）
#define RPM_TO_ARR_FLOAT(rpm, psc) (float)((TIM_COUNT_FREQ(psc) * 60.0f) / ( (rpm) * R_TO_PULSE ) - 1.0f) // 转速转ARR值（rpm转ARR值）
#define RPM_TO_ARR(rpm, psc) (uint16_t)((TIM_COUNT_FREQ(psc) * 60.0f) / ( (float)(rpm) * R_TO_PULSE ) - 1.0f) // 转速转ARR值（rpm转ARR值）
#define ARR_TO_RPM(arr, psc) (float)(TIM_COUNT_FREQ(psc) / R_TO_PULSE * 60.0f / ((uint16_t)arr + 1.0f)) // ARR值转转速（ARR值转rpm，单位：rpm）
#define MOTOR_SPEED_MIN(psc) (TIM_COUNT_FREQ(psc) / (65536.0f)) // 最小速度（计数器频率/(最大ARR)）

// 定时器相关
#define TIM_CLK_FREQ 72000000                          // 定时器时钟频率 72MHz
#define TIM_COUNT_FREQ(psc) (float)(TIM_CLK_FREQ / (psc + 1.0f)) // 定时器计数频率（定时器时钟 / (psc + 1)）（单位：Hz）
#define TIM_COUNT_MAX 65535                             // 定时器计数器最大值 65535

// 速度阶段对应的参数
#define TIM_LOW_SPEED_PSC 205                          // 低速阶段定时器预分频值
#define TIM_HIGH_SPEED_PSC 0                           // 高速阶段定时器预分频值
#define LOW_HIGH_SWITCH_SPEED 18.1                     // 低速到高速切换速度（分界线）
#define HIGH_ADJUST_SWITCH_SPEED 259.8                 // 高速到微调切换速度（分界线）

// 高速细分参数（提高速度分辨率）
#define SUBDIVIDE_RATIO 7                              // 7细分
#define SUBDIVIDE_CYCLE_MS 7                           // 细分周期（7ms = 7 * 1ms）

// float 类型（适配浮点型 ARR 计算、速度计算）
static inline float IFR_CLAMP(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

// 函数声明
void motor_all_init(void);                             // 所有电机初始化
void motor_start(motorindex_enum motor_index, uint16_t target_speed); // 电机控制
void motor_stop(motorindex_enum motor_index);          // 电机停止
float motor_get_speed(motorindex_enum motor_index);    // 获取电机当前速度

#endif
