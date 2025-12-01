/*
 * @FilePath     : motor
 * @Author       : dingyun
 * @Date         : 2022-10-11
 * @LastEditors  : starryding
 * @LastEditTime : 2023-04-20
 * @Description  : 
 * 
 * Copyright (c) 2022 by BZLZ , All Rights Reserved. 
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "main.h"
#include "tim.h"
#include "math.h"
#include "string.h"
#include "iwdg.h"
#include "communication.h"

#define TIMER_FREQ       (7200000)                    /* 时钟频率 */
#define TIMER_PRESCALER  (72000000 / TIMER_FREQ - 1)

#define RESOLUTION      (32)
#define STEP_ANGLE      (1.8f)

#define RPM_PULSE                   (0.1f * 360 / (STEP_ANGLE / RESOLUTION) / 60)   /* 0.1rpm对应的脉冲数(步数) */
#define _x01RPM_TO_STEP_S(_x01rpm)  ((_x01rpm) * RPM_PULSE)         /* 将 0.1rpm 转速转化成 step/s */
#define STEP_S_TO_x01RPM(step_s)    (step_s / RPM_PULSE)            /* 将 step/s 转化为 0.1rpm */
#define STEP_S_TO_TIMER(step_s)			(TIMER_FREQ / (step_s) * 0.5)
#define TIMER_TO_STEP_S(pluse)			(TIMER_FREQ / (pluse) * 0.5)
#define TIMER_TO_x01RPM(pluse)			(STEP_S_TO_x01RPM(TIMER_TO_STEP_S(pluse)))

#define RPM_TO_TIMER(_x01rpm)     	(TIMER_FREQ / (_x01rpm * RPM_PULSE))

#define STEP_S_x600RPM   (64000)
#define _0_600RPM_S      (0.1f)
#define STEP_S_J         (float)(STEP_S_x600RPM / ((_0_600RPM_S / 2) * (_0_600RPM_S / 2)))
#define STEP_S_A(t)      (STEP_S_J * (t))
#define DELTA_STEP_S(t)  (0.5f * STEP_S_A(t) * (t))

#define HALF_TIME(vi, vo) (float)(sqrt(fabs((vo) - (vi)) / STEP_S_J))
#define INCACCELSTEP(t)   (STEP_S_J * pow((t) , 3) / 6.0f )

#define SPEED_MIN         (TIMER_FREQ / (65535.0f))           /* 最低频率/速度 */

#ifndef TRUE
#define TRUE		1
#endif

#ifndef FALSE
#define FALSE		0
#endif

#define IS_ACC  (int8_t)1
#define IS_DEC  (int8_t)-1

#define SPEED_INDEX 1	/// 3500 !!!!!!!!

typedef struct {
    uint16_t accel_step;                 /*变速段的步数 单位step*/
    uint16_t toggle_pulse[SPEED_INDEX];  /*电机调速每步CCR寄存器值*/
	uint16_t debug;
} speed_calc_t;

typedef union
{
    uint8_t data[2];
    uint16_t result;
    /* data */
}splice_u;

typedef struct
{
    enum
    {
        STATE_AVESPEED = 0,     /* 电机匀速状态 */
        STATE_ACTIVE,           /* 电机运动状态 */
        STATE_IDLE,             /* 电机失能（空闲）状态*/
    } state;
    uint16_t speed;             /*电机当前转速*/
    splice_u target_speed;      /*电机目标转速共用体*/

    speed_calc_t s_calc;        /*电机运动计算结构体*/
	
} motorState_t;

typedef struct 
{
    TIM_HandleTypeDef* timer;
    uint32_t channel;
    GPIO_TypeDef* enable_port;
    uint32_t enable_pin;
} motorParams_t;                /*电机初始化参数结构体*/

typedef enum
{
    M1 = 0,
    M2,
    M3,
    M4,
    Mmax,
} motorIndex_e;

void motorInit(void);
void motorSetSpeedList(motorIndex_e index);
void motorEnable(motorIndex_e index, bool enabled);
void motor_set_test_speed(motorIndex_e index, int tar_speed);

uint8_t sMoveSpeedCalculate(motorIndex_e index, uint16_t vi,  uint16_t vo);
void sMoveSpeedStart(motorIndex_e index, uint16_t tar_speed); //_speed * 0.1rpm
uint16_t getDMACounter(motorIndex_e index);

#endif
