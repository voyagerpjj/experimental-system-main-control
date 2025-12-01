#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "main.h"
#include "tim.h"
#include "math.h"
#include "string.h"
#include "iwdg.h"
#include "communication.h"
#include "timer.h"
#include <stdlib.h>
#define SPEED_MAX_PLUSE 3500
typedef enum
{
    MOTOR_1 = 0,
    MOTOR_2 = 1,
    MOTOR_3 = 2,
    MOTOR_4 = 3,
	MOTOR_COUNT,
} motorindex_enum;	//	������

typedef enum
{
	MOTOR_IDLE = 0,         /* ���ʧ�ܣ����У�״̬*/
	MOTOR_AVESPEED,    		/* �������״̬ */
	MOTOR_ACTIVE,           /* ��������˶�״̬ */
} motor_state_typedef;

typedef enum
{
	MOTOR_STOP = 0,				// ���ֹͣ״̬	
	MOTOR_LOW_SPEED_STATE,		// ������ٽ׶�
	MOTOR_HIGH_SPEED_STATE,		// ������ٽ׶�
	MOTOR_HIGH_MINOR_ADJUST,   	// ������ٽ׶�΢��
} motor_speed_state_typedef;

typedef union{
	uint8_t data[2];
	uint16_t target_speed;
} motor_target_speed;

typedef struct 
{
	uint16_t psc;             				/*��ʱ��Ԥ��Ƶֵ*/
  	uint16_t accel_pulse;                	/*���ٶεĲ��� ��λpulse*/
 	uint16_t toggle_pulse[SPEED_MAX_PLUSE];  /*�������ÿ��CCR�Ĵ���ֵ*/
} speed_calc_typedef;

typedef struct 
{
	TIM_HandleTypeDef* timer;	// ��ʱ�����
	uint32_t channel;
	GPIO_TypeDef* enable_port;
	uint32_t enable_pin;
} motor_params_typedef;       /*�����ʼ�������ṹ��*/

#define ACCEL 1.0f
#define DECEL -1.0f

// DMA���
typedef enum {
    DMA_MODE_NORMAL = 0,	// DMA����ģʽ��������ɺ�ֹͣ
    DMA_MODE_CIRCULAR = 1	// DMAѭ��ģʽ��������ɺ��ظ���������
} dma_mode_enum;

typedef enum 
{
    DMA_STATE_READY = 0,	// DMA����״̬
    DMA_STATE_BUSY,			// DMAæ״̬
    DMA_STATE_COMPLETE		// DMA�������
} dma_state_enum;

typedef struct
{
	dma_mode_enum dma_mode;
	dma_state_enum dma_state;
} dma_prames;

typedef struct
{
	motorindex_enum motorindex;						// ������
	motor_state_typedef state;						// ���״̬
	motor_state_typedef last_state;						// ���״̬
	motor_speed_state_typedef motor_speed_state;	// ����ٶ�״̬
	motor_speed_state_typedef last_motor_speed_state;	// ��һ�ε���ٶ�״̬
	float current_speed;             				// �����ǰת��	��λ��rpm
	float last_target_speed;						// ��һ�ε��Ŀ��ת�� ��λ��rpm
	motor_target_speed target_speed;  				// ���Ŀ��ת��������
	speed_calc_typedef speed_calc;					// ����˶�����ó�������
	float accel_decel;					// ������ٻ���ٱ�־
	float sum_time_all;
	motor_params_typedef motor_params;				// �������
	dma_prames dma_prame;							// DMA����	
} Motor;


// ��ʽ�Ƶ��� 1r = 360 / 1.8 * 32 = 6400 pulse ��> �������Ƶ�ʣ�72M / 6400 * 60 = 0.675M
// ���Jerk������ز���
#define JERK_MAX_SPEED RPM_TO_PULSE(MOTOR_MAX_SPEED_RPM)		// Jerk����ٶ� ��λ��pulse/s
#define JERK_MAX_TIM 0.1f						// Jerk���ʱ�䣨һ��ʱ����٣�һ����٣� ��λ��s
	// Jerk�Ӽ��ٶȣ���Ϊ�Ӽ��ٶκͼ����ٶΣ�Jerk = 2 * max_speed_error / max_half_time ^ 2  ��λ��pulse/s^3
#define JERK (float)(JERK_MAX_SPEED / ((JERK_MAX_TIM / 2.0f) * (JERK_MAX_TIM / 2.0f)))	

	// �ٶȲ�תʱ�� t = sqrt(2 * speed / Jerk) ��λ��s��ֻ��Ҫô���ڼӼ��ٽ׶Σ�Ҫô���ڼ����ٽ׶�
#define SPEED_TO_TIME(speer_error) (float)(sqrtf(fabs(speer_error) / (float)JERK))

	// ʱ��ת�ٶ� v = Jerk * time * time / 2 ��λ��pulse/s
#define TIME_TO_SPEED(time) (float)(JERK * time * time / 2)

	// ʱ���ת����������������ʼ�ٶȡ��Ӽ��ٶȡ�ʱ�䣩 s = start_speed * time + 1 / 6 * accel * time * time * time ��λ��pulse
#define TIME_TO_PULSE(start_speed, accel, time) (uint16_t)(time * start_speed + 1.0f / 6.0f * accel * time * time * time)

	// ��������תʱ�� t = pow(3 * pulse / Jerk) ��λ��s��ֻ��Ҫô���ڼӼ��ٽ׶Σ�Ҫô���ڼ����ٽ׶�
#define PULSE_TO_TIME(pulse_error) (float)(powf(6 * pulse_error / (float)JERK, 1.0f / 3.0f))	

	// �������ת�ٷֱ��� rpm * rpm * (psc + 1) / 0.675M ��ת�ٵ�λ��rpm��
#define SPEED_RESOLUTION(rpm, psc) ARR_TO_RPM(RPM_TO_ARR_FLOAT(rpm, psc), psc) - ARR_TO_RPM(RPM_TO_ARR_FLOAT(rpm, psc) - 1.0f, psc)

// ��������������Բ���
#define MOTOR_MAX_SPEED_RPM 600.0f	// ����������ת�� ��λ��RPM
#define MOTOR_STEP_ANGLE 1.8f		// ������������Ƕ� һ��1.8��
#define MOTOR_SUBDIVIDE	 32			// ���ϸ�ֶ� 32����һ��
#define R_TO_PULSE (float)(360.0f / MOTOR_STEP_ANGLE * MOTOR_SUBDIVIDE)	// һȦת����������� 6400pulse
#define _01RPM_TO_RPM(_01rpm) (float)((_01rpm) * 0.1f)	// 0.1rpmתrpm
#define RPM_TO_PULSE(rpm) (float)((rpm) * (float)R_TO_PULSE / 60.0f)		// RPMת������ ��rpmת��pulse/s
#define PULSE_TO_ARR(pulse, psc) (TIM_COUNT_FREQ(psc) / pulse - 1.0f)		// �������ٶ�ת��ʱ��ARRֵ ��pulse/sת��ARRֵ
#define RPM_TO_ARR_FLOAT(rpm, psc) (float)((TIM_COUNT_FREQ(psc) * 60.0f) / ( (rpm) * R_TO_PULSE ) - 1.0f)	// ��ʱ��ARRֵת�������ת�� ��rpmת��ARRֵ
#define RPM_TO_ARR(rpm, psc) (uint16_t)((TIM_COUNT_FREQ(psc) * 60.0f) / ( (float)(rpm) * R_TO_PULSE ) - 1.0f)	// ��ʱ��ARRֵת�������ת�� ��rpmת��ARRֵ
#define ARR_TO_RPM(arr, psc) (float)(TIM_COUNT_FREQ(psc) / R_TO_PULSE * 60.0f / ((uint16_t)arr + 1.0f))	// ��ʱ��ARRֵת�������ת�� ��ARRֵת��rpm ��λ��rpm	72M / (psc + 1) / 6400 * 60 / (arr + 1)
#define MOTOR_SPEED_MIN(psc) (TIM_COUNT_FREQ(psc) / (65536.0f))  				// ����ٶȣ�����Ƶ��/(���ARR)

// ��ʱ������
#define TIM_CLK_FREQ 72000000	// ��ʱ����Ƶ�� 72MHz
#define TIM_COUNT_FREQ(psc) (float)(TIM_CLK_FREQ / (psc + 1.0f))	// ��ʱ������Ƶ�� ��ʱ����Ƶ / psc + 1 ��λ��Hz
#define TIM_COUNT_MAX 65535		// ��ʱ���������ֵ 65535

// ���١����ٽ׶ζ�Ӧ�Ĳ������ٶȵ�λrpm��
#define TIM_LOW_SPEED_PSC 205			// 102
#define TIM_HIGH_SPEED_PSC 0			// ���ٽ׶ζ�ʱ��Ԥ��Ƶֵ��20.6-259.6��
#define LOW_HIGH_SWITCH_SPEED 18.1		// ����������л��ٶȣ��ֽ��ߣ� 20.6rpm 20.6
#define HIGH_ADJUST_SWITCH_SPEED 259.8	// ������΢���л��ٶȣ��ֽ��ߣ� 259.8rpm
// ����ϸ�ֲ���������ʱ��߷ֱ��ʣ�
#define SUBDIVIDE_RATIO 7				// 7��ϸ��
#define SUBDIVIDE_INTERVAL_US 1000		// ϸ����������1ms��
#define SUBDIVIDE_CYCLE_MS 7			// ϸ�����ڣ�7ms=7��1ms��

#define IFR_CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))


// ���е����ʼ��
void motor_all_init(void);

// ���ʹ��
void motor_enable(motorindex_enum motor_index);

// ���ʧ��
void motor_disable(motorindex_enum motor_index);

// �������
void motor_control(motorindex_enum motor_index, uint16_t target_speed);

// ���õ��Ŀ���ٶ�
void motor_set_speed(motorindex_enum motor_index, uint16_t speed);

// ��ȡ�����ǰ�ٶ�
float motor_get_speed(motorindex_enum motor_index);

static void motor_init(motorindex_enum motor_index, TIM_HandleTypeDef* timer, uint32_t channel,	GPIO_TypeDef* enable_port, uint32_t enable_pin);

static void motor_tim_config_set(motorindex_enum motor_index);

static void motor_set_state(motorindex_enum motor_index, uint16_t target_speed);

static void motor_jerk_control(motorindex_enum motor_index, uint16_t start_speed, uint16_t target_speed, float jerk);

static void motor_high_speed_minor_adjust(motorindex_enum motor_index, uint16_t target_speed);

static HAL_StatusTypeDef motor_dma_transmit(motorindex_enum motor_index, uint16_t *arr_values, 
                                          uint32_t arr_count, dma_mode_enum mode);

static void motor_dma_transfer_complete_callback(DMA_HandleTypeDef *hdma);

static void motor_dma_error_callback(DMA_HandleTypeDef *hdma);

void test_control_time(void);
#endif
