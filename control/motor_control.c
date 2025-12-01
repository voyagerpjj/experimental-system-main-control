#include "motor_control.h"
Motor Motor_list[MOTOR_COUNT];
float now_speed = 0;
// 所有电机初始化
void motor_all_init()
{
	motor_init(MOTOR_1, &htim2, TIM_CHANNEL_2, M1_EN_GPIO_Port, M1_EN_Pin);
	motor_init(MOTOR_2, &htim3, TIM_CHANNEL_3, M2_EN_GPIO_Port, M2_EN_Pin);
	motor_init(MOTOR_3, &htim4, TIM_CHANNEL_1, M3_EN_GPIO_Port, M3_EN_Pin);
	motor_init(MOTOR_4, &htim5, TIM_CHANNEL_2, M4_EN_GPIO_Port, M4_EN_Pin);
}
uint16_t target_arr = 65535;
uint8_t start = 1;
// 单个电机对应的控制
void motor_control(motorindex_enum motor_index, uint16_t target_speed)
{

	target_speed = IFR_CLAMP(target_speed, 0, 6000);	// 将电机转速限制在0-600rpm之间
	motor_enable(motor_index);	// 使能电机
	if (start == 1)
	{
		start = 1;
		uint32_t psc = Motor_list[motor_index].speed_calc.psc;
		// 计算目标ARR（50rpm对应13499）

		// 存储单个目标ARR，循环DMA传输
		Motor_list[motor_index].speed_calc.toggle_pulse[0] = target_arr;
		Motor_list[motor_index].speed_calc.accel_pulse = 1;
		// 启动循环DMA，维持固定脉冲频率
		motor_dma_transmit(motor_index, Motor_list[motor_index].speed_calc.toggle_pulse, 1, DMA_MODE_NORMAL);
	}
//	motor_set_state(motor_index, target_speed); 			// 设置电机状态   
//	motor_set_speed(motor_index, target_speed); 			// 根据电机状态设置电机速度
	// test_control_time();
}

// 电机使能
void motor_enable(motorindex_enum motor_index)
{
	if(motor_index >= MOTOR_COUNT)
		return;
	HAL_GPIO_WritePin(Motor_list[motor_index].motor_params.enable_port, Motor_list[motor_index].motor_params.enable_pin, GPIO_PIN_SET);
}

// 电机失能
void motor_disable(motorindex_enum motor_index)
{
	// 检查电机索引是否有效，如果超出范围则直接返回
	if(motor_index >= MOTOR_COUNT)
		return;
	// 将对应电机的使能引脚设置为低电平，禁用电机
	HAL_GPIO_WritePin(Motor_list[motor_index].motor_params.enable_port, Motor_list[motor_index].motor_params.enable_pin, GPIO_PIN_RESET);
	Motor_list[motor_index].current_speed = 0;
}

int ARR_2 = 0;
float resolution = 0;
// 电机状态机设置状态（根据当前速度和目标速度）
static void motor_set_state(motorindex_enum motor_index, uint16_t target_speed)
{
	Motor *this_motor = &Motor_list[motor_index];	//	对临时变量赋值，缩短代码长度
	float tar_speed = _01RPM_TO_RPM(target_speed);
	now_speed = ARR_TO_RPM(this_motor->motor_params.timer->Instance->ARR, this_motor->motor_params.timer->Instance->PSC);
	ARR_2 = this_motor->motor_params.timer->Instance->ARR;
	resolution = SPEED_RESOLUTION(tar_speed, this_motor->speed_calc.psc);
	// 判断电机处于停止、匀速、变速状态
	if (this_motor->current_speed + tar_speed < 0.01f)
	    this_motor->state = MOTOR_IDLE;
	
	else if (fabs(this_motor->current_speed - tar_speed) <= 0.54f && this_motor->motor_speed_state == MOTOR_HIGH_MINOR_ADJUST)	// 若电机当前速度等于目标速度（达到当前分辨率上限） SPEED_RESOLUTION(tar_speed, this_motor->speed_calc.psc) * 2.0f
	    this_motor->state = MOTOR_AVESPEED;

	else if (fabs(this_motor->current_speed - tar_speed) >= 0.1f)	// 若电机当前速度不等于目标速度啊（误差0.1）
	    this_motor->state = MOTOR_ACTIVE;

	// 判断电机速度范围
	if (tar_speed == 0 || this_motor->state == MOTOR_IDLE)
		this_motor->motor_speed_state = MOTOR_STOP;	// 停止状态

    else if (0 < tar_speed && tar_speed < LOW_HIGH_SWITCH_SPEED)
		this_motor->motor_speed_state = MOTOR_LOW_SPEED_STATE;	// 低速状态

	else if (LOW_HIGH_SWITCH_SPEED <= tar_speed && tar_speed < HIGH_ADJUST_SWITCH_SPEED)
		this_motor->motor_speed_state = MOTOR_HIGH_SPEED_STATE;	// 高速状态

	else if (HIGH_ADJUST_SWITCH_SPEED <= tar_speed && tar_speed <= MOTOR_MAX_SPEED_RPM)
		this_motor->motor_speed_state = MOTOR_HIGH_MINOR_ADJUST;	// 高速高精度调整状态
	
}

// 电机设置速度
void motor_set_speed(motorindex_enum motor_index, uint16_t speed)
{
	if (motor_index >= MOTOR_COUNT)
		return;
	
	Motor *motor = &Motor_list[motor_index];	//	对临时变量赋值，缩短代码长度
	float target_rpm = _01RPM_TO_RPM(speed);
	if (motor->last_motor_speed_state != motor->motor_speed_state)	// 若电机速度范围发生变化
		motor_tim_config_set(motor_index);
		
	motor->last_motor_speed_state = motor->motor_speed_state;
	
	if (motor->motor_speed_state == MOTOR_STOP)
	{
		motor_disable(motor_index);	// 失能电机
		motor->last_target_speed = 0;
		return;
	}
	else 
		motor_enable(motor_index);	// 使能电机

	if (motor->last_target_speed != target_rpm)	//	若电机目标速度发生变化，处于变速状态 this_motor->state == MOTOR_ACTIVE && 
	{
		motor_jerk_control(motor_index, motor->current_speed * 10.0f, speed, JERK);	// 计算电机每一步参数，并完成DMA发送			
		motor->last_target_speed = target_rpm;
	}

	else if (motor->state == MOTOR_AVESPEED)	// 若电机当前速度等于目标速度
	{

		if (motor->motor_speed_state == MOTOR_HIGH_MINOR_ADJUST && motor->last_state == MOTOR_ACTIVE)	// 若电机处于高转速高精度调整状态并且上一刻为变速模式
			motor_high_speed_minor_adjust(motor_index, speed); // 启动软件7倍细分算法
//		else 
//    {
//        uint32_t psc = motor->speed_calc.psc;
//        // 计算目标ARR（50rpm对应13499）
//        uint16_t target_arr = (uint16_t)IFR_CLAMP(RPM_TO_ARR_FLOAT(target_rpm, psc), 1, TIM_COUNT_MAX);
//        // 存储单个目标ARR，循环DMA传输
//        motor->speed_calc.toggle_pulse[0] = target_arr;
//        motor->speed_calc.accel_pulse = 1;
//        // 启动循环DMA，维持固定脉冲频率
//        motor_dma_transmit(motor_index, motor->speed_calc.toggle_pulse, 1, DMA_MODE_CIRCULAR);
//    }
	}
	if (motor->state == MOTOR_ACTIVE)
	{
		
	}
	motor->last_state = motor->state;
}

// 单个电机JERK控制降速到0
void motor_stop(motorindex_enum motor_index)
{
	motor_jerk_control(motor_index, Motor_list[motor_index].current_speed, 0, JERK);
}

// 获取电机当前速度
float motor_get_speed(motorindex_enum motor_index)
{
	return ARR_TO_RPM(Motor_list[motor_index].motor_params.timer->Instance->ARR, 
		Motor_list[motor_index].motor_params.timer->Instance->PSC);
};

// 单个电机初始化
static void motor_init(motorindex_enum motor_index, TIM_HandleTypeDef* timer, uint32_t channel,	GPIO_TypeDef* enable_port, uint32_t enable_pin)
{
	Motor_list[motor_index].motorindex = motor_index;
	Motor_list[motor_index].state = MOTOR_IDLE;
	Motor_list[motor_index].current_speed = 0;
	memset(Motor_list[motor_index].speed_calc.toggle_pulse, 0, sizeof(Motor_list[motor_index].speed_calc.toggle_pulse));
	Motor_list[motor_index].target_speed.target_speed = 0;
	Motor_list[motor_index].motor_params.timer = timer;
	Motor_list[motor_index].motor_params.channel = channel;
	Motor_list[motor_index].motor_params.enable_port = enable_port;
	Motor_list[motor_index].motor_params.enable_pin = enable_pin;
	Motor_list[motor_index].last_target_speed = 0;
	Motor_list[motor_index].last_motor_speed_state = MOTOR_STOP;
	motor_tim_config_set(motor_index);
}

float MIN_SPEED = 0;
int num_3 = 0;

// 电机Jerk控制（S型曲线：时间迭代+按脉冲存储ARR）
static void motor_jerk_control(motorindex_enum motor_index, uint16_t start_speed, uint16_t target_speed, float jerk)
{
	num_3++;
	float start_rpm = _01RPM_TO_RPM(start_speed);
	float target_rpm = _01RPM_TO_RPM(target_speed);
	Motor* motor = &Motor_list[motor_index];
	motor->target_speed.target_speed = target_speed;
	uint32_t psc = motor->speed_calc.psc;						// 获取当前电机设置的psc

	// 最小速度限制（pulse/s）
	MIN_SPEED = MOTOR_SPEED_MIN(psc);

	// 转换为脉冲频率（pulse/s）
	float start_pulse = RPM_TO_PULSE(start_rpm);
	float target_pulse = RPM_TO_PULSE(target_rpm);
	float speed_error = target_pulse - start_pulse;

	// 终止条件：速度差过小或目标速度为0
	if (target_rpm == 0.0f) 
	{
		motor_disable(motor_index);	// 失能电机
		return;
	}

	// 加速方向（1:加速，-1:减速）
	int8_t accel_direction = (speed_error > 0) ? 1 : -1;
	float JERK_effective = accel_direction * jerk;

	// 计算加加速/减加速阶段时间（T）和总时间
	float T = SPEED_TO_TIME(speed_error);  // 单阶段时间
	float total_time = 2 * T;                  // 总加减速时间

	// 计算加加速阶段终点速度v_mid（不低于最小速度）
	float v_mid = start_pulse + JERK_effective * 0.5f * (T * T);
	if (v_mid < MIN_SPEED)
			v_mid = MIN_SPEED;

	// 初始化变量
	float current_speed = start_pulse;  // 当前速度（pulse/s）
	float sum_time = 0.0f;              // 累计时间
	uint16_t pulse_count = 0;           // 脉冲计数
	memset(motor->speed_calc.toggle_pulse, 0, sizeof(motor->speed_calc.toggle_pulse));

	// 第一步：若当前速度低于最小速度，先加速到最小速度
	if (current_speed < MIN_SPEED) 
	{
		float t_start = sqrt(2 * (MIN_SPEED - current_speed) / jerk);
		t_start = (t_start < 1e-6f) ? 1e-6f : t_start;  // 避免时间为0

		current_speed = MIN_SPEED;
		uint16_t start_arr = (uint16_t)IFR_CLAMP(PULSE_TO_ARR(current_speed, psc), 1, TIM_COUNT_MAX);
		motor->speed_calc.toggle_pulse[pulse_count++] = start_arr;
		sum_time = t_start;
	}

	// 核心循环：S型曲线迭代计算
	while (pulse_count < SPEED_MAX_PLUSE && sum_time < total_time) 
	{
		// 计算当前步时间间隔（1/当前速度）
		float step_time = 1.0f / current_speed;
		sum_time += step_time;

		// 分阶段计算当前速度
		if (sum_time < T) 
			// 加加速阶段：v = v0 + 0.5 * J_eff * t2
			current_speed = start_pulse + JERK_effective * 0.5f * (sum_time * sum_time);
		else 
			// 减加速阶段：v = -0.5*J_eff*t2 + 2*J_eff*T*t - J_eff*T2 + v0
			current_speed = -0.5f * JERK_effective * sum_time * sum_time + 2 * JERK_effective * T * sum_time 
										- JERK_effective * T * T + start_pulse;

		// 限制最小速度
		if (current_speed < MIN_SPEED) 
				current_speed = MIN_SPEED;

		// 计算当前ARR并存储
		uint16_t arr_value = (uint16_t)IFR_CLAMP(PULSE_TO_ARR(current_speed, psc), 1, TIM_COUNT_MAX);
		motor->speed_calc.toggle_pulse[pulse_count++] = arr_value;

		// 终止条件：速度接近目标（误差<0.1rpm）且已过加加速阶段
		float current_rpm = ARR_TO_RPM(arr_value, psc);
		if (fabs(current_rpm - target_rpm) < 0.1f && sum_time >= T)
		{
				// 补充最后一步到目标速度
			target_arr = roundf(IFR_CLAMP(PULSE_TO_ARR(target_pulse, psc), 1, TIM_COUNT_MAX));
			motor->speed_calc.toggle_pulse[pulse_count++] = target_arr + 13;
			break;
		}
	}
	if (sum_time >= 2.0f * T) 
	{
		// 补充最后一步到目标速度
		target_arr = roundf(IFR_CLAMP(PULSE_TO_ARR(target_pulse, psc), 1, TIM_COUNT_MAX));
		motor->speed_calc.toggle_pulse[pulse_count++] = target_arr + 13;
	}
	// 更新电机状态并启动DMA
	motor->sum_time_all = sum_time;
	motor->speed_calc.accel_pulse = pulse_count;
	motor_dma_transmit(motor_index, motor->speed_calc.toggle_pulse, pulse_count, DMA_MODE_NORMAL);
}

int ARR_1 = 0;
float high_speed_minor_adjust[SUBDIVIDE_RATIO] = {0};
int32_t actual_arr[SUBDIVIDE_RATIO] = {0};
// 电机高转速时软件细分
static void motor_high_speed_minor_adjust(motorindex_enum motor_index, uint16_t target_speed)
{
		Motor* motor = &Motor_list[motor_index];
	
		motor->target_speed.target_speed = target_speed;
    float target_rpm = _01RPM_TO_RPM(target_speed);  // 转换为实际RPM（0.1rpm精度输入）
    uint32_t psc = motor->speed_calc.psc;
		
		memset(motor->speed_calc.toggle_pulse, 0, sizeof(motor->speed_calc.toggle_pulse));	// 清空存储ARR的数组
	
    // 1. 计算目标ARR及误差
    float target_arr = RPM_TO_ARR_FLOAT(target_rpm, psc);  // 理论ARR（浮点）
    int32_t integer_arr = (int32_t)round(target_arr);    // 取整后的ARR
    float arr_error = target_arr - integer_arr;           // ARR误差

    // 2. 分配误差到7个间隔
    int32_t total_adjust = (int32_t)round(arr_error * SUBDIVIDE_RATIO);
    total_adjust = fmax(-(int32_t)SUBDIVIDE_RATIO, fmin((int32_t)SUBDIVIDE_RATIO, total_adjust));


    for (uint8_t i = 0; i < SUBDIVIDE_RATIO; i++)
        actual_arr[i] = integer_arr;  // 初始化为整数ARR

    // 按总调整量分配±1
    uint8_t index = 0;
    while (index < abs(total_adjust)) 
		{
			if (total_adjust > 0) 
					actual_arr[index] += 1;
			else
					actual_arr[index] -= 1;
			index++;
    }

    // 3. 限制ARR范围并存储
    for (uint8_t i = 0; i < SUBDIVIDE_RATIO; i++) 
		{
			// ARR限制在100~TIM_COUNT_MAX（避免过低导致频率过高）
			actual_arr[i] = IFR_CLAMP(actual_arr[i], 100, TIM_COUNT_MAX);
			motor->speed_calc.toggle_pulse[i] = (uint16_t)actual_arr[i];
    }

    // 4. 计算实际平均速度并更新
		float actual_avg_rpm = 0.0f;
    for (uint8_t i = 0; i < SUBDIVIDE_RATIO; i++) 
		{
			high_speed_minor_adjust[i] = ARR_TO_RPM(motor->speed_calc.toggle_pulse[i], psc);
      actual_avg_rpm += ARR_TO_RPM(motor->speed_calc.toggle_pulse[i], psc);
		}

    actual_avg_rpm /= SUBDIVIDE_RATIO;
    motor->current_speed = actual_avg_rpm;
		motor->speed_calc.accel_pulse = SUBDIVIDE_RATIO;
		ARR_1 = motor->motor_params.timer->Instance->ARR;
    // 5. 启动循环DMA传输
    motor_dma_transmit(motor_index, motor->speed_calc.toggle_pulse, motor->speed_calc.accel_pulse, DMA_MODE_CIRCULAR);

}

// 单个电机对应的定时器参数初始化
static void motor_tim_config_set(motorindex_enum motor_index)
{
	HAL_GPIO_TogglePin(STATE_GPIO_Port, STATE_Pin);
		Motor* motor = &Motor_list[motor_index]; // 简化指针操作，提高可读性
    TIM_HandleTypeDef* htim = motor->motor_params.timer;
    uint32_t channel = motor->motor_params.channel;

    // 1. 安全终止定时器与DMA（避免配置时波形异常）
		__HAL_TIM_DISABLE(htim); // 关闭定时器核心，确保PSC/ARR修改立即生效
    HAL_TIM_OC_Stop(htim, channel); // 停止OC输出
    if (htim->hdma[TIM_DMA_ID_UPDATE] != NULL) // 若存在DMA，先终止并禁用
    {
        HAL_DMA_Abort(htim->hdma[TIM_DMA_ID_UPDATE]);
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_UPDATE); // 禁用定时器DMA请求
    }

    // 2. 正确选择预分频（PSC）
    if (motor->motor_speed_state == MOTOR_LOW_SPEED_STATE)
    {
        motor->speed_calc.psc = TIM_LOW_SPEED_PSC; // 低速/停止：用大PSC
    }
    else if (motor->motor_speed_state == MOTOR_HIGH_SPEED_STATE || motor->motor_speed_state == MOTOR_HIGH_MINOR_ADJUST || motor->motor_speed_state == MOTOR_STOP)
    {
        motor->speed_calc.psc = TIM_HIGH_SPEED_PSC; // 高速/细分：用小PSC
    }

    // 3. 配置定时器基础参数（决定计数频率和周期）
    htim->Init.Prescaler = motor->speed_calc.psc; // 应用选择的预分频motor->speed_calc.psc
    htim->Init.CounterMode = TIM_COUNTERMODE_UP; // 向上计数（生成固定周期）
    htim->Init.Period = TIM_COUNT_MAX; // 初始ARR设为最大值（避免计数溢出，后续DMA动态更新）
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟不分频，保证计数精度
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // 开启ARR预装载
    HAL_TIM_Base_Init(htim); // 初始化定时器基础模式（必须在OC配置前）

    // 4. 配置PWM模式1（核心输出逻辑）
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1; 	// CNT < CCR：高电平；CNT >= CCR：低电平
		if (motor->speed_calc.psc == TIM_HIGH_SPEED_PSC)
			sConfigOC.Pulse = 1000; 								// 初始占空比（避免脉冲过窄导致驱动器误判）
		else 
			sConfigOC.Pulse = 150; 								// 初始占空比（避免脉冲过窄导致驱动器误判）
		
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; // 高极性（有效电平为高，适配多数驱动器）
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE; 	// 关闭快速模式（波形稳定优先，避免毛刺）
    HAL_TIM_OC_ConfigChannel(htim, &sConfigOC, channel); // 应用OC通道配置

    // 5. 初始化计数器（避免残留计数导致首次脉冲异常）
    __HAL_TIM_SET_COUNTER(htim, 0); // 计数器清零

}

int num_2 = 0;
// 统一的DMA发送函数
static HAL_StatusTypeDef motor_dma_transmit(motorindex_enum motor_index, uint16_t *arr_values, uint32_t arr_count, dma_mode_enum mode)
{
	HAL_GPIO_TogglePin(STATE_GPIO_Port, STATE_Pin);
	num_2++;
	if (motor_index >= MOTOR_COUNT || arr_values == NULL || arr_count == 0)
			return HAL_ERROR;
	
	Motor* motor = &Motor_list[motor_index];
	TIM_HandleTypeDef* htim = motor->motor_params.timer;
	DMA_HandleTypeDef* hdma = htim->hdma[TIM_DMA_ID_UPDATE];
	
	if (hdma == NULL)
			return HAL_ERROR;
    
	// 停止当前传输
	HAL_TIM_OC_Stop(htim, motor->motor_params.channel);
	HAL_DMA_Abort(hdma);
	__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_UPDATE);
    
  // 配置DMA模式
	hdma->Init.Mode = (mode == DMA_MODE_CIRCULAR) ? DMA_CIRCULAR : DMA_NORMAL; // 若为循环模式，则设置为循环传输，否则为单次传输
	hdma->Init.PeriphInc = DMA_PINC_DISABLE;
	hdma->Init.MemInc = DMA_MINC_ENABLE;
	hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;

	HAL_DMA_RegisterCallback(hdma, HAL_DMA_XFER_CPLT_CB_ID, motor_dma_transfer_complete_callback);	// 注册DMA传输完成回调函数
	HAL_DMA_RegisterCallback(hdma, HAL_DMA_XFER_ERROR_CB_ID, motor_dma_error_callback);	// 注册DMA传输错误回调函数
	motor->motor_params.timer->Instance->CNT = 0;		// 将定时器计数器清零
	// 重新初始化DMA
	if (HAL_DMA_Init(hdma) != HAL_OK)
			return HAL_ERROR;
	
	// 启用DMA和定时器
	__HAL_TIM_ENABLE_DMA(htim, TIM_DMA_UPDATE);
	HAL_TIM_Base_Start(htim);
	
	// 启动DMA传输
	if (HAL_DMA_Start_IT(hdma, (uint32_t)arr_values, (uint32_t)&htim->Instance->ARR, arr_count) != HAL_OK)
			return HAL_ERROR;
	
	if (HAL_TIM_OC_Start(htim, motor->motor_params.channel) != HAL_OK)
			return HAL_ERROR;
	
	
	// 更新控制状态
	motor->dma_prame.dma_mode = mode;
	motor->dma_prame.dma_state = DMA_STATE_BUSY;
	
	return HAL_OK;
}
int num_1 = 0;
// DMA传输完成回调函数
static void motor_dma_transfer_complete_callback(DMA_HandleTypeDef *hdma)
{
	num_1++;
	// 查找是哪个电机的DMA完成
	for (int i = 0; i < MOTOR_COUNT; i++) 
	{
		Motor* motor = &Motor_list[i];
		if (motor->motor_params.timer->hdma[TIM_DMA_ID_UPDATE] == hdma) 
		{
			// 如果是Jerk加速单次模式完成，需要特殊处理
			if (motor->dma_prame.dma_mode == DMA_MODE_NORMAL) 
			{
				motor->dma_prame.dma_state = DMA_STATE_COMPLETE;
				HAL_GPIO_TogglePin(STATE_GPIO_Port, STATE_Pin);
				// Jerk加速完成，电机进入匀速状态
				motor->state = MOTOR_AVESPEED;
				motor->current_speed = ARR_TO_RPM(motor->motor_params.timer->Instance->ARR, motor->motor_params.timer->Instance->PSC);	// 通过当前的ARR反推出当前速度
			}
			break;
		}
	}
}
int flag_error = 0;
// DMA传输错误回调函数
static void motor_dma_error_callback(DMA_HandleTypeDef *hdma)
{
	flag_error = 1;
	for (motorindex_enum i = 0; i < MOTOR_COUNT; i++) 
	{
		Motor* motor = &Motor_list[i];
		if (motor->motor_params.timer->hdma[TIM_DMA_ID_UPDATE] == hdma) 
		{
			motor_tim_config_set(i);	// 重新配置定时器
		}
	}
}
