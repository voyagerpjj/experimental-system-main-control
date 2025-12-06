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

// 电机控制函数
void motor_control(motorindex_enum motor_index, uint16_t target_speed)
{
    target_speed = IFR_CLAMP(target_speed, 0, 6000); // 限制电机速度范围为0-600rpm
//    motor_enable(motor_index); // 使能电机
    
//    if (start == 1)
//    {
//        start = 1;
//        uint32_t psc = Motor_list[motor_index].speed_calc.psc;
//        
//        // 计算目标ARR值（50rpm对应13499）
//        // 将目标ARR值放入DMA传输数组中
//        Motor_list[motor_index].speed_calc.toggle_pulse[0] = target_arr;
//        Motor_list[motor_index].speed_calc.accel_pulse = 1;
//        
//        // 启动DMA传输更新ARR值
//        motor_dma_transmit(motor_index, Motor_list[motor_index].speed_calc.toggle_pulse, 1, DMA_MODE_NORMAL);
//    }
	motor_set_state(motor_index, target_speed);
	motor_set_speed(motor_index, target_speed);
}

// 电机使能
void motor_enable(motorindex_enum motor_index)
{
    if(motor_index >= MOTOR_COUNT)
        return;
    HAL_GPIO_WritePin(Motor_list[motor_index].motor_params.enable_port, 
                     Motor_list[motor_index].motor_params.enable_pin, GPIO_PIN_SET);
}

// 电机失能
void motor_disable(motorindex_enum motor_index)
{
    // 当电机需要停止时，先进行减速处理，然后再关闭使能
    if(motor_index >= MOTOR_COUNT)
        return;
    
    // 将电机使能引脚设置为低电平，从而关闭电机
    HAL_GPIO_WritePin(Motor_list[motor_index].motor_params.enable_port, 
                     Motor_list[motor_index].motor_params.enable_pin, GPIO_PIN_RESET);
    Motor_list[motor_index].current_speed = 0;
}

int ARR_2 = 0;
float resolution = 0;

// 电机状态机：根据当前速度和目标速度设置状态
static void motor_set_state(motorindex_enum motor_index, uint16_t target_speed)
{
    Motor *this_motor = &Motor_list[motor_index]; // 获取当前电机对象
    float tar_speed = _01RPM_TO_RPM(target_speed);
    now_speed = ARR_TO_RPM(this_motor->motor_params.timer->Instance->ARR, 
                          this_motor->motor_params.timer->Instance->PSC);
    ARR_2 = this_motor->motor_params.timer->Instance->ARR;
    resolution = SPEED_RESOLUTION(tar_speed, this_motor->speed_calc.psc);
    
    // 判断电机当前状态：停止、恒速、加速减速
    if (this_motor->current_speed + tar_speed < 0.01f)
        this_motor->state = MOTOR_IDLE;
    
    else if (fabs(this_motor->current_speed - tar_speed) <= 0.54f && this_motor->motor_speed_state == MOTOR_HIGH_MINOR_ADJUST)
        this_motor->state = MOTOR_AVESPEED;

    else if (fabs(this_motor->current_speed - tar_speed) >= 0.1f)
        this_motor->state = MOTOR_ACTIVE;
		
		else 
			this_motor->state = MOTOR_AVESPEED;

    // 判断电机速度状态
    if (tar_speed == 0 || this_motor->state == MOTOR_IDLE)
        this_motor->motor_speed_state = MOTOR_STOP; // 停止状态

    else if (0 < tar_speed && tar_speed < LOW_HIGH_SWITCH_SPEED)
        this_motor->motor_speed_state = MOTOR_LOW_SPEED_STATE; // 低速状态

    else if (LOW_HIGH_SWITCH_SPEED <= tar_speed && tar_speed < HIGH_ADJUST_SWITCH_SPEED)
        this_motor->motor_speed_state = MOTOR_HIGH_SPEED_STATE; // 高速状态

    else if (HIGH_ADJUST_SWITCH_SPEED <= tar_speed && tar_speed <= MOTOR_MAX_SPEED_RPM)
        this_motor->motor_speed_state = MOTOR_HIGH_MINOR_ADJUST; // 高速微调状态
}

// 电机速度设置
void motor_set_speed(motorindex_enum motor_index, uint16_t speed)
{
    if (motor_index >= MOTOR_COUNT)
        return;
    
    Motor *motor = &Motor_list[motor_index]; // 获取当前电机对象
    float target_rpm = _01RPM_TO_RPM(speed);
    
    if (motor->last_motor_speed_state != motor->motor_speed_state) // 如果电机速度状态发生变化
        motor_tim_config_set(motor_index);
        
    motor->last_motor_speed_state = motor->motor_speed_state;
    
    if (motor->motor_speed_state == MOTOR_STOP)
    {
        motor_disable(motor_index); // 失能电机
        motor->last_target_speed = 0;
        return;
    }
    else 
        motor_enable(motor_index); // 使能电机

    if (motor->last_target_speed != target_rpm) // 如果电机目标速度发生变化
    {
        motor_jerk_control(motor_index, motor->current_speed * 10.0f, speed, JERK); // 使用Jerk控制平滑加速       
        motor->last_target_speed = target_rpm;
    }

    else if (motor->state == MOTOR_AVESPEED) // 如果电机当前速度接近目标速度
    {
        if (motor->motor_speed_state == MOTOR_HIGH_MINOR_ADJUST && motor->last_state == MOTOR_ACTIVE) // 如果电机在高速微调状态中。从加速过程切换至匀速过程
				{
					
           motor_high_speed_minor_adjust(motor_index, speed); // 使用高速微调算法
					
				}
    }
    
    if (motor->state == MOTOR_ACTIVE)
    {
        // 加速/减速处理
    }
    motor->last_state = motor->state;
   
}

// 电机停止控制（使用Jerk平滑停止到0）
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

// 电机初始化
static void motor_init(motorindex_enum motor_index, TIM_HandleTypeDef* timer, uint32_t channel,
                      GPIO_TypeDef* enable_port, uint32_t enable_pin)
{
    Motor_list[motor_index].motorindex = motor_index;
    Motor_list[motor_index].state = MOTOR_IDLE;
    Motor_list[motor_index].current_speed = 0;
    memset(Motor_list[motor_index].speed_calc.toggle_pulse, 0, 
           sizeof(Motor_list[motor_index].speed_calc.toggle_pulse));
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

// 电机Jerk控制：S曲线加减速，时间同步计算+更新ARR值
static void motor_jerk_control(motorindex_enum motor_index, uint16_t start_speed, 
                              uint16_t target_speed, float jerk)
{
    num_3++;
    float start_rpm = _01RPM_TO_RPM(start_speed);
    float target_rpm = _01RPM_TO_RPM(target_speed);
    Motor* motor = &Motor_list[motor_index];
    motor->target_speed.target_speed = target_speed;
    uint32_t psc = motor->speed_calc.psc; // 获取当前电机的psc值

    // 计算最小速度限制（脉冲/秒）
    MIN_SPEED = MOTOR_SPEED_MIN(psc);

    // 转换为脉冲频率（脉冲/秒）
    float start_pulse = RPM_TO_PULSE(start_rpm);
    float target_pulse = RPM_TO_PULSE(target_rpm);
    float speed_error = target_pulse - start_pulse;

    // 如果目标速度为0，则直接停止
    if (target_rpm == 0.0f) 
    {
        motor_disable(motor_index); // 失能电机
        return;
    }

    // 加速度方向判断：1:加速，-1:减速
    int8_t accel_direction = (speed_error > 0) ? 1 : -1;
    float JERK_effective = accel_direction * jerk;

    // 计算加速/减速段的时间T和总时间
    float T = SPEED_TO_TIME(speed_error); // 加速段时间
    float total_time = 2 * T; // 总加速减速时间

    // 计算加速段中间速度v_mid，确保不低于最小速度
    float v_mid = start_pulse + JERK_effective * 0.5f * (T * T);
    if (v_mid < MIN_SPEED)
        v_mid = MIN_SPEED;

    // 初始化变量
    float current_speed = start_pulse; // 当前速度（脉冲/秒）
    float sum_time = 0.0f; // 累计时间
    uint16_t pulse_count = 0; // 脉冲计数
    memset(motor->speed_calc.toggle_pulse, 0, sizeof(motor->speed_calc.toggle_pulse));

    // 如果当前速度低于最小速度，先加速到最小速度
    if (current_speed < MIN_SPEED) 
    {
        float t_start = sqrt(2 * (MIN_SPEED - current_speed) / jerk);
        t_start = (t_start < 1e-6f) ? 1e-6f : t_start; // 防止时间为0

        current_speed = MIN_SPEED;
        uint16_t start_arr = (uint16_t)IFR_CLAMP(PULSE_TO_ARR(current_speed, psc), 1, TIM_COUNT_MAX);
        motor->speed_calc.toggle_pulse[pulse_count++] = start_arr;
        sum_time = t_start;
    }

    // 循环计算S曲线每个点的ARR值
    while (pulse_count < SPEED_MAX_PLUSE && sum_time < total_time) 
    {
        // 计算当前步长时间（1/当前速度）
        float step_time = 1.0f / current_speed;
        sum_time += step_time;

        // 根据时间段计算当前速度
        if (sum_time < T) 
            // 加速段：v = v0 + 0.5 * J_eff * t^2
            current_speed = start_pulse + JERK_effective * 0.5f * (sum_time * sum_time);
        else 
            // 减速段：v = -0.5*J_eff*t^2 + 2*J_eff*T*t - J_eff*T^2 + v0
            current_speed = -0.5f * JERK_effective * sum_time * sum_time + 
                            2 * JERK_effective * T * sum_time - 
                            JERK_effective * T * T + start_pulse;

        // 限制最小速度
        if (current_speed < MIN_SPEED) 
            current_speed = MIN_SPEED;

        // 计算当前ARR值并存储
        uint16_t arr_value = (uint16_t)IFR_CLAMP(PULSE_TO_ARR(current_speed, psc), 1, TIM_COUNT_MAX);
        motor->speed_calc.toggle_pulse[pulse_count++] = arr_value;

        // 如果速度接近目标（误差<0.1rpm）且已过加速段，则结束
        float current_rpm = ARR_TO_RPM(arr_value, psc);
        if (fabs(current_rpm - target_rpm) < 0.1f && sum_time >= T)
        {
            // 添加最后一个目标速度的ARR值
            target_arr = roundf(IFR_CLAMP(PULSE_TO_ARR(target_pulse, psc), 1, TIM_COUNT_MAX));
            motor->speed_calc.toggle_pulse[pulse_count++] = target_arr;
            break;
        }
    }
    
    if (sum_time >= 2.0f * T) 
    {
        // 添加最后一个目标速度的ARR值
        target_arr = roundf(IFR_CLAMP(PULSE_TO_ARR(target_pulse, psc), 1, TIM_COUNT_MAX));
        motor->speed_calc.toggle_pulse[pulse_count++] = target_arr;
    }
    
    // 更新电机状态并启动DMA
    motor->sum_time_all = sum_time;
    motor->speed_calc.accel_pulse = pulse_count;
    motor_dma_transmit(motor_index, motor->speed_calc.toggle_pulse, pulse_count, DMA_MODE_NORMAL);
}

int ARR_1 = 0;
float high_speed_minor_adjust[SUBDIVIDE_RATIO] = {0};
int32_t actual_arr[SUBDIVIDE_RATIO] = {0};

// 电机高速微调算法
static void motor_high_speed_minor_adjust(motorindex_enum motor_index, uint16_t target_speed)
{
    Motor* motor = &Motor_list[motor_index];
    motor->target_speed.target_speed = target_speed;
    float target_rpm = _01RPM_TO_RPM(target_speed); // 转换为实际RPM（0.1rpm分辨率）
    uint32_t psc = motor->speed_calc.psc;
    
    memset(motor->speed_calc.toggle_pulse, 0, sizeof(motor->speed_calc.toggle_pulse)); // 清除ARR值数组

    // 1. 计算目标ARR的浮点值
    float target_arr_float = RPM_TO_ARR_FLOAT(target_rpm, psc); // 目标ARR（浮点）
    int32_t integer_arr = (int32_t)round(target_arr_float); // 取整ARR
    float arr_error = target_arr_float - integer_arr; // ARR误差

    // 2. 将误差分配到7个细分周期中
    int32_t total_adjust = (int32_t)round(arr_error * SUBDIVIDE_RATIO);
    total_adjust = fmax(-(int32_t)SUBDIVIDE_RATIO, fmin((int32_t)SUBDIVIDE_RATIO, total_adjust));

    for (uint8_t i = 0; i < SUBDIVIDE_RATIO; i++)
        actual_arr[i] = integer_arr; // 初始化为整型ARR

    // 循环调整每个细分周期
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
        // ARR限制在100~TIM_COUNT_MAX，防止频率过低或过高
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
    motor_dma_transmit(motor_index, motor->speed_calc.toggle_pulse, 
                      motor->speed_calc.accel_pulse, DMA_MODE_CIRCULAR);
}

// 电机定时器配置设置
static void motor_tim_config_set(motorindex_enum motor_index)
{
    HAL_GPIO_TogglePin(STATE_GPIO_Port, STATE_Pin);
    Motor* motor = &Motor_list[motor_index]; // 获取当前电机对象
    TIM_HandleTypeDef* htim = motor->motor_params.timer;
    uint32_t channel = motor->motor_params.channel;

    // 1. 停止定时器和DMA，确保配置前定时器完全停止
    __HAL_TIM_DISABLE(htim); // 禁用定时器，确保PSC/ARR修改生效
    HAL_TIM_OC_Stop(htim, channel); // 停止OC输出
    
    if (htim->hdma[TIM_DMA_ID_UPDATE] != NULL) // 如果存在DMA，则中止
    {
        HAL_DMA_Abort(htim->hdma[TIM_DMA_ID_UPDATE]);
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_UPDATE); // 禁用定时器DMA请求
    }

    // 2. 根据速度状态选择预分频器（PSC）
    if (motor->motor_speed_state == MOTOR_LOW_SPEED_STATE)
    {
        motor->speed_calc.psc = TIM_LOW_SPEED_PSC; // 低速/停止：较大PSC
    }
    else if (motor->motor_speed_state == MOTOR_HIGH_SPEED_STATE || 
             motor->motor_speed_state == MOTOR_HIGH_MINOR_ADJUST || 
             motor->motor_speed_state == MOTOR_STOP)
    {
        motor->speed_calc.psc = TIM_HIGH_SPEED_PSC; // 高速/微调：较小PSC
    }
		HAL_TIM_Base_DeInit(htim); // 反初始化
    // 3. 配置定时器基本参数
    htim->Init.Prescaler = motor->speed_calc.psc; // 使用选择的预分频器
    htim->Init.CounterMode = TIM_COUNTERMODE_UP; // 向上计数模式
    htim->Init.Period = TIM_COUNT_MAX; // 初始ARR设为最大值，防止计数溢出
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟分频为1
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // 自动重装载预装载禁用
    HAL_TIM_Base_Init(htim); // 初始化定时器基础模式

    // 4. 配置PWM模式1，确保输出正确
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1; // CNT < CCR时高电平，CNT >= CCR时低电平
    
    if (motor->speed_calc.psc == TIM_HIGH_SPEED_PSC)
        sConfigOC.Pulse = 1000; // 初始占空比，确保脉冲宽度合适
    else 
        sConfigOC.Pulse = 150; // 初始占空比，确保脉冲宽度合适
        
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; // 输出极性高，有效电平为高
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE; // 禁用快速模式
    HAL_TIM_OC_ConfigChannel(htim, &sConfigOC, channel); // 配置OC通道

    // 5. 重置计数器，确保从0开始计数
    __HAL_TIM_SET_COUNTER(htim, 0); // 计数器清零
}

int num_2 = 0;

// DMA传输函数
static HAL_StatusTypeDef motor_dma_transmit(motorindex_enum motor_index, uint16_t *arr_values, 
                                          uint32_t arr_count, dma_mode_enum mode)
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
    hdma->Init.Mode = (mode == DMA_MODE_CIRCULAR) ? DMA_CIRCULAR : DMA_NORMAL; // 循环模式或正常模式
    hdma->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma->Init.MemInc = DMA_MINC_ENABLE;
    hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;

    HAL_DMA_RegisterCallback(hdma, HAL_DMA_XFER_CPLT_CB_ID, motor_dma_transfer_complete_callback); // 注册DMA传输完成回调
    HAL_DMA_RegisterCallback(hdma, HAL_DMA_XFER_ERROR_CB_ID, motor_dma_error_callback); // 注册DMA传输错误回调
    motor->motor_params.timer->Instance->CNT = 0; // 定时器计数器清零
    
    // 初始化DMA
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
	// 查找是哪个电机的DMA传输完成
	for (int i = 0; i < MOTOR_COUNT; i++) 
	{
		Motor* motor = &Motor_list[i];
		if (motor->motor_params.timer->hdma[TIM_DMA_ID_UPDATE] == hdma) 
		{
			// 如果是Jerk加速的正常模式完成，则电机进入匀速状态
			if (motor->dma_prame.dma_mode == DMA_MODE_NORMAL) 
			{
					motor->dma_prame.dma_state = DMA_STATE_COMPLETE;
					HAL_GPIO_TogglePin(STATE_GPIO_Port, STATE_Pin);
					// Jerk加速完成，电机进入匀速状态
					motor->state = MOTOR_AVESPEED;
					motor->current_speed = ARR_TO_RPM(motor->motor_params.timer->Instance->ARR, 
																					 motor->motor_params.timer->Instance->PSC); // 通过当前ARR值计算当前速度
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
            motor_tim_config_set(i); // 重新配置定时器
        }
    }
}
