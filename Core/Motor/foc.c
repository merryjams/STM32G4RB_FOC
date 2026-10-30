/**
  ******************************************************************************
  * @attention
  * 
  * 功能：foc电机控制相关函数（位置、速度、电流环、pid、svpwm...）
  * 作者: B站UP 点灯子曰 -- https://space.bilibili.com/3546977746815362
  *       部分代码移植的 CSDN 朴人 -- https://blog.csdn.net/qq570437459
  * 授权: 移植/再开发请注明原作者，商业行为请联系原作者
  *
  ******************************************************************************
  */ 

#include "foc.h"
#include "tim.h"
#include <stdbool.h>
#include <stdio.h>
#include "open_serial_debugger.h"

#define rad60  1.0471976f   // 60°转换为弧度（π/3）
#define SQRT3  1.7320508f   // 根号3

float motor_speed = 0;        // 电机转速，弧度/秒
float motor_i_u = 0;          // W 相采样电流，安培
float motor_i_v = 0;          // V 相采样电流，安培
float motor_i_w = 0;          // W 相采样电流，安培
float motor_i_bus = 0;        // 低侧总线电流，安培
float motor_i_d = 0;          // d 轴电流
float motor_i_q = 0;          // q 轴电流
float encoder_init_angle = 0; // 刚上电时编码器角度
float rotor_zero_angle = 0;   // 转子d轴与A相重合的电气零点
uint8_t Speed_I_Limit_Flag = 0;  // 是否使用串口调试SPEED_I_LIMIT值，1为是
uint8_t Pos_I_Limit_Flag = 0;    // 是否使用串口调试POS_I_LIMIT值，1为是
float SPEED_I_LIMIT = 0;         // 电流环积分项限幅值
float POS_I_LIMIT = 0;           // 速度环积分项限幅值
float TORQUE_I_LIMIT = CURRENT_I_LIMIT;  // 位置环积分项限幅值
motor_control_context_t motor_control_context;


/****************************************************** 占空比 ****************************************************************************** */
// 设置Pwm占空比函数 ******************************************************************************************************************
// max_duty: 最大占空比接近100%时，由于mos管开关并非瞬间完成，会导致电流不稳定，建议90%-95%
// d_u/d_v/d_w: u/v/w 相占空比（占空比 = CCR值 / ARR值 = d_u）
void set_pwm_duty(float d_u, float d_v, float d_w, float max_duty)
{
    d_u = min(d_u, max_duty); 
    d_v = min(d_v, max_duty);
    d_w = min(d_w, max_duty);
    __disable_irq();
    __HAL_TIM_SET_COMPARE(&Pwm_Control_Timer, TIM_CHANNEL_1, (uint32_t)(d_u * Pwm_Control_Timer.Instance->ARR)); // HAL库的PWM占空比设置宏(修改定时器捕获比较寄存器（CCR）)
    __HAL_TIM_SET_COMPARE(&Pwm_Control_Timer, TIM_CHANNEL_2, (uint32_t)(d_v * Pwm_Control_Timer.Instance->ARR));
    __HAL_TIM_SET_COMPARE(&Pwm_Control_Timer, TIM_CHANNEL_3, (uint32_t)(d_w * Pwm_Control_Timer.Instance->ARR));
    __enable_irq();
}


/****************************************************** SVPWM ******************************************************************************** */
// SVPWM（空间矢量脉宽调制）函数，将FOC控制中的 d/q 轴电压转换为 U/V/W 三相的 PWM 占空比 *******************************************************
// phi：转子电角度（弧度）
// d/q: d/q 轴电压
// Udc: 母线电压
// d_u/d_v/d_w: u/v/w 相占空比
static void Ud_Uq2SVPWM(float phi, float d, float q, float Udc, float *d_u, float *d_v, float *d_w)
{
    // 母线电压有效性判断（避免除0/负数）
    if (Udc <= 2.0f) 
    { 
        // Udc小于2V视为无效，输出0占空比
        *d_u = *d_v = *d_w = 0;
        return;
    }

    // 电压归一化
    // SVPWM中，α/β轴最大线性输出电压幅值为Udc/2（三相桥的固有特性）
    float d_norm = d * 2.0f / Udc; 
    float q_norm = q * 2.0f / Udc; 

    // 限幅，避免过调制
    d_norm = min(d_norm, MAX_NORM_V);
    d_norm = max(d_norm, -MAX_NORM_V);
    q_norm = min(q_norm, MAX_NORM_V);
    q_norm = max(q_norm, -MAX_NORM_V);

    // 6个有效矢量对应 U/V/W 三相的 PWM 状态（比如v[0]={1,0,0}表示 U 相上桥臂导通，V/W 下桥臂导通）
    const int v[6][3] = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}, {1, 0, 1}};
    // 扇区查找表，通过3个布尔值组合成8种状态（K=0~7），快速定位电压矢量所在的扇区（1~6），避免条件判断，提高运行效率
    const int K_to_sector[] = {4, 6, 5, 5, 3, 1, 2, 2};

    // Park 逆变换（d/q 轴 → α/β 轴），旋转坐标系到静止坐标系
    // alpha = d*cosφ - q*sinφ
    // beta  = d*sinφ + q*cosφ
    float sin_phi = arm_sin_f32(phi);
    float cos_phi = arm_cos_f32(phi);
    float alpha = 0;
    float beta = 0;
    arm_inv_park_f32(d_norm, q_norm, &alpha, &beta, sin_phi, cos_phi);

    // 扇区判断
    bool A = beta > 0;                                 // β轴电压是否大于0
    bool B = fabs(beta) > SQRT3 * fabs(alpha);         // |β| > √3|α|
    bool C = alpha > 0;                                // α轴电压是否大于0
    int K = 4 * A + 2 * B + C;                         // 组合成0~7的索引值
    int sector = K_to_sector[K];                       // 查表得到扇区（1~6）

    // 计算矢量作用时间
    float t_m = arm_sin_f32(sector * rad60) * alpha - arm_cos_f32(sector * rad60) * beta;
    float t_n = beta * arm_cos_f32(sector * rad60 - rad60) - alpha * arm_sin_f32(sector * rad60 - rad60);
    float t_0 = 1 - t_m - t_n;

    // 过调处理
    float t_sum = t_m + t_n;
    if (t_sum > 1.0f)
    {
        t_m /= t_sum;  // 归一化到总和为1
        t_n /= t_sum;
        t_0 = 0;       // 零矢量时间为0，进入过调制模式
    }

    // 分配三相占空比
    // 第一个有效矢量对 x 相的作用时间 + 第二个有效矢量对 x 相的作用时间 + 零矢量平分到 000 和 111，各占 t_0/2
    *d_u = t_m * v[sector - 1][0] + t_n * v[sector % 6][0] + t_0 / 2;
    *d_v = t_m * v[sector - 1][1] + t_n * v[sector % 6][1] + t_0 / 2;
    *d_w = t_m * v[sector - 1][2] + t_n * v[sector % 6][2] + t_0 / 2;

    // 占空比最终限幅（防止浮点精度问题导致超0~1）
    *d_u = max(0.0f, min(*d_u, 1.0f));
    *d_v = max(0.0f, min(*d_v, 1.0f));
    *d_w = max(0.0f, min(*d_w, 1.0f));
}


/****************************************************** 电机转动调用 ****************************************************************************** */
// d/q 轴电压 --- 反帕克变换 --- SVPWM --- 三相PWM设置 ******************************************************************************
// d/q: d/q 轴电压
// electrical_rad: 转子电角度（弧度）
void foc_forward(float d, float q, float electrical_rad)
{
    float d_u = 0;
    float d_v = 0;
    float d_w = 0;

    Ud_Uq2SVPWM(electrical_rad, d, q, BUS_VOLTAGE, &d_u, &d_v, &d_w);
    set_pwm_duty(d_u, d_v, d_w, 0.9f);
    //printf("PWM duty: U = %.2f, V = %.2f, W = %.2f\r\n", d_u, d_v, d_w);
    //printf("encoder_angle:%.3f, rotor_zero_angle:%.3f, rotor_electrical_angle:%.3f\r\n\n", encoder_angle, rotor_zero_angle, rotor_electrical_angle);
        
}


/****************************************************** 周期差值最短路径 **************************************************************************** */
// 周期差值归一化函数，确保差值始终是周期内的最短路径差值 ******************************************************************************
float cycle_diff(float diff, float cycle)
{
    // 情况1：差值大于周期的一半 → 减去周期，得到负的最短差值
    if (diff > (cycle / 2))
        diff -= cycle;
    // 情况2：差值小于负的周期一半 → 加上周期，得到正的最短差值
    else if (diff < (-cycle / 2))
        diff += cycle;
    // 情况3：差值在[-cycle/2, cycle/2]范围内 → 直接返回（无需处理）
    return diff;
}


/****************************************************** 低通滤波 ************************************************************************** */
// 低通滤波器 ***********************************************************************************************************************
float low_pass_filter(float input, float last_output, float alpha)
{
	float _filtered = alpha * input + (1.0f - alpha) * last_output;
    return _filtered;
}


/****************************************************** 零点校准 ************************************************************************** */
// 转子电气零点校准 ******************************************************************************************************************
void set_rotor_zero_angle(void)
{
    set_pwm_duty(0.5f, 0, 0, 0.9f);      // 拖动转子转到d轴与A相重合的电气零点
    HAL_Delay(500);                      // 保持一会，转子吸引过来需要时间
    //MT6816_Start_DMA_Read();
    //HAL_Delay(10);
    rotor_zero_angle = encoder_angle;
    set_pwm_duty(0, 0, 0, 0.9f);         // 电机静止
    HAL_Delay(1000);
}


/****************************************************** PID ************************************************************************** */
// 创建pid控制器 
arm_pid_instance_f32 pid_position;
arm_pid_instance_f32 pid_speed;
arm_pid_instance_f32 pid_torque_d;
arm_pid_instance_f32 pid_torque_q;

arm_pid_instance_f32 pid_position_PS_Ctrl;
arm_pid_instance_f32 pid_speed_PS_Ctrl;

arm_pid_instance_f32 pid_position_PT_Ctrl;
arm_pid_instance_f32 pid_torque_q_PT_Ctrl;

arm_pid_instance_f32 pid_speed_ST_Ctrl;
arm_pid_instance_f32 pid_torque_q_ST_Ctrl;

arm_pid_instance_f32 pid_position_PST_Ctrl;
arm_pid_instance_f32 pid_speed_PST_Ctrl;
arm_pid_instance_f32 pid_torque_q_PST_Ctrl;

// 设置PID函数 ***************************************************************************************
// arm_pid_init_f32 位置式pid
// PID输出 = Kp×当前误差 + Ki×∫(误差)dt + Kd×d(误差)/dt
// 状态数组state[3]：[0]=积分项累积值，[1]=上一次误差，[2]=上上次误差
// 位置式 PID	输出 = Kp×e + Ki×∫e dt + Kd×de/dt           （输出是「绝对控制量」，比如电压 / 电流）	 依赖积分项累积，输出直接控设备
// 增量式 PID	输出增量 = Kp×(e-e?) + Ki×e + Kd×(e-2e?+e?)  （输出是「控制量的变化量」）	            无积分累积，输出是 “加 / 减多少”
// 位置式： 缺点：积分项易饱和，需手动加抗积分饱和   优点：稳态精度高（积分能消静差）
// 增量式： 缺点：无积分项，有稳态误差（需额外补偿） 优点：无积分累积，天然抗饱和（不会超调） 适合步进电机
void set_motor_pid_single(arm_pid_instance_f32* pid_instance, float p, float i, float d)
{
    pid_instance->Kp = p;
    pid_instance->Ki = i / PWM_FREQ; // 库没有乘控制周期Ts，需用户在Ki中提前包含Ts
    pid_instance->Kd = d;   // 库没有除以控制周期Ts，若需标准微分，需 * PWM_FREQ
    arm_pid_init_f32(pid_instance, false);  // false - PID的积分项/微分项继承之前的值
}

// 积分项累积值清零 ***************************************************************************
void pid_i_reset(void)
{
    pid_position.state[0] = 0.0f;
    pid_speed.state[0] = 0.0f;
    pid_torque_d.state[0] = 0.0f;
    pid_torque_q.state[0] = 0.0f;

    pid_position_PS_Ctrl.state[0] = 0.0f;
    pid_speed_PS_Ctrl.state[0] = 0.0f;

    pid_position_PT_Ctrl.state[0] = 0.0f;
    pid_torque_q_PT_Ctrl.state[0] = 0.0f;

    pid_speed_ST_Ctrl.state[0] = 0.0f;
    pid_torque_q_ST_Ctrl.state[0] = 0.0f;

    pid_position_PST_Ctrl.state[0] = 0.0f;
    pid_speed_PST_Ctrl.state[0] = 0.0f;
    pid_torque_q_PST_Ctrl.state[0] = 0.0f;
}



/****************************************************** 位置环 ***************************************************************** */
/**********************************************************************************************
 * @brief  被位置环控制函数调用
 * @param  position: 目标位置（弧度）
 * @return pid处理后的下一环节的输入
 * @note   没有使用 arm_pid_f32 这个函数的，都只有pi控制，想使用微分d项的需自己代码补上
 *********************************************************************************************/
static float position_loop(float position)
{
    position = load2motor(position); // 设定的是负载端位置，需转化为电机端位置

    static float last_out = 0;    // 保存上一次输出
    float out = 0;  // 最终输出
    //float out_pid = 0;
    float out_p = 0; // p项输出
    float out_i = 0; // i项输出
    float current_ki = 0; 
    float feedforward = 0; // 速度前馈系数
    float error_integral_windup = 0; // 积分饱和误差

    float diff = cycle_diff(position - cumulative_encoder_angle, position_cycle); // 防溢出                   


    // 死区判断，超出死区，正常PID
    if (!IN_DEADBAND(diff, POS_DEADBAND))
    {
        if(Pos_I_Limit_Flag == 0) // 不使用串口调试SPEED_I_LIMIT，赋值
        {
            switch (motor_control_context.type)
            {
                case control_type_position: POS_I_LIMIT = POS_I_LIMIT_for_U; break;
                case control_type_position_speed: POS_I_LIMIT = POS_I_LIMIT_for_S; break;
                case control_type_position_speed_torque: POS_I_LIMIT = POS_I_LIMIT_for_S; break;
                case control_type_position_torque: POS_I_LIMIT = POS_I_LIMIT_for_I; break;
                default: break;
            }
        }

        switch (motor_control_context.type)
        {
            case control_type_position: 
                // 控制效果不太好
                // out_pid = arm_pid_f32(&pid_position, diff);
                // out = min(out_pid, VOLT_MAX); // 限幅
                // out = max(out, -VOLT_MAX);
                // // 抗积分饱和：仅输出饱和时补偿积分项
                // error_integral_windup = out - out_pid;  // 积分饱和误差
                
                // 计算P项
                out_p = pid_position.Kp * diff;
                
                // 动态调整Ki（分段积分）
                if (fabs(diff) < POS_ERROR_THRESHOLD_P)
                {
                    // 小误差：用大Ki，快速消差
                    current_ki = pid_position.Ki;
                    // 积分项累积
                    pid_position.state[0] += diff * current_ki;
                    // 积分限幅
                    pid_position.state[0] = min(pid_position.state[0], POS_I_LIMIT);
                    pid_position.state[0] = max(pid_position.state[0], -POS_I_LIMIT);
                }
                else
                {
                    // 大误差：不使用积分项，不累积积分，避免超调
                    current_ki = 0;
                    // 大误差时积分项缓慢衰减，防止残留
                    pid_position.state[0] *= 0.98f;
                }
                out_i = pid_position.state[0];

                out = out_p + out_i;

                // 限幅，确保不超电压
                out = min(out, VOLT_MAX);
                out = max(out, -VOLT_MAX);

                // 抗积分饱和：仅输出饱和时补偿积分项
                error_integral_windup = out - (out_p + out_i);  // 积分饱和误差

                break;
                
            case control_type_position_speed: 
                // 分段积分+P/I拆分+速度前馈

                // 计算P项
                out_p = pid_position_PS_Ctrl.Kp * diff;
                
                // 动态调整Ki（分段积分）
                if (fabs(diff) < POS_ERROR_THRESHOLD_PS)
                {
                    // 小误差：用大Ki，快速消差
                    current_ki = pid_position_PS_Ctrl.Ki;
                    // 积分项累积
                    pid_position_PS_Ctrl.state[0] += diff * current_ki;
                    // 积分限幅
                    pid_position_PS_Ctrl.state[0] = min(pid_position_PS_Ctrl.state[0], POS_I_LIMIT);
                    pid_position_PS_Ctrl.state[0] = max(pid_position_PS_Ctrl.state[0], -POS_I_LIMIT);
                }
                else
                {
                    // 大误差：不使用积分项，不累积积分，避免超调
                    current_ki = 0;
                    // 大误差时积分项缓慢衰减，防止残留
                    pid_position_PS_Ctrl.state[0] *= 0.98f;
                }
                out_i = pid_position_PS_Ctrl.state[0];

                // 叠加速度前馈
                feedforward = POS_FEEDFORWARD_K * motor_speed; // 电机当前速度作为前馈
                out = out_p + out_i + feedforward;

                // 限幅，确保不超设定速度
                out = min(out, MAX_SPEED);
                out = max(out, -MAX_SPEED);

                // 抗积分饱和：仅输出饱和时补偿积分项
                error_integral_windup = out - (out_p + out_i);  // 积分饱和误差

                break;

            case control_type_position_speed_torque: 
                // 计算P项
                out_p = pid_position_PST_Ctrl.Kp * diff;
                
                // 动态调整Ki（分段积分）
                if (fabs(diff) < POS_ERROR_THRESHOLD_PS)
                {
                    // 小误差：用大Ki，快速消差
                    current_ki = pid_position_PST_Ctrl.Ki;
                    // 积分项累积
                    pid_position_PST_Ctrl.state[0] += diff * current_ki;
                    // 积分限幅
                    pid_position_PST_Ctrl.state[0] = min(pid_position_PST_Ctrl.state[0], POS_I_LIMIT);
                    pid_position_PST_Ctrl.state[0] = max(pid_position_PST_Ctrl.state[0], -POS_I_LIMIT);
                }
                else
                {
                    // 大误差：不使用积分项，不累积积分，避免超调
                    current_ki = 0;
                    // 大误差时积分项缓慢衰减，防止残留
                    pid_position_PST_Ctrl.state[0] *= 0.98f;
                }
                out_i = pid_position_PST_Ctrl.state[0];

                // 叠加速度前馈
                feedforward = POS_FEEDFORWARD_K * motor_speed; // 电机当前速度作为前馈
                out = out_p + out_i + feedforward;

                // 限幅，确保不超设定速度
                out = min(out, MAX_SPEED);
                out = max(out, -MAX_SPEED);

                // 抗积分饱和：仅输出饱和时补偿积分项
                error_integral_windup = out - (out_p + out_i);  // 积分饱和误差
                
                break;

            case control_type_position_torque: 
                
                // 计算P项
                out_p = pid_position_PT_Ctrl.Kp * diff;

                // 启动力矩补偿触发条件：
                // 1. 误差绝对值 > 最小启动误差（避免噪声触发）
                // 2. 误差占目标比例 > xx%（启动阶段）
                if ((fabs(diff) > MIN_START_ERROR) && ((fabs(diff)/fabs(position) > START_ERROR_RATIO))) {
                    out_p += (diff > 0) ? START_TORQUE_PT : -START_TORQUE_PT;
                }

                // 小误差保底力矩
                if (fabs(diff) > POS_DEADBAND && fabs(diff) < MIN_TORQUE_START_ERROR && fabs(out_p) < MIN_TORQUE_PT) { 
                        out_p = MIN_TORQUE_PT * ((diff > 0) ? 1.0f : -1.0f);
                }

                out_p = fminf(out_p, MAX_CURRENT * 0.8f);
                out_p = fmaxf(out_p, -MAX_CURRENT * 0.8f);
                
                // 动态调整Ki（分段积分）
                if (fabs(diff) < POS_ERROR_THRESHOLD_PT)
                {
                    // 小误差：用大Ki，快速消差
                    current_ki = pid_position_PT_Ctrl.Ki;
                    // 积分项累积
                    pid_position_PT_Ctrl.state[0] += diff * current_ki;
                    // 积分限幅
                    pid_position_PT_Ctrl.state[0] = min(pid_position_PT_Ctrl.state[0], POS_I_LIMIT);
                    pid_position_PT_Ctrl.state[0] = max(pid_position_PT_Ctrl.state[0], -POS_I_LIMIT);
                }
                else
                {
                    // 大误差：不使用积分项，不累积积分，避免超调
                    current_ki = 0;
                    // 大误差时积分项缓慢衰减，防止残留
                    pid_position_PT_Ctrl.state[0] *= 0.98f;
                }
                out_i = pid_position_PT_Ctrl.state[0];

                // 比例项输出+积分项输出+速度阻尼 
                out = out_p + out_i - POS_DAMPING_PT * motor_speed;

                // 限幅，确保不超额定电流
                out = min(out, MAX_CURRENT);
                out = max(out, -MAX_CURRENT);

                // 抗积分饱和：仅输出饱和时补偿积分项
                error_integral_windup = out - (out_p + out_i);  // 积分饱和误差

                break;

            default: break;
        }

        if (fabs(error_integral_windup) > 1e-6f) 
        {
            // 标准抗积分饱和公式：积分项 -= Ki × 饱和误差
            switch (motor_control_context.type)
            {
                case control_type_position: 
                    if(fabs(diff) < POS_ERROR_THRESHOLD_P)
                    {
                        pid_position.state[0] -= pid_position.Ki * error_integral_windup; 
                        // 积分项单独硬限幅（避免积分溢出）
                        pid_position.state[0] = min(pid_position.state[0], POS_I_LIMIT);
                        pid_position.state[0] = max(pid_position.state[0], -POS_I_LIMIT);
                    }
                    break;

                case control_type_position_speed: 
                    if(fabs(diff) < POS_ERROR_THRESHOLD_PS)
                    {
                        pid_position_PS_Ctrl.state[0] -= current_ki * error_integral_windup;
                        // 积分项单独硬限幅（避免积分溢出）
                        pid_position_PS_Ctrl.state[0] = min(pid_position_PS_Ctrl.state[0], POS_I_LIMIT);
                        pid_position_PS_Ctrl.state[0] = max(pid_position_PS_Ctrl.state[0], -POS_I_LIMIT);
                    }
                    break;

                case control_type_position_speed_torque: 
                    if(fabs(diff) < POS_ERROR_THRESHOLD_PS)
                    {
                        pid_position_PST_Ctrl.state[0] -= current_ki * error_integral_windup;
                        // 积分项单独硬限幅（避免积分溢出）
                        pid_position_PST_Ctrl.state[0] = min(pid_position_PST_Ctrl.state[0], POS_I_LIMIT);
                        pid_position_PST_Ctrl.state[0] = max(pid_position_PST_Ctrl.state[0], -POS_I_LIMIT);
                    }
                    break;

                case control_type_position_torque: 
                    if(fabs(diff) < POS_ERROR_THRESHOLD_PT)
                    {
                        pid_position_PT_Ctrl.state[0] -= current_ki * error_integral_windup;
                        // 积分项单独硬限幅（避免积分溢出）
                        pid_position_PT_Ctrl.state[0] = min(pid_position_PT_Ctrl.state[0], POS_I_LIMIT);
                        pid_position_PT_Ctrl.state[0] = max(pid_position_PT_Ctrl.state[0], -POS_I_LIMIT);
                    }
                    break;

                default: break;
            }
        }

        last_out = out;
    }

    else
    {
        // 正常死区：保持上一次输出，抑制转速抖振
        out = last_out;
        // 积分项缓慢衰减，避免突变
        // 0.95：半衰时间≈13.5个控制周期，0.98：半衰时间≈34.3个控制周期，若衰减太快，会导致重新退出死区时转速响应慢；若太慢，积分项残留会让转速抖振
        switch (motor_control_context.type)
        {
            case control_type_position: 
                pid_position.state[0] = pid_position.state[0] * 0.98f; ; break;
            case control_type_position_speed: 
                pid_position_PS_Ctrl.state[0] = pid_position_PS_Ctrl.state[0] * 0.98f; break;
            case control_type_position_speed_torque: 
                pid_position_PST_Ctrl.state[0] = pid_position_PST_Ctrl.state[0] * 0.98f; break;
            case control_type_position_torque: 
                pid_position_PT_Ctrl.state[0] = pid_position_PT_Ctrl.state[0] * 0.98f; break;
            default: break;
        }
    }

    return out;
}


/**********************************************************************************************
 * @brief  位置环控制函数
 * @param  position: 目标位置（弧度）
 *********************************************************************************************/
void lib_position_control(float position)
{
    float d = 0;    // d轴不提供转向力矩
    float q = position_loop(position);
    //printf("position_out_pid:%.2f \r\n", q);
    foc_forward(d, q, rotor_electrical_angle);
}



/****************************************************** 速度环 ***************************************************************** */
/**********************************************************************************************
 * @brief  被速度环控制函数调用
 * @param  speed:  目标速度（弧度/s）
 * @return iq_ref: q轴电流/电压
 * @note   没有使用 arm_pid_f32 这个函数的，都只有pi控制，想使用微分d项的需自己代码补上，速度环一般也不用d项
 *         WK8110参数：额定转速349rpm  最大转速566rpm 即 5.8圈/s  9.6圈/s
 *********************************************************************************************/
static float speed_loop(float speed)
{
    speed = load2motor(speed);
    speed = min(MAX_SPEED, speed);

    float diff = speed - motor_speed;  // 设定-实际

    static float last_iq_ref = 0;  // 保存上一次输出
    float iq_ref = 0;  // 最终输出
    float iq_pid = 0;  // pid输出
    
    float out = 0;   // 最终输出
    float out_p = 0; // p项输出
    float out_i = 0; // i项输出
    float current_ki = 0; 
    float error_integral_windup = 0; // 积分饱和误差

    // 死区判断，超出死区，正常PID
    if (!IN_DEADBAND(diff, SPEED_DEADBAND))
    {
        if(Speed_I_Limit_Flag == 0) // 不使用串口调试SPEED_I_LIMIT，赋值
        {
            switch (motor_control_context.type)
            {
                case control_type_speed: SPEED_I_LIMIT = SPEED_I_LIMIT_for_U; break;
                case control_type_position_speed: SPEED_I_LIMIT = SPEED_I_LIMIT_for_U; break;
                case control_type_speed_torque: SPEED_I_LIMIT = SPEED_I_LIMIT_for_I; break;
                case control_type_position_speed_torque: SPEED_I_LIMIT = SPEED_I_LIMIT_for_I; break;
                default: break;
            }
        }

        switch (motor_control_context.type)
        {
            case control_type_speed: 
                iq_pid = arm_pid_f32(&pid_speed, diff);
                iq_ref = iq_pid; // 放弃使用前馈+PID合成最终电流
                iq_ref = min(iq_ref, VOLT_MAX); // 限幅
                iq_ref = max(iq_ref, -VOLT_MAX);
                // 抗积分饱和：仅输出饱和时补偿积分项
                error_integral_windup = iq_ref - iq_pid;  // 积分饱和误差
                break;

            case control_type_position_speed: 
                iq_pid = arm_pid_f32(&pid_speed_PS_Ctrl, diff); 
                iq_ref = iq_pid; 
                iq_ref = min(iq_ref, VOLT_MAX); // 限幅
                iq_ref = max(iq_ref, -VOLT_MAX);
                error_integral_windup = iq_ref - iq_pid;  // 积分饱和误差
                break;

            case control_type_speed_torque: 
                // 计算P项
                out_p = pid_speed_ST_Ctrl.Kp * diff;
                
                // 动态调整Ki（分段积分）
                if (fabs(diff) < SPEED_ERROR_THRESHOLD_ST)
                {
                    // 小误差：用大Ki，快速消差
                    current_ki = pid_speed_ST_Ctrl.Ki;
                    // 积分项累积
                    pid_speed_ST_Ctrl.state[0] += diff * current_ki;
                    // 积分限幅
                    pid_speed_ST_Ctrl.state[0] = min(pid_speed_ST_Ctrl.state[0], SPEED_I_LIMIT);
                    pid_speed_ST_Ctrl.state[0] = max(pid_speed_ST_Ctrl.state[0], -SPEED_I_LIMIT);
                }
                else
                {
                    // 大误差：不使用积分项，不累积积分，避免超调
                    current_ki = 0;
                    // 大误差时积分项缓慢衰减，防止残留
                    pid_speed_ST_Ctrl.state[0] *= 0.98f;
                }
                out_i = pid_speed_ST_Ctrl.state[0];

                out = out_p + out_i;

                // 限幅，确保不超额定电流
                out = min(out, MAX_CURRENT);
                out = max(out, -MAX_CURRENT);

                iq_ref = out;

                // 抗积分饱和：仅输出饱和时补偿积分项
                error_integral_windup = out - (out_p + out_i);  // 积分饱和误差

                break;

            case control_type_position_speed_torque: 
                // 计算P项
                out_p = pid_speed_PST_Ctrl.Kp * diff;
                
                // 动态调整Ki（分段积分）
                if (fabs(diff) < SPEED_ERROR_THRESHOLD_ST)
                {
                    // 小误差：用大Ki，快速消差
                    current_ki = pid_speed_PST_Ctrl.Ki;
                    // 积分项累积
                    pid_speed_PST_Ctrl.state[0] += diff * current_ki;
                    // 积分限幅
                    pid_speed_PST_Ctrl.state[0] = min(pid_speed_PST_Ctrl.state[0], SPEED_I_LIMIT);
                    pid_speed_PST_Ctrl.state[0] = max(pid_speed_PST_Ctrl.state[0], -SPEED_I_LIMIT);
                }
                else
                {
                    // 大误差：不使用积分项，不累积积分，避免超调
                    current_ki = 0;
                    // 大误差时积分项缓慢衰减，防止残留
                    pid_speed_PST_Ctrl.state[0] *= 0.98f;
                }
                out_i = pid_speed_PST_Ctrl.state[0];

                out = out_p + out_i;

                // 限幅，确保不超额定电流
                out = min(out, MAX_CURRENT);
                out = max(out, -MAX_CURRENT);

                iq_ref = out;

                // 抗积分饱和：仅输出饱和时补偿积分项
                error_integral_windup = out - (out_p + out_i);  // 积分饱和误差

                break;

            default: break;
        }

        if (fabs(error_integral_windup) > 1e-6f) 
        {
            // 标准抗积分饱和公式：积分项 -= Ki × 饱和误差
            switch (motor_control_context.type)
            {
                case control_type_speed: 
                    pid_speed.state[0] -= pid_speed.Ki * error_integral_windup; 
                    // 积分项单独硬限幅（避免积分溢出）
                    pid_speed.state[0] = min(pid_speed.state[0], SPEED_I_LIMIT);
                    pid_speed.state[0] = max(pid_speed.state[0], -SPEED_I_LIMIT);
                    break;

                case control_type_position_speed: 
                    pid_speed_PS_Ctrl.state[0] -= pid_speed_PS_Ctrl.Ki * error_integral_windup;
                    // 积分项单独硬限幅（避免积分溢出）
                    pid_speed_PS_Ctrl.state[0] = min(pid_speed_PS_Ctrl.state[0], SPEED_I_LIMIT);
                    pid_speed_PS_Ctrl.state[0] = max(pid_speed_PS_Ctrl.state[0], -SPEED_I_LIMIT);
                    break;

                case control_type_speed_torque: 
                    if(fabs(diff) < SPEED_ERROR_THRESHOLD_ST)
                    {
                        pid_speed_ST_Ctrl.state[0] -= current_ki * error_integral_windup;
                        // 积分项单独硬限幅（避免积分溢出）
                        pid_speed_ST_Ctrl.state[0] = min(pid_speed_ST_Ctrl.state[0], SPEED_I_LIMIT);
                        pid_speed_ST_Ctrl.state[0] = max(pid_speed_ST_Ctrl.state[0], -SPEED_I_LIMIT);
                    }
                    break;

                case control_type_position_speed_torque: 
                    if(fabs(diff) < SPEED_ERROR_THRESHOLD_ST)
                    {
                        pid_speed_PST_Ctrl.state[0] -= current_ki * error_integral_windup;
                        // 积分项单独硬限幅（避免积分溢出）
                        pid_speed_PST_Ctrl.state[0] = min(pid_speed_PST_Ctrl.state[0], SPEED_I_LIMIT);
                        pid_speed_PST_Ctrl.state[0] = max(pid_speed_PST_Ctrl.state[0], -SPEED_I_LIMIT);
                    }
                    break;

                default: break;
            }
        }

        last_iq_ref = iq_ref;
    }

    else
    {
        // 在死区：保持上一次输出，抑制转速抖振
        iq_ref = last_iq_ref;
        // 积分项缓慢衰减，避免突变
        // 0.95：半衰时间≈13.5个控制周期，0.98：半衰时间≈34.3个控制周期，若衰减太快，会导致重新退出死区时转速响应慢；若太慢，积分项残留会让转速抖振
        switch (motor_control_context.type)
        {
            case control_type_speed: 
                pid_speed.state[0] = pid_speed.state[0] * 0.98f; ; break;
            case control_type_position_speed: 
                pid_speed_PS_Ctrl.state[0] = pid_speed_PS_Ctrl.state[0] * 0.98f; break;
            case control_type_speed_torque: 
                pid_speed_ST_Ctrl.state[0] = pid_speed_ST_Ctrl.state[0] * 0.98f; break;
            case control_type_position_speed_torque: 
                pid_speed_PST_Ctrl.state[0] = pid_speed_PST_Ctrl.state[0] * 0.98f; break;
            default: break;
        }
    }

    return iq_ref;
}

/**********************************************************************************************
 * @brief  速度环控制函数
 * @param  speed: 目标速度（弧度/s）
 *********************************************************************************************/
void lib_speed_control(float speed)
{
    float d = 0;    // d轴不提供转向力矩
    float q = speed_loop(speed);

    foc_forward(d, q, rotor_electrical_angle);
}



/****************************************************** 电流环 ***************************************************************** */
/**********************************************************************************************
 * @brief  被力矩（电流）环控制函数调用
 * @param  current_d/current_q: d、q轴目标电流
 * @return d、q轴电压
 * @note   速度环调pi即可
 *********************************************************************************************/
static float torque_q_loop(float current_q)
{
    float current_diff = current_q - motor_i_q;  // 电流误差：给定-实际

    float volt_iq_pid = 0; 

    switch (motor_control_context.type)
    {
        case control_type_torque: 
            volt_iq_pid = arm_pid_f32(&pid_torque_q, current_diff);
            break;
        case control_type_position_torque: 
            volt_iq_pid = arm_pid_f32(&pid_torque_q_PT_Ctrl, current_diff); 
            break;
        case control_type_speed_torque: 
            volt_iq_pid = arm_pid_f32(&pid_torque_q_ST_Ctrl, current_diff); 
            break;
        case control_type_position_speed_torque: 
            volt_iq_pid = arm_pid_f32(&pid_torque_q_PST_Ctrl, current_diff); 
            break;
        default: break;
    }

    float volt_iq_ref = fminf(volt_iq_pid, VOLT_MAX);  // 输出限幅
    volt_iq_ref = fmaxf(volt_iq_ref, -VOLT_MAX);

    // 抗积分饱和
    float error_integral_windup = volt_iq_ref - volt_iq_pid;
    if (fabsf(error_integral_windup) > 1e-6f) // 浮点精度判断
    {
        switch (motor_control_context.type)
        {
            case control_type_torque: 
                pid_torque_q.state[0] -= pid_torque_q.Ki * error_integral_windup; 
                pid_torque_q.state[0] = fminf(pid_torque_q.state[0], TORQUE_I_LIMIT);
                pid_torque_q.state[0] = fmaxf(pid_torque_q.state[0], -TORQUE_I_LIMIT);
                break;
            case control_type_position_torque: 
                pid_torque_q_PT_Ctrl.state[0] -= pid_torque_q_PT_Ctrl.Ki * error_integral_windup; 
                pid_torque_q_PT_Ctrl.state[0] = fminf(pid_torque_q_PT_Ctrl.state[0], TORQUE_I_LIMIT);
                pid_torque_q_PT_Ctrl.state[0] = fmaxf(pid_torque_q_PT_Ctrl.state[0], -TORQUE_I_LIMIT);
                break;
            case control_type_speed_torque: 
                pid_torque_q_ST_Ctrl.state[0] -= pid_torque_q_ST_Ctrl.Ki * error_integral_windup;
                pid_torque_q_ST_Ctrl.state[0] = fminf(pid_torque_q_ST_Ctrl.state[0], TORQUE_I_LIMIT);
                pid_torque_q_ST_Ctrl.state[0] = fmaxf(pid_torque_q_ST_Ctrl.state[0], -TORQUE_I_LIMIT);
                break;
            case control_type_position_speed_torque: 
                pid_torque_q_PST_Ctrl.state[0] -= pid_torque_q_PST_Ctrl.Ki * error_integral_windup;
                pid_torque_q_PST_Ctrl.state[0] = fminf(pid_torque_q_PST_Ctrl.state[0], TORQUE_I_LIMIT);
                pid_torque_q_PST_Ctrl.state[0] = fmaxf(pid_torque_q_PST_Ctrl.state[0], -TORQUE_I_LIMIT);
                break;
            default: break;
        }
    }

    return volt_iq_ref;
}

static float torque_d_loop(float i_d)
{
    return 0;
}

/**********************************************************************************************
 * @brief  力矩（电流）环控制函数
 * @param  current_d/current_q: d、q轴目标电流 
 * @note   没有弱磁控制的话d轴力矩为0
 *********************************************************************************************/
void lib_torque_control(float current_d, float current_q)
{
    float d = torque_d_loop(current_d);
    float q = torque_q_loop(current_q);
    foc_forward(d, q, rotor_electrical_angle);
}



/****************************************************** 位置-速度环 ***************************************************************** */
/**********************************************************************************************
 * @brief  位置-速度环控制函数
 * @param  position: 目标位置（弧度）
 * @param  speed: 到达目标位置前设定最大速度（弧度/s）
 *********************************************************************************************/
void lib_position_speed_control(float position, float speed)
{
    speed = load2motor(speed);
    float speed_rad = position_loop(position);
    speed_rad = min(fabs(speed_rad), speed) * (speed_rad > 0 ? 1 : -1);
    speed_rad = motor2load(speed_rad);
    lib_speed_control(speed_rad);
}



/****************************************************** 位置-电流环 ***************************************************************** */
/**********************************************************************************************
 * @brief  位置-电流环控制函数
 * @param  position: 目标位置（弧度）
 * @param  current_q: 到达目标位置前设定最大电流（A）
 *********************************************************************************************/
void lib_position_torque_control(float position, float current_q)
{
    float _current_q = position_loop(position);
    _current_q = min(fabs(_current_q), fabs(current_q)) * (_current_q > 0 ? 1 : -1);
    lib_torque_control(0, _current_q);
}



/****************************************************** 速度-电流环 ***************************************************************** */
/**********************************************************************************************
 * @brief 速度-电流环控制函数
 * @param  speed: 目标速度（弧度/s）
 * @param  current_q: 到达目标速度前设定最大电流（A）
 *********************************************************************************************/
void lib_speed_torque_control(float speed, float current_q)
{
    float _current_q = speed_loop(speed);
    _current_q = min(fabs(_current_q), fabs(current_q)) * (_current_q > 0 ? 1 : -1);
    lib_torque_control(0, _current_q);
}



/****************************************************** 位置-速度-电流环 ***************************************************************** */
/**********************************************************************************************
 * @brief 位置-速度-电流环控制函数
 * @param  position: 目标位置（弧度）
 * @param  speed: 到达目标位置前设定最大速度（弧度/s）
 * @param  current_q: 到达目标速度前设定最大电流（A）
 *********************************************************************************************/
void lib_position_speed_torque_control(float position, float speed, float current_q)
{
    speed = load2motor(speed);
    float speed_rad = position_loop(position);
    speed_rad = min(fabs(speed_rad), speed) * (speed_rad > 0 ? 1 : -1);
    speed_rad = motor2load(speed_rad);
    lib_speed_torque_control(speed_rad, current_q);
}

