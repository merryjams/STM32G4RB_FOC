#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"
#include "spi.h"
#include "arm_math.h"   // ARM DSP库（针对arm架构优化的数学库），提供sin/cos/inv_park等函数

#define Pwm_Control_Timer htim8

extern float encoder_init_angle; // 刚上电时编码器角度
extern float rotor_zero_angle;   // 转子d轴与A相重合的电气零点

#define rotor_physics_angle (encoder_angle - rotor_zero_angle)       // 转子机械角度
// 转子电角度（正负根据实际情况调整），同步修改多圈位置计算、速度计算两函数中的diff_angle，电角度为负，diff_angle = -diff_angle;
#define rotor_electrical_angle (-rotor_physics_angle * POLE_PAIRS)   

extern float motor_speed;        // 电机转速，弧度/秒
extern float motor_i_u;          // W 相采样电流，安培
extern float motor_i_v;          // V 相采样电流，安培
extern float motor_i_w;          // W 相采样电流，安培
extern float motor_i_bus;        // 低侧总线电流，安培
extern float motor_i_d;          // d 轴电流
extern float motor_i_q;          // q 轴电流

extern uint8_t Speed_I_Limit_Flag; // 是否使用串口调试SPEED_I_LIMIT值，1为是
extern uint8_t Pos_I_Limit_Flag;   // 是否使用串口调试POS_I_LIMIT值，1为是
extern float TORQUE_I_LIMIT;       // 电流环积分项限幅值
extern float SPEED_I_LIMIT;        // 速度环积分项限幅值
extern float POS_I_LIMIT;          // 位置环积分项限幅值

typedef enum
{
    control_type_null,                  // 不进行控制
    control_type_position,              // 位置控制
    control_type_speed,                 // 速度控制
    control_type_torque,                // 力矩控制
    control_type_position_speed,        // 位置-速度控制
    control_type_position_torque,       // 位置-力矩控制
    control_type_speed_torque,          // 速度-力矩控制
    control_type_position_speed_torque, // 位置-速度-力矩控制
} motor_control_type;

typedef struct
{
    motor_control_type type;
    float position;        // 目标角度，单位rad
    float speed;           // 目标速度，单位rad/s
    float current_d;       // 目标d轴电流，单位A
    float current_q;       // 目标q轴电流，单位A
} motor_control_context_t;

extern motor_control_context_t motor_control_context;


// 创建pid控制器
extern arm_pid_instance_f32 pid_position;
extern arm_pid_instance_f32 pid_speed;
extern arm_pid_instance_f32 pid_torque_d;
extern arm_pid_instance_f32 pid_torque_q;

extern arm_pid_instance_f32 pid_position_PS_Ctrl;
extern arm_pid_instance_f32 pid_speed_PS_Ctrl;

extern arm_pid_instance_f32 pid_position_PT_Ctrl;
extern arm_pid_instance_f32 pid_torque_q_PT_Ctrl;

extern arm_pid_instance_f32 pid_speed_ST_Ctrl;
extern arm_pid_instance_f32 pid_torque_q_ST_Ctrl;

extern arm_pid_instance_f32 pid_position_PST_Ctrl;
extern arm_pid_instance_f32 pid_speed_PST_Ctrl;
extern arm_pid_instance_f32 pid_torque_q_PST_Ctrl;


// 函数声明
void set_pwm_duty(float d_u, float d_v, float d_w, float max_duty);
void foc_forward(float d, float q, float electrical_rad);
float cycle_diff(float diff, float cycle);
float low_pass_filter(float input, float last_output, float alpha);
void set_rotor_zero_angle(void);
void lib_position_control(float position);
void lib_speed_control(float speed);
void lib_torque_control(float current_d, float current_q);
void lib_position_speed_control(float position, float speed);
void lib_position_torque_control(float position, float current_q);
void lib_speed_torque_control(float speed, float current_q);
void lib_position_speed_torque_control(float position, float speed, float current_q);
void set_motor_pid_single(arm_pid_instance_f32* pid_instance, float p, float i, float d);
void pid_i_reset(void);


#endif 
