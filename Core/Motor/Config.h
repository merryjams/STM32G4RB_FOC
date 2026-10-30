#ifndef __CONFIG_H__
#define __CONFIG_H__

/************************************************
* wk8110部分参数
额定电压 (Nominal voltage)		   24 V
额定电流 (Nominal current)		   2.49 A
额定扭矩 (Nominal torque)		   0.93 N.M
额定转速 (Nominal speed)		   349 rpm
最大转速 (Max speed)	    	   566 rpm
堵转扭矩 (Stall torque)	    	   2.4 N.M
堵转电流 (Stall current)		   6.51 A
相间电阻 (Interphase resistance)   2.29 Ω
相间电感 (Interphase inductance)   2.32 Mh
转速常数 (Speed constant)	       24 rpm/v
扭矩常数 (Torque constant)	       0.39 N.M/A
转子惯量 (Rotor inertia)	       1716 gcm^2
极对数 (Number of pole pairs)	   21
工作温度范围 (Working temperature)  -20~80 ℃
最高退磁温度 (Max demagnetize temperature)  120 ℃
************************************************ */

// 相电压 = 反电动势 + IR + Ldi/dt

// 代码对低速大扭矩电机进行pid算法优化，未对低扭矩高速电机进行测试

// 基本运算参数：
#define PI 3.1415927f
#define deg2rad(a) (PI * (a) / 180)
#define rad2deg(a) (180 * (a) / PI)
#define motor2load(x) (x / REDUCTION_RATIO)
#define load2motor(x) (x * REDUCTION_RATIO)
#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

// 电机物理参数：
#define POLE_PAIRS 21          // 极对数
#define REDUCTION_RATIO 16     // 减速比

// 电路参数：
#define MAX_CURRENT 2.5f        // 额定电流，安培
#define BUS_VOLTAGE 24.0f       // 母线电压，伏
//#define MAX_SPEED 36.5f         // 电机端额定转速，rad/s
#define MAX_SPEED 58.0f         // 电机端最大转速，rad/s

// 运放相关参数：
#define ADC_VOLTAGE 3.3f            // 电流采样adc参考电压，伏
#define ADC_RESOLUTION 4096         // ADC分辨率
#define R_SHUNT 0.005f              // 电流采样电阻，欧姆
#define OP_GAIN 50                  // 运放放大倍数
#define BUS_VREF_CORRECT   0.055f   // 母线运放Vref补偿，实测而来 (1.65-1.47)/3.3=0.055
#define MOTOR_I_U_CORRECT  0.012f   // 零电流时U相电流校正
#define MOTOR_I_V_CORRECT  0.01f    // 零电流时V相电流校正

// 保护相关
#define OVER_CURR_THRESH_Con   3.5f  // 持续过流阈值，安培
#define OVER_CURR_THRESH_Ins   6.4f  // 瞬间大电流过流阈值，安培
#define OVER_CURR_CNT_Con      160   // 持续过流确认次数（连续多次超过阈值才触发，防误触）（pwm16khz下，160对应10ms）
#define OVER_CURR_CNT_Ins      48    // 瞬时大电流过流确认次数
#define TEMPREATURE_THRESHOLD  65    // 过温阈值，°C

// 环配置参数：
#define PWM_FREQ 16000          //pwm1、2、3频率，Hz
#define SPEED_CALCU_FREQ 4000	//电机速度计算频率，Hz

// 低通滤波器参数，α 越大：滤波越弱（输出越贴近输入，响应快但噪声多）；α 越小：滤波越强（输出越平滑，响应慢但噪声少），一般0.1-0.3
#define motor_speed_low_pass_filter_alpha 0.1f
#define i_d_low_pass_filter_alpha 0.1f
#define i_q_low_pass_filter_alpha 0.1f

// 卡尔曼滤波参数，要平滑 → 增大r、减小q；要响应 → 减小r、增大q；详见kalman_filter.c
#define KF_R 1.0f
#define KF_Q 0.001f

// 多圈周期，用来防止溢出的，只要不溢出可以任意修改(比如改为1234567*pi)
#define position_cycle (2048 * PI)    //最多设置跑1024圈，如果设为(2048*PI+1)，则等于设置为1rad

// 串口模块宏开关：二选一启用（注释掉另一个即可）
#define MODULE_OPEN_SERIAL_DEBUGGER_EN  1  // 使用vofa或OPEN_SERIAL_DEBUGGER时使用，更方便，同时可调pid
//#define MODULE_GENERAL_SERIAL_EN  1  // 通用串口工具都可使用（已放弃，vofa用着还不错，也免费）


// 闭环死区判断宏：val = 偏差值，deadband = 死区阈值
#define IN_DEADBAND(val, deadband)  (fabs(val) < (deadband) ? 1 : 0)
// wk8110 额定转速36.5 rad/s，最大转速 60.3 rad/s
#define SPEED_DEADBAND  0.5f    // 0.1即为 电机端速度死区0.1rad/s（5.7°/s）,负载端（减速比16）到误差0.356°/s后不再调节，单位rad/s
#define POS_DEADBAND    0.15f   // 位置死区同理，单位rad
#define q_DEADBAND      0.01f   // 0.01即为10mA的电流误差不再调节


// 抗积分饱和，积分项最大输出（取值5%-30%）
// 位置环：（Ki × 该值）= 下级（母线电压/最大速度/额定电流）的 10%
#define POS_I_LIMIT_for_U 250.0f
#define POS_I_LIMIT_for_S 100.0f
#define POS_I_LIMIT_for_I 8.0f
// 速度环：（Ki × 该值）= 下级（母线电压/额定电流）的 10%
#define SPEED_I_LIMIT_for_U 0.3f
#define SPEED_I_LIMIT_for_I 0.08f
// 电流环：（Ki × 该值）= 母线电压的 20%
#define CURRENT_I_LIMIT 0.05f


#define POS_FEEDFORWARD_K   0.1f       // 位置-速度环速度前馈系数（0-1）：不使用时设为0
#define POS_ERROR_THRESHOLD_PS 10.0f   // 当位置-速度环位置误差小于该值时使用积分项，大误差时只使用比例项，单位rad（电机端）
#define POS_ERROR_THRESHOLD_PT 10.0f   // 当位置-电流环...
#define POS_ERROR_THRESHOLD_P 80.0f   // 当位置环位置误差小于该值时使用积分项，大误差时只使用比例项，单位rad（电机端）

#define SPEED_ERROR_THRESHOLD_ST 0.6f  // 当速度-电流环速度误差小于该值时使用积分项，大误差时只使用比例项，单位rad/s（电机端）


// 位置-电流环参数（使用了启动力矩补偿、速度阻尼、小角度时力矩补偿，不然控制效果太差）
#define START_TORQUE_PT   0.2f    // 启动力矩（电流，安培）
#define START_ERROR_RATIO 0.9f    // 启动比例阈值（误差占目标多少比例（%）就触发）
#define MIN_START_ERROR   0.5f    // 最小启动误差（误差>多少角度（rad）才触发，避免噪声）
#define POS_DAMPING_PT    0.03f   // 速度阻尼系数
#define MIN_TORQUE_PT     0.2f    // 刚好克服摩擦的保底力矩（电流，安培）
#define MIN_TORQUE_START_ERROR  0.7f  // 使用保底力矩的角度范围（rad）
      

#define MAX_NORM_V  0.8f               // svpwm归一化电压最大幅值（0.866以下线性区，留一定阈值）
#define VOLT_MAX (BUS_VOLTAGE * 0.9f)  // 各环输出给SVPWM的最大q轴电压


// pid值 ***************************************************

// 位置环
#define POSITION_MODE_POS_P 0.05f
#define POSITION_MODE_POS_I 0.01f
#define POSITION_MODE_POS_D 0

// 速度环
#define SPEED_MODE_SPEED_P 0.2f
#define SPEED_MODE_SPEED_I 9.0f
#define SPEED_MODE_SPEED_D 0

// 电流环
#define TORQUE_MODE_Q_P 0.6f
#define TORQUE_MODE_Q_I 80.0f
#define TORQUE_MODE_Q_D 0

// 位置-速度环
#define POS_SPD_MODE_POS_P 0.36f
#define POS_SPD_MODE_POS_I 0.06f
#define POS_SPD_MODE_POS_D 0
#define POS_SPD_MODE_SPEED_P 0.2f
#define POS_SPD_MODE_SPEED_I 9.0f
#define POS_SPD_MODE_SPEED_D 0

// 位置-电流环
#define POS_TOR_MODE_POS_P 0.006f
#define POS_TOR_MODE_POS_I 0.03f
#define POS_TOR_MODE_POS_D 0
#define POS_TOR_MODE_Q_P 0.6f
#define POS_TOR_MODE_Q_I 80.0f
#define POS_TOR_MODE_Q_D 0

// 速度-电流环
#define SPD_TOR_MODE_SPD_P 0.25f
#define SPD_TOR_MODE_SPD_I 3.0f
#define SPD_TOR_MODE_SPD_D 0
#define SPD_TOR_MODE_Q_P 0.6f
#define SPD_TOR_MODE_Q_I 80.0f
#define SPD_TOR_MODE_Q_D 0

// 位置-速度-电流环
#define POS_SPD_TOR_MODE_POS_P 0.68f
#define POS_SPD_TOR_MODE_POS_I 0.4f
#define POS_SPD_TOR_MODE_POS_D 0
#define POS_SPD_TOR_MODE_SPD_P 0.25f
#define POS_SPD_TOR_MODE_SPD_I 3.0f
#define POS_SPD_TOR_MODE_SPD_D 0
#define POS_SPD_TOR_MODE_Q_P 0.6f
#define POS_SPD_TOR_MODE_Q_I 80.0f
#define POS_SPD_TOR_MODE_Q_D 0


#endif 
