/**
  ******************************************************************************
  * @attention
  * 
  * 功能：Open Serial Debugger / Vofa 串口工具协议解析
  * 作者: B站UP 点灯子曰 -- https://space.bilibili.com/3546977746815362
  *       B站UP 不重要的tan -- https://space.bilibili.com/627995923
  * 授权: 移植/再开发请注明原作者，商业行为请联系原作者
  *
  ******************************************************************************
  */ 

#include "open_serial_debugger.h"
#include "usart.h"
#include <stdbool.h>
#include <stdio.h>
#include "foc.h"
#include "adc.h"

// 串口变量
uint8_t uart_rx_buf[UART_RX_BUF_SIZE];  // 接收缓冲区
uint16_t uart_rx_head = 0;              // 缓冲区读指针
uint16_t uart_rx_tail = 0;              // 缓冲区写指针
uint8_t uart_rx_temp = 0;               // 中断接收临时字节

volatile uint8_t uart_tx_busy = 0;  //volatile禁止优化，不然会出错
uint8_t uart_flag_justFloat = 0; 

UART_HandleTypeDef *d_huart;

Packet send_packet;

float Position_P = 0;
float Position_I = 0;
float Position_D = 0;
float Speed_P = 0;
float Speed_I = 0;
float Speed_D = 0;
float Torque_Q_P = 0;
float Torque_Q_I = 0;
float Torque_Q_D = 0;

// Define commands as defined in commands.txt
typedef enum{
    DefaultMode = 0x01,
    PositionMode = 0x02,
    SpeedMode = 0x03,
    TorqueMode = 0x17,
    PosSpdMode = 0x04,
    PosTorMode = 0x14,
    SpdTorMode = 0x15,
    PosSpdTorMode = 0x16,

    SetPos_P = 0x05,
    SetPos_I = 0x06,
    SetPos_D = 0x07,
    SetPos = 0x08,

    SetSpd_P = 0x09,
    SetSpd_I = 0x0A,
    SetSpd_D = 0x0B,
    SetSpd = 0x0C,

    SetTorQ_P = 0x10,
    SetTorQ_I = 0x11,
    SetTorQ_D = 0x12,
    SetTorQ = 0x13,

    Speed_I_Limit = 0x18,
    Torque_I_Limit = 0x19,
    Position_I_Limit = 0x1B,

    Over_Curr_Recovery = 0x1A,

    PositionMode_F = 0x30,
    SpeedMode_F = 0x31,
    TorqueMode_F = 0x32,
    PosSpdMode_F = 0x33,
    PosTorMode_F = 0x34,
    SpdTorMode_F = 0x35,
    PosSpdTorMode_F = 0x36,

    GetEncoderAngle = 0x0D,
    GetCumulativeAngle = 0x0E,
    GetElectricalAngle = 0x0F,
}Command;


// 初始化串口接收中断 ***************************************************************************
void open_serial_debugger_init(UART_HandleTypeDef *huart)
{
    // 绑定全局串口句柄
    d_huart = huart;
    // 初始化串口接收中断
    memset(uart_rx_buf, 0, UART_RX_BUF_SIZE);
    HAL_UART_Receive_IT(d_huart, &uart_rx_temp, 1);
}

/**********************************************************************************************
 * @brief  等效Arduino Serial.available()
 * @return int 串口接收缓冲区中未读取的字节数
 * @note   参数/返回值和Arduino完全一致
 *********************************************************************************************/
int Serial_available(void)
{
    // 计算环形缓冲区中可用字节数（处理缓冲区溢出）
    return (uart_rx_tail - uart_rx_head + UART_RX_BUF_SIZE) % UART_RX_BUF_SIZE;
}

/**********************************************************************************************
 * @brief  等效Arduino Serial.readBytes(buf, len)
 * @param  buf: 接收数据的缓冲区指针
 * @param  len: 要读取的字节数
 * @return size_t: 实际读取的字节数
 *********************************************************************************************/
size_t Serial_readBytes(uint8_t *buf, size_t len)
{
    size_t read_count = 0;  // 实际读取字节数
    
    // 循环读取指定长度字节，直到读完或缓冲区为空
    while (read_count < len && Serial_available() > 0)
    {
        buf[read_count++] = uart_rx_buf[uart_rx_head];          // 读取1字节
        uart_rx_head = (uart_rx_head + 1) % UART_RX_BUF_SIZE;   // 移动读指针
    }
    
    return read_count;  // 返回实际读取的字节数
}

/**********************************************************************************************
 * @brief  等效Arduino Serial.write(const uint8_t *buf, size_t len)
 * @param  buf: 要发送的数据缓冲区指针
 * @param  len: 要发送的字节数
 * @return size_t: 实际发送的字节数（成功则返回len）
 *********************************************************************************************/
size_t Serial_write(const uint8_t *buf, size_t len)
{
    // HAL_UART_Transmit返回HAL_OK表示发送成功，否则返回0
    if (HAL_UART_Transmit(d_huart, (uint8_t*)buf, len, UART_WRITE_TIMEOUT) == HAL_OK)
    {
        return len;  // 发送成功，返回发送的字节数
    }
    else
    {
        return 0;    // 发送失败，返回0
    }
}

/**********************************************************************************************
 * @brief  使用DMA方式发送串口数据
 * @param  buf: 待发送的数据缓冲区指针
 * @param  len: 待发送的字节数
 * @retval 成功返回发送的字节数，失败返回0
 *********************************************************************************************/
size_t Serial_write_DMA(UART_HandleTypeDef *huart, const uint8_t *buf, size_t len)
{
    if (buf == NULL || len == 0 || uart_tx_busy)
    {
        return 0;
    }

    // 标记DMA发送中
    uart_tx_busy = 1;
    uart_flag_justFloat = 1; 

    // 使用DMA非阻塞发送数据
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(huart, (uint8_t*)buf, len);
    
    if (status == HAL_OK)
    {
        // 等待发送完成
        uint32_t timeout = 3400000000;
        while (uart_tx_busy && timeout--)
        {
            
        }

        // 检查是否超时/发送完成
        if (uart_tx_busy)
        {
            // 发送超时，停止DMA
            uart_tx_busy = 0;
            return 0;
        }
        else
        {
            // 发送成功
            return len;
        }
    }
    else
    {
        // DMA发送启动失败，清除忙标记
        uart_tx_busy = 0;
        return 0;
    }
}


#ifdef MODULE_OPEN_SERIAL_DEBUGGER_EN
// HAL库串口中断回调 ***************************************************************************
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 计算下一个写指针位置
        uint16_t next_tail = (uart_rx_tail + 1) % UART_RX_BUF_SIZE;
        
        // 缓冲区未满时写入数据（避免溢出）
        if (next_tail != uart_rx_head)
        {
            uart_rx_buf[uart_rx_tail] = uart_rx_temp;
            uart_rx_tail = next_tail;
        }
        
        // 重启中断接收下1字节（必须）
        HAL_UART_Receive_IT(d_huart, &uart_rx_temp, 1);
    }
}
#endif



void pid_position_ctrl(void);
void pid_speed_ctrl(void);
void pid_torque_q_ctrl(void);
void pid_PS_ctrl(void);
void pid_PT_ctrl(void);
void pid_ST_ctrl(void);
void pid_PST_ctrl(void);


// 协议包处理函数 ***************************************************************************
// 上位机参数都使用角度值（直观），单片机都使用弧度值
void handle_packet(const uint8_t* packet) 
{
    if (IS_VALID_PACKET(packet)) 
    {
        uint16_t cmdNum = GET_CMD_NUM(packet);
		float data = GET_RAW_DATA_FLOAT(packet); 
        //printf("cmd:%d, data:%.3f\r\n", cmdNum, data); 
        switch (cmdNum)
         {
            case GetEncoderAngle:
                //send_data(GetEncoderAngle, rad2deg(encoder_angle));
                //send_data(GetEncoderAngle, Position_P);
                break;
            case GetCumulativeAngle:
                //send_data(GetLogicAngle, rad2deg(cumulative_encoder_angle));
                //send_data(GetCumulativeAngle, Position_I);
                break;
            case GetElectricalAngle:
                //send_data(GetElectricalAngle, rad2deg(rotor_electrical_angle));
                //send_data(GetElectricalAngle, Position_D);
                break;
            case DefaultMode:
                motor_control_context.type = control_type_null;
                //printf("motor_control_context.type = %d\r\n", motor_control_context.type);
                break;
            case PositionMode:
                motor_control_context.type = control_type_position;
                //printf("motor_control_context.type = %d\r\n", motor_control_context.type);
                break;
            case SpeedMode:
                motor_control_context.type = control_type_speed;
                //printf("motor_control_context.type = %d\r\n", motor_control_context.type);
                break;
            case TorqueMode:
                motor_control_context.type = control_type_torque;
                //printf("motor_control_context.type = %d\r\n", motor_control_context.type);
                break;
            case PosSpdMode:
                motor_control_context.type = control_type_position_speed;
                //printf("motor_control_context.type = %d\r\n", motor_control_context.type);
                break;
            case PosTorMode:
                motor_control_context.type = control_type_position_torque;
                break;
            case SpdTorMode:
                motor_control_context.type = control_type_speed_torque;
                break;
            case PosSpdTorMode:
                motor_control_context.type = control_type_position_speed_torque;
                break;
            case SetPos:
                motor_control_context.position = deg2rad(data); 
                //printf("motor_control_context.position = %.5f\r\n", motor_control_context.position);
                break;
            case SetSpd:
                data = min(data, 164.0f);  // 实测，单位：度/秒
                data = max(data, -164.0f);
                motor_control_context.speed = deg2rad(data); 
                //printf("motor_control_context.speed = %.5f\r\n", motor_control_context.speed);
                break;
            case SetTorQ:
                motor_control_context.current_q = data; 
                //printf("motor_control_context.current_q = %.5f\r\n", motor_control_context.current_q);
                break;
            case SetPos_P:
                Position_P = data;
                //printf("Position_P = %.5f\r\n", Position_P);
                break;
            case SetPos_I:
                Position_I = data;
                //printf("Position_I = %.5f\r\n", Position_I);
                break;
            case SetPos_D:
                Position_D = data;
                //printf("Position_D = %.5f\r\n", Position_D);
                break;
            case SetSpd_P:
                Speed_P = data;
                //printf("Speed_P = %.5f\r\n", Speed_P);
                break;
            case SetSpd_I:
                Speed_I = data;
                //printf("Speed_I = %.5f\r\n", Speed_I);
                break;
            case SetSpd_D:
                Speed_D = data;
                //printf("Speed_D = %.5f\r\n", Speed_D);
                break;
            case SetTorQ_P:
                Torque_Q_P = data;
                //printf("Torque_Q_P = %.5f\r\n", Torque_Q_P);
                break;
            case SetTorQ_I:
                Torque_Q_I = data;
                //printf("Torque_Q_I = %.5f\r\n", Torque_Q_I);
                break;
            case SetTorQ_D:
                Torque_Q_D = data;
                //printf("Torque_Q_D = %.5f\r\n", Torque_Q_D);
                break;
            case Speed_I_Limit:
                Speed_I_Limit_Flag = 1;
                SPEED_I_LIMIT = data;
                break;
            case Torque_I_Limit:
                TORQUE_I_LIMIT = data;
                break;
            case Position_I_Limit:
                Pos_I_Limit_Flag = 1;
                POS_I_LIMIT = data;
                break;
            case Over_Curr_Recovery:
                Over_Current_Recovery();
                break;
            case PositionMode_F:
                    pid_position_ctrl();
                break;
            case SpeedMode_F:
                    pid_speed_ctrl();
                break;
            case TorqueMode_F:
                    pid_torque_q_ctrl();
                break;
            case PosSpdMode_F:
                    pid_PS_ctrl();
                break;
            case PosTorMode_F:
                    pid_PT_ctrl();
                break;
            case SpdTorMode_F:
                    pid_ST_ctrl();
                break;  
            case PosSpdTorMode_F:
                    pid_PST_ctrl();
                break;
            default:
                break;
        }
    }
}

// 更新上位机 Update parameters 一栏里的数据 *****************************************************
void send_data(uint8_t cmd_num, float data) 
{
    send_packet.magicNum = MAGIC_NUM;
    send_packet.chunkNum = 1;
    send_packet.cmdNum = cmd_num;
	
    float temp_data = data;
	// 将float的4字节二进制数据映射到uint32_t的data字段
    send_packet.data = *(uint32_t*)&temp_data; 
	
    Serial_write((uint8_t*)(&send_packet), sizeof(send_packet));
}



// vofa控制界面2（简化，不可调pid）
void pid_position_ctrl(void)
{
    motor_control_context.type = control_type_position;
    Position_P = POSITION_MODE_POS_P;
    Position_I = POSITION_MODE_POS_I;
    Position_D = POSITION_MODE_POS_D;
}

void pid_speed_ctrl(void)
{
    motor_control_context.type = control_type_speed;
    Speed_P = SPEED_MODE_SPEED_P;
    Speed_I = SPEED_MODE_SPEED_I;
    Speed_D = SPEED_MODE_SPEED_D;
}

void pid_torque_q_ctrl(void)
{
    motor_control_context.type = control_type_torque;
    Torque_Q_P = TORQUE_MODE_Q_P;
    Torque_Q_I = TORQUE_MODE_Q_I;
    Torque_Q_D = TORQUE_MODE_Q_D;
}

void pid_PS_ctrl(void)
{
    motor_control_context.type = control_type_position_speed;
    Position_P = POS_SPD_MODE_POS_P;
    Position_I = POS_SPD_MODE_POS_I;
    Position_D = POS_SPD_MODE_POS_D;
    Speed_P = POS_SPD_MODE_SPEED_P;
    Speed_I = POS_SPD_MODE_SPEED_I;
    Speed_D = POS_SPD_MODE_SPEED_D;
}

void pid_PT_ctrl(void)
{
    motor_control_context.type = control_type_position_torque;
    Position_P = POS_TOR_MODE_POS_P;
    Position_I = POS_TOR_MODE_POS_I;
    Position_D = POS_TOR_MODE_POS_D;
    Torque_Q_P = POS_TOR_MODE_Q_P;
    Torque_Q_I = POS_TOR_MODE_Q_I;
    Torque_Q_D = POS_TOR_MODE_Q_D;
}

void pid_ST_ctrl(void)
{
    motor_control_context.type = control_type_speed_torque;
    Speed_P = SPD_TOR_MODE_SPD_P;
    Speed_I = SPD_TOR_MODE_SPD_I;
    Speed_D = SPD_TOR_MODE_SPD_D;
    Torque_Q_P = SPD_TOR_MODE_Q_P;
    Torque_Q_I = SPD_TOR_MODE_Q_I;
    Torque_Q_D = SPD_TOR_MODE_Q_D;
}

void pid_PST_ctrl(void)
{
    motor_control_context.type = control_type_position_speed_torque;
    Position_P = POS_SPD_TOR_MODE_POS_P;
    Position_I = POS_SPD_TOR_MODE_POS_I;
    Position_D = POS_SPD_TOR_MODE_POS_D;
    Speed_P = POS_SPD_TOR_MODE_SPD_P;
    Speed_I = POS_SPD_TOR_MODE_SPD_I;
    Speed_D = POS_SPD_TOR_MODE_SPD_D;
    Torque_Q_P = POS_SPD_TOR_MODE_Q_P;
    Torque_Q_I = POS_SPD_TOR_MODE_Q_I;
    Torque_Q_D = POS_SPD_TOR_MODE_Q_D;
}

