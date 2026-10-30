#ifndef __OPEN_SERIAL_DEBUGGER_H__
#define __OPEN_SERIAL_DEBUGGER_H__

#ifdef __cplusplus
extern "C"{
#endif

#include "main.h"
#include <stdint.h>

#define UART_RX_BUF_SIZE  128           // 环形缓冲区大小
#define UART_WRITE_TIMEOUT 100          // 超时时间（ms）

#define MAGIC_NUM 0xAB // 1byte
#define PACKET_LEN 8   // bytes
#define CHUNK_LEN 6    // bytes

#define IS_VALID_PACKET(packet) ((packet)[0] == MAGIC_NUM)
#define GET_MAGIC_NUM(packet) ((packet)[0])
#define GET_CHUNK_NUM(packet) ((packet)[1])
#define GET_CMD_NUM(packet) (*(uint16_t*)&(packet)[2])

#define GET_RAW_DATA_UINT(packet) (*(uint32_t*)&(packet)[4])
#define GET_RAW_DATA_BOOL(packet) (*(uint32_t*)&(packet)[4])
#define GET_RAW_DATA_INT(packet) (*(int32_t*)&(packet)[4])
#define GET_RAW_DATA_FLOAT(packet) (*(float*)&(packet)[4])

#define DATA_TO_UINT(data) (*(uint32_t*)(data))
#define DATA_TO_BOOL(data) (*(uint32_t*)(data))
#define DATA_TO_INT(data) (*(int32_t*)(data))
#define DATA_TO_FLOAT(data) (*(float*)(data))

#define PACKET_ID_STR "--Packet-- "

// TODO: should be adapted if multiple chunk enabled.

typedef struct{
    uint8_t magicNum;
    uint8_t chunkNum;
    uint16_t cmdNum;
    uint32_t data;
}Packet;

extern float Position_P;
extern float Position_I;
extern float Position_D;
extern float Speed_P;
extern float Speed_I;
extern float Speed_D;
extern float Torque_Q_P;
extern float Torque_Q_I;
extern float Torque_Q_D;

extern volatile uint8_t uart_tx_busy;  //volatile禁止优化，不然会出错
extern uint8_t uart_flag_justFloat; 

void open_serial_debugger_init(UART_HandleTypeDef *huart);
int Serial_available(void);
size_t Serial_readBytes(uint8_t *buf, size_t len);
size_t Serial_write_DMA(UART_HandleTypeDef *huart, const uint8_t *buf, size_t len);
void send_data(uint8_t cmd_num, float data);
void handle_packet(const uint8_t* packet);

// bool isValidPacket(uint8_t *packet);
// uint8_t getChunkNum(uint8_t *packet);
// uint16_t getCmdNum(uint8_t *packet);
// uint32_t getRawData(uint8_t *packet);

// bool isValidPacket(uint8_t *packet) {
//     return packet[0] == MAGIC_NUM;
// }

// uint8_t getChunkNum(uint8_t *packet) {
//     return packet[1];
// }

// uint16_t getCmdNum(uint8_t *packet) {
//     uint16_t cmd;
//     memcpy(&cmd, &packet[2], sizeof(uint16_t));
//     return cmd;
// }

// uint32_t getRawData(uint8_t *packet) {
//     uint32_t data;
//     memcpy(&data, &packet[4], sizeof(uint32_t));
//     return data;
// }

#ifdef __cplusplus
}
#endif

#endif 
