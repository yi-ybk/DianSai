#ifndef __SLAVE_H
#define __SLAVE_H

#include <stdint.h>
#include "usart.h"
#include "control.h"
#include "CRC.h"
#include "util.h"
#include <string.h>
#include "bsp_usart.h"
#include "mechanism_convert.h"


#define UART_HEADER1    0x55    // 数据包头1
#define UART_HEADER2    0xaa    // 数据包头2
#define UART_END1       0x0d    // 数据包尾1
#define UART_END2       0x0a    // 数据包尾2

#define CMD_ACK             0x10

typedef struct {
    uint8_t header1;        // 包头1 0x55
    uint8_t header2;        // 包头2 0xAA
    uint8_t cmd;            // 命令字
    uint8_t len;            // 数据长度
    uint8_t data[64];       // 数据域
    uint8_t crc;            // CRC8校验
    uint8_t end1;           // 包尾1 0x0D
    uint8_t end2;           // 包尾2 0x0A
    formatTrans32Struct_t wheelSpeedTarget[2];
    formatTrans32Struct_t clawSpeedTarget[2];
    formatTrans32Struct_t clawAngleTarget[2];
    formatTrans32Struct_t jointSpeedTarget;
    formatTrans32Struct_t jointAngleTarget[4];
} SlavePacket_t;

typedef struct {
    float wheelSpeedTarget[2];
    float clawSpeedTarget[2];
    //    float clawAngleTarget[2];
    float jointSpeedTarget;
    float jointAngleTarget[4];
    uint32_t loops;
    uint32_t count;
} masterControlData_t;

masterControlData_t* get_master_control_data(void);
extern void slave_receive_handler(uint8_t* data, uint16_t size);
void slave_init(void);
void nuc_update(void const* argument);

BaseType_t slave_rx_push_from_isr(const uint8_t* data, uint16_t size, BaseType_t* pxHigherPriorityTaskWoken);
void slaveData_update(void const* argument);

void slave_request_rx_restart_from_isr(void);
void slave_service_rx_restart(void);
#endif