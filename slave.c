#include "slave.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"

/* 流缓冲区用来存储接收到的数据 */
#define SLAVE_RX_STREAM_SIZE 512u

static StaticStreamBuffer_t slaveRxStreamStruct;
static uint8_t slaveRxStreamStorage[SLAVE_RX_STREAM_SIZE];
static StreamBufferHandle_t slaveRxStream;
/* 这组统计量用于区分“需要重启接收”和“流缓冲被打满”两类问题。 */
volatile uint8_t slave_rx_restart_req = 0u;
volatile uint32_t slave_rx_restart_req_count = 0u;
volatile uint32_t slave_rx_restart_do_count = 0u;
volatile uint32_t slave_rx_stream_drop_count = 0u;
volatile uint32_t slave_rx_stream_drop_bytes = 0u;


static uint8_t rx_buffer[256];
static uint16_t rx_len = 0;
SlavePacket_t rx_packet;
masterControlData_t masterControlData;

masterControlData_t* get_master_control_data(void){
    return &masterControlData;
}

void slave_request_rx_restart_from_isr(void)
{
    /* ISR 只置位请求，真正恢复留给任务上下文处理。 */
    slave_rx_restart_req = 1u;
    slave_rx_restart_req_count++;
}

void slave_service_rx_restart(void)
{
    if (slave_rx_restart_req != 0u)
    {
        taskENTER_CRITICAL();
        slave_rx_restart_req = 0u;
        taskEXIT_CRITICAL();

        /* 统一走同一个初始化入口，避免恢复路径和首次启动路径不一致。 */
        usart1_rx_dma_init();
        slave_rx_restart_do_count++;
    }
}

BaseType_t slave_rx_push_from_isr(const uint8_t *data, uint16_t size, BaseType_t *pxHigherPriorityTaskWoken)
{
    size_t space = 0u;
    size_t sent = 0u;

    if ((data == NULL) || (size == 0u) || (slaveRxStream == NULL))
    {
        return pdFAIL;
    }

    space = xStreamBufferSpacesAvailable(slaveRxStream);
    if (space < size)
    {
        /* 空间不够时整段丢弃，避免只写入半包导致协议流错位。 */
        slave_rx_stream_drop_count++;
        slave_rx_stream_drop_bytes += size;
        return pdFAIL;
    }

    sent = xStreamBufferSendFromISR(slaveRxStream, data, size, pxHigherPriorityTaskWoken);
    if (sent != size)
    {
        slave_rx_stream_drop_count++;
        slave_rx_stream_drop_bytes += (uint32_t)(size - sent);
        return pdFAIL;
    }

    return pdPASS;
}


volatile int rx_success = 0;
volatile int rx_crc_fail = 0; // 新增：CRC校验失败计数
void slave_receive_handler(uint8_t *data, uint16_t size){
    // Validate input
    if (data == NULL || size == 0) return;
    // Prevent buffer overflow
    if (size > sizeof(rx_buffer)) {
        data += (size - sizeof(rx_buffer));
        size = sizeof(rx_buffer);
    }
    // update rx_buffer and rx_len
    if ((uint32_t)rx_len + size > sizeof(rx_buffer)) {
        uint16_t overflow = (uint16_t)(rx_len + size - sizeof(rx_buffer));
        if (overflow >= rx_len) {
            rx_len = 0;
        } else {
            memmove(rx_buffer, rx_buffer + overflow, rx_len - overflow);
            rx_len -= overflow;
        }
    }
    memcpy(rx_buffer + rx_len, data, size);
    rx_len += size;
    for (;;) {
        if (rx_len < 7u) return;
        uint16_t head = 0xFFFF;
        for (uint16_t i = 0; i + 1 < rx_len; i++) {
            if (rx_buffer[i] == UART_HEADER1 && rx_buffer[i + 1] == UART_HEADER2) {
                head = i;
                break;
            }
        }
        if (head == 0xFFFF) {
            uint8_t keep = (rx_buffer[rx_len - 1] == UART_HEADER1) ? 1u : 0u;
            if (keep) rx_buffer[0] = UART_HEADER1;
            rx_len = keep;
            return;
        }
        if (head > 0) {
            memmove(rx_buffer, rx_buffer + head, rx_len - head);
            rx_len -= head;
            if (rx_len < 7u) return;
        }
        if (rx_len < 4) return;
        uint8_t cmd = rx_buffer[2];
        uint8_t len = rx_buffer[3];
        if (len != 44u) {
            memmove(rx_buffer, rx_buffer + 1, rx_len - 1);
            rx_len -= 1;
            continue;
        }
        uint16_t frame_len = (uint16_t)len + 7u;
        if (rx_len < frame_len) return;
        if (rx_buffer[frame_len - 2] != UART_END1 || rx_buffer[frame_len - 1] != UART_END2) {
            memmove(rx_buffer, rx_buffer + 1, rx_len - 1);
            rx_len -= 1;
            continue;
        }
        uint8_t crc_rx = rx_buffer[4 + len];
        uint8_t crc_calc = Get_CRC8_Check_Sum(rx_buffer, (uint16_t)len + 4u, 0xFF);
        if (crc_calc != crc_rx) {
            rx_crc_fail++; // 记录校验失败
            memmove(rx_buffer, rx_buffer + 1, rx_len - 1);
            rx_len -= 1;
            continue;
        }
        
        rx_packet.header1 = UART_HEADER1;
        rx_packet.header2 = UART_HEADER2;
        rx_packet.cmd = cmd;
        rx_packet.len = len;
        uint16_t index = 4;
        for (uint16_t i = 0; i < 4; i++) {
            rx_packet.wheelSpeedTarget[0].u8_temp[i] = rx_buffer[index++];
        }
        for (uint16_t i = 0; i < 4; i++) {
            rx_packet.wheelSpeedTarget[1].u8_temp[i] = rx_buffer[index++];
        }
        for (uint16_t i = 0; i < 4; i++) {
            rx_packet.clawSpeedTarget[0].u8_temp[i] = rx_buffer[index++];
        }
        for (uint16_t i = 0; i < 4; i++) {
            rx_packet.clawSpeedTarget[1].u8_temp[i] = rx_buffer[index++];
        }
        for (uint16_t i = 0; i < 4; i++) {
            rx_packet.clawAngleTarget[0].u8_temp[i] = rx_buffer[index++];
        }
        for (uint16_t i = 0; i < 4; i++) {
            rx_packet.clawAngleTarget[1].u8_temp[i] = rx_buffer[index++];
        }
        for (uint16_t i = 0; i < 4; i++) {
            rx_packet.jointSpeedTarget.u8_temp[i] = rx_buffer[index++];
        }
        for (uint16_t jointIndex = 0; jointIndex < 4; jointIndex++) {
            for (uint16_t i = 0; i < 4; i++) {
                rx_packet.jointAngleTarget[jointIndex].u8_temp[i] = rx_buffer[index++];
            }
        }
        rx_packet.crc = crc_rx;
        rx_packet.end1 = UART_END1;
        rx_packet.end2 = UART_END2;
        
        //从 rx_packet 更新 masterControlData
        masterControlData.wheelSpeedTarget[0] = rx_packet.wheelSpeedTarget[0].f_temp;
        masterControlData.wheelSpeedTarget[1] = rx_packet.wheelSpeedTarget[1].f_temp;
        masterControlData.clawSpeedTarget[0] = rx_packet.clawSpeedTarget[0].f_temp;
        masterControlData.clawSpeedTarget[1] = rx_packet.clawSpeedTarget[1].f_temp;
//        masterControlData.clawAngleTarget[0] = rx_packet.clawAngleTarget[0].f_temp;
//        masterControlData.clawAngleTarget[1] = rx_packet.clawAngleTarget[1].f_temp;
        masterControlData.jointSpeedTarget = rx_packet.jointSpeedTarget.f_temp;
        for(int i=0; i<4; i++){
            masterControlData.jointAngleTarget[i] = rx_packet.jointAngleTarget[i].f_temp;
        }
        masterControlData.count++; 
        
        rx_success++;
        memmove(rx_buffer, rx_buffer + frame_len, rx_len - frame_len);
        rx_len -= frame_len;
    }
}

static void motor_add_data(uint8_t *buffer, uint16_t *index,motor_num motor_index, uint8_t *motor_status, uint8_t motor_status_bit_pos){
    formatTrans32Struct_t motor_data;
    const motor_t *m = &motor[motor_index];
    // id
    buffer[(*index)++] = (uint8_t)(m->id & 0xFF);
    // pos
    motor_data.f_temp = m->para.pos;
    memcpy(&buffer[*index], motor_data.u8_temp, 4); *index += 4;
    // vel
    motor_data.f_temp = m->para.vel;
    memcpy(&buffer[*index], motor_data.u8_temp, 4); *index += 4;
    // tor
    motor_data.f_temp = m->para.tor;
    memcpy(&buffer[*index], motor_data.u8_temp, 4); *index += 4;

    // 状态字构建：state=0 或 1 表示正常，其他值表示异常
    *motor_status |= ((m->para.state == 0 || m->para.state == 1) ? 1 : 0) << motor_status_bit_pos;
}


volatile int test_count=0;
volatile uint8_t debug_tx_buffer[256]; // 在Watch窗口添加此变量以查看发送内容
void slave_send_packet(void){
    const uint8_t motor_count = (uint8_t)num;
    const uint8_t motor_payload_len = (uint8_t)(motor_count * 13u);
    const uint8_t imu_payload_len = 10u * 4u;
    const uint8_t status_payload_len = 3u; // 电机状态1 + IMU状态1 + 夹爪状态1
    const uint8_t payload_len = (uint8_t)(motor_payload_len + imu_payload_len + status_payload_len);
    const imuDataStruct_t *imu = get_imu_data();
    
    static uint8_t tx_buf[256];
    // 状态字构建：电机状态字 + IMU状态字 + 夹爪状态字
    uint8_t motor_status = 0u; 

    uint8_t imu_status = 0u;

    uint8_t claw_status = 0u; // 0表示夹爪未动作，1表示夹爪正在动作（可以根据实际情况调整定义）

    uint16_t index = 0;
    // Clear buffer to ensure no garbage data
    memset(tx_buf, 0, 256);
    
    tx_buf[index++] = UART_HEADER1;
    tx_buf[index++] = UART_HEADER2;
    tx_buf[index++] = 0x01;
    tx_buf[index++] = payload_len;
    /*电机部分*/
    // for (uint8_t i = 0; i < motor_count; i++) {
    //     formatTrans32Struct_t motor_data;
    //     const motor_t *m = &motor[i];
    //     // id
    //     tx_buf[index++] = (uint8_t)(m->id & 0xFF);
    //     // pos
    //     // motor_data.f_temp = (float)m->para.p_int;
    
    //     // motor_data.f_temp = motor_to_mechanism_rad(i, m->para.pos);
    //     if (i == Motor3 || i == Motor7) {
    //         motor_data.f_temp = motor_to_mechanism_rad(i, m->pos_track.theta_total_rad);
    //     }
    //     else if(i == Motor4 || i == Motor8)
    //     {
    //         motor_data.f_temp = motor_to_mechanism_position(i, m->pos_track.theta_total_rad); 
    //     }
    //     else 
    //     {
    //         motor_data.f_temp = m->para.pos;
    //     }
        
    //     memcpy(&tx_buf[index], motor_data.u8_temp, 4); index += 4;
    //     // vel
    //     //motor_data.f_temp = (float)m->para.v_int;
    //     motor_data.f_temp = m->para.vel;
    //     // motor_data.f_temp = motor_to_mechanism_radps(i, motor_data.f_temp);
    //     memcpy(&tx_buf[index], motor_data.u8_temp, 4); index += 4;
    //     // tor
    //     //motor_data.f_temp = (float)m->para.t_int;
    //     motor_data.f_temp = m->para.tor;
    //     memcpy(&tx_buf[index], motor_data.u8_temp, 4); index += 4;
        
    //     //  motor_data.f_temp = 1.0;
    //     // memcpy(&tx_buf[index], motor_data.u8_temp, 4); index += 4;
    //     // // vel
    //     // //motor_data.f_temp = (float)m->para.v_int;
    //     // motor_data.f_temp = 2.0;
    //     // memcpy(&tx_buf[index], motor_data.u8_temp, 4); index += 4;
    //     // // tor
    //     // //motor_data.f_temp = (float)m->para.t_int;
    //     // motor_data.f_temp = 3.0;
    //     // memcpy(&tx_buf[index], motor_data.u8_temp, 4); index += 4;

    //     // 状态字构建：state=0 或 1 表示正常，其他值表示异常
    //     motor_status |= ((m->para.state == 0 || m->para.state == 1) ? 1 : 0) << i; 

    // }

    motor_add_data(tx_buf, &index, Motor1, &motor_status, 0);
    motor_add_data(tx_buf, &index, Motor2, &motor_status, 2);
    motor_add_data(tx_buf, &index, Motor3, &motor_status, 4);
    motor_add_data(tx_buf, &index, Motor4, &motor_status, 6);
    motor_add_data(tx_buf, &index, Motor5, &motor_status, 3);
    motor_add_data(tx_buf, &index, Motor6, &motor_status, 1);
    motor_add_data(tx_buf, &index, Motor7, &motor_status, 5);
    motor_add_data(tx_buf, &index, Motor8, &motor_status, 7);


    /* 加速度计数据 */
    for (uint8_t i = 0; i < 3u; i++)
    {
        memcpy(&tx_buf[index], imu->acc[i].u8_temp, 4u);
        index += 4u;
    }
    /* 角速度数据 */
    for (uint8_t i = 0; i < 3u; i++)
    {
        memcpy(&tx_buf[index], imu->gyr[i].u8_temp, 4u);
        index += 4u;
    }



    /* IMU部分仅发送四元数 w/x/y/z */
    for (uint8_t i = 0; i < 4u; i++)
    {
        memcpy(&tx_buf[index], imu->quat[i].u8_temp, 4u);
        index += 4u;
    }

    /* 电机状态字,表示8个电机的状态 1表示正常 0表示异常 */
    
    tx_buf[index++] = motor_status; 

    static uint32_t last_imu_count = 0u;

    if(imu->count != last_imu_count)
    {
        imu_status |= 0x01; // IMU数据更新标志
        last_imu_count = imu->count;
    }
    else
    {
        imu_status = 0u; // IMU数据未更新
    }
    tx_buf[index++] = imu_status; 

    /* 夹爪状态字,根据实际情况设置 */
    claw_status = 1u; 
    tx_buf[index++] = claw_status; 


    tx_buf[index++] = Get_CRC8_Check_Sum(tx_buf, (uint16_t)payload_len + 4u, 0xFF);

    tx_buf[index++] = UART_END1;
    tx_buf[index++] = UART_END2;

    // 调试
    for(int k=0; k<index; k++){
        debug_tx_buffer[k] = tx_buf[k];
    }  

    (void)HAL_UART_Transmit_DMA(&huart1, tx_buf, index);

	test_count++; //测试函数调用次数
}


// Initialize slave communication
void slave_init(void){
    rx_len = 0;
    slaveRxStream = xStreamBufferCreateStatic(
        SLAVE_RX_STREAM_SIZE,
        1,
        slaveRxStreamStorage,
        &slaveRxStreamStruct
    );
    configASSERT(slaveRxStream != NULL);
    /* 接收启动放到解析任务里做，避免系统未调度前先收到数据。 */
}

void nuc_update(void const * argument){
    for(;;){
        // osDelay(200);
        
        // osDelay(100);  //10hz
        osDelay(20); 
        slave_send_packet();
        
        // 不需要进行模式检测，因为选择控制数据源在 control.c 中已经处理过了
        // 直接更新 masterControlData 即可
        // if(robotMode == MODE_AUTO && get_remote_control_data()->rcSbus.CH4 == RCSW_TOP){
            // masterControlData.wheelSpeedTarget[0] = rx_packet.wheelSpeedTarget[0].f_temp;
            // masterControlData.wheelSpeedTarget[1] = rx_packet.wheelSpeedTarget[1].f_temp;
            // masterControlData.clawSpeedTarget[0] = rx_packet.clawSpeedTarget[0].f_temp;
            // masterControlData.clawSpeedTarget[1] = rx_packet.clawSpeedTarget[1].f_temp;
            // masterControlData.clawAngleTarget[0] = rx_packet.clawAngleTarget[0].f_temp;
            // masterControlData.clawAngleTarget[1] = rx_packet.clawAngleTarget[1].f_temp;
            // masterControlData.jointSpeedTarget = rx_packet.jointSpeedTarget.f_temp;
            // for(uint8_t i = 0; i < 4; i++){
            //     masterControlData.jointAngleTarget[i] = rx_packet.jointAngleTarget[i].f_temp;
            // }
        // }
    }
}

void slaveData_update(void const * argument)
{
    uint8_t chunk[64];
    size_t n = 0;
    /* 任务准备好后再启动接收，确保收到的数据能立即被消费。 */
    usart1_rx_dma_init();
    for (;;) {
        n = xStreamBufferReceive(slaveRxStream, chunk, sizeof(chunk), pdMS_TO_TICKS(20));
        if (n > 0) {
            slave_receive_handler(chunk, (uint16_t)n);
        }

        /* 若 ISR 发现 HAL 状态异常，这里负责真正执行恢复。 */
        if (slave_rx_restart_req != 0u) {
            slave_service_rx_restart();
        }
    }
}


