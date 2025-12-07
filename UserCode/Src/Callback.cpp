//
// Created by JiangYC on 2025/11/27.
//

#include "can.h"
#include "motor.h"

#include "cmsis_os2.h"
#include "usart.h"
#include "RemoteController.h"

extern CAN_RxHeaderTypeDef rx_header;
uint8_t rx_data[8] = {0};
extern GM6020Motor yaw_motor;   // id 1
extern GM6020Motor pitch_motor; // id 4

extern RC rc;
extern osSemaphoreId_t rc_semaphore_handle;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == hcan1.Instance) {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
        if (rx_header.StdId == 0x205) {
            yaw_motor.read_RxMsg(rx_data);
        }
        else if (rx_header.StdId == 0x208) {
            pitch_motor.read_RxMsg(rx_data);
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if (huart == &huart3) {
        osSemaphoreRelease(rc_semaphore_handle);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc.rx_buf, 18);
    }
}