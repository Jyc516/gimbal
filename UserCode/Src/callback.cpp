//
// Created by JiangYC on 2025/11/23.
//

#include "usart.h"
#include "RemoteController.h"

#include "can.h"
#include "motor.h"

#include "cmsis_os2.h"


extern RC rc;

extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
uint8_t rx_buffer[8];
uint8_t yaw_buffer[8];
uint8_t pitch_buffer[8];
extern uint32_t can_tx_mailbox;
extern GM6020Motor yaw_motor;       // id 1
extern GM6020Motor pitch_motor;     // id 4

extern osSemaphoreId_t yaw_semaphore_handle;
extern osSemaphoreId_t pitch_semaphore_handle;
extern osSemaphoreId_t rc_semaphore_handle;




void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if (huart == &huart3) {
        osSemaphoreRelease(rc_semaphore_handle);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_buffer);
        if (rx_header.StdId == 0x205) {
            if (osSemaphoreGetCount(yaw_semaphore_handle) == 0) {
                for (int i = 0; i < 8; ++i) {
                    yaw_buffer[i] = rx_buffer[i];
                }
                osSemaphoreRelease(yaw_semaphore_handle);
            }
        }
        else if (rx_header.StdId == 0x208) {
            if (osSemaphoreGetCount(pitch_semaphore_handle) == 0) {
                for (int i = 0; i < 8; ++i) {
                    pitch_buffer[i] = rx_buffer[i];
                }
                osSemaphoreRelease(pitch_semaphore_handle);
            }
        }
    }
}