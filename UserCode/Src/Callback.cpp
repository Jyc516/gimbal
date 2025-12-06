//
// Created by JiangYC on 2025/11/27.
//

#include "can.h"
#include "motor.h"

extern CAN_RxHeaderTypeDef rx_header;
uint8_t rx_data[8] = {0};
extern GM6020Motor yaw_motor;   // id 1
extern GM6020Motor pitch_motor; // id 4

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