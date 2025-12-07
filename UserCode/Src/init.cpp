//
// Created by JiangYC on 2025/12/7.
//

#include "init.h"
#include "RemoteController.h"
#include "usart.h"

extern RC rc;

void rc_init() {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc.rx_buf, 18);
}