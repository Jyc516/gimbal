//
// Created by JiangYC on 2025/11/23.
//

#include "usart.h"
#include "RemoteController.h"
#include "tim.h"


extern RC rc;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if (huart == &huart3) {
        const int ticks = HAL_GetTick();
        rc.handle(ticks);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc.rx_buf, 18);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim6) {

    }
}