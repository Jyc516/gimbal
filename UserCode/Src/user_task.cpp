//
// Created by JiangYC on 2025/11/27.
//

#include "user_task.h"
#include "cmsis_os2.h"
#include "can.h"
#include "motor.h"

int stop_flag = 1; // 置1停止
extern GM6020Motor yaw_motor;   // id 1
extern GM6020Motor pitch_motor; // id 4
extern CAN_TxHeaderTypeDef tx_header;
extern uint32_t can_tx_mailbox;
uint8_t tx_data[8] = {0};

osThreadId_t yaw_task_handle;
osThreadAttr_t yaw_task_attr{
    .name = "yaw_task_attr",
    .stack_size = 256 * 4,
    .priority = osPriorityNormal,
};


[[noreturn]] void yaw_task(void *) {
    while (true) {
        if (stop_flag == 0) {
            yaw_motor.handle();
            yaw_motor.write_TxMsg(tx_data);
        }
        else{
            for (int i = 0; i < 8; ++i) {
                tx_data[i] = {0};
            }
        }
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mailbox);
        osDelay(1);
    }
}


osThreadId_t pitch_task_handle;
osThreadAttr_t pitch_task_attr{
    .name = "yaw_task_attr",
    .stack_size = 256 * 4,
    .priority = osPriorityNormal,
};


[[noreturn]] void pitch_task(void *) {
    while (true) {
        if (stop_flag == 0) {
            pitch_motor.handle();
            pitch_motor.write_TxMsg(tx_data);
        }
        else{
            for (int i = 0; i < 8; ++i) {
                tx_data[i] = {0};
            }
        }
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mailbox);
        osDelay(1);
    }
}


osThreadId_t motor_task_handle;
osThreadAttr_t motor_task_attr{
    .name = "motor_task_attr",
    .stack_size = 256 * 4,
    .priority = osPriorityNormal,
};


[[noreturn]] void motor_task(void *) {
    while (true) {
        if (stop_flag == 0) {
            yaw_motor.handle();
            yaw_motor.write_TxMsg(tx_data);
            pitch_motor.handle();
            pitch_motor.write_TxMsg(tx_data);
        }
        else{
            for (int i = 0; i < 8; ++i) {
                tx_data[i] = {0};
            }
        }
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mailbox);
        osDelay(1);
    }
}

void user_task_init() {
    // yaw_task_handle = osThreadNew(yaw_task, nullptr, &yaw_task_attr);
    // pitch_task_handle = osThreadNew(pitch_task, nullptr, &pitch_task_attr);
    motor_task_handle = osThreadNew(motor_task, nullptr, &motor_task_attr);
}