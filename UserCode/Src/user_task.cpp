//
// Created by JiangYC on 2025/11/23.
//

#include "user_task.h"
#include "cmsis_os2.h"
#include "iwdg.h"
#include "RemoteController.h"
#include "imu.h"
#include "motor.h"
#include "can.h"
#include "usart.h"

extern RC rc;

extern IMU imu;

extern GM6020Motor pitch_motor;
extern GM6020Motor yaw_motor;
extern CAN_TxHeaderTypeDef tx_header;
extern uint32_t can_tx_mailbox;
extern uint8_t yaw_buffer[8];
extern uint8_t pitch_buffer[8];
uint8_t tx_data[8] = {0};

// canIRQ -> yaw_can
osSemaphoreId_t yaw_semaphore_handle;
constexpr osSemaphoreAttr_t yaw_semaphore_attr{.name = "yaw_semaphore",};

// canIRQ -> pitch_can
osSemaphoreId_t pitch_semaphore_handle;
constexpr osSemaphoreAttr_t pitch_semaphore_attr{.name = "pitch_semaphore",};

// dmaIRQ -> rc
osSemaphoreId_t rc_semaphore_handle;
constexpr osSemaphoreAttr_t rc_semaphore_attr{.name = "rc_semaphore",};

// -> yaw_motor
constexpr auto yaw_can_flag = 1 << 0;
constexpr auto yaw_rc_flag = 1 << 1;
osEventFlagsId_t yaw_flag_handle;
constexpr osEventFlagsAttr_t yaw_flag_attr{.name = "yaw_flag",};

// -> pitch_motor
constexpr auto pitch_can_flag = 1 << 0;
constexpr auto pitch_rc_flag = 1 << 1;
osEventFlagsId_t pitch_flag_handle;
constexpr osEventFlagsAttr_t pitch_flag_attr{.name = "pitch_flag",};

// -> control
constexpr auto yaw_flag = 1 << 0;
constexpr auto pitch_flag = 1 << 1;
osEventFlagsId_t control_flag_handle;
constexpr osEventFlagsAttr_t control_flag_attr{.name = "control_flag",};


osThreadId_t imu_task_handle;
constexpr osThreadAttr_t imu_task_attr{
    .name = "imu_task",
    .stack_size = 256 * 4,
    .priority = osPriorityNormal,
};

[[noreturn]] void imu_task(void *) {
    while (true) {
        const int ticks = osKernelGetTickCount();
        imu.accel_calculate();
        imu.gyro_calculate();
        osDelayUntil(ticks + 10);
    }
}


osThreadId_t yaw_can_task_handle;
constexpr osThreadAttr_t yaw_can_task_attr{
    .name = "yaw_can_task",
    .stack_size = 256 * 4,
    .priority = osPriorityNormal,
};

[[noreturn]] void yaw_can_task(void *) {
    while (true) {
        osSemaphoreAcquire(yaw_semaphore_handle, osWaitForever);
        yaw_motor.read_RxMsg(yaw_buffer);
        osEventFlagsSet(yaw_flag_handle, yaw_can_flag);
    }
}


osThreadId_t pitch_can_task_handle;
constexpr osThreadAttr_t pitch_can_task_attr{
    .name = "pitch_can_task",
    .stack_size = 256 * 4,
    .priority = osPriorityNormal,
};

[[noreturn]] void pitch_can_task(void *) {
    while (true) {
        osSemaphoreAcquire(pitch_semaphore_handle, osWaitForever);
        pitch_motor.read_RxMsg(pitch_buffer);
        osEventFlagsSet(pitch_flag_handle, pitch_can_flag);
    }
}


osThreadId_t rc_task_handle;
constexpr osThreadAttr_t rc_task_attr{
    .name = "rc_task",
    .stack_size = 256 * 4,
    .priority = osPriorityNormal,
};

[[noreturn]] void rc_task(void *) {
    while (true) {
        rc.handle(osKernelGetTickCount());
        osEventFlagsSet(yaw_flag_handle, yaw_rc_flag);
        osEventFlagsSet(pitch_flag_handle, pitch_rc_flag);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc.rx_buf, 18);
    }
}


osThreadId_t yaw_motor_task_handle;
constexpr osThreadAttr_t yaw_motor_task_attr{
    .name = "yaw_motor_task",
    .stack_size = 256 * 4,
    .priority = osPriorityNormal,
};

[[noreturn]] void yaw_motor_task(void *) {
    while (true) {
        osEventFlagsWait(yaw_flag_handle, yaw_rc_flag | yaw_can_flag, osFlagsWaitAll, osWaitForever);
        yaw_motor.SetAngle(rc.CH0 * 30, 0, 0);
        yaw_motor.handle();
        yaw_motor.write_TxMsg(tx_data);
        osEventFlagsSet(control_flag_handle, yaw_flag);
    }
}


osThreadId_t pitch_motor_task_handle;
constexpr osThreadAttr_t pitch_motor_task_attr{
    .name = "pitch_motor_task",
    .stack_size = 256 * 4,
    .priority = osPriorityNormal,
};

[[noreturn]] void pitch_motor_task(void *) {
    while (true) {
        osEventFlagsWait(pitch_flag_handle, pitch_rc_flag | pitch_can_flag, osFlagsWaitAll, osWaitForever);
        pitch_motor.SetAngle(rc.CH1 * 30, 0, 0);
        pitch_motor.handle();
        pitch_motor.write_TxMsg(tx_data);
        osEventFlagsSet(control_flag_handle, pitch_flag);
    }
}


osThreadId_t control_task_handle;
constexpr osThreadAttr_t control_task_attr{
    .name = "control_task",
    .stack_size = 256 * 4,
    .priority = osPriorityNormal,
};

[[noreturn]] void control_task(void *) {
    while (true) {
        HAL_IWDG_Refresh(&hiwdg);
        if (rc.S2 == RC::DOWN) {
            for (int i = 0; i < 8; ++i) {
                tx_data[i] = 0;
            }
            HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mailbox);
        }
        else if (rc.S2 == RC::UP) {
            osEventFlagsWait(control_flag_handle, pitch_flag | yaw_flag, osFlagsWaitAny, osWaitForever);
            HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mailbox);
        }
    }
}



void user_task_init() {
    imu_task_handle = osThreadNew(imu_task, nullptr, &imu_task_attr);
    yaw_can_task_handle = osThreadNew(yaw_can_task, nullptr, &yaw_can_task_attr);
    pitch_can_task_handle = osThreadNew(pitch_can_task, nullptr, &pitch_can_task_attr);
    yaw_motor_task_handle = osThreadNew(yaw_motor_task, nullptr, &yaw_motor_task_attr);
    pitch_motor_task_handle = osThreadNew(pitch_motor_task, nullptr, &pitch_motor_task_attr);
    control_task_handle = osThreadNew(control_task, nullptr, &control_task_attr);

    yaw_semaphore_handle = osSemaphoreNew(1, 0, &yaw_semaphore_attr);
    pitch_semaphore_handle = osSemaphoreNew(1, 0, &pitch_semaphore_attr);
    rc_semaphore_handle = osSemaphoreNew(1, 0, &rc_semaphore_attr);
    yaw_flag_handle = osEventFlagsNew(&yaw_flag_attr);
    pitch_flag_handle = osEventFlagsNew(&pitch_flag_attr);
    control_flag_handle = osEventFlagsNew(&control_flag_attr);
}