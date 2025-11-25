//
// Created by JiangYC on 2025/11/23.
//

#include "user_task.h"
#include "cmsis_os2.h"
#include "iwdg.h"
#include "RemoteController.h"
#include "imu.h"

extern RC rc;
extern IMU imu;

osThreadId_t control_task_handle;
constexpr osThreadAttr_t control_task_attr{
    .name = "control_task",
    .stack_size = 256 * 4,
    .priority = osPriorityNormal,
};

[[noreturn]] void control_task(void *) {
    while (true) {
        HAL_IWDG_Refresh(&hiwdg);
        if (rc.S1 == RC::DOWN) {

        }
        else if (rc.S1 == RC::UP) {

        }
    }
}


osThreadId_t imu_task_handle;
constexpr osThreadAttr_t imu_task_attr{
    .name = "imu_task",
    .stack_size = 256 * 4,
    .priority = osPriorityNormal,
};

[[noreturn]] void imu_task(void *) {
    while (true) {
        imu.accel_calculate();
        imu.gyro_calculate();
        osDelay(1000);
    }
}


osThreadId_t dma_task_handle;
constexpr osThreadAttr_t dma_task_attr{
    .name = "dma_task",
    .stack_size = 256 * 4,
    .priority = osPriorityNormal,
};

[[noreturn]] void dma_task(void *) {
    while (true) {

    }
}


osThreadId_t can_task_handle;
constexpr osThreadAttr_t can_task_attr{
    .name = "can_task",
    .stack_size = 256 * 4,
    .priority = osPriorityNormal,
};

[[noreturn]] void can_task(void *) {
    while (true) {

    }
}


void user_task_init() {
    control_task_handle = osThreadNew(control_task, nullptr, &control_task_attr);
    imu_task_handle = osThreadNew(imu_task, nullptr, &imu_task_attr);
    dma_task_handle = osThreadNew(dma_task, nullptr, &dma_task_attr);
    can_task_handle = osThreadNew(can_task, nullptr, &can_task_attr);


}