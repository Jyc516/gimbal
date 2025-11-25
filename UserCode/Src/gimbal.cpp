//
// Created by JiangYC on 2025/11/24.
//

#include "gimbal.h"

void gimbal::imu_calculate(){
    imu.accel_calculate();
    imu.gyro_calculate();
}


