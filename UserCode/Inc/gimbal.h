//
// Created by JiangYC on 2025/11/24.
//

#ifndef GIMBAL_GIMBAL_H
#define GIMBAL_GIMBAL_H

#include "imu.h"

class gimbal {
private:
    IMU imu;

public:
    gimbal();
    ~gimbal();

    void imu_calculate();
};

#endif //GIMBAL_GIMBAL_H
