//
// Created by JiangYC on 2025/12/6.
//

#ifndef GIMBAL_IMU_H
#define GIMBAL_IMU_H

#include <stdint.h>

// #ifdef __cplusplus
// extern "C"{
// #endif

class IMU {
private:
    uint8_t accel_raw_range[2];
    uint8_t accel_raw_data[7];
    int accel_range;          // gps
    float accel_data[3];      // x y z mpss

    uint8_t gyro_raw_range;
    uint8_t gyro_raw_data[6];
    float gyro_res;           // mdps
    float gyro_data[3];       // x y z dps
    float filtered_gyro_data[3];   // oversample 5 times

    int cnt;                  // oversample counter

    static float accel_int8_to_mps(uint8_t raw_msb, uint8_t raw_lsb, int accel_range);
    static float gyro_int8_to_dps(uint8_t raw_msb, uint8_t raw_lsb, float gyro_res);

    static constexpr float g = 9.8;     // mpss

public:
    IMU() {
        cnt = 0;
        gyro_data[0] = gyro_data[1] = gyro_data[2] = 0.f;
    };
    ~IMU() = default;

    void accel_calculate();
    void gyro_calculate();
};

// #ifdef __cplusplus
// }
// #endif

#endif //GIMBAL_IMU_H
