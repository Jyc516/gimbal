//
// Created by JiangYC on 2025/12/6.
//
#include "imu.h"
#include "bmi088.h"

// int16_t raw_data;      //for debugging

float IMU::accel_int8_to_mps(uint8_t raw_msb, uint8_t raw_lsb, int accel_range) {
    int16_t raw_data = int(raw_msb) * 256 + int(raw_lsb);
    return float(raw_data) / 32768 * accel_range * g;
}

float IMU::gyro_int8_to_dps(uint8_t raw_msb, uint8_t raw_lsb, float gyro_res){
    int16_t raw_data = int(raw_msb) * 256 + int(raw_lsb);
    return gyro_res * raw_data / 1000;
}

void IMU::accel_calculate(){
    bmi088_accel_read_reg(0x41, accel_raw_range, 1);
    switch (accel_raw_range[1]) {
        case 0x00: accel_range = 3; break;
        case 0x01: accel_range = 6; break;
        case 0x02: accel_range = 12; break;
        case 0x03: accel_range = 24; break;
        default: return;
    }
    bmi088_accel_read_reg(0x12, accel_raw_data, 6);
    for (int i=0; i<3; ++i) {
        accel_data[i] = accel_int8_to_mps(accel_raw_data[i*2+2], accel_raw_data[i*2+1], accel_range);
    }
}

void IMU::gyro_calculate() {
    bmi088_gyro_read_reg(0x0F, &gyro_raw_range, 1);
    switch (gyro_raw_range) {
        case 0x00: gyro_res = 61.0; break;
        case 0x01: gyro_res = 30.5; break;
        case 0x02: gyro_res = 15.3; break;
        case 0x03: gyro_res = 7.6; break;
        case 0x04: gyro_res = 3.8; break;
        default: return;
    }
    bmi088_gyro_read_reg(0x02, gyro_raw_data, 6);
    for (int i=0; i<3; ++i) {
        gyro_data[i] += gyro_int8_to_dps(gyro_raw_data[i*2+1], gyro_raw_data[i*2], gyro_res);
    }
    if (++cnt == 5) {
        for (int i=0; i<3; ++i) {
            filtered_gyro_data[i] = gyro_data[i] / 5;
            gyro_data[i] = 0.f;
        }
        cnt = 0;
    }
}

IMU imu;