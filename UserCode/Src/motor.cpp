//
// Created by JiangYC on 2025/11/27.
//

#include "Motor.h"

#include <cmath>
#include <algorithm>

float GM6020Motor::normalize_angle(float angle, const bool has_direction) {
    if (!has_direction) {
        while (angle >= 360.f) {
            angle -= 360.f;
        }
        while (angle < 0.f) {
            angle += 360.f;
        }
    }
    else {
        while (angle >= 180.f) {
            angle -= 360.f;
        }
        while (angle < -180.f) {
            angle += 360.f;
        }
    }
    return angle;
}

void GM6020Motor::read_RxMsg(const uint8_t rx_data[]) {
    last_ecd_angle = ecd_angle;
    uint16_t rx_ecd = rx_data[0] <<8 | rx_data[1];
    ecd_angle = linear_mapping(rx_ecd, rx_ecd_angle, res_ecd_angle);
    delta_ecd_angle = normalize_angle(ecd_angle - last_ecd_angle, true);
    delta_angle = delta_ecd_angle / ratio;
    angle = normalize_angle(angle + delta_angle);

    int16_t rx_speed = rx_data[2] << 8 | rx_data[3];
    rotate_speed = linear_mapping(rx_speed, rx_rotate_speed, res_rotate_speed);
    rotate_speed /= ratio;

    int16_t rx_current = rx_data[4] << 8 | rx_data[5];
    current = linear_mapping(rx_current, rx_max_current, res_max_current);

    temperature = rx_data[6];
}

void GM6020Motor::write_TxMsg(uint8_t tx_data[8]){
    int i = (id - 1) % 4 * 2;
    int16_t output = static_cast<int16_t>(output_intensity);
    tx_data[i] = output >> 8;
    tx_data[i+1] = output & 0xFF;
}

void GM6020Motor::SetIntensity(const float intensity){
    mode = TORQUE;

    output_intensity = intensity;
}

void GM6020Motor::SetSpeed(const float tgt_speed_, const float ff_intensity_){
    mode = SPEED;

    tgt_speed = tgt_speed_;
    ff_intensity = ff_intensity_;
}

void GM6020Motor::SetAngle(const float tgt_angle_, float ff_speed_, const float ff_intensity_){
    // 使用者须确保tgt_angle_是正确且被归一化到0-360°的

    mode = POSITION_SPEED;

    tgt_angle = tgt_angle_;
    ff_intensity = ff_intensity_;
    // ff_speed = ff_speed_;
}

void GM6020Motor::calc_ff_intensity(){
    // 原pid教具残留代码
    // float rad_angle = linear_mapping(fdb_angle, 180.f, 3.1415926f);   // deg -> rad
    // float torque = 0.5 * 9.8 * sin(rad_angle) * 0.05;
    // // float ff_current = linear_mapping(torque, 3.f, 8.f, 1.f, 3.f);
    // float ff_current = linear_mapping(torque, 3.f, 8.2f);
    // ff_intensity = linear_mapping(ff_current, 20.f, 16384.f);

    // pitch轴前馈多项式拟合
    ff_intensity = -3.25385e-4 * pow(fdb_angle, 4)
                   - 0.03948 * pow(fdb_angle, 3)
                   - 0.19133 * pow(fdb_angle, 2)
                   - 10.11905 * fdb_angle + 2000;
}

void GM6020Motor::handle(){
    // fdb_speed = rotate_speed;
    fdb_speed = fdb_speed * 0.4 + rotate_speed * 0.6;                        // 一阶滤波，GM6020速度反馈噪声太大了
    fdb_angle = normalize_angle(angle - init_angle, true);        // 设置角度0点
    if (!const_ff_intensity) {
        calc_ff_intensity();
    }

    if (mode == TORQUE) {
        output_intensity = ff_intensity;
    }
    else if (mode == SPEED) {
        // // 用于调试pitch_spid
        // if (fdb_angle <= -20.f) {
        //     output_intensity = ff_intensity;
        // }
        // else {
        //     output_intensity = ff_intensity + spid.calc(tgt_speed, fdb_speed);
        // }

        output_intensity = ff_intensity + spid.calc(tgt_speed, fdb_speed);
    }
    else if (mode == POSITION_SPEED) {
        tgt_speed = ff_speed + ppid.calc(tgt_angle, fdb_angle);
        output_intensity = ff_intensity + spid.calc(tgt_speed, fdb_speed);
    }
}

PID yaw_ppid(72, 1, 1000, 30, 100, 1080, 0.1);
// PID yaw_spid(70, 40, 60, 40, 100, 10000, 0.1);
PID yaw_spid(34, 60, 20, 40, 50, 12000, 0.1);
GM6020Motor yaw_motor(1, 1, yaw_ppid, yaw_spid, 0, 0);

PID pitch_ppid(45, 5, 960, 0, 100, 0, 0.09);
PID pitch_spid(12, 40, 0, 40, 50, 15000, 0.1);
GM6020Motor pitch_motor(1, 4, pitch_ppid, pitch_spid, -1, 142);
// 114 - 142 - 170