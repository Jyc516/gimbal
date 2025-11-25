//
// Created by JiangYC on 2025/11/25.
//

#include "motor.h"
#include "PID.h"

#include <cmath>
#include <algorithm>

float Motor::normalize_angle(float angle, const bool has_direction){
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

void Motor::SetIntensity(const float intensity){
    mode = TORQUE;

    output_intensity = intensity;
}

void Motor::SetSpeed(const float tgt_speed_, const float ff_intensity_){
    mode = SPEED;

    tgt_speed = tgt_speed_;
    ff_intensity = ff_intensity_;
}

void Motor::SetAngle(const float tgt_angle_, float ff_speed_, const float ff_intensity_){
    mode = POSITION_SPEED;

    tgt_angle = tgt_angle_;
    ff_intensity = ff_intensity_;
    // ff_speed = ff_speed_;
}


void GM6020Motor::read_RxMsg(const uint8_t rx_data[8]){
    last_ecd_angle = ecd_angle;
    const uint16_t rx_ecd = rx_data[0] <<8 | rx_data[1];
    ecd_angle = linear_mapping(rx_ecd, rx_ecd_angle, res_ecd_angle);
    delta_ecd_angle = normalize_angle(ecd_angle - last_ecd_angle, true);
    delta_angle = delta_ecd_angle / ratio;
    angle = normalize_angle(angle + delta_angle);

    const int16_t rx_speed = rx_data[2] << 8 | rx_data[3];
    rotate_speed = linear_mapping(rx_speed, rx_rotate_speed, res_rotate_speed);
    rotate_speed /= ratio;

    const int16_t rx_current = rx_data[4] << 8 | rx_data[5];
    current = linear_mapping(rx_current, rx_max_current, res_max_current);

    temperature = rx_data[6];
}

void GM6020Motor::write_TxMsg(uint8_t tx_data[8]) {
    const int i = (id - 1) % 4 * 2;
    const int16_t output = static_cast<int16_t>(output_intensity);
    tx_data[i] = output >> 8;
    tx_data[i+1] = output & 0xFF;
}

void GM6020Motor::calculate_ff_intensity() {
    if (ff_intensity_flag) {
        const float rad_angle = linear_mapping(fdb_angle, 180.f, 3.1415926f);   // deg -> rad
        const float torque = 0.5 * 9.8 * sin(rad_angle) * 0.05;
        const float ff_current = torque / torque_current_ratio;
        ff_intensity = linear_mapping(ff_current, res_max_current, 16384.f);
    }
}

void GM6020Motor::handle(){
    fdb_speed = rotate_speed;
    fdb_angle = normalize_angle(fdb_angle, true);
    calculate_ff_intensity();

    if (mode == TORQUE) {}
    else if (mode == SPEED) {
        output_intensity = ff_intensity + spid.calc(tgt_speed, fdb_speed);
    }
    else if (mode == POSITION_SPEED) {
        tgt_speed = ff_speed + ppid.calc(tgt_angle, fdb_angle);
        output_intensity = ff_intensity + spid.calc(tgt_speed, fdb_speed);
    }
}


void M3508Motor::read_RxMsg(const uint8_t rx_data[8]){
    last_ecd_angle = ecd_angle;
    const uint16_t rx_ecd = rx_data[0] <<8 | rx_data[1];
    ecd_angle = linear_mapping(rx_ecd, rx_ecd_angle, res_ecd_angle);
    delta_ecd_angle = normalize_angle(ecd_angle - last_ecd_angle, true);
    delta_angle = delta_ecd_angle / ratio;
    angle = normalize_angle(angle + delta_angle);

    const int16_t rx_speed = rx_data[2] << 8 | rx_data[3];
    rotate_speed = linear_mapping(rx_speed, rx_rotate_speed, res_rotate_speed);
    rotate_speed /= ratio;

    const int16_t rx_current = rx_data[4] << 8 | rx_data[5];
    current = linear_mapping(rx_current, rx_max_current, res_max_current);

    temperature = rx_data[6];
}

void M3508Motor::write_TxMsg(uint8_t tx_data[8]) {
    const int i = (id - 1) % 4 * 2;
    const int16_t output = static_cast<int16_t>(output_intensity);
    tx_data[i] = output >> 8;
    tx_data[i+1] = output & 0xFF;
}

void M3508Motor::calculate_ff_intensity() {

}

void M3508Motor::handle() {

}


PID pitch_ppid(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
PID pitch_spid(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
GM6020Motor pitch_motor(1, 4, pitch_ppid, pitch_spid);

PID yaw_ppid(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
PID yaw_spid(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
GM6020Motor yaw_motor(1, 4, yaw_ppid, yaw_spid);