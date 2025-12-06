//
// Created by JiangYC on 2025/11/27.
//

#ifndef GIMBAL_MOTOR_H
#define GIMBAL_MOTOR_H

#include <stdint.h>
#include "PID.h"

class GM6020Motor {
private:
    const float ratio;                  //电机减速比
    const int id;                       // CAN ID 1-8
    float init_angle;                   // 用于设置0点

    float angle = 0.f;                  // deg 输出端累计转动角度
    float delta_angle = 0.f;            // deg 输出端新转动角度
    float ecd_angle = 0.f;              // deg 当前编码器转动角度
    float last_ecd_angle = 0.f;         // deg 上次编码器转动角度
    float delta_ecd_angle = 0.f;        // deg 编码器端新转动角度
    float rotate_speed = 0.f;           // dps 反馈转子速度
    float current = 0.f;                // A 反馈转矩电流
    float temperature = 0.f;            // °C 反馈电机温度

    static constexpr uint16_t rx_ecd_angle = 8191;
    static constexpr float res_ecd_angle = 360.f;     // deg
    static constexpr int16_t rx_rotate_speed = 1;     // rpm
    static constexpr float res_rotate_speed = 6.f;    // dps
    static constexpr int16_t rx_max_current = 16384;
    static constexpr float res_max_current = 20.f;    // A

    PID ppid; // position PID
    PID spid; // speed PID
    float tgt_angle = 0.f;
    float fdb_angle = 0.f;
    float tgt_speed = 0.f;
    float fdb_speed = 0.f;
    float output_intensity = 0.f;       // 扭矩输出接口
    const float ff_speed = 0.f;         // 前馈，设置目标速度？ff_speed暂时没用，置为常数0，SetAngle有注释化
    float ff_intensity = 0.f;           // 解算设置前馈力矩
    const bool const_ff_intensity;      // 常量前馈标志

    enum ControlMode {
        TORQUE,                 // 开环力矩控制
        SPEED,                  // 速度闭环控制
        POSITION_SPEED          // 位置速度双环控制
    };
    ControlMode mode = TORQUE;

    template<class T1, class T2>
    static T2 linear_mapping(T1 org, T1 org_max, T2 res_max, T1 org_min=0, T2 res_min=0) {
        return T2((org - org_min) * (res_max - res_min) / (org_max - org_min) + res_min);
    }

    static float normalize_angle(float angle, bool has_direction=false);

    void calc_ff_intensity();
public:
    explicit GM6020Motor(float ratio_, int id_, const PID& ppid_, const PID& spid_, float ff_intensity_, float init_angle_=0.f):
    // ff_intensity_ 非负表示使用常值前馈
        ratio(ratio_),
        id(id_),
        init_angle(init_angle_),
        ppid(ppid_),
        spid(spid_),
        ff_intensity(ff_intensity_),
        const_ff_intensity(ff_intensity_ >= 0){};
    void read_RxMsg(const uint8_t rx_data[8]);
    void write_TxMsg(uint8_t tx_data[8]);

    void SetIntensity(float intensity);
    void SetSpeed(float tgt_speed_, float ff_intensity_=0.f);
    void SetAngle(float tgt_angle_, float ff_speed_=0.f, float ff_intensity_=0.f);
    void handle();
};

#endif //GIMBAL_MOTOR_H
