//
// Created by JiangYC on 2025/11/25.
//

#ifndef GIMBAL_MOTOR_H
#define GIMBAL_MOTOR_H

#include "PID.h"
#include <stdint.h>

class Motor {
protected:
    const float ratio;
    const int id;

    float angle = 0.f;                  // deg 输出端累计转动角度
    float delta_angle = 0.f;            // deg 输出端新转动角度
    float ecd_angle = 0.f;              // deg 当前编码器转动角度
    float last_ecd_angle = 0.f;         // deg 上次编码器转动角度
    float delta_ecd_angle = 0.f;        // deg 编码器端新转动角度
    float rotate_speed = 0.f;           // dps 反馈转子速度
    float current = 0.f;                // A 反馈转矩电流
    float temperature = 0.f;            // °C 反馈电机温度

    PID ppid;
    PID spid;
    float tgt_angle = 0.f, fdb_angle = 0.f;
    float tgt_speed = 0.f, fdb_speed = 0.f;
    float output_intensity = 0.f;       // 扭矩输出接口
    const float ff_speed = 0.f;         // 前馈，设置目标速度？ff_speed暂时没用，置为常数0，SetAngle有注释化
    float ff_intensity = 0.f;           // 解算设置前馈力矩
    const bool ff_intensity_flag;       // false 表示常值前馈

    enum ControlMode {
        TORQUE,                         // 开环力矩控制
        SPEED,                          // 速度闭环控制
        POSITION_SPEED,                 // 位置速度双环控制
    };
    ControlMode mode = TORQUE;

    template<class T1, class T2>
    static T2 linear_mapping(T1 org, T1 org_max, T2 res_max, T1 org_min=0, T2 res_min=0) {
        return T2((org - org_min) * (res_max - res_min) / (org_max - org_min) + res_min);
    }

    static float normalize_angle(float angle, bool has_direction=false);

    // void calc_ff_intensity();

public:
    Motor(const float ratio_, const int id_, const PID& ppid_, const PID& spid_, const float ff_intensity_ = 0.f):
        ratio(ratio_),
        id(id_),
        ppid(ppid_),
        spid(spid_),
        ff_intensity(ff_intensity_),
        ff_intensity_flag(ff_intensity_ < 0.f){}

    virtual ~Motor() = default;

    virtual void read_RxMsg(const uint8_t rx_data[8]) = 0;
    virtual void write_TxMsg(uint8_t tx_data[8]);

    void SetIntensity(float intensity);
    void SetSpeed(float tgt_speed_, float ff_intensity_=0.f);
    void SetAngle(float tgt_angle_, float ff_speed_=0.f, float ff_intensity_=0.f);
    virtual void calculate_ff_intensity() = 0;
    virtual void handle() = 0;
};


class GM6020Motor final : public Motor {
private:
    static constexpr uint16_t rx_ecd_angle = 8191;
    static constexpr float res_ecd_angle = 360.f;           // deg
    static constexpr int16_t rx_rotate_speed = 1;           // rpm
    static constexpr float res_rotate_speed = 6.f;          // dps
    static constexpr int16_t rx_max_current = 16384;
    static constexpr float res_max_current = 3.f;           // A
    static constexpr float torque_current_ratio = 0.741f;   // Nm/A

public:
    GM6020Motor(const float ratio_, const int id_, const PID& ppid_, const PID& spid_, const float ff_intensity_ = -1.f):
        Motor(ratio_, id_, ppid_, spid_, ff_intensity_) {}
    ~GM6020Motor() override = default;

    void read_RxMsg(const uint8_t rx_data[8]) override;
    void write_TxMsg(uint8_t tx_data[8]) override;

    void calculate_ff_intensity() override;
    void handle() override;
};


class M3508Motor final : public Motor {
private:
    static constexpr uint16_t rx_ecd_angle = 8191;
    static constexpr float res_ecd_angle = 360.f;     // deg
    static constexpr int16_t rx_rotate_speed = 1;     // rpm
    static constexpr float res_rotate_speed = 6.f;    // dps
    static constexpr int16_t rx_max_current = 16384;
    static constexpr float res_max_current = 20.f;    // A

public:
    M3508Motor(const float ratio_, const int id_, const PID& ppid_, const PID& spid_):
        Motor(ratio_, id_, ppid_, spid_) {}
    ~M3508Motor() override = default;

    void read_RxMsg(const uint8_t rx_data[8]) override;
    void write_TxMsg(uint8_t tx_data[8]) override;

    void calculate_ff_intensity() override;
    void handle() override;
};

#endif //GIMBAL_MOTOR_H
