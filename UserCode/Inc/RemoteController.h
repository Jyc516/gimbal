//
// Created by JiangYC on 2025/12/7.
//

#ifndef GIMBAL_REMOTECONTROLLER_H
#define GIMBAL_REMOTECONTROLLER_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
    #endif

    class RC {
    private:
        uint8_t rx_data[18] = {0};
        int last_ticks = 0;
        static constexpr int ticks_timeout = 1000;
        // bool connection_status = true;

        static constexpr int ch_low = 364;
        static constexpr int ch_mid = 1024;
        static constexpr int ch_high = 1684;

        static float linear_mapping(const int raw, const int raw_low, const int raw_high, const float res_low, const float res_high) {
            return static_cast<float>(raw - raw_low) / static_cast<float>(raw_high - raw_low) * (res_high - res_low) + res_low;
        }
        static void set_dead(float &CH) {
            if (CH < 0.1 && CH > -0.1) {
                CH = 0.f;
            }
            else if (CH > 0.1) {
                CH = (CH - 0.1) / 0.9;
            }
            else {
                CH = (CH + 0.1) / 0.9;
            }
        }

    public:
        enum SwitchState {
            UP = 1,
            DOWN = 2,
            MID = 3
          };

        float CH0 = 0;     // 364 - 1024 - 1684
        float CH1 = 0;     // 364 - 1024 - 1684
        float CH2 = 0;     // 364 - 1024 - 1684
        float CH3 = 0;     // 364 - 1024 - 1684
        SwitchState S1 = DOWN;         // 1 - 3
        SwitchState S2 = DOWN;         // 1 - 3
        int axis_X = 0;          // -32768 - 0 - 32767
        int axis_Y = 0;          // -32768 - 0 - 32767
        int axis_Z = 0;          // -32768 - 0 - 32767
        int left = 0;       // 0 - 1
        int right = 0;      // 0 - 1
        int key_W = 0;      // 0 - 1
        int key_S = 0;      // 0 - 1
        int key_A = 0;      // 0 - 1
        int key_D = 0;      // 0 - 1
        int key_Q = 0;      // 0 - 1
        int key_E = 0;      // 0 - 1
        int key_SHIFT = 0;  // 0 - 1
        int key_CTRL = 0;   // 0 - 1


    public:
        uint8_t rx_buf[32] = {0};

        RC()= default;
        void handle(const int ticks);
        bool check_connection(int ticks)const;
    };

    #ifdef __cplusplus
}
#endif

#endif //GIMBAL_REMOTECONTROLLER_H
