//
// Created by JiangYC on 2025/12/7.
//

#include "RemoteController.h"
// #include "usart.h"

void RC::handle(const int ticks){
    last_ticks = ticks;

    for (int i = 0; i < 18; ++i) {
        rx_data[i] = rx_buf[i];
    }

    int ch_tmp = (rx_data[0] | rx_data[1] << 8) & 0x07FF;
    CH0 = linear_mapping(ch_tmp, ch_low, ch_high, -1.0f, 1.0f);
    set_dead(CH0);

    ch_tmp = (rx_data[1] >> 3 | rx_data[2] << 5) & 0x07FF;
    CH1 = linear_mapping(ch_tmp, ch_low, ch_high, -1.0f, 1.0f);
    set_dead(CH1);

    ch_tmp = (rx_data[2] >> 6 | rx_data[3] << 2 | rx_data[4] << 10) & 0x07FF;
    CH2 = linear_mapping(ch_tmp, ch_low, ch_high, -1.0f, 1.0f);
    set_dead(CH2);

    ch_tmp = (rx_data[4] >> 1 | rx_data[5] << 7) & 0x07FF;
    CH3 = linear_mapping(ch_tmp, ch_low, ch_high, -1.0f, 1.0f);
    set_dead(CH3);

    S1 = static_cast<SwitchState>((rx_data[5] >> 4) & 0x03);
    S2 = static_cast<SwitchState>((rx_data[5] >> 6) & 0x03);

    axis_X = static_cast<int16_t>(rx_data[6] | rx_data[7] << 8);
    axis_Y = static_cast<int16_t>(rx_data[8] | rx_data[9] << 8);
    axis_Z = static_cast<int16_t>(rx_data[10] | rx_data[11] << 8);

    left = rx_data[12] & 0x01;
    right = rx_data[13] & 0x01;

    key_W = (rx_data[14] >> 7) & 0x01;
    key_S = (rx_data[14] >> 6) & 0x01;
    key_A = (rx_data[14] >> 5) & 0x01;
    key_D = (rx_data[14] >> 4) & 0x01;
    key_Q = (rx_data[14] >> 3) & 0x01;
    key_E = (rx_data[14] >> 2) & 0x01;
    key_SHIFT = (rx_data[14] >> 1) & 0x01;
    key_CTRL = rx_data[14] & 0x01;
}

bool RC::check_connection(const int ticks)const{
    return (ticks - last_ticks < ticks_timeout);
}

RC rc;