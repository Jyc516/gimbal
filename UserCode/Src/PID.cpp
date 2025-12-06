//
// Created by JiangYC on 2025/11/27.
//

#include "PID.h"

float PID::clamp(const float val, const float min, const float max) {
    if (val > max) {
        return max;
    }
    if (val < min) {
        return min;
    }
    return val;
}

void PID::set_out_max(const float out_max_) {
    out_max = out_max_ > 0.f? out_max_: 0.f;
}

float PID::calc(const float ref_, const float fdb_) {
    fdb = fdb_;

    // 千万不要删。ref更新时需要重启 I 和 D
    if (ref == ref_) {
        pre_err = cur_err;
        cur_err = ref - fdb;
        delta_err = d_filter * (cur_err - pre_err) + (1.f - d_filter) * delta_err;
        err_sum = clamp(err_sum * (1 - 1.f / i_iter) + cur_err / i_iter, -i_max, i_max);
    }
    else {
        ref = ref_;
        cur_err = ref - fdb;
        delta_err = 0.f;
        err_sum = clamp(cur_err / i_iter, -i_max, i_max);
    }

    p_out = kp * cur_err;
    d_out = kd * delta_err;
    i_out = ki * err_sum;
    out = p_out + i_out + d_out;

    return out_max == 0.f? out: clamp(out, -out_max, out_max);
}