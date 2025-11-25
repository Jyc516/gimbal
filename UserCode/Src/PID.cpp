//
// Created by JiangYC on 2025/11/25.
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

void PID::reset_err_queue() {
    for (int i=0; i < i_iter; ++i) {
        err_head->err = 0.f;
        err_head = err_head->next;
    }
    err_sum = 0.f;
}

void PID::update_err_queue() {
    err_sum = err_sum - err_head->err + cur_err;
    err_head->err = cur_err;
    err_head = err_head->next;
}

void PID::set_out_max(const float out_max_) {
    out_max = out_max_ > 0.f? out_max_: 0.f;
}

float PID::calc(float ref_, float fdb_) {
    fdb = fdb_;
    if (ref_ == ref) {
        pre_err = cur_err;
        cur_err = ref - fdb;
        delta_err = d_filter * (cur_err - pre_err) + (1 - d_filter) * delta_err;
        update_err_queue();
    }
    else {
        ref = ref_;
        cur_err = ref - fdb;
        delta_err = 0;
        reset_err_queue();
        update_err_queue();
    }

    raw_output =  kp * cur_err + kd * delta_err + ki * clamp(err_sum / i_iter, -i_max, i_max);
    return out_max == 0.f? raw_output: clamp(raw_output, -out_max, out_max);
}