//
// Created by JiangYC on 2025/11/27.
//

#ifndef GIMBAL_PID_H
#define GIMBAL_PID_H

#ifdef __cplusplus
extern "C" {
    #endif

    class PID {
    private:
        const float kp;
        const float ki;
        const float kd;

        const float d_filter;       // 对d一阶线性滤波
        const int i_iter;           // 循环链表大小
        const float i_max;          // 限制误差积分大小
        float out_max;              // 限制输出大小，可被set_out_max()修改，0.f代表没有限制

        float ref = 0.f;                  // 目标值
        float fdb = 0.f;                  // 反馈值
        float cur_err = 0.f;
        float pre_err = 0.f;
        float delta_err = 0.f;
        float err_sum = 0.f;

        float p_out = 0.f;
        float i_out = 0.f;
        float d_out = 0.f;
        float out = 0.f;

        static float clamp(const float val, const float min, const float max);
    public:
        PID(const float kp_, const float ki_, const float kd_, const float i_max_, const int i_iter_=100, const float out_max_ = 0.f, const float d_filter_=0.1):
            kp(kp_),
            ki(ki_),
            kd(kd_),
            d_filter(d_filter_ > 0.f && d_filter_ < 1.f? d_filter_: 0.1),
            i_iter(i_iter_ > 0? i_iter_: 5),
            i_max(i_max_ > 0.f? i_max_: 1.f)
        {
            set_out_max(out_max_);
        };
        PID(const PID& other):
            kp(other.kp),
            ki(other.ki),
            kd(other.kd),
            d_filter(other.d_filter),
            i_iter(other.i_iter),
            i_max(other.i_max),
            out_max(other.out_max){}
        ~PID() = default;

        void set_out_max(float out_max_);           // 设置out_max（为虚弱状态预留更新接口）
        float calc(float _ref, float _fdb);
    };

    #ifdef __cplusplus
}
#endif

#endif //GIMBAL_PID_H
