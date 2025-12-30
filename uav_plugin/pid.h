#pragma once
#include <cmath>

class PID {
public:
    float p, i, d;
    float dt;
    float windup = 100.0f;  // Default, can be set per PID

    float derivative;
    float integral;

    PID(float p, float i, float d)
        : p(p)
        , i(i)
        , d(d)
        , dt(0.0f)
        , derivative(0.0f)
        , integral(0.0f)
        , prevError(0.0f)
        , initialized(false)
    {}

    // t = thời gian tuyệt đối (ví dụ info.simTime.Double())
    float update(float error, float dt) {
        // Lần đầu gọi: chỉ khởi tạo, chưa tính I, D
        if (!initialized) {
            prevError   = error;
            integral    = 0.0f;
            derivative  = 0.0f;
            dt          = 0.0f;
            initialized = true;
            return p * error;
        }

        if (dt > 0.0f) {
            // Tích phân
            integral += error * dt;

            if (integral >  windup) integral =  windup;
            else if (integral < -windup) integral = -windup;

            // Đạo hàm
            derivative = (error - prevError) / dt;
        } else {
            // Nếu thời gian nhảy bất thường → reset D, không bơm thêm I
            derivative = 0.0f;
            // Có thể giữ nguyên integral hoặc reset tùy bạn; ở đây mình giữ nguyên
        }

        prevError = error;

        return p * error + i * integral + d * derivative;
    }

    void reset() {
        integral    = 0.0f;
        derivative  = 0.0f;
        prevError   = 0.0f;
        initialized = false;
    }

private:
    float prevError;
    bool  initialized;
};
