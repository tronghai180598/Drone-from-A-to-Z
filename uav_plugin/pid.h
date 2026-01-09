#pragma once
#include <cmath>
#include "lowpass_filter.h"

class PID {
public:
    float p, i, d;
    float dt;
    float windup = 100.0f;  // Default, can be set per PID
    float derivativeFilterCutoffHz = 20.0f;  // Low-pass filter cutoff frequency for derivative (default 20 Hz)

    float derivative;
    float integral;
    LowPassFilter derivativeFilter;  // Low-pass filter for derivative term

    PID(float p, float i, float d)
        : p(p)
        , i(i)
        , d(d)
        , dt(0.0f)
        , derivative(0.0f)
        , integral(0.0f)
        , derivativeFilter(40.0f)  // Default 20 Hz cutoff
        , prevError(0.0f)
        , initialized(false)
    {
        derivativeFilter.setCutoff(derivativeFilterCutoffHz);
    }

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

            // Đạo hàm (raw) - filtered using LowPassFilter
            float derivativeRaw = (error - prevError) / dt;
            derivative = derivativeFilter.update(derivativeRaw, dt);
        } else {
            // Nếu thời gian nhảy bất thường → reset D, không bơm thêm I
            derivative = 0.0f;
            derivativeFilter.reset();
        }

        prevError = error;

        return p * error + i * integral + d * derivative;
    }

    void reset() {
        integral         = 0.0f;
        derivative       = 0.0f;
        derivativeFilter.reset();
        prevError        = 0.0f;
        initialized      = false;
    }

    void setDerivativeFilterCutoff(float cutoffHz) {
        derivativeFilterCutoffHz = cutoffHz;
        derivativeFilter.setCutoff(cutoffHz);
    }

private:
    float prevError;
    bool  initialized;
};
