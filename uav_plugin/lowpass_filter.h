#pragma once
#include "vector.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Low-pass filter for scalar values
class LowPassFilter {
public:
    float value;           // Filtered output value
    float cutoffHz;       // Cutoff frequency in Hz
    bool initialized;

    LowPassFilter(float cutoffHz = 20.0f)
        : value(0.0f)
        , cutoffHz(cutoffHz)
        , initialized(false)
    {}

    // Update filter with new input value
    float update(float input, float dt) {
        if (!initialized) {
            value = input;
            initialized = true;
            return value;
        }

        // Calculate time constant: tau = 1/(2*pi*fc)
        const float tau = 1.0f / (2.0f * M_PI * cutoffHz);
        const float alpha = dt / (dt + tau);
        
        value = alpha * input + (1.0f - alpha) * value;
        return value;
    }

    void reset() {
        value = 0.0f;
        initialized = false;
    }

    void setCutoff(float cutoffHz) {
        this->cutoffHz = cutoffHz;
    }
};

// Low-pass filter for Vector (3D)
class LowPassFilterVector {
public:
    Vector value;          // Filtered output vector
    float cutoffHz;       // Cutoff frequency in Hz
    bool initialized;

    LowPassFilterVector(float cutoffHz = 40.0f)
        : value(0, 0, 0)
        , cutoffHz(cutoffHz)
        , initialized(false)
    {}

    // Update filter with new input vector
    Vector update(const Vector& input, float dt) {
        if (!initialized) {
            value = input;
            initialized = true;
            return value;
        }

        // Calculate time constant: tau = 1/(2*pi*fc)
        const float tau = 1.0f / (2.0f * M_PI * cutoffHz);
        const float alpha = dt / (dt + tau);
        
        value.x = alpha * input.x + (1.0f - alpha) * value.x;
        value.y = alpha * input.y + (1.0f - alpha) * value.y;
        value.z = alpha * input.z + (1.0f - alpha) * value.z;
        
        return value;
    }

    void reset() {
        value = Vector(0, 0, 0);
        initialized = false;
    }

    void setCutoff(float cutoffHz) {
        this->cutoffHz = cutoffHz;
    }
};

