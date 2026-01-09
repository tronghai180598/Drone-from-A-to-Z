#include "pid.h"
#include "quaternion.h"
#include "vector.h"
#include <cmath>
#include <cstring>
#include "imu.h"
#include "KrenCtrl.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =======================================================
// Externs from simulator/plugin
// =======================================================
extern float roll_H, pitch_H, yaw_H_filtered;
extern Vector gyro, pos, vel;
extern Quaternion attitude;     // Current attitude (from UavPlugin.cpp)
extern float dt;
extern float __micros;          // Time in microseconds (from UavPlugin.cpp)


// =======================================================
// Outputs / Setpoints
// =======================================================
float motors[4] = {0, 0, 0, 0};

float roll_set  = 0.0f;
float pitch_set = 0.0f;
float yaw_set   = 0.0f;

float throttle_u    = 0.0f;   // [0..1]
float target_roll   = 0.0f;
float target_pitch  = 0.0f;
float target_yaw    = 0.0f;


float Z_set = 0.0f;

// Mode
char controller_mode[16] = "stick";  // Default: stick mode

// Attitude control mode: "pid" or "pdpi"
char attitude_mode[16] = "pid";  // Default: PID controller

// Armed flag (your project uses this)
bool armed = false;


// =======================================================
// Constants / Limits
// =======================================================
static constexpr float ONE_G = 9.81f;

// Note: in your code MAX_RATE is used both for XY desired vel limit and also rate set clamp.
// Keep as-is because your tuning is working.
float MAX_RATE     = 3.14f;
float MAX_YAW_RATE = 2.0f;

// =======================================================
// Helpers
// =======================================================
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline float wrapAngle(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static inline float slewRate(float current, float desired, float max_step) {
    float d = desired - current;
    if (d >  max_step) d =  max_step;
    if (d < -max_step) d = -max_step;
    return current + d;
}

// =======================================================
// Controllers
// =======================================================

// Position -> velocity

PID pidZ (1.0f, 0.0f, 0.0f);
PID pidVz(1.0f, 0.05f, 0.002f);

// Attitude -> rate
PID pidRoll (0.8f, 0.0f, 0.0f);
PID pidPitch(0.8f, 0.0f, 0.0f);
PID pidYaw  (1.0f, 0.0f, 0.0f);

// Rate -> torque
PID pidRollRate (0.2f, 0.01f, 0.002f);
PID pidPitchRate(0.2f, 0.01f, 0.002f);
PID pidYawRate  (0.15f, 0.1f, 0.002f);

// Your PDPI controllers (used when not in stick mode, or in auto logic)
KrenCtrl pdpiRoll;
KrenCtrl pdpiPitch;

static void resetAllPids() {
    pidZ.reset();
    pidVz.reset();
    pidRoll.reset(); pidPitch.reset(); pidYaw.reset();
    pidRollRate.reset(); pidPitchRate.reset(); pidYawRate.reset();
    // Reset PDPI controllers
    pdpiRoll.reset();
    pdpiPitch.reset();
}
// =======================================================
// Low-level control
// =======================================================

void ctrlAltitudeToThrottle() {
    float ez = Z_set - pos.z;
    float vz_set = pidZ.update(ez, dt);
    float evz = vz_set - vel.z;

    float thrustCmd = pidVz.update(evz, dt);
    thrustCmd = clampf(thrustCmd, -1.0f, 1.0f);

    const float hoverThrust = 0.30f;  // your current tuning
    throttle_u = clampf(hoverThrust + thrustCmd, 0.0f, 1.0f);
}

void ctrlAttitudeToRates() {
    // Use filtered IMU angles to reduce noise
    float er = roll_set  - roll_H_filtered;
    float ep = pitch_set - pitch_H_filtered;

    float rate_roll_set  = clampf(pidRoll.update(er, dt),  -MAX_RATE, MAX_RATE);
    float rate_pitch_set = clampf(pidPitch.update(ep, dt), -MAX_RATE, MAX_RATE);

    float error_roll_rate  = rate_roll_set  - gyro.x;
    float error_pitch_rate = rate_pitch_set - gyro.y;

    target_roll  = pidRollRate.update(error_roll_rate, dt);
    target_pitch = pidPitchRate.update(error_pitch_rate, dt);

    target_roll  = clampf(target_roll,  -0.5f, 0.5f);
    target_pitch = clampf(target_pitch, -0.5f, 0.5f);
}

void ctrlYawToTorque() {
    float error_yaw_rate = yaw_set - gyro.z;
    target_yaw = pidYawRate.update(error_yaw_rate, dt);
    target_yaw = clampf(target_yaw, -0.5f, 0.5f);
}

void mixToMotors() {
    // If not armed or throttle too low, set motors to zero
    if (!armed || throttle_u < 0.1f) {
        motors[0] = motors[1] = motors[2] = motors[3] = 0.0f;
        return;
    }

    motors[0] = throttle_u + target_roll - target_pitch + target_yaw;
    motors[1] = throttle_u - target_roll - target_pitch - target_yaw;
    motors[2] = throttle_u + target_roll + target_pitch - target_yaw;
    motors[3] = throttle_u - target_roll + target_pitch + target_yaw;

    for (int i = 0; i < 4; i++) motors[i] = clampf(motors[i], 0.0f, 1.0f);
}
// =======================================================
// Main control loop
// =======================================================
void control() {
    // If not armed, reset everything and set motors to zero
    if (!armed) {
        resetAllPids();
        target_roll = target_pitch = target_yaw = 0.0f;
        throttle_u = 0.0f;
        motors[0] = motors[1] = motors[2] = motors[3] = 0.0f;
        return;
    }
    
    // Track previous mode to detect mode changes
    static char prev_attitude_mode[16] = "";
    static bool first_call = true;
    
    // Reset controllers when switching between PID and PDPI modes
    if (first_call || strcmp(prev_attitude_mode, attitude_mode) != 0) {
        if (strcmp(attitude_mode, "pdpi") == 0) {
            // Switching to PDPI: reset PDPI controllers and initialize with current state
            pdpiRoll.reset();
            pdpiPitch.reset();
            // Initialize Kalman filter states with current angles to avoid jump
            pdpiRoll.mFi = roll_H;
            pdpiRoll.mVi = gyro.x;
            pdpiPitch.mFi = pitch_H;
            pdpiPitch.mVi = gyro.y;
        } else {
            // Switching to PID: reset PID controllers
            pidRoll.reset();
            pidPitch.reset();
            pidRollRate.reset();
            pidPitchRate.reset();
        }
        strncpy(prev_attitude_mode, attitude_mode, sizeof(prev_attitude_mode) - 1);
        prev_attitude_mode[sizeof(prev_attitude_mode) - 1] = '\0';
        first_call = false;
    }
    
    // Always use altitude control (RC throttle sets Z_set, altitude controller handles it)
    ctrlAltitudeToThrottle();

    // Attitude control: choose between PID or PDPI controller
    if (strcmp(attitude_mode, "pdpi") == 0) {
        // Use PDPI controller for attitude
        // PDPI has its own Kalman filter, so use raw angles (roll_H, pitch_H)
        target_roll  = pdpiRoll.updateCtrl(dt, roll_set,  roll_H,  gyro.x);
        target_pitch = pdpiPitch.updateCtrl(dt, pitch_set, pitch_H, gyro.y);
        target_roll  = clampf(target_roll,  -0.5f, 0.5f);
        target_pitch = clampf(target_pitch, -0.5f, 0.5f);
    } else {
        // Use PID controller for attitude (default)
        ctrlAttitudeToRates();
    }

    // Yaw control (common for both modes - rate control)
    ctrlYawToTorque();
    
    // Motor mix
    mixToMotors();
}
