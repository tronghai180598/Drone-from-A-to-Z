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
extern Vector gyro_filtered, vel_filtered;
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

// Position setpoints (from mission logic)
float X_set = 0.0f;
float Y_set = 0.0f;
float Z_set = 0.0f;

// Mode
char controller_mode[16] = "stick";  // Default: stick mode

// Armed flag (your project uses this)
bool armed = false;

// Heart trajectory flag
bool heart_active = false;

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
// Heart Trajectory
// =======================================================
static float heart_start_time = 0.0f;
static float heart_x0 = 0.0f;
static float heart_y0 = 0.0f;
static constexpr float HEART_SCALE = 0.4f;  // Scale factor for heart size
static constexpr float HEART_PERIOD = 40.0f; // Period in seconds (time to complete one heart)

void updateHeartTrajectory() {
    if (!heart_active) return;
    
    // Initialize start position and time on first activation
    if (heart_start_time == 0.0f) {
        heart_start_time = __micros / 1000000.0f;
        heart_x0 = pos.x;
        heart_y0 = pos.y;
    }
    
    const float currentTime = __micros / 1000000.0f;
    const float elapsed = currentTime - heart_start_time;
    
    // Calculate phase (0 to 2*PI over HEART_PERIOD)
    const float phi = 2.0f * (float)M_PI * (elapsed / HEART_PERIOD);
    
    const float s = sinf(phi);
    const float c = cosf(phi);
    
    // Heart equation
    const float x_raw = 16.0f * s * s * s;
    const float y_raw = 13.0f * c - 5.0f * cosf(2.0f * phi) - 2.0f * cosf(3.0f * phi) - cosf(4.0f * phi);
    
    // Apply scale and offset
    X_set = heart_x0 + HEART_SCALE * x_raw;
    Y_set = heart_y0 + HEART_SCALE * y_raw;
    
    // Keep Z constant (or you can set it to a fixed altitude)
    // Z_set remains unchanged (use current Z_set from Qt)
}

// =======================================================
// Controllers
// =======================================================

// Position -> velocity
PID pidX (0.7f, 0.0f, 0.0f);
PID pidY (0.7f, 0.0f, 0.0f);
PID pidZ (1.5f, 0.0f, 0.0f);

// Velocity -> accel/thrust
PID pidVx(1.4f, 0.04f, 0.035f);
PID pidVy(1.4f, 0.04f, 0.035f);
PID pidVz(2.0f, 0.05f, 0.002f);

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
    pidX.reset(); pidY.reset(); pidZ.reset();
    pidVx.reset(); pidVy.reset(); pidVz.reset();
    pidRoll.reset(); pidPitch.reset(); pidYaw.reset();
    pidRollRate.reset(); pidPitchRate.reset(); pidYawRate.reset();
}
// =======================================================
// Low-level control
// =======================================================

void ctrlAltitudeToThrottle() {
    const float ez = Z_set - pos.z;
    const float vz_set = pidZ.update(ez, dt);
    const float evz = vz_set - vel.z;

    float thrustCmd = pidVz.update(evz, dt);
    thrustCmd = clampf(thrustCmd, -1.0f, 1.0f);

    const float hoverThrust = 0.30f;  // your current tuning
    throttle_u = clampf(hoverThrust + thrustCmd, 0.0f, 1.0f);
}

// Feedforward from setpoint change (kept, because you are using it)
static float prev_X_set = 0.0f;
static float prev_Y_set = 0.0f;
static bool  firstCall_pos = true;

void ctrlpostoRates() {
    const float ex = X_set - pos.x;
    const float ey = Y_set - pos.y;

    // Feedforward velocity from setpoint change
    float vx_ff = 0.0f;
    float vy_ff = 0.0f;
    if (!firstCall_pos && dt > 0.0f) {
        vx_ff = (X_set - prev_X_set) / dt;
        vy_ff = (Y_set - prev_Y_set) / dt;
    }
    firstCall_pos = false;
    prev_X_set = X_set;
    prev_Y_set = Y_set;

    // Clamp feedforward velocity
    const float MAX_FF_VEL = 5.0f;
    vx_ff = clampf(vx_ff, -MAX_FF_VEL, MAX_FF_VEL);
    vy_ff = clampf(vy_ff, -MAX_FF_VEL, MAX_FF_VEL);

    // Position PID -> desired velocity
    float rate_x_set = clampf(pidX.update(ex, dt), -MAX_RATE, MAX_RATE);
    float rate_y_set = clampf(pidY.update(ey, dt), -MAX_RATE, MAX_RATE);

    // Add feedforward
    rate_x_set = clampf(rate_x_set + vx_ff, -MAX_RATE, MAX_RATE);
    rate_y_set = clampf(rate_y_set + vy_ff, -MAX_RATE, MAX_RATE);

    const float evx = rate_x_set - vel.x;
    const float evy = rate_y_set - vel.y;

    // Velocity PID -> desired accel in WORLD
    const float ax_w = pidVx.update(evx, dt);
    const float ay_w = pidVy.update(evy, dt);

    // WORLD -> BODY by yaw
    const float yaw = attitude.getYaw();
    const float cy = cosf(yaw);
    const float sy = sinf(yaw);

    const float ax_b = ax_w * cy + ay_w * sy;
    const float ay_b = -ax_w * sy + ay_w * cy;

    // accel -> tilt
    roll_set  = -ay_b / ONE_G;
    pitch_set =  ax_b / ONE_G;

    // Clamp to ±45 degrees
    const float MAX_ATTITUDE_RAD = 0.785398163f;
    roll_set  = clampf(roll_set,  -MAX_ATTITUDE_RAD, MAX_ATTITUDE_RAD);
    pitch_set = clampf(pitch_set, -MAX_ATTITUDE_RAD, MAX_ATTITUDE_RAD);
}

void ctrlAttitudeToRates() {
    // Use filtered IMU angles to reduce noise
    const float er = roll_set  - roll_H_filtered;
    const float ep = pitch_set - pitch_H_filtered;

    const float rate_roll_set  = clampf(pidRoll.update(er, dt),  -MAX_RATE, MAX_RATE);
    const float rate_pitch_set = clampf(pidPitch.update(ep, dt), -MAX_RATE, MAX_RATE);

    const float error_roll_rate  = rate_roll_set  - gyro.x;
    const float error_pitch_rate = rate_pitch_set - gyro.y;

    target_roll  = pidRollRate.update(error_roll_rate, dt);
    target_pitch = pidPitchRate.update(error_pitch_rate, dt);

    target_roll  = clampf(target_roll,  -0.4f, 0.4f);
    target_pitch = clampf(target_pitch, -0.4f, 0.4f);
}

void ctrlYawToTorque() {
    const float er_yaw = wrapAngle(yaw_set - yaw_H_filtered);
    const float rate_yaw_set = clampf(pidYaw.update(er_yaw, dt), -MAX_YAW_RATE, MAX_YAW_RATE);

    const float error_yaw_rate = rate_yaw_set - gyro.z;
    target_yaw = pidYawRate.update(error_yaw_rate, dt);
    target_yaw = clampf(target_yaw, -0.4f, 0.4f);
}

void mixToMotors() {
    if (throttle_u < 0.1f) {
        motors[0] = motors[1] = motors[2] = motors[3] = 0.1f;
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
    if (!armed) {
        resetAllPids();
        // Reset heart trajectory when disarmed
        if (heart_active) {
            heart_active = false;
            heart_start_time = 0.0f;
        }
        return;
    }

    // Always altitude control
    ctrlAltitudeToThrottle();

    // Mode selection: stick = attitude control, auto = position control
    if (strcmp(controller_mode, "stick") == 0) {
        // STICK MODE: Direct attitude control via roll_set, pitch_set, yaw_set from slider
        // Do NOT control position - roll_set, pitch_set come from slider (rc command)
        ctrlAttitudeToRates();
    } else {
        // AUTO MODE: Position control via X_set, Y_set, Z_set from Qt
        // Update heart trajectory if active (overrides X_set, Y_set from Qt)
        if (heart_active) {
            updateHeartTrajectory();
        }
        // Otherwise, X_set, Y_set, Z_set, yaw_set are set from Qt via UDP
        
        // XY position control - calculates roll_set, pitch_set from X_set, Y_set
        ctrlpostoRates();
        
        // Use PDPI controller for attitude
        target_roll  = pdpiRoll.updateCtrl(dt, roll_set,  attitude.getRoll(),  gyro.x);
        target_pitch = pdpiPitch.updateCtrl(dt, pitch_set, attitude.getPitch(), gyro.y);
    }

    // Yaw control
    ctrlYawToTorque();

    // Motor mix
    mixToMotors();
}
