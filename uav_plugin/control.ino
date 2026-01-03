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
extern float roll_H_filtered, pitch_H_filtered, yaw_H_filtered;
extern Vector gyro, pos, vel;
extern Quaternion attitude;     // Current attitude (from UavPlugin.cpp)
extern float dt;
extern float __micros;          // Time in microseconds (from UavPlugin.cpp)

// Target tracking variables (defined in UavPlugin.cpp before this include)
extern TargetPos target_pos_world;
extern bool target_valid;

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
PID pidX (2.5f, 0.0f, 0.0f);
PID pidY (2.0f, 0.0f, 0.0f);
PID pidZ (1.5f, 0.0f, 0.0f);

// Velocity -> accel/thrust
PID pidVx(2.2f, 0.06f, 0.03f);
PID pidVy(2.2f, 0.06f, 0.01f);
PID pidVz(2.0f, 0.05f, 0.001f);

// Attitude -> rate
PID pidRoll (1.5f, 0.0f, 0.01f);
PID pidPitch(1.5f, 0.0f, 0.01f);
PID pidYaw  (1.0f, 0.0f, 0.05f);

// Rate -> torque
PID pidRollRate (0.2f, 0.10f, 0.002f);
PID pidPitchRate(0.2f, 0.10f, 0.002f);
PID pidYawRate  (0.15f, 0.1f, 0.002f);

// Your PDPI controllers (used when not in stick mode, or in auto logic)
KrenCtrl pdpiRoll;
KrenCtrl pdpiPitch;

// =======================================================
// Mission State Machine: INIT -> SEARCH -> TRACK -> ATTACK
// =======================================================

enum TrackingState {
    STATE_INIT   = 0,   // climb to 4m + yaw scan
    STATE_SEARCH = 1,   // scan/aim yaw until target stable
    STATE_TRACK  = 2,   // follow target at stand-off
    STATE_ATTACK = 3    // dive/attack
};

static TrackingState trackingState = STATE_INIT;

static int   target_valid_count     = 0;
static float last_target_lost_time  = 0.0f;
static float setpoint_update_accum  = 0.0f;
static float track_start_time       = 0.0f;

// INIT yaw scan override
static bool  init_scan_yaw_active = false;
static float init_scan_yaw_rate   = 0.0f;   // rad/s

static float init_start_time      = 0.0f;
static float init_yaw_accumulated = 0.0f;

// Parameters
static constexpr float R_FOLLOW = 4.0f;
static constexpr float Z_FOLLOW = 2.0f;

static constexpr float ATTACK_DISTANCE = 3.5f;
static constexpr float ATTACK_Z_OFFSET = 0.15f;

static constexpr float VSP_MAX       = 1.0f;
static constexpr float VSP_ATTACK    = 4.0f;  // 2x faster in attack mode
static constexpr float VSP_ATTACK_Z  = 2.0f;   // 2x faster vertical in attack mode

static constexpr float SETPOINT_UPDATE_RATE  = 50.0f;  // Hz
static constexpr int   TARGET_VALID_THRESHOLD = 10;
static constexpr float TARGET_LOST_TIMEOUT    = 0.7f;
static constexpr float TRACK_TO_ATTACK_TIME   = 2.0f;

// INIT sequence
static constexpr float INIT_ALTITUDE       = 4.0f;
static constexpr float INIT_ROTATIONS      = 0.2f;  // your current value
static constexpr float INIT_YAW_RATE       = 1.0f;
static constexpr float ALTITUDE_TOLERANCE  = 0.2f;

// =======================================================
// updateTargetTracking(): generates X_set, Y_set, Z_set, yaw_set
// =======================================================
void updateTargetTracking() {
    const float currentTime = __micros / 1000000.0f;

    // ---------------------------------------------------
    // 1) INIT: climb to 4m, then yaw scan (sục sạo) N rotations
    // ---------------------------------------------------
    if (trackingState == STATE_INIT) {
        static bool  rotation_started   = false;
        static float rotation_start_time = 0.0f;
        static float prev_yaw_meas      = 0.0f;
        static bool  prev_yaw_inited    = false;

        // Init on first entry
        if (init_start_time == 0.0f) {
            init_start_time = currentTime;
            yaw_set = yaw_H_filtered;

            init_yaw_accumulated = 0.0f;
            init_scan_yaw_active = false;
            init_scan_yaw_rate   = 0.0f;

            rotation_started = false;
            rotation_start_time = 0.0f;
            prev_yaw_inited = false;
        }

        // Phase A: climb to INIT_ALTITUDE
        const float altitude_error = INIT_ALTITUDE - pos.z;

        if (fabsf(altitude_error) > ALTITUDE_TOLERANCE) {
            // Hold XY, climb Z, hold yaw
            X_set = pos.x;
            Y_set = pos.y;
            Z_set = INIT_ALTITUDE;
            yaw_set = yaw_H_filtered;

            // Reset scan state while climbing
            init_scan_yaw_active = false;
            init_scan_yaw_rate   = 0.0f;
            init_yaw_accumulated = 0.0f;

            rotation_started = false;
            rotation_start_time = 0.0f;
            prev_yaw_inited = false;
            return;
        }

        // Phase B: at altitude, rotate yaw by measured accumulation
        X_set = pos.x;
        Y_set = pos.y;
        Z_set = INIT_ALTITUDE;

        // Keep yaw_set synced (yaw control uses override rate below)
        yaw_set = yaw_H_filtered;

        if (!rotation_started) {
            rotation_started = true;
            rotation_start_time = currentTime;

            prev_yaw_meas   = yaw_H_filtered;
            prev_yaw_inited = true;

            init_yaw_accumulated = 0.0f;

            init_scan_yaw_active = true;
            init_scan_yaw_rate   = INIT_YAW_RATE;
        }

        // accumulate actual yaw change
        if (prev_yaw_inited) {
            const float dyaw = wrapAngle(yaw_H_filtered - prev_yaw_meas);
            const float abs_dyaw = fabsf(dyaw);

            // ignore tiny noise and impossible jumps
            if (abs_dyaw > 0.001f && abs_dyaw < 3.0f) {
                init_yaw_accumulated += abs_dyaw;
            }
            prev_yaw_meas = yaw_H_filtered;
        }

        const float target_yaw_rotation = INIT_ROTATIONS * 2.0f * (float)M_PI;
        const float expected_time = (INIT_YAW_RATE > 1e-6f) ? (target_yaw_rotation / INIT_YAW_RATE) : 0.0f;
        const bool timeout_reached =
            (rotation_start_time > 0.0f) &&
            (expected_time > 0.0f) &&
            (currentTime - rotation_start_time > expected_time * 1.5f);

        if (init_yaw_accumulated >= target_yaw_rotation || timeout_reached) {
            // Stop scan immediately
            init_scan_yaw_active = false;
            init_scan_yaw_rate   = 0.0f;

            // Next: SEARCH (find target stable)
            trackingState = STATE_SEARCH;

            // reset init vars
            init_start_time = 0.0f;
            init_yaw_accumulated = 0.0f;

            rotation_started = false;
            rotation_start_time = 0.0f;
            prev_yaw_inited = false;

            return;
        }

        return;
    }

    // ---------------------------------------------------
    // 2) SEARCH: if target visible -> aim yaw; else slow yaw scan.
    //            When stable N frames -> TRACK.
    // ---------------------------------------------------
    if (trackingState == STATE_SEARCH) {
        // gating to enter TRACK
        if (target_valid) {
            target_valid_count++;
            if (target_valid_count >= TARGET_VALID_THRESHOLD) {
                trackingState = STATE_TRACK;
                track_start_time = currentTime;
                target_valid_count = 0;
            }
        } else {
            target_valid_count = 0;
        }

        // SEARCH yaw behavior
        if (target_valid) {
            const float dx = target_pos_world.x - pos.x;
            const float dy = target_pos_world.y - pos.y;
            const float yaw_to_target = atan2f(dy, dx);

            const float max_yaw_step = 3.0f * (1.0f / SETPOINT_UPDATE_RATE);
            float yaw_error = wrapAngle(yaw_to_target - yaw_set);
            yaw_error = clampf(yaw_error, -max_yaw_step, max_yaw_step);

            yaw_set = wrapAngle(yaw_set + yaw_error);
        } else {
            static float search_yaw_rate = 0.2f;   // rad/s
            const float dt_setpoint = 1.0f / SETPOINT_UPDATE_RATE;

            yaw_set = wrapAngle(yaw_set + search_yaw_rate * dt_setpoint);
        }

        return;
    }

    // ---------------------------------------------------
    // 3) TRACK: follow target with stand-off R_FOLLOW, keep altitude Z_FOLLOW.
    //           If close enough or time exceeded -> ATTACK.
    // ---------------------------------------------------
    if (trackingState == STATE_TRACK) {
        // lost handling
        if (!target_valid) {
            if (last_target_lost_time == 0.0f) last_target_lost_time = currentTime;
            if (currentTime - last_target_lost_time > TARGET_LOST_TIMEOUT) {
                trackingState = STATE_SEARCH;
                last_target_lost_time = 0.0f;
                target_valid_count = 0;
            }
            return;
        }
        last_target_lost_time = 0.0f;

        const float dx = target_pos_world.x - pos.x;
        const float dy = target_pos_world.y - pos.y;
        const float distance_2d = sqrtf(dx * dx + dy * dy);

        const bool should_attack =
            (distance_2d < ATTACK_DISTANCE) ||
            (track_start_time > 0.0f && (currentTime - track_start_time) > TRACK_TO_ATTACK_TIME);

        if (should_attack) {
            trackingState = STATE_ATTACK;
            track_start_time = 0.0f;
            // do not return: allow ATTACK to run next tick
        }

        const float yaw_to_target = atan2f(dy, dx);

        // yaw set with limited step
        const float max_yaw_step = 10.0f * (1.0f / SETPOINT_UPDATE_RATE);
        float yaw_error = wrapAngle(yaw_to_target - yaw_set);
        if (fabsf(yaw_error) < 0.05f) {
            yaw_set = yaw_to_target;
        } else {
            yaw_error = clampf(yaw_error, -max_yaw_step, max_yaw_step);
            yaw_set = wrapAngle(yaw_set + yaw_error);
        }

        // stand-off position behind the line-of-sight
        const float X_des = target_pos_world.x - R_FOLLOW * cosf(yaw_set);
        const float Y_des = target_pos_world.y - R_FOLLOW * sinf(yaw_set);
        const float Z_des = Z_FOLLOW;

        const float max_step = VSP_MAX * (1.0f / SETPOINT_UPDATE_RATE);
        X_set = slewRate(X_set, X_des, max_step);
        Y_set = slewRate(Y_set, Y_des, max_step);
        Z_set = slewRate(Z_set, Z_des, max_step);

        return;
    }

    // ---------------------------------------------------
    // 4) ATTACK: dive into target position and lower altitude near target.
    // ---------------------------------------------------
    // (trackingState == STATE_ATTACK)
    if (!target_valid) {
        if (last_target_lost_time == 0.0f) last_target_lost_time = currentTime;
        if (currentTime - last_target_lost_time > TARGET_LOST_TIMEOUT) {
            trackingState = STATE_SEARCH;
            last_target_lost_time = 0.0f;
            target_valid_count = 0;
        }
        return;
    }
    last_target_lost_time = 0.0f;

    const float dx = target_pos_world.x - pos.x;
    const float dy = target_pos_world.y - pos.y;
    const float distance_2d = sqrtf(dx * dx + dy * dy);

    // If target runs away, go back TRACK
    if (distance_2d > ATTACK_DISTANCE * 1.8f) {
        trackingState = STATE_TRACK;
        track_start_time = currentTime;
        return;
    }

    const float yaw_to_target = atan2f(dy, dx);

    const float max_yaw_step = 20.0f * (1.0f / SETPOINT_UPDATE_RATE);
    float yaw_error = wrapAngle(yaw_to_target - yaw_set);
    if (fabsf(yaw_error) < 0.05f) {
        yaw_set = yaw_to_target;
    } else {
        yaw_error = clampf(yaw_error, -max_yaw_step, max_yaw_step);
        yaw_set = wrapAngle(yaw_set + yaw_error);
    }

    float X_des = target_pos_world.x;
    float Y_des = target_pos_world.y;
    float Z_des = target_pos_world.z + ATTACK_Z_OFFSET;
    if (Z_des < 0.15f) Z_des = 0.15f;

    const float max_step_xy = VSP_ATTACK   * (1.0f / SETPOINT_UPDATE_RATE);
    const float max_step_z  = VSP_ATTACK_Z * (1.0f / SETPOINT_UPDATE_RATE);

    X_set = slewRate(X_set, X_des, max_step_xy);
    Y_set = slewRate(Y_set, Y_des, max_step_xy);
    Z_set = slewRate(Z_set, Z_des, max_step_z);
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
    const float er = roll_set  - attitude.getRoll();
    const float ep = pitch_set - attitude.getPitch();

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
    float rate_yaw_set = 0.0f;

    // INIT yaw scan override (only during STATE_INIT scan)
    if (trackingState == STATE_INIT && init_scan_yaw_active) {
        rate_yaw_set = clampf(init_scan_yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE);
    } else {
        const float er_yaw = wrapAngle(yaw_set - yaw_H_filtered);
        rate_yaw_set = clampf(pidYaw.update(er_yaw, dt), -MAX_YAW_RATE, MAX_YAW_RATE);
    }

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
        // When disarmed: reset mission init on next arm
        trackingState = STATE_INIT;
        init_start_time = 0.0f;
        init_yaw_accumulated = 0.0f;
        init_scan_yaw_active = false;
        init_scan_yaw_rate = 0.0f;
        return;
    }

    // Detect rising edge of arm to reset INIT cleanly
    static bool was_armed = false;
    if (armed && !was_armed) {
        trackingState = STATE_INIT;
        init_start_time = 0.0f;
        init_yaw_accumulated = 0.0f;
        init_scan_yaw_active = false;
        init_scan_yaw_rate = 0.0f;

        target_valid_count = 0;
        last_target_lost_time = 0.0f;
        track_start_time = 0.0f;
        setpoint_update_accum = 0.0f;
    }
    was_armed = armed;

    // Mission setpoints update at fixed rate
    setpoint_update_accum += dt;
    const float update_period = 1.0f / SETPOINT_UPDATE_RATE;
    if (setpoint_update_accum >= update_period) {
        updateTargetTracking();
        setpoint_update_accum = 0.0f;
    }

    // Always altitude control
    ctrlAltitudeToThrottle();

    // XY position control only after INIT completes
    if (trackingState != STATE_INIT) {
        ctrlpostoRates();
    } else {
        // During INIT: keep level (no XY tilt)
        roll_set  = 0.0f;
        pitch_set = 0.0f;
    }

    // Attitude controller selection
    if (strcmp(controller_mode, "stick") == 0 && trackingState != STATE_INIT) {
        ctrlAttitudeToRates();
    } else {
        target_roll  = pdpiRoll.updateCtrl(dt, roll_set,  attitude.getRoll(),  gyro.x);
        target_pitch = pdpiPitch.updateCtrl(dt, pitch_set, attitude.getPitch(), gyro.y);
    }

    // Yaw always active (INIT scan uses override inside ctrlYawToTorque)
    ctrlYawToTorque();

    // Motor mix
    mixToMotors();
}
