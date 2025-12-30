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
 
float motors[4] = {0,0,0,0};
float roll_set = 0.0f;
float pitch_set = 0.0f;
float yaw_set = 0.0f;
// Controller mode: "stick" = manual control from Qt, "auto" = other controller (to be implemented)
// Note: Using char array instead of const char* to allow modification
char controller_mode[16] = "stick";  // Default: stick mode
float MAX_RATE = 3.14f;
float MAX_YAW_RATE = 2.0f;  // Increased for better yaw rotation
extern float roll_H_filtered, pitch_H_filtered, yaw_H_filtered;
extern Vector acc, gyro, pos, vel;
extern float mass;
const float ONE_G = 9.81f;
extern Quaternion attitude;  // Current attitude (from UavPlugin.cpp)
float target_roll = 0.0f;
float target_pitch = 0.0f;
float target_yaw = 0.0f;
// Helper function
static inline float wrapAngle(float a) {
    while (a >  (float)M_PI) a -= 2.0f*(float)M_PI;
    while (a < -(float)M_PI) a += 2.0f*(float)M_PI;
    return a;
}
float throttle_u = 0.0f; // [0..1]

bool armed = false;
extern float dt;
KrenCtrl pdpiRoll;
KrenCtrl pdpiPitch;
// State variables (need to be declared)
float Z_set = 0.0f;
float X_set = 0.0f;
float Y_set = 0.0f;

// Heart trajectory variables
bool heartMode = false;  // Enable/disable heart trajectory mode
bool heartStarted = false;
float searchYaw = 0.0f;
float heartStartTime = 0.0f;
extern float __micros;  // Time in microseconds (from UavPlugin.cpp)

// Target tracking variables (defined in UavPlugin.cpp before this include)
extern TargetPos target_pos_world;
extern bool target_valid;

// Target tracking state machine
enum TrackingState {
    STATE_INIT = 0,    // Initialization: climb to 4m and scan 2 rotations
    STATE_SEARCH = 1,
    STATE_TRACK = 2,
    STATE_ATTACK = 3
};
static TrackingState trackingState = STATE_INIT;
static int target_valid_count = 0;
static float last_target_lost_time = 0.0f;
static float setpoint_update_accum = 0.0f;
static float track_start_time = 0.0f;
// INIT yaw scan override (minimal)
static bool  init_scan_yaw_active = false;
static float init_scan_yaw_rate   = 0.0f;   // rad/s
static float init_start_time = 0.0f;
static float init_start_yaw = 0.0f;
static float init_yaw_accumulated = 0.0f;
static bool init_completed = false;

// Target tracking parameters
static constexpr float R_FOLLOW = 4.0f;          // Stand 4m away from target
static constexpr float Z_FOLLOW = 2.0f;         // Fly at 2m altitude
static constexpr float ATTACK_DISTANCE = 3.5f;  // Attack when distance < 3.5m
static constexpr float ATTACK_Z_OFFSET = 0.15f; // Attack at robot height + 0.15m
static constexpr float VSP_MAX = 1.5f;           // Max setpoint velocity (m/s)
static constexpr float VSP_ATTACK = 5.0f;       // Max setpoint velocity in attack mode (m/s)
static constexpr float VSP_ATTACK_Z = 4.0f;    // Max vertical velocity when attacking (m/s)
static constexpr float SETPOINT_UPDATE_RATE = 50.0f;  // 50 Hz
static constexpr int TARGET_VALID_THRESHOLD = 10;     // Need 10 consecutive valid detections
static constexpr float TARGET_LOST_TIMEOUT = 0.7f;     // 0.7s timeout before SEARCH
static constexpr float TRACK_TO_ATTACK_TIME = 2.0f;   // Attack after tracking for 2 seconds
static constexpr float INIT_ALTITUDE = 4.0f;         // Initial climb altitude (m)
static constexpr float INIT_ROTATIONS = 0.2f;        // Number of rotations to scan (2 full circles = 720°)
static constexpr float INIT_YAW_RATE = 1.0f;          // Yaw rotation rate during scan (rad/s) - smooth rotation
static constexpr float ALTITUDE_TOLERANCE = 0.2f;     // Altitude tolerance for init (m)

static inline float slewRate(float current, float desired, float max_step) {
    float d = desired - current;
    if (d > max_step) d = max_step;
    if (d < -max_step) d = -max_step;
    return current + d;
}

void updateTargetTracking() {
    float currentTime = __micros / 1000000.0f;
    
    // State machine: INIT -> SEARCH <-> TRACK -> ATTACK
    if (trackingState == STATE_INIT) {
        // Initialize on first call
        static bool rotation_started = false;
        static float prev_yaw_meas = 0.0f;
        static bool  prev_yaw_inited = false;
        
        if (init_start_time == 0.0f) {
            init_start_time = currentTime;
            init_start_yaw = yaw_H_filtered;
            yaw_set = yaw_H_filtered;  // Initialize yaw_set
            init_yaw_accumulated = 0.0f;
            init_completed = false;
            rotation_started = false;
            prev_yaw_inited = false;
        }
        
        // Phase 1: Climb to INIT_ALTITUDE (4m)
        float altitude_error = INIT_ALTITUDE - pos.z;
        
        if (fabsf(altitude_error) > ALTITUDE_TOLERANCE) {
            // Still climbing - set Z_set to climb, hold X/Y position
            X_set = pos.x;  // Hold current X position
            Y_set = pos.y;  // Hold current Y position
            Z_set = INIT_ALTITUDE;  // Climb to 4m
            yaw_set = yaw_H_filtered;  // Hold yaw while climbing
            
            // Reset flags while climbing
            init_scan_yaw_active = false;
            init_scan_yaw_rate   = 0.0f;
            prev_yaw_inited      = false;
            init_yaw_accumulated = 0.0f;
            rotation_started = false;  // Reset rotation flag
        } else {
            // Phase 2: At altitude, rotate yaw 720 degrees (measured)
            static float rotation_start_time = 0.0f;
            if (!rotation_started) {
                yaw_set = yaw_H_filtered;
                init_yaw_accumulated = 0.0f;
                rotation_started = true;
                rotation_start_time = currentTime;  // Record start time for timeout safety
                
                prev_yaw_meas = yaw_H_filtered;
                prev_yaw_inited = true;
                
                // Enable yaw-rate override to ensure rotation
                init_scan_yaw_active = true;
                init_scan_yaw_rate   = INIT_YAW_RATE;  // Change sign if want reverse rotation
            }
            
            // Hold position and altitude while rotating
            X_set = pos.x;
            Y_set = pos.y;
            Z_set = INIT_ALTITUDE;
            
            // (1) command yaw rotation: keep yaw_set synced with measured yaw to avoid angle PID fighting
            yaw_set = yaw_H_filtered;
            
            // (2) accumulate rotation by ACTUAL yaw change
            if (prev_yaw_inited) {
                float dyaw = wrapAngle(yaw_H_filtered - prev_yaw_meas);
                float abs_dyaw = fabsf(dyaw);
                // Only accumulate if change is significant (avoid noise and wrap-around)
                if (abs_dyaw > 0.001f && abs_dyaw < 3.0f) {  // Ignore wrap-around jumps
                    init_yaw_accumulated += abs_dyaw;
                }
                prev_yaw_meas = yaw_H_filtered;
            }
            
            float target_yaw_rotation = INIT_ROTATIONS * 2.0f * (float)M_PI;  // 720° = 4π rad ≈ 12.566 rad
            // Check if rotation completed - use >= to ensure we stop exactly at 720°
            // Also add timeout safety: if rotating for more than expected time + margin, force stop
            float expected_time = target_yaw_rotation / INIT_YAW_RATE;  // Time needed for 720° at current rate (≈12.57s at 1.0 rad/s)
            bool timeout_reached = (rotation_start_time > 0.0f) && 
                                   (currentTime - rotation_start_time > expected_time * 1.5f);  // 50% margin
            
            if (init_yaw_accumulated >= target_yaw_rotation || timeout_reached) {
                // Rotation completed - disable yaw override FIRST (critical!)
                init_scan_yaw_active = false;
                init_scan_yaw_rate   = 0.0f;
                
                // Then change state to SEARCH immediately
                trackingState = STATE_SEARCH;
                init_completed = true;
                
                // Reset all init variables
                init_start_time = 0.0f;
                init_yaw_accumulated = 0.0f;
                rotation_started = false;
                prev_yaw_inited = false;
                rotation_start_time = 0.0f;  // Reset rotation timer
                
                // Exit immediately to prevent further rotation
                return;
            }
        }
    } else if (trackingState == STATE_SEARCH) {
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
        // In SEARCH mode: if target visible, point yaw at it; otherwise rotate slowly
        if (target_valid) {
            float dx = target_pos_world.x - pos.x;
            float dy = target_pos_world.y - pos.y;
            float yaw_to_target = atan2f(dy, dx);
            
            float max_yaw_step = 3.0f * (1.0f / SETPOINT_UPDATE_RATE);
            float yaw_error = wrapAngle(yaw_to_target - yaw_set);
            if (yaw_error > max_yaw_step) yaw_error = max_yaw_step;
            if (yaw_error < -max_yaw_step) yaw_error = -max_yaw_step;
            yaw_set += yaw_error;
            yaw_set = wrapAngle(yaw_set);
        } else {
            static float search_yaw_rate = 0.2f;  // rad/s
            float dt_setpoint = 1.0f / SETPOINT_UPDATE_RATE;
            yaw_set += search_yaw_rate * dt_setpoint;
            yaw_set = wrapAngle(yaw_set);
        } 
    } else if (trackingState == STATE_TRACK) {
        if (!target_valid) {
            if (last_target_lost_time == 0.0f) {
                last_target_lost_time = currentTime;
            }
            if (currentTime - last_target_lost_time > TARGET_LOST_TIMEOUT) {
                trackingState = STATE_SEARCH;
                target_valid_count = 0;
                last_target_lost_time = 0.0f;
            }
        } else {
            last_target_lost_time = 0.0f;
        }
        // In TRACK mode: always point nose (front) at target
        if (target_valid) {
            float dx = target_pos_world.x - pos.x;
            float dy = target_pos_world.y - pos.y;
            float distance_2d = sqrtf(dx * dx + dy * dy);
            // Check if should attack: either close enough OR tracked for long enough
            bool should_attack = (distance_2d < ATTACK_DISTANCE) || 
                                 (track_start_time > 0.0f && (currentTime - track_start_time) > TRACK_TO_ATTACK_TIME);
            
            if (should_attack) {
                trackingState = STATE_ATTACK;
                track_start_time = 0.0f;
            }
            float yaw_to_target = atan2f(dy, dx);   
            float max_yaw_step = 10.0f * (1.0f / SETPOINT_UPDATE_RATE);
            float yaw_error = wrapAngle(yaw_to_target - yaw_set);
            if (fabsf(yaw_error) < 0.05f) {
                yaw_set = yaw_to_target;
            } else {
                if (yaw_error > max_yaw_step) yaw_error = max_yaw_step;
                if (yaw_error < -max_yaw_step) yaw_error = -max_yaw_step;
                yaw_set += yaw_error;
            }
            yaw_set = wrapAngle(yaw_set);
            float X_des = target_pos_world.x - R_FOLLOW * cosf(yaw_set);
            float Y_des = target_pos_world.y - R_FOLLOW * sinf(yaw_set);
            float Z_des = Z_FOLLOW;
            
            float max_step = VSP_MAX * (1.0f / SETPOINT_UPDATE_RATE);            
            X_set = slewRate(X_set, X_des, max_step);
            Y_set = slewRate(Y_set, Y_des, max_step);
            Z_set = slewRate(Z_set, Z_des, max_step);
        }
    } else {  // STATE_ATTACK
        if (!target_valid) {
            if (last_target_lost_time == 0.0f) {
                last_target_lost_time = currentTime;
            }
            if (currentTime - last_target_lost_time > TARGET_LOST_TIMEOUT) {
                trackingState = STATE_SEARCH;
                target_valid_count = 0;
                last_target_lost_time = 0.0f;
            }
        } else {
            last_target_lost_time = 0.0f;
        }
        
        // In ATTACK mode: dive straight into target
        if (target_valid) {
            float dx = target_pos_world.x - pos.x;
            float dy = target_pos_world.y - pos.y;
            float distance_2d = sqrtf(dx * dx + dy * dy);
            
            // If target moves away significantly, return to TRACK mode
            if (distance_2d > ATTACK_DISTANCE * 1.8f) {
                trackingState = STATE_TRACK;
                track_start_time = currentTime;
            }
            
            float yaw_to_target = atan2f(dy, dx);
            
            float max_yaw_step = 20.0f * (1.0f / SETPOINT_UPDATE_RATE);
            float yaw_error = wrapAngle(yaw_to_target - yaw_set);
            if (fabsf(yaw_error) < 0.05f) {
                yaw_set = yaw_to_target;
            } else {
                if (yaw_error > max_yaw_step) yaw_error = max_yaw_step;
                if (yaw_error < -max_yaw_step) yaw_error = -max_yaw_step;
                yaw_set += yaw_error;
            }
            yaw_set = wrapAngle(yaw_set);
            
            // Attack: dive straight into target at robot's height
            float X_des = target_pos_world.x;
            float Y_des = target_pos_world.y;
            float Z_des = target_pos_world.z + ATTACK_Z_OFFSET;
            
            // Clamp Z to minimum safe altitude
            if (Z_des < 0.15f) Z_des = 0.15f;
            
            float max_step_xy = VSP_ATTACK * (1.0f / SETPOINT_UPDATE_RATE);
            float max_step_z = VSP_ATTACK_Z * (1.0f / SETPOINT_UPDATE_RATE);
            
            X_set = slewRate(X_set, X_des, max_step_xy);
            Y_set = slewRate(Y_set, Y_des, max_step_xy);
            Z_set = slewRate(Z_set, Z_des, max_step_z);
        }
    }
}

static inline float clampf(float v, float lo, float hi){
    return (v < lo) ? lo : (v > hi) ? hi : v;
}
// Altitude PID
// Increased gains for better trajectory tracking
PID pidX (2.5f, 0.0f, 0.0f);  // Increased Kp and added Kd for faster response
PID pidY (2.0f, 0.0f, 0.0f);  // Increased Kp and added Kd for faster response
PID pidZ (1.5f, 0.0f, 0.0f);

// Increased velocity PID gains for better tracking
PID pidVx(2.2f, 0.06f, 0.03f); // Increased Kp and Kd for faster acceleration
PID pidVy(2.2f, 0.06f, 0.01f); // Increased Kp and Kd for faster acceleration
PID pidVz(2.0f, 0.05f, 0.001f); // z -> vz_set
// Attitude PID
PID pidRoll (1.5f, 0.0f, 0.01f);
PID pidPitch(1.5f, 0.0f, 0.01f);
PID pidYaw  (1.0f, 0.0f, 0.05f);

// Rate PID
PID pidRollRate (0.2f, 0.10f, 0.002f);  
PID pidPitchRate(0.2f, 0.10f, 0.002f); 
PID pidYawRate  (0.15f, 0.1f, 0.002f);

void ctrlAltitudeToThrottle(){
    const float ez = Z_set - pos.z;
    const float vz_set = pidZ.update(ez, dt);
    const float evz = vz_set - vel.z;
    
    // PID output is thrust delta in Newton (like Flix: velPID.update)
    float thrustCmd = pidVz.update(evz, dt);
    thrustCmd = clampf(thrustCmd, -1.0f, 1.0f);
    
    const float hoverThrust = 0.30f;  // Base hover throttle (tune: 0.3-0.5)
    float raw = hoverThrust + thrustCmd;
    throttle_u = clampf(raw, 0.0f, 1.0f);
}
// Feedforward velocity from heart trajectory (for better tracking)
static float prev_X_set = 0.0f;
static float prev_Y_set = 0.0f;
static bool firstCall_pos = true;

void ctrlpostoRates(){
    float ex = X_set - pos.x;
    float ey = Y_set - pos.y;

    // Calculate feedforward velocity from setpoint change
    float vx_ff = 0.0f;
    float vy_ff = 0.0f;
    if (!firstCall_pos && dt > 0.0f) {
        vx_ff = (X_set - prev_X_set) / dt;
        vy_ff = (Y_set - prev_Y_set) / dt;
    }
    firstCall_pos = false;
    prev_X_set = X_set;
    prev_Y_set = Y_set;
    
    // Clamp feedforward velocity to reasonable limits
    const float MAX_FF_VEL = 5.0f;  // m/s
    vx_ff = clampf(vx_ff, -MAX_FF_VEL, MAX_FF_VEL);
    vy_ff = clampf(vy_ff, -MAX_FF_VEL, MAX_FF_VEL);

    // Position PID outputs desired velocity
    float rate_x_set = clampf(pidX.update(ex, dt), -MAX_RATE, MAX_RATE);
    float rate_y_set = clampf(pidY.update(ey, dt), -MAX_RATE, MAX_RATE);
    
    // Add feedforward to desired velocity (helps track sharp turns)
    rate_x_set += vx_ff;
    rate_y_set += vy_ff;
    rate_x_set = clampf(rate_x_set, -MAX_RATE, MAX_RATE);
    rate_y_set = clampf(rate_y_set, -MAX_RATE, MAX_RATE);

    float evx = rate_x_set - vel.x;
    float evy = rate_y_set - vel.y;

    // Gia tốc mong muốn trong WORLD
    float ax_w = pidVx.update(evx, dt);
    float ay_w = pidVy.update(evy, dt);
    float yaw = attitude.getYaw();
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    // Chuyển gia tốc từ WORLD sang LOCAL
    float ax_b = ax_w * cy + ay_w * sy;
    float ay_b = -ax_w * sy + ay_w * cy;

    roll_set  =  -ay_b / ONE_G;
    pitch_set = ax_b / ONE_G;
  
    // Clamp to ±45 degrees (±0.785 rad) for safety
    const float MAX_ATTITUDE_RAD = 0.785398163f;  // 45 degrees in radians
    roll_set  = clampf(roll_set,  -MAX_ATTITUDE_RAD, MAX_ATTITUDE_RAD);
    pitch_set = clampf(pitch_set, -MAX_ATTITUDE_RAD, MAX_ATTITUDE_RAD);
}
void ctrlAttitudeToRates(){   

 //   imu_update(acc);
    float er = roll_set  - attitude.getRoll();
    float ep = pitch_set - attitude.getPitch();

    // Attitude PID outputs desired rates
    float rate_roll_set  = clampf(pidRoll.update(er, dt), -MAX_RATE, MAX_RATE);
    float rate_pitch_set = clampf(pidPitch.update(ep, dt), -MAX_RATE, MAX_RATE);

    // Rate PID outputs torque (like Flix: torqueTarget)
    float error_roll_rate = rate_roll_set - gyro.x;
    float error_pitch_rate = rate_pitch_set - gyro.y;

    target_roll = pidRollRate.update(error_roll_rate, dt);
    target_pitch = pidPitchRate.update(error_pitch_rate, dt);

    target_roll  = clampf(target_roll,  -0.4f, 0.4f);
    target_pitch = clampf(target_pitch, -0.4f, 0.4f);
}
void ctrlYawToTorque(){
    float rate_yaw_set = 0.0f;
    
    // Only use yaw override if in INIT state AND override is active
    if (trackingState == STATE_INIT && init_scan_yaw_active) {
        rate_yaw_set = clampf(init_scan_yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE);
    } else {
        float er_yaw = wrapAngle(yaw_set - yaw_H_filtered);
        rate_yaw_set = clampf(pidYaw.update(er_yaw, dt), -MAX_YAW_RATE, MAX_YAW_RATE);
    }
    
    float error_yaw_rate = rate_yaw_set - gyro.z;
    target_yaw = pidYawRate.update(error_yaw_rate, dt);
    target_yaw = clampf(target_yaw, -0.4f, 0.4f);
}
void mixToMotors(){
    if (throttle_u < 0.1f) {
        motors[0] = motors[1] = motors[2] = motors[3] = 0.1f;  // idle thrust
        return;
    }
    motors[0] = throttle_u + target_roll - target_pitch + target_yaw;
    motors[1] = throttle_u - target_roll - target_pitch - target_yaw;
    motors[2] = throttle_u + target_roll + target_pitch - target_yaw;
    motors[3] = throttle_u - target_roll + target_pitch + target_yaw;

    for (int i=0;i<4;i++) motors[i] = clampf(motors[i], 0.0f, 1.0f);
}
void control(){ 
    if (!armed) {
        // Reset init state when disarmed
        if (trackingState == STATE_INIT) {
            trackingState = STATE_INIT;
            init_start_time = 0.0f;
            init_yaw_accumulated = 0.0f;
            init_completed = false;
        }
        return;
    }
    // Reset init when first armed
    static bool was_armed = false;
    if (armed && !was_armed) {
        trackingState = STATE_INIT;
        init_start_time = 0.0f;
        init_yaw_accumulated = 0.0f;
        init_completed = false;
        init_scan_yaw_active = false;
        init_scan_yaw_rate = 0.0f;
    }
    was_armed = armed;
    
    // Update target tracking setpoint (at 50 Hz)
    // In INIT state, updateTargetTracking() only rotates yaw, no position control
    setpoint_update_accum += dt;
    if (setpoint_update_accum >= (1.0f / SETPOINT_UPDATE_RATE)) {
        updateTargetTracking();
        setpoint_update_accum = 0.0f;
    }
    
    // Otherwise, use X_set, Y_set, Z_set from manual input (Qt or CLI) or target tracking
    
    // In INIT state, still need altitude control to climb, but no X/Y tracking
    // After rotation completes, enter full tracking control loop
    ctrlAltitudeToThrottle();  // Always needed for altitude control
    if (trackingState != STATE_INIT) {
        ctrlpostoRates();  // Only position tracking after INIT completes
    }
    
    // In INIT state, set roll/pitch to 0 to avoid tilting from previous commands
    if (trackingState == STATE_INIT) {
        roll_set = 0.0f;
        pitch_set = 0.0f;
    }
    
    // In INIT state, set roll/pitch to 0 to avoid tilting from previous commands
    if (trackingState == STATE_INIT) {
        roll_set = 0.0f;
        pitch_set = 0.0f;
    }
    
    // Select controller based on mode
    // In INIT state, always use auto mode for yaw control
    if (strcmp(controller_mode, "stick") == 0 && trackingState != STATE_INIT) {
        // Stick mode: manual control from Qt (current controller)
        ctrlAttitudeToRates();
    } else {
        target_roll = pdpiRoll.updateCtrl(dt, roll_set, attitude.getRoll(), gyro.x);
        target_pitch = pdpiPitch.updateCtrl(dt, pitch_set, attitude.getPitch(), gyro.y);
    }
    // Yaw control always uses yaw_set (from tracking or init)
    ctrlYawToTorque();
    mixToMotors();
}
