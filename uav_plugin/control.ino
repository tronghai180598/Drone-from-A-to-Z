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
float MAX_YAW_RATE = 1.57f;
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
PID pidYaw  (0.5f, 0.0f, 0.0f);

// Rate PID
PID pidRollRate (0.2f, 0.10f, 0.002f);  
PID pidPitchRate(0.2f, 0.10f, 0.002f); 
PID pidYawRate  (0.15f, 0.05f, 0.0001f);

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
    float er_yaw = wrapAngle(yaw_set - yaw_H_filtered);
    float rate_yaw_set = clampf(pidYaw.update(er_yaw, dt), -MAX_YAW_RATE, MAX_YAW_RATE);
    float error_yaw_rate = rate_yaw_set - gyro.z;
    target_yaw = pidYawRate.update(error_yaw_rate, dt);
    target_yaw   = clampf(target_yaw,   -0.4f, 0.4f);
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
        return;
    }
    // Otherwise, use X_set, Y_set, Z_set from manual input (Qt or CLI)
    
    ctrlAltitudeToThrottle();
    ctrlpostoRates();
    // Select controller based on mode
    if (strcmp(controller_mode, "stick") == 0) {
        // Stick mode: manual control from Qt (current controller)
        ctrlAttitudeToRates();
    } else {
        target_roll = pdpiRoll.updateCtrl(dt, roll_set, attitude.getRoll(), gyro.x);
        target_pitch = pdpiPitch.updateCtrl(dt, pitch_set, attitude.getPitch(), gyro.y);
    }
    ctrlYawToTorque();
    mixToMotors();
}
