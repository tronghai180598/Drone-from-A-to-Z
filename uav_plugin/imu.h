#ifndef IMU_H
#define IMU_H

#include <cmath>
#include "vector.h"
#include "quaternion.h"
extern Quaternion attitude;
extern float roll_H, pitch_H, yaw_H;
extern float roll_H_filtered, pitch_H_filtered, yaw_H_filtered;
extern float dt;
extern Vector gyro;

// Forward declarations
void updateAttitude(const Vector& acc);
void updateFilteredAttitude(const Vector& acc);

inline void imu_update(const Vector& acc){
    updateAttitude(acc);
    updateFilteredAttitude(acc);
}
inline float approx_atan2_quadrant(float y, float z) {
    const float kEPS = 1e-6f;
    float ay = fabsf(y);
    float az = fabsf(z);
    if (ay < kEPS && az < kEPS) { return 0.0f; }
    float beta;
    if (ay <= az)  beta = M_PI_4 * (ay / az);
    else  beta = M_PI_4 * (2.0f - az / ay);

    float th;
    if (z >= 0.0f) th = (y >= 0.0f) ? beta : -beta;
    else th = (y >= 0.0f) ? (M_PI - beta) : (-(M_PI) + beta);
    return th;
}
inline void updateAttitude(const Vector& acc) {
    roll_H = approx_atan2_quadrant(acc.y, acc.z);  // roll
    pitch_H = approx_atan2_quadrant(-acc.x, acc.z); // pitch
    yaw_H = attitude.getYaw(); // yaw from quaternion
}
// Kalman filter for 1D signal
inline void kalman_for_angle(float& KalmanState,
                      float& KalmanUncertainty,
                      float  KalmanInput,
                      float  KalmanMeasurement,
                      float  dt) {
    KalmanState = KalmanState + dt * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + dt * dt * 1.0f * 1.0f;

    float KalmanGain = KalmanUncertainty * 1.0f /
                       (1.0f * KalmanUncertainty + 0.1f * 0.1f);

    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1.0f - KalmanGain) * KalmanUncertainty;
}
inline void updateFilteredAttitude(const Vector& acc) {
    // Kalman uncertainty for each angle
    static float roll_uncertainty = 1.0f;
    static float pitch_uncertainty = 1.0f;
    static float yaw_uncertainty = 1.0f;
    
    // Use gyro as input (rate of change) for prediction
    kalman_for_angle(roll_H_filtered, roll_uncertainty, gyro.x, roll_H, dt);
    kalman_for_angle(pitch_H_filtered, pitch_uncertainty, gyro.y, pitch_H, dt);
    kalman_for_angle(yaw_H_filtered, yaw_uncertainty, gyro.z, yaw_H, dt);
}

#endif // IMU_H