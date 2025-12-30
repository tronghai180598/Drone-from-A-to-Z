#include <cmath>
#include "vector.h"
#include "quaternion.h"
extern Quaternion attitude;
extern float roll_H, pitch_H, yaw_H;
extern float roll_H_filtered, pitch_H_filtered, yaw_H_filtered;

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
inline void lowPassFilter(float& value, float input, float alpha) {
    value = alpha * input + (1 - alpha) * value;
}
inline void updateFilteredAttitude(const Vector& acc) {
    static const float alpha = 0.1f;
    lowPassFilter(roll_H_filtered, roll_H, alpha);
    lowPassFilter(pitch_H_filtered, pitch_H, alpha);
    lowPassFilter(yaw_H_filtered, yaw_H, alpha);
}