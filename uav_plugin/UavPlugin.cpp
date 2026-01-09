#include <functional>
#include <cmath>
#include <thread>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <iostream>
#include <fstream>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_4
#define M_PI_4 (M_PI / 4.0)
#endif
#include "imu.h"
#include "cli.cpp"
#include "vector.h"
#include "quaternion.h"
#include "lowpass_filter.h"

#include "control.ino"
#include "KrenCtrl.hpp"
#include "rc_sim_all.hpp"

// Global variables
Vector gyro, acc, pos, vel;
extern float yaw_set;
extern float throttle_u;
extern float Z_set;
extern char controller_mode[16];  // Controller mode: "stick" or "auto"
extern char attitude_mode[16];     // Attitude control mode: "pid" or "pdpi"
extern float target_roll;
extern float target_pitch;
extern class KrenCtrl pdpiRoll;
extern class KrenCtrl pdpiPitch;
Quaternion attitude{1,0,0,0};
float __micros;
float dt;
float mass;

// Low-pass filters for IMU data
static LowPassFilterVector gyroFilter(40.0f);  // 40 Hz cutoff for gyro
static LowPassFilterVector accFilter(20.0f);   // 20 Hz cutoff for accelerometer

// IMU variables (defined here to avoid multiple definition)
float roll_H = 0.0f, pitch_H = 0.0f, yaw_H = 0.0f;
float roll_H_filtered = 0.0f, pitch_H_filtered = 0.0f, yaw_H_filtered = 0.0f;

extern bool armed;
extern float motors[4];
extern const float ONE_G;

using ignition::math::Vector3d;
using namespace gazebo;
using namespace std;

class ModelFlix : public ModelPlugin {
private:
	physics::ModelPtr model;
	physics::LinkPtr body;
	sensors::ImuSensorPtr imu;
	event::ConnectionPtr updateConnection, resetConnection;
	transport::NodePtr nodeHandle;
	ofstream logFile;
	bool logFileOpened;
	ofstream controlLogFile;
	bool controlLogFileOpened;

public:
	ModelFlix() : logFileOpened(false), controlLogFileOpened(false) {}
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
		this->model = _parent;
		this->body = this->model->GetLink("body");
		this->imu = dynamic_pointer_cast<sensors::ImuSensor>(sensors::get_sensor(model->GetScopedName(true) + "::body::imu")); // default::flix::body::imu
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelFlix::OnUpdate, this));
		this->resetConnection = event::Events::ConnectWorldReset(std::bind(&ModelFlix::OnReset, this));
		initNode();
		initLogFile();
		initControlLogFile();
		std::thread(simCliThread).detach();
		
		// Initialize RC joystick control
		rc_sim::setupRC(true);
		gzmsg << "UAV plugin loaded - RC control via joystick enabled" << endl;
	}

	void OnReset() {
		attitude = Quaternion();
		armed = false;
		// Reset filters
		gyroFilter.reset();
		accFilter.reset();
		// Close and reopen log file on reset
		if (logFileOpened) {
			logFile.close();
			logFileOpened = false;
		}
		if (controlLogFileOpened) {
			controlLogFile.close();
			controlLogFileOpened = false;
		}
		initLogFile();
		initControlLogFile();
		gzmsg << "UAV plugin reset" << endl;
	}
	void OnUpdate() {

		__micros = model->GetWorld()->SimTime().Double() * 1000000;
		
		// Calculate dt (delta time in seconds)
		static float lastMicros = 0.0f;
		if (lastMicros > 0.0f) {
			dt = (__micros - lastMicros) / 1000000.0f;  // Convert to seconds
		} else {
			dt = 0.002f;  // Default: 500 Hz
		}
		lastMicros = __micros;
		// Clamp dt to prevent spikes
		if (dt <= 0.0f || dt > 0.02f) dt = 0.002f;
		if (dt < 0.0005f) dt = 0.0005f;
		// Update mass
		mass = (float)body->GetInertial()->Mass();
		// Read IMU data and apply low-pass filters
		Vector gyro_raw = Vector(imu->AngularVelocity().X(), imu->AngularVelocity().Y(), imu->AngularVelocity().Z());
		gyro = gyroFilter.update(gyro_raw, dt);  // Filtered gyro for control
		
		Vector acc_raw = Vector(imu->LinearAcceleration().X(), imu->LinearAcceleration().Y(), imu->LinearAcceleration().Z());
		acc = accFilter.update(acc_raw, dt);  // Filtered acc for control
		
		// Update attitude from IMU - this calculates roll_H, pitch_H from accelerometer
		// In PDPI mode: only update raw angles (no Kalman filter, PDPI has its own)
		// In PID mode: update both raw and filtered angles
		if (strcmp(attitude_mode, "pdpi") == 0) {
			updateAttitude(acc);  // Only raw angles, no Kalman filter
		} else {
			imu_update(acc);  // Both raw and filtered angles
		}
		
		const auto pose = this->model->WorldPose();
		const auto qGazebo = pose.Rot();
		attitude = Quaternion(qGazebo.W(), qGazebo.X(), qGazebo.Y(), qGazebo.Z());
		attitude.normalize();
	
		pos.x = this->model->WorldPose().Pos().X();
		pos.y = this->model->WorldPose().Pos().Y();
		pos.z = this->model->WorldPose().Pos().Z();
        
		vel.x = this->model->WorldLinearVel().X();
		vel.y = this->model->WorldLinearVel().Y();
		vel.z = this->model->WorldLinearVel().Z();
		
		// Read RC joystick input
		double simTimeSec = model->GetWorld()->SimTime().Double();
		bool rc_available = rc_sim::readRC(simTimeSec);
		
		// Show RC status on first successful connection
		static bool rc_status_shown = false;
		if (rc_available && !rc_status_shown) {
			gzmsg << "[RC] Joystick connected and RC control active!" << endl;
			rc_status_shown = true;
		} else if (!rc_available && rc_status_shown) {
			gzwarn << "[RC] Joystick disconnected!" << endl;
			rc_status_shown = false;
		}
		
		// Handle arm/disarm toggle from channel 2 (works in all modes)
		static float prevArmChannel = 0.0f;
		static bool armChannelInitialized = false;
		if (rc_available && !std::isnan(rc_sim::controlArm)) {
			float armChannel = rc_sim::controlArm;
			const float ARM_THRESHOLD = 0.7f;  // Threshold to trigger arm toggle (70% up)
			
			if (!armChannelInitialized) {
				prevArmChannel = armChannel;
				armChannelInitialized = true;
			}
			
			// Detect rising edge: was low (< threshold) and now high (>= threshold)
			if (prevArmChannel < ARM_THRESHOLD && armChannel >= ARM_THRESHOLD) {
				// Toggle arm state
				armed = !armed;
				if (armed) {
					gzmsg << "[RC] >>> ARMED via Channel 2" << endl;
				} else {
					gzmsg << "[RC] >>> DISARMED via Channel 2" << endl;
				}
			}
			prevArmChannel = armChannel;
		}
		
		if (strcmp(controller_mode, "stick") == 0 && rc_available) {
			const float RC_DEADZONE = 0.05f;
			const float RC_CENTER = 0.5f;
			
			auto applyDeadzoneAndMap = [RC_DEADZONE, RC_CENTER](float rc_val, float min_out, float max_out, bool reverse) -> float {
				if (std::abs(rc_val - RC_CENTER) <= RC_DEADZONE) {
					return 0.0f;
				}
				float remapped;
				if (rc_val < RC_CENTER) {
					remapped = rc_sim::mapf(rc_val, 0.0f, RC_CENTER - RC_DEADZONE, 0.0f, 0.5f);
				} else {
					remapped = rc_sim::mapf(rc_val, RC_CENTER + RC_DEADZONE, 1.0f, 0.5f, 1.0f);
				}
				if (reverse) {
					return rc_sim::mapf(remapped, 0.0f, 1.0f, max_out, min_out);
				} else {
					return rc_sim::mapf(remapped, 0.0f, 1.0f, min_out, max_out);
				}
			};
			
			if (!std::isnan(rc_sim::controlRoll)) {
				roll_set = applyDeadzoneAndMap(rc_sim::controlRoll, -0.785398163f, 0.785398163f, rc_sim::rollReverse);
			}
			if (!std::isnan(rc_sim::controlPitch)) {
				pitch_set = applyDeadzoneAndMap(rc_sim::controlPitch, -0.785398163f, 0.785398163f, rc_sim::pitchReverse);
			}
			if (!std::isnan(rc_sim::controlYaw)) {
				yaw_set = applyDeadzoneAndMap(rc_sim::controlYaw, -(float)M_PI, (float)M_PI, rc_sim::yawReverse);
			}
			if (!std::isnan(rc_sim::controlThrottle)) {
				float throttle_val = rc_sim::controlThrottle;
				if (rc_sim::throttleReverse) {
					throttle_val = 1.0f - throttle_val;
				}
				// Map throttle to altitude setpoint (RC throttle sets Z_set, altitude controller handles it)
				Z_set = rc_sim::mapf(throttle_val, 0.0f, 1.0f, 0.0f, 5.0f);
			} 
		} 
		
		control();
		applyMotorForces();
		logData();  // Log attitude data
		logControlData();  // Log control signals
	}

	void applyMotorForces() {
		// thrusts
		const double dist = 0.165; // motors shift from the center, m
		const double maxThrust = 0.03 * ONE_G; // ~30 g, https://youtu.be/VtKI4Pjx8Sk?&t=78

		const float scale0 = 1.0, scale1 = 1.0, scale2 = 1.0, scale3 = 1.0; // imitating motors asymmetry
		float mfl = scale0 * maxThrust * motors[0];
		float mfr = scale1 * maxThrust * motors[1];
		float mrl = scale2 * maxThrust * motors[2];
		float mrr = scale3 * maxThrust * motors[3];

		body->AddLinkForce(Vector3d(0.0, 0.0, mfl), Vector3d(dist, dist, 0.0));
		body->AddLinkForce(Vector3d(0.0, 0.0, mfr), Vector3d(dist, -dist, 0.0));
		body->AddLinkForce(Vector3d(0.0, 0.0, mrl), Vector3d(-dist, dist, 0.0));
		body->AddLinkForce(Vector3d(0.0, 0.0, mrr), Vector3d(-dist, -dist, 0.0));
		// torque
		const double maxTorque = 0.01 * ONE_G; // ~24 g*cm
		body->AddRelativeTorque(Vector3d(0.0, 0.0, scale0 * maxTorque * motors[0]));
		body->AddRelativeTorque(Vector3d(0.0, 0.0, scale1 * -maxTorque * motors[1]));
		body->AddRelativeTorque(Vector3d(0.0, 0.0, scale2 * -maxTorque * motors[2]));
		body->AddRelativeTorque(Vector3d(0.0, 0.0, scale3 * maxTorque * motors[3]));
	}
	void initNode() {
		nodeHandle = transport::NodePtr(new transport::Node());
		nodeHandle->Init();
	}

	void initLogFile() {
		logFile.open("uav_attitude.log", ios::out | ios::trunc);
		if (logFile.is_open()) {
			logFileOpened = true;
			// Write header - format depends on mode:
			// PID mode: t roll_H roll_H_filtered pitch_H pitch_H_filtered
			// PDPI mode: t roll_mFi pitch_mFi roll_angle pitch_angle
			logFile << "# Format: PID mode -> t roll_H roll_H_filtered pitch_H pitch_H_filtered (deg)\n";
			logFile << "# Format: PDPI mode -> t roll_mFi pitch_mFi roll_angle pitch_angle (deg)\n";
			logFile.flush();
			gzmsg << "Log file opened: uav_attitude.log" << endl;
		} else {
			logFileOpened = false;
			gzerr << "Failed to open log file: uav_attitude.log" << endl;
		}
	}
	void initControlLogFile() {
		controlLogFile.open("uav_control.log", ios::out | ios::trunc);
		if (controlLogFile.is_open()) {
			controlLogFileOpened = true;
			// Write header - format only
			controlLogFile << "# t mode target_roll target_pitch pdpiRoll_Us\n";
			controlLogFile.flush();
			gzmsg << "Control log file opened: uav_control.log" << endl;
		} else {
			controlLogFileOpened = false;
			gzerr << "Failed to open control log file: uav_control.log" << endl;
		}
	}
	void logData() {
		if (!logFileOpened || !logFile.is_open()) return;
		
		// Only log when in stick mode
		if (strcmp(controller_mode, "stick") != 0) return;
		
		double t = model->GetWorld()->SimTime().Double();
		
		logFile.precision(6);
		logFile << std::fixed;
		
		// Check attitude control mode
		if (strcmp(attitude_mode, "pdpi") == 0) {
			// PDPI mode: log mFi (Kalman filtered) and actual angles
			float roll_mFi = pdpiRoll.mFi* 180.0f / M_PI;   // Kalman filtered roll angle from PDPI
			float pitch_mFi = pdpiPitch.mFi* 180.0f / M_PI; // Kalman filtered pitch angle from PDPI
			float roll_deg = roll_H * 180.0f / M_PI;   // Actual roll angle
			float pitch_deg = pitch_H * 180.0f / M_PI; // Actual pitch angle
			
			// Format: t roll_mFi pitch_mFi roll_angle pitch_angle (all in degrees)
			logFile << t << " "
			        << roll_mFi * 180.0f / M_PI << " " << pitch_mFi * 180.0f / M_PI << " "
			        << roll_deg << " " << pitch_deg << "\n";
		} else {
			// PID mode: log original format (raw and filtered IMU angles)
			float roll_H_deg = roll_H * 180.0f / M_PI;
			float roll_H_filtered_deg = roll_H_filtered * 180.0f / M_PI;
			float pitch_H_deg = pitch_H * 180.0f / M_PI;
			float pitch_H_filtered_deg = pitch_H_filtered * 180.0f / M_PI;
			
			// Format: t roll_H roll_H_filtered pitch_H pitch_H_filtered (all in degrees)
			logFile << t << " "
			        << roll_H_deg << " " << roll_H_filtered_deg << " "
			        << pitch_H_deg << " " << pitch_H_filtered_deg << "\n";
		}
		
		// Flush periodically (every 100 lines or so)
		static int lineCount = 0;
		if (++lineCount % 100 == 0) {
			logFile.flush();
		}
	}
	void logControlData() {
		if (!controlLogFileOpened || !controlLogFile.is_open()) return;
		
		double t = model->GetWorld()->SimTime().Double();
		
		// Determine mode: 0 = stick, 1 = auto
		int mode = (strcmp(controller_mode, "stick") == 0) ? 0 : 1;
		
		// Get pdpiRoll.Us (only valid when using PDPI controller)
		float pdpiRoll_Us = pdpiRoll.Us;
		
		// Format: t mode target_roll target_pitch pdpiRoll_Us
		controlLogFile.precision(6);
		controlLogFile << std::fixed;
		controlLogFile << t << " "
		               << mode << " "
		               << target_roll << " " << target_pitch << " "
		               << pdpiRoll_Us << "\n";
		
		// Flush periodically (every 100 lines or so)
		static int controlLineCount = 0;
		if (++controlLineCount % 100 == 0) {
			controlLogFile.flush();
		}
	}
	
	~ModelFlix() {
		if (logFileOpened && logFile.is_open()) {
			logFile.close();
			gzmsg << "Log file closed" << endl;
		}
		if (controlLogFileOpened && controlLogFile.is_open()) {
			controlLogFile.close();
			gzmsg << "Control log file closed" << endl;
		}
	}
};

GZ_REGISTER_MODEL_PLUGIN(ModelFlix)