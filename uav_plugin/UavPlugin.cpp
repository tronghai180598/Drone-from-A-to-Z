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
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
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

#include "control.ino"
#include "KrenCtrl.hpp"

// Global variables
Vector gyro, acc, pos, vel;
extern float X_set;
extern float Y_set;
extern float Z_set;
extern float yaw_set;
extern char controller_mode[16];  // Controller mode: "stick" or "auto"
extern float target_roll;
extern float target_pitch;
extern class KrenCtrl pdpiRoll;
Quaternion attitude{1,0,0,0};
float __micros;
float dt;
float mass;

// Gyro low-pass filter parameters
static constexpr float GYRO_LPF_CUTOFF_HZ = 40.0f;  // Cutoff frequency in Hz (adjustable)
static Vector gyro_filtered{0, 0, 0};
static bool gyro_filter_initialized = false;

// IMU variables (defined here to avoid multiple definition)
float roll_H = 0.0f, pitch_H = 0.0f, yaw_H = 0.0f;
float roll_H_filtered = 0.0f, pitch_H_filtered = 0.0f, yaw_H_filtered = 0.0f;

extern bool armed;
extern bool heart_active;
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
	transport::PublisherPtr motorPub[4];
	ofstream logFile;
	bool logFileOpened;
	ofstream controlLogFile;
	bool controlLogFileOpened;
	ofstream positionLogFile;
	bool positionLogFileOpened;
	
	// UDP socket for sending position to Qt
	int udpSocket;
	struct sockaddr_in udpAddr;
	

public:
	ModelFlix() : logFileOpened(false), controlLogFileOpened(false), positionLogFileOpened(false), udpSocket(-1) {}
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
		this->model = _parent;
		this->body = this->model->GetLink("body");
		this->imu = dynamic_pointer_cast<sensors::ImuSensor>(sensors::get_sensor(model->GetScopedName(true) + "::body::imu")); // default::flix::body::imu
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelFlix::OnUpdate, this));
		this->resetConnection = event::Events::ConnectWorldReset(std::bind(&ModelFlix::OnReset, this));
		initNode();
		initUDP();
		initLogFile();
		initControlLogFile();
		initPositionLogFile();
		std::thread(simCliThread).detach();
		gzmsg << "UAV plugin loaded" << endl;
	}
	
	void initUDP() {
		udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
		if (udpSocket < 0) {
			gzerr << "Failed to create UDP socket" << endl;
			return;
		}
		
		memset(&udpAddr, 0, sizeof(udpAddr));
		udpAddr.sin_family = AF_INET;
		udpAddr.sin_port = htons(5005);
		udpAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
		
		gzmsg << "UDP socket initialized for port 5005" << endl;
	}
	
	void publishUDP() {
		if (udpSocket < 0) return;
		
		double t = model->GetWorld()->SimTime().Double();
		char buffer[256];
		int len = snprintf(buffer, sizeof(buffer), "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f",
			t, pos.x, pos.y, pos.z,
			attitude.x, attitude.y, attitude.z, attitude.w);
		
		sendto(udpSocket, buffer, len, 0, (struct sockaddr*)&udpAddr, sizeof(udpAddr));
	}

	void OnReset() {
		attitude = Quaternion();
		armed = false;
		// Reset gyro filter
		gyro_filter_initialized = false;
		gyro_filtered = Vector(0, 0, 0);
		// Close and reopen log file on reset
		if (logFileOpened) {
			logFile.close();
			logFileOpened = false;
		}
		if (controlLogFileOpened) {
			controlLogFile.close();
			controlLogFileOpened = false;
		}
		if (positionLogFileOpened) {
			positionLogFile.close();
			positionLogFileOpened = false;
		}
		initLogFile();
		initControlLogFile();
		initPositionLogFile();
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
		// Read IMU data
		Vector gyro_raw = Vector(imu->AngularVelocity().X(), imu->AngularVelocity().Y(), imu->AngularVelocity().Z());
		
		// Apply low-pass filter to gyro
		if (!gyro_filter_initialized) {
			gyro_filtered = gyro_raw;
			gyro_filter_initialized = true;
		} else {
			// Low-pass filter: alpha = dt / (dt + tau), where tau = 1/(2*pi*fc)
			const float tau = 1.0f / (2.0f * M_PI * GYRO_LPF_CUTOFF_HZ);
			const float alpha = dt / (dt + tau);
			gyro_filtered.x = alpha * gyro_raw.x + (1.0f - alpha) * gyro_filtered.x;
			gyro_filtered.y = alpha * gyro_raw.y + (1.0f - alpha) * gyro_filtered.y;
			gyro_filtered.z = alpha * gyro_raw.z + (1.0f - alpha) * gyro_filtered.z;
		}
		gyro = gyro_filtered;  // Use filtered gyro for control
		
		acc = Vector(imu->LinearAcceleration().X(), imu->LinearAcceleration().Y(), imu->LinearAcceleration().Z());
		
		// Update attitude from IMU (with noise) - this calculates roll_H, pitch_H from accelerometer
		imu_update(acc);
		
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
		
		control();
		applyMotorForces();
		logData();  // Log attitude data
		logControlData();  // Log control signals
		logPositionData();  // Log position data (X, Y, Z)
		publishUDP();  // Publish position to Qt via UDP
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
		string ns = "~/" + model->GetName();
		// create motors output topics for debugging and plotting
		motorPub[0] = nodeHandle->Advertise<msgs::Int>(ns + "/motor0");
		motorPub[1] = nodeHandle->Advertise<msgs::Int>(ns + "/motor1");
		motorPub[2] = nodeHandle->Advertise<msgs::Int>(ns + "/motor2");
		motorPub[3] = nodeHandle->Advertise<msgs::Int>(ns + "/motor3");
	}

	void initLogFile() {
		logFile.open("uav_attitude.log", ios::out | ios::trunc);
		if (logFile.is_open()) {
			logFileOpened = true;
			// Write header - format only
			logFile << "# t roll_H roll_H_filtered pitch_H pitch_H_filtered\n";
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
	void initPositionLogFile() {
		positionLogFile.open("uav_position.log", ios::out | ios::trunc);
		if (positionLogFile.is_open()) {
			positionLogFileOpened = true;
			// Write header - format only
			positionLogFile << "# t X Y Z X_set Y_set Z_set\n";
			positionLogFile.flush();
			gzmsg << "Position log file opened: uav_position.log" << endl;
		} else {
			positionLogFileOpened = false;
			gzerr << "Failed to open position log file: uav_position.log" << endl;
		}
	}
	void logData() {
		if (!logFileOpened || !logFile.is_open()) return;
		
		// Only log when in stick mode
		if (strcmp(controller_mode, "stick") != 0) return;
		
		double t = model->GetWorld()->SimTime().Double();
		
		// Convert IMU angles to degrees (raw and filtered)
		float roll_H_deg = roll_H * 180.0f / M_PI;
		float roll_H_filtered_deg = roll_H_filtered * 180.0f / M_PI;
		float pitch_H_deg = pitch_H * 180.0f / M_PI;
		float pitch_H_filtered_deg = pitch_H_filtered * 180.0f / M_PI;
		
		// Format: t roll_H roll_H_filtered pitch_H pitch_H_filtered (all in degrees)
		logFile.precision(6);
		logFile << std::fixed;
		logFile << t << " "
		        << roll_H_deg << " " << roll_H_filtered_deg << " "
		        << pitch_H_deg << " " << pitch_H_filtered_deg << "\n";
		
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
		
		// Get pdpiRoll.Us (only valid in auto mode, but log it for both modes)
		float pdpiRoll_Us = (mode == 1) ? pdpiRoll.Us : 0.0f;
		
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
	void logPositionData() {
		if (!positionLogFileOpened || !positionLogFile.is_open()) return;
		
		double t = model->GetWorld()->SimTime().Double();
		
		// Format: t X Y Z X_set Y_set Z_set
		positionLogFile.precision(6);
		positionLogFile << std::fixed;
		positionLogFile << t << " "
		                << pos.x << " " << pos.y << " " << pos.z << " "
		                << X_set << " " << Y_set << " " << Z_set << "\n";
		
		// Flush periodically (every 100 lines or so)
		static int positionLineCount = 0;
		if (++positionLineCount % 100 == 0) {
			positionLogFile.flush();
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
		if (positionLogFileOpened && positionLogFile.is_open()) {
			positionLogFile.close();
			gzmsg << "Position log file closed" << endl;
		}
	}
};

GZ_REGISTER_MODEL_PLUGIN(ModelFlix)