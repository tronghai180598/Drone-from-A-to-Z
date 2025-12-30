#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include <unistd.h>
#include <sstream>
#include <map>
#include <functional>
#include <algorithm>
#include <cmath>
#include <cstring>
#include "quaternion.h"

// Forward declarations (control.ino is included in UavPlugin.cpp, not here)
// Helper lambda for clamping (clampf is static in control.ino, so we define our own)
auto clamp = [](float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; };
extern bool armed;
extern float roll_set;
extern float pitch_set;
extern float yaw_set;
extern char controller_mode[16];  // Controller mode: "stick" or "auto"
extern Quaternion attitude;  // Current attitude from UavPlugin.cpp
extern float Z_set;
extern float X_set;
extern float Y_set;
extern float mass;
extern bool heartMode;  // Heart trajectory mode flag

// Map variable names to their pointers and descriptions
struct VariableInfo {
    float* ptr;
    std::string description;
    std::string unit;
};

std::map<std::string, VariableInfo> variables;

// Initialize variable map
void initVariables() {
    variables["x"] = {&X_set, "X position setpoint", "m"};
    variables["y"] = {&Y_set, "Y position setpoint", "m"};
    variables["z"] = {&Z_set, "Altitude setpoint", "m"};
    variables["roll_set"] = {&roll_set, "Roll angle setpoint", "rad"};
    variables["pitch_set"] = {&pitch_set, "Pitch angle setpoint", "rad"};
    variables["yaw_set"] = {&yaw_set, "Yaw angle setpoint", "rad"};
}

// Set a variable value
bool setVariable(const std::string& name, float value) {
    auto it = variables.find(name);
    if (it != variables.end()) {
        *(it->second.ptr) = value;
        std::cout << "[CLI] >>> " << it->second.description 
                  << " (" << name << ") = " << value;
        if (!it->second.unit.empty()) {
            std::cout << " " << it->second.unit;
        }
        std::cout << "\n";
        return true;
    }
    return false;
}

// Get a variable value
void getVariable(const std::string& name) {
    auto it = variables.find(name);
    if (it != variables.end()) {
        std::cout << "[CLI] " << it->second.description 
                  << " (" << name << ") = " << *(it->second.ptr);
        if (!it->second.unit.empty()) {
            std::cout << " " << it->second.unit;
        }
        std::cout << "\n";
    } else {
        std::cout << "[CLI] Unknown variable: " << name << "\n";
    }
}

// List all available variables
void listVariables() {
    std::cout << "[CLI] Available variables:\n";
    for (const auto& v : variables) {
        std::cout << "  - " << v.first << ": " << v.second.description;
        if (!v.second.unit.empty()) {
            std::cout << " [" << v.second.unit << "]";
        }
        std::cout << " (current: " << *(v.second.ptr) << ")\n";
    }
}
// Parse and execute command
void executeCommand(const std::string& line) {
    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;

    if (cmd == "arm") {
        armed = true;
        std::cout << "[CLI] >>> ARMED - UAV will take off!\n";
    } else if (cmd == "disarm") {
        armed = false;
        std::cout << "[CLI] >>> DISARMED - UAV will land!\n";
    }   else if (cmd == "rc") {
        float r, p, y;
        if (iss >> r >> p >> y) {
            // Clamp to ±45 degrees (±0.785 rad) for roll and pitch
            const float MAX_ATTITUDE_RAD = 0.785398163f;  // 45 degrees in radians
            // Inline clamp function
            auto clamp = [](float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; };
            roll_set  = clamp(r, -MAX_ATTITUDE_RAD, MAX_ATTITUDE_RAD);
            pitch_set = clamp(p, -MAX_ATTITUDE_RAD, MAX_ATTITUDE_RAD);
            yaw_set   = y;  // Yaw can be wider range
            
            // Get actual roll and pitch from attitude quaternion
            float actual_roll  = attitude.getRoll();
            float actual_pitch = attitude.getPitch();
            float actual_yaw   = attitude.getYaw();
            
            std::cout << "[CLI] >>> RC setpoints: roll_set=" << roll_set
                      << " pitch_set=" << pitch_set
                      << " yaw_set=" << yaw_set << "\n";
            std::cout << "[CLI] >>> Actual angles: roll=" << actual_roll
                      << " pitch=" << actual_pitch
                      << " yaw=" << actual_yaw << "\n";
            std::cout.flush();  // Force output immediately
        } else {
            std::cout << "[CLI] Error: Use 'rc <roll_rad> <pitch_rad> <yaw_rad>'\n";
        }
    } else if (cmd == "mode") {
        std::string mode;
        if (iss >> mode) {
            if (mode == "stick" || mode == "auto") {
                strncpy(controller_mode, mode.c_str(), sizeof(controller_mode) - 1);
                controller_mode[sizeof(controller_mode) - 1] = '\0';
                std::cout << "[CLI] >>> Controller mode set to: " << controller_mode << "\n";
                std::cout.flush();
            } else {
                std::cout << "[CLI] Error: Invalid mode. Use 'mode stick' or 'mode auto'\n";
            }
        } else {
            std::cout << "[CLI] Current controller mode: " << controller_mode << "\n";
        }
    } else if (cmd == "set") {
        std::string varName;
        float value;
        if (iss >> varName >> value) {
            // Convert to lowercase for case-insensitive
            std::transform(varName.begin(), varName.end(), varName.begin(), ::tolower);
            if (!setVariable(varName, value)) {
                std::cout << "[CLI] Error: Unknown variable '" << varName << "'\n";
                std::cout << "[CLI] Use 'list' to see available variables\n";
            }
        } else {
            std::cout << "[CLI] Error: Invalid format. Use 'set <variable> <value>'\n";
        }
    } else if (cmd == "get") {
        std::string varName;
        if (iss >> varName) {
            std::transform(varName.begin(), varName.end(), varName.begin(), ::tolower);
            getVariable(varName);
        } else {
            std::cout << "[CLI] Error: Use 'get <variable>'\n";
        }
    } else if (cmd == "list") {
        listVariables();
    } else if (!line.empty()) {
        std::cout << "[CLI] Unknown command: '" << line << "'\n";
        std::cout << "[CLI] Commands: arm, disarm, set <var> <value>, get <var>, list, heart <on/off>\n";
    }
}

void simCliThread() {
    initVariables();
    
    std::cout << "[CLI] Thread started - Monitoring commands...\n";
    std::cout.flush();
    
    std::string cmdFile = "/tmp/uav_cmd";
    
    // Tạo file nếu chưa có
    std::ofstream create(cmdFile);
    create.close();
    
    std::cout << "[CLI] Ready! Commands:\n";
    std::cout << "[CLI]   - arm/disarm: control UAV\n";
    std::cout << "[CLI]   - set <var> <value>: set variable (e.g., 'set z 3')\n";
    std::cout << "[CLI]   - get <var>: get variable value\n";
    std::cout << "[CLI]   - list: list all variables\n";
    std::cout << "[CLI] Write commands to: " << cmdFile << "\n";
    std::cout.flush();
    
    while (true) {
        std::ifstream file(cmdFile);
        if (file.is_open()) {
            std::string line;
            if (std::getline(file, line)) {
                if (!line.empty()) {
                    executeCommand(line);
                    std::cout.flush();
                }
                file.close();
                // Xóa nội dung file sau khi đọc
                std::ofstream clear(cmdFile, std::ios::trunc);
                clear.close();
            }
        }
        usleep(10000); // Đợi 10ms
    }
}
