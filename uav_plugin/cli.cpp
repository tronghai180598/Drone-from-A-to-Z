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
extern char attitude_mode[16];    // Attitude control mode: "pid" or "pdpi"
extern Quaternion attitude;  // Current attitude from UavPlugin.cpp
extern float Z_set;
extern float mass;

// Map variable names to their pointers and descriptions
struct VariableInfo {
    float* ptr;
    std::string description;
    std::string unit;
};

std::map<std::string, VariableInfo> variables;

// Initialize variable map
void initVariables() {
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
    } else if (cmd == "attitude") {
        std::string mode;
        if (iss >> mode) {
            if (mode == "pid" || mode == "pdpi") {
                strncpy(attitude_mode, mode.c_str(), sizeof(attitude_mode) - 1);
                attitude_mode[sizeof(attitude_mode) - 1] = '\0';
                std::cout << "[CLI] >>> Attitude control mode set to: " << attitude_mode << "\n";
                std::cout.flush();
            } else {
                std::cout << "[CLI] Error: Invalid attitude mode. Use 'attitude pid' or 'attitude pdpi'\n";
            }
        } else {
            std::cout << "[CLI] Current attitude control mode: " << attitude_mode << "\n";
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
        std::cout << "[CLI] Commands: arm, disarm, mode <stick/auto>, set <var> <value>, get <var>, list\n";
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
    std::cout << "[CLI]   - mode <stick/auto>: set controller mode\n";
    std::cout << "[CLI]   - attitude <pid/pdpi>: set attitude control mode (PID or PDPI)\n";
    std::cout << "[CLI]   - set <var> <value>: set variable (e.g., 'set z 3' or 'set roll_set 0.1')\n";
    std::cout << "[CLI]   - get <var>: get variable value\n";
    std::cout << "[CLI]   - list: list all variables\n";
    std::cout << "[CLI] Write commands to: " << cmdFile << "\n";
    std::cout << "[CLI] Note: RC control via joystick - Stick mode works without arm!\n";
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
