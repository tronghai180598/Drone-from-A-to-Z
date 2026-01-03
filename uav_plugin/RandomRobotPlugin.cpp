#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <random>
#include <cmath>
#include <iostream>

using namespace gazebo;
using namespace std;

class RandomRobotPlugin : public ModelPlugin {
private:
    physics::ModelPtr model;
    physics::LinkPtr baseLink;
    event::ConnectionPtr updateConnection;
    
    double lastDirectionChange;
    double directionChangeInterval;
    double currentSpeed;
    double currentAngularVel;
    double targetX, targetY;
    double maxSpeed;
    double maxAngularVel;
    double searchRadius;
    
    random_device rd;
    mt19937 gen;
    uniform_real_distribution<double> speedDist;
    uniform_real_distribution<double> angleDist;
    uniform_real_distribution<double> timeDist;

public:
    RandomRobotPlugin() : gen(rd()) {
        directionChangeInterval = 8.0;
        lastDirectionChange = 0.0;
        maxSpeed = 1.2;
        maxAngularVel = 0.2;
        searchRadius = 15.0;
        speedDist = uniform_real_distribution<double>(0.5, maxSpeed);
        angleDist = uniform_real_distribution<double>(0.0, 2.0 * M_PI);
        timeDist = uniform_real_distribution<double>(5.0, 12.0);
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        this->model = _parent;
        this->baseLink = this->model->GetLink("base_link");
        
        if (!this->baseLink) {
            gzerr << "RandomRobotPlugin: base_link not found!" << endl;
            return;
        }
        
        if (_sdf && _sdf->HasElement("max_speed")) {
            maxSpeed = _sdf->Get<double>("max_speed");
            speedDist = uniform_real_distribution<double>(0.1, maxSpeed);
        }
        
        if (_sdf && _sdf->HasElement("max_angular_vel")) {
            maxAngularVel = _sdf->Get<double>("max_angular_vel");
        }
        
        if (_sdf && _sdf->HasElement("search_radius")) {
            searchRadius = _sdf->Get<double>("search_radius");
        }
        
        if (_sdf && _sdf->HasElement("direction_change_interval")) {
            directionChangeInterval = _sdf->Get<double>("direction_change_interval");
        }
        
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&RandomRobotPlugin::OnUpdate, this));
        
        ignition::math::Pose3d pose = this->model->WorldPose();
        targetX = pose.Pos().X();
        targetY = pose.Pos().Y();
        
        gzmsg << "RandomRobotPlugin loaded for model: " << this->model->GetName() << endl;
        gzmsg << "  Max speed: " << maxSpeed << " m/s" << endl;
        gzmsg << "  Max angular velocity: " << maxAngularVel << " rad/s" << endl;
    }

    void OnUpdate() {
        double currentTime = this->model->GetWorld()->SimTime().Double();
        
        if (currentTime - lastDirectionChange >= directionChangeInterval) {
            chooseNewDirection();
            lastDirectionChange = currentTime;
            directionChangeInterval = timeDist(gen);
        }
        
        moveTowardsTarget();
        avoidBoundaries();
    }

private:
    void chooseNewDirection() {
        ignition::math::Pose3d pose = this->model->WorldPose();
        double currentX = pose.Pos().X();
        double currentY = pose.Pos().Y();
        double currentYaw = pose.Rot().Yaw();
        
        double angleChange = (gen() % 200 - 100) / 1000.0;
        double angle = currentYaw + angleChange;
        
        double distance = speedDist(gen) * directionChangeInterval * 1.5;
        
        targetX = currentX + distance * cos(angle);
        targetY = currentY + distance * sin(angle);
        
        currentSpeed = speedDist(gen);
        currentAngularVel = maxAngularVel * 0.1;
    }
    
    void moveTowardsTarget() {
        ignition::math::Pose3d pose = this->model->WorldPose();
        double currentX = pose.Pos().X();
        double currentY = pose.Pos().Y();
        double currentYaw = pose.Rot().Yaw();
        
        double dx = targetX - currentX;
        double dy = targetY - currentY;
        double distance = sqrt(dx * dx + dy * dy);
        
        if (distance < 0.1) {
            chooseNewDirection();
            return;
        }
        
        double targetAngle = atan2(dy, dx);
        double angleError = targetAngle - currentYaw;
        
        while (angleError > M_PI) angleError -= 2.0 * M_PI;
        while (angleError < -M_PI) angleError += 2.0 * M_PI;
        
        double linearVel = currentSpeed;
        double angularVel = 0.0;
        
        if (fabs(angleError) > 0.15) {
            angularVel = currentAngularVel;
            if (angleError < 0) angularVel = -angularVel;
            linearVel *= 0.1;
        }
        
        ignition::math::Vector3d linearVelocity(linearVel * cos(currentYaw), 
                                                 linearVel * sin(currentYaw), 
                                                 0.0);
        ignition::math::Vector3d angularVelocity(0.0, 0.0, angularVel);
        
        this->baseLink->SetLinearVel(linearVelocity);
        this->baseLink->SetAngularVel(angularVelocity);
        
        ignition::math::Pose3d currentPose = this->baseLink->WorldPose();
        double currentZ = currentPose.Pos().Z();
        if (currentZ > 0.1) {
            ignition::math::Pose3d fixedPose(currentPose.Pos().X(), 
                                             currentPose.Pos().Y(), 
                                             0.1, 
                                             currentPose.Rot().Roll(), 
                                             currentPose.Rot().Pitch(), 
                                             currentPose.Rot().Yaw());
            this->baseLink->SetWorldPose(fixedPose);
        }
        
        ignition::math::Vector3d currentAngVel = this->baseLink->WorldAngularVel();
        ignition::math::Vector3d stabilizedAngVel(0.0, 0.0, currentAngVel.Z());
        this->baseLink->SetAngularVel(stabilizedAngVel);
    }
    
    void avoidBoundaries() {
        ignition::math::Pose3d pose = this->model->WorldPose();
        double x = pose.Pos().X();
        double y = pose.Pos().Y();
        
        double boundary = searchRadius;
        double margin = 1.0;
        
        if (fabs(x) > boundary - margin || fabs(y) > boundary - margin) {
            double centerX = 0.0;
            double centerY = 0.0;
            
            targetX = centerX + (gen() % 200 - 100) / 10.0;
            targetY = centerY + (gen() % 200 - 100) / 10.0;
            
            lastDirectionChange = this->model->GetWorld()->SimTime().Double();
        }
    }
};

GZ_REGISTER_MODEL_PLUGIN(RandomRobotPlugin)
