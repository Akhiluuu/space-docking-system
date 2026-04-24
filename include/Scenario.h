#pragma once

#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <Eigen/Dense>

// Forward readings from a single load cell sensor
struct ForceReading {
    int sensorID;                      // Sensor 1-6
    double fx, fy, fz;                 // Force components (Newtons)
    
    Eigen::Vector3d getForceVector() const {
        return Eigen::Vector3d(fx, fy, fz);
    }
};

// Contact event with force readings
struct ContactEvent {
    double time;                        // Time when contact occurs (seconds)
    std::vector<ForceReading> forces;   // Force readings for all 6 sensors
    
    Eigen::Vector3d getNetForce() const {
        Eigen::Vector3d netForce = Eigen::Vector3d::Zero();
        for (const auto& f : forces) {
            netForce += f.getForceVector();
        }
        return netForce;
    }
};

// Platform parameters
struct PlatformParameters {
    double mass = 50.0;                        // kg
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();  // Inertia tensor
    
    PlatformParameters() {
        // Default inertia: diag(5, 5, 10) kg⋅m²
        inertia(0, 0) = 5.0;
        inertia(1, 1) = 5.0;
        inertia(2, 2) = 10.0;
    }
};

// Pose state for docking platform
struct DockingPose {
    double x, y, z;         // Position (meters)
    double roll, pitch, yaw; // Orientation (radians)
    
    // Sensor positions in body frame (hexagonal arrangement)
    static const std::map<int, Eigen::Vector3d> SENSOR_POSITIONS;
    
    DockingPose() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {}
    
    // Convert to position vector
    Eigen::Vector3d getPosition() const {
        return Eigen::Vector3d(x, y, z);
    }
    
    // Convert to orientation quaternion
    Eigen::Quaterniond getQuaternion() const {
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
        
        return yawAngle * pitchAngle * rollAngle;
    }

    static DockingPose fromPositionAndQuaternion(const Eigen::Vector3d& position,
                                                 const Eigen::Quaterniond& rotation) {
        DockingPose pose;
        pose.x = position.x();
        pose.y = position.y();
        pose.z = position.z();
        quaternionToEuler(rotation.normalized(), pose.roll, pose.pitch, pose.yaw);
        return pose;
    }

    static DockingPose compose(const DockingPose& basePose, const DockingPose& suspendedPose) {
        const Eigen::Quaterniond baseRotation = basePose.getQuaternion().normalized();
        const Eigen::Quaterniond suspendedRotation = suspendedPose.getQuaternion().normalized();

        const Eigen::Quaterniond combinedRotation = (baseRotation * suspendedRotation).normalized();
        const Eigen::Vector3d combinedPosition =
            basePose.getPosition() + baseRotation * suspendedPose.getPosition();

        return fromPositionAndQuaternion(combinedPosition, combinedRotation);
    }
    
    // Convert quaternion to Euler angles
    static void quaternionToEuler(const Eigen::Quaterniond& q,
                                   double& roll, double& pitch, double& yaw) {
        constexpr double kHalfPi = 1.5707963267948966;

        // Roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
        double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
        roll = std::atan2(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis rotation)
        double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(kHalfPi, sinp);
        else
            pitch = std::asin(sinp);
        
        // Yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
        double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }
};

// Simulation configuration
struct SimulationConfig {
    double duration = 5.0;       // Total simulation time (seconds)
    double timestep = 0.002;     // Timestep (seconds) - 500 Hz
    
    // Derived properties
    int getTotalSteps() const {
        return static_cast<int>(duration / timestep);
    }
};

// Complete scenario definition
struct Scenario {
    std::string name;
    SimulationConfig config;
    PlatformParameters platformParams;
    std::vector<ContactEvent> events;  // Contact events sorted by time
    
    Scenario() = default;
    explicit Scenario(const std::string& _name) : name(_name) {}
};
