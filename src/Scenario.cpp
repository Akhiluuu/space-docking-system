#include "Scenario.h"

// Initialize static sensor positions (hexagonal arrangement)
// These are in the body frame of the platform
const std::map<int, Eigen::Vector3d> DockingPose::SENSOR_POSITIONS = {
    {1, Eigen::Vector3d(0.5, 0.0, 0.0)},
    {2, Eigen::Vector3d(0.25, 0.433, 0.0)},
    {3, Eigen::Vector3d(-0.25, 0.433, 0.0)},
    {4, Eigen::Vector3d(-0.5, 0.0, 0.0)},
    {5, Eigen::Vector3d(-0.25, -0.433, 0.0)},
    {6, Eigen::Vector3d(0.25, -0.433, 0.0)}
};
