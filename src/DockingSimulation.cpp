#include "DockingSimulation.h"
#include "ScenarioLoader.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDockProxyRadius = 0.030;
constexpr double kDockProxyHalfThickness = 0.0015;
constexpr double kDockProxyAxialMargin = 0.0020;

DockingPose interpolatePose(const DockingPose& startPose, const DockingPose& endPose, double alpha) {
    DockingPose pose;
    pose.x = startPose.x + (endPose.x - startPose.x) * alpha;
    pose.y = startPose.y + (endPose.y - startPose.y) * alpha;
    pose.z = startPose.z + (endPose.z - startPose.z) * alpha;
    pose.roll = startPose.roll + (endPose.roll - startPose.roll) * alpha;
    pose.pitch = startPose.pitch + (endPose.pitch - startPose.pitch) * alpha;
    pose.yaw = startPose.yaw + (endPose.yaw - startPose.yaw) * alpha;
    return pose;
}

DockingPose getScenarioApproachTarget(const std::string& scenarioName) {
    DockingPose targetPose;

    if (scenarioName == "off_center_contact" || scenarioName == "offcenter_docking_new") {
        targetPose.x = 0.012;
        targetPose.y = 0.0;
        targetPose.z = 0.0610;
        targetPose.roll = 0.0;
        targetPose.pitch = 0.0;
        targetPose.yaw = 0.34;
        return targetPose;
    }

    // Default to a centered, near-contact docking pose for normal docking.
    targetPose.x = 0.0;
    targetPose.y = 0.0;
    targetPose.z = 0.0640;
    targetPose.roll = 0.0;
    targetPose.pitch = 0.0;
    targetPose.yaw = 0.0;
    return targetPose;
}

DockingPose getScenarioApproachStart(const std::string& scenarioName) {
    DockingPose startPose;
    // Set z to -0.015 to pull the platform down so legs are fully retracted/inside casing
    startPose.z = -0.010; 
    return startPose;
}

struct DockingProxyMetrics {
    double lateralOffset = 0.0;
    double axialGap = 0.0;
    bool facesOverlap = false;
};

DockingProxyMetrics computeProxyMetrics(const DockingPose& lowerFacePose,
                                        const DockingPose& upperFacePose,
                                        double collisionMargin) {
    DockingProxyMetrics metrics;
    const double dx = lowerFacePose.x - upperFacePose.x;
    const double dy = lowerFacePose.y - upperFacePose.y;
    metrics.lateralOffset = std::sqrt(dx * dx + dy * dy);
    metrics.axialGap = upperFacePose.z - lowerFacePose.z;
    metrics.facesOverlap =
        metrics.lateralOffset <= (kDockProxyRadius * 2.0 + collisionMargin) &&
        metrics.axialGap <= (kDockProxyHalfThickness * 2.0 + collisionMargin);
    return metrics;
}

double wrapAngle(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

double radiansToDegrees(double angle) {
    return angle * (180.0 / kPi);
}
}

DockingSimulation::DockingSimulation(std::shared_ptr<Scenario> scenario)
    : scenario(scenario),
      physicsEngine(scenario->platformParams),
      currentTime(0.0),
      nextEventIndex(0) {
    if (!scenario) throw std::runtime_error("Scenario cannot be null");
    updateCombinedPose();
}

DockingSimulation::DockingSimulation(const std::string& scenarioPath)
    : physicsEngine(Scenario().platformParams),
      currentTime(0.0),
      nextEventIndex(0) {
    scenario = ScenarioLoader::loadFromFile(scenarioPath);
    if (!scenario) throw std::runtime_error("Failed to load scenario from: " + scenarioPath);
    physicsEngine = DockingPhysicsEngine(scenario->platformParams);
    updateCombinedPose();
}

const ContactEvent* DockingSimulation::getEventAtTime(double time) {
    const double timeTolerance = 1e-6;
    if (nextEventIndex < scenario->events.size()) {
        const ContactEvent& event = scenario->events[nextEventIndex];
        if (std::abs(event.time - time) < timeTolerance) return &event;
    }
    return nullptr;
}

void DockingSimulation::printContactEventInfo(const ContactEvent& event) const {
    Eigen::Vector3d netForce = physicsEngine.computeNetForce(event);
    Eigen::Vector3d netTorque = physicsEngine.computeNetTorque(event);
    Eigen::Vector3d contactPoint = physicsEngine.estimateContactPoint(netForce, netTorque);
    
    std::cout << "\n>>> CONTACT EVENT at t=" << std::fixed << std::setprecision(4) 
              << event.time << " s <<<\n";
    std::cout << "  Net Force: [" << netForce.norm() << " N] = (" 
              << netForce(0) << ", " << netForce(1) << ", " << netForce(2) << ")\n";
    std::cout << "  Net Torque: [" << netTorque.norm() << " Nm]\n";
    std::cout << "  Contact Point: [" << contactPoint(0) << ", " << contactPoint(1) << ", " << contactPoint(2) << "]\n";
    
    // DEBUG: Show individual sensor forces
    std::cout << "  Individual Forces from " << event.forces.size() << " sensors:\n";
    for (const auto& fr : event.forces) {
        std::cout << "    Sensor " << fr.sensorID << ": [" << fr.fx << ", " << fr.fy << ", " << fr.fz << "]\n";
    }
}

bool DockingSimulation::step() {
    double timestep = scenario->config.timestep;
    currentTime += timestep;

    updateBaseTargetPose(timestep);

    // Check for XML Events (like impact forces)
    const ContactEvent* event = getEventAtTime(currentTime);
    if (event) {
        printContactEventInfo(*event);
        physicsEngine.updatePoseFromForces(*event, suspendedTargetPose, timestep);
        nextEventIndex++;
    } else {
        physicsEngine.updatePoseByCoasting(suspendedTargetPose, timestep);
    }

    updateCombinedPose();
    enforceDockingFaceConstraint(timestep);
    
    // We return true indefinitely so the simulation continues to drift after the XML duration ends
    return true; 
}

std::vector<DockingPose> DockingSimulation::runAndCollectPoses() {
    std::vector<DockingPose> poses;
    double duration = scenario->config.duration;
    double timestep = scenario->config.timestep;
    int totalSteps = static_cast<int>(duration / timestep);
    
    for (int stepIdx = 0; stepIdx <= totalSteps; ++stepIdx) {
        step();
        poses.push_back(combinedPose);
    }
    return poses;
}

void DockingSimulation::run() {
    double duration = scenario->config.duration;
    double timestep = scenario->config.timestep;
    int totalSteps = static_cast<int>(duration / timestep);
    
    std::cout << "=== Physics Simulation ===\n";
    std::cout << "Duration: " << duration << " s\n";
    std::cout << "Timestep: " << timestep << " s (frequency: " << (1.0/timestep) << " Hz)\n";
    std::cout << "Total steps: " << totalSteps << "\n";
    std::cout << "Platform mass: " << scenario->platformParams.mass << " kg\n\n";

    printPoseHeader();
    printPoseData();

    for (int stepIdx = 1; stepIdx <= totalSteps; ++stepIdx) {
        step();
        printPoseData();
    }
    
    std::cout << "----------------------------------------------------------------------------------\n";
    std::cout << "\nSimulation completed.\n";
}

void DockingSimulation::printPoseHeader() {
    std::cout << std::left << std::setw(10) << "Time (s)" 
              << std::setw(12) << "X (m)" 
              << std::setw(12) << "Y (m)" 
              << std::setw(12) << "Z (m)" 
              << std::setw(12) << "Roll (rad)" 
              << std::setw(12) << "Pitch (rad)" 
              << std::setw(12) << "Yaw (rad)" << "\n";
    std::cout << std::string(82, '-') << "\n";
}

void DockingSimulation::printPoseData() const {
    std::cout << std::fixed << std::setprecision(6)
              << std::left << std::setw(10) << currentTime
              << std::setw(12) << combinedPose.x
              << std::setw(12) << combinedPose.y
              << std::setw(12) << combinedPose.z
              << std::setw(12) << combinedPose.roll
              << std::setw(12) << combinedPose.pitch
              << std::setw(12) << combinedPose.yaw << "\n";
}

void DockingSimulation::updateBaseTargetPose(double timestep) {
    (void)timestep;
    const double kApproachDuration = 1.5;

    const DockingPose startPose = getScenarioApproachStart(scenario ? scenario->name : std::string());
    const DockingPose targetPose = getScenarioApproachTarget(scenario ? scenario->name : std::string());
    const double alpha = std::clamp(currentTime / kApproachDuration, 0.0, 1.0);

    baseTargetPose = interpolatePose(startPose, targetPose, alpha);
}

void DockingSimulation::updateCombinedPose() {
    combinedPose = DockingPose::compose(baseTargetPose, suspendedTargetPose);
}

void DockingSimulation::enforceDockingFaceConstraint(double timestep) {
    (void)timestep;
    const DockingTolerances tol = getTolerancesForScenario();

    const DockingPose upperFacePose = getScenarioApproachTarget(scenario ? scenario->name : std::string());
    const DockingProxyMetrics proxyMetrics = computeProxyMetrics(combinedPose, upperFacePose, tol.collisionMargin);

    const double dRoll = std::abs(radiansToDegrees(wrapAngle(combinedPose.roll - upperFacePose.roll)));
    const double dPitch = std::abs(radiansToDegrees(wrapAngle(combinedPose.pitch - upperFacePose.pitch)));
    const double dYaw = std::abs(radiansToDegrees(wrapAngle(combinedPose.yaw - upperFacePose.yaw)));

    // The proxy collision layer uses invisible discs around the docking ring.
    const bool inContactWindow = proxyMetrics.facesOverlap;
    const double penetration = -proxyMetrics.axialGap;
    const bool alignmentValid =
        proxyMetrics.lateralOffset <= tol.lateralOffset &&
        dRoll <= tol.angularErrorRP &&
        dPitch <= tol.angularErrorRP &&
        dYaw <= tol.angularErrorYaw;

    DockingFeasibilityState newState = DockingFeasibilityState::APPROACHING;
    if (inContactWindow) {
        newState = alignmentValid
            ? DockingFeasibilityState::VALID_CONTACT
            : DockingFeasibilityState::INVALID_CONTACT;
    }

    if (newState != lastDockingState) {
        // Use max error for logging
        const double maxAngError = std::max({dRoll, dPitch, dYaw});
        printDockingStateChange(newState, proxyMetrics.lateralOffset, maxAngError, penetration);
        lastDockingState = newState;
    }

    if (!inContactWindow || alignmentValid) {
        return;
    }

    // Invalid overlap: keep the pose stable and only damp residual motion.
    physicsEngine.overrideLinearVelocityZ(std::min(physicsEngine.getLinearVelocity().z(), 0.0));
    physicsEngine.applyConstraintDamping(0.85, 0.90);
}

void DockingSimulation::printDockingStateChange(DockingFeasibilityState state,
                                                double lateralOffset,
                                                double angularErrorDeg,
                                                double penetration) const {
    std::cout << std::fixed << std::setprecision(4);
    if (state == DockingFeasibilityState::APPROACHING) {
        std::cout << "[DOCKING] APPROACHING\n";
        return;
    }

    if (state == DockingFeasibilityState::VALID_CONTACT) {
        std::cout << "[DOCKING] VALID CONTACT | lateral=" << lateralOffset
                  << " m | angular=" << angularErrorDeg
                  << " deg | penetration=" << penetration << " m\n";
        return;
    }

    if (scenario && scenario->name == "dynamic_vortex_new") {
        std::cout << "[STATUS] uncessful docking\n";
    } else {
        std::cout << "[DOCKING] BAD DOCKING / NOT POSSIBLE | lateral=" << lateralOffset
                  << " m | angular=" << angularErrorDeg
                  << " deg | penetration=" << penetration << " m\n";
    }
}

DockingTolerances DockingSimulation::getTolerancesForScenario() const {
    const std::string name = (scenario ? scenario->name : "");
    if (name == "off_center_contact" || name == "offcenter_docking_new") {
        return {0.0030, 0.8, 0.8, 0.0015};
    }

    return {0.006, 2.0, 2.0, 0.0025};
}

const DockingPose& DockingSimulation::getCurrentPose() const { return combinedPose; }
const DockingPose& DockingSimulation::getCombinedPose() const { return combinedPose; }
const DockingPose& DockingSimulation::getBaseTargetPose() const { return baseTargetPose; }
const DockingPose& DockingSimulation::getSuspendedTargetPose() const { return suspendedTargetPose; }
double DockingSimulation::getCurrentTime() const { return currentTime; }
bool DockingSimulation::isComplete() const { return false; } // Never complete, stays drifting
std::shared_ptr<Scenario> DockingSimulation::getScenario() const { return scenario; }
