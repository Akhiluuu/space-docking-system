#pragma once
#ifndef _DOCKING_SIMULATION_H_
#define _DOCKING_SIMULATION_H_

#include "Scenario.h"
#include "DockingPhysicsEngine.h"
#include <memory>
#include <vector>

enum class DockingFeasibilityState {
    APPROACHING,
    VALID_CONTACT,
    INVALID_CONTACT
};

struct DockingTolerances {
    double lateralOffset;      // meters
    double angularErrorRP;     // degrees
    double angularErrorYaw;    // degrees
    double collisionMargin;    // meters
};

/**
 * Main docking simulation controller.
 * Manages the simulation loop and orchestrates physics updates.
 */
class DockingSimulation {
public:
    /**
     * Initialize simulation with a scenario
     */
    explicit DockingSimulation(std::shared_ptr<Scenario> scenario);
    
    /**
     * Initialize simulation with a scenario file path
     * Loads the scenario from XML
     */
    explicit DockingSimulation(const std::string& scenarioPath);
    
    /**
     * Run the complete simulation
     * Prints pose updates at each timestep to console
     */
    void run();
    
    /**
     * Run simulation and collect all poses
     * @return Vector of poses at each timestep
     */
    std::vector<DockingPose> runAndCollectPoses();
    
    /**
     * Advance simulation by one timestep
     * @return true if simulation has more steps, false if complete
     */
    bool step();
    
    const DockingPose& getCurrentPose() const;
    const DockingPose& getCombinedPose() const;
    const DockingPose& getBaseTargetPose() const;
    const DockingPose& getSuspendedTargetPose() const;
    double getCurrentTime() const;
    bool isComplete() const;
    std::shared_ptr<Scenario> getScenario() const;
    DockingFeasibilityState getDockingState() const { return lastDockingState; }

private:
    std::shared_ptr<Scenario> scenario;
    DockingPhysicsEngine physicsEngine;
    DockingPose baseTargetPose;
    DockingPose suspendedTargetPose;
    DockingPose combinedPose;
    double currentTime;
    size_t nextEventIndex;  // Index of next event to process
    DockingFeasibilityState lastDockingState = DockingFeasibilityState::APPROACHING;
    
    /**
     * Find the next event at a given time
     */
    const ContactEvent* getEventAtTime(double time);
    
    /**
     * Print contact event diagnostic information
     */
    void printContactEventInfo(const ContactEvent& event) const;

    /**
     * Update the intended commanded pose of the base Stewart platform.
     */
    void updateBaseTargetPose(double timestep);

    /**
     * Recompute the effective actuation pose from base and suspended poses.
     */
    void updateCombinedPose();

    /**
     * Check invisible docking-face overlap and enforce simple contact rules.
     */
    void enforceDockingFaceConstraint(double timestep);

    /**
     * Print only when docking validity changes so the logs stay readable.
     */
    void printDockingStateChange(DockingFeasibilityState state,
                                 double lateralOffset,
                                 double angularErrorDeg,
                                 double penetration) const;
    
    /**
     * Get the appropriate tolerances for the current scenario.
     */
    DockingTolerances getTolerancesForScenario() const;
    
    /**
     * Print pose header
     */
    static void printPoseHeader();
    
    /**
     * Print pose data for current state
     */
    void printPoseData() const;
};

#endif // _DOCKING_SIMULATION_H_
