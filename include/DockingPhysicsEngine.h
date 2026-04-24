#pragma once

#include "Scenario.h"
#include <vector>

/**
 * Physics engine for docking simulation.
 * Handles rigid body forward dynamics based on 6-axis load cell measurements.
 */
class DockingPhysicsEngine {
public:
    explicit DockingPhysicsEngine(const PlatformParameters& params);
    
    /**
     * Update the pose based on contact forces.
     * @param contactEvent The contact event with force readings
     * @param pose Current pose to update
     * @param timestep Time step for integration (seconds)
     */
    void updatePoseFromForces(const ContactEvent& contactEvent,
                               DockingPose& pose,
                               double timestep);
    
    /**
     * Update pose due to coasting (no new forces applied).
     * Platform moves with current velocity.
     * @param pose Current pose to update
     * @param timestep Time step for coasting (seconds)
     */
    void updatePoseByCoasting(DockingPose& pose, double timestep);

    /**
     * Apply extra damping and centering for gentle soft-docking behavior.
     * This keeps the suspended body aligned and helps it settle without oscillation.
     */
    void applySoftDockingStabilization(DockingPose& pose, double timestep);
    
    /**
     * Compute net force from all sensors
     */
    static Eigen::Vector3d computeNetForce(const ContactEvent& contactEvent);
    
    /**
     * Compute net torque about the platform center of mass
     * Uses cross product: tau = sum(r_i × F_i)
     */
    static Eigen::Vector3d computeNetTorque(const ContactEvent& contactEvent);
    
    /**
     * Estimate the contact point from net force and torque
     * Uses: Pc = (F_net × tau_net) / |F_net|^2
     */
    static Eigen::Vector3d estimateContactPoint(const Eigen::Vector3d& netForce,
                                                 const Eigen::Vector3d& netTorque);
    
    /**
     * Compute linear and angular velocities using rigid body dynamics
     */
    void computeAccelerations(const Eigen::Vector3d& netForce,
                              const Eigen::Vector3d& netTorque,
                              Eigen::Vector3d& linearAccel,
                              Eigen::Vector3d& angularAccel) const;
    
    /**
     * Integrate kinematics to update pose
     * Uses simple Euler integration: x_new = x_old + v*dt + 0.5*a*dt^2
     */
    static void integratePose(DockingPose& pose,
                              const Eigen::Vector3d& linearAccel,
                              const Eigen::Vector3d& angularAccel,
                              double timestep);
    
    /**
     * Get linear velocity (for external use/debugging)
     */
    Eigen::Vector3d getLinearVelocity() const { return lastLinearVelocity; }
    
    /**
     * Get angular velocity (for external use/debugging)
     */
    Eigen::Vector3d getAngularVelocity() const { return lastAngularVelocity; }
    
    /**
     * Force rigid body constraint control across simulation timeline phases
     */
    void overrideLinearVelocityZ(double vz);

    /**
     * Apply a quick damping response after an invalid docking contact.
     * This reduces the chance of continued penetration or violent bounce.
     */
    void applyConstraintDamping(double linearScale, double angularScale);
    
    /**
     * Print diagnostic information about forces and accelerations
     */
    void printDiagnostics(double time,
                         const ContactEvent& contactEvent,
                         const DockingPose& pose) const;

private:
    PlatformParameters parameters;
    Eigen::Vector3d lastLinearVelocity;
    Eigen::Vector3d lastAngularVelocity;
};
