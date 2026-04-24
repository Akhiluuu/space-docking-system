#include "DockingPhysicsEngine.h"
#include <iostream>
#include <iomanip>
#include <cmath>

DockingPhysicsEngine::DockingPhysicsEngine(const PlatformParameters& params)
    : parameters(params),
      lastLinearVelocity(Eigen::Vector3d::Zero()),
      lastAngularVelocity(Eigen::Vector3d::Zero()) {}

void DockingPhysicsEngine::updatePoseFromForces(const ContactEvent& contactEvent,
                                                 DockingPose& pose,
                                                 double timestep) {
    // Compute net force and torque from the XML contact sensors
    Eigen::Vector3d netForce = computeNetForce(contactEvent);
    Eigen::Vector3d netTorque = computeNetTorque(contactEvent);
    
    // Compute accelerations (a = F/m, alpha = I^-1 * tau)
    Eigen::Vector3d linearAccel;
    Eigen::Vector3d angularAccel;
    computeAccelerations(netForce, netTorque, linearAccel, angularAccel);
    
    // --- UPDATED ZERO-G TRANSLATION ---
    // Instead of locking X/Y, we allow full 6-DOF physics movement
    // Velocity update: V_new = V_old + a * dt
    lastLinearVelocity += linearAccel * timestep;
    lastAngularVelocity += angularAccel * timestep;
    
    // Position Update (Verlet Integration approximation)
    Eigen::Vector3d deltaX = lastLinearVelocity * timestep;
    pose.x += deltaX.x();
    pose.y += deltaX.y();
    pose.z += deltaX.z();
    
    // --- FULL ROTATIONAL PHYSICS ---
    Eigen::Vector3d dTheta = lastAngularVelocity * timestep;
    pose.roll += dTheta.x();
    pose.pitch += dTheta.y();
    pose.yaw += dTheta.z(); 
}

void DockingPhysicsEngine::updatePoseByCoasting(DockingPose& pose, double timestep) {
    // Zero-G drifting requires very low damping.
    // DAMPING = 1.0 would be pure vacuum. 
    // We use 0.999 to allow long-term drift but prevent numerical explosion.
    const double DAMPING = 0.999; 
    lastLinearVelocity *= DAMPING;
    lastAngularVelocity *= DAMPING;
    
    // Apply drift velocity to pose
    Eigen::Vector3d deltaX = lastLinearVelocity * timestep;
    pose.x += deltaX.x();
    pose.y += deltaX.y();
    pose.z += deltaX.z();
    
    Eigen::Vector3d dTheta = lastAngularVelocity * timestep;
    pose.roll += dTheta.x();
    pose.pitch += dTheta.y();
    pose.yaw += dTheta.z();
}

void DockingPhysicsEngine::applySoftDockingStabilization(DockingPose& pose, double timestep) {
    // Stronger damping plus a small centering term keeps soft docking smooth
    // and prevents visible oscillation after first contact.
    const Eigen::Vector3d linearDamping(0.985, 0.985, 0.975);
    const Eigen::Vector3d angularDamping(0.94, 0.94, 0.94);
    const double translationalCentering = 1.4;
    const double verticalCentering = 0.7;
    const double rotationalCentering = 1.8;

    lastLinearVelocity = lastLinearVelocity.cwiseProduct(linearDamping);
    lastAngularVelocity = lastAngularVelocity.cwiseProduct(angularDamping);

    // Pull the suspended body gently back toward centered alignment.
    lastLinearVelocity.x() -= pose.x * translationalCentering * timestep;
    lastLinearVelocity.y() -= pose.y * translationalCentering * timestep;
    lastLinearVelocity.z() -= pose.z * verticalCentering * timestep;

    lastAngularVelocity.x() -= pose.roll * rotationalCentering * timestep;
    lastAngularVelocity.y() -= pose.pitch * rotationalCentering * timestep;
    lastAngularVelocity.z() -= pose.yaw * rotationalCentering * timestep;

    pose.x += lastLinearVelocity.x() * timestep;
    pose.y += lastLinearVelocity.y() * timestep;
    pose.z += lastLinearVelocity.z() * timestep;

    pose.roll += lastAngularVelocity.x() * timestep;
    pose.pitch += lastAngularVelocity.y() * timestep;
    pose.yaw += lastAngularVelocity.z() * timestep;
}

Eigen::Vector3d DockingPhysicsEngine::computeNetForce(const ContactEvent& contactEvent) {
    return contactEvent.getNetForce();
}

Eigen::Vector3d DockingPhysicsEngine::computeNetTorque(const ContactEvent& contactEvent) {
    Eigen::Vector3d netTorque = Eigen::Vector3d::Zero();
    for (const auto& forceReading : contactEvent.forces) {
        auto it = DockingPose::SENSOR_POSITIONS.find(forceReading.sensorID);
        if (it != DockingPose::SENSOR_POSITIONS.end()) {
            const Eigen::Vector3d& sensorPos = it->second;
            Eigen::Vector3d force = forceReading.getForceVector();
            netTorque += sensorPos.cross(force);
        }
    }
    return netTorque;
}

Eigen::Vector3d DockingPhysicsEngine::estimateContactPoint(
    const Eigen::Vector3d& netForce,
    const Eigen::Vector3d& netTorque) {
    double forceSquared = netForce.squaredNorm();
    if (forceSquared < 1e-10) return Eigen::Vector3d::Zero();
    return netForce.cross(netTorque) / forceSquared;
}

void DockingPhysicsEngine::computeAccelerations(
    const Eigen::Vector3d& netForce,
    const Eigen::Vector3d& netTorque,
    Eigen::Vector3d& linearAccel,
    Eigen::Vector3d& angularAccel) const {
    linearAccel = netForce / parameters.mass;
    Eigen::Matrix3d inertiaInv = parameters.inertia.inverse();
    angularAccel = inertiaInv * netTorque;
}

void DockingPhysicsEngine::integratePose(DockingPose& pose,
                                          const Eigen::Vector3d& linearAccel,
                                          const Eigen::Vector3d& angularAccel,
                                          double timestep) {
    Eigen::Vector3d posChange = 0.5 * linearAccel * timestep * timestep;
    pose.x += posChange(0);
    pose.y += posChange(1);
    pose.z += posChange(2);
    
    Eigen::Vector3d rotChange = 0.5 * angularAccel * timestep * timestep;
    pose.roll += rotChange(0);
    pose.pitch += rotChange(1);
    pose.yaw += rotChange(2);
}

void DockingPhysicsEngine::overrideLinearVelocityZ(double vz) {
    lastLinearVelocity.z() = vz;
}

void DockingPhysicsEngine::applyConstraintDamping(double linearScale, double angularScale) {
    lastLinearVelocity *= linearScale;
    lastAngularVelocity *= angularScale;
}

void DockingPhysicsEngine::printDiagnostics(double time,
                                             const ContactEvent& contactEvent,
                                             const DockingPose& pose) const {
    Eigen::Vector3d netForce = computeNetForce(contactEvent);
    Eigen::Vector3d netTorque = computeNetTorque(contactEvent);
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Time: " << time << " s | Net Force: [" << netForce.norm() << " N] Torque: [" << netTorque.norm() << " Nm]\n";
}
