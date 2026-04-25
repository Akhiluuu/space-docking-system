#pragma once

#include <btBulletDynamicsCommon.h>
#include <vector>
#include <string>
#include <memory>
#include <glm/glm.hpp>

struct SimulationConfig {
    float gravity = -9.81f;
    float timeStep = 1.0f / 240.0f;
    int maxSubSteps = 10;
};

class BulletSimulation {
public:
    BulletSimulation(const SimulationConfig& config = SimulationConfig());
    ~BulletSimulation();

    void step();
    
    // Model Loading & assembly
    void loadBase(const std::string& path, float mass = 0.0f);
    void loadTopPlatform(const std::string& path, float mass = 5.0f);
    void addLeg(int index, const std::string& path, float mass = 1.0f);

    // Kinematics (IK Targets)
    void setTargetPose(glm::vec3 pos, glm::vec3 euler_angles);
    
    // Feedback (FK)
    glm::mat4 getPlatformPose();
    std::vector<float> getLegForces();
    std::vector<float> getJointTorques();

private:
    std::unique_ptr<btDefaultCollisionConfiguration> collisionConfig;
    std::unique_ptr<btCollisionDispatcher> dispatcher;
    std::unique_ptr<btBroadphaseInterface> overlappingPairCache;
    std::unique_ptr<btSequentialImpulseConstraintSolver> solver;
    std::unique_ptr<btDiscreteDynamicsWorld> dynamicsWorld;

    btRigidBody* baseBody = nullptr;
    btRigidBody* topPlatformBody = nullptr;
    std::vector<btRigidBody*> legBodies;
    
    SimulationConfig config;
};
