#include "BulletSimulation.h"
#include <iostream>

BulletSimulation::BulletSimulation(const SimulationConfig& cfg) : config(cfg) {
    collisionConfig = std::make_unique<btDefaultCollisionConfiguration>();
    dispatcher = std::make_unique<btCollisionDispatcher>(collisionConfig.get());
    overlappingPairCache = std::make_unique<btDbvtBroadphase>();
    solver = std::make_unique<btSequentialImpulseConstraintSolver>();
    
    dynamicsWorld = std::make_unique<btDiscreteDynamicsWorld>(
        dispatcher.get(), overlappingPairCache.get(), solver.get(), collisionConfig.get());
    
    dynamicsWorld->setGravity(btVector3(0, config.gravity, 0));
}

BulletSimulation::~BulletSimulation() {
    // Cleanup rigid bodies
    for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState()) {
            delete body->getMotionState();
        }
        dynamicsWorld->removeCollisionObject(obj);
        delete obj;
    }
}

void BulletSimulation::step() {
    dynamicsWorld->stepSimulation(config.timeStep, config.maxSubSteps);
}

void BulletSimulation::loadBase(const std::string& path, float mass) {
    // Placeholder for actual mesh loading and physics creation
    std::cout << "Bullet: Loading Base from " << path << std::endl;
}

void BulletSimulation::loadTopPlatform(const std::string& path, float mass) {
    std::cout << "Bullet: Loading Platform from " << path << std::endl;
}

void BulletSimulation::setTargetPose(glm::vec3 pos, glm::vec3 euler) {
    // This will implement the Force-based control toward targets
}

glm::mat4 BulletSimulation::getPlatformPose() {
    if (!topPlatformBody) return glm::mat4(1.0f);
    
    btTransform trans;
    topPlatformBody->getMotionState()->getWorldTransform(trans);
    
    btScalar m[16];
    trans.getOpenGLMatrix(m);
    return glm::mat4(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
}
