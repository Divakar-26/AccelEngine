#pragma once
#include "demo.h"
#include <AccelEngine/body.h>
#include <AccelEngine/world.h>
#include <AccelEngine/ForceRegistry.h>
#include <AccelEngine/ForceGenerator.h>
#include <AccelEngine/joint.h>
#include <vector>

class BetterCarDemo : public Demo
{
private:
    RigidBody* chassis;
    RigidBody* wheelL;
    RigidBody* wheelR;
    Spring* motorSpringL;
    Spring* motorSpringR;
    
public:
    virtual const char* getName() const{ return "Better Car + Terrain"; }

    void init(
        
        World& world,
        
        std::vector<RigidBody*>& bodies,
        ForceRegistry& registry,
        ForceGenerator* gravity
    ) override
    {

        createTerrain(world, bodies);

        chassis = new RigidBody();
        chassis->shapeType = ShapeType::AABB;
        chassis->aabb.halfSize = {60, 15};  // Lower, wider chassis
        chassis->position = {200, 350};
        chassis->inverseMass = 1.0f;
        chassis->restitution = 0.05f;       // Less bouncy
        chassis->dynamicFriction = 0.3f;
        chassis->c = {200, 40, 40, 255};
        chassis->calculateInertia();
        chassis->calculateDerivativeData();

        world.addBody(chassis);
        bodies.push_back(chassis);
        registry.add(chassis, gravity);

        // =====================================================
        // WHEELS (SMALLER, MORE REALISTIC)
        // =====================================================
        float wheelRadius = 18.0f;
        float wheelMass = 0.8f;

        wheelL = new RigidBody();
        wheelL->shapeType = ShapeType::CIRCLE;
        wheelL->circle.radius = wheelRadius;
        wheelL->position = {200 - 45, 350 + 25};  // Position relative to chassis
        wheelL->inverseMass = 1.0f / wheelMass;
        wheelL->restitution = 0.1f;
        wheelL->dynamicFriction = 0.9f;  // High friction for tires
        wheelL->c = {30, 30, 30, 255};
        wheelL->calculateInertia();
        wheelL->calculateDerivativeData();

        wheelR = new RigidBody();
        wheelR->shapeType = ShapeType::CIRCLE;
        wheelR->circle.radius = wheelRadius;
        wheelR->position = {200 + 45, 350 + 25};
        wheelR->inverseMass = 1.0f / wheelMass;
        wheelR->restitution = 0.1f;
        wheelR->dynamicFriction= 0.9f;
        wheelR->c = {30, 30, 30, 255};
        wheelR->calculateInertia();
        wheelR->calculateDerivativeData();

        world.addBody(wheelL);
        world.addBody(wheelR);
        bodies.push_back(wheelL);
        bodies.push_back(wheelR);
        registry.add(wheelL, gravity);
        registry.add(wheelR, gravity);

        // =====================================================
        // IMPROVED SUSPENSION SPRINGS
        // =====================================================
        float restLength = 35.0f;
        float springK = 8000.0f;    // Stiffer springs
        float springD = 100.0f;     // Better damping

        Spring* susL = new Spring(
            Vector2(-45, -10), wheelL, Vector2(0, 0),
            springK, restLength
        );
        susL->damping = springD;

        Spring* susR = new Spring(
            Vector2(45, -10), wheelR, Vector2(0, 0),
            springK, restLength
        );
        susR->damping = springD;

        registry.add(chassis, susL);
        registry.add(chassis, susR);

        // =====================================================
        // RIGID AXLE CONNECTIONS (STIFFER)
        // =====================================================
        float axleK = 2000.0f;
        float axleD = 150.0f;

        Spring* axleL = new Spring(
            Vector2(-45, 0), wheelL, Vector2(0, 0),
            axleK, 0.0f  // Zero rest length for rigid connection
        );
        axleL->damping = axleD;

        Spring* axleR = new Spring(
            Vector2(45, 0), wheelR, Vector2(0, 0),
            axleK, 0.0f
        );
        axleR->damping = axleD;

        registry.add(chassis, axleL);
        registry.add(chassis, axleR);

        // =====================================================
        // WHEEL MOTOR SYSTEM (IMPROVED)
        // =====================================================
        float motorStrength = 8.0f;
        float motorDamping = 5.0f;
        
        // Motor springs that apply rotational force
        motorSpringL = new Spring(
            Vector2(-45, 5), wheelL, Vector2(0, 5),
            motorStrength, 10.0f
        );
        motorSpringL->damping = motorDamping;

        motorSpringR = new Spring(
            Vector2(45, 5), wheelR, Vector2(0, 5),
            motorStrength, 10.0f
        );
        motorSpringR->damping = motorDamping;

        registry.add(chassis, motorSpringL);
        registry.add(chassis, motorSpringR);

        // Initial wheel rotation for visual effect
        wheelL->rotation = 0.0f;
        wheelR->rotation = 0.0f;
    }

    void createTerrain(World& world, std::vector<RigidBody*>& bodies)
    {
        // Main ground
        {
            RigidBody* ground = new RigidBody();
            ground->shapeType = ShapeType::AABB;
            ground->aabb.halfSize = {800, 30};
            ground->position = {600, 100};
            ground->inverseMass = 0.0f;
            ground->inverseInertia = 0.0f;
            ground->orientation = 0;
            ground->c = {100, 100, 100, 255};
            ground->calculateDerivativeData();
            world.addBody(ground);
            bodies.push_back(ground);
        }

        // Starting ramp
        {
            RigidBody* ramp = new RigidBody();
            ramp->shapeType = ShapeType::AABB;
            ramp->aabb.halfSize = {150, 15};
            ramp->position = {300, 130};
            ramp->inverseMass = 0.0f;
            ramp->inverseInertia = 0.0f;
            ramp->orientation = 0.3f;
            ramp->c = {120, 110, 100, 255};
            ramp->calculateDerivativeData();
            world.addBody(ramp);
            bodies.push_back(ramp);
        }

        // Hill
        {
            RigidBody* hill = new RigidBody();
            hill->shapeType = ShapeType::AABB;
            hill->aabb.halfSize = {200, 12};
            hill->position = {800, 180};
            hill->inverseMass = 0.0f;
            hill->inverseInertia = 0.0f;
            hill->orientation = 0.4f;
            hill->c = {110, 120, 110, 255};
            hill->calculateDerivativeData();
            world.addBody(hill);
            bodies.push_back(hill);
        }

        // Down slope
        {
            RigidBody* slope = new RigidBody();
            slope->shapeType = ShapeType::AABB;
            slope->aabb.halfSize = {180, 10};
            slope->position = {1100, 220};
            slope->inverseMass = 0.0f;
            slope->inverseInertia = 0.0f;
            slope->orientation = -0.35f;
            slope->c = {100, 110, 120, 255};
            slope->calculateDerivativeData();
            world.addBody(slope);
            bodies.push_back(slope);
        }

        // Small bump
        {
            RigidBody* bump = new RigidBody();
            bump->shapeType = ShapeType::AABB;
            bump->aabb.halfSize = {30, 8};
            bump->position = {500, 125};
            bump->inverseMass = 0.0f;
            bump->inverseInertia = 0.0f;
            bump->orientation = 0;
            bump->c = {90, 90, 90, 255};
            bump->calculateDerivativeData();
            world.addBody(bump);
            bodies.push_back(bump);
        }
    }

    // Optional: Add method to control the car
    void accelerate(float strength)
    {
        if (motorSpringL && motorSpringR) {
            // Adjust motor springs to apply force
            motorSpringL->restLength = 10.0f + strength * 2.0f;
            motorSpringR->restLength = 10.0f + strength * 2.0f;
        }
    }

    void brake(float strength)
    {
        if (wheelL && wheelR) {
            // Apply negative rotation for braking
            wheelL->rotation -= strength * 0.1f;
            wheelR->rotation -= strength * 0.1f;
        }
    }
};