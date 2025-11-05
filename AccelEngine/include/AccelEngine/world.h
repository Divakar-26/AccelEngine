#pragma once
#include <AccelEngine/core.h>
#include <AccelEngine/body.h>

namespace AccelEngine
{
    /**
     * The world represents an independent simulation of physics.
     * It manages all rigid bodies and updates them each frame.
     */
    class World
    {
    protected:
        /**
         * Holds a single rigid body in a linked list.
         */
        struct BodyRegistration
        {
            RigidBody *body;
            BodyRegistration *next;
        };

        /**
         * The head of the list of registered bodies.
         */
        BodyRegistration *firstBody;

    public:
        World() : firstBody(nullptr) {}

        ~World()
        {
            clear();
        }

        /**
         * Adds a new rigid body to the world.
         */
        void addBody(RigidBody *body)
        {
            BodyRegistration *registration = new BodyRegistration();
            registration->body = body;
            registration->next = firstBody;
            firstBody = registration;
        }

        /**
         * Removes all bodies.
         */
        void clear()
        {
            BodyRegistration *reg = firstBody;
            while (reg)
            {
                BodyRegistration *next = reg->next;
                delete reg;
                reg = next;
            }
            firstBody = nullptr;
        }

        /**
         * Initializes the world for a new simulation frame.
         * Clears accumulators and updates derived data.
         */
        void startFrame()
        {
            BodyRegistration *reg = firstBody;
            while (reg)
            {
                reg->body->clearAccumulators();
                reg->body->calculateDerivativeData();
                reg = reg->next;
            }
        }

        /**
         * Runs the physics update for all rigid bodies.
         */
        void runPhysics(real duration)
        {
            BodyRegistration *reg = firstBody;
            while (reg)
            {
                reg->body->integrate(duration);
                reg = reg->next;
            }
        }
    };
}
