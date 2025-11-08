#pragma once
#include <AccelEngine/core.h>
#include <AccelEngine/body.h>
#include <AccelEngine/collision_coarse.h>
#include <AccelEngine/collision_narrow.h>
#include <AccelEngine/collision_resolve.h>

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

        BodyRegistration *getFirstBody() { return firstBody; }

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

        std::vector<Contact> getContacts()
        {
            std::vector<RigidBody *> bodies;
            bodies.reserve(64);

            for (auto *reg = firstBody; reg; reg = reg->next)
                bodies.push_back(reg->body);

            std::vector<std::pair<RigidBody *, RigidBody *>> potentialPairs;
            std::vector<Contact> contacts;

            potentialPairs.clear();
            CoarseCollision::FindPotentialPairs(bodies, potentialPairs);

            // narrowphase
            contacts.clear();
            NarrowCollision::FindContacts(potentialPairs, contacts);

            return contacts;
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

        void runPhysics(real duration)
        {
            // Integrate motion
            BodyRegistration *reg = firstBody;
            while (reg)
            {
                reg->body->integrate(duration);
                reg = reg->next;
            }
        }

        void step(float dt, int iteration)
        {
            const real subdt = dt / iteration;

            std::vector<RigidBody *> bodies;
            bodies.reserve(64);

            for (auto *reg = firstBody; reg; reg = reg->next)
                bodies.push_back(reg->body);

            std::vector<std::pair<RigidBody *, RigidBody *>> potentialPairs;
            potentialPairs.reserve(256);

            std::vector<Contact> contacts;
            contacts.reserve(256);

            for (int i = 0; i < iteration; ++i)
            {
                for (auto *b : bodies)
                    b->integrate(subdt);

                // broadphase
                potentialPairs.clear();
                CoarseCollision::FindPotentialPairs(bodies, potentialPairs);

                // narrowphase
                contacts.clear();
                NarrowCollision::FindContacts(potentialPairs, contacts);

                // resolve
                for (auto &c : contacts)
                    CollisionResolve::Solve(c, subdt);
            }
        }
    };
}
