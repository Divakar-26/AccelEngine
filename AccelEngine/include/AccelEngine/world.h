#pragma once
#include <AccelEngine/core.h>
#include <AccelEngine/body.h>
#include <AccelEngine/collision_coarse.h>
#include <AccelEngine/collision_narrow.h>
#include <AccelEngine/collision_resolve.h>

namespace AccelEngine
{

    class World
    {
    protected:
        std::vector<RigidBody *> bodies;
        std::vector<std::pair<RigidBody *, RigidBody *>> potentialPairs;
        std::vector<Contact> contacts;
        std::vector<Contact> contactsThisFrame;

    public:
        World() {}

        ~World()
        {
            clear();
        }

        void addBody(RigidBody *body)
        {
            bodies.push_back(body);
        }

        /**
         * Removes all bodies.
         */
        void clear()
        {
            bodies.clear();
        }

        const std::vector<Contact> getContacts() const
        {
            return contactsThisFrame;
        }

        /**
         * Initializes the world for a new simulation frame.
         * Clears accumulators and updates derived data.
         */
        void startFrame()
        {

            for (RigidBody *r : bodies)
            {
                r->clearAccumulators();
                r->calculateDerivativeData();
            }
        }

        void runPhysics(real duration)
        {
            for (RigidBody *r : bodies)
            {
                r->integrate(duration);
            }
        }

        void step(float dt, int substeps)
        {
            float subdt = dt / substeps;

            for (int i = 0; i < substeps; i++)
            {
                for (auto *b : bodies)
                    b->integrate(subdt);

                potentialPairs.clear();
                contacts.clear();

                CoarseCollision::FindPotentialPairs(bodies, potentialPairs);
                NarrowCollision::FindContacts(potentialPairs, contacts);

                for (auto &c : contacts)
                    CollisionResolve::Solve(c, subdt);
                
                }
            contactsThisFrame = contacts;
        }
    };
}
