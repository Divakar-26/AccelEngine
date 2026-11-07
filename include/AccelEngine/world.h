#pragma once
#include <AccelEngine/core.h>
#include <AccelEngine/body.h>
#include <AccelEngine/collision_coarse.h>
#include <AccelEngine/collision_narrow.h>
#include <AccelEngine/collision_resolve.h>
#include <AccelEngine/joint.h>
#include <AccelEngine/BVH.h>
#include <AccelEngine/profiler.h>

namespace AccelEngine
{

    struct CollisionEvent
    {
        RigidBody *a;
        RigidBody *b;
    };

    class World
    {
    protected:
        std::vector<RigidBody *> bodies;
        std::vector<std::pair<RigidBody *, RigidBody *>> potentialPairs;
        std::vector<Contact> contacts;
        std::vector<Contact> contactsThisFrame;

    public:
        std::vector<Joint *> joints;
        std::vector<CollisionEvent> collisionEvents;
        BVHTree broadPhase;

        World() {}

        ~World()
        {
            clear();
        }

        void addBody(RigidBody *body)
        {
            bodies.push_back(body);
        }

        void addJoint(Joint *j)
        {
            joints.push_back(j);
        }

        std::vector<Joint *> &getJoints()
        {
            return joints;
        }

        const std::vector<CollisionEvent> &GetCollisionEvents() const
        {
            return collisionEvents;
        }

        void clear()
        {
            bodies.clear();
        }

        const std::vector<Contact> getContacts() const
        {
            return contactsThisFrame;
        }

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

            broadPhase.build(bodies);
            for (int i = 0; i < substeps; i++)
            {
                for (auto *b : bodies)
                    b->integrate(subdt);

                potentialPairs.clear();
                contacts.clear();

                PROFILE_SCOPE("Collision");
                broadPhase.findPairs(potentialPairs);
                NarrowCollision::FindContacts(potentialPairs, contacts);

                collisionEvents.clear();

                for (auto &c : contacts)
                {
                    collisionEvents.push_back({c.a, c.b});
                }

                for (auto *j : joints)
                    j->preSolve(subdt);

                for (auto &c : contacts)
                {
                    PROFILE_SCOPE("Solve");
                    CollisionResolve::Solve(c, subdt);
                }

                for (int it = 0; it < 100; it++)
                {
                    for (auto *j : joints)
                    {
                        j->solve(subdt);
                    }
                }
            }
            contactsThisFrame = contacts;
        }
    };
}
