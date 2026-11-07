#pragma once
#include <vector>
#include <AccelEngine/ForceGenerator.h>

namespace AccelEngine
{

    class ForceRegistry
    {
    private:
        struct Registration
        {
            RigidBody *body;
            ForceGenerator *fg;
        };

        std::vector<Spring *> springs;

    public:
        // Add a force generator for a specific body
        std::vector<Registration> registrations;
        void add(RigidBody *body, ForceGenerator *fg)
        {
            registrations.push_back({body, fg});

            Spring *s = dynamic_cast<Spring *>(fg);
            if (s)
                springs.push_back(s);
        }

        const std::vector<Spring *> &getSprings() const
        {
            return springs;
        }

        // Remove all generators (optional)
        void clear()
        {
            registrations.clear();
        }

        void updateForces(real dt)
        {
            for (auto &r : registrations)
            {
                r.fg->updateForce(r.body, dt);
            }
        }

    };

}
