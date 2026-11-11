#pragma once
#include <algorithm>
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

        void rebuildSprings()
        {
            springs.clear();
            for (const Registration &r : registrations)
                if (Spring *sp = dynamic_cast<Spring *>(r.fg))
                    springs.push_back(sp);
        }

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

        // Remove every registration of `body` (call before destroying a body).
        void remove(RigidBody *body)
        {
            registrations.erase(std::remove_if(registrations.begin(), registrations.end(),
                                               [body](const Registration &r) { return r.body == body; }),
                                registrations.end());
            rebuildSprings();
        }

        // Remove one body/generator registration.
        void remove(RigidBody *body, ForceGenerator *fg)
        {
            registrations.erase(std::remove_if(registrations.begin(), registrations.end(),
                                               [&](const Registration &r) { return r.body == body && r.fg == fg; }),
                                registrations.end());
            rebuildSprings();
        }

        // Remove all generators
        void clear()
        {
            registrations.clear();
            springs.clear(); // used to be left behind, dangling after the generators were freed
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
