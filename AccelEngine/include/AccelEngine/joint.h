#pragma once
#include <AccelEngine/body.h>
#include <algorithm>
#include <iostream>

namespace AccelEngine
{
    class Joint
    {
    public: 
        RigidBody *A{nullptr};
        RigidBody *B{nullptr};

        virtual void preSolve(float dt) = 0; // cache terms, warm-start
        virtual void solve(float dt) = 0;    // apply corrective impulse(s)
        virtual ~Joint() {}
    };

    /**
     * DistanceJoint keeps the distance between two local anchor points
     * on A and B equal to restLength (like a massless rod).
     *
     * Solver: sequential impulses, velocity form with Baumgarte position bias.
     */
    class DistanceJoint : public Joint
    {
    public:
        // local-space anchors
        Vector2 localA{0, 0};
        Vector2 localB{0, 0};

        // target distance
        float restLength{0.0f};

        // optional softness/compliance (gamma). Set to 0 for fully rigid.
        // Larger compliance -> softer joint.
        float compliance{0.0f};

        // Optional limits (set min<=max; set both to restLength to disable limits).
        bool useLimits{false};
        float minLength{0.0f};
        float maxLength{0.0f};

        // ---- cached for solver (per step) ----
        Vector2 worldA{0, 0};
        Vector2 worldB{0, 0};
        Vector2 rA{0, 0};    // worldA - A->position
        Vector2 rB{0, 0};    // worldB - B->position
        Vector2 n{0, 0};     // unit constraint direction from A→B
        float effMass{0.0f}; // 1 / K
        float bias{0.0f};    // Baumgarte / positional correction term
        float gamma{0.0f};   // softness term for velocity solve

        // warm starting (accumulate scalar impulse along n)
        float accumulatedLambda{0.0f};

        DistanceJoint(RigidBody *a,
                      RigidBody *b,
                      const Vector2 &localA_,
                      const Vector2 &localB_,
                      float restLen = -1.0f)
        {
            A = a;
            B = b;
            localA = localA_;
            localB = localB_;

            Vector2 pA = A->getPointInWorldSpace(localA);
            Vector2 pB = B->getPointInWorldSpace(localB);

            if (restLen > 0.0f)
                restLength = restLen;
            else
                restLength = Vector2::distance(pA, pB);
        }

        void setLimits(float minL, float maxL)
        {
            useLimits = true;
            minLength = minL;
            maxLength = maxL;
        }

        void setCompliance(float c) { compliance = (c < 0.0f ? 0.0f : c); }

        void preSolve(float dt) override
        {
            // world anchors and radii
            worldA = A->getPointInWorldSpace(localA);
            worldB = B->getPointInWorldSpace(localB);
            rA = worldA - A->position;
            rB = worldB - B->position;

            // direction and current length
            Vector2 d = worldB - worldA;
            float len = d.magnitude();
            if (len < 1e-6f)
            {
                n = Vector2(1, 0);
            }
            else
            {
                n = d / len;
            }

            // If limits are enabled, clamp the target distance
            float targetLen = restLength;
            if (useLimits)
            {
                targetLen = std::min(std::max(len, minLength), maxLength);
            }

            // positional error C = currentLen - targetLen
            float C = len - targetLen;

            // Effective mass with angular terms:
            // K = mA + mB + ( (rA⊥·n)^2 )*iA + ( (rB⊥·n)^2 )*iB  (+ gamma if soft)
            float mA = A->inverseMass;
            float mB = B->inverseMass;
            float iA = A->inverseInertia;
            float iB = B->inverseInertia;

            // r⊥·n == cross(r, n)
            float rAperpDotN = rA.cross(n);
            float rBperpDotN = rB.cross(n);

            float K = mA + mB + (rAperpDotN * rAperpDotN) * iA + (rBperpDotN * rBperpDotN) * iB;

            // Softness (XPBD-like): gamma = compliance / dt^2; adds to K and biases the solve
            gamma = 0.0f;
            if (compliance > 0.0f)
            {
                gamma = compliance / (dt * dt);
                K += gamma;
            }

            effMass = (K > 0.0f) ? (1.0f / K) : 0.0f;

            // Baumgarte position bias to reduce drift
            // bias = -beta/dt * C ; beta in [0.1, 0.3] typical
            const float beta = 0.2f;
            bias = -(beta / dt) * C;

            // Warm start: apply previous accumulated impulse to current velocities
            if (accumulatedLambda != 0.0f && effMass > 0.0f)
            {
                Vector2 P = n * accumulatedLambda;

                if (mA > 0.0f)
                {
                    A->velocity -= P * mA;
                    A->rotation -= rA.cross(P) * iA;
                }
                if (mB > 0.0f)
                {
                    B->velocity += P * mB;
                    B->rotation += rB.cross(P) * iB;
                }
            }
        }

        void solve(float dt) override
        {
            Vector2 raPerp(-rA.y, rA.x);
            Vector2 rbPerp(-rB.y, rB.x);

            Vector2 velA = A->velocity + raPerp * A->rotation;
            Vector2 velB = B->velocity + rbPerp * B->rotation;

            Vector2 relVel = velB - velA;
            float Cdot = relVel.scalarProduct(n);

            // correct sign: bias should make bodies pull together
            float lambda = -effMass * (Cdot - bias + gamma * accumulatedLambda);

            if (compliance > 0.0f)
            {
                accumulatedLambda += lambda;
                accumulatedLambda = std::clamp(accumulatedLambda, -1000.0f, 1000.0f);
            }

            Vector2 P = n * lambda;

            if (A->inverseMass > 0.0f)
            {
                A->velocity -= P * A->inverseMass;
                A->rotation -= rA.cross(P) * A->inverseInertia;
            }

            if (B->inverseMass > 0.0f)
            {
                B->velocity += P * B->inverseMass;
                B->rotation += rB.cross(P) * B->inverseInertia;
            }
        }
    };
}
