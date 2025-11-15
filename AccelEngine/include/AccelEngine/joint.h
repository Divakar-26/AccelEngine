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

        virtual void preSolve(float dt) = 0; 
        virtual void solve(float dt) = 0;    
        virtual ~Joint() {}
    };


    class DistanceJoint : public Joint
    {
    public:
        Vector2 localA{0, 0};
        Vector2 localB{0, 0};

        float restLength{0.0f};

        float compliance{0.0f};

        bool useLimits{false};
        float minLength{0.0f};
        float maxLength{0.0f};

        Vector2 worldA{0, 0};
        Vector2 worldB{0, 0};
        Vector2 rA{0, 0};    
        Vector2 rB{0, 0};    
        Vector2 n{0, 0};     
        float effMass{0.0f}; 
        float bias{0.0f};    
        float gamma{0.0f};  

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
            worldA = A->getPointInWorldSpace(localA);
            worldB = B->getPointInWorldSpace(localB);
            rA = worldA - A->position;
            rB = worldB - B->position;

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

            float targetLen = restLength;
            if (useLimits)
            {
                targetLen = std::min(std::max(len, minLength), maxLength);
            }

            float C = len - targetLen;

            float mA = A->inverseMass;
            float mB = B->inverseMass;
            float iA = A->inverseInertia;
            float iB = B->inverseInertia;

            float rAperpDotN = rA.cross(n);
            float rBperpDotN = rB.cross(n);

            float K = mA + mB + (rAperpDotN * rAperpDotN) * iA + (rBperpDotN * rBperpDotN) * iB;

            gamma = 0.0f;
            if (compliance > 0.0f)
            {
                gamma = compliance / (dt * dt);
                K += gamma;
            }

            effMass = (K > 0.0f) ? (1.0f / K) : 0.0f;

           
            const float beta = 0.2f;
            bias = -(beta / dt) * C;

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

    class GearJoint : public Joint
    {
    public:
        RigidBody *A;
        RigidBody *B;
        float ratio; 

        GearJoint(RigidBody *a, RigidBody *b)
        {
            A = a;
            B = b;
            ratio = a->circle.radius / b->circle.radius;
        }

        void preSolve(float dt) override
        {
            float error = A->rotation + B->rotation * ratio;

            float k = A->inverseInertia + B->inverseInertia * ratio * ratio;

            if (k <= 0)
                return;

            float lambda = -error / k;

            A->rotation += lambda * A->inverseInertia;
            B->rotation -= lambda * ratio * B->inverseInertia;
        }

        void solve(float dt) override {}
    };

}
