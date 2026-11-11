#pragma once
#include <AccelEngine/body.h>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace AccelEngine
{
    class Joint
    {
    public:
        RigidBody *A{nullptr};
        RigidBody *B{nullptr};

        // false: the two bodies do not collide with each other (what a hinge usually wants). The older joints
        // keep the original behaviour (true).
        bool collideConnected{true};

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
        // uses Joint::A / Joint::B (World reads them for sleeping islands)
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


    // ------------------------------------------------------------------------------------------------------
    // Joints below work on velocities like DistanceJoint: preSolve() once per substep, solve() many times, with
    // a Baumgarte-style bias that pulls drift back (position error is corrected through velocity).
    // Both bodies must be non-null; use a static body to pin something to the world. Anchors/axes are given in
    // world space at creation and stored in each body's local frame.
    // ------------------------------------------------------------------------------------------------------
    namespace detail
    {
        constexpr real kJointBeta = 0.2f;

        inline real wrapAngle(real a)
        {
            const real twoPi = 6.28318530718f;
            a = std::fmod(a, twoPi);
            if (a > 3.14159265359f) a -= twoPi;
            else if (a < -3.14159265359f) a += twoPi;
            return a;
        }

        inline Vector2 toLocal(const RigidBody *b, const Vector2 &world)
        {
            return b->transformMatrix.transformTranspose(world - b->position);
        }

        // Keeps two anchor points together (2 constraint rows, solved as a 2x2 block).
        struct PointLock
        {
            Vector2 localA, localB;
            Vector2 rA, rB, bias;
            real k11 = 0, k12 = 0, k22 = 0;

            void prepare(const RigidBody *A, const RigidBody *B, real dt)
            {
                rA = A->transformMatrix * localA;
                rB = B->transformMatrix * localB;
                const real mA = A->inverseMass, mB = B->inverseMass, iA = A->inverseInertia, iB = B->inverseInertia;
                k11 = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
                k12 = -rA.y * rA.x * iA - rB.y * rB.x * iB;
                k22 = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
                const Vector2 C = (B->position + rB) - (A->position + rA);
                bias = C * (kJointBeta / dt);
            }

            void solve(RigidBody *A, RigidBody *B) const
            {
                const real mA = A->inverseMass, mB = B->inverseMass, iA = A->inverseInertia, iB = B->inverseInertia;
                const Vector2 vA = A->velocity + Vector2(-A->rotation * rA.y, A->rotation * rA.x);
                const Vector2 vB = B->velocity + Vector2(-B->rotation * rB.y, B->rotation * rB.x);
                const Vector2 rhs = (vB - vA) + bias;
                const real det = k11 * k22 - k12 * k12;
                if (std::fabs(det) < 1e-12f)
                    return;
                const real id = 1.0f / det;
                const Vector2 P(-id * (k22 * rhs.x - k12 * rhs.y), -id * (k11 * rhs.y - k12 * rhs.x));
                A->velocity -= P * mA;
                A->rotation -= iA * rA.cross(P);
                B->velocity += P * mB;
                B->rotation += iB * rB.cross(P);
            }
        };

        // Clamped scalar rows shared by the angular limit and the motor.
        inline void applyAngular(RigidBody *A, RigidBody *B, real lambda)
        {
            A->rotation -= A->inverseInertia * lambda;
            B->rotation += B->inverseInertia * lambda;
        }
    }

    // Pin / hinge: the bodies share a point and rotate freely around it, optionally between angle limits and/or
    // driven by a motor. Angles are the rotation of B relative to A (radians, counter-clockwise), measured from the
    // pose the joint was created in; limits must lie within (-pi, pi).
    class RevoluteJoint : public Joint
    {
    public:
        bool enableLimit{false};
        real lowerAngle{0.0f}, upperAngle{0.0f};
        bool enableMotor{false};
        real motorSpeed{0.0f};     // rad/s, relative angular velocity target (B relative to A)
        real maxMotorTorque{0.0f}; // N*px-ish: the motor never applies more than this torque

        RevoluteJoint(RigidBody *a, RigidBody *b, const Vector2 &worldAnchor)
        {
            A = a;
            B = b;
            collideConnected = false;
            lock.localA = detail::toLocal(a, worldAnchor);
            lock.localB = detail::toLocal(b, worldAnchor);
            referenceAngle = b->orientation - a->orientation;
        }

        void setLimits(real lower, real upper) { enableLimit = true; lowerAngle = lower; upperAngle = upper; }
        void setMotor(real speed, real maxTorque) { enableMotor = true; motorSpeed = speed; maxMotorTorque = maxTorque; }
        void disableMotor() { enableMotor = false; }

        real getAngle() const { return detail::wrapAngle(B->orientation - A->orientation - referenceAngle); }
        real getSpeed() const { return B->rotation - A->rotation; }
        Vector2 getAnchorA() const { return A->getPointInWorldSpace(lock.localA); }
        Vector2 getAnchorB() const { return B->getPointInWorldSpace(lock.localB); }

        void preSolve(float dt) override
        {
            lock.prepare(A, B, dt);
            const real k = A->inverseInertia + B->inverseInertia;
            axialMass = k > 0.0f ? 1.0f / k : 0.0f;
            angle = getAngle();
            motorImpulse = lowerImpulse = upperImpulse = 0.0f;
        }

        void solve(float dt) override
        {
            if (enableMotor && axialMass > 0.0f)
            {
                const real Cdot = B->rotation - A->rotation - motorSpeed;
                real lambda = -axialMass * Cdot;
                const real maxImpulse = dt * maxMotorTorque;
                const real old = motorImpulse;
                motorImpulse = std::clamp(old + lambda, -maxImpulse, maxImpulse);
                detail::applyAngular(A, B, motorImpulse - old);
            }
            if (enableLimit && axialMass > 0.0f)
            {
                {   // lower
                    const real C = angle - lowerAngle;
                    const real bias = C > 0.0f ? C / dt : detail::kJointBeta / dt * C;
                    const real Cdot = B->rotation - A->rotation;
                    const real lambda = -axialMass * (Cdot + bias);
                    const real old = lowerImpulse;
                    lowerImpulse = std::max(old + lambda, 0.0f);
                    detail::applyAngular(A, B, lowerImpulse - old);
                }
                {   // upper (sign flipped)
                    const real C = upperAngle - angle;
                    const real bias = C > 0.0f ? C / dt : detail::kJointBeta / dt * C;
                    const real Cdot = A->rotation - B->rotation;
                    const real lambda = -axialMass * (Cdot + bias);
                    const real old = upperImpulse;
                    upperImpulse = std::max(old + lambda, 0.0f);
                    detail::applyAngular(A, B, -(upperImpulse - old));
                }
            }
            lock.solve(A, B);
        }

    private:
        detail::PointLock lock;
        real referenceAngle{0.0f};
        real axialMass{0.0f};
        real angle{0.0f};
        real motorImpulse{0.0f}, lowerImpulse{0.0f}, upperImpulse{0.0f};
    };

    // Glues two bodies together at their current relative pose.
    class WeldJoint : public Joint
    {
    public:
        WeldJoint(RigidBody *a, RigidBody *b)
        {
            A = a;
            B = b;
            collideConnected = false;
            // anchor at B's centre
            lock.localB = Vector2(0, 0);
            lock.localA = detail::toLocal(a, b->position);
            referenceAngle = b->orientation - a->orientation;
        }

        void preSolve(float dt) override
        {
            lock.prepare(A, B, dt);
            const real k = A->inverseInertia + B->inverseInertia;
            axialMass = k > 0.0f ? 1.0f / k : 0.0f;
            bias = detail::kJointBeta / dt * detail::wrapAngle(B->orientation - A->orientation - referenceAngle);
        }

        void solve(float) override
        {
            if (axialMass > 0.0f)
            {
                const real Cdot = B->rotation - A->rotation;
                detail::applyAngular(A, B, -axialMass * (Cdot + bias));
            }
            lock.solve(A, B);
        }

    private:
        detail::PointLock lock;
        real referenceAngle{0.0f};
        real axialMass{0.0f};
        real bias{0.0f};
    };

    // Wheel on a suspension. B (the wheel) is held on a line through the anchor along `axis` (fixed in A, the
    // chassis), spins freely, and is sprung along the line. Optional motor on the wheel's spin and optional
    // limits on the suspension travel.
    class WheelJoint : public Joint
    {
    public:
        // suspension spring: stiffness as a frequency (0 = no spring), damping ratio (1 = critically damped)
        real springHertz{4.0f};
        real springDamping{0.7f};
        bool enableMotor{false};
        real motorSpeed{0.0f};     // rad/s of the wheel relative to the chassis
        real maxMotorTorque{0.0f};
        bool enableLimit{false};
        real lowerTranslation{0.0f}, upperTranslation{0.0f};

        WheelJoint(RigidBody *a, RigidBody *b, const Vector2 &worldAnchor, const Vector2 &worldAxis)
        {
            A = a;
            B = b;
            collideConnected = false;
            localA = detail::toLocal(a, worldAnchor);
            localB = detail::toLocal(b, worldAnchor);
            localAxis = a->transformMatrix.transformTranspose(worldAxis.normalized());
        }

        void setSpring(real hertz, real dampingRatio) { springHertz = hertz; springDamping = dampingRatio; }
        void setMotor(real speed, real maxTorque) { enableMotor = true; motorSpeed = speed; maxMotorTorque = maxTorque; }
        void disableMotor() { enableMotor = false; }
        void setLimits(real lower, real upper) { enableLimit = true; lowerTranslation = lower; upperTranslation = upper; }

        real getTranslation() const
        {
            const Vector2 d = B->getPointInWorldSpace(localB) - A->getPointInWorldSpace(localA);
            return d.scalarProduct(A->transformMatrix * localAxis);
        }
        real getWheelSpeed() const { return B->rotation - A->rotation; }
        const Vector2 &getLocalAnchorA() const { return localA; }
        const Vector2 &getLocalAnchorB() const { return localB; }

        void preSolve(float dt) override
        {
            const real mA = A->inverseMass, mB = B->inverseMass, iA = A->inverseInertia, iB = B->inverseInertia;
            rA = A->transformMatrix * localA;
            rB = B->transformMatrix * localB;
            const Vector2 d = (B->position + rB) - (A->position + rA);
            axis = A->transformMatrix * localAxis;
            perp = axis.perpendicular();

            // rigid row: stay on the line
            sAy = (d + rA).cross(perp);
            sBy = rB.cross(perp);
            const real kPerp = mA + mB + iA * sAy * sAy + iB * sBy * sBy;
            perpMass = kPerp > 0.0f ? 1.0f / kPerp : 0.0f;
            perpBias = perp.scalarProduct(d) * (detail::kJointBeta / dt);

            // spring row along the axis
            sAx = (d + rA).cross(axis);
            sBx = rB.cross(axis);
            const real kAxis = mA + mB + iA * sAx * sAx + iB * sBx * sBx;
            axisMass = kAxis > 0.0f ? 1.0f / kAxis : 0.0f;
            translation = axis.scalarProduct(d);

            springGamma = springBias = springMass = 0.0f;
            if (springHertz > 0.0f && axisMass > 0.0f)
            {
                const real m = axisMass;
                const real omega = 6.28318530718f * springHertz;
                const real damp = 2.0f * m * springDamping * omega;
                const real k = m * omega * omega;
                springGamma = dt * (damp + dt * k);
                springGamma = springGamma > 0.0f ? 1.0f / springGamma : 0.0f;
                springBias = translation * dt * k * springGamma;
                const real inv = kAxis + springGamma;
                springMass = inv > 0.0f ? 1.0f / inv : 0.0f;
            }

            const real kMotor = iA + iB;
            motorMass = kMotor > 0.0f ? 1.0f / kMotor : 0.0f;
            springImpulse = motorImpulse = lowerImpulse = upperImpulse = 0.0f;
        }

        void solve(float dt) override
        {
            const real mA = A->inverseMass, mB = B->inverseMass, iA = A->inverseInertia, iB = B->inverseInertia;

            if (springMass > 0.0f)
            {
                const real Cdot = axis.scalarProduct(B->velocity - A->velocity) + sBx * B->rotation - sAx * A->rotation;
                const real lambda = -springMass * (Cdot + springBias + springGamma * springImpulse);
                springImpulse += lambda;
                applyLinear(axis, sAx, sBx, lambda, mA, mB, iA, iB);
            }

            if (enableMotor && motorMass > 0.0f)
            {
                const real Cdot = B->rotation - A->rotation - motorSpeed;
                const real lambda = -motorMass * Cdot;
                const real maxImpulse = dt * maxMotorTorque;
                const real old = motorImpulse;
                motorImpulse = std::clamp(old + lambda, -maxImpulse, maxImpulse);
                detail::applyAngular(A, B, motorImpulse - old);
            }

            if (enableLimit && axisMass > 0.0f)
            {
                {   // lower
                    const real C = translation - lowerTranslation;
                    const real bias = C > 0.0f ? C / dt : detail::kJointBeta / dt * C;
                    const real Cdot = axis.scalarProduct(B->velocity - A->velocity) + sBx * B->rotation - sAx * A->rotation;
                    const real lambda = -axisMass * (Cdot + bias);
                    const real old = lowerImpulse;
                    lowerImpulse = std::max(old + lambda, 0.0f);
                    applyLinear(axis, sAx, sBx, lowerImpulse - old, mA, mB, iA, iB);
                }
                {   // upper
                    const real C = upperTranslation - translation;
                    const real bias = C > 0.0f ? C / dt : detail::kJointBeta / dt * C;
                    const real Cdot = -(axis.scalarProduct(B->velocity - A->velocity) + sBx * B->rotation - sAx * A->rotation);
                    const real lambda = -axisMass * (Cdot + bias);
                    const real old = upperImpulse;
                    upperImpulse = std::max(old + lambda, 0.0f);
                    applyLinear(axis, sAx, sBx, -(upperImpulse - old), mA, mB, iA, iB);
                }
            }

            if (perpMass > 0.0f)
            {
                const real Cdot = perp.scalarProduct(B->velocity - A->velocity) + sBy * B->rotation - sAy * A->rotation;
                const real lambda = -perpMass * (Cdot + perpBias);
                applyLinear(perp, sAy, sBy, lambda, mA, mB, iA, iB);
            }
        }

    private:
        void applyLinear(const Vector2 &dir, real sA, real sB, real lambda, real mA, real mB, real iA, real iB)
        {
            const Vector2 P = dir * lambda;
            A->velocity -= P * mA;
            A->rotation -= iA * sA * lambda;
            B->velocity += P * mB;
            B->rotation += iB * sB * lambda;
        }

        Vector2 localA, localB, localAxis;
        Vector2 rA, rB, axis, perp;
        real sAx{0}, sBx{0}, sAy{0}, sBy{0};
        real perpMass{0}, perpBias{0}, axisMass{0};
        real translation{0};
        real springMass{0}, springGamma{0}, springBias{0};
        real motorMass{0};
        real springImpulse{0}, motorImpulse{0}, lowerImpulse{0}, upperImpulse{0};
    };

}
