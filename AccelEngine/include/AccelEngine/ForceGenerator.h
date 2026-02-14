#pragma once
#include <AccelEngine/core.h>
#include <AccelEngine/body.h>
#include <iostream>

namespace AccelEngine
{
    /**
     * Abstract base class for force generators.
     * Each generator applies forces to a RigidBody each frame.
     */
    class ForceGenerator
    {
    public:
        virtual void updateForce(RigidBody *body, real duration) = 0;
        virtual ~ForceGenerator() = default;
    };

    class Gravity : public ForceGenerator
    {
    public:
        Vector2 gravity;

        Gravity(const Vector2 &g) : gravity(g) {}

        virtual void updateForce(RigidBody *body, real duration) override
        {
            if (body->inverseMass <= 0.0f)
                return;


            if (body->ignoreGravity)
                return;


            body->forceAccum += gravity * (1.0f / body->inverseMass);
        }
    };

    class Spring : public ForceGenerator
    {
    public:
        real damping;

        Vector2 localA; // local connection point on body A
        Vector2 localB; // local connection point on body B

        RigidBody *other;

        real k;          // spring constant
        real restLength; // rest distance

        Spring(const Vector2 &localA,
               RigidBody *otherBody,
               const Vector2 &localB,
               real springK,
               real restLength)
            : localA(localA),
              localB(localB),
              other(otherBody),
              k(springK),
              restLength(restLength),
              damping(0.0f)
        {
        }

        virtual void updateForce(RigidBody *bodyA, real duration) override
        {
            RigidBody *bodyB = other;

            // If body has infinite mass no force needed
            bool A_static = (bodyA->inverseMass <= 0.0f);
            bool B_static = (bodyB->inverseMass <= 0.0f);

            // Both static → nothing to do
            if (A_static && B_static)
                return;

            // World points
            Vector2 pA = bodyA->getPointInWorldSpace(localA);
            Vector2 pB = bodyB->getPointInWorldSpace(localB);

            // Spring vector
            Vector2 stretch = pA - pB;
            real len = stretch.magnitude();
            if (len < 1e-6f)
                return;

            Vector2 dir = stretch / len;

            real displacement = len - restLength;
            real Fs = -k * displacement;

            Vector2 relVel = (bodyA->velocity - bodyB->velocity);
            real dampingForce = -damping * relVel.scalarProduct(dir);

            real totalForceScalar = Fs + dampingForce;
            Vector2 F = dir * totalForceScalar;

            if (!A_static)
                bodyA->addForceAtPoint(F, pA);

            if (!B_static)
                bodyB->addForceAtPoint(F * -1, pB);
        }
    };

    class AnchoredSpring : public ForceGenerator
    {
    public:
        Vector2 anchor;     // fixed point in world
        Vector2 localPoint; // local point on body
        real springConstant;
        real restLength;
        real damping;

        AnchoredSpring(const Vector2 &anchorPoint,
                       const Vector2 &localPointOnBody,
                       real k, real rest, real damping)
            : anchor(anchorPoint),
              localPoint(localPointOnBody),
              springConstant(k),
              restLength(rest),
              damping(damping)
        {
        }

        virtual void updateForce(RigidBody *body, real duration) override
        {
            if (body->inverseMass <= 0.0f)
                return;

            // world-space point on body
            Vector2 bodyWS = body->getPointInWorldSpace(localPoint);

            // spring vector
            Vector2 stretch = bodyWS - anchor;
            real length = stretch.magnitude();
            if (length < 1e-6f)
                return;

            Vector2 dir = stretch / length;

            // Hooke's law
            real displacement = length - restLength;
            real Fs = -springConstant * displacement;

            // damping
            Vector2 vel = body->velocity;
            real Fd = -damping * vel.scalarProduct(dir);

            Vector2 totalForce = dir * (Fs + Fd);

            // Apply force at body attachment point
            body->addForceAtPoint(totalForce, bodyWS);
        }
    };

    class Aero : public ForceGenerator
    {
    protected:
        Matrix2 tensor;

        Vector2 position;
        const Vector2 *windSpeed;

    public:
        Aero(const Matrix2 &tensor, const Vector2 &position,
             const Vector2 *windSpeed = nullptr)
            : tensor(tensor), position(position), windSpeed(windSpeed) {}

        virtual void updateForce(RigidBody *body, real duration) override
        {
            if (body->inverseMass <= 0.0f)
                return;

            Vector2 bodyVel = body->velocity;

            Vector2 relativeVel = bodyVel;
            if (windSpeed)
                relativeVel -= *windSpeed;

            Matrix2 inverseRot;
            inverseRot.setInverse(body->transformMatrix);
            Vector2 bodySpaceVel = inverseRot * relativeVel;

            Vector2 bodyForce = tensor * bodySpaceVel;
            bodyForce *= -1;

            Vector2 worldForce = body->transformMatrix * bodyForce;

            Vector2 worldPos = body->getPointInWorldSpace(position);
            body->addForceAtPoint(worldForce, worldPos);
        }
    };

    class AngledAero : public Aero
    {
    protected:
        real orientationOffset;

    public:
        AngledAero(const Matrix2 &tensor, const Vector2 &position,
                   const Vector2 *windSpeed = nullptr)
            : Aero(tensor, position, windSpeed),
              orientationOffset(0.0f) {}

        void setOrientation(real angle)
        {
            orientationOffset = angle;
        }

        virtual void updateForce(RigidBody *body, real duration) override
        {
            if (body->inverseMass <= 0.0f)
                return;

            Vector2 totalVelocity = body->velocity;
            if (windSpeed)
                totalVelocity -= *windSpeed;

            Matrix2 inverseBodyRot;
            inverseBodyRot.setInverse(body->transformMatrix);
            Vector2 bodyVel = inverseBodyRot * totalVelocity;

            Matrix2 surfaceRot;
            surfaceRot.setOrientation(orientationOffset);
            Vector2 localVel = surfaceRot * bodyVel;

            Vector2 bodyForce = tensor * localVel;
            bodyForce *= -1; // Opposes motion

            Vector2 worldForce = body->transformMatrix * (surfaceRot * bodyForce);

            Vector2 worldPos = body->getPointInWorldSpace(position);
            body->addForceAtPoint(worldForce, worldPos);
        }
    };

    class AeroControl : public Aero
    {
    protected:
        Matrix2 maxTensor;
        Matrix2 minTensor;
        real controlSetting;

        Matrix2 getTensor()
        {
            Matrix2 result;

            if (controlSetting <= 0)
            {
                real k = controlSetting + 1.0f;
                for (int i = 0; i < 4; ++i)
                    result.data[i] = minTensor.data[i] * (1.0f - k) + tensor.data[i] * k;
            }
            else
            {
                real k = controlSetting;
                for (int i = 0; i < 4; ++i)
                    result.data[i] = tensor.data[i] * (1.0f - k) + maxTensor.data[i] * k;
            }

            return result;
        }

    public:
        AeroControl(const Matrix2 &base, const Matrix2 &min, const Matrix2 &max,
                    const Vector2 &pos, const Vector2 *wind = nullptr)
            : Aero(base, pos, wind),
              maxTensor(max),
              minTensor(min),
              controlSetting(0.0f)
        {
        }

        void setControl(real value)
        {
            if (value < -1.0f)
                value = -1.0f;
            if (value > 1.0f)
                value = 1.0f;
            controlSetting = value;
        }

        virtual void updateForce(RigidBody *body, real duration) override
        {
            Matrix2 current = getTensor();
            Aero temp(current, position, windSpeed);
            temp.updateForce(body, duration);
        }
    };

    class Buoyancy : public ForceGenerator
    {
        Vector2 centerOfBuoyancy;
        real maxDepth;
        real volume;
        real waterHeight;
        real liquidDensity;

    public:
        Buoyancy(const Vector2 &cOfB, real maxDepth, real volume,
                 real waterHeight, real liquidDensity = 1000.0f)
            : centerOfBuoyancy(cOfB),
              maxDepth(maxDepth),
              volume(volume),
              waterHeight(waterHeight),
              liquidDensity(liquidDensity)
        {
        }

        virtual void updateForce(RigidBody *body, real duration) override
        {
            Vector2 point = body->getPointInWorldSpace(centerOfBuoyancy);

            real depth = waterHeight - point.y;

            // Not in water
            if (depth <= 0.0f)
                return;

            Vector2 force(0, 0);

            if (depth >= maxDepth)
            {
                // Fully submerged
                force.y = liquidDensity * volume;
            }
            else
            {
                // Partially submerged
                force.y = liquidDensity * volume * (depth / maxDepth);
            }

            body->addForceAtPoint(force, point);
        }
    };

}
