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

            // F = m * g
            body->addForce(gravity * (1.0f / body->inverseMass));
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

            // Hooke's law
            real displacement = len - restLength;
            real Fs = -k * displacement;

            // Relative velocity (in direction of spring)
            Vector2 relVel = (bodyA->velocity - bodyB->velocity);
            real dampingForce = -damping * relVel.scalarProduct(dir);

            real totalForceScalar = Fs + dampingForce;
            Vector2 F = dir * totalForceScalar;

            // ---------- Apply force to both bodies ----------

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
        /**
         * Aerodynamic tensor for the surface in body space.
         * Maps air velocity to force.
         */
        Matrix2 tensor;

        /**
         * Relative position of the aerodynamic surface in body coordinates.
         */
        Vector2 position;

        /**
         * Pointer to the wind speed vector (shared environment).
         */
        const Vector2 *windSpeed;

    public:
        /**
         * Creates a new aerodynamic force generator.
         */
        Aero(const Matrix2 &tensor, const Vector2 &position,
             const Vector2 *windSpeed = nullptr)
            : tensor(tensor), position(position), windSpeed(windSpeed) {}

        /**
         * Applies aerodynamic force to the given rigid body.
         */
        virtual void updateForce(RigidBody *body, real duration) override
        {
            if (body->inverseMass <= 0.0f)
                return;

            // Calculate total velocity of the body surface in world space.
            Vector2 bodyVel = body->velocity;

            // Add wind effect (if present)
            Vector2 relativeVel = bodyVel;
            if (windSpeed)
                relativeVel -= *windSpeed;

            // Convert to body space
            Matrix2 inverseRot;
            inverseRot.setInverse(body->transformMatrix);
            Vector2 bodySpaceVel = inverseRot * relativeVel;

            // Calculate aerodynamic force in body space
            Vector2 bodyForce = tensor * bodySpaceVel;
            bodyForce *= -1; // Opposes motion

            // Convert force to world space
            Vector2 worldForce = body->transformMatrix * bodyForce;

            // Apply force at the aerodynamic surface point
            Vector2 worldPos = body->getPointInWorldSpace(position);
            body->addForceAtPoint(worldForce, worldPos);
        }
    };

    class AngledAero : public Aero
    {
    protected:
        /**
         * Orientation of the aerodynamic surface relative to the rigid body (radians).
         */
        real orientationOffset;

    public:
        /**
         * Creates a new angled aerodynamic surface with the given properties.
         */
        AngledAero(const Matrix2 &tensor, const Vector2 &position,
                   const Vector2 *windSpeed = nullptr)
            : Aero(tensor, position, windSpeed),
              orientationOffset(0.0f) {}

        /**
         * Sets the relative orientation of the aerodynamic surface.
         * @param angle Angle offset in radians relative to the body.
         */
        void setOrientation(real angle)
        {
            orientationOffset = angle;
        }

        /**
         * Applies aerodynamic force to the given rigid body,
         * accounting for the surface’s relative orientation.
         */
        virtual void updateForce(RigidBody *body, real duration) override
        {
            if (body->inverseMass <= 0.0f)
                return;

            // Compute the total air velocity (body velocity + wind)
            Vector2 totalVelocity = body->velocity;
            if (windSpeed)
                totalVelocity -= *windSpeed;

            // Transform to body space
            Matrix2 inverseBodyRot;
            inverseBodyRot.setInverse(body->transformMatrix);
            Vector2 bodyVel = inverseBodyRot * totalVelocity;

            // Apply the local surface rotation offset
            Matrix2 surfaceRot;
            surfaceRot.setOrientation(orientationOffset);
            Vector2 localVel = surfaceRot * bodyVel;

            // Apply aerodynamic tensor
            Vector2 bodyForce = tensor * localVel;
            bodyForce *= -1; // Opposes motion

            // Transform back to world space
            Vector2 worldForce = body->transformMatrix * (surfaceRot * bodyForce);

            // Apply at surface position
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

        // Linear interpolation of matrices
        Matrix2 getTensor()
        {
            Matrix2 result;

            if (controlSetting <= 0)
            {
                // Blend between minTensor (-1) and base tensor (0)
                real k = controlSetting + 1.0f; // maps [-1,0] → [0,1]
                for (int i = 0; i < 4; ++i)
                    result.data[i] = minTensor.data[i] * (1.0f - k) + tensor.data[i] * k;
            }
            else
            {
                // Blend between base tensor (0) and maxTensor (+1)
                real k = controlSetting; // [0,1]
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
        Vector2 centerOfBuoyancy; // local-space center
        real maxDepth;            // depth for full buoyancy
        real volume;              // displaced volume
        real waterHeight;         // y-coordinate of water surface
        real liquidDensity;       // kg/m³ (≈1000 for water)

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
            // Find world position of the center of buoyancy
            Vector2 point = body->getPointInWorldSpace(centerOfBuoyancy);

            // Depth relative to water surface (positive = below surface)
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
