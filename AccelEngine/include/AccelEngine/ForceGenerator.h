#pragma once
#include <AccelEngine/core.h>
#include <AccelEngine/body.h>

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
                return; // infinite mass → no gravity

            // F = m * g
            body->addForce(gravity * (1.0f / body->inverseMass));
        }
    };

    class Spring : public ForceGenerator
    {
    public:
        // Connection point on this body (local coordinates)
        Vector2 connectionPoint;

        // Connection point on the other body (local coordinates)
        Vector2 otherConnectionPoint;

        // Pointer to the other body
        RigidBody *other;

        // Spring properties
        real springConstant;
        real restLength;

        Spring(const Vector2 &localConnectionPoint,
               RigidBody *otherBody,
               const Vector2 &otherConnectionPoint,
               real springConstant,
               real restLength)
            : connectionPoint(localConnectionPoint),
              otherConnectionPoint(otherConnectionPoint),
              other(otherBody),
              springConstant(springConstant),
              restLength(restLength)
        {
        }

        virtual void updateForce(RigidBody *body, real duration) override
        {
            // Convert both connection points to world space
            Vector2 lws = body->getPointInWorldSpace(connectionPoint);
            Vector2 ows = other->getPointInWorldSpace(otherConnectionPoint);

            // Calculate the vector of the spring
            Vector2 force = lws - ows;

            // Calculate the magnitude
            real magnitude = force.magnitude();
            magnitude = std::abs(magnitude - restLength);
            magnitude *= springConstant;

            // Normalize and invert
            force.normalize();
            force *= -magnitude;

            // Apply the force at the local connection point
            body->addForceAtPoint(force, lws);
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
            real depth = point.y - waterHeight;
            if (depth <= 0.0f)
                return; // above water

            Vector2 force(0, 0);
            if (depth >= maxDepth)
                force.y = -liquidDensity * volume; // full upward
            else
                force.y = -liquidDensity * volume * (depth / maxDepth); // partial upward

            body->addForceAtPoint(force, point);
        }
    };

}
