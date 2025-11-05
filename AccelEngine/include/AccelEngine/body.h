#pragma once

#include <AccelEngine/core.h>

namespace AccelEngine
{
    class RigidBody
    {
    public:
        real inverseMass;
        real inverseInertia;

        Vector2 position;

        // in radians
        real orientation;

        Vector2 velocity;
        real rotation;

        // local -> world
        Matrix2 transformMatrix;

        Vector2 forceAccum;
        real torqueAccum;

        real linearDamping;
        real angularDamping;

        RigidBody() : inverseMass(0.0f),
                      inverseInertia(0.0f),
                      position(0, 0),
                      orientation(0),
                      velocity(0, 0),
                      rotation(0),
                      forceAccum(0, 0),
                      torqueAccum(0),
                      linearDamping(0.99f),
                      angularDamping(0.99f)

        {
            transformMatrix.setIdentity();
        }

        // recalculates derived data from the body's state. call this after manually changing position or orientation
        void calculateDerivativeData()
        {
            _calculateTransformMatrix(transformMatrix, position, orientation);
        }

        void addForce(const Vector2 &force)
        {
            forceAccum += force;
        }

        void addForceAtPoint(const Vector2 &force, const Vector2 &point)
        {
            Vector2 r = point - position;
            forceAccum += force;
            torqueAccum += r.x * force.y - r.y * force.x;
        }

        void addForceAtBodyPoint(const Vector2 &force, const Vector2 &localpoint)
        {
            Vector2 worldPoint = getPointInWorldSpace(localpoint);
            addForceAtPoint(force, worldPoint);
        }

        Vector2 getPointInWorldSpace(const Vector2 &localPoint) const
        {
            // Rotate local point
            Vector2 world = transformMatrix * localPoint;

            // Translate by position
            world += position;

            return world;
        }

        void clearAccumulators()
        {
            forceAccum.clear();
            torqueAccum = 0;
        }

        void integrate(real duration)
        {
            if (inverseMass <= 0.0f)
                return;

            // Linear acceleration
            Vector2 acceleration = forceAccum * inverseMass;

            // Angular acceleration
            real angularAcceleration = torqueAccum * inverseInertia;

            // Integrate linear velocity
            velocity += acceleration * duration;

            // Integrate angular velocity
            rotation += angularAcceleration * duration;

            velocity *= std::pow(linearDamping, duration);
            rotation *= std::pow(angularDamping, duration);
            // Integrate position & orientation
            position += velocity * duration;
            orientation += rotation * duration;

            // Recompute transform
            calculateDerivativeData();

            // Clear accumulators
            clearAccumulators();
        }

    private:
        static inline void _calculateTransformMatrix(Matrix2 &transformMatrix, const Vector2 &position, const real orientation)
        {
            transformMatrix.setOrientation(orientation);
        }
    };
}