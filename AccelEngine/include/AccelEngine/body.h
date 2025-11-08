#pragma once

#include <AccelEngine/core.h>

struct Color
{
    float r, g, b, a;
};

namespace AccelEngine
{
    enum class ShapeType
    {
        CIRCLE,
        AABB
    };

    struct Circle
    {
        real radius;
    };

    struct AABB
    {
        Vector2 halfSize;
    };

    class RigidBody
    {
    public:
        Color c;

        // ---- Transform ----
        Vector2 position;
        real orientation;
        Matrix2 transformMatrix;

        real restitution;
        real inverseMass;
        real inverseInertia;

        // ---- Velocities ----
        Vector2 velocity;
        real rotation;

        // ---- Acumulating Force ----
        Vector2 forceAccum;
        real torqueAccum;

        // ---- Damping ----
        real linearDamping;
        real angularDamping;

        // ---- Shape ----
        ShapeType shapeType;
        Circle circle;
        AABB aabb;

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
            updateTransformMatrix(transformMatrix, position, orientation);
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
        }

        Vector2 getPosition() const
        {
            return position;
        }

        real getInverseMass()
        {
            return inverseMass;
        }

        Vector2 getVelocity() const
        {
            return velocity;
        }

        static void getTransformedVertices(const RigidBody *body, Vector2 outVertices[4])
        {
            if (body->shapeType != ShapeType::AABB)
                return;

            const Vector2 &half = body->aabb.halfSize;

            Vector2 localCorners[4] = {
                Vector2(-half.x, -half.y),
                Vector2(half.x, -half.y),
                Vector2(half.x, half.y),
                Vector2(-half.x, half.y)};

            for (int i = 0; i < 4; ++i)
            {
                outVertices[i] = body->transformMatrix * localCorners[i] + body->position;
            }
        }

    private:
        static inline void updateTransformMatrix(Matrix2 &transformMatrix, const Vector2 &position, const real orientation)
        {
            transformMatrix.setOrientation(orientation);
        }
    };
}