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

        // ----- Frictions -----
        real staticFriction;
        real dynamicFriction;

        // ---- Velocities ----
        Vector2 velocity;
        real rotation;

        // ---- Acumulating Force ----
        Vector2 forceAccum; // this force is not a single force, in this we add multiple force (vector addition)
        real torqueAccum;

        // ---- Damping ----
        real linearDamping;
        real angularDamping;

        // ---- Shape ----
        ShapeType shapeType;
        Circle circle;
        AABB aabb;

        Vector2 worldAABBMin;
        Vector2 worldAABBMax;

        bool enableCollision;
        real boundingRadius = 0.0f;
        // mimic kinermatic body
        bool lockPosition = false;
        bool lockRotation = false;
        bool ignoreGravity = false;

        RigidBody() : inverseMass(0.0f),
                      inverseInertia(0.0f),
                      position(0, 0),
                      orientation(0),
                      velocity(0, 0),
                      rotation(0),
                      forceAccum(0, 0),
                      torqueAccum(0),
                      linearDamping(1.0f),
                      angularDamping(1.0f),
                      staticFriction(0.6),
                      dynamicFriction(0.4),
                      enableCollision(true),
                      lockPosition(false)

        {
            transformMatrix.setIdentity();
        }

        // recalculates derived data from the body's state. call this after manually changing position or orientation
        void calculateDerivativeData()
        {
            updateTransformMatrix(transformMatrix, position, orientation);
            updateAABB();

            if (shapeType == ShapeType::CIRCLE)
            {
                boundingRadius = circle.radius;
            }
            else
            {
                // tightest sphere that contains the rotated rectangle
                const real hx = aabb.halfSize.x;
                const real hy = aabb.halfSize.y;
                boundingRadius = std::sqrt(hx * hx + hy * hy);
            }
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
            // ============================================
            // 1. POSITION LOCK  (Kinematic translation)
            // ============================================
            if (lockPosition)
            {
                // no linear motion
                velocity = Vector2(0, 0);
                forceAccum.clear();
                position = position;

                if (!lockRotation)
                {
                    real angularAcceleration = torqueAccum * inverseInertia;
                    rotation += angularAcceleration * duration;
                    rotation *= std::pow(angularDamping, duration);
                    orientation += rotation * duration;

                    // wrap to [0, 2*pi]
                    while (orientation >= 6.28318531f)
                        orientation -= 6.28318531f;
                    while (orientation < 0)
                        orientation += 6.28318531f;
                }
                else
                {
                    rotation = 0;
                    torqueAccum = 0;
                }

                calculateDerivativeData();
                return;
            }

            if (inverseMass <= 0.0f)
                return;

            Vector2 acceleration = forceAccum * inverseMass;
            real angularAcceleration = torqueAccum * inverseInertia;

            velocity += acceleration * duration;

            // rotation lock for dynamic bodies
            if (!lockRotation)
                rotation += angularAcceleration * duration;
            else
            {
                rotation = 0;
                torqueAccum = 0;
            }

            velocity *= std::pow(linearDamping, duration);

            if (!lockRotation)
                rotation *= std::pow(angularDamping, duration);

            position += velocity * duration;

            if (!lockRotation)
                orientation += rotation * duration;

            calculateDerivativeData();
        }

        void updateAABB()
        {
            if (shapeType == ShapeType::CIRCLE)
            {
                float r = circle.radius;
                worldAABBMin = {position.x - r, position.y - r};
                worldAABBMax = {position.x + r, position.y + r};
                return;
            }

            // AABB for rotated box
            Vector2 corners[4] = {
                {-aabb.halfSize.x, -aabb.halfSize.y},
                {aabb.halfSize.x, -aabb.halfSize.y},
                {aabb.halfSize.x, aabb.halfSize.y},
                {-aabb.halfSize.x, aabb.halfSize.y}};

            worldAABBMin = {+1e9f, +1e9f};
            worldAABBMax = {-1e9f, -1e9f};

            for (int i = 0; i < 4; i++)
            {
                Vector2 p = transformMatrix * corners[i] + position;

                worldAABBMin.x = std::min(worldAABBMin.x, p.x);
                worldAABBMin.y = std::min(worldAABBMin.y, p.y);
                worldAABBMax.x = std::max(worldAABBMax.x, p.x);
                worldAABBMax.y = std::max(worldAABBMax.y, p.y);
            }
        }

        void calculateInertia()
        {
            float mass = (inverseMass > 0) ? (1.0f / inverseMass) : 0.0f;

            if (shapeType == ShapeType::AABB)
            {
                float w = aabb.halfSize.x * 2.0f;
                float h = aabb.halfSize.y * 2.0f;

                float inertia = (1.0f / 12.0f) * mass * (w * w + h * h);

                if (inertia > 1e-6f)
                    inverseInertia = 1.0f / inertia;
                else
                    inverseInertia = 0.0f;
            }

            else if (shapeType == ShapeType::CIRCLE)
            {
                float r = circle.radius;

                float inertia = 0.5f * mass * r * r;

                if (inertia > 1e-6f)
                    inverseInertia = 1.0f / inertia;
                else
                    inverseInertia = 0.0f;
            }
        }
        // GETTERS
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

        real getWidth() const
        {
            return aabb.halfSize.x * 2;
        }
        real getHeigt() const
        {
            return aabb.halfSize.y * 2;
        }

        inline real getBoundingRadius() const { return boundingRadius; }

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