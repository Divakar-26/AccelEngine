#pragma once

#include <AccelEngine/core.h>
#include <AccelEngine/shape.h>
#include <cstdint>
#include <limits>
#include <vector>

struct Color
{
    float r, g, b, a;
};

namespace AccelEngine
{
    enum class ShapeType
    {
        CIRCLE,   // circle.radius
        AABB,     // oriented box, aabb.halfSize (the name is historical)
        POLYGON,  // one convex polygon in `parts`
        CAPSULE,  // one capsule in `parts`
        COMPOUND  // several convex parts in `parts`
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

        real restitution = 0.0f;
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
        ShapeType shapeType = ShapeType::CIRCLE;
        Circle circle{0.0f};
        AABB aabb;

        // POLYGON / CAPSULE / COMPOUND bodies: the convex parts, in the body's local frame (origin = centre of mass).
        // Build them with World::createPolygon / createCapsule / createCompound, or fill the vector and call
        // calculateInertia() + calculateDerivativeData().
        std::vector<ConvexPart> parts;
        Vector2 localBoundsCenter; // local box around all parts (derived by updatePartsBounds)
        Vector2 localBoundsHalf;
        real partsBoundRadius = 0.0f; // radius around the body origin that contains every part

        Vector2 worldAABBMin;
        Vector2 worldAABBMax;

        bool enableCollision;
        real boundingRadius = 0.0f;
        // mimic kinermatic body
        bool lockPosition = false;
        bool lockRotation = false;
        bool ignoreGravity = false;

        // ---- Collision filtering ----
        // Two bodies collide when (a.maskBits & b.categoryBits) && (b.maskBits & a.categoryBits), unless they share a
        // non-zero groupIndex: a positive group always collides, a negative group never does.
        uint16_t categoryBits = 0x0001;
        uint16_t maskBits = 0xFFFF;
        int16_t groupIndex = 0;

        // Sensors detect overlaps (reported in World::getSensorEvents()) but are not solid: no contact forces,
        // and they never wake a sleeping body.
        bool isSensor = false;

        // One-way platform: solid only for bodies that come from the side `oneWayNormal` points to (a direction in
        // the platform's local frame; default up). Bodies moving through it from behind are ignored.
        bool oneWay = false;
        Vector2 oneWayNormal{0.0f, 1.0f};

        static bool shouldCollide(const RigidBody &a, const RigidBody &b)
        {
            if (a.groupIndex == b.groupIndex && a.groupIndex != 0)
                return a.groupIndex > 0;
            return (a.maskBits & b.categoryBits) != 0 && (b.maskBits & a.categoryBits) != 0;
        }

        uint32_t entityID = 0; // used by engine
        void* userData = nullptr;

        // ---- Sleeping ----
        // A sleeping body is not integrated and is skipped by the solver until something wakes it
        // (a contact or joint with an awake body, a spring pulled by an awake body, or wakeUp()).
        bool isAwake = true;
        bool canSleep = true;   // set false for bodies driven by user code (motors, dragged bodies...)
        real sleepTime = 0.0f;  // seconds spent below the World's sleep thresholds
        int worldIndex = -1;    // scratch index used by World while stepping



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
            // sin/cos only when the orientation actually changed (resting/sleeping/static bodies skip it)
            if (orientation != cachedOrientation)
            {
                updateTransformMatrix(transformMatrix, position, orientation);
                cachedOrientation = orientation;
            }
            updateAABB();

            if (shapeType == ShapeType::CIRCLE)
            {
                boundingRadius = circle.radius;
            }
            else if (usesParts())
            {
                boundingRadius = partsBoundRadius;
            }
            else
            {
                // tightest sphere that contains the rotated rectangle
                const real hx = aabb.halfSize.x;
                const real hy = aabb.halfSize.y;
                boundingRadius = std::sqrt(hx * hx + hy * hy);
            }
        }

        // Wake the body (resets its sleep timer). Call after changing position/velocity by hand.
        void wakeUp()
        {
            setAwake(true);
        }

        void setAwake(bool awake)
        {
            isAwake = awake;
            if (awake)
            {
                sleepTime = 0.0f;
            }
            else
            {
                // keep sleepTime: if something wakes this body without disturbing it (e.g. a spring
                // from a slow neighbour), it can fall straight back asleep instead of ping-ponging
                velocity.clear();
                rotation = 0.0f;
                clearAccumulators();
            }
        }

        // Never moves: infinite mass, or both position and rotation locked.
        bool isStatic() const
        {
            return inverseMass <= 0.0f || (lockPosition && lockRotation);
        }

        // Inverse mass / inertia as seen by the contact solver: locked axes behave as infinite mass.
        real getEffectiveInverseMass() const
        {
            return lockPosition ? 0.0f : inverseMass;
        }

        real getEffectiveInverseInertia() const
        {
            return (lockRotation || inverseMass <= 0.0f) ? 0.0f : inverseInertia;
        }
        // ---- Convenience setters (all wake the body and refresh derived data) ----

        // Instant change of velocity: v += impulse / mass. No effect on static bodies.
        void applyImpulse(const Vector2 &impulse)
        {
            if (isStatic())
                return;
            velocity += impulse * getEffectiveInverseMass();
            wakeUp();
        }

        // Impulse at a world point: also spins the body.
        void applyImpulseAtPoint(const Vector2 &impulse, const Vector2 &worldPoint)
        {
            if (isStatic())
                return;
            const Vector2 r = worldPoint - position;
            velocity += impulse * getEffectiveInverseMass();
            rotation += r.cross(impulse) * getEffectiveInverseInertia();
            wakeUp();
        }

        // Moves the body (and optionally rotates it) without touching its velocity.
        void setPose(const Vector2 &newPosition, real newOrientation)
        {
            position = newPosition;
            orientation = newOrientation;
            calculateDerivativeData();
            wakeUp();
        }

        void setVelocity(const Vector2 &v, real angularVelocity = 0.0f)
        {
            velocity = v;
            rotation = angularVelocity;
            wakeUp();
        }

        // Mass in kg-like units; 0 (or less) makes the body static. Recomputes the inertia.
        void setMass(real mass)
        {
            inverseMass = mass > 0.0f ? 1.0f / mass : 0.0f;
            calculateInertia();
            if (mass <= 0.0f)
                inverseInertia = 0.0f;
            wakeUp();
        }

        real getMass() const { return inverseMass > 0.0f ? 1.0f / inverseMass : 0.0f; }

        // True if position, orientation and velocities are all finite numbers.
        bool isFinite() const
        {
            return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(orientation) &&
                   std::isfinite(velocity.x) && std::isfinite(velocity.y) && std::isfinite(rotation);
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
            if (!isAwake)
                return;

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
                    rotation *= dampingFactor(angularDamping, duration, cachedAngDamping);
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

            velocity *= dampingFactor(linearDamping, duration, cachedLinDamping);

            if (!lockRotation)
                rotation *= dampingFactor(angularDamping, duration, cachedAngDamping);

            position += velocity * duration;

            if (!lockRotation)
                orientation += rotation * duration;

            calculateDerivativeData();
        }

        bool usesParts() const { return shapeType >= ShapeType::POLYGON; }

        // Recomputes partsBoundRadius and the local bounds from `parts`.
        void updatePartsBounds()
        {
            if (parts.empty())
            {
                localBoundsCenter = localBoundsHalf = Vector2(0, 0);
                partsBoundRadius = 0.0f;
                return;
            }
            Vector2 mn = parts[0].boundMin, mx = parts[0].boundMax;
            partsBoundRadius = 0.0f;
            for (const ConvexPart &p : parts)
            {
                mn = {std::min(mn.x, p.boundMin.x), std::min(mn.y, p.boundMin.y)};
                mx = {std::max(mx.x, p.boundMax.x), std::max(mx.y, p.boundMax.y)};
                partsBoundRadius = std::max(partsBoundRadius, p.center.magnitude() + p.boundRadius);
            }
            localBoundsCenter = (mn + mx) * 0.5f;
            localBoundsHalf = (mx - mn) * 0.5f;
        }

        // Part i (or the whole circle/box) as a world-space rounded polygon.
        int worldPartCount() const { return usesParts() ? (int)parts.size() : 1; }

        void getWorldPart(int i, WorldPoly &out) const
        {
            const real *m = transformMatrix.data;
            auto rot = [m](const Vector2 &p) { return Vector2(m[0] * p.x + m[1] * p.y, m[2] * p.x + m[3] * p.y); };
            if (usesParts())
            {
                const ConvexPart &p = parts[i];
                out.count = p.count;
                out.radius = p.radius;
                for (int k = 0; k < p.count; k++)
                {
                    out.v[k] = rot(p.v[k]) + position;
                    out.n[k] = rot(p.n[k]);
                }
                out.center = rot(p.center) + position;
                out.boundRadius = p.boundRadius;
            }
            else if (shapeType == ShapeType::CIRCLE)
            {
                out.count = 1;
                out.v[0] = position;
                out.n[0] = Vector2(1, 0);
                out.radius = circle.radius;
                out.center = position;
                out.boundRadius = circle.radius;
            }
            else
            {
                getTransformedVertices(this, out.v);
                out.count = 4;
                out.radius = 0.0f;
                for (int k = 0; k < 4; k++)
                {
                    const Vector2 e = out.v[(k + 1) % 4] - out.v[k];
                    out.n[k] = Vector2(e.y, -e.x).normalized();
                }
                out.center = position;
                out.boundRadius = boundingRadius;
            }
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

            if (usesParts())
            {
                const real *m = transformMatrix.data;
                const Vector2 c = position + Vector2(m[0] * localBoundsCenter.x + m[1] * localBoundsCenter.y,
                                                     m[2] * localBoundsCenter.x + m[3] * localBoundsCenter.y);
                const real ex = std::fabs(m[0]) * localBoundsHalf.x + std::fabs(m[1]) * localBoundsHalf.y;
                const real ey = std::fabs(m[2]) * localBoundsHalf.x + std::fabs(m[3]) * localBoundsHalf.y;
                worldAABBMin = {c.x - ex, c.y - ey};
                worldAABBMax = {c.x + ex, c.y + ey};
                return;
            }

            // AABB of a rotated box: extents = |R| * halfSize (same result as transforming the 4 corners)
            const real *m = transformMatrix.data;
            const real ex = std::fabs(m[0]) * aabb.halfSize.x + std::fabs(m[1]) * aabb.halfSize.y;
            const real ey = std::fabs(m[2]) * aabb.halfSize.x + std::fabs(m[3]) * aabb.halfSize.y;
            worldAABBMin = {position.x - ex, position.y - ey};
            worldAABBMax = {position.x + ex, position.y + ey};
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

            else if (usesParts())
            {
                updatePartsBounds();
                real totalArea = 0.0f, unitInertia = 0.0f;
                for (const ConvexPart &p : parts)
                {
                    real a, i;
                    Vector2 c;
                    p.massProperties(a, c, i);
                    totalArea += a;
                    unitInertia += i;
                }
                const real inertia = totalArea > 1e-9f ? unitInertia * (mass / totalArea) : 0.0f;
                inverseInertia = inertia > 1e-6f ? 1.0f / inertia : 0.0f;
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
        // ---- caches (implementation detail, never read these) ----
        real cachedOrientation = 0.0f; // orientation transformMatrix was built for (identity at construction)
        struct DampingCache
        {
            real damping = -1.0f, duration = -1.0f, factor = 1.0f;
        };
        DampingCache cachedLinDamping, cachedAngDamping;

        // pow(damping, duration), recomputed only when either input changes
        static real dampingFactor(real damping, real duration, DampingCache &c)
        {
            if (damping == 1.0f)
                return 1.0f;
            if (damping != c.damping || duration != c.duration)
            {
                c.damping = damping;
                c.duration = duration;
                c.factor = std::pow(damping, duration);
            }
            return c.factor;
        }

        static inline void updateTransformMatrix(Matrix2 &transformMatrix, const Vector2 &position, const real orientation)
        {
            transformMatrix.setOrientation(orientation);
        }
    };
}