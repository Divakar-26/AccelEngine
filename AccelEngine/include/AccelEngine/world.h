#pragma once
#include <AccelEngine/core.h>
#include <AccelEngine/body.h>
#include <AccelEngine/collision_coarse.h>
#include <AccelEngine/collision_narrow.h>
#include <AccelEngine/collision_resolve.h>
#include <AccelEngine/joint.h>
#include <AccelEngine/shape.h>
#include <AccelEngine/BVH.h>
#include <AccelEngine/profiler.h>
#include <AccelEngine/thread_pool.h>
#include <AccelEngine/ForceRegistry.h>
#include <algorithm>
#include <memory>
#include <thread>
#include <atomic>
#include <limits>
#include <vector>

namespace AccelEngine
{

    struct CollisionEvent
    {
        RigidBody *a;
        RigidBody *b;
        bool sensor = false; // one of the two is a sensor (see RigidBody::isSensor)
    };

    // Everything needed to create a body with World::createCircle / createBox.
    struct BodyDef
    {
        Vector2 position{0, 0};
        real angle = 0.0f;              // radians, counter-clockwise
        Vector2 velocity{0, 0};
        real angularVelocity = 0.0f;
        real mass = 1.0f;               // <= 0 makes the body static
        real restitution = 0.3f;
        real staticFriction = 0.6f;
        real dynamicFriction = 0.4f;
        real linearDamping = 1.0f;      // fraction of velocity kept per second (1 = none)
        real angularDamping = 1.0f;
        Color color{200.0f, 200.0f, 200.0f, 255.0f};
        bool collides = true;

        // filtering (see RigidBody)
        uint16_t categoryBits = 0x0001;
        uint16_t maskBits = 0xFFFF;
        int16_t groupIndex = 0;
        bool isSensor = false;
        bool oneWay = false;
        Vector2 oneWayNormal{0.0f, 1.0f};
    };

    struct RayHit
    {
        RigidBody *body = nullptr;
        real distance = 0.0f; // along the ray
        Vector2 point;
        Vector2 normal;       // surface normal at the hit, facing the ray origin
    };

    class World
    {
    protected:
        std::vector<RigidBody *> bodies;
        // bodies/joints created through createX(): freed by World (destroyBody / clear / destructor)
        std::vector<std::shared_ptr<RigidBody>> ownedBodies;
        std::vector<std::shared_ptr<Joint>> ownedJoints;
        int disabledBodies = 0;
        std::vector<std::pair<RigidBody *, RigidBody *>> potentialPairs;
        std::vector<Contact> contacts;
        std::vector<Contact> contactsThisFrame;

        ContactSolver solver;

        // worker threads (shared_ptr keeps World copyable, as before)
        std::shared_ptr<ThreadPool> pool;
        int poolWorkers = -2;
        std::vector<std::vector<Contact>> chunkContacts;
        std::vector<std::vector<CollisionEvent>> chunkEvents;
        std::vector<std::vector<RigidBody *>> chunkWake;
        int narrowChunks = 0;
        std::vector<size_t> chunkOffset;

        // island scratch (union-find over bodies), reused every substep
        std::vector<int> islandParent;
        int substepsSinceSleepCheck = 0;
        std::vector<real> islandMinSleepTime;

    public:
        std::vector<Joint *> joints;
        std::vector<CollisionEvent> collisionEvents;
        BVHTree broadPhase;
        PhysicsSettings settings;

        // Used by step(dt) only (step(dt, substeps) leaves forces to the caller, as before).
        Vector2 gravity{0.0f, -980.0f};
        ForceRegistry forces;

        World() {}

        ~World()
        {
            clear();
        }

        // ---- Bodies ------------------------------------------------------------------------------------

        // Adds a body you own (you keep the pointer valid until removeBody / clear).
        void addBody(RigidBody *body)
        {
            body->wakeUp();
            bodies.push_back(body);
        }

        // Creates a body owned by the World. It is freed by destroyBody(), clear() or the destructor.
        RigidBody *createCircle(real radius, const BodyDef &def = BodyDef())
        {
            RigidBody *b = newBody(def);
            b->shapeType = ShapeType::CIRCLE;
            b->circle.radius = radius;
            return finishBody(b, def);
        }

        RigidBody *createBox(const Vector2 &halfSize, const BodyDef &def = BodyDef())
        {
            RigidBody *b = newBody(def);
            b->shapeType = ShapeType::AABB;
            b->aabb.halfSize = halfSize;
            return finishBody(b, def);
        }

        // Convex polygon from any point cloud (its convex hull, at most kMaxPolygonVertices vertices). The points are
        // in the body's frame: that frame's origin is placed at def.position, the body's centre of mass is wherever
        // the hull's centroid is. `radius` > 0 rounds the corners. Returns nullptr if the points are degenerate.
        RigidBody *createPolygon(const std::vector<Vector2> &points, const BodyDef &def = BodyDef(), real radius = 0.0f)
        {
            ConvexPart part = ConvexPart::polygon(points.data(), (int)points.size(), radius);
            if (part.count < 3)
                return nullptr;
            return createFromParts({part}, ShapeType::POLYGON, def);
        }

        // Regular n-gon (3..8 sides) with the given circumradius; first vertex on +x.
        RigidBody *createRegularPolygon(int sides, real circumRadius, const BodyDef &def = BodyDef())
        {
            sides = std::clamp(sides, 3, kMaxPolygonVertices);
            std::vector<Vector2> pts;
            for (int i = 0; i < sides; i++)
            {
                const real a = 6.28318531f * (real)i / (real)sides;
                pts.push_back({std::cos(a) * circumRadius, std::sin(a) * circumRadius});
            }
            return createPolygon(pts, def);
        }

        // Capsule lying along the body's x axis: a segment of total length 2 * halfLength grown by `radius`.
        RigidBody *createCapsule(real halfLength, real radius, const BodyDef &def = BodyDef())
        {
            return createFromParts({ConvexPart::capsule({-halfLength, 0}, {halfLength, 0}, radius)}, ShapeType::CAPSULE, def);
        }

        // Capsule between two points of the body's frame.
        RigidBody *createCapsule(const Vector2 &a, const Vector2 &b, real radius, const BodyDef &def = BodyDef())
        {
            return createFromParts({ConvexPart::capsule(a, b, radius)}, ShapeType::CAPSULE, def);
        }

        // Rigid body made of several convex parts (ConvexPart::box / circle / capsule / polygon), e.g. an L, a T or a
        // dumbbell. Parts are in the body's frame; the frame's origin is placed at def.position and the centre of
        // mass is computed from the parts (area-weighted).
        RigidBody *createCompound(const std::vector<ConvexPart> &parts, const BodyDef &def = BodyDef())
        {
            std::vector<ConvexPart> valid;
            for (const ConvexPart &p : parts)
                if (p.count > 0)
                    valid.push_back(p);
            if (valid.empty())
                return nullptr;
            return createFromParts(valid, ShapeType::COMPOUND, def);
        }

        // Takes the body out of the simulation (and its joints, forces, contacts). Safe at any time except
        // from inside step(). The memory is freed only if the body was created with createX().
        void removeBody(RigidBody *body)
        {
            auto it = std::find(bodies.begin(), bodies.end(), body);
            if (it == bodies.end())
                return;

            // wake whatever may have been resting on it (sleeping pairs are not in the contact list)
            std::vector<RigidBody *> neighbours;
            const real m = settings.linearSlop * 2.0f + 1.0f;
            queryAABB(body->worldAABBMin - Vector2(m, m), body->worldAABBMax + Vector2(m, m), neighbours);
            for (RigidBody *n : neighbours)
                if (n != body)
                    n->wakeUp();

            bodies.erase(it);
            for (size_t k = joints.size(); k-- > 0;)
                if (joints[k]->A == body || joints[k]->B == body)
                    removeJoint(joints[k]);
            forces.remove(body);

            auto uses = [body](const Contact &c) { return c.a == body || c.b == body; };
            contactsThisFrame.erase(std::remove_if(contactsThisFrame.begin(), contactsThisFrame.end(), uses), contactsThisFrame.end());
            collisionEvents.erase(std::remove_if(collisionEvents.begin(), collisionEvents.end(),
                                                 [body](const CollisionEvent &e) { return e.a == body || e.b == body; }),
                                  collisionEvents.end());
            potentialPairs.clear();
            contacts.clear();
            solver.reset();      // warm-start data is indexed by body position in the list
            broadPhase.destroy();

            ownedBodies.erase(std::remove_if(ownedBodies.begin(), ownedBodies.end(),
                                             [body](const std::shared_ptr<RigidBody> &o) { return o.get() == body; }),
                              ownedBodies.end());
        }

        void destroyBody(RigidBody *body) { removeBody(body); }

        const std::vector<RigidBody *> &getBodies() const { return bodies; }
        int getBodyCount() const { return (int)bodies.size(); }

        // ---- Joints ------------------------------------------------------------------------------------

        void addJoint(Joint *j)
        {
            if (j->A) j->A->wakeUp();
            if (j->B) j->B->wakeUp();
            joints.push_back(j);
        }

        // A DistanceJoint owned by the World (freed by removeJoint / removeBody of an end / clear).
        // restLength < 0 keeps the current distance between the two anchor points.
        DistanceJoint *createDistanceJoint(RigidBody *a, RigidBody *b, const Vector2 &localA = Vector2(),
                                           const Vector2 &localB = Vector2(), real restLength = -1.0f)
        {
            auto j = std::make_shared<DistanceJoint>(a, b, localA, localB, restLength);
            ownedJoints.push_back(j);
            addJoint(j.get());
            return j.get();
        }

        // Pin / hinge: the bodies share the world point `anchor` and rotate freely around it (optionally limited or
        // motorised, see RevoluteJoint). `a` or `b` can be a static body to pin something to the world.
        RevoluteJoint *createRevoluteJoint(RigidBody *a, RigidBody *b, const Vector2 &anchor)
        {
            auto j = std::make_shared<RevoluteJoint>(a, b, anchor);
            ownedJoints.push_back(j);
            addJoint(j.get());
            return j.get();
        }

        // Wheel on a suspension: `b` (the wheel) is held on the line through `anchor` along `axis` (world space),
        // spins freely, and is sprung along the axis (see WheelJoint). `a` is usually the chassis.
        WheelJoint *createWheelJoint(RigidBody *a, RigidBody *b, const Vector2 &anchor, const Vector2 &axis)
        {
            auto j = std::make_shared<WheelJoint>(a, b, anchor, axis);
            ownedJoints.push_back(j);
            addJoint(j.get());
            return j.get();
        }

        // Rigidly glues two bodies together at their current relative pose.
        WeldJoint *createWeldJoint(RigidBody *a, RigidBody *b)
        {
            auto j = std::make_shared<WeldJoint>(a, b);
            ownedJoints.push_back(j);
            addJoint(j.get());
            return j.get();
        }

        void removeJoint(Joint *j)
        {
            joints.erase(std::remove(joints.begin(), joints.end(), j), joints.end());
            ownedJoints.erase(std::remove_if(ownedJoints.begin(), ownedJoints.end(),
                                             [j](const std::shared_ptr<Joint> &o) { return o.get() == j; }),
                              ownedJoints.end());
        }

        std::vector<Joint *> &getJoints()
        {
            return joints;
        }

        // ---- Events ------------------------------------------------------------------------------------

        // Pairs that touched during the last step() call (the whole call, not just its last substep).
        const std::vector<CollisionEvent> &GetCollisionEvents() const
        {
            return collisionEvents;
        }

        // Sensor overlaps of the last step() call (events with sensor == true; also part of GetCollisionEvents()).
        std::vector<CollisionEvent> getSensorEvents() const
        {
            std::vector<CollisionEvent> out;
            for (const CollisionEvent &e : collisionEvents)
                if (e.sensor)
                    out.push_back(e);
            return out;
        }

        // Empties the world: bodies, joints, force registrations, contacts. Memory of createX() bodies/joints is
        // freed; bodies you added with addBody() are NOT deleted. (Used to leave joints behind, dangling.)
        void clear()
        {
            bodies.clear();
            joints.clear();
            forces.clear();
            ownedBodies.clear();
            ownedJoints.clear();
            potentialPairs.clear();
            contacts.clear();
            contactsThisFrame.clear();
            collisionEvents.clear();
            solver.reset();
            broadPhase.destroy();
            disabledBodies = 0;
        }

        // Bodies the safety net had to switch off (NaN/inf state) since the last clear().
        int getDisabledBodyCount() const { return disabledBodies; }

        // ---- Queries (linear scans over the bodies' world boxes; fine for picking, not for thousands per frame) ----

        // Bodies whose bounding box overlaps [min, max].
        void queryAABB(const Vector2 &min, const Vector2 &max, std::vector<RigidBody *> &out) const
        {
            for (RigidBody *b : bodies)
                if (b->worldAABBMax.x >= min.x && b->worldAABBMin.x <= max.x &&
                    b->worldAABBMax.y >= min.y && b->worldAABBMin.y <= max.y)
                    out.push_back(b);
        }

        // A body containing the point (dynamic bodies are preferred over static ones), or nullptr.
        RigidBody *queryPoint(const Vector2 &p) const
        {
            RigidBody *found = nullptr;
            for (RigidBody *b : bodies)
            {
                if (!b->enableCollision || !containsPoint(b, p))
                    continue;
                if (!b->isStatic())
                    return b;
                if (!found)
                    found = b;
            }
            return found;
        }

        // Closest hit along origin + t * direction, 0 <= t <= maxDistance. A ray starting inside a body hits it at 0.
        bool raycast(const Vector2 &origin, const Vector2 &direction, real maxDistance, RayHit &hit) const
        {
            const real len = direction.magnitude();
            if (!(len > 0.0f) || !(maxDistance > 0.0f))
                return false;
            const Vector2 d = direction / len;
            real bestT = maxDistance;
            bool any = false;
            for (const RigidBody *b : bodies)
            {
                if (!b->enableCollision)
                    continue;
                real t;
                Vector2 n;
                if (rayVsBody(b, origin, d, bestT, t, n) && t <= bestT)
                {
                    bestT = t;
                    hit.body = const_cast<RigidBody *>(b);
                    hit.distance = t;
                    hit.point = origin + d * t;
                    hit.normal = n;
                    any = true;
                }
            }
            return any;
        }

        // ---- Settings shortcuts ------------------------------------------------------------------------

        void setGravity(const Vector2 &g) { gravity = g; }
        const Vector2 &getGravity() const { return gravity; }

        const std::vector<Contact> getContacts() const
        {
            return contactsThisFrame;
        }

        void wakeAll()
        {
            for (RigidBody *b : bodies)
                b->wakeUp();
        }

        int getAwakeBodyCount() const
        {
            int n = 0;
            for (const RigidBody *b : bodies)
                if (!b->isStatic() && b->isAwake)
                    n++;
            return n;
        }

        int getDynamicBodyCount() const
        {
            int n = 0;
            for (const RigidBody *b : bodies)
                if (!b->isStatic())
                    n++;
            return n;
        }

        void startFrame()
        {
            parallelFor((int)bodies.size(), 512, [this](int lo, int hi)
            {
                for (int i = lo; i < hi; i++)
                {
                    bodies[i]->clearAccumulators();
                    bodies[i]->calculateDerivativeData();
                }
            });
        }

        void runPhysics(real duration)
        {
            for (RigidBody *r : bodies)
            {
                r->integrate(duration);
            }
        }

        // One simulation step of `dt` seconds using settings.substeps substeps. Applies `gravity` and `forces`
        // itself (startFrame() + forces + step per substep), so this is all a game loop has to call.
        void step(float dt)
        {
            if (!(dt > 0.0f) || !std::isfinite(dt))
                return;
            const int n = std::max(1, settings.substeps);
            const float h = dt / (float)n;
            beginEvents();
            for (int i = 0; i < n; i++)
            {
                startFrame();
                applyGravity();
                forces.updateForces(h);
                substep(h);
            }
            endStep(n);
        }

        // Advances by dt in `substeps` substeps WITHOUT applying gravity or forces: the caller does that
        // (startFrame() + its own force registry) before each call, as the Sandbox does with substeps = 1.
        void step(float dt, int substeps)
        {
            if (!(dt > 0.0f) || !std::isfinite(dt) || substeps < 1)
                return;
            const float subdt = dt / (float)substeps;
            beginEvents();
            for (int i = 0; i < substeps; i++)
                substep(subdt);
            endStep(substeps);
        }

    private:
        void beginEvents() { collisionEvents.clear(); }

        // Merge duplicate pairs reported by several substeps, in a deterministic order.
        void endStep(int substeps)
        {
            if (substeps > 1 && collisionEvents.size() > 1)
            {
                auto key = [](const CollisionEvent &e)
                {
                    const int ia = e.a->worldIndex, ib = e.b->worldIndex;
                    return std::make_pair(std::min(ia, ib), std::max(ia, ib));
                };
                std::sort(collisionEvents.begin(), collisionEvents.end(),
                          [&](const CollisionEvent &x, const CollisionEvent &y) { return key(x) < key(y); });
                collisionEvents.erase(std::unique(collisionEvents.begin(), collisionEvents.end(),
                                                  [&](const CollisionEvent &x, const CollisionEvent &y) { return key(x) == key(y); }),
                                      collisionEvents.end());
            }
            contactsThisFrame.swap(contacts); // O(1); `contacts` is overwritten by the next substep
        }

        void applyGravity()
        {
            if (gravity.x == 0.0f && gravity.y == 0.0f)
                return;
            const Vector2 g = gravity;
            parallelFor((int)bodies.size(), 1024, [this, g](int lo, int hi)
            {
                for (int i = lo; i < hi; i++)
                {
                    RigidBody *b = bodies[i];
                    if (b->inverseMass > 0.0f && !b->ignoreGravity && b->isAwake)
                        b->forceAccum += g * (1.0f / b->inverseMass);
                }
            });
        }

        // Clamps runaway velocities and switches off bodies whose state became NaN/inf, so one bad body
        // (an exploding joint, a user typo) cannot spread NaN through contacts to the whole world.
        void sanitize(RigidBody *b, std::atomic<int> &disabled)
        {
            if (b->isStatic() || !b->isAwake)
                return;
            if (!b->isFinite())
            {
                if (!std::isfinite(b->position.x) || !std::isfinite(b->position.y)) b->position = Vector2(0, 0);
                if (!std::isfinite(b->orientation)) b->orientation = 0.0f;
                b->velocity = Vector2(0, 0);
                b->rotation = 0.0f;
                b->enableCollision = false;
                b->calculateDerivativeData();
                b->setAwake(false);
                disabled.fetch_add(1, std::memory_order_relaxed);
                return;
            }
            const real maxV = settings.maxLinearSpeed;
            const real v2 = b->velocity.squareMagnitude();
            if (v2 > maxV * maxV)
                b->velocity *= maxV / std::sqrt(v2);
            const real maxW = settings.maxAngularSpeed;
            if (b->rotation > maxW) b->rotation = maxW;
            else if (b->rotation < -maxW) b->rotation = -maxW;
        }

        void substep(float subdt)
        {
            ThreadPool *tp = getPool();
            {
                {
                    PROFILE_SCOPE("1 Integrate");
                    std::atomic<int> disabled{0};
                    parallelFor((int)bodies.size(), 512, [this, subdt, &disabled](int lo, int hi)
                    {
                        for (int k = lo; k < hi; k++)
                        {
                            bodies[k]->worldIndex = k;
                            bodies[k]->integrate(subdt);
                            sanitize(bodies[k], disabled);
                        }
                    });
                    disabledBodies += disabled.load(std::memory_order_relaxed);
                }

                // (contacts is resized by findContacts; not clearing it avoids re-constructing every element)

                {
                    PROFILE_SCOPE("2 Broad phase");
                    broadPhase.margin = settings.linearSlop;
                    broadPhase.build(bodies, tp); // after integration, so pairs use current AABBs
                    broadPhase.findPairs(potentialPairs, tp); // only pairs with an awake, dynamic body
                }
                {
                    PROFILE_SCOPE("3 Narrow phase");
                    findContacts();
                }

                {
                    PROFILE_SCOPE("4 Events + wake");
                    // collision events and bodies to wake were collected per chunk by the narrow phase
                    for (int c = 0; c < narrowChunks; c++)
                    {
                        collisionEvents.insert(collisionEvents.end(), chunkEvents[c].begin(), chunkEvents[c].end());
                        // a sleeping body touched by an awake one wakes (keeping its sleep timer) before solving
                        for (RigidBody *b : chunkWake[c])
                            b->isAwake = true;
                    }
                    for (const Joint *j : joints)
                    {
                        if (!jointIsActive(j))
                            continue;
                        if (j->A && !j->A->isAwake && !j->A->isStatic()) j->A->isAwake = true;
                        if (j->B && !j->B->isAwake && !j->B->isStatic()) j->B->isAwake = true;
                    }
                }

                for (auto *j : joints)
                    if (jointIsActive(j))
                        j->preSolve(subdt);

                {
                    PROFILE_SCOPE("5 Solve prepare");
                    solver.prepare(contacts, bodies, subdt, settings, tp);
                }
                {
                    PROFILE_SCOPE("6 Solve velocities");
                    solver.warmStart();
                    for (int it = 0; it < settings.velocityIterations; it++)
                        solver.solveVelocities();
                }
                {
                    PROFILE_SCOPE("7 Solve finish");
                    solver.finishVelocities();
                }

                {
                    PROFILE_SCOPE("8 Joints");
                    for (int it = 0; it < settings.jointIterations; it++)
                    {
                        for (auto *j : joints)
                        {
                            if (jointIsActive(j))
                                j->solve(subdt);
                        }
                    }
                }

                {
                    PROFILE_SCOPE("9 Positions");
                    solver.solvePositions(settings.positionIterations);

                    parallelFor((int)bodies.size(), 512, [this](int lo, int hi)
                    {
                        for (int k = lo; k < hi; k++)
                            if (bodies[k]->isAwake && !bodies[k]->isStatic())
                                bodies[k]->calculateDerivativeData();
                    });
                }

                {
                    PROFILE_SCOPE("10 Sleep");
                    if (settings.enableSleep)
                        updateSleep(subdt);
                    else
                        for (auto *b : bodies)
                            if (!b->isAwake)
                                b->wakeUp();
                }
            }
        }

        ThreadPool *getPool()
        {
            int want = settings.workerThreads;
            if (want < 0)
            {
                // hardware_concurrency() reads /sys on Linux: query it once
                static const int hw = std::max(1, (int)std::thread::hardware_concurrency());
                want = hw - 1;
            }
            if (want != poolWorkers)
            {
                pool = want > 0 ? std::make_shared<ThreadPool>(want) : nullptr;
                poolWorkers = want;
            }
            return pool.get();
        }

        // fn(begin, end) over [0, count), on the worker threads when available
        template <typename F>
        void parallelFor(int count, int grain, F &&fn)
        {
            ThreadPool *tp = getPool();
            if (tp)
                tp->parallelFor(count, grain, [&fn](int lo, int hi, int) { fn(lo, hi); });
            else if (count > 0)
                fn(0, count);
        }

        RigidBody *newBody(const BodyDef &def)
        {
            auto sp = std::make_shared<RigidBody>();
            ownedBodies.push_back(sp);
            RigidBody *b = sp.get();
            b->position = def.position;
            b->orientation = def.angle;
            b->velocity = def.velocity;
            b->rotation = def.angularVelocity;
            b->restitution = def.restitution;
            b->staticFriction = def.staticFriction;
            b->dynamicFriction = def.dynamicFriction;
            b->linearDamping = def.linearDamping;
            b->angularDamping = def.angularDamping;
            b->c = def.color;
            b->enableCollision = def.collides;
            b->categoryBits = def.categoryBits;
            b->maskBits = def.maskBits;
            b->groupIndex = def.groupIndex;
            b->isSensor = def.isSensor;
            b->oneWay = def.oneWay;
            b->oneWayNormal = def.oneWayNormal;
            return b;
        }

        RigidBody *finishBody(RigidBody *b, const BodyDef &def)
        {
            b->inverseMass = def.mass > 0.0f ? 1.0f / def.mass : 0.0f;
            b->calculateInertia();
            b->calculateDerivativeData();
            addBody(b);
            return b;
        }

        // Shared by createPolygon / createCapsule / createCompound: moves the parts so the centre of mass is the
        // body origin (and the body so the user's frame origin stays at def.position).
        RigidBody *createFromParts(std::vector<ConvexPart> parts, ShapeType type, const BodyDef &def)
        {
            real totalArea = 0.0f;
            Vector2 com(0, 0);
            for (const ConvexPart &p : parts)
            {
                real a, i;
                Vector2 c;
                p.massProperties(a, c, i);
                totalArea += a;
                com += c * a;
            }
            if (totalArea > 1e-9f)
                com = com / totalArea;
            else
                com = Vector2(0, 0);
            for (ConvexPart &p : parts)
            {
                for (int k = 0; k < p.count; k++)
                    p.v[k] -= com;
                p.finalize();
            }
            RigidBody *b = newBody(def);
            b->shapeType = type;
            b->parts = std::move(parts);
            const real cs = std::cos(def.angle), sn = std::sin(def.angle);
            b->position = def.position + Vector2(cs * com.x - sn * com.y, sn * com.x + cs * com.y);
            return finishBody(b, def);
        }

        // signed distance from p (body frame) to the rounded part; <= 0 means inside
        static real distanceToPart(const ConvexPart &part, const Vector2 &p)
        {
            if (part.count == 1)
                return (p - part.v[0]).magnitude() - part.radius;
            real best = std::numeric_limits<real>::lowest();
            for (int i = 0; i < part.count; i++)
                best = std::max(best, part.n[i].scalarProduct(p - part.v[i]));
            if (part.count >= 3 && best <= 0.0f)
                return best - part.radius;
            real d2 = std::numeric_limits<real>::max();
            for (int i = 0; i < part.count; i++)
            {
                const Vector2 a = part.v[i], b = part.v[(i + 1) % part.count];
                const Vector2 ab = b - a;
                const real l2 = ab.squareMagnitude();
                const real t = l2 > 1e-12f ? std::clamp((p - a).scalarProduct(ab) / l2, 0.0f, 1.0f) : 0.0f;
                d2 = std::min(d2, (p - (a + ab * t)).squareMagnitude());
            }
            return std::sqrt(d2) - part.radius;
        }

        static bool containsPoint(const RigidBody *b, const Vector2 &p)
        {
            if (b->usesParts())
            {
                const Vector2 l = b->transformMatrix.transformTranspose(p - b->position);
                for (const ConvexPart &part : b->parts)
                    if (distanceToPart(part, l) <= 0.0f)
                        return true;
                return false;
            }
            if (b->shapeType == ShapeType::CIRCLE)
                return (p - b->position).squareMagnitude() <= b->circle.radius * b->circle.radius;
            const Vector2 l = b->transformMatrix.transformTranspose(p - b->position);
            return std::fabs(l.x) <= b->aabb.halfSize.x && std::fabs(l.y) <= b->aabb.halfSize.y;
        }

        // ray (local frame) vs circle: nearest entry distance in [0, maxT]
        static bool rayVsCircleLocal(const Vector2 &o, const Vector2 &d, const Vector2 &c, real r, real maxT, real &t)
        {
            const Vector2 m = o - c;
            const real bq = m.scalarProduct(d);
            const real cq = m.squareMagnitude() - r * r;
            const real disc = bq * bq - cq;
            if (disc < 0.0f || (cq > 0.0f && bq > 0.0f))
                return false;
            t = -bq - std::sqrt(disc);
            return t >= 0.0f && t <= maxT;
        }

        // ray (local frame) vs convex polygon with vertices v[0..n) (CCW): slab clipping against every edge
        static bool rayVsPolyLocal(const Vector2 &o, const Vector2 &d, const Vector2 *v, int n, real maxT, real &t, Vector2 &normal)
        {
            real tmin = 0.0f, tmax = maxT;
            int hit = -1;
            for (int i = 0; i < n; i++)
            {
                const Vector2 e = v[(i + 1) % n] - v[i];
                const Vector2 nn = Vector2(e.y, -e.x).normalized();
                const real denom = nn.scalarProduct(d);
                const real dist = nn.scalarProduct(o - v[i]);
                if (std::fabs(denom) < 1e-9f)
                {
                    if (dist > 0.0f)
                        return false;
                    continue;
                }
                const real tt = -dist / denom;
                if (denom < 0.0f) // entering
                {
                    if (tt > tmin) { tmin = tt; hit = i; normal = nn; }
                }
                else
                    tmax = std::min(tmax, tt);
                if (tmin > tmax)
                    return false;
            }
            if (hit < 0)
                return false;
            t = tmin;
            return true;
        }

        static bool rayVsParts(const RigidBody *b, const Vector2 &o, const Vector2 &d, real maxT, real &t, Vector2 &n)
        {
            const Vector2 lo = b->transformMatrix.transformTranspose(o - b->position);
            const Vector2 ld = b->transformMatrix.transformTranspose(d);
            real bestT = maxT;
            Vector2 bestN;
            bool any = false;
            auto consider = [&](real tt, const Vector2 &ln)
            {
                if (tt <= bestT)
                {
                    bestT = tt;
                    bestN = ln;
                    any = true;
                }
            };
            for (const ConvexPart &part : b->parts)
            {
                real tt;
                Vector2 ln;
                if (part.count == 1)
                {
                    if (rayVsCircleLocal(lo, ld, part.v[0], part.radius, bestT, tt))
                        consider(tt, (lo + ld * tt - part.v[0]).normalized());
                }
                else if (part.count == 2)
                {
                    // capsule = two end circles + the rectangle between them
                    for (int k = 0; k < 2; k++)
                        if (rayVsCircleLocal(lo, ld, part.v[k], part.radius, bestT, tt))
                            consider(tt, (lo + ld * tt - part.v[k]).normalized());
                    const Vector2 side = part.n[0] * part.radius;
                    const Vector2 quad[4] = {part.v[0] - side, part.v[1] - side, part.v[1] + side, part.v[0] + side};
                    // quad above is clockwise or counter-clockwise depending on n[0]; build it CCW
                    Vector2 ccw[4] = {quad[0], quad[1], quad[2], quad[3]};
                    if ((quad[1] - quad[0]).cross(quad[2] - quad[1]) < 0.0f)
                        std::reverse(ccw, ccw + 4);
                    if (rayVsPolyLocal(lo, ld, ccw, 4, bestT, tt, ln))
                        consider(tt, ln);
                }
                else if (rayVsPolyLocal(lo, ld, part.v, part.count, bestT, tt, ln))
                    consider(tt, ln); // (rounded polygons: the core outline is used)
            }
            if (!any)
                return false;
            t = bestT;
            n = b->transformMatrix * bestN;
            return true;
        }

        // d must be normalised. Hit distance in t (clamped to [0, maxT]), normal facing the ray.
        static bool rayVsBody(const RigidBody *b, const Vector2 &o, const Vector2 &d, real maxT, real &t, Vector2 &n)
        {
            if (containsPoint(b, o))
            {
                t = 0.0f;
                n = d * -1.0f;
                return true;
            }
            if (b->usesParts())
                return rayVsParts(b, o, d, maxT, t, n);
            if (b->shapeType == ShapeType::CIRCLE)
            {
                const Vector2 m = o - b->position;
                const real bq = m.scalarProduct(d);
                const real cq = m.squareMagnitude() - b->circle.radius * b->circle.radius;
                const real disc = bq * bq - cq;
                if (cq > 0.0f && bq > 0.0f)
                    return false; // outside and pointing away
                if (disc < 0.0f)
                    return false;
                t = -bq - std::sqrt(disc);
                if (t < 0.0f || t > maxT)
                    return false;
                n = (o + d * t - b->position).normalized();
                return true;
            }
            // box: slab test in the box's local frame
            const Vector2 lo = b->transformMatrix.transformTranspose(o - b->position);
            const Vector2 ld = b->transformMatrix.transformTranspose(d);
            real tmin = 0.0f, tmax = maxT;
            int hitAxis = -1;
            real hitSign = 1.0f;
            for (int a = 0; a < 2; a++)
            {
                const real h = a == 0 ? b->aabb.halfSize.x : b->aabb.halfSize.y;
                if (std::fabs(ld[a]) < 1e-9f)
                {
                    if (std::fabs(lo[a]) > h)
                        return false;
                    continue;
                }
                real t1 = (-h - lo[a]) / ld[a], t2 = (h - lo[a]) / ld[a];
                real sign = -1.0f; // entering through the -h face
                if (t1 > t2)
                {
                    std::swap(t1, t2);
                    sign = 1.0f;
                }
                if (t1 > tmin)
                {
                    tmin = t1;
                    hitAxis = a;
                    hitSign = sign;
                }
                tmax = std::min(tmax, t2);
                if (tmin > tmax)
                    return false;
            }
            if (hitAxis < 0)
                return false;
            t = tmin;
            Vector2 ln(0, 0);
            ln[hitAxis] = hitSign;
            n = b->transformMatrix * ln;
            return true;
        }

        // A one-way platform only pushes bodies that approach it from its front side.
        static bool oneWayAllows(const RigidBody *A, const RigidBody *B, const Contact &c)
        {
            auto allows = [](const RigidBody *plat, const RigidBody *other, const Vector2 &normalOut)
            {
                const Vector2 up = plat->transformMatrix * plat->oneWayNormal;
                if (normalOut.scalarProduct(up) < 0.5f)
                    return false; // touching from behind or from the side
                // moving away from the front face (e.g. jumping up through it): let it pass
                return (other->velocity - plat->velocity).scalarProduct(up) <= 1.0f;
            };
            if (A->oneWay && !allows(A, B, c.normal))
                return false;
            if (B->oneWay && !allows(B, A, c.normal * -1.0f))
                return false;
            return true;
        }

        // joints created with collideConnected == false switch off collisions between their two bodies
        std::vector<std::pair<const RigidBody *, const RigidBody *>> noCollidePairs;

        void gatherNoCollidePairs()
        {
            noCollidePairs.clear();
            for (const Joint *j : joints)
                if (!j->collideConnected && j->A && j->B)
                    noCollidePairs.push_back(j->A < j->B ? std::make_pair((const RigidBody *)j->A, (const RigidBody *)j->B)
                                                         : std::make_pair((const RigidBody *)j->B, (const RigidBody *)j->A));
            std::sort(noCollidePairs.begin(), noCollidePairs.end());
        }

        bool isNoCollidePair(const RigidBody *a, const RigidBody *b) const
        {
            return std::binary_search(noCollidePairs.begin(), noCollidePairs.end(),
                                      a < b ? std::make_pair(a, b) : std::make_pair(b, a));
        }

        // Narrow phase over potentialPairs, in parallel; output order matches the pair order.
        void findContacts()
        {
            gatherNoCollidePairs();
            const int n = (int)potentialPairs.size();
            const int grain = 256;
            const int chunks = (n + grain - 1) / grain;
            if ((int)chunkContacts.size() < chunks)
            {
                chunkContacts.resize(chunks);
                chunkEvents.resize(chunks);
                chunkWake.resize(chunks);
            }
            narrowChunks = chunks;
            const real spec = settings.linearSlop;

            auto work = [this, spec](int lo, int hi, int chunk)
            {
                auto &out = chunkContacts[chunk];
                auto &events = chunkEvents[chunk];
                auto &wake = chunkWake[chunk];
                out.clear();
                events.clear();
                wake.clear();
                for (int k = lo; k < hi; k++)
                {
                    RigidBody *A = potentialPairs[k].first;
                    RigidBody *B = potentialPairs[k].second;
                    const Vector2 diff = B->position - A->position;
                    const real rsum = A->boundingRadius + B->boundingRadius + spec;
                    if (diff.scalarProduct(diff) > rsum * rsum)
                        continue;
                    if (!RigidBody::shouldCollide(*A, *B))
                        continue;
                    if (!noCollidePairs.empty() && isNoCollidePair(A, B))
                        continue;
                    const size_t first = out.size();
                    if (NarrowCollision::Collide(A, B, spec, out) == 0)
                        continue;

                    if (A->isSensor || B->isSensor)
                    {
                        // overlap only: report it, but no contact forces and nobody gets woken
                        bool overlap = false;
                        for (size_t q = first; q < out.size(); q++)
                            overlap |= out[q].depths[0] > 0.0f || (out[q].contactCount > 1 && out[q].depths[1] > 0.0f);
                        out.resize(first);
                        if (overlap)
                            events.push_back({A, B, true});
                        continue;
                    }

                    if (A->oneWay || B->oneWay)
                    {
                        size_t keep = first;
                        for (size_t q = first; q < out.size(); q++)
                            if (oneWayAllows(A, B, out[q]))
                                out[keep++] = out[q];
                        out.resize(keep);
                        if (keep == first)
                            continue;
                    }

                    bool touching = false;
                    for (size_t q = first; q < out.size(); q++)
                    {
                        out[q].indexA = A->worldIndex;
                        out[q].indexB = B->worldIndex;
                        touching |= out[q].depths[0] > 0.0f || (out[q].contactCount > 1 && out[q].depths[1] > 0.0f);
                    }
                    if (touching)
                        events.push_back({A, B});
                    if (!A->isAwake && !A->isStatic()) wake.push_back(A);
                    if (!B->isAwake && !B->isStatic()) wake.push_back(B);
                }
            };
            ThreadPool *tp = getPool();
            if (tp)
                tp->parallelFor(n, grain, work);
            else
                for (int c = 0; c < chunks; c++)
                    work(c * grain, std::min(n, (c + 1) * grain), c);

            chunkOffset.resize(chunks + 1);
            chunkOffset[0] = 0;
            for (int c = 0; c < chunks; c++)
                chunkOffset[c + 1] = chunkOffset[c] + chunkContacts[c].size();
            contacts.resize(chunkOffset[chunks]);
            auto copy = [this](int lo, int hi, int)
            {
                for (int c = lo; c < hi; c++)
                    std::copy(chunkContacts[c].begin(), chunkContacts[c].end(), contacts.begin() + chunkOffset[c]);
            };
            if (tp)
                tp->parallelFor(chunks, 4, copy);
            else
                copy(0, chunks, 0);
        }

        static bool isActive(const RigidBody *b)
        {
            return b && b->isAwake && !b->isStatic();
        }

        // A joint needs solving if at least one end can move and is awake.
        static bool jointIsActive(const Joint *j)
        {
            return isActive(j->A) || isActive(j->B);
        }

        int findRoot(int i)
        {
            while (islandParent[i] != i)
            {
                islandParent[i] = islandParent[islandParent[i]];
                i = islandParent[i];
            }
            return i;
        }

        void unite(const RigidBody *a, const RigidBody *b)
        {
            // static bodies do not connect islands (everything resting on the ground would merge)
            if (!a || !b || a->isStatic() || b->isStatic())
                return;
            // a joint may reference a body that is not (or no longer) in this world
            const int n = (int)bodies.size();
            if (a->worldIndex < 0 || a->worldIndex >= n || bodies[a->worldIndex] != a ||
                b->worldIndex < 0 || b->worldIndex >= n || bodies[b->worldIndex] != b)
                return;
            int ra = findRoot(a->worldIndex);
            int rb = findRoot(b->worldIndex);
            if (ra != rb)
                islandParent[ra] = rb;
        }

        // Islands = bodies connected through contacts and joints (static bodies do not connect).
        void buildIslands()
        {
            const int n = (int)bodies.size();
            islandParent.resize(n);
            for (int i = 0; i < n; i++)
                islandParent[i] = i; // worldIndex was assigned at the start of the substep

            for (const Contact &c : contacts)
                unite(c.a, c.b);
            for (const Joint *j : joints)
                unite(j->A, j->B);
        }

        // Every substep: per-body sleep timers (parallel). An island falls asleep once every body in it has
        // been slow for settings.timeToSleep; islands are only built when some body could sleep, and at most
        // every kSleepCheckInterval substeps.
        void updateSleep(real dt)
        {
            const int n = (int)bodies.size();
            const real linTol2 = settings.sleepLinearTolerance * settings.sleepLinearTolerance;
            const real angTol = settings.sleepAngularTolerance;
            const real timeToSleep = settings.timeToSleep;

            std::atomic<bool> anyCandidate{false};
            parallelFor(n, 1024, [&](int lo, int hi)
            {
                bool candidate = false;
                for (int i = lo; i < hi; i++)
                {
                    RigidBody *b = bodies[i];
                    if (b->isStatic() || !b->isAwake)
                        continue;
                    if (!b->canSleep ||
                        b->velocity.squareMagnitude() > linTol2 ||
                        std::fabs(b->rotation) > angTol)
                        b->sleepTime = 0.0f;
                    else
                        b->sleepTime += dt;
                    candidate |= b->sleepTime >= timeToSleep;
                }
                if (candidate)
                    anyCandidate.store(true, std::memory_order_relaxed);
            });

            constexpr int kSleepCheckInterval = 4;
            if (++substepsSinceSleepCheck < kSleepCheckInterval || !anyCandidate.load())
                return;
            substepsSinceSleepCheck = 0;

            buildIslands();
            islandMinSleepTime.assign(n, std::numeric_limits<real>::max());
            for (int i = 0; i < n; i++)
            {
                const RigidBody *b = bodies[i];
                if (b->isStatic() || !b->isAwake)
                    continue;
                real &m = islandMinSleepTime[findRoot(i)];
                m = std::min(m, b->sleepTime);
            }
            for (int i = 0; i < n; i++)
            {
                RigidBody *b = bodies[i];
                if (b->isStatic() || !b->isAwake)
                    continue;
                if (islandMinSleepTime[findRoot(i)] >= timeToSleep)
                    b->setAwake(false);
            }
        }
    };
}
