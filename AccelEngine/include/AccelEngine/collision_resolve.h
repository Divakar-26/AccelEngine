#pragma once
#include <AccelEngine/body.h>
#include <AccelEngine/collision_narrow.h>
#include <atomic>
#include <memory>
#include <vector>

namespace AccelEngine
{
    // Tunables for the contact solver and sleeping. Units: pixels, seconds, radians.
    struct PhysicsSettings
    {
        // ---- Stepping (used by World::step(dt)) ----
        int substeps = 6; // substeps per World::step(dt); joints need >= 6 to stay steady

        // ---- Safety limits: a body beyond these is clamped (speed) or disabled (NaN/inf) so one bad body can't
        //      poison the whole world. Set to infinity to turn a limit off. ----
        real maxLinearSpeed = 20000.0f;   // px/s
        real maxAngularSpeed = 300.0f;    // rad/s

        // ---- Contact solver (all per substep) ----
        int velocityIterations = 4;   // sequential-impulse passes over all contacts
        int positionIterations = 2;   // penetration-correction passes over all contacts
        int jointIterations = 100;    // joint velocity passes (kept from the original solver)

        real linearSlop = 0.5f;           // allowed penetration in px; keeps resting contacts alive between substeps
        real baumgarte = 0.2f;            // fraction of (penetration - slop) removed per position iteration
        real maxLinearCorrection = 10.0f; // px moved per position iteration, avoids popping out of deep overlaps
        real restitutionThreshold = 60.0f; // px/s; impacts slower than this don't bounce (kills resting micro-bounce)
        bool warmStarting = true;          // start each contact from last substep's impulse (needed for tall stacks)

        // ---- Sleeping ----
        bool enableSleep = true;
        real sleepLinearTolerance = 4.0f;   // px/s
        real sleepAngularTolerance = 0.05f; // rad/s
        real timeToSleep = 0.5f;            // s an entire island must stay below the tolerances before it sleeps

        // ---- Threading ----
        int workerThreads = -1; // extra threads used by World::step; -1 = auto (hardware threads - 1), 0 = single-threaded
    };

    class ThreadPool;

    // Sequential-impulse contact solver with accumulated (clamped) impulses, Coulomb friction,
    // restitution threshold and a separate position-correction pass that does not add velocity.
    class ContactSolver
    {
    public:
        // bodies[i]->worldIndex must equal i. Copies the bodies' velocities into solver arrays;
        // RigidBody velocities are stale until finishVelocities(). `pool` may be null (single-threaded).
        void prepare(const std::vector<Contact> &contacts, const std::vector<RigidBody *> &bodies, real dt,
                     const PhysicsSettings &settings, ThreadPool *pool = nullptr);
        void warmStart();
        void solveVelocities();
        // Writes velocities back to the bodies and remembers impulses for warm starting the next substep.
        void finishVelocities();
        // Runs `iterations` position passes and writes the corrected positions/orientations back.
        void solvePositions(int iterations);
        void reset() { cacheEntries.clear(); cacheHeadSize = 0; cacheHead.reset(); constraints.clear(); } // drop warm-start data

    private:
        // Compact per-body state used by the hot loops.
        struct SolverBody
        {
            Vector2 v;       // linear velocity
            real w;          // angular velocity
            real mInv, iInv; // effective inverse mass / inertia (locks applied)
            Vector2 dp;      // position correction accumulated by solvePositions
            real dq;         // orientation correction accumulated by solvePositions
        };
        // Per-body data only needed while building constraints (kept apart so the hot loops stay small).
        struct BodyInfo
        {
            Vector2 position;
            real cosQ, sinQ;  // transformMatrix (for the owner-local warm-start anchors)
            real restitution, staticFriction, dynamicFriction;
        };

        struct Point
        {
            Vector2 rA, rB;       // contact point relative to body centres
            Vector2 localAnchor;  // point in the owner body's frame (warm-start matching)
            real normalMass = 0;
            real tangentMass = 0;
            real normalImpulse = 0;  // accumulated, >= 0
            real tangentImpulse = 0; // accumulated, |.| <= friction * normalImpulse
            real velocityBias = 0;   // restitution target velocity
            real separation = 0;     // at prepare time; < 0 means penetrating
        };

        struct Constraint
        {
            int ia, ib;           // SolverBody indices
            bool movA, movB;      // false for bodies the solver never moves (never written -> no data races)
            bool aOwns;           // a has the lower worldIndex (owns the warm-start cache entry)
            RigidBody *a, *b;
            Vector2 normal, tangent;
            real friction;
            int count;
            Point points[2];
            // 2-point block solver: normal mass matrix K and its inverse (valid when count == 2)
            real k11, k12, k22;
            real invK11, invK12, invK22;
        };

        // Warm-start cache: last substep's impulses grouped by owner body (the one with the lower
        // worldIndex), matched by contact position in the owner's local frame.
        struct CachedPoint
        {
            Vector2 localAnchor;
            real normalImpulse, tangentImpulse;
        };
        struct CacheEntry
        {
            const RigidBody *other;
            int next;   // next entry with the same owner, -1 = end
            int count;
            CachedPoint points[2];
        };
        std::vector<CacheEntry> cacheEntries;     // one per constraint of the previous substep
        std::unique_ptr<std::atomic<int>[]> cacheHead; // per worldIndex: first entry owned by that body, -1 = none
        int cacheHeadSize = 0;

        static Vector2 toLocal(const RigidBody *body, const Vector2 &worldPoint);
        void setupConstraint(Constraint &k, const Contact &c, real dt, real invDt, const PhysicsSettings &settings);
        template <typename F>
        void forEachColor(F &&fn); // runs fn(begin, end) over constraint ranges, colours in order

        std::vector<SolverBody> solverBodies; // indexed by RigidBody::worldIndex
        std::vector<BodyInfo> bodyInfo;       // indexed by RigidBody::worldIndex
        const std::vector<RigidBody *> *bodies = nullptr;
        std::vector<uint32_t> colorMask;      // per worldIndex: colours already used by its constraints
        std::vector<int> scratchSlots;        // per input contact: its constraint slot, -1 = skipped
        int bodyCount = 0;

        static constexpr int kColors = 32;  // constraints that do not fit go to one extra serial bucket
        std::vector<int> contactColor;      // per input contact: colour, or -1 when skipped
        int colorStart[kColors + 2] = {};   // constraints[colorStart[c] .. colorStart[c+1]) have colour c

        std::vector<Constraint> constraints; // ordered by colour
        ThreadPool *pool = nullptr;
        bool warmStarting = true;
        real slop = 0.5f;
        real baumgarte = 0.2f;
        real maxCorrection = 10.0f;
        real matchDist2 = 4.0f;
    };

    // Legacy single-contact helpers (no longer used by World::step).
    class CollisionResolve
    {
    public:
        static void SolvePosition(Contact& contact, float correctionFactor = 0.8f, float slop = 0.01f);
        static void SolvePositionWithRotation(Contact &contact, float baumgarte, float slop);

        static void SolveVelocity(Contact& contact, float friction = 0.4f);
        static void SolveVelocityWithRoatation(Contact & contact);
        static void SolveVelocityWithRoatationAndFriction(Contact & contact);
        // Complete resolution (position + velocity)
        static void Solve(Contact& contact, float dt);
    };
}
