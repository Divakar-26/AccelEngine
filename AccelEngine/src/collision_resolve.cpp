#include <AccelEngine/collision_resolve.h>
#include <AccelEngine/thread_pool.h>
#include <algorithm>
#include <cmath>
#include <iostream>
using namespace AccelEngine;

// ============================================================================
// ContactSolver
// ============================================================================

namespace
{
    // relative sliding speed below which static friction is used instead of dynamic (px/s)
    constexpr real kStaticFrictionSpeed = 2.0f;

    template <typename F>
    inline void runParallel(ThreadPool *pool, int count, int grain, F &&fn)
    {
        if (pool)
            pool->parallelFor(count, grain, fn);
        else if (count > 0)
            fn(0, count, 0);
    }

    template <typename Body>
    inline void applyImpulse(Body &A, Body &B, bool movA, bool movB, const Vector2 &rA, const Vector2 &rB, const Vector2 &P)
    {
        if (movA)
        {
            A.v.x -= P.x * A.mInv;
            A.v.y -= P.y * A.mInv;
            A.w -= A.iInv * rA.cross(P);
        }
        if (movB)
        {
            B.v.x += P.x * B.mInv;
            B.v.y += P.y * B.mInv;
            B.w += B.iInv * rB.cross(P);
        }
    }

    template <typename Body>
    inline Vector2 relativeVelocity(const Body &A, const Body &B, const Vector2 &rA, const Vector2 &rB)
    {
        return Vector2(B.v.x - B.w * rB.y - (A.v.x - A.w * rA.y),
                       B.v.y + B.w * rB.x - (A.v.y + A.w * rA.x));
    }
}

Vector2 ContactSolver::toLocal(const RigidBody *body, const Vector2 &worldPoint)
{
    return body->transformMatrix.transformTranspose(worldPoint - body->position);
}

template <typename F>
void ContactSolver::forEachColor(F &&fn)
{
    // constraints of one colour never share a moving body, so each colour can be split across threads
    for (int c = 0; c < kColors; c++)
    {
        const int b = colorStart[c], e = colorStart[c + 1];
        if (e > b)
            runParallel(pool, e - b, 128, [&](int lo, int hi, int) { fn(b + lo, b + hi); });
    }
    // overflow bucket: bodies with more than kColors contacts; solved serially
    if (colorStart[kColors + 1] > colorStart[kColors])
        fn(colorStart[kColors], colorStart[kColors + 1]);
}

void ContactSolver::prepare(const std::vector<Contact> &contacts, const std::vector<RigidBody *> &bodies_, real dt,
                            const PhysicsSettings &settings, ThreadPool *pool_)
{
    pool = pool_;
    bodies = &bodies_;
    const int n = (int)bodies_.size();
    if (n < bodyCount)
    {
        // World only appends bodies (indices stay valid); a shrink means indices were reused: start cold
        cacheEntries.clear();
        cacheHeadSize = 0;
    }
    bodyCount = n;
    if (cacheHeadSize < n)
    {
        // grow the head table, keeping the existing lists
        std::unique_ptr<std::atomic<int>[]> grown(new std::atomic<int>[n]);
        for (int i = 0; i < n; i++)
            grown[i].store(i < cacheHeadSize ? cacheHead[i].load(std::memory_order_relaxed) : -1, std::memory_order_relaxed);
        cacheHead = std::move(grown);
        cacheHeadSize = n;
    }

    warmStarting = settings.warmStarting;
    // contact points closer than this (px, owner-local) are treated as the same point as last substep
    const real matchDist = std::max(settings.linearSlop * 4.0f, 1.0f);
    matchDist2 = matchDist * matchDist;
    slop = settings.linearSlop;
    baumgarte = settings.baumgarte;
    maxCorrection = settings.maxLinearCorrection;
    const real invDt = dt > 0.0f ? 1.0f / dt : 0.0f;

    // ---- per-body solver state (parallel copy) ----
    solverBodies.resize(n);
    bodyInfo.resize(n);
    colorMask.assign(n, 0u);
    runParallel(pool, n, 1024, [&](int lo, int hi, int)
    {
        for (int i = lo; i < hi; i++)
        {
            const RigidBody *b = bodies_[i];
            solverBodies[i] = {b->velocity, b->rotation, b->getEffectiveInverseMass(), b->getEffectiveInverseInertia(),
                               Vector2(0, 0), 0.0f};
            bodyInfo[i] = {b->position, b->transformMatrix.data[0], b->transformMatrix.data[2],
                           b->restitution, b->staticFriction, b->dynamicFriction};
        }
    });

    // ---- serial pass: colour the constraints (greedy, lowest free colour per body pair) ----
    const int nc = (int)contacts.size();
    contactColor.resize(nc);
    int colorCount[kColors + 1] = {};
    for (int i = 0; i < nc; i++)
    {
        const Contact &c = contacts[i];
        const int ia = c.indexA >= 0 ? c.indexA : c.a->worldIndex;
        const int ib = c.indexB >= 0 ? c.indexB : c.b->worldIndex;
        const SolverBody &A = solverBodies[ia];
        const SolverBody &B = solverBodies[ib];
        const bool movA = A.mInv > 0.0f || A.iInv > 0.0f;
        const bool movB = B.mInv > 0.0f || B.iInv > 0.0f;
        // nothing can move (e.g. two overlapping static bodies): skip, otherwise 0/0 = NaN
        if (c.contactCount <= 0 || (!movA && !movB))
        {
            contactColor[i] = -1;
            continue;
        }
        const uint32_t used = (movA ? colorMask[ia] : 0u) | (movB ? colorMask[ib] : 0u);
        int color = kColors; // overflow
        if (used != 0xFFFFFFFFu)
        {
            color = __builtin_ctz(~used);
            if (movA) colorMask[ia] |= 1u << color;
            if (movB) colorMask[ib] |= 1u << color;
        }
        contactColor[i] = color;
        colorCount[color]++;
    }

    colorStart[0] = 0;
    for (int c = 0; c <= kColors; c++)
        colorStart[c + 1] = colorStart[c] + colorCount[c];
    constraints.resize(colorStart[kColors + 1]);

    int cursor[kColors + 1];
    for (int c = 0; c <= kColors; c++)
        cursor[c] = colorStart[c];
    scratchSlots.resize(nc);
    for (int i = 0; i < nc; i++)
        scratchSlots[i] = contactColor[i] >= 0 ? cursor[contactColor[i]]++ : -1;

    runParallel(pool, nc, 256, [&](int lo, int hi, int)
    {
        for (int i = lo; i < hi; i++)
            if (scratchSlots[i] >= 0)
                setupConstraint(constraints[scratchSlots[i]], contacts[i], dt, invDt, settings);
    });
}

void ContactSolver::setupConstraint(Constraint &k, const Contact &c, real dt, real invDt, const PhysicsSettings &settings)
{
    k.a = c.a;
    k.b = c.b;
    k.ia = c.indexA >= 0 ? c.indexA : c.a->worldIndex;
    k.ib = c.indexB >= 0 ? c.indexB : c.b->worldIndex;
    const SolverBody &A = solverBodies[k.ia];
    const SolverBody &B = solverBodies[k.ib];
    const BodyInfo &infoA = bodyInfo[k.ia];
    const BodyInfo &infoB = bodyInfo[k.ib];
    const real mA = A.mInv, mB = B.mInv, iA = A.iInv, iB = B.iInv;
    k.movA = mA > 0.0f || iA > 0.0f;
    k.movB = mB > 0.0f || iB > 0.0f;
    k.normal = c.normal;
    k.tangent = Vector2(c.normal.y, -c.normal.x);
    k.count = std::min(c.contactCount, 2);
    k.aOwns = k.ia < k.ib;

    const real restitution = std::min(infoA.restitution, infoB.restitution);
    const real staticFriction = (infoA.staticFriction + infoB.staticFriction) * 0.5f;
    const real dynamicFriction = (infoA.dynamicFriction + infoB.dynamicFriction) * 0.5f;
    bool sliding = false;

    for (int i = 0; i < k.count; i++)
    {
        Point &p = k.points[i];
        p.rA = c.contactPoints[i] - infoA.position;
        p.rB = c.contactPoints[i] - infoB.position;

        const real rnA = p.rA.cross(k.normal);
        const real rnB = p.rB.cross(k.normal);
        const real kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
        p.normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

        const real rtA = p.rA.cross(k.tangent);
        const real rtB = p.rB.cross(k.tangent);
        const real kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
        p.tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

        p.separation = -c.depths[i];
        p.normalImpulse = 0.0f;
        p.tangentImpulse = 0.0f;

        const Vector2 dv = relativeVelocity(A, B, p.rA, p.rB);
        const real vn = dv.scalarProduct(k.normal);

        const bool impact = vn < -settings.restitutionThreshold;
        if (p.separation > 0.0f && !(impact && vn * dt <= -p.separation))
            p.velocityBias = -p.separation * invDt; // speculative: may approach until the gap closes
        else if (impact)
            p.velocityBias = -restitution * vn;     // real impact (or one that lands this substep): bounce
        else
            p.velocityBias = 0.0f;                  // resting contact: no bounce

        if (std::fabs(dv.scalarProduct(k.tangent)) > kStaticFrictionSpeed)
            sliding = true;
    }

    k.friction = sliding ? dynamicFriction : staticFriction;

    // Solving both points of a manifold together (instead of point 0 then point 1) removes the
    // small torque bias that otherwise makes stacks lean and topple.
    if (k.count == 2)
    {
        const real rn1A = k.points[0].rA.cross(k.normal);
        const real rn1B = k.points[0].rB.cross(k.normal);
        const real rn2A = k.points[1].rA.cross(k.normal);
        const real rn2B = k.points[1].rB.cross(k.normal);

        k.k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
        k.k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
        k.k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

        const real det = k.k11 * k.k22 - k.k12 * k.k12;
        const real maxConditionNumber = 1000.0f;
        if (k.k11 * k.k11 < maxConditionNumber * det && det != 0.0f)
        {
            const real invDet = 1.0f / det;
            k.invK11 = k.k22 * invDet;
            k.invK12 = -k.k12 * invDet;
            k.invK22 = k.k11 * invDet;
        }
        else
        {
            // points are (nearly) redundant: keep the deeper one
            if (k.points[1].separation < k.points[0].separation)
                k.points[0] = k.points[1];
            k.count = 1;
        }
    }

    // ---- warm start from last substep's cache (read-only here) ----
    const RigidBody *other = k.aOwns ? k.b : k.a;
    const int ownerIndex = k.aOwns ? k.ia : k.ib;
    const BodyInfo &ownerInfo = k.aOwns ? infoA : infoB;
    for (int i = 0; i < k.count; i++)
    {
        Point &p = k.points[i];
        // world offset -> owner-local (transpose of the rotation)
        const Vector2 r = k.aOwns ? p.rA : p.rB;
        p.localAnchor = Vector2(ownerInfo.cosQ * r.x + ownerInfo.sinQ * r.y, -ownerInfo.sinQ * r.x + ownerInfo.cosQ * r.y);
    }
    if (warmStarting && ownerIndex < cacheHeadSize)
    {
        for (int e = cacheHead[ownerIndex].load(std::memory_order_relaxed); e >= 0; e = cacheEntries[e].next)
        {
            const CacheEntry &entry = cacheEntries[e];
            if (entry.other != other)
                continue;
            for (int i = 0; i < k.count; i++)
            {
                Point &p = k.points[i];
                for (int j = 0; j < entry.count; j++)
                {
                    if ((entry.points[j].localAnchor - p.localAnchor).squareMagnitude() < matchDist2)
                    {
                        p.normalImpulse = entry.points[j].normalImpulse;
                        const real maxF = k.friction * p.normalImpulse;
                        p.tangentImpulse = std::clamp(entry.points[j].tangentImpulse, -maxF, maxF);
                        break;
                    }
                }
            }
            // (no break: a compound body pair can own several entries; matching is by contact position)
        }
    }
}

void ContactSolver::warmStart()
{
    if (!warmStarting)
        return;
    forEachColor([this](int lo, int hi)
    {
        for (int ci = lo; ci < hi; ci++)
        {
            const Constraint &k = constraints[ci];
            SolverBody &A = solverBodies[k.ia];
            SolverBody &B = solverBodies[k.ib];
            for (int i = 0; i < k.count; i++)
            {
                const Point &p = k.points[i];
                applyImpulse(A, B, k.movA, k.movB, p.rA, p.rB, k.normal * p.normalImpulse + k.tangent * p.tangentImpulse);
            }
        }
    });
}

void ContactSolver::solveVelocities()
{
    forEachColor([this](int lo, int hi)
    {
    for (int ci = lo; ci < hi; ci++)
    {
        Constraint &k = constraints[ci];
        SolverBody &A = solverBodies[k.ia];
        SolverBody &B = solverBodies[k.ib];

        // ---- friction first (clamped by the current normal impulse), normal is more important ----
        for (int i = 0; i < k.count; i++)
        {
            Point &p = k.points[i];
            const real vt = relativeVelocity(A, B, p.rA, p.rB).scalarProduct(k.tangent);
            real lambda = -p.tangentMass * vt;

            const real maxFriction = k.friction * p.normalImpulse;
            const real newImpulse = std::clamp(p.tangentImpulse + lambda, -maxFriction, maxFriction);
            lambda = newImpulse - p.tangentImpulse;
            p.tangentImpulse = newImpulse;

            applyImpulse(A, B, k.movA, k.movB, p.rA, p.rB, k.tangent * lambda);
        }

        // ---- normal, single point ----
        if (k.count == 1)
        {
            Point &p = k.points[0];
            const real vn = relativeVelocity(A, B, p.rA, p.rB).scalarProduct(k.normal);
            real lambda = -p.normalMass * (vn - p.velocityBias);

            const real newImpulse = std::max(p.normalImpulse + lambda, 0.0f);
            lambda = newImpulse - p.normalImpulse;
            p.normalImpulse = newImpulse;

            applyImpulse(A, B, k.movA, k.movB, p.rA, p.rB, k.normal * lambda);
            continue;
        }

        // ---- normal, two points: solve the 2x2 LCP exactly (Box2D's block solver) ----
        // vn = K * x + b, with x >= 0, vn >= 0, x_i * vn_i = 0
        Point &p1 = k.points[0];
        Point &p2 = k.points[1];
        const real a1 = p1.normalImpulse;
        const real a2 = p2.normalImpulse;

        real b1 = relativeVelocity(A, B, p1.rA, p1.rB).scalarProduct(k.normal) - p1.velocityBias;
        real b2 = relativeVelocity(A, B, p2.rA, p2.rB).scalarProduct(k.normal) - p2.velocityBias;
        b1 -= k.k11 * a1 + k.k12 * a2;
        b2 -= k.k12 * a1 + k.k22 * a2;

        real x1, x2;
        // case 1: both points active
        x1 = -(k.invK11 * b1 + k.invK12 * b2);
        x2 = -(k.invK12 * b1 + k.invK22 * b2);
        if (!(x1 >= 0.0f && x2 >= 0.0f))
        {
            // case 2: only point 1 active
            x1 = -p1.normalMass * b1;
            x2 = 0.0f;
            const real vn2 = k.k12 * x1 + b2;
            if (!(x1 >= 0.0f && vn2 >= 0.0f))
            {
                // case 3: only point 2 active
                x1 = 0.0f;
                x2 = -p2.normalMass * b2;
                const real vn1 = k.k12 * x2 + b1;
                if (!(x2 >= 0.0f && vn1 >= 0.0f))
                {
                    // case 4: both separating (or no solution found): no impulse
                    x1 = 0.0f;
                    x2 = 0.0f;
                    if (!(b1 >= 0.0f && b2 >= 0.0f))
                    {
                        // numerically stuck: keep previous impulses
                        x1 = a1;
                        x2 = a2;
                    }
                }
            }
        }

        p1.normalImpulse = x1;
        p2.normalImpulse = x2;
        applyImpulse(A, B, k.movA, k.movB, p1.rA, p1.rB, k.normal * (x1 - a1));
        applyImpulse(A, B, k.movA, k.movB, p2.rA, p2.rB, k.normal * (x2 - a2));
    }
    });
}

void ContactSolver::finishVelocities()
{
    const std::vector<RigidBody *> &bs = *bodies;

    // velocities back to the bodies (bodies the solver cannot move are left untouched)
    runParallel(pool, bodyCount, 1024, [&](int lo, int hi, int)
    {
        for (int i = lo; i < hi; i++)
        {
            const SolverBody &sb = solverBodies[i];
            if (sb.mInv == 0.0f && sb.iInv == 0.0f)
                continue;
            bs[i]->velocity = sb.v;
            bs[i]->rotation = sb.w;
        }
        // reset the warm-start lists for this range of owners
        for (int i = lo; i < hi; i++)
            cacheHead[i].store(-1, std::memory_order_relaxed);
    });
    if (!warmStarting)
    {
        cacheEntries.clear();
        return;
    }

    // ---- rebuild the warm-start cache: entry i = constraint i, linked into its owner's list ----
    cacheEntries.resize(constraints.size());
    runParallel(pool, (int)constraints.size(), 512, [&](int lo, int hi, int)
    {
        for (int ci = lo; ci < hi; ci++)
        {
            const Constraint &k = constraints[ci];
            CacheEntry &e = cacheEntries[ci];
            const int owner = k.aOwns ? k.ia : k.ib;
            e.other = k.aOwns ? k.b : k.a;
            e.count = k.count;
            for (int i = 0; i < k.count; i++)
            {
                e.points[i].localAnchor = k.points[i].localAnchor;
                e.points[i].normalImpulse = k.points[i].normalImpulse;
                e.points[i].tangentImpulse = k.points[i].tangentImpulse;
            }
            // lock-free push onto the owner's list (list order does not matter: one entry per pair)
            e.next = cacheHead[owner].exchange(ci, std::memory_order_acq_rel);
        }
    });
}

void ContactSolver::solvePositions(int iterations)
{
    if (iterations <= 0 || constraints.empty())
        return;

    runParallel(pool, bodyCount, 2048, [this](int lo, int hi, int)
    {
        for (int i = lo; i < hi; i++)
        {
            solverBodies[i].dp = Vector2(0, 0);
            solverBodies[i].dq = 0.0f;
        }
    });

    // Moves bodies directly (no velocity change), so penetration recovery never adds energy.
    for (int it = 0; it < iterations; it++)
    {
        forEachColor([this](int lo, int hi)
        {
            for (int ci = lo; ci < hi; ci++)
            {
                const Constraint &k = constraints[ci];
                SolverBody &A = solverBodies[k.ia];
                SolverBody &B = solverBodies[k.ib];

                for (int i = 0; i < k.count; i++)
                {
                    const Point &p = k.points[i];

                    // current separation, linearised around the state at prepare()
                    const Vector2 moveA = A.dp + p.rA.perpendicular() * A.dq;
                    const Vector2 moveB = B.dp + p.rB.perpendicular() * B.dq;
                    const real separation = p.separation + (moveB - moveA).scalarProduct(k.normal);

                    const real C = std::clamp(baumgarte * (separation + slop), -maxCorrection, 0.0f);
                    if (C >= 0.0f)
                        continue;

                    const Vector2 P = k.normal * (-C * p.normalMass);
                    if (k.movA)
                    {
                        A.dp -= P * A.mInv;
                        A.dq -= A.iInv * p.rA.cross(P);
                    }
                    if (k.movB)
                    {
                        B.dp += P * B.mInv;
                        B.dq += B.iInv * p.rB.cross(P);
                    }
                }
            }
        });
    }

    const std::vector<RigidBody *> &bs = *bodies;
    runParallel(pool, bodyCount, 1024, [&](int lo, int hi, int)
    {
        for (int i = lo; i < hi; i++)
        {
            const SolverBody &sb = solverBodies[i];
            if (sb.dq == 0.0f && sb.dp.x == 0.0f && sb.dp.y == 0.0f)
                continue;
            bs[i]->position += sb.dp;
            bs[i]->orientation += sb.dq;
        }
    });
}

// ============================================================================
// Legacy single-contact helpers
// ============================================================================

void CollisionResolve::SolvePosition(Contact &contact, float correctionFactor, float slop)
{
    RigidBody *A = contact.a;
    RigidBody *B = contact.b;

    if (contact.penetration <= slop)
        return;

    Vector2 correction = contact.normal * (contact.penetration - slop) * correctionFactor;

    // Apply correction based on inverse mass (heavier objects move less)
    float totalInverseMass = A->getInverseMass() + B->getInverseMass();

    if (totalInverseMass > 0.0f)
    {
        // Move each body proportionally to their mass
        if (A->getInverseMass() > 0.0f)
        {
            A->position -= correction * (A->getInverseMass() / totalInverseMass);
            A->calculateDerivativeData();
        }

        if (B->getInverseMass() > 0.0f)
        {
            B->position += correction * (B->getInverseMass() / totalInverseMass);
            B->calculateDerivativeData();
        }
    }
}

void CollisionResolve::SolvePositionWithRotation(Contact &contact, float baumgarte, float slop)
{
    RigidBody *A = contact.a;
    RigidBody *B = contact.b;

    if (contact.penetration <= slop)
        return;

    int count = contact.contactCount;
    if (count <= 0)
        return;

    float totalMass = A->inverseMass + B->inverseMass;

    for (int i = 0; i < count; i++)
    {
        Vector2 point = contact.contactPoints[i];

        Vector2 ra = point - A->position;
        Vector2 rb = point - B->position;

        Vector2 n = contact.normal;

        float baum = baumgarte * (contact.penetration - slop);

        float raCrossN = ra.cross(n);
        float rbCrossN = rb.cross(n);

        float denom =
            A->inverseMass +
            B->inverseMass +
            (raCrossN * raCrossN) * A->inverseInertia +
            (rbCrossN * rbCrossN) * B->inverseInertia;

        if (denom <= 0.0f)
            continue;

        float impulseMag = baum / denom;
        impulseMag /= (float)count;

        Vector2 impulse = n * impulseMag;

        if (A->inverseMass > 0.0f)
        {
            A->position -= impulse * A->inverseMass;
            A->orientation -= ra.cross(impulse) * A->inverseInertia;
        }

        if (B->inverseMass > 0.0f)
        {
            B->position += impulse * B->inverseMass;
            B->orientation += rb.cross(impulse) * B->inverseInertia;
        }
    }

    A->calculateDerivativeData();
    B->calculateDerivativeData();
}

void CollisionResolve::SolveVelocity(Contact &contact, float friction)
{
    RigidBody *A = contact.a;
    RigidBody *B = contact.b;

    Vector2 relativeVelocity = B->velocity - A->velocity;
    float velocityAlongNormal = relativeVelocity.scalarProduct(contact.normal);

    if (velocityAlongNormal > 0.0f)
        return;

    float totalInverseMass = A->getInverseMass() + B->getInverseMass();
    if (totalInverseMass <= 0.0f)
        return;

    real restituion = std::min(A->restitution, B->restitution);
    float j = -(1.0f + restituion) * velocityAlongNormal;
    j /= totalInverseMass;

    // Apply impulse
    Vector2 impulse = contact.normal * j;

    if (A->getInverseMass() > 0.0f)
    {
        A->velocity -= impulse * A->getInverseMass();
    }

    if (B->getInverseMass() > 0.0f)
    {
        B->velocity += impulse * B->getInverseMass();
    }
}

void CollisionResolve::SolveVelocityWithRoatation(Contact &contact)
{
    RigidBody *A = contact.a;
    RigidBody *B = contact.b;
    Vector2 normal = contact.normal;
    Vector2 contact1 = contact.contactPoints[0];
    Vector2 contact2 = contact.contactPoints[1];
    int contactCount = contact.contactCount;

    Vector2 contactList[2];
    contactList[0] = contact1;
    contactList[1] = contact2;

    Vector2 impulseList[2];
    Vector2 raList[2];
    Vector2 rbList[2];

    real restituion = std::min(A->restitution, B->restitution);

    for (int i = 0; i < contactCount; i++)
    {
        Vector2 ra = contactList[i] - A->position;
        Vector2 rb = contactList[i] - B->position;

        raList[i] = ra;
        rbList[i] = rb;

        Vector2 raPerp(-ra.y, ra.x);
        Vector2 rbPerp(-rb.y, rb.x);

        Vector2 angularLinearVelocityA = raPerp * A->rotation;
        Vector2 angularLinearVelocityB = rbPerp * B->rotation;

        Vector2 relativeVelocity = (B->velocity + angularLinearVelocityB) - (A->velocity + angularLinearVelocityA);

        real contactVelocityMag = relativeVelocity.scalarProduct(normal);

        if (contactVelocityMag > 0.0f)
        {
            continue;
        }

        real raPerpDotN = raPerp.scalarProduct(normal);
        real rbPerpDotN = rbPerp.scalarProduct(normal);

        real denom = A->inverseMass + B->inverseMass +
                     (raPerpDotN * raPerpDotN) * A->inverseInertia +
                     (rbPerpDotN * rbPerpDotN) * B->inverseInertia;

        real j = -(1.0f + restituion) * contactVelocityMag;
        j /= denom;
        j /= (real)contactCount;

        Vector2 impulse = normal * j;
        impulseList[i] = impulse;
    }

    for (int i = 0; i < contactCount; i++)
    {
        Vector2 impulse = impulseList[i];

        Vector2 ra = raList[i];
        Vector2 rb = rbList[i];

        A->velocity += (impulse * -1) * A->inverseMass;
        A->rotation += (ra.cross(impulse) * -1) * A->inverseInertia;
        B->velocity += impulse * B->inverseMass;
        B->rotation += (rb.cross(impulse)) * B->inverseInertia;
    }
}

void CollisionResolve::SolveVelocityWithRoatationAndFriction(Contact &contact)
{
    RigidBody *A = contact.a;
    RigidBody *B = contact.b;
    Vector2 normal = contact.normal;
    Vector2 contact1 = contact.contactPoints[0];
    Vector2 contact2 = contact.contactPoints[1];
    int contactCount = contact.contactCount;

    Vector2 contactList[2];
    contactList[0] = contact1;
    contactList[1] = contact2;

    Vector2 impulseList[2];
    Vector2 raList[2];
    Vector2 rbList[2];

    real jList[2] = {0.0f, 0.0f};

    Vector2 frictionImpulseList[2] = {Vector2(0, 0), Vector2(0, 0)};

    real sf = (A->staticFriction + B->staticFriction) * 0.5f;
    real df = (A->dynamicFriction + B->dynamicFriction) * 0.5f;

    real restituion = std::min(A->restitution, B->restitution);

    for (int i = 0; i < contactCount; i++)
    {
        Vector2 ra = contactList[i] - A->position;
        Vector2 rb = contactList[i] - B->position;

        raList[i] = ra;
        rbList[i] = rb;

        Vector2 raPerp(-ra.y, ra.x);
        Vector2 rbPerp(-rb.y, rb.x);

        Vector2 angularLinearVelocityA = raPerp * A->rotation;
        Vector2 angularLinearVelocityB = rbPerp * B->rotation;

        Vector2 relativeVelocity = (B->velocity + angularLinearVelocityB) - (A->velocity + angularLinearVelocityA);

        real contactVelocityMag = relativeVelocity.scalarProduct(normal);

        if (contactVelocityMag > 0.0f)
        {
            continue;
        }

        real raPerpDotN = raPerp.scalarProduct(normal);
        real rbPerpDotN = rbPerp.scalarProduct(normal);

        real denom = A->inverseMass + B->inverseMass +
                     (raPerpDotN * raPerpDotN) * A->inverseInertia +
                     (rbPerpDotN * rbPerpDotN) * B->inverseInertia;

        real j = -(1.0f + restituion) * contactVelocityMag;
        j /= denom;
        j /= (real)contactCount;

        jList[i] = j;

        Vector2 impulse = normal * j;
        impulseList[i] = impulse;
    }

    for (int i = 0; i < contactCount; i++)
    {
        Vector2 impulse = impulseList[i];

        Vector2 ra = raList[i];
        Vector2 rb = rbList[i];

        A->velocity += (impulse * -1) * A->inverseMass;
        A->rotation += (ra.cross(impulse) * -1) * A->inverseInertia;
        B->velocity += impulse * B->inverseMass;
        B->rotation += (rb.cross(impulse)) * B->inverseInertia;
    }

    for (int i = 0; i < contactCount; i++)
    {
        Vector2 ra = contactList[i] - A->position;
        Vector2 rb = contactList[i] - B->position;

        raList[i] = ra;
        rbList[i] = rb;

        Vector2 raPerp(-ra.y, ra.x);
        Vector2 rbPerp(-rb.y, rb.x);

        Vector2 angularLinearVelocityA = raPerp * A->rotation;
        Vector2 angularLinearVelocityB = rbPerp * B->rotation;

        Vector2 relativeVelocity = (B->velocity + angularLinearVelocityB) - (A->velocity + angularLinearVelocityA);

        Vector2 tangent = relativeVelocity - normal * relativeVelocity.scalarProduct(normal);
        if (Vector2::nearlyEqual(tangent, Vector2(0.0f, 0.0f)))
        {
            continue;
        }
        else
        {
            tangent.normalize();
        }

        real raPerpDotT = raPerp.scalarProduct(tangent);
        real rbPerpDotT = rbPerp.scalarProduct(tangent);

        real denom = A->inverseMass + B->inverseMass +
                     (raPerpDotT * raPerpDotT) * A->inverseInertia +
                     (rbPerpDotT * rbPerpDotT) * B->inverseInertia;

        real jt = -1 * relativeVelocity.scalarProduct(tangent);
        ;
        jt /= denom;
        jt /= (real)contactCount;

        real j = jList[i];

        Vector2 frictionImpulse;
        if (Vector2::abs(jt) <= j * sf)
        {
            frictionImpulse = tangent * jt;
        }
        else
        {
            frictionImpulse = tangent * -j * df;
        }

        frictionImpulseList[i] = frictionImpulse;
    }

    for (int i = 0; i < contactCount; i++)
    {
        Vector2 frictionImpulse = frictionImpulseList[i];

        Vector2 ra = raList[i];
        Vector2 rb = rbList[i];

        A->velocity += (frictionImpulse * -1) * A->inverseMass;
        A->rotation += (ra.cross(frictionImpulse) * -1) * A->inverseInertia;
        B->velocity += frictionImpulse * B->inverseMass;
        B->rotation += (rb.cross(frictionImpulse)) * B->inverseInertia;
    }
}

void CollisionResolve::Solve(Contact &contact, float dt)
{
    RigidBody *A = contact.a;
    RigidBody *B = contact.b;

    SolvePosition(contact);
    SolveVelocityWithRoatationAndFriction(contact);
}
