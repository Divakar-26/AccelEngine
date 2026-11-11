#pragma once
#include <AccelEngine/core.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace AccelEngine
{
    constexpr int kMaxPolygonVertices = 8;

    // A convex piece of a body, in thce body's local frame. Every non-primitive shape (polygon, capsule,
    // compound) is a list of these. A part is a "rounded polygon": the convex hull of `count` vertices grown
    // by `radius`.
    //   count == 1  -> circle      (v[0] is the centre)
    //   count == 2  -> capsule     (v[0]..v[1] is the core segment)
    //   count >= 3  -> polygon     (counter-clockwise, convex; radius 0 for a sharp polygon)
    struct ConvexPart
    {
        int count = 0;
        Vector2 v[kMaxPolygonVertices];
        Vector2 n[kMaxPolygonVertices]; // outward normal of edge i (v[i] -> v[i+1])
        real radius = 0.0f;

        // derived by finalize()
        Vector2 center;       // bounding-circle centre (local)
        real boundRadius = 0; // bounding-circle radius, including `radius`
        Vector2 boundMin, boundMax; // local box, including `radius`

        void finalize()
        {
            for (int i = 0; i < count; i++)
            {
                const Vector2 e = v[(i + 1) % count] - v[i];
                n[i] = count == 1 ? Vector2(1, 0) : Vector2(e.y, -e.x).normalized();
            }
            boundMin = boundMax = v[0];
            for (int i = 1; i < count; i++)
            {
                boundMin = {std::min(boundMin.x, v[i].x), std::min(boundMin.y, v[i].y)};
                boundMax = {std::max(boundMax.x, v[i].x), std::max(boundMax.y, v[i].y)};
            }
            center = (boundMin + boundMax) * 0.5f;
            boundRadius = 0.0f;
            for (int i = 0; i < count; i++)
                boundRadius = std::max(boundRadius, (v[i] - center).magnitude());
            boundRadius += radius;
            boundMin = boundMin - Vector2(radius, radius);
            boundMax = boundMax + Vector2(radius, radius);
        }

        static ConvexPart circle(const Vector2 &c, real r)
        {
            ConvexPart p;
            p.count = 1;
            p.v[0] = c;
            p.radius = r;
            p.finalize();
            return p;
        }

        // Capsule: segment a..b grown by r.
        static ConvexPart capsule(const Vector2 &a, const Vector2 &b, real r)
        {
            ConvexPart p;
            if ((b - a).squareMagnitude() < 1e-8f)
                return circle(a, r);
            p.count = 2;
            p.v[0] = a;
            p.v[1] = b;
            p.radius = r;
            p.finalize();
            return p;
        }

        // Oriented box centred at c, rotated by `angle` (radians) inside the body.
        static ConvexPart box(const Vector2 &halfSize, const Vector2 &c = Vector2(), real angle = 0.0f)
        {
            ConvexPart p;
            p.count = 4;
            const Vector2 corner[4] = {{-halfSize.x, -halfSize.y}, {halfSize.x, -halfSize.y},
                                       {halfSize.x, halfSize.y}, {-halfSize.x, halfSize.y}};
            const real cs = std::cos(angle), sn = std::sin(angle);
            for (int i = 0; i < 4; i++)
                p.v[i] = Vector2(cs * corner[i].x - sn * corner[i].y, sn * corner[i].x + cs * corner[i].y) + c;
            p.finalize();
            return p;
        }

        // Convex hull of the points (any order), reduced to kMaxPolygonVertices. count == 0 if degenerate.
        static ConvexPart polygon(const Vector2 *points, int pointCount, real radius = 0.0f)
        {
            ConvexPart part;
            if (pointCount < 3)
                return part;
            std::vector<Vector2> pts(points, points + pointCount);
            std::sort(pts.begin(), pts.end(), [](const Vector2 &a, const Vector2 &b)
                      { return a.x < b.x || (a.x == b.x && a.y < b.y); });
            std::vector<Vector2> hull(2 * pts.size());
            int k = 0;
            auto turn = [](const Vector2 &o, const Vector2 &a, const Vector2 &b)
            { return (a - o).cross(b - o); };
            for (size_t i = 0; i < pts.size(); i++) // lower hull (counter-clockwise)
            {
                while (k >= 2 && turn(hull[k - 2], hull[k - 1], pts[i]) <= 1e-6f) k--;
                hull[k++] = pts[i];
            }
            for (size_t i = pts.size() - 1, t = k + 1; i > 0; i--) // upper hull
            {
                while (k >= (int)t && turn(hull[k - 2], hull[k - 1], pts[i - 1]) <= 1e-6f) k--;
                hull[k++] = pts[i - 1];
            }
            hull.resize(k - 1);
            // too many vertices: drop the ones that change the area least
            while ((int)hull.size() > kMaxPolygonVertices)
            {
                size_t drop = 0;
                real best = std::numeric_limits<real>::max();
                for (size_t i = 0; i < hull.size(); i++)
                {
                    const Vector2 &p0 = hull[(i + hull.size() - 1) % hull.size()];
                    const Vector2 &p1 = hull[i];
                    const Vector2 &p2 = hull[(i + 1) % hull.size()];
                    const real a = std::fabs(turn(p0, p1, p2));
                    if (a < best) { best = a; drop = i; }
                }
                hull.erase(hull.begin() + drop);
            }
            if (hull.size() < 3)
                return part;
            part.count = (int)hull.size();
            for (int i = 0; i < part.count; i++)
                part.v[i] = hull[i];
            part.radius = radius;
            part.finalize();
            return part;
        }

        // ---- mass properties (density 1): area and inertia about the local origin ----
        void massProperties(real &area, Vector2 &centroid, real &inertia) const
        {
            const real pi = 3.14159265358979f;
            if (count == 1)
            {
                area = pi * radius * radius;
                centroid = v[0];
                inertia = area * (0.5f * radius * radius + v[0].squareMagnitude());
                return;
            }
            if (count == 2)
            {
                const real len = (v[1] - v[0]).magnitude();
                const real rr = radius * radius;
                const real circleArea = pi * rr, boxArea = 2.0f * radius * len;
                area = circleArea + boxArea;
                centroid = (v[0] + v[1]) * 0.5f;
                const real lc = 4.0f * radius / (3.0f * pi), h = 0.5f * len;
                const real circleI = circleArea * (0.5f * rr + h * h + 2.0f * h * lc);
                const real boxI = boxArea * (4.0f * rr + len * len) / 12.0f;
                inertia = circleI + boxI + area * centroid.squareMagnitude();
                return;
            }
            area = 0.0f;
            real I0 = 0.0f; // about v[0]
            Vector2 c(0, 0);
            for (int i = 1; i + 1 < count; i++)
            {
                const Vector2 e1 = v[i] - v[0], e2 = v[i + 1] - v[0];
                const real D = e1.cross(e2);
                const real tri = 0.5f * D;
                area += tri;
                c += (e1 + e2) * (tri / 3.0f);
                I0 += (0.25f / 3.0f * D) * (e1.x * e1.x + e2.x * e1.x + e2.x * e2.x + e1.y * e1.y + e2.y * e1.y + e2.y * e2.y);
            }
            if (area < 1e-9f)
            {
                area = 0.0f;
                centroid = v[0];
                inertia = 0.0f;
                return;
            }
            const Vector2 cRel = c / area;
            centroid = v[0] + cRel;
            const real Ic = I0 - area * cRel.squareMagnitude();
            inertia = Ic + area * centroid.squareMagnitude();
        }
    };

    // A part transformed into world space (what the narrow phase works on).
    struct WorldPoly
    {
        int count = 0;
        Vector2 v[kMaxPolygonVertices];
        Vector2 n[kMaxPolygonVertices];
        real radius = 0.0f;
        Vector2 center;
        real boundRadius = 0.0f;
    };
}
