#include "renderer2D.h"

#include <Engine/Rendering/Renderer.h>

#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

using namespace AccelEngine;

int Renderer2D::WINDOW_W = 0;
int Renderer2D::WINDOW_H = 0;
Camera Renderer2D::camera;
Renderer2D::Style Renderer2D::style;

namespace
{
    using SIMPEngine::Renderer;

    // z layers (the renderer sorts by z)
    constexpr float kZBackground = -50.0f;
    constexpr float kZGrid = -40.0f;
    constexpr float kZShadow = -20.0f;
    constexpr float kZOutline = 0.0f;
    constexpr float kZFill = 0.1f;
    constexpr float kZDetail = 0.2f;
    constexpr float kZJoint = 5.0f;
    constexpr float kZOverlay = 10.0f;
    constexpr float kZText = 20.0f;

    constexpr float kRad2Deg = 57.29577951308232f;

    inline SDL_Color rgba(int r, int g, int b, int a = 255)
    {
        return {(Uint8)std::clamp(r, 0, 255), (Uint8)std::clamp(g, 0, 255), (Uint8)std::clamp(b, 0, 255), (Uint8)std::clamp(a, 0, 255)};
    }

    inline SDL_Color lighten(SDL_Color c, float t)
    {
        return rgba((int)(c.r + (255 - c.r) * t), (int)(c.g + (255 - c.g) * t), (int)(c.b + (255 - c.b) * t), c.a);
    }

    inline SDL_Color darken(SDL_Color c, float t)
    {
        return rgba((int)(c.r * (1.0f - t)), (int)(c.g * (1.0f - t)), (int)(c.b * (1.0f - t)), c.a);
    }

    inline glm::vec4 toVec4(SDL_Color c) { return {c.r / 255.0f, c.g / 255.0f, c.b / 255.0f, c.a / 255.0f}; }

    // SIMPEngine's DrawCapsuleLine takes the HALF width (the end-cap radius); everything here passes the full width.
    inline void capsuleLine(float x1, float y1, float x2, float y2, float fullWidth, SDL_Color c, float aa, float z)
    {
        Renderer::DrawCapsuleLine(x1, y1, x2, y2, fullWidth * 0.5f, c, aa, z);
    }

    // one screen pixel in world units
    inline float px() { return 1.0f / std::max(Renderer2D::camera.zoom, 1e-4f); }

    inline Vector2 rot(const Matrix2 &m, const Vector2 &v) { return m * v; }

    // Filled convex polygon with a lighter centre (triangle fan with per-vertex colours).
    void fillFan(const Vector2 *v, int n, SDL_Color edgeColor, SDL_Color centerColor, float z)
    {
        if (n < 3)
            return;
        glm::vec2 pos[kMaxPolygonVertices + 1];
        glm::vec4 col[kMaxPolygonVertices + 1];
        uint32_t idx[kMaxPolygonVertices * 3];
        Vector2 c(0, 0);
        for (int i = 0; i < n; i++)
            c += v[i];
        c = c * (1.0f / (float)n);
        pos[0] = {c.x, c.y};
        col[0] = toVec4(centerColor);
        for (int i = 0; i < n; i++)
        {
            pos[i + 1] = {v[i].x, v[i].y};
            col[i + 1] = toVec4(edgeColor);
            idx[i * 3 + 0] = 0;
            idx[i * 3 + 1] = i + 1;
            idx[i * 3 + 2] = (i + 1) % n + 1;
        }
        Renderer::DrawTriangles(pos, col, n + 1, idx, n * 3, z);
    }

    void outlinePoly(const Vector2 *v, int n, SDL_Color color, float widthWorld, float z)
    {
        for (int i = 0; i < n; i++)
        {
            const Vector2 &a = v[i], &b = v[(i + 1) % n];
            capsuleLine(a.x, a.y, b.x, b.y, widthWorld, color, 1.0f, z);
        }
    }

    // A part of a body in world space (circle parts are drawn as discs, 2-vertex parts as capsules).
    struct PartPass
    {
        static void outline(const WorldPoly &p, SDL_Color color, float grow, float z)
        {
            if (p.count == 1)
                Renderer::DrawCircle(p.v[0].x, p.v[0].y, p.radius + grow, color, 1.0f, z);
            else if (p.count == 2)
                capsuleLine(p.v[0].x, p.v[0].y, p.v[1].x, p.v[1].y, 2.0f * (p.radius + grow), color, 1.0f, z);
            else
            {
                if (p.radius > 0.0f || grow > 0.0f)
                {
                    // rounded / grown polygon: stroke the edges with round caps
                    outlinePoly(p.v, p.count, color, 2.0f * (p.radius + grow), z);
                }
                fillFan(p.v, p.count, color, color, z);
            }
        }

        static void fill(const WorldPoly &p, SDL_Color color, bool shading, float z)
        {
            if (p.count == 1)
            {
                Renderer::DrawCircle(p.v[0].x, p.v[0].y, p.radius, color, 1.0f, z);
                if (shading && p.radius > 4.0f * px())
                    Renderer::DrawCircle(p.v[0].x - p.radius * 0.28f, p.v[0].y + p.radius * 0.28f, p.radius * 0.42f,
                                         rgba(255, 255, 255, 46), 1.0f, z + 0.01f);
            }
            else if (p.count == 2)
            {
                capsuleLine(p.v[0].x, p.v[0].y, p.v[1].x, p.v[1].y, 2.0f * p.radius, color, 1.0f, z);
                if (shading)
                {
                    const Vector2 n = p.n[0] * -1.0f; // the side facing "up" for a horizontal capsule is +y; pick the +y-ish normal
                    const Vector2 up = p.n[0].y >= 0.0f ? p.n[0] : n;
                    const float o = p.radius * 0.45f;
                    capsuleLine(p.v[0].x + up.x * o, p.v[0].y + up.y * o, p.v[1].x + up.x * o, p.v[1].y + up.y * o,
                                              p.radius * 0.5f, rgba(255, 255, 255, 40), 1.0f, z + 0.01f);
                }
            }
            else
            {
                if (p.radius > 0.0f)
                    outlinePoly(p.v, p.count, color, 2.0f * p.radius, z);
                fillFan(p.v, p.count, color, shading ? lighten(color, 0.16f) : color, z);
            }
        }
    };

    void bodyColors(const RigidBody &b, SDL_Color fill, SDL_Color &outline)
    {
        outline = rgba(20, 22, 28, fill.a);
    }
}

void Renderer2D::init(int w, int h)
{
    WINDOW_W = w;
    WINDOW_H = h;
}

void Renderer2D::resize(int w, int h)
{
    WINDOW_W = w;
    WINDOW_H = h;
}

void Renderer2D::setCamera(const Camera &c)
{
    camera = c;
}

SDL_FPoint Renderer2D::worldToScreen(float x, float y)
{
    float sx = (x - camera.x) * camera.zoom;
    float sy = (y - camera.y) * camera.zoom;
    sy = WINDOW_H - sy;
    return {sx, sy};
}

SDL_FPoint Renderer2D::screenToWorld(float sx, float sy)
{
    float wy = WINDOW_H - sy;
    wy = wy / camera.zoom + camera.y;
    float wx = sx / camera.zoom + camera.x;
    return {wx, wy};
}

void Renderer2D::beginFrame(const Camera &c)
{
    camera = c;
    const float vw = WINDOW_W / camera.zoom, vh = WINDOW_H / camera.zoom;
    const glm::mat4 proj = glm::ortho(camera.x, camera.x + vw, camera.y, camera.y + vh, -1000.0f, 1000.0f);
    Renderer::SetClearColor(0.07f, 0.09f, 0.13f, 1.0f);
    Renderer::Clear();
    Renderer::BeginScene(glm::mat4(1.0f), proj);

    // vertical sky gradient behind everything
    const glm::vec2 pos[4] = {{camera.x, camera.y}, {camera.x + vw, camera.y}, {camera.x + vw, camera.y + vh}, {camera.x, camera.y + vh}};
    const glm::vec4 bottom(0.14f, 0.17f, 0.24f, 1.0f), top(0.32f, 0.40f, 0.52f, 1.0f);
    const glm::vec4 col[4] = {bottom, bottom, top, top};
    const uint32_t idx[6] = {0, 1, 2, 0, 2, 3};
    Renderer::DrawTriangles(pos, col, 4, idx, 6, kZBackground);

    if (style.grid)
    {
        const float minorPx = 50.0f;
        float step = minorPx;
        while (step * camera.zoom < 28.0f) step *= 2.0f;
        const float x0 = std::floor(camera.x / step) * step, y0 = std::floor(camera.y / step) * step;
        for (float x = x0; x <= camera.x + vw; x += step)
        {
            const bool major = std::fmod(std::fabs(x), step * 5.0f) < 0.5f;
            Renderer::DrawLine(x, camera.y, x, camera.y + vh, rgba(255, 255, 255, major ? 26 : 12), kZGrid);
        }
        for (float y = y0; y <= camera.y + vh; y += step)
        {
            const bool major = std::fmod(std::fabs(y), step * 5.0f) < 0.5f;
            Renderer::DrawLine(camera.x, y, camera.x + vw, y, rgba(255, 255, 255, major ? 26 : 12), kZGrid);
        }
    }
}

void Renderer2D::endFrame()
{
    Renderer::Flush();
}

// ---------------------------------------------------------------------------------------------------------------
// Bodies
// ---------------------------------------------------------------------------------------------------------------

void Renderer2D::DrawBody(const RigidBody &b, SDL_Color fill)
{
    // view culling by bounding box
    {
        const Vector2 c = (b.worldAABBMin + b.worldAABBMax) * 0.5f;
        const Vector2 h = (b.worldAABBMax - b.worldAABBMin) * 0.5f;
        if (!Renderer::IsVisible({c.x, c.y}, {h.x + 4.0f * px(), h.y + 4.0f * px()}))
            return;
    }

    SDL_Color outline;
    bodyColors(b, fill, outline);
    const float o = 1.6f * px(); // outline thickness in world units
    const bool fancy = style.shading;

    if (b.isSensor)
    {
        // translucent zone with a dashed-looking outline: sensors are not solid
        fill.a = 70;
        outline = lighten(fill, 0.4f);
        outline.a = 200;
    }

    const int n = b.worldPartCount();
    WorldPoly parts[16];
    const int shown = std::min(n, 16);
    for (int i = 0; i < shown; i++)
        b.getWorldPart(i, parts[i]);

    // shadow
    if (style.shadows && !b.isSensor && !b.isStatic())
    {
        const float so = 4.0f * px();
        for (int i = 0; i < shown; i++)
        {
            WorldPoly s = parts[i];
            for (int k = 0; k < s.count; k++) { s.v[k].x += so; s.v[k].y -= so; }
            PartPass::outline(s, rgba(0, 0, 0, 55), 0.0f, kZShadow);
        }
    }

    // all outlines first, then all fills: the seams between the parts of a compound disappear
    for (int i = 0; i < shown; i++)
        PartPass::outline(parts[i], outline, o, kZOutline);
    for (int i = 0; i < shown; i++)
        PartPass::fill(parts[i], fill, fancy && !b.isSensor && !b.isStatic(), kZFill);

    // rotation notch on circles so spinning is visible
    if (b.shapeType == ShapeType::CIRCLE && fancy && b.circle.radius > 5.0f * px())
    {
        const float c = std::cos(b.orientation), s = std::sin(b.orientation);
        const float r = b.circle.radius;
        capsuleLine(b.position.x + c * r * 0.18f, b.position.y + s * r * 0.18f,
                                  b.position.x + c * r * 0.82f, b.position.y + s * r * 0.82f,
                                  std::max(1.5f * px(), r * 0.09f), darken(fill, 0.45f), 1.0f, kZDetail);
    }

    // one-way platform: bright rim on the solid-from side with small chevrons
    if (b.oneWay)
    {
        const Vector2 up = b.transformMatrix * b.oneWayNormal;
        for (int i = 0; i < shown; i++)
        {
            const WorldPoly &p = parts[i];
            if (p.count < 3)
                continue;
            for (int k = 0; k < p.count; k++)
            {
                if (p.n[k].scalarProduct(up) > 0.9f)
                {
                    const Vector2 a = p.v[k], c = p.v[(k + 1) % p.count];
                    capsuleLine(a.x, a.y, c.x, c.y, 3.0f * px(), rgba(255, 244, 160, 230), 1.0f, kZDetail);
                    const float len = (c - a).magnitude();
                    const Vector2 d = (c - a) / std::max(len, 1e-3f);
                    const float spacing = 26.0f * px();
                    for (float t = spacing * 0.5f; t < len; t += spacing)
                    {
                        const Vector2 m = a + d * t;
                        const Vector2 tip = m + up * (6.0f * px());
                        Renderer::DrawLine(m.x - d.x * 5 * px(), m.y - d.y * 5 * px(), tip.x, tip.y, rgba(255, 244, 160, 200), kZDetail);
                        Renderer::DrawLine(m.x + d.x * 5 * px(), m.y + d.y * 5 * px(), tip.x, tip.y, rgba(255, 244, 160, 200), kZDetail);
                    }
                }
            }
        }
    }
}

void Renderer2D::DrawRectangle(float x, float y, float w, float h, float orientation, SDL_Color fill)
{
    const float deg = orientation * kRad2Deg;
    const float o = 1.6f * px();
    Renderer::DrawQuad(x, y, w + 2 * o, h + 2 * o, deg, rgba(20, 22, 28, fill.a), true, kZOutline);
    Renderer::DrawQuad(x, y, w, h, deg, fill, true, kZFill);
}

void Renderer2D::DrawCircle(float x, float y, float radius, float orientation, SDL_Color fill)
{
    const float o = 1.6f * px();
    Renderer::DrawCircle(x, y, radius + o, rgba(20, 22, 28, fill.a), 1.0f, kZOutline);
    Renderer::DrawCircle(x, y, radius, fill, 1.0f, kZFill);
    (void)orientation;
}

void Renderer2D::DrawPolygon(const Vector2 *v, int count, SDL_Color fill, float z)
{
    fillFan(v, count, fill, fill, z);
}

void Renderer2D::DrawCapsule(const Vector2 &a, const Vector2 &b, float radius, SDL_Color fill)
{
    const float o = 1.6f * px();
    capsuleLine(a.x, a.y, b.x, b.y, 2 * (radius + o), rgba(20, 22, 28, fill.a), 1.0f, kZOutline);
    capsuleLine(a.x, a.y, b.x, b.y, 2 * radius, fill, 1.0f, kZFill);
}

// ---------------------------------------------------------------------------------------------------------------
// Overlays
// ---------------------------------------------------------------------------------------------------------------

void Renderer2D::DrawAABBOutline(float minX, float minY, float maxX, float maxY, SDL_Color color)
{
    Renderer::DrawRect((minX + maxX) * 0.5f, (minY + maxY) * 0.5f, maxX - minX, maxY - minY, px(), color, 0.0f, kZOverlay);
}

void Renderer2D::DrawLine(const Vector2 &a, const Vector2 &b, SDL_Color color, float widthPx)
{
    capsuleLine(a.x, a.y, b.x, b.y, widthPx * px(), color, 1.0f, kZOverlay);
}

void Renderer2D::DrawDot(const Vector2 &p, float radiusPx, SDL_Color color)
{
    Renderer::DrawCircle(p.x, p.y, radiusPx * px(), color, 1.0f, kZOverlay);
}

void Renderer2D::DrawText(const std::string &text, const Vector2 &p, float sizePx, SDL_Color color)
{
    Renderer::DrawText(text, p.x, p.y, sizePx * px(), color, SIMPEngine::TextAlign::Center, 0.0f, kZText);
}

void Renderer2D::DrawContacts(const std::vector<Contact> &contacts)
{
    for (const Contact &c : contacts)
        for (int i = 0; i < c.contactCount && i < 2; i++)
        {
            DrawDot(c.contactPoints[i], 3.0f, rgba(255, 70, 70));
            DrawLine(c.contactPoints[i], c.contactPoints[i] + c.normal * (14.0f * px()), rgba(255, 220, 90), 1.5f);
        }
}

// ---------------------------------------------------------------------------------------------------------------
// Springs and joints
// ---------------------------------------------------------------------------------------------------------------

namespace
{
    void zigzag(const Vector2 &a, const Vector2 &b, int coils, float amplitude, SDL_Color color, float widthWorld, float z)
    {
        const Vector2 d = b - a;
        const float len = d.magnitude();
        if (len < 1e-3f)
            return;
        const Vector2 t = d / len, nrm(-t.y, t.x);
        const float lead = std::min(len * 0.15f, 12.0f * (amplitude / 6.0f));
        Vector2 prev = a;
        Vector2 p = a + t * lead;
        capsuleLine(prev.x, prev.y, p.x, p.y, widthWorld, color, 1.0f, z);
        prev = p;
        const float span = len - 2 * lead;
        const int segs = coils * 2;
        for (int i = 1; i <= segs; i++)
        {
            const float s = (float)i / (float)segs;
            const float side = (i % 2) ? 1.0f : -1.0f;
            Vector2 q = a + t * (lead + span * s) + nrm * (side * amplitude * (i == segs ? 0.0f : 1.0f));
            capsuleLine(prev.x, prev.y, q.x, q.y, widthWorld, color, 1.0f, z);
            prev = q;
        }
        capsuleLine(prev.x, prev.y, b.x, b.y, widthWorld, color, 1.0f, z);
    }
}

void Renderer2D::drawSprings(ForceRegistry &registry)
{
    for (auto &reg : registry.registrations)
    {
        Spring *s = dynamic_cast<Spring *>(reg.fg);
        if (!s || !reg.body || !s->other)
            continue;

        const Vector2 p1 = reg.body->getPointInWorldSpace(s->localA);
        const Vector2 p2 = s->other->getPointInWorldSpace(s->localB);
        if (style.jointDetails)
            zigzag(p1, p2, 8, 6.0f * px(), rgba(235, 235, 240, 230), 1.6f * px(), kZJoint);
        else
            Renderer::DrawLine(p1.x, p1.y, p2.x, p2.y, rgba(235, 235, 240, 230), kZJoint);
        DrawDot(p1, 3.0f, rgba(30, 30, 36));
        DrawDot(p2, 3.0f, rgba(30, 30, 36));
    }
}

void Renderer2D::drawJoints(const std::vector<Joint *> &joints)
{
    const SDL_Color rod = rgba(250, 214, 80);
    const SDL_Color pinDark = rgba(30, 32, 40), pinLight = rgba(235, 235, 240);

    for (const Joint *j : joints)
    {
        if (const auto *dj = dynamic_cast<const DistanceJoint *>(j))
        {
            DrawJoint(dj->A->getPointInWorldSpace(dj->localA), dj->B->getPointInWorldSpace(dj->localB), rod);
        }
        else if (const auto *rj = dynamic_cast<const RevoluteJoint *>(j))
        {
            const Vector2 p = rj->getAnchorA();
            if (style.jointDetails && rj->enableLimit)
            {
                // wedge showing the allowed range, drawn from body A's frame
                const float base = rj->A->orientation;
                const float r = 24.0f * px();
                Vector2 prev = p + Vector2(std::cos(base + rj->lowerAngle), std::sin(base + rj->lowerAngle)) * r;
                Renderer::DrawLine(p.x, p.y, prev.x, prev.y, rgba(255, 120, 90, 200), kZJoint);
                for (int i = 1; i <= 12; i++)
                {
                    const float a = base + rj->lowerAngle + (rj->upperAngle - rj->lowerAngle) * (float)i / 12.0f;
                    const Vector2 q = p + Vector2(std::cos(a), std::sin(a)) * r;
                    Renderer::DrawLine(prev.x, prev.y, q.x, q.y, rgba(255, 120, 90, 200), kZJoint);
                    prev = q;
                }
                Renderer::DrawLine(p.x, p.y, prev.x, prev.y, rgba(255, 120, 90, 200), kZJoint);
            }
            Renderer::DrawCircle(p.x, p.y, 6.5f * px(), pinDark, 1.0f, kZJoint);
            Renderer::DrawCircle(p.x, p.y, 3.8f * px(), rj->enableMotor ? rgba(110, 220, 130) : pinLight, 1.0f, kZJoint + 0.1f);
        }
        else if (const auto *wj = dynamic_cast<const WheelJoint *>(j))
        {
            const Vector2 pa = wj->A->getPointInWorldSpace(wj->getLocalAnchorA());
            const Vector2 pb = wj->B->getPointInWorldSpace(wj->getLocalAnchorB());
            if (style.jointDetails)
                zigzag(pa, pb, 5, 5.0f * px(), rgba(190, 200, 215, 240), 1.8f * px(), kZJoint);
            else
                Renderer::DrawLine(pa.x, pa.y, pb.x, pb.y, rgba(190, 200, 215, 240), kZJoint);
            Renderer::DrawCircle(pb.x, pb.y, 4.5f * px(), pinDark, 1.0f, kZJoint);
            Renderer::DrawCircle(pb.x, pb.y, 2.5f * px(), wj->enableMotor ? rgba(110, 220, 130) : pinLight, 1.0f, kZJoint + 0.1f);
        }
        else if (const auto *wd = dynamic_cast<const WeldJoint *>(j))
        {
            const Vector2 p = wd->B->position;
            const float s = 4.0f * px();
            Renderer::DrawLine(p.x - s, p.y - s, p.x + s, p.y + s, pinLight, kZJoint);
            Renderer::DrawLine(p.x - s, p.y + s, p.x + s, p.y - s, pinLight, kZJoint);
        }
    }
}

void Renderer2D::DrawJoint(const Vector2 &p1, const Vector2 &p2, SDL_Color color)
{
    capsuleLine(p1.x, p1.y, p2.x, p2.y, 5.0f * px(), rgba(20, 22, 28), 1.0f, kZJoint);
    capsuleLine(p1.x, p1.y, p2.x, p2.y, 3.0f * px(), color, 1.0f, kZJoint + 0.1f);
    Renderer::DrawCircle(p1.x, p1.y, 3.2f * px(), rgba(20, 22, 28), 1.0f, kZJoint + 0.2f);
    Renderer::DrawCircle(p2.x, p2.y, 3.2f * px(), rgba(20, 22, 28), 1.0f, kZJoint + 0.2f);
}
