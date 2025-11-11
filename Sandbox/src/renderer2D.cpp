#include "renderer2D.h"
#include <AccelEngine/world.h>

using namespace AccelEngine;

SDL_Renderer *Renderer2D::renderer = nullptr;
SDL_Texture *Renderer2D::rectangle = nullptr;
SDL_Texture *Renderer2D::circle = nullptr;
int Renderer2D::WINDOW_W = 0;
int Renderer2D::WINDOW_H = 0;
Camera Renderer2D::camera;

static const float OUTLINE_THICKNESS = 2.0f;

void Renderer2D::init(SDL_Renderer *r, int w, int h)
{
    renderer = r;

    // asset loading
    rectangle = GetTexture("../Sandbox/assets/rectangle.png");
    circle = GetTexture("../Sandbox/assets/circle.png");

    if (!rectangle || !circle)
    {
        SDL_Log("Cannot load images for rectangle or cirlce ", SDL_GetError());
    }

    WINDOW_W = w;
    WINDOW_H = h;
}

SDL_Texture *Renderer2D::GetTexture(const char *path)
{
    SDL_Surface *s = IMG_Load(path);
    SDL_Texture *t = SDL_CreateTextureFromSurface(renderer, s);

    SDL_DestroySurface(s);

    return t;
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

    SDL_FPoint p = {sx, sy};
    return p;
}

SDL_FPoint Renderer2D::screenToWorld(float sx, float sy)
{
    float wy = WINDOW_H - sy;

    wy = wy / camera.zoom + camera.y;
    float wx = sx / camera.zoom + camera.x;

    SDL_FPoint p = {wx, wy};
    return p;
}

void Renderer2D::DrawRectangle(float x, float y, float w, float h, float orientation, SDL_Color fill)
{
    float degrees = -orientation * (180.0f / 3.14159f);

    SDL_FPoint center = worldToScreen(x, y);

    float scaledW = w * camera.zoom;
    float scaledH = h * camera.zoom;

    float sx = center.x - scaledW / 2;
    float sy = center.y - scaledH / 2;

    float inset = OUTLINE_THICKNESS;

    float innerW = scaledW - inset * 2;
    float innerH = scaledH - inset * 2;

    // Rotation pivot (pivot is always half rect size)
    SDL_FPoint outerPivot = {scaledW / 2, scaledH / 2};
    SDL_FPoint innerPivot = {innerW / 2, innerH / 2};

    // Outer rect
    SDL_FRect outerRect = {sx, sy, scaledW, scaledH};
    SDL_SetTextureColorMod(rectangle, 255, 255, 255);
    SDL_RenderTextureRotated(renderer, rectangle, nullptr, &outerRect,
                             degrees, &outerPivot, SDL_FLIP_NONE);

    // Inner rect (filled body)
    SDL_FRect innerRect = {sx + inset, sy + inset, innerW, innerH};
    SDL_SetTextureColorMod(rectangle, fill.r, fill.g, fill.b);
    SDL_RenderTextureRotated(renderer, rectangle, nullptr, &innerRect,
                             degrees, &innerPivot, SDL_FLIP_NONE);
}

void Renderer2D::DrawCircle(float x, float y, float radius, float orientation, SDL_Color outline)
{
    SDL_FPoint center = worldToScreen(x, y);
    float scaledRadius = radius * camera.zoom;

    SDL_FPoint pivot = {scaledRadius, scaledRadius};

    float degrees = -orientation * (180.0f / 3.14159f);

    // Outer white outline
    SDL_FRect outlineDst = {
        center.x - scaledRadius - OUTLINE_THICKNESS,
        center.y - scaledRadius - OUTLINE_THICKNESS,
        (scaledRadius * 2) + OUTLINE_THICKNESS * 2,
        (scaledRadius * 2) + OUTLINE_THICKNESS * 2};

    SDL_SetTextureColorMod(circle, 255, 255, 255);
    SDL_RenderTextureRotated(renderer, circle, nullptr, &outlineDst, 0, &pivot, SDL_FLIP_NONE);

    // Inner colored circle
    SDL_FRect dst = {
        center.x - scaledRadius,
        center.y - scaledRadius,
        scaledRadius * 2,
        scaledRadius * 2};

    SDL_SetTextureColorMod(circle, outline.r, outline.g, outline.b);
    SDL_RenderTextureRotated(renderer, circle, nullptr, &dst, degrees, &pivot, SDL_FLIP_NONE);
}

void Renderer2D::drawSprings(ForceRegistry &registry)
{
    const auto &springs = registry.getSprings();

    for (Spring *s : springs)
    {
        RigidBody *a = nullptr;
        RigidBody *b = s->other;

        // Find body 'a'
        for (auto &reg : registry.registrations)
        {
            if (reg.fg == s)
            {
                a = reg.body;
                break;
            }
        }

        if (!a || !b)
            continue;

        // World positions of endpoints
        Vector2 p1 = a->getPointInWorldSpace(s->localA);
        Vector2 p2 = b->getPointInWorldSpace(s->localB);

        // Convert to screen coordinates
        SDL_FPoint sp1 = worldToScreen(p1.x, p1.y);
        SDL_FPoint sp2 = worldToScreen(p2.x, p2.y);

        // Draw spring line
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderLine(renderer, sp1.x, sp1.y, sp2.x, sp2.y);

        // Draw endpoints
        SDL_FRect c1 = {sp1.x - 4, sp1.y - 4, 8, 8};
        SDL_FRect c2 = {sp2.x - 4, sp2.y - 4, 8, 8};

        SDL_RenderFillRect(renderer, &c1);
        SDL_RenderFillRect(renderer, &c2);
    }
}

void Renderer2D::drawJoints(const std::vector<Joint *> &joints)
{
    SDL_Color jointColor = {255, 255, 0, 255};

    for (auto *j : joints)
    {
        DistanceJoint *dj = dynamic_cast<DistanceJoint *>(j);
        if (!dj)
            continue;

        Vector2 p1 = dj->A->getPointInWorldSpace(dj->localA);
        Vector2 p2 = dj->B->getPointInWorldSpace(dj->localB);

        DrawJoint(p1, p2, jointColor);
    }
}

void Renderer2D::DrawJoint(const Vector2 &p1, const Vector2 &p2, SDL_Color color)
{
    SDL_FPoint sp1 = worldToScreen(p1.x, p1.y);
    SDL_FPoint sp2 = worldToScreen(p2.x, p2.y);

    // distance in world
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;

    float length = sqrtf(dx * dx + dy * dy);
    float angle = atan2f(dy, dx);

    // convert to screen-space center
    SDL_FPoint center = {
        (sp1.x + sp2.x) * 0.5f,
        (sp1.y + sp2.y) * 0.5f};

    // convert length to screen space
    float w = length * camera.zoom;
    float h = 6.0f * camera.zoom; // joint thickness

    // convert to top-left:
    float sx = center.x - w / 2;
    float sy = center.y - h / 2;

    SDL_FRect rect = {sx, sy, w, h};

    float degrees = -angle * (180.0f / 3.14159f);

    SDL_SetTextureColorMod(rectangle, color.r, color.g, color.b);

    SDL_FPoint pivot = {w / 2, h / 2};

    SDL_RenderTextureRotated(renderer, rectangle, nullptr, &rect,
                             degrees, &pivot, SDL_FLIP_NONE);
}

void Renderer2D::DrawAABBOutline(float minX, float minY, float maxX, float maxY, SDL_Color color)
{
    SDL_FPoint p1 = worldToScreen(minX, minY);
    SDL_FPoint p2 = worldToScreen(maxX, minY);
    SDL_FPoint p3 = worldToScreen(maxX, maxY);
    SDL_FPoint p4 = worldToScreen(minX, maxY);

    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

    SDL_RenderLine(renderer, p1.x, p1.y, p2.x, p2.y);
    SDL_RenderLine(renderer, p2.x, p2.y, p3.x, p3.y);
    SDL_RenderLine(renderer, p3.x, p3.y, p4.x, p4.y);
    SDL_RenderLine(renderer, p4.x, p4.y, p1.x, p1.y);
}
