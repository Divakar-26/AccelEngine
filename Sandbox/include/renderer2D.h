#pragma once

#include <SDL3/SDL.h>
#include <AccelEngine/ForceRegistry.h>
#include <AccelEngine/world.h>
#include <string>
#include <vector>

struct Camera
{
    float x, y;
    float zoom = 1.0f;
    // right now now rotation
};

using namespace AccelEngine;

// Thin layer between the Sandbox and SIMPEngine's batched GL renderer (SIMPEngine::Renderer).
// World space is the physics space: pixels, y up. `camera.x/y` is the world point at the bottom-left corner of the
// window and `camera.zoom` the pixels per world unit.
class Renderer2D
{
public:
    struct Style
    {
        bool shadows = true;     // soft drop shadow under bodies
        bool shading = true;     // highlight + rotation notch on bodies
        bool grid = true;        // background grid
        bool jointDetails = true; // springs, limits, suspension...
    };
    static Style style;

    static void init(int w, int h);
    static void resize(int w, int h);

    // Frame: beginFrame() clears and sets the camera, endFrame() flushes the batch (call before ImGui draws).
    static void beginFrame(const Camera &c);
    static void endFrame();

    // ---- bodies ----
    // Draws any body (circle, box, polygon, capsule, compound, sensors, one-way platforms).
    static void DrawBody(const RigidBody &body, SDL_Color fill);
    static void DrawRectangle(float x, float y, float w, float h, float orientation, SDL_Color fill);
    static void DrawCircle(float x, float y, float radius, float orientation, SDL_Color fill);
    static void DrawPolygon(const Vector2 *worldVerts, int count, SDL_Color fill, float z = 0.0f);
    static void DrawCapsule(const Vector2 &a, const Vector2 &b, float radius, SDL_Color fill);

    // ---- overlays ----
    static void DrawAABBOutline(float minX, float minY, float maxX, float maxY, SDL_Color color);
    static void DrawLine(const Vector2 &a, const Vector2 &b, SDL_Color color, float widthPx = 1.5f);
    static void DrawDot(const Vector2 &p, float radiusPx, SDL_Color color);
    static void DrawText(const std::string &text, const Vector2 &worldPos, float sizePx, SDL_Color color);
    static void DrawContacts(const std::vector<Contact> &contacts);

    static void drawSprings(ForceRegistry &registry);
    static void drawJoints(const std::vector<Joint *> &joints);
    static void DrawJoint(const Vector2 &p1, const Vector2 &p2, SDL_Color color);

    static int WINDOW_W, WINDOW_H;
    static Camera camera;
    static void setCamera(const Camera &c);
    static SDL_FPoint worldToScreen(float x, float y);
    static SDL_FPoint screenToWorld(float sx, float sy);
};
