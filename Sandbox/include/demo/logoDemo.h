#pragma once
#include "demo.h"
#include <SDL3/SDL.h>

static const char *GLYPH_A[7] = {
    " XXX",
    "X   X",
    "X   X",
    "XXXXX",
    "X   X",
    "X   X",
    "X   X"};

static const char *GLYPH_C[7] = {
    " XXX",
    "X   X",
    "X    ",
    "X    ",
    "X    ",
    "X   X",
    " XXX "};

static const char *GLYPH_E[7] = {
    "XXXXX",
    "X    ",
    "X    ",
    "XXXX ",
    "X    ",
    "X    ",
    "XXXXX"};

static const char *GLYPH_L[7] = {
    "X    ",
    "X    ",
    "X    ",
    "X    ",
    "X    ",
    "X    ",
    "XXXXX"};

static const char *GLYPH_N[7] = {
    "X   X",
    "XX  X",
    "X X X",
    "X  XX",
    "X  XX",
    "X   X",
    "X   X"};

static const char *GLYPH_G[7] = {
    " XXX ",
    "X   X",
    "X    ",
    "X XXX",
    "X   X",
    "X   X",
    " XXX "};

static const char *GLYPH_I[7] = {
    "XXXXX",
    "  X  ",
    "  X  ",
    "  X  ",
    "  X  ",
    "  X  ",
    "XXXXX"};

static const char **GetGlyph(char ch)
{
    switch (ch)
    {
    case 'A':
        return GLYPH_A;
    case 'C':
        return GLYPH_C;
    case 'E':
        return GLYPH_E;
    case 'L':
        return GLYPH_L;
    case 'N':
        return GLYPH_N;
    case 'G':
        return GLYPH_G;
    case 'I':
        return GLYPH_I;
    default:
        return nullptr;
    }
}

static Color RandomColor1()
{
    return {
        float(rand() % 256),
        float(rand() % 256),
        float(rand() % 256),
        255.0f};
}

class LogoDemo : public Demo
{
    virtual const char *getName() const { return "Logo"; }

    virtual void init(World &world, std::vector<RigidBody *> &bodies,ForceRegistry& registry,
        ForceGenerator* gravity)
    {
        float cell = 16;
        float thick = 14;
        float spacing = cell;

        float baseY = 800;
        float startX = 200;
        SDL_Color white = {255, 255, 255, 255};

        stampWord(world, bodies, "ACCELENGINE", startX, baseY, cell, thick, spacing, white);
    }

    void addLetterCell(World &world, std::vector<RigidBody *> &bodies, float cx, float cy, float w, float h, SDL_Color col)
    {
        // use your getBody() so we can set flags before adding to world
        RigidBody *b = new RigidBody();
        b->enableCollision = true; // <- doesn't affect physics contacts
        b->restitution = 0.0f;
        b->angularDamping = 1.0f;
        b->velocity = {0, 0};
        b->rotation = 0.0f;
        b->inverseMass = 1.0f;
        b->position = Vector2(cx, cy);
        b->shapeType = ShapeType::AABB;
        b->aabb.halfSize = {w / 2, h / 2};
        b->c = RandomColor1();
        b->calculateDerivativeData();
        world.addBody(b);
        bodies.push_back(b);
    }


    void stampGlyph(World &world, std::vector<RigidBody *> &bodies, char ch, float ox, float oy, float cell, float thickness, SDL_Color col)
    {
        const char **g = GetGlyph(ch);
        if (!g)
            return;

        // center the “thickness” inside each cell
        float pad = 0.5f * (cell - thickness);
        float halfW = thickness * 0.5f;
        float halfH = thickness * 0.5f;

        // 5 columns × 7 rows
        for (int r = 0; r < 7; ++r)
        {
            for (int c = 0; c < 5; ++c)
            {
                char mark = g[r][c];
                if (mark != ' ' && mark != '\0')
                {
                    float x = ox + c * cell + pad + halfW;
                    float y = oy - r * cell - pad - halfH; // going “up” per row
                    addLetterCell(world, bodies, x, y, thickness, thickness, col);
                }
            }
        }
    }

    // Stamp a full word; returns total width used so you can chain if needed
    float stampWord(World &world, std::vector<RigidBody *> &bodies, const std::string &text, float startX, float baselineY,
                    float cell, float thickness, float letterSpacing, SDL_Color col)
    {
        float x = startX;
        for (char ch : text)
        {
            if (ch == ' ')
            {
                x += 3.0f * cell;
                continue;
            }
            stampGlyph(world, bodies, (char)std::toupper(ch), x, baselineY, cell, thickness, col);
            x += 5.0f * cell + letterSpacing;
        }
        return x - startX;
    }
};