#pragma once
#include "showcase.h"

// Beads pour from a funnel through a lattice of pegs into bins. Each peg sends a bead left or right
// at random, so the bins fill into a bell curve; the overlay shows the matching normal distribution.
class GaltonDemo : public ShowcaseDemo
{
public:
    const char *getName() const override { return "Galton Board"; }

    void drawImGui() override
    {
        ImGui::Begin("Galton Board", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Beads: %d / %d", released, totalBeads);
        ImGui::SliderInt("Total beads", &totalBeads, 200, 4000);
        ImGui::SliderInt("Beads per update", &rate, 1, 4);
        ImGui::SliderFloat("Air drag (velocity kept/s)", &beadDamping, 0.002f, 1.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
        ImGui::Checkbox("Pour", &pouring);
        ImGui::SameLine();
        ImGui::Checkbox("Show bell curve", &showCurve);
        if (ImGui::Button("Restart"))
            requestReset();
        tip("Every bead takes 14 random left/right bounces, so bin counts follow a binomial (bell) distribution. "
            "Turn the air drag down (towards 1.0) and the beads skid down the lattice's diagonals instead: the bell flattens.");
        ImGui::End();

        if (showCurve)
            drawBellCurve();
    }

    // bin heights (top of the highest bead in each bin, minus the floor)
    std::vector<float> binHeights() const
    {
        std::vector<float> h(bins, 0.0f);
        for (RigidBody *b : beads)
        {
            if (b->position.y > binTop)
                continue;
            int k = (int)std::floor((b->position.x - binLeft) / binWidth);
            if (k >= 0 && k < bins)
                h[k] = std::max(h[k], b->position.y + b->circle.radius - floorY);
        }
        return h;
    }

    int releasedCount() const { return released; }

protected:
    void build() override
    {
        released = 0;
        beads.clear();
        const Color wood = rgb(95, 70, 55);
        const Color brass = rgb(230, 190, 90);

        // floor
        box({960, 40}, {930, 30}, 0.0f)->c = wood;

        // bins
        binLeft = 960.0f - bins * binWidth * 0.5f;
        for (int k = 0; k <= bins; k++)
        {
            RigidBody *d = box({binLeft + k * binWidth, floorY + (binTop - floorY) * 0.5f}, {1.5f, (binTop - floorY) * 0.5f}, 0.0f);
            d->c = wood;
        }

        // classic triangular peg lattice: row i has i+1 pegs, so every peg is on some bead's path.
        // After pegRows rows a bead is at x = (m - pegRows/2) * binWidth, the centre of a bin.
        const float wallX = bins * binWidth * 0.5f + 6.0f;
        for (int row = 0; row < pegRows; row++)
        {
            const float y = pegTop - row * pegDy;
            for (int j = 0; j <= row; j++)
            {
                RigidBody *peg = ball({960 + (j - row * 0.5f) * binWidth, y}, pegRadius, 0.0f);
                peg->c = brass;
                peg->restitution = 0.0f; // dead pegs: a bead drops one column per row instead of skipping
            }
        }
        segment({960 - wallX, floorY}, {960 - wallX, binTop + 40}, 4)->c = wood;
        segment({960 + wallX, floorY}, {960 + wallX, binTop + 40}, 4)->c = wood;

        // slanted walls hugging the triangle (parallel to its outer pegs, one column outside the
        // outermost bead path), so beads cannot roll down the outside of the lattice
        for (int side = -1; side <= 1; side += 2)
        {
            const float top = pegTop + 18, bottom = pegTop - (pegRows - 1) * pegDy - 10;
            const float slope = (binWidth * 0.5f) / pegDy; // x per y, same as the outer pegs
            RigidBody *w = segment({960 + side * (binWidth + 4.0f), top},
                                   {960 + side * (binWidth + 4.0f + (top - bottom) * slope), bottom}, 3);
            w->c = wood;
            w->restitution = 0.0f;
        }

        // funnel above the pegs: narrow outlet (~4 beads) so beads enter one at a time, right above the top peg.
        // Frictionless planks so the outlet does not jam; the funnel fills and drains like an hourglass.
        for (int side = -1; side <= 1; side += 2)
        {
            RigidBody *plank = segment({960 + side * 260.0f, 985}, {960 + side * 19.0f, pegTop + 45}, 6);
            plank->c = wood;
            plank->staticFriction = 0.0f;
            plank->dynamicFriction = 0.0f;
        }
    }

    void step() override
    {
        if (!pouring)
            return;
        for (int i = 0; i < rate && released < totalBeads; i++)
        {
            const Vector2 p(960 + frand(-120, 120), 975 + frand(0, 10));
            // never spawn inside a bead that is still up in the funnel (the newest ones)
            bool blocked = false;
            for (size_t k = beads.size() > 400 ? beads.size() - 400 : 0; k < beads.size() && !blocked; k++)
                blocked = (beads[k]->position - p).squareMagnitude() < (2.2f * beadRadius) * (2.2f * beadRadius);
            if (blocked)
                continue;
            RigidBody *b = ball(p, beadRadius, 1.0f);
            b->restitution = 0.0f;
            b->staticFriction = 0.3f;
            b->dynamicFriction = 0.2f;
            b->linearDamping = beadDamping;   // "air drag": caps the speed so every peg really redirects the bead
            b->angularDamping = beadDamping;
            b->c = hsv(released * 0.0025f, 0.8f, 1.0f); // rainbow stream -> striped bins
            beads.push_back(b);
            released++;
        }
    }

private:
    void drawBellCurve()
    {
        std::vector<float> h = binHeights();
        float peak = 0;
        for (float v : h)
            peak = std::max(peak, v);
        if (peak < 20.0f)
            return;

        // A bead's final position is a sum of pegRows steps of +-binWidth/2: sigma = sqrt(rows) * binWidth / 2.
        // Scale the curve to the tallest bin and draw it over the board.
        const float sigma = std::sqrt((float)pegRows) * 0.5f * binWidth;
        ImDrawList *dl = ImGui::GetBackgroundDrawList();
        std::vector<ImVec2> pts;
        for (float x = -bins * binWidth * 0.5f; x <= bins * binWidth * 0.5f; x += 4.0f)
        {
            const float y = floorY + peak * std::exp(-0.5f * (x / sigma) * (x / sigma));
            SDL_FPoint s = Renderer2D::worldToScreen(960 + x, y);
            pts.push_back(ImVec2(s.x, s.y));
        }
        dl->AddPolyline(pts.data(), (int)pts.size(), IM_COL32(255, 255, 255, 220), 0, 3.0f);
    }

    std::vector<RigidBody *> beads;
    int totalBeads = 1300;
    int released = 0;
    int rate = 1;
public:
    // fraction of velocity kept per second. Without drag beads build up sideways speed and shoot down the
    // lattice's diagonal lanes (flat/bimodal bins); 0.005 gives a clean bell (std ~60 px vs ideal ~52).
    float beadDamping = 0.005f;
private:
    bool pouring = true;
    bool showCurve = true;

    // layout
    static constexpr int bins = 31;
    static constexpr float binWidth = 28.0f;
    static constexpr int pegRows = 14;
    static constexpr float pegDy = 24.0f;       // ~ binWidth * sqrt(3)/2: hexagonal lattice
    static constexpr float pegRadius = 7.0f;    // gap between pegs ~ 2.3 bead diameters
    static constexpr float pegTop = 830.0f;
    static constexpr float floorY = 70.0f;
    static constexpr float binTop = 480.0f;
    static constexpr float beadRadius = 3.0f;
    float binLeft = 0;
};
