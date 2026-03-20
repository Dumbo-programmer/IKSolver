// Minimal 2D CCD IK demo with SDL2 visualization
// Controls:
//  - left click: set target
//  - space: pause/resume solving
//  - r: randomize joint angles
//  - +/- : change iteration speed (solver iterations per frame)
//  - esc or window close: quit

#include <SDL2/SDL.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <random>
#include <chrono>
#include <cmath>
#include <stdio.h>
#include <SDL2/SDL_main.h>
#define M_PI 3.14159265358979323846

struct Vec2 {
    double x{0}, y{0};
    Vec2() = default;
    Vec2(double X, double Y) : x(X), y(Y) {}
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    Vec2& operator+=(const Vec2& o){ x+=o.x; y+=o.y; return *this; }
    double dot(const Vec2& o) const { return x*o.x + y*o.y; }
    double norm() const { return std::sqrt(x*x + y*y); }
    Vec2 normalized() const { double n = norm(); return (n>1e-12)? Vec2{x/n, y/n} : Vec2{0,0}; }
};

static inline double clamp(double v, double lo, double hi){ return std::max(lo, std::min(hi, v)); }
static inline double rad(double deg){ return deg * M_PI / 180.0; }
static inline double deg(double rad){ return rad * 180.0 / M_PI; }
static inline double sgn(double v){ return (v>0)-(v<0); }

// simple revolute joint+link
struct Segment {
    double length;   // link length
    double angle;    // absolute angle in radians (world)
};

// forward-kinematics: given base position and joint angles (absolute),
// compute joint world positions (including end effector)
static void forwardKinematics(const Vec2& base,
                              const std::vector<Segment>& segs,
                              std::vector<Vec2>& out_jointPos) {
    out_jointPos.resize(segs.size() + 1);
    out_jointPos[0] = base;
    double accAngle = 0.0;
    for (size_t i=0; i<segs.size(); ++i){
        accAngle += segs[i].angle; // interpret angle as relative; flip to absolute by accumulation
        Vec2 dir{ std::cos(accAngle), std::sin(accAngle) };
        out_jointPos[i+1] = out_jointPos[i] + dir * segs[i].length;
    }
}

// CCD step: iterate joints from end to base, rotating each to reduce end effector error
// Returns current end effector error distance after this step
static double ccdStep(Vec2 base,
                      std::vector<Segment>& segs,
                      const Vec2& target,
                      double maxDeltaRadPerJoint = rad(15.0)) // limit per-joint change per pass
{
    std::vector<Vec2> J;
    forwardKinematics(base, segs, J);
    Vec2 end = J.back();

    for (int i = (int)segs.size()-1; i >= 0; --i){
        Vec2 Ji = J[i];
        Vec2 r  = end    - Ji; // vector from joint to end effector
        Vec2 t  = target - Ji; // vector from joint to target

        if (r.norm() < 1e-9 || t.norm() < 1e-9) continue;

        // Signed angle between r and t
        double cr = r.x*t.y - r.y*t.x; // 2D cross (z-component)
        double dp = r.dot(t);
        double ang = std::atan2(cr, dp); // signed angle (-pi, pi)

        // Limit the rotation per joint to keep things stable
        double delta = clamp(ang, -maxDeltaRadPerJoint, maxDeltaRadPerJoint);

        // Apply rotation to this joint angle (relative model)
        segs[i].angle += delta;

        // Recompute FK from this joint forward for next joints to see updates
        forwardKinematics(base, segs, J);
        end = J.back();
    }
    return (J.back() - target).norm();
}

// helper to draw a small filled circle (naive)
static void drawFilledCircle(SDL_Renderer* r, int cx, int cy, int radius){
    for (int dy = -radius; dy <= radius; ++dy){
        int dxlim = (int)std::sqrt(radius*radius - dy*dy);
        for (int dx = -dxlim; dx <= dxlim; ++dx){
            SDL_RenderDrawPoint(r, cx+dx, cy+dy);
        }
    }
}

// world->screen mapping (centered world, +y up)
struct Camera2D {
    int screenW{1000}, screenH{700};
    double pixelsPerUnit{60.0};
    Vec2 worldCenter{0,0}; // world coords at screen center

    SDL_Point worldToScreen(const Vec2& p) const {
        double sx = (p.x - worldCenter.x) * pixelsPerUnit + screenW * 0.5;
        double sy = (worldCenter.y - p.y) * pixelsPerUnit + screenH * 0.5; // flip Y
        return SDL_Point{ (int)std::lround(sx), (int)std::lround(sy) };
    }

    Vec2 screenToWorld(int sx, int sy) const {
        double x = (sx - screenW * 0.5) / pixelsPerUnit + worldCenter.x;
        double y = (screenH * 0.5 - sy) / pixelsPerUnit + worldCenter.y; // flip back
        return Vec2{x, y};
    }
};


int SDL_main(int argc, char** argv)
{

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0){
        std::fprintf(stderr, "SDL_Init error: %s\n", SDL_GetError());
        return 1;
    }

    Camera2D cam;
    cam.screenW = 1000;
    cam.screenH = 700;
    cam.pixelsPerUnit = 80.0;

    SDL_Window* win = SDL_CreateWindow(
        "2D CCD IK (SDL2) - Left click to set target",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        cam.screenW, cam.screenH, SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI
    );
    if (!win){
        std::fprintf(stderr, "SDL_CreateWindow error: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    SDL_Renderer* renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer){
        std::fprintf(stderr, "SDL_CreateRenderer error: %s\n", SDL_GetError());
        SDL_DestroyWindow(win);
        SDL_Quit();
        return 1;
    }

    // kinematic chain: start simple (3 segments)
    std::vector<Segment> segs = {
        {0.9, rad(10)}, {0.7, rad(-20)}, {0.5, rad(30)}
    };
    Vec2 base{0.0, 0.0};
    Vec2 target{1.6, 0.3};

    bool running = true;
    bool solving = true;
    int itersPerFrame = 4; // how many CCD passes per frame
    std::mt19937 rng( (uint32_t)std::chrono::high_resolution_clock::now().time_since_epoch().count() );
    std::uniform_real_distribution<double> Uang(-M_PI, M_PI);

    while (running){
        // ----- input -----
        SDL_Event e;
        while (SDL_PollEvent(&e)){
            if (e.type == SDL_QUIT) running = false;
            else if (e.type == SDL_KEYDOWN){
                if (e.key.keysym.sym == SDLK_ESCAPE) running = false;
                else if (e.key.keysym.sym == SDLK_SPACE) solving = !solving;
                else if (e.key.keysym.sym == SDLK_r){
                    for (auto& s : segs) s.angle = Uang(rng)*0.5; // random-ish
                }
                else if (e.key.keysym.sym == SDLK_EQUALS || e.key.keysym.sym == SDLK_PLUS){
                    itersPerFrame = std::min(64, itersPerFrame + 1);
                } else if (e.key.keysym.sym == SDLK_MINUS){
                    itersPerFrame = std::max(1, itersPerFrame - 1);
                }
            } else if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT){
                int mx = e.button.x, my = e.button.y;
                target = cam.screenToWorld(mx, my);
            }
        }

        // ----- update (solver) -----
        double err = 0.0;
        if (solving){
            for (int i=0; i<itersPerFrame; ++i){
                err = ccdStep(base, segs, target, rad(10.0));
            }

            std::printf("Error: %f\n", err);
        }

        // ----- draw -----
        SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
        SDL_SetRenderDrawColor(renderer, 22, 24, 28, 255); // background
        SDL_RenderClear(renderer);

        // draw grid
        SDL_SetRenderDrawColor(renderer, 48, 52, 56, 255);
        for (int gx=-10; gx<=10; ++gx){
            Vec2 a{(double)gx, -10.0}, b{(double)gx, 10.0};
            SDL_Point A = cam.worldToScreen(a), B = cam.worldToScreen(b);
            SDL_RenderDrawLine(renderer, A.x, A.y, B.x, B.y);
        }
        for (int gy=-10; gy<=10; ++gy){
            Vec2 a{-10.0, (double)gy}, b{10.0, (double)gy};
            SDL_Point A = cam.worldToScreen(a), B = cam.worldToScreen(b);
            SDL_RenderDrawLine(renderer, A.x, A.y, B.x, B.y);
        }

        // forward kinematics for drawing
        std::vector<Vec2> J;
        forwardKinematics(base, segs, J);

        // draw target
        SDL_Point T = cam.worldToScreen(target);
        SDL_SetRenderDrawColor(renderer, 200, 90, 90, 255);
        drawFilledCircle(renderer, T.x, T.y, 5);
        SDL_RenderDrawLine(renderer, T.x-10, T.y, T.x+10, T.y);
        SDL_RenderDrawLine(renderer, T.x, T.y-10, T.x, T.y+10);

        // draw links
        SDL_SetRenderDrawColor(renderer, 180, 220, 255, 255);
        for (size_t i=0; i+1<J.size(); ++i){
            SDL_Point A = cam.worldToScreen(J[i]);
            SDL_Point B = cam.worldToScreen(J[i+1]);
            SDL_RenderDrawLine(renderer, A.x, A.y, B.x, B.y);
        }

        // draw joints
        SDL_SetRenderDrawColor(renderer, 240, 240, 240, 255);
        for (size_t i=0; i<J.size(); ++i){
            SDL_Point P = cam.worldToScreen(J[i]);
            drawFilledCircle(renderer, P.x, P.y, (i==J.size()-1) ? 5 : 6);
        }

        // visualize reach circle (arm max reach)
        double reach=0.0; for (auto& s:segs) reach += s.length;
        SDL_Point C = cam.worldToScreen(base);
        SDL_SetRenderDrawColor(renderer, 120, 120, 120, 60);
        // crude circle: draw many points
        for (int k=0;k<360;k+=2){
            double a = rad((double)k);
            Vec2 p = base + Vec2{std::cos(a), std::sin(a)} * reach;
            SDL_Point P = cam.worldToScreen(p);
            SDL_RenderDrawPoint(renderer, P.x, P.y);
        }
        std::printf("Circle center: (%i, %i)\n", C.x, C.y);

        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
