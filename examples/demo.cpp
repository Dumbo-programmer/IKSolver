#include <SDL2/SDL.h>
#include <iostream>
#include "iksolver/IKChain2D.h"
#include "iksolver/CCDSolver.h"
#include "iksolver/FABRIKSolver.h"

using namespace ik;

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;

// Draw a line between two Vector2 points
void DrawLine(SDL_Renderer* renderer, const Vector2& a, const Vector2& b) {
    SDL_RenderDrawLine(renderer, static_cast<int>(a.x), static_cast<int>(a.y),
                                 static_cast<int>(b.x), static_cast<int>(b.y));
}

int SDL_main(int argc, char* argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << "\n";
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("IK Solver Demo", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                          WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    if (!window || !renderer) {
        std::cerr << "SDL Create Error: " << SDL_GetError() << "\n";
        SDL_Quit();
        return 1;
    }

    // Create a simple 5-joint 2D chain
    std::vector<Vector2> joints = {
        {400, 300},
        {450, 300},
        {500, 300},
        {550, 300},
        {600, 300}
    };
    IKChain2D chain(joints);

    bool quit = false;
    SDL_Event e;
    bool useFABRIK = true;

    while (!quit) {
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) quit = true;
            if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_SPACE)
                useFABRIK = !useFABRIK; // toggle solver
        }

        int mouseX, mouseY;
        SDL_GetMouseState(&mouseX, &mouseY);
        Vector2 target(mouseX, mouseY);

        // Solve IK
        if (useFABRIK)
            FABRIKSolver::Solve(chain, target);
        else
            CCDSolver::Solve(chain, target);

        // Render
        SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
        SDL_RenderClear(renderer);

        // Draw chain
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
        for (size_t i = 0; i < chain.bones.size() - 1; ++i)
            DrawLine(renderer, chain.bones[i].position, chain.bones[i + 1].position);

        // Draw target
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        SDL_Rect targetRect = {static_cast<int>(target.x) - 5, static_cast<int>(target.y) - 5, 10, 10};
        SDL_RenderFillRect(renderer, &targetRect);

        SDL_RenderPresent(renderer);
        SDL_Delay(16); // ~60 FPS
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
