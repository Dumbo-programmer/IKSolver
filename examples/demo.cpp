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
    (void)argc;
    (void)argv;

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

    // Demo scenarios: because variety is the spice of IK!
    std::vector<std::vector<Vector2>> scenarios;

    // Scenario 1: Straight chain
    scenarios.push_back({
        {400, 300}, {450, 300}, {500, 300}, {550, 300}, {600, 300}
    });

    // Scenario 2: Zigzag chain
    scenarios.push_back({
        {400, 300}, {430, 330}, {460, 300}, {490, 330}, {520, 300}
    });

    // Scenario 3: Circle chain (because robots love geometry)
    std::vector<Vector2> circleChain;
    double radius = 100;
    for (int i = 0; i < 5; ++i) {
        double angle = i * 2 * 3.14159 / 5;
        circleChain.push_back(Vector2(400 + radius * std::cos(angle), 300 + radius * std::sin(angle)));
    }
    scenarios.push_back(circleChain);

    // Scenario 4: Random chain (for the adventurous)
    std::vector<Vector2> randomChain;
    srand(42); // Consistent randomness, because chaos needs rules
    for (int i = 0; i < 5; ++i) {
        randomChain.push_back(Vector2(400 + rand() % 200, 300 + rand() % 200));
    }
    scenarios.push_back(randomChain);

    // Scenario 5: Double pendulum physics simulation
    scenarios.push_back({ {400, 300}, {400, 400}, {400, 500} }); // Initial positions

    int currentScenario = 0;
    IKChain2D chain(scenarios[currentScenario]);

    bool quit = false;
    SDL_Event e;
    bool useFABRIK = true;

    // Instructions: Press SPACE to toggle solver, TAB to change scenario
    std::cout << "IK Demo: Press SPACE to toggle solver, TAB to change scenario.\n";

    Vector2 target(400, 300); // Initial target
    bool dragging = false;

    // Double pendulum state
    double theta1 = 1.0; // Angle of first pendulum
    double theta2 = 2.0; // Angle of second pendulum
    double omega1 = 0.0; // Angular velocity 1
    double omega2 = 0.0; // Angular velocity 2
    double m1 = 1.0, m2 = 1.0; // Masses
    double l1 = 100.0, l2 = 100.0; // Lengths
    double g = 9.81; // Gravity

    while (!quit) {
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) quit = true;
            if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_SPACE)
                useFABRIK = !useFABRIK; // toggle solver
            if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_TAB) {
                currentScenario = (currentScenario + 1) % scenarios.size();
                chain = IKChain2D(scenarios[currentScenario]);
                std::cout << "Switched to scenario " << currentScenario + 1 << "!\n";
            }
            if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
                dragging = true;
            }
            if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_LEFT) {
                dragging = false;
            }
        }

        if (dragging) {
            int mouseX, mouseY;
            SDL_GetMouseState(&mouseX, &mouseY);
            target = Vector2(mouseX, mouseY);
        }

        // Double pendulum simulation if selected
        if (currentScenario == 4) {
            // Physics update (simple Euler integration)
            double dt = 0.016;
            double delta = theta2 - theta1;
            double denom1 = (m1 + m2) * l1 - m2 * l1 * std::cos(delta) * std::cos(delta);
            double denom2 = (l2 / l1) * denom1;

            double a1 = (m2 * l1 * omega1 * omega1 * std::sin(delta) * std::cos(delta)
                        + m2 * g * std::sin(theta2) * std::cos(delta)
                        + m2 * l2 * omega2 * omega2 * std::sin(delta)
                        - (m1 + m2) * g * std::sin(theta1)) / denom1;
            double a2 = (-m2 * l2 * omega2 * omega2 * std::sin(delta) * std::cos(delta)
                        + (m1 + m2) * g * std::sin(theta1) * std::cos(delta)
                        - (m1 + m2) * l1 * omega1 * omega1 * std::sin(delta)
                        - (m1 + m2) * g * std::sin(theta2)) / denom2;

            omega1 += a1 * dt;
            omega2 += a2 * dt;
            theta1 += omega1 * dt;
            theta2 += omega2 * dt;

            // Calculate positions
            Vector2 base(400, 300);
            Vector2 joint1 = base + Vector2(l1 * std::sin(theta1), l1 * std::cos(theta1));
            Vector2 joint2 = joint1 + Vector2(l2 * std::sin(theta2), l2 * std::cos(theta2));
            chain.bones[0].position = base;
            chain.bones[1].position = joint1;
            chain.bones[2].position = joint2;
        } else {
            // Solve IK
            if (useFABRIK)
                FABRIKSolver::Solve(chain, target);
            else
                CCDSolver::Solve(chain, target);
            // If your chain starts dancing, it's not a bug, it's a feature.
        }

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
