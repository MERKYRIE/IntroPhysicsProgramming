//
// Created by Gaëtan Blaise-Cazalet on 13/01/2024.
//

#include <iostream>
#include "SDL3/SDL_main.h"
#include "Time.hpp"
#include "Window.hpp"
#include "Renderer.hpp"
#include "Scene01Clear.hpp"
#include "Scene02Quad.hpp"
#include "Scene03Cube.hpp"
#include "Scene04Sphere.hpp"

using namespace std;

using gdraw::Window;
using gdraw::Renderer;

int main(int argc, char **argv) {
    Window window {};
    Renderer renderer {};
    Time time {};
    window.Init();
    renderer.Init(window);

    auto scene = std::make_unique<Scene04Sphere>();
    scene->Load(renderer);

    bool isRunning { true };
    while (isRunning) {
        const float dt = time.ComputeDeltaTime();

        isRunning = scene->Update(dt);
        scene->Draw(renderer);

        time.DelayTime();
    }

    scene->Unload(renderer);

    renderer.Close();
    window.Close();
    return 0;
}