#include "Scene04Sphere.hpp"

void Scene04Sphere::Load(Renderer& renderer) {
    sphere = std::make_unique<SphereCube>(renderer);
    sphere->Load();
}

bool Scene04Sphere::Update(float dt) {
    bool stillRunning = ManageInput(inputState);
    sphere->Update(dt);
    return stillRunning;
}

void Scene04Sphere::Draw(Renderer& renderer) {
    renderer.Begin();

    sphere->Draw(renderer);

    renderer.End();
}

void Scene04Sphere::Unload(Renderer& renderer) {
    sphere->Unload();
}