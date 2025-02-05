#include "Scene03Cube.hpp"

void Scene03Cube::Load(Renderer& renderer) {
    cube = std::make_unique<Cube>(renderer);
    cube->Load();
}

bool Scene03Cube::Update(float dt) {
    bool stillRunning = ManageInput(inputState);
    cube->Update(dt);
    return stillRunning;
}

void Scene03Cube::Draw(Renderer& renderer) {
    renderer.Begin();

    cube->Draw(renderer);

    renderer.End();
}

void Scene03Cube::Unload(Renderer& renderer) {
    cube->Unload();
}