#include "Scene02Quad.hpp"

void Scene02Quad::Load(Renderer& renderer) {
    quad = std::make_unique<Quad>(renderer);
    quad->Load();
}

bool Scene02Quad::Update(float dt) {
    bool stillRunning = ManageInput(inputState);
    quad->Update(dt);
    return stillRunning;
}

void Scene02Quad::Draw(Renderer& renderer) {
    renderer.Begin();

    quad->Draw(renderer);

    renderer.End();
}

void Scene02Quad::Unload(Renderer& renderer) {
    quad->Unload();
}