#include "Scene01Clear.hpp"

void Scene01Clear::Load(Renderer& renderer) {

}

bool Scene01Clear::Update(float dt) {
    bool stillRunning = ManageInput(inputState);

    return stillRunning;
}

void Scene01Clear::Draw(Renderer& renderer) {
    renderer.Begin();

    renderer.End();
}

void Scene01Clear::Unload(Renderer& renderer) {

}