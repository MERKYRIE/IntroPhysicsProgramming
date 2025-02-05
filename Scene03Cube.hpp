//
// Created by Gaëtan Blaise-Cazalet on 28/01/2024.
//

#ifndef SCENE03CUBE_HPP
#define SCENE03CUBE_HPP

#include <Cube.hpp>

#include "Scene.hpp"

using gdraw::Cube;

class Scene03Cube : public Scene {
public:
    void Load(Renderer& renderer) override;
    bool Update(float dt) override;
    void Draw(Renderer& renderer) override;
    void Unload(Renderer& renderer) override;

private:
    uptr<Cube> cube;
    InputState inputState;
};



#endif //SCENE03CUBE_HPP