//
// Created by Gaëtan Blaise-Cazalet on 28/01/2024.
//

#ifndef SCENE02QUAD_HPP
#define SCENE02QUAD_HPP

#include <Quad.hpp>

#include "Scene.hpp"

using gdraw::Quad;

class Scene02Quad : public Scene {
public:
    void Load(Renderer& renderer) override;
    bool Update(float dt) override;
    void Draw(Renderer& renderer) override;
    void Unload(Renderer& renderer) override;

private:
    uptr<Quad> quad;
    InputState inputState;
};



#endif //SCENE02QUAD_HPP