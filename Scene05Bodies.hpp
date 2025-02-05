//
// Created by Gaëtan Blaise-Cazalet on 31/01/2024.
//

#ifndef SCENE05BODIES_HPP
#define SCENE05BODIES_HPP

#include "Scene.hpp"
#include "Body.hpp"
#include "SphereCube.hpp"

using gphysics::Body;
using gdraw::SphereCube;

class Scene05Bodies : public Scene {
public:
    void Load(Renderer& renderer) override;
    bool Update(float dt) override;
    void Draw(Renderer& renderer) override;
    void Unload(Renderer& renderer) override;

private:
    vector<Body> bodies;
    InputState inputState;
};



#endif //SCENE05BODIES_HPP